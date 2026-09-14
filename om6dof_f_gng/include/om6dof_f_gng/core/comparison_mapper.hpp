#pragma once

// ROS-neutral adapters around the unchanged, locally supplied native cores.
// Depth is converted into metric world XYZ before either baseline learns.
// See BASELINE_PROVENANCE.json for exact sources and algorithm limitations.
#include "bio_mapper.hpp"
#include "ddgng_core.hpp"

#include <numeric>

namespace comparison {

struct Config {
    bio_fgng::MapperConfig mapping;
    std::string algorithm="fgng";
    std::string memoryMode="world"; // "current" is available for baselines only.
    int ddUpdates=500;              // Stochastic updates per epoch, native default.
    int dblEdgeCutPeriodFrames=10;  // Valid depth frames per accumulated-strength cut.
};

inline dblgng::Config dblConfig(const fgng::Config& source) {
    dblgng::Config result;
    result.maxNodes=source.maxNodes;
    result.initialRegions=source.initialRegions;
    result.seed=source.seed;
    result.winnerRate=source.winnerRate;
    result.neighborRate=source.neighborRate;
    result.errorDecay=source.errorDecay;
    result.insertionDecay=source.insertionDecay;
    // Retain the native DBL-GNG prune interval, growth quantile and edge cut.
    return result;
}

class Mapper {
public:
    explicit Mapper(Config config={}) : config_(validated(std::move(config))) {
        if (attentionEnabled()) {
            fg_=std::make_unique<bio_fgng::Mapper>(config_.mapping);
            return;
        }
        focus_=std::make_unique<bio_fgng::FocusField>(config_.mapping.focus);
        if (config_.memoryMode=="world")
            memory_=std::make_unique<world_fgng::VoxelMemory>(
                config_.mapping.voxelSize,config_.mapping.memoryCapacity,
                config_.mapping.graph.seed);
        if (config_.algorithm=="ddgng")
            dd_=std::make_unique<DynamicGrowingNeuralGas>(config_.mapping.graph.maxNodes);
        else
            dbl_=std::make_unique<dblgng::DistributedBatchLearningGNG>(
                dblConfig(config_.mapping.graph));
    }
    Mapper(const Mapper&)=delete;
    Mapper& operator=(const Mapper&)=delete;

    bio_fgng::MapperStats update(const bio_fgng::DepthFrame& frame,
                                const bio_fgng::FocusControl& control) {
        if (fg_) return fg_->update(frame,control);
        const auto start=std::chrono::steady_clock::now();
        // The field validates the entire frame and rigid pose before committing
        // any support. It is maintained solely for camera/focus visualization;
        // its weights never enter baseline point sampling or core learning.
        focus_->update(frame,control);
        bio_fgng::MapperStats stats;
        stats.inputPixels=frame.depth.size();
        std::vector<size_t> valid;
        valid.reserve(frame.depth.size());
        const auto& cfg=config_.mapping;
        for (size_t i=0;i<frame.depth.size();++i) {
            const float z=frame.depth[i];
            if (std::isfinite(z) && z>=cfg.focus.minDepth && z<=cfg.focus.maxDepth)
                valid.push_back(i);
        }
        stats.validPoints=valid.size();
        const size_t count=std::min(valid.size(),cfg.inputBudget);
        std::vector<world_fgng::Point> incoming;
        incoming.reserve(count);
        const auto& k=frame.intrinsics;
        for (size_t j=0;j<count;++j) {
            const size_t i=valid[j*valid.size()/count];
            const float z=frame.depth[i];
            const float u=static_cast<float>(i%static_cast<size_t>(k.width));
            const float v=static_cast<float>(i/static_cast<size_t>(k.width));
            const world_fgng::Point camera{(u-k.cx)*z/k.fx,(v-k.cy)*z/k.fy,z};
            const auto world=frame.pose.toWorld(camera);
            if (!world_fgng::finite(world)) continue;
            incoming.push_back(world);
            if (memory_ && memory_->insert(world)) ++stats.insertedPoints;
        }
        // A sensor dropout retains positions, node IDs and all exported edges.
        // Do not learn history alone when no current observation is available.
        if (!incoming.empty()) {
            const auto current=boundedCurrent(incoming,cfg.replayBudget);
            for (int epoch=0;epoch<cfg.epochsPerFrame;++epoch) {
                const auto batch=memory_ ? memory_->replay(cfg.replayBudget,replayEpoch_++) : current;
                stats.replayPoints=batch.size();
                learn(batch);
            }
            if (dbl_) {
                ++dblFrames_;
                dblReachedCapacity_|=dbl_->nodeCount()>=cfg.graph.maxNodes;
                if (dblReachedCapacity_ &&
                    dblFrames_%static_cast<uint64_t>(config_.dblEdgeCutPeriodFrames)==0)
                    dbl_->cutWeakEdges();
            }
            copyRawGraph();
            refreshSurfaceGraph(memory_ ? memory_->representatives() : incoming);
        }
        stats.memoryPoints=memory_ ? memory_->size() : 0;
        stats.evictions=memory_ ? memory_->evictions() : 0;
        stats.nodes=surfaceGraph_.nodes.size();
        stats.edges=surfaceGraph_.edges.size();
        stats.rawEdges=rawGraph_.edges.size();
        stats.rejectedEdges=stats.rawEdges-stats.edges;
        stats.rejectedUnsupportedEdges=rejectedUnsupportedEdges_;
        // Native baseline APIs do not report F-GNG split/merge churn.
        stats.updateMs=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-start).count();
        return stats;
    }

    world_fgng::Graph graph() const { return fg_ ? fg_->graph() : surfaceGraph_; }
    // The existing bio Mapper does not expose its unfiltered core graph.
    // For F-GNG this method returns the same exported graph as graph().
    world_fgng::Graph rawGraph() const { return fg_ ? fg_->graph() : rawGraph_; }
    const bio_fgng::FocusField& focus() const { return fg_ ? fg_->focus() : *focus_; }
    bool attentionEnabled() const { return config_.algorithm=="fgng"; }
    const Config& config() const { return config_; }

private:
    static Config validated(Config config) {
        if (config.algorithm!="fgng" && config.algorithm!="ddgng" && config.algorithm!="dblgng")
            throw std::invalid_argument("algorithm must be fgng, ddgng or dblgng");
        if (config.memoryMode!="world" && config.memoryMode!="current")
            throw std::invalid_argument("memory_mode must be world or current");
        if (config.algorithm=="fgng" && config.memoryMode!="world")
            throw std::invalid_argument("F-GNG requires memory_mode world; current is baseline-only");
        if (config.ddUpdates<1 || config.ddUpdates>100000)
            throw std::invalid_argument("dd_updates must be in [1,100000]");
        if (config.dblEdgeCutPeriodFrames<1 || config.dblEdgeCutPeriodFrames>10000)
            throw std::invalid_argument("dbl_edge_cut_period_frames must be in [1,10000]");
        const auto& c=config.mapping;
        if (c.inputBudget<3 || c.inputBudget>4000000 || c.memoryCapacity>4000000 ||
            c.replayBudget>4000000 || c.epochsPerFrame>100)
            throw std::invalid_argument("unsupported input/memory/replay budget or epochs");
        if (!std::isfinite(c.edgeSupportRadius) || c.edgeSupportRadius<0)
            throw std::invalid_argument("edge support radius must be finite and nonnegative");
        world_fgng::Config base;
        // Reuse the complete graph/budget validation without constructing a
        // dense graph: the FPS branch allocates no native core matrix.
        base.mode=world_fgng::Mode::MemoryFps;
        base.graph=c.graph;
        base.memoryCapacity=c.memoryCapacity;
        base.replayBudget=c.replayBudget;
        base.voxelSize=c.voxelSize;
        base.epochsPerFrame=c.epochsPerFrame;
        const world_fgng::WorldFoveatedGNG validateGraph(base);
        c.focus.validate();
        return config;
    }

    static std::vector<world_fgng::Point> boundedCurrent(
            const std::vector<world_fgng::Point>& points,size_t budget) {
        if (points.size()<=budget) return points;
        std::vector<world_fgng::Point> selected;
        selected.reserve(budget);
        for (size_t i=0;i<budget;++i) selected.push_back(points[i*points.size()/budget]);
        return selected;
    }

    void learn(const std::vector<world_fgng::Point>& batch) {
        if (dd_) {
            std::vector<GngPoint3f> points;
            points.reserve(batch.size());
            for (const auto& p:batch) points.push_back({p.x,p.y,p.z});
            dd_->partialFit(points,config_.ddUpdates);
        } else {
            std::vector<dblgng::Point3f> points;
            points.reserve(batch.size());
            for (const auto& p:batch) points.push_back({p.x,p.y,p.z});
            dbl_->learnBatch(points);
        }
    }

    void copyRawGraph() {
        rawGraph_.nodes.clear();
        rawGraph_.ids.clear();
        if (dd_) {
            std::vector<GngPoint3f> points;
            std::vector<uint32_t> ids;
            dd_->copyGraph(points,ids,rawGraph_.edges);
            rawGraph_.nodes.reserve(points.size());
            for (const auto& p:points) rawGraph_.nodes.push_back({p.x,p.y,p.z});
            rawGraph_.ids.assign(ids.begin(),ids.end());
        } else {
            std::vector<dblgng::Point3f> points;
            dbl_->copyGraph(points,rawGraph_.edges);
            rawGraph_.nodes.reserve(points.size());
            for (const auto& p:points) rawGraph_.nodes.push_back({p.x,p.y,p.z});
            // Native DBL-GNG has no persistent IDs. These are current array
            // indices, suitable for one graph snapshot only.
            rawGraph_.ids.resize(points.size());
            std::iota(rawGraph_.ids.begin(),rawGraph_.ids.end(),int64_t{0});
        }
    }

    void refreshSurfaceGraph(const std::vector<world_fgng::Point>& observations) {
        surfaceGraph_=rawGraph_;
        rejectedUnsupportedEdges_=0;
        const auto& cfg=config_.mapping;
        std::unique_ptr<bio_fgng::SurfaceSupport> support;
        if (cfg.edgeSupportRadius>0)
            support=std::make_unique<bio_fgng::SurfaceSupport>(observations,cfg.edgeSupportRadius);
        std::erase_if(surfaceGraph_.edges,[&](const auto& edge) {
            const auto& a=surfaceGraph_.nodes[edge.first];
            const auto& b=surfaceGraph_.nodes[edge.second];
            if (support && !support->supportsSegment(a,b)) {
                ++rejectedUnsupportedEdges_;
                return true;
            }
            return false;
        });
    }

    Config config_;
    std::unique_ptr<bio_fgng::Mapper> fg_;
    std::unique_ptr<bio_fgng::FocusField> focus_;
    std::unique_ptr<world_fgng::VoxelMemory> memory_;
    std::unique_ptr<DynamicGrowingNeuralGas> dd_;
    std::unique_ptr<dblgng::DistributedBatchLearningGNG> dbl_;
    world_fgng::Graph rawGraph_,surfaceGraph_;
    size_t rejectedUnsupportedEdges_=0;
    uint64_t replayEpoch_=0;
    uint64_t dblFrames_=0;
    bool dblReachedCapacity_=false;
};

} // namespace comparison
