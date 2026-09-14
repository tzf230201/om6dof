#pragma once
#include "bio_focus.hpp"

namespace bio_fgng {

struct MapperConfig {
    fgng::Config graph;
    FocusConfig focus;
    size_t memoryCapacity=8000, replayBudget=2000, inputBudget=6000;
    float voxelSize=0.025F;
    // Observed support constrains the exported surface graph. Zero disables it.
    float edgeSupportRadius=0.05F;
    int epochsPerFrame=1;
    MapperConfig() { graph.maxNodes=256; }
};

struct MapperStats {
    size_t inputPixels=0, validPoints=0, insertedPoints=0, memoryPoints=0;
    size_t replayPoints=0, nodes=0, edges=0, evictions=0;
    size_t rawEdges=0, rejectedEdges=0, rejectedUnsupportedEdges=0;
    int churn=0;
    double updateMs=0;
};

// Spatial index over measured world-space support, not over graph nodes. An
// edge is a local surface relation only when points along its straight segment
// stay near observations. This does not certify free space or traversability.
class SurfaceSupport {
public:
    SurfaceSupport(const std::vector<world_fgng::Point>& points,float radius)
        : radius_(radius) {
        if (!std::isfinite(radius) || radius<=0)
            throw std::invalid_argument("edge support radius must be positive");
        cells_.reserve(points.size());
        for (const auto& p:points) {
            world_fgng::VoxelKey key;
            if (cell(p,key)) cells_[key].push_back(p);
        }
    }

    bool supportsSegment(const world_fgng::Point& a,const world_fgng::Point& b) const {
        if (!world_fgng::finite(a) || !world_fgng::finite(b)) return false;
        const double dx=double(b.x)-a.x,dy=double(b.y)-a.y,dz=double(b.z)-a.z;
        const double length=std::sqrt(dx*dx+dy*dy+dz*dz);
        const double steps=std::ceil(length/(0.5*radius_));
        // Fail closed for extreme coordinates/radius combinations; no
        // unbounded sampling work or undefined floating-to-integer casts.
        if (!std::isfinite(steps) || steps>10000) return false;
        const size_t count=std::max<size_t>(1,static_cast<size_t>(steps));
        for(size_t i=0;i<=count;++i) {
            const double t=double(i)/count;
            if (!near({static_cast<float>(a.x+t*dx),static_cast<float>(a.y+t*dy),
                       static_cast<float>(a.z+t*dz)})) return false;
        }
        return true;
    }

private:
    struct Hash {
        size_t operator()(const world_fgng::VoxelKey& k) const {
            return world_fgng::mix64(world_fgng::mix64(uint64_t(k.x)) ^
                world_fgng::mix64(uint64_t(k.y)+UINT64_C(0x517cc1b727220a95)) ^
                world_fgng::mix64(uint64_t(k.z)+UINT64_C(0x6eed0e9da4d94a4f)));
        }
    };
    bool cell(const world_fgng::Point& p,world_fgng::VoxelKey& key) const {
        if (!world_fgng::finite(p)) return false;
        const double x=std::floor(double(p.x)/radius_);
        const double y=std::floor(double(p.y)/radius_);
        const double z=std::floor(double(p.z)/radius_);
        if (std::abs(x)>9e18 || std::abs(y)>9e18 || std::abs(z)>9e18) return false;
        key={static_cast<int64_t>(x),static_cast<int64_t>(y),static_cast<int64_t>(z)};
        return true;
    }
    bool near(const world_fgng::Point& p) const {
        world_fgng::VoxelKey key;
        if (!cell(p,key)) return false;
        const double radius2=double(radius_)*radius_;
        for(int x=-1;x<=1;++x) for(int y=-1;y<=1;++y) for(int z=-1;z<=1;++z) {
            const auto found=cells_.find({key.x+x,key.y+y,key.z+z});
            if (found==cells_.end()) continue;
            for (const auto& q:found->second) {
                const double dx=double(p.x)-q.x,dy=double(p.y)-q.y,dz=double(p.z)-q.z;
                if (dx*dx+dy*dy+dz*dz<=radius2) return true;
            }
        }
        return false;
    }
    double radius_;
    std::unordered_map<world_fgng::VoxelKey,std::vector<world_fgng::Point>,Hash> cells_;
};

// Coordinates learned by the F-GNG core are always world coordinates.
// Retinal attention modifies its density demand, not the raw metric geometry.
class Mapper {
public:
    explicit Mapper(MapperConfig config={})
        : config_(validated(std::move(config))), focus_(config_.focus),
          memory_(config_.voxelSize,config_.memoryCapacity,config_.graph.seed),
          graph_(config_.graph) {}
    Mapper(const Mapper&)=delete;
    Mapper& operator=(const Mapper&)=delete;

    MapperStats update(const DepthFrame& frame,const FocusControl& control) {
        const auto start=std::chrono::steady_clock::now();
        // Validation happens inside focus before mutating its state or memory.
        focus_.update(frame,control);
        MapperStats stats;
        stats.inputPixels=frame.depth.size();
        std::vector<size_t> valid;
        valid.reserve(frame.depth.size());
        for (size_t i=0;i<frame.depth.size();++i) {
            const float z=frame.depth[i];
            if (std::isfinite(z) && z>=config_.focus.minDepth && z<=config_.focus.maxDepth)
                valid.push_back(i);
        }
        stats.validPoints=valid.size();
        const size_t n=std::min(valid.size(),config_.inputBudget);
        const auto& k=frame.intrinsics;
        // Uniform deterministic support ingestion avoids multiplying retinal
        // importance once in sampling and a second time in graph allocation.
        for (size_t j=0;j<n;++j) {
            const size_t index=valid[j*valid.size()/n];
            const float z=frame.depth[index];
            const float u=static_cast<float>(index%static_cast<size_t>(k.width));
            const float v=static_cast<float>(index/static_cast<size_t>(k.width));
            const world_fgng::Point p{(u-k.cx)*z/k.fx,(v-k.cy)*z/k.fy,z};
            const auto world=frame.pose.toWorld(p);
            if (world_fgng::finite(world) && memory_.insert(world)) ++stats.insertedPoints;
        }
        graph_.setAttention([this](const world_fgng::Point& p){ return focus_.evaluate(p); });
        // With no current observation, retain graph exactly; don't churn history
        // simply because the depth sensor temporarily returned no support.
        if (n>0) for (int epoch=0;epoch<config_.epochsPerFrame;++epoch) {
            const auto batch=memory_.replay(config_.replayBudget,replayEpoch_++);
            stats.replayPoints=batch.size();
            graph_.learnBatch(batch);
            if (batch.size()>=3) stats.churn+=graph_.churn();
        }
        stats.memoryPoints=memory_.size(); stats.evictions=memory_.evictions();
        if (n>0) refreshSurfaceGraph();
        stats.nodes=surfaceGraph_.nodes.size(); stats.edges=surfaceGraph_.edges.size();
        stats.rawEdges=rawEdges_;
        stats.rejectedUnsupportedEdges=rejectedUnsupportedEdges_;
        stats.rejectedEdges=rawEdges_-stats.edges;
        stats.updateMs=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-start).count();
        return stats;
    }

    world_fgng::Graph graph() const {
        return surfaceGraph_;
    }
    const FocusField& focus() const { return focus_; }
    const world_fgng::VoxelMemory& memory() const { return memory_; }
    const MapperConfig& config() const { return config_; }

private:
    void refreshSurfaceGraph() {
        graph_.copyGraph(surfaceGraph_.nodes,surfaceGraph_.edges);
        graph_.copyIds(surfaceGraph_.ids);
        rawEdges_=surfaceGraph_.edges.size(); rejectedUnsupportedEdges_=0;
        std::unique_ptr<SurfaceSupport> support;
        if (config_.edgeSupportRadius>0)
            support=std::make_unique<SurfaceSupport>(memory_.representatives(),config_.edgeSupportRadius);
        std::erase_if(surfaceGraph_.edges,[&](const auto& edge) {
            const auto& a=surfaceGraph_.nodes[edge.first];
            const auto& b=surfaceGraph_.nodes[edge.second];
            if (support && !support->supportsSegment(a,b)) {
                ++rejectedUnsupportedEdges_; return true;
            }
            return false;
        });
    }

    static MapperConfig validated(MapperConfig c) {
        // Reuse the existing complete graph-parameter validation before dense
        // allocations. Temporary wrapper is destroyed at constructor exit.
        world_fgng::Config base;
        base.graph=c.graph; base.memoryCapacity=c.memoryCapacity;
        base.replayBudget=c.replayBudget; base.voxelSize=c.voxelSize;
        base.epochsPerFrame=c.epochsPerFrame;
        if (c.inputBudget<3 || c.inputBudget>4000000 || c.memoryCapacity>4000000 ||
            c.replayBudget>4000000 || c.epochsPerFrame>100)
            throw std::invalid_argument("unsupported input/memory/replay budget or epochs");
        if (!std::isfinite(c.edgeSupportRadius) || c.edgeSupportRadius<0)
            throw std::invalid_argument("edge support radius must be finite and nonnegative");
        const world_fgng::WorldFoveatedGNG validateGraph(base);
        const FocusField validateFocus(c.focus);
        return c;
    }
    MapperConfig config_;
    FocusField focus_;
    world_fgng::VoxelMemory memory_;
    fgng::FoveatedGNG graph_;
    // The exported graph optionally requires nearby observed segment support.
    world_fgng::Graph surfaceGraph_;
    size_t rawEdges_=0,rejectedUnsupportedEdges_=0;
    uint64_t replayEpoch_=0;
};

} // namespace bio_fgng
