#pragma once

// Pose-conditioned, static-scene extension of the existing F-GNG core.
// This adapter estimates neither camera poses nor visibility/free space.
#include "fgng_core.hpp"
#include "dblgng_core.hpp"

#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>

namespace world_fgng {

using Point = fgng::Point3f;

inline bool finite(const Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

struct Pose {
    // Row-major rigid transform T_world_camera; coordinates and translation in m.
    std::array<double, 16> m{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

    void validate() const {
        for (double v : m) if (!std::isfinite(v))
            throw std::invalid_argument("pose contains non-finite values");
        if (std::abs(m[12]) > 1e-7 || std::abs(m[13]) > 1e-7 ||
            std::abs(m[14]) > 1e-7 || std::abs(m[15]-1) > 1e-7)
            throw std::invalid_argument("pose must have homogeneous last row 0 0 0 1");
        for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) {
            double dot=0;
            for (int k=0; k<3; ++k) dot += m[k*4+i]*m[k*4+j];
            if (std::abs(dot-(i==j ? 1.0 : 0.0)) > 1e-3)
                throw std::invalid_argument("pose rotation is not orthonormal");
        }
        const double det=m[0]*(m[5]*m[10]-m[6]*m[9])
                       -m[1]*(m[4]*m[10]-m[6]*m[8])
                       +m[2]*(m[4]*m[9]-m[5]*m[8]);
        if (std::abs(det-1.0)>1e-3)
            throw std::invalid_argument("pose must contain a proper rotation");
    }
    Point toWorld(const Point& p) const {
        return {static_cast<float>(m[0]*p.x+m[1]*p.y+m[2]*p.z+m[3]),
                static_cast<float>(m[4]*p.x+m[5]*p.y+m[6]*p.z+m[7]),
                static_cast<float>(m[8]*p.x+m[9]*p.y+m[10]*p.z+m[11])};
    }
    Point toCamera(const Point& p) const {
        const double x=p.x-m[3], y=p.y-m[7], z=p.z-m[11];
        return {static_cast<float>(m[0]*x+m[4]*y+m[8]*z),
                static_cast<float>(m[1]*x+m[5]*y+m[9]*z),
                static_cast<float>(m[2]*x+m[6]*y+m[10]*z)};
    }
};

struct VoxelKey {
    int64_t x, y, z;
    auto operator<=>(const VoxelKey&) const = default;
};

inline uint64_t mix64(uint64_t x) {
    x += UINT64_C(0x9e3779b97f4a7c15);
    x = (x ^ (x >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)) * UINT64_C(0x94d049bb133111eb);
    return x ^ (x >> 31);
}

class VoxelMemory {
public:
    VoxelMemory(float voxelSize, size_t capacity, uint64_t seed=7)
        : voxelSize_(voxelSize), capacity_(capacity), seed_(seed) {
        if (!(voxelSize>0) || !std::isfinite(voxelSize) || capacity<3)
            throw std::invalid_argument("voxel size must be positive and memory capacity >= 3");
    }

    bool insert(const Point& p) {
        if (!finite(p)) return false;
        const double x=std::floor(double(p.x)/voxelSize_);
        const double y=std::floor(double(p.y)/voxelSize_);
        const double z=std::floor(double(p.z)/voxelSize_);
        // Avoid undefined conversion and leave room for voxel-center arithmetic.
        constexpr double bound=9.0e18;
        if (std::abs(x)>bound || std::abs(y)>bound || std::abs(z)>bound) return false;
        const VoxelKey key{static_cast<int64_t>(x),static_cast<int64_t>(y),static_cast<int64_t>(z)};
        auto found=cells_.find(key);
        if (found!=cells_.end()) {
            // The observation closest to the voxel center wins. Repeated identical
            // views do not increase the statistical weight or move an average.
            if (representativeRank(p,key)<representativeRank(found->second,key))
                found->second=p;
            return true;
        }
        const Priority priority{priorityOf(key),key};
        if (cells_.size()==capacity_) {
            if (!(priority<priorities_.top())) return false;
            cells_.erase(priorities_.top().second);
            priorities_.pop();
            ++evictions_;
        }
        cells_.emplace(key,p);
        priorities_.push(priority);
        return true;
    }

    void insert(const std::vector<Point>& points) {
        for (const Point& p:points) insert(p);
    }
    size_t size() const noexcept { return cells_.size(); }
    size_t capacity() const noexcept { return capacity_; }
    size_t evictions() const noexcept { return evictions_; }

    std::vector<Point> representatives() const {
        std::vector<Point> out;
        out.reserve(cells_.size());
        for (const auto& [key,p]:cells_) out.push_back(p);
        return out;
    }

    std::vector<Point> replay(size_t budget, uint64_t epoch) const {
        if (budget<3) throw std::invalid_argument("replay budget must be >= 3");
        const std::vector<Point> all=representatives();
        if (all.size()<=budget) return all;
        // Equal-stride sampling of lexicographically ordered occupied voxels.
        // A seeded offset changes each epoch. Every selected voxel contributes
        // exactly one unweighted point, independent of its observation count.
        const size_t offset=mix64(seed_^epoch)%all.size();
        std::vector<Point> selected;
        selected.reserve(budget);
        for (size_t j=0;j<budget;++j)
            selected.push_back(all[(j*all.size()/budget+offset)%all.size()]);
        return selected;
    }

private:
    using Priority=std::pair<uint64_t,VoxelKey>;
    uint64_t priorityOf(const VoxelKey& k) const {
        return mix64(mix64(uint64_t(k.x)^seed_) ^
                     mix64(uint64_t(k.y)+UINT64_C(0x517cc1b727220a95)) ^
                     mix64(uint64_t(k.z)+UINT64_C(0x6eed0e9da4d94a4f)));
    }
    std::tuple<double,float,float,float> representativeRank(const Point& p,const VoxelKey& key) const {
        const double dx=p.x-(double(key.x)+0.5)*voxelSize_;
        const double dy=p.y-(double(key.y)+0.5)*voxelSize_;
        const double dz=p.z-(double(key.z)+0.5)*voxelSize_;
        return std::tuple{dx*dx+dy*dy+dz*dz,p.x,p.y,p.z};
    }
    float voxelSize_;
    size_t capacity_;
    uint64_t seed_;
    size_t evictions_=0;
    std::map<VoxelKey,Point> cells_;
    std::priority_queue<Priority> priorities_;
};

enum class Mode { Camera, World, Memory, MemoryUniform, MemoryDbl, MemoryFps };
inline Mode parseMode(const std::string& name) {
    if (name=="camera" || name=="camera_only") return Mode::Camera;
    if (name=="world" || name=="world_only") return Mode::World;
    if (name=="memory" || name=="world_memory") return Mode::Memory;
    if (name=="memory_uniform" || name=="world_memory_uniform") return Mode::MemoryUniform;
    if (name=="memory_dbl") return Mode::MemoryDbl;
    if (name=="memory_fps") return Mode::MemoryFps;
    throw std::invalid_argument("unknown mode: "+name);
}

struct Config {
    Mode mode=Mode::Memory;
    fgng::Config graph;
    size_t memoryCapacity=5000;
    size_t replayBudget=5000;
    float voxelSize=0.025F;
    int epochsPerFrame=1;
    float attentionFloor=0.15F;
};

struct UpdateStats {
    size_t inputPoints=0, validPoints=0, memoryPoints=0, replayPoints=0;
    size_t nodes=0, edges=0, evictions=0;
    int churn=0;
    double updateMs=0;
};

struct Graph {
    std::vector<Point> nodes;
    std::vector<int64_t> ids;
    std::vector<std::pair<uint16_t,uint16_t>> edges;
};

class WorldFoveatedGNG {
public:
    explicit WorldFoveatedGNG(Config config)
        : config_(validated(std::move(config))),
          memory_(config_.voxelSize,config_.memoryCapacity,config_.graph.seed) {
        if (config_.mode==Mode::MemoryDbl) {
            dblgng::Config c;
            c.maxNodes=config_.graph.maxNodes;
            c.initialRegions=config_.graph.initialRegions;
            c.seed=config_.graph.seed;
            c.winnerRate=config_.graph.winnerRate;
            c.neighborRate=config_.graph.neighborRate;
            c.errorDecay=config_.graph.errorDecay;
            c.insertionDecay=config_.graph.insertionDecay;
            dbl_=std::make_unique<dblgng::DistributedBatchLearningGNG>(c);
        } else if (config_.mode!=Mode::MemoryFps) {
            fg_=std::make_unique<fgng::FoveatedGNG>(config_.graph);
        }
    }

    UpdateStats update(const std::vector<Point>& cameraPoints,
                       const Pose& worldFromCamera, const Point& worldAttention,
                       float radius) {
        const auto start=std::chrono::steady_clock::now();
        worldFromCamera.validate();
        if (!finite(worldAttention) || !std::isfinite(radius))
            throw std::invalid_argument("attention must be finite; radius <= 0 disables it");
        lastPose_=worldFromCamera;
        UpdateStats stats;
        stats.inputPoints=cameraPoints.size();
        std::vector<Point> incoming;
        incoming.reserve(cameraPoints.size());
        for (const Point& p:cameraPoints) {
            if (!finite(p)) continue;
            const Point q=config_.mode==Mode::Camera ? p : worldFromCamera.toWorld(p);
            if (finite(q)) incoming.push_back(q);
        }
        stats.validPoints=incoming.size();
        const bool memoryMode=config_.mode==Mode::Memory || config_.mode==Mode::MemoryUniform ||
                              config_.mode==Mode::MemoryDbl || config_.mode==Mode::MemoryFps;
        if (memoryMode) memory_.insert(incoming);
        if (fg_) {
            if (config_.mode==Mode::MemoryUniform) {
                // Preserve the allocator's external-demand gate for a clean
                // spatial-attention ablation. An empty callback would also
                // disable capacity-triggered demand reallocation in the core.
                fg_->setAttention([](const Point&) { return 1.0F; });
            } else if (radius>0) {
                const Point center=config_.mode==Mode::Camera ? worldFromCamera.toCamera(worldAttention)
                                                              : worldAttention;
                const float floor=config_.attentionFloor;
                fg_->setAttention([center,radius,floor](const Point& p) {
                    const double x=double(p.x)-center.x,y=double(p.y)-center.y,z=double(p.z)-center.z;
                    return static_cast<float>(floor+(1-floor)/(1+(x*x+y*y+z*z)/(double(radius)*radius)));
                });
            } else fg_->setAttention({});
        }
        for (int i=0;i<config_.epochsPerFrame;++i) {
            const std::vector<Point> batch=memoryMode ? memory_.replay(config_.replayBudget,replayEpoch_)
                                                      : boundedCurrent(incoming,config_.replayBudget);
            ++replayEpoch_;
            stats.replayPoints=batch.size();
            if (dbl_) {
                std::vector<dblgng::Point3f> points;
                points.reserve(batch.size());
                for (const auto& p:batch) points.push_back({p.x,p.y,p.z});
                dbl_->learnBatch(points);
            } else if (fg_) {
                fg_->learnBatch(batch);
                if (batch.size()>=3) stats.churn+=fg_->churn();
            } else {
                fpsGraph_=farthestPoints(batch,static_cast<size_t>(config_.graph.maxNodes),
                                         static_cast<uint64_t>(config_.graph.seed));
            }
        }
        stats.memoryPoints=memory_.size();
        stats.evictions=memory_.evictions();
        const Graph out=graph();
        stats.nodes=out.nodes.size();
        stats.edges=out.edges.size();
        stats.updateMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        return stats;
    }

    Graph graph() const {
        Graph out;
        if (config_.mode==Mode::MemoryFps) return fpsGraph_;
        if (dbl_) {
            std::vector<dblgng::Point3f> nodes;
            dbl_->copyGraph(nodes,out.edges);
            for (const auto& p:nodes) out.nodes.push_back({p.x,p.y,p.z});
            for (size_t i=0;i<nodes.size();++i) out.ids.push_back(static_cast<int64_t>(i));
        } else {
            fg_->copyGraph(out.nodes,out.edges);
            fg_->copyIds(out.ids);
        }
        if (config_.mode==Mode::Camera)
            for (auto& p:out.nodes) p=lastPose_.toWorld(p);
        return out;
    }
    const VoxelMemory& memory() const noexcept { return memory_; }
    const Config& config() const noexcept { return config_; }

private:
    static Graph farthestPoints(const std::vector<Point>& points,size_t capacity,uint64_t seed) {
        Graph out;
        const size_t count=std::min(capacity,points.size());
        if (!count) return out;
        std::vector<double> distance(points.size(),std::numeric_limits<double>::infinity());
        size_t selected=mix64(seed)%points.size();
        out.nodes.reserve(count); out.ids.reserve(count);
        for (size_t i=0;i<count;++i) {
            const Point p=points[selected];
            out.nodes.push_back(p); out.ids.push_back(static_cast<int64_t>(i));
            for (size_t j=0;j<points.size();++j) {
                const double x=double(points[j].x)-p.x,y=double(points[j].y)-p.y,z=double(points[j].z)-p.z;
                distance[j]=std::min(distance[j],x*x+y*y+z*z);
            }
            // Mark used entries below all legitimate squared distances.
            distance[selected]=-1;
            selected=static_cast<size_t>(std::max_element(distance.begin(),distance.end())-distance.begin());
        }
        return out;
    }
    static Config validated(Config c) {
        if (c.graph.maxNodes<2 || c.graph.maxNodes>4096)
            throw std::invalid_argument("max nodes must be in [2,4096]");
        if (c.graph.initialRegions<1 || c.epochsPerFrame<1 || c.replayBudget<3 ||
            c.memoryCapacity<3 || !(c.voxelSize>0) || !std::isfinite(c.voxelSize))
            throw std::invalid_argument("invalid initialization, epoch, replay, or memory budget");
        if (!(c.attentionFloor>0 && c.attentionFloor<=1))
            throw std::invalid_argument("attention floor must be in (0,1]");
        for (float v:{c.graph.winnerRate,c.graph.neighborRate,c.graph.errorDecay,
                      c.graph.insertionDecay,c.graph.mu,c.graph.kappa,
                      c.graph.splitRatio,c.graph.mergeRatio})
            if (!std::isfinite(v)) throw std::invalid_argument("non-finite graph parameter");
        if (c.graph.winnerRate<0 || c.graph.winnerRate>1 || c.graph.neighborRate<0 ||
            c.graph.neighborRate>1 || c.graph.errorDecay<0 || c.graph.errorDecay>1 ||
            c.graph.insertionDecay<0 || c.graph.insertionDecay>1 || c.graph.mu<0 ||
            c.graph.kappa<0 || c.graph.mergeRatio<0 || c.graph.splitRatio<=1 ||
            c.graph.mergeRatio>=c.graph.splitRatio || c.graph.intrinsicDim<1 ||
            c.graph.intrinsicDim>3 || c.graph.hysteresis<1 || c.graph.reallocationPatience<1)
            throw std::invalid_argument("graph parameter outside supported range");
        return c;
    }
    static std::vector<Point> boundedCurrent(const std::vector<Point>& points,size_t cap) {
        if (points.size()<=cap) return points;
        std::vector<Point> selected;
        selected.reserve(cap);
        for (size_t j=0;j<cap;++j) selected.push_back(points[j*points.size()/cap]);
        return selected;
    }
    Config config_;
    VoxelMemory memory_;
    std::unique_ptr<fgng::FoveatedGNG> fg_;
    std::unique_ptr<dblgng::DistributedBatchLearningGNG> dbl_;
    Pose lastPose_;
    uint64_t replayEpoch_=0;
    Graph fpsGraph_;
};

} // namespace world_fgng
