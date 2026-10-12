#pragma once

// ATC-DT core following Toda & Masuyama, IEEE Access 2024, Sec. IV:
// winner selection and node addition (Eq. 2--3), decaying winner/neighbour
// updates (Eq. 4--5), adaptive edge-age threshold (Eq. 9--10), PCA normals
// (Eq. 11--12), and traversability topology (Eq. 13--14).
// This is deliberately a distinct core from the GNG variants in this package.

#include "world_fgng.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

namespace atcdt {

using Point = world_fgng::Point;

struct Config {
    float vigilance = 0.025F;             // V_thr, metres.
    float maxSlopeRadians = 0.523599F;    // deg_max: 30 degrees.
    float normalSimilarityRadians = 0.349066F; // normal topology decision f_nor: 20 degrees.
    size_t maxNodes = 2048;               // Runtime guard; paper itself has no fixed cap.
    uint32_t seed = 7;                    // Reproducible random Step-0 references.
};

struct Node {
    Point position{};
    Point normal{};
    uint64_t id{};
    uint64_t wins{1};
    bool traversable{false};
};

class Mapper {
public:
    explicit Mapper(Config config = {}) : config_(std::move(config)), random_(config_.seed) { validate(); }

    void learn(const std::vector<Point>& observations) {
        if (nodes_.empty()) initialise(observations);
        for (const auto& point : observations) if (world_fgng::finite(point)) learnOne(point);
    }

    size_t nodeCount() const { return nodes_.size(); }
    const std::vector<Node>& nodes() const { return nodes_; }

    world_fgng::Graph positionGraph() const {
        world_fgng::Graph graph;
        graph.nodes.reserve(nodes_.size()); graph.ids.reserve(nodes_.size());
        for (const auto& node : nodes_) { graph.nodes.push_back(node.position); graph.ids.push_back(node.id); }
        for (const auto& [edge, age] : positionEdges_) {
            (void)age;
            graph.edges.emplace_back(edge.first, edge.second);
        }
        return graph;
    }

    std::vector<Point> normals() const {
        std::vector<Point> result; result.reserve(nodes_.size());
        for (const auto& node : nodes_) result.push_back(node.normal);
        return result;
    }

    // Separate topologies specified by ATC-DT. Indices refer to positionGraph().nodes.
    std::vector<std::pair<size_t,size_t>> normalEdges() const { return keys(normalEdges_); }
    std::vector<std::pair<size_t,size_t>> traversabilityEdges() const { return keys(traversabilityEdges_); }

    void removeNodes(const std::vector<bool>& remove) {
        if (remove.size()!=nodes_.size()) throw std::invalid_argument("ATC deletion mask size mismatch");
        std::vector<int> map(nodes_.size(),-1); std::vector<Node> kept;
        kept.reserve(nodes_.size());
        for (size_t i=0;i<nodes_.size();++i) if (!remove[i]) { map[i]=static_cast<int>(kept.size()); kept.push_back(nodes_[i]); }
        if (kept.size()<2 || kept.size()==nodes_.size()) return;
        std::vector<std::map<size_t,uint32_t>> adjacency(kept.size());
        for (size_t a=0;a<nodes_.size();++a) if (map[a]>=0)
            for (const auto& [b,age]:adjacency_[a]) if (map[b]>=0)
                adjacency[static_cast<size_t>(map[a])][static_cast<size_t>(map[b])]=age;
        nodes_.swap(kept); adjacency_.swap(adjacency);
        positionEdges_.clear(); normalEdges_.clear(); traversabilityEdges_.clear();
        for (size_t a=0;a<adjacency_.size();++a) for (const auto& [b,age]:adjacency_[a]) if (a<b)
            positionEdges_[edge(a,b)]=age;
    }

private:
    using Edge = std::pair<size_t,size_t>;
    static Edge edge(size_t a, size_t b) { return a < b ? Edge{a,b} : Edge{b,a}; }
    static float distance(const Point& a, const Point& b) {
        const float x=a.x-b.x, y=a.y-b.y, z=a.z-b.z; return std::sqrt(x*x+y*y+z*z);
    }
    static float dot(const Point& a, const Point& b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
    static Point normalised(Point point) {
        const float length=std::sqrt(dot(point,point));
        if (length <= 1e-8F) return {};
        point.x/=length; point.y/=length; point.z/=length; return point;
    }
    static float percentile(std::vector<uint32_t> values, float fraction) {
        if (values.empty()) return 1.0F;
        std::sort(values.begin(), values.end());
        const float index=fraction*static_cast<float>(values.size()-1);
        const size_t lo=static_cast<size_t>(std::floor(index));
        const size_t hi=static_cast<size_t>(std::ceil(index));
        return static_cast<float>(values[lo])+(index-static_cast<float>(lo))*
               static_cast<float>(values[hi]-values[lo]);
    }
    static std::vector<Edge> keys(const std::map<Edge,uint32_t>& source) {
        std::vector<Edge> result; result.reserve(source.size());
        for (const auto& [key, value] : source) { (void)value; result.push_back(key); }
        return result;
    }

    void validate() const {
        if (!std::isfinite(config_.vigilance) || config_.vigilance<=0 || config_.vigilance>10)
            throw std::invalid_argument("atc_vigilance must be finite and in (0, 10] metres");
        if (!std::isfinite(config_.maxSlopeRadians) || config_.maxSlopeRadians<=0 ||
            config_.maxSlopeRadians>1.570797F)
            throw std::invalid_argument("atc_max_slope_deg must be in (0, 90]");
        if (!std::isfinite(config_.normalSimilarityRadians) || config_.normalSimilarityRadians<0 ||
            config_.normalSimilarityRadians>1.570797F)
            throw std::invalid_argument("atc_normal_similarity_deg must be in [0, 90]");
        if (config_.maxNodes<2 || config_.maxNodes>100000)
            throw std::invalid_argument("atc_max_nodes must be in [2, 100000]");
    }

    void initialise(const std::vector<Point>& observations) {
        std::vector<Point> valid;
        valid.reserve(observations.size());
        for (const auto& point : observations) if (world_fgng::finite(point)) valid.push_back(point);
        if (valid.empty()) return;
        // Paper Step 0: randomly choose two reference vectors from P. The PRNG
        // is seeded so an identical recorded depth sequence is reproducible.
        std::uniform_int_distribution<size_t> pick(0,valid.size()-1);
        nodes_.push_back({valid[pick(random_)],{},nextId_++,1,false});
        nodes_.push_back({valid[pick(random_)],{},nextId_++,1,false});
        adjacency_.resize(2);
    }

    void learnOne(const Point& point) {
        if (nodes_.empty()) return;
        if (nodes_.size()==1) { nodes_.push_back({point,{},nextId_++,1,false}); return; }

        size_t first=0, second=1;
        float firstDistance=distance(point,nodes_[0].position);
        float secondDistance=distance(point,nodes_[1].position);
        if (secondDistance<firstDistance) { std::swap(first,second); std::swap(firstDistance,secondDistance); }
        for (size_t index=2;index<nodes_.size();++index) {
            const float value=distance(point,nodes_[index].position);
            if (value<firstDistance) { second=first; secondDistance=firstDistance; first=index; firstDistance=value; }
            else if (value<secondDistance) { second=index; secondDistance=value; }
        }

        if (firstDistance>=config_.vigilance) {
            // Eq. (3). At the runtime cap, retain the stable learnt map rather
            // than silently replacing old nodes (a behaviour not in the paper).
            if (nodes_.size()<config_.maxNodes) { nodes_.push_back({point,{},nextId_++,1,false}); adjacency_.emplace_back(); }
            return;
        }

        // Paper Eq. (4): winner-rate decays with its win count.
        move(nodes_[first].position,point,1.0F/(10.0F*static_cast<float>(nodes_[first].wins)));
        ++nodes_[first].wins;

        if (secondDistance<config_.vigilance) {
            // Eq. (5): only existing position neighbours of the winner move.
            for (const auto& [index, age] : adjacency_[first]) {
                (void)age;
                move(nodes_[index].position,point,1.0F/(100.0F*static_cast<float>(nodes_[index].wins)));
            }
        }

        // Eq. (6) and the position edge rule in Step 5.
        for (auto& [neighbor, age] : adjacency_[first]) {
            ++age; adjacency_[neighbor][first]=age; positionEdges_[edge(first,neighbor)]=age;
        }
        if (secondDistance<config_.vigilance) {
            adjacency_[first][second]=adjacency_[second][first]=1;
            positionEdges_[edge(first,second)]=1;
        }

        // Eq. (9)--(10), then Step 6. The threshold is recomputed from current
        // incident ages and deleted-edge ages, exactly as the paper describes.
        pruneAgedEdges(first);
        updateLocalAttributes(first);
        if (secondDistance<config_.vigilance) updateAttributeEdges(first,second);
    }

    static void move(Point& destination, const Point& source, float rate) {
        destination.x+=rate*(source.x-destination.x);
        destination.y+=rate*(source.y-destination.y);
        destination.z+=rate*(source.z-destination.z);
    }

    void pruneAgedEdges(size_t winner) {
        std::vector<uint32_t> current;
        for (const auto& [neighbor, age] : adjacency_[winner]) { (void)neighbor; current.push_back(age); }
        const float q1=percentile(current,0.25F), q3=percentile(current,0.75F);
        const float gthr=q3+(q3-q1);
        const float deletedMean=deletedAges_.empty() ? gthr :
            std::accumulate(deletedAges_.begin(),deletedAges_.end(),0.0F)/static_cast<float>(deletedAges_.size());
        const float denominator=static_cast<float>(deletedAges_.size()+current.size());
        const float gmax=denominator>0 ? deletedMean*static_cast<float>(deletedAges_.size())/denominator+
            gthr*static_cast<float>(current.size())/denominator : gthr;
        std::vector<Edge> expired;
        for (const auto& [neighbor, age] : adjacency_[winner])
            if (static_cast<float>(age)>gmax) expired.push_back(edge(winner,neighbor));
        for (const auto& key : expired) {
            deletedAges_.push_back(positionEdges_.at(key)); positionEdges_.erase(key);
            adjacency_[key.first].erase(key.second); adjacency_[key.second].erase(key.first);
            normalEdges_.erase(key); traversabilityEdges_.erase(key);
        }
    }

    // Jacobi eigensolver for a symmetric 3x3 covariance. The eigenvector of
    // the smallest eigenvalue is the PCA surface normal in Eq. (11).
    static Point smallestEigenvector(std::array<std::array<float,3>,3> matrix) {
        std::array<std::array<float,3>,3> vectors{{{{1,0,0}},{{0,1,0}},{{0,0,1}}}};
        for (int iteration=0;iteration<18;++iteration) {
            int p=0,q=1;
            if (std::abs(matrix[0][2])>std::abs(matrix[p][q])) { p=0; q=2; }
            if (std::abs(matrix[1][2])>std::abs(matrix[p][q])) { p=1; q=2; }
            if (std::abs(matrix[p][q])<1e-8F) break;
            const float angle=0.5F*std::atan2(2.0F*matrix[p][q],matrix[q][q]-matrix[p][p]);
            const float c=std::cos(angle),s=std::sin(angle);
            for (int k=0;k<3;++k) if (k!=p && k!=q) {
                const float kp=matrix[k][p],kq=matrix[k][q];
                matrix[k][p]=matrix[p][k]=c*kp-s*kq;
                matrix[k][q]=matrix[q][k]=s*kp+c*kq;
            }
            const float pp=matrix[p][p],qq=matrix[q][q],pq=matrix[p][q];
            matrix[p][p]=c*c*pp-2*s*c*pq+s*s*qq;
            matrix[q][q]=s*s*pp+2*s*c*pq+c*c*qq;
            matrix[p][q]=matrix[q][p]=0;
            for (int k=0;k<3;++k) {
                const float vp=vectors[k][p],vq=vectors[k][q];
                vectors[k][p]=c*vp-s*vq; vectors[k][q]=s*vp+c*vq;
            }
        }
        int smallest=0; if (matrix[1][1]<matrix[smallest][smallest]) smallest=1;
        if (matrix[2][2]<matrix[smallest][smallest]) smallest=2;
        Point result{vectors[0][smallest],vectors[1][smallest],vectors[2][smallest]};
        result=normalised(result);
        // PCA has a sign ambiguity. Orient toward +Z to make Eq. (12) a slope.
        if (result.z<0) { result.x=-result.x; result.y=-result.y; result.z=-result.z; }
        return result;
    }

    void updateLocalAttributes(size_t index) {
        std::vector<size_t> neighbours;
        neighbours.reserve(adjacency_[index].size());
        for (const auto& [neighbor, age] : adjacency_[index]) { (void)age; neighbours.push_back(neighbor); }
        if (neighbours.size()<2) { nodes_[index].normal={}; nodes_[index].traversable=false; return; }
        std::array<std::array<float,3>,3> covariance{};
        for (const auto neighbour : neighbours) {
            const Point d{nodes_[neighbour].position.x-nodes_[index].position.x,
                          nodes_[neighbour].position.y-nodes_[index].position.y,
                          nodes_[neighbour].position.z-nodes_[index].position.z};
            const float values[3]{d.x,d.y,d.z};
            for (int r=0;r<3;++r) for (int c=0;c<3;++c) covariance[r][c]+=values[r]*values[c];
        }
        nodes_[index].normal=smallestEigenvector(covariance);
        const float z=std::clamp(nodes_[index].normal.z,-1.0F,1.0F);
        nodes_[index].traversable=std::acos(z)<config_.maxSlopeRadians;
    }

    void updateAttributeEdges(size_t first, size_t second) {
        const Edge key=edge(first,second);
        const Point& a=nodes_[first].normal; const Point& b=nodes_[second].normal;
        if (std::sqrt(dot(a,a))>1e-6F && std::sqrt(dot(b,b))>1e-6F &&
            std::abs(dot(a,b))>=std::cos(config_.normalSimilarityRadians)) normalEdges_[key]=1;
        else normalEdges_.erase(key);
        if (nodes_[first].traversable==nodes_[second].traversable) traversabilityEdges_[key]=1;
        else traversabilityEdges_.erase(key);
    }

    Config config_;
    std::vector<Node> nodes_;
    std::map<Edge,uint32_t> positionEdges_,normalEdges_,traversabilityEdges_;
    // Sparse neighbour lookup preserves the exact same undirected edges while
    // avoiding a full graph scan for every ATC-DT winner update.
    std::vector<std::map<size_t,uint32_t>> adjacency_;
    std::vector<uint32_t> deletedAges_;
    uint64_t nextId_=1;
    std::mt19937 random_;
};

} // namespace atcdt
