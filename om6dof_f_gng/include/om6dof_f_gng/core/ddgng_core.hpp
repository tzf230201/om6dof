#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>
#include <vector>

struct GngPoint3f {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

// Table I of Saputra et al., IROS 2019.  lambda is supplied as the number of
// samples to partialFit(); the remaining values are stored here so that the
// ROS profile and the learning rule cannot silently diverge.
struct DDGNGConfig {
    int maxNodes = 100;       // g_max
    int maxEdgeAge = 10;      // kappa
    float winnerRate = 0.08F; // eta_1
    float neighborRate = 0.008F; // eta_2
    float alpha = 0.5F;
    float beta = 0.005F;
    float chi = 0.005F;
    float mu = 0.5F;
    float utilityK = 1000.0F; // k
};

// Dynamic Growing Neural Gas core used by the optional DD-GNG + YOLO mode.
// Node degrees are tracked explicitly so isolated-node removal does not scan
// the complete adjacency matrix for every online update.
class DynamicGrowingNeuralGas {
public:
    explicit DynamicGrowingNeuralGas(int maxNodes) : DynamicGrowingNeuralGas(DDGNGConfig{.maxNodes=maxNodes}) {}
    explicit DynamicGrowingNeuralGas(DDGNGConfig config = {})
        : config_(validated(config)), capacity_(config_.maxNodes), nodes_(capacity_), nodeIds_(capacity_), errors_(capacity_),
          utility_(capacity_), strength_(capacity_, 1.0F), degrees_(capacity_),
          ages_(static_cast<size_t>(capacity_) * capacity_, kNoEdge), rng_(7) {
        neighbors_.reserve(capacity_);
    }

    void reset() {
        count_ = 0;
        totalUpdates_ = 0;
        nextNodeId_ = 1;
        rng_.seed(7);
        std::fill(nodeIds_.begin(), nodeIds_.end(), 0);
        std::fill(errors_.begin(), errors_.end(), 0.0F);
        std::fill(utility_.begin(), utility_.end(), 0.0F);
        std::fill(strength_.begin(), strength_.end(), 1.0F);
        std::fill(degrees_.begin(), degrees_.end(), 0);
        std::fill(ages_.begin(), ages_.end(), kNoEdge);
    }

    void partialFit(const std::vector<GngPoint3f>& points, int updates,
                    const std::vector<float>& attention = {}) {
        if (points.size() < 2 || updates <= 0) return;
        if (!attention.empty() && attention.size()!=points.size())
            throw std::invalid_argument("DD-GNG attention weights must match points");
        if (count_ < 2) initialize(points);
        if (count_ < 2) return;

        std::uniform_int_distribution<size_t> uniformPick(0, points.size() - 1);
        std::vector<double> density;
        if (!attention.empty()) {
            density.reserve(attention.size());
            for (const float a:attention)
                density.push_back(config_.mu+(1.0-config_.mu)*std::max(0.0F,a));
        }
        std::discrete_distribution<size_t> focusedPick(density.begin(),density.end());
        for (int update = 0; update < updates; ++update) {
            const GngPoint3f point = points[attention.empty() ? uniformPick(rng_) : focusedPick(rng_)];
            int winner = -1;
            int second = -1;
            float best = std::numeric_limits<float>::max();
            float next = std::numeric_limits<float>::max();

            for (int i = 0; i < count_; ++i) {
                const float dx = nodes_[i].x - point.x;
                const float dy = nodes_[i].y - point.y;
                const float dz = nodes_[i].z - point.z;
                const float distance = dx * dx + dy * dy + dz * dz;
                if (distance < best) {
                    next = best;
                    second = winner;
                    best = distance;
                    winner = i;
                } else if (distance < next) {
                    next = distance;
                    second = i;
                }
            }
            if (winner < 0 || second < 0) continue;

            neighbors_.clear();
            const size_t row = static_cast<size_t>(winner) * capacity_;
            for (int i = 0; i < count_; ++i) {
                int16_t& age = ages_[row + i];
                if (age >= 0) {
                    neighbors_.push_back(i);
                    ++age;
                    ages_[static_cast<size_t>(i) * capacity_ + winner] = age;
                }
            }

            errors_[winner] += best;
            utility_[winner] += next - best;
            moveToward(nodes_[winner], point, config_.winnerRate);
            for (const int neighbor : neighbors_)
                moveToward(nodes_[neighbor], point, config_.neighborRate);
            setEdge(winner, second, 0);

            bool removedEdge = false;
            for (int i = 0; i < count_; ++i) {
                if (ages_[row + i] > config_.maxEdgeAge) {
                    setEdge(winner, i, kNoEdge);
                    removedEdge = true;
                }
            }
            if (removedEdge) removeIsolated();

            for (int i = 0; i < count_; ++i) {
                errors_[i] *= 1.0F-config_.beta;
                utility_[i] *= 1.0F-config_.chi;
            }
            ++totalUpdates_;
            if (totalUpdates_ % 100 == 0) insertNode();
        }
    }

    void copyGraph(std::vector<GngPoint3f>& nodes,
                   std::vector<uint32_t>& nodeIds,
                   std::vector<std::pair<uint16_t, uint16_t>>& edges) const {
        nodes.assign(nodes_.begin(), nodes_.begin() + count_);
        nodeIds.assign(nodeIds_.begin(), nodeIds_.begin() + count_);
        edges.clear();
        for (int a = 0; a < count_; ++a) {
            for (int b = a + 1; b < count_; ++b) {
                if (edge(a, b) >= 0)
                    edges.emplace_back(static_cast<uint16_t>(a),
                                       static_cast<uint16_t>(b));
            }
        }
    }

private:
    static DDGNGConfig validated(DDGNGConfig c) {
        if (c.maxNodes < 2 || c.maxNodes > 4096 || c.maxEdgeAge < 1 ||
            !std::isfinite(c.winnerRate) || !std::isfinite(c.neighborRate) ||
            !std::isfinite(c.alpha) || !std::isfinite(c.beta) || !std::isfinite(c.chi) ||
            !std::isfinite(c.mu) || !std::isfinite(c.utilityK) ||
            c.winnerRate <= 0 || c.neighborRate <= 0 || c.alpha <= 0 || c.alpha >= 1 ||
            c.beta < 0 || c.beta >= 1 || c.chi < 0 || c.chi >= 1 ||
            c.mu < 0 || c.mu > 1 || c.utilityK <= 0)
            throw std::invalid_argument("invalid DD-GNG paper parameters");
        return c;
    }

    static constexpr int16_t kNoEdge = -1;

    static void moveToward(GngPoint3f& node, const GngPoint3f& point,
                           float rate) {
        node.x += rate * (point.x - node.x);
        node.y += rate * (point.y - node.y);
        node.z += rate * (point.z - node.z);
    }

    int16_t edge(int a, int b) const {
        return ages_[static_cast<size_t>(a) * capacity_ + b];
    }

    void setEdge(int a, int b, int16_t age) {
        if (a < 0 || b < 0 || a >= capacity_ || b >= capacity_) return;
        const bool existed = edge(a, b) >= 0;
        const bool exists = age >= 0;
        ages_[static_cast<size_t>(a) * capacity_ + b] = age;
        ages_[static_cast<size_t>(b) * capacity_ + a] = age;
        if (a != b && existed != exists) {
            const int delta = exists ? 1 : -1;
            degrees_[a] += delta;
            degrees_[b] += delta;
        }
    }

    void initialize(const std::vector<GngPoint3f>& points) {
        std::uniform_int_distribution<size_t> pick(0, points.size() - 1);
        const size_t first = pick(rng_);
        size_t second = pick(rng_);
        while (second == first) second = pick(rng_);
        nodes_[0] = points[first];
        nodes_[1] = points[second];
        nodeIds_[0] = nextNodeId_++;
        nodeIds_[1] = nextNodeId_++;
        count_ = 2;
        degrees_[0] = degrees_[1] = 0;
        setEdge(0, 1, 0);
    }

    void recomputeDegrees() {
        std::fill(degrees_.begin(), degrees_.end(), 0);
        for (int a = 0; a < count_; ++a) {
            for (int b = a + 1; b < count_; ++b) {
                if (edge(a, b) >= 0) {
                    ++degrees_[a];
                    ++degrees_[b];
                }
            }
        }
    }

    void removeNode(int index) {
        if (count_ <= 2 || index < 0 || index >= count_) return;
        const int last = count_ - 1;
        if (index != last) {
            nodes_[index] = nodes_[last];
            nodeIds_[index] = nodeIds_[last];
            errors_[index] = errors_[last];
            utility_[index] = utility_[last];
            strength_[index] = strength_[last];
            for (int k = 0; k < last; ++k) {
                const int16_t moved = edge(last, k);
                setEdge(index, k, moved);
            }
            setEdge(index, index, kNoEdge);
        }
        for (int k = 0; k < count_; ++k) setEdge(last, k, kNoEdge);
        errors_[last] = utility_[last] = 0.0F;
        strength_[last] = 1.0F;
        nodeIds_[last] = 0;
        --count_;
        recomputeDegrees();
    }

    void removeIsolated() {
        while (count_ > 2) {
            const auto isolated = std::find(degrees_.begin(),
                                            degrees_.begin() + count_, 0);
            if (isolated == degrees_.begin() + count_) break;
            removeNode(static_cast<int>(isolated - degrees_.begin()));
        }
    }

    void insertNode() {
        if (count_ < 2) return;
        if (count_ >= capacity_) {
            const int weakest = static_cast<int>(std::min_element(utility_.begin(), utility_.begin() + count_) - utility_.begin());
            const float largestError=*std::max_element(errors_.begin(),errors_.begin()+count_);
            // GNG-U retention from the DD-GNG paper: only replace a weak node
            // when its utility is insufficient relative to its local strength.
            if (largestError <= config_.utilityK*strength_[weakest]*std::max(utility_[weakest],1e-12F)) return;
            removeNode(weakest);
            removeIsolated();
        }
        if (count_ < 2) return;
        const int q = static_cast<int>(
            std::max_element(errors_.begin(), errors_.begin() + count_) -
            errors_.begin());
        int f = -1;
        for (int i = 0; i < count_; ++i) {
            if (edge(q, i) >= 0 && (f < 0 || errors_[i] > errors_[f])) f = i;
        }
        if (f < 0 || count_ >= capacity_) return;
        const int r = count_++;
        nodes_[r] = {(nodes_[q].x + nodes_[f].x) * 0.5F,
                     (nodes_[q].y + nodes_[f].y) * 0.5F,
                     (nodes_[q].z + nodes_[f].z) * 0.5F};
        nodeIds_[r] = nextNodeId_++;
        errors_[q] *= config_.alpha;
        errors_[f] *= config_.alpha;
        errors_[r] = errors_[q];
        utility_[r] = (utility_[q] + utility_[f]) * 0.5F;
        strength_[r] = (strength_[q] + strength_[f]) * 0.5F;
        degrees_[r] = 0;
        setEdge(q, f, kNoEdge);
        setEdge(q, r, 0);
        setEdge(f, r, 0);
    }

    DDGNGConfig config_;
    int capacity_;
    int count_ = 0;
    uint64_t totalUpdates_ = 0;
    uint32_t nextNodeId_ = 1;
    std::vector<GngPoint3f> nodes_;
    std::vector<uint32_t> nodeIds_;
    std::vector<float> errors_;
    std::vector<float> utility_;
    std::vector<float> strength_;
    std::vector<int> degrees_;
    std::vector<int16_t> ages_;
    std::vector<int> neighbors_;
    std::mt19937 rng_;
};
