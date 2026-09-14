#pragma once

// DBL-GNG algorithm adapted from CornerSiow/DBL-GNG (MIT License), 2024.
// This core is intentionally independent of RealSense, Win32, and the viewer.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>
#include <vector>

namespace dblgng {

struct Point3f {
    float x;
    float y;
    float z;
};

struct Config {
    int maxNodes = 500;
    int initialRegions = 10;
    int pruneInterval = 10;
    int edgeCutInterval = 0;
    int seed = 7;
    float winnerRate = 0.5F;
    float neighborRate = 0.01F;
    float errorDecay = 0.5F;
    float insertionDecay = 0.5F;
    float growthQuantile = 0.85F;
};

class DistributedBatchLearningGNG {
public:
    explicit DistributedBatchLearningGNG(const Config& config)
        : capacity_(config.maxNodes), initialRegions_(config.initialRegions),
          pruneInterval_(config.pruneInterval), edgeCutInterval_(config.edgeCutInterval),
          alpha_(config.winnerRate), beta_(config.neighborRate),
          delta_(config.errorDecay), rho_(config.insertionDecay),
          growthQuantile_(config.growthQuantile),
          nodes_(capacity_), delta1_(capacity_), delta2_(capacity_), nodeScratch_(capacity_),
          errors_(capacity_), errorScratch_(capacity_),
          activation1_(capacity_), activation2_(capacity_), strength_(matrixSize(), 0),
          activation1Scratch_(capacity_), activation2Scratch_(capacity_),
          strengthScratch_(matrixSize(), 0),
          edgeEvidence_(matrixSize(), 0), edgeEvidenceScratch_(matrixSize(), 0),
          adjacency_(matrixSize(), 0), adjacencyScratch_(matrixSize(), 0),
          mapping_(capacity_, -1), rng_(static_cast<uint32_t>(config.seed)) {}

    void reset() {
        count_ = 0;
        epochs_ = 0;
        std::fill(errors_.begin(), errors_.end(), 0.0F);
        std::fill(adjacency_.begin(), adjacency_.end(), uint8_t{0});
        std::fill(strength_.begin(), strength_.end(), uint32_t{0});
        std::fill(edgeEvidence_.begin(), edgeEvidence_.end(), uint64_t{0});
    }

    void learnBatch(const std::vector<Point3f>& points) {
        if (points.size() < 3) return;
        if (count_ < 2 && !initializeDistributed(points)) return;

        const int batchNodes = count_;
        std::fill(delta1_.begin(), delta1_.begin() + batchNodes, Point3f{});
        std::fill(delta2_.begin(), delta2_.begin() + batchNodes, Point3f{});
        std::fill(activation1_.begin(), activation1_.begin() + batchNodes, uint32_t{0});
        std::fill(activation2_.begin(), activation2_.begin() + batchNodes, uint32_t{0});
        for (int i = 0; i < batchNodes; ++i) {
            std::fill_n(strength_.begin() + static_cast<size_t>(i) * capacity_,
                        batchNodes, uint32_t{0});
        }

        // The old adjacency remains immutable for the complete batch, as required
        // by DBL-GNG. Only accumulators and the new edge-strength matrix change.
        for (const Point3f& point : points) {
            int winner = -1;
            int second = -1;
            float best = std::numeric_limits<float>::max();
            float next = std::numeric_limits<float>::max();
            for (int i = 0; i < batchNodes; ++i) {
                const float d2 = squaredDistance(point, nodes_[i]);
                if (d2 < best) {
                    next = best;
                    second = winner;
                    best = d2;
                    winner = i;
                } else if (d2 < next) {
                    next = d2;
                    second = i;
                }
            }
            if (winner < 0 || second < 0) continue;

            errors_[winner] += alpha_ * std::sqrt(std::max(0.0F, best));
            addScaled(delta1_[winner], point, nodes_[winner], alpha_);
            ++activation1_[winner];

            const size_t winnerRow = static_cast<size_t>(winner) * capacity_;
            for (int neighbor = 0; neighbor < batchNodes; ++neighbor) {
                if (adjacency_[winnerRow + neighbor]) {
                    addScaled(delta2_[neighbor], point, nodes_[neighbor], beta_);
                    ++activation2_[neighbor];
                }
            }

            uint32_t& forward = strength_[winnerRow + second];
            if (forward != std::numeric_limits<uint32_t>::max()) ++forward;
            strength_[static_cast<size_t>(second) * capacity_ + winner] = forward;
        }

        for (int i = 0; i < batchNodes; ++i) {
            const float inv1 = 1.0F / (static_cast<float>(activation1_[i]) + kEpsilon);
            const float inv2 = 1.0F / (static_cast<float>(activation2_[i]) + kEpsilon);
            nodes_[i].x += delta1_[i].x * inv1 + delta2_[i].x * inv2;
            nodes_[i].y += delta1_[i].y * inv1 + delta2_[i].y * inv2;
            nodes_[i].z += delta1_[i].z * inv1 + delta2_[i].z * inv2;
        }

        rebuildTopologyFromStrength();
        accumulateEdgeEvidence(batchNodes);
        removeIsolatedNodes();
        for (int i = 0; i < count_; ++i) errors_[i] *= delta_;

        ++epochs_;
        if (pruneInterval_ > 0 && epochs_ % static_cast<uint64_t>(pruneInterval_) == 0)
            removeNonActivatedNodes();
        growDistributed();
        if (edgeCutInterval_ > 0 && epochs_ % static_cast<uint64_t>(edgeCutInterval_) == 0)
            applyEdgeCut();
    }

    void copyGraph(std::vector<Point3f>& nodes,
                   std::vector<std::pair<uint16_t, uint16_t>>& edges) const {
        nodes.assign(nodes_.begin(), nodes_.begin() + count_);
        edges.clear();
        edges.reserve(static_cast<size_t>(count_) * 3);
        for (int a = 0; a < count_; ++a) {
            for (int b = a + 1; b < count_; ++b) {
                if (isConnected(a, b))
                    edges.emplace_back(static_cast<uint16_t>(a), static_cast<uint16_t>(b));
            }
        }
    }

    uint64_t epoch() const noexcept { return epochs_; }
    int nodeCount() const noexcept { return count_; }

    // Paper-style edge-strength pruning. The streaming ROS adapter invokes
    // this once after all learning epochs for one depth frame have completed.
    void cutWeakEdges() { applyEdgeCut(); }

private:
    static constexpr float kEpsilon = 1e-4F;

    size_t matrixSize() const {
        return static_cast<size_t>(capacity_) * static_cast<size_t>(capacity_);
    }
    size_t matrixIndex(int a, int b) const {
        return static_cast<size_t>(a) * capacity_ + b;
    }
    bool isConnected(int a, int b) const { return adjacency_[matrixIndex(a, b)] != 0; }
    void setConnection(int a, int b, bool connected, uint32_t edgeStrength = 0) {
        adjacency_[matrixIndex(a, b)] = adjacency_[matrixIndex(b, a)] = connected ? 1 : 0;
        strength_[matrixIndex(a, b)] = strength_[matrixIndex(b, a)] = connected ? edgeStrength : 0;
    }
    static float squaredDistance(const Point3f& a, const Point3f& b) {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz;
    }
    static void addScaled(Point3f& accumulator, const Point3f& point,
                          const Point3f& node, float rate) {
        accumulator.x += rate * (point.x - node.x);
        accumulator.y += rate * (point.y - node.y);
        accumulator.z += rate * (point.z - node.z);
    }

    bool initializeDistributed(const std::vector<Point3f>& points) {
        int regions = std::min({initialRegions_, capacity_ / 2,
                                static_cast<int>(points.size() / 3)});
        if (regions < 1) return false;

        std::vector<Point3f> candidates(points);
        std::shuffle(candidates.begin(), candidates.end(), rng_);
        const size_t batchSize = std::max<size_t>(3, candidates.size() / regions);

        for (int region = 0; region < regions && candidates.size() >= 3 && count_ + 2 <= capacity_;
             ++region) {
            const size_t tailSize = std::min(batchSize, candidates.size());
            std::uniform_int_distribution<size_t> pick(candidates.size() - tailSize,
                                                       candidates.size() - 1);
            const Point3f current = candidates[pick(rng_)];

            order_.resize(candidates.size());
            for (size_t i = 0; i < candidates.size(); ++i) order_[i] = i;
            std::sort(order_.begin(), order_.end(), [&](size_t a, size_t b) {
                return squaredDistance(current, candidates[a]) < squaredDistance(current, candidates[b]);
            });
            const Point3f neighbor = candidates[order_[2]];
            const int first = count_++;
            const int second = count_++;
            nodes_[first] = current;
            nodes_[second] = neighbor;
            errors_[first] = errors_[second] = 0.0F;
            activation1_[first] = activation1_[second] = 1;
            activation2_[first] = activation2_[second] = 1;
            setConnection(first, second, true, 1);

            const size_t removeCount = std::min(batchSize, candidates.size());
            candidateScratch_.clear();
            candidateScratch_.reserve(candidates.size() - removeCount);
            for (size_t i = removeCount; i < order_.size(); ++i)
                candidateScratch_.push_back(candidates[order_[i]]);
            candidates.swap(candidateScratch_);
        }
        return count_ >= 2;
    }

    void rebuildTopologyFromStrength() {
        for (int i = 0; i < count_; ++i)
            std::fill_n(adjacency_.begin() + static_cast<size_t>(i) * capacity_, count_, uint8_t{0});
        for (int a = 0; a < count_; ++a) {
            for (int b = a + 1; b < count_; ++b) {
                if (strength_[matrixIndex(a, b)] > 0) {
                    adjacency_[matrixIndex(a, b)] = 1;
                    adjacency_[matrixIndex(b, a)] = 1;
                }
            }
        }
    }

    void removeIsolatedNodes() {
        if (count_ <= 2) return;
        keep_.assign(static_cast<size_t>(count_), uint8_t{0});
        int kept = 0;
        for (int i = 0; i < count_; ++i) {
            for (int j = 0; j < count_; ++j) {
                if (isConnected(i, j)) {
                    keep_[i] = 1;
                    ++kept;
                    break;
                }
            }
        }
        if (kept >= 2 && kept < count_) compactNodes(keep_);
    }

    void removeNonActivatedNodes() {
        if (count_ <= 2) return;
        keep_.assign(static_cast<size_t>(count_), uint8_t{0});
        int kept = 0;
        for (int i = 0; i < count_; ++i) {
            if (activation1_[i] > 0) {
                keep_[i] = 1;
                ++kept;
            }
        }
        if (kept >= 2 && kept < count_) compactNodes(keep_);
    }

    void compactNodes(const std::vector<uint8_t>& keep) {
        std::fill(mapping_.begin(), mapping_.end(), -1);
        int newCount = 0;
        for (int old = 0; old < count_; ++old) {
            if (!keep[old]) continue;
            mapping_[old] = newCount;
            nodeScratch_[newCount] = nodes_[old];
            errorScratch_[newCount] = errors_[old];
            activation1Scratch_[newCount] = activation1_[old];
            activation2Scratch_[newCount] = activation2_[old];
            ++newCount;
        }
        if (newCount < 2 || newCount == count_) return;

        std::fill(adjacencyScratch_.begin(), adjacencyScratch_.end(), uint8_t{0});
        std::fill(strengthScratch_.begin(), strengthScratch_.end(), uint32_t{0});
        std::fill(edgeEvidenceScratch_.begin(), edgeEvidenceScratch_.end(), uint64_t{0});
        for (int a = 0; a < count_; ++a) {
            if (mapping_[a] < 0) continue;
            for (int b = a + 1; b < count_; ++b) {
                if (mapping_[b] < 0 || !isConnected(a, b)) continue;
                const int na = mapping_[a];
                const int nb = mapping_[b];
                adjacencyScratch_[matrixIndex(na, nb)] = adjacencyScratch_[matrixIndex(nb, na)] = 1;
                const uint32_t s = strength_[matrixIndex(a, b)];
                strengthScratch_[matrixIndex(na, nb)] = strengthScratch_[matrixIndex(nb, na)] = s;
                const uint64_t evidence = edgeEvidence_[matrixIndex(a, b)];
                edgeEvidenceScratch_[matrixIndex(na, nb)] =
                    edgeEvidenceScratch_[matrixIndex(nb, na)] = evidence;
            }
        }
        std::copy_n(nodeScratch_.begin(), newCount, nodes_.begin());
        std::copy_n(errorScratch_.begin(), newCount, errors_.begin());
        std::copy_n(activation1Scratch_.begin(), newCount, activation1_.begin());
        std::copy_n(activation2Scratch_.begin(), newCount, activation2_.begin());
        adjacency_.swap(adjacencyScratch_);
        strength_.swap(strengthScratch_);
        edgeEvidence_.swap(edgeEvidenceScratch_);
        count_ = newCount;
    }

    void accumulateEdgeEvidence(int batchNodes) {
        for (int a=0;a<batchNodes;++a) for (int b=a+1;b<batchNodes;++b) {
            const uint64_t increment=strength_[matrixIndex(a,b)];
            uint64_t& forward=edgeEvidence_[matrixIndex(a,b)];
            forward=increment>std::numeric_limits<uint64_t>::max()-forward ?
                std::numeric_limits<uint64_t>::max() : forward+increment;
            edgeEvidence_[matrixIndex(b,a)]=forward;
        }
    }

    static float linearQuantile(std::vector<float>& values, float probability) {
        if (values.empty()) return 0.0F;
        std::sort(values.begin(), values.end());
        const float position = (values.size() - 1) * std::clamp(probability, 0.0F, 1.0F);
        const size_t low = static_cast<size_t>(std::floor(position));
        const size_t high = static_cast<size_t>(std::ceil(position));
        const float fraction = position - static_cast<float>(low);
        return values[low] + (values[high] - values[low]) * fraction;
    }

    void growDistributed() {
        if (count_ < 2 || count_ >= capacity_) return;
        quantileScratch_.assign(errors_.begin(), errors_.begin() + count_);
        const float threshold = linearQuantile(quantileScratch_, growthQuantile_);
        int additions = 0;
        for (int i = 0; i < count_; ++i) if (errors_[i] > threshold) ++additions;
        if (additions == 0) additions = 1; // Avoid a stalled graph when errors are tied.
        additions = std::min(additions, capacity_ - count_);

        for (int insertion = 0; insertion < additions; ++insertion) {
            int q1 = -1;
            int q2 = -1;
            for (int candidate = 0; candidate < count_; ++candidate) {
                int bestNeighbor = -1;
                for (int neighbor = 0; neighbor < count_; ++neighbor) {
                    if (isConnected(candidate, neighbor) &&
                        (bestNeighbor < 0 || errors_[neighbor] > errors_[bestNeighbor]))
                        bestNeighbor = neighbor;
                }
                if (bestNeighbor >= 0 && (q1 < 0 || errors_[candidate] > errors_[q1])) {
                    q1 = candidate;
                    q2 = bestNeighbor;
                }
            }
            if (q1 < 0 || q2 < 0 || count_ >= capacity_) break;

            const int q3 = count_++;
            nodes_[q3] = {(nodes_[q1].x + nodes_[q2].x) * 0.5F,
                          (nodes_[q1].y + nodes_[q2].y) * 0.5F,
                          (nodes_[q1].z + nodes_[q2].z) * 0.5F};
            errors_[q1] *= rho_;
            errors_[q2] *= rho_;
            errors_[q3] = (errors_[q1] + errors_[q2]) * 0.5F;
            activation1_[q3] = activation2_[q3] = 1;
            for (int i = 0; i < count_; ++i) {
                adjacency_[matrixIndex(q3, i)] = adjacency_[matrixIndex(i, q3)] = 0;
                strength_[matrixIndex(q3, i)] = strength_[matrixIndex(i, q3)] = 0;
                edgeEvidence_[matrixIndex(q3, i)] = edgeEvidence_[matrixIndex(i, q3)] = 0;
            }
            setConnection(q1, q2, false);
            setConnection(q1, q3, true, 1);
            setConnection(q2, q3, true, 1);
        }
    }

    void applyEdgeCut() {
        removeNonActivatedNodes();
        edgeStrengthScratch_.clear();
        for (int a = 0; a < count_; ++a)
            for (int b = a + 1; b < count_; ++b)
                if (isConnected(a, b) && edgeEvidence_[matrixIndex(a, b)] > 0)
                    edgeStrengthScratch_.push_back(static_cast<float>(edgeEvidence_[matrixIndex(a, b)]));
        if (edgeStrengthScratch_.empty()) {
            std::fill(edgeEvidence_.begin(),edgeEvidence_.end(),uint64_t{0});
            return;
        }
        const float threshold = linearQuantile(edgeStrengthScratch_, 0.15F);
        for (int a = 0; a < count_; ++a) {
            for (int b = a + 1; b < count_; ++b) {
                // Equation (33) retains only S_ij > percentile(S, p), so an
                // edge exactly on the threshold is cut as well.
                if (isConnected(a, b) && static_cast<float>(edgeEvidence_[matrixIndex(a, b)]) <= threshold)
                    setConnection(a, b, false);
            }
        }
        removeIsolatedNodes();
        std::fill(edgeEvidence_.begin(),edgeEvidence_.end(),uint64_t{0});
    }

    int capacity_;
    int initialRegions_;
    int pruneInterval_;
    int edgeCutInterval_;
    float alpha_;
    float beta_;
    float delta_;
    float rho_;
    float growthQuantile_;
    int count_ = 0;
    uint64_t epochs_ = 0;
    std::vector<Point3f> nodes_, delta1_, delta2_, nodeScratch_;
    std::vector<float> errors_, errorScratch_, quantileScratch_, edgeStrengthScratch_;
    std::vector<uint32_t> activation1_, activation2_, strength_;
    std::vector<uint32_t> activation1Scratch_, activation2Scratch_, strengthScratch_;
    std::vector<uint64_t> edgeEvidence_,edgeEvidenceScratch_;
    std::vector<uint8_t> adjacency_, adjacencyScratch_, keep_;
    std::vector<int> mapping_;
    std::vector<size_t> order_;
    std::vector<Point3f> candidateScratch_;
    std::mt19937 rng_;
};

} // namespace dblgng
