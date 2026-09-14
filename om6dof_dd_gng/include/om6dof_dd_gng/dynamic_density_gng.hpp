#pragma once

// Semantic-attention Dynamic Density Growing Neural Gas for environment points.
// This is a bounded, portable adaptation of the legacy DD-GNG strength rules,
// not a bit-exact reproduction and not the robot reachability GNG. The legacy
// common factor 4^4 cancels when selecting max(error * strength^4).
//
// Each node has geometric attention strength S in [1,20]. After each update:
//   E <- E * (1 - error_decay / S^4), U <- U * (1 - utility_decay).
// A node/neighbor pair is selected for splitting by E*S^4. Uniform world
// sampling is retained: at most half of each partialFit batch is drawn from
// attention regions; all other draws use the complete valid point cloud.
// Optional recycling ranks eligible nodes by U*S (a soft preference, not a
// coverage/safety guarantee). Edges remain aged competitive-Hebbian edges.
// Semantic labels, detector scheduling, and attention expiry belong to caller.

#include "om6dof_dd_gng/ddgng.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace om6dof_dd_gng {

struct DensityAttentionRegion {
    GngPoint3f center;
    float radius = 0.10F;
    float strength = 1.0F;
};

struct DynamicDensityGngParameters {
    int max_nodes = 500;
    int insertion_interval = 100;
    int max_edge_age = 50;
    float winner_learning_rate = 0.08F;
    float neighbor_learning_rate = 0.0008F;
    float error_decay = 0.001F;
    float utility_decay = 0.001F;
    float attention_sample_fraction = 0.5F;
    bool protect_attention_utility = true;
    uint32_t random_seed = 7;
};

struct DynamicDensityGngStats {
    // Current configured non-neutral regions and current graph nodes with S>1.
    std::size_t active_attention_regions = 0;
    std::size_t focused_nodes = 0;
    // Cumulative successful update draws / insertions / capacity replacements
    // since reset. Uniform draws can also fall inside an attention region.
    uint64_t focused_samples = 0;
    uint64_t uniform_samples = 0;
    uint64_t insertions = 0;
    uint64_t recycled_nodes = 0;
};

class DynamicDensityGrowingNeuralGas {
public:
    explicit DynamicDensityGrowingNeuralGas(
        DynamicDensityGngParameters parameters = DynamicDensityGngParameters{})
        : parameters_(validated(parameters)), capacity_(parameters_.max_nodes),
          nodes_(capacity_), node_ids_(capacity_), errors_(capacity_),
          utility_(capacity_), strengths_(capacity_, 1.0F), degrees_(capacity_),
          ages_(static_cast<std::size_t>(capacity_) * capacity_, kNoEdge),
          rng_(parameters_.random_seed) {
        neighbors_.reserve(capacity_);
    }

    // Starts a new graph/ID epoch and also clears semantic attention. IDs are
    // never reused within an epoch, including when compacting/recycling nodes.
    void reset() {
        count_ = 0;
        total_updates_ = 0;
        next_node_id_ = 1;
        counters_ = DynamicDensityGngStats{};
        attention_regions_.clear();
        valid_indices_.clear();
        focused_indices_.clear();
        rng_.seed(parameters_.random_seed);
        std::fill(node_ids_.begin(), node_ids_.end(), 0);
        std::fill(errors_.begin(), errors_.end(), 0.0);
        std::fill(utility_.begin(), utility_.end(), 0.0);
        std::fill(strengths_.begin(), strengths_.end(), 1.0F);
        std::fill(degrees_.begin(), degrees_.end(), 0);
        std::fill(ages_.begin(), ages_.end(), kNoEdge);
    }

    // Replaces the complete attention snapshot; an empty vector immediately
    // removes all focus. Validate first so an invalid replacement is atomic.
    // S=1 regions are accepted as a neutral experimental control.
    void setAttentionRegions(const std::vector<DensityAttentionRegion>& regions) {
        if (regions.size() > kMaxAttentionRegions)
            throw std::invalid_argument("DD-GNG supports at most 64 attention regions");
        for (const auto& region : regions) {
            if (!finite(region.center) || !std::isfinite(region.radius) ||
                region.radius <= 0.0F || !std::isfinite(region.strength) ||
                region.strength < 1.0F || region.strength > 20.0F)
                throw std::invalid_argument(
                    "DD-GNG attention needs a finite center, radius>0, strength in [1,20]");
        }
        attention_regions_ = regions;
        refreshStrengths();
    }

    void partialFit(const std::vector<GngPoint3f>& points, int updates) {
        if (updates <= 0 || points.size() < 2) return;
        valid_indices_.clear();
        focused_indices_.clear();
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (!finite(points[i])) continue;
            valid_indices_.push_back(i);
            if (strengthAt(points[i]) > 1.0F) focused_indices_.push_back(i);
        }
        if (valid_indices_.size() < 2) return;
        if (count_ < 2) initialize(points);

        // Exact, randomly interleaved quota: every call has >=50% global draws,
        // not merely that expectation. For tiny batches floor() can give zero
        // focused draws. No focused candidates means an all-global fallback.
        int focused_remaining = focused_indices_.empty() ? 0 : static_cast<int>(
            std::floor(static_cast<double>(updates) * parameters_.attention_sample_fraction));
        std::uniform_int_distribution<std::size_t> global_pick(0, valid_indices_.size() - 1);
        for (int update = 0; update < updates; ++update) {
            bool focused = false;
            if (focused_remaining > 0) {
                std::uniform_int_distribution<int> choose(0, updates - update - 1);
                focused = choose(rng_) < focused_remaining;
            }
            std::size_t sample_index;
            if (focused) {
                std::uniform_int_distribution<std::size_t> focus_pick(0, focused_indices_.size() - 1);
                sample_index = focused_indices_[focus_pick(rng_)];
                --focused_remaining;
                ++counters_.focused_samples;
            } else {
                sample_index = valid_indices_[global_pick(rng_)];
                ++counters_.uniform_samples;
            }
            updateOne(points[sample_index]);
        }
    }

    void copyGraph(std::vector<GngPoint3f>& nodes,
                   std::vector<uint32_t>& node_ids,
                   std::vector<std::pair<uint16_t, uint16_t>>& edges) const {
        nodes.assign(nodes_.begin(), nodes_.begin() + count_);
        node_ids.assign(node_ids_.begin(), node_ids_.begin() + count_);
        edges.clear();
        for (int a = 0; a < count_; ++a)
            for (int b = a + 1; b < count_; ++b)
                if (edge(a, b) >= 0)
                    edges.emplace_back(static_cast<uint16_t>(a), static_cast<uint16_t>(b));
    }

    void copyNodeStrengths(std::vector<float>& strengths) const {
        strengths.assign(strengths_.begin(), strengths_.begin() + count_);
    }

    DynamicDensityGngStats stats() const {
        DynamicDensityGngStats result = counters_;
        for (const auto& region : attention_regions_)
            if (region.strength > 1.0F) ++result.active_attention_regions;
        for (int i = 0; i < count_; ++i)
            if (strengths_[i] > 1.0F) ++result.focused_nodes;
        return result;
    }

private:
    static constexpr int16_t kNoEdge = -1;
    static constexpr std::size_t kMaxAttentionRegions = 64;
    static constexpr double kMaxAccumulator = 1.0e250;

    static DynamicDensityGngParameters validated(DynamicDensityGngParameters p) {
        if (p.max_nodes < 2 || p.max_nodes > 4096)
            throw std::invalid_argument("DD-GNG max_nodes must be in [2,4096]");
        if (p.insertion_interval <= 0)
            throw std::invalid_argument("DD-GNG insertion_interval must be positive");
        if (p.max_edge_age < 0 || p.max_edge_age >= std::numeric_limits<int16_t>::max())
            throw std::invalid_argument("DD-GNG max_edge_age must be in [0,32766]");
        const auto unit_interval = [](float value) {
            return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
        };
        if (!unit_interval(p.winner_learning_rate) || !unit_interval(p.neighbor_learning_rate) ||
            !unit_interval(p.error_decay) || !unit_interval(p.utility_decay))
            throw std::invalid_argument("DD-GNG learning rates and decay parameters must be in [0,1]");
        if (!std::isfinite(p.attention_sample_fraction) || p.attention_sample_fraction < 0.0F ||
            p.attention_sample_fraction > 0.5F)
            throw std::invalid_argument("DD-GNG attention_sample_fraction must be in [0,0.5]");
        return p;
    }

    static bool finite(const GngPoint3f& point) {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    static double squaredDistance(const GngPoint3f& a, const GngPoint3f& b) {
        const double dx = static_cast<double>(a.x) - b.x;
        const double dy = static_cast<double>(a.y) - b.y;
        const double dz = static_cast<double>(a.z) - b.z;
        return dx * dx + dy * dy + dz * dz;
    }

    float strengthAt(const GngPoint3f& point) const {
        float strength = 1.0F;
        for (const auto& region : attention_regions_) {
            if (region.strength > strength && squaredDistance(point, region.center) <=
                static_cast<double>(region.radius) * region.radius)
                strength = region.strength;
        }
        return strength;
    }

    void refreshStrengths() {
        for (int i = 0; i < count_; ++i) strengths_[i] = strengthAt(nodes_[i]);
    }

    static double strengthFourth(float strength) {
        const double squared = static_cast<double>(strength) * strength;
        return squared * squared;
    }

    void moveToward(int index, const GngPoint3f& point, float rate) {
        // Double convex arithmetic avoids overflow for finite float extremes.
        const double keep = 1.0 - static_cast<double>(rate);
        nodes_[index].x = static_cast<float>(keep * nodes_[index].x + static_cast<double>(rate) * point.x);
        nodes_[index].y = static_cast<float>(keep * nodes_[index].y + static_cast<double>(rate) * point.y);
        nodes_[index].z = static_cast<float>(keep * nodes_[index].z + static_cast<double>(rate) * point.z);
        strengths_[index] = strengthAt(nodes_[index]);
    }

    int16_t edge(int a, int b) const {
        return ages_[static_cast<std::size_t>(a) * capacity_ + b];
    }

    void setEdge(int a, int b, int16_t age) {
        if (a == b) return;
        const bool existed = edge(a, b) >= 0;
        const bool exists = age >= 0;
        ages_[static_cast<std::size_t>(a) * capacity_ + b] = age;
        ages_[static_cast<std::size_t>(b) * capacity_ + a] = age;
        if (existed != exists) {
            const int delta = exists ? 1 : -1;
            degrees_[a] += delta;
            degrees_[b] += delta;
        }
    }

    uint32_t newNodeId() {
        if (next_node_id_ > std::numeric_limits<uint32_t>::max())
            throw std::overflow_error("DD-GNG node ID epoch exhausted; reset graph and semantic state");
        return static_cast<uint32_t>(next_node_id_++);
    }

    void initialize(const std::vector<GngPoint3f>& points) {
        std::uniform_int_distribution<std::size_t> first_pick(0, valid_indices_.size() - 1);
        const std::size_t first = first_pick(rng_);
        std::uniform_int_distribution<std::size_t> second_pick(0, valid_indices_.size() - 2);
        std::size_t second = second_pick(rng_);
        if (second >= first) ++second;
        nodes_[0] = points[valid_indices_[first]];
        nodes_[1] = points[valid_indices_[second]];
        node_ids_[0] = newNodeId();
        node_ids_[1] = newNodeId();
        count_ = 2;
        refreshStrengths();
        setEdge(0, 1, 0);
    }

    void updateOne(const GngPoint3f& point) {
        int winner = -1;
        int second = -1;
        double best = std::numeric_limits<double>::infinity();
        double next = std::numeric_limits<double>::infinity();
        for (int i = 0; i < count_; ++i) {
            const double distance = squaredDistance(nodes_[i], point);
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

        neighbors_.clear();
        for (int i = 0; i < count_; ++i) {
            const int16_t age = edge(winner, i);
            if (age < 0) continue;
            neighbors_.push_back(i);
            setEdge(winner, i, static_cast<int16_t>(
                std::min(static_cast<int>(age) + 1, static_cast<int>(std::numeric_limits<int16_t>::max()))));
        }
        errors_[winner] = std::min(kMaxAccumulator, errors_[winner] + best);
        utility_[winner] = std::min(kMaxAccumulator, utility_[winner] + std::max(0.0, next - best));
        moveToward(winner, point, parameters_.winner_learning_rate);
        for (int neighbor : neighbors_) moveToward(neighbor, point, parameters_.neighbor_learning_rate);
        setEdge(winner, second, 0);

        bool removed_edge = false;
        for (int i = 0; i < count_; ++i) {
            if (edge(winner, i) > parameters_.max_edge_age) {
                setEdge(winner, i, kNoEdge);
                removed_edge = true;
            }
        }
        if (removed_edge) removeIsolated();
        for (int i = 0; i < count_; ++i) {
            errors_[i] *= 1.0 - parameters_.error_decay / strengthFourth(strengths_[i]);
            utility_[i] *= 1.0 - parameters_.utility_decay;
        }
        ++total_updates_;
        if (total_updates_ % static_cast<uint64_t>(parameters_.insertion_interval) == 0) insertNode();
    }

    // Compact by moving the last node together with its ID and incident edges.
    // Degree updates are O(N); no all-pairs rescan is necessary after removal.
    void removeNode(int index) {
        if (count_ <= 2) return;
        const int last = count_ - 1;
        for (int i = 0; i < count_; ++i) setEdge(index, i, kNoEdge);
        if (index != last) {
            nodes_[index] = nodes_[last];
            node_ids_[index] = node_ids_[last];
            errors_[index] = errors_[last];
            utility_[index] = utility_[last];
            strengths_[index] = strengths_[last];
            for (int i = 0; i < last; ++i) {
                if (i == index) continue;
                const int16_t moved_age = edge(last, i);
                setEdge(index, i, moved_age);
                setEdge(last, i, kNoEdge);
            }
        }
        node_ids_[last] = 0;
        errors_[last] = utility_[last] = 0.0;
        strengths_[last] = 1.0F;
        degrees_[last] = 0;
        --count_;
    }

    void removeIsolated() {
        int index = 0;
        while (count_ > 2 && index < count_) {
            if (degrees_[index] == 0) removeNode(index);
            else ++index;
        }
    }

    double insertionPriority(int index) const {
        return errors_[index] * strengthFourth(strengths_[index]);
    }

    int indexOfId(uint32_t id) const {
        return static_cast<int>(std::find(node_ids_.begin(), node_ids_.begin() + count_, id) - node_ids_.begin());
    }

    void insertNode() {
        if (count_ < 2 || capacity_ == 2) return;
        int q = -1;
        for (int i = 0; i < count_; ++i)
            if (degrees_[i] > 0 && (q < 0 || insertionPriority(i) > insertionPriority(q))) q = i;
        if (q < 0) return;
        int f = -1;
        for (int i = 0; i < count_; ++i)
            if (edge(q, i) >= 0 && (f < 0 || insertionPriority(i) > insertionPriority(f))) f = i;
        if (f < 0) return;

        // Check ID exhaustion before any structural mutation. Keep the chosen
        // split edge intact while recycling, then relocate endpoints by ID.
        const uint32_t new_id = newNodeId();
        if (count_ >= capacity_) {
            int weakest = -1;
            double minimum_utility = std::numeric_limits<double>::infinity();
            for (int i = 0; i < count_; ++i) {
                if (i == q || i == f) continue;
                const double rank = utility_[i] * (parameters_.protect_attention_utility ? strengths_[i] : 1.0F);
                if (rank < minimum_utility) {
                    minimum_utility = rank;
                    weakest = i;
                }
            }
            const uint32_t q_id = node_ids_[q];
            const uint32_t f_id = node_ids_[f];
            removeNode(weakest);
            ++counters_.recycled_nodes;
            removeIsolated();
            q = indexOfId(q_id);
            f = indexOfId(f_id);
        }
        const int r = count_++;
        nodes_[r] = {
            static_cast<float>((static_cast<double>(nodes_[q].x) + nodes_[f].x) * 0.5),
            static_cast<float>((static_cast<double>(nodes_[q].y) + nodes_[f].y) * 0.5),
            static_cast<float>((static_cast<double>(nodes_[q].z) + nodes_[f].z) * 0.5)};
        node_ids_[r] = new_id;
        strengths_[r] = strengthAt(nodes_[r]);
        errors_[q] *= 0.5;
        errors_[f] *= 0.5;
        errors_[r] = errors_[q];
        utility_[r] = (utility_[q] + utility_[f]) * 0.5;
        degrees_[r] = 0;
        setEdge(q, f, kNoEdge);
        setEdge(q, r, 0);
        setEdge(f, r, 0);
        ++counters_.insertions;
    }

    DynamicDensityGngParameters parameters_;
    int capacity_;
    int count_ = 0;
    uint64_t total_updates_ = 0;
    uint64_t next_node_id_ = 1;
    DynamicDensityGngStats counters_;
    std::vector<GngPoint3f> nodes_;
    std::vector<uint32_t> node_ids_;
    std::vector<double> errors_;
    std::vector<double> utility_;
    std::vector<float> strengths_;
    std::vector<int> degrees_;
    std::vector<int16_t> ages_;
    std::vector<int> neighbors_;
    std::vector<DensityAttentionRegion> attention_regions_;
    std::vector<std::size_t> valid_indices_;
    std::vector<std::size_t> focused_indices_;
    std::mt19937 rng_;
};

}  // namespace om6dof_dd_gng
