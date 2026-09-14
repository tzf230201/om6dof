#pragma once

// F-GNG: Foveated Growing Neural Gas.
//
// Batch learning and the distributed seeding come from DBL-GNG
// (CornerSiow/DBL-GNG, MIT License, 2024).  What F-GNG adds on top:
//
//   1. An explicit density target per node, so how non-uniform the graph should
//      be becomes a parameter instead of a side effect of the error criterion.
//   2. A merge operator, so a region can lose nodes again.  Plain DBL-GNG only
//      ever gains them, which makes density a ratchet.
//   3. Persistent node ids, so a downstream policy can name the same node
//      across frames even though array positions are reshuffled constantly.
//   4. Hysteresis on split and merge, so the node set actually settles.
//
// The density target is the part that is easy to get wrong:
//
//     rho_i  ~ mass_i^mu * spacing_i^(D*(1-mu)) * (1 + kappa*complexity_i)
//     ratio_i = rho_i * maxNodes            // nodes this region deserves
//
// mu has to be applied to mass per unit territory, not to mass.  Mass alone is
// the quantity split/merge is controlling, so feeding it straight back settles
// at equal mass per node for *any* positive mu, and mu only changes how fast it
// gets there.  Folding in spacing^D makes mu a real magnification exponent:
// mu=0 spreads nodes evenly and ignores the data, mu=1 gives equal mass per
// node, mu=2 crowds dense regions on purpose.
//
// Top-down attention (the fovea) multiplies into rho in updateDensity() via
// setAttention(); everything else stays as it is.
//
// Like dblgng_core.hpp this is deliberately independent of RealSense, Win32 and
// the viewer, and allocates nothing after construction.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fgng {

struct Point3f {
    float x;
    float y;
    float z;
};

struct Config {
    int maxNodes = 500;
    int initialRegions = 10;
    int seed = 7;

    float winnerRate = 0.5F;      // alpha
    float neighborRate = 0.01F;   // beta
    float errorDecay = 0.5F;      // delta
    float insertionDecay = 0.5F;  // error split between q1/q2 on insertion

    // Density control.
    float mu = 1.0F;              // magnification exponent on local data density
    float kappa = 1.0F;           // weight of local geometric complexity
    // Dimension of the manifold the data lies on, not of the feature space:
    // a depth surface in 3D is intrinsically 2D.
    int intrinsicDim = 2;

    float splitRatio = 1.35F;
    float mergeRatio = 0.45F;
    // Consecutive epochs a node must stay out of balance before anything fires.
    int hysteresis = 3;
    // Multiples of `hysteresis` a node must go on being starved before the
    // graph will force a merge elsewhere to serve it.  Routine imbalance is
    // resolved by ordinary split/merge within a few epochs; only a genuine
    // deadlock lasts this long, so this must stay well above 1 or the graph
    // reshuffles itself every epoch.
    int reallocationPatience = 8;

    // How many retired ids stay resolvable.  Bounded on purpose: an unbounded
    // alias table is a slow leak in a session that runs for hours.
    int aliasHistory = 4096;

    // Semantics.  0 disables label tracking entirely and costs nothing.
    int labelCount = 0;
    // Evidence is an exponential accumulator, not a per-frame assignment:
    // detections flicker, and a node has to outvote its own noise over time.
    float labelDecay = 0.95F;
    // Floor on the semantic weight, for the same reason the fovea has one --
    // driving non-target classes to zero collapses the map into the target.
    float classWeightFloor = 0.15F;
};

enum class NodeState { Live, Merged, Gone };

struct NodeRef {
    NodeState state;
    int index;  // -1 when Gone
};

class FoveatedGNG {
public:
    explicit FoveatedGNG(const Config& config)
        : capacity_(config.maxNodes), initialRegions_(config.initialRegions),
          alpha_(config.winnerRate), beta_(config.neighborRate),
          delta_(config.errorDecay), insertionDecay_(config.insertionDecay),
          mu_(config.mu), kappa_(config.kappa),
          intrinsicDim_(config.intrinsicDim),
          splitRatio_(config.splitRatio), mergeRatio_(config.mergeRatio),
          hysteresis_(std::max(1, config.hysteresis)),
          reallocationPatience_(std::max(1, config.reallocationPatience)),
          aliasHistory_(std::max(0, config.aliasHistory)),
          labelCount_(std::max(0, config.labelCount)),
          labelDecay_(config.labelDecay),
          classWeightFloor_(config.classWeightFloor),
          nodes_(capacity_), delta1_(capacity_), delta2_(capacity_),
          nodeScratch_(capacity_),
          errors_(capacity_), errorScratch_(capacity_),
          mass_(capacity_), neighborMass_(capacity_),
          massScratch_(capacity_), neighborMassScratch_(capacity_),
          spacing_(capacity_), complexity_(capacity_),
          rho_(capacity_), ratio_(capacity_),
          ids_(capacity_), idScratch_(capacity_),
          pressure_(capacity_), pressureScratch_(capacity_),
          labelEvidence_(labelRowSize() * capacity_, 0.0F),
          labelScratch_(labelRowSize() * capacity_, 0.0F),
          classWeights_(static_cast<size_t>(std::max(1, labelCount_)), 1.0F),
          strength_(matrixSize(), 0), strengthScratch_(matrixSize(), 0),
          adjacency_(matrixSize(), 0), adjacencyScratch_(matrixSize(), 0),
          keep_(capacity_, 0), consumed_(capacity_, 0),
          mapping_(capacity_, -1), order_(capacity_),
          rng_(static_cast<uint32_t>(config.seed)) {}

    void reset() {
        count_ = 0;
        epochs_ = 0;
        churn_ = 0;
        nextId_ = 0;
        alias_.clear();
        aliasOrder_.clear();
        retired_.clear();
        std::fill(errors_.begin(), errors_.end(), 0.0F);
        std::fill(pressure_.begin(), pressure_.end(), 0);
        std::fill(adjacency_.begin(), adjacency_.end(), uint8_t{0});
        std::fill(strength_.begin(), strength_.end(), uint32_t{0});
        std::fill(labelEvidence_.begin(), labelEvidence_.end(), 0.0F);
    }

    // Per-class multiplier folded into rho.  This is semantic attention: it
    // raises the node budget for a class wherever it happens to be, instead of
    // pointing at one place in the image.  Empty restores uniform weights.
    void setClassWeights(const std::vector<float>& weights) {
        if (labelCount_ <= 0) return;
        std::fill(classWeights_.begin(), classWeights_.end(), 1.0F);
        const size_t count = std::min(weights.size(), classWeights_.size());
        for (size_t i = 0; i < count; ++i) classWeights_[i] = std::max(0.0F, weights[i]);
        weightedClasses_ = false;
        for (float w : classWeights_)
            if (std::fabs(w - 1.0F) > 1e-3F) { weightedClasses_ = true; break; }
        externalDemand_ = static_cast<bool>(attention_) || weightedClasses_;
    }

    // Top-down attention.  Returns a positive weight for a position; the fovea
    // will be a hyperbolic falloff in image space.  Empty means uniform.
    void setAttention(std::function<float(const Point3f&)> field) {
        attention_ = std::move(field);
        externalDemand_ = static_cast<bool>(attention_) || weightedClasses_;
    }

    void setMu(float mu) noexcept { mu_ = mu; }
    void setKappa(float kappa) noexcept { kappa_ = kappa; }

    // `weights` is optional and must match `points` when given.  It exists for
    // foveated sampling: once the periphery is sampled more coarsely than the
    // fovea, raw win counts measure the sampling pattern rather than the data,
    // which would quietly corrupt the mu magnification.  Pass the inverse
    // sampling density (roughly step^2) to cancel that out.
    // `labels` is optional and must match `points` when given: the semantic
    // class of the pixel each point came from, or a negative value for "no
    // label", which contributes no evidence rather than voting for background.
    void learnBatch(const std::vector<Point3f>& points,
                    const std::vector<float>* weights = nullptr,
                    const std::vector<int>* labels = nullptr) {
        if (points.size() < 3) return;
        if (weights != nullptr && weights->size() != points.size()) return;
        if (labels != nullptr && labels->size() != points.size()) return;
        if (count_ < 2 && !initializeDistributed(points)) return;

        idsBefore_.clear();
        for (int i = 0; i < count_; ++i) idsBefore_.insert(ids_[i]);

        accumulateBatch(points, weights, labels);
        applyBatch();
        rebuildTopologyFromStrength();
        removeIsolatedNodes();
        for (int i = 0; i < count_; ++i) errors_[i] *= delta_;

        updateDensity();
        updatePressure();
        lastMerged_ = mergeOverDenseNodes();
        lastMerged_ += reallocateForDemand();
        lastSplit_ = splitUnderServedNodes();
        removeIsolatedNodes();
        updateDensity();

        churn_ = 0;
        for (int i = 0; i < count_; ++i)
            if (idsBefore_.find(ids_[i]) == idsBefore_.end()) ++churn_;
        for (int64_t id : idsBefore_) {
            bool alive = false;
            for (int i = 0; i < count_ && !alive; ++i) alive = ids_[i] == id;
            if (!alive) ++churn_;
        }
        ++epochs_;
    }

    // ---------------------------------------------------------------- access

    void copyGraph(std::vector<Point3f>& nodes,
                   std::vector<std::pair<uint16_t, uint16_t>>& edges) const {
        nodes.assign(nodes_.begin(), nodes_.begin() + count_);
        edges.clear();
        edges.reserve(static_cast<size_t>(count_) * 3);
        for (int a = 0; a < count_; ++a)
            for (int b = a + 1; b < count_; ++b)
                if (isConnected(a, b))
                    edges.emplace_back(static_cast<uint16_t>(a), static_cast<uint16_t>(b));
    }

    void copyIds(std::vector<int64_t>& ids) const {
        ids.assign(ids_.begin(), ids_.begin() + count_);
    }
    void copyRatios(std::vector<float>& ratios) const {
        ratios.assign(ratio_.begin(), ratio_.begin() + count_);
    }

    // -------------------------------------------------------------- semantics

    // Winning class and how dominant it is (share of accumulated evidence).
    // Class -1 with confidence 0 means the node has never been labelled.
    std::pair<int, float> dominantLabel(int index) const {
        if (labelCount_ <= 0 || index < 0 || index >= count_) return {-1, 0.0F};
        const float* row = &labelEvidence_[static_cast<size_t>(index) * labelCount_];
        float total = 0.0F;
        float best = 0.0F;
        int bestClass = -1;
        for (int k = 0; k < labelCount_; ++k) {
            total += row[k];
            if (row[k] > best) { best = row[k]; bestClass = k; }
        }
        if (total <= kTiny) return {-1, 0.0F};
        return {bestClass, best / total};
    }

    void copyLabels(std::vector<int>& classes, std::vector<float>& confidence) const {
        classes.resize(count_);
        confidence.resize(count_);
        for (int i = 0; i < count_; ++i) {
            const auto [label, score] = dominantLabel(i);
            classes[i] = label;
            confidence[i] = score;
        }
    }

    // Nodes that currently stand for a class -- this is what an instruction
    // grounds onto.  Returns ids, not indices: indices are reshuffled by the
    // next merge, ids are not.
    void findByLabel(int label, float minConfidence, std::vector<int64_t>& ids) const {
        ids.clear();
        for (int i = 0; i < count_; ++i) {
            const auto [nodeLabel, score] = dominantLabel(i);
            if (nodeLabel == label && score >= minConfidence) ids.push_back(ids_[i]);
        }
    }

    int labelCount() const noexcept { return labelCount_; }

    int64_t resolve(int64_t id) const {
        std::unordered_set<int64_t> seen;
        auto it = alias_.find(id);
        while (it != alias_.end() && seen.insert(id).second) {
            id = it->second;
            it = alias_.find(id);
        }
        return id;
    }

    int indexOf(int64_t id) const {
        const int64_t target = resolve(id);
        for (int i = 0; i < count_; ++i)
            if (ids_[i] == target) return i;
        return -1;
    }

    // Live, Merged and Gone are kept apart on purpose.  Merged means the node's
    // mass was absorbed and `index` is a fair stand-in; Gone means the region
    // stopped existing and there is nothing to redirect to.  Collapsing both
    // into -1 lets a policy keep chasing a target that is no longer there.
    NodeRef status(int64_t id) const {
        const int64_t target = resolve(id);
        const int index = indexOf(target);
        if (index < 0) return {NodeState::Gone, -1};
        return {target == id ? NodeState::Live : NodeState::Merged, index};
    }

    int nodeCount() const noexcept { return count_; }
    uint64_t epoch() const noexcept { return epochs_; }
    int churn() const noexcept { return churn_; }
    int lastSplit() const noexcept { return lastSplit_; }
    int lastMerged() const noexcept { return lastMerged_; }
    int64_t idsMinted() const noexcept { return nextId_; }
    const Point3f& node(int index) const { return nodes_[index]; }
    float ratio(int index) const { return ratio_[index]; }
    bool connected(int a, int b) const { return isConnected(a, b); }

private:
    static constexpr float kEpsilon = 1e-4F;
    static constexpr float kTiny = 1e-8F;

    size_t matrixSize() const {
        return static_cast<size_t>(capacity_) * static_cast<size_t>(capacity_);
    }
    size_t labelRowSize() const { return static_cast<size_t>(std::max(0, labelCount_)); }
    float* labelRow(int index) {
        return &labelEvidence_[static_cast<size_t>(index) * labelRowSize()];
    }
    size_t matrixIndex(int a, int b) const {
        return static_cast<size_t>(a) * capacity_ + b;
    }
    bool isConnected(int a, int b) const { return adjacency_[matrixIndex(a, b)] != 0; }
    void setConnection(int a, int b, bool connected, uint32_t edgeStrength = 0) {
        adjacency_[matrixIndex(a, b)] = adjacency_[matrixIndex(b, a)] = connected ? 1 : 0;
        strength_[matrixIndex(a, b)] = strength_[matrixIndex(b, a)] =
            connected ? edgeStrength : 0;
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

    int64_t mintId() { return nextId_++; }

    void recordAlias(int64_t from, int64_t to) {
        alias_[from] = to;
        aliasOrder_.push_back(from);
        while (aliasHistory_ > 0 &&
               static_cast<int>(aliasOrder_.size()) > aliasHistory_) {
            // Evicted references degrade to Gone rather than growing forever.
            alias_.erase(aliasOrder_.front());
            aliasOrder_.pop_front();
        }
    }

    // ------------------------------------------------------------ seeding

    bool initializeDistributed(const std::vector<Point3f>& points) {
        int regions = std::min({initialRegions_, capacity_ / 2,
                                static_cast<int>(points.size() / 3)});
        if (regions < 1) return false;

        candidates_.assign(points.begin(), points.end());
        std::shuffle(candidates_.begin(), candidates_.end(), rng_);
        const size_t batchSize = std::max<size_t>(3, candidates_.size() / regions);

        for (int region = 0;
             region < regions && candidates_.size() >= 3 && count_ + 2 <= capacity_;
             ++region) {
            const size_t tailSize = std::min(batchSize, candidates_.size());
            std::uniform_int_distribution<size_t> pick(candidates_.size() - tailSize,
                                                       candidates_.size() - 1);
            const Point3f current = candidates_[pick(rng_)];

            sortOrder_.resize(candidates_.size());
            for (size_t i = 0; i < candidates_.size(); ++i) sortOrder_[i] = i;
            std::sort(sortOrder_.begin(), sortOrder_.end(), [&](size_t a, size_t b) {
                return squaredDistance(current, candidates_[a]) <
                       squaredDistance(current, candidates_[b]);
            });
            // Third closest, so the seed pair starts with some room between them.
            const Point3f neighbor = candidates_[sortOrder_[2]];

            const int first = count_++;
            const int second = count_++;
            nodes_[first] = current;
            nodes_[second] = neighbor;
            errors_[first] = errors_[second] = 0.0F;
            mass_[first] = mass_[second] = 0.0F;
            neighborMass_[first] = neighborMass_[second] = 0.0F;
            pressure_[first] = pressure_[second] = 0;
            ids_[first] = mintId();
            ids_[second] = mintId();
            setConnection(first, second, true, 1);

            const size_t removeCount = std::min(batchSize, candidates_.size());
            candidateScratch_.clear();
            candidateScratch_.reserve(candidates_.size() - removeCount);
            for (size_t i = removeCount; i < sortOrder_.size(); ++i)
                candidateScratch_.push_back(candidates_[sortOrder_[i]]);
            candidates_.swap(candidateScratch_);
        }
        if (count_ >= 2) updateDensity();
        return count_ >= 2;
    }

    // ------------------------------------------------------------ learning

    void accumulateBatch(const std::vector<Point3f>& points,
                         const std::vector<float>* weights,
                         const std::vector<int>* labels) {
        const int batchNodes = count_;
        // Evidence persists across frames and only decays -- unlike mass_,
        // which is zeroed every batch.  That is what lets a node outvote a
        // flickering detector instead of following it frame by frame.
        if (labelCount_ > 0)
            for (size_t i = 0; i < labelRowSize() * batchNodes; ++i)
                labelEvidence_[i] *= labelDecay_;
        std::fill(delta1_.begin(), delta1_.begin() + batchNodes, Point3f{});
        std::fill(delta2_.begin(), delta2_.begin() + batchNodes, Point3f{});
        std::fill(mass_.begin(), mass_.begin() + batchNodes, 0.0F);
        std::fill(neighborMass_.begin(), neighborMass_.begin() + batchNodes, 0.0F);
        for (int i = 0; i < batchNodes; ++i)
            std::fill_n(strength_.begin() + static_cast<size_t>(i) * capacity_,
                        batchNodes, uint32_t{0});

        // The old adjacency stays immutable for the whole batch, as DBL-GNG
        // requires.  Only accumulators and the new strength matrix change.
        for (size_t p = 0; p < points.size(); ++p) {
            const Point3f& point = points[p];
            const float weight = weights != nullptr ? (*weights)[p] : 1.0F;
            if (!(weight > 0.0F)) continue;

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

            errors_[winner] += alpha_ * weight * std::sqrt(std::max(0.0F, best));
            addScaled(delta1_[winner], point, nodes_[winner], alpha_ * weight);
            mass_[winner] += weight;

            if (labelCount_ > 0 && labels != nullptr) {
                const int label = (*labels)[p];
                if (label >= 0 && label < labelCount_)
                    labelRow(winner)[label] += weight;
            }

            const size_t winnerRow = static_cast<size_t>(winner) * capacity_;
            for (int neighbor = 0; neighbor < batchNodes; ++neighbor) {
                if (isConnected(winner,neighbor)) {
                    addScaled(delta2_[neighbor], point, nodes_[neighbor], beta_ * weight);
                    neighborMass_[neighbor] += weight;
                }
            }

            uint32_t& forward = strength_[winnerRow + second];
            if (forward != std::numeric_limits<uint32_t>::max()) ++forward;
            strength_[static_cast<size_t>(second) * capacity_ + winner] = forward;
        }
    }

    void applyBatch() {
        for (int i = 0; i < count_; ++i) {
            const float inv1 = 1.0F / (mass_[i] + kEpsilon);
            const float inv2 = 1.0F / (neighborMass_[i] + kEpsilon);
            nodes_[i].x += delta1_[i].x * inv1 + delta2_[i].x * inv2;
            nodes_[i].y += delta1_[i].y * inv1 + delta2_[i].y * inv2;
            nodes_[i].z += delta1_[i].z * inv1 + delta2_[i].z * inv2;
        }
    }

    void rebuildTopologyFromStrength() {
        for (int i = 0; i < count_; ++i)
            std::fill_n(adjacency_.begin() + static_cast<size_t>(i) * capacity_,
                        count_, uint8_t{0});
        for (int a = 0; a < count_; ++a)
            for (int b = a + 1; b < count_; ++b)
                if (strength_[matrixIndex(a, b)] > 0) {
                    adjacency_[matrixIndex(a, b)] = 1;
                    adjacency_[matrixIndex(b, a)] = 1;
                }
    }

    // ------------------------------------------------------------- density

    // Largest eigenvalue of a symmetric PSD 3x3 by power iteration.  Only the
    // dominant one is needed: complexity is 1 - lambdaMax/trace, and the trace
    // already gives the sum of all three.
    static float dominantEigenvalue(const float m[6]) {
        // Deliberately not (1,1,1): that is orthogonal to eigenvectors such as
        // (1,-1,0) and the iteration would start on a null component.
        float vx = 0.577F;
        float vy = 0.511F;
        float vz = 0.638F;
        float lambda = 0.0F;
        for (int iteration = 0; iteration < 24; ++iteration) {
            const float nx = m[0] * vx + m[3] * vy + m[4] * vz;
            const float ny = m[3] * vx + m[1] * vy + m[5] * vz;
            const float nz = m[4] * vx + m[5] * vy + m[2] * vz;
            const float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (norm < kTiny) return 0.0F;
            vx = nx / norm;
            vy = ny / norm;
            vz = nz / norm;
            lambda = norm;
        }
        return lambda;
    }

    void computeLocalGeometry() {
        for (int i = 0; i < count_; ++i) {
            float cov[6] = {0, 0, 0, 0, 0, 0};
            float distanceSum = 0.0F;
            int neighbors = 0;
            for (int j = 0; j < count_; ++j) {
                if (j == i || !isConnected(i, j)) continue;
                const float dx = nodes_[j].x - nodes_[i].x;
                const float dy = nodes_[j].y - nodes_[i].y;
                const float dz = nodes_[j].z - nodes_[i].z;
                const float length = std::sqrt(dx * dx + dy * dy + dz * dz);
                distanceSum += length;
                ++neighbors;
                if (length < kTiny) continue;
                const float ux = dx / length;
                const float uy = dy / length;
                const float uz = dz / length;
                cov[0] += ux * ux;
                cov[1] += uy * uy;
                cov[2] += uz * uz;
                cov[3] += ux * uy;
                cov[4] += ux * uz;
                cov[5] += uy * uz;
            }
            if (neighbors == 0) {
                spacing_[i] = 0.0F;
                complexity_[i] = 0.0F;
                continue;
            }
            spacing_[i] = distanceSum / static_cast<float>(neighbors);
            if (neighbors < 2) {
                complexity_[i] = 0.0F;
                continue;
            }
            const float trace = cov[0] + cov[1] + cov[2];
            if (trace < kTiny) {
                complexity_[i] = 0.0F;
                continue;
            }
            const float lambdaMax = dominantEigenvalue(cov);
            complexity_[i] = std::clamp(1.0F - lambdaMax / trace, 0.0F, 1.0F);
        }

        // A node stretched across a gap would otherwise claim a huge territory.
        quantileScratch_.clear();
        for (int i = 0; i < count_; ++i)
            if (spacing_[i] > 0.0F) quantileScratch_.push_back(spacing_[i]);
        const float fill =
            quantileScratch_.empty() ? 1.0F : linearQuantile(quantileScratch_, 0.5F);
        for (int i = 0; i < count_; ++i) {
            if (spacing_[i] <= 0.0F) spacing_[i] = fill;
            spacing_[i] = std::clamp(spacing_[i], 0.2F * fill, 5.0F * fill);
        }
    }

    void updateDensity() {
        if (count_ == 0) return;
        computeLocalGeometry();

        const float dimExponent = static_cast<float>(intrinsicDim_) * (1.0F - mu_);
        float sum = 0.0F;
        for (int i = 0; i < count_; ++i) {
            float value = std::pow(mass_[i] + kTiny, mu_) *
                          std::pow(spacing_[i], dimExponent);
            value *= 1.0F + kappa_ * complexity_[i];
            if (attention_) value *= std::max(0.0F, attention_(nodes_[i]));
            // Semantic attention: raise the budget for a class wherever it is,
            // rather than pointing at one place in the image.  Floored for the
            // same reason the fovea is -- a class weight of zero would merge
            // every other node away and leave nothing but the target.
            if (labelCount_ > 0) {
                const auto [label, confidence] = dominantLabel(i);
                if (label >= 0) {
                    const float weight = classWeights_[static_cast<size_t>(label)];
                    // An unconfident node is only partly subject to its class.
                    const float blended = 1.0F + confidence * (weight - 1.0F);
                    value *= classWeightFloor_ + (1.0F - classWeightFloor_) * blended;
                }
            }
            // A node that won nothing is dead weight at any mu, including mu=0
            // where the mass term drops out entirely.
            if (mass_[i] <= 0.0F) value = 0.0F;
            if (!std::isfinite(value)) value = 0.0F;
            rho_[i] = value;
            sum += value;
        }

        const float budget = static_cast<float>(capacity_);
        if (sum <= kTiny) {
            for (int i = 0; i < count_; ++i) {
                rho_[i] = 1.0F / static_cast<float>(count_);
                ratio_[i] = budget / static_cast<float>(count_);
            }
            return;
        }
        for (int i = 0; i < count_; ++i) {
            rho_[i] /= sum;
            // Measured against the budget, not the current count: while the
            // graph is small every node is overloaded and it grows; at the cap
            // the mean ratio is 1 and only real imbalance moves nodes around.
            ratio_[i] = rho_[i] * budget;
        }
    }

    void updatePressure() {
        for (int i = 0; i < count_; ++i) {
            if (ratio_[i] > splitRatio_) {
                pressure_[i] = std::max(pressure_[i], 0) + 1;
            } else if (ratio_[i] < mergeRatio_) {
                pressure_[i] = std::min(pressure_[i], 0) - 1;
            } else if (pressure_[i] > 0) {
                // An epoch back in balance walks the counter down, so a node has
                // to be persistently wrong rather than merely wrong often.
                --pressure_[i];
            } else if (pressure_[i] < 0) {
                ++pressure_[i];
            }
        }
    }

    // ------------------------------------------------------ density control

    // q swallows f: position, error, mass, semantic evidence and connections.
    // The caller marks f for removal and compacts afterwards.
    void absorbNode(int q, int f) {
        nodes_[q].x = 0.5F * (nodes_[q].x + nodes_[f].x);
        nodes_[q].y = 0.5F * (nodes_[q].y + nodes_[f].y);
        nodes_[q].z = 0.5F * (nodes_[q].z + nodes_[f].z);
        errors_[q] += errors_[f];
        mass_[q] += mass_[f];
        neighborMass_[q] += neighborMass_[f];
        // The absorbed node's evidence is real observation of the same region,
        // so it adds rather than averages.
        for (size_t k = 0; k < labelRowSize(); ++k) labelRow(q)[k] += labelRow(f)[k];
        pressure_[q] = 0;
        // q keeps its identity and inherits f's, so an outside reference to f
        // still resolves to something real.
        recordAlias(ids_[f], ids_[q]);

        // Inherit f's connections so the collapse cannot cut the graph.
        for (int other = 0; other < count_; ++other) {
            if (other == q || other == f) continue;
            if (isConnected(f, other) && !isConnected(q, other))
                setConnection(q, other, true, strength_[matrixIndex(f, other)]);
        }
        setConnection(q, f, false);
    }

    int mergeOverDenseNodes() {
        if (count_ <= 2) return 0;

        order_.resize(count_);
        for (int i = 0; i < count_; ++i) order_[i] = i;
        std::sort(order_.begin(), order_.end(),
                  [&](int a, int b) { return ratio_[a] < ratio_[b]; });

        std::fill(consumed_.begin(), consumed_.begin() + count_, uint8_t{0});
        std::fill(keep_.begin(), keep_.begin() + count_, uint8_t{1});
        const int budget = count_ - 2;  // never merge below two nodes
        int merges = 0;

        for (int index = 0; index < count_ && merges < budget; ++index) {
            const int q = order_[index];
            if (ratio_[q] >= mergeRatio_) break;
            if (consumed_[q] || pressure_[q] > -hysteresis_) continue;

            int f = -1;
            for (int candidate = 0; candidate < count_; ++candidate) {
                if (candidate == q || consumed_[candidate]) continue;
                if (!isConnected(q, candidate)) continue;
                if (f < 0 || ratio_[candidate] < ratio_[f]) f = candidate;
            }
            if (f < 0) continue;

            absorbNode(q, f);
            consumed_[q] = consumed_[f] = 1;
            keep_[f] = 0;
            ++merges;
        }

        if (merges > 0) compactNodes();
        return merges;
    }

    // At capacity, an absolute merge threshold deadlocks redistribution: nodes
    // that deserve more cannot grow because nobody is poor enough to release a
    // slot.  With semantic attention that is the common case -- the untargeted
    // classes settle comfortably above mergeRatio and the target starves.
    //
    // So when demand exists and the graph is full, merge the weakest pairs
    // regardless of the absolute threshold.  The 2x gap requirement keeps this
    // from thrashing between nodes of similar standing.
    int reallocateForDemand() {
        // Only an outside signal -- a fovea or a weighted class -- can create
        // demand the ordinary dynamics cannot satisfy.  Left to itself the
        // graph settles on its own, and forcing merges there only adds churn
        // for nothing.
        if (!externalDemand_) return 0;
        if (count_ < capacity_ || count_ <= 2) return 0;

        // Chronically unserved, not merely wanting: a node that gets its split
        // has its pressure reset, so only one that has asked and been refused
        // for this long accumulates here.
        const int starved = hysteresis_ * reallocationPatience_;
        float strongest = 0.0F;
        int demand = 0;
        for (int i = 0; i < count_; ++i) {
            if (pressure_[i] >= starved && ratio_[i] > splitRatio_) {
                ++demand;
                strongest = std::max(strongest, ratio_[i]);
            }
        }
        if (demand == 0) return 0;

        order_.resize(count_);
        for (int i = 0; i < count_; ++i) order_[i] = i;
        std::sort(order_.begin(), order_.end(),
                  [&](int a, int b) { return ratio_[a] < ratio_[b]; });

        std::fill(consumed_.begin(), consumed_.begin() + count_, uint8_t{0});
        std::fill(keep_.begin(), keep_.begin() + count_, uint8_t{1});
        const int budget = std::min(demand, count_ - 2);
        int merges = 0;

        for (int index = 0; index < count_ && merges < budget; ++index) {
            const int q = order_[index];
            if (ratio_[q] * 2.0F >= strongest) break;  // no real gradient left
            if (consumed_[q]) continue;

            int f = -1;
            for (int candidate = 0; candidate < count_; ++candidate) {
                if (candidate == q || consumed_[candidate]) continue;
                if (!isConnected(q, candidate)) continue;
                if (ratio_[candidate] * 2.0F >= strongest) continue;
                if (f < 0 || ratio_[candidate] < ratio_[f]) f = candidate;
            }
            if (f < 0) continue;

            absorbNode(q, f);
            consumed_[q] = consumed_[f] = 1;
            keep_[f] = 0;
            ++merges;
        }

        if (merges > 0) compactNodes();
        return merges;
    }

    int splitUnderServedNodes() {
        if (count_ >= capacity_) return 0;

        order_.resize(count_);
        for (int i = 0; i < count_; ++i) order_[i] = i;
        std::sort(order_.begin(), order_.end(),
                  [&](int a, int b) { return ratio_[a] > ratio_[b]; });

        std::fill(consumed_.begin(), consumed_.begin() + count_, uint8_t{0});
        const int originalCount = count_;
        int splits = 0;

        for (int index = 0; index < originalCount && count_ < capacity_; ++index) {
            const int q = order_[index];
            if (ratio_[q] <= splitRatio_) break;
            if (consumed_[q] || pressure_[q] < hysteresis_) continue;

            int f = -1;
            for (int candidate = 0; candidate < originalCount; ++candidate) {
                if (candidate == q || consumed_[candidate]) continue;
                if (!isConnected(q, candidate)) continue;
                if (f < 0 || ratio_[candidate] > ratio_[f]) f = candidate;
            }
            if (f < 0) continue;

            const int r = count_++;
            nodes_[r] = {0.5F * (nodes_[q].x + nodes_[f].x),
                         0.5F * (nodes_[q].y + nodes_[f].y),
                         0.5F * (nodes_[q].z + nodes_[f].z)};
            errors_[q] *= insertionDecay_;
            errors_[f] *= insertionDecay_;
            errors_[r] = 0.5F * (errors_[q] + errors_[f]);
            mass_[r] = 0.5F * (mass_[q] + mass_[f]);
            neighborMass_[r] = 0.5F * (neighborMass_[q] + neighborMass_[f]);
            // The new node sits between its parents, so it inherits their
            // average belief.  On a class boundary that lands genuinely
            // ambiguous, which is the correct answer for a node on a boundary.
            for (size_t k = 0; k < labelRowSize(); ++k)
                labelRow(r)[k] = 0.5F * (labelRow(q)[k] + labelRow(f)[k]);
            pressure_[q] = pressure_[f] = pressure_[r] = 0;
            ids_[r] = mintId();
            ratio_[r] = 0.5F * (ratio_[q] + ratio_[f]);
            spacing_[r] = 0.5F * (spacing_[q] + spacing_[f]);
            complexity_[r] = 0.5F * (complexity_[q] + complexity_[f]);

            for (int i = 0; i < count_; ++i) {
                adjacency_[matrixIndex(r, i)] = adjacency_[matrixIndex(i, r)] = 0;
                strength_[matrixIndex(r, i)] = strength_[matrixIndex(i, r)] = 0;
            }
            setConnection(q, f, false);
            setConnection(q, r, true, 1);
            setConnection(f, r, true, 1);

            consumed_[q] = consumed_[f] = 1;
            ++splits;
        }
        return splits;
    }

    void removeIsolatedNodes() {
        if (count_ <= 2) return;
        std::fill(keep_.begin(), keep_.begin() + count_, uint8_t{0});
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
        if (kept < 2 || kept == count_) return;
        // Isolated nodes have no neighbour to inherit them, so they retire
        // rather than alias.  This is what makes NodeState::Gone reachable.
        for (int i = 0; i < count_; ++i)
            if (!keep_[i]) retired_.insert(ids_[i]);
        compactNodes();
    }

    void compactNodes() {
        std::fill(mapping_.begin(), mapping_.end(), -1);
        int newCount = 0;
        for (int old = 0; old < count_; ++old) {
            if (!keep_[old]) continue;
            mapping_[old] = newCount;
            nodeScratch_[newCount] = nodes_[old];
            errorScratch_[newCount] = errors_[old];
            massScratch_[newCount] = mass_[old];
            neighborMassScratch_[newCount] = neighborMass_[old];
            idScratch_[newCount] = ids_[old];
            pressureScratch_[newCount] = pressure_[old];
            for (size_t k = 0; k < labelRowSize(); ++k)
                labelScratch_[static_cast<size_t>(newCount) * labelRowSize() + k] =
                    labelEvidence_[static_cast<size_t>(old) * labelRowSize() + k];
            ++newCount;
        }
        if (newCount < 2 || newCount == count_) return;

        std::fill(adjacencyScratch_.begin(), adjacencyScratch_.end(), uint8_t{0});
        std::fill(strengthScratch_.begin(), strengthScratch_.end(), uint32_t{0});
        for (int a = 0; a < count_; ++a) {
            if (mapping_[a] < 0) continue;
            for (int b = a + 1; b < count_; ++b) {
                if (mapping_[b] < 0 || !isConnected(a, b)) continue;
                const int na = mapping_[a];
                const int nb = mapping_[b];
                adjacencyScratch_[matrixIndex(na, nb)] =
                    adjacencyScratch_[matrixIndex(nb, na)] = 1;
                const uint32_t s = strength_[matrixIndex(a, b)];
                strengthScratch_[matrixIndex(na, nb)] =
                    strengthScratch_[matrixIndex(nb, na)] = s;
            }
        }
        std::copy_n(nodeScratch_.begin(), newCount, nodes_.begin());
        std::copy_n(errorScratch_.begin(), newCount, errors_.begin());
        std::copy_n(massScratch_.begin(), newCount, mass_.begin());
        std::copy_n(neighborMassScratch_.begin(), newCount, neighborMass_.begin());
        std::copy_n(idScratch_.begin(), newCount, ids_.begin());
        std::copy_n(pressureScratch_.begin(), newCount, pressure_.begin());
        if (labelCount_ > 0)
            std::copy_n(labelScratch_.begin(), labelRowSize() * newCount,
                        labelEvidence_.begin());
        adjacency_.swap(adjacencyScratch_);
        strength_.swap(strengthScratch_);
        count_ = newCount;
    }

    static float linearQuantile(std::vector<float>& values, float probability) {
        if (values.empty()) return 0.0F;
        std::sort(values.begin(), values.end());
        const float position =
            (values.size() - 1) * std::clamp(probability, 0.0F, 1.0F);
        const size_t low = static_cast<size_t>(std::floor(position));
        const size_t high = static_cast<size_t>(std::ceil(position));
        const float fraction = position - static_cast<float>(low);
        return values[low] + (values[high] - values[low]) * fraction;
    }

    int capacity_;
    int initialRegions_;
    float alpha_;
    float beta_;
    float delta_;
    float insertionDecay_;
    float mu_;
    float kappa_;
    int intrinsicDim_;
    float splitRatio_;
    float mergeRatio_;
    int hysteresis_;
    int reallocationPatience_;
    int aliasHistory_;
    int labelCount_;
    float labelDecay_;
    float classWeightFloor_;

    int count_ = 0;
    uint64_t epochs_ = 0;
    int churn_ = 0;
    int lastSplit_ = 0;
    int lastMerged_ = 0;
    int64_t nextId_ = 0;

    std::vector<Point3f> nodes_, delta1_, delta2_, nodeScratch_;
    std::vector<float> errors_, errorScratch_;
    std::vector<float> mass_, neighborMass_, massScratch_, neighborMassScratch_;
    std::vector<float> spacing_, complexity_, rho_, ratio_;
    std::vector<int64_t> ids_, idScratch_;
    std::vector<int32_t> pressure_, pressureScratch_;
    std::vector<float> labelEvidence_, labelScratch_, classWeights_;
    std::vector<uint32_t> strength_, strengthScratch_;
    std::vector<uint8_t> adjacency_, adjacencyScratch_, keep_, consumed_;
    std::vector<int> mapping_, order_;
    std::vector<float> quantileScratch_;
    std::vector<Point3f> candidates_, candidateScratch_;
    std::vector<size_t> sortOrder_;
    std::unordered_map<int64_t, int64_t> alias_;
    std::deque<int64_t> aliasOrder_;
    std::unordered_set<int64_t> retired_, idsBefore_;
    std::function<float(const Point3f&)> attention_;
    bool weightedClasses_ = false;
    bool externalDemand_ = false;
    std::mt19937 rng_;
};

}  // namespace fgng
