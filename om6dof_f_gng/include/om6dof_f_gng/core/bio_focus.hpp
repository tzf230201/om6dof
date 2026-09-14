#pragma once

// A computational, primate-inspired attention field, not a calibrated eye model.
// Depth is optical-axis z in metres; pose is T_world_camera. Gaze, dioptric
// accommodation and depth-buffer visibility are independent mechanisms.
#include "world_fgng.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bio_fgng {

using Point = world_fgng::Point;

struct Intrinsics {
    int width = 0, height = 0;
    float fx = 0, fy = 0, cx = 0, cy = 0;

    void validate() const {
        if (width <= 0 || height <= 0 || !std::isfinite(fx) ||
            !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
            !std::isfinite(cx) || !std::isfinite(cy))
            throw std::invalid_argument("invalid image dimensions or camera intrinsics");
        if (static_cast<size_t>(width) >
            std::numeric_limits<size_t>::max() / static_cast<size_t>(height))
            throw std::invalid_argument("image dimensions overflow");
    }
};

struct DepthFrame {
    Intrinsics intrinsics;
    std::vector<float> depth;
    double timestamp = 0; // Seconds; finite and nondecreasing within one field.
    world_fgng::Pose pose;
};

enum class FocusMode { Manual, Auto, WorldLock };

struct FocusControl {
    FocusMode mode = FocusMode::Auto;
    float gazeU = .5F, gazeV = .5F; // Normalized pixel centres: [0, 1].
    float focusDistance = 2;       // Axial distance along gaze, in metres.
    Point worldTarget{};
};

struct FocusConfig {
    float e2Degrees = 8;
    float retinalExponent = 2;
    float peripheralFloor = .15F;
    float gain = .85F;
    float focusSigmaDiopters = .35F;
    float accommodationTau = .25F; // Model time constant, not a biological claim.
    float pupilScale = 1;
    float visibilityTolerance = .05F;
    float relativeDepthTolerance = .01F;
    float targetTimeout = .3F;
    int autoPatchRadius = 2;
    int autoMinSupport = 3;
    float targetDepthTolerance = .08F;
    float minDepth = .05F, maxDepth = 100;

    void validate() const {
        const auto positive = [](float x) { return std::isfinite(x) && x > 0; };
        const auto nonnegative = [](float x) { return std::isfinite(x) && x >= 0; };
        if (!positive(e2Degrees) || e2Degrees > 180 || !positive(retinalExponent) ||
            !nonnegative(peripheralFloor) || peripheralFloor > 1 ||
            !nonnegative(gain) || double(peripheralFloor) + gain > 1.000001 ||
            !positive(focusSigmaDiopters) || !positive(accommodationTau) ||
            !positive(pupilScale) || !nonnegative(visibilityTolerance) ||
            !nonnegative(relativeDepthTolerance) || !nonnegative(targetTimeout) ||
            !nonnegative(targetDepthTolerance) || !positive(minDepth) ||
            !positive(maxDepth) || maxDepth <= minDepth ||
            autoPatchRadius < 0 || autoPatchRadius > 32 || autoMinSupport < 1 ||
            autoMinSupport > (2 * autoPatchRadius + 1) * (2 * autoPatchRadius + 1))
            throw std::invalid_argument("invalid bio-inspired focus configuration");
    }
};

struct FocusStatus {
    Point gaze{0, 0, 1}; // Unit direction in the current camera frame.
    float targetDistance = 2;
    float activeDistance = 2;
    float activeDiopters = .5F;
    bool targetValid = false; // Current support, not merely a retained target.
    std::string state = "LOST";
};

struct Evaluation {
    float weight = 0, retinal = 0, focus = 0;
    bool visible = false;
};

class FocusField {
public:
    explicit FocusField(FocusConfig config = {}) : config_(std::move(config)) {
        config_.validate();
    }

    const FocusConfig& config() const noexcept { return config_; }
    const FocusStatus& status() const noexcept { return status_; }

    // A complete depth/pose snapshot is owned by the field. Invalid metadata,
    // controls or backward time throw before changing state. Invalid depth
    // samples (including NaN/Inf) are missing observations, not an exception.
    // Equal timestamps accept a new observation with zero accommodation step.
    // The first frame initializes accommodation at its selected demand because
    // no earlier physiological state is known. Subsequent steps use exp(-dt/tau).
    FocusStatus update(const DepthFrame& frame, const FocusControl& control) {
        validateFrame(frame, control);
        DepthFrame nextFrame = frame;
        FocusStatus next = status_;
        next.gaze = pixelRay(frame.intrinsics,
                            double(control.gazeU) * (frame.intrinsics.width - 1),
                            double(control.gazeV) * (frame.intrinsics.height - 1));
        next.targetValid = false;
        bool continuingTarget = hasTarget_ && initialized_ && control.mode == control_.mode;
        if (control.mode == FocusMode::WorldLock && initialized_) {
            continuingTarget = continuingTarget &&
                control.worldTarget.x == control_.worldTarget.x &&
                control.worldTarget.y == control_.worldTarget.y &&
                control.worldTarget.z == control_.worldTarget.z;
        }
        double nextLastSeen = lastTargetSeen_;

        if (control.mode == FocusMode::Manual) {
            next.targetDistance = control.focusDistance;
            next.targetValid = true; // A valid demand does not imply visibility.
        } else if (control.mode == FocusMode::Auto) {
            float selectedDistance = 0;
            if (selectAutoTarget(frame, next.gaze, control, selectedDistance)) {
                next.targetDistance = selectedDistance;
                next.targetValid = true;
            }
        } else {
            const Point target = frame.pose.toCamera(control.worldTarget);
            const double distance = length(target);
            if (world_fgng::finite(target) && distance > 0 &&
                distance <= std::numeric_limits<float>::max()) {
                next.gaze = normalized(target);
                if (isVisible(frame, target) &&
                    std::isfinite(1.0F / static_cast<float>(distance))) {
                    next.targetDistance = static_cast<float>(distance);
                    next.targetValid = true;
                }
            }
            // An occluder never replaces the explicitly selected world target.
        }

        if (next.targetValid) {
            nextLastSeen = frame.timestamp;
            continuingTarget = true;
        }
        const bool held = !next.targetValid && continuingTarget &&
            frame.timestamp - nextLastSeen <= config_.targetTimeout;
        const bool nextTracked = next.targetValid || held;
        if (!initialized_) {
            if (!next.targetValid) next.targetDistance = control.focusDistance;
            next.activeDiopters = 1 / next.targetDistance;
        } else if (nextTracked) {
            const double dt = frame.timestamp - frame_.timestamp;
            const double alpha = -std::expm1(-dt / config_.accommodationTau);
            next.activeDiopters = static_cast<float>(
                (1 - alpha) * next.activeDiopters + alpha / next.targetDistance);
        }
        next.activeDistance = static_cast<float>(std::min(
            1.0 / next.activeDiopters, double(std::numeric_limits<float>::max())));
        if (!nextTracked) next.state = "LOST";
        else if (held) next.state = "HOLD";
        else if (std::abs(double(next.activeDiopters) - 1.0 / next.targetDistance) > 1e-3)
            next.state = "ACCOMMODATING";
        else next.state = "FIXATE";

        // Everything that can validate or allocate completes before committing.
        frame_ = std::move(nextFrame);
        status_ = std::move(next);
        control_ = control;
        initialized_ = true;
        hasTarget_ = continuingTarget;
        lastTargetSeen_ = nextLastSeen;
        // HOLD preserves target identity and accommodation, but no foveal
        // bonus is allocated without present target support. Manual focus is
        // an explicit valid demand; its per-point visibility still applies.
        bonusEnabled_ = status_.targetValid;
        return status_;
    }

    float retinalWeightCamera(const Point& point) const {
        if (!world_fgng::finite(point)) return 0;
        const double distance = length(point);
        if (!(distance > 0) || point.z <= 0) return 0;
        const double cosine = std::clamp(dot(point, status_.gaze) / distance, -1.0, 1.0);
        const double eccentricity = std::acos(cosine);
        constexpr double radiansPerDegree = 0.017453292519943295;
        return static_cast<float>(std::pow(
            1 + eccentricity / (config_.e2Degrees * radiansPerDegree),
            -config_.retinalExponent));
    }

    Evaluation evaluateDetailed(const Point& world) const {
        Evaluation result{config_.peripheralFloor, 0, 0, false};
        if (!initialized_ || !world_fgng::finite(world)) return result;
        const Point camera = frame_.pose.toCamera(world);
        if (!world_fgng::finite(camera) || camera.z <= 0) return result;
        result.retinal = retinalWeightCamera(camera);
        const double axialDistance = dot(camera, status_.gaze);
        if (axialDistance > 0) {
            const double blur = config_.pupilScale *
                (1.0 / axialDistance - status_.activeDiopters) / config_.focusSigmaDiopters;
            result.focus = static_cast<float>(1.0 / (1.0 + blur * blur));
        }
        result.visible = isVisible(frame_, camera);
        if (bonusEnabled_ && result.visible)
            result.weight = std::min(1.0F, config_.peripheralFloor +
                config_.gain * result.retinal * result.focus);
        return result;
    }

    float evaluate(const Point& world) const { return evaluateDetailed(world).weight; }

private:
    static double dot(const Point& a, const Point& b) {
        return double(a.x) * b.x + double(a.y) * b.y + double(a.z) * b.z;
    }
    static double length(const Point& p) {
        return std::hypot(double(p.x), double(p.y), double(p.z));
    }
    static Point normalized(const Point& p) {
        const double r = length(p);
        return {static_cast<float>(p.x / r), static_cast<float>(p.y / r),
                static_cast<float>(p.z / r)};
    }
    static Point pixelRay(const Intrinsics& k, double u, double v) {
        const double x = (u - k.cx) / k.fx, y = (v - k.cy) / k.fy;
        const double r = std::hypot(x, y, 1.0);
        return {static_cast<float>(x / r), static_cast<float>(y / r),
                static_cast<float>(1 / r)};
    }
    bool validDepth(float depth) const {
        return std::isfinite(depth) && depth >= config_.minDepth && depth <= config_.maxDepth;
    }
    void validateFrame(const DepthFrame& frame, const FocusControl& control) const {
        frame.intrinsics.validate();
        frame.pose.validate();
        if (frame.depth.size() != static_cast<size_t>(frame.intrinsics.width) *
                                      static_cast<size_t>(frame.intrinsics.height))
            throw std::invalid_argument("depth size does not match image dimensions");
        if (!std::isfinite(frame.timestamp) ||
            (initialized_ && frame.timestamp < frame_.timestamp))
            throw std::invalid_argument("focus timestamps must be finite and nondecreasing");
        if (control.mode != FocusMode::Manual && control.mode != FocusMode::Auto &&
            control.mode != FocusMode::WorldLock)
            throw std::invalid_argument("invalid focus mode");
        if (!std::isfinite(control.gazeU) || !std::isfinite(control.gazeV) ||
            control.gazeU < 0 || control.gazeU > 1 || control.gazeV < 0 || control.gazeV > 1 ||
            !std::isfinite(control.focusDistance) || control.focusDistance <= 0 ||
            !std::isfinite(1.0F / control.focusDistance) ||
            !world_fgng::finite(control.worldTarget))
            throw std::invalid_argument("invalid gaze, focus distance or world target");
    }

    bool isVisible(const DepthFrame& frame, const Point& point) const {
        if (!world_fgng::finite(point) || point.z <= 0) return false;
        const auto& k = frame.intrinsics;
        const double u = double(k.fx) * point.x / point.z + k.cx;
        const double v = double(k.fy) * point.y / point.z + k.cy;
        // Nearest-pixel depth association deliberately avoids interpolation
        // across foreground/background boundaries. Both front and rear
        // mismatches are unsupported; absence is not evidence of deletion.
        if (!(u >= -.5 && u < double(k.width) - .5 &&
              v >= -.5 && v < double(k.height) - .5)) return false;
        const int x = static_cast<int>(std::floor(u + .5));
        const int y = static_cast<int>(std::floor(v + .5));
        const float observed = frame.depth[static_cast<size_t>(y) * k.width + x];
        if (!validDepth(observed)) return false;
        const double tolerance = config_.visibilityTolerance +
                                 config_.relativeDepthTolerance * double(observed);
        return std::abs(double(point.z) - observed) <= tolerance;
    }

    bool selectAutoTarget(const DepthFrame& frame, const Point& gaze,
                          const FocusControl& control, float& distance) const {
        const auto& k = frame.intrinsics;
        const double gazeX = double(control.gazeU) * (k.width - 1);
        const double gazeY = double(control.gazeV) * (k.height - 1);
        const int centreX = static_cast<int>(std::floor(gazeX + .5));
        const int centreY = static_cast<int>(std::floor(gazeY + .5));
        const int x0 = std::max(0, centreX - config_.autoPatchRadius);
        const int y0 = std::max(0, centreY - config_.autoPatchRadius);
        const int x1 = static_cast<int>(std::min<int64_t>(k.width - 1,
                                         int64_t(centreX) + config_.autoPatchRadius));
        const int y1 = static_cast<int>(std::min<int64_t>(k.height - 1,
                                         int64_t(centreY) + config_.autoPatchRadius));
        struct Sample { float z; double axial, pixelDistance; };
        std::vector<Sample> samples;
        for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) {
            const float z = frame.depth[static_cast<size_t>(y) * k.width + x];
            if (!validDepth(z)) continue;
            const double axial = ((double(x) - k.cx) * z / k.fx) * gaze.x +
                                 ((double(y) - k.cy) * z / k.fy) * gaze.y + z * gaze.z;
            if (!(axial > 0) || axial > std::numeric_limits<float>::max()) continue;
            samples.push_back({z, axial, (x - gazeX) * (x - gazeX) + (y - gazeY) * (y - gazeY)});
        }
        const size_t required = static_cast<size_t>(std::min(config_.autoMinSupport,
                                                          (x1 - x0 + 1) * (y1 - y0 + 1)));
        double bestPixelDistance = std::numeric_limits<double>::infinity();
        size_t bestSupport = 0;
        float bestAnchor = std::numeric_limits<float>::infinity();
        std::vector<double> selected;
        for (const auto& anchor : samples) {
            std::vector<double> cluster;
            double closest = std::numeric_limits<double>::infinity();
            for (const auto& sample : samples) {
                const double tolerance = config_.targetDepthTolerance +
                    config_.relativeDepthTolerance * double(std::min(anchor.z, sample.z));
                if (std::abs(double(anchor.z) - sample.z) <= tolerance) {
                    cluster.push_back(sample.axial);
                    closest = std::min(closest, sample.pixelDistance);
                }
            }
            if (cluster.size() < required) continue;
            if (closest < bestPixelDistance ||
                (closest == bestPixelDistance && cluster.size() > bestSupport) ||
                (closest == bestPixelDistance && cluster.size() == bestSupport && anchor.z < bestAnchor)) {
                bestPixelDistance = closest;
                bestSupport = cluster.size();
                bestAnchor = anchor.z;
                selected = std::move(cluster);
            }
        }
        if (selected.empty()) return false;
        // Pick an observed axial depth rather than averaging across an edge.
        const auto median = selected.begin() + static_cast<std::ptrdiff_t>(selected.size() / 2);
        std::nth_element(selected.begin(), median, selected.end());
        distance = static_cast<float>(*median);
        return std::isfinite(distance) && distance > 0 && std::isfinite(1.0F / distance);
    }

    FocusConfig config_;
    DepthFrame frame_;
    FocusControl control_;
    FocusStatus status_;
    bool initialized_ = false, hasTarget_ = false, bonusEnabled_ = false;
    double lastTargetSeen_ = 0;
};

} // namespace bio_fgng
