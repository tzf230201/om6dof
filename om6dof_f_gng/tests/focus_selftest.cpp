#include "bio_focus.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using bio_fgng::DepthFrame;
using bio_fgng::FocusConfig;
using bio_fgng::FocusControl;
using bio_fgng::FocusField;
using bio_fgng::FocusMode;
using bio_fgng::Point;

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(message + ": got " + std::to_string(actual) +
                                 ", expected " + std::to_string(expected));
}
template<class Function>
void rejected(Function function, const std::string& message) {
    bool threw = false;
    try { function(); } catch (const std::invalid_argument&) { threw = true; }
    require(threw, message);
}
DepthFrame plane(float depth = 2, double timestamp = 0) {
    DepthFrame frame;
    frame.intrinsics = {41, 21, 60, 60, 20, 10};
    frame.depth.assign(41 * 21, depth);
    frame.timestamp = timestamp;
    return frame;
}
Point atPixel(const DepthFrame& frame, int x, int y, float depth) {
    const auto& k = frame.intrinsics;
    return {depth * (x - k.cx) / k.fx, depth * (y - k.cy) / k.fy, depth};
}
FocusControl manual(float distance = 2) {
    FocusControl control;
    control.mode = FocusMode::Manual;
    control.focusDistance = distance;
    return control;
}

void angularAndDioptricField() {
    auto frame = plane();
    FocusField field;
    field.update(frame, manual());
    near(field.evaluate({0, 0, 2}), 1, 1e-6, "on-axis focused surface receives full weight");
    near(field.retinalWeightCamera({.25F, 0, 1}),
         field.retinalWeightCamera({1, 0, 4}), 1e-6,
         "retinal resolution depends on angle, not world distance");
    require(field.retinalWeightCamera({.1F, 0, 2}) >
            field.retinalWeightCamera({.5F, 0, 2}), "angular resolution decreases toward periphery");

    const auto nearSurface = field.evaluateDetailed({-.05F, 0, 1.0F / .7F});
    const auto farSurface = field.evaluateDetailed({.05F, 0, 1.0F / .3F});
    near(nearSurface.focus, farSurface.focus, 1e-6,
         "equal positive/negative diopter offsets have equal blur");
    FocusConfig narrow;
    narrow.pupilScale = 2;
    FocusField widePupil(narrow);
    widePupil.update(frame, manual());
    require(widePupil.evaluateDetailed({-.05F, 0, 1.0F / .7F}).focus < nearSurface.focus,
            "larger virtual pupil narrows depth of field");
}

void focusAndGazeAreIndependent() {
    auto frame = plane(3);
    // A near surface beside a far one; both remain in view near fixation.
    for (int y = 0; y < frame.intrinsics.height; ++y)
        for (int x = 0; x < 20; ++x) frame.depth[static_cast<size_t>(y) * 41 + x] = 1;
    const Point nearPoint = atPixel(frame, 19, 10, 1);
    const Point farPoint = atPixel(frame, 21, 10, 3);
    FocusField field;
    field.update(frame, manual(1));
    require(field.evaluate(nearPoint) > field.evaluate(farPoint), "near accommodation favours near surface");
    const float oldRetinal = field.retinalWeightCamera(farPoint);
    frame.timestamp = 3;
    field.update(frame, manual(3));
    require(field.evaluate(farPoint) > field.evaluate(nearPoint), "far accommodation favours visible far surface");
    near(field.status().targetDistance, 3, 1e-6, "near peripheral surface cannot steal manual focus");
    near(field.retinalWeightCamera(farPoint), oldRetinal, 1e-6,
         "changing focus leaves angular field unchanged");

    auto shifted = manual(3);
    shifted.gazeU = .75F;
    const Point shiftedRay = atPixel(frame, 30, 10, 3);
    const float oldWeight = field.retinalWeightCamera(shiftedRay);
    frame.timestamp = 4;
    field.update(frame, shifted);
    require(field.retinalWeightCamera(shiftedRay) > oldWeight,
            "gaze moves the angular-resolution maximum");
    near(field.status().targetDistance, 3, 1e-6, "gaze shift preserves independent manual distance");
}

void autoUsesRobustFixationPatch() {
    auto frame = plane(3);
    // At the centre, foreground has three columns of the local patch and
    // background two. The chosen depth must be a surface, never their mean.
    for (int y = 0; y < 21; ++y)
        for (int x = 0; x <= 20; ++x) frame.depth[static_cast<size_t>(y) * 41 + x] = 1;
    FocusControl control;
    FocusField field;
    field.update(frame, control);
    near(field.status().targetDistance, 1, 1e-6,
         "auto selects supported surface at gaze rather than averaging depth edge");

    frame = plane(3, 1);
    frame.depth[10 * 41 + 20] = .5F;
    // Also put a real near object away from the local fixation patch.
    for (int y = 0; y < 21; ++y)
        for (int x = 0; x < 10; ++x) frame.depth[static_cast<size_t>(y) * 41 + x] = .3F;
    field.update(frame, control);
    near(field.status().targetDistance, 3, 1e-6,
         "isolated central outlier and peripheral near object cannot steal autofocus");
}

void visibilityAndMissingObservations() {
    auto frame = plane(1);
    FocusField field;
    field.update(frame, manual(3));
    const float floor = field.config().peripheralFloor;
    near(field.evaluate({0, 0, 3}), floor, 1e-6, "far focus cannot see through an occluder");
    require(!field.evaluateDetailed({0, 0, 3}).visible, "occluded graph point is not supported");
    near(field.evaluate({0, 0, .2F}), floor, 1e-6, "point in observed free space receives no bonus");
    near(field.evaluate({100, 0, 1}), floor, 1e-6, "outside FoV receives floor");
    near(field.evaluate({0, 0, -1}), floor, 1e-6, "behind camera receives floor");
    near(field.evaluate({std::numeric_limits<float>::quiet_NaN(), 0, 1}), floor, 1e-6,
         "non-finite query is safely unsupported");

    frame.timestamp = 1;
    frame.depth.assign(frame.depth.size(), std::numeric_limits<float>::quiet_NaN());
    frame.depth[0] = std::numeric_limits<float>::infinity();
    frame.depth[1] = -1;
    field.update(frame, manual(3));
    near(field.evaluate({0, 0, 3}), floor, 1e-6, "missing current depth cannot reuse last frame visibility");
}

void noObjectComponentMask() {
    auto frame = plane(2);
    // Disconnected visible fragments separated by unobserved pixels.
    for (int y = 0; y < 21; ++y) frame.depth[static_cast<size_t>(y) * 41 + 20] = 0;
    FocusField field;
    field.update(frame, manual(2));
    require(field.evaluate(atPixel(frame, 19, 10, 2)) > .8F &&
            field.evaluate(atPixel(frame, 21, 10, 2)) > .8F,
            "two disconnected surfaces at the same focus can both receive detail");
}

void worldPoseAndLock() {
    auto frame = plane(2);
    // +90 degrees around camera/world y and a nonzero translation.
    frame.pose.m = {0, 0, 1, 4, 0, 1, 0, -2, -1, 0, 0, 1, 0, 0, 0, 1};
    const Point target = frame.pose.toWorld({0, 0, 2});
    FocusField field;
    FocusControl control;
    control.mode = FocusMode::WorldLock;
    control.worldTarget = target;
    field.update(frame, control);
    require(field.status().targetValid, "world target associates with transformed current depth");
    near(field.evaluate(target), 1, 1e-6, "field evaluates in camera coordinates under rigid pose");

    frame.timestamp = .1;
    frame.pose.m[7] += .1; // Camera shifts along y; target must move in image.
    field.update(frame, control);
    require(field.status().gaze.y < -.04F, "world fixation is reprojected as camera moves");
    require(field.status().targetValid, "moving camera retains visible target identity");

    frame.timestamp = .2;
    frame.depth.assign(frame.depth.size(), 1);
    const float retainedDistance = field.status().targetDistance;
    field.update(frame, control);
    require(!field.status().targetValid && field.status().state == "HOLD",
            "occluded world target enters brief hold");
    near(field.status().targetDistance, retainedDistance, 1e-6,
         "world lock never retargets to the foreground occluder");
    near(field.evaluate(target), field.config().peripheralFloor, 1e-6,
         "occluded locked target receives no current-observation bonus");
    near(field.evaluate(frame.pose.toWorld({0, 0, 1})), field.config().peripheralFloor, 1e-6,
         "foreground occluder receives no substitute fixation bonus during hold");
    frame.timestamp = .5;
    field.update(frame, control);
    require(field.status().state == "LOST", "unobserved target times out");
    frame.timestamp = .6;
    frame.depth.assign(frame.depth.size(), 2);
    field.update(frame, control);
    require(field.status().targetValid, "same world target can be reacquired after occlusion");

    // Turn the camera away: during the grace period the locked gaze must not
    // jump back to a central visible object in the new view.
    frame.timestamp = .7;
    frame.pose.m = {0, 0, -1, 4, 0, 1, 0, -2, 1, 0, 0, 1, 0, 0, 0, 1};
    field.update(frame, control);
    require(field.status().gaze.z < 0, "world fixation remains behind after turning away");
    near(field.evaluate(frame.pose.toWorld({0, 0, 2})), field.config().peripheralFloor, 1e-6,
         "turning away cannot create a new frontal fixation during target hold");
}

void temporalAccommodationAndTransactionalValidation() {
    FocusConfig config;
    config.accommodationTau = .5F;
    auto frame = plane(1, 10);
    FocusField field(config);
    field.update(frame, manual(1));
    frame.timestamp = 10.5;
    field.update(frame, manual(3));
    const double expected = 1.0 / 3 + (1 - 1.0 / 3) * std::exp(-1);
    near(field.status().activeDiopters, expected, 1e-6,
         "one time constant follows exponential accommodation in diopters");
    const auto saved = field.status();
    field.update(frame, manual(1));
    near(field.status().activeDiopters, saved.activeDiopters, 1e-6,
         "repeated timestamp does not advance accommodation");
    frame.timestamp = 10;
    rejected([&] { field.update(frame, manual()); }, "backward timestamp rejected");
    near(field.status().activeDiopters, saved.activeDiopters, 1e-6,
         "rejected frame leaves accommodation unchanged");
    frame.timestamp = 11;
    frame.depth.pop_back();
    rejected([&] { field.update(frame, manual()); }, "malformed depth dimensions rejected");
    require(field.evaluateDetailed({0, 0, 1}).visible,
            "rejected frame preserves previously committed visibility snapshot");
    frame = plane(1, 11);
    frame.pose.m[0] = 2;
    rejected([&] { field.update(frame, manual()); }, "non-rigid pose rejected");
    frame = plane(1, 11);
    auto badControl = manual();
    badControl.gazeU = 1.1F;
    rejected([&] { field.update(frame, badControl); }, "gaze outside image rejected");
    frame.timestamp = std::numeric_limits<double>::quiet_NaN();
    rejected([&] { field.update(frame, manual()); }, "non-finite timestamp rejected");
    FocusConfig badConfig;
    badConfig.accommodationTau = 0;
    rejected([&] { FocusField invalid(badConfig); }, "zero accommodation constant rejected");

    frame = plane(1, 1000);
    field.update(frame, manual(1.0e30F));
    require(field.status().activeDiopters > 0 && std::isfinite(field.status().activeDistance),
            "fully settled large focus change avoids subtractive cancellation");
}

void lostAutoTargetAndSnapshotOwnership() {
    FocusField field;
    auto frame = plane(2);
    FocusControl control;
    field.update(frame, control);
    frame.depth.assign(frame.depth.size(), 0);
    require(field.evaluateDetailed({0, 0, 2}).visible,
            "field owns depth snapshot independently of caller's buffer");
    frame.timestamp = .1;
    frame.depth[10 * 41 + 24] = 2; // Visible, but outside the target's local patch.
    field.update(frame, control);
    require(!field.status().targetValid && field.status().state == "HOLD",
            "brief missing auto target retains demand with invalid observation flag");
    near(field.evaluate({0, 0, 2}), field.config().peripheralFloor, 1e-6,
         "hold does not provide unsupported visibility bonus");
    const Point remainingSurface = atPixel(frame, 24, 10, 2);
    require(field.evaluateDetailed(remainingSurface).visible,
            "other surfaces may remain visible while the auto target is missing");
    near(field.evaluate(remainingSurface), field.config().peripheralFloor, 1e-6,
         "missing auto target suppresses foveal bonus on other visible surfaces during hold");
    frame.timestamp = .5;
    field.update(frame, control);
    require(field.status().state == "LOST", "auto target timeout is finite");
}
} // namespace

int main() {
    try {
        angularAndDioptricField();
        focusAndGazeAreIndependent();
        autoUsesRobustFixationPatch();
        visibilityAndMissingObservations();
        noObjectComponentMask();
        worldPoseAndLock();
        temporalAccommodationAndTransactionalValidation();
        lostAutoTargetAndSnapshotOwnership();
        std::cout << "focus_selftest: all biological-field behaviour checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << "focus_selftest: " << error.what() << '\n';
        return 1;
    }
}
