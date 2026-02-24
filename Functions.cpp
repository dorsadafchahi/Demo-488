#include "Header.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

namespace {

constexpr double kPi = 3.14159265358979323846;

void CopyJoint(const JOINT &src, JOINT &dst) {
    for (int i = 0; i < 4; ++i) {
        dst[i] = src[i];
    }
}

double DegToRad(double deg) {
    return deg * kPi / 180.0;
}

double RadToDeg(double rad) {
    return rad * 180.0 / kPi;
}

double WrapAngleDeg(double deg) {
    while (deg > 180.0) {
        deg -= 360.0;
    }
    while (deg <= -180.0) {
        deg += 360.0;
    }
    return deg;
}

double AngleDiffDeg(double target, double current) {
    return WrapAngleDeg(target - current);
}

double ClampValue(double value, double minV, double maxV) {
    if (value < minV) {
        return minV;
    }
    if (value > maxV) {
        return maxV;
    }
    return value;
}

double ClampD3(double d3) {
    return ClampValue(d3, kD3Min, kD3Max);
}

void PrintJointInline(const JOINT &q) {
    std::cout << std::fixed << std::setprecision(2)
              << "[" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]";
}

bool MoveToSolvedPose(const Pose &p, JOINT &qCurrent) {
    JOINT qBefore = {qCurrent[0], qCurrent[1], qCurrent[2], qCurrent[3]};
    JOINT qChosen = {0.0, 0.0, 0.0, 0.0};

    if (!SOLVE(p, qCurrent, qChosen)) {
        return false;
    }

    if (!MoveToConfiguration(qChosen, true)) {
        std::cout << "MoveToConfiguration failed for solved pose." << std::endl;
        CopyJoint(qBefore, qCurrent);
        return false;
    }

    CopyJoint(qChosen, qCurrent);
    return true;
}

bool MoveD3Stepped(JOINT &qCurrent, double targetD3) {
    targetD3 = ClampD3(targetD3);
    qCurrent[2] = ClampD3(qCurrent[2]);

    if (std::fabs(targetD3 - qCurrent[2]) <= kIKReachEps) {
        return true;
    }

    const double direction = (targetD3 > qCurrent[2]) ? 1.0 : -1.0;
    double currentD3 = qCurrent[2];

    while ((direction > 0.0 && currentD3 < targetD3 - kIKReachEps) ||
           (direction < 0.0 && currentD3 > targetD3 + kIKReachEps)) {
        double nextD3 = currentD3 + direction * kPickPlaceStepMm;
        if ((direction > 0.0 && nextD3 > targetD3) || (direction < 0.0 && nextD3 < targetD3)) {
            nextD3 = targetD3;
        }

        JOINT stepQ = {qCurrent[0], qCurrent[1], ClampD3(nextD3), qCurrent[3]};
        if (!MoveToConfiguration(stepQ, true)) {
            std::cout << "MoveToConfiguration failed during stepped d3 motion." << std::endl;
            return false;
        }

        CopyJoint(stepQ, qCurrent);
        currentD3 = qCurrent[2];
    }

    return true;
}

}  // namespace

bool IsWithinLimits(const JOINT &q) {
    return (q[0] >= kQ1Min - kIKReachEps && q[0] <= kQ1Max + kIKReachEps) &&
           (q[1] >= kQ2Min - kIKReachEps && q[1] <= kQ2Max + kIKReachEps) &&
           (q[2] >= kD3Min - kIKReachEps && q[2] <= kD3Max + kIKReachEps) &&
           (q[3] >= kQ4Min - kIKReachEps && q[3] <= kQ4Max + kIKReachEps);
}

Pose WHERE(const JOINT &q) {
    Pose p = {};

    const double q1Rad = DegToRad(q[0]);
    const double q12Rad = DegToRad(q[0] + q[1]);

    p.x = kL1 * std::cos(q1Rad) + kL2 * std::cos(q12Rad);
    p.y = kL1 * std::sin(q1Rad) + kL2 * std::sin(q12Rad);
    p.z = kZOffset - q[2];
    p.phi = WrapAngleDeg(q[0] + q[1] + q[3]);

    JOINT qDisplay = {q[0], q[1], q[2], q[3]};
    (void)DisplayConfiguration(qDisplay);

    return p;
}

IKAll INVKIN(const Pose &p) {
    IKAll all = {};
    all.count = 2;

    for (int i = 0; i < 2; ++i) {
        all.sols[i].q[0] = 0.0;
        all.sols[i].q[1] = 0.0;
        all.sols[i].q[2] = 0.0;
        all.sols[i].q[3] = 0.0;
        all.sols[i].reachable = false;
        all.sols[i].withinLimits = false;
    }

    const double r2 = p.x * p.x + p.y * p.y;
    const double cosQ2Raw = (r2 - kL1 * kL1 - kL2 * kL2) / (2.0 * kL1 * kL2);

    if (cosQ2Raw > 1.0 + kIKReachEps || cosQ2Raw < -1.0 - kIKReachEps) {
        return all;
    }

    const double cosQ2 = ClampValue(cosQ2Raw, -1.0, 1.0);
    const double q2CandidatesRad[2] = {std::acos(cosQ2), -std::acos(cosQ2)};

    for (int i = 0; i < 2; ++i) {
        const double q2Rad = q2CandidatesRad[i];
        const double q1Rad =
            std::atan2(p.y, p.x) -
            std::atan2(kL2 * std::sin(q2Rad), kL1 + kL2 * std::cos(q2Rad));

        JOINT q = {
            WrapAngleDeg(RadToDeg(q1Rad)),
            WrapAngleDeg(RadToDeg(q2Rad)),
            kZOffset - p.z,
            0.0,
        };
        q[3] = WrapAngleDeg(p.phi - (q[0] + q[1]));

        CopyJoint(q, all.sols[i].q);
        all.sols[i].reachable = true;
        all.sols[i].withinLimits = IsWithinLimits(all.sols[i].q);
    }

    return all;
}

int SelectClosest(const IKAll &all, const JOINT &qCurrent) {
    int bestIdx = -1;
    double bestDist = std::numeric_limits<double>::infinity();

    for (int i = 0; i < all.count; ++i) {
        const IKSol &sol = all.sols[i];
        if (!sol.reachable || !sol.withinLimits) {
            continue;
        }

        const double dist =
            std::fabs(AngleDiffDeg(sol.q[0], qCurrent[0])) +
            std::fabs(AngleDiffDeg(sol.q[1], qCurrent[1])) +
            kD3DistanceWeight * std::fabs(sol.q[2] - qCurrent[2]) +
            std::fabs(AngleDiffDeg(sol.q[3], qCurrent[3]));

        if (dist < bestDist) {
            bestDist = dist;
            bestIdx = i;
        }
    }

    return bestIdx;
}

bool SOLVE(const Pose &p, JOINT &qCurrent, JOINT &qChosen) {
    const IKAll all = INVKIN(p);

    for (int i = 0; i < all.count; ++i) {
        std::cout << "Sol " << (i + 1) << ": q=";
        PrintJointInline(all.sols[i].q);
        std::cout << " ";

        if (!all.sols[i].reachable) {
            std::cout << "REJECT: unreachable";
        } else if (!all.sols[i].withinLimits) {
            std::cout << "REJECT: joint limits";
        } else {
            std::cout << "OK";
        }
        std::cout << std::endl;
    }

    const int idx = SelectClosest(all, qCurrent);
    if (idx < 0) {
        std::cout << "No valid IK solution found." << std::endl;
        return false;
    }

    CopyJoint(all.sols[idx].q, qChosen);
    CopyJoint(qChosen, qCurrent);

    std::cout << "Chosen (closest): q=";
    PrintJointInline(qChosen);
    std::cout << std::endl;

    return true;
}

bool PickAndPlace(const Pose &pickAbove, const Pose &placeAbove, double down_mm, JOINT &qCurrent) {
    const double down = std::fabs(down_mm);

    std::cout << "Move to pick hover..." << std::endl;
    if (!MoveToSolvedPose(pickAbove, qCurrent)) {
        return false;
    }

    const double pickHoverD3 = qCurrent[2];
    const double pickDownD3 = ClampD3(pickHoverD3 + down);

    std::cout << "Lower to pick..." << std::endl;
    if (!MoveD3Stepped(qCurrent, pickDownD3)) {
        return false;
    }

    if (!Grasp(true)) {
        std::cout << "Grasp(true) failed." << std::endl;
        return false;
    }

    std::cout << "Raise after pick..." << std::endl;
    if (!MoveD3Stepped(qCurrent, pickHoverD3)) {
        return false;
    }

    std::cout << "Move to place hover..." << std::endl;
    if (!MoveToSolvedPose(placeAbove, qCurrent)) {
        return false;
    }

    const double placeHoverD3 = qCurrent[2];
    const double placeDownD3 = ClampD3(placeHoverD3 + down);

    std::cout << "Lower to place..." << std::endl;
    if (!MoveD3Stepped(qCurrent, placeDownD3)) {
        return false;
    }

    if (!Grasp(false)) {
        std::cout << "Grasp(false) failed." << std::endl;
        return false;
    }

    std::cout << "Raise after place..." << std::endl;
    if (!MoveD3Stepped(qCurrent, placeHoverD3)) {
        return false;
    }

    return true;
}
