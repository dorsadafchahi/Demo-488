#ifndef HEADER_H
#define HEADER_H

#ifndef _MSC_VER
#ifndef __declspec
#define __declspec(x)
#endif
#endif

typedef double JOINT[4];

__declspec(dllimport) bool MoveToConfiguration(JOINT &conf, bool wait = false);
__declspec(dllimport) bool DisplayConfiguration(JOINT &conf);
__declspec(dllimport) bool GetConfiguration(JOINT &conf);
__declspec(dllimport) bool Grasp(bool close);

struct Pose {
    double x;
    double y;
    double z;
    double phi;
};

struct IKSol {
    JOINT q;
    bool reachable;
    bool withinLimits;
};

struct IKAll {
    IKSol sols[2];
    int count;
};

constexpr double kL1 = 195.0;  // mm
constexpr double kL2 = 142.0;  // mm

// Single place for Z frame/tool offset tuning.
constexpr double kZOffset = 0.0;  // mm

constexpr double kQ1Min = -150.0;
constexpr double kQ1Max = 150.0;
constexpr double kQ2Min = -100.0;
constexpr double kQ2Max = 100.0;
constexpr double kD3Min = -200.0;
constexpr double kD3Max = -100.0;
constexpr double kQ4Min = -160.0;
constexpr double kQ4Max = 160.0;

constexpr double kIKReachEps = 1e-6;
constexpr double kPickPlaceStepMm = 5.0;
constexpr double kD3DistanceWeight = 0.1;

bool IsWithinLimits(const JOINT &q);
Pose WHERE(const JOINT &q);
IKAll INVKIN(const Pose &p);
int SelectClosest(const IKAll &all, const JOINT &qCurrent);
bool SOLVE(const Pose &p, JOINT &qCurrent, JOINT &qChosen);
bool PickAndPlace(const Pose &pickAbove, const Pose &placeAbove, double down_mm, JOINT &qCurrent);

#endif  // HEADER_H
