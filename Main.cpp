#include "Header.h"

#include <iomanip>
#include <iostream>
#include <limits>

namespace {

void CopyJoint(const JOINT &src, JOINT &dst) {
    for (int i = 0; i < 4; ++i) {
        dst[i] = src[i];
    }
}

void ClearInput() {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void PrintJointInline(const JOINT &q) {
    std::cout << std::fixed << std::setprecision(2)
              << "[" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]";
}

bool ReadJoint(JOINT &q) {
    std::cout << "Enter q1 q2 d3 q4 (deg deg mm deg): ";
    if (!(std::cin >> q[0] >> q[1] >> q[2] >> q[3])) {
        ClearInput();
        return false;
    }
    return true;
}

bool ReadPose(const char *label, Pose &p) {
    std::cout << label << " x y z phi (mm mm mm deg): ";
    if (!(std::cin >> p.x >> p.y >> p.z >> p.phi)) {
        ClearInput();
        return false;
    }
    return true;
}

void PrintLimits() {
    std::cout << "Joint limits: "
              << "q1[" << kQ1Min << ", " << kQ1Max << "] "
              << "q2[" << kQ2Min << ", " << kQ2Max << "] "
              << "d3[" << kD3Min << ", " << kD3Max << "] "
              << "q4[" << kQ4Min << ", " << kQ4Max << "]" << std::endl;
}

void PrintMenu() {
    std::cout << "\n========== ENSC 488 Demo 1 ==========\n";
    std::cout << "1) Move to joint configuration (TA gives q)\n";
    std::cout << "2) WHERE (FK) for current q\n";
    std::cout << "3) SOLVE (IK) for pose (TA gives pose)\n";
    std::cout << "4) Pick and Place\n";
    std::cout << "5) Exit\n";
    std::cout << "Choose: ";
}

}  // namespace

int main() {
    std::cout << std::fixed << std::setprecision(2);

    JOINT qCurrent = {0.0, 0.0, -200.0, 0.0};
    if (GetConfiguration(qCurrent)) {
        std::cout << "Startup configuration from robot: q=";
        PrintJointInline(qCurrent);
        std::cout << std::endl;
    } else {
        std::cout << "GetConfiguration failed, using safe default q=";
        PrintJointInline(qCurrent);
        std::cout << std::endl;
    }

    bool running = true;
    while (running) {
        PrintMenu();

        int option = 0;
        if (!(std::cin >> option)) {
            ClearInput();
            std::cout << "Invalid menu input." << std::endl;
            continue;
        }

        switch (option) {
            case 1: {
                JOINT qIn = {0.0, 0.0, 0.0, 0.0};
                if (!ReadJoint(qIn)) {
                    std::cout << "Invalid joint input." << std::endl;
                    break;
                }
                if (!IsWithinLimits(qIn)) {
                    std::cout << "REJECT: joint limits." << std::endl;
                    PrintLimits();
                    break;
                }

                if (!MoveToConfiguration(qIn, true)) {
                    std::cout << "MoveToConfiguration failed." << std::endl;
                    break;
                }

                CopyJoint(qIn, qCurrent);
                std::cout << "Moved to q=";
                PrintJointInline(qCurrent);
                std::cout << std::endl;
                break;
            }

            case 2: {
                const Pose p = WHERE(qCurrent);
                std::cout << "WHERE q=";
                PrintJointInline(qCurrent);
                std::cout << " => (x=" << p.x << ", y=" << p.y << ", z=" << p.z << ", phi=" << p.phi
                          << ")" << std::endl;
                break;
            }

            case 3: {
                Pose target = {};
                if (!ReadPose("Enter target pose", target)) {
                    std::cout << "Invalid pose input." << std::endl;
                    break;
                }

                JOINT qBefore = {qCurrent[0], qCurrent[1], qCurrent[2], qCurrent[3]};
                JOINT qChosen = {0.0, 0.0, 0.0, 0.0};
                if (!SOLVE(target, qCurrent, qChosen)) {
                    std::cout << "No motion performed." << std::endl;
                    break;
                }

                if (!MoveToConfiguration(qChosen, true)) {
                    std::cout << "MoveToConfiguration failed for chosen IK solution." << std::endl;
                    CopyJoint(qBefore, qCurrent);
                    break;
                }

                CopyJoint(qChosen, qCurrent);
                std::cout << "Move complete: q=";
                PrintJointInline(qCurrent);
                std::cout << std::endl;
                break;
            }

            case 4: {
                Pose pickAbove = {};
                Pose placeAbove = {};
                double downMm = 0.0;

                if (!ReadPose("Enter pickAbove pose", pickAbove)) {
                    std::cout << "Invalid pick pose input." << std::endl;
                    break;
                }
                if (!ReadPose("Enter placeAbove pose", placeAbove)) {
                    std::cout << "Invalid place pose input." << std::endl;
                    break;
                }

                std::cout << "Enter down_mm (suggest 30 to 60): ";
                if (!(std::cin >> downMm)) {
                    ClearInput();
                    std::cout << "Invalid down_mm input." << std::endl;
                    break;
                }

                if (PickAndPlace(pickAbove, placeAbove, downMm, qCurrent)) {
                    std::cout << "Pick and place complete." << std::endl;
                } else {
                    std::cout << "Pick and place failed." << std::endl;
                }
                break;
            }

            case 5:
                running = false;
                break;

            default:
                std::cout << "Invalid menu option." << std::endl;
                break;
        }
    }

    return 0;
}
