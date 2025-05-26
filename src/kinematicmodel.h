#ifndef Kinematicmodel_h
#define Kinematicmodel_h

#include <math.h>
// #include <Arduino.h>

#define PI 3.1415926535897932384626433832795

const double topDistance = 500.0;

// Geometry parameters:
// Effective diameter of the pulley+belts. Use EStep calibration to refine this value.
constexpr double diameter = 12.69;
const double circumference = diameter * PI;
// What is this? The inner distance between the pulleys? For me it's 
// more like 70.0 to 72.00mm (if measuring the point distance of belts touching pulleys.)
constexpr double bottomDistance = 67.4; 
constexpr double midPulleyToWall = 41.0;    // (Height) distance from mid of pulley to wall [mm].
constexpr float homedStepOffsetMM = 40.0;   // Length of fully retracted belt hitting stop screw.
                                            // Measured from outer edge of screw to the point
                                            // of tangency between belt and pulley. [mm]
// const int homedStepsOffset = int((homedStepOffsetMM / circumference) * stepsPerRotation);
constexpr double mass_bot = 0.55;   // Mass of the mural bot [kg].
constexpr double g_constant = 9.81; // Earth's gravitational acceleration constant [m/s^2]. Please change when running Mural on other planets!.
constexpr double d_t = 72.0;        // Distance of tangent points, wher belts touch the pulley. [mm]
constexpr double d_m = 16.0;        // Distance from line connecting tangent points to center of mass of bot (projected onto wall plane). [mm]
                                    // The center of mass sits roughly at the bottom of the pen opening. 

inline void getLeftTangetPoint(const double frameX, const double frameY, const double gamma, double& x_PL, double& y_PL);
inline void getRightTangetPoint(const double frameX, const double frameY, const double gamma, double& x_PR, double& y_PR);
void getBeltAngles(const double frameX, const double frameY, const double gamma, double& phi_L, double& phi_R);
void getBeltForces(const double phi_L, const double phi_R, double& F_L, double&F_R);
    
#endif