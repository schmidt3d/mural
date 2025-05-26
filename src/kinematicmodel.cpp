#include "kinematicmodel.h"


inline void getLeftTangetPoint(const double frameX, const double frameY, const double gamma, double& x_PL, double& y_PL){
    const double s_L = d_t / 2.0;   // Distance of left and right tangent point from pen center. [mm]    
    const double P_LX = s_L * cos(gamma); // [mm] distance from pen center in x
    const double P_LY = s_L * sin(gamma); // [mm] .. and y
    x_PL = frameX - P_LX;    // [mm] Left pulley tangent point in frame coordinate system.
    y_PL = frameY - P_LY;    // [mm]
}

inline void getRightTangetPoint(const double frameX, const double frameY, const double gamma, double& x_PR, double& y_PR){
    // Coordinates of right pulley tangent point:
    const double s_R = d_t / 2.0;
    const double P_RX = s_R * cos(gamma); // [mm]
    const double P_RY = s_R * sin(gamma); // [mm]
    x_PR = frameX + P_RX;    // [mm] Right pulley tangent point in frame coordinate system.
    y_PR = frameY + P_RY;    // [mm]
}

// Compute angles of the belts and the forces on them.
// Input: - Mural coordinates X and Y in frame coordinate system [mm].
//        - Mural inclination gamma [rad].
// Output: - belt angles phi_L, phi_R [rad], measured against the line connecting the pins.
void getBeltAngles(const double frameX, const double frameY, const double gamma, double& phi_L, double& phi_R){
    // // Coordinates of left pulley tangent point:
    // const double P_LX = s_L * cos(gamma); // [mm] distance from pen center in x
    // const double P_LY = s_L * sin(gamma); // [mm] .. and y
    // const double x_PL = frameX - P_LX;    // [mm] Left pulley tangent point in frame coordinate system.
    // const double y_PL = frameY - P_LY;    // [mm]
    double x_PL;
    double y_PL;
    getLeftTangetPoint(frameX, frameY, gamma, x_PL, y_PL);
    phi_L = atan2(y_PL, x_PL);     // Angle of left belt, measured from line connecting the pins. [rad]
    // Serial.printf("  getLeftTangetPoint: frameX(%s), frameY(%s), gamma(%s), x(%s), y(%s), phi_L(%s)\n", 
    //         String(frameX), String(frameY), String(gamma), String(x_PL), String(y_PL), String(phi_L));
    // Coordinates of right pulley tangent point:
    // const double P_RX = s_R * cos(gamma); // [mm]
    // const double P_RY = s_R * sin(gamma); // [mm]
    // const double x_PR = frameX + P_RX;    // [mm] Right pulley tangent point in frame coordinate system.
    // const double y_PR = frameY + P_RY;    // [mm]
    double x_PR;
    double y_PR;
    getRightTangetPoint(frameX, frameY, gamma, x_PR, y_PR);
    phi_R = atan2(y_PR, topDistance - x_PR);     // Angle of left belt, measured from line connecting the pins. [rad]
    // Serial.printf("  getRightTangetPoint: frameX(%s), frameY(%s), gamma(%s), x(%s), y(%s), phi_R(%s)\n", 
    //         String(frameX), String(frameY), String(gamma), String(topDistance - x_PR), String(y_PR), String(phi_R));
}

void getBeltForces(const double phi_L, const double phi_R, double& F_L, double&F_R){
    // Computing the Forces. 
    // Force vectors are parallel to their belts, so the direction is given by phi_R and phi_L.
    // We assume that the bot is in a stable state (no torque), which allows us for having
    // the force vectors of left (L) and right (R) pulley meet in a single point. 
    // In this stable state the pulley forces cancel out the gravity force in x and y.
    // Note this is an approximation which is refined due to iteratively updating the values (torque, angles, forces). 
    const double F_G = mass_bot * g_constant;               // [N] Gravity force is pulling bot down. No x component.
    F_R = F_G * cos(phi_L) / sin(phi_L + phi_R);    // [N] magnitude of the force vector
    F_L = F_G * cos(phi_R) / sin(phi_L + phi_R);    // [N]
    // double F_Ly = F_L * sin(phi_L);                         // [N] components in y and x
    // double F_Lx = F_L * sin(phi_L);                         // [N] ...
    // double F_Ry = F_R * sin(phi_R);                         // [N]
    // double F_Rx = F_R * sin(phi_R);                         // [N]
}