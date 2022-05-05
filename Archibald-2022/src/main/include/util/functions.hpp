#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP
#include <constants.h>

namespace util{
    double sigmoid(double base, double x){
        return (1/(1 + (1 / pow(base, x))) - 0.5) * 2; // Gets a sigmoid designed specifically for custom PID implementations
    }

    void setPIDPresets(rev::SparkMaxPIDController sparky){
        sparky.SetP (PID_kP);
        sparky.SetI (PID_kI);
        sparky.SetD (PID_kD);
        sparky.SetIZone (PID_kIz);
        sparky.SetFF (PID_kFF);
        sparky.SetOutputRange (PID_kMinOutput, PID_kMaxOutput);
    }

    double normalizeAngle(double angle){ // Basic algebra, finding a coterminal angle. Maybe it's trig?
        while (angle < 0){
            angle += 360;
        }
        while (angle >= 360){ // More than or Equal To because I want 0 to be == 360.
            angle -= 360;
        }
        return angle;
    }

    double minAngleDistance(double angle1, double angle2){ // Find min distance between any two angles. Expect it to nromalize.
        angle1 = normalizeAngle(angle1); // This is just to make life easier.
        angle2 = normalizeAngle(angle2);
        double dist1 = normalizeAngle(angle1 - angle2);
        double dist2 = normalizeAngle(angle2 - angle1);
        if (dist1 < dist2){
            return -dist1;
        }
        return dist2;
    }

    double maxAngleDistance(double angle1, double angle2){ // Find max distance between any two angles. Expect it to nromalize.
        angle1 = normalizeAngle(angle1); // This is just to make life easier.
        angle2 = normalizeAngle(angle2);
        double dist1 = normalizeAngle(angle1 - angle2);
        double dist2 = normalizeAngle(angle2 - angle1);
        if (dist1 < dist2){
            return -dist2;
        }
        return dist1;
    }
}
#endif
