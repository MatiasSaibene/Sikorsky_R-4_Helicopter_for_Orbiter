#include "R4.h"
#include <cmath>

VECTOR3 R4::GetHelp_RotatePitch(VECTOR3 point, double pitch){

    //function to rotate point vector around X-axis in vessel coordinates into relative coordinates
    //IN LEFT HAND COORDINATE SYSTEM!!!

    //populate rotation matrix

    double m11 = 1.0;
    double m12 = 0.0;
    double m13 = 0.0;

    double m21 = 0.0;
    double m22 = std::cos(pitch);
    double m23 = std::sin(pitch);

    double m31 = 0.0;
    double m32 = -std::sin(pitch);
    double m33 = std::cos(pitch);

    MATRIX3 rot_matrix = {m11, m12, m13,
                          m21, m22, m23,
                          m31, m32, m33};

    VECTOR3 rotated_point = mul(rot_matrix, point);


    return rotated_point;
}

VECTOR3 R4::GetHelp_RotateYaw(VECTOR3 point, double yaw){

    //function to rotate point vector around Y-axis in vessel coordinates into relative coordinates
    //IN LEFT HAND COORDINATE SYSTEM!!!

    //populate rotation matrix

    double m11 = std::cos(yaw);
    double m12 = 0.0;
    double m13 = -std::sin(yaw);

    double m21 = 0.0;
    double m22 = 1.0;
    double m23 = 0.0;

    double m31 = std::sin(yaw);
    double m32 = 0.0;
    double m33 = std::cos(yaw);

    MATRIX3 rot_matrix = {m11, m12, m13,
                          m21, m22, m23,
                          m31, m32, m33};

    VECTOR3 rotated_point = mul(rot_matrix, point);


    return rotated_point;

}

VECTOR3 R4::GetHelp_RotateBank(VECTOR3 point, double bank){

    //function to rotate point vector around Z?-axis in vessel coordinates into relative coordinates
    //IN LEFT HAND COORDINATE SYSTEM!!!

    //populate rotation matrix

    double m11 = std::cos(bank);
    double m12 = std::sin(bank);
    double m13 = 0.0;

    double m21 = -std::sin(bank);
    double m22 = std::cos(bank);
    double m23 = 0.0;

    double m31 = 0.0;
    double m32 = 0.0;
    double m33 = 1.0;

    MATRIX3 rot_matrix = {m11, m12, m13,
                          m21, m22, m23,
                          m31, m32, m33};

    VECTOR3 rotated_point = mul(rot_matrix, point);


    return rotated_point;

}

VECTOR3 R4::GetHelp_Rotate(VECTOR3 point, double pitch, double yaw, double bank){

    VECTOR3 rotated_point = GetHelp_RotateYaw(point, yaw);

    rotated_point = GetHelp_RotatePitch(rotated_point, pitch);

    rotated_point = GetHelp_RotateBank(rotated_point, bank);

    return rotated_point;
}