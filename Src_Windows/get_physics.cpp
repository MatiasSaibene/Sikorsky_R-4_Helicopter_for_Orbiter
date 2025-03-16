#include "R4.h"
#include <cstdlib>

void R4::GetPhysics_Drag(){

    VECTOR3 Va = _V(0, 0, 0);

    GetAirspeedVector(FRAME_LOCAL, Va);
    double rho = GetAtmDensity();

    VECTOR3 cd = _V(2, 2, 1.5);
    VECTOR3 A = _V(10, 5, 2);

    VECTOR3 drag = _V(0, 0, 0);

    drag.x = -cd.x * A.x * 0.5 * rho * Va.x * std::abs(Va.x);
    drag.y = -cd.y * A.y * 0.5 * rho * Va.y * std::abs(Va.y);
    drag.z = -cd.z * A.z * 0.5 * rho * Va.z * std::abs(Va.z);

    AddForce(drag, _V(0, 0, 0));
}