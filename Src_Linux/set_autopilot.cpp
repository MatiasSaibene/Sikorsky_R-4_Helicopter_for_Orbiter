#include "R4.h"

void R4::SetAutopilot_Altitude(){

    VECTOR3 airspdvec;

    if(altitude_hold == true){

        //Proportional error

        double altitude = GetAltitude(ALTMODE_MEANRAD);

        double propoportional_error = altitude_target - altitude;

        //Derivative error

        GetAirspeedVector(FRAME_LOCAL, airspdvec);
        double derivative_error = airspdvec.y;

        //calculate control response using PD gains determined by Ziegler-Nichols tuning method.

        double Ku = 0.84; //Ultimate gain
        double Tu = 2.73; //Ultimate period (s)

        double Kp = 0.8 * Ku;
        double Kd = 0.125 * Tu;

        double collective_input = Kp * propoportional_error + Kd * derivative_error;

        SetThrusterGroupLevel(THGROUP_HOVER, collective_input);

    }
    
}