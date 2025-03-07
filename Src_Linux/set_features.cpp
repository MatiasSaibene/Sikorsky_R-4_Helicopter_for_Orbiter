#include "R4.h"
#include <array>
#include <cstdlib>

void R4::SetContactPoints(){

    double travel = 0.1;

    double stiffness_value = ((empty_mass+main_fuel_tank_max) * G / travel);

    double damping_value = sqrt((empty_mass+main_fuel_tank_max) * stiffness_value);

    std::array<VECTOR3, 13> pos = {
         VECTOR3{operator-(tail_wheel_contact, cg)}, //tail wheel

         VECTOR3{operator-(right_main_wheel_contact, cg)}, //right wheel

         VECTOR3{operator-(left_main_wheel_contact, cg)}, //left wheel

         VECTOR3{operator-(tail_rotor_axis, cg)}, //tail rotor

         VECTOR3{operator-(main_rotor_axis)}, //main rotor mast

         VECTOR3{operator-(_V(1.4048, 0.17521, 0.26377), cg)}, //left gear strut top

         VECTOR3{operator-(_V(1.4048, 0.17521, 0.26377), cg)}, //right gear strut top

         VECTOR3{operator-(_V(-0.54482, 0.80672, 1.5369), cg)}, //left top canopy

         VECTOR3{operator-(_V(0.54482, 0.80672, 1.5369), cg)}, //right top canopy

         VECTOR3{operator-(_V(-0.40454, 0.0, 2.3633), cg)}, //left nose

         VECTOR3{operator-(_V(0.40454, 0.0, 2.3633), cg)}, //right nose

         VECTOR3{operator-(_V(-0.46431, -0.49492, 2.093), cg)}, //left chin

         VECTOR3{operator-(_V(0.46431, -0.49492, 2.093), cg)} //right chin

    };

    TOUCHDOWNVTX td_points[sizeof(pos)];

    for(int i = 0; i < sizeof(pos); i++){

        td_points[i].pos = pos[i];
        td_points[i].stiffness = stiffness_value;
        td_points[i].damping = damping_value;
        td_points[i].mu = 1.0;

        if(i < 3){ //touchdown point is landing gear, allow it to roll forward.

            td_points[i].mu_lng = 0.0;

        } else {

            td_points[i].mu_lng = 1.0;

        }

    }
    SetTouchdownPoints(td_points, sizeof(pos));

}

void R4::SetRollingWheels(){

    //Following is to mimic realistic rolling tires without side scrub

    VECTOR3 groundspeed = _V(0, 0, 0);
    
    GetGroundspeedVector(FRAME_LOCAL, groundspeed);

    if(GroundContact()){

        double side_slip = groundspeed.x;

        double mass = empty_mass + GetPropellantMass(main_fuel_tank);

        double simdt = oapiGetSimStep();

        double side_force = mass * side_slip / simdt;

        AddForce(_V(-side_force, 0, 0), right_main_wheel_contact);
    }
}

double R4::ApplyBrakeForce(){

    VECTOR3 groundspeed = _V(0, 0, 0);
    double brake_force;
    
    GetGroundspeedVector(FRAME_LOCAL, groundspeed);

    double forward_speed = groundspeed.z;

    double mass = empty_mass + GetPropellantMass(main_fuel_tank);
    double simdt = oapiGetSimStep();

    double brake_acceleration = 0.25*G;

    if(forward_speed == 0){ //set brake force to counter thrust parallel to ground

        brake_force = 0.5 * main_rotor_thrust_vec.z;

    } else {

        if(abs(forward_speed/simdt) > brake_acceleration){

            //determine if velocity is positive or negative

            if(forward_speed > 0){

                brake_force = 0.5 * mass * brake_acceleration;

            } else {

                brake_force = -0.5 * mass * brake_acceleration;

            }
        } else {

            brake_force = 0.5 * mass * forward_speed / simdt;

        }
    }

    return brake_force;

}

void R4::SetLeftBrakeForce(){

    if(GroundContact()){

        double brake_force = ApplyBrakeForce();

        AddForce(_V(0, 0, -brake_force), left_main_wheel_contact);
    }

}

void R4::SetRightBrakeForce(){

    if(GroundContact()){

        double brake_force = ApplyBrakeForce();

        AddForce(_V(0, 0, -brake_force), right_main_wheel_contact);
    }
    
}

void R4::ParkingBrake(){

    if(brake_hold == true){

        SetLeftBrakeForce();

        SetRightBrakeForce();

    }

}