#include "R4.h"
#include <array>
#include <cmath>
#include <cstdlib>

void R4::SetFeature_SetContactPointsWheels(){

    double travel = 0.1;

    double stiffness_value = ((empty_mass+main_fuel_tank_max) * G / travel);

    double damping_value = sqrt((empty_mass+main_fuel_tank_max) * stiffness_value);

    std::array<VECTOR3, 13> wheels_pos = {
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

    TOUCHDOWNVTX td_points[13];

    for(int i = 0; i < 13; i++){

        td_points[i].pos = wheels_pos[i];
        td_points[i].stiffness = stiffness_value;
        td_points[i].damping = damping_value;
        td_points[i].mu = 1.0;

        if(i < 3){ //touchdown point is landing gear, allow it to roll forward.

            td_points[i].mu_lng = 0.0;

        } else {

            td_points[i].mu_lng = 1.0;

        }

    }
    SetTouchdownPoints(td_points, 13);

}

void R4::SetFeature_MakeContactPointsFloats(){

    unsigned int n_points = 10;
    /*std::array<VECTOR3, 4> pos = {
        VECTOR3{operator-(_V(left_pontoon_front.x, left_pontoon_front.y, pontoon_length / 2), cg)},

        VECTOR3{operator-(_V(left_pontoon_rear.x, left_pontoon_rear.y, -pontoon_length / 2), cg)},

        VECTOR3{operator-(_V(right_pontoon_rear.x, right_pontoon_rear.y, -pontoon_length / 2), cg)},

        VECTOR3{operator-(_V(right_pontoon_front.x, right_pontoon_front.y, pontoon_length / 2), cg)}
    };*/

    for(int i = 0; i <= 3; i++){

        pos[i+4] = _V(pos[3].x, pos[3].y, pos[3].z + i * 0.25 * pontoon_length);

        pos[i+7] = _V(pos[2].x, pos[2].y, pos[2].z + i * 0.25 * pontoon_length);

    }

    pos[11] = operator-(tail_rotor_axis, cg); //tail rotor
    pos[12] = operator-(main_rotor_axis, cg); //main rotor mast
    pos[13] = operator-(_V(-1.4048, 0.17521, 0.26377), cg); //left gear strut top
    pos[14] = operator-(_V(1.4048, 0.17521, 0.26377), cg); //right gear strut top
    pos[15] = operator-(_V(-0.54482, 0.80672, 1.5369), cg); //left top canopy
    pos[16] = operator-(_V(0.54482, 0.80672, 1.5369), cg); //right top canopy
    pos[17] = operator-(_V(-0.40454, 0.0, 2.3633), cg); //left nose
    pos[18] = operator-(_V(0.40454, 0.0, 2.3633), cg); //right nose
    pos[19] = operator-(_V(-0.46431, -0.49492, 2.093), cg); //left chin
    pos[20] = operator-(_V(0.46431, -0.49492, 2.093), cg);

    //Make touchdown points table for pontoons on dry land

    double stiffness_value_pontoon_land = ((empty_mass + main_fuel_tank_max) * G) / (0.01 * n_points);

    double damping_value_pontoon_land = std::sqrt((empty_mass + main_fuel_tank_max) * stiffness_value_pontoon_land / n_points);

    for(int i = 0; i < 4; i++){

        td_points_pontoon_land[i].pos = pos[i];

        td_points_pontoon_land[i].stiffness = stiffness_value_pontoon_land;

        td_points_pontoon_land[i].damping = damping_value_pontoon_land;

        td_points_pontoon_land[i].mu = mu_d;

        td_points_pontoon_land[i].mu_lng = mu_d;

    }

    //Make touchdown points table for pontoons on water

    TOUCHDOWNVTX td_points_pontoon_water[4];

    double dA = (2 * pontoon_length * pontoon_dia) / n_points;

    double stiffness_value_pontoon_water = rho_sea * G * dA;

    double damping_value_pontoon_water = 0.1 * sqrt((empty_mass + main_fuel_tank_max) * stiffness_value_pontoon_water / n_points);



    for(int i = 0; i <= n_points; i++){

        td_points_pontoon_water[i].pos = pos[i];

        td_points_pontoon_water[i].stiffness = stiffness_value_pontoon_water;

        td_points_pontoon_water[i].damping = damping_value_pontoon_water;

        td_points_pontoon_water[i].mu = 0;

        td_points_pontoon_water[i].mu_lng = 0;

    }

}

void R4::SetFeature_SetRollingWheels(){

    //Following is to mimic realistic rolling tires without side scrub
    //Ground contact is determined in main script

    VECTOR3 groundspeed = _V(0, 0, 0);
    
    GetGroundspeedVector(FRAME_LOCAL, groundspeed);

    double side_slip = groundspeed.x;

    double mass = empty_mass + GetPropellantMass(main_fuel_tank);

    double simdt = oapiGetSimStep();

    double side_force = mass * side_slip / simdt;

    AddForce(_V(-side_force, 0, 0), right_main_wheel_contact);

}

double R4::SetFeature_ApplyBrakeForce(){

    //Ground contact is determined in main script

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

void R4::SetFeature_SetLeftBrakeForce(){

    //Ground contact is determined in main script

    double brake_force = SetFeature_ApplyBrakeForce();

    AddForce(_V(0, 0, -brake_force), left_main_wheel_contact);
    

}

void R4::SetFeature_SetRightBrakeForce(){

    //Ground contact is determined in main script

    double brake_force = SetFeature_ApplyBrakeForce();

    AddForce(_V(0, 0, -brake_force), right_main_wheel_contact);
    
}

void R4::SetFeature_ParkingBrake(){

    //Ground contact is determined in main script

    if(brake_hold == true){

        SetFeature_SetLeftBrakeForce();

        SetFeature_SetRightBrakeForce();

        SetAngularVel(_V(0, 0, 0));

    }

}

void R4::SetFeature_CrashOrSplash(){

    double AGL = GetAltitude(ALTMODE_GROUND);

    double MSL = GetAltitude(ALTMODE_MEANRAD);

    double elevation = MSL - AGL;

    
    if(elevation >= 1){ //over ground

        water = false;
        SetTouchdownPoints(td_points_pontoon_land, 4);

    }

}