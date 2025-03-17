#include "../include/OrbiterAPI.h"
#include "R4.h"
#include <cmath>
#include <cstdlib>

void R4::SetFeature_SetContactPointsWheels(){

    double travel = 0.1;

    double stiffness_value = ((empty_mass+main_fuel_tank_max) * G / travel);

    double damping_value = sqrt((empty_mass+main_fuel_tank_max) * stiffness_value);

    VECTOR3 wheels_pos1 = {operator-(tail_wheel_contact, cg)}; //tail wheel

    VECTOR3 wheels_pos2 = {operator-(right_main_wheel_contact, cg)}; //right wheel

    VECTOR3 wheels_pos3 = {operator-(left_main_wheel_contact, cg)}; //left wheel

    VECTOR3 wheels_pos4 = {operator-(tail_rotor_axis, cg)}; //tail rotor

    VECTOR3 wheels_pos5 = {operator-(main_rotor_axis, cg)}; //main rotor mast

    VECTOR3 wheels_pos6 = {operator-(_V(1.4048, 0.17521, 0.26377), cg)}; //left gear strut top

    VECTOR3 wheels_pos7 = {operator-(_V(1.4048, 0.17521, 0.26377), cg)}; //right gear strut top

    VECTOR3 wheels_pos8 = {operator-(_V(-0.54482, 0.80672, 1.5369), cg)}; //left top canopy

    VECTOR3 wheels_pos9 = {operator-(_V(0.54482, 0.80672, 1.5369), cg)}; //right top canopy

    VECTOR3 wheels_pos10 = {operator-(_V(-0.40454, 0.0, 2.3633), cg)}; //left nose

    VECTOR3 wheels_pos11 = {operator-(_V(0.40454, 0.0, 2.3633), cg)}; //right nose

    VECTOR3 wheels_pos12 = {operator-(_V(-0.46431, -0.49492, 2.093), cg)}; //left chin

    VECTOR3 wheels_pos13 = {operator-(_V(0.46431, -0.49492, 2.093), cg)}; //right chin
    

    td_points[0] = {wheels_pos1, stiffness_value, damping_value, 1.0, 0.0};
    td_points[1] = {wheels_pos2, stiffness_value, damping_value, 1.0, 0.0};
    td_points[2] = {wheels_pos3, stiffness_value, damping_value, 1.0, 0.0};
    td_points[3] = {wheels_pos4, stiffness_value, damping_value, 1.0, 1.0};
    td_points[4] = {wheels_pos5, stiffness_value, damping_value, 1.0, 1.0};
    td_points[5] = {wheels_pos6, stiffness_value, damping_value, 1.0, 1.0};
    td_points[6] = {wheels_pos7, stiffness_value, damping_value, 1.0, 1.0};
    td_points[7] = {wheels_pos8, stiffness_value, damping_value, 1.0, 1.0};
    td_points[8] = {wheels_pos9, stiffness_value, damping_value, 1.0, 1.0};
    td_points[9] = {wheels_pos10, stiffness_value, damping_value, 1.0, 1.0};
    td_points[10] = {wheels_pos11, stiffness_value, damping_value, 1.0, 1.0};
    td_points[11] = {wheels_pos12, stiffness_value, damping_value, 1.0, 1.0};
    td_points[12] = {wheels_pos13, stiffness_value, damping_value, 1.0, 1.0};

    SetTouchdownPoints(td_points, ntdvtx_td_points);

}

void R4::SetFeature_MakeContactPointsFloats(){

    unsigned int n_points = 10;

    VECTOR3 pontoon_pos1 = {operator-(_V(left_pontoon_front.x, left_pontoon_front.y, pontoon_length/2), cg)};

    VECTOR3 pontoon_pos2 = {operator-(_V(left_pontoon_rear.x, left_pontoon_rear.y, -pontoon_length/2), cg)};

    VECTOR3 pontoon_pos3 = {operator-(_V(right_pontoon_rear.x, right_pontoon_rear.y, -pontoon_length/2), cg)};

    VECTOR3 pontoon_pos4 = {operator-(_V(right_pontoon_front.x, right_pontoon_front.y, pontoon_length/2), cg)};

    VECTOR3 pontoon_pos5 = {pontoon_pos3.x, pontoon_pos3.y, pontoon_pos3.z + 1 * 0.25 * pontoon_length};

    VECTOR3 pontoon_pos6 = {pontoon_pos3.x, pontoon_pos3.y, pontoon_pos3.z + 2 * 0.25 * pontoon_length};

    VECTOR3 pontoon_pos7 = {pontoon_pos3.x, pontoon_pos3.y, pontoon_pos3.z + 3 * 0.25 * pontoon_length};

    VECTOR3 pontoon_pos8 = {pontoon_pos2.x, pontoon_pos2.y, pontoon_pos2.z + 1 * 0.25 * pontoon_length};

    VECTOR3 pontoon_pos9 = {pontoon_pos2.x, pontoon_pos2.y, pontoon_pos2.z + 2 * 0.25 * pontoon_length};

    VECTOR3 pontoon_pos10 = {pontoon_pos2.x, pontoon_pos2.y, pontoon_pos2.z + 3 * 0.25 * pontoon_length};

    VECTOR3 pontoon_pos11 = {operator-(tail_rotor_axis, cg)}; //tail rotor

    VECTOR3 pontoon_pos12 = {operator-(main_rotor_axis, cg)}; //main rotor mast

    VECTOR3 pontoon_pos13 = {operator-(_V(-1.4048, 0.17521, 0.26377), cg)}; //left gear strut top

    VECTOR3 pontoon_pos14 = {operator-(_V(1.4048, 0.17521, 0.26377), cg)}; //right gear strut top

    VECTOR3 pontoon_pos15 = {operator-(_V(-0.54482, 0.80672, 1.5369), cg)}; //left top canopy

    VECTOR3 pontoon_pos16 = {operator-(_V(0.54482, 0.80672, 1.5369), cg)}; //right top canopy

    VECTOR3 pontoon_pos17 = {operator-(_V(-0.40454, 0.0, 2.3633), cg)}; //left nose

    VECTOR3 pontoon_pos18 = {operator-(_V(0.40454, 0.0, 2.3633), cg)}; //right nose

    VECTOR3 pontoon_pos19 = {operator-(_V(-0.46431, -0.49492, 2.093), cg)}; //left chin

    VECTOR3 pontoon_pos20 = {operator-(_V(0.46431, -0.49492, 2.093), cg)};


    //Make touchdown points table for pontoons on dry land

    double stiffness_value_pontoon_land = ((empty_mass + main_fuel_tank_max) * G) / (0.01 * n_points);

    double damping_value_pontoon_land = std::sqrt((empty_mass + main_fuel_tank_max) * stiffness_value_pontoon_land / n_points);

    td_points_pontoon_land[0] = {pontoon_pos1, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[1] = {pontoon_pos2, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[2] = {pontoon_pos3, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[3] = {pontoon_pos4, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[4] = {pontoon_pos5, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[5] = {pontoon_pos6, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[6] = {pontoon_pos7, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[7] = {pontoon_pos8, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[8] = {pontoon_pos9, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[9] = {pontoon_pos10, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[10] = {pontoon_pos11, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[11] = {pontoon_pos12, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[12] = {pontoon_pos13, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[13] = {pontoon_pos14, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[14] = {pontoon_pos15, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[15] = {pontoon_pos16, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[16] = {pontoon_pos17, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[17] = {pontoon_pos18, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[18] = {pontoon_pos19, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};

    td_points_pontoon_land[19] = {pontoon_pos20, stiffness_value_pontoon_land, damping_value_pontoon_land, mu_d, mu_d};


    //Make touchdown points table for pontoons on water

    double dA = (2 * pontoon_length * pontoon_dia) / n_points;

    double stiffness_value_pontoon_water = rho_sea * G * dA;

    double damping_value_pontoon_water = 0.1 * sqrt((empty_mass + main_fuel_tank_max) * stiffness_value_pontoon_water / n_points);

    td_points_pontoon_water[0] = {pontoon_pos1, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

    td_points_pontoon_water[1] = {pontoon_pos2, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

    td_points_pontoon_water[2] = {pontoon_pos3, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

    td_points_pontoon_water[3] = {pontoon_pos4, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

    td_points_pontoon_water[4] = {pontoon_pos5, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

    td_points_pontoon_water[5] = {pontoon_pos6, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

    td_points_pontoon_water[6] = {pontoon_pos7, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

    td_points_pontoon_water[7] = {pontoon_pos8, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

    td_points_pontoon_water[8] = {pontoon_pos9, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

    td_points_pontoon_water[9] = {pontoon_pos10, stiffness_value_pontoon_water, damping_value_pontoon_water, 0.0, 0.0};

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

    
    if(elevation > 1){ //over ground

        water = false;
        SetTouchdownPoints(td_points_pontoon_land, ntdvtx_td_points_pontoon_land);

    } else if (elevation < 1){ //over sea

        water = true;
        SetTouchdownPoints(td_points_pontoon_water, ntdvtx_td_points_pontoon_water);

    }

}