#include "R4.h"
#include <cmath>
#include <cstdlib>

void R4::SetAnim_MainRotor(){

    MainRotorSpec main_rotor_spec;

    double dt = oapiGetSimStep();

    main_rotor_anim_state = std::fmod((main_rotor_anim_state + (rpm * (dt/60))), 1);

    SetAnimation(anim_main_rotor, main_rotor_anim_state);

    
    SetAnimation(anim_main_blade_1, main_rotor_spec.prop_eff);
    SetAnimation(anim_main_blade_2, main_rotor_spec.prop_eff);
    SetAnimation(anim_main_blade_3, main_rotor_spec.prop_eff);

}

void R4::SetAnim_TailRotor(){

    double dt = oapiGetSimStep();

    tail_rotor_anim_state = std::fmod((tail_rotor_anim_state + rpm * tail_rotor_ratio * (dt/ 60)), 1);
    
    SetAnimation(anim_tail_rotor, tail_rotor_anim_state);

    SetAnimation(anim_tail_blade_1, tail_rotor_dir);
    SetAnimation(anim_tail_blade_2, tail_rotor_dir);
    SetAnimation(anim_tail_blade_3, tail_rotor_dir);

}

void R4::SetAnim_AirspeedIndicator(){

    //Indicated airspeed will differ from true airspeed with altitude because the dynamic pressure that is measured
    //by the Pitot-static system decreases with atmospheric density. Older altimeters would be calibrated for mean sea level.
    //Modern altimeters can be manually adjusted using local atmospheric pressure before take-off.

    VECTOR3 velocity = _V(0, 0, 0);

    GetAirspeedVector(FRAME_LOCAL, velocity);

    double density = GetAtmDensity();

    double density_msl = ATMD; //density in kg/m3 at standard sea level

    double dynamic_pressure = 0.5 * density * velocity.z * velocity.z;

    //indicated airspeed in z-direction, converted to knots:

    double airspeed = std::sqrt(2 * dynamic_pressure / density_msl) * (velocity.z / std::abs(velocity.z)) * 1.94384;

    if(airspeed >= 0){

        airspeed_anim_state = airspeed / 120;

    } else {

        airspeed_anim_state = 0.0;

    }

    SetAnimation(anim_airspeed, airspeed_anim_state);

}

void R4::SetAnim_Altimeter(){

    double altitude = GetAltitude() / 0.3048;

    altimeter_10k_anim_state = std::fmod(altitude / 10000, 1);

    SetAnimation(anim_altimeter_10k, altimeter_10k_anim_state);

    altimeter_1k_anim_state = std::fmod(altitude / 1000, 1);

    SetAnimation(anim_altimeter_1k, altimeter_1k_anim_state);

}

void R4::SetAnim_Compass(){

    compass_anim_state = (GetYaw() * DEG) / 360;

    SetAnimation(anim_compass, compass_anim_state);

}

void R4::SetAnim_VerticalSpeed(){

    VECTOR3 airspd = _V(0, 0, 0);

    GetAirspeedVector(FRAME_LOCAL, airspd);

    double vertical_speed = airspd.y * 196.85;

    vertical_speed_anim_state = vertical_speed / 2000;

    SetAnimation(anim_vertical_speed, vertical_speed_anim_state);

}

void R4::SetAnim_ArtificialHorizon(){

    double pitch = GetPitch();
    
    double roll = GetBank();


    double pitch_anim_state = 0.5 * (1 - (pitch / (20 * RAD)));

    double roll_anim_state = 0.5 * (1 - (roll * PI));

    SetAnimation(anim_horizon_circle, roll_anim_state);

    SetAnimation(anim_horizon_ball_pitch, pitch_anim_state);

}

void R4::SetAnim_Tachometer(){

    SetAnimation(anim_tachometer, rpm / 3000);

}

void R4::SetAnim_FuelIndicator(){

    SetAnimation(anim_fuel_indicator, GetPropellantMass(main_fuel_tank)/main_fuel_tank_max);

}

void R4::SetAnim_MainWheels(){

    VECTOR3 airspd = _V(0, 0, 0);
    VECTOR3 angvel = _V(0, 0, 0);

    GetGroundspeedVector(FRAME_LOCAL, airspd);
    GetAngularVel(angvel);

    double dt = oapiGetSimStep();

    double Vz = airspd.z;

    double omega = angvel.y;

    double V_left = Vz + left_main_wheel_axis.x * omega;
    double V_right = Vz + right_main_wheel_axis.x * omega;

    left_wheel_anim_state = std::fmod(left_wheel_anim_state + (V_left * dt) / (PI * main_wheel_diameter), 1);
    SetAnimation(anim_left_main_wheel, left_wheel_anim_state);

    right_wheel_anim_state = std::fmod(right_wheel_anim_state + (V_right * dt) / (PI * main_wheel_diameter), 1);

    SetAnimation(anim_right_main_wheel, right_wheel_anim_state);


}

void R4::SetAnim_TailWheel(){

    VECTOR3 angvel = _V(0, 0, 0);
    VECTOR3 grndspd = _V(0, 0, 0);
    GetAngularVel(angvel);
    GetGroundspeedVector(FRAME_LOCAL, angvel);
    double tail_wheel_angle;

    double dt = oapiGetSimStep();
    double omega = angvel.y;

    //define vector of tail wheel velocity in local coordinates

    VECTOR3 V = _V(tail_wheel_axis.z * omega, 0, grndspd.z);


    //define forward vector where animation state is 0

    VECTOR3 Vz = _V(0, 0, 1);

    if(V.x < 0){

        tail_wheel_angle = std::acos(dotp(V, Vz) / length(V) * length(Vz));

    } else if(V.x > 0){

        tail_wheel_angle = -std::acos(dotp(V, Vz) / length(V) * length(Vz));

    }

    tail_wheel_strut_anim_state = tail_wheel_angle / PI;
    SetAnimation(anim_tail_wheel_strut, tail_wheel_strut_anim_state);

    tail_wheel_anim_state = std::fmod((tail_wheel_anim_state + length(V) * dt) / (PI * tail_wheel_diameter), 1);
    SetAnimation(anim_tail_wheel, tail_wheel_anim_state);
}