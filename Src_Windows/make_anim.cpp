#include "R4.h"

void R4::MakeAnim_MainRotor(){

    static unsigned int MainRotorGrp[2] = {33, 36};
    static MGROUP_ROTATE main_rotor(
        0,
        MainRotorGrp,
        2,
        main_rotor_axis,
        _V(0, 1, 0),
        (float)(-360*RAD)
    );

    anim_main_rotor = CreateAnimation(0.0);

    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_main_rotor, 0, 1, &main_rotor);

    static unsigned int MainBlade1PitchGrp[2] = {37, 39};

    auto main_blade_1_pitch = new MGROUP_ROTATE(
        0,
        MainBlade1PitchGrp,
        2,
        _V(0, 1.5862, 0),
        _V(0, 0, -1),
        (float)(-15*RAD)
    );


    static unsigned int MainBlade2PitchGrp[2] = {38, 40};

    auto main_blade_2_pitch = new MGROUP_ROTATE(
        0,
        MainBlade2PitchGrp,
        2,
        _V(0, 1.5862, 0),
        _V(0.8660, 0, 0.500),
        (float)(-15*RAD)
    );

    static unsigned int MainBlade3PitchGrp[2] = {34, 35};

    auto main_blade_3_pitch = new MGROUP_ROTATE(
        0,
        MainBlade3PitchGrp,
        2,
        main_rotor_axis,
        _V(-0.8660, 0, 0.5000),
        (float)(-15*RAD)
    );

    anim_main_blade_1 = CreateAnimation(0.0);
    anim_main_blade_2 = CreateAnimation(0.0);
    anim_main_blade_3 = CreateAnimation(0.0);

    AddAnimationComponent(anim_main_blade_1, 0, 1, main_blade_1_pitch, parent);
    AddAnimationComponent(anim_main_blade_2, 0, 1, main_blade_2_pitch, parent);
    AddAnimationComponent(anim_main_blade_3, 0, 1, main_blade_3_pitch, parent);

}

void R4::MakeAnim_TailRotor(){

    static unsigned int TailRotorGrp = 41;

    static MGROUP_ROTATE tail_rotor(
        0,
        &TailRotorGrp,
        1,
        tail_rotor_axis,
        _V(1, 0, 0),
        (float)(-360*RAD)
    );

    anim_tail_rotor = CreateAnimation(0.0);

    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_tail_rotor, 0, 1, &tail_rotor);

    static unsigned int TailBlade1PitchGrp[2] = {44, 46};

    auto tail_blade1_pitch = new MGROUP_ROTATE(
        0,
        TailBlade1PitchGrp,
        2,
        tail_rotor_axis,
        _V(0, 0, -1),
        (float)(-30*RAD)
    );

    static unsigned int TailBlade2PitchGrp[2] = {45, 47};

    auto tail_blade2_pitch = new MGROUP_ROTATE(
        0,
        TailBlade2PitchGrp,
        2,
        tail_rotor_axis,
        _V(0, 0.8660, 0.5),
        (float)(-30*RAD)
    );

    static unsigned int TailBlade3PitchGrp[2] = {42, 43};

    auto tail_blade3_pitch = new MGROUP_ROTATE(
        0,
        TailBlade3PitchGrp,
        2,
        tail_rotor_axis,
        _V(0, 0.8660, 0.5),
        (float)(-30*RAD)
    );

    anim_tail_blade_1 = CreateAnimation(0.0);
    anim_tail_blade_2 = CreateAnimation(0.0);
    anim_tail_blade_3 = CreateAnimation(0.0);

    AddAnimationComponent(anim_tail_blade_1, -1, 1, tail_blade1_pitch, parent);
    AddAnimationComponent(anim_tail_blade_2, -1, 1, tail_blade2_pitch, parent);
    AddAnimationComponent(anim_tail_blade_3, -1, 1, tail_blade3_pitch, parent);

}

void R4::MakeAnim_AirspeedIndicator(){

    static unsigned int AirspeedGrp = 21;
    anim_airspeed = CreateAnimation(0.0);

    static MGROUP_ROTATE airspeed(
        0,
        &AirspeedGrp,
        1,
        airspeed_needle_location,
        indicator_axis,
        (float)(320*RAD)
    );

    AddAnimationComponent(anim_airspeed, 0, 1, &airspeed);

}

void R4::MakeAnim_Altimeter(){

    static unsigned int altimeter_1kGRP = 19;

    anim_altimeter_1k = CreateAnimation(0.0);

    static MGROUP_ROTATE altimeter_1k(
        0,
        &altimeter_1kGRP,
        1,
        altimeter_needle_location,
        indicator_axis,
        (float)(360*RAD)
    );

    AddAnimationComponent(anim_altimeter_1k, 0, 1, &altimeter_1k);

    static unsigned int altimeter_10kGRP = 20;

    anim_altimeter_10k = CreateAnimation(0.0);

    static MGROUP_ROTATE altimeter_10k(
        0,
        &altimeter_10kGRP,
        1,
        altimeter_needle_location,
        indicator_axis,
        (float)(360*RAD)
    );

    AddAnimationComponent(anim_altimeter_10k, 0, 1, &altimeter_10k);

}

void R4::MakeAnim_Compass(){

    static unsigned int compassGrp = 10;

    static MGROUP_ROTATE compass(
        0,
        &compassGrp,
        1,
        compass_wheel_location,
        indicator_axis,
        (float)(-360*RAD)
    );

    anim_compass = CreateAnimation(0.0);
    AddAnimationComponent(anim_compass, 0, 1, &compass);
    
}

void R4::MakeAnim_VerticalSpeed(){

    static unsigned int vertical_speedGrp = 22;

    anim_vertical_speed = CreateAnimation(0.0);

    static MGROUP_ROTATE vertical_speed(
        0,
        &vertical_speedGrp,
        1,
        vertical_speed_needle_location,
        indicator_axis,
        (float)(344*RAD)
    );

    AddAnimationComponent(anim_vertical_speed, -1, 1, &vertical_speed);
}

void R4::MakeAnim_ArtificialHorizon(){

    static unsigned int horizon_circleGrp = 14;

    anim_horizon_circle = CreateAnimation(0.5);

    static MGROUP_ROTATE horizon_circle(
        0,
        &horizon_circleGrp,
        1,
        horizon_circle_location,
        indicator_axis,
        (float)(-360*RAD)
    );

    static unsigned int horizon_ball_pitchGrp = 13;

    anim_horizon_ball_pitch = CreateAnimation(0.5);

    auto horizon_ball_pitch = new MGROUP_TRANSLATE(
        0,
        &horizon_ball_pitchGrp,
        1,
        GetHelp_RotatePitch(_V(0, 0.034, 0), dash_angle)
    );

    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_horizon_circle, 0, 1, &horizon_circle);

    AddAnimationComponent(anim_horizon_ball_pitch, 0, 1, horizon_ball_pitch, parent);

}

void R4::MakeAnim_Tachometer(){

    static unsigned int anim_tachometerGrp = 23;

    anim_tachometer = CreateAnimation(0.0);

    static MGROUP_ROTATE tachometer(
        0,
        &anim_tachometerGrp,
        1,
        tachometer_needle_location,
        indicator_axis,
        (float)(270*RAD)
    );

    AddAnimationComponent(anim_tachometer, 0, 1, &tachometer);

}

void R4::MakeAnim_FuelIndicator(){
    
    static unsigned int fuel_indicatorGrp = 24;

    anim_fuel_indicator = CreateAnimation(0.0);

    static MGROUP_ROTATE fuel_indicator(
        0,
        &fuel_indicatorGrp,
        1,
        fuel_indicator_needle_location,
        indicator_axis,
        (float)(270*RAD)
    );

    AddAnimationComponent(anim_fuel_indicator, 0, 1, &fuel_indicator);

}

void R4::MakeAnim_CollectiveIndicator(){

    static unsigned int anim_collective_indicatorGrp = 6;

    anim_collective_indicator = CreateAnimation(0.0);

    static MGROUP_TRANSLATE collective_indicator(
        0,
        &anim_collective_indicatorGrp,
        1,
        GetHelp_RotatePitch(_V(0, -0.0405, 0), dash_angle)
    );

    AddAnimationComponent(anim_collective_indicator, 0, 1, &collective_indicator);

}

void R4::MakeAnim_MainWheels(){

    static unsigned int left_main_wheelGrp[2] = {3, 4};

    static MGROUP_ROTATE left_main_wheel(
        1,
        left_main_wheelGrp,
        2,
        left_main_wheel_axis,
        _V(1, 0, 0),
        (float)(360*RAD)
    );

    anim_left_main_wheel = CreateAnimation(0.0);

    AddAnimationComponent(anim_left_main_wheel, 0, 1, &left_main_wheel);

    static unsigned int right_main_wheelGrp[2] = {1, 2};

    static MGROUP_ROTATE right_main_wheel(
        1,
        right_main_wheelGrp,
        2,
        right_main_wheel_axis,
        _V(1, 0, 0),
        (float)(360*RAD)
    );

    anim_right_main_wheel = CreateAnimation(0.0);

    AddAnimationComponent(anim_right_main_wheel, 0, 1, &right_main_wheel);

}

void R4::MakeAnim_TailWheel(){

    static unsigned int tail_wheel_strutGrp = 5;

    static MGROUP_ROTATE tail_wheel_strut(
        1,
        &tail_wheel_strutGrp,
        1,
        tail_wheel_axis,
        _V(0, 1, 0),
        (float)(360*RAD)
    );

    anim_tail_wheel_strut = CreateAnimation(0.0);

    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_tail_wheel_strut, -1, 1, &tail_wheel_strut);

    static unsigned int tail_wheelGrp[2] = {6, 7};

    static MGROUP_ROTATE tail_wheel(
        1,
        tail_wheelGrp,
        2,
        tail_wheel_axis,
        _V(1, 0, 0),
        (float)(360*RAD)
    );

    anim_tail_wheel = CreateAnimation(0.0);

    AddAnimationComponent(anim_tail_wheel, 0, 1, &tail_wheel, parent);

}
