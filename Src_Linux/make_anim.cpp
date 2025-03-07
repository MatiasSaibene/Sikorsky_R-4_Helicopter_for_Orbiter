#include "R4.h"
#include <cmath>

VECTOR3 indicator_axis = _V(0, 0.255766, 0.966739);

VECTOR3 vecA = _V(0, 0, -1);
VECTOR3 vecB = indicator_axis;

double dash_angle = std::acos(dotp(vecA, vecB) / length(vecA) * length(vecB));

VECTOR3 airspeed_needle_location = _V(-0.2410, 0.0818, 2.0201);

VECTOR3 altimeter_needle_location = _V(-0.1205, 0.0818, 2.0201);

VECTOR3 compass_wheel_location = _V(0.2410, 0.0818, 2.0201);

VECTOR3 vertical_speed_needle_location = _V(0.1205, 0.0818, 2.0201);

VECTOR3 horizon_circle_location = _V(0, 0.0818, 2.0201);

VECTOR3 tachometer_needle_location = _V(-0.3512, 0.0822, 2.0188);

VECTOR3 fuel_indicator_needle_location = _V(0.3512, 0.0822, 2.0188);

void R4::MakeAnim_MainRotor(){

    ANIMATIONCOMPONENT_HANDLE parent;

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

    parent = AddAnimationComponent(anim_main_rotor, 0, 1, &main_rotor);

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
        _V(0.8660, 0, 0.5),
        (float)(-15*RAD)
    );

    static unsigned int MainBlade3PitchGrp[2] = {34, 35};

    auto main_blade_3_pitch = new MGROUP_ROTATE(
        0,
        MainBlade3PitchGrp,
        2,
        main_rotor_axis,
        _V(-0.8660, 0, 0.5),
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

    ANIMATIONCOMPONENT_HANDLE parent;

    static unsigned int TailRotorGrp[1] = {41};

    static MGROUP_ROTATE tail_rotor(
        0,
        TailRotorGrp,
        1,
        tail_rotor_axis,
        _V(1, 0, 0),
        (float)(360*RAD)
    );

    anim_tail_rotor = CreateAnimation(0.0);

    parent = AddAnimationComponent(anim_tail_rotor, 0, 1, &tail_rotor);

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

    static unsigned int AirspeedGrp[1] = {21};
    anim_airspeed = CreateAnimation(0.0);

    static MGROUP_ROTATE airspeed(
        0,
        AirspeedGrp,
        1,
        airspeed_needle_location,
        indicator_axis,
        (float)(320*RAD)
    );

    AddAnimationComponent(anim_airspeed, 0, 1, &airspeed);

}

