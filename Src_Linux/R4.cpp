#include <algorithm>
#include <cmath>
#include <cstddef>
#define ORBITER_MODULE
#include "R4.h"

void LiftFlatPlate(VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd){

    //Simple flat plate airfoil cl = 4 pi sin(alpha)

    const int n_alpha = 37;
    double alpha[n_alpha] = {-180*RAD, -170*RAD, -160*RAD, -150*RAD, -140*RAD, -130*RAD, -120*RAD, -110*RAD, -100*RAD, -90*RAD, -80*RAD, -70*RAD, -60*RAD, -50*RAD, -40*RAD, -30*RAD, -20*RAD, -10*RAD, 0*RAD, 10*RAD, 20*RAD, 30*RAD, 40*RAD, 50*RAD, 60*RAD, 70*RAD, 80*RAD, 90*RAD, 100*RAD, 110*RAD, 120*RAD, 130*RAD, 140*RAD, 150*RAD, 160*RAD, 170*RAD, 180*RAD};

    double cl_a[n_alpha] = {0.000, -2.182, -4.298, -6.283, -8.078, -9.626, -10.883, -11.809, -12.375, -12.566, -12.375, -11.809, -10.883, -9.626, -8.078, -6.283, -4.298, -2.182, 0.000, 2.182, 4.298, 6.283, 8.078, 9.626, 10.883, 11.809, 12.375, 12.566, 12.375, 11.809, 10.883, 9.626, 8.078, 6.283, 4.298, 2.182, 0.000};

    for(int i = 0; i < n_alpha - 1; i++){
        
        if((aoa >= alpha[i]) && (aoa < alpha[i+1])){

            *cl = cl_a[i] + ((aoa - alpha[i]) / (alpha[i+1] - alpha[i]) * cl_a[i+1] - cl_a[i]);

        }

    }

    *cm = 0.0;

    *cd = 0.0;
}

//Constructor

R4::R4(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel){

    //Initialize variables...
    hmesh = NULL;

    ui_hmesh = 0;

    main_fuel_tank = nullptr;

    altitude_hold = false;

    altitude_target = 0.0;

    brake_hold = false;

    engine_on = false;

    show_help = false;

    info_switched = false;

    lights_on = false;

    lights_switched = false;

    light_level_switched = false;

    instrument_light_level = 0.0;

    cabin_light_level = 0.0;

    th_dummy = nullptr;

    thg_dummy = nullptr;

    th_hover = nullptr;

    thg_hover = nullptr;

    anim_main_rotor = 0;

    anim_main_blade_1 = 0;

    anim_main_blade_2 = 0;

    anim_main_blade_3 = 0;

    anim_tail_rotor = 0;

    anim_tail_blade_1 = 0;

    anim_tail_blade_2 = 0;

    anim_tail_blade_3 = 0;

    anim_airspeed = 0;

    anim_airspeed = 0;

    anim_airspeed = 0;

    anim_airspeed = 0;

    anim_airspeed = 0;

    anim_airspeed = 0;

    anim_airspeed = 0;

    anim_altimeter_1k = 0;

    anim_altimeter_10k = 0;

    anim_compass = 0;

    anim_vertical_speed = 0;

    anim_horizon_circle = 0;

    anim_horizon_ball_pitch = 0;

    anim_tachometer = 0;

    anim_fuel_indicator = 0;

    anim_collective_indicator = 0;

    anim_left_main_wheel = 0;

    anim_right_main_wheel = 0;

    anim_tail_wheel_strut = 0;

    anim_tail_wheel = 0;

    beaconspec = nullptr;

    searchlight_beaconspec = nullptr;

    message1_annotation = nullptr;
    message2_annotation = nullptr;
    message3_annotation = nullptr;
    message4_annotation = nullptr;
    message5_annotation = nullptr;
    message6_annotation = nullptr;
    message7_annotation = nullptr;
    message8_annotation = nullptr;
    message9_annotation = nullptr;
    message10_annotation = nullptr;
    message11_annotation = nullptr;
    message12_annotation = nullptr;
    message13_annotation = nullptr;
    message14_annotation = nullptr;
    message15_annotation = nullptr;


    //Initial control settings

    double roll = 0.0;

    double pitch = 0.0;

    bool show_help = false;

    int info_delay = 10;
}

//Destructor

R4::~R4(){

}

void R4::clbkSetClassCaps(FILEHANDLE cfg){

    hmesh = oapiLoadMeshGlobal(MESH_NAME);
    ui_hmesh = AddMesh(hmesh);
    SetMeshVisibilityMode(ui_hmesh, MESHVIS_ALWAYS);
    ShiftMesh(ui_hmesh, operator*(cg, -1));

    SetEmptyMass(empty_mass);
    main_fuel_tank = CreatePropellantResource(main_fuel_tank_max);

    SetPMI(_V(2, 2, 2));

    SetContactPoints();

    double efficiency = GetEngine_OttoEfficiency(engine_spec);

    SetRotDrag(_V(0.5, 1.0, 0.5));

    //Define vertical stabilizer

    CreateAirfoil3(LIFT_HORIZONTAL, tail_rotor_axis, LiftFlatPlate, 0, 2, 4, 4);

    //Define artificial ailerons, elevator, and rudder to take control inputs

    CreateControlSurface(AIRCTRL_ELEVATOR, 0, 0, _V(0, 0, 0), AIRCTRL_AXIS_AUTO, 1);

    CreateControlSurface(AIRCTRL_AILERON, 0, 0, _V(0, 0, 0), AIRCTRL_AXIS_AUTO, 1);

    CreateControlSurface(AIRCTRL_RUDDER, 0, 0, _V(0, 0, 0), AIRCTRL_AXIS_AUTO, 2);

    //Following makes dummy thruster to take main throttle input and utilize throttle level
    //visuals in Orbiter. Major thrust forces are implemented with add_force instances.

    th_dummy = CreateThruster(cg, _V(0, 1, 0), 0, main_fuel_tank, INFINITY);

    thg_dummy = CreateThrusterGroup(&th_dummy, 1, THGROUP_MAIN);

    //Following makes dummy thruster to take default hover thuster input and apply it to the collective.

    th_hover = CreateThruster(cg, _V(0, 1, 0), 0, main_fuel_tank, INFINITY);

    thg_hover = CreateThrusterGroup(&th_hover, 1, THGROUP_HOVER);



}

int R4::clbkConsumeBufferedKey(int key, bool down, char *kstate){

    if(key == OAPI_KEY_A && down){

        altitude_hold = true;

        altitude_target = GetAltitude(ALTMODE_MEANRAD);

    } else if(altitude_hold == true){

        altitude_hold = false;

    }

    if(KEYMOD_LCONTROL(kstate) || KEYMOD_RCONTROL(kstate)){

        if(key == OAPI_KEY_B){

            if(brake_hold == false){

                brake_hold = true;

            } else if(brake_hold == true){

                brake_hold = false;

            }

        }

    }

    if(key == OAPI_KEY_E && down){

        if(engine_on == false){

            engine_on = true;

        } else if (engine_on == true) {

            engine_on = false;
            SetThrusterGroupLevel(THGROUP_MAIN, 0);
        }

    }

    if(key == OAPI_KEY_I && down){

        if(show_help == false){

            show_help = true;

        } else if(show_help == true){

            show_help = false;

        }

        info_switched = true;

    }

    if(KEYMOD_LCONTROL(kstate) || KEYMOD_RCONTROL(kstate)){

        if(key == OAPI_KEY_L && down){

            if(lights_on == false){

                lights_on = true;

            } else if(lights_on == true){

                lights_on = false;

            }

            lights_switched = true;

        }

    }

    if(KEYMOD_LSHIFT(kstate) || KEYMOD_RSHIFT(kstate)){

        if(key == OAPI_KEY_EQUALS && down){

            instrument_light_level = std::min(instrument_light_level + 0.1, 1.0);

        }else if(key == OAPI_KEY_MINUS && down){

            instrument_light_level = std::max(instrument_light_level - 0.1, 0.0);

        }

        light_level_switched = true;

    }

    if(KEYMOD_LCONTROL(kstate) || KEYMOD_RCONTROL(kstate)){

        if(OAPI_KEY_EQUALS && down){

            cabin_light_level = std::min(cabin_light_level + 0.1, 1.0);

        } else if(OAPI_KEY_MINUS && down){

            cabin_light_level = std::max(cabin_light_level - 0.1, 0.0);

        }

        light_level_switched = true;

    }


    return 0;
}

int R4::clbkConsumeDirectKey(char *kstate){

    if(KEYDOWN(kstate, OAPI_KEY_B)){

        SetLeftBrakeForce();
        SetRightBrakeForce();
    }

    return 0;
}