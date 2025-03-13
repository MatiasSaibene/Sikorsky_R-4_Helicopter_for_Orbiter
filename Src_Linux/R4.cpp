#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
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

    hR4 = GetHandle();

    vi = oapiGetVesselInterface(hR4);

    EngineSpec _engine_spec;

    /* hmesh = oapiLoadMeshGlobal(MESH_NAME);
    ui_hmesh = AddMesh(hmesh);
    SetMeshVisibilityMode(ui_hmesh, MESHVIS_ALWAYS);
    ShiftMesh(ui_hmesh, operator*(cg, -1)); */

    SetEmptyMass(empty_mass);
    main_fuel_tank = CreatePropellantResource(main_fuel_tank_max);

    SetPMI(_V(2, 2, 2));

    SetContactPoints();

    double efficiency = GetEngine_OttoEfficiency(_engine_spec); //get thermal efficiency of engine

    SetRotDrag(_V(0.5, 0.5, 0.5));

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

    MakeAnim_MainRotor();
    MakeAnim_TailRotor();
    MakeAnim_AirspeedIndicator();
    MakeAnim_Tachometer();
    MakeAnim_Altimeter();
    MakeAnim_Compass();
    MakeAnim_VerticalSpeed();
    MakeAnim_ArtificialHorizon();
    MakeAnim_FuelIndicator();
    MakeAnim_CollectiveIndicator();

    MakePretty_NavLights();
    MakePretty_SearchLight();
    MakePretty_CabinLights();

    MakeAnnotation_Messages();

}

void R4::clbkPostCreation(){

    hfuselage = oapiLoadMeshGlobal(FUS_MESH_NAME);

    AddMesh(hfuselage);

    ShiftMesh(0, operator*(cg, -1));

    SetMeshVisibilityMode(0, MESHVIS_ALWAYS);



    if(floats == false){

        hgear = oapiLoadMeshGlobal(GEAR_MESH_NAME);

        AddMesh(hgear);

        ShiftMesh(1, operator*(cg, -1));

        SetMeshVisibilityMode(1, MESHVIS_ALWAYS);

        SetFeature_SetContactPointsWheels();

        MakeAnim_MainWheels();
        
        MakeAnim_TailWheel();

    } else if(floats == false){

        hfloats = oapiLoadMeshGlobal(FLOAT_MESH_NAME);

        AddMesh(hfloats);

        ShiftMesh(1, operator*(cg, -1));

        SetMeshVisibilityMode(1, MESHVIS_ALWAYS);

        SetFeature_MakeContactPointsFloats();
        
        SetTouchdownPoints(td_points_pontoon_land, sizeof(pos));
    }

}

void R4::clbkVisualCreated(VISHANDLE vis, int refcount){

    hdevmesh0 = GetDevMesh(vis, 0);
    hdevmesh1 = GetDevMesh(vis, 1);

    SetPretty_ClearWindows(); //control inner window reflections
    SetPretty_CabinLights();
    SetPretty_CockpitGlow();
    SetPretty_HelicopterColor();

}

void R4::clbkVisualDestroyed(VISHANDLE vis, int refcount){

    hdevmesh0 = NULL;

    hdevmesh1 = NULL;

}

void R4::clbkSaveState(FILEHANDLE scn){

    MainRotorSpec main_rotor_spec;

    std::string strboolfloats = std::to_string(floats);
    const char *boolfloats = strboolfloats.c_str();


    std::string strboolbrk = std::to_string(brake_hold);
    const char *boolbrk = strboolbrk.c_str();


    std::string strbooleng = std::to_string(engine_on);
    const char *booleng = strbooleng.c_str();

    std::string strboollights = std::to_string(lights_on);
    const char *boollights = strboollights.c_str();

    std::string strboolatthold = std::to_string(altitude_hold);
    const char *boolatthold = strboolatthold.c_str();

    std::string stralttgt = std::to_string(altitude_target);
    const char *alt_target = stralttgt.c_str();

    std::string strrpm = std::to_string(rpm);
    const char *n_rpm = strrpm.c_str();

    std::string strthlevel = std::to_string(throttle_level);
    const char *thlevel = strthlevel.c_str();

    std::string strcollective = std::to_string( main_rotor_spec.prop_eff);
    const char *collectivepropeff = strcollective.c_str();

    std::string strcabinlightlvl = std::to_string(cabin_light_level);
    const char *cabinlightlvl = strcabinlightlvl.c_str();

    std::string strinstlightlvl = std::to_string(instrument_light_level);
    const char *instlightlvl = strinstlightlvl.c_str();

    oapiWriteScenario_string(scn, "float", boolfloats);

    if(color != NULL){

        std::string colorStr = std::format("{{r={},g={},b={},a={}}}", color->r, color->g, color->b, color->a);

        oapiWriteScenario_string(scn, "color", colorStr.c_str());

    }



    oapiWriteScenario_string(scn, "brake_hold", boolbrk);

    oapiWriteScenario_string(scn, "engine_on", booleng);

    oapiWriteScenario_string(scn, "lights_on", boollights);

    oapiWriteScenario_string(scn, "altitude_hold", boolatthold);

    oapiWriteScenario_string(scn, "altitude_target", alt_target);

    oapiWriteScenario_string(scn, "rpm", n_rpm);

    oapiWriteScenario_string(scn, "throttle_level", thlevel);

    oapiWriteScenario_string(scn, "collective", collectivepropeff);

    oapiWriteScenario_string(scn, "cabin_light_level", cabinlightlvl);

    oapiWriteScenario_string(scn, "instrument_light_level", instlightlvl);

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

        if(floats == false){

            SetFeature_SetLeftBrakeForce();
            SetFeature_SetRightBrakeForce();

        }
        
    }

    return 0;
}