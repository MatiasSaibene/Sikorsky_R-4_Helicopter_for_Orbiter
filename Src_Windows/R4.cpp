#define NOMINMAX
#define ORBITER_MODULE
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
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

    main_rotor_thrust_vec = _V(0, 0, 0);

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

    for(int i = 0; i < 3; i++){

        beaconspec[i] = {0};

    }

    searchlight_beaconspec[0] = {0};

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

    hfuselage = nullptr;
    
    hgear = nullptr;

    hfloats = nullptr;
    
    hdevmesh0 = nullptr;

    hdevmesh1 = nullptr;

    beaconlight = nullptr;

    searchlight_spec = nullptr;

    cabinlight = nullptr;

    vi = nullptr;

    hR4 = nullptr;

    pitch = 0.0;

    roll = 0.0;

    bank = 0.0;

    main_rotor_anim_state = 0.0;

    tail_rotor_anim_state = 0.0;

    airspeed_anim_state = 0.0;

    altimeter_10k_anim_state = 0.0;

    altimeter_1k_anim_state = 0.0;

    compass_anim_state = 0.0;

    vertical_speed_anim_state = 0.0;

    left_wheel_anim_state = 0.0;

    left_wheel_anim_state = 0.0;

    right_wheel_anim_state = 0.0;
    
    tail_wheel_strut_anim_state = 0.0;

    tail_wheel_anim_state = 0.0;

    floats = false;

    water = false;

    throttle_level = 0.0;

    collective_input = 0.0;

    tail_rotor_dir = 0.0;

    power = 0.0;

    torque = 0.0;

    rpm = 0.0;

    rpm_comm = 0.0;

    efficiency = 0.0;

    for(int i = 0; i < ntdvtx_td_points_pontoon_land; i++){

        td_points_pontoon_land[i] = {0};

    }

    for(int i = 0; i < ntdvtx_td_points; i++){

        td_points[i] = {0};

    }

    for(int i = 0; i < ntdvtx_td_points_pontoon_water; i++){

        td_points_pontoon_water[i] = {0};

    }

    
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

    hfuselage = oapiLoadMeshGlobal(FUS_MESH_NAME);

    AddMesh(hfuselage);

    ShiftMesh(0, operator*(cg, -1));

    SetMeshVisibilityMode(0, MESHVIS_ALWAYS);

    /* hmesh = oapiLoadMeshGlobal(MESH_NAME);
    ui_hmesh = AddMesh(hmesh);
    SetMeshVisibilityMode(ui_hmesh, MESHVIS_ALWAYS);
    ShiftMesh(ui_hmesh, operator*(cg, -1)); */

    SetEmptyMass(empty_mass);
    main_fuel_tank = CreatePropellantResource(main_fuel_tank_max);

    th_hover = CreateThruster(cg, _V(0, 1, 0), 1, main_fuel_tank, INFINITY);

    thg_hover = CreateThrusterGroup(&th_hover, 1, THGROUP_HOVER);

    th_dummy = CreateThruster(cg, _V(0, 1, 0), 1, main_fuel_tank, INFINITY);

    thg_dummy = CreateThrusterGroup(&th_dummy, 1, THGROUP_MAIN);

    SetPMI(_V(2, 2, 2));

    efficiency = GetEngine_OttoEfficiency(); //get thermal efficiency of engine

    SetRotDrag(_V(0.5, 0.5, 0.5));

    //Define vertical stabilizer

    CreateAirfoil3(LIFT_HORIZONTAL, tail_rotor_axis, LiftFlatPlate, 0, 4, 4, 0.25);

    //Define artificial ailerons, elevator, and rudder to take control inputs

    CreateControlSurface(AIRCTRL_ELEVATOR, 0, 0, _V(0, 0, 0), AIRCTRL_AXIS_AUTO, 1);

    CreateControlSurface(AIRCTRL_AILERON, 0, 0, _V(0, 0, 0), AIRCTRL_AXIS_AUTO, 1);

    CreateControlSurface(AIRCTRL_RUDDER, 0, 0, _V(0, 0, 0), AIRCTRL_AXIS_AUTO, 2);

    //Following makes dummy thruster to take main throttle input and utilize throttle level
    //visuals in Orbiter. Major thrust forces are implemented with add_force instances.


    //Following makes dummy thruster to take default hover thuster input and apply it to the collective.

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

void R4::clbkLoadStateEx(FILEHANDLE scn, void *vs){

    MainRotorSpec main_rotor_spec;
    char *line;

    while(oapiReadScenario_nextline(scn, line)){

        if(!_strnicmp(line, "floats", 6)){
            
            char boolvalue[10];
            sscanf(line + 6, "%9s", boolvalue);

            if(!_strnicmp(boolvalue, "true", 4)){
                
                floats = true;

            } else if(!_strnicmp(boolvalue, "false", 5)){

                floats = false;

            }

        } else if (!_strnicmp(line, "color", 5)) {
            double r, g, b, a;
        
            if (sscanf(line + 5, " {r=%lf,g=%lf,b=%lf,a=%lf}", &r, &g, &b, &a) == 4) {
                color = COLOUR4{static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), static_cast<float>(a)};
            }

        } else if(!_strnicmp(line, "brake_hold", 10)){
            char boolvalue[10];
            sscanf(line+10, "%9s", boolvalue);

            if(_strnicmp(boolvalue, "true", 4)){

                brake_hold = true;

            } else if(!_strnicmp(boolvalue, "false", 5)){

                brake_hold = false;

            }
        } else if(!_strnicmp(line, "engine_on", 9)){
            char boolvalue[10];
            sscanf(line+9, "%9s", boolvalue);

            if(_strnicmp(boolvalue, "true", 4)){

                engine_on = true;

            } else if(!_strnicmp(boolvalue, "false", 5)){

                engine_on = false;

            }
        } else if(!_strnicmp(line, "lights_on", 9)){
            char boolvalue[10];
            sscanf(line+9, "%9s", boolvalue);

            if(_strnicmp(boolvalue, "true", 4)){

                lights_on = true;

            } else if(!_strnicmp(boolvalue, "false", 5)){

                lights_on = false;

            }
        } else if(!_strnicmp(line, "altitude_hold", 13)){
            char boolvalue[10];
            sscanf(line+13, "%9s", boolvalue);

            if(_strnicmp(boolvalue, "true", 4)){

                altitude_hold = true;

            } else if(!_strnicmp(boolvalue, "false", 5)){

                altitude_hold = false;

            }
        } else if(!_strnicmp(line, "altitude_target", 15)){
            double doublevalue;
            sscanf(line+15, "%lf", &doublevalue);

            altitude_target = doublevalue;

        } else if(!_strnicmp(line, "rpm", 3)){
            double doublevalue;
            sscanf(line+3, "%lf", &doublevalue);

            rpm = doublevalue;

        } else if(!_strnicmp(line, "throttle_level", 14)){
            double doublevalue;
            sscanf(line+14, "%lf", &doublevalue);

            throttle_level = doublevalue;
        } else if(!_strnicmp(line, "collective", 10)){
            double doublevalue;
            sscanf(line+10, "%lf", &doublevalue);

            main_rotor_spec.prop_eff = doublevalue;

        } else if(!_strnicmp(line, "cabin_light_level", 17)){
            double doublevalue;
            sscanf(line+17, "%lf", &doublevalue);

            cabin_light_level = doublevalue;
        } else if(!_strnicmp(line, "instrument_light_level", 22)){
            double doublevalue;
            sscanf(line+22, "%lf", &doublevalue);

            instrument_light_level = doublevalue;
        } else {

            ParseScenarioLineEx(line, vs);

        }

    }

}

void R4::clbkPostCreation(){

    /* hfuselage = oapiLoadMeshGlobal(FUS_MESH_NAME);

    AddMesh(hfuselage);

    ShiftMesh(0, operator*(cg, -1));

    SetMeshVisibilityMode(0, MESHVIS_ALWAYS); */



    if(floats == false){

        hgear = oapiLoadMeshGlobal(GEAR_MESH_NAME);

        AddMesh(hgear);

        ShiftMesh(1, operator*(cg, -1));

        SetMeshVisibilityMode(1, MESHVIS_ALWAYS);

        SetFeature_SetContactPointsWheels();

        MakeAnim_MainWheels();
        
        MakeAnim_TailWheel();

    } else if(floats == true){

        hfloats = oapiLoadMeshGlobal(FLOAT_MESH_NAME);

        AddMesh(hfloats);

        ShiftMesh(1, operator*(cg, -1));

        SetMeshVisibilityMode(1, MESHVIS_ALWAYS);

        SetFeature_MakeContactPointsFloats();
        
        SetTouchdownPoints(td_points_pontoon_land, ntdvtx_td_points_pontoon_land);
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

bool R4::clbkLoadVC(int id){

    SetCameraMovement(_V(0, 0, 0.05), 0, -20*RAD, _V(-0.1, 0, 0), 90*RAD, 0, _V(0.1, 0, 0), -90*RAD, 0);
    SetCameraDefaultDirection(_V(0, 0, 1));

    switch(id){

        case 0 : 
            SetCameraOffset(camera_loc[1]);
            oapiVCSetNeighbours(-1, 1, -1, -1);
        break;

        case 1:
            SetCameraOffset(camera_loc[2]);
            oapiVCSetNeighbours(0, 2, -1, -1);
        break;

        case 2:
            SetCameraOffset(camera_loc[3]);
            oapiVCSetNeighbours(1, -1, -1, -1);
        break;

    }

    return true;
}

void R4::clbkFocusChanged(bool getfocus, OBJHANDLE hNewVessel, OBJHANDLE hOldVessel){

    if(getfocus){

        SetAnnotation_Messages(show_help);

    } else {

        SetAnnotation_Messages(getfocus);

    }

}

void R4::clbkPreStep (double simt, double simdt, double mjd){

    VECTOR3 airspd = _V(0, 0, 0);
    GetAirspeedVector(FRAME_LOCAL, airspd);

    //Get flight control inputs

    pitch = GetControlSurfaceLevel(AIRCTRL_ELEVATOR);

    roll = GetControlSurfaceLevel(AIRCTRL_AILERON);

    tail_rotor_dir = -GetControlSurfaceLevel(AIRCTRL_RUDDER);

    tail_rotor_spec.prop_eff = std::abs(tail_rotor_dir);

    throttle_level = GetThrusterGroupLevel(THGROUP_MAIN);

    main_rotor_spec.prop_eff = GetThrusterGroupLevel(THGROUP_HOVER);
    
    //Set engine power and rotor thrust

    GetEngine_ReciprocatingPower(efficiency, main_fuel_tank, throttle_level);

    double main_rotor_thrust = GetPropeller_Thrust_MainRotor(0.9 * power, airspd.y);

    main_rotor_thrust_vec = GetHelp_Rotate(_V(0, main_rotor_thrust, 0), pitch * 6 * RAD, 0, roll * 6 * RAD);

    AddForce(main_rotor_thrust_vec, operator-(main_rotor_axis, cg));

    //Main rotor axial torque (realistic, but hard to fly with keyboard)

    //main_rotor_torque = main_engine_torque/main_rotor_ratio

    //vi:add_force({x=0, y=0, z=-main_rotor_torque}, {x= 0.5, y=main_rotor_axis.y, z=main_rotor_axis.z})
    //vi:add_force({x=0, y=0, z= main_rotor_torque}, {x=-0.5, y=main_rotor_axis.y, z=main_rotor_axis.z})

    //Get tail rotor thrust and net torque
    double tail_rotor_thrust = GetPropeller_Thrust_TailRotor(0.1 * power, airspd.x);

    VECTOR3 tail_rotor_thrust_vec = _V(tail_rotor_dir * tail_rotor_thrust, 0, 0);

    VECTOR3 tail_rotor_torque = crossp(tail_rotor_thrust_vec, tail_rotor_axis);


    AddForce(_V(0, 0, tail_rotor_torque.y), _V(0.5, main_rotor_axis.y, main_rotor_axis.z));

    AddForce(_V(0, 0, -tail_rotor_torque.y), _V(-0.5, main_rotor_axis.y, main_rotor_axis.z));

    
    //Get drag force on fuselage

    GetPhysics_Drag();

    //Set altitude hold autopilot

    SetAutopilot_Altitude();

    //Set contact points if using floats on either land or water

    if(floats == true){

        SetFeature_CrashOrSplash();

    }

    //Set wheel forces and animations

    if(GroundContact()){

        if(floats == false){

            SetFeature_SetRollingWheels();

            SetFeature_ParkingBrake();

            SetAnim_MainWheels();

            SetAnim_TailWheel();

        } else if(floats == true){

            if(water == false){

                SetAngularVel(_V(0, 0, 0));

            }

        }

    }

    //Set animations

    SetAnim_MainRotor();
    SetAnim_TailRotor();
    SetAnim_AirspeedIndicator();
    SetAnim_Tachometer();
    SetAnim_Altimeter();
    SetAnim_Compass();
    SetAnim_VerticalSpeed();
    SetAnim_ArtificialHorizon();
    SetAnim_FuelIndicator();
    SetAnim_CollectiveIndicator();
    
    //Set lights
    SetPretty_StatusLights();

    if(lights_switched == true){

        SetPretty_NavLights(lights_on);

        SetPretty_SearchLight(lights_on);

    }

    if(light_level_switched == true){

        SetPretty_ClearWindows(); //control inner window reflections

        SetPretty_CabinLights();

        SetPretty_CockpitGlow();

        light_level_switched = false;

    }

}

void R4::clbkVisualDestroyed(VISHANDLE vis, int refcount){

    hdevmesh0 = NULL;

    hdevmesh1 = NULL;

}

void R4::clbkSaveState(FILEHANDLE scn){

    SaveDefaultState(scn);

    MainRotorSpec main_rotor_spec;

    //char * strings (for std=C++20)

    char ch_floats[] = "floats";
    char ch_color[] = "color";
    char ch_true[] = "true";
    char ch_false[] = "false";
    char ch_brakeh[] = "brake_hold";
    char ch_engine[] = "engine_on";
    char ch_lights[] = "lights_on";
    char ch_althld[] = "altitude_hold";
    char ch_alttgt[] = "altitude_target";
    char ch_rpm[] = "rpm";
    char ch_thlvl[] = "throttle_level";
    char ch_collective[] = "collective";
    char ch_cabin_light_level[] = "cabin_light_level";
    char ch_instr_light_levl[] = "instrument_light_level";

    if(floats == true){

        oapiWriteScenario_string(scn, ch_floats, ch_true);

    } else if(floats == false){

        oapiWriteScenario_string(scn, ch_floats, ch_false);

    } else if(brake_hold == true){

        oapiWriteScenario_string(scn, ch_brakeh, ch_true);

    } else if(brake_hold == false){

        oapiWriteScenario_string(scn, ch_brakeh, ch_false);

    } else if(lights_on == true){

        oapiWriteScenario_string(scn, ch_lights, ch_true);

    } else if(lights_on == false){

        oapiWriteScenario_string(scn, ch_lights, ch_false);

    } else if(altitude_hold == true){

        oapiWriteScenario_string(scn, ch_althld, ch_true);

    }

    oapiWriteScenario_float(scn, ch_alttgt, altitude_target);

    oapiWriteScenario_float(scn, ch_rpm, rpm);

    oapiWriteScenario_float(scn, ch_thlvl, throttle_level);

    oapiWriteScenario_float(scn, ch_collective, main_rotor_spec.prop_eff);

    oapiWriteScenario_float(scn, ch_cabin_light_level, cabin_light_level);

    oapiWriteScenario_float(scn, ch_instr_light_levl, instrument_light_level);

    if(color.a != 0){

        std::string col_fmt = std::format("r=%lf, g=%lf, b=%lf, a=%lf", color.r, color.g, color.b, color.a);
        char* ch_col = new char[col_fmt.length() + 1];
        std::strcpy(ch_col, col_fmt.c_str());

        oapiWriteScenario_string(scn, ch_color, ch_col);
    }

}

int R4::clbkConsumeBufferedKey(DWORD key, bool down, char *kstate){

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

////////////////////////

DLLCLBK void InitModule(HINSTANCE hModule){


}

DLLCLBK void ExitModule(HINSTANCE *hModule){

}



///////////////Vessel initialization

DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){
    
	return new R4(hvessel, flightmodel);

}

/////////////Vessel memory cleanup
DLLCLBK void ovcExit(VESSEL *vessel){
    
	if(vessel) delete(R4*)vessel;
	
}
