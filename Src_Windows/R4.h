//Lua Script R-4 for Orbiter
//Script written by Thunder Chicken
//R-4 mesh made by misha.physics
//February 2025

#pragma once

#include "HEADERS/DrawAPI.h"
#include <array>
#include <chrono>
#include <cstddef>
#ifndef R4_H
#define R4_H

#include "HEADERS//Orbitersdk.h"
#include "HEADERS//OrbiterAPI.h"

const double FT = 0.3048; //multiply this by length in feet to get m
const double LB = 0.4538; //multiply this by mass in lbm to get kg

const char *const FUS_MESH_NAME = "R-4/Fuselage";

const char *const GEAR_MESH_NAME = "R-4/Gears";

const char *const FLOAT_MESH_NAME = "R-4/Floats";


//Vessel properties (R-4, https://en.wikipedia.org/wiki/Sikorsky_R-4, https://www.nationalmuseum.af.mil/Visit/Museum-Exhibits/Fact-Sheets/Display/Article/195868/sikorsky-r-4b-hoverfly/)

const double empty_mass = 2011*LB + 100.0; //empty R-4 plus chonky pilot
const double main_fuel_tank_max = 180*LB;  //30 gallon capacity at 6 lbm/gal for avgas.

const double rho_sea = 1047; //density of sea water, kg/m3
const double rho_h20 = 1000; //density of fresh water, kg/m3

const double mu_s = 0.9; //Static friction coefficient, rubber on concrete (dry), dimensionless
const double mu_d = 0.7;  //Dynamic friction coefficient, rubber on concrete (dry), dimensionless




class R4 : public VESSEL4{

    public:

        R4(OBJHANDLE hVessel, int flightmodel);
        virtual ~R4();

        //Engine spec table (Warner Super Scarab 50 https://en.wikipedia.org/wiki/Warner_Scarab, https://www.warnerenginecompany.com/Warner Engines Specs.pdf)

        struct EngineSpec{
            
            double r = 5.20; //compression ratio

            int rc = 1; //cut-off ratio, 1 for Otto cycle, >1 for Diesel

            double displacement = 6.82e-3; //engine displacement in cubic meters

            int max_rpm = 2050; //maximum engine speed (rpm)

            int min_rpm = 750; //idle engine speed (rpm)

            int n_stroke = 4; //2 if 2-stroke, 4 if 4-stroke

            double HV = 43e6; //lower heating value of fuel (J/kg) (~43 MJ/kg for typical liquid fuels)

            double AF = 14.6;  //air fuel ratio (mass air/mass fuel) (stochiometric ~14.6:1 for typical liquid fuels)

        };

        struct GasTurbine_EngineSpec{

            int rp = 20; //pressure ratio

            int max_rpm = 3600; //maximum engine speed (rpm)

            int min_rpm = 1000; //idle engine speed (rpm)

            double max_air_flow = 69.4; //mass air flow at max rpm

            double HV = 43e6; //lower heating value of fuel (J/kg) (~43 MJ/kg for typical liquid fuels)

            double AF = 77.1; //air fuel ratio (mass air/mass fuel) (stochiometric ~14.6:1 for typical liquid fuels)
        };


        //Rotor diameters (R-4)
        struct MainRotorSpec{
            
            double diameter = 38*FT;

            double prop_eff = 0.0; //initialize to 0, update with collective

            std::string fluid = "air"; //options: "air", "water", "sea_water"

            double rp = 0.0;
        };

        struct TailRotorSpec{

            double diameter = 7.33*FT;

            double prop_eff = 0.0; //initialize to 0, set with rudder pedals

            std::string fluid = "air"; //options: "air", "water", "sea_water"

        };

        void clbkSetClassCaps(FILEHANDLE cfg) override;
        int clbkConsumeBufferedKey(DWORD key, bool down, char *kstate) override;
        int clbkConsumeDirectKey(char *kstate) override;
        void clbkPostCreation() override;
        void clbkVisualCreated(VISHANDLE vis, int refcount) override;
        void clbkPreStep (double simt, double simdt, double mjd) override;
        bool clbkLoadVC(int id) override;
        void clbkFocusChanged (bool getfocus, OBJHANDLE hNewVessel, OBJHANDLE hOldVessel) override;
        void clbkVisualDestroyed(VISHANDLE vis, int refcount) override;
        void clbkSaveState(FILEHANDLE scn) override;
        void clbkLoadStateEx(FILEHANDLE scn, void *vs) override;

        void SetFeature_SetContactPointsWheels();
        void SetFeature_MakeContactPointsFloats();
        void SetFeature_SetRollingWheels();
        void SetFeature_ParkingBrake();
        double SetFeature_ApplyBrakeForce();
        void SetFeature_SetLeftBrakeForce();
        void SetFeature_SetRightBrakeForce();
        void SetFeature_CrashOrSplash();

        double GetEngine_OttoEfficiency(struct EngineSpec);
        double GetEngine_DieselEfficiency(struct EngineSpec);
        double GetEngine_BraytonEfficiency(struct GasTurbine_EngineSpec);
        void GetEngine_ReciprocatingPower(double efficiency, struct GasTurbine_EngineSpec, PROPELLANT_HANDLE fuel_tank_handle, double throttle_level);
        void GetEngine_GasTurbinePower(double efficiency, struct GasTurbine_EngineSpec, PROPELLANT_HANDLE fuel_tank_handle, double throttle_level);


        void MakeAnim_MainRotor();
        void MakeAnim_TailRotor();
        void MakeAnim_AirspeedIndicator();
        void MakeAnim_Tachometer();
        void MakeAnim_Altimeter();
        void MakeAnim_Compass();
        void MakeAnim_VerticalSpeed();
        void MakeAnim_ArtificialHorizon();
        void MakeAnim_FuelIndicator();
        void MakeAnim_CollectiveIndicator();
        void MakeAnim_MainWheels();
        void MakeAnim_TailWheel();

        void SetAnim_MainRotor();
        void SetAnim_TailRotor();
        void SetAnim_AirspeedIndicator();
        void SetAnim_Altimeter();
        void SetAnim_Compass();
        void SetAnim_VerticalSpeed();
        void SetAnim_ArtificialHorizon();
        void SetAnim_Tachometer();
        void SetAnim_CollectiveIndicator();
        void SetAnim_FuelIndicator();
        void SetAnim_MainWheels();
        void SetAnim_TailWheel();


        VECTOR3 GetHelp_RotatePitch(VECTOR3 point, double pitch);
        VECTOR3 GetHelp_RotateYaw(VECTOR3 point, double yaw);
        VECTOR3 GetHelp_RotateBank(VECTOR3 point, double bank);
        VECTOR3 GetHelp_Rotate(VECTOR3 point, double pitch, double yaw, double bank);

        void GetPhysics_Drag();

        void SetAutopilot_Altitude();

        void MakePretty_NavLights();
        void MakePretty_SearchLight();
        void MakePretty_CabinLights();

        void SetAnnotation_Messages(bool show_help);
        void MakeAnnotation_Messages();

        template <typename T>
        double GetPropeller_Thrust(const T &rotor_spec, double, double);

        void SetPretty_NavLights(bool lights_on);
        void SetPretty_SearchLight(bool lights_on);
        void SetPretty_CabinLights();
        void SetPretty_ClearWindows();
        void SetPretty_StatusLights();
        void SetPretty_HelicopterColor();
        void SetPretty_CockpitGlow();


                

    private:

        MESHHANDLE hfuselage, hgear, hfloats;
        DEVMESHHANDLE hdevmesh0, hdevmesh1;
        oapi::FVECTOR4 color;
        THRUSTER_HANDLE th_dummy, thg_dummy, th_hover, thg_hover;
        PROPELLANT_HANDLE main_fuel_tank;
        unsigned int ui_hmesh;
        BEACONLIGHTSPEC beaconspec[3];
        BEACONLIGHTSPEC searchlight_beaconspec[1];
        LightEmitter *beaconlight;
        LightEmitter *searchlight_spec;
        LightEmitter *cabinlight;
        VESSEL *vi;
        OBJHANDLE hR4;
        NOTEHANDLE message1_annotation, message2_annotation, message3_annotation, message4_annotation, message5_annotation, message6_annotation, message7_annotation, message8_annotation, message9_annotation, message10_annotation, message11_annotation, message12_annotation, message13_annotation, message14_annotation, message15_annotation;

        double pitch, roll, bank;

        unsigned int anim_main_rotor;
        unsigned int anim_main_blade_1;
        unsigned int anim_main_blade_2;
        unsigned int anim_main_blade_3;
        unsigned int anim_tail_rotor;
        unsigned int anim_tail_blade_1;
        unsigned int anim_tail_blade_2;
        unsigned int anim_tail_blade_3;

        unsigned int anim_airspeed;
        unsigned int anim_altimeter_1k,anim_altimeter_10k;

        unsigned int anim_compass;

        unsigned int anim_vertical_speed;

        unsigned int anim_horizon_circle;
        unsigned int anim_horizon_ball_pitch;

        unsigned int anim_tachometer;

        unsigned int anim_fuel_indicator;

        unsigned int anim_collective_indicator;

        unsigned int anim_left_main_wheel,anim_right_main_wheel;

        unsigned int anim_tail_wheel_strut, anim_tail_wheel;

        double main_rotor_anim_state;
        double tail_rotor_anim_state;
        double airspeed_anim_state;
        double altimeter_10k_anim_state;
        double altimeter_1k_anim_state;
        double compass_anim_state;
        double vertical_speed_anim_state;
        double left_wheel_anim_state;
        double right_wheel_anim_state;
        double tail_wheel_strut_anim_state;
        double tail_wheel_anim_state;

        //Boolean flags
        bool altitude_hold;
        bool brake_hold;
        bool engine_on;
        bool show_help;
        bool info_switched;
        bool lights_on;
        bool lights_switched;
        bool light_level_switched;
        bool floats;
        bool water;

        double throttle_level;

        double collective_input;

        double instrument_light_level;
        double cabin_light_level;

        double altitude_target;

        double tail_rotor_dir;

        double main_rotor_ratio = 0.5; //ratio of main rotor rpm to engine rpm

        double tail_rotor_ratio = 0.5; //ratio of tail rotor rpm to engine rpm

        double power;

        double torque;

        double rpm;

        double efficiency;

        //Main vessel locations for animations, useful mesh dimensions, etc..

        VECTOR3 main_rotor_axis = {0, 1.5862, 0};
        VECTOR3 tail_rotor_axis = {0.16415, 1.4049, -7.1219};

        VECTOR3 cg = {main_rotor_axis.x, -0.5, main_rotor_axis.z}; //location of CG relative to mesh

        //Wheels
        double main_wheel_diameter = 0.488;
        double tail_wheel_diameter = 0.298;

        VECTOR3 left_main_wheel_axis = {-1.737, -1.0349, 0.26379};
        
        VECTOR3 right_main_wheel_axis = {1.737, -1.0349, 0.26379};

        VECTOR3 tail_wheel_axis = {0.0, -1.1298, -3.5673};

        VECTOR3 left_main_wheel_contact = operator+(left_main_wheel_axis, {0, -0.5*main_wheel_diameter, 0});

        VECTOR3 right_main_wheel_contact = operator+(right_main_wheel_axis, {0, -0.5*main_wheel_diameter, 0});

        VECTOR3 tail_wheel_contact = operator+(tail_wheel_axis, {0, -0.5*tail_wheel_diameter, 0});

        VECTOR3 main_rotor_thrust_vec;

        
        //Pontoons
        VECTOR3 right_pontoon_front = _V(1.3924, 1.3480, 1.5829);
        VECTOR3 right_pontoon_rear = _V(1.3924, -1.3480, -1.8318);
        VECTOR3 left_pontoon_front = _V(-1.3924, -1.3480, 1.5829);
        VECTOR3 left_pontoon_rear = _V(-1.3924, -1.3480, -1.8318);

        double pontoon_dia = 0.45;

        double pontoon_length = right_pontoon_front.z - right_pontoon_rear.z;


        //Instruments

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

        //Touchdown points
        static const int ntdvtx_td_points_pontoon_land = 20;
        TOUCHDOWNVTX td_points_pontoon_land[ntdvtx_td_points_pontoon_land];

        static const int ntdvtx_td_points = 13;
        TOUCHDOWNVTX td_points[13];

        static const int ntdvtx_td_points_pontoon_water = 10;
        TOUCHDOWNVTX td_points_pontoon_water[ntdvtx_td_points_pontoon_water];

        //Camera viewpoints

        // Camera viewpoints
        std::array<VECTOR3, 3> camera_loc = {
            VECTOR3{-0.2, 0.3, 1.5} - cg, //pilot seat (was on left side in R-4)
            VECTOR3{ 0.0, 0.3, 1.5} - cg, //centered seat
            VECTOR3{ 0.2, 0.3, 1.5} - cg  //right seat
        };

};


#endif //!R4_H