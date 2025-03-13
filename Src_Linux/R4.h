//Lua Script R-4 for Orbiter
//Script written by Thunder Chicken
//R-4 mesh made by misha.physics
//February 2025

#pragma once

#include <array>
#include <cstddef>
#ifndef R4_H
#define R4_H

#include "HEADERS//Orbitersdk.h"
#include "HEADERS//OrbiterAPI.h"

const double FT = 0.3048; //multiply this by length in feet to get m
const double LB = 0.4538; //multiply this by mass in lbm to get kg

const char *const MESH_NAME = "R-4/R-4";

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

        //Dummy struct
        struct EngStruct{
            double diameter = 0.0;

            double prop_eff = 0.0;

            std::string fluid = "";

            double rp = 0.0;
        };

        //Rotor diameters (R-4)
        struct MainRotorSpec{
            
            double diameter = 38*FT;

            double prop_eff = 0.0; //initialize to 0, update with collective

            std::string fluid = "air"; //options: "air", "water", "sea_water"

        };

        struct TailRotorSpec{

            double diameter = 7.33*FT;

            double prop_eff = 0.0; //initialize to 0, set with rudder pedals

            std::string fluid = "air"; //options: "air", "water", "sea_water"

        };

        void clbkSetClassCaps(FILEHANDLE cfg) override;
        int clbkConsumeBufferedKey(int key, bool down, char *kstate) override;
        int clbkConsumeDirectKey(char *kstate) override;

        void SetContactPoints();
        void SetRollingWheels();
        void ParkingBrake();
        double ApplyBrakeForce();
        void SetLeftBrakeForce();
        void SetRightBrakeForce();

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

        double GetPropeller_Thrust(EngStruct, double, double);

        void SetPretty_NavLights(bool lights_on);
        void SetPretty_SearchLight(bool lights_on);
        void SetPretty_CabinLights();
        void SetPretty_ClearWindows();
        void SetPretty_StatusLights();
        void SetPretty_HelicopterColor();
        void SetPretty_CockpitGlow();


                

    private:

        MESHHANDLE hmesh;
        DEVMESHHANDLE hdevmesh0, hdevmesh1;
        MATERIAL *diffusive_color;
        MATERIAL *emissive_color;
        MATERIAL *mat_color;
        MATERIAL *cockpit_lights_emissive;
        MATERIAL *orange_mat;
        COLOUR4 *color;
        THRUSTER_HANDLE th_dummy, thg_dummy, th_hover, thg_hover;
        PROPELLANT_HANDLE main_fuel_tank;
        unsigned int ui_hmesh;
        BEACONLIGHTSPEC *beaconspec;
        BEACONLIGHTSPEC *searchlight_beaconspec;
        LightEmitter *beaconlight;
        LightEmitter *searchlight_spec;
        LightEmitter *cabinlight;
        NOTEHANDLE message1_annotation, message2_annotation, message3_annotation, message4_annotation, message5_annotation, message6_annotation, message7_annotation, message8_annotation, message9_annotation, message10_annotation, message11_annotation, message12_annotation, message13_annotation, message14_annotation, message15_annotation;

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


        bool altitude_hold;
        bool brake_hold;
        bool engine_on;
        bool show_help;
        bool info_switched;
        bool lights_on;
        bool lights_switched;
        bool light_level_switched;
        bool floats;

        double throttle_level;

        double collective_input;

        double instrument_light_level;
        double cabin_light_level;

        double altitude_target;

        double main_rotor_ratio = 0.5; //ratio of main rotor rpm to engine rpm

        double tail_rotor_ratio = 0.5; //ratio of tail rotor rpm to engine rpm

        double power;

        double torque;

        double rpm;

        //Main vessel locations for animations, useful mesh dimensions, etc..

        VECTOR3 main_rotor_axis = {0, 1.5862, 0};
        VECTOR3 tail_rotor_axis = {0.16415, 1.4049, -7.1219};

        VECTOR3 cg = {main_rotor_axis.x, -0.5, main_rotor_axis.z}; //location of CG relative to mesh

        double main_wheel_diameter = 0.488;
        double tail_wheel_diameter = 0.298;

        VECTOR3 left_main_wheel_axis = {-1.737, -1.0349, 0.26379};
        
        VECTOR3 right_main_wheel_axis = {1.737, -1.0349, 0.26379};

        VECTOR3 tail_wheel_axis = {0.0, -1.1298, -3.5673};

        VECTOR3 left_main_wheel_contact = operator+(left_main_wheel_axis, {0, -0.5*main_wheel_diameter, 0});

        VECTOR3 right_main_wheel_contact = operator+(right_main_wheel_axis, {0, -0.5*main_wheel_diameter, 0});

        VECTOR3 tail_wheel_contact = operator+(tail_wheel_axis, {0, -0.5*tail_wheel_diameter, 0});

        VECTOR3 main_rotor_thrust_vec = _V(0, 0, 0);


        //Camera viewpoints

        // Camera viewpoints
        std::array<VECTOR3, 3> camera_loc = {
            VECTOR3{-0.2, 0.3, 1.5} - cg, //pilot seat (was on left side in R-4)
            VECTOR3{ 0.0, 0.3, 1.5} - cg, //centered seat
            VECTOR3{ 0.2, 0.3, 1.5} - cg  //right seat
        };

};


#endif //!R4_H