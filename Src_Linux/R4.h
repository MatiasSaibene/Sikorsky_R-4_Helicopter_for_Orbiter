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

        } engine_spec;

        //Rotor diameters (R-4)
        struct MainRotorSpec{
            
            double diameter = 38*FT;

            double prop_eff = 0.0; //initialize to 0, update with collective

            std::string fluid = "air"; //options: "air", "water", "sea_water"

        } main_rotor_spec;

        struct TailRotorSpec{

            double diameter = 7.33*FT;

            double prop_eff = 0.0; //initialize to 0, set with rudder pedals

            std::string fluid = "air"; //options: "air", "water", "sea_water"

        } tail_rotor_spec;

        void clbkSetClassCaps(FILEHANDLE cfg) override;
        int clbkConsumeBufferedKey(int key, bool down, char *kstate) override;
        int clbkConsumeDirectKey(char *kstate) override;



        void SetContactPoints();
        void SetRollingWheels();
        void ParkingBrake();
        double ApplyBrakeForce();
        void SetLeftBrakeForce();
        void SetRightBrakeForce();
        double GetEngine_OttoEfficiency(struct EngineSpec engine_spec);
        double GetEngine_DieselEfficiency(struct EngineSpec engine_spec);

        void MakeAnim_MainRotor();
        void MakeAnim_TailRotor();
        void MakeAnim_AirspeedIndicator();
        void MakeAnim_Tachometer();
        void MakeAnim_Altimeter();
        void MakeAnim_Compass();
        void MakeAnim_Vertical_Speed();
        void MakeAnim_Artificial_Horizon();
        void MakeAnim_Fuel_Indicator();
        void MakeAnim_Collective_Indicator();


    private:

        MESHHANDLE hmesh;
        THRUSTER_HANDLE th_dummy, thg_dummy, th_hover, thg_hover;
        PROPELLANT_HANDLE main_fuel_tank;
        unsigned int ui_hmesh;

        unsigned int anim_main_rotor;
        unsigned int anim_main_blade_1;
        unsigned int anim_main_blade_2;
        unsigned int anim_main_blade_3;
        unsigned int anim_tail_rotor;
        unsigned int anim_tail_blade_1;
        unsigned int anim_tail_blade_2;
        unsigned int anim_tail_blade_3;

        unsigned int anim_airspeed;

        bool altitude_hold;
        bool brake_hold;
        bool engine_on;
        bool show_help;
        bool info_switched;
        bool lights_on;
        bool lights_switched;
        bool light_level_switched;

        double instrument_light_level;
        double cabin_light_level;

        double altitude_target;

        double main_rotor_ratio = 0.5; //ratio of main rotor rpm to engine rpm

        double tail_rotor_ratio = 0.5; //ratio of tail rotor rpm to engine rpm

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

        VECTOR3 main_rotor_thrust_vec;


        //Camera viewpoints

        // Camera viewpoints
        std::array<VECTOR3, 3> camera_loc = {
            VECTOR3{-0.2, 0.3, 1.5} - cg, //pilot seat (was on left side in R-4)
            VECTOR3{ 0.0, 0.3, 1.5} - cg, //centered seat
            VECTOR3{ 0.2, 0.3, 1.5} - cg  //right seat
        };

};


#endif //!R4_H