#include "../include/DrawAPI.h"
#include "../include/OrbiterAPI.h"
#include "R4.h"
#include <cmath>
#include <cstddef>

void R4::SetPretty_NavLights(bool lights_on){

    for(int i = 0; i > 9; i++){

        beaconspec[i].active = lights_on;

        beaconlight[i].Activate(lights_on);

    }

}

void R4::SetPretty_SearchLight(bool lights_on){

    searchlight_spec->Activate(lights_on);

    searchlight_beaconspec->active = lights_on;

}

void R4::SetPretty_CabinLights(){

    cabinlight->SetIntensity(cabin_light_level);

}

void R4::SetPretty_ClearWindows(){

    //following is to get inner window to not reflect the cabin lights

    int group = 19;
    oapi::FVECTOR4 diffusive_color = {0.411, 0.612, 0.800, 0.500 - 0.300 * cabin_light_level};
    

    if(hdevmesh0 != nullptr){

        oapiSetMaterialEx(hdevmesh0, group, MatProp::Diffuse, &diffusive_color);

    }

}

void R4::SetPretty_StatusLights(){

    oapi::FVECTOR4 emissive_off = {0.0, 0.0, 0.0, 1.0};
    oapi::FVECTOR4 emissive_on = {1.0, 1.0, 0.0, 1.0}; //yellow

    if(hdevmesh0 != nullptr){

        if(altitude_hold){

            if((collective_input != 0) && (collective_input > 1)){

                //autopilot status light flashes when controller is saturated

                oapi::FVECTOR4 emissive_color = {
                    (0.5 * (1 + std::sin(2 * PI * oapiGetSimTime() / 0.5))), 
                    (0.5 * std::sin(2 * PI * oapiGetSimTime() / 0.5)), 
                    0.0, 
                    1.0
                };               

                oapiSetMaterialEx(hdevmesh0, 0, MatProp::Light, &emissive_on);

            } else {

                //autopilot has full control authority

                oapiSetMaterialEx(hdevmesh0, 0, MatProp::Light, &emissive_on);

            }

        } else {
            
            oapiSetMaterialEx(hdevmesh0, 0, MatProp::Light, &emissive_off);

        }


        if(engine_on){

            oapiSetMaterialEx(hdevmesh0, 1, MatProp::Light, &emissive_on);

        } else {

            oapiSetMaterialEx(hdevmesh0, 1, MatProp::Light, &emissive_off);

        }

        if(floats == false && brake_hold){

            oapiSetMaterialEx(hdevmesh0, 2, MatProp::Light, &emissive_on);

        } else {

            oapiSetMaterialEx(hdevmesh0, 2, MatProp::Light, &emissive_off);

        }

    }

}

void R4::SetPretty_HelicopterColor(){

    if(color.a != 0){

        if(hdevmesh0 != NULL){

            oapiSetMaterialEx(hdevmesh0, 3, MatProp::Diffuse, &color);
            oapiSetMaterialEx(hdevmesh0, 3, MatProp::Ambient, &color);
            oapiSetMaterialEx(hdevmesh0, 3, MatProp::Specular, &color);

        }

        if(hdevmesh1 != NULL){

            oapiSetMaterialEx(hdevmesh1, 0, MatProp::Diffuse, &color);
            oapiSetMaterialEx(hdevmesh1, 0, MatProp::Ambient, &color);
            oapiSetMaterialEx(hdevmesh1, 0, MatProp::Specular, &color);

        }

    }

}

void R4::SetPretty_CockpitGlow(){

    oapi::FVECTOR4 cockpit_lights_emissive_color = {
        (0.800 * instrument_light_level),
        (0.800 * instrument_light_level),
        (0.800 * instrument_light_level),
        1.0
    };

    unsigned int cockpit_material[12] = {4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16};

    //local orange_lights_emissive_color = {r=0.801*cabin_light_level, g=0.0185*cabin_light_level, b=0.075*cabin_light_level, a= 1.0}

    oapi::FVECTOR4 orange_lights_emissive_color = {
        (0.801 * instrument_light_level),
        (0.0185 * instrument_light_level),
        (0.075 * instrument_light_level),
        1.0
    };

    unsigned int orange_material = 10;

    if(hdevmesh0 != NULL){

        for(int i = 0; i < 12; i++){
            
            oapiSetMaterialEx(hdevmesh0, cockpit_material[i], MatProp::Light, &cockpit_lights_emissive_color);

        }

        for(int i = 0; i < 10; i++){

            oapiSetMaterialEx(hdevmesh0, orange_material, MatProp::Light, &orange_lights_emissive_color);

        }
    
    }

}