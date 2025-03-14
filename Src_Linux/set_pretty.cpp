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
    COLOUR4 diff_col = {0.411, 0.612, 0.800, static_cast<float>(0.500 - 0.300 * cabin_light_level)};
    diffusive_color.diffuse = diff_col;
    

    if(hdevmesh0 != nullptr){

        oapiSetMaterial(hdevmesh0, group, &diffusive_color);

    }

}

void R4::SetPretty_StatusLights(){

    COLOUR4 emissive_off = {0.0, 0.0, 0.0, 1.0};
    COLOUR4 emissive_on = {1.0, 1.0, 0.0, 1.0}; //yellow

    if(hdevmesh0 != nullptr){

        if(altitude_hold){

            if((collective_input != 0) && (collective_input > 1)){

                //autopilot status light flashes when controller is saturated

                COLOUR4 emiss_col = {
                    static_cast<float>(0.5 * (1 + std::sin(2 * PI * oapiGetSimTime() / 0.5))),
                    static_cast<float>(0.5 * std::sin(2 * PI * oapiGetSimTime() / 0.5)),
                    0.0,
                    1.0
                };
                

                emissive_color.emissive = emiss_col;

                oapiSetMaterial(hdevmesh0, 0, &emissive_color);

            } else {

                //autopilot has full control authority

                emissive_color.emissive = emissive_on;

                oapiSetMaterial(hdevmesh0, 0, &emissive_color);

            }

        } else {
            
            emissive_color.emissive = emissive_off;

            oapiSetMaterial(hdevmesh0, 0, &emissive_color);

        }


        if(engine_on){

            emissive_color.emissive = emissive_on;

            oapiSetMaterial(hdevmesh0, 1, &emissive_color);
        } else {

            emissive_color.emissive = emissive_off;

            oapiSetMaterial(hdevmesh0, 1, &emissive_color);

        }

        if(floats == false && brake_hold){

            emissive_color.emissive = emissive_on;

            oapiSetMaterial(hdevmesh0, 2, &emissive_color);

        } else {

            emissive_color.emissive = emissive_off;

            oapiSetMaterial(hdevmesh0, 2, &emissive_color);

        }

    }

}

void R4::SetPretty_HelicopterColor(){

    if(color.a != 0){

        if(hdevmesh0 != NULL){

            mat_color.diffuse = color;
            mat_color.ambient = color;
            mat_color.specular = color;

            oapiSetMaterial(hdevmesh0, 3, &mat_color);

        }

        if(hdevmesh1 != NULL){

            mat_color.diffuse = color;
            mat_color.ambient = color;
            mat_color.specular = color;

            oapiSetMaterial(hdevmesh1, 0, &mat_color);
        }

    }

}

void R4::SetPretty_CockpitGlow(){

    COLOUR4 cockpit_lights_emissive_color = {
        static_cast<float>(0.800 * instrument_light_level),
        static_cast<float>(0.800 * instrument_light_level),
        static_cast<float>(0.800 * instrument_light_level),
        1.0
    };

    unsigned int cockpit_material[12] = {4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16};

    //local orange_lights_emissive_color = {r=0.801*cabin_light_level, g=0.0185*cabin_light_level, b=0.075*cabin_light_level, a= 1.0}

    COLOUR4 orange_lights_emissive_color = {
        static_cast<float>(0.801 * instrument_light_level),
        static_cast<float>(0.0185 * instrument_light_level),
        static_cast<float>(0.075 * instrument_light_level),
        1.0
    };

    unsigned int orange_material = 10;

    if(hdevmesh0 != NULL){

        for(int i = 0; i < 12; i++){
            
            cockpit_lights_emissive.emissive = cockpit_lights_emissive_color;

            oapiSetMaterial(hdevmesh0, cockpit_material[i], &cockpit_lights_emissive);

        }

        for(int i = 0; i < 10; i++){

            orange_mat.emissive = orange_lights_emissive_color;

            oapiSetMaterial(hdevmesh0, orange_material, &orange_mat);

        }
    
    }

}