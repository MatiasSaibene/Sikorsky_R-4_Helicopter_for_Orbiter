#include "R4.h"

void R4::MakePretty_NavLights(){



    VECTOR3 beaconpos[3];

    beaconpos[0] = operator-(_V(-0.58, 0.77, 0.81), cg);
    beaconpos[1] = operator-(_V(0.58, 0.77, 0.81), cg);
    beaconpos[2] = operator-(_V(0, 1.4049, 7.2), cg);


    VECTOR3 *beaconcol[3];

    *beaconcol[0] = _V(1, 0, 0); //Red
    *beaconcol[1] = _V(0, 1, 0); //Green
    *beaconcol[2] = _V(1, 1, 1); //White

    COLOUR4 beaconcol_light[3];

    beaconcol_light[0] = {1, 0, 0, 0}; //Red
    beaconcol_light[1] = {0, 1, 0, 0}; //Green
    beaconcol_light[2] = {1, 1, 1, 0}; //White


    VECTOR3 beacondir[3];

    beacondir[0] = _V(-1, 0, 0);
    beacondir[1] = _V(1, 0, 0);
    beacondir[2] = _V(0, 0, -1);

    for(int i = 0; i < sizeof(beaconpos); i++){

        beaconspec[i].shape = BEACONSHAPE_STAR;
        beaconspec[i].pos = &beaconpos[i];
        beaconspec[i].col = beaconcol[i];
        beaconspec[i].size = 0.1;
        beaconspec[i].falloff = 0.3;
        beaconspec[i].period = 0;
        beaconspec[i].duration = 0;
        beaconspec[i].tofs = 0;
        beaconspec[i].active = false;

        AddBeacon(beaconspec);


    
        beaconlight = AddSpotLight(beaconpos[i], beacondir[i], 0.1, 1e-2, 0, 2e-2, PI, PI, beaconcol_light[i], beaconcol_light[i], beaconcol_light[i]);
        beaconlight[i].SetVisibility(LightEmitter::VIS_ALWAYS);
        beaconlight[i].SetIntensity(0.4);
        beaconlight[i].Activate(false);

    }
    

}

void R4::MakePretty_SearchLight(){

    VECTOR3 searchlight_pos = operator-(_V(0, -0.49492, 2.093), cg);

    VECTOR3 searchlight_dir = GetHelp_RotatePitch(_V(0, 0, 1), (0 * RAD));

    searchlight_spec = AddSpotLight(searchlight_pos, searchlight_dir, 1000, 1e-5, 0, 2e-5, 0.3, 0.5*PI, {1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 1});

    searchlight_spec->SetVisibility(LightEmitter::VIS_ALWAYS);
    searchlight_spec->SetIntensity(1);
    searchlight_spec->Activate(false);

    VECTOR3 color_white = _V(1, 1, 1);

    searchlight_beaconspec->shape = BEACONSHAPE_COMPACT;
    searchlight_beaconspec->pos = &searchlight_pos;
    searchlight_beaconspec->col = &color_white;
    searchlight_beaconspec->size = 0.18;
    searchlight_beaconspec->falloff = 0.6;
    searchlight_beaconspec->period = 0;
    searchlight_beaconspec->duration = 0;
    searchlight_beaconspec->tofs = 0.6;
    searchlight_beaconspec->active = false;

    AddBeacon(searchlight_beaconspec);
    searchlight_beaconspec->active = false;

}

void R4::MakePretty_CabinLights(){

    VECTOR3 cabinlight_pos = _V(0.0, 1.0, 1.0);
    COLOUR4 cabinlight_col = {1, 0, 0};

    cabinlight = AddPointLight(cabinlight_pos, 19, 1e-3, 0, 2e-3, cabinlight_col, cabinlight_col, cabinlight_col);
    cabinlight->SetVisibility(LightEmitter::VIS_COCKPIT);
    cabinlight->SetIntensity(0.0);
    cabinlight->Activate(true);
    
}