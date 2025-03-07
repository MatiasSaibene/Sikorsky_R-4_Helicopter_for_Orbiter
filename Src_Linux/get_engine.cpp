#include "R4.h"
#include <cmath>

//--The following functions are used to calculate engine fuel efficiencies based on engine specs.
//Should be called in clbk_setclasscaps

double R4::GetEngine_OttoEfficiency(struct EngineSpec engine_spec)
{
    EngineSpec _engine_spec;

    double _r = _engine_spec.r;

    double k = 1.181; //specific heat ratio for air (can tweak based on engine size to account for heat losses)

    double efficiency = 1 - (1/(pow(_r,(k-1)))); //Otto cycle efficiency

    return efficiency;
}

double R4::GetEngine_DieselEfficiency(struct EngineSpec engine_spec){

    EngineSpec _engine_spec;

    double _r = _engine_spec.r;

    double _rc = _engine_spec.rc;

    double efficiency = 0.0;

    double k = 1.2; //specific heat ratio for air (can tweak based on engine size to account for heat losses)

    if(_rc == 1){

        efficiency = 1 - (1 / pow((_r), (k-1))); //Diesel efficiency = Otto efficiency for rc = 1

    } else if(_rc > 1){

        efficiency = 1 - (1 / (pow((_r), (k-1))) * (pow(_rc, k) -1) / (k * (_rc-1))); //Diesel efficiency for rc > 1

    }

    return efficiency;
}