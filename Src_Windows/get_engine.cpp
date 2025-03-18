#define NOMINMAX
#include <algorithm>

#include "R4.h"
#include <cmath>

//--The following functions are used to calculate engine fuel efficiencies based on engine specs.
//Should be called in clbk_setclasscaps

double R4::GetEngine_OttoEfficiency()
{
    double _r = engine_spec.r;

    double k = 1.181; //specific heat ratio for air (can tweak based on engine size to account for heat losses)

    double efficiency = 1 - (1/(pow(_r,(k-1)))); //Otto cycle efficiency

    return efficiency;
}

double R4::GetEngine_DieselEfficiency(){

    double _r = engine_spec.r;

    double _rc = engine_spec.rc;

    double k = 1.2; //specific heat ratio for air (can tweak based on engine size to account for heat losses)

    if(_rc == 1){

        efficiency = 1 - (1 / pow((_r), (k-1))); //Diesel efficiency = Otto efficiency for rc = 1

    } else if(_rc > 1){

        efficiency = 1 - (1 / (pow((_r), (k-1))) * (pow(_rc, k) -1) / (k * (_rc-1))); //Diesel efficiency for rc > 1

    }

    return efficiency;
}

/* double R4::GetEngine_BraytonEfficiency(){

    double rp = gas_turbine_engine_spec.rp;
    double k = 1.2; //specific heat ratio for air (can tweak based on engine size to account for heat losses)

    efficiency = 1 - (1 / (pow(rp, (k-1) / k)));

    return efficiency;
} */

void R4::GetEngine_ReciprocatingPower(double efficiency, PROPELLANT_HANDLE fuel_tank_handle, double throttle_level){
   
    //Determines power and torque based on efficiency for Otto and Diesel engines

    //Ambient air properties

    double air_density = GetAtmDensity();

    //Engine specs needed for power calculation
    
    int min_rpm = engine_spec.min_rpm;

    int max_rpm = engine_spec.max_rpm;

    double displacement = engine_spec.displacement;

    int n_stroke = engine_spec.n_stroke;

    double HV = engine_spec.HV;

    double AF = engine_spec.AF;

    if(engine_on == true){

        rpm_comm = min_rpm + throttle_level * (max_rpm - min_rpm);

    } else if(engine_on == false && GroundContact() == false){

        rpm_comm = 0.5 * engine_spec.max_rpm;

    } else if (engine_on == false && GroundContact()){

        rpm_comm = 0;

    }

    if(rpm == 0){

        rpm = rpm + 0.01 * (rpm_comm - rpm);

    } else {

        rpm = rpm_comm;

    }

    //Calculate engine speed (omega in rad/s)
    double omega = rpm * (2 * PI) / 60;

    if(engine_on == true){

        //Calculate air mass flow rate

        double mass_flow_air = air_density * displacement * (2.0 / n_stroke) * rpm * (1.0 / 60); //air flow in kg/s
        double mass_flow_fuel = mass_flow_air / AF; //fuel flow in kg/s

        //Update fuel mass in tank

        double fuel_level = GetPropellantMass(fuel_tank_handle) - mass_flow_fuel * oapiGetSimStep();

        SetPropellantMass(fuel_tank_handle, std::max(fuel_level, 0.0));

        if(fuel_level > 0 && throttle_level > 0){

            double QH = mass_flow_fuel * HV;
            
            power = efficiency * QH;
            
            torque = power / omega;

        } else if(fuel_level <= 0){

            engine_on = false;

        }

    } else {
        
        power = 0.0;

        torque = 0.0;

    }

}

/* void R4::GetEngine_GasTurbinePower(double efficiency, PROPELLANT_HANDLE fuel_tank_handle, double throttle_level){

    //Determines power and torque based on efficiency for Brayton cycle turboshaft engines

    //Ambient air properties

    double air_density = GetAtmDensity();

    double air_density_stp = ATMD; //air density at STP from Orbiter

    //Engine specs needed for power calculation

    int min_rpm = gas_turbine_engine_spec.min_rpm;
    
    int max_rpm = gas_turbine_engine_spec.max_rpm;

    double max_air_flow = gas_turbine_engine_spec.max_air_flow;

    double HV = gas_turbine_engine_spec.HV;

    double AF = gas_turbine_engine_spec.AF;

    VECTOR3 airspd = _V(0, 0, 0);
    
    GetAirspeedVector(FRAME_LOCAL, airspd);


    if(engine_on == true){

        rpm_comm = min_rpm + throttle_level * (max_rpm - min_rpm);

    } else if(engine_on == false && GroundContact() == false){
        

        rpm_comm = ((2*7*airspd.z) / main_rotor_spec.diameter) * (60 / (2 * PI));
        //rpm_comm = 0.5*engine_spec.max_rpm

    } else if(engine_on == false && GroundContact()){

        rpm_comm = 0;

    }

    if(rpm != 0){

        rpm = rpm + 0.002 * (rpm_comm - rpm);

    } else {
        
        rpm = rpm_comm;

    }

    //Calculate engine speed (omega in rad/s)

    double omega = rpm * (2 * PI) / 60;

    if(engine_on == true){

        //Calculate air mass flow rate based on spec air flow and corrected for density

        double mass_flow_air = max_air_flow * (air_density / air_density_stp) * (rpm / max_rpm);
        double mass_flow_fuel = mass_flow_air / AF;
        
        double fuel_level = GetPropellantMass(fuel_tank_handle) - mass_flow_fuel * oapiGetSimStep();

        SetPropellantMass(fuel_tank_handle, fuel_level);

        if(fuel_level > 0 && throttle_level > 0){

            double QH = mass_flow_fuel * HV;

            power = efficiency * QH;

            torque = power / omega;

        } else if(fuel_level <= 0){ //out of fuel

            engine_on = false;

        }

    } else {

        power = 0.0;

        torque = 0.0;

    }

} */