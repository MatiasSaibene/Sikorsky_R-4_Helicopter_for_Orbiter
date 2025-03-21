#define NOMINMAX
#include <algorithm>

#include "R4.h"

double R4::GetPropeller_Thrust_MainRotor(double power, double V_0){

    double V_E = 0;

    double dia = main_rotor_spec.diameter;

    double prop_eff = main_rotor_spec.prop_eff;

    double prop_area = 0.25 * PI * dia * dia;

    double rho = 0.0;

    if(main_rotor_spec.fluid == "air"){

        rho = GetAtmDensity();

    } else if(main_rotor_spec.fluid == "water"){

        rho = rho_h20;

    } else if(main_rotor_spec.fluid == "sea_water"){

        rho = rho_sea;

    }

    if(power == 0){

        power = -prop_eff * 0.593 * 0.5 * rho * prop_area * V_0 * V_0 * V_0;

    }

    double A = 1;

    double B = V_0;

    double C = -V_0 * V_0;

    double D = -V_0 * V_0 * V_0 - (4 * prop_eff * power) / (rho * prop_area);

    //Y is a function derived from the energy equation across the propeller disk. Engine power
    //(derated by the propeller efficiency) changes the kinetic energy of the air between the far upstream
    //velocity V_0 (relative airspeed in the z direction) and the exit velocity V_E. In idealized
    //momentum flow the induced velocity through the disk is the average of V_O and V_E. This
    //yields a cubic polynomial expression for V_E with a single real root. The following finds the
    //corresponding value of this root at V_E and solves for the thrust.

    double Y[351];

    Y[0] = D;

    for(int i = 1; i < 350; i++){

        Y[i] = (A * i * i * i) + (B * i * i) + (C * i) + D;

        if(Y[i] > 0 && Y[i-1] < 0){

            double slope = Y[i-1]-Y[i];
            double x = -Y[i-1] / slope;
            V_E = i + x;

            break;

        } else {

            V_E = 0;

        }


    }

    double V = 0.5 * (V_E + V_0); //velocity at propeller disk, needed for mass flow

    double thrust = std::max(rho * prop_area * V * (V_E - V_0), 0.0);

    return thrust;

}

double R4::GetPropeller_Thrust_TailRotor(double power, double V_0){

    double V_E = 0;

    double dia = tail_rotor_spec.diameter;

    double prop_eff = tail_rotor_spec.prop_eff;

    double prop_area = 0.25 * PI * dia * dia;

    double rho = 0.0;

    if(tail_rotor_spec.fluid == "air"){

        rho = GetAtmDensity();

    } else if(tail_rotor_spec.fluid == "water"){

        rho = rho_h20;

    } else if(tail_rotor_spec.fluid == "sea_water"){

        rho = rho_sea;

    }

    if(power == 0){

        power = -prop_eff * 0.593 * 0.5 * rho * prop_area * V_0 * V_0 * V_0;

    }

    double A = 1;

    double B = V_0;

    double C = -V_0 * V_0;

    double D = -V_0 * V_0 * V_0 - (4 * prop_eff * power) / (rho * prop_area);

    //Y is a function derived from the energy equation across the propeller disk. Engine power
    //(derated by the propeller efficiency) changes the kinetic energy of the air between the far upstream
    //velocity V_0 (relative airspeed in the z direction) and the exit velocity V_E. In idealized
    //momentum flow the induced velocity through the disk is the average of V_O and V_E. This
    //yields a cubic polynomial expression for V_E with a single real root. The following finds the
    //corresponding value of this root at V_E and solves for the thrust.

    double Y[351];

    Y[0] = D;

    for(int i = 1; i < 350; i++){

        Y[i] = (A * i * i * i) + (B * i * i) + (C * i) + D;

        if(Y[i] > 0 && Y[i-1] < 0){

            double slope = Y[i-1]-Y[i];
            double x = -Y[i-1] / slope;
            V_E = i + x;

            break;

        } else {

            V_E = 0;

        }


    }

    double V = 0.5 * (V_E + V_0); //velocity at propeller disk, needed for mass flow

    double thrust = std::max(rho * prop_area * V * (V_E - V_0), 0.0);

    return thrust;

}