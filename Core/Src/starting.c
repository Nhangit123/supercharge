/*
 * starting.c
 *
 *  Created on: Aug 2, 2025
 *      Author: Nhan
 */
#include "staring.h"
#include "timmer.h"
#include "bypass.h"
#include "digital_PI_rI_V2.h"
void Control_Init(void) {
    // Initialize hardware peripherals (PWM, ADC, etc.) here

    // Initialize parameters
    iParas.count = 0;
    iParas.RL = 0.1;
    iParas.C = 1e-3;       // Replaced pow(10,-3) with 1e-3
    iParas.L = 100e-6;     // Replaced 100*pow(10,-6)
    iParas.r_ds = 7.5e-3;
    iParas.r_f = 6.3e-3;
    iParas.V_f = 0.7;
    iParas.Imax = 2;
    iParas.Vmax = 30;

    // Switching frequency
    convParas.Fsw = 200000;
    convParas.Td = 1.0/convParas.Fsw;
    convParas.Tfi = 1.0/(5*convParas.Fsw);

    // PI voltage controller
    PIVarV.Tsv = 0.1/convParas.Fsw;
    PIVarV.Kpv = 0.05;
    PIVarV.Kiv = 0.14;
    PIVarV.Tiv = 1.0/PIVarV.Kiv;
    PIVarV.Dv = 1-PIVarV.Tsv/PIVarV.Tiv;
    PIVarV.uk_1_v = 0; PIVarV.uk_v = 0; PIVarV.usk_1_v = 0;
    PIVarV.ek_v = 0; PIVarV.ek_1_v = 0; PIVarV.esk_v = 0; PIVarV.esk_1_v = 0;
    PIVarV.uMax_v = iParas.Vmax; PIVarV.uMin_v = -PIVarV.uMax_v;

    // PI battery current controller
    PIVarIb.Tsb = 10.0/convParas.Fsw;
    PIVarIb.Kpb = -0.04;
    PIVarIb.Kib = -0.05;
    PIVarIb.Tib = 1.0/PIVarIb.Kib;
    PIVarIb.Db = 1-PIVarIb.Tsb/PIVarIb.Tib;
    PIVarIb.uk_1_b = 0; PIVarIb.uk_b = 0; PIVarIb.usk_1_b = 0;
    PIVarIb.ek_b = 0; PIVarIb.ek_1_b = 0; PIVarIb.esk_b = 0; PIVarIb.esk_1_b = 0;
    PIVarIb.iMax_b = iParas.Imax; PIVarIb.iMin_b = -PIVarIb.iMax_b;

    // PI capacitor current controller
    PIVarIc.Tsc = 10.0/convParas.Fsw;
    PIVarIc.Kpc = 1.33;
    PIVarIc.Kic = 10474;
    PIVarIc.Tic = 1.0/PIVarIc.Kic;
    PIVarIc.Dc = 1-PIVarIc.Tsc/PIVarIc.Tic;
    PIVarIc.uk_1_c = 0; PIVarIc.uk_c = 0; PIVarIc.usk_1_c = 0;
    PIVarIc.ek_c = 0; PIVarIc.ek_1_c = 0; PIVarIc.esk_c = 0; PIVarIc.esk_1_c = 0;
    PIVarIc.iMax_c = iParas.Imax; PIVarIc.iMin_c = -PIVarIc.iMax_c;

    // Low pass filter
    LP.tau = 10000;
    LP.T = 10e-3;       // Replaced 10*pow(10,-3)
    LP.ek_lp = 0; LP.ek_1_lp = 0;
    LP.uk_lp = 0; LP.uk_1_lp = 0;

    // Rate limit
    RL.count = 200000-1;
    RL.v = 20000;
    RL.bf1 = 0;
    RL.bf2 = 0;
    RL.bf3 = 0;

    //system state
    sysState = TU_0_DEN_12V;
}

void Control_Update(double V_CB, double V_Bat, double Vref, double Ib,
                    double* D, double* uk_V, double* uk_Ib,
                   double* I_SPb, double* Count, double* bf1,
                   double* bf2, double* bf3) {

    switch(sysState) {
        case TU_0_DEN_12V:
            if (V_CB < 13.0) {
                *D = 1.0;  // Full duty cycle if voltage too low
            }
            else {
            	delay_ns(50000);
            	sysState = TU_12_DEN_24V;
            }
            break;

        case TU_12_DEN_24V:
            RL.bf3 = Vref - V_CB;

            // Rate limited update
            RL.bf1 = RL.bf3;
            RL.count++;

            if(RL.count == RL.v) {
                PIVarV.ek_v = RL.bf3;
                RL.bf2 = PIVarV.ek_v;
                RL.count = 0;
            }
            else {
                PIVarV.ek_v = RL.bf2;
            }

            // Outer loop (voltage control)
            PIVarV.uk_v = PIVarV.Kpv * PIVarV.ek_v;

            // Current limiting
            if (PIVarV.uk_v > PIVarIb.iMax_b) {
                *I_SPb = PIVarIb.iMax_b;
            }
            else if (PIVarV.uk_v < PIVarIb.iMin_b) {
                *I_SPb = PIVarIb.iMin_b;
            }
            else {
                *I_SPb = PIVarV.uk_v;
            }

            *uk_V = PIVarV.uk_v;

            // Inner loop (current control)
            PIVarIb.ek_b = *I_SPb - Ib / 2.0;
            PIVarIb.esk_1_b = PIVarIb.ek_1_b + (1.0/PIVarIb.Kpb)*(PIVarIb.usk_1_b-PIVarIb.uk_1_b);
            PIVarIb.uk_b = PIVarIb.usk_1_b + PIVarIb.Kpb*(PIVarIb.ek_b-PIVarIb.Db*PIVarIb.esk_1_b);

            // Clamp duty cycle to [0, 1]
            if(PIVarIb.uk_b > 1.0) PIVarIb.uk_b = 1.0;
            if(PIVarIb.uk_b < 0.0) PIVarIb.uk_b = 0.0;

            *D = PIVarIb.uk_b;
            *uk_Ib = PIVarIb.uk_b;

            // Update states
            PIVarIb.ek_1_b = PIVarIb.ek_b;
            PIVarIb.uk_1_b = PIVarIb.uk_b;
            PIVarIb.usk_1_b = *D;

            iParas.count = 100;  // Maintain count at 100
            *Count = iParas.count;
            *bf1 = RL.bf1;
            *bf2 = RL.bf2;
            *bf3 = RL.bf3;

            if(V_CB >= 24.0) {
            	HAL_Delay(3000);
                enable_bypass();  // Enable bypass if voltage too high
                sysState = TU_24V_DEN_XV;
            }
            else {
                disable_bypass();
            }
            break;
        case TU_24V_DEN_XV:
        	PI_IBM_2P_Outputs_wrapper_vip(Vref, V_CB, 0, 0, D, 0, 0);


        	break;
    }

}

