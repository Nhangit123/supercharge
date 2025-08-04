#include "digital_PI_rI_V2.h"
typedef struct{
    double RL; // Ohm of inductor
    double L;  // inductor
    double C;  // capacitor
    double R;  // Load
    double r_ds; // Res of IGBT
    double r_f; // Res of DIODE
    double V_f; // voltage in diode
    double Imax; // max current in inductor
    double Vmax; // maxx voltage in Bankcap
}Interleave;
static Interleave iParas;

typedef struct{
    double D;    // Duty
    double I;    //current in load
    double IL;   // Current in Inductor
    double t;   //time constant
    double td;  //dumping time constant
}Energy;
static Energy Eng;
typedef struct{
    double Fsw; // switch frequency Hz
    double Td;	//Equivalent time constant of converter
    double Tfi;	//Time constant of sensors
}Conv;
static Conv convParas;

typedef struct
{
 double Kpv, Ksv, Kiv,Tiv, Dv;
 double uk_1_v, uk_v, usk_1_v;
 double ek_v, ek_1_v, esk_v, esk_1_v;
 double uMax_v, uMin_v;
 double Tsv;			//second_sampling time
}PI_V;
static PI_V PIVarV;
typedef struct
{
 double Kpi, Ksi, Kii,Tii, Di;
 double uk_1_i, uk_i, usk_1_i;
 double ek_i, ek_1_i, esk_i, esk_1_i;
 double iMax_i, iMin_i;
 double Tsi;			//second_sampling time
}PI_I;
static PI_I PIVarI;
/* Externs_END */

void PI_IBM_2P_Start_wrapper()
{
/* Start_BEGIN */
//Interleave parameters
iParas.RL=0.1;
iParas.C=pow(10,-3);
iParas.L=4.7*pow(10,-4);
iParas.r_ds=7.5*pow(10,-3);
iParas.r_f=6.3*pow(10,-3);
iParas.V_f=0.7;
iParas.Imax=4;
iParas.Vmax=30;
//switching frequence
convParas.Fsw = 200000;
convParas.Td = 1/(convParas.Fsw);
convParas.Tfi = 1/(5*convParas.Fsw);
//PI current controller
PIVarI.Tsi= 10/(convParas.Fsw);
PIVarI.Kpi=-0.05 ;
PIVarI.Kii=0.04 ;
PIVarI.Tii=1/PIVarI.Kii;
PIVarI.Di=1-PIVarI.Tsi/PIVarI.Tii;
PIVarI.uk_1_i = 0; PIVarI.uk_i = 0; PIVarI.usk_1_i = 0;
PIVarI.ek_i = 0; PIVarI.ek_1_i = 0; PIVarI.esk_i = 0; PIVarI.esk_1_i = 0;
PIVarI.iMax_i = iParas.Imax; PIVarI.iMin_i = -PIVarI.iMax_i;
//PI voltage controller
PIVarV.Tsv = 10/(convParas.Fsw*3);
PIVarV.Kpv= 0.05;
PIVarV.Kiv= 0.15;
PIVarV.Tiv=1.0/PIVarV.Kiv;
PIVarV.Dv =1-PIVarV.Tsv/PIVarV.Tiv;
PIVarV.uk_1_v = 0; PIVarV.uk_v = 0; PIVarV.usk_1_v = 0;
PIVarV.ek_v = 0; PIVarV.ek_1_v = 0; PIVarV.esk_v = 0; PIVarV.esk_1_v = 0;
PIVarV.uMax_v = iParas.Vmax; PIVarV.uMin_v = -PIVarV.uMax_v;
/* Start_END */
}

void PI_IBM_2P_Outputs_wrapper(double Vref,
								double V_cb,
								double It,
								double I_SP,
								double *D,
								double *uk_V,
								double *uk_I)
{
/* Output_BEGIN */
//outer loop
PIVarV.ek_v = Vref - V_cb;
PIVarV.esk_1_v = PIVarV.ek_1_v + (1/PIVarV.Kpv)*(PIVarV.usk_1_v-PIVarV.uk_1_v);
PIVarV.uk_v = PIVarV.usk_1_v +PIVarV.Kpv*(PIVarV.ek_v-PIVarV.Dv*PIVarV.esk_1_v);

if(PIVarV.uk_v > PIVarI.iMax_i)
{
	I_SP = PIVarI.iMax_i;
}

else if(PIVarV.uk_v < PIVarI.iMin_i)
{
	 I_SP = PIVarI.iMin_i;
}

else
{
	 I_SP = PIVarV.uk_v;
	    *uk_V = PIVarV.uk_v;
}

//update data
PIVarV.ek_1_v = PIVarV.ek_v;
PIVarV.uk_1_v = PIVarV.uk_v;
PIVarV.usk_1_v=I_SP;
/* Includes_BEGIN */

//inner loop
PIVarI.ek_i = I_SP - It/2;
PIVarI.esk_1_i = PIVarI.ek_1_i + (1/PIVarI.Kpi)*(PIVarI.usk_1_i-PIVarI.uk_1_i);
PIVarI.uk_i = PIVarI.usk_1_i +PIVarV.Kpv*(PIVarI.ek_i-PIVarI.Di*PIVarI.esk_1_i);

    *D = PIVarI.uk_i;
    *uk_I = PIVarI.uk_i;

//update data
PIVarI.ek_1_i = PIVarI.ek_i;
PIVarI.uk_1_i = PIVarI.uk_i;
PIVarI.usk_1_i=*D;
/* Output_END */
}
void PI_IBM_2P_Outputs_wrapper_vip(double Vref,
								double V_cb,
								double It,
								double I_SP,
								double *D,
								double *uk_V,
								double *uk_I)
{
/* Output_BEGIN */
//outer loop
	adc_get_value();
	V_cb = vcbsense;
PIVarV.ek_v = Vref - V_cb;
PIVarV.esk_1_v = PIVarV.ek_1_v + (1/PIVarV.Kpv)*(PIVarV.usk_1_v-PIVarV.uk_1_v);
PIVarV.uk_v = PIVarV.usk_1_v +PIVarV.Kpv*(PIVarV.ek_v-PIVarV.Dv*PIVarV.esk_1_v);

if(PIVarV.uk_v > PIVarV.uMax_v)
{
	I_SP = PIVarV.uMax_v;
}

else if(PIVarV.uk_v < PIVarV.uMin_v)
{
	 I_SP = PIVarV.uMin_v;
}

else
{
	 I_SP = PIVarV.uk_v;
	    *uk_V = PIVarV.uk_v;
}

//update data
PIVarV.ek_1_v = PIVarV.ek_v;
PIVarV.uk_1_v = PIVarV.uk_v;
PIVarV.usk_1_v=I_SP;
/* Includes_BEGIN */

//inner loop
adc_get_value();
It = isense;
PIVarI.ek_i = I_SP - It/2;
PIVarI.esk_1_i = PIVarI.ek_1_i + (1/PIVarI.Kpi)*(PIVarI.usk_1_i-PIVarI.uk_1_i);
PIVarI.uk_i = PIVarI.usk_1_i +PIVarV.Kpv*(PIVarI.ek_i-PIVarI.Di*PIVarI.esk_1_i);

if(PIVarI.uk_i > PIVarI.iMax_i)
{

	    *D = PIVarI.iMax_i;
}

else if(PIVarI.uk_i < PIVarI.iMin_i)
{
	    *D= PIVarI.iMin_i;
}

else
{
    *D = PIVarI.uk_i;
    *uk_I = PIVarI.uk_i;
}

//update data
PIVarI.ek_1_i = PIVarI.ek_i;
PIVarI.uk_1_i = PIVarI.uk_i;
PIVarI.usk_1_i=*D;
/* Output_END */
}
void PI_IBM_2P_Terminate_wrapper(void)
{
/* Terminate_BEGIN */
/*
 * Custom Terminate code goes here.
 */
/* Terminate_END */
}
