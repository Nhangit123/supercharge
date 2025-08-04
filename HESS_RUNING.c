#include <math.h>

// Structure Definitions (unchanged from original)
typedef struct {
    double RL, L, C, R, r_ds, r_f, V_f, Ilm, Vcbm, Vbat, Vcap, Vinic, tau;
    int n, m;
} Interleave;

typedef struct {
    double Db, Ds, I, IL, t, td;
} Energy;

typedef struct {
    double Fsw, Td, Tfi;
} Conv;

typedef struct {
    double tau, ek_lp, ek_1_lp, uk_lp, uk_1_lp, T;
} Lowpass;

typedef struct {
    double Kpv, Ksv, Kiv, Tiv, Dv;
    double uk_1_v, uk_v, usk_1_v;
    double ek_v, ek_1_v, esk_v, esk_1_v;
    double uMax_v, uMin_v;
    double Tsv;
} PI_V;

typedef struct {
    double Kpb, Ksb, Kib, Tib, Db;
    double uk_1_b, uk_b, usk_1_b;
    double ek_b, ek_1_b, esk_b, esk_1_b;
    double iMax_b, iMin_b;
    double Tsb;
} PI_I;

// Global Variables
static Interleave iParas;
static Energy Eng;
static Conv convParas;
static Lowpass LP;
static PI_V PIVarV;
static PI_I PIVarIb;
static PI_I PIVarIc;
static int count = 0;

// Initialization Function
void PI_IBM_2P_Init(void) {
    // Hardware peripherals initialization would go here
    // For example: PWM, ADC, GPIO, etc.
    
    // Interleave parameters
    iParas.RL = 0.1;
    iParas.C = 1e-3;  // Replaced pow(10,-3) with 1e-3
    iParas.L = 100e-6; // Replaced 100*pow(10,-6)
    iParas.r_ds = 7.5e-3;
    iParas.r_f = 6.3e-3;
    iParas.V_f = 0.7;
    iParas.Ilm = 2;
    iParas.Vcbm = 30;
    
    // Switching frequency
    convParas.Fsw = 200000;
    convParas.Td = 1.0/convParas.Fsw;
    convParas.Tfi = 1.0/(5*convParas.Fsw);
    
    // PI voltage controller
    PIVarV.Tsv = 10.0/(convParas.Fsw*3);
    PIVarV.Kpv = 0.0863;
    PIVarV.Kiv = 74.5;
    PIVarV.Tiv = 1.0/PIVarV.Kiv;
    PIVarV.Dv = 1-PIVarV.Tsv/PIVarV.Tiv;
    PIVarV.uk_1_v = 0; PIVarV.uk_v = 0; PIVarV.usk_1_v = 0;
    PIVarV.ek_v = 0; PIVarV.ek_1_v = 0; PIVarV.esk_v = 0; PIVarV.esk_1_v = 0;
    PIVarV.uMax_v = iParas.Vcbm; PIVarV.uMin_v = -PIVarV.uMax_v;
    
    // PI battery current controller
    PIVarIb.Tsb = 10.0/convParas.Fsw;
    PIVarIb.Kpb = 1.33;
    PIVarIb.Kib = 10474;
    PIVarIb.Tib = 1.0/PIVarIb.Kib;
    PIVarIb.Db = 1-PIVarIb.Tsb/PIVarIb.Tib;
    PIVarIb.uk_1_b = 0; PIVarIb.uk_b = 0; PIVarIb.usk_1_b = 0;
    PIVarIb.ek_b = 0; PIVarIb.ek_1_b = 0; PIVarIb.esk_b = 0; PIVarIb.esk_1_b = 0;
    PIVarIb.iMax_b = iParas.Ilm; PIVarIb.iMin_b = -PIVarIb.iMax_b;
    
    // PI capacitor current controller
    PIVarIc.Tsc = 10.0/convParas.Fsw;
    PIVarIc.Kpc = 1.33;
    PIVarIc.Kic = 10474;
    PIVarIc.Tic = 1.0/PIVarIc.Kic;
    PIVarIc.Dc = 1-PIVarIc.Tsc/PIVarIc.Tic;
    PIVarIc.uk_1_c = 0; PIVarIc.uk_c = 0; PIVarIc.usk_1_c = 0;
    PIVarIc.ek_c = 0; PIVarIc.ek_1_c = 0; PIVarIc.esk_c = 0; PIVarIc.esk_1_c = 0;
    PIVarIc.iMax_c = iParas.Ilm; PIVarIc.iMin_c = -PIVarIc.iMax_c;
    
    // Low pass filter
    LP.tau = 10000;
    LP.T = 10e-3;  // Replaced 10*pow(10,-3)
    LP.ek_lp = 0; LP.ek_1_lp = 0;
    LP.uk_lp = 0; LP.uk_1_lp = 0;
}

// Main Control Function
void PI_IBM_2P_Control(double Vref, double V_cb, double Ib, double Ic, 
                      double Vb, double Vc, double I_cb,
                      double* I_T, double* I_SPC, double* I_SPb,
                      double* DB, double* DC, double* Ilp,
                      double* uk_V, double* uk_Ib, double* uk_Ic) {
    
    // Voltage PI Controller
    PIVarV.ek_v = Vref - V_cb; //error calculation
    PIVarV.esk_1_v = PIVarV.ek_1_v + (1.0/PIVarV.Kpv)*(PIVarV.usk_1_v-PIVarV.uk_1_v); //integral
    PIVarV.uk_v = PIVarV.usk_1_v + PIVarV.Kpv*(PIVarV.ek_v-PIVarV.Dv*PIVarV.esk_1_v); //pi output
    
    // limit output of voltage pi controller
    if(PIVarV.uk_v > PIVarV.uMax_v) PIVarV.uk_v = PIVarV.uMax_v;
    if(PIVarV.uk_v < PIVarV.uMin_v) PIVarV.uk_v = PIVarV.uMin_v;
    
    *I_T = PIVarV.uk_v; //total current require from source
    *uk_V = PIVarV.uk_v;
    
    // Update current states to previous state
    PIVarV.ek_1_v = PIVarV.ek_v;
    PIVarV.uk_1_v = PIVarV.uk_v;
    PIVarV.usk_1_v = *I_T;
    
    // Low-pass filter
    double a = (LP.T-2*LP.tau)/(LP.T+2*LP.tau);
    double b = LP.T/(1+2*LP.tau);
    LP.ek_lp = *I_T;
    LP.uk_lp = -LP.uk_1_lp*a + b*LP.ek_lp + b*LP.ek_1_lp;
    *Ilp = LP.uk_lp; //output of filter (low-frequent) require from battery
    
    // Update filter states
    LP.ek_1_lp = LP.ek_lp;
    LP.uk_1_lp = LP.uk_lp;
    
    // Limit battery current max = 2A
    double Ib1 = (*Ilp > PIVarIb.iMax_b) ? PIVarIb.iMax_b : *Ilp;
    
    // Rate limited update (every 100 calls) prevent rapid change in battery
    count++;
    if(count >= 100) {
        *I_SPb = Ib1;
        count = 0;
    }
    
    // Capacitor current reference
    *I_SPC = (*I_T - *I_SPb) * Vb / Vc;
    
    // Battery current PI controller
    PIVarIb.ek_b = *I_SPb - Ib/2.0;
    PIVarIb.esk_1_b = PIVarIb.ek_1_b + (1.0/PIVarIb.Kpb)*(PIVarIb.usk_1_b-PIVarIb.uk_1_b);
    PIVarIb.uk_b = PIVarIb.usk_1_b + PIVarIb.Kpb*(PIVarIb.ek_b-PIVarIb.Db*PIVarIb.esk_1_b); //output is duty
    
    // Clamp duty cycle to [0,1]
    if(PIVarIb.uk_b > 1.0) PIVarIb.uk_b = 1.0;
    if(PIVarIb.uk_b < 0.0) PIVarIb.uk_b = 0.0;
    
    *DB = PIVarIb.uk_b;
    *uk_Ib = PIVarIb.uk_b;
    
    // Update states
    PIVarIb.ek_1_b = PIVarIb.ek_b;
    PIVarIb.uk_1_b = PIVarIb.uk_b;
    PIVarIb.usk_1_b = *DB;
    
    // Capacitor current PI controller
    PIVarIc.ek_c = *I_SPC - Ic/2.0;
    PIVarIc.esk_1_c = PIVarIc.ek_1_c + (1.0/PIVarIc.Kpc)*(PIVarIc.usk_1_c-PIVarIc.uk_1_c);
    PIVarIc.uk_c = PIVarIc.usk_1_c + PIVarIc.Kpc*(PIVarIc.ek_c-PIVarIc.Dc*PIVarIc.esk_1_c);
    
    // Clamp duty cycle to [0,1]
    if(PIVarIc.uk_c > 1.0) PIVarIc.uk_c = 1.0;
    if(PIVarIc.uk_c < 0.0) PIVarIc.uk_c = 0.0;
    
    *DC = PIVarIc.uk_c;
    *uk_Ic = PIVarIc.uk_c;
    
    // Update states
    PIVarIc.ek_1_c = PIVarIc.ek_c;
    PIVarIc.uk_1_c = PIVarIc.uk_c;
    PIVarIc.usk_1_c = *DC;
}
