/*
 * staring.h
 *
 *  Created on: Aug 2, 2025
 *      Author: Nhan
 */

#ifndef INC_STARING_H_
#define INC_STARING_H_


#include <math.h>

typedef struct {
    double RL;     // Ohm of inductor
    double L;      // inductor (H)
    double C;      // capacitor (F)
    double R;      // Load resistance (Ω)
    double r_ds;   // IGBT resistance (Ω)
    double r_f;    // Diode resistance (Ω)
    double V_f;    // Diode voltage drop (V)
    double Imax;   // Max current in inductor (A)
    double Vmax;   // Max voltage in Bankcap (V)
    double Vbat;   // Battery voltage (V)
    double Vcap;   // Voltage of one capacitor (V)
    int n;         // Series capacitors
    int m;         // Parallel capacitors
    double Vinic;  // Initial capacitor voltage (V)
    double tau;    // Low-pass filter frequency
    int count;
} Interleave;

typedef struct {
    double Db;     // Battery duty cycle
    double Ds;     // Supercapacitor duty cycle
    double I;      // Load current (A)
    double IL;     // Inductor current (A)
    double t;      // Time constant
    double td;     // Damping time constant
} Energy;

typedef struct {
    double Fsw;    // Switching frequency (Hz)
    double Td;     // Equivalent time constant of converter
    double Tfi;    // Time constant of sensors
} Conv;

typedef struct {
    double tau;    // Low-pass filter frequency
    double ek_lp, ek_1_lp;
    double uk_lp, uk_1_lp;
    double T;      // Sampling time
} Lowpass;

typedef struct {
    double Kpv, Ksv, Kiv, Tiv, Dv;
    double uk_1_v, uk_v, usk_1_v;
    double ek_v, ek_1_v, esk_v, esk_1_v;
    double uMax_v, uMin_v;
    double Tsv;    // Second sampling time
} PI_V;

typedef struct {
    double Kpb, Ksb, Kib, Tib, Db;
    double uk_1_b, uk_b, usk_1_b;
    double ek_b, ek_1_b, esk_b, esk_1_b;
    double iMax_b, iMin_b;
    double Tsb;    // Second sampling time
} PI_Ib;

typedef struct {
    double Kpc, Ksc, Kic, Tic, Dc;
    double uk_1_c, uk_c, usk_1_c;
    double ek_c, ek_1_c, esk_c, esk_1_c;
    double iMax_c, iMin_c;
    double Tsc;    // Second sampling time
} PI_Ic;

typedef struct {
    double bf1;
    double bf2;
    double bf3;
    int count;
    int v;        // Delay v sampling time
} Ratel;
typedef enum {
    TU_0_DEN_12V,
    TU_12_DEN_24V,
	TU_24V_DEN_XV
} SystemState;

// Function prototypes
void Control_Init(void);
void Control_Update(double V_CB, double V_Bat, double Vref, double Ib,double Ic,
                    double* D,double* Dc, double* uk_V, double* uk_Ib,double* uk_Ic,
                   double* I_SPb,double* I_SPc, double* Count, double* bf1,
                   double* bf2, double* bf3);

#endif /* INC_STARING_H_ */
