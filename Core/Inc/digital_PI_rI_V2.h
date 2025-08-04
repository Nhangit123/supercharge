/*
 * digital_PI_rI_V2.h
 *
 *  Created on: May 16, 2025
 *      Author: Nhan
 */

#ifndef INC_DIGITAL_PI_RI_V2_H_
#define INC_DIGITAL_PI_RI_V2_H_
#include <math.h>
#include "stdio.h"
#include "adc.h"
extern double isense,voutsense,vinsense,vcbsense;
void PI_IBM_2P_Start_wrapper();
void PI_IBM_2P_Outputs_wrapper(double Vref,
								double V_cb,
								double It,
								double I_SP,
								double *D,
								double *uk_V,
								double *uk_I);
void PI_IBM_2P_Outputs_wrapper_vip(double Vref,
									double V_cb,
									double It,
									double I_SP,
									double *D,
									double *uk_V,
									double *uk_I);
void PI_IBM_2P_Terminate_wrapper();


#endif /* INC_DIGITAL_PI_RI_V2_H_ */
