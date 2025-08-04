/*
 * winform.h
 *
 *  Created on: Jun 21, 2025
 *      Author: Nhan
 */

#ifndef INC_WINFORM_H_
#define INC_WINFORM_H_

#include"stm32h7xx_hal.h"
void send_data_to_winform(double isense, double voutsense, double vinsense, double vcbsense);
#endif /* INC_WINFORM_H_ */
