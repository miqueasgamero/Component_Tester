/*
 * measurements.h
 *
 *  Created on: Jun 7, 2023
 *      Author: mique
 */

#ifndef INC_MEASUREMENTS_H_
#define INC_MEASUREMENTS_H_

#include "main.h"
#include "config.h"
#include <math.h> // Para trabajar con float

float get_resistance(void);
void change_known_resistance(float Res);
float get_capacitance(void);


#endif /* INC_MEASUREMENTS_H_ */
