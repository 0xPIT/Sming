/*
 * WiringFrameworkDependencies.h
 *
 *  Created on: 28 џэт. 2015 у.
 *      Author: Anakonda
 */

#ifndef WIRING_WIRINGFRAMEWORKDEPENDENCIES_H_
#define WIRING_WIRINGFRAMEWORKDEPENDENCIES_H_

#include "../include/user_config.h"

//#include <c_types.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

#define F_CPU 80000000L ////?

#include "WConstants.h"
#include "BitManipulations.h"
#include "FakePgmSpace.h"
#include "../SmingCore/pins_arduino.h"


typedef enum {
    GPIO_PIN_INTR_DISABLE = 0,
    GPIO_PIN_INTR_POSEDGE = 1,
    GPIO_PIN_INTR_NEGEDGE = 2,
    GPIO_PIN_INTR_ANYEGDE = 3,
    GPIO_PIN_INTR_LOLEVEL = 4,
    GPIO_PIN_INTR_HILEVEL = 5
} GPIO_INT_TYPE;

#endif /* WIRING_WIRINGFRAMEWORKDEPENDENCIES_H_ */
