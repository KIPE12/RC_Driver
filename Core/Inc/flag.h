/*
 * flag.h
 *
 *  Created on: Dec 6, 2024
 *      Author: HALAB_G
 */

#ifndef INC_FLAG_H_
#define INC_FLAG_H_

#include "main.h"

struct CONTROL_FLAG {
    uint16_t READY;
    uint16_t FAULT;
    uint16_t FAULT_CLEAR;
    uint16_t INV_RUN;
    uint16_t INV_OLC;
    uint16_t INV_VOLC;
    uint16_t INV_Vref_Gen;
    uint16_t INV_ALIGN;
    uint16_t INV_NLC;
    uint16_t TS_MODE;
    uint16_t TS_MODE_STOP;
    uint16_t DUTY_TEST;
    uint16_t INJECTION_TEST;
    uint16_t Param_Estimation;
    uint16_t HALL_POS_TEST;
};
extern struct CONTROL_FLAG FLAG;

#endif /* INC_FLAG_H_ */
