/*
 * fault.h
 *
 *  Created on: Dec 6, 2024
 *      Author: HALAB_G
 */

#ifndef INC_FAULT_H_
#define INC_FAULT_H_


struct FAULT_VAL {
    float Vdc;
    float Idc;
    float Ia, Ib, Ic;
    float Wrpm;
};

extern struct FAULT_VAL FLTVAL;

extern void HardWareFault(void);
extern void SoftWareFault(void);

#endif /* INC_FAULT_H_ */
