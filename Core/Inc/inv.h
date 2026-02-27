/*
 * inv.h
 *
 *  Created on: Nov 29, 2024
 *      Author: HALAB_G
 */

#ifndef INC_INV_H_
#define INC_INV_H_

#include "main.h"
#include "variable.h"

struct INVERTER {
	 // Parameters
	    float Rs;
	    float Ractive;
	    float Ld, Lq, Ls;
	    float Lamf;
	    float PP, INV_PP;
	    float Kt, INV_Kt;
	    float Jm, INV_Jm, Bm;
	    float Is_rated;
	    float Is_limit, Te_limit;
	    float Wrpm_rated, Te_rated;

	    // Sensing values
	    float Vdc, Vdc_control, INV_Vdc, Vdc_sense;
	    float Ia, Ib, Ic;
	    float Ia_debug, Ib_debug, Ic_debug, Vdc_debug; // ADC_injected_test
	    uint32_t ADC_A_debug, ADC_B_debug, ADC_C_debug, ADC_Vdc_debug; // ADC_injected_test
	    float Te, Idc;

	    // Speed controller
	    float Wsc, zeta_sc;
	    float Kp_sc, Ki_sc, Ka_sc;
	    float Ki_scale; // KMH

	    float Wr, Wrm, Wrpm;
	    float Wrm_ref_set, Wrpm_ref_set, Wrpm_ref_set_old;
	    float Wrm_ref, Wrpm_ref;
	    float dWrm;

	    float Wrm_err, Te_ref_integ, Te_ref_ff, Te_ref_unsat, Te_ref, Te_ref_aw;

	    // Flux Weakening Controller
	    float Kp_fw, Ki_fw, Ka_fw;

	    float Vmag_err, Idsr_ref_integ, Idsr_ref_ff, Idsr_ref_unsat, Idsr_ref, Idsr_ref_aw;
	    float Idsr_ref_set, Iqsr_ref_set;  //GCL
	    float Idsr_ref_fw;                 //GCL
	    float Is_ref;                      //GCL

	    // Torque Controller
	    float Te_ref_set;
	    float dTe;
	    float Iqsr_ref_unsat;
	    float Iqsr_max;
	    float Iqsr_ref;
	    float Te_ref_sat;
	    float Te_calc;
	    float Te_max;

	    // Current Controller
	    float Wcc;
	    float Kpd_cc, Kpq_cc;
	    float Kid_cc, Kiq_cc;
	    float Kad_cc, Kaq_cc;

	    float Idss, Iqss;
	    float Idsr, Iqsr;
	    float Idsr_err, Iqsr_err;
	    float Vdsr_ref_integ, Vqsr_ref_integ;
	    float Vdsr_ref_ff, Vqsr_ref_ff;
	    float Vdsr_ref_unsat, Vqsr_ref_unsat;
	    float alpha_LPF;
	    float Vdsr_ref_unsat_old, Vqsr_ref_unsat_old;
	    float Vdsr_ref, Vqsr_ref, Vdsr_ref_old, Vqsr_ref_old, Vmag_ref;
	    float Vdsr_ref_aw, Vqsr_ref_aw;
	    float Vdss_ref, Vqss_ref;
	    float Vas_ref, Vbs_ref, Vcs_ref;
	    float Vmax, Vmin, Voffset;
	    float Van_ref, Vbn_ref, Vcn_ref;
	    float Duty_A, Duty_B, Duty_C;

	    // Angle
	    float Thetar, Thetar_adv;
	    float Thetarm;
	    float Thetar_offset;
	    float cosThetar, sinThetar;
	    float cosThetar_adv, sinThetar_adv;
	    float Thetar_err;

	    // Open-loop control
	    float Idsr_ref_OLC;
	    float Iqsr_ref_OLC;
	    float Idsr_ref_set_OLC;
	    float Idsr_slope_OLC;



	    float Vdsr_ref_OLC;
	    float Vqsr_ref_OLC;
	    float Thetar_OLC, Thetar_OLC_buffer;
	    float Wrpm_ref_OLC, Wr_ref_OLC;
	    float Wrpm_ref_set_OLC;
	    float Wrpm_slope_OLC;

	    float Vdsr_ref_set;
	    float dV;

	    // Align
	    float Idsr_align;
	    int init_align_done;

	    // Nonlinearity compensation
	    float Idsr_NLC;
	    float Vdsr_NLC_array[41];
	    float Idss_ref, Iqss_ref;
	    float Ia_ref, Ib_ref, Ic_ref;
	    float A_NLC, B_NLC, C_NLC;
	    float Va_NLC, Vb_NLC, Vc_NLC;

	    // Duty test
	    float TestDuty_A, TestDuty_B, TestDuty_C;
	    float Kp_sc_ratio, Ki_sc_ratio;
	    float MTPA_Te_gap, MTPA_Te_max, MTPA_Te_gap_INV;

	    // Ld Estimation
	    float Vmag_inj;

	    float integ_Thetar_PLL;
	    float Kp_PLL, Ki_PLL;
	    float W_PLL;

	    // Hall Sensor
	    GPIO_PinState hall_a;
	    GPIO_PinState hall_b;
	    GPIO_PinState hall_c;
	    uint32_t hallSensorValue;
	    uint8_t hall_state, hall_state_old;
	    int hall_fault;
	    int duty_state;

	    // PLL
	    float Thetar_Hall;
	    float Thetar_Hall_PLL, Thetar_est_Hall, Thetar_err_Hall; // for Speed PLL
	    float W_PLL_HF, Kp_PLL_HF, Ki_PLL_HF, integ_PLL_HF;
	    float integ_PLL_ENC;
	    float Wr_est, Wr_est_f, Wr_est_ENC, Wr_est_Hall;

	    float W_PLL_Hall, Kp_PLL_Hall, Ki_PLL_Hall, integ_PLL_Hall;

	    // Hall Position Test
	    float Duty_Test;

	    float Wrpm_ref_cmd;


};
extern struct INVERTER INV;
extern struct CONTROL_FLAG FLAG;

extern int Mode_align;
extern float Time_align;

extern int Theta_mode;

///Extended EMF sensorless
extern float W_SPD_PLL, Kp_SPD_PLL, Ki_SPD_PLL, integ_Thetar_PLL;

void PwmSwOn();
void PwmSwOff();
void PwmDutyUpt();
void InitParameter(struct INVERTER *INV, float Rs, float Ld, float Lq, float Lamf, float PP, float Jm, float Bm, float Idsr_align, float Is_rated,float Is_limit,float Wrpm_rated,float Te_rated);
void Init_Spd_PLL(struct INVERTER* INV, float Ws);
//void GenerateTorqueReference(struct INVERTER*);
void TorqueControl(struct INVERTER *INV);
void CurrentControl(struct INVERTER*);
void CurrentControl_OLC(struct INVERTER *INV);
void SpeedControl(struct INVERTER *INV);

void InitSpeedController(struct INVERTER *INV, float Wsc, float zeta);
void InitFluxWeakeningController(struct INVERTER *INV, float Kp_fw, float Ki_fw);
void InitTorqueController(struct INVERTER *INV);
void InitCurrentController(struct INVERTER *INV, float Wcc);
void ResetController(struct INVERTER *INV);
void UpdateController(struct INVERTER *INV);

void Hallsensor_Observer(struct INVERTER *INV);
uint8_t GetHallSensorState(GPIO_PinState hall_sensor_1, GPIO_PinState hall_sensor_2, GPIO_PinState hall_sensor_3);

void OpenLoopControl(struct INVERTER*);
void VoltageOpenLoopControl(struct INVERTER *INV);
void Vref_GenControl(struct INVERTER *INV);
void Align(struct INVERTER *INV);
#endif /* INC_INV_H_ */
