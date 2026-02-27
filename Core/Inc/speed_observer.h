/*
 * speed_observer.h
 *
 *  Created on: Feb 12, 2025
 *      Author: HALAB_G
 */

#ifndef INC_SPEED_OBSERVER_H_
#define INC_SPEED_OBSERVER_H_

#include "main.h"


struct SPEED_OBSERVER {

    float Thetarm_est, Thetar_est;
    float Wrm_est, Wrm_est_fb, integ_Wrm_est;
    float Wr_est;

    float Wso;
    float l1, l2, l3;
    float K1, K2, K3;

    float Thetarm_err, Wrm_err;
    float Te_est, Tl_est, Te_ff, integ_double;
    float Tload_est;

    float INV_PP, PP;
    float Lamf_hat, Ld_hat, Lq_hat;
    float Jm_hat, Bm_hat, INV_Jm_hat;

};

struct EXT_Sensorless {
    float Rs_hat, Ld_hat, Lq_hat, INV_Ld_hat;
    float Wec, Kpd_EXT, Kid_EXT, Kpq_EXT, Kiq_EXT;
    float Vdss_ref_EXT, Vqss_ref_EXT;
    float Vdss_ref_old, Vqss_ref_old;
    float Err_Thetar_EXT, Wr_EXT, Thetar_EXT, Thetar_EXT_old, Sin_Thetar_EXT, Cos_Thetar_EXT;
    float Thetar_EXT_PLL_input;
    float k_debug;
    float Thetarm_EXT, Thetarm_EXT_old, Sin_Thetarm_EXT, Cos_Thetarm_EXT;
    float Wr_EXT_f, Wrm_EXT_f, Wrpm_EXT_f;
    float Lamf_est;

    float Vdse_ref_EXT, Vqse_ref_EXT;
    float Idse_EXT, Iqse_EXT;
    float Err_Idse_EXT, Err_Iqse_EXT;
    float Idse_EXT_est, Iqse_EXT_est;
    float integ_Idse_EXT_est, integ_Iqse_EXT_est;
    float EEMFd_est, EEMFq_est, EEMF_mag;
    float Vdse_FF_EXT, Vqse_FF_EXT;

    float W_LPF, a_LPF;
};


extern struct EXT_Sensorless EXT_1;

void InitSpeedObserver_(struct SPEED_OBSERVER* SPDOBS, float Beta, float PP, float Ld, float Lq, float Lamf, float Jm, float Bm);
void SpeedObserver_4_34(struct SPEED_OBSERVER* SPDOBS, float Err_Thetar, float Idse_ff, float Iqse_ff);
void SpeedObserver_4_35(struct SPEED_OBSERVER* SPDOBS, float Err_Thetar, float Idse_ff, float Iqse_ff);
void initExtended_Sensorless_Synchronous_Frame(struct EXT_Sensorless* EXT, float Wc, float Rs, float Ld, float Lq);
void EXT_SS_Sync(struct EXT_Sensorless* EXT, float Vdss, float Vqss, float Idss, float Iqss);


#endif /* INC_SPEED_OBSERVER_H_ */
