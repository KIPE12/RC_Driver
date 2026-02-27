/**
 * @file    speed_observer.c
 * @brief   속도 옵저버 및 확장 역기전력(EEMF) 기반 센서리스 제어 모듈
 *
 * PMSM 제어를 위한 두 가지 속도/위치 추정 방법을 제공한다.
 *
 * @details 옵저버 종류:
 * | 옵저버                                | 함수                               | 특징                            |
 * |---------------------------------------|------------------------------------|---------------------------------|
 * | 전체차원 속도 옵저버 (4-34 방식)     | SpeedObserver_4_34()               | 순수 적분 출력, 위치에 K1 보정  |
 * | 전체차원 속도 옵저버 (4-35 방식)     | SpeedObserver_4_35()               | 출력에 K1 보정 포함             |
 * | 확장 EEMF 센서리스 옵저버            | EXT_SS_Sync()                      | 동기 좌표계 전류 모델 기반      |
 *
 * @details 옵저버 공통 게인 설계 (Butterworth 극 배치):
 * - l1 = -2×Wso - Bm/Jm
 * - l2 = 2×Wso² - l1×Bm/Jm
 * - l3 = Wso³ × Jm
 *
 * @details EEMF 센서리스 동작 원리:
 * 추정 전기각으로 정지→동기 좌표 변환 후 전류 모델 오차로부터
 * 확장 역기전력(EEMFd, EEMFq)을 추정하고, atan2(-EEMFd, EEMFq)로
 * 위치 오차를 계산한다.
 *
 * @author  HALAB_G
 * @date    2025-02-12
 */

#include "speed_observer.h"
#include "math.h"
#include "variable.h"

struct EXT_Sensorless EXT_1; ///< 확장 센서리스 옵저버 전역 구조체

/* ======================================================================
 * 속도 옵저버 초기화 및 실행
 * ====================================================================== */

/**
 * @brief 전체차원 속도 옵저버를 초기화한다.
 *
 * Butterworth 극 배치를 사용하여 옵저버 게인(l1, l2, l3)을 계산한다.
 * 옵저버는 위치 오차를 입력으로 기계적 각속도와 부하 토크를 추정한다.
 *
 * @details Butterworth 3극 배치 공식:
 * - l1 = -2×Wso - Bm/Jm
 * - l2 = 2×Wso² - l1×Bm/Jm
 * - l3 = Wso³ × Jm
 * - K1 = l1, K2 = Jm×l2, K3 = -l3
 *
 * @param[out] SPDOBS  속도 옵저버 구조체 포인터
 * @param[in]  Beta    옵저버 대역폭 [rad/s]
 * @param[in]  PP      극쌍수 [-]
 * @param[in]  Ld      d축 인덕턴스 [H]
 * @param[in]  Lq      q축 인덕턴스 [H]
 * @param[in]  Lamf    영구자석 자속 [Wb]
 * @param[in]  Jm      관성 모멘트 [kg·m²]
 * @param[in]  Bm      점성 마찰 계수 [N·m·s/rad]
 */
void InitSpeedObserver_(struct SPEED_OBSERVER *SPDOBS,
                         float Beta, float PP,
                         float Ld, float Lq, float Lamf,
                         float Jm, float Bm)
{
    /* 상태 변수 초기화 */
    SPDOBS->Thetarm_est   = 0.f;
    SPDOBS->Thetar_est    = 0.f;
    SPDOBS->Wrm_est       = 0.f;
    SPDOBS->Wr_est        = 0.f;
    SPDOBS->Wrm_est_fb    = 0.f;

    /* 전동기 파라미터 저장 */
    SPDOBS->PP        = PP;
    SPDOBS->INV_PP    = 1.f / PP;
    SPDOBS->Ld_hat    = Ld;
    SPDOBS->Lq_hat    = Lq;
    SPDOBS->Lamf_hat  = Lamf;
    SPDOBS->Jm_hat    = Jm;
    SPDOBS->Bm_hat    = Bm;
    SPDOBS->INV_Jm_hat = 1.f / Jm;

    /* Butterworth 극 배치 게인 계산 */
    SPDOBS->Wso = -Beta;
    SPDOBS->l1  = -2.f * SPDOBS->Wso - SPDOBS->Bm_hat * SPDOBS->INV_Jm_hat;
    SPDOBS->l2  =  2.f * SPDOBS->Wso * SPDOBS->Wso - SPDOBS->l1 * SPDOBS->Bm_hat * SPDOBS->INV_Jm_hat;
    SPDOBS->l3  =  SPDOBS->Wso * SPDOBS->Wso * SPDOBS->Wso * SPDOBS->Jm_hat;

    SPDOBS->K1 =  SPDOBS->l1;
    SPDOBS->K2 =  SPDOBS->Jm_hat * SPDOBS->l2;
    SPDOBS->K3 = -SPDOBS->l3;

    /* 오차 및 추정 토크 초기화 */
    SPDOBS->Thetarm_err   = 0.f;
    SPDOBS->Wrm_err       = 0.f;
    SPDOBS->Te_est        = 0.f;
    SPDOBS->Tl_est        = 0.f;
    SPDOBS->Te_ff         = 0.f;
    SPDOBS->integ_double  = 0.f;
    SPDOBS->integ_Wrm_est = 0.f;
    SPDOBS->Tload_est     = 0.f;
}

/**
 * @brief 속도 옵저버 실행 (수식 4-34 방식)
 *
 * 위치 오차를 입력으로 하여 전자 토크 및 부하 토크를 추정하고,
 * 적분을 통해 기계적 각속도와 위치를 갱신한다.
 *
 * @details 갱신 순서:
 * 1. Te_est = K2 × Thetarm_err
 * 2. Tl_est += K3 × Thetarm_err × Tsamp (적분)
 * 3. Te_ff = 1.5 × PP × (λf × Iq + (Ld-Lq) × Id × Iq)
 * 4. integ_Wrm_est += (Te_est + Te_ff + Tl_est - Bm × Wrm_est_fb) / Jm × Tsamp
 * 5. Wrm_est = integ_Wrm_est (4-34: 피드백은 순수 적분값)
 * 6. Thetarm_est += (Wrm_est + K1 × Thetarm_err) × Tsamp
 *
 * @param[in,out] SPDOBS    속도 옵저버 구조체 포인터
 * @param[in]     Err_Thetar 전기각 위치 오차 [rad]
 * @param[in]     Idse_ff   피드포워드 d축 전류 [A]
 * @param[in]     Iqse_ff   피드포워드 q축 전류 [A]
 */
void SpeedObserver_4_34(struct SPEED_OBSERVER *SPDOBS,
                         float Err_Thetar, float Idse_ff, float Iqse_ff)
{
    SPDOBS->Thetarm_err = Err_Thetar * SPDOBS->INV_PP;

    SPDOBS->Te_est  = SPDOBS->K2 * SPDOBS->Thetarm_err;
    SPDOBS->Tl_est += SPDOBS->K3 * SPDOBS->Thetarm_err * Tsamp;
    SPDOBS->Te_ff   = 1.5f * SPDOBS->PP * (SPDOBS->Lamf_hat * Iqse_ff
                      + (SPDOBS->Ld_hat - SPDOBS->Lq_hat) * Idse_ff * Iqse_ff);

    SPDOBS->integ_Wrm_est += (SPDOBS->Te_est + SPDOBS->Te_ff + SPDOBS->Tl_est
                               - SPDOBS->Bm_hat * SPDOBS->Wrm_est_fb) * SPDOBS->INV_Jm_hat * Tsamp;
    SPDOBS->Wrm_est    = SPDOBS->integ_Wrm_est;
    SPDOBS->Wrm_est_fb = SPDOBS->integ_Wrm_est;
    SPDOBS->Wr_est     = SPDOBS->Wrm_est * SPDOBS->PP;

    SPDOBS->Thetarm_est += (SPDOBS->Wrm_est + SPDOBS->K1 * SPDOBS->Thetarm_err) * Tsamp;
    SPDOBS->Thetarm_est  = BOUND_PI(SPDOBS->Thetarm_est);
    SPDOBS->Thetar_est   = BOUND_PI(SPDOBS->PP * SPDOBS->Thetarm_est);

    SPDOBS->Tload_est = -SPDOBS->Tl_est;
}

/**
 * @brief 속도 옵저버 실행 (수식 4-35 방식)
 *
 * 4-34와 동일하지만 Wrm_est 출력에 K1 보정항이 포함된다.
 * (Wrm_est_fb는 순수 적분값, Wrm_est는 K1 보정 포함)
 *
 * @details 4-34와 다른 점:
 * - Wrm_est     = integ_Wrm_est + K1 × Thetarm_err (출력에 직접 보정)
 * - Wrm_est_fb  = integ_Wrm_est (피드백은 순수 적분값 유지)
 * - Thetarm_est += Wrm_est × Tsamp (K1항이 Wrm_est에 포함됨)
 *
 * @param[in,out] SPDOBS    속도 옵저버 구조체 포인터
 * @param[in]     Err_Thetar 전기각 위치 오차 [rad]
 * @param[in]     Idse_ff   피드포워드 d축 전류 [A]
 * @param[in]     Iqse_ff   피드포워드 q축 전류 [A]
 */
void SpeedObserver_4_35(struct SPEED_OBSERVER *SPDOBS,
                         float Err_Thetar, float Idse_ff, float Iqse_ff)
{
    SPDOBS->Thetarm_err = Err_Thetar * SPDOBS->INV_PP;

    SPDOBS->Te_est  = SPDOBS->K2 * SPDOBS->Thetarm_err;
    SPDOBS->Tl_est += SPDOBS->K3 * SPDOBS->Thetarm_err * Tsamp;
    SPDOBS->Te_ff   = 1.5f * SPDOBS->PP * (SPDOBS->Lamf_hat * Iqse_ff
                      + (SPDOBS->Ld_hat - SPDOBS->Lq_hat) * Idse_ff * Iqse_ff);

    SPDOBS->integ_Wrm_est += (SPDOBS->Te_est + SPDOBS->Te_ff + SPDOBS->Tl_est
                               - SPDOBS->Bm_hat * SPDOBS->Wrm_est_fb) * SPDOBS->INV_Jm_hat * Tsamp;
    SPDOBS->Wrm_est    = SPDOBS->integ_Wrm_est + SPDOBS->K1 * SPDOBS->Thetarm_err;
    SPDOBS->Wrm_est_fb = SPDOBS->integ_Wrm_est;
    SPDOBS->Wr_est     = SPDOBS->Wrm_est * SPDOBS->PP;

    SPDOBS->Thetarm_est += SPDOBS->Wrm_est * Tsamp;
    SPDOBS->Thetarm_est  = BOUND_PI(SPDOBS->Thetarm_est);
    SPDOBS->Thetar_est   = BOUND_PI(SPDOBS->PP * SPDOBS->Thetarm_est);

    SPDOBS->Tload_est = -SPDOBS->Tl_est;
}

/* ======================================================================
 * 확장 역기전력(EEMF) 기반 센서리스 옵저버
 * ====================================================================== */

/**
 * @brief 확장 동기 좌표계 센서리스 옵저버를 초기화한다.
 *
 * 전류 모델 기반의 확장 역기전력(EEMF) 추정기를 초기화하며,
 * 추정된 EEMF의 d/q성분 비율로 위치 오차를 계산한다.
 *
 * @details PI 게인 설정:
 * - Kpd = Ld × Wc, Kid = Rs × Wc
 * - Kpq = Ld × Wc, Kiq = Rs × Wc (내부 모델 방식)
 *
 * @param[out] EXT 확장 센서리스 구조체 포인터
 * @param[in]  Wc  옵저버 대역폭 [rad/s]
 * @param[in]  Rs  고정자 저항 [Ω]
 * @param[in]  Ld  d축 인덕턴스 [H]
 * @param[in]  Lq  q축 인덕턴스 [H]
 */
void initExtended_Sensorless_Synchronous_Frame(struct EXT_Sensorless *EXT,
                                                float Wc, float Rs, float Ld, float Lq)
{
    EXT->Rs_hat     = Rs;
    EXT->Ld_hat     = Ld;
    EXT->Lq_hat     = Lq;
    EXT->INV_Ld_hat = 1.f / Ld;
    EXT->k_debug    = 1.f;

    EXT->Wec      = 1.0f * Wc;
    EXT->Kpd_EXT  = EXT->Ld_hat * EXT->Wec;
    EXT->Kid_EXT  = EXT->Rs_hat * EXT->Wec;
    EXT->Kpq_EXT  = EXT->Ld_hat * EXT->Wec;
    EXT->Kiq_EXT  = EXT->Rs_hat * EXT->Wec;

    /* 전압/전류 상태 변수 초기화 */
    EXT->Vdss_ref_EXT = 0.f; EXT->Vqss_ref_EXT = 0.f;
    EXT->Vdss_ref_old = 0.f; EXT->Vqss_ref_old = 0.f;

    EXT->Err_Thetar_EXT = 0.f;
    EXT->Wr_EXT         = 0.f;
    EXT->Thetar_EXT     = 0.f;
    EXT->Thetar_EXT_old = 0.f;
    EXT->Sin_Thetar_EXT = 0.f;
    EXT->Cos_Thetar_EXT = 0.f;
    EXT->Wr_EXT_f       = 0.f;

    EXT->Thetarm_EXT     = 0.f;
    EXT->Thetarm_EXT_old = 0.f;
    EXT->Sin_Thetarm_EXT = 0.f;
    EXT->Cos_Thetarm_EXT = 0.f;

    EXT->Vdse_ref_EXT = 0.f; EXT->Vqse_ref_EXT = 0.f;
    EXT->Idse_EXT     = 0.f; EXT->Iqse_EXT     = 0.f;
    EXT->Err_Idse_EXT = 0.f; EXT->Err_Iqse_EXT = 0.f;
    EXT->Idse_EXT_est = 0.f; EXT->Iqse_EXT_est = 0.f;
    EXT->integ_Idse_EXT_est = 0.f;
    EXT->integ_Iqse_EXT_est = 0.f;
    EXT->EEMFd_est    = 0.f; EXT->EEMFq_est    = 0.f;
    EXT->Vdse_FF_EXT  = 0.f; EXT->Vqse_FF_EXT  = 0.f;
}

/**
 * @brief 확장 EEMF 센서리스 옵저버를 실행한다.
 *
 * 정지 좌표계 전압/전류를 추정 전기각으로 회전 변환하여
 * 동기 좌표계 전류 모델 기반 EEMF를 추정하고,
 * 위치 오차(Err_Thetar_EXT)를 출력한다.
 *
 * @details 처리 순서:
 * 1. 추정 각도로 정지→동기 좌표 변환 (Vdss→Vdse, Idss→Idse)
 * 2. 전류 오차 계산: Err_Id = Idse - Idse_est
 * 3. PI 관측기: EEMFd = -(Kp×Err_Id + ∫Ki×Err_Id dt)
 * 4. 피드포워드 전압: Vdse_FF = Vdse_ref + Wr×Lq×Iqse
 * 5. 전류 모델 적분: Idse_est += (Vdse_FF - EEMFd - Rs×Idse) / Ld × Ts
 * 6. 위치 오차: Err_Thetar = atan2(-EEMFd, EEMFq)
 *
 * @param[in,out] EXT  확장 센서리스 구조체 포인터
 * @param[in]     Vdss 정지 좌표계 α축 기준 전압 [V]
 * @param[in]     Vqss 정지 좌표계 β축 기준 전압 [V]
 * @param[in]     Idss 정지 좌표계 α축 전류 [A]
 * @param[in]     Iqss 정지 좌표계 β축 전류 [A]
 *
 * @note |EEMFq_est| < 1.0 이면 1.0으로 제한하여 atan2 발산을 방지한다.
 */
void EXT_SS_Sync(struct EXT_Sensorless *EXT,
                  float Vdss, float Vqss, float Idss, float Iqss)
{
    /* 게인 갱신 */
    EXT->Kpd_EXT = EXT->Ld_hat * EXT->Wec;
    EXT->Kid_EXT = EXT->Rs_hat * EXT->Wec;
    EXT->Kpq_EXT = EXT->Ld_hat * EXT->Wec;
    EXT->Kiq_EXT = EXT->Rs_hat * EXT->Wec;

    EXT->Sin_Thetar_EXT = sinf(EXT->Thetar_EXT);
    EXT->Cos_Thetar_EXT = cosf(EXT->Thetar_EXT);

    /* 전압 평균 (1/2 스텝 지연 보상) */
    EXT->Vdss_ref_EXT = (EXT->Vdss_ref_old + Vdss) * 0.5f;
    EXT->Vqss_ref_EXT = (EXT->Vqss_ref_old + Vqss) * 0.5f;
    EXT->Vdss_ref_old = Vdss;
    EXT->Vqss_ref_old = Vqss;

    /* 정지 → 동기 좌표 변환 */
    EXT->Vdse_ref_EXT = EXT->Vdss_ref_EXT *  EXT->Cos_Thetar_EXT + EXT->Vqss_ref_EXT * EXT->Sin_Thetar_EXT;
    EXT->Vqse_ref_EXT = EXT->Vdss_ref_EXT * (-EXT->Sin_Thetar_EXT) + EXT->Vqss_ref_EXT * EXT->Cos_Thetar_EXT;

    EXT->Idse_EXT = Idss *  EXT->Cos_Thetar_EXT + Iqss * EXT->Sin_Thetar_EXT;
    EXT->Iqse_EXT = Idss * (-EXT->Sin_Thetar_EXT) + Iqss * EXT->Cos_Thetar_EXT;

    /* PI 전류 관측기 */
    EXT->Err_Idse_EXT = EXT->Idse_EXT - EXT->Idse_EXT_est;
    EXT->Err_Iqse_EXT = EXT->Iqse_EXT - EXT->Iqse_EXT_est;

    EXT->integ_Idse_EXT_est += Tsamp * EXT->Kid_EXT * EXT->Err_Idse_EXT;
    EXT->integ_Iqse_EXT_est += Tsamp * EXT->Kiq_EXT * EXT->Err_Iqse_EXT;

    /* EEMF 추정 */
    EXT->EEMFd_est = -(EXT->Err_Idse_EXT * EXT->Kpd_EXT + EXT->integ_Idse_EXT_est);
    EXT->EEMFq_est = -(EXT->Err_Iqse_EXT * EXT->Kpq_EXT + EXT->integ_Iqse_EXT_est);

    /* 피드포워드 전압 (역기전력 분리) */
    EXT->Vdse_FF_EXT = EXT->Vdse_ref_EXT + EXT->Wr_EXT * EXT->Lq_hat * EXT->Iqse_EXT;
    EXT->Vqse_FF_EXT = EXT->Vqse_ref_EXT - EXT->Wr_EXT * EXT->Lq_hat * EXT->Idse_EXT;

    /* 전류 모델 적분 */
    EXT->Idse_EXT_est += EXT->INV_Ld_hat * Tsamp * (EXT->Vdse_FF_EXT - EXT->EEMFd_est - EXT->Rs_hat * EXT->Idse_EXT);
    EXT->Iqse_EXT_est += EXT->INV_Ld_hat * Tsamp * (EXT->Vqse_FF_EXT - EXT->EEMFq_est - EXT->Rs_hat * EXT->Iqse_EXT);

    /* EEMFq 발산 방지 (분모 제한) */
    if (fabs(EXT->EEMFq_est) < 1.f) {
        EXT->EEMFq_est = 1.f;
    }

    /* 위치 오차 계산: θ_err = atan2(-EEMFd, EEMFq) */
    EXT->Err_Thetar_EXT = atan2f(-EXT->EEMFd_est, EXT->EEMFq_est);
    EXT->Thetar_EXT_old  = EXT->Thetar_EXT;
    EXT->Thetarm_EXT_old = EXT->Thetarm_EXT;
}
