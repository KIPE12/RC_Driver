/**
 * @file    inv.c
 * @brief   인버터 제어 핵심 알고리즘 모듈
 *
 * PMSM(영구자석 동기 전동기) 제어에 필요한 파라미터 초기화,
 * 전류/속도/전압 제어기, 개루프 제어, Hall 센서 옵저버,
 * PWM 출력 관리 함수를 포함한다.
 *
 * @details 좌표계 표기 규칙:
 * | 접미사 | 좌표계               | 축       |
 * |--------|----------------------|----------|
 * | ss     | 정지 좌표계 (α-β)   | α(d), β(q) |
 * | sr     | 동기 좌표계 (d-q)   | d(자속), q(토크) |
 * | r      | 회전자 기준          | -        |
 *
 * @details 주요 제어 함수 목록:
 * | 함수                       | 역할                              |
 * |----------------------------|-----------------------------------|
 * | InitParameter()            | 전동기/인버터 파라미터 초기화     |
 * | InitCurrentController()    | PI 전류 제어기 초기화             |
 * | InitSpeedController()      | PI 속도 제어기 초기화             |
 * | Init_Spd_PLL()             | 속도 PLL 옵저버 초기화            |
 * | UpdateController()         | 실시간 게인 갱신                  |
 * | ResetController()          | 모든 상태 변수 초기화             |
 * | TorqueControl()            | 토크 기준 생성 (dutyCycle 기반)   |
 * | CurrentControl()           | d-q 축 PI 전류 제어 + SVPWM 출력 |
 * | SpeedControl()             | PI 속도 제어 → 토크 기준 생성    |
 * | OpenLoopControl()          | 전류 개루프 제어                  |
 * | Vref_GenControl()          | 전압 피드포워드 기준 생성         |
 * | VoltageOpenLoopControl()   | 전압 개루프 제어                  |
 * | VoltageInjection_SquareWave()| d축 구형파 주입 (파라미터 추정) |
 * | Hallsensor_Observer()      | Hall PLL 속도/위치 추정           |
 * | GetHallSensorState()       | Hall 3비트 상태 읽기              |
 * | HallPosition_Test()        | 기본 벡터 순차 인가 (Hall 확인)  |
 * | Align()                    | d축 초기 정렬 (4단계 상태 머신)  |
 * | PwmSwOn() / PwmSwOff()     | TIM1 PWM 채널 시작/정지           |
 * | PwmDutyUpt()               | TIM1 CCR 레지스터 듀티 갱신       |
 *
 * @author  HALAB_G
 * @date    2024-11-29
 */

#include "inv.h"
#include "flag.h"
#include "variable.h"
#include "speed_observer.h"
#include "math.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern float dutyCycle;

struct INVERTER INV; ///< 인버터 전역 상태/파라미터 구조체

int   Mode_align = 0;   ///< 정렬 상태 머신 단계 (0~3)
float Time_align  = 0.; ///< 정렬 경과 시간 [s]
int   cnt_inj     = 0;  ///< 구형파 주입 카운터 (홀짝 토글)
float W_LPF_set   = 50.f; ///< 속도 LPF 차단 주파수 설정값 [Hz]

/* ======================================================================
 * 초기화 함수
 * ====================================================================== */

/**
 * @brief 전동기 및 인버터 기본 파라미터를 초기화한다.
 *
 * @param[out] INV        인버터 구조체 포인터
 * @param[in]  Rs         고정자 저항 [Ω]
 * @param[in]  Ld         d축 인덕턴스 [H]
 * @param[in]  Lq         q축 인덕턴스 [H]
 * @param[in]  Lamf       영구자석 자속 [Wb]
 * @param[in]  PP         극쌍수 [-]
 * @param[in]  Jm         관성 모멘트 [kg·m²]
 * @param[in]  Bm         점성 마찰 계수 [N·m·s/rad]
 * @param[in]  Idsr_align 정렬용 d축 전류 기준값 [A]
 * @param[in]  Is_rated   정격 전류 [A]
 * @param[in]  Is_limit   최대 허용 전류 [A]
 * @param[in]  Wrpm_rated 정격 속도 [rpm]
 * @param[in]  Te_rated   정격 토크 [N·m]
 *
 * @note Kt = 1.5 × PP × Lamf, MTPA 테이블 간격도 내부에서 설정된다.
 */
void InitParameter(struct INVERTER *INV,
                   float Rs, float Ld, float Lq, float Lamf, float PP,
                   float Jm, float Bm,
                   float Idsr_align, float Is_rated, float Is_limit,
                   float Wrpm_rated, float Te_rated)
{
    INV->Rs      = Rs;
    INV->Ld      = Ld;
    INV->Lq      = Lq;
    INV->Ls      = Ld;
    INV->Lamf    = Lamf;
    INV->PP      = PP;
    INV->INV_PP  = 1.f / INV->PP;
    INV->Kt      = 1.5f * INV->PP * INV->Lamf;
    INV->INV_Kt  = 1.f / INV->Kt;
    INV->Jm      = Jm;
    INV->INV_Jm  = 1.f / INV->Jm;
    INV->Bm      = Bm;
    INV->Idsr_align = Idsr_align;
    INV->Is_rated   = Is_rated;
    INV->Is_limit   = Is_limit;
    INV->Te_rated   = INV->Kt * INV->Is_rated;
    INV->Te_limit   = INV->Kt * INV->Is_limit;
    INV->Wrpm_rated = Wrpm_rated;
    INV->Te_rated   = Te_rated;

    INV->MTPA_Te_gap     = 0.105263157894737f;
    INV->MTPA_Te_max     = 2.f;
    INV->MTPA_Te_gap_INV = 1.f / 0.105263157894737f;
}

/**
 * @brief 전류 제어기를 초기화하고 관련 상태 변수를 0으로 리셋한다.
 *
 * PI 전류 제어기 게인을 대역폭(Wcc)으로 설정한다 (내부 모델 방식):
 * - Kp = Wcc × L
 * - Ki = Wcc × Rs
 *
 * @param[out] INV 인버터 구조체 포인터
 * @param[in]  Wcc 전류 제어 대역폭 [rad/s]
 */
void InitCurrentController(struct INVERTER *INV, float Wcc)
{
    INV->Wcc     = Wcc;
    INV->Kpd_cc  = INV->Wcc * INV->Ld;
    INV->Kpq_cc  = INV->Wcc * INV->Lq;
    INV->Kid_cc  = INV->Wcc * INV->Rs;
    INV->Kiq_cc  = INV->Wcc * INV->Rs;
    INV->Kad_cc  = 1.f / fmaxf(INV->Kpd_cc, 1.e-9f);
    INV->Kaq_cc  = 1.f / fmaxf(INV->Kpq_cc, 1.e-9f);
    INV->Ractive = INV->Rs;

    /* 상태 변수 초기화 */
    INV->Idss = 0.; INV->Iqss = 0.;
    INV->Idsr = 0.; INV->Iqsr = 0.;
    INV->Idsr_err = 0.; INV->Iqsr_err = 0.;
    INV->Vdsr_ref_integ = 0.; INV->Vqsr_ref_integ = 0.;
    INV->Vdsr_ref_ff    = 0.; INV->Vqsr_ref_ff    = 0.;
    INV->Vdsr_ref_unsat = 0.; INV->Vqsr_ref_unsat = 0.;
    INV->Vdsr_ref = 0.; INV->Vqsr_ref = 0.; INV->Vmag_ref = 0.;
    INV->Vdsr_ref_old = 0.; INV->Vqsr_ref_old = 0.;
    INV->Vdsr_ref_aw  = 0.; INV->Vqsr_ref_aw  = 0.;
    INV->Vdss_ref = 0.; INV->Vqss_ref = 0.;
    INV->Vas_ref = 0.; INV->Vbs_ref = 0.; INV->Vcs_ref = 0.;
    INV->Vmax = 0.; INV->Vmin = 0.; INV->Voffset = 0.;
    INV->Van_ref = 0.; INV->Vbn_ref = 0.; INV->Vcn_ref = 0.;
    INV->Duty_A = 0.; INV->Duty_B = 0.; INV->Duty_C = 0.;

    /* 개루프 제어 변수 초기화 */
    INV->Idsr_ref_OLC = 0.;
    INV->Iqsr_ref_OLC = 0.;
    INV->Vdsr_ref_OLC = 0.;
    INV->Thetar_OLC   = 0.;
    INV->Thetar_OLC_buffer = 0.;
    INV->Wrpm_ref_OLC     = 0.;
    INV->Wrpm_ref_set_OLC = 0.;
    INV->Wrpm_slope_OLC   = 10.f;
    INV->Idsr_slope_OLC   = 20.f;
    INV->Idsr_ref_set_OLC = 0.;

    /* 비선형 보상(NLC) 변수 초기화 */
    INV->Idsr_NLC = 0.1f;
    INV->Idss_ref = 0.; INV->Iqss_ref = 0.;
    INV->Ia_ref = 0.; INV->Ib_ref = 0.; INV->Ic_ref = 0.;
    INV->A_NLC = 3.f;
    INV->B_NLC = 4.f;
    INV->C_NLC = 0.;
    INV->Va_NLC = 0.; INV->Vb_NLC = 0.; INV->Vc_NLC = 0.;

    /* 각도 관련 변수 초기화 */
    INV->Thetar         = 0.;
    INV->Thetar_adv     = 0.;
    INV->Thetar_offset  = 0.;
    INV->sinThetar      = 0.;
    INV->cosThetar      = 0.;
    INV->sinThetar_adv  = 0.;
    INV->cosThetar_adv  = 0.;
    INV->init_align_done = 0;

    INV->TestDuty_A = 0.f;
    INV->TestDuty_B = 0.f;
    INV->TestDuty_C = 0.f;
    INV->Duty_Test  = 0.01f;
}

/**
 * @brief 속도 제어기를 초기화하고 관련 상태 변수를 0으로 리셋한다.
 *
 * PI 속도 제어기 게인을 대역폭(Wsc)으로 설정한다:
 * - Kp = Jm × Wsc
 * - Ki = Kp × Wsc × 0.25
 *
 * @param[out] INV  인버터 구조체 포인터
 * @param[in]  Wsc  속도 제어 대역폭 [rad/s]
 * @param[in]  zeta 감쇠비 (현재 미사용, 향후 확장용)
 */
void InitSpeedController(struct INVERTER *INV, float Wsc, float zeta)
{
    INV->Wsc      = Wsc;
    INV->zeta_sc  = zeta;
    INV->Kp_sc    = INV->Jm * INV->Wsc;
    INV->Ki_scale = 0.25f;
    INV->Ki_sc    = INV->Kp_sc * INV->Wsc * INV->Ki_scale;
    INV->Ka_sc    = 1.f / fmaxf(INV->Kp_sc, 1.e-9f);

    INV->Wrpm = 0.; INV->Wrm = 0.; INV->Wr = 0.;
    INV->Wrpm_ref_set = 0.; INV->Wrm_ref_set = 0.;
    INV->Wrpm_ref = 0.; INV->Wrm_ref = 0.;
    INV->dWrm    = 3000.f * RPM2RM * Tsamp; /* 가속도 제한: 3000rpm/s */
    INV->Wrm_err = 0.;
    INV->Te_ref_integ  = 0.;
    INV->Te_ref_ff     = 0.;
    INV->Te_ref_unsat  = 0.;
    INV->Te_ref        = 0.;
    INV->Te_ref_aw     = 0.;
}

/**
 * @brief 속도 PLL(Phase-Locked Loop) 옵저버를 초기화한다.
 *
 * Hall 센서 또는 사용자 설정 대역폭으로 PLL 게인을 계산한다.
 * - 기본 PLL:  Kp = 2ζWs,  Ki = Ws²  (ζ=0.707)
 * - Hall PLL:  Kp = 2ζW_Hall, Ki = W_Hall² (W_Hall = 2π×10 rad/s)
 *
 * @param[out] INV 인버터 구조체 포인터
 * @param[in]  Ws  PLL 자연 주파수 [rad/s]
 */
void Init_Spd_PLL(struct INVERTER *INV, float Ws)
{
    INV->W_PLL           = Ws;
    INV->Kp_PLL          = 2.f * 0.707f * INV->W_PLL;
    INV->Ki_PLL          = INV->W_PLL * INV->W_PLL;
    INV->integ_Thetar_PLL = 0.;

    INV->W_PLL_Hall      = PI2 * 10.f;
    INV->Kp_PLL_Hall     = 2.f * 0.707f * INV->W_PLL_Hall;
    INV->Ki_PLL_Hall     = INV->W_PLL_Hall * INV->W_PLL_Hall;
    INV->integ_PLL_Hall  = 0.;
}

/**
 * @brief 제어기 게인을 현재 파라미터로 갱신한다.
 *
 * 실시간으로 Rs, Ld, Lq, Wcc, Wsc가 변경되었을 때 호출하여
 * 전류/속도 제어기 게인을 재계산한다. 매 샘플링마다 Control()에서 호출된다.
 *
 * @param[in,out] INV 인버터 구조체 포인터
 */
void UpdateController(struct INVERTER *INV)
{
    INV->Kpd_cc = INV->Wcc * INV->Ld;
    INV->Kpq_cc = INV->Wcc * INV->Lq;
    INV->Kid_cc = INV->Wcc * INV->Rs;
    INV->Kiq_cc = INV->Wcc * INV->Rs;
    INV->Kad_cc = 1.f / fmaxf(INV->Kpd_cc, 1.e-9f);
    INV->Kaq_cc = 1.f / fmaxf(INV->Kpq_cc, 1.e-9f);

    INV->Kp_sc   = INV->Jm * INV->Wsc;
    INV->Ki_scale = 0.25f;
    INV->Ki_sc   = INV->Kp_sc * INV->Wsc * INV->Ki_scale;
    INV->Ka_sc   = 1.f / fmaxf(INV->Kp_sc, 1.e-9f);
}

/**
 * @brief 모든 제어 상태 변수를 초기값으로 리셋한다.
 *
 * 고장 발생, 운전 정지, 모드 전환 시 호출하여 적분기 등
 * 내부 상태를 초기화한다.
 *
 * @param[out] INV 인버터 구조체 포인터
 */
void ResetController(struct INVERTER *INV)
{
    INV->Wrm_ref_set      = 0.;
    INV->Wrpm_ref_set     = 0.;
    INV->Wrpm_ref_set_old = 0.;
    INV->Wrm_ref          = 0.;
    INV->Wrpm_ref         = 0.;
    INV->Te_ref_integ     = 0.;
    INV->Te_ref_aw        = 0.;
    INV->Idsr_ref_integ   = 0.;
    INV->Idsr_ref_aw      = 0.;
    INV->Te_ref_sat       = 0.;
    INV->Te_ref           = 0.;
    INV->Idsr_ref         = 0.;
    INV->Iqsr_ref         = 0.;
    INV->Idsr_ref_OLC     = 0.;
    INV->Iqsr_ref_OLC     = 0.;
    INV->Vdsr_ref_OLC     = 0.;
    INV->Vqsr_ref_OLC     = 0.;
    INV->Thetar_OLC       = 0.;
    INV->Wrpm_ref_OLC     = 0.;
    INV->Wrpm_ref_set_OLC = 0.;
    INV->Wrpm_slope_OLC   = 5.f;
    INV->Vdsr_ref_integ   = 0.;
    INV->Vqsr_ref_integ   = 0.;
    INV->Vdsr_ref_unsat   = 0;
    INV->Vqsr_ref_unsat   = 0;
    INV->Vdsr_ref_aw      = 0.;
    INV->Vqsr_ref_aw      = 0.;
    INV->TestDuty_A       = 0.;
    INV->TestDuty_B       = 0.;
    INV->TestDuty_C       = 0.;
    INV->alpha_LPF        = 0;
}

/* ======================================================================
 * 제어 알고리즘
 * ====================================================================== */

/**
 * @brief 토크 제어 함수 (듀티 사이클 기반 토크 기준 생성)
 *
 * 외부 입력(dutyCycle)으로부터 토크 기준을 생성하고
 * q축 전류 기준만 사용하는 단순 토크 제어를 수행한다.
 * (Id=0 제어, MTPA 미적용)
 *
 * @param[in,out] INV 인버터 구조체 포인터
 * @note 데드존: |Te_ref| < 5% × Te_rated → Te_ref = 0
 */
void TorqueControl(struct INVERTER *INV)
{
    INV->Te_ref = (dutyCycle - 0.15f) * 20.f * INV->Te_rated;

    if (ABS(INV->Te_ref) < 0.05f * INV->Te_rated) INV->Te_ref = 0.f;

    INV->Idsr_ref        = 0.f;
    INV->Iqsr_ref_unsat  = INV->Te_ref * INV->INV_Kt;
    INV->Iqsr_max        = sqrtf(INV->Is_limit * INV->Is_limit - INV->Idsr_ref * INV->Idsr_ref);
    INV->Iqsr_ref        = LIMIT(INV->Iqsr_ref_unsat, -INV->Iqsr_max, INV->Iqsr_max);
}

/**
 * @brief PI 전류 제어기 (d-q 동기 좌표계)
 *
 * dq 전류 오차에 대한 PI 제어를 수행하고 결과를 3상 듀티로 변환한다.
 *
 * @details 처리 순서:
 * 1. Clarke 변환: (Ia,Ib,Ic) → (Idss, Iqss)
 * 2. Park 변환:   (Idss, Iqss) + Thetar → (Idsr, Iqsr)
 * 3. PI 제어 (Anti-windup 포함) → Vdsr_ref, Vqsr_ref
 * 4. 역Park 변환 → (Vdss_ref, Vqss_ref)
 * 5. 역Clarke 변환 → (Vas_ref, Vbs_ref, Vcs_ref)
 * 6. SVPWM 오프셋 계산 (min-max 방식)
 * 7. NLC(비선형 보상) 전압 추가
 * 8. 듀티비 계산 및 PWM 출력
 *
 * @param[in,out] INV 인버터 구조체 포인터
 * @note Theta_mode==1 이면 OLC 각도 사용, 아니면 Hall PLL 각도 사용
 * @warning FLAG.FAULT 발생 시 PWM_BUF_OFF 유지
 */
void CurrentControl(struct INVERTER *INV)
{
    if (FLAG.INV_RUN) {
        INV->Iqsr_ref_unsat = INV->Te_ref * INV->INV_Kt;
        INV->Iqsr_ref = LIMIT(INV->Iqsr_ref_unsat, -3.f * INV->Is_rated, 3.f * INV->Is_rated);
        INV->Idsr_ref = 0;
    }

    INV->Vdsr_ref_old = INV->Vdsr_ref;
    INV->Vqsr_ref_old = INV->Vqsr_ref;

    /* Clarke 변환 (α-β) */
    INV->Idss = (2.f * INV->Ia - INV->Ib - INV->Ic) * INV_3;
    INV->Iqss = (INV->Ib - INV->Ic) * INV_SQRT3;

    Vdss_ref_set = (2.f * Van - Vbn - Vcn) * INV_3;
    Vqss_ref_set = INV_SQRT3 * (Vbn - Vcn);

    /* 각도 선택 */
    if (Theta_mode == 1) {
        INV->Thetar = INV->Thetar_OLC;
        INV->Wr     = INV->Wrpm_ref_OLC * RPM2RM * INV->PP;
    } else {
        INV->Thetar = INV->Thetar_est_Hall;
        INV->Wr     = INV->Wr_est_Hall;
    }

    INV->Wrm  = INV->Wr * INV->INV_PP;
    INV->Wrpm = INV->Wrm * RM2RPM;

    INV->Thetar_adv    = BOUND_PI(INV->Thetar + 1.5f * INV->Wr * Tsamp);
    INV->cosThetar     = COS(INV->Thetar * INV->Thetar);
    INV->sinThetar     = SIN(INV->Thetar, INV->Thetar * INV->Thetar);
    INV->cosThetar_adv = COS(INV->Thetar_adv * INV->Thetar_adv);
    INV->sinThetar_adv = SIN(INV->Thetar_adv, INV->Thetar_adv * INV->Thetar_adv);

    /* Park 변환 (d-q) */
    INV->Idsr = INV->Idss * INV->cosThetar + INV->Iqss * INV->sinThetar;
    INV->Iqsr = -INV->Idss * INV->sinThetar + INV->Iqss * INV->cosThetar;

    /* PI 전류 제어 (Anti-windup 포함) */
    INV->Idsr_err = INV->Idsr_ref - INV->Idsr;
    INV->Iqsr_err = INV->Iqsr_ref - INV->Iqsr;

    INV->Vdsr_ref_integ += INV->Kid_cc * (INV->Idsr_err - INV->Kad_cc * INV->Vdsr_ref_aw) * Tsamp;
    INV->Vqsr_ref_integ += INV->Kiq_cc * (INV->Iqsr_err - INV->Kaq_cc * INV->Vqsr_ref_aw) * Tsamp;

    /* 피드포워드: 역기전력 분리 */
    INV->Vdsr_ref_ff = -INV->Wr * INV->Lq * INV->Iqsr_ref;
    INV->Vqsr_ref_ff =  INV->Wr * (INV->Ld * INV->Idsr_ref + INV->Lamf);

    INV->Vdsr_ref_unsat_old = INV->Kpd_cc * INV->Idsr_err + INV->Vdsr_ref_integ + INV->Vdsr_ref_ff - INV->Ractive * INV->Idsr;
    INV->Vqsr_ref_unsat_old = INV->Kpq_cc * INV->Iqsr_err + INV->Vqsr_ref_integ + INV->Vqsr_ref_ff - INV->Ractive * INV->Iqsr;

    INV->Vdsr_ref_unsat = INV->alpha_LPF * INV->Vdsr_ref_unsat + (1 - INV->alpha_LPF) * INV->Vdsr_ref_unsat_old;
    INV->Vqsr_ref_unsat = INV->alpha_LPF * INV->Vqsr_ref_unsat + (1 - INV->alpha_LPF) * INV->Vqsr_ref_unsat_old;

    /* 전압 포화 제한 (√3/3 × Vdc) */
    INV->Vdsr_ref = LIMIT(INV->Vdsr_ref_unsat, -INV->Vdc * INV_SQRT3, INV->Vdc * INV_SQRT3);
    INV->Vqsr_ref = LIMIT(INV->Vqsr_ref_unsat, -INV->Vdc * INV_SQRT3, INV->Vdc * INV_SQRT3);
    INV->Vdsr_ref_aw = INV->Vdsr_ref_unsat - INV->Vdsr_ref;
    INV->Vqsr_ref_aw = INV->Vqsr_ref_unsat - INV->Vqsr_ref;

    /* 역Park 변환 */
    INV->Vdss_ref = INV->Vdsr_ref * INV->cosThetar_adv - INV->Vqsr_ref * INV->sinThetar_adv;
    INV->Vqss_ref = INV->Vdsr_ref * INV->sinThetar_adv + INV->Vqsr_ref * INV->cosThetar_adv;

    /* 역Clarke 변환 */
    INV->Vas_ref = INV->Vdss_ref;
    INV->Vbs_ref = -0.5f * INV->Vdss_ref + SQRT3HALF * INV->Vqss_ref;
    INV->Vcs_ref = -0.5f * INV->Vdss_ref - SQRT3HALF * INV->Vqss_ref;

    /* SVPWM 오프셋 (min-max 중간값) */
    INV->Vmax = INV->Vas_ref;
    if (INV->Vmax < INV->Vbs_ref) INV->Vmax = INV->Vbs_ref;
    if (INV->Vmax < INV->Vcs_ref) INV->Vmax = INV->Vcs_ref;
    INV->Vmin = INV->Vas_ref;
    if (INV->Vmin > INV->Vbs_ref) INV->Vmin = INV->Vbs_ref;
    if (INV->Vmin > INV->Vcs_ref) INV->Vmin = INV->Vcs_ref;
    INV->Voffset = -0.5f * (INV->Vmax + INV->Vmin);

    /* 전류 기준 역변환 (참조용) */
    INV->Idss_ref = INV->Idsr_ref * INV->cosThetar_adv - INV->Iqsr_ref * INV->sinThetar_adv;
    INV->Iqss_ref = INV->Idsr_ref * INV->sinThetar_adv + INV->Iqsr_ref * INV->cosThetar_adv;
    INV->Ia_ref = INV->Idss_ref;
    INV->Ib_ref = -0.5f * INV->Idss_ref + SQRT3HALF * INV->Iqss_ref;
    INV->Ic_ref = -0.5f * INV->Idss_ref - SQRT3HALF * INV->Iqss_ref;

    /* NLC 비선형 보상 전압 */
    INV->Va_NLC = INV->A_NLC * atanf(INV->B_NLC * INV->Ia_ref);
    INV->Vb_NLC = INV->A_NLC * atanf(INV->B_NLC * INV->Ib_ref);
    INV->Vc_NLC = INV->A_NLC * atanf(INV->B_NLC * INV->Ic_ref);

    /* 최종 상전압 (NLC 포함) */
    INV->Van_ref = LIMIT(INV->Vas_ref + INV->Voffset + INV->Va_NLC, -0.5f * INV->Vdc, 0.5f * INV->Vdc);
    INV->Vbn_ref = LIMIT(INV->Vbs_ref + INV->Voffset + INV->Vb_NLC, -0.5f * INV->Vdc, 0.5f * INV->Vdc);
    INV->Vcn_ref = LIMIT(INV->Vcs_ref + INV->Voffset + INV->Vc_NLC, -0.5f * INV->Vdc, 0.5f * INV->Vdc);

    Van = INV->Van_ref; Vbn = INV->Vbn_ref; Vcn = INV->Vcn_ref;

    /* 듀티비 계산 [0, 1] */
    INV->Duty_A = LIMIT(INV->Van_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);
    INV->Duty_B = LIMIT(INV->Vbn_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);
    INV->Duty_C = LIMIT(INV->Vcn_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);

    PwmSwOn();
    PwmDutyUpt();
    if (!FLAG.FAULT) PWM_BUF_ON;
}

/**
 * @brief PI 속도 제어기 (rpm → 토크 기준 생성)
 *
 * 속도 기준(Wrpm_ref_set)을 경사 제한(dWrm)하여 Wrm_ref를 생성하고,
 * 속도 오차에 PI 제어를 적용하여 Te_ref를 출력한다.
 *
 * @param[in,out] INV 인버터 구조체 포인터
 * @note 데드존: |Wrpm_ref_set| < 5% × Wrpm_rated → Wrpm_ref_set = 0
 */
void SpeedControl(struct INVERTER *INV)
{
    INV->Wrpm_ref_set = INV->Wrpm_ref_cmd;
    if (ABS(INV->Wrpm_ref_set) < 0.05f * INV->Wrpm_rated) INV->Wrpm_ref_set = 0.;

    /* 속도 경사 제한 */
    INV->Wrm_ref_set = INV->Wrpm_ref_set * RPM2RM;
    if      (INV->Wrm_ref < INV->Wrm_ref_set - INV->dWrm) INV->Wrm_ref += INV->dWrm;
    else if (INV->Wrm_ref > INV->Wrm_ref_set + INV->dWrm) INV->Wrm_ref -= INV->dWrm;
    else    INV->Wrm_ref = INV->Wrm_ref_set;
    INV->Wrpm_ref = INV->Wrm_ref * RM2RPM;

    /* PI 속도 제어 (Anti-windup 포함) */
    INV->Wrm_err = INV->Wrm_ref - INV->Wrm;
    INV->Te_ref_integ += INV->Ki_sc * (INV->Wrm_err - INV->Ka_sc * INV->Te_ref_aw) * Tsamp;
    INV->Te_ref_unsat  = INV->Kp_sc * INV->Wrm_err + INV->Te_ref_integ;
    INV->Te_ref        = LIMIT(INV->Te_ref_unsat, -INV->Te_rated, INV->Te_rated);
    INV->Te_ref_aw     = INV->Te_ref_unsat - INV->Te_ref;
}

/**
 * @brief 전류 개루프 제어 (Open-Loop Current Control)
 *
 * 외부에서 설정한 d/q 전류 기준(Idsr_ref_OLC, Iqsr_ref_OLC)과
 * OLC 각속도(Wrpm_ref_OLC)로 생성한 각도를 사용하여
 * CurrentControl()을 호출한다.
 *
 * @param[in,out] INV 인버터 구조체 포인터
 * @note CurrentControl() 내에서 Theta_mode=1을 통해 OLC 각도가 선택된다.
 */
void OpenLoopControl(struct INVERTER *INV)
{
    /* d축 전류 경사 제한 */
    if      (INV->Idsr_ref_set_OLC > INV->Idsr_ref_OLC + Tsamp * INV->Idsr_slope_OLC)
             INV->Idsr_ref_OLC += Tsamp * INV->Idsr_slope_OLC;
    else if (INV->Idsr_ref_set_OLC < INV->Idsr_ref_OLC - Tsamp * INV->Idsr_slope_OLC)
             INV->Idsr_ref_OLC -= Tsamp * INV->Idsr_slope_OLC;
    else     INV->Idsr_ref_OLC  = INV->Idsr_ref_set_OLC;

    INV->Idsr_ref = INV->Idsr_ref_OLC;
    INV->Iqsr_ref = INV->Iqsr_ref_OLC;

    /* 속도 경사 제한 */
    if      (INV->Wrpm_ref_set_OLC > INV->Wrpm_ref_OLC + Tsamp * INV->Wrpm_slope_OLC)
             INV->Wrpm_ref_OLC += Tsamp * INV->Wrpm_slope_OLC;
    else if (INV->Wrpm_ref_set_OLC < INV->Wrpm_ref_OLC - Tsamp * INV->Wrpm_slope_OLC)
             INV->Wrpm_ref_OLC -= Tsamp * INV->Wrpm_slope_OLC;
    else     INV->Wrpm_ref_OLC  = INV->Wrpm_ref_set_OLC;

    INV->Thetar_OLC = BOUND_PI(INV->Thetar_OLC + INV->Wrpm_ref_OLC * RPM2RM * INV->PP * Tsamp);
}

/**
 * @brief 전압 기준 생성 제어 (전압 피드포워드 방식)
 *
 * Theta_mode에 따라 두 가지 동작을 수행한다:
 * - mode=1 (OLC): OLC 기준으로 모델 기반 피드포워드 전압 생성
 * - mode=0 (Hall): Hall PLL 각도 + 속도 제어 토크 기준으로 전압 생성
 *
 * @param[in,out] INV 인버터 구조체 포인터
 */
void Vref_GenControl(struct INVERTER *INV)
{
    if (Theta_mode == 1) {
        /* OLC 각도 + 모델 기반 피드포워드 전압 */
        if      (INV->Idsr_ref_set_OLC > INV->Idsr_ref_OLC + Tsamp * INV->Idsr_slope_OLC)
                 INV->Idsr_ref_OLC += Tsamp * INV->Idsr_slope_OLC;
        else if (INV->Idsr_ref_set_OLC < INV->Idsr_ref_OLC - Tsamp * INV->Idsr_slope_OLC)
                 INV->Idsr_ref_OLC -= Tsamp * INV->Idsr_slope_OLC;
        else     INV->Idsr_ref_OLC  = INV->Idsr_ref_set_OLC;

        INV->Idsr_ref = INV->Idsr_ref_OLC;
        INV->Iqsr_ref = INV->Iqsr_ref_OLC;

        if      (INV->Wrpm_ref_set_OLC > INV->Wrpm_ref_OLC + Tsamp * INV->Wrpm_slope_OLC)
                 INV->Wrpm_ref_OLC += Tsamp * INV->Wrpm_slope_OLC;
        else if (INV->Wrpm_ref_set_OLC < INV->Wrpm_ref_OLC - Tsamp * INV->Wrpm_slope_OLC)
                 INV->Wrpm_ref_OLC -= Tsamp * INV->Wrpm_slope_OLC;
        else     INV->Wrpm_ref_OLC  = INV->Wrpm_ref_set_OLC;

        INV->Wr_ref_OLC = INV->Wrpm_ref_OLC * RPM2RM * INV->PP;

        /* 정상 상태 전압 모델: Vd = Rs×Id - Wr×Lq×Iq, Vq = Rs×Iq + Wr×(Ld×Id + λf) */
        INV->Vdsr_ref_OLC = INV->Rs * INV->Idsr_ref_OLC - INV->Wr_ref_OLC * INV->Lq * INV->Iqsr_ref_OLC;
        INV->Vqsr_ref_OLC = INV->Rs * INV->Iqsr_ref_OLC + INV->Wr_ref_OLC * (INV->Ld * INV->Idsr_ref_OLC + INV->Lamf);
        INV->Vdsr_ref_unsat = INV->Vdsr_ref_OLC;
        INV->Vqsr_ref_unsat = INV->Vqsr_ref_OLC;

        INV->Thetar_OLC = BOUND_PI(INV->Thetar_OLC + INV->Wr_ref_OLC * Tsamp);
        INV->Thetar     = INV->Thetar_OLC;
    }
    else {
        /* Hall PLL 각도 + 속도 제어 토크 → Iq 기준 전환 */
        INV->Iqsr_ref_unsat = INV->Te_ref * INV->INV_Kt;
        INV->Iqsr_ref = LIMIT(INV->Iqsr_ref_unsat, -1.3f * INV->Is_rated, 1.3f * INV->Is_rated);
        INV->Idsr_ref = 0;

        INV->Vdsr_ref = INV->Rs * INV->Idsr_ref - INV->Wrm_ref * INV->PP * INV->Lq * INV->Iqsr_ref;
        INV->Vqsr_ref = INV->Rs * INV->Iqsr_ref + INV->Wrm_ref * INV->PP * (INV->Ld * INV->Idsr_ref + INV->Lamf);
        INV->Vdsr_ref_unsat = INV->Vdsr_ref;
        INV->Vqsr_ref_unsat = INV->Vqsr_ref;

        INV->Thetar = INV->Thetar_est_Hall;
    }

    /* 이하 역변환 및 PWM 출력 (CurrentControl과 동일 구조) */
    INV->Thetar_adv    = BOUND_PI(INV->Thetar + 1.5f * INV->Wr * Tsamp);
    INV->cosThetar     = COS(INV->Thetar * INV->Thetar);
    INV->sinThetar     = SIN(INV->Thetar, INV->Thetar * INV->Thetar);
    INV->cosThetar_adv = COS(INV->Thetar_adv * INV->Thetar_adv);
    INV->sinThetar_adv = SIN(INV->Thetar_adv, INV->Thetar_adv * INV->Thetar_adv);

    INV->Vdsr_ref = LIMIT(INV->Vdsr_ref_unsat, -INV->Vdc * INV_SQRT3, INV->Vdc * INV_SQRT3);
    INV->Vqsr_ref = LIMIT(INV->Vqsr_ref_unsat, -INV->Vdc * INV_SQRT3, INV->Vdc * INV_SQRT3);
    INV->Vdsr_ref_aw  = INV->Vdsr_ref_unsat - INV->Vdsr_ref;
    INV->Vqsr_ref_aw  = INV->Vqsr_ref_unsat - INV->Vqsr_ref;
    INV->Vdss_ref = INV->Vdsr_ref * INV->cosThetar_adv - INV->Vqsr_ref * INV->sinThetar_adv;
    INV->Vqss_ref = INV->Vdsr_ref * INV->sinThetar_adv + INV->Vqsr_ref * INV->cosThetar_adv;

    INV->Vas_ref = INV->Vdss_ref;
    INV->Vbs_ref = -0.5f * INV->Vdss_ref + SQRT3HALF * INV->Vqss_ref;
    INV->Vcs_ref = -0.5f * INV->Vdss_ref - SQRT3HALF * INV->Vqss_ref;

    INV->Vmax = INV->Vas_ref;
    if (INV->Vmax < INV->Vbs_ref) INV->Vmax = INV->Vbs_ref;
    if (INV->Vmax < INV->Vcs_ref) INV->Vmax = INV->Vcs_ref;
    INV->Vmin = INV->Vas_ref;
    if (INV->Vmin > INV->Vbs_ref) INV->Vmin = INV->Vbs_ref;
    if (INV->Vmin > INV->Vcs_ref) INV->Vmin = INV->Vcs_ref;
    INV->Voffset = -0.5f * (INV->Vmax + INV->Vmin);

    INV->Van_ref = LIMIT(INV->Vas_ref + INV->Voffset, -0.5f * INV->Vdc, 0.5f * INV->Vdc);
    INV->Vbn_ref = LIMIT(INV->Vbs_ref + INV->Voffset, -0.5f * INV->Vdc, 0.5f * INV->Vdc);
    INV->Vcn_ref = LIMIT(INV->Vcs_ref + INV->Voffset, -0.5f * INV->Vdc, 0.5f * INV->Vdc);

    Van = INV->Van_ref; Vbn = INV->Vbn_ref; Vcn = INV->Vcn_ref;

    INV->Duty_A = LIMIT(INV->Van_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);
    INV->Duty_B = LIMIT(INV->Vbn_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);
    INV->Duty_C = LIMIT(INV->Vcn_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);

    PwmSwOn();
    PwmDutyUpt();
    if (!FLAG.FAULT) PWM_BUF_ON;
}

/**
 * @brief 전압 개루프 제어 (Voltage Open-Loop Control)
 *
 * 외부에서 직접 설정한 dq 전압 기준(Vdsr_ref_OLC, Vqsr_ref_OLC)을
 * OLC 각도로 역변환하여 PWM을 출력한다.
 *
 * @param[in,out] INV 인버터 구조체 포인터
 */
void VoltageOpenLoopControl(struct INVERTER *INV)
{
    INV->Vdsr_ref_unsat = INV->Vdsr_ref_OLC;
    INV->Vqsr_ref_unsat = INV->Vqsr_ref_OLC;

    INV->Thetar_OLC = BOUND_PI(INV->Thetar_OLC + INV->Wrpm_ref_OLC * RPM2RM * INV->PP * Tsamp);

    if (Theta_mode) {
        INV->Thetar = EXT_1.Thetar_EXT_old;
        INV->Wr     = EXT_1.Wr_EXT_f;
    } else {
        INV->Thetar = INV->Thetar_OLC;
        INV->Wr     = INV->Wrpm_ref_OLC * RPM2RM * INV->PP;
    }

    INV->Idss = (2.f * INV->Ia - INV->Ib - INV->Ic) * INV_3;
    INV->Iqss = (INV->Ib - INV->Ic) * INV_SQRT3;

    INV->Thetar_adv    = BOUND_PI(INV->Thetar + 1.5f * INV->Wr * Tsamp);
    INV->cosThetar     = COS(INV->Thetar * INV->Thetar);
    INV->sinThetar     = SIN(INV->Thetar, INV->Thetar * INV->Thetar);
    INV->cosThetar_adv = COS(INV->Thetar_adv * INV->Thetar_adv);
    INV->sinThetar_adv = SIN(INV->Thetar_adv, INV->Thetar_adv * INV->Thetar_adv);

    INV->Vdsr_ref = LIMIT(INV->Vdsr_ref_unsat, -INV->Vdc * INV_SQRT3, INV->Vdc * INV_SQRT3);
    INV->Vqsr_ref = LIMIT(INV->Vqsr_ref_unsat, -INV->Vdc * INV_SQRT3, INV->Vdc * INV_SQRT3);
    INV->Vdsr_ref_aw  = INV->Vdsr_ref_unsat - INV->Vdsr_ref;
    INV->Vqsr_ref_aw  = INV->Vqsr_ref_unsat - INV->Vqsr_ref;
    INV->Vdss_ref = INV->Vdsr_ref * INV->cosThetar_adv - INV->Vqsr_ref * INV->sinThetar_adv;
    INV->Vqss_ref = INV->Vdsr_ref * INV->sinThetar_adv + INV->Vqsr_ref * INV->cosThetar_adv;

    INV->Vas_ref = INV->Vdss_ref;
    INV->Vbs_ref = -0.5f * INV->Vdss_ref + SQRT3HALF * INV->Vqss_ref;
    INV->Vcs_ref = -0.5f * INV->Vdss_ref - SQRT3HALF * INV->Vqss_ref;

    INV->Vmax = INV->Vas_ref;
    if (INV->Vmax < INV->Vbs_ref) INV->Vmax = INV->Vbs_ref;
    if (INV->Vmax < INV->Vcs_ref) INV->Vmax = INV->Vcs_ref;
    INV->Vmin = INV->Vas_ref;
    if (INV->Vmin > INV->Vbs_ref) INV->Vmin = INV->Vbs_ref;
    if (INV->Vmin > INV->Vcs_ref) INV->Vmin = INV->Vcs_ref;
    INV->Voffset = -0.5f * (INV->Vmax + INV->Vmin);

    INV->Van_ref = LIMIT(INV->Vas_ref + INV->Voffset, -0.5f * INV->Vdc, 0.5f * INV->Vdc);
    INV->Vbn_ref = LIMIT(INV->Vbs_ref + INV->Voffset, -0.5f * INV->Vdc, 0.5f * INV->Vdc);
    INV->Vcn_ref = LIMIT(INV->Vcs_ref + INV->Voffset, -0.5f * INV->Vdc, 0.5f * INV->Vdc);

    Van = INV->Van_ref; Vbn = INV->Vbn_ref; Vcn = INV->Vcn_ref;

    INV->Duty_A = LIMIT(INV->Van_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);
    INV->Duty_B = LIMIT(INV->Vbn_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);
    INV->Duty_C = LIMIT(INV->Vcn_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);

    PwmSwOn();
    PwmDutyUpt();
    if (!FLAG.FAULT) PWM_BUF_ON;
}

/**
 * @brief d축 구형파 전압 주입 (파라미터 추정용)
 *
 * 매 제어 주기마다 d축 전압을 ±Vmag_inj로 토글하여 주입한다.
 * 인덕턴스, 저항 등 파라미터 추정에 사용된다.
 *
 * @param[in,out] INV 인버터 구조체 포인터
 * @note 주입 방향: cnt_inj≥2 → +d주입, cnt_inj<2 → -d주입 (2주기 토글)
 */
void VoltageInjection_SquareWave(struct INVERTER *INV)
{
    INV->Vmag_inj = 1;
    INV->Thetar   = INV->Thetar_OLC;
    INV->Thetar_adv    = INV->Thetar_OLC;
    INV->cosThetar     = 1;
    INV->sinThetar     = 0;
    INV->cosThetar_adv = 1;
    INV->sinThetar_adv = 0;

    if (cnt_inj >= 2) {
        INV->Vdsr_ref_unsat = +INV->Vmag_inj; /* +d 주입 */
        INV->Vqsr_ref_unsat =  0;
        cnt_inj = 0;
    } else {
        INV->Vdsr_ref_unsat = -INV->Vmag_inj; /* -d 주입 */
        INV->Vqsr_ref_unsat =  0;
        cnt_inj = 1;
    }
    cnt_inj++;

    INV->Vdsr_ref = LIMIT(INV->Vdsr_ref_unsat, -INV->Vdc * INV_SQRT3, INV->Vdc * INV_SQRT3);
    INV->Vqsr_ref = LIMIT(INV->Vqsr_ref_unsat, -INV->Vdc * INV_SQRT3, INV->Vdc * INV_SQRT3);
    INV->Vdsr_ref_aw  = INV->Vdsr_ref_unsat - INV->Vdsr_ref;
    INV->Vqsr_ref_aw  = INV->Vqsr_ref_unsat - INV->Vqsr_ref;
    INV->Vdss_ref = INV->Vdsr_ref * INV->cosThetar_adv - INV->Vqsr_ref * INV->sinThetar_adv;
    INV->Vqss_ref = INV->Vdsr_ref * INV->sinThetar_adv + INV->Vqsr_ref * INV->cosThetar_adv;

    INV->Vas_ref = INV->Vdss_ref;
    INV->Vbs_ref = -0.5f * INV->Vdss_ref + SQRT3HALF * INV->Vqss_ref;
    INV->Vcs_ref = -0.5f * INV->Vdss_ref - SQRT3HALF * INV->Vqss_ref;

    INV->Vmax = INV->Vas_ref;
    if (INV->Vmax < INV->Vbs_ref) INV->Vmax = INV->Vbs_ref;
    if (INV->Vmax < INV->Vcs_ref) INV->Vmax = INV->Vcs_ref;
    INV->Vmin = INV->Vas_ref;
    if (INV->Vmin > INV->Vbs_ref) INV->Vmin = INV->Vbs_ref;
    if (INV->Vmin > INV->Vcs_ref) INV->Vmin = INV->Vcs_ref;
    INV->Voffset = -0.5f * (INV->Vmax + INV->Vmin);

    INV->Idss_ref = INV->Idsr_ref * INV->cosThetar_adv - INV->Iqsr_ref * INV->sinThetar_adv;
    INV->Iqss_ref = INV->Idsr_ref * INV->sinThetar_adv + INV->Iqsr_ref * INV->cosThetar_adv;
    INV->Ia_ref = INV->Idss_ref;
    INV->Ib_ref = -0.5f * INV->Idss_ref + SQRT3HALF * INV->Iqss_ref;
    INV->Ic_ref = -0.5f * INV->Idss_ref - SQRT3HALF * INV->Iqss_ref;

    INV->Va_NLC = INV->A_NLC * atanf(INV->B_NLC * INV->Ia_ref);
    INV->Vb_NLC = INV->A_NLC * atanf(INV->B_NLC * INV->Ib_ref);
    INV->Vc_NLC = INV->A_NLC * atanf(INV->B_NLC * INV->Ic_ref);

    INV->Van_ref = LIMIT(INV->Vas_ref + INV->Voffset + INV->Va_NLC, -0.5f * INV->Vdc, 0.5f * INV->Vdc);
    INV->Vbn_ref = LIMIT(INV->Vbs_ref + INV->Voffset + INV->Vb_NLC, -0.5f * INV->Vdc, 0.5f * INV->Vdc);
    INV->Vcn_ref = LIMIT(INV->Vcs_ref + INV->Voffset + INV->Vc_NLC, -0.5f * INV->Vdc, 0.5f * INV->Vdc);

    INV->Duty_A = LIMIT(INV->Van_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);
    INV->Duty_B = LIMIT(INV->Vbn_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);
    INV->Duty_C = LIMIT(INV->Vcn_ref * INV->INV_Vdc + 0.5f, 0.f, 1.f);

    PwmDutyUpt();
    PwmSwOn();
    if (!FLAG.FAULT) PWM_BUF_ON;
}

/* ======================================================================
 * Hall 센서 옵저버 / 위치 테스트
 * ====================================================================== */

/**
 * @brief Hall 센서 기반 속도/위치 PLL 옵저버
 *
 * Hall 센서 3비트 상태로부터 전기각을 이산적으로 결정하고,
 * PI-PLL을 사용하여 전기각(Thetar_est_Hall)과 전기 각속도(Wr_est_Hall)를
 * 연속적으로 추정한다.
 *
 * @details Hall 상태와 전기각 매핑 (0~2π, 60° 간격):
 * | hall_state | Thetar_Hall_PLL |
 * |------------|-----------------|
 * | 6          | 0               |
 * | 4          | π/3             |
 * | 5          | 2π/3            |
 * | 1          | π               |
 * | 3          | -2π/3           |
 * | 2          | -π/3            |
 *
 * @param[in,out] INV 인버터 구조체 포인터
 * @note GPIOC PIN6=hall_a, GPIOC PIN7=hall_b, GPIOD PIN2=hall_c
 */
void Hallsensor_Observer(struct INVERTER *INV)
{
    /* PLL: 위치 오차로 속도/각도 추정 */
    INV->Thetar_err_Hall = BOUND_PI(INV->Thetar_Hall_PLL - INV->Thetar_est_Hall);
    INV->integ_PLL_Hall += Tsamp * INV->Ki_PLL_Hall * INV->Thetar_err_Hall;
    INV->Wr_est_Hall     = INV->Kp_PLL_Hall * INV->Thetar_err_Hall + INV->integ_PLL_Hall;
    INV->Thetar_est_Hall += Tsamp * INV->Wr_est_Hall;
    INV->Thetar_est_Hall  = BOUND_PI(INV->Thetar_est_Hall);

    INV->Wr   = INV->Wr_est_Hall;
    INV->Wrm  = INV->Wr * INV->INV_PP;
    INV->Wrpm = INV->Wrm * Rm2Rpm;

    /* Hall 센서 읽기 */
    INV->hall_a = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
    INV->hall_b = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
    INV->hall_c = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
    INV->hall_state = GetHallSensorState(INV->hall_a, INV->hall_b, INV->hall_c);

    /* Hall 상태 → 전기각 매핑 */
    switch (INV->hall_state) {
        case 6: INV->Thetar_Hall_PLL =  0.f;        break;
        case 4: INV->Thetar_Hall_PLL =  PIOF3;      break;
        case 5: INV->Thetar_Hall_PLL =  2.f * PIOF3; break;
        case 1: INV->Thetar_Hall_PLL =  3.f * PIOF3; break;
        case 3: INV->Thetar_Hall_PLL = -2.f * PIOF3; break;
        case 2: INV->Thetar_Hall_PLL = -PIOF3;      break;
    }
}

/**
 * @brief Hall 센서 3개의 상태를 3비트 정수로 변환한다.
 *
 * @param[in] hall_sensor_1 Hall A 핀 상태
 * @param[in] hall_sensor_2 Hall B 핀 상태
 * @param[in] hall_sensor_3 Hall C 핀 상태
 * @return    3비트 Hall 상태값 (0~7)
 */
uint8_t GetHallSensorState(GPIO_PinState hall_sensor_1,
                            GPIO_PinState hall_sensor_2,
                            GPIO_PinState hall_sensor_3)
{
    uint8_t hall_state = 0;
    hall_state |= (hall_sensor_1 == GPIO_PIN_SET) ? 0x01 : 0x00;
    hall_state |= (hall_sensor_2 == GPIO_PIN_SET) ? 0x02 : 0x00;
    hall_state |= (hall_sensor_3 == GPIO_PIN_SET) ? 0x04 : 0x00;
    return hall_state;
}

/**
 * @brief Hall 위치 테스트: 기본 전압 벡터를 순차 인가한다.
 *
 * duty_state 값에 따라 6개 기본 전압 벡터 중 하나를 선택하여
 * Hall 센서 설치 위치와 극성을 확인하는 데 사용한다.
 *
 * @param[in,out] INV 인버터 구조체 포인터
 * @note duty_state 1~6 → 기본 벡터, 그 외 → 영 벡터
 */
void HallPosition_Test(struct INVERTER *INV)
{
    switch (INV->duty_state) {
        case 1: INV->Duty_A = INV->Duty_Test; INV->Duty_B = 0;              INV->Duty_C = 0;              break;
        case 2: INV->Duty_A = INV->Duty_Test; INV->Duty_B = INV->Duty_Test; INV->Duty_C = 0;              break;
        case 3: INV->Duty_A = 0;              INV->Duty_B = INV->Duty_Test; INV->Duty_C = 0;              break;
        case 4: INV->Duty_A = 0;              INV->Duty_B = INV->Duty_Test; INV->Duty_C = INV->Duty_Test; break;
        case 5: INV->Duty_A = 0;              INV->Duty_B = 0;              INV->Duty_C = INV->Duty_Test; break;
        case 6: INV->Duty_A = INV->Duty_Test; INV->Duty_B = 0;              INV->Duty_C = INV->Duty_Test; break;
        default:INV->Duty_A = 0;              INV->Duty_B = 0;              INV->Duty_C = 0;              break;
    }
}

/**
 * @brief d축 전류 정렬 절차 (4단계 상태 머신)
 *
 * 엔코더/센서 초기화를 위해 d축 방향으로 전류를 인가하여 회전자를 정렬시킨다.
 *
 * | Mode_align | 동작                                   |
 * |------------|----------------------------------------|
 * | 0          | 초기화 (Id_ref = Idsr_align)           |
 * | 1          | CurrentControl 실행, 4초 대기          |
 * | 2          | Id_ref = 0으로 감쇠, 1초 더 대기       |
 * | 3          | 정렬 완료, FLAG.INV_ALIGN = 0          |
 *
 * @param[in,out] INV 인버터 구조체 포인터
 * @note Time_align은 Tsamp 단위로 매 호출마다 누산된다.
 */
void Align(struct INVERTER *INV)
{
    INV->Thetar = 0.;

    switch (Mode_align) {
        case 0:
            Align_done        = 0;
            INV->Thetar_offset = 0.;
            INV->Idsr_ref     = INV->Idsr_align;
            Time_align        = 0.;
            Mode_align++;
            break;
        case 1:
            CurrentControl(INV);
            PwmDutyUpt();
            PwmSwOn();
            if (!FLAG.FAULT) PWM_BUF_ON;
            if (Time_align >= 4.) Mode_align++;
            break;
        case 2:
            INV->Idsr_ref = 0.;
            CurrentControl(INV);
            PwmDutyUpt();
            PwmSwOn();
            if (!FLAG.FAULT) PWM_BUF_ON;
            if (Time_align >= 5.) Mode_align++;
            break;
        case 3:
            Mode_align     = 0;
            Align_done     = 1;
            FLAG.INV_ALIGN = 0;
            break;
        default:
            break;
    }
    Time_align += Tsamp;
}

/* ======================================================================
 * PWM 제어 함수
 * ====================================================================== */

/**
 * @brief TIM1 PWM 출력을 활성화한다 (3상 상하 암 모두).
 *
 * CH1~CH3 및 보완 채널(CH1N~CH3N)을 모두 시작한다.
 */
void PwmSwOn(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief TIM1 PWM 출력을 비활성화한다.
 *
 * 고장 발생 또는 정지 시 호출된다. PWM 버퍼를 먼저 끄고
 * 듀티를 0으로 설정한 뒤 모든 채널을 정지한다.
 *
 * @warning 이 함수 호출 즉시 인버터 출력이 차단된다.
 */
void PwmSwOff(void)
{
    PWM_BUF_OFF;
    INV.Duty_A = 0.f;
    INV.Duty_B = 0.f;
    INV.Duty_C = 0.f;

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief TIM1 CCR 레지스터에 듀티비를 업데이트한다.
 *
 * INV.Duty_A/B/C [0,1] 값을 TIM1 ARR(자동 리로드 값)에 곱하여
 * CCR1~CCR3에 직접 기록한다.
 */
void PwmDutyUpt(void)
{
    htim1.Instance->CCR1 = (unsigned int)(INV.Duty_A * htim1.Instance->ARR);
    htim1.Instance->CCR2 = (unsigned int)(INV.Duty_B * htim1.Instance->ARR);
    htim1.Instance->CCR3 = (unsigned int)(INV.Duty_C * htim1.Instance->ARR);
}
