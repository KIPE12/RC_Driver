/**
 * @file    SpeedControl.c
 * @author  lsj50
 * @date    Sep 4, 2025
 * @brief   모터의 속도 제어(PI 제어) 및 토크 지령 생성 소스 파일
 * * @details [제어기 설계 및 튜닝 방식]
 * 목표 제어 대역폭(Bandwidth, Wc)과 모터의 물리적 파라미터(관성 모멘트 Jm 등)를 기반으로
 * PI 제어기의 비례 이득(Kp)과 적분 이득(Ki)을 자동 산출합니다.
 * 또한, 출력 포화(Saturation) 발생 시 적분기가 끝없이 누적되는 현상을 막기 위해
 * 역계산(Back-calculation) 기반의 Anti-windup 기법을 적용하여 제어 안정성을 높였습니다.
 *
 * @details [주요 함수 (Functions)]
 * | 함수명 | 주요 파라미터 | 역할 및 특징 |
 * | :--- | :--- | :--- |
 * | **vInitSpeedControl** | `sMotorCtrl*`, `sSpeedCtrl*` | 대역폭 기반 Kp, Ki, Ka 이득 산출 및 출력 토크(Te) 상/하한 물리적 한계치 설정 |
 * | **vSpeedControl** | `Motor`, `SObs`, `SCtrl` | 실시간 속도 PI 제어 연산 수행 및 최종 Q축 전류(Iq_ref) 지령 출력 |
 *
 * @details [속도 제어 루프 연산 흐름 (Control Flow)]
 * 속도 제어기 루프(`vSpeedControl`)는 아래의 4단계 파이프라인으로 실행됩니다.
 * | 단계 | 연산 과정 | 상세 설명 |
 * | :--- | :--- | :--- |
 * | **1. 지령 프로파일** | 비대칭 Ramp 적용 | 가속(1000)과 감속(500)의 기울기를 다르게 적용하여 급격한 변화 완화 및 rad/s 단위 변환 |
 * | **2. 오차 연산** | Error = Wrm_Ref - Wrm_SC | 기계적 각속도 지령값과 관측기(Observer) 피드백 속도 간의 오차 계산 |
 * | **3. PI & Anti-windup** | Te_Ref = Kp*Err + Integ | PI 연산을 통해 요구 토크 산출, 제한치(Limit) 초과 시 오차를 적분항에서 감산하여 Windup 방지 |
 * | **4. 전류 지령 변환** | Iq_Ref = Te_Ref / Kt | 산출된 최종 요구 토크에 토크 상수 역수(InvKT)를 곱하여 Q축 전류 지령으로 변환 |
 */

#include "GlobalVar.h"
#include "math.h"
#include "MotorControl.h"
#include "UserMath.h"

/**
 * @brief  속도 제어기(PI) 파라미터 및 변수들을 초기화합니다.
 * @details
 * 1. 제어 대역폭(WC_SC)과 모터의 관성 모멘트(JM)를 이용하여 비례 이득(Kp)과 적분 이득(Ki)을 자동 계산합니다.
 * 2. 제어기 포화 시 적분기 누적을 방지하기 위한 Anti-windup 이득(Ka)을 설정합니다.
 * 3. 모터의 극쌍수(PP), 쇄교자속(LAMF), 정격 전류(MOT_IS_RATED)를 기반으로 최대/최소 토크 제한값을 계산합니다.
 * @param  MotorControl 모터의 물리적 파라미터(관성, 극쌍수 등)를 포함하는 구조체 포인터
 * @param  SCtrl 초기화할 속도 제어기 구조체 포인터
 * @retval 없음
 */
void vInitSpeedControl(sMotorCtrl* MotorControl, sSpeedCtrl* SCtrl){

    SCtrl->fWrpmRefSet = 0.0f;
    SCtrl->fWrpmRef = 0.0f;

    /* 비례 이득 (Kp = Wc * Jm) 및 적분 이득 (Ki) 계산 */
    SCtrl->fKpSc = WC_SC* MotorControl->JM;
    SCtrl->fKiSc = 0.2f * WC_SC * WC_SC * MotorControl->JM;

    /* Anti-windup 이득 계산 (비례 이득의 역수 활용) */
    SCtrl->fKaSc = 1.f / SCtrl->fKpSc;

    SCtrl->fWrmRef = 0.0f;
    SCtrl->fWrmSC = 0.0f;

    /* 제어기 내부 변수 초기화 */
    SCtrl->fTeInteg = 0.0f;
    SCtrl->fTeRef = 0.0f;
    SCtrl->fTeRefAW = 0.0f;
    SCtrl->fTeRefUnsat = 0.0f;

    SCtrl->fIqsrRamp_LIMIT = 0.0f;
    SCtrl->fIqsrRefSC = 0.0f;

	/* 출력 토크(Te) 최대/최소 제한값 설정 (Te = 1.5 * P * Flux * Iq) */
	SCtrl-> fTeRefMin = -1.5 * MotorControl->PP * MotorControl->LAMF * (MOT_IS_RATED);
	SCtrl-> fTeRefMax = 1.5 * MotorControl->PP * MotorControl->LAMF * (MOT_IS_RATED);
}

/**
 * @brief  속도 PI 제어 알고리즘을 수행하고 최종 Q축 전류 지령(Iq_ref)을 생성합니다.
 * @details
 * 1. 속도 지령에 대한 슬로프(Ramp)를 생성하여 급격한 지령 변화를 완화합니다. (가속과 감속 기울기를 다르게 적용)
 * 2. RPM 단위를 기계적 각속도(rad/s)로 변환한 후 속도 오차를 계산합니다.
 * 3. Anti-windup이 적용된 PI 제어기를 통해 요구 토크(Te)를 계산하고 상하한치로 제한합니다.
 * 4. 제한된 최종 요구 토크를 토크 상수(KT)의 역수를 곱하여 Q축 전류 지령으로 변환합니다.
 * @param  MotorControl 모터 파라미터 구조체 포인터 (InvKT 등 참조용)
 * @param  SObs 속도 관측기 구조체 포인터 (현재 피드백 속도 참조용)
 * @param  SCtrl 속도 제어기 구조체 포인터 (지령 연산 및 상태 저장용)
 * @retval 없음
 */
void vSpeedControl(sMotorCtrl* MotorControl, sSpeedObs* SObs, sSpeedCtrl* SCtrl){

    /* 1. 속도 지령 프로파일 생성 (가속 및 감속 기울기 비대칭 적용) */
    if (SCtrl->fWrpmRefSet >= SCtrl->fWrpmRef) {
        vSlopeGenerator(&SCtrl->fWrpmRef, SCtrl->fWrpmRefSet, 1000.0f * fTSc);
    }else {
        vSlopeGenerator(&SCtrl->fWrpmRef, SCtrl->fWrpmRefSet, 0.5f * 1000.0f * fTSc);
    }

    /* RPM 지령 제한 및 기계적 각속도(rad/s)로 단위 변환 */
    SCtrl->fWrpmRef = LIMIT(SCtrl->fWrpmRef, -MOT_WRPM_RATED, MOT_WRPM_RATED);
    SCtrl->fWrmRef = RPM2RM * SCtrl->fWrpmRef;

    /* 현재 관측된 피드백 속도(RPM)를 기계적 각속도(rad/s)로 변환 */
    SCtrl->fWrmSC = RPM2RM * SObs->fWrpmSC;

    /* 2. 속도 오차 계산 */
    SCtrl->fErrWrm = SCtrl->fWrmRef - SCtrl->fWrmSC;

    /* 3. PI 제어기 연산 (적분항 누적 시 Anti-windup 보상 적용) */
    SCtrl->fTeInteg += (40.0f * fTsamp) * SCtrl->fKiSc * (SCtrl->fErrWrm - (SCtrl->fKaSc * SCtrl->fTeRefAW));

    /* 포화 전(Unsaturated) 토크 지령 산출 */
    SCtrl->fTeRefUnsat = SCtrl->fKpSc * SCtrl->fErrWrm + SCtrl->fTeInteg;

    /* 출력 토크 지령 상하한 제한(Saturation) */
    SCtrl->fTeRef = LIMIT(SCtrl->fTeRefUnsat, SCtrl->fTeRefMin, SCtrl->fTeRefMax);

    /* 4. Anti-windup을 위한 오차량 계산 (다음 주기의 적분항 보상용) */
    SCtrl->fTeRefAW = SCtrl->fTeRefUnsat - SCtrl->fTeRef;

    /* 5. 최종 요구 토크를 토크 상수의 역수를 이용하여 Q축 전류(Iq) 지령으로 변환 */
    SCtrl->fIqsrRefSC = MotorControl->InvKT * SCtrl->fTeRef;
}
