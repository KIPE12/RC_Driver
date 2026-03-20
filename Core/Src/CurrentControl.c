/**
 * @file    CurrentControl.c
 * @author  lsj50
 * @date    Sep 4, 2025
 * @brief   전동기 벡터 제어를 위한 전류 제어기(PI) 및 전압 변조(SVPWM) 로직 구현 소스 파일
 */

#include "MotorControl.h"
#include "UserMath.h"
#include "GlobalVar.h"
#include "Adc.h"
#include "math.h"

/** @brief DQ축 전압 지령 설정을 위한 전역 변수 (V/f 제어 등에서 사용) */
float fVdqsrRefSet = 0.0f;

/**
 * @brief  입력값을 주어진 최소값과 최대값 사이로 제한하는 인라인 함수
 * @param  val 입력값
 * @param  min 최소값
 * @param  max 최대값
 * @retval 제한된 결과값
 */
static inline float LIMIT_OPT(float val, float min, float max) {
	return fminf(max, fmaxf(min, val));
}

/**
 * @brief  전류 제어기 구조체 및 관련 변수들을 초기화합니다.
 * @note   제어기 이득(Gain) 계산 시 모터의 파라미터(L, R)와 차단 주파수(WC_CC)를 사용합니다.
 * @param  MotorControl 모터 파라미터 정보를 담고 있는 구조체 포인터
 * @param  CCtrl 초기화할 전류 제어기 구조체 포인터
 * @retval 없음
 */
void vInitCurrentControl(sMotorCtrl* MotorControl, sCurrentCtrl* CCtrl){
	/* 전류 및 전압 변수 초기화 */
	CCtrl->fIasHall = 0.0f;
	CCtrl->fIbsHall = 0.0f;
	CCtrl->fIcsHall = 0.0f;

	CCtrl->fVas = 0.0f; CCtrl->fVbs = 0.0f; CCtrl->fVcs = 0.0f;
	CCtrl->fVan = 0.0f; CCtrl->fVbn = 0.0f; CCtrl->fVcn = 0.0f;

	/* 좌표 변환 관련 변수 초기화 */
	CCtrl->fIdss = 0.0f; CCtrl->fIqss = 0.0f;
	CCtrl->fIdsr = 0.0f; CCtrl->fIqsr = 0.0f;

	/* 제어 오차 및 지령값 초기화 */
	CCtrl->fIdsrErr = 0.0f; CCtrl->fIqsrErr = 0.0f;
	CCtrl->fIdsrFF = 0.0f;  CCtrl->fIqsrFF = 0.0f;
	CCtrl->fIdsrRef = 0.0f; CCtrl->fIqsrRef = 0.0f;
	CCtrl->fIdsrRefSet = 0.0f; CCtrl->fIqsrRefSet = 0.0f; CCtrl->fIdqrRefSet = 0.0f;
	CCtrl->fIdsrRefMax = 0.0f; CCtrl->fIqsrRefMax = 0.0f;

	/* D축 전류 제어기 이득 설정 (Kpd = Ld * Wc, Kid = Rs * Wc) */
	CCtrl->fKpdCc = MotorControl->LD * WC_CC;
	CCtrl->fKidCc = MotorControl->RS * WC_CC;

	/* D축 Anti-windup 계수 계산 */
	if (CCtrl->fKpdCc > 0.0f) {
		CCtrl->fKadCc = 1.0f / CCtrl->fKpdCc;
	} else {
		CCtrl->fKadCc = 0.0f;
	}

	/* Q축 전류 제어기 이득 설정 (Kpq = Lq * Wc, Kiq = Rs * Wc) */
	CCtrl->fKpqCc = MotorControl->LQ * WC_CC;
	CCtrl->fKiqCc = MotorControl->RS * WC_CC;

	/* Q축 Anti-windup 계수 계산 */
	if (CCtrl->fKpqCc > 0.0f) {
		CCtrl->fKaqCc = 1.0f / CCtrl->fKpqCc;
	} else {
		CCtrl->fKaqCc = 0.0f;
	}

	/* 적분항 및 전압 출력 변수 초기화 */
	CCtrl->fIdsrInteg = 0.0f; CCtrl->fIqsrInteg = 0.0f;
	CCtrl->fVdsrRef = 0.0f;   CCtrl->fVqsrRef = 0.0f;
	CCtrl->fVdsrAwRef = 0.0f; CCtrl->fVqsrAwRef = 0.0f;
	CCtrl->fVdsrOut = 0.0f;   CCtrl->fVqsrOut = 0.0f;

	/* 전압 변조 관련 변수 초기화 */
	CCtrl->fVdssRef = 0.0f; CCtrl->fVqssRef = 0.0f;
	CCtrl->fVdssOut = 0.0f; CCtrl->fVqssOut = 0.0f;
	CCtrl->fVasRef = 0.0f;  CCtrl->fVbsRef = 0.0f; CCtrl->fVcsRef = 0.0f;
	CCtrl->fVanRef = 0.0f;  CCtrl->fVbnRef = 0.0f; CCtrl->fVcnRef = 0.0f;
	CCtrl->fVsRefMax = 0.0f; CCtrl->fVsRefMin = 0.0f;
	CCtrl->fVsnOffset = 0.0f;

	/* 듀티 및 약자속 제어 관련 변수 초기화 */
	CCtrl->fDutyA = 0.0f; CCtrl->fDutyB = 0.0f; CCtrl->fDutyC = 0.0f;
	CCtrl->fVanOut = 0.0f; CCtrl->fVbnOut = 0.0f; CCtrl->fVcnOut = 0.0f;
	CCtrl->fVmagErr = 0.0f; CCtrl->fVdqsrOutMag = 0.0f;
	CCtrl->fDelIdsrRefFWAW = 0.0f; CCtrl->fDelIdsrRefFWInteg = 0.0f;
	CCtrl->fDelIdsrRefFWUnsat = 0.0f; CCtrl->fDelIdsrRefFW = 0.0f;

	CCtrl->fDutyA_Test = 0.0f; CCtrl->fDutyB_Test = 0.0f; CCtrl->fDutyC_Test = 0.0f;

	/* 약자속(Field Weakening) 제어 계수 설정 */
	if (KP_FW > 0.0f) {
		CCtrl->fKaFW = 1.0f / KP_FW;
	} else {
		CCtrl->fKaFW = 0.0f;
	}

	CCtrl->fBetaAngleRad = 0.0f;
	CCtrl->fBetaAngle = 0.0f;

	fVdqsrRefSet = 0.0f;
}

/**
 * @brief  현재 운전 모드에 따라 전류 지령값(Id, Iq)을 생성합니다.
 * @details
 * - CONST_CUR_MODE: 슬로프 생성기를 통한 전류 지령 추종
 * - VECTCONTL_MODE: 외부 설정된 지령값에 대해 슬로프 적용
 * - SPDCONTL_MODE: 속도 제어기 출력값을 Q축 전류 지령으로 사용
 * @param  CCtrl 전류 제어 구조체 포인터
 * @param  SCtrl 속도 제어 구조체 포인터
 * @retval 없음
 */
void vCurrentRef(sCurrentCtrl* CCtrl, sSpeedCtrl* SCtrl){
	switch (uControlMode){
	case CONST_CUR_MODE:
		CCtrl->fIqsrRefSet = 0.0f;
		vSlopeGenerator(&CCtrl->fIdsrRef, CCtrl->fIdsrRefSet, 500.0f * fTsamp);
		vSlopeGenerator(&CCtrl->fIdsrRef, CCtrl->fIqsrRefSet, 500.0f * fTsamp);
		break;

	case VECTCONTL_MODE:
		vSlopeGenerator(&CCtrl->fIdsrRef, CCtrl->fIdsrRefSet, 10.0f *fTsamp);
		vSlopeGenerator(&CCtrl->fIqsrRef, CCtrl->fIqsrRefSet, 10.0f * fTsamp);
		break;

	case SPDCONTL_MODE:
		CCtrl->fIdsrRef = 0.0f;
		CCtrl->fIqsrRef = SCtrl->fIqsrRefSC;
		break;

	default:	// DUTY_TEST_MODE, CONST_VOLT_MODE 포함
		CCtrl->fIdsrRefSet = 0.0f;
		CCtrl->fIqsrRefSet = 0.0f;
		CCtrl->fIdsrRef = CCtrl->fIdsrRefSet;
		CCtrl->fIqsrRef = CCtrl->fIqsrRefSet;
		break;
	}
}

/**
 * @brief  동기 좌표계 PI 전류 제어를 수행합니다.
 * @details
 * 1. Anti-windup 계산
 * 2. Clark/Park 변환 (정지 좌표계 -> 동기 좌표계)
 * 3. 전류 오차 계산 및 PI 제어 (전향 보상항 포함 가능)
 * 4. 적분항 업데이트
 * @param  CCtrl 전류 제어 구조체 포인터
 * @param  SObs 속도 및 위치 관측기 구조체 포인터
 * @retval 없음
 */
void vCurrentControl(sCurrentCtrl* CCtrl, sSpeedObs* SObs){

	/* Anti-windup 항 계산: 지령 전압과 실제 출력 전압의 차이에 비례 이득의 역수를 곱함 */
	CCtrl->fVdsrAwRef = CCtrl->fKadCc * (CCtrl->fVdsrRef - CCtrl->fVdsrOut);
	CCtrl->fVqsrAwRef = CCtrl->fKaqCc * (CCtrl->fVqsrRef - CCtrl->fVqsrOut);

	/* Clarke Transformation (3-phase to 2-phase stationary) */
	CCtrl->fIdss = CCtrl->fIasHall;
	CCtrl->fIqss = INV_SQRT3 * (CCtrl->fIbsHall -  CCtrl->fIcsHall);

	/* Park Transformation (Stationary to Synchronous frame) */
	CCtrl->fIdsr = CCtrl->fIdss * SObs->fCosThetarCC + CCtrl->fIqss * SObs->fSinThetarCC;
	CCtrl->fIqsr = -CCtrl->fIdss * SObs->fSinThetarCC + CCtrl->fIqss * SObs->fCosThetarCC;

	/* 전류 오차 계산 */
	CCtrl->fIdsrErr = CCtrl->fIdsrRef - CCtrl->fIdsr;
	CCtrl->fIqsrErr = CCtrl->fIqsrRef - CCtrl->fIqsr;

	CCtrl->fIdsrFF = 0.0f;
	CCtrl->fIqsrFF = 0.0f;

	/* PI 제어기 적분항 업데이트 (Anti-windup 고려) */
	CCtrl->fIdsrInteg += fTsamp * CCtrl->fKidCc * (CCtrl->fIdsrErr - CCtrl->fVdsrAwRef);
	CCtrl->fIqsrInteg += fTsamp * CCtrl->fKiqCc * (CCtrl->fIqsrErr - CCtrl->fVqsrAwRef);

	/* 최종 동기 좌표계 전압 지령 계산 */
	CCtrl->fVdsrRef = CCtrl->fKpdCc * CCtrl->fIdsrErr + CCtrl->fIdsrInteg + CCtrl->fIdsrFF;
	CCtrl->fVqsrRef = CCtrl->fKpqCc * CCtrl->fIqsrErr + CCtrl->fIqsrInteg + CCtrl->fIqsrFF;
}

/**
 * @brief  전압 지령을 기반으로 PWM 듀티를 계산하고 타이머 레지스터를 업데이트합니다.
 * @details
 * - 역 Park 변환을 통해 정지 좌표계 전압 생성
 * - Offset Addition 방식을 사용하여 SVPWM 효과 구현
 * - 계산된 듀티를 타이머의 CCR(Capture Compare Register)에 반영
 * - 다음 연산을 위해 실제 출력 전압을 재구성(Reconstruction)
 * @param  htim 타이머 핸들러 포인터 (PWM 출력용)
 * @param  CCtrl 전류 제어 구조체 포인터
 * @param  SObs 속도 및 위치 관측기 구조체 포인터
 * @retval 없음
 */
void vVoltageModulationTIM(TIM_HandleTypeDef *htim, sCurrentCtrl *CCtrl, sSpeedObs* SObs){

	/* V/f 운전 모드 처리 */
	if(uControlMode == CONST_VOLT_MODE){
		CCtrl->fVdsrRef = fVdqsrRefSet;
		CCtrl->fVqsrRef = 0.0f;
	}

	/* Inverse Park Transformation (Sync to Stationary) */
	CCtrl->fVdssRef = CCtrl->fVdsrRef * SObs->fCosThetarCC - CCtrl->fVqsrRef * SObs->fSinThetarCC;
	CCtrl->fVqssRef = CCtrl->fVdsrRef * SObs->fSinThetarCC + CCtrl->fVqsrRef * SObs->fCosThetarCC;

	/* Inverse Clarke Transformation */
	CCtrl->fVasRef = CCtrl->fVdssRef;
	CCtrl->fVbsRef = 0.5f * (-CCtrl->fVdssRef + SQRT3 * CCtrl->fVqssRef);
	CCtrl->fVcsRef = -0.5f * (CCtrl->fVdssRef + SQRT3 * CCtrl->fVqssRef);

	/* SVPWM을 위한 Offset 전압 계산 (Min-Max Injection) */
	CCtrl->fVsRefMax = MAX(MAX(CCtrl->fVasRef, CCtrl->fVbsRef), CCtrl->fVcsRef);
	CCtrl->fVsRefMin = MIN(MIN(CCtrl->fVasRef, CCtrl->fVbsRef), CCtrl->fVcsRef);
	CCtrl->fVsnOffset = -0.5f * (CCtrl->fVsRefMax + CCtrl->fVsRefMin);

	/* 최종 3상 상전압 지령 */
	CCtrl->fVanRef = CCtrl->fVasRef + CCtrl->fVsnOffset;
	CCtrl->fVbnRef = CCtrl->fVbsRef + CCtrl->fVsnOffset;
	CCtrl->fVcnRef = CCtrl->fVcsRef + CCtrl->fVsnOffset;

	/* 전압 지령을 듀티비(0~1)로 변환 및 제한 */
	CCtrl->fDutyA = LIMIT(fInvVdc * CCtrl->fVanRef + 0.5f, 0.0f, 0.95f);
	CCtrl->fDutyB = LIMIT(fInvVdc * CCtrl->fVbnRef + 0.5f, 0.0f, 0.95f);
	CCtrl->fDutyC = LIMIT(fInvVdc * CCtrl->fVcnRef + 0.5f, 0.0f, 0.95f);

	/* 타이머 CCR 레지스터 업데이트 */
	if(uControlMode == DUTY_TEST_MODE){
		htim->Instance->CCR1 = (unsigned int)(CCtrl->fDutyA_Test * htim->Instance->ARR);
		htim->Instance->CCR2 = (unsigned int)(CCtrl->fDutyB_Test * htim->Instance->ARR);
		htim->Instance->CCR3 = (unsigned int)(CCtrl->fDutyC_Test * htim->Instance->ARR);
	}else{
		htim->Instance->CCR1 = (unsigned int)(CCtrl->fDutyA * htim->Instance->ARR);
		htim->Instance->CCR2 = (unsigned int)(CCtrl->fDutyB * htim->Instance->ARR);
		htim->Instance->CCR3 = (unsigned int)(CCtrl->fDutyC * htim->Instance->ARR);
	}

	/* 다음 샘플링 시 제어기 피드백용 전압 출력 재구성 */
	CCtrl->fVanOut = (CCtrl->fDutyA - 0.5f) * fVdc;
	CCtrl->fVbnOut = (CCtrl->fDutyB - 0.5f) * fVdc;
	CCtrl->fVcnOut = (CCtrl->fDutyC - 0.5f) * fVdc;

	/* 실제 출력 전압을 다시 정지/동기 좌표계로 변환 */
	CCtrl->fVdssOut = INV3 * (2.0f * CCtrl->fVanOut - CCtrl->fVbnOut - CCtrl->fVcnOut);
	CCtrl->fVqssOut = INV_SQRT3 * (CCtrl->fVbnOut - CCtrl->fVcnOut);

	CCtrl->fVdsrOut = CCtrl->fVdssOut * SObs->fCosThetarCompCC + CCtrl->fVqssOut * SObs->fSinThetarCompCC;
	CCtrl->fVqsrOut = -CCtrl->fVdssOut * SObs->fSinThetarCompCC + CCtrl->fVqssOut * SObs->fCosThetarCompCC;

	/* 출력 전압 벡터 크기 계산 */
	CCtrl->fVdqsrOutMag = __builtin_sqrtf(CCtrl->fVdsrOut * CCtrl->fVdsrOut + CCtrl->fVqsrOut * CCtrl->fVqsrOut);
}
