/**
 * @file    GlobalVar.c
 * @author  lsj50
 * @date    Aug 25, 2025
 * @brief   전역 변수 정의 및 하드웨어 하위 수준(Low-level) 제어 구현 소스 파일
 * * @details [주요 역할 및 기능]
 * 시스템 클럭, 전동기 파라미터 초기화, 타이머 출력 제어(PWM Enable/Disable),
 * 부트스트랩 커패시터 충전 시퀀스 및 입력 캡처(IC) 인터럽트를 통한 외부 신호 분석 로직을 포함합니다.
 *
 * @details [전역 변수 그룹 (Global Variables)]
 * 여러 C 파일에서 외부 참조(`extern`)하여 사용하는 핵심 상태 변수들을 관리합니다.
 * | 변수 그룹 | 주요 변수명 | 설명 및 용도 |
 * | :--- | :--- | :--- |
 * | **시스템 및 시간** | `fSysClkFreq`, `fTsamp`, `fTSc` | CPU 클럭 주파수, 전류/속도 제어 샘플링 주기 |
 * | **전압 및 상태** | `fVdc`, `uBootStrapEnd`, `uControlMode` | DC 링크 전압, 부트스트랩 완료 상태, 현재 제어 모드 |
 * | **모터 구조체** | `INV` (`sMotorCtrl`) | 다축 확장을 고려한 전동기 제어 구조체 인스턴스 |
 *
 * @details [주요 제어 및 초기화 함수 (Functions)]
 * | 함수명 | 파라미터 / 대상 | 주요 동작 및 특징 |
 * | :--- | :--- | :--- |
 * | **vInintMotorParameter** | `sMotorCtrl*` | 구조체에 모터 파라미터(Ld, Lq, Rs, 극쌍수 등) 및 연산 최적화용 역수값 할당 |
 * | **vInitController** | - | 제어 모드 설정 및 속도/전류 제어기, 관측기 초기화 수행 |
 * | **vSwitchOnSettingTIM** | `TIM_HandleTypeDef*` | 게이트 드라이버 Enable 및 타이머 채널/MOE 활성화 (PWM 출력 시작) |
 * | **vSwitchOffSettingTIM** | `TIM_HandleTypeDef*` | 게이트 드라이버 Disable 및 MOE 차단 (Emergency Stop, 고장 시 즉시 차단) |
 * | **vBootstrapCharge** | `TIM_HandleTypeDef*` | 상측 스위치 구동을 위해 하측(N-ch) 스위치만 일정 듀티로 켜서 커패시터 충전 |
 * | **HAL_TIM_IC_Capture** | `TIM_HandleTypeDef*` | 외부 PWM 입력 신호의 주기/펄스폭을 캡처하여 주파수와 듀티(%) 계산 |
 * | **vEnableCycleCounter** | - | DWT(Data Watchpoint and Trace) 레지스터를 활성화하여 정밀한 연산 시간 측정 준비 |
 */

#include <stdint.h>
#include "GlobalVar.h"
#include "MotorControl.h"

/** @brief PWM 생성을 위한 메인 타이머 핸들러 참조 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
/* 시스템 운전 주파수 및 시간 관련 변수 */
float fSysClkFreq = 0.0f, fTimIntFreq = 0.0f;
float fLptClkFreq = 0.0f, fLptimIntFreq = 0.0f;
float fTsamp = 0.0f;         /**< 제어 샘플링 주기 */
float fTSc = 0.0f;           /**< 속도 제어 주기 */
float fInvVdc = 0.0f;        /**< DC-Link 전압의 역수 (연산 최적화) */
float fVdc = 0.0f;           /**< 현재 DC-Link 전압 [V] */

/* 부트스트랩 초기 충전 관련 변수 */
uint16_t uBootStrapEnd = 0u;      /**< 부트스트랩 충전 완료 플래그 */
uint16_t uBootStrapStepCnt = 0u;  /**< 부트스트랩 충전 시퀀스 단계 카운터 */
float fBootStrapDuty = 0.1f;      /**< 부트스트랩 충전 시 인가할 듀티 (10%) */

/* 시스템 상태 관리 변수 */
uint16_t uInterruptCnt = 0u;      /**< 인터럽트 발생 횟수 카운터 */
uint16_t uMainControl = 0u;       /**< 메인 제어 루프 상태 */
uint16_t uControlMode = 0u;       /**< 현재 제어 모드 (속도/전류 등) */
uint16_t uMaxCountSampHalf = 0u;  /**< 타이머 주기의 절반 값 (센터 정렬 PWM용) */

/* 모터 제어 구조체 인스턴스 (다축 제어 확장을 대비한 선언) */
sMotorCtrl INV;
sMotorCtrl MOT2;
sMotorCtrl MOT3;
sMotorCtrl MOT4;
sMotorCtrl MOT5;

/**
 * @brief  지령값의 급격한 변화를 방지하는 램프(Slope) 생성 함수
 * @param  fVar 현재 제어 변수의 포인터
 * @param  fCmd 목표 지령값
 * @param  fDelPerTs 샘플링 주기당 최대 변화 허용량
 * @retval 없음
 */
void vSlopeGenerator(float *fVar, float fCmd, float fDelPerTs) {
	if(*fVar < (fCmd - fDelPerTs))
		*(float *)fVar += fDelPerTs;
	else if(*fVar > (fCmd + fDelPerTs))
		*(float *)fVar -= fDelPerTs;
	else
		*(float *)fVar = fCmd;
}

/**
 * @brief  전동기 모델 기반 제어에 필요한 물리 파라미터를 구조체에 할당합니다.
 * @param  MotorControl 초기화할 모터 제어 구조체 포인터
 * @retval 없음
 */
void vInintMotorParameter(sMotorCtrl* MotorControl){
	MotorControl->LD = MOT_LD;
	MotorControl->LQ = MOT_LQ;
	MotorControl->RS = MOT_RS;
	MotorControl->PP = MOT_PP;
	MotorControl->LAMF = MOT_LAMF;
	MotorControl->JM = MOT_JM;
	MotorControl->BM = MOT_BM;
	MotorControl->KT = MOT_KT;

	/* 역수 및 파생 파라미터 계산 (실시간 연산 부하 감소) */
	MotorControl->fInvPP = 1. / MotorControl->PP;
	MotorControl->fInvJm = 1. / MotorControl->JM;
	MotorControl->fInvLamf = 1. / MotorControl->LAMF;
	MotorControl->InvKT = 1. / MotorControl->KT;
	MotorControl->ENC_PPR = ENCORDER_PPR;

	/* PWM 카운트 최대치 설정 */
	uMaxCountSampHalf = (uint16_t)((htim1.Instance->ARR >> 1) + 1u);
}

/**
 * @brief  전체 제어 시스템 초기화 (모터 파라미터, 각 제어기 초기화)
 * @retval 없음
 */
void vInitController(void){
	uControlMode = SPDCONTL_MODE; /**< 기본 제어 모드를 속도 제어로 설정 */

	vInintMotorParameter(&INV);
	//// MOTOR1 초기화 시퀀스 ////
	vInitCurrentControl(&INV, &INV.CC);
	vInitSpeedControl(&INV, &INV.SC);
	vInitSpeedObserver(&INV, &INV.SO);
}

/**
 * @brief  타이머의 모든 PWM 채널 출력을 활성화합니다.
 * @param  htim 제어할 타이머 핸들러 포인터
 * @retval 없음
 */
void vSwitchOnSettingTIM(TIM_HandleTypeDef *htim){
	PWM_ENABLE; /**< 하드웨어 게이트 드라이버 Enable */
	/* 채널 1, 2, 3 및 보조(N) 채널 출력 활성화 */
	htim->Instance->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE |
			TIM_CCER_CC2E | TIM_CCER_CC2NE |
			TIM_CCER_CC3E | TIM_CCER_CC3NE);

	/* Main Output Enable (MOE) 설정 */
	htim->Instance->BDTR |= TIM_BDTR_MOE;
}

/**
 * @brief  타이머의 모든 PWM 출력을 즉시 차단합니다 (Emergency Stop용).
 * @param  htim 제어할 타이머 핸들러 포인터
 * @retval 없음
 */
void vSwitchOffSettingTIM(TIM_HandleTypeDef *htim){
	PWM_DISABLE; /**< 하드웨어 게이트 드라이버 Disable */

	htim->Instance->BDTR &= ~TIM_BDTR_MOE; /**< MOE 차단 */

	/* 모든 출력 채널 비활성화 */
	htim->Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
			TIM_CCER_CC2E | TIM_CCER_CC2NE |
			TIM_CCER_CC3E | TIM_CCER_CC3NE);
}

/**
 * @brief  상측 게이트 드라이버 전원 공급용 부트스트랩 커패시터 충전 시퀀스
 * @details 하측 스위치만 일정 시간 On 하여 커패시터를 충전합니다.
 * @param  htim 제어할 타이머 핸들러 포인터
 * @retval 없음
 */
void vBootstrapCharge(TIM_HandleTypeDef *htim) {
	switch(uBootStrapStepCnt) {
	case 0:
		htim->Instance->BDTR &= ~(TIM_BDTR_MOE); /**< 초기 상태 출력 차단 */
		uBootStrapStepCnt++;
		break;

	case 1:
		/* 기존 채널 설정 클리어 */
		htim->Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E |
				TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

		/* 하측(N-channel) 스위치만 활성화하여 충전 경로 형성 */
		htim->Instance->CCER |= (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

		/* 설정된 듀티만큼 하측 스위치 On */
		htim->Instance->CCR1 = (uint32_t)(fBootStrapDuty * (float)htim->Instance->ARR);
		htim->Instance->CCR2 = (uint32_t)(fBootStrapDuty * (float)htim->Instance->ARR);
		htim->Instance->CCR3 = (uint32_t)(fBootStrapDuty * (float)htim->Instance->ARR);

		htim->Instance->BDTR |= TIM_BDTR_MOE; /**< 출력 개시 */

		uBootStrapStepCnt++;
		break;

	case 2:
	case 3:
		uBootStrapStepCnt++; /**< 충전 대기 시간 유지 */
		break;

	case 4:
		htim->Instance->BDTR &= ~(TIM_BDTR_MOE); /**< 충전 완료 후 차단 */

		uBootStrapStepCnt = 0u;
		uBootStrapEnd = 1u; /**< 충전 완료 플래그 셋 */

		/* 듀티 초기화 (Center-aligned 기준 50%) */
		htim->Instance->CCR1 = (unsigned int)(uMaxCountSampHalf * htim->Instance->ARR);
		htim->Instance->CCR2 = (unsigned int)(uMaxCountSampHalf * htim->Instance->ARR);
		htim->Instance->CCR3 = (unsigned int)(uMaxCountSampHalf * htim->Instance->ARR);
		break;

	default:
		break;
	}
}

/* 외부 신호(PWM 등) 캡처 결과 저장 변수 */
volatile uint32_t uFrequency = 0;           /**< 입력 신호 측정 주파수 [Hz] */
volatile float fDuty_Cycle = 0.0f;          /**< 입력 신호 측정 듀티 [%] */
volatile uint32_t uLast_Capture_Time = 0;   /**< 마지막 캡처 시점 (타임아웃 감지용) */

/**
 * @brief  타이머 입력 캡처 인터럽트 콜백 함수
 * @details 외부 PWM 신호의 주기(Period)와 펄스폭(Pulse Width)을 측정하여 주파수와 듀티를 계산합니다.
 * @param  htim 인터럽트가 발생한 타이머 핸들러
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        /* 채널 1에서 주기 읽기, 채널 2에서 펄스폭 읽기 (PWM Input Mode) */
        uint32_t uPeriod = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t uPulseWidth = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        if (uPeriod != 0)
        {
            /* 1MHz 타이머 클럭 기준 주파수 계산 */
            uFrequency = 1000000 / uPeriod;
            fDuty_Cycle = ((float)uPulseWidth / (float)uPeriod) * 100.0f;

            /* 생존 확인을 위한 타임스탬프 기록 */
            uLast_Capture_Time = HAL_GetTick();
        }
    }
}

/**
 * @brief  데이터 워치포인트 및 추적(DWT) 유닛을 사용하여 정밀한 사이클 카운터를 활성화합니다.
 * @details 코드의 실행 시간을 클럭 사이클 단위로 측정할 때 사용합니다.
 */
void vEnableCycleCounter(void){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /**< DWT 추적 기능 활성화 */
	DWT->CYCCNT = 0;                                /**< 사이클 카운터 초기화 */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /**< 사이클 카운트 시작 */
}

/**
 * @brief  증분형 엔코더 인터페이스 초기화 및 타이머 시작
 * @param  htim 엔코더 모드로 설정된 타이머 핸들러
 */
void vInitEncoder(TIM_HandleTypeDef *htim){
	HAL_TIM_Base_Start_IT(htim);                /**< 오버플로우 처리를 위한 인터럽트 시작 */
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL); /**< 엔코더 모드 활성화 */
}
