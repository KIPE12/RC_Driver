/**
 * @file    GlobalVar.h
 * @author  lsj50
 * @date    Aug 25, 2025
 * @brief   인버터 및 전동기 제어 시스템의 전역 변수 및 상수 정의 헤더 파일
 * @details 시스템 보호 레벨, 제어 상태(State), 모터 파라미터 및 하드웨어 매크로를 포함합니다.
 */

#ifndef INC_GLOBALVAR_H_
#define INC_GLOBALVAR_H_


#include <stdint.h>
#include "stm32g4xx_hal.h"

/** @name 시스템 보호 임계치 (Fault Levels)
 * @{ */
#define CURR_FAULT_LEV      12.0f       /**< 과전류 차단 레벨 [A] */
#define VDC_FAULT_LEV       17.0f       /**< 직류단 과전압 차단 레벨 [V] */
#define SPD_FAULT_LEV		10000.0f    /**< 과속도 차단 레벨 [RPM] */
/** @} */

/** @name 시스템 주 상태 제어 (System States)
 * @{ */
#define IDLE_STATE			0u          /**< 대기 상태 */
#define ALIGN_STATE			1u          /**< 위치 정렬(Align) 상태 */
#define RUN_STATE			2u          /**< 운전 상태 */
#define FAULT_STATE			3u          /**< 결함 발생 상태 */
/** @} */

/** @name 애플리케이션 타입 정의 */
#define GEAR_HEAD 0u
#define ROBOT_HAND 1u

/** @name 하드웨어 직접 제어 매크로 (Gate Driver Enable/Disable)
 * @details GPIOC PIN 13을 사용하여 PWM 출력을 물리적으로 차단하거나 허용합니다.
 * @{ */
#define PWM_ENABLE  GPIOC->BSRR = (uint32_t)GPIO_PIN_13 << 16   /**< PWM 출력 활성화 (Low-active 가정 시) */
#define PWM_DISABLE GPIOC->BSRR = GPIO_PIN_13                   /**< PWM 출력 비활성화 (High-z or Low) */
/** @} */


/* 시스템 운전 플래그 및 주파수 관련 외부 변수 */
extern uint16_t uFlag_Start, uFlag_Reset;       /**< 시작 및 리셋 플래그 */
extern float fSysClkFreq, fTimIntFreq;          /**< 시스템 클럭 및 타이머 인터럽트 주파수 */
extern float fLptClkFreq, fLptimIntFreq;        /**< 저전력 타이머 클럭 및 주파수 */
extern float fTsamp, fTSc;                      /**< 전류 제어 및 속도 제어 샘플링 주기 [s] */
extern uint16_t uInterruptCnt, uMainControl, uLSCnt; /**< 각종 카운터 변수 */

extern float fVdc;                              /**< 현재 측정된 직류단 전압 [V] */
extern float fInvVdc;                           /**< 직류단 전압의 역수 (연산 최적화용) */

extern float fDutyTest1, fDutyTest2, fDutyTest3; /**< 듀티 테스트용 변수 */

extern uint16_t uControlMode;                   /**< 현재 제어 모드 (속도, 전류 등) */
extern uint16_t uMaxCountSampHalf;              /**< PWM 샘플링 관련 카운트 값 */
extern uint16_t uBootStrapEnd;                  /**< 부트스트랩 충전 완료 여부 */


/** @name Motor Parameters (전동기 물리 파라미터)
 * @{ */
#define MOT_PP			2.                      /**< 극쌍수 (Pole Pairs) */
#define MOT_RS			19e-3f                  /**< 상저항 (Stator Resistance) [Ohm] */
#define MOT_LD			(3.2e-6f)               /**< d축 인덕턴스 [H] */
#define MOT_LQ			(3.2e-6f)               /**< q축 인덕턴스 [H] */
#define MOT_LAMF		(2e-3)                  /**< 영구자석 자속 쇄교수 (Flux Linkage) [Wb] */
#define MOT_KT			(1.5f * MOT_PP * MOT_LAMF) /**< 토크 상수 (Torque Constant) */
#define MOT_JM  		1.e-6f                  /**< 회전자 관성 (Inertia) [kg*m^2] */
#define MOT_BM  		1.e-3f                  /**< 점성 마찰 계수 (Viscous Friction) */
#define INV_MOT_JM		(1.f/MOT_JM)            /**< 관성 역수 (가속도 계산용) */
#define MOT_IS_RATED	(10.0f)                 /**< 정격 전류 [A] */
#define MOT_WRPM_RATED	(10000.0f)              /**< 정격 속도 [RPM] */

#define ENCORDER_PPR	2000.0f                 /**< 엔코더 Pulse Per Revolution */
#define FSAMP_PC		1.e3f                   /**< PC 통신/모니터링 샘플링 주파수 */
/** @} */


/** @name Control Modes (제어 모드 정의)
 * @{ */
#define CONST_CUR_MODE				0u          /**< 고정 전류 제어 모드 */
#define VECTCONTL_MODE				1u          /**< 벡터 제어(전류) 모드 */
#define SPDCONTL_MODE				2u          /**< 속도 제어 모드 */

#define DUTY_TEST_MODE				3u          /**< PWM 듀티 테스트 모드 */
#define CONST_VOLT_MODE				4u          /**< V/f 제어(고정 전압) 모드 */
#define ALIGN_MODE					5u          /**< 위치 정렬 모드 */
/** @} */

/**
 * @brief  전체 제어기 변수 및 상태를 초기화합니다.
 */
extern void vInitController();

/**
 * @brief  지령값의 급격한 변화를 방지하기 위해 기울기(Slope)를 생성합니다.
 * @param  fVar 현재 값의 포인터
 * @param  fCmd 목표 지령값
 * @param  fDelPerTs 샘플링 주기당 변화량 제한치
 */
extern void vSlopeGenerator(float* fVar, float fCmd, float fDelPerTs);

/**
 * @brief  상측(High-side) 게이트 드라이버 구동을 위한 부트스트랩 커패시터를 충전합니다.
 * @param  htim 제어용 타이머 핸들러
 */
extern void vBootstrapCharge(TIM_HandleTypeDef *htim);

/** @brief  CPU 사이클 카운터를 활성화합니다. (성능 측정용) */
extern void vEnableCycleCounter(void);

/** @brief  타이머의 스위칭 출력을 On 설정으로 변경합니다. */
extern void vSwitchOnSettingTIM(TIM_HandleTypeDef *htim);

/** @brief  타이머의 스위칭 출력을 Off 설정(차단)으로 변경합니다. */
extern void vSwitchOffSettingTIM(TIM_HandleTypeDef *htim);

/** @name Fault 관련 플래그 */
extern uint16_t SW_Fault, TZ_Fault;             /**< 소프트웨어적 결함 및 하드웨어 트리거(Trip Zone) 결함 플래그 */

/* 엔코더 및 속도 측정 관련 변수 */
extern volatile uint32_t uFrequency, uLast_Capture_Time;
extern volatile float fDuty_Cycle;

/**
 * @brief  엔코더 인터페이스를 초기화합니다.
 * @param  htim 엔코더 입력용 타이머 핸들러
 */
extern void vInitEncoder(TIM_HandleTypeDef *htim);

/**
 * @brief  DAC 출력을 위한 설정을 초기화합니다.
 * @param  htim 관련 타이머 핸들러
 */
extern void vInitDac(TIM_HandleTypeDef *htim);



#endif /* INC_GLOBALVAR_H_ */
