/**
 * @file    MotorControl.h
 * @author  lsj50
 * @date    Sep 4, 2025
 * @brief   전동기 제어 시스템의 통합 관리 및 구조체 정의 헤더 파일
 * @details ADC, 전류 제어, 속도 제어, 관측기 등 모든 제어 모듈을 포함하는
 * sMotorCtrl 구조체를 정의하며, 제어 루프와 관련된 주요 함수들을 선언합니다.
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_
#include <stdint.h>

#include "stm32g4xx_hal.h"

#include "Adc.h"
#include "SpeedObserver.h"
#include "CurrentControl.h"
#include "SpeedControl.h"
#include "Filter.h"
#include "Fault.h"

/**
 * @struct sMotorCtrl
 * @brief  전동기 제어에 필요한 모든 데이터와 파라미터를 통합 관리하는 구조체
 */
typedef struct {
	sADC1Meas 	 ADC1Meas;	   /**< ADC 측정 데이터 및 오프셋 정보 */
	sSpeedObs	 SO;		   /**< 속도 및 위치 관측기 상태 변수 */
	sCurrentCtrl CC;           /**< 전류 제어기(PI) 및 SVPWM 변수 */
	sSpeedCtrl   SC;           /**< 속도 제어기(PI) 변수 */
	sFault_Info  Fault_Info;   /**< 결함(Fault) 발생 시 저장되는 시스템 상태 정보 */

	// === 모터 파라미터 ===
	float PP;       /**< 극쌍수 (Pole Pairs) */
	float RS;       /**< 상저항 (Stator resistance) [Ω] */
	float LD;       /**< d축 인덕턴스 [H] */
	float LQ;       /**< q축 인덕턴스 [H] */
	float LAMF;     /**< 영구자석 자속 쇄교수 (Flux Linkage) [Wb] */
	float JM;       /**< 회전자 관성 (Inertia) [kg·m^2] */
	float BM;       /**< 점성 마찰 계수 (Viscous friction) */
	float KT;       /**< 토크 상수 (Torque constant) [Nm/A] */
	float InvKT;    /**< 토크 상수의 역수 (1/KT) */

	float fInvPP;   /**< 극쌍수의 역수 (1/PP) */
	float fInvJm;   /**< 관성의 역수 (1/Jm) */
	float ENC_PPR;  /**< 엔코더 분해능 (Pulse Per Revolution) */

	float fInvLamf; /**< 자속 쇄교수의 역수 (1/LAMF) */


	float IS_RATED;     /**< 정격 전류 [A] */
	float WRPM_RATED;   /**< 정격 속도 [rpm] */
} sMotorCtrl;

/** @brief 전역 전동기 제어 객체 외칭 참조 */
extern sMotorCtrl INV;

/**
 * @struct Flag_Reg
 * @brief  시스템 동작 제어를 위한 플래그 레지스터 구조체
 */
typedef struct _FLAG_{
	uint16_t START; /**< 인버터 구동 시작 플래그 */
	uint16_t RESET; /**< 시스템 리셋 플래그 */
} FLAG_REG;

/** @brief 전역 플래그 객체 외칭 참조 */
extern FLAG_REG Flag;

/**
 * @brief  모터의 물리적 파라미터(R, L, J 등)를 초기화합니다.
 * @param  MotorControl 모터 제어 구조체 포인터
 */
void vInintMotorParameter(sMotorCtrl* MotorControl);


/* --- 전류 제어 관련 함수 --- */
/**
 * @brief  전류 제어기(PI)를 초기화합니다.
 */
void vInitCurrentControl(sMotorCtrl* MotorControl, sCurrentCtrl* CCtrl);
/**
 * @brief  운전 모드에 따른 전류 지령을 생성합니다.
 */
void vCurrentRef(sCurrentCtrl* CCtrl, sSpeedCtrl* SCtrl);
/**
 * @brief  동기 좌표계 PI 전류 제어를 수행합니다.
 */
void vCurrentControl(sCurrentCtrl* CCtrl, sSpeedObs* SObs);


/* --- 속도 제어 관련 함수 --- */
/**
 * @brief  속도 제어기(PI)를 초기화합니다.
 */
void vInitSpeedControl(sMotorCtrl* MotorControl, sSpeedCtrl* SCtrl);
/**
 * @brief  속도 PI 제어를 수행하여 전류 지령을 출력합니다.
 */
void vSpeedControl(sMotorCtrl* MotorControl, sSpeedObs* SObs, sSpeedCtrl* SCtrl);


/* --- 전압 변조 관련 함수 --- */
/**
 * @brief  전압 지령을 기반으로 PWM 타이머의 듀티를 업데이트합니다.
 */
extern void vVoltageModulationTIM(TIM_HandleTypeDef *htim, sCurrentCtrl *CCtrl, sSpeedObs* SObs);


/* --- 관측기 및 위치 센서 관련 함수 --- */
/**
 * @brief  속도 및 위치 관측기를 초기화합니다.
 */
void vInitSpeedObserver(sMotorCtrl* MotorContorl, sSpeedObs* SObs);
/**
 * @brief  모터 정렬(Align) 동작을 수행합니다.
 */
void vAlign(sCurrentCtrl* CCtrl, sSpeedObs *SObs, TIM_TypeDef *TIMx);
/**
 * @brief  홀 센서 기반의 위치 정렬을 수행합니다.
 */
void vAlignHallSensor(sCurrentCtrl* CCtrl, sSpeedObs *SObs);
/**
 * @brief  모터의 속도 및 위치를 추정/계산합니다.
 */
void vSpeedObserver(sMotorCtrl* MotorControl, sSpeedObs* SObs, sSpeedCtrl* SCtrl);
/**
 * @brief  증분형 엔코더 정보를 읽어 속도/위치를 계산합니다.
 */
float fGetEncoderInfo(TIM_HandleTypeDef *htim, sSpeedObs* SObs);
/**
 * @brief  홀 센서 신호를 처리하여 전기적 위치 정보를 반환합니다.
 */
float fGetHallSensorInfo(sSpeedObs* SObs);


/* --- 최상위 제어 루프 함수 --- */
/** @brief  고속 제어 루프 (전류 제어, SVPWM 등) */
extern void vControl();
/** @brief  저속 제어 루프 (속도 제어, 통신 등) */
extern void vLowSpdControl();


/* --- 보호 및 결함 관리 함수 --- */
/** @brief  소프트웨어 결함 발생 시 차단 동작을 수행합니다. */
extern void vSWFaultOperation();
/** @brief  발생한 Fault 상태를 클리어하고 초기화합니다. */
extern void vClearFault();
/**
 * @brief  결함 발생 시 현재 시스템 상태를 기록합니다.
 * @param  MotorControl 모터 제어 구조체
 * @param  Fault_Infomation 정보를 저장할 Fault 구조체
 */
extern void vFaultEvent(sMotorCtrl* MotorControl, sFault_Info* Fault_Infomation);

#endif /* INC_MOTORCONTROL_H_ */
