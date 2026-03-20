/**
 * @file    SpeedControl.h
 * @author  lsj50
 * @date    Sep 4, 2025
 * @brief   전동기 속도 제어기(ASR, Adjustable Speed Regulator) 정의 헤더 파일
 * @details 속도 PI 제어를 위한 구조체와 차단 주파수 상수를 포함하며,
 * 피드백된 속도와 지령 속도 사이의 오차를 바탕으로 토크(q축 전류) 지령을 생성합니다.
 */

#ifndef INC_SPEEDCONTROL_H_
#define INC_SPEEDCONTROL_H_

/** @brief 속도 제어기 차단 주파수 (Bandwidth): 5Hz를 Radian 단위로 변환 */
#define WC_SC	(5.0f * PI2)

/**
 * @struct sSpeedCtrl
 * @brief  속도 제어 루프의 상태 변수 및 이득을 관리하는 구조체
 */
typedef struct {
	// 1. 속도 지령 및 램프 (Speed Reference & Ramp)
	float fWrpmRefSet;      /**< 사용자 또는 상위 제어기에서 설정한 목표 RPM */
	float fWrpmRef;         /**< 가속/감속 램프가 적용된 실제 지령 RPM */
	float fWrmRef;          /**< 기계각 속도 지령 [rad/s] */
	float fWrmSC;           /**< 속도 제어기 입력으로 사용되는 피드백 속도 [rad/s] */
	float fErrWrm;          /**< 속도 오차 (Reference - Feedback) [rad/s] */

	// 2. 제어기 이득 (Controller Gains)
	float fKpSc;            /**< 속도 제어기 비례 이득 (Proportional Gain) */
	float fKiSc;            /**< 속도 제어기 적분 이득 (Integral Gain) */
	float fKaSc;            /**< Anti-windup 이득 (적분기 포화 방지용) */

	// 3. 토크 및 적분기 상태 (Torque & Integrator State)
	float fTeInteg;         /**< 속도 제어기 적분항 누적 상태 값 */
	float fTeRefUnsat;      /**< 제한(Saturation) 전의 토크 지령값 */
	float fTeRef;           /**< 최종 제한이 적용된 출력 토크 지령값 */
	float fTeRefAW;         /**< Anti-windup 계산을 위한 지령치와 실제 출력의 차분 */

	// 4. 출력 및 제한 (Output & Limits)
	volatile float fIqsrRefSC;       /**< 속도 제어기 출력인 q축 전류 지령값 (토크 성분) */
	float fIqsrRamp_LIMIT;  /**< q축 전류의 급격한 변화를 막기 위한 램프 제한치 */
	float fTeRefMax;        /**< 출력 토크의 최대 제한치 */
	float fTeRefMin;        /**< 출력 토크의 최소 제한치 */

} sSpeedCtrl;



/* === 함수 프로토타입 === */
/* 필요 시 vInitSpeedControl, vSpeedControl 등의 프로토타입을 여기에 추가할 수 있습니다. */



#endif /* INC_SPEEDCONTROL_H_ */
