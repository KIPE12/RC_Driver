/**
 * @file    CurrentControl.h
 * @author  lsj50
 * @date    Sep 4, 2025
 * @brief   전동기 전류 제어(PI) 및 SVPWM 변조를 위한 상수 정의와 구조체 선언 헤더 파일
 * @details 동기 좌표계 전류 제어기에 필요한 이득, 상태 변수, 약자속 제어 파라미터 및
 * 공간 벡터 변조(SVPWM)를 위한 전압 변수들을 포함합니다.
 */

#ifndef INC_CURRENTCONTROL_H_
#define INC_CURRENTCONTROL_H_

#include "UserMath.h"

/** @brief 전류 제어기 차단 주파수 (Bandwidth): 300Hz를 Radian 단위로 변환 */
#define WC_CC (300.0f * 6.283185307179586476925286766559)

/** @brief 약자속 제어 시작 전압 제한치 (1 / sqrt(3)) */
#define VLIM_FW                 0.5773502691896257645091487805019 // 1. / sqrt(3)

/** @brief 약자속 제어기 비례 이득 (Proportional Gain) */
#define KP_FW                   (0.04f)
/** @brief 약자속 제어기 적분 이득 (Integral Gain) */
#define KI_FW                   (KP_FW * 70.0f)

/**
 * @struct sCurrentCtrl
 * @brief  전류 제어 및 전압 변조에 관련된 모든 변수를 관리하는 구조체
 */
typedef struct {
    // ---------------------------------------------------------
    // 1. Phase Measurments & Feedback (상전류 및 전압 피드백)
    // ---------------------------------------------------------
    float fIasHall;             /**< A상 전류 측정값 (Hall Sensor/ADC) */
    float fIbsHall;             /**< B상 전류 측정값 */
    float fIcsHall;             /**< C상 전류 측정값 */

    float fVas;                 /**< A상 인가 전압 */
    float fVbs;                 /**< B상 인가 전압 */
    float fVcs;                 /**< C상 인가 전압 */

    float fVan;                 /**< A상 상전압 (중성점 기준) */
    float fVbn;                 /**< B상 상전압 (중성점 기준) */
    float fVcn;                 /**< C상 상전압 (중성점 기준) */

    // ---------------------------------------------------------
    // 2. Coordinate Transformation (좌표 변환 데이터)
    // ---------------------------------------------------------
    // Stationary Frame (정지 좌표계 - Alpha/Beta)
    float fIdss;                /**< 정지 좌표계 d축(Alpha) 전류 */
    float fIqss;                /**< 정지 좌표계 q축(Beta) 전류 */

    // Rotating Frame (회전 좌표계 - d/q)
    float fIdsr;                /**< 회전 동기 좌표계 d축(자속성분) 전류 피드백 */
    float fIqsr;                /**< 회전 동기 좌표계 q축(토크성분) 전류 피드백 */

    float fIdsrErr;             /**< d축 전류 오차 (Ref - Feedback) */
	float fIqsrErr;             /**< q축 전류 오차 (Ref - Feedback) */

    float fIdsrFF;              /**< d축 전향 보상(Feed-forward) 항 */
	float fIqsrFF;              /**< q축 전향 보상(Feed-forward) 항 */

    // ---------------------------------------------------------
    // 3. Current References (전류 지령)
    // ---------------------------------------------------------
    float fIdsrRef;             /**< 최종 d축 전류 지령 */
    float fIqsrRef;             /**< 최종 q축 전류 지령 */

	float fIdsrRefSet;          /**< 사용자가 설정한 d축 전류 목표값 */
	float fIqsrRefSet;          /**< 사용자가 설정한 q축 전류 목표값 */

    float fIdqrRefSet;          /**< DQ축 복합 지령 설정값 (필요 시 사용) */

    float fIdsrRefMax;          /**< d축 전류 지령 최대 제한치 */
    float fIqsrRefMax;          /**< q축 전류 지령 최대 제한치 */

    // ---------------------------------------------------------
    // 4. PI Controller Gains (전류 제어기 이득)
    // ---------------------------------------------------------
    // d-axis Controller
    float fKpdCc;               /**< d축 전류 제어기 비례 이득 */
    float fKidCc;               /**< d축 전류 제어기 적분 이득 */
    float fKadCc;               /**< d축 Anti-windup 이득 */

    // q-axis Controller
    float fKpqCc;               /**< q축 전류 제어기 비례 이득 */
    float fKiqCc;               /**< q축 전류 제어기 적분 이득 */
    float fKaqCc;               /**< q축 Anti-windup 이득 */

    // ---------------------------------------------------------
    // 5. PI Controller State & Outputs (제어기 상태 및 출력)
    // ---------------------------------------------------------
    float fIdsrInteg;           /**< d축 제어기 적분 누적값 */
    float fIqsrInteg;           /**< q축 제어기 적분 누적값 */

    float fVdsrRef;             /**< d축 전압 지령 (PI 제어기 출력) */
    float fVqsrRef;             /**< q축 전압 지령 (PI 제어기 출력) */

    float fVdsrAwRef;           /**< d축 Anti-windup 보정값 */
    float fVqsrAwRef;           /**< q축 Anti-windup 보정값 */

    float fVdsrOut;             /**< 제한 적용 후 최종 d축 출력 전압 */
    float fVqsrOut;             /**< 제한 적용 후 최종 q축 출력 전압 */

    // ---------------------------------------------------------
    // 6. Field Weakening Control (약계자 제어)
    // ---------------------------------------------------------
    float fVmagErr;             /**< 전압 크기 오차 (제한치 - 현재전압) */
    float fVdqsrOutMag;         /**< 현재 출력 전압 벡터의 크기 */

    float fDelIdsrRefFWAW;      /**< 약자속 제어용 d축 전류 변동분 (Anti-windup) */
    float fDelIdsrRefFWInteg;   /**< 약자속 제어용 d축 전류 변동분 (적분항) */
    float fDelIdsrRefFWUnsat;   /**< 약자속 제어용 d축 전류 변동분 (제한 전) */
    float fDelIdsrRefFW;        /**< 최종 약자속 d축 보상 전류 */

    float fKaFW;                /**< 약자속 제어 Anti-windup 이득 */

    float fBetaAngleRad;        /**< 전류 진각(Advance Angle) [Radian] */
    float fBetaAngle;           /**< 전류 진각(Advance Angle) [Degree] */

    // ---------------------------------------------------------
    // 7. Inverse Transform & SVPWM (역변환 및 공간 벡터 변조)
    // ---------------------------------------------------------
    // Inverse Park Output (to Stationary)
    float fVdssOut;             /**< 역 Park 변환 출력 d축 전압 (실제 출력 추정용) */
    float fVqssOut;             /**< 역 Park 변환 출력 q축 전압 (실제 출력 추정용) */

    float fVdssRef;             /**< 정지 좌표계 d축 전압 지령 */
    float fVqssRef;             /**< 정지 좌표계 q축 전압 지령 */

    // Inverse Clarke Output (to 3-Phase Reference)
    float fVasRef;              /**< 역 Clarke 변환 출력 A상 전압 지령 */
    float fVbsRef;              /**< 역 Clarke 변환 출력 B상 전압 지령 */
    float fVcsRef;              /**< 역 Clarke 변환 출력 C상 전압 지령 */

    float fVanRef;              /**< Offset이 포함된 최종 A상 전압 지령 */
    float fVbnRef;              /**< Offset이 포함된 최종 B상 전압 지령 */
    float fVcnRef;              /**< Offset이 포함된 최종 C상 전압 지령 */

    // SVPWM Offset Calculation
    float fVsRefMax;            /**< 3상 전압 지령 중 최댓값 */
    float fVsRefMin;            /**< 3상 전압 지령 중 최솟값 */
    float fVsnOffset;           /**< SVPWM 구현을 위한 영상분 오프셋 전압 */

    // ---------------------------------------------------------
    // 8. PWM Generation Outputs (PWM 듀티 및 비교 레지스터)
    // ---------------------------------------------------------
    float fDutyA;               /**< A상 PWM Duty (0.0 ~ 1.0) */
    float fDutyB;               /**< B상 PWM Duty (0.0 ~ 1.0) */
    float fDutyC;               /**< C상 PWM Duty (0.0 ~ 1.0) */

    // ---------------------------------------------------------
    // 9. Final Estimated Outputs (최종 출력 전압 추정치)
    // ---------------------------------------------------------
    float fVanOut;              /**< 실제 인버터에서 출력된 것으로 추정되는 A상 전압 */
    float fVbnOut;              /**< 실제 인버터에서 출력된 것으로 추정되는 B상 전압 */
    float fVcnOut;              /**< 실제 인버터에서 출력된 것으로 추정되는 C상 전압 */

    // ---------------------------------------------------------
    // 10. DutyTest
    // ---------------------------------------------------------
    float fDutyA_Test;          /**< 디버깅용 A상 강제 Duty 설정값 */
    float fDutyB_Test;          /**< 디버깅용 B상 강제 Duty 설정값 */
    float fDutyC_Test;          /**< 디버깅용 C상 강제 Duty 설정값 */

} sCurrentCtrl;

#endif /* INC_CURRENTCONTROL_H_ */
