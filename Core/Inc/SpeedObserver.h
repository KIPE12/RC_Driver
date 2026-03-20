/**
 * @file    SpeedObserver.h
 * @author  lsj50
 * @date    Sep 7, 2025
 * @brief   전동기의 속도 및 위치(각도) 추정을 위한 관측기 및 센서 처리 헤더 파일
 * @details 상태 관측기(State Observer), PLL(Phase Locked Loop), 홀 센서 처리,
 * 그리고 초기 기동을 위한 Align 및 I-f(전류 대 주파수) 제어 변수를 포함합니다.
 */

#ifndef INC_SPEEDOBSERVER_H_
#define INC_SPEEDOBSERVER_H_

#include <stdint.h>

/** @brief I-by-F 기동 시의 가속도 (Delta RPM per Step) */
#define DEL_WRPM_REF_IBYF       3000.0f

/** @name 관측기 설계 파라미터 (Observer Design Parameters)
 * @{ */
#define WC_SO1       (10.0f * 6.283185307179586476925286766559f) /**< 관측기 이득 1 차단 주파수 */
#define WC_SO23      (10.0f * 6.283185307179586476925286766559f) /**< 관측기 이득 2,3 차단 주파수 */
#define ZETA_SO      (0.7071067811865475244008443621048f)         /**< 관측기 감쇠비 */
/** @} */

/** @name PLL 설계 파라미터 (PLL Design Parameters)
 * @{ */
#define WC_PLL		(50.0f * 6.283185307179586476925286766559f)  /**< PLL 차단 주파수 */
#define ZETA_PLL	 (0.7071067811865475244008443621048f)         /**< PLL 감쇠비 */
/** @} */

/** @brief 속도 피드백용 LPF 차단 주파수 [rad/s] */
#define WC_WRPMSC_LPF   (6.283185307179586476925286766559f * 30.0f)
#define WC_WRM_LPF      (6.283185307179586476925286766559f * 30.0f)

/** @brief 홀 센서 물리적 장착 위치 오프셋 [rad] */
#define HALL_OFFSET_RAD	(0.0f)

/** @name Align(위치 정렬) 운전 파라미터
 * @{ */
#define IDSR_REF_SET_ALIGN      2.0f       /**< 정렬 시 인가할 d축 전류 [A] */
#define DEL_IDSR_REF_ALIGN      3.0f       /**< 정렬 전류 증분 제한 */
#define WR_REF_SET_ALIGN        12.566370614359172953850573533118f /**< 정렬 시 회전 속도 (2Hz) */
#define DEL_WR_REF_ALIGN        12.566370614359172953850573533118f /**< 속도 증분 제한 */
#define ALIGN_CNT_MAX           40000      /**< 정렬 수행 시간 (FSAMP=20kHz 기준 2초) */
/** @} */

/**
 * @struct sSpeedObs
 * @brief  속도/위치 추정 및 기동 시퀀스를 관리하는 구조체
 */
typedef struct {

	uint16_t uAlignStep, uAlignEnd;     /**< 정렬 단계 및 종료 플래그 */
	uint32_t lAlignCnt;                 /**< 정렬 진행 카운터 */
	float fThetarmOffset, fIdsrRefAlign, fWrRefAlign, fThetarAlign, fThetarmOffsetTemp; /**< 정렬 관련 각도/지령 */
	float fDelIdsrAlign;                /**< 정렬 전류 변화량 */
	float fDelWrRefAlign;               /**< 정렬 속도 변화량 */
	float fINV_AlignCntPlus1, fThetarAlignComp; /**< 연산 최적화 변수 */

	uint16_t uHall_A, uHall_B, uHall_C; /**< 홀 센서 디지털 입력 상태 */
	uint16_t uHall_State;               /**< 3상 홀 센서 조합 상태 (1~6) */

	float fThetar_Hall;                 /**< 홀 센서 기반 전기각 */

    // ---------------------------------------------------------
    // 1. Estimated Angle & Trigonometry (추정 각도 및 삼각함수)
    // ---------------------------------------------------------
    float fThetarm;                     /**< 추정 기계각 [rad] */
	float fThetarmErr;                  /**< 기계각 추정 오차 */

    float fThetar;                      /**< 최종 추정 전기각 [rad] */
    float fThetarErr;                   /**< 전기각 추정 오차 */
    float fThetarEst;                   /**< 관측기 기반 전기각 추정치 */
    float fThetarmEst;                  /**< 관측기 기반 기계각 추정치 */

    float fThetarCC;                    /**< 전류 제어(좌표변환)에 사용되는 최종 각도 */
    float fThetarCompCC;                /**< 지연 보상된 최종 각도 */

    float fSinThetarCC;                 /**< 전류 제어용 각도의 Sine 값 */
    float fCosThetarCC;                 /**< 전류 제어용 각도의 Cosine 값 */

    float fSinThetarCompCC;             /**< 지연 보상 각도의 Sine 값 */
    float fCosThetarCompCC;             /**< 지연 보상 각도의 Cosine 값 */

    // ---------------------------------------------------------
    // 2. Estimated Speed (추정 속도)
    // ---------------------------------------------------------
    float fWrEst;                       /**< 추정 전기각 속도 [rad/s] */
    float fWrmEst;                      /**< 추정 기계각 속도 [rad/s] */
    float fWrmEstLPF;                   /**< LPF 처리된 기계각 추정 속도 */
    float fWrpmEst;                     /**< 추정 속도 [RPM] */
    float fWrpmEstLPF;                  /**< LPF 처리된 추정 RPM */

    float fWrm;                         /**< 기계각 속도 Raw 데이터 */
    float fWrCC;                        /**< 역기전력 보상(Decoupling)용 전기각 속도 */
    float fWrpmSC;                      /**< 속도 제어기 피드백용 RPM */

    // ---------------------------------------------------------
    // 3. Observer Gains & Parameters (관측기 이득 및 파라미터)
    // ---------------------------------------------------------
    float fK1;                          /**< 관측기 이득 1 (위치 오차 보정) */
    float fK2Ts;                        /**< 관측기 이득 2 * 샘플링 시간 (속도 오차 보정) */
    float fK3Ts;                        /**< 관측기 이득 3 * 샘플링 시간 (부하 토크/가속도 추정) */

    float fBperJ;                       /**< 마찰계수/관성비 (B/J) */
    float fInvJ;                        /**< 관성 역수 (1/J) */
    float fInvPP;                       /**< 극쌍수 역수 (1/PP) */

    float fAccEstInteg;                 /**< 가속도 추정기 적분 상태 */
    float fAccFF;                       /**< 가속도 전향 보상항 */

    // ---------------------------------------------------------
    // 3-2. PLL Gains & Parameter (PLL 이득 및 파라미터)
    // ---------------------------------------------------------
    float fKpPLL;                       /**< PLL 비례 이득 */
    float fKiPLL;                       /**< PLL 적분 이득 */
    float fThetarmInteg;                /**< PLL 내부 기계각 적분기 상태 */
    float fThetarInteg;                 /**< PLL 내부 전기각 적분기 상태 */

    // ---------------------------------------------------------
    // 4. I-by-F Control (Open-loop Startup / Sensorless Startup)
    // ---------------------------------------------------------
    float fThetarIbyF;                  /**< I-f 운전용 전기각 */
    float fThetarCompIbyF;              /**< I-f 운전용 지연 보상 각도 */

    float fWrRefIbyF;                   /**< I-f 속도 지령 [rad/s] */
    float fWrpmRefIbyF;                 /**< I-f 속도 지령 [RPM] */
    float fDelWrpmRefIbyF;              /**< I-f 속도 변화량(가속도) */


    float fEncScale;                    /**< 엔코더 펄스-각도 변환 스케일 계수 */
} sSpeedObs;

#endif /* INC_SPEEDOBSERVER_H_ */
