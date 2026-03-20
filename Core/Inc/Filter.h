/**
 * @file    Filter.h
 * @author  lsj50
 * @date    2026. 3. 9.
 * @brief   디지털 신호 처리를 위한 1차 및 2차 IIR 필터 설계 헤더 파일
 * @details LPF, HPF, BPF, Notch 필터 등 다양한 필터 타입을 지원하며,
 * Bilinear Transformation 등을 통해 설계된 계수를 기반으로 실시간 필터링을 수행합니다.
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

/** @name 필터 타입 정의 (Filter Types)
 * @{ */
#define K_ALLPASS   0  /**< All-pass 필터: 모든 주파수 통과 */
#define K_LPF       1  /**< Low-pass 필터: 저역 통과 */
#define K_HPF       2  /**< High-pass 필터: 고역 통과 */
#define K_BPF       3  /**< Band-pass 필터: 특정 대역 통과 */
#define K_NOTCH     4  /**< Notch 필터: 특정 주파수 제거 */
/** @} */

/**
 * @struct IIR1
 * @brief  1차 IIR(Infinite Impulse Response) 필터 구조체
 */
typedef struct {
    int type;           /**< 필터 타입 (K_LPF, K_HPF 등) */
    float w0;           /**< 차단 주파수 (Cut-off frequency) [rad/s] */
    float delT;         /**< 샘플링 주기 (Sampling period) [s] */
    float coeff[3];     /**< 필터 계수 배열 */
    float reg;          /**< 상태 변수 (내부 레지스터/지연 요소) */
    float a0, b0, b1;   /**< 차분 방정식 계수 */
    float INV_alpha;    /**< 연산 최적화를 위한 알파 역수 */
}   IIR1;

/**
 * @struct IIR2
 * @brief  2차 IIR(Infinite Impulse Response) 필터 구조체
 */
typedef struct {
    int type;           /**< 필터 타입 (K_LPF, K_HPF, K_NOTCH 등) */
    float w0, zeta;     /**< 고유 주파수 [rad/s] 및 감쇠비 (Damping ratio) */
    float delT;         /**< 샘플링 주기 (Sampling period) [s] */
    float coeff[5];     /**< 필터 계수 배열 */
    float reg[2];       /**< 상태 변수 배열 (지연 요소 z^-1, z^-2) */
    float a0, a1;       /**< 분모 계수 (Denominator coefficients) */
    float b0, b1, b2;   /**< 분자 계수 (Numerator coefficients) */
    float INV_alpha;    /**< 연산 최적화를 위한 알파 역수 */
}   IIR2;

/**
 * @brief  1차 IIR 필터의 파라미터를 설정하고 계수를 초기화합니다.
 * @param  p_gIIR 초기화할 IIR1 구조체 포인터
 * @param  type 필터 타입 (K_LPF, K_HPF 등)
 * @param  w0 차단 주파수 [rad/s]
 * @param  Ts 샘플링 주기 [s]
 * @retval 없음
 */
extern void initiateIIR1(IIR1 *p_gIIR, int type, float w0, float Ts);

/**
 * @brief  2차 IIR 필터의 파라미터를 설정하고 계수를 초기화합니다.
 * @param  p_gIIR 초기화할 IIR2 구조체 포인터
 * @param  type 필터 타입 (K_LPF, K_HPF, K_NOTCH 등)
 * @param  w0 고유 주파수 [rad/s]
 * @param  zeta 감쇠비 (보통 0.707 등으로 설정)
 * @param  Ts 샘플링 주기 [s]
 * @retval 없음
 */
extern void initiateIIR2(IIR2 *p_gIIR, int type, float w0, float zeta, float Ts);

/**
 * @brief  1차 IIR 필터 연산을 수행하여 출력을 반환합니다.
 * @param  p_gIIR 연산에 사용할 IIR1 구조체 포인터
 * @param  input 현재 샘플의 입력값
 * @return 필터링된 출력값
 */
extern float IIR1Update(IIR1 *p_gIIR, float input);

/**
 * @brief  2차 IIR 필터 연산을 수행하여 출력을 반환합니다.
 * @param  p_gIIR 연산에 사용할 IIR2 구조체 포인터
 * @param  input 현재 샘플의 입력값
 * @return 필터링된 출력값
 */
extern float IIR2Update(IIR2 *p_gIIR, const float input);

#endif /* INC_FILTER_H_ */
