/**
 * @file    Filter.c
 * @author  lsj50
 * @date    2026. 3. 9.
 * @brief   1차 및 2차 IIR(Infinite Impulse Response) 디지털 필터 구현 소스 파일
 * * @details [필터 설계 및 구현 알고리즘]
 * 연속 시간(Continuous-time, s-domain) 전달 함수를 기반으로 필터를 설계한 후,
 * 쌍선형 변환(Bilinear / Tustin Transform)을 적용하여 이산 시간(Discrete-time, z-domain) 계수를 산출합니다.
 * 연산 최적화 및 오버플로우 방지를 위해 **Direct Form II Transposed** 구조의 차분 방정식(Difference Equation)을
 * 사용하여 필터를 업데이트합니다.
 *
 * @details [1차 IIR 필터 (IIR1) 지원 타입]
 * 차단 주파수(w0) 파라미터를 기반으로 설계되며, 3가지 필터 타입을 지원합니다.
 * | 타입 (Type) | 필터 종류 | 주요 특징 및 용도 |
 * | :--- | :--- | :--- |
 * | **K_LPF** | 저역통과 필터 (Low-Pass Filter) | 고주파 노이즈 제거 (예: 전류/전압 센서 노이즈 필터링) |
 * | **K_HPF** | 고역통과 필터 (High-Pass Filter) | 직류 및 저주파 성분 차단 (예: 센서 오프셋 제거) |
 * | **K_ALLPASS** | 전대역통과 필터 (All-Pass Filter) | 크기는 유지하고 위상만 지연시킴 (예: 신호 위상 보상) |
 *
 * @details [2차 IIR 필터 (IIR2) 지원 타입]
 * 자연 주파수(w0)와 감쇠비(zeta) 파라미터를 기반으로 설계되며, 5가지 필터 타입을 지원합니다.
 * | 타입 (Type) | 필터 종류 | 주요 특징 및 용도 |
 * | :--- | :--- | :--- |
 * | **K_LPF** | 저역통과 필터 (Low-Pass Filter) | 속도 관측기 등에서 1차 대비 더 급격한 감쇠가 필요할 때 사용 |
 * | **K_HPF** | 고역통과 필터 (High-Pass Filter) | 저주파 외란 차단 |
 * | **K_BPF** | 대역통과 필터 (Band-Pass Filter) | 특정 주파수 대역의 신호만 추출 |
 * | **K_NOTCH** | 대역차단 필터 (Notch Filter) | 기계적 공진 등 특정 주파수의 진동만 예리하게 제거할 때 사용 |
 * | **K_ALLPASS** | 전대역통과 필터 (All-Pass Filter) | 복잡한 신호 동기화 및 정밀한 위상 튜닝 |
 *
 * @note **사용 주의사항:** 계수 산출 함수(`initiateIIR1`, `initiateIIR2`)는 초기화 시 또는 파라미터가
 * 변경될 때만 호출해야 하며, 실제 제어 루프 내부에서는 계산량이 적은 `IIR1Update`, `IIR2Update`
 * 함수만 반복 호출하여 필터링된 결과값을 얻어야 합니다.
 */

#include "MotorControl.h"
#include "UserMath.h"
#include <math.h>
#include "Filter.h"

/**
 * @brief  1차 IIR 필터의 계수를 계산하고 초기화합니다.
 * @details 선택된 필터 타입(LPF, HPF, All-pass)에 따라 연속 시간 계수를 설정하고,
 * 쌍선형 변환(Bilinear Transform)을 통해 디지털 필터 계수를 도출합니다.
 * @param  p_gIIR 초기화할 1차 IIR 필터 구조체 포인터
 * @param  type 필터 종류 (K_LPF: 저역통과, K_HPF: 고역통과, K_ALLPASS: 전대역통과)
 * @param  w0 차단 주파수 (Cut-off frequency, rad/s)
 * @param  Ts 샘플링 시간 (Sampling period, sec)
 * @retval 없음
 */
void initiateIIR1(IIR1 *p_gIIR, int type, float w0, float Ts)
{
    float a0, b0, b1;
    float INV_alpha;

    // Continuous-time Filter Coefficients (s-domain)
    p_gIIR->w0 = w0;
    p_gIIR->type = type;
    p_gIIR->delT = Ts;

    a0 = w0;

    switch (type)
    {
    case K_LPF:
        b0 = w0;
        b1 = 0.0f;
        break;
    case K_HPF:
        b0 = 0.0f;
        b1 = 1.0f;
        break;
    default:
    case K_ALLPASS:
        b0 = w0;
        b1 = -1.0f;
    }

    // Discrete-time Filter Coefficients calculation (Tustin transformation)
    INV_alpha = 1.0f / (2.0f + Ts * a0);
    p_gIIR->coeff[0] = (2.0f * b1 + Ts * b0) * INV_alpha;
    p_gIIR->coeff[1] = (-2.0f * b1 + Ts * b0) * INV_alpha;
    p_gIIR->coeff[2] = (2.0f - Ts * a0) * INV_alpha;
    p_gIIR->reg = 0.0f;  /**< 필터 상태 레지스터 초기화 */
}

/**
 * @brief  1차 IIR 필터의 출력을 업데이트합니다.
 * @note   Direct Form II Transposed 구조를 사용하여 연산을 수행합니다.
 * @param  p_gIIR 실행할 IIR1 구조체 포인터
 * @param  x 필터의 현재 입력값
 * @retval y 필터의 현재 출력값
 */
float IIR1Update(IIR1 *p_gIIR, float x)
{
    float y;

    y = p_gIIR->reg + p_gIIR->coeff[0] * x;
    p_gIIR->reg = p_gIIR->coeff[1] * x + p_gIIR->coeff[2] * y;

    return(y);
}

/**
 * @brief  2차 IIR 필터의 계수를 계산하고 초기화합니다.
 * @details 2차 시스템의 감쇠비(zeta)와 차단 주파수(w0)를 기반으로
 * LPF, HPF, BPF, Notch, All-pass 필터를 설계합니다.
 * @param  p_gIIR 초기화할 2차 IIR 필터 구조체 포인터
 * @param  type 필터 종류 (K_LPF, K_HPF, K_BPF, K_NOTCH, K_ALLPASS)
 * @param  w0 자연 주파수 (Natural frequency, rad/s)
 * @param  zeta 감쇠비 (Damping ratio)
 * @param  Ts 샘플링 시간 (Sampling period, sec)
 * @retval 없음
 */
void initiateIIR2(IIR2 *p_gIIR, int type, float w0, float zeta, float Ts)
{
    float a0, a1, b0, b1, b2;
    float INV_alpha;

    // Continuous-time Filter Coefficients (s-domain)
    p_gIIR->w0 = w0;
    p_gIIR->zeta = zeta;
    p_gIIR->delT = Ts;
    p_gIIR->type = type;

    a0 = w0 * w0;
    a1 = 2.0f * zeta * w0;

    switch (type)
    {
    case K_LPF:
        b0 = w0 * w0;
        b1 = 0.0f;
        b2 = 0.0f;
        break;
    case K_HPF:
        b0 = 0.0f;
        b1 = 0.0f;
        b2 = 1.0f;
        break;
    case K_BPF:
        b0 = 0.0f;
        b1 = 2.0f * zeta * w0;
        b2 = 0.0f;
        break;
    case K_NOTCH:
        b0 = w0 * w0;
        b1 = 0.0f;
        b2 = 1.0f;
        break;
    case K_ALLPASS:
    default:
        b0 = w0 * w0;
        b1 = -2.0f * zeta * w0;
        b2 = 1.0f;
    }

    // Discrete-time Filter Coefficients calculation (Tustin transformation)
    INV_alpha = 1.0f / (4.0f + 2.0f * Ts * a1 + Ts * Ts * a0);
    p_gIIR->coeff[0] = (4.0f * b2 + 2.0f * Ts * b1 + Ts * Ts * b0) * INV_alpha;
    p_gIIR->coeff[1] = (2.0f * Ts * Ts * b0 - 8.0f * b2) * INV_alpha;
    p_gIIR->coeff[2] = -(2.0f * Ts * Ts * a0 - 8.0f) * INV_alpha;
    p_gIIR->coeff[3] = (4.0f * b2 - 2.0f * Ts * b1 + Ts * Ts * b0) * INV_alpha;
    p_gIIR->coeff[4] = -(4.0f - 2.0f * Ts * a1 + Ts * Ts * a0) * INV_alpha;

    p_gIIR->reg[0] = 0.0f;
    p_gIIR->reg[1] = 0.0f;
}

/**
 * @brief  2차 IIR 필터의 출력을 업데이트합니다.
 * @details 2개의 상태 레지스터를 사용하여 입력을 필터링합니다.
 * 정밀한 제어가 필요한 속도 관측기나 센서 데이터 노이즈 제거에 사용됩니다.
 * @param  p_gIIR 실행할 IIR2 구조체 포인터
 * @param  x 필터의 현재 입력값 (Const float)
 * @retval y 필터의 현재 출력값
 */
float IIR2Update(IIR2 *p_gIIR, const float x)
{
    float y;

    /* Direct Form II Transposed Difference Equation */
    y = p_gIIR->reg[0] + p_gIIR->coeff[0] * x;
    p_gIIR->reg[0] = p_gIIR->reg[1] + p_gIIR->coeff[1] * x + p_gIIR->coeff[2] * y;
    p_gIIR->reg[1] = p_gIIR->coeff[3] * x + p_gIIR->coeff[4] * y;

    return(y);
}
