/**
 * @file    control.c
 * @brief   PMSM 메인 제어 루프 모듈
 *
 * TIM2 인터럽트에서 100µs(10kHz)마다 호출되는 Control() 함수를 정의한다.
 *
 * @details 제어 루프 실행 순서:
 * 1. ADC JDR1~JDR4 직접 읽기
 * 2. 오프셋 보정 미완료 → Offset(), 완료 → AdcProcess()
 * 3. 과전류 보호: |I| ≥ 80A → SoftWareFault()
 * 4. 제어기 게인 갱신: UpdateController()
 * 5. Hall 옵저버: Hallsensor_Observer()
 * 6. FLAG 기반 운전 모드 분기
 *
 * @details 운전 모드 분기 표:
 * | FLAG             | 함수                               | 설명              |
 * |------------------|------------------------------------|-------------------|
 * | INV_RUN          | SpeedControl + Vref_GenControl     | 폐루프 속도 제어  |
 * | HALL_POS_TEST    | HallPosition_Test                  | Hall 위치 확인    |
 * | DUTY_TEST        | 고정 듀티 설정                     | 인버터 동작 확인  |
 * | INV_OLC          | OpenLoopControl + CurrentControl   | 전류 개루프 제어  |
 * | INV_Vref_Gen     | Vref_GenControl                    | 전압 기준 생성    |
 * | INV_VOLC         | VoltageOpenLoopControl             | 전압 개루프 제어  |
 * | Param_Estimation | VoltageInjection_SquareWave        | 파라미터 추정     |
 * | INV_ALIGN        | Align                              | d축 초기 정렬     |
 * | 없음 / FAULT     | PwmSwOff + ResetController         | 전체 정지         |
 *
 * @author  HALAB_G
 * @date    2024-12-06
 */

#include "control.h"
#include "inv.h"
#include "variable.h"
#include "fault.h"
#include "flag.h"
#include "adc.h"
#include "speed_observer.h"

/**
 * @brief 제어 루프 실행 횟수 카운터 (디버깅용)
 * TIM2 인터럽트마다 1씩 증가한다. uint32_t 오버플로우 시 자연 순환.
 */
uint32_t ControlCnt;

extern uint8_t           T_buffer[0];
extern ADC_HandleTypeDef hadc1;

/**
 * @brief  PMSM 메인 제어 함수 (TIM2 인터럽트에서 100µs 주기로 호출)
 *
 * ADC 원시값 읽기 → 과전류 보호 → 운전 모드 선택 → 제어 출력 순서로 실행된다.
 *
 * @note  ADC JDR 레지스터를 직접 읽어 HAL 콜백 지연을 제거한다.
 * @note  과전류 임계값: ±80A (소프트웨어 보호)
 * @warning 실행 시간이 100µs를 초과하면 제어 주기 위반이 발생한다.
 *
 * @retval None
 */
void Control(void)
{
    ControlCnt++;

    /* ADC Injected 레지스터 직접 읽기 */
    adc1Val[0] = hadc1.Instance->JDR1; /* Ia  [LSB] */
    adc1Val[1] = hadc1.Instance->JDR2; /* Ib  [LSB] */
    adc1Val[2] = hadc1.Instance->JDR3; /* Ic  [LSB] */
    adc1Val[3] = hadc1.Instance->JDR4; /* Vdc [LSB] */

    if (!AdInitFlag)
        Offset();
    else
        AdcProcess();

    /* 소프트웨어 과전류 보호 */
    if (ABS(INV.Ia) >= 80.f) { SoftWareFault(); }
    if (ABS(INV.Ib) >= 80.f) { SoftWareFault(); }
    if (ABS(INV.Ic) >= 80.f) { SoftWareFault(); }

    UpdateController(&INV);
    Hallsensor_Observer(&INV);

    if (FLAG.READY && !(FLAG.FAULT)) {

        if (FLAG.INV_RUN) {
            SpeedControl(&INV);
            Vref_GenControl(&INV);
        }
        else if (FLAG.HALL_POS_TEST) {
            HallPosition_Test(&INV);
            PwmDutyUpt();
            PwmSwOn();
            PWM_BUF_ON;
        }
        else if (FLAG.DUTY_TEST) {
            INV.Duty_A = LIMIT(INV.Duty_A, 0.f, 0.95f);
            INV.Duty_B = LIMIT(INV.Duty_B, 0.f, 0.95f);
            INV.Duty_C = LIMIT(INV.Duty_C, 0.f, 0.95f);
            INV.Duty_A = 0.2f;
            INV.Duty_B = 0.3f;
            INV.Duty_C = 0.8f;
            PwmDutyUpt();
            PwmSwOn();
            PWM_BUF_ON;
        }
        else if (FLAG.INV_OLC) {
            Theta_mode = 1;
            OpenLoopControl(&INV);
            CurrentControl(&INV);
        }
        else if (FLAG.INV_Vref_Gen) {
            Vref_GenControl(&INV);
        }
        else if (FLAG.INV_VOLC) {
            VoltageOpenLoopControl(&INV);
        }
        else if (FLAG.Param_Estimation) {
            VoltageInjection_SquareWave(&INV);
        }
        else if (FLAG.INV_ALIGN) {
            Align(&INV);
        }
        else {
            PwmSwOff();
            PwmDutyUpt();
            ResetController(&INV);
            FLAG.INV_RUN = FLAG.INV_OLC = FLAG.INV_VOLC = 0;
            FLAG.INV_ALIGN = FLAG.INV_NLC = FLAG.TS_MODE = 0;
        }
    }
    else {
        PwmSwOff();
        PwmDutyUpt();
        ResetController(&INV);
        FLAG.INV_RUN = FLAG.INV_OLC = FLAG.INV_VOLC = 0;
        FLAG.INV_ALIGN = FLAG.INV_NLC = FLAG.TS_MODE = FLAG.READY = 0;
    }
}
