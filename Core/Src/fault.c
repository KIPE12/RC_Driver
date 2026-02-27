/**
 * @file    fault.c
 * @brief   인버터 고장 감지 및 처리 모듈
 *
 * 하드웨어 고장(GPIO EXTI)과 소프트웨어 고장(과전류 등)을 감지하여
 * PWM을 즉시 차단하고 고장 발생 시점의 상태를 FLTVAL에 저장한다.
 *
 * @details 고장 종류 및 FLAG.FAULT 값:
 * | 값 | 종류             | 트리거                             |
 * |----|------------------|------------------------------------|
 * | 0  | 정상             | -                                  |
 * | 1  | 하드웨어 고장    | GPIO EXTI (OCP, OVP 등 외부 회로)  |
 * | 2  | 소프트웨어 고장  | 전류 임계값 초과 (|I| ≥ 80A)      |
 *
 * @note 하드웨어 고장(1)이 발생한 후에는 소프트웨어 고장(2)이 덮어쓰지 않는다.
 *       (우선순위: 하드웨어 > 소프트웨어)
 *
 * @author  HALAB_G
 * @date    2024-12-06
 */

#include "inv.h"
#include "control.h"
#include "fault.h"
#include "flag.h"

uint16_t         FaultCnt = 0; ///< 누적 고장 발생 횟수 (HW + SW 합산)
struct FAULT_VAL FLTVAL;       ///< 고장 발생 시점의 전기적 상태 스냅샷

/**
 * @brief  GPIO 외부 인터럽트 콜백 (하드웨어 고장 진입점)
 *
 * DRV8300 OCP, 과전압 보호 등 하드웨어 보호 회로가 EXTI를 발생시킬 때 호출된다.
 * 인터럽트 발생 즉시 PWM을 차단한 후 HardWareFault()를 호출한다.
 *
 * @param[in] GPIO_Pin 트리거된 GPIO 핀 마스크 (stm32g4xx_it.c에서 전달)
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    PwmSwOff();
    HardWareFault();
}

/**
 * @brief  하드웨어 고장 처리 함수
 *
 * 외부 보호 회로(EXTI)에 의해 트리거된 고장을 처리한다.
 * FLAG.FAULT = 1로 설정하고 고장 시점의 전기량을 FLTVAL에 저장한다.
 *
 * @details 저장 항목:
 * - FLTVAL.Vdc  : DC 링크 전압 [V]
 * - FLTVAL.Idc  : DC 링크 전류 [A]
 * - FLTVAL.Ia/Ib/Ic : 3상 전류 [A]
 * - FLTVAL.Wrpm : 추정 속도 [rpm]
 *
 * @retval None
 */
void HardWareFault(void)
{
    PwmSwOff();

    FLAG.FAULT = 1;
    FLAG.READY = 0;

    FLTVAL.Vdc  = INV.Vdc;
    FLTVAL.Idc  = INV.Idc;
    FLTVAL.Ia   = INV.Ia;
    FLTVAL.Ib   = INV.Ib;
    FLTVAL.Ic   = INV.Ic;
    FLTVAL.Wrpm = INV.Wrpm;

    FaultCnt++;
}

/**
 * @brief  소프트웨어 고장 처리 함수
 *
 * 제어 소프트웨어 내에서 감지된 고장(과전류 등)을 처리한다.
 * FLAG.FAULT == 1 (하드웨어 고장)이 이미 발생한 경우에는 덮어쓰지 않는다.
 *
 * @details 호출 조건 (control.c 내):
 * @code
 *   if (ABS(INV.Ia) >= 80.f) SoftWareFault();
 * @endcode
 *
 * @note 하드웨어 고장 우선 원칙:
 *       FLAG.FAULT != 1 인 경우에만 FLAG.FAULT = 2로 설정한다.
 *
 * @retval None
 */
void SoftWareFault(void)
{
    PwmSwOff();

    FLAG.READY = 0;
    if (FLAG.FAULT != 1) FLAG.FAULT = 2; /* 하드웨어 고장 우선 */

    FLTVAL.Vdc  = INV.Vdc;
    FLTVAL.Idc  = INV.Idc;
    FLTVAL.Ia   = INV.Ia;
    FLTVAL.Ib   = INV.Ib;
    FLTVAL.Ic   = INV.Ic;
    FLTVAL.Wrpm = INV.Wrpm;

    FaultCnt++;
}
