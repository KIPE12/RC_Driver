/**
 * @file    Fault.c
 * @author  lsj50
 * @date    2026. 3. 9.
 * @brief   모터 제어 시스템의 Fault(고장) 감지 및 보호 동작 수행 소스 파일
 *
 * * @details [고장(Fault) 감지 및 처리 로직]
 * 제어 루프 내에서 실시간으로 센서 데이터를 모니터링하여 소프트웨어 고장(SW_Fault)을 판별합니다.
 * 고장 조건이 하나라도 발생하면 `vSWFaultOperation()` 함수가 호출되어 `SW_Fault = 1`로 설정되고,
 * 다음 제어 주기에서 FAULT_STATE로 전이하여 PWM 출력을 즉시 차단합니다.
 *
 * | 변수명 | 상태값 | 판별 조건 (트리거) |
 * | :--- | :--- | :--- |
 * | **SW_Fault** | 0 (정상) | 시스템 정상 동작 중 |
 * | **SW_Fault** | 1 (고장) | 아래 3가지 조건 중 하나라도 만족할 경우 발생<br> 1. **과전압 (OVP)** : `|fVdc| >= VDC_FAULT_LEV`<br> 2. **과전류 (OCP)** : `|Ias|`, `|Ibs|`, `|Ics|` 중 하나라도 `>= CURR_FAULT_LEV`<br> 3. **과속도 (OSP)** : 모터 속도 `|fWrpmSC| >= SPD_FAULT_LEV` |
 */

#include <Fault.h>
#include <GlobalVar.h>
#include <MotorControl.h>
#include <stm32g474xx.h>
#include <stm32g4xx_hal_tim.h>
#include <sys/_stdint.h>

/** @brief 하드웨어 트리거(Trip Zone)에 의한 고장 플래그 */
uint16_t TZ_Fault = 0u;

/** @brief 소프트웨어 조건에 의한 고장 플래그 */
uint16_t SW_Fault = 0u;

/** @brief 시스템 전체 Fault 통합 플래그 */
uint16_t uFaultFlag = 0u;

/** @brief PWM 제어에 사용되는 메인 타이머 핸들러 외부 참조 */
extern TIM_HandleTypeDef htim1;

/**
 * @brief  Fault 발생 시 호출되어 하드웨어 출력을 차단하고 당시의 시스템 상태를 기록합니다.
 * @details
 * 1. TIM1의 Break Interrupt Flag를 클리어하고 Main Output(MOE)을 차단합니다.
 * 2. 시스템 시작 플래그를 0으로 리셋합니다.
 * 3. 현재의 3상 전류, 동기좌표계 전류, DC링크 전압, 회전 속도를 Fault 구조체에 저장합니다.
 * @param  MotorControl 모터 제어 통합 구조체 포인터 (현재 상태 데이터 참조용)
 * @param  Fault_Infomation 고장 정보를 저장할 구조체 포인터
 * @retval 없음
 */
void vFaultEvent(sMotorCtrl* MotorControl, sFault_Info* Fault_Infomation){
	/* 하드웨어 레지스터 직접 조작을 통한 PWM 즉각 차단 */
	TIM1->SR = ~TIM_SR_BIF;      /**< Break Interrupt Flag 클리어 */
	TIM1->BDTR &= ~TIM_BDTR_MOE; /**< Main Output Enable 비트 해제 (PWM 출력 차단) */

	Flag.START = 0u;             /**< 시스템 가동 플래그 해제 */

	vSwitchOffSettingTIM(&htim1); /**< 타이머 채널별 안전 상태 설정 */

	/* 고장 시점의 데이터 캡처 (Black Box 역할) */
	Fault_Infomation->Ia_Fault = MotorControl->CC.fIasHall;
	Fault_Infomation->Ib_Fault = MotorControl->CC.fIbsHall;
	Fault_Infomation->Ic_Fault = MotorControl->CC.fIcsHall;

	Fault_Infomation->Idsr_Fault = MotorControl->CC.fIdsr;
	Fault_Infomation->Iqsr_Fault = MotorControl->CC.fIqsr;

	Fault_Infomation->Vdc_Fault = fVdc;
	Fault_Infomation->Wrpm_Fault = MotorControl->SO.fWrpmSC;
}

/**
 * @brief  소프트웨어적으로 Fault 상황을 강제 발생시킵니다.
 * @note   이 함수가 호출되면 TIM1의 Break 이벤트를 강제로 발생시켜 하드웨어 인터럽트를 유도합니다.
 * @param  없음
 * @retval 없음
 */
void vSWFaultOperation(){
	SW_Fault = 1u;
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK); /**< 타이머 Break 인터럽트 활성화 */

	TIM1->EGR |= TIM_EGR_BG; /**< Break Generation (BG) 비트 설정을 통한 강제 트리거 */
}

/**
 * @brief  Fault 관련 플래그 및 기록된 데이터 구조체를 초기화합니다.
 * @param  Fault_Infomation 초기화할 고장 정보 구조체 포인터
 * @retval 없음
 */
void vInitFault(sFault_Info* Fault_Infomation){
	TZ_Fault = 0u;
	SW_Fault = 0u;

	Fault_Infomation->Ia_Fault = 0.0f;
	Fault_Infomation->Ib_Fault = 0.0f;
	Fault_Infomation->Ic_Fault = 0.0f;

	Fault_Infomation->Idsr_Fault = 0.0f;
	Fault_Infomation->Iqsr_Fault = 0.0f;

	Fault_Infomation->Vdc_Fault = 0.0f;
	Fault_Infomation->Wrpm_Fault = 0.0f;
}

/**
 * @brief  Fault 상태를 해제하고 시스템을 다시 가동 가능한 상태로 복구합니다.
 * @note   모든 Fault 기록을 초기화한 후, 차단되었던 TIM1의 Main Output(MOE)을 다시 활성화합니다.
 * @param  없음
 * @retval 없음
 */
void vClearFault(){
	vInitFault(&INV.Fault_Info); /**< 내부 고장 데이터 초기화 */
	TIM1->SR = ~TIM_SR_BIF;      /**< 남아있는 Break 플래그 클리어 */
	TIM1->BDTR |= TIM_BDTR_MOE;  /**< Main Output 재활성화 (PWM 가동 가능 상태) */
}
