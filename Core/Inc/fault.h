/**
 * @file    Fault.h
 * @author  lsj50
 * @date    2026. 3. 9.
 * @brief   시스템 결함(Fault) 발생 시 상태 정보 저장을 위한 헤더 파일
 * @details 과전류, 과전압, 속도 이상 등 결함이 감지된 시점의 주요 운전 파라미터를
 * 기록하여 사후 분석 및 시스템 보호 로직에 활용하기 위한 구조체를 정의합니다.
 */

#ifndef INC_FAULT_H_
#define INC_FAULT_H_

/**
 * @struct sFault_Info
 * @brief  Fault 발생 순간의 모터 및 인버터 상태 데이터를 저장하는 구조체
 */
typedef	struct{
	float Ia_Fault;     /**< Fault 발생 시점의 A상 전류 [A] */
	float Ib_Fault;     /**< Fault 발생 시점의 B상 전류 [A] */
	float Ic_Fault;     /**< Fault 발생 시점의 C상 전류 [A] */

	float Idsr_Fault;   /**< Fault 발생 시점의 동기 좌표계 d축 전류 [A] */
	float Iqsr_Fault;   /**< Fault 발생 시점의 동기 좌표계 q축 전류 [A] */

	float Vdc_Fault;    /**< Fault 발생 시점의 직류단 전압 (DC-Link) [V] */
	float Wrpm_Fault;   /**< Fault 발생 시점의 모터 회전 속도 [RPM] */

}sFault_Info;

#endif /* INC_FAULT_H_ */
