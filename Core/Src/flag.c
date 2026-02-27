/**
 * @file    flag.c
 * @brief   인버터 제어 플래그 초기화 모듈
 *
 * 시스템 운전 상태를 나타내는 CONTROL_FLAG 구조체를 정의하고 초기화한다.
 * 각 플래그는 Control() 함수에서 운전 모드를 선택하는 데 사용된다.
 *
 * @details 플래그 목록:
 * | 플래그            | 초기값 | 의미                              |
 * |-------------------|--------|-----------------------------------|
 * | READY             | 0      | 인버터 운전 준비 완료             |
 * | FAULT             | 0      | 고장 상태 (0=정상, 1=HW, 2=SW)   |
 * | FAULT_CLEAR       | 0      | 고장 해제 요청                    |
 * | INV_RUN           | 0      | 폐루프 속도 제어 활성화           |
 * | INV_OLC           | 0      | 전류 개루프 제어 활성화           |
 * | INV_VOLC          | 0      | 전압 개루프 제어 활성화           |
 * | INV_Vref_Gen      | 0      | 전압 기준 생성 모드 활성화        |
 * | INV_ALIGN         | 0      | d축 초기 정렬 활성화              |
 * | INV_NLC           | 0      | 비선형 보상 제어 활성화           |
 * | TS_MODE           | 0      | 토크 제어 모드 활성화             |
 * | TS_MODE_STOP      | 0      | 토크 제어 정지                    |
 * | DUTY_TEST         | 0      | 듀티 직접 테스트 활성화           |
 * | Param_Estimation  | 0      | 파라미터 추정 모드 활성화         |
 * | HALL_POS_TEST     | 0      | Hall 위치 테스트 활성화           |
 *
 * @note 시스템 시작 시 모든 플래그는 0으로 초기화된다.
 *       외부 디버거 또는 통신 명령으로 플래그를 변경하여 모드를 전환한다.
 *
 * @author  HALAB_G
 * @date    2024-12-06
 */

#include "flag.h"

/**
 * @brief 제어 플래그 전역 구조체 (시스템 시작 시 모두 0으로 초기화)
 *
 * Control() 함수가 이 구조체를 참조하여 현재 운전 모드를 결정한다.
 * 플래그 변경은 디버거(Live Expression) 또는 UART/SPI 명령으로 수행한다.
 */
struct CONTROL_FLAG FLAG = {
    .READY            = 0,
    .FAULT            = 0,
    .FAULT_CLEAR      = 0,
    .INV_RUN          = 0,
    .INV_OLC          = 0,
    .INV_VOLC         = 0,
    .INV_ALIGN        = 0,
    .INV_NLC          = 0,
    .TS_MODE          = 0,
    .TS_MODE_STOP     = 0,
    .DUTY_TEST        = 0,
    .Param_Estimation = 0,
    .HALL_POS_TEST    = 0
};
