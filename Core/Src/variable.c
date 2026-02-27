/**
 * @file    variable.c
 * @brief   전역 공유 변수 정의 모듈
 *
 * 여러 모듈(inv.c, control.c, speed_observer.c 등)에서 공유되는
 * 전역 변수를 정의한다.
 *
 * @details 포함 변수:
 * - 제어 모드 플래그 (Theta_mode, Align_done)
 * - 속도 PLL 상태 변수 (W_SPD_PLL, Kp/Ki, integ)
 * - 3상 전압 기준 (Van, Vbn, Vcn)
 * - 정지/동기 좌표계 전압 기준 (Vdss_ref_set 등)
 *
 * @note 이 파일의 변수들은 extern 선언으로 다른 모듈에서 참조된다.
 *       헤더 파일(variable.h)에 extern 선언이 위치해야 한다.
 *
 * @author  HALAB_G
 * @date    2024-12-06
 */

/** @defgroup ControlMode 제어 모드 플래그
 *  @{
 */
int Align_done = 0; ///< d축 정렬 완료 플래그 (0: 미완료, 1: 완료)
int Theta_mode = 0; ///< 전기각 소스 선택 (0: Hall PLL, 1: OLC 개루프 각도)
/** @} */

/** @defgroup SpeedPLL 속도 PLL 전역 변수
 *  @{
 */
float W_SPD_PLL        = 0.f; ///< 속도 PLL 자연 주파수 [rad/s]
float Kp_SPD_PLL       = 0.f; ///< 속도 PLL 비례 게인
float Ki_SPD_PLL       = 0.f; ///< 속도 PLL 적분 게인
float integ_Thetar_PLL = 0.f; ///< 속도 PLL 전기각 적분 누산값 [rad]
/** @} */

/** @defgroup VoltageRef 3상 전압 기준 전역 변수
 *  @{
 */
float Van = 0.f; ///< A상 중성점 전압 기준 [V]  (CurrentControl 출력)
float Vbn = 0.f; ///< B상 중성점 전압 기준 [V]
float Vcn = 0.f; ///< C상 중성점 전압 기준 [V]

float Vdss_ref_set = 0.f; ///< 정지 좌표계 α축 전압 기준 [V] (디버깅/모니터링용)
float Vqss_ref_set = 0.f; ///< 정지 좌표계 β축 전압 기준 [V]
float Vdse_ref_set = 0.f; ///< 동기 좌표계 d축 전압 기준 [V]
float Vqse_ref_set = 0.f; ///< 동기 좌표계 q축 전압 기준 [V]
/** @} */
