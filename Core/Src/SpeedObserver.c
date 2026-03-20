/**
 * @file    SpeedObserver.c
 * @author  lsj50
 * @date    Sep 7, 2025
 * @brief   모터의 속도/위치 관측, 초기 위치 정렬(Align) 및 센서 피드백 처리 소스 파일
 *
 * @details [주요 알고리즘 및 하드웨어 가속]
 * 1. **PLL (Phase-Locked Loop) 기반 관측기**: 위치 오차를 PI 연산하여 노이즈에 강인한 속도 및 각도 추정값을 산출합니다.
 * 2. **하드웨어 가속 (CORDIC)**: MCU 내장 CORDIC 코프로세서를 활용하여 Q31 포맷 기반의 고속 Sin/Cos 삼각함수 연산을 수행, 제어 루프의 CPU 연산 부하를 최소화합니다.
 * 3. **다중 센서 인터페이스**: Hall 센서(6-step 전기각 변환) 및 증분형 엔코더(타이머 카운트 기반 기계각 변환) 정보를 모두 처리할 수 있습니다.
 *
 * @details [주요 관측 및 센서 함수 (Functions)]
 * | 함수명 | 주요 파라미터 | 역할 및 특징 |
 * | :--- | :--- | :--- |
 * | **vInitSpeedObserver** | `Motor`, `SObs` | 관측기 PLL 이득(Kp, Ki), 속도 노이즈 필터(IIR) 초기화 및 관련 변수 리셋 |
 * | **vSinCos_Calculation** | `angle`, `*Cos`, `*Sin` | 라디안 각도를 Q31로 변환 후 하드웨어 CORDIC 모듈을 호출하여 결과값을 반환 |
 * | **vSpeedObserver** | `Motor`, `SObs`, `SCtrl` | 제어 모드(V/F 개루프 vs 벡터 제어 폐루프)에 따라 위상각을 생성하거나 PLL을 통해 속도/각도를 관측 |
 * | **fGetHallSensorInfo** | `SObs` | 3상 홀 센서 GPIO 핀 상태를 조합하여 1~6 상태 코드를 만들고, 이를 60도 간격의 전기각으로 출력 |
 * | **fGetEncoderInfo** | `htim`, `SObs` | 증분형 엔코더의 타이머 카운트 레지스터(CNT)를 읽어 기계적 각도(-PI ~ PI)로 스케일링 |
 *
 * @details [초기 회전자 위치 정렬 (Align) 시퀀스]
 * `vAlignHallSensor` 함수는 정확한 FOC 제어를 위한 초기 전기각 오프셋을 찾기 위해 아래의 State Machine 순서로 동작합니다.
 * | 단계 (Step) | 동작 상태 | 상세 설명 |
 * | :--- | :--- | :--- |
 * | **Step 0~1** | 변수 초기화 및 전류 인가 | 관측기 변수를 초기화하고 D축 전류(Idsr) 지령을 목표치까지 점진적으로 램프(Ramp) 인가 |
 * | **Step 2** | 속도 인가 및 에지 탐색 | 미세 속도 지령을 주어 회전자를 돌리며 특정 홀 센서 상태 전환(State 2 &rarr; 6)을 감지 |
 * | **Step 3~4** | 정지 및 회전자 고정 | 목표 상태를 찾으면 속도 지령을 0으로 낮추고, 일정 시간 대기하여 회전자의 기계적 진동을 안정화 |
 * | **Step 5** | 위치 오프셋 연산 | 회전자가 고정된 상태에서 읽힌 각도를 누적 및 평균 내어 정밀한 기준 오프셋 산출 |
 * | **Step 6~7** | 전류 차단 및 정렬 종료 | D축 전류를 다시 0으로 내리고, 연산된 오프셋을 관측기에 적용하며 Align 완료(uAlignEnd=1) 선언 |
 */

#include "MotorControl.h"
#include "Globalvar.h"
#include "UserMath.h"

/** @brief CORDIC 연산을 위한 외부 핸들러 참조 */
extern CORDIC_HandleTypeDef hcordic;

/** @brief 속도 추정값의 노이즈 필터링을 위한 2차 IIR LPF (RPM용) */
IIR2 IIR2WrpmSCLPF;
/** @brief 속도 추정값의 노이즈 필터링을 위한 2차 IIR LPF (rad/s용) */
IIR2 IIR2WrmSCLPF;

/**
 * @brief  속도 및 위치 관측기와 관련된 변수 및 필터를 초기화합니다.
 * @details
 * 1. 위치 정렬(Align) 프로세스 관련 변수를 초기화합니다.
 * 2. 좌표 변환에 필요한 초기 Sin/Cos 값을 설정합니다.
 * 3. 기계 파라미터(관성, 마찰 등)를 기반으로 관측기 및 PLL의 이득을 계산합니다.
 * 4. 추정 속도를 위한 LPF(Low Pass Filter)를 초기화합니다.
 * @param  MotorContorl 모터 물리 파라미터 구조체 포인터
 * @param  SObs 초기화할 속도/위치 관측기 구조체 포인터
 * @retval 없음
 */
void vInitSpeedObserver(sMotorCtrl* MotorContorl, sSpeedObs* SObs){

	SObs->uAlignStep = 0u;
	SObs->lAlignCnt = 0l;

	SObs->fThetarmOffset = 0.0f;
	SObs->fThetarmOffsetTemp = 0.0f;
	SObs->fIdsrRefAlign = 0.0f;
	SObs->fWrRefAlign = 0.0f;
	SObs->fThetarAlign = 0.0f;
	SObs->fThetarAlignComp = 0.0f;
	SObs->uAlignEnd = 0u;	/// Only uses Hall Sensor

	SObs->fDelIdsrAlign = DEL_IDSR_REF_ALIGN * fTsamp;
	SObs->fDelWrRefAlign = DEL_WR_REF_ALIGN * fTsamp;

	SObs-> uHall_A = 0u;
	SObs-> uHall_B = 0u;
	SObs-> uHall_C = 0u;
	SObs-> uHall_State = 0u;
	SObs-> fThetar_Hall =0.0f;

	SObs->fSinThetarCC = 0.0f;
	SObs->fCosThetarCC = 1.0f;
	SObs->fSinThetarCompCC = 0.0f;
	SObs->fCosThetarCompCC = 1.0f;

	SObs->fThetarCC = 0.0f;
	SObs->fThetarCompCC = 0.0f;

	SObs->fWrpmSC = 0.0f;
	SObs->fThetarmEst = 0.0f;
	SObs->fThetarm = 0.0f;
	SObs->fThetarEst = SObs->fThetarm;
	SObs->fThetarmErr = 0.0f;

	SObs->fInvJ = 1.0f / MotorContorl->JM;
	SObs->fBperJ = MotorContorl->BM * SObs->fInvJ;
	SObs->fInvPP = 1.0f / MotorContorl->PP;

	SObs->fWrmEst = 0.0f;
	SObs->fWrmEstLPF = 0.0f;
	SObs->fWrEst = 0.0f;
	SObs->fWrpmEst = 0.0f;
	SObs->fWrpmEstLPF = 0.0f;
	SObs->fAccEstInteg = 0.0f;
	SObs->fAccFF = 0.0f;

	SObs->fThetarIbyF = 0.0f;
	SObs->fThetarCompIbyF = 0.0f;
	SObs->fWrRefIbyF = 0.0f;
	SObs->fWrpmRefIbyF = 0.0f;
	SObs->fDelWrpmRefIbyF = DEL_WRPM_REF_IBYF / 20000.0f;

	SObs->fEncScale = PI2 / (float)((ENCORDER_PPR * 4) - 1);

	SObs->fK1 = WC_SO1 + 2 * ZETA_SO * WC_SO23 - SObs->fBperJ;
	SObs->fK2Ts = fTsamp * (2 * WC_SO1 * ZETA_SO * WC_SO23 + WC_SO23 * WC_SO23 - SObs->fBperJ * SObs->fK1);
	SObs->fK3Ts = fTsamp * (WC_SO1 * WC_SO23 * WC_SO23);

	SObs-> fKpPLL = 2.0f * ZETA_PLL * WC_PLL;
	SObs-> fKiPLL = WC_PLL * WC_PLL;
	SObs-> fThetarmInteg = 0.0f;

	initiateIIR2(&IIR2WrpmSCLPF, K_LPF, WC_WRPMSC_LPF, 0.707, fTsamp);
	initiateIIR2(&IIR2WrmSCLPF, K_LPF, WC_WRM_LPF, 0.707, fTsamp);
}

/**
 * @brief  CORDIC 하드웨어를 활용하여 주어진 각도에 대한 Cosine 및 Sine 값을 동시 계산합니다.
 * @note   이 함수는 입력 각도(rad)를 하드웨어가 요구하는 Q31 포맷으로 변환한 뒤 처리합니다.
 * @param  angle_rad 계산할 각도 (라디안)
 * @param  pCos 계산된 Cosine 값을 저장할 포인터
 * @param  pSin 계산된 Sine 값을 저장할 포인터
 * @retval 없음
 */
static void vSinCos_Calculation(float angle_rad, float *pCos, float *pSin){
	int32_t input_q31;
	int32_t output_q31[2]; // [0]:Cos, [1]:Sin

	const float SCALE_Q31 = 2147483648.0f; // 2^31
	const float INV_SCALE_Q31 = 4.656612873077393e-10;

	// (중요) CORDIC 입력 전 Q31 변환
	input_q31 = (int32_t)((angle_rad / PI) * SCALE_Q31);

	// CORDIC 하드웨어 계산 요청
	HAL_CORDIC_Calculate(&hcordic, &input_q31, output_q31, 1, 0);

	// 결과 변환 (Q31 -> Float)
	*pCos = (float)output_q31[0] * INV_SCALE_Q31;
	*pSin = (float)output_q31[1] * INV_SCALE_Q31;
}

/**
 * @brief  현재 운전 모드에 따라 모터의 속도 및 각도를 추정(관측)합니다.
 * @details
 * - V/F(I/F) 제어 모드일 때는 지령값을 기반으로 개루프(Open-loop) 위상을 적분하여 생성합니다.
 * - 벡터 및 속도 제어 모드일 때는 센서 피드백 오차를 이용한 PLL(Phase-Locked Loop)
 * 알고리즘을 통해 필터링된 속도 및 각도를 추정하고 CORDIC으로 삼각함수를 연산합니다.
 * @param  MotorControl 모터 파라미터 구조체 포인터
 * @param  SObs 속도 및 위치 관측기 구조체 포인터
 * @param  SCtrl 속도 제어기 구조체 포인터 (지령 속도 참조용)
 * @retval 없음
 */
void vSpeedObserver(sMotorCtrl* MotorControl, sSpeedObs* SObs, sSpeedCtrl* SCtrl){
	switch (uControlMode){
	case CONST_CUR_MODE:
		vSlopeGenerator(&SObs->fWrpmRefIbyF, SCtrl-> fWrpmRefSet, SObs->fDelWrpmRefIbyF);    //fWrpmRefSet 으로 변경

		SObs->fWrpmRefIbyF = LIMIT(SObs->fWrpmRefIbyF, 0.0f, 0.8f * MOT_WRPM_RATED);
		SObs->fWrRefIbyF = SObs->fWrpmRefIbyF * RPM2RM * MotorControl->PP;


		SObs->fThetarIbyF = BOUND_PI(SObs->fThetarIbyF + fTsamp * SObs->fWrRefIbyF);
		SObs->fThetarCompIbyF = BOUND_PI(SObs->fThetarIbyF + 1.5f * fTsamp * SObs->fWrRefIbyF);

		SObs->fWrCC = SObs->fWrRefIbyF;
		SObs->fWrpmSC = SObs->fWrRefIbyF * RM2RPM * SObs->fInvPP;

		vSinCos_Calculation(SObs->fThetarIbyF, &SObs->fCosThetarCC, &SObs->fSinThetarCC);
		vSinCos_Calculation(SObs->fThetarCompIbyF, &SObs->fCosThetarCompCC, &SObs->fSinThetarCompCC);
		break;

	case VECTCONTL_MODE:
	case SPDCONTL_MODE:
		//		/* PLL: 위치 오차로 속도/각도 추정 */
		SObs->fThetarErr = BOUND_PI(SObs->fThetar - SObs->fThetarEst);
		SObs->fThetarInteg += fTsamp * SObs->fKiPLL * SObs->fThetarErr;

		SObs->fWrEst     = SObs->fKpPLL * SObs-> fThetarErr + SObs->fThetarInteg;
		SObs->fWrpmEst = RM2RPM * SObs->fWrEst * SObs->fInvPP;

		SObs->fThetarEst += fTsamp * SObs->fWrEst;
		SObs->fThetarEst = BOUND_PI(SObs->fThetarEst);

		SObs->fWrpmEstLPF = IIR2Update(&IIR2WrpmSCLPF, SObs->fWrpmEst);
		SObs->fWrCC = SObs->fWrpmEstLPF * MotorControl->PP * RPM2RM;
		SObs-> fWrpmSC = SObs->fWrpmEstLPF;

		SObs->fThetarCC = SObs->fThetarEst;
		SObs->fThetarCompCC = BOUND_PI(SObs->fThetarEst + 1.5f * SObs->fWrCC * fTsamp);

		vSinCos_Calculation(SObs->fThetarCC, &SObs->fCosThetarCC, &SObs->fSinThetarCC);
		vSinCos_Calculation(SObs->fThetarCompCC, &SObs->fCosThetarCompCC, &SObs->fSinThetarCompCC);

		break;
	}
}

/**
 * @brief  3개의 디지털 홀 센서 입력 핀 상태를 읽어 1~6 사이의 상태 코드로 조합합니다.
 * @param  hall_sensor_1 A상 홀 센서의 핀 상태 (SET/RESET)
 * @param  hall_sensor_2 B상 홀 센서의 핀 상태 (SET/RESET)
 * @param  hall_sensor_3 C상 홀 센서의 핀 상태 (SET/RESET)
 * @retval hall_state 조합된 홀 센서 상태값 (1~6 범위)
 */
uint8_t GetHallSensorState(GPIO_PinState hall_sensor_1, GPIO_PinState hall_sensor_2, GPIO_PinState hall_sensor_3){
	uint8_t hall_state = 0;
	hall_state |= (hall_sensor_1 == GPIO_PIN_SET) ? 0x01 : 0x00;	// LSB
	hall_state |= (hall_sensor_2 == GPIO_PIN_SET) ? 0x02 : 0x00;
	hall_state |= (hall_sensor_3 == GPIO_PIN_SET) ? 0x04 : 0x00;	// MSB
	return hall_state;
}

/**
 * @brief  홀 센서 상태를 기반으로 60도 간격의 회전자 전기각(전기적 위치)을 반환합니다.
 * @param  SObs 속도 및 위치 관측기 구조체 포인터 (핀 상태 저장용)
 * @retval fThetar_HallSensor 홀 센서 상태에 따른 전기적 각도(라디안)
 */
float fGetHallSensorInfo(sSpeedObs* SObs){
	SObs->uHall_A = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	SObs->uHall_B  = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	SObs->uHall_C  = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);

	SObs->uHall_State = GetHallSensorState(SObs->uHall_A, SObs->uHall_B, SObs->uHall_C);

	float fThetar_HallSensor = 0.0f;
	switch (SObs->uHall_State) {
	case 6: fThetar_HallSensor =  0.f;        break;
	case 4: fThetar_HallSensor =  PIBY3;      break;
	case 5: fThetar_HallSensor =  2.f * PIBY3; break;
	case 1: fThetar_HallSensor =  3.f * PIBY3; break;
	case 3: fThetar_HallSensor = -2.f * PIBY3; break;
	case 2: fThetar_HallSensor = -PIBY3;      break;
	}
	return fThetar_HallSensor;
}

/**
 * @brief  홀 센서 기반의 초기 회전자 위치 정렬(Align) 시퀀스를 수행합니다.
 * @details D축 전류를 점진적으로 인가한 뒤, 모터를 미세하게 회전시켜
 * 특정 홀 센서 상태 전환(예: 상태 2 -> 6)을 감지합니다.
 * 이를 통해 기준 전기각 오프셋을 계산하여 벡터 제어의 기준으로 설정합니다.
 * @param  CCtrl 전류 제어기 구조체 포인터 (D축 전류 지령 설정용)
 * @param  SObs 관측기 구조체 포인터 (홀 센서 상태 추적 및 오프셋 연산)
 * @retval 없음
 */
void vAlignHallSensor(sCurrentCtrl* CCtrl, sSpeedObs *SObs){
	static uint8_t uPrev_Hall_State = 0u;
	static uint8_t uCurr_Hall_State = 0u;

	uPrev_Hall_State = uCurr_Hall_State;
	uCurr_Hall_State = SObs->uHall_State;

	switch(SObs->uAlignStep) {
	case 0:	// Clear Variable
		SObs->fThetarmOffset = 0.0f;
		SObs->fIdsrRefAlign = 0.0f;
		SObs->fWrRefAlign = 0.0f;
		SObs->lAlignCnt = 0;


		SObs->uAlignStep++;
		break;

	case 1:	// Current Set
		vSlopeGenerator(&SObs->fIdsrRefAlign, IDSR_REF_SET_ALIGN, SObs->fDelIdsrAlign);
		if(SObs->fIdsrRefAlign == IDSR_REF_SET_ALIGN) SObs->uAlignStep++;
		break;

	case 2:	// Speed Set
		vSlopeGenerator(&SObs->fWrRefAlign, WR_REF_SET_ALIGN, SObs->fDelWrRefAlign);

		if((uPrev_Hall_State == 2u) && (uCurr_Hall_State == 6u)) {	// Find Theta to the uHall_State = 6 --> Next Step
			SObs->uAlignStep++;
		}
		break;

	case 3:	// Speed 0
		vSlopeGenerator(&SObs->fWrRefAlign, 0.0f, 100.0f * SObs->fDelWrRefAlign);
		if(SObs->fWrRefAlign == 0.0f) {
			SObs->fThetarAlign = 0.0f;
			SObs->uAlignStep++;
		}
		break;

	case 4:	// Constant Current	--> Rotor Fix
		SObs->lAlignCnt++;
		if(SObs->lAlignCnt == (ALIGN_CNT_MAX >> 4)) {
			SObs->uAlignStep++;
			SObs->lAlignCnt = 0u;
		}
		break;

	case 5:	// Theta Offset Calculation
		SObs->fINV_AlignCntPlus1 = 1.0f / ((float)SObs->lAlignCnt + 1.0f);

		SObs->fThetarmOffsetTemp = (SObs->fThetarmOffsetTemp * (float)SObs->lAlignCnt + SObs->fThetarm) * SObs->fINV_AlignCntPlus1;

		SObs->lAlignCnt++;

		if(SObs->lAlignCnt == ALIGN_CNT_MAX) {
			SObs->uAlignStep++;
			SObs->lAlignCnt = 0l;
		}
		break;

	case 6:	// Currnet 0
		vSlopeGenerator(&SObs->fIdsrRefAlign, 0.0f, SObs->fDelIdsrAlign);
		if(SObs->fIdsrRefAlign == 0.0f)
			SObs->uAlignStep++;
		break;

	default: // Align End State
		SObs->fThetarmOffset = SObs->fThetarmOffsetTemp;
		SObs->fThetarmOffsetTemp = 0.0f;
		SObs->fIdsrRefAlign = 0.0f;
		SObs->fWrRefAlign = 0.0f;
		SObs->uAlignEnd = 1u;
		SObs->uAlignStep = 0u;
		break;
	}

	SObs->uHall_State = (uint8_t)(GetHallSensorState(SObs->uHall_A, SObs->uHall_B, SObs->uHall_C));

	SObs->fWrCC = SObs->fWrRefAlign;
	CCtrl->fIdsrRef = SObs->fIdsrRefAlign;
	CCtrl->fIqsrRef = 0.0f;
	SObs->fThetarAlign = BOUND_PI(SObs->fThetarAlign + fTsamp * SObs->fWrRefAlign);
	SObs-> fThetarAlignComp = SObs->fThetarAlign + 1.5f * SObs->fWrRefAlign * fTsamp;

	vSinCos_Calculation(SObs->fThetarAlign, &SObs->fCosThetarCC, &SObs->fSinThetarCC);
	vSinCos_Calculation(SObs->fThetarAlignComp, &SObs->fCosThetarCompCC, &SObs->fSinThetarCompCC);
}

/**
 * @brief  증분형 엔코더(Incremental Encoder) 펄스 값을 읽어 기계적 각도(라디안)로 변환합니다.
 * @param  htim 엔코더 타이머 핸들러 포인터 (타이머의 카운트 레지스터 참조)
 * @param  SObs 속도 및 위치 관측기 구조체 포인터 (연산 결과 저장)
 * @retval SObs->fThetarm 엔코더 펄스로부터 변환된 기계적 각도(-PI ~ PI 범위로 정규화)
 */
float fGetEncoderInfo(TIM_HandleTypeDef *htim, sSpeedObs* SObs){

	SObs->fThetarm = BOUND_PI(SObs->fEncScale * (float)(htim->Instance->CNT));

	return SObs->fThetarm;
}
