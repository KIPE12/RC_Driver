/*
 * variable.h
 *
 *  Created on: Dec 6, 2024
 *      Author: HALAB_G
 */

#ifndef INC_VARIABLE_H_
#define INC_VARIABLE_H_

////////////////////////////
#define TRUE  1
#define FALSE 0
//#define Tsamp	(1.f/10e3f)
//#define Fsamp	(10e3f)
#define Tsamp	(1.f/10e3f)
#define Fsamp	(10e3f)

#define ON 1
#define OFF 0

//#define ST_TICK_MANAGER(x)    CALK_TERM(x);TICK_START(x);x.cnt++
//#define CALK_TERM(x)       x.term = (DWT->CYCCNT - x.tickStart) * TIME_CALC_us
//#define TICK_START(x)       x.tickStart = DWT->CYCCNT

//#define END_TICK_MANAGER(x) TICK_STOP_CHECk(x);TICK_MAX_CHECk(x);TERM_MAX_CHECk(x)
//#define TICK_STOP_CHECk(x)    x.tickStop = ((DWT->CYCCNT - x.tickStart)   * TIME_CALC_us)
//#define TICK_MAX_CHECk(x)    x.tickStopMax = ((x.tickStop>x.tickStopMax) ? x.tickStop : x.tickStopMax)
//#define TERM_MAX_CHECk(x)    x.termMax = ((x.term>x.termMax) ? x.term : x.termMax)
//
//typedef struct _timeManage {
//   uint32_t cnt;
//   uint32_t tickStart;
//   __IO float tickStop; // time elapsed (us)
//   __IO float tickStopMax; // max time elapsed (us)
//   __IO float term; // period (us)
//   __IO float termMax; // max period (us)
//} timeManage;

////////////////////////////
// Floating-point Constants
#define	PI			3.14159265358979323846f
#define	PI2			6.28318530717958647692f
#define INV_PI2		0.15915494309189533576f
#define INV_PI		0.31830988618379067153f
#define	SQRT2		1.41421356237309504880f
#define	SQRT3		1.73205080756887729352f
#define SQRT3HALF   0.86602540378443864676f
#define	INV_SQRT2	0.70710678118654752440f
#define	INV_SQRT3	0.57735026918962576450f
#define INV_SQRT6   0.40824829046386301636f
#define	INV3		0.33333333333333333333f
#define RM2RPM		9.54929658551372014613f
#define RPM2RM		0.10471975511965977461f
// Floating-point Constants
#define INV_2PI     0.159154943091895f
#define INV_3       ((float)0.333333333333333)
#define INV_6       ((float)0.166666666666667)
#define Rm2Rpm      ((float)9.549296585513720)
#define Rpm2Rm      ((float)0.104719755119659)
#define RAD2DEG     ((float)57.29577951308232)
#define DEG2RAD		((float)0.017453292519943)
#define PIOF6       ((PI*INV_6))
#define PIOF3       ((PI*INV_3))
#define PIOF2       ((PI*0.5))
#define PI5OF6      ((PIOF6*5))


#define f2          (0.5)
#define f3          (f2*INV_3)
#define f4          (f3*0.25)
#define f5          (f4*0.2)
#define f6          (f5*INV_6)
#define f7          (f6*0.14285714285714285714285714285714)
#define f8          (f7*0.125)
#define f9          (f8*0.11111111111111111111111111111111)
#define f10         (f9*0.1)
#define f11         (f10*0.09090909090909090909090909090909)
#define f12         (f11*0.08333333333333333333333333333333)
#define f13         (f12*0.07692307692307692307692307692308)
#define f14         (f13*0.07142857142857142857142857142857)
#define f15         (f14*0.06666666666666666666666666666667)
#define SIN(x, x2)  ((x)*(1.-(x2)*(f3-(x2)*(f5-(x2)*(f7-(x2)*(f9-(x2)*(f11-(x2)*(f13-(x2)*f15))))))))
#define COS(x2)     (1.-(x2)*(f2-(x2)*(f4-(x2)*(f6-(x2)*(f8-(x2)*(f10-(x2)*(f12-(x2)*f14)))))))

// Macro Functions
#define LIMIT(x, m, M)  (((x) > (M)) ? (M) : ((x) < (m)) ? (m) : (x))
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))
#define MIN(a, b)       (((a) > (b)) ? (b) : (a))
#define BOUND_PI(x)     (((x) > 0.f) ? (x) - PI2*((int)(((x)*INV_PI2 + 0.5f))) : (x) - PI2*((int)(((x)*INV_PI2 - 0.5f))))
//#define BOUND_PI(x)     (((x) > 0) ? (x) - 2.*PI*(int)(((x) + PI)/(2.*PI)) : (x) - 2.*PI*(int)(((x) - PI)/(2.*PI)))
#define ABS(x)          (((x) > 0.f) ? (x) : -(x))
#define SIGN(x)         (((x) < 0.f) ? -1.f : 1.f)
#define ROUND(x)        ((int)((x) >= 0 ? (x) + 0.5 : (x) - 0.5))


////////////////////////////
#define SYS_CLOCK          (170.f*1.e6f)

#define TIME_CALC_ns      1.f/(SYS_CLOCK)*1.e9f
#define TIME_CALC_us      1.f/(SYS_CLOCK)*1.e6f

//typedef struct _timeManage {
//   uint32_t cnt;
//   uint32_t tickStart;
//   __IO float tickStop; // time elapsed (us)
//   __IO float tickStopMax; // max time elapsed (us)
//   __IO float term; // period (us)
//   __IO float termMax; // max period (us)
//} timeManage;
//
//#define ST_TICK_MANAGER(x)    CALK_TERM(x);TICK_START(x);x.cnt++
//#define CALK_TERM(x)       x.term = (DWT->CYCCNT - x.tickStart) * TIME_CALC_us
//#define TICK_START(x)       x.tickStart = DWT->CYCCNT
//
//#define END_TICK_MANAGER(x) TICK_STOP_CHECk(x);TICK_MAX_CHECk(x);TERM_MAX_CHECk(x)
//#define TICK_STOP_CHECk(x)    x.tickStop = ((DWT->CYCCNT - x.tickStart)   * TIME_CALC_us)
//#define TICK_MAX_CHECk(x)    x.tickStopMax = ((x.tickStop>x.tickStopMax) ? x.tickStop : x.tickStopMax)
//#define TERM_MAX_CHECk(x)    x.termMax = ((x.term>x.termMax) ? x.term : x.termMax)

#define PWM_BUF_ON	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define PWM_BUF_OFF	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

#define CS_LOW	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)
//////////////////////////////////////////////

extern int Align_done;


///Extended EMF sensorless
extern float W_SPD_PLL, Kp_SPD_PLL, Ki_SPD_PLL, integ_Thetar_PLL;
extern int Theta_mode;


extern float Van, Vbn, Vcn;
extern float Vdss_ref_set, Vqss_ref_set, Vdse_ref_set, Vqse_ref_set;

#endif /* INC_VARIABLE_H_ */
