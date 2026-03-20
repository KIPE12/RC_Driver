/**
 * @file    UserMath.h
 * @author  lsj50
 * @date    Aug 25, 2025
 * @brief   고속 제어 연산을 위한 수학 상수 및 매크로 함수 정의 헤더 파일
 * @details 표준 라이브러리의 math.h보다 빠른 연산을 위해 테일러 급수 기반 근사식과
 * 모터 제어에 자주 쓰이는 각종 변환 계수(RPM <-> Rad/s 등)를 정의합니다.
 */

#ifndef INC_USERMATH_H_
#define INC_USERMATH_H_

/** @name 테일러 급수용 팩토리얼 역수 상수 (Triangular Function Constants)
 * @details SIN/COS 매크로 연산 시 팩토리얼 분모 항을 미리 계산한 값 (1/n!)
 * @{ */
#define f2          ((float)0.5)
#define f3          ((float)0.16666666666666666666666666666667)
#define f4          ((float)0.04166666666666666666666666666667)
#define f5          ((float)0.00833333333333333333333333333333)
#define f6          ((float)0.00138888888888888888888888888889)
#define f7          ((float)1.9841269841269841269841269841e-4)
#define f8          ((float)2.480158730158730158730158730125e-5)
#define f9          ((float)2.75573192239858906525573192e-6)
#define f10         ((float)2.7557319223985890652557319e-7)
#define f11         ((float)2.505210838544171877505211e-8)
#define f12         ((float)2.08767569878680989792101e-9)
#define f13         ((float)1.6059043836821614599392e-10)
#define f14         ((float)1.147074559772972471385e-11)
#define f15         ((float)7.6471637318198164759e-13)
/** @} */

/** @name 부동소수점 수학 상수 (Floating-point Constants)
 * @{ */
#define PI          ((float)3.1415926535897932384626433832795)      /**< 원주율 */
#define PI2         ((float)6.283185307179586476925286766559)       /**< 2 * PI */
#define SQRT2       ((float)1.4142135623730950488016887242097)      /**< 루트 2 */
#define SQRT3       ((float)1.7320508075688772935274463415059)      /**< 루트 3 */
#define INV3        ((float)0.3333333333333333333333333333333)      /**< 1 / 3 */
#define INV_SQRT3   ((float)0.57735026918962576450914878050196)     /**< 1 / 루트 3 */
#define INV_SQRT2   ((float)0.70710678118654752440084436210485)     /**< 1 / 루트 2 */
#define SQRT3HALF   ((float)0.86602540378443864676372317075294)     /**< 루트 3 / 2 */
#define INV_PI      ((float)0.31830988618379067153776752674503)     /**< 1 / PI */
#define INV_2PI     ((float)0.15915494309189533576888376337251)     /**< 1 / 2PI */
#define PIBY3       ((float)1.0471975511965977461542144610932)      /**< PI / 3 */
#define INV_PIBY3   ((float)0.9549296585513720146133025802350)      /**< 3 / PI */

/** @brief RPM 단위를 Mechanical Radian/s 단위로 변환하는 계수 (2*PI/60) */
#define RPM2RM      ((float)0.1047197551196597746154214461093)
/** @brief Mechanical Radian/s 단위를 RPM 단위로 변환하는 계수 (60/2*PI) */
#define RM2RPM      ((float)9.5492965855137201461330258023509)
/** @brief Degree 단위를 Radian 단위로 변환하는 계수 (PI/180) */
#define DEG2RAD     ((float)0.01745329251994329576923690768489)
/** @} */

/** @name 매크로 함수 (Macro Functions)
 * @{ */

/** @brief 하한/상한 제한 (Saturation) 함수 */
#define LIMIT(x,s,l)            (((x)>(l))?(l):((x)<(s))?((s)):(x))

/** @brief 최댓값 반환 함수 */
#define MAX(a, b)               ((a)>(b) ? (a) : (b))

/** @brief 최솟값 반환 함수 */
#define MIN(a, b)               ((a)>(b) ? (b) : (a))

/** @brief 입력 각도를 -PI ~ PI 범위로 정규화 (Phase Wrapping) */
#define BOUND_PI(x)             (((x)>0.)?((x)-PI2*(int)(((x)+PI)*INV_2PI)):((x)-PI2*(int)(((x)-PI)*INV_2PI)))

/** @brief 절대값 반환 함수 */
#define ABS(x)                  (((x)>0.)?(x):(-(x)))

/** @brief 부호 판별 함수 (-1 또는 1 반환) */
#define SIGN(x)                 (((x)<0.)? -1. : 1. )

/** * @brief  테일러 급수 기반 고속 Sine 연산 매크로
 * @param  x  각도 (Radian)
 * @param  x2 각도의 제곱 (x * x)
 */
#define SIN(x,x2)               ((x)*(1.-(x2)*(f3-(x2)*(f5-(x2)*(f7-(x2)*(f9-(x2)*(f11-(x2)*(f13-(x2)*f15))))))))

/** * @brief  테일러 급수 기반 고속 Cosine 연산 매크로
 * @param  x2 각도의 제곱 (x * x)
 */
#define COS(x2)                 (1.-(x2)*(f2-(x2)*(f4-(x2)*(f6-(x2)*(f8-(x2)*(f10-(x2)*(f12-(x2)*f14)))))))
/** @} */

#endif /* INC_USERMATH_H_ */
