#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_
#include "main.h"
typedef struct
{
	float R;         //测量噪声
	float Q;         //过程噪声
	float p_past;    //上一时刻误差协方差
	float p_now;     //上一时刻过程对当前时刻估计值的协方差
	float K;         //卡尔曼增益
	float output;
	float output_Max;
}Kalman_TypeDef;
typedef struct
{
	float R;
	float Q_angle;
	float Q_gyro;

	float angle;
	float Q_bias;

	float P[2][2];
	float K[2];
	float output;
}IMU_fliter_TypeDef;
extern Kalman_TypeDef Klm_Motor[6];
extern IMU_fliter_TypeDef imu_fliter[3];
extern void KalmanFilter_Init(Kalman_TypeDef *klm_typedef);
extern float KalmanFilter(Kalman_TypeDef *f, float input);
extern void IMU_fliter_Init(IMU_fliter_TypeDef *f);
extern void IMU_fliter(IMU_fliter_TypeDef *f,float newangle,float newgyro);
#endif /* INC_KALMAN_H_ */
