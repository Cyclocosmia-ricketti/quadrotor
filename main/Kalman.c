
#include "kalman.h"

double xQ_angle; // 处理加速度计的离散噪声
double xQ_bias; // 处理陀螺仪的离散偏差噪声
double xR_measure; // 测量噪声 - this is actually the variance of the measurement noise

double xangle; // 通过卡尔曼滤波器计算角度- part of the 2x1 state vector
double xbias; // 通过卡尔曼滤波器计算陀螺仪偏差- part of the 2x1 state vector
double xrate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

double xP[2][2]; // 误差协方差矩阵- This is a 2x2 matrix（矩阵）
double xK[2]; //卡尔曼增益 - This is a 2x1 vector（向量）
double xy; // 角度差，加速度计通过三角函数计算的角度减去无偏陀螺仪数据积分得到的角度 的差
double xS; // 估计误差

void xInit_Kalman(void){
	xQ_angle = 0.001;
	xQ_bias = 0.03;
	xR_measure = 0.3;

	xangle = 0; // 清零角度
	xbias = 0; //清零偏差

	xP[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	xP[0][1] = 0;
	xP[1][0] = 0;
	xP[1][1] = 0;
}

void xKalmanSetAngle(double newAngle)
{
	xangle=newAngle;
}

double xgetAngle(double newAngle, double newRate, double dt) {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        xrate = newRate - xbias;
        xangle += dt * xrate;

        // 更新 误差协方差- Project the error covariance ahead
        /* Step 2 */
        xP[0][0] += dt * (dt*xP[1][1] - xP[0][1] - xP[1][0] + xQ_angle);
        xP[0][1] -= dt * xP[1][1];
        xP[1][0] -= dt * xP[1][1];
        xP[1][1] += xQ_bias * dt;

        // 离散卡尔曼滤波器测量更新方程式- 测量更新 ("Correct")
        // 计算卡尔曼增益 - Compute the Kalman gain
        /* Step 4 */
        xS = xP[0][0] + xR_measure;
        /* Step 5 */
        xK[0] = xP[0][0] / xS;
        xK[1] = xP[1][0] / xS;

        // 计算角度和偏差 - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        xy = newAngle - xangle;
        /* Step 6 */
        xangle += xK[0] * xy;
        xbias += xK[1] * xy;

        // 计算 误差协方差 - Update the error covariance
        /* Step 7 */
        xP[0][0] -= xK[0] * xP[0][0];
        xP[0][1] -= xK[0] * xP[0][1];
        xP[1][0] -= xK[1] * xP[0][0];
        xP[1][1] -= xK[1] * xP[0][1];

        return xangle;
}




double yQ_angle; // Process noise variance for the accelerometer
double yQ_bias; // Process noise variance for the gyro bias
double yR_measure; // Measurement noise variance - this is actually the variance of the measurement noise

double yangle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
double ybias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
double yrate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

double yP[2][2]; // Error covariance matrix - This is a 2x2 matrix
double yK[2]; // Kalman gain - This is a 2x1 vector
double yy; // Angle difference
double yS; // Estimate error

void yInit_Kalman(void){
	yQ_angle = 0.001;
	yQ_bias = 0.03;
	yR_measure = 0.3;

	yangle = 0; // Reset the angle
	xbias = 0; // Reset bias

	yP[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	yP[0][1] = 0;
	yP[1][0] = 0;
	yP[1][1] = 0;
}

void yKalmanSetAngle(double newAngle)
{
	yangle=newAngle;
}

 double ygetAngle(double newAngle, double newRate, double dt) {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        yrate = newRate - ybias;
        yangle += dt * yrate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        yP[0][0] += dt * (dt*yP[1][1] - yP[0][1] - yP[1][0] + yQ_angle);
        yP[0][1] -= dt * yP[1][1];
        yP[1][0] -= dt * yP[1][1];
        yP[1][1] += yQ_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        yS = yP[0][0] + yR_measure;
        /* Step 5 */
        yK[0] = yP[0][0] / yS;
        yK[1] = yP[1][0] / yS;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        yy = newAngle - yangle;
        /* Step 6 */
        yangle += yK[0] * yy;
        ybias += yK[1] * yy;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        yP[0][0] -= yK[0] * yP[0][0];
        yP[0][1] -= yK[0] * yP[0][1];
        yP[1][0] -= yK[1] * yP[0][0];
        yP[1][1] -= yK[1] * yP[0][1];

        return yangle;
}













double zQ_angle; // Process noise variance for the accelerometer
double zQ_bias; // Process noise variance for the gyro bias
double zR_measure; // Measurement noise variance - this is actually the variance of the measurement noise

double zangle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
double zbias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
double zrate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

double zP[2][2]; // Error covariance matrix - This is a 2x2 matrix
double zK[2]; // Kalman gain - This is a 2x1 vector
double zy; // Angle difference
double zS; // Estimate error

void zInit_Kalman(void){
	zQ_angle = 0.001;
	zQ_bias = 0.003;
	zR_measure = 0.03;

	zangle = 0; // Reset the angle
	zbias = 0; // Reset bias

	zP[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	zP[0][1] = 0;
	zP[1][0] = 0;
	zP[1][1] = 0;
}

void zKalmanSetAngle(double newAngle)
{
	zangle=newAngle;
}

 double zgetAngle(double newAngle, double newRate, double dt) {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        zrate = newRate - zbias;
        zangle += dt * zrate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        zP[0][0] += dt * (dt*zP[1][1] - zP[0][1] - zP[1][0] + zQ_angle);
        zP[0][1] -= dt * zP[1][1];
        zP[1][0] -= dt * zP[1][1];
        zP[1][1] += zQ_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        zS = zP[0][0] + zR_measure;
        /* Step 5 */
        zK[0] = zP[0][0] / zS;
        zK[1] = zP[1][0] / zS;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        zy = newAngle - zangle;
        /* Step 6 */
        zangle += zK[0] * zy;
        zbias += zK[1] * zy;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        zP[0][0] -= zK[0] * zP[0][0];
        zP[0][1] -= zK[0] * zP[0][1];
        zP[1][0] -= zK[1] * zP[0][0];
        zP[1][1] -= zK[1] * zP[0][1];

        return zangle;
}

























