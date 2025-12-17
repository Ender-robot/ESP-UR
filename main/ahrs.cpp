#include "ahrs.hpp"

AHRS::AHRS(const Vec3lf gyroBias_, const Vec3lf accelBias_, const Vec3lf accelGain_) :
    gyroBias(gyroBias_), 
    accelBias(accelBias_), 
    accelGain(accelGain_) {
}

AHRS::~AHRS() {
}

/**
 * @brief 互补滤波姿态角估计，由与欧拉角存在万象死锁的问题，因此会导致中间角趋近90度时另外两个角无法稳定计算。
 * 因此该算法不适于高速机动的结构。（右手系）
 * 
 * @param gyroData 陀螺仪数据
 * @param accelData 加速度计数据
 */
Vec3lf AHRS::attiEst(const Vec3lf& gyroData, const Vec3lf& accelData, float dt, AHRS_MODE::CF) {
    const float T = 0.2; // 越大代表对陀螺仪数据越信任，纠正力度越小。
    const float ALPHA = T / (T + dt);
    Vec3lf accelAtti;
    Vec3lf cailGyro;
    Vec3lf cailAccel;
    Vec3lf gyroPredAtti;
    Vec3lf err;

    /**
     * 对加速度计进行缩放和零偏校准，对陀螺仪进行零偏校准
     */
    cailGyro.x =  (gyroData.x - gyroBias.x);
    cailGyro.y =  (gyroData.y - gyroBias.y);
    cailGyro.z =  (gyroData.z - gyroBias.z);
    cailAccel.x =  (accelData.x - accelBias.x) / accelGain.x;
    cailAccel.y =  (accelData.y - accelBias.y) / accelGain.y;
    cailAccel.z =  (accelData.z - accelBias.z) / accelGain.z;

    /**
     * 加速度计可以在静止或匀速运动时解算出稳定的姿态角用于校准陀螺仪积分漂变，
     * 但其无法区分运动加速度，同时也无法解出Yaw所以须与陀螺仪和磁力计融合。
     * 存在万向死锁问题pitch趋于+-90度时Roll和Yaw无法稳定解算。
     */
    accelAtti.x = atan2f(cailAccel.y, cailAccel.z) * 180 / M_PI; // 加速度计解算Roll
    accelAtti.y = atan2f(-cailAccel.x, sqrtf(cailAccel.y * cailAccel.y + cailAccel.z * cailAccel.z)) * 180 / M_PI; // 加速度计解算Pitch

    // 陀螺仪积分
    gyroPredAtti.x = cailGyro.x * dt + lastAtti.x;
    gyroPredAtti.y = cailGyro.y * dt + lastAtti.y;
    gyroPredAtti.z = cailGyro.z * dt + lastAtti.z;

    // 互补滤波（暂时缺乏磁力计数据）
    if (fabsf(gyroPredAtti.y) < 75) { // 非万象死锁时启用
        // 计算误差消除积分漂变（暂无磁力计数据）
        err.x = accelAtti.x - gyroPredAtti.x;
        err.y = accelAtti.y - gyroPredAtti.y;

        // 角度环绕，让误差走最短路径
        if (err.x > 180.0f) err.x -= 360;
        else if (err.x < -180.0f) err.x += 360;
        if (err.y > 180.0f) err.y -= 360;
        else if (err.y < -180.0f) err.y += 360; 
        if (err.z > 180.0f) err.z -= 360;
        else if (err.z < -180.0f) err.z += 360;  

        // 滤波
        lastAtti.x = gyroPredAtti.x + (1.0f - ALPHA) * err.x;
        lastAtti.y = gyroPredAtti.y + (1.0f - ALPHA) * err.y;
        lastAtti.z = gyroPredAtti.z; // 暂时没有磁力计数据
    }
    else { // 若发生万象死锁就纯积分
        lastAtti = gyroPredAtti;
    }

    // 将角度控制到-180~0~180度
    if (lastAtti.x > 180.0f) lastAtti.x -= 360.0f;
    else if (lastAtti.x < -180.0f) lastAtti.x += 360.0f;
    if (lastAtti.y > 180.0f) lastAtti.y -= 360.0f;
    else if (lastAtti.y < -180.0f) lastAtti.y += 360.0f;
    if (lastAtti.z > 180.0f) lastAtti.z -= 360.0f;
    else if (lastAtti.z < -180.0f) lastAtti.z += 360.0f;

    return lastAtti;
}


