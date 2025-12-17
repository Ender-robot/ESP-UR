#ifndef AHRS_HPP
#define AHRS_HPP

#include <cmath>
#include "struct.hpp"

// 可选的位姿估计算法
namespace AHRS_MODE {
    struct CF{}; // 互补滤波
    struct MahonyQ{}; // 互补滤波四元数
}


/**
 * @brief 用于姿态角估计的类
 * 
 * @param gyroBias 陀螺仪零偏校准数据
 * @param accelBias 加速度计零偏校准数据
 * @param accelGain 加速度计缩放校准数据
 */
class AHRS {
    public:
        AHRS(const Vec3lf gyroBias = {}, const Vec3lf accelBias = {}, const Vec3lf accelGain = {});
        ~AHRS();

        Vec3lf attiEst(const Vec3lf& gyroData, const Vec3lf& accelData, float dt, AHRS_MODE::CF); // 互补滤波
        Vec3lf attiEst(const Vec3lf& gyroData, const Vec3lf& accelData, const Vec3lf& meglData, float dt, AHRS_MODE::MahonyQ); // 互补滤波四元数
    private:
        Vec3lf gyroBias, accelBias, accelGain; // 传感器校准数据
        Vec3lf lastAtti; // 姿态角数据（欧拉角）
        Quaternionlf lastAttiQ; // 姿态角数据（四元数）
};

#endif
