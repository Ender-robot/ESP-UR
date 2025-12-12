#ifndef MPU9250_HPP
#define MPU9250_HPP

#include <cmath>
#include "i2c.hpp"
#include "system.hpp"
#include "struct.hpp"
#include "esp_log.h"

/**
 * @brief 初始化MPU9250
 * 
 * @param i2c_id  使用I2C类
 * 
 * @note 暂无磁力计
 * 
 * @note MPU的地址有两种0x68和0x69，这里默认为0x68，请注意你的传感器配置模式
 *       若是0x69请修改mpu9250.hpp内的定义
*/
class MPU9250 {
    public:
        MPU9250(I2C& driver);
        ~MPU9250();

        bool init(
            uint8_t SMPLRT_DIV_BYTE       = 0x01,
            uint8_t GYRO_CONFIG_BYTE      = 0x08, 
            uint8_t CONFIG_BYTE           = 0x00,
            uint8_t ACCEL_CONFIG_BYTE     = 0x08, 
            uint8_t ACCEL_CONFIG_2_BYTE   = 0x00,
            float _GYRO_LSB               = 65.534,
            float _ACCEL_LSB              = 8192.0
        ); // 初始化

        bool connective(); // 检测连接是否成功
        bool wake_up(); // 唤醒传感器

        bool read_gyro(Vec3lf& data); // 读陀螺仪
        bool cail_gyro(Vec3lf& cailData); // 校准陀螺仪
        bool read_accel(Vec3lf& data); // 读加速度计
        bool cail_accel(Vec3lf& cailBiasData, Vec3lf& cailGainData); // 校准加速度计

    private:
        static constexpr const char* TAG = "MPU9250"; // 日志标签

        enum REG : uint8_t {
            MPU_ADDR          = 0x68, // MPU9250的默认地址
            WHO_AM_I_REG      = 0x75, // Should return 0x71
            PWR_MGMT_1        = 0x6B,
            SMPLRT_DIV        = 0x19,
            CONFIG            = 0x1A,
            GYRO_CONFIG       = 0x1B,
            ACCEL_CONFIG      = 0x1C,
            ACCEL_CONFIG_2    = 0x1D,
            INT_PIN_CFG       = 0x37,
            USER_CTRL         = 0x6A,

            ACCEL_XOUT_H      = 0x3B,
            GYRO_XOUT_H       = 0x43,
            TEMP_OUT_H        = 0x41
        };

        I2C& i2c;
        float GYRO_LSB;
        float ACCEL_LSB;
        bool success;
};

#endif
