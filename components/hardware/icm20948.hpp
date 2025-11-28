#ifndef ICM20948_HPP
#define ICM20948_HPP

#include <cmath>
#include "i2c.hpp"
#include "system.hpp"
#include "struct.hpp"
#include "esp_log.h"

/**
 * @brief 九轴IMU ICM20948
 * 
 * @param driver IIC驱动类
 * 
 * @note ICM的地址有两种0x68和0x69，这里默认为0x68，请注意你的传感器配置模式
 *       若是0x69请修改icm20948.hpp内的定义
 */
class ICM20948 {
    public:
        ICM20948(I2C& driver);
        ~ICM20948();

        bool selUserBank(uint8_t bankNum);
        bool wakeUp();
        bool connective();
        bool init(
            uint8_t PWR_MGMT_2_PARAM = 0x00,
            uint8_t GYRO_CONFIG_1_PARAM = 0x03,
            uint8_t ACCEL_CONFIG_PARAM = 0x03,
            uint8_t I2C_SLV4_DO_PARAM = 0x08
        );

        bool readGyro(Vec3f& data); // 读陀螺仪

    private:
        static constexpr const char* TAG = "ICM20948"; // 日志标签
        
        enum PARAMS : uint8_t {
            ICM20948_ADDR        = 0x68, // ICM设备地址
            AK09916_I2C_ADDR     = 0x0C, // AK设备地址

            // AK
            AK09916_CNTL2        = 0x31, // 磁力计的控制寄存器
            AK09916_HXL          = 0x11, // 磁力计数据寄存器的起始地址

            // User Bank
            USER_BANK_0          = 0x00, // 零号寄存器组
            USER_BANK_1          = 0x10, // 一号寄存器组
            USER_BANK_2          = 0x20, // 二号寄存器组
            USER_BANK_3          = 0x30, // 三号寄存器组

            // Bank 0
            REG_BANK_SEL         = 0x7F, // 组选择寄存器
            PWR_MGMT_1           = 0x06, // 主电源及时钟管理寄存器
            PWR_MGMT_2           = 0x07, // 陀螺仪及加速度计电源管理寄存器
            WHO_AM_I             = 0x00, // 设备识别寄存器
            USER_CTRL            = 0x03, // 用与控制传感器的主要功能
            INT_PIN_CFG          = 0x0F, // 中断引脚配置
            EXT_SLV_SENS_DATA_00 = 0x3B, // 在配置好 I2C 主机读取操作后，磁力计的数据将被存放在这些寄存器中
            // Bank 2
            GYRO_CONFIG_1        = 0x01, // 陀螺仪配置1，用于配置滤波器和量程
            ACCEL_CONFIG         = 0x14, // 加速度计配置1，用于配置滤波器和量程
            // Bank 3
            I2C_MST_CTRL         = 0x01, // IIC主机控制
            I2C_SLV0_ADDR        = 0x03, // 设置磁力计的 I2C 地址
            I2C_SLV0_REG         = 0x04, // 设置要读取的磁力计内部寄存器的起始地址
            I2C_SLV0_CTRL        = 0x05, // 启用从机读取操作并设置读取的数据长度
            I2C_SLV4_ADDR        = 0x13, // 用于指定磁力计地址
            I2C_SLV4_REG         = 0x14, // 用于指定磁力计控制寄存器地址
            I2C_SLV4_DO          = 0x16, // 用于为磁力计设置控制位
            I2C_SLV4_CTRL        = 0x15, // 用于启用SLV4的所有设置
        };

        I2C& i2c;
        bool success;
};

#endif
