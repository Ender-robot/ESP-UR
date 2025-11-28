#include "icm20948.hpp"

ICM20948::ICM20948(I2C& driver) 
    : i2c(driver), success(false) {

}

ICM20948::~ICM20948() {

}

/**
 * @brief 切换寄存器组
 * 
 * @param bankNum 寄存器组序号
 */
bool ICM20948::selUserBank(uint8_t bankNum) {
    if (!(bankNum == 0x00 || bankNum == 0x10 || bankNum == 0x20 || bankNum == 0x30)) return false;
    bool ret = i2c.write_byte_to_mem(ICM20948_ADDR, REG_BANK_SEL, bankNum);
    return ret;
}

/**
 * @brief 唤醒传感器
 */
bool ICM20948::wakeUp() {
    if (!selUserBank(USER_BANK_0)) return false;
    bool ret = i2c.write_byte_to_mem(ICM20948_ADDR, PWR_MGMT_1, 0x01);
    return ret;
}

/**
 * @brief 检测MPU20948连接性
 */
bool ICM20948::connective() {
    uint8_t data = 0x00;
    i2c.read_bytes_from_mem(ICM20948_ADDR, WHO_AM_I, &data, 1);
    if (data == 0xEA) return true;
    else return false;
}

/**
 * @brief 初始化设置ICM
 * 
 * @param PWR_MGMT_2_PARAM 唤醒陀螺仪和加速度计，默认启用所有轴
 * @param GYRO_CONFIG_1_PARAM 设置陀螺仪，默认失能低通滤波器，+-500°/s量程
 * @param ACCEL_CONFIG_PARAM 设置加速度计，默认失能低通滤波器，+-4g量程
 * @param I2C_SLV4_DO_PARAM 给从机4的控制命令，默认是磁力计，100Hz连续测量模式
 * 
 * @note 初始化很多参数都是硬编码，可在初始化完后再单独设置
 */
bool ICM20948::init(
    uint8_t PWR_MGMT_2_PARAM,
    uint8_t GYRO_CONFIG_1_PARAM,
    uint8_t ACCEL_CONFIG_PARAM,
    uint8_t I2C_SLV4_DO_PARAM
)
{
    //--- 唤醒并进行连接性检查 ---//
    if (!wakeUp()) return false;
    if (!connective()) return false;

    //--- 使能外设电源 ---//
    selUserBank(USER_BANK_0);
    i2c.write_byte_to_mem(ICM20948_ADDR, PWR_MGMT_2, PWR_MGMT_2_PARAM);

    //--- 设置陀螺仪 ---//
    selUserBank(USER_BANK_2);
    i2c.write_byte_to_mem(ICM20948_ADDR, GYRO_CONFIG_1, GYRO_CONFIG_1_PARAM); // 默认失能低通滤波器，500°量程

    //--- 设置加速度计 ---//
    i2c.write_byte_to_mem(ICM20948_ADDR, ACCEL_CONFIG, ACCEL_CONFIG_PARAM); // 默认失能低通滤波器，+-4g

    //--- 设置磁力计 ---//
    /*磁力计实际是独立与ICM的一颗芯片，可选择直接挂载总线上，或者把ICM当主机，从ICM读取AK磁力计的数据*/
    selUserBank(USER_BANK_0);

    // 默认禁用旁路模式通过ICM与AK通讯
    uint8_t INT_PIN_CFG_DATA;
    i2c.read_bytes_from_mem(ICM20948_ADDR, INT_PIN_CFG, &INT_PIN_CFG_DATA, 1);
    INT_PIN_CFG_DATA &= ~0x02; // 确保禁用旁路模式
    i2c.write_byte_to_mem(ICM20948_ADDR, INT_PIN_CFG, INT_PIN_CFG_DATA);

    // 默认启用主机模式
    uint8_t USER_CTRL_DATA;
    i2c.read_bytes_from_mem(ICM20948_ADDR, USER_CTRL, &USER_CTRL_DATA, 1);
    USER_CTRL_DATA |= 0x20; // 启用主机模式
    i2c.write_byte_to_mem(ICM20948_ADDR, USER_CTRL, USER_CTRL_DATA);
    // 设置主机模式参数
    selUserBank(USER_BANK_3);
    i2c.write_byte_to_mem(ICM20948_ADDR, I2C_MST_CTRL, 0x07); // IIC 时钟频率，默认400kHz（推荐）
    // 配置磁力计
    i2c.write_byte_to_mem(ICM20948_ADDR, I2C_SLV4_ADDR, AK09916_I2C_ADDR); // 指定磁力计地址
    i2c.write_byte_to_mem(ICM20948_ADDR, I2C_SLV4_REG, AK09916_CNTL2); // 指定磁力计的控制寄存器地址
    i2c.write_byte_to_mem(ICM20948_ADDR, I2C_SLV4_DO, I2C_SLV4_DO_PARAM); // 默认连续测量模式100Hz
    i2c.write_byte_to_mem(ICM20948_ADDR, I2C_SLV4_CTRL, 0x80); // 启用传输，执行我们设置的以上操作
    // 为IIC主机设置从何读取磁力计数据
    i2c.write_byte_to_mem(ICM20948_ADDR, I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // AK09916 的 I2C 地址 + 读标志位 (bit 7 = 1) -> 0x0C | 0x80 = 0x8C
    i2c.write_byte_to_mem(ICM20948_ADDR, I2C_SLV0_REG, AK09916_HXL); // 设置IIC主机要读取的磁力计数据起始地址
    i2c.write_byte_to_mem(ICM20948_ADDR, I2C_SLV0_CTRL, 0x89); // 起始位向后读取9个B

    selUserBank(USER_BANK_0);

    return true;
}
