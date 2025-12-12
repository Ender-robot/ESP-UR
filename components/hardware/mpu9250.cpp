#include "mpu9250.hpp"

/**
 * @brief 初始化MPU9250
 * 
 * @param driver  I2C驱动类
*/
MPU9250::MPU9250(I2C& driver)
    : i2c(driver), GYRO_LSB(1.0), ACCEL_LSB(1.0), success(false) {
}

MPU9250::~MPU9250() {
}

/**
 * @brief 初始化MPU9050
 * 
 * @return true  成功
 * @return false 失败
 * 
 * @note SMPLRT_DIV_BYTE     采样频率  
 *       GYRO_CONFIG_BYTE    陀螺仪设置（默认量程下LSB = 65.534） 
 *       CONFIG_BYTE         陀螺仪滤波器设置（默认无滤波） 
 *       ACCEL_CONFIG_BYTE   加速度计设置（默认量程下LSB = 8192） 
 *       ACCEL_CONFIG_2_BYTE 加速度计滤波器设置（默认无滤波） 
 *       _GYRO_LSB           数字量到物理量的转换  
 *       _ACCEL_LSB          数字量到物理量的转换  
*/
bool MPU9250::init(
    uint8_t SMPLRT_DIV_BYTE,
    uint8_t GYRO_CONFIG_BYTE, 
    uint8_t CONFIG_BYTE, 
    uint8_t ACCEL_CONFIG_BYTE, 
    uint8_t ACCEL_CONFIG_2_BYTE,
    float _GYRO_LSB,
    float _ACCEL_LSB
) {
    /* 唤醒并检查连接 */
    wake_up();
    success = connective();
    if (!success) return success;

    /* 设置 */
    i2c.write_byte_to_mem(MPU_ADDR, SMPLRT_DIV, SMPLRT_DIV_BYTE); // 设置MPU采样率

    i2c.write_byte_to_mem(MPU_ADDR, GYRO_CONFIG, GYRO_CONFIG_BYTE); // 设置陀螺仪量程，该量程下LSB = 65.534
    i2c.write_byte_to_mem(MPU_ADDR, CONFIG, CONFIG_BYTE); // 设置陀螺仪高频滤波器
    
    i2c.write_byte_to_mem(MPU_ADDR, ACCEL_CONFIG, ACCEL_CONFIG_BYTE); // 设置加速度计量程，该量程下LSB = 8192
    i2c.write_byte_to_mem(MPU_ADDR, ACCEL_CONFIG_2, ACCEL_CONFIG_2_BYTE); // 设置加速度计低通滤波器

    /* 初始化LSB */
    GYRO_LSB = _GYRO_LSB;
    ACCEL_LSB = _ACCEL_LSB;

    return success;
}

/**
 * @brief 检查与MPU9050的连接
 * 
 * @note MPU系列的I_AM_WHO寄存器的值有很多种这里只检测0x71,0x75,0x70(这是内置MPU6500)
 * 
 * @return true  连接
 * @return false 未连接
*/
bool MPU9250::connective() {
    uint8_t data_byte;
    i2c.read_bytes_from_mem(MPU_ADDR, WHO_AM_I_REG, &data_byte, 1);
    if (data_byte == 0x71 || data_byte == 0x75 || data_byte == 0x70)
        return true;
    return false;
}

/**
 * @brief 将MPU从休眠模式唤醒
 */
bool MPU9250::wake_up() {
    bool _success = i2c.write_byte_to_mem(MPU_ADDR, PWR_MGMT_1, 0x01); // 唤醒MPU并设置时钟
    return _success;
}

/**
 * @brief 一次性读取MPU的三轴角速度
 * 
 * @param data 存储3轴角速度数据的float数组的首地址，长度为三
 * 
 * @return true  成功读取
 * @return false 未成功读取
*/
bool MPU9250::read_gyro(Vec3lf& data) {
    if (!success) return false;

    uint8_t raw_data[6]; // 原始数据

    bool ret = i2c.read_bytes_from_mem(MPU_ADDR, GYRO_XOUT_H, raw_data, 6); // 连续读取6位

    if (ret) { // 进行数据处理
        int16_t gyro_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        int16_t gyro_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        int16_t gyro_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

        data.x = gyro_x_raw / GYRO_LSB;
        data.y = gyro_y_raw / GYRO_LSB;
        data.z = gyro_z_raw / GYRO_LSB;

        return true;
    }
    else return false;
}

/**
 * @brief 一次性读取MPU的三轴加速度
 * 
 * @param data 存储3轴加速度数据的float数组的首地址，长度为三
 * 
 * @return true  成功读取
 * @return false 未成功读取
*/
bool MPU9250::read_accel(Vec3lf& data) {
    if (!success) return false;

    uint8_t raw_data[6]; // 原始数据

    bool ret = i2c.read_bytes_from_mem(MPU_ADDR, ACCEL_XOUT_H, raw_data, 6); // 连续读取6位

    if (ret) { // 进行数据处理
        int16_t accel_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        int16_t accel_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        int16_t accel_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

        data.x = accel_x_raw / ACCEL_LSB;
        data.y = accel_y_raw / ACCEL_LSB;
        data.z = accel_z_raw / ACCEL_LSB;

        return true;
    }
    else return false;
}

/**
 * @brief 取多组数据进行陀螺仪零偏校准
 * 
 * @param cailData 存储校准数据
*/
bool MPU9250::cail_gyro(Vec3lf& cailData) {
    if (!success) return false;

    Vec3lf data;
    float x_avg, y_avg, z_avg, x_err, y_err, z_err;
    int cnt;
    Rate rate(50);
    cnt = 1;

    ESP_LOGI(TAG, "gyro cail begining !");
    // 取得首次测量值
    read_gyro(data);
    x_avg = data.x; y_avg = data.y; z_avg = data.z;

    for (int i = 0; i < 200; i++) {
        rate.sleep(); // 延时控制频率
        
        read_gyro(data);

        // 计算本次噪声
        x_err = data.x - x_avg;
        y_err = data.y - y_avg;
        z_err = data.z - z_avg;

        if (fabsf(x_err )> 1.0f || fabsf(y_err) > 1.0f || fabsf(z_err) > 1.0f) continue; // 若测出远高于其他值的值就丢弃

        cnt ++;
        x_avg += x_err / cnt; 
        y_avg += y_err / cnt; 
        z_avg += z_err / cnt; 
    }

    if (cnt >= 150) {
        ESP_LOGI(TAG, "gyro cail successful !");
        cailData.x = x_avg; cailData.y = y_avg; cailData.z = z_avg;
        ESP_LOGI(TAG, "bias: x %.2f, y %.2f, z %.2f", cailData.x, cailData.y, cailData.z);
        return true;
    }
    else {
        ESP_LOGW(TAG, "gyro cail failed !");
        cailData.x = 0.0; cailData.y = 0.0; cailData.z = 0.0;
        return false;
    }
}

/**
 * @brief 六面法校准加速度计
 * 
 * @param cailBiasData 存储零偏校准数据
 * @param cailGainData 存储增益校准数据
*/
bool MPU9250::cail_accel(Vec3lf& cailBiasData, Vec3lf& cailGainData) {
    int status = 0; // 记录校准阶段
    int cnt;
    Vec3lf data; // 采样读数暂存
    float posSum[3] = {0, 0, 0}; // 正采样值和暂存
    float negSum[3] = {0, 0, 0}; // 负采样值和暂存
    Rate rate(50); // 采样频率控制

    const float GAIN = 1.0; // 1g
    const float TOL = 0.1; // 自动检查摆放时所允许的与1g的差值
    const int SAMPLE = 150; // 每个轴向的一个方向的采样数

    // 采样取均值
    for (;status <3; status++) {
        if (status == 0) ESP_LOGI(TAG, "x accel cail begining !");
        else if (status == 1) ESP_LOGI(TAG, "y accel cail begining !");
        else if (status == 2) ESP_LOGI(TAG, "z accel cail begining !");

        // 正面取值
        ESP_LOGI(TAG, "pos");
        for (cnt = 0; cnt < SAMPLE; ) { // 采样数记录
            read_accel(data);
            double dataList[3] = {data.x, data.y, data.z}; // 为了能使用索引把结构体重新存入数组

            if (fabsf(dataList[status] - GAIN) < TOL) { // 检测摆放是否正确
                posSum[status] += dataList[status];
                cnt ++;
                ESP_LOGI(TAG, "collected %d", cnt);
            }

            rate.sleep();
        }

        // 反面取值
        ESP_LOGI(TAG, "neg");
        for (cnt = 0; cnt < SAMPLE; ) { // 采样数记录
            read_accel(data);
            double dataList[3] = {data.x, data.y, data.z}; // 为了能使用索引把结构体重新存入数组

            if (fabsf(dataList[status] + GAIN) < TOL) { // 检测摆放是否正确
                negSum[status] += dataList[status];
                cnt ++;
                ESP_LOGI(TAG, "collected %d", cnt);
            }

            rate.sleep();
        }
    }

    // 零偏计算
    cailBiasData.x = ((posSum[0] / SAMPLE) + (negSum[0] / SAMPLE)) / 2;
    cailBiasData.y = ((posSum[1] / SAMPLE) + (negSum[1] / SAMPLE)) / 2;
    cailBiasData.z = ((posSum[2] / SAMPLE) + (negSum[2] / SAMPLE)) / 2;

    // 增益计算
    cailGainData.x = ((posSum[0] / SAMPLE) - (negSum[0] / SAMPLE)) / 2;
    cailGainData.y = ((posSum[1] / SAMPLE) - (negSum[1] / SAMPLE)) / 2;
    cailGainData.z = ((posSum[2] / SAMPLE) - (negSum[2] / SAMPLE)) / 2;

    ESP_LOGI(TAG, "cail successful !");
    ESP_LOGI(TAG, "bias: x %.2f, y %.2f, z %.2f", cailBiasData.x, cailBiasData.y, cailBiasData.z);
    ESP_LOGI(TAG, "gain: x %.2f, y %.2f, z %.2f", cailGainData.x, cailGainData.y, cailGainData.z);

    return true;
}
