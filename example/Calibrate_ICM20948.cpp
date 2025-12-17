#include "main.hpp"

/*实例化化各外设及硬件驱动*/
I2C i2c(I2C_NUM_0, 15, 16); // 实例化化IIC
ICM20948 icm20948(i2c); // 实例化ICM20948传感器
Flash flash_nvs; // 实例化NVS

/* 传感器lsb */
namespace PARAMS {
    double ACCEL_LSB = 8192.0;
    double GYRO_LSB = 65.534;
}

/* 工具函数 */
namespace UTILS {
    bool caliGyro(Vec3i& rawGyroBias) { // 一定在陀螺仪初始化后使用
        Vec3li sum; //使用长整型，防止溢出
        Vec3i buf; // 暂存数据

        ESP_LOGI("GyroCail", "Start !");
        for (int cnt = 0; cnt < 500; cnt++) {
            if (icm20948.readGyro(buf)) {
                sum.x += buf.x;
                sum.y += buf.y;
                sum.z += buf.z;

                delay_ms(2);
            }
            else {
                ESP_LOGE("GyroCail", "dsiconnection !");
                return false;
            }
        }

        rawGyroBias.x = sum.x / 500;
        rawGyroBias.y = sum.y / 500;
        rawGyroBias.z = sum.z / 500;

        ESP_LOGI("GyroCail", "Success !\nGyroBias: %d  %d  %d", rawGyroBias.x, rawGyroBias.y, rawGyroBias.z);

        return true;
    }

    bool caliAccel(Vec3i& rawAccelBias, Vec3lf& rawAccelGain) {
        Vec3i buf; // 暂存数据
        long sum = 0; // 存储测量和
        int mean[6]; // 存储平均值
        int temp[3]; // 辅助索引存储
        double lsb = PARAMS::ACCEL_LSB;
        const std::string tag[6] = {
            "X+", "X-", "Y+", "Y-", "Z+", "Z-"
        };

        ESP_LOGI("AccelCali", "Start 6-Point Calibration!");

        /* 
            单面采样500，六个面就是3000 
            顺序是（朝上）：
            X+  X-  Y+  Y-  Z+  Z-
        */
        int tempIndex = 0;
        int sign = 1;
        for (int i = 0; i < 6; i++) {
            ESP_LOGI("AccelCali", "Pose: %s", tag[i].c_str());
            for (int j = 0; j < 500; j++) {
                if (!icm20948.readAccel(buf)) {
                    ESP_LOGE("AccelCail", "dsiconnection !");
                    return false;
                }

                temp[0] = buf.x;
                temp[1] = buf.y;
                temp[2] = buf.z;

                /* 只有当数据误差在1g的10%内时该次采样才有效 */
                if (abs(temp[tempIndex] - (sign * PARAMS::ACCEL_LSB)) < lsb * 0.1) {
                    sum += temp[tempIndex];
                }
                else {
                    j--;
                    ESP_LOGW("AccelCali", "Pose erro !");
                    delay_ms(998);
                }
                delay_ms(2);
            }
            mean[i] = sum / 500; // 取平均
            sign *= -1; // 符号取反
            if (!((i + 1) % 2)) tempIndex++; // 偶数次时轴索引前进一
            sum = 0; // 和归零
        }
        
        // 计算Bias和Gain
        rawAccelBias.x = (mean[0] + mean[1]) / 2;
        rawAccelBias.y = (mean[2] + mean[3]) / 2;
        rawAccelBias.z = (mean[4] + mean[5]) / 2;
        rawAccelGain.x = lsb * 2.0 / abs(mean[0] - mean[1]);
        rawAccelGain.y = lsb * 2.0 / abs(mean[2] - mean[3]);
        rawAccelGain.z = lsb * 2.0 / abs(mean[4] - mean[5]);

        ESP_LOGI("AccelCali", "Success !");
        ESP_LOGI("AccelCali", "Bias: %d %d %d", rawAccelBias.x, rawAccelBias.y, rawAccelBias.z);
        ESP_LOGI("AccelCali", "Gain: %lf %lf %lf", rawAccelGain.x, rawAccelGain.y, rawAccelGain.z);

        return true;
    }
}

/* 创建RTOS任务函数 */ 
void demo(void *pvParameters) {
    (void) pvParameters; // 告诉编译器我知道这个没有别警告我

    /* 初始化各外设 */
    if (i2c.init()) {
        ESP_LOGI("I2C", "I2C Init !");
    }
    else {
        ESP_LOGE("I2C", "I2C Init Fail !");
    }

    /* 初始化ICM */
    if (icm20948.init()) {
        ESP_LOGI("ICM", "ICM Init !");
    }
    else {
        ESP_LOGE("ICM", "ICM Init Fail !");
    }

    /* 初始化NVS */
    if (flash_nvs.init()) {
        ESP_LOGI("NVS", "NVS Init !");
    }
    else {
        ESP_LOGE("NVS", "NVS Init Fail !");
    }

    /*  陀螺仪零偏 */
    Vec3i rawGyroBias;

    /* 加速度计零偏和缩放 */
    Vec3i rawAccelBias;
    Vec3lf rawAccelGain;

    /* 校准 */
    if (!UTILS::caliGyro(rawGyroBias)) ESP_LOGE("GyroCali", "GyroCali Fail !"); // 陀螺仪零偏校准
    if (!UTILS::caliAccel(rawAccelBias, rawAccelGain)) ESP_LOGE("AccelCali", "AccelCali Fail !"); // 加速度计校准

    /* 存入NVS */
    if(flash_nvs.saveAsBlob("rawGyroBias", &rawGyroBias, sizeof(rawGyroBias)))
        ESP_LOGI("NVS", "GyroBias saved in key rawGyroBias !");
    else
        ESP_LOGI("NVS", "GyroBias save failed !");
    if(flash_nvs.saveAsBlob("rawAccelBias", &rawAccelBias, sizeof(rawAccelBias)))
        ESP_LOGI("NVS", "AccelBias saved in key rawAccelBias !");
    else
        ESP_LOGI("NVS", "AccelBias save failed !");
    if(flash_nvs.saveAsBlob("rawAccelGain", &rawAccelGain, sizeof(rawAccelGain)))
        ESP_LOGI("NVS", "AccelGain saved in key rawAccelGain !");
    else
        ESP_LOGI("NVS", "AccelGain save failed !");

    /* 初始化任务循环控制类 */
    Rate rate(1);

    /*  nvs陀螺仪零偏 */
    Vec3i _rawGyroBias;
    /* nvs加速度计零偏和缩放 */
    Vec3i _rawAccelBias;
    Vec3lf _rawAccelGain;
    /* 重新从nvs中读取来进行验证 */
    size_t len;
    len = sizeof(_rawGyroBias);
    flash_nvs.readAsBlob("rawGyroBias", &_rawGyroBias, &len);
    len = sizeof(_rawAccelBias);
    flash_nvs.readAsBlob("rawAccelBias", &_rawAccelBias, &len);
    len = sizeof(_rawAccelGain);
    flash_nvs.readAsBlob("rawAccelGain", &_rawAccelGain, &len);
    while (1)
    {
        ESP_LOGI("GyroBias", "%lf, %lf, %lf", _rawGyroBias.x / PARAMS::GYRO_LSB, _rawGyroBias.y / PARAMS::GYRO_LSB, _rawGyroBias.z / PARAMS::GYRO_LSB);
        ESP_LOGI("AccelBias", "%lf, %lf, %lf", _rawAccelBias.x / PARAMS::ACCEL_LSB, _rawAccelBias.y / PARAMS::ACCEL_LSB, _rawAccelBias.z / PARAMS::ACCEL_LSB);
        ESP_LOGI("AccelGain", "%lf, %lf, %lf", _rawAccelGain.x / PARAMS::ACCEL_LSB, _rawAccelGain.y / PARAMS::ACCEL_LSB, _rawAccelGain.z / PARAMS::ACCEL_LSB);

        rate.sleep(); // 控制循环频率
    }
}

extern "C" void app_main(void) {
    xTaskCreate(demo, "demo", 4096, NULL, 1, NULL); // 创建RTOS任务
}
