#include "main.hpp"

/*实例化化各外设及硬件驱动*/
I2C i2c(I2C_NUM_0, 15, 16); // 实例化化IIC
ICM20948 icm20948(i2c); // 实例化ICM20948传感器

/* 参数 */
namespace PARAMS {
    const double GYRO_LSB = 65.534; // 陀螺仪
    const double ACCEL_LSB = 8192.0; // 加速度计
}

/* 工具函数 */
namespace UTILS {
    Vec3i caliGyro() { // 一定在陀螺仪初始化后使用
        Vec3li sum; //使用长整型，防止溢出
        Vec3i buf; // 暂存数据
        Vec3i rawGyroBias; // 陀螺仪原始数据零偏

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
                cnt--;
                delay_ms(200);
            }
        }

        rawGyroBias.x = sum.x / 500;
        rawGyroBias.y = sum.y / 500;
        rawGyroBias.z = sum.z / 500;

        ESP_LOGI("GyroCail", "Success ! GyroBias: %d  %d  %d", rawGyroBias.x, rawGyroBias.y, rawGyroBias.z);

        return rawGyroBias;
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
                if (!icm20948.readAccel(buf)) return false;
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

    /* 初始化任务循环控制类 */
    Rate rate(50);

    /*  陀螺仪零偏 */
    Vec3i rawGyroBias;

    /* 加速度计零偏和缩放 */
    Vec3i rawAccelBais;
    Vec3lf rawAccelGain;

    /* 校准 */
    rawGyroBias = UTILS::caliGyro(); // 陀螺仪零偏校准
    if (!UTILS::caliAccel(rawAccelBais, rawAccelGain)) ESP_LOGE("AccelCali", "AccelCali Fail !"); // 加速度计校准

    while (1)
    {
        Vec3i rawData;
        Vec3lf data;

        icm20948.readGyro(rawData); // 读取一次三轴数据

        /* 数据处理 */
        data.x = (rawData.x - rawGyroBias.x) / PARAMS::GYRO_LSB;
        data.y = (rawData.y - rawGyroBias.y) / PARAMS::GYRO_LSB;
        data.z = (rawData.z - rawGyroBias.z) / PARAMS::GYRO_LSB;

        // ESP_LOGI("Gyro", "%lf,%lf,%lf", data.x, data.y, data.z); // 数据输出，可以用串口绘图器绘图

        rate.sleep(); // 控制循环频率
    }
}

extern "C" void app_main(void) {
    xTaskCreate(demo, "demo", 4096, NULL, 1, NULL); // 创建RTOS任务
}
