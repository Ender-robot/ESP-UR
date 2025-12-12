#include "main.hpp"

/*实例化化各外设及硬件驱动*/
I2C i2c(I2C_NUM_0, 15, 16); // 实例化化IIC
ICM20948 icm20948(i2c); // 实例化ICM20948传感器

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
            }
        }

        rawGyroBias.x = sum.x / 500;
        rawGyroBias.y = sum.y / 500;
        rawGyroBias.z = sum.z / 500;

        ESP_LOGI("GyroCail", "Success ! GyroBias: %d  %d  %d", rawGyroBias.x, rawGyroBias.y, rawGyroBias.z);

        return rawGyroBias;
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

    // 将原始16位数据转换为物理量的系数
    double LSB = 65.534; // 陀螺仪

    /*  陀螺仪零偏 */
    Vec3i rawGyroBias;

    /* 校准 */
    rawGyroBias = UTILS::caliGyro(); // 陀螺仪零偏校准

    while (1)
    {
        Vec3i rawData;
        Vec3lf data;

        icm20948.readGyro(rawData); // 读取一次三轴数据

        /* 数据处理 */
        data.x = (rawData.x - rawGyroBias.x) / LSB;
        data.y = (rawData.y - rawGyroBias.y) / LSB;
        data.z = (rawData.z - rawGyroBias.z) / LSB;

        ESP_LOGI("Gyro", "%lf,%lf,%lf", data.x, data.y, data.z); // 数据输出，可以用串口绘图器绘图

        rate.sleep(); // 控制循环频率
    }
}

extern "C" void app_main(void) {
    xTaskCreate(demo, "demo", 4096, NULL, 1, NULL); // 创建RTOS任务
}
