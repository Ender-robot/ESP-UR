#include "main.hpp"

/*实例化化各外设及硬件驱动*/
I2C i2c(I2C_NUM_0, 15, 16); // 实例化化IIC
ICM20948 icm20948(i2c); // 实例化ICM20948传感器

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

    while (1)
    {
        double LSB = 65.534; // 将原始16位数据转换为物理量的系数
        Vec3f rawData;
        Vec3f data;

        icm20948.readGyro(rawData); // 读取一次三轴磁力计数据

        /* 数据处理 */
        data.x = rawData.x / LSB;
        data.y = rawData.y / LSB;
        data.z = rawData.z / LSB;

        ESP_LOGI("ICM", "%lf,%lf,%lf", data.x, data.y, data.z); // 数据输出，可以用串口绘图器绘图

        rate.sleep(); // 控制循环频率
    }
    

}

extern "C" void app_main(void) {
    xTaskCreate(demo, "demo", 1024, NULL, 1, NULL); // 创建RTOS任务
}
