#include "main.hpp"

GPIO gpio();

void demo(void *pvParameters) {
    (void) pvParameters; // 告诉编译器我知道这个没有别警告我

    /* 初始化任务循环控制类 */
    Rate rate(50);

}

extern "C" void app_main(void) {
    xTaskCreate(demo, "demo", 4096, NULL, 1, NULL); // 创建RTOS任务
}
