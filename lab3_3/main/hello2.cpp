#include "DFRobot_LCD.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstddef>


#define LCD_ADDR 0x3E
#define RGB_ADDR 0x2D
#define LCD_COLS 16
#define LCD_ROWS 2


DFRobot_LCD lcd(LCD_COLS, LCD_ROWS, LCD_ADDR, RGB_ADDR);

void lcdTask(void *pvParameters) {
	while(true) {
		lcd.init();
		lcd.setRGB(0, 255, 0);
		lcd.setCursor(0, 0);
		lcd.printstr("Hello CSE121!");
		lcd.setCursor(0, 1);
		lcd.printstr("Cele");
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}



extern "C" void app_main() {
	xTaskCreate(lcdTask, "LCD Task", 4096, NULL, 1, NULL);
}
