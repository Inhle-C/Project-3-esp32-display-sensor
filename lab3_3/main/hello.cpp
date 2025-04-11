#include <iostream>
#include "driver/i2c.h"
#include "DFRobot_LCD.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstddef>

#define LCD_ADDR 0x3E
#define RGB_ADDR 0x2D
#define LCD_COLS 16
#define LCD_ROWS 2

#define I2C_MASTER_SCL_IO 8            // GPIO for SCL
#define I2C_MASTER_SDA_IO 10           // GPIO for SDA
#define I2C_MASTER_NUM I2C_NUM_0       // I2C port number
#define I2C_MASTER_FREQ_HZ 100000      // Frequency of I2C
#define I2C_TIMEOUT_MS 1000            // I2C timeout in ms
#define SHTC3_SENSOR_ADDR 0x70         // I2C address for SHTC3

// SHTC3 command codes
#define SHTC3_POWER_UP 0x3517          
#define SHTC3_POWER_DOWN 0xB098       
#define SHTC3_READ_TEMP_HUMID 0x7CA2   

DFRobot_LCD lcd(LCD_COLS, LCD_ROWS, LCD_ADDR, RGB_ADDR);

class SHTC3Sensor {
public:
    // Initialize I2C master
    void init() {
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

        conf.clk_flags = 0;
        i2c_param_config(I2C_MASTER_NUM, &conf);
        i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    }
    
    // Power-up function
    void powerUp() {
        uint8_t power_up_cmd[] = { static_cast<uint8_t>(SHTC3_POWER_UP >> 8), static_cast<uint8_t>(SHTC3_POWER_UP & 0xFF) };
        i2c_master_write_to_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, power_up_cmd, sizeof(power_up_cmd), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    // Power-down function
    void powerDown() {
        uint8_t power_down_cmd[] = { static_cast<uint8_t>(SHTC3_POWER_DOWN >> 8), static_cast<uint8_t>(SHTC3_POWER_DOWN & 0xFF) };
        i2c_master_write_to_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, power_down_cmd, sizeof(power_down_cmd), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    // Read and validate sensor data
    uint16_t readData(uint16_t &humidity) {
        uint8_t read_cmd[] = { static_cast<uint8_t>(SHTC3_READ_TEMP_HUMID >> 8), static_cast<uint8_t>(SHTC3_READ_TEMP_HUMID & 0xFF) };
        uint8_t data[6]; // 2 bytes temp, 1 byte temp CRC, 2 bytes humidity, 1 byte humidity CRC
        
        i2c_master_write_read_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, read_cmd, sizeof(read_cmd), data, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        
        uint16_t rawTemp = (data[0] << 8) | data[1];
        uint16_t rawHumid = (data[3] << 8) | data[4];
        
        // Validate CRC
        if (checkCRC(rawTemp, data[2]) && checkCRC(rawHumid, data[5])) {
            humidity = rawHumid;
            return rawTemp;
        } else {
            std::cerr << "CRC Check failed!" << std::endl;
            return 0xFFFF; // Invalid value
        }
    }

    // Calculate temperature from raw data
    float calculateTemperature(uint16_t rawTemp) const {
        return -45.0 + 175.0 * (static_cast<float>(rawTemp) / 65535.0);
    }

    // Calculate humidity from raw data
    float calculateHumidity(uint16_t rawHumid) const {
        return 100.0 * (static_cast<float>(rawHumid) / 65535.0);
    }

private:
    // CRC-8 checksum validation
    bool checkCRC(uint16_t data, uint8_t crc) const {
        uint8_t crc_calc = 0xFF;
        crc_calc ^= (data >> 8);
        for (int i = 0; i < 8; ++i) crc_calc = (crc_calc & 0x80) ? (crc_calc << 1) ^ 0x31 : (crc_calc << 1);
        
        crc_calc ^= (data & 0xFF);
        for (int i = 0; i < 8; ++i) crc_calc = (crc_calc & 0x80) ? (crc_calc << 1) ^ 0x31 : (crc_calc << 1);
        
        return crc_calc == crc;
    }
};

extern "C" void app_main() {
    SHTC3Sensor sensor;
    sensor.init();  // Initialize I2C master
    lcd.init();

    while (true) {
        // Power up the sensor
        sensor.powerUp();
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Small delay for power-up

        // Read temperature and humidity data
        uint16_t rawHumid;
        uint16_t rawTemp = sensor.readData(rawHumid);

        // Only calculate if data is valid
        if (rawTemp != 0xFFFF) {
            // Convert raw data
            float temperatureC = sensor.calculateTemperature(rawTemp);
            float temperatureF = temperatureC * 9.0 / 5.0 + 32.0;
            float humidity = sensor.calculateHumidity(rawHumid);

            // Print temperature and humidity
            std::cout << "Temperature: " << temperatureC << "°C (" << temperatureF << "°F), Humidity: " << humidity << "%" << std::endl;
        
            // Display on LCD
            char temp_str[16], hum_str[16];
            snprintf(temp_str, sizeof(temp_str), "Temp: %.0fC", temperatureC);
            snprintf(hum_str, sizeof(hum_str), "Hum: %.0f%%", humidity);
	

            lcd.setRGB(0, 255, 0);
            lcd.setCursor(0, 0);
            lcd.printstr(temp_str);
            lcd.setCursor(0, 1);
            lcd.printstr(hum_str);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        
        // Power down sensor
        sensor.powerDown();
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait 2 seconds
    }
}

