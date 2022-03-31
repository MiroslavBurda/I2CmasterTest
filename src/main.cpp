#include <esp_log.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <driver/i2c.h>

#define RETURN_IF_ERR(x) do {                                         \
        esp_err_t __err_rc = (x);                                       \
        if (__err_rc != ESP_OK) {                                       \
            return __err_rc;                                            \
        }                                                               \
    } while(0)

gpio_num_t sda_pin = GPIO_NUM_18;
gpio_num_t scl_pin = GPIO_NUM_19; 
uint32_t speed_hz = 100000;
uint8_t address = 0x15;
i2c_port_t bus_num = I2C_NUM_0; 
uint8_t DataToSend[] = {1, 2, 3, 4};
size_t len = sizeof(DataToSend);
uint8_t DataToRead[] = {3, 3, 3, 3};

i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = sda_pin,
    .scl_io_num = scl_pin,            
    .sda_pullup_en = GPIO_PULLUP_ENABLE,  
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master = {
        .clk_speed = speed_hz
    },
    .clk_flags = 0, 
};

esp_err_t sendData(const uint8_t *data, size_t len )
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    RETURN_IF_ERR(i2c_master_start(cmd));
    RETURN_IF_ERR(i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true));
    RETURN_IF_ERR(i2c_master_write(cmd, data, len, I2C_MASTER_LAST_NACK));
    RETURN_IF_ERR(i2c_master_stop(cmd));
    RETURN_IF_ERR(i2c_master_cmd_begin(bus_num, cmd, pdMS_TO_TICKS(125)));
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t readData(uint8_t *data, size_t len )
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    RETURN_IF_ERR(i2c_master_start(cmd));
    RETURN_IF_ERR(i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true));
    RETURN_IF_ERR(i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK));
    RETURN_IF_ERR(i2c_master_stop(cmd));
    RETURN_IF_ERR(i2c_master_cmd_begin(bus_num, cmd, pdMS_TO_TICKS(125)));
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

int i = 0;

extern "C" void app_main() {  // example for connect ESP32 with I2C
    ESP_ERROR_CHECK(i2c_param_config(bus_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(bus_num, I2C_MODE_MASTER, 0, 0, 0));
    
    while (i < 2) {
        ESP_ERROR_CHECK(readData( DataToRead, 4 ));
        for (int k = 0; k < 4; k++ ) 
            ESP_LOGI("READ: ", "%i, ", DataToRead[k]);
        ESP_LOGI("TAG0: ", "%i \n", i++);  
        vTaskDelay(pdMS_TO_TICKS(5000));   
    }

    while (i < 250) {
        ESP_ERROR_CHECK(sendData( DataToSend, len ));
        for (int k = 0; k < 4; k++ ) 
            DataToSend[k] += 1; 
        for (int k = 0; k < 4; k++ ) 
            ESP_LOGI("DATA: ", "%i, ", DataToSend[k]);
        ESP_LOGI("TAG1: ", "%i \n", i++);  
        vTaskDelay(pdMS_TO_TICKS(5000));   
    }
}


