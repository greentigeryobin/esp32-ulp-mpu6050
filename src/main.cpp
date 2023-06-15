#include <esp_log.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hulp.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include <hulp_i2cbb.h>

static const uint8_t SLAVE_ADDR = 0x68;
static const gpio_num_t SCL_PIN = GPIO_NUM_33, SDA_PIN = GPIO_NUM_32;

RTC_SLOW_ATTR ulp_var_t ulp_read_cmd[HULP_I2C_CMD_BUF_SIZE(2)] = {
        HULP_I2C_CMD_HDR(SLAVE_ADDR, 0x3B, 2),
};

static RTC_DATA_ATTR ulp_var_t last_x;
static RTC_DATA_ATTR ulp_var_t last_y;
static RTC_DATA_ATTR ulp_var_t last_z;

void init_ulp() {
    enum
  {
    LABEL_I2C_READ,
    LABEL_I2C_READ_RETURN,
    LABEL_I2C_WRITE,
    LABEL_I2C_WRITE_RETURN,
    LABEL_CHECK_THRESHOLD,
    LABEL_I2C_ERROR,
    LABEL_NEGATIVE,
    LABEL_WAKE,
  };

  const ulp_insn_t program[] = {

      I_MOVO(R1, ulp_read_cmd),
      M_MOVL(R3, LABEL_I2C_READ_RETURN),
      M_BX(LABEL_I2C_READ),
      M_LABEL(LABEL_I2C_READ_RETURN),

      I_MOVI(R1, 0),
      I_GET(R1, R1, ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET]),

      I_MOVI(R2, 0),
      I_GET(R2, R2, last_x),

      I_SUBR(R0, R1, R2),

      I_MOVI(R2, 0),
      I_PUT(R1, R2, last_x),

      M_BX(LABEL_CHECK_THRESHOLD),

      I_MOVI(R1, 0),
      I_GET(R1, R1, ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET + 1]),

      I_MOVI(R2, 0),
      I_GET(R2, R2, last_y),

      I_SUBR(R0, R1, R2),

      I_MOVI(R2, 0),
      I_PUT(R1, R2, last_y),

      M_BX(LABEL_CHECK_THRESHOLD),

      I_MOVI(R1, 0),
      I_GET(R1, R1, ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET + 2]),

      I_MOVI(R2, 0),
      I_GET(R2, R2, last_z),

      I_SUBR(R0, R1, R2),

      I_MOVI(R2, 0),
      I_PUT(R1, R2, last_z),

      M_BX(LABEL_CHECK_THRESHOLD),

      M_LABEL(LABEL_CHECK_THRESHOLD),
      M_BGE(LABEL_NEGATIVE, 0x8000),
      M_BGE(LABEL_WAKE, 0x666),
      I_HALT(),

      M_LABEL(LABEL_NEGATIVE),
      M_BL(LABEL_WAKE, 0xF999),
      I_HALT(),

      M_LABEL(LABEL_WAKE),
      I_WAKE(),
      I_END(),
      I_HALT(),

      M_INCLUDE_I2CBB_CMD(LABEL_I2C_READ, LABEL_I2C_WRITE, SCL_PIN, SDA_PIN)

  };

  ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
  ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));

  hulp_peripherals_on();

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 200 * 1000, 0));
  ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void print_result() {
    const int16_t x_raw = ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET].val;
    const int16_t y_raw = ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET + 1].val;
    const int16_t z_raw = ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET + 2].val;

    int16_t x = ((float)x_raw/16384)*100;
    int16_t y = ((float)y_raw/16384)*100;
    int16_t z = ((float)z_raw/16384)*100;

    ESP_LOGE("MPU", "X: %i, Y: %i, Z: %i", x, y, z);
}

extern "C" void app_main(void) {

    if (hulp_is_deep_sleep_wakeup()) {
        ESP_LOGE("CORE","WOKE UP FROM SLEEP");
        print_result();
    } else {
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = SDA_PIN;
        conf.scl_io_num = SCL_PIN;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 100000;
        ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

        i2c_cmd_handle_t cmd;
        vTaskDelay(200/portTICK_PERIOD_MS);

        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_WRITE, 1));
        i2c_master_write_byte(cmd, 0x6B, 1);
        i2c_master_write_byte(cmd, 0, 1);
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_WRITE, 1));
        i2c_master_write_byte(cmd, 0x1B, 1);
        i2c_master_write_byte(cmd, 0, 1);
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_WRITE, 1));
        i2c_master_write_byte(cmd, 0x1C, 1);
        i2c_master_write_byte(cmd, 0, 1);
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        init_ulp();
    }

    ESP_LOGI("CORE", "SLEEPING");
    hulp_peripherals_on();
    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}
