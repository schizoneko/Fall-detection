#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>

#define I2C_MASTER_SCL_IO 4      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 test";
static const char *TAG_2 = "mpu6050 test 2";
static mpu6050_handle_t mpu6050 = NULL;
static mpu6050_handle_t mpu6050_2 = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

static void i2c_sensor_mpu6050_init_2(void)
{
    esp_err_t ret;

    mpu6050_2 = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS_1);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050_2, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050_2, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void app_main(void)
{
    esp_err_t ret;

    esp_rmaker_param_t *acce_x_1, *acce_x_2;
    esp_rmaker_param_t *acce_y_1, *acce_y_2;
    esp_rmaker_param_t *acce_z_1, *acce_z_2;

    esp_rmaker_param_t *gyro_x_1, *gyro_x_2;
    esp_rmaker_param_t *gyro_y_1, *gyro_y_2;
    esp_rmaker_param_t *gyro_z_1, *gyro_z_2;

    esp_rmaker_param_t *temp_1, *temp_2;

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
    mpu6050_acce_value_t acce2;
    mpu6050_gyro_value_t gyro2;
    mpu6050_temp_value_t temp2;

    i2c_bus_init(); 

    esp_rmaker_node_t *node = esp_rmaker_node_init("My Device", "Body", "ESP32S3");
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    esp_rmaker_device_t *MPU_1 = esp_rmaker_device_create("MPU Device 1", NULL, NULL);
    esp_rmaker_device_t *MPU_2 = esp_rmaker_device_create("MPU Device 2", NULL, NULL);

    ret = esp_rmaker_node_add_device(node, MPU_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_node_add_device(node, MPU_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    acce_x_1 = esp_rmaker_param_create("Accel X 1", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    acce_y_1 = esp_rmaker_param_create("Accel Y 1", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    acce_z_1 = esp_rmaker_param_create("Accel Z 1", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);

    gyro_x_1 = esp_rmaker_param_create("gyro X 1", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    gyro_y_1 = esp_rmaker_param_create("gyro Y 1", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    gyro_z_1 = esp_rmaker_param_create("gyro Z 1", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);

    temp_1 = esp_rmaker_param_create("Temp 1", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);

    acce_x_2 = esp_rmaker_param_create("Accel X 2", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    acce_y_2 = esp_rmaker_param_create("Accel Y 2", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    acce_z_2 = esp_rmaker_param_create("Accel Z 2", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);

    gyro_x_2 = esp_rmaker_param_create("gyro X 2", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    gyro_y_2 = esp_rmaker_param_create("gyro Y 2", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    gyro_z_2 = esp_rmaker_param_create("gyro Z 2", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);

    temp_2 = esp_rmaker_param_create("Temp 2", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);

    ret = esp_rmaker_device_add_param(MPU_1, acce_x_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, acce_y_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, acce_z_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, gyro_x_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, gyro_y_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, gyro_z_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, temp_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = esp_rmaker_device_add_param(MPU_2, acce_x_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_2, acce_y_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_2, acce_z_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_2, gyro_x_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_2, gyro_y_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_2, gyro_z_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_2, temp_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    esp_rmaker_start();

    // ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    // TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

    // ret = mpu6050_get_deviceid(mpu6050_2, &mpu6050_deviceid_2);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    // TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_I2C_ADDRESS_1, mpu6050_deviceid_2, "Who Am I register does not contain expected data");
    // ret = selftest(mpu6050);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    while (1)
    {
        i2c_sensor_mpu6050_init();

        ret = mpu6050_get_acce(mpu6050, &acce);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);
        
        ret = mpu6050_get_gyro(mpu6050, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        ret = mpu6050_get_temp(mpu6050, &temp);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "t:%.2f \n", temp.temp);

        ret = esp_rmaker_param_update_and_report(acce_x_1, esp_rmaker_float(acce.acce_x));
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = esp_rmaker_param_update_and_report(acce_y_1, esp_rmaker_float(acce.acce_y));
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = esp_rmaker_param_update_and_report(acce_z_1, esp_rmaker_float(acce.acce_z));
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        ret = esp_rmaker_param_update_and_report(gyro_x_1, esp_rmaker_float(gyro.gyro_x));
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = esp_rmaker_param_update_and_report(gyro_y_1, esp_rmaker_float(gyro.gyro_y));
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = esp_rmaker_param_update_and_report(gyro_z_1, esp_rmaker_float(gyro.gyro_z));
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        ret = esp_rmaker_param_update_and_report(temp_1, esp_rmaker_float(temp.temp));
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        i2c_sensor_mpu6050_init_2();

        ret = mpu6050_get_acce(mpu6050_2, &acce2);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG_2, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce2.acce_x, acce2.acce_y, acce2.acce_z);

        ret = mpu6050_get_gyro(mpu6050_2, &gyro2);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG_2, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro2.gyro_x, gyro2.gyro_y, gyro2.gyro_z);

        ret = mpu6050_get_temp(mpu6050_2, &temp2);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG_2, "t:%.2f \n", temp2.temp);

        ret = esp_rmaker_param_update_and_report(acce_x_2, esp_rmaker_float(acce2.acce_x));
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = esp_rmaker_param_update_and_report(acce_y_2, esp_rmaker_float(acce2.acce_y));
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = esp_rmaker_param_update_and_report(acce_z_2, esp_rmaker_float(acce2.acce_z));
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        ret = esp_rmaker_param_update_and_report(gyro_x_2, esp_rmaker_float(gyro2.gyro_x));
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = esp_rmaker_param_update_and_report(gyro_y_2, esp_rmaker_float(gyro2.gyro_y));
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = esp_rmaker_param_update_and_report(gyro_z_2, esp_rmaker_float(gyro2.gyro_z));
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        ret = esp_rmaker_param_update_and_report(temp_2, esp_rmaker_float(temp2.temp));
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}