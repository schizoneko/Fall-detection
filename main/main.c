#include <stdio.h>
#include "unity.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/i2c.h"
#include "mpu6050.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_MASTER_SCL_IO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5       /*!< gpio number for I2C master data */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define ACCE_THRESHOLD 1.0 
//#define GYRO_THRESHOLD 

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
 * @brief Initialize MPU6050 sensor 1
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

/**
 * @brief Initialize MPU6050 sensor 2
 */
static void i2c_sensor_mpu6050_init_2(void)
{
    esp_err_t ret;

    mpu6050_2 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS_1);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050_2, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050_2, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050_2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/**
 * @brief Main application
 */
void app_main(void)
{
    esp_err_t ret;
    mpu6050_acce_value_t acce, prev_acce = {0, 0, 0};
    mpu6050_gyro_value_t gyro, prev_gyro;
    mpu6050_temp_value_t temp;

    mpu6050_acce_value_t acce2, prev_acce2 = {0, 0, 0};
    mpu6050_gyro_value_t gyro2, prev_gyro2;
    mpu6050_temp_value_t temp2;

    FILE *fpt;
    FILE *fpt2;
    
    bool continuous_mode = false;
    int sample_count = 0;

    // Sum 1
    float total_acce_x1 = 0, total_acce_y1 = 0, total_acce_z1 = 0;
    float gyro_x1 = 0, gyro_y1 = 0, gyro_z1 = 0;

    // Sum 2
    float total_acce_x2 = 0, total_acce_y2 = 0, total_acce_z2 = 0;
    float gyro_x2 = 0, gyro_y2 = 0, gyro_z2 = 0;

    i2c_bus_init();
    i2c_sensor_mpu6050_init();
    i2c_sensor_mpu6050_init_2();

    //open csv file
    // fpt = fopen("Result 1.csv", "w");
    // fpt2 = fopen("Result 2.csv", "w");
    // if (!fpt) ESP_LOGI(TAG, "Can't open CSV file for data recording");
    // if (!fpt2) ESP_LOGI(TAG_2, "Can't open CSV file for data recording");
    // fprintf(fpt, "Count, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z\n");
    // fprintf(fpt2, "Count, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z\n");

    while (1)
    {
        // Reading stats from Sensor 1
        ret = mpu6050_get_acce(mpu6050, &acce);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = mpu6050_get_gyro(mpu6050, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        // Reading stats from Sensor 2
        ret = mpu6050_get_acce(mpu6050_2, &acce2);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = mpu6050_get_gyro(mpu6050_2, &gyro2);
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        // Checking the threshold
        if ((fabs(acce.acce_x - prev_acce.acce_x) > ACCE_THRESHOLD ||
             fabs(acce.acce_y - prev_acce.acce_y) > ACCE_THRESHOLD ||
             fabs(acce.acce_z - prev_acce.acce_z) > ACCE_THRESHOLD ||
             fabs(acce2.acce_x - prev_acce2.acce_x) > ACCE_THRESHOLD ||
             fabs(acce2.acce_y - prev_acce2.acce_y) > ACCE_THRESHOLD ||
             fabs(acce2.acce_z - prev_acce2.acce_z) > ACCE_THRESHOLD) && !continuous_mode) 
        {
            ESP_LOGI(TAG, "Sudden movement detected! Starting continuous mode.");
            ESP_LOGI(TAG, "Count, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z");
            continuous_mode = true;
            sample_count = 0;

            // Reset the sum
            total_acce_x1 = total_acce_y1 = total_acce_z1 = 0;
            gyro_x1 = gyro_y1 = gyro_z1 = 0;
            total_acce_x2 = total_acce_y2 = total_acce_z2 = 0;
            gyro_x2 = gyro_y2 = gyro_z2 = 0;
        }

        // Continous mode
        if (continuous_mode)
        {
            // Sum of the stats of sensor 1
            total_acce_x1 += acce.acce_x;
            total_acce_y1 += acce.acce_y;
            total_acce_z1 += acce.acce_z;

            gyro_x1 += gyro.gyro_x;
            gyro_y1 += gyro.gyro_y;
            gyro_z1 += gyro.gyro_z;

            // Sum of the stats of sensor 2
            total_acce_x2 += acce2.acce_x;
            total_acce_y2 += acce2.acce_y;
            total_acce_z2 += acce2.acce_z;

            gyro_x2 += gyro2.gyro_x;
            gyro_y2 += gyro2.gyro_y;
            gyro_z2 += gyro2.gyro_z;

            sample_count++;
            ESP_LOGI(TAG, "%d, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", sample_count, acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
            sample_count++;
            ESP_LOGI(TAG, "%d, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", sample_count, acce2.acce_x, acce2.acce_y, acce2.acce_z, gyro2.gyro_x, gyro2.gyro_y, gyro2.gyro_z);

            // //write to csv file
            // fprintf(fpt, "%d, %f, %f, %f, %f, %f, %f\n", sample_count, acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
            // fprintf(fpt2, "%d, %f, %f, %f, %f, %f, %f\n", sample_count, acce2.acce_x, acce2.acce_y, acce2.acce_z, gyro2.gyro_x, gyro2.gyro_y, gyro2.gyro_z);
            
            sample_count++;

            // End the continuous mode after 15s
            if (sample_count >= 15 * (1000 / 100)) // 15 seconds, 100ms per sample
            {
                ESP_LOGI(TAG, "Continuous mode complete. Calculating averages...");

                // Average stats for sensor 1
                float avg_acce_x1 = total_acce_x1 / sample_count;
                float avg_acce_y1 = total_acce_y1 / sample_count;
                float avg_acce_z1 = total_acce_z1 / sample_count;

                float avg_gyro_x1 = gyro_x1 / sample_count;
                float avg_gyro_y1 = gyro_y1 / sample_count;
                float avg_gyro_z1 = gyro_z1 / sample_count;

                // Average stats for sensor 2
                float avg_acce_x2 = total_acce_x2 / sample_count;
                float avg_acce_y2 = total_acce_y2 / sample_count;
                float avg_acce_z2 = total_acce_z2 / sample_count;

                float avg_gyro_x2 = gyro_x2 / sample_count;
                float avg_gyro_y2 = gyro_y2 / sample_count;
                float avg_gyro_z2 = gyro_z2 / sample_count;

                ESP_LOGI(TAG, "Sensor 1 - Avg Accel: X: %.2f, Y: %.2f, Z: %.2f | Avg Gyro: X: %.2f, Y: %.2f, Z: %.2f",
                         avg_acce_x1, avg_acce_y1, avg_acce_z1, gyro_x1, avg_gyro_y1, avg_gyro_z1);

                ESP_LOGI(TAG_2, "Sensor 2 - Avg Accel: X: %.2f, Y: %.2f, Z: %.2f | Avg Gyro: X: %.2f, Y: %.2f, Z: %.2f",
                         avg_acce_x2, avg_acce_y2, avg_acce_z2, avg_gyro_x2, avg_gyro_y2, avg_gyro_z2);

                continuous_mode = false; 
            }
        }

        // Update previous values
        prev_acce = acce;
        prev_gyro = gyro;
        prev_acce2 = acce2;
        prev_gyro2 = gyro2;

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay 100ms
    }
}
