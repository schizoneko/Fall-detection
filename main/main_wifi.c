#include <stdio.h>
#include "unity.h"
#include "esp_system.h"
#include "esp_log.h"
#include <esp_event.h>
#include <string.h>

#include "forward.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <nvs_flash.h>

#include "esp_rmaker_core.h"
#include "driver/i2c.h"
#include "mpu6050.h"

#include <esp_wifi.h>
#include <network_provisioning/manager.h>
#include <network_provisioning/scheme_softap.h>
#include "qrcode.h"

#define I2C_MASTER_SCL_IO 4      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define ACCE_THRESHOLD 1.0 

static const char *TAG = "mpu6050 test";
static const char *TAG_3 = "wifi_ap";
static mpu6050_handle_t mpu6050 = NULL;


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

/* Signal Wi-Fi events on this event-group */
const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;

#define PROV_QR_VERSION         "v1"
#define PROV_TRANSPORT_SOFTAP   "softap"
#define PROV_TRANSPORT_BLE      "ble"
#define QRCODE_BASE_URL         "https://espressif.github.io/esp-jumpstart/qrcode.html"

/* Event handler for catching system events */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == NETWORK_PROV_EVENT) {
        switch (event_id) {
        case NETWORK_PROV_START:
            ESP_LOGI(TAG_3, "Provisioning started");
            break;
        case NETWORK_PROV_WIFI_CRED_RECV: {
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG_3, "Received Wi-Fi credentials"
                     "\n\tSSID     : %s\n\tPassword : %s",
                     (const char *) wifi_sta_cfg->ssid,
                     (const char *) wifi_sta_cfg->password);
            break;
        }
        case NETWORK_PROV_WIFI_CRED_FAIL: {
            network_prov_wifi_sta_fail_reason_t *reason = (network_prov_wifi_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG_3, "Provisioning failed!\n\tReason : %s"
                     "\n\tPlease reset to factory and retry provisioning",
                     (*reason == NETWORK_PROV_WIFI_STA_AUTH_ERROR) ?
                     "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
            break;
        }
        case NETWORK_PROV_WIFI_CRED_SUCCESS:
            ESP_LOGI(TAG_3, "Provisioning successful");
            break;
        case NETWORK_PROV_END:
            /* De-initialize manager once provisioning is finished */
            network_prov_mgr_deinit();
            break;
        default:
            break;
        }
    } else if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG_3, "Disconnected. Connecting to the AP again...");
            esp_wifi_connect();
            break;
                case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG_3, "SoftAP transport: Connected!");
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG_3, "SoftAP transport: Disconnected!");
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG_3, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        /* Signal main application to continue execution */
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
    } else if (event_base == PROTOCOMM_SECURITY_SESSION_EVENT) {
        switch (event_id) {
        case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
            ESP_LOGI(TAG_3, "Secured session established!");
            break;
        case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
            ESP_LOGE(TAG_3, "Received invalid security parameters for establishing secure session!");
            break;
        case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
            ESP_LOGE(TAG_3, "Received incorrect username and/or PoP for establishing secure session!");
            break;
        default:
            break;
        }
    }
}

static void wifi_init_sta(void)
{
    /* Start Wi-Fi in station mode */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X",
             ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                   uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    if (inbuf) {
        ESP_LOGI(TAG_3, "Received data: %.*s", inlen, (char *)inbuf);
    }
    char response[] = "SUCCESS";
    *outbuf = (uint8_t *)strdup(response);
    if (*outbuf == NULL) {
        ESP_LOGE(TAG_3, "System out of memory");
        return ESP_ERR_NO_MEM;
    }
    *outlen = strlen(response) + 1; /* +1 for NULL terminating byte */

    return ESP_OK;
}

static void wifi_prov_print_qr(const char *name, const char *username, const char *pop, const char *transport)
{
    if (!name || !transport) {
        ESP_LOGW(TAG_3, "Cannot generate QR code payload. Data missing.");
        return;
    }
    char payload[150] = {0};
    if (pop) {
        snprintf(payload, sizeof(payload), "{\"ver\":\"%s\",\"name\":\"%s\"" \
                 ",\"pop\":\"%s\",\"transport\":\"%s\"}",
                 PROV_QR_VERSION, name, pop, transport);
    } else {
        snprintf(payload, sizeof(payload), "{\"ver\":\"%s\",\"name\":\"%s\"" \
                 ",\"transport\":\"%s\",\"network\":\"wifi\"}",
                 PROV_QR_VERSION, name, transport);
    }
    ESP_LOGI(TAG, "Scan this QR code from the provisioning application for Provisioning.");
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&cfg, payload);
    ESP_LOGI(TAG, "If QR code is not visible, copy paste the below URL in a browser.\n%s?data=%s", QRCODE_BASE_URL, payload);
}

void app_main(void)
{
    esp_err_t ret;

    esp_rmaker_param_t *acce1_x, *acce1_y, *acce1_z;
    esp_rmaker_param_t *gyro1_x, *gyro1_y, *gyro1_z;

    mpu6050_acce_value_t acce, prev_acce = {0, 0, 0}, avg_acce;
    mpu6050_gyro_value_t gyro, prev_gyro, avg_gyro;
    mpu6050_temp_value_t temp;

    bool continuous_mode = false;
    int sample_count = 0;

    // Sum 1
    float total_acce_x1 = 0, total_acce_y1 = 0, total_acce_z1 = 0;
    float gyro_x1 = 0, gyro_y1 = 0, gyro_z1 = 0;

    i2c_bus_init();
    i2c_sensor_mpu6050_init();

    // ret = nvs_flash_erase();
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Initialize the event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_event_group = xEventGroupCreate();

    /* Register our event handler for Wi-Fi, IP and Provisioning related events */
    ESP_ERROR_CHECK(esp_event_handler_register(NETWORK_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    /* Initialize Wi-Fi including netif with default config */
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "Body", "ESP32S3");

    esp_rmaker_device_t *MPU_1 = esp_rmaker_device_create("MPU Device 1", NULL, NULL);

    ret = esp_rmaker_node_add_device(node, MPU_1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    acce1_x = esp_rmaker_param_create("Accel X", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    acce1_y = esp_rmaker_param_create("Accel Y", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    acce1_z = esp_rmaker_param_create("Accel Z", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);

    ret = esp_rmaker_device_add_param(MPU_1, acce1_x);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, acce1_y);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, acce1_z);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    gyro1_x = esp_rmaker_param_create("Gyro X", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    gyro1_y = esp_rmaker_param_create("Gyro Y", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    gyro1_z = esp_rmaker_param_create("Gyro Z", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ);
    
    ret = esp_rmaker_device_add_param(MPU_1, gyro1_x);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, gyro1_y);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = esp_rmaker_device_add_param(MPU_1, gyro1_z);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    esp_rmaker_start();

    /* Configuration for the provisioning manager */
    network_prov_mgr_config_t config = {
        .scheme = network_prov_scheme_softap,
        .scheme_event_handler = NETWORK_PROV_EVENT_HANDLER_NONE
    };
    ESP_ERROR_CHECK(network_prov_mgr_init(config));
    bool provisioned = false;
    ESP_ERROR_CHECK(network_prov_mgr_is_wifi_provisioned(&provisioned));
    if (!provisioned) {
        ESP_LOGI(TAG_3, "Starting provisioning");

        char service_name[12];

        get_device_service_name(service_name, sizeof(service_name));
        network_prov_security_t security = NETWORK_PROV_SECURITY_1;
        const char *pop = "abcd1234";
        network_prov_security1_params_t *sec_params = pop;

        const char *username  = NULL;
        const char *service_key = "linhminh0407";
        ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(security, (const void *) sec_params, service_name, service_key));
        /* Print QR code for provisioning */
        wifi_prov_print_qr(service_name, username, pop, PROV_TRANSPORT_SOFTAP);
        } else {
        ESP_LOGI(TAG_3, "Already provisioned, starting Wi-Fi STA");

        network_prov_mgr_wait();
        network_prov_mgr_deinit();
        /* Start Wi-Fi station */
        wifi_init_sta();
    }

    /* Wait for Wi-Fi connection */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);

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
        // Reading stats from Sensor 1
        ret = mpu6050_get_acce(mpu6050, &acce);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = mpu6050_get_gyro(mpu6050, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        // Checking the threshold
        if ((fabs(acce.acce_x - prev_acce.acce_x) > ACCE_THRESHOLD ||
             fabs(acce.acce_y - prev_acce.acce_y) > ACCE_THRESHOLD ||
             fabs(acce.acce_z - prev_acce.acce_z) > ACCE_THRESHOLD
            ) && !continuous_mode) 
        {
            ESP_LOGI(TAG, "Sudden movement detected! Starting continuous mode.");
            ESP_LOGI(TAG, "Count, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z");
            continuous_mode = true;
            sample_count = 0;

            // Reset the sum
            total_acce_x1 = total_acce_y1 = total_acce_z1 = 0;
            gyro_x1 = gyro_y1 = gyro_z1 = 0;
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

            sample_count++;
            ESP_LOGI(TAG, "%d, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", sample_count, acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

            
            // //write to csv file
            // fprintf(fpt, "%d, %f, %f, %f, %f, %f, %f\n", sample_count, acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
            // fprintf(fpt2, "%d, %f, %f, %f, %f, %f, %f\n", sample_count, acce2.acce_x, acce2.acce_y, acce2.acce_z, gyro2.gyro_x, gyro2.gyro_y, gyro2.gyro_z);

            // End the continuous mode after 15s
            if (sample_count >= (int)(7.5 * (1000 / 100))) // 7.5 seconds, 100ms per sample
            {
                ESP_LOGI(TAG, "Continuous mode complete. Calculating averages...");

                // Average stats for sensor 1
                avg_acce.acce_x = total_acce_x1 / sample_count;
                avg_acce.acce_y = total_acce_y1 / sample_count;
                avg_acce.acce_z = total_acce_z1 / sample_count;

                avg_gyro.gyro_x = gyro_x1 / sample_count;
                avg_gyro.gyro_y = gyro_y1 / sample_count;
                avg_gyro.gyro_z = gyro_z1 / sample_count;

                ESP_LOGI(TAG, "Sensor 1 - Avg Accel: X: %.2f, Y: %.2f, Z: %.2f | Avg Gyro: X: %.2f, Y: %.2f, Z: %.2f",
                         avg_acce.acce_x, avg_acce.acce_y, avg_acce.acce_z, avg_gyro.gyro_x, avg_gyro.gyro_y, avg_gyro.gyro_z);

                float input[INPUT_LAYER_SIZE] = {
                    avg_acce.acce_x, 
                    avg_acce.acce_y, 
                    avg_acce.acce_z, 
                    avg_gyro.gyro_x, 
                    avg_gyro.gyro_y, 
                    avg_gyro.gyro_z
                };

                float output = forward(input);

                if (output < 0.5) {
                    ESP_LOGI(TAG, "Neural Network Output: %.6f - Result: Not Fall", output);
                }
                else {
                    ret = esp_rmaker_raise_alert("Fall detected");
                    TEST_ASSERT_EQUAL(ESP_OK, ret);
                    ESP_LOGI(TAG, "Neural Network Output: %.6f - Result: Fall", output);
                }

                ret = esp_rmaker_param_update_and_report(acce1_x, esp_rmaker_float(avg_acce.acce_x));
                TEST_ASSERT_EQUAL(ESP_OK, ret);
                ret = esp_rmaker_param_update_and_report(acce1_y, esp_rmaker_float(avg_acce.acce_y));
                TEST_ASSERT_EQUAL(ESP_OK, ret);
                ret = esp_rmaker_param_update_and_report(acce1_z, esp_rmaker_float(avg_acce.acce_z));
                TEST_ASSERT_EQUAL(ESP_OK, ret);
                ret = esp_rmaker_param_update_and_report(gyro1_x, esp_rmaker_float(avg_gyro.gyro_x));
                TEST_ASSERT_EQUAL(ESP_OK, ret);
                ret = esp_rmaker_param_update_and_report(gyro1_y, esp_rmaker_float(avg_gyro.gyro_y));
                TEST_ASSERT_EQUAL(ESP_OK, ret);
                ret = esp_rmaker_param_update_and_report(gyro1_z, esp_rmaker_float(avg_gyro.gyro_z));
                TEST_ASSERT_EQUAL(ESP_OK, ret);
                continuous_mode = false; 
            }
        }

        // Update previous values
        prev_acce = acce;
        prev_gyro = gyro;

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay 100ms
    }
}
//     while (1)
//     {
//         // Reading stats from Sensor 1
//         ret = mpu6050_get_acce(mpu6050, &acce);
//         TEST_ASSERT_EQUAL(ESP_OK, ret);
//         ret = mpu6050_get_gyro(mpu6050, &gyro);
//         TEST_ASSERT_EQUAL(ESP_OK, ret);
//         ret = mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &comp1);
//         TEST_ASSERT_EQUAL(ESP_OK, ret);

//         // Reading stats from Sensor 2
//         ret = mpu6050_get_acce(mpu6050_2, &acce2);
//         TEST_ASSERT_EQUAL(ESP_OK, ret);
//         ret = mpu6050_get_gyro(mpu6050_2, &gyro2);
//         TEST_ASSERT_EQUAL(ESP_OK, ret);
//         ret = mpu6050_complimentory_filter(mpu6050_2, &acce2, &gyro2, &comp2);
//         TEST_ASSERT_EQUAL(ESP_OK, ret);

//         // Checking the threshold
//         if ((fabs(acce.acce_x - prev_acce.acce_x) > ACCE_THRESHOLD ||
//              fabs(acce.acce_y - prev_acce.acce_y) > ACCE_THRESHOLD ||
//              fabs(acce.acce_z - prev_acce.acce_z) > ACCE_THRESHOLD ||
//              fabs(acce2.acce_x - prev_acce2.acce_x) > ACCE_THRESHOLD ||
//              fabs(acce2.acce_y - prev_acce2.acce_y) > ACCE_THRESHOLD ||
//              fabs(acce2.acce_z - prev_acce2.acce_z) > ACCE_THRESHOLD) && !continuous_mode) 
//         {
//             ESP_LOGI(TAG, "Sudden movement detected! Starting continuous mode.");
//             ESP_LOGI(TAG, "Count, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z");
//             continuous_mode = true;
//             sample_count = 0;

//             // Reset the sum
//             total_acce_x1 = total_acce_y1 = total_acce_z1 = 0;
//             gyro_x1 = gyro_y1 = gyro_z1 = 0;
//             total_acce_x2 = total_acce_y2 = total_acce_z2 = 0;
//             gyro_x2 = gyro_y2 = gyro_z2 = 0;
//         }

//         // Continous mode
//         if (continuous_mode)
//         {
//             // Sum of the stats of sensor 1
//             total_acce_x1 += acce.acce_x;
//             total_acce_y1 += acce.acce_y;
//             total_acce_z1 += acce.acce_z;

//             gyro_x1 += gyro.gyro_x;
//             gyro_y1 += gyro.gyro_y;
//             gyro_z1 += gyro.gyro_z;

//             // Sum of the stats of sensor 2
//             total_acce_x2 += acce2.acce_x;
//             total_acce_y2 += acce2.acce_y;
//             total_acce_z2 += acce2.acce_z;

//             gyro_x2 += gyro2.gyro_x;
//             gyro_y2 += gyro2.gyro_y;
//             gyro_z2 += gyro2.gyro_z;

//             sample_count++;
//             ESP_LOGI(TAG, "%d, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", sample_count, acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
//             sample_count++;
//             ESP_LOGI(TAG, "%d, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", sample_count, acce2.acce_x, acce2.acce_y, acce2.acce_z, gyro2.gyro_x, gyro2.gyro_y, gyro2.gyro_z);

//             // End the continuous mode after 15s
//             if (sample_count >= 15 * (1000 / 100)) // 7.5 seconds, 100ms per sample
//             {
//                 ESP_LOGI(TAG, "Continuous mode complete. Calculating averages...");

//                 // Average stats for sensor 1
//                 float avg_acce_x1 = total_acce_x1 / sample_count;
//                 float avg_acce_y1 = total_acce_y1 / sample_count;
//                 float avg_acce_z1 = total_acce_z1 / sample_count;

//                 float avg_gyro_x1 = gyro_x1 / sample_count;
//                 float avg_gyro_y1 = gyro_y1 / sample_count;
//                 float avg_gyro_z1 = gyro_z1 / sample_count;

//                 // Average stats for sensor 2
//                 float avg_acce_x2 = total_acce_x2 / sample_count;
//                 float avg_acce_y2 = total_acce_y2 / sample_count;
//                 float avg_acce_z2 = total_acce_z2 / sample_count;

//                 float avg_gyro_x2 = gyro_x2 / sample_count;
//                 float avg_gyro_y2 = gyro_y2 / sample_count;
//                 float avg_gyro_z2 = gyro_z2 / sample_count;

//                 ESP_LOGI(TAG, "Sensor 1 - Avg Accel: X: %.2f, Y: %.2f, Z: %.2f | Avg Gyro: X: %.2f, Y: %.2f, Z: %.2f",
//                          avg_acce_x1, avg_acce_y1, avg_acce_z1, gyro_x1, avg_gyro_y1, avg_gyro_z1);

//                 ESP_LOGI(TAG_2, "Sensor 2 - Avg Accel: X: %.2f, Y: %.2f, Z: %.2f | Avg Gyro: X: %.2f, Y: %.2f, Z: %.2f",
//                          avg_acce_x2, avg_acce_y2, avg_acce_z2, avg_gyro_x2, avg_gyro_y2, avg_gyro_z2);

//                 continuous_mode = false; 
//             }
//         }

//         // Update previous values
//         prev_acce = acce;
//         prev_gyro = gyro;
//         prev_acce2 = acce2;
//         prev_gyro2 = gyro2;

//         vTaskDelay(pdMS_TO_TICKS(100)); // Delay 100ms
//     }
// }