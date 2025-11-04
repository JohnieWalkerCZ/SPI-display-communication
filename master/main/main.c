#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

// Custom SPI pins for ESP32-S3 MASTER
#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 7
#define PIN_NUM_MOSI 12
#define PIN_NUM_CLK 13
#define PIN_NUM_CS 14

// Potentiometer pins
#define POT1_GPIO 1
#define POT2_GPIO 2

// ADC parameters
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_11

// Display parameters
#define DISPLAY_WIDTH 64
#define DISPLAY_HEIGHT 64

static const char *TAG = "SPI_MASTER_S3";

// Framebuffer structure - each pixel is 1 byte (grayscale)
typedef struct {
    uint8_t pixels[DISPLAY_WIDTH * DISPLAY_HEIGHT];
} framebuffer_t;

// Function declarations (REMOVE 'static' from declarations)
esp_err_t initialize_spi_bus(void);
esp_err_t initialize_spi_device(spi_device_handle_t *spi_handle);
esp_err_t initialize_adc(void);
uint8_t read_potentiometer_value(int gpio_num);
void update_framebuffer(framebuffer_t *fb, uint8_t x_pos, uint8_t y_pos);
void send_framebuffer(spi_device_handle_t spi_handle, framebuffer_t *fb);

// Function definitions (keep as they are, without 'static')
esp_err_t initialize_spi_bus(void) {
    ESP_LOGI(TAG, "Initializing SPI bus...");

    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        //                               .max_transfer_sz =
        //                               sizeof(framebuffer_t) + 10,
        .max_transfer_sz = 4097,
        .flags = 0,
        .intr_flags = 0};

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "SPI bus initialized successfully");
    return ESP_OK;
}

esp_err_t initialize_spi_device(spi_device_handle_t *spi_handle) {
    ESP_LOGI(TAG, "Adding SPI device...");

    // SPI device configuration
    spi_device_interface_config_t devcfg = {.command_bits = 0,
                                            .address_bits = 0,
                                            .dummy_bits = 0,
                                            .mode = 0, // SPI mode 0
                                            .duty_cycle_pos = 0,
                                            .cs_ena_pretrans = 0,
                                            .cs_ena_posttrans = 0,
                                            .clock_speed_hz =
                                                1 * 1000 * 1000, // 1 MHz
                                            .input_delay_ns = 0,
                                            .spics_io_num = PIN_NUM_CS,
                                            .flags = 0,
                                            .queue_size = 3,
                                            .pre_cb = NULL,
                                            .post_cb = NULL};

    esp_err_t ret = spi_bus_add_device(SPI_HOST, &devcfg, spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: 0x%x", ret);
        spi_bus_free(SPI_HOST);
        return ret;
    }

    ESP_LOGI(TAG, "SPI device added successfully");
    return ESP_OK;
}

esp_err_t initialize_adc(void) {
    ESP_LOGI(TAG, "Initializing ADC for potentiometers...");

    // Configure ADC width and attenuation
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC1_CHANNEL_0,
                              ADC_ATTEN); // GPIO 1 = ADC1_CHANNEL_0
    adc1_config_channel_atten(ADC1_CHANNEL_1,
                              ADC_ATTEN); // GPIO 2 = ADC1_CHANNEL_1

    ESP_LOGI(TAG,
             "ADC initialized for GPIO %d (Channel 0) and GPIO %d (Channel 1)",
             POT1_GPIO, POT2_GPIO);

    return ESP_OK;
}

uint8_t read_potentiometer_value(int gpio_num) {
    int adc_reading = 0;

    // Read ADC based on GPIO number
    switch (gpio_num) {
    case POT1_GPIO:
        adc_reading = adc1_get_raw(ADC1_CHANNEL_0);
        break;
    case POT2_GPIO:
        adc_reading = adc1_get_raw(ADC1_CHANNEL_1);
        break;
    default:
        ESP_LOGE(TAG, "Invalid GPIO for potentiometer: %d", gpio_num);
        return 0;
    }

    // Convert 12-bit ADC reading (0-4095) to 0-9 range for 10x10 display
    uint8_t mapped_value = (uint8_t)((adc_reading * DISPLAY_WIDTH) / 4096);

    // Clamp to 0-9 range
    if (mapped_value >= DISPLAY_WIDTH) {
        mapped_value = DISPLAY_WIDTH - 1;
    }

    return mapped_value;
}

void draw_circle(framebuffer_t *fb, uint8_t x_center, uint8_t y_center,
                 uint8_t radius) {
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x * x + y * y <= radius * radius) {
                int draw_x = x_center + x;
                int draw_y = y_center + y;
                if (draw_x >= 0 && draw_x < DISPLAY_WIDTH && draw_y >= 0 &&
                    draw_y < DISPLAY_HEIGHT) {
                    fb->pixels[draw_y * DISPLAY_WIDTH + draw_x] = 255; // White
                }
            }
        }
    }
}

void update_framebuffer(framebuffer_t *fb, uint8_t x_pos, uint8_t y_pos) {
    // Clear the framebuffer (set all pixels to 0)
    memset(fb->pixels, 0, sizeof(fb->pixels));

    draw_circle(fb, x_pos, y_pos, 15); // Draw circle with radius 3
}

void send_framebuffer(spi_device_handle_t spi_handle, framebuffer_t *fb) {
    // Allocate receive buffer on HEAP instead of stack
    uint8_t *receive_buffer = (uint8_t *)malloc(sizeof(framebuffer_t));
    if (receive_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate receive buffer");
        return;
    }

    spi_transaction_t t = {.flags = 0,
                           .cmd = 0,
                           .addr = 0,
                           .length = sizeof(framebuffer_t) * 8, // In bits
                           .rxlength = sizeof(framebuffer_t) * 8,
                           .user = NULL,
                           .tx_buffer = fb,
                           .rx_buffer = receive_buffer};

    // Perform SPI transaction
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Received %d bytes from slave", sizeof(framebuffer_t));
    } else {
        ESP_LOGE(TAG, "SPI framebuffer transaction failed: 0x%x", ret);
    }

    // Free the receive buffer
    free(receive_buffer);
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-S3 Master Initializing...");

    // Initialize SPI bus
    esp_err_t ret = initialize_spi_bus();
    if (ret != ESP_OK) {
        return;
    }

    // Initialize SPI device
    spi_device_handle_t spi;
    ret = initialize_spi_device(&spi);
    if (ret != ESP_OK) {
        return;
    }

    // Initialize ADC for potentiometers
    ret = initialize_adc();
    if (ret != ESP_OK) {
        return;
    }

    // Create framebuffer on HEAP instead of stack
    framebuffer_t *display_fb = (framebuffer_t *)malloc(sizeof(framebuffer_t));
    if (display_fb == NULL) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer memory");
        return;
    }
    memset(display_fb, 0, sizeof(framebuffer_t));

    ESP_LOGI(TAG, "ESP32-S3 SPI Master initialized successfully");
    ESP_LOGI(TAG, "Display size: %dx%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    ESP_LOGI(TAG, "Framebuffer size: %d bytes (allocated on heap)",
             sizeof(framebuffer_t));
    ESP_LOGI(TAG, "MISO: GPIO %d, MOSI: GPIO %d, SCLK: GPIO %d, CS: GPIO %d",
             PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
    ESP_LOGI(TAG, "Reading potentiometers and sending framebuffer to slave...");

    // Main loop for SPI communication
    while (1) {
        // Read potentiometer values
        uint8_t pot1_val = read_potentiometer_value(POT1_GPIO);
        uint8_t pot2_val = read_potentiometer_value(POT2_GPIO);

        ESP_LOGI(TAG, "Pot1 (X): %d, Pot2 (Y): %d", pot1_val, pot2_val);

        // Update framebuffer with pixel at potentiometer positions
        update_framebuffer(display_fb, pot1_val, pot2_val);

        // Send framebuffer over SPI
        send_framebuffer(spi, display_fb);

        // Small delay to avoid flooding the SPI bus
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay for smoother updates
    }

    // Free memory (though we never get here in this example)
    free(display_fb);
}
