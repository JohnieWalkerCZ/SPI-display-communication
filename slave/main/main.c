#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "led_strip.h"
#include <stdio.h>
#include <string.h>

// Custom SPI pins for ESP32 SLAVE
#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 33
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK 12
#define PIN_NUM_CS 5

// WS2812B LED Strip configuration
#define LED_STRIP_GPIO 23
#define LED_NUM 100 // 10x10 = 100 LEDs

static const char *TAG = "SPI_SLAVE_ESP32";

// RGB pixel structure
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_pixel_t;

// Framebuffer structures
#define INPUT_WIDTH 64
#define INPUT_HEIGHT 64
#define OUTPUT_WIDTH 10
#define OUTPUT_HEIGHT 10

typedef struct {
    rgb_pixel_t pixels[OUTPUT_WIDTH * OUTPUT_HEIGHT]; // RGB pixels
} framebuffer_t;

typedef struct {
    rgb_pixel_t pixels[INPUT_WIDTH * INPUT_HEIGHT]; // RGB pixels
    uint32_t checksum;                              // Simple checksum
} framebuffer_64x64_t;

// Function to shrink 64x64 RGB framebuffer to 10x10
static void shrink_framebuffer_64x64_to_10x10(framebuffer_64x64_t *src,
                                              framebuffer_t *dest) {
    // Clear destination framebuffer
    memset(dest->pixels, 0, sizeof(dest->pixels));

    const int block_size =
        6; // Each 10x10 pixel represents a 6x6 block in the 64x64 input

    for (int dest_y = 0; dest_y < OUTPUT_HEIGHT; dest_y++) {
        for (int dest_x = 0; dest_x < OUTPUT_WIDTH; dest_x++) {
            // Calculate source block boundaries
            int src_start_x = dest_x * block_size;
            int src_start_y = dest_y * block_size;
            int src_end_x = src_start_x + block_size;
            int src_end_y = src_start_y + block_size;

            // Clamp to source boundaries
            if (src_end_x > INPUT_WIDTH)
                src_end_x = INPUT_WIDTH;
            if (src_end_y > INPUT_HEIGHT)
                src_end_y = INPUT_HEIGHT;

            // Calculate average RGB values in this block
            uint32_t sum_r = 0, sum_g = 0, sum_b = 0;
            int pixel_count = 0;

            for (int src_y = src_start_y; src_y < src_end_y; src_y++) {
                for (int src_x = src_start_x; src_x < src_end_x; src_x++) {
                    int src_index = src_y * INPUT_WIDTH + src_x;
                    sum_r += src->pixels[src_index].r;
                    sum_g += src->pixels[src_index].g;
                    sum_b += src->pixels[src_index].b;
                    pixel_count++;
                }
            }

            // Calculate average and set destination pixel
            if (pixel_count > 0) {
                int dest_index = dest_y * OUTPUT_WIDTH + dest_x;
                dest->pixels[dest_index].r = (uint8_t)(sum_r / pixel_count);
                dest->pixels[dest_index].g = (uint8_t)(sum_g / pixel_count);
                dest->pixels[dest_index].b = (uint8_t)(sum_b / pixel_count);
            }
        }
    }
}

// LED strip handle
struct led_strip_t led_strip;

// Function declarations
static esp_err_t initialize_spi_slave(void);
static esp_err_t initialize_led_strip(void);
static void update_led_strip(framebuffer_t *fb);
static void handle_spi_communication(void);

esp_err_t initialize_led_strip(void) {
    ESP_LOGI(TAG, "Initializing WS2812B LED strip on GPIO %d", LED_STRIP_GPIO);

    // Configure and enable power pin (GPIO 16)
    gpio_config_t power_pin_config = {.pin_bit_mask = (1ULL << 16), // GPIO 16
                                      .mode = GPIO_MODE_OUTPUT,
                                      .pull_up_en = GPIO_PULLUP_DISABLE,
                                      .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                      .intr_type = GPIO_INTR_DISABLE};
    esp_err_t ret = gpio_config(&power_pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure power pin: 0x%x", ret);
        return ret;
    }

    // Turn on the LED strip power
    gpio_set_level(16, 1);
    ESP_LOGI(TAG, "LED strip power enabled on GPIO 16");

    // Allocate buffers for LED strip
    struct led_color_t *buf1 =
        (struct led_color_t *)malloc(sizeof(struct led_color_t) * LED_NUM);
    struct led_color_t *buf2 =
        (struct led_color_t *)malloc(sizeof(struct led_color_t) * LED_NUM);

    if (!buf1 || !buf2) {
        ESP_LOGE(TAG, "Failed to allocate LED strip buffers");
        return ESP_FAIL;
    }

    // Create semaphore
    SemaphoreHandle_t access_semaphore = xSemaphoreCreateBinary();
    if (!access_semaphore) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        free(buf1);
        free(buf2);
        return ESP_FAIL;
    }

    // Configure LED strip
    led_strip.led_strip_length = LED_NUM;
    led_strip.rmt_channel = RMT_CHANNEL_0;
    led_strip.gpio = LED_STRIP_GPIO;
    led_strip.led_strip_buf_1 = buf1;
    led_strip.led_strip_buf_2 = buf2;
    led_strip.access_semaphore = access_semaphore;
    led_strip.showing_buf_1 = true;
    led_strip.rgb_led_type = RGB_LED_TYPE_WS2812;

    // Initialize LED strip
    bool init_success = led_strip_init(&led_strip);
    if (!init_success) {
        ESP_LOGE(TAG, "Failed to initialize LED strip");
        free(buf1);
        free(buf2);
        vSemaphoreDelete(access_semaphore);
        return ESP_FAIL;
    }

    // Clear the LED strip
    led_strip_clear(&led_strip);
    led_strip_show(&led_strip);

    ESP_LOGI(TAG, "WS2812B LED strip initialized successfully");
    return ESP_OK;
}

void update_led_strip(framebuffer_t *fb) {
    // Convert RGB framebuffer to LED strip colors
    for (int i = 0; i < LED_NUM; i++) {
        uint8_t r = fb->pixels[i].r / 3;
        uint8_t g = fb->pixels[i].g / 3;
        uint8_t b = fb->pixels[i].b / 3;
        led_strip_set_pixel_rgb(&led_strip, i, r, g, b);
        // Set pixel color with RGB values
    }

    // Update the LED strip
    led_strip_show(&led_strip);
}

esp_err_t initialize_spi_slave(void) {
    ESP_LOGI(TAG, "ESP32 Slave Initializing...");

    // SPI bus configuration
    spi_bus_config_t buscfg = {.miso_io_num = PIN_NUM_MISO,
                               .mosi_io_num = PIN_NUM_MOSI,
                               .sclk_io_num = PIN_NUM_CLK,
                               .quadwp_io_num = -1,
                               .quadhd_io_num = -1,
                               .max_transfer_sz =
                                   sizeof(framebuffer_64x64_t) + 10};

    // SPI slave configuration
    spi_slave_interface_config_t slvcfg = {
        .mode = 0, // SPI mode 0 (must match master)
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL};

    // Initialize SPI slave
    esp_err_t ret =
        spi_slave_initialize(SPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "ESP32 SPI Slave initialized successfully");
    ESP_LOGI(TAG, "MISO: GPIO %d, MOSI: GPIO %d, SCLK: GPIO %d, CS: GPIO %d",
             PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);

    return ESP_OK;
}

void log_received_64x64_data(framebuffer_64x64_t *fb) {
    printf("\033[2J\033[H"); // Clear screen and move cursor to top
    printf("=== SPI SLAVE - Received Data Debug ===\n");

    // Show 16x16 scaled view of received 64x64 data
    printf("Received 64x64 Data (16x16 scaled view):\n");
    printf("┌────────────────┐\n");
    for (int y = 0; y < INPUT_HEIGHT; y += 4) {
        printf("│");
        for (int x = 0; x < INPUT_WIDTH; x += 4) {
            int index = y * INPUT_WIDTH + x;
            rgb_pixel_t pixel = fb->pixels[index];

            // Determine color based on RGB values
            if (pixel.r == 255 && pixel.g == 0 && pixel.b == 0) {
                printf("\033[1;31m█\033[0m"); // Player (red)
            } else if (pixel.r == 128 && pixel.g == 128 && pixel.b == 128) {
                printf("\033[1;37m█\033[0m"); // Platforms (gray/white)
            } else if (pixel.r > 0 || pixel.g > 0 || pixel.b > 0) {
                printf("\033[1;37m░\033[0m"); // Other colors
            } else {
                printf(" "); // Empty
            }
        }
        printf("│\n");
    }
    printf("└────────────────┘\n");
}

bool verify_checksum(framebuffer_64x64_t *fb) {
    uint32_t checksum = 0;
    for (size_t i = 0; i < INPUT_WIDTH * INPUT_HEIGHT; i++) {
        checksum += fb->pixels[i].r;
        checksum += fb->pixels[i].g;
        checksum += fb->pixels[i].b;
    }

    bool valid = checksum == fb->checksum;
    if (!valid) {
        printf("CHECKSUM MISMATCH! Calculated: 0x%08lx, Received: 0x%08lx\n",
               checksum, fb->checksum);
    }
    return valid;
}

void handle_spi_communication(void) {
    static int frame_counter = 0;
    static int error_counter = 0;
    frame_counter++;

    // Allocate large buffers on HEAP instead of stack
    framebuffer_64x64_t *receive_data_64x64 =
        (framebuffer_64x64_t *)malloc(sizeof(framebuffer_64x64_t));
    uint8_t *send_data = (uint8_t *)malloc(sizeof(framebuffer_64x64_t));

    if (!receive_data_64x64 || !send_data) {
        ESP_LOGE(TAG, "Failed to allocate SPI buffers");
        if (receive_data_64x64)
            free(receive_data_64x64);
        if (send_data)
            free(send_data);
        return;
    }

    memset(send_data, 0, sizeof(framebuffer_64x64_t)); // We can send zeros back

    // Small 10x10 buffer can stay on stack (300 bytes for RGB)
    framebuffer_t display_fb_10x10;

    spi_slave_transaction_t t = {.length =
                                     sizeof(framebuffer_64x64_t) * 8, // In bits
                                 .tx_buffer = send_data,
                                 .rx_buffer = receive_data_64x64,
                                 .user = NULL};

    // Wait for SPI transaction from master
    esp_err_t ret = spi_slave_transmit(SPI_HOST, &t, portMAX_DELAY);
    if (ret == ESP_OK) {
        bool data_valid = verify_checksum(receive_data_64x64);
        if (!data_valid) {
            error_counter++;
            printf("\n=== FRAME %d - DATA CORRUPTED (Errors: %d) ===\n",
                   frame_counter, error_counter);

            // Debug: show what we received vs what we expected
            uint32_t calculated = 0;
            for (int i = 0; i < INPUT_WIDTH * INPUT_HEIGHT; i++) {
                calculated += receive_data_64x64->pixels[i].r;
                calculated += receive_data_64x64->pixels[i].g;
                calculated += receive_data_64x64->pixels[i].b;
            }
            printf("Expected checksum: 0x%08lx\n",
                   receive_data_64x64->checksum);
            printf("Calculated checksum: 0x%08lx\n", calculated);

            // Show first few corrupted pixels
            printf("First 5 corrupted pixels:\n");
            for (int i = 0; i < 5; i++) {
                printf("  Pixel %d: R=%d G=%d B=%d\n", i,
                       receive_data_64x64->pixels[i].r,
                       receive_data_64x64->pixels[i].g,
                       receive_data_64x64->pixels[i].b);
            }
        } else {
            printf("\n=== FRAME %d - DATA VALID ===\n", frame_counter);
        }
        // Shrink 64x64 RGB framebuffer down to 10x10
        shrink_framebuffer_64x64_to_10x10(receive_data_64x64,
                                          &display_fb_10x10);

        // Display debug information
        printf("\n=== SPI Frame %d ===\n", frame_counter);

        // Show received 64x64 data (scaled)
        log_received_64x64_data(receive_data_64x64);

        printf("\nPress any key to continue to next frame...\n");

        // Update LED strip with shrunken 10x10 RGB framebuffer
        update_led_strip(&display_fb_10x10);

    } else {
        ESP_LOGE(TAG, "SPI slave transaction failed: 0x%x", ret);
    }

    // Free the heap-allocated buffers
    free(receive_data_64x64);
    free(send_data);
}

void app_main(void) {
    // Initialize SPI slave
    esp_err_t ret = initialize_spi_slave();
    if (ret != ESP_OK) {
        return;
    }

    // Initialize LED strip
    ret = initialize_led_strip();
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to initialize LED strip, continuing without LED output");
        // Continue without LED functionality
    }

    ESP_LOGI(TAG,
             "Slave ready - waiting for RGB framebuffer data from master...");
    ESP_LOGI(TAG, "LED strip: %d LEDs on GPIO %d", LED_NUM, LED_STRIP_GPIO);

    // Clear screen at start
    printf("\033[2J\033[H");

    // Main loop for SPI communication
    while (1) {
        handle_spi_communication();
    }
}
