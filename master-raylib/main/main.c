#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <stdint.h>
#include <string.h>

// Custom SPI pins for ESP32-S3 MASTER
#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 7
#define PIN_NUM_MOSI 12
#define PIN_NUM_CLK 13
#define PIN_NUM_CS 14

// Button pins for platformer controls
#define BUTTON_JUMP_GPIO 16  // Red button - Jump
#define BUTTON_RIGHT_GPIO 42 // Green button - Move Right
#define BUTTON_LEFT_GPIO 18  // Blue button - Move Left

// Display parameters
#define DISPLAY_WIDTH 64
#define DISPLAY_HEIGHT 64

// Platformer physics parameters
#define GRAVITY 100
#define PLAYER_JUMP_SPD 40.0f
#define PLAYER_HOR_SPD 20.0f

static const char *TAG = "PLATFORMER_S3";

// RGB pixel structure
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_pixel_t;

// Framebuffer structure - each pixel is 3 bytes (RGB)
typedef struct {
    rgb_pixel_t pixels[DISPLAY_WIDTH * DISPLAY_HEIGHT];
    uint32_t checksum;
} framebuffer_t;

// Player structure
typedef struct {
    float position_x;
    float position_y;
    float speed;
    bool canJump;
} Player;

// Environment item structure
typedef struct {
    int x;
    int y;
    int width;
    int height;
} EnvItem;

// Function declarations
esp_err_t initialize_spi_bus(void);
esp_err_t initialize_spi_device(spi_device_handle_t *spi_handle);
esp_err_t initialize_buttons(void);
void update_player(Player *player, EnvItem *envItems, int envItemsLength,
                   float delta);
void update_framebuffer(framebuffer_t *fb, Player *player, EnvItem *envItems,
                        int envItemsLength);
void draw_rectangle(framebuffer_t *fb, int x, int y, int width, int height,
                    uint8_t r, uint8_t g, uint8_t b);
void send_framebuffer(spi_device_handle_t spi_handle, framebuffer_t *fb);

// Global player instance
Player player = {32.0f, 20.0f, 0.0f, false};

// Environment items (platforms)
EnvItem envItems[] = {
    {0, 56, 64, 8},  // Ground platform
    {16, 44, 32, 4}, // Middle platform
    {8, 32, 16, 4},  // Left platform
    {40, 32, 16, 4}  // Right platform
};
int envItemsLength = sizeof(envItems) / sizeof(envItems[0]);

// Function definitions
esp_err_t initialize_spi_bus(void) {
    ESP_LOGI(TAG, "Initializing SPI bus...");

    spi_bus_config_t buscfg = {.miso_io_num = PIN_NUM_MISO,
                               .mosi_io_num = PIN_NUM_MOSI,
                               .sclk_io_num = PIN_NUM_CLK,
                               .quadwp_io_num = -1,
                               .quadhd_io_num = -1,
                               .data4_io_num = -1,
                               .data5_io_num = -1,
                               .data6_io_num = -1,
                               .data7_io_num = -1,
                               .max_transfer_sz = sizeof(framebuffer_t) + 10,
                               .flags = 0,
                               .intr_flags = 0};

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

    spi_device_interface_config_t devcfg = {.command_bits = 0,
                                            .address_bits = 0,
                                            .dummy_bits = 0,
                                            .mode = 0,
                                            .duty_cycle_pos = 0,
                                            .cs_ena_pretrans = 0,
                                            .cs_ena_posttrans = 0,
                                            .clock_speed_hz = 1 * 1000 * 1000,
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

esp_err_t initialize_buttons(void) {
    ESP_LOGI(TAG, "Initializing buttons for platformer controls...");

    gpio_config_t button_config = {.pin_bit_mask = (1ULL << BUTTON_JUMP_GPIO) |
                                                   (1ULL << BUTTON_RIGHT_GPIO) |
                                                   (1ULL << BUTTON_LEFT_GPIO),
                                   .mode = GPIO_MODE_INPUT,
                                   .pull_up_en = GPIO_PULLUP_ENABLE,
                                   .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                   .intr_type = GPIO_INTR_DISABLE};

    esp_err_t ret = gpio_config(&button_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure buttons: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(
        TAG,
        "Buttons initialized: Jump(GPIO %d), Right(GPIO %d), Left(GPIO %d)",
        BUTTON_JUMP_GPIO, BUTTON_RIGHT_GPIO, BUTTON_LEFT_GPIO);

    return ESP_OK;
}

void update_player(Player *player, EnvItem *envItems, int envItemsLength,
                   float delta) {
    // Read button states
    bool leftPressed = (gpio_get_level(BUTTON_LEFT_GPIO) == 0);
    bool rightPressed = (gpio_get_level(BUTTON_RIGHT_GPIO) == 0);
    bool jumpPressed = (gpio_get_level(BUTTON_JUMP_GPIO) == 0);

    // Horizontal movement
    if (leftPressed)
        player->position_x -= PLAYER_HOR_SPD * delta;
    if (rightPressed)
        player->position_x += PLAYER_HOR_SPD * delta;

    // Jumping
    if (jumpPressed && player->canJump) {
        player->speed = -PLAYER_JUMP_SPD;
        player->canJump = false;
    }

    // Apply gravity and check collisions
    bool hitObstacle = false;

    // Temporary position for collision checking
    float new_y = player->position_y + player->speed * delta;

    for (int i = 0; i < envItemsLength; i++) {
        EnvItem *ei = &envItems[i];

        // Check if player would collide with this platform
        // Player is 8x8 pixels, positioned at center-bottom
        float player_left = player->position_x - 4;
        float player_right = player->position_x + 4;
        float player_bottom = new_y + 4; // Bottom of player

        // Platform bounds
        float platform_left = ei->x;
        float platform_right = ei->x + ei->width;
        float platform_top = ei->y;

        // Check if player is above platform and falling onto it
        if (player_bottom >= platform_top &&
            player->position_y + 4 <= platform_top && // Was above platform
            player_right > platform_left && player_left < platform_right &&
            player->speed >= 0) { // Only check when falling

            hitObstacle = true;
            player->speed = 0.0f;
            player->position_y =
                platform_top - 4; // Position player on top of platform
            break;
        }
    }

    if (!hitObstacle) {
        player->position_y = new_y;
        player->speed += GRAVITY * delta;
        player->canJump = false;
    } else {
        player->canJump = true;
    }

    // Keep player within screen bounds
    if (player->position_x < 4)
        player->position_x = 4;
    if (player->position_x > DISPLAY_WIDTH - 4)
        player->position_x = DISPLAY_WIDTH - 4;
    if (player->position_y > DISPLAY_HEIGHT - 4) {
        player->position_y = DISPLAY_HEIGHT - 4;
        player->speed = 0;
        player->canJump = true;
    }
}

void draw_rectangle(framebuffer_t *fb, int x, int y, int width, int height,
                    uint8_t r, uint8_t g, uint8_t b) {
    for (int py = y; py < y + height && py < DISPLAY_HEIGHT; py++) {
        for (int px = x; px < x + width && px < DISPLAY_WIDTH; px++) {
            if (px >= 0 && py >= 0) {
                int index = py * DISPLAY_WIDTH + px;
                fb->pixels[index].r = r;
                fb->pixels[index].g = g;
                fb->pixels[index].b = b;
            }
        }
    }
}

uint32_t calculate_checksum(framebuffer_t *fb) {
    uint32_t checksum = 0;
    for (int i = 0; i < DISPLAY_WIDTH * DISPLAY_HEIGHT; i++) {
        checksum += fb->pixels[i].r;
        checksum += fb->pixels[i].g;
        checksum += fb->pixels[i].b;
    }
    return checksum;
}

void update_framebuffer(framebuffer_t *fb, Player *player, EnvItem *envItems,
                        int envItemsLength) {
    // Clear framebuffer (transparent/black background)
    memset(fb->pixels, 0, sizeof(fb->pixels));

    // Draw platforms (gray)
    for (int i = 0; i < envItemsLength; i++) {
        draw_rectangle(fb, envItems[i].x, envItems[i].y, envItems[i].width,
                       envItems[i].height, 128, 128, 128); // Gray color
    }

    // Draw player (red) - 8x8 pixels centered at player position
    int player_x = (int)player->position_x - 4;
    int player_y = (int)player->position_y - 4;
    draw_rectangle(fb, player_x, player_y, 8, 8, 255, 0, 0); // Red color

    fb->checksum = calculate_checksum(fb);
}

void send_framebuffer(spi_device_handle_t spi_handle, framebuffer_t *fb) {
    uint8_t *receive_buffer = (uint8_t *)malloc(sizeof(framebuffer_t));
    if (receive_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate receive buffer");
        return;
    }

    spi_transaction_t t = {.flags = 0,
                           .cmd = 0,
                           .addr = 0,
                           .length = sizeof(framebuffer_t) * 8,
                           .rxlength = sizeof(framebuffer_t) * 8,
                           .user = NULL,
                           .tx_buffer = fb,
                           .rx_buffer = receive_buffer};

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Framebuffer sent successfully");
    } else {
        ESP_LOGE(TAG, "SPI framebuffer transaction failed: 0x%x", ret);
    }

    free(receive_buffer);
}

void log_mini_framebuffer(framebuffer_t *fb, Player *player) {
    printf("\033[2J\033[H"); // Clear screen and move cursor to top
    printf("=== ESP32-S3 Platformer (Live 16x16 View) ===\n");
    printf("Player: (%.1f, %.1f) Speed: %.1f CanJump: %d\n", player->position_x,
           player->position_y, player->speed, player->canJump);

    // Button states
    bool leftPressed = (gpio_get_level(BUTTON_LEFT_GPIO) == 0);
    bool rightPressed = (gpio_get_level(BUTTON_RIGHT_GPIO) == 0);
    bool jumpPressed = (gpio_get_level(BUTTON_JUMP_GPIO) == 0);
    printf("Buttons: Left:%-3s Right:%-3s Jump:%-3s\n",
           leftPressed ? "\033[1;32mON\033[0m" : "OFF",
           rightPressed ? "\033[1;32mON\033[0m" : "OFF",
           jumpPressed ? "\033[1;32mON\033[0m" : "OFF");

    printf("┌────────────────┐\n");
    for (int y = 0; y < DISPLAY_HEIGHT; y += 4) {
        printf("│");
        for (int x = 0; x < DISPLAY_WIDTH; x += 4) {
            int index = y * DISPLAY_WIDTH + x;
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
    printf("Legend: \033[1;31m█\033[0m=Player \033[1;37m█\033[0m=Platforms\n");
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-S3 Platformer Initializing...");

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

    // Initialize buttons for platformer controls
    ret = initialize_buttons();
    if (ret != ESP_OK) {
        return;
    }

    // Create framebuffer on heap
    framebuffer_t *display_fb = (framebuffer_t *)malloc(sizeof(framebuffer_t));
    if (display_fb == NULL) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer memory");
        return;
    }
    memset(display_fb, 0, sizeof(framebuffer_t));

    ESP_LOGI(TAG, "Platformer initialized successfully");
    ESP_LOGI(TAG, "Display: %dx%d pixels", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    ESP_LOGI(TAG, "Controls: Left(GPIO %d), Right(GPIO %d), Jump(GPIO %d)",
             BUTTON_LEFT_GPIO, BUTTON_RIGHT_GPIO, BUTTON_JUMP_GPIO);

    TickType_t last_frame_time = xTaskGetTickCount();
    const TickType_t frame_delay = pdMS_TO_TICKS(16); // ~60 FPS
    int frame_counter = 0;

    // Clear screen at start
    printf("\033[2J\033[H");

    // Main game loop
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        float delta_time = (float)(current_time - last_frame_time) *
                           portTICK_PERIOD_MS / 1000.0f;
        last_frame_time = current_time;
        frame_counter++;

        // Update player physics and input
        update_player(&player, envItems, envItemsLength, delta_time);

        // Update framebuffer with current game state
        update_framebuffer(display_fb, &player, envItems, envItemsLength);

        // Send framebuffer over SPI
        send_framebuffer(spi, display_fb);

        // Display mini framebuffer for every frame
        log_mini_framebuffer(display_fb, &player);
        printf("Frame: %d, FPS: %.1f\n", frame_counter, 1.0f / delta_time);

        // Frame rate control
        vTaskDelay(frame_delay);
    }

    free(display_fb);
}
