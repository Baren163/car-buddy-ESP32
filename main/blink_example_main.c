
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
//mpu6050 library uses i2cdev.h
#include <mpu6050.h>
#include <esp_err.h>
#include "driver/spi_master.h"

static const char *LED_TAG = "example";
static const char *TAG = "mpu6050_test";
static const char *EEPROM_TAG = "eeprom";

#define PIN_NUM_MOSI   6
#define PIN_NUM_SCLK   4
#define PIN_NUM_CS     2 
#define PIN_NUM_DC     1
#define PIN_NUM_RST    0

#define MPU_ADDRESS 104
#define EEPROM_ADDR 0x50

#define SDA_PIN 20
#define SCL_PIN 21

// Display dimensions
#define SCREEN_WIDTH   132
#define SCREEN_HEIGHT  162
#define PIXEL_COUNT    (SCREEN_WIDTH * SCREEN_HEIGHT)  // 21,384 pixels

static uint8_t *frame_buffer = NULL;  // DMA buffer for batch transfer
static uint8_t *plane_buffer = NULL;

// ESP32-C3 DMA max transfer is 4092 bytes
// We'll use 4000 bytes per chunk for safety
#define MAX_DMA_TRANSFER 4000
#define CHUNK_SIZE       (MAX_DMA_TRANSFER / 2)  // 2000 pixels per chunk
#define NUM_CHUNKS       ((PIXEL_COUNT + CHUNK_SIZE - 1) / CHUNK_SIZE)

// SPI configuration
#define SPI_HOST       SPI2_HOST  // Using SPI2 peripheral (VSPI)
#define SPI_CLOCK_SPEED 4000000   // 4 MHz (fosc/4 equivalent for 16MHz ESP32)

static spi_device_handle_t spi_handle;


i2c_dev_t eeprom_dev;
mpu6050_dev_t mpu_dev;


float AccelY = 0;
int pos = 1;
int reverseBool = 0;
uint16_t REG = 0;
uint16_t ROW = 0;
int drawImageY = 35;


// Initialize SPI peripheral
void spi_init(void) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,  // Not used (SPI display typically doesn't use MISO)
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_DMA_TRANSFER
    };
    
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED,
        .mode = 0,                    // SPI mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX  // Half-duplex for display communication
    };
    
    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Bus initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Add SPI device
    ret = spi_bus_add_device(SPI_HOST, &dev_cfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Device add failed: %s", esp_err_to_name(ret));
        return;
    }
}


// Batch transfer with chunking for large data
void spi_transfer_batch(const uint8_t *data, size_t length) {

    // DC high for data mode
    gpio_set_level(PIN_NUM_DC, 1);

    size_t bytes_sent = 0;
    
    while (bytes_sent < length) {
        size_t chunk = length - bytes_sent;
        if (chunk > MAX_DMA_TRANSFER) {
            chunk = MAX_DMA_TRANSFER;
        }
        
        spi_transaction_t trans = {
            .length = chunk * 8,
            .tx_buffer = data + bytes_sent,
            .rx_buffer = NULL
        };
        
        ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &trans));
        bytes_sent += chunk;
    }
}


// Single byte transfer
uint8_t spi_transfer(uint8_t data) {
    spi_transaction_t trans = {
        .length = 8,          // 8 bits
        .tx_buffer = &data,
        .rx_buffer = NULL,    // We don't need to read back for display
        .rxlength = 0
    };
    
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &trans));
    return 0;  // Return dummy value for compatibility
}


// Write command byte
void spi_write_command(uint8_t cmd) {
    gpio_set_level(PIN_NUM_DC, 0);  // Command mode (DC LOW)
    spi_transfer(cmd);
}

// Write data byte
void spi_write_data(uint8_t data) {
    gpio_set_level(PIN_NUM_DC, 1);  // Data mode (DC HIGH)
    spi_transfer(data);
}

// Write constant color multiple times (16-bit color)
void spi_write_x_constant_colour(uint16_t data, uint16_t x) {
    if (frame_buffer == NULL) return;
    
    // Fill buffer quickly (2 bytes per pixel, 16-bit color)
    uint16_t *buffer_16 = (uint16_t*)frame_buffer;
    for (int i = 0; i < x; i++) {
        buffer_16[i] = data;
    }
    
    gpio_set_level(PIN_NUM_DC, 1);  // Data mode
    
    spi_transaction_t trans = {
        .length = x * 16,      // Total bits (x * 2 bytes * 8 bits)
        .tx_buffer = frame_buffer,
        .rx_buffer = NULL
    };
    
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &trans));
}


// Reset display
void spi_reset(void) {
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

// Initialize screen with ST7735S
void screen_init(void) {

    // Allocate DMA-capable frame buffer
    frame_buffer = heap_caps_malloc(PIXEL_COUNT * 2, MALLOC_CAP_DMA);
    if (frame_buffer == NULL) {
        ESP_LOGE("SCREEN", "Failed to allocate frame buffer!");
        return;
    }

    // Allocate DMA-capable frame buffer for plane image
    plane_buffer = heap_caps_malloc((48*80)*2, MALLOC_CAP_DMA); // 48 rows * 80 pixels per row * 2 bytes per pixel
    if (plane_buffer == NULL) {
        ESP_LOGE("SCREEN", "Failed to allocate frame buffer!");
        return;
    }

    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_CS) | 
                       (1ULL << PIN_NUM_DC) | 
                       (1ULL << PIN_NUM_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Initialize SPI
    spi_init();

    // Reset sequence
    spi_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // ST7735S Initialization sequence
    spi_write_command(0x01);  // Software reset
    vTaskDelay(pdMS_TO_TICKS(150));
    
    spi_write_command(0x11);  // Sleep out
    vTaskDelay(pdMS_TO_TICKS(100));
    
    spi_write_command(0x3A);  // Interface pixel format
    spi_write_data(0x05);     // 16-bit/pixel
    
    spi_write_command(0x36);  // MADCTL
    spi_write_data(0xA0);     // 0b10100000 = MY | MV | BGR
    
    spi_write_command(0x38);  // Idle mode OFF
    vTaskDelay(pdMS_TO_TICKS(100));
    
    spi_write_command(0x29);  // Display ON
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Set address window
void spi_set_addr_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    spi_write_command(0x2A);  // Column address
    spi_write_data(0x00);
    spi_write_data(x0);
    spi_write_data(0x00);
    spi_write_data(x1);

    spi_write_command(0x2B);  // Row address
    spi_write_data(0x00);
    spi_write_data(y0);
    spi_write_data(0x00);
    spi_write_data(y1);

    spi_write_command(0x2C);  // Memory write
}

uint16_t spi_return_colour_from_pallete(uint8_t palleteID) {

  switch (palleteID) {

    case 0:
    //   spi_transfer(0x00);
    //   spi_transfer(0x00);
      return 0x0000;  // Black
      break;

    case 1:
    //   spi_transfer(0);
    //   spi_transfer(0);
      return 0x0000;  // Black
      break;

    case 2:
    //   spi_transfer(0xA6);
    //   spi_transfer(0x18);
      return 0xA618;
      break;

    case 3:
    //   spi_transfer(0x64);
    //   spi_transfer(0x10);
      return 0x6410;
      break;

    case 4:
    //   spi_transfer(0x85);
    //   spi_transfer(0x14);
      return 0x8514;
      break;

    case 5:
    //   spi_transfer(0xC7);
    //   spi_transfer(0x1C);
      return 0xC71C;
      break;

    case 6:
    //   spi_transfer(0xE7);
    //   spi_transfer(0x1C);
      return 0xE71C;
      break;

    case 7:
    //   spi_transfer(0x01);
    //   spi_transfer(0x04);
      return 0x0104;
      break;

    case 8:
    //   spi_transfer(0x22);
    //   spi_transfer(0x08);
      return 0x2208;
      break;

    case 9:
    //   spi_transfer(0xE5);
    //   spi_transfer(0x0C);
        return 0xE50C;
      break;

    case 10:
    //   spi_transfer(0xA0);
    //   spi_transfer(0x00);
        return 0xA000;
      break;

    case 11:
    //   spi_transfer(0x21);
    //   spi_transfer(0x04);
        return 0x2104;
      break;

    case 12:
    //   spi_transfer(0xC6);
    //   spi_transfer(0x18);
        return 0xC618;
      break;

    default:
    //   spi_transfer(0x00);
    //   spi_transfer(0x00);
        return 0x0000;  // Default to black
      break;
  }
}


void drawBackground() {
    if (frame_buffer == NULL) return;
    
    // Set address window
    spi_set_addr_window(0, 0, SCREEN_HEIGHT - 1, SCREEN_WIDTH - 1);
    
    // DC high for data mode
    gpio_set_level(PIN_NUM_DC, 1);
    
    // Send frame buffer in chunks (automatically handles splitting)
    spi_transfer_batch(frame_buffer, PIXEL_COUNT * 2);
}

// Update frame buffer with new color
void updateFrameBuffer(uint16_t color) {
    if (frame_buffer == NULL) return;
    
    // Fill buffer quickly (2 bytes per pixel, 16-bit color)
    uint16_t *buffer_16 = (uint16_t*)frame_buffer;
    for (int i = 0; i < PIXEL_COUNT; i++) {
        buffer_16[i] = color;
    }
}


float read_mpu(mpu6050_dev_t *dev)
{
    mpu6050_raw_acceleration_t accel = { 0 };

    ESP_ERROR_CHECK(mpu6050_get_raw_acceleration(dev, &accel));

    // ESP_LOGI("MPU", "Accel: x=%.4f y=%.4f z=%.4f",
    //          accel.x, accel.y, accel.z);
    
    return accel.x;
}


esp_err_t eeprom_sequential_read_from_register(uint16_t read_addr, uint8_t *data, uint16_t number_of_bytes_to_read)
{
    // Control byte, address HIGH byte, address LOW byte, repeated start,
	uint8_t out_buf[2];
	int out_index = 0;
	uint8_t high_addr = (read_addr >> 8) & 0xff;
	uint8_t low_addr = read_addr & 0xff;
	out_buf[out_index++] = high_addr;
	out_buf[out_index++] = low_addr;

	//esp_err_t ret = i2c_master_transmit_receive(eeprom_dev, out_buf, sizeof(out_buf), data, number_of_bytes_to_read, -1);
	
    esp_err_t ret = i2c_dev_read(&eeprom_dev, out_buf, sizeof(out_buf), data, number_of_bytes_to_read);

    return ret;
}


void update_plane_buffer(uint16_t reg_index) {

    if (plane_buffer == NULL) {
        return;
    }

    // Read packed data into the START of plane_buffer (1920 bytes used, rest untouched for now)
    eeprom_sequential_read_from_register(reg_index, plane_buffer, 1920);

    uint8_t packed, colorID;
    uint16_t color;

    if (reverseBool == 0) {
        // Iterate backwards so writes never clobber unread packed bytes
        for (int i = ((48 * 80) / 2) - 1; i >= 0; i--) {
            packed = plane_buffer[i];  // Read packed byte BEFORE it gets overwritten
            for (int pixel = 1; pixel >= 0; pixel--) {
                colorID = (pixel == 0) ? (packed >> 4) : (packed & 0x0F);
                color = spi_return_colour_from_pallete(colorID);

                plane_buffer[(i * 4) + (pixel * 2)]     = (color >> 8) & 0xFF;  // High byte
                plane_buffer[(i * 4) + (pixel * 2) + 1] =  color & 0xFF;        // Low byte
            }
        }
    } else {
        // Image is already stored a certain way in eeprom so I need to rewrite/convert the colorIDs in a way where each row of the image is reversed
        //making the plane appear mirrored on the display while still preventing clobbering of unread packed bytes
        // Iterate backwards so writes never clobber unread packed bytes
        for (int i = (48 - 1); i >= 0; i--) {
            if (i == 0) {
                // For the first row, we need to read the packed bytes before we overwrite them, so we read them into a temporary buffer
                uint8_t temp_buffer[80/2];  // 40 packed bytes for 80 pixels
                for (int j = 0; j < 80/2; j++) {
                    temp_buffer[j] = plane_buffer[(i * 40) + j];
                }
                // Now we can safely write to the plane_buffer without worrying about clobbering unread data
                 for (int j = 0; j < 80/2; j++) {
                    packed = temp_buffer[j];
                    for (int pixel = 1; pixel >= 0; pixel--) {
                        colorID = (pixel == 1) ? (packed >> 4) : (packed & 0x0F);   // Flip pixel order for mirroring
                        color = spi_return_colour_from_pallete(colorID);

                        plane_buffer[(i * 160) + ((39-j) * 4) + (pixel * 2)]     = (color >> 8) & 0xFF;  // High byte
                        plane_buffer[(i * 160) + ((39-j) * 4) + (pixel * 2) + 1] =  color & 0xFF;        // Low byte
                    }
                }
                break;  // We've processed the first row, we can break out of the loop now
            }

            for (int j = 0; j < 80/2; j++) {
                packed = plane_buffer[(i * 40) + j];
                for (int pixel = 1; pixel >= 0; pixel--) {
                    colorID = (pixel == 1) ? (packed >> 4) : (packed & 0x0F);   // Flip pixel order for mirroring
                    color = spi_return_colour_from_pallete(colorID);

                    plane_buffer[(i * 160) + ((39-j) * 4) + (pixel * 2)]     = (color >> 8) & 0xFF;  // High byte
                    plane_buffer[(i * 160) + ((39-j) * 4) + (pixel * 2) + 1] =  color & 0xFF;        // Low byte
                }
            }
        }
    }
}


void draw_plane_buffer(int num) {
    if (plane_buffer == NULL) return;

    // Set address window for the plane
    spi_set_addr_window(0 + num, drawImageY, 79 + num, drawImageY + 47);

    // DC high for data mode
    gpio_set_level(PIN_NUM_DC, 1);

    // Send plane buffer in chunks (automatically handles splitting)
    spi_transfer_batch(plane_buffer, (80 * 48) * 2);
}


void app_main(void)
{
    ESP_ERROR_CHECK(i2cdev_init());

    // Init mpu6050 i2c dev using mpu6050.h library (wrapper library for i2cdev.h)
    ESP_ERROR_CHECK(mpu6050_init_desc(&mpu_dev, MPU_ADDRESS, 0, SDA_PIN, SCL_PIN));
    ESP_ERROR_CHECK(mpu6050_init(&mpu_dev));

    // Init 24LC256 eeprom i2c dev with i2cdev.h library (one level lower than mpu6050, no wrapper library)
    eeprom_dev.cfg.sda_io_num = SDA_PIN;
    eeprom_dev.cfg.scl_io_num = SCL_PIN;
    eeprom_dev.cfg.master.clk_speed = 400000;
    eeprom_dev.addr = EEPROM_ADDR;

    ESP_ERROR_CHECK(i2c_dev_create_mutex(&eeprom_dev));


    // Initialize the screen
    screen_init();

    updateFrameBuffer(0x0000);
    // Draw entire screen in one DMA transfer
    drawBackground();
    
    update_plane_buffer(0);

    int counter = 0;


    while (1)
    {

        draw_plane_buffer(pos);

        vTaskDelay(pdMS_TO_TICKS(10));

        AccelY = read_mpu(&mpu_dev);

        // Map [-17000, +17000] to [0, 90]
        if (AccelY < -17000) AccelY = -17000;
        if (AccelY > 17000) AccelY = 17000;
        pos = (((int)AccelY + 17000) / 424);

        // pos = (0.9*pos) + (0.1*AccelY);

        if (pos < 40) {
        reverseBool = 1;
        } else {
        reverseBool = 0;
        }


        counter++;

        if (counter >= 5) {  // Update every so often
            counter = 0;
            if (abs(pos - 40) < 5) {
                update_plane_buffer(0);
            } else if (abs(pos - 40) < 10) {
                update_plane_buffer(1920);
            } else if (abs(pos - 40) < 15) {
                update_plane_buffer(3840);
            } else if (abs(pos - 40) < 20) {
                update_plane_buffer(5760);
            } else if (abs(pos - 40) < 25) {
                update_plane_buffer(7680);
            } else if (abs(pos - 40) < 30) {
                update_plane_buffer(9600);
            } else if (abs(pos - 40) < 35) {
                update_plane_buffer(11520);
            } else {
                update_plane_buffer(13440);
            }
        }

            
            // countDown--;  // Clear the side-debris
            // if (countDown == 0) {

            //     spi_set_addr_window(0, drawImageY, pos, drawImageY + 48);

            //     digitalWrite(spi_DC, HIGH); // Data mode
            //     digitalWrite(spi_CS, LOW);
            //     for (int i = 0; i < (pos*48); i++) {
            //     spi_transfer(0);
            //     spi_transfer(0);
            //     }
            //     digitalWrite(spi_CS, HIGH);


            //     spi_set_addr_window(pos+80, drawImageY, 160, drawImageY + 48);

            //     digitalWrite(spi_DC, HIGH); // Data mode
            //     digitalWrite(spi_CS, LOW);
            //     for (int i = 0; i < ((80-pos)*48); i++) {
            //     spi_transfer(0);
            //     spi_transfer(0);  
            //     }
            //     digitalWrite(spi_CS, HIGH);


            //     countDown = 4;

        // }


    }
}
