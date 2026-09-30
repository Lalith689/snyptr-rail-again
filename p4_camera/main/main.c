#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/jpeg_encode.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_cam_sensor.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "driver/uart_vfs.h"

// ============================================================================
// LALITH689/CAMERA ARCHITECTURE + ORANGE/YELLOW OPTIMAL-ZONE NERF DETECTOR
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t magic[4];       // 0xDE, 0xAD, 0xBE, 0xEF
    uint8_t laser_found;    // 0 = no, 1 = yes (Nerf Hit in Optimal Zone!)
    uint8_t zone;           // 0 = none, 1 = green, 2 = red
    uint8_t score_ring;     // 0 = miss, 1..10 = ring, 11 = 10X
    uint8_t is_calibrated;  // 0 = no, 1 = yes
    uint16_t laser_x_px;    // Camera pixel coordinates (0..799)
    uint16_t laser_y_px;
    float laser_x_mm;       // Calibrated coordinates in mm relative to target card
    float laser_y_mm;
    float laser_dist_mm;    // Radial distance from center in mm
    uint8_t padding[8];     // Padding to ensure exactly 32 bytes
} shot_metadata_t;

static float g_homography[9] = {1.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,  0.0f, 0.0f, 1.0f};
static bool g_is_calibrated = true;

#define STREAM_MAX_FPS          15
#define I2C_MASTER_SCL_IO       GPIO_NUM_8
#define I2C_MASTER_SDA_IO       GPIO_NUM_7
#define I2C_MASTER_NUM          I2C_NUM_0

#define BRIDGE_UART_NUM         UART_NUM_1
#define BRIDGE_TX_PIN           GPIO_NUM_21
#define BRIDGE_RX_PIN_PRIMARY   GPIO_NUM_22
#define BRIDGE_UART_ALT_NUM     UART_NUM_2
#define BRIDGE_RX_PIN_ALT       GPIO_NUM_27
#define BRIDGE_BAUD             115200

// Optimal Zone (Green Circle in Camera Viewport: center=(400,400), radius=260px)
#define TARGET_CENTER_X         400
#define TARGET_CENTER_Y         400
#define OPTIMAL_ZONE_RADIUS_PX  210
#define OPTIMAL_ZONE_RADIUS_SQ  (OPTIMAL_ZONE_RADIUS_PX * OPTIMAL_ZONE_RADIUS_PX)
#define GRID_DIM                145
static uint8_t s_dark_target_mask[GRID_DIM][GRID_DIM];
static uint8_t s_base_r[GRID_DIM][GRID_DIM];
static uint8_t s_base_g[GRID_DIM][GRID_DIM];
static uint8_t s_base_b[GRID_DIM][GRID_DIM];
static uint8_t s_tip_hit_grid[GRID_DIM][GRID_DIM];

static int g_video_fd = -1;
static uint8_t *g_buffers[2] = {NULL, NULL};
static uint32_t g_buf_len[2] = {0, 0};

static jpeg_encoder_handle_t g_jpeg_handle = NULL;
static uint8_t *g_jpeg_out_buf = NULL;
static uint32_t g_jpeg_out_size = 0;

static uint32_t calculate_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static void send_packet(uint32_t frame_id, const uint8_t *payload, uint32_t payload_len)
{
    uint8_t header[15];
    header[0] = 0xAA;
    header[1] = 0x55;
    header[2] = 0x01;

    header[3] = (uint8_t)(frame_id & 0xFF);
    header[4] = (uint8_t)((frame_id >> 8) & 0xFF);
    header[5] = (uint8_t)((frame_id >> 16) & 0xFF);
    header[6] = (uint8_t)((frame_id >> 24) & 0xFF);

    header[7] = (uint8_t)(payload_len & 0xFF);
    header[8] = (uint8_t)((payload_len >> 8) & 0xFF);
    header[9] = (uint8_t)((payload_len >> 16) & 0xFF);
    header[10] = (uint8_t)((payload_len >> 24) & 0xFF);

    uint32_t crc32 = calculate_crc32(payload, payload_len);
    header[11] = (uint8_t)(crc32 & 0xFF);
    header[12] = (uint8_t)((crc32 >> 8) & 0xFF);
    header[13] = (uint8_t)((crc32 >> 16) & 0xFF);
    header[14] = (uint8_t)((crc32 >> 24) & 0xFF);

    uart_write_bytes(UART_NUM_0, header, sizeof(header));
    uart_write_bytes(UART_NUM_0, payload, payload_len);
}

static void get_target_score(float target_x, float target_y, uint8_t *ring, uint8_t *zone, float *distance)
{
    float dx = target_x - 25.0f;
    float dy = target_y - 25.0f;
    float r = sqrtf(dx * dx + dy * dy);
    *distance = r;

    if (r <= 10.0f) {
        *zone = 1;
    } else {
        *zone = 2;
    }

    if (r <= 0.25f)      *ring = 11;
    else if (r <= 1.5f)  *ring = 10;
    else if (r <= 4.0f)  *ring = 9;
    else if (r <= 7.0f)  *ring = 8;
    else if (r <= 10.0f) *ring = 7;
    else if (r <= 13.0f) *ring = 6;
    else if (r <= 16.0f) *ring = 5;
    else if (r <= 19.0f) *ring = 4;
    else if (r <= 22.0f) *ring = 3;
    else if (r <= 25.0f) *ring = 2;
    else if (r <= 28.0f) *ring = 1;
    else                 *ring = 0;
}

static void set_camera_exposure_target(int value)
{
    struct v4l2_ext_control ext_ctrl = {
        .id = V4L2_CID_EXPOSURE,
        .value = value,
    };
    struct v4l2_ext_controls ext_ctrls = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = 1,
        .controls = &ext_ctrl,
    };
    ioctl(g_video_fd, VIDIOC_S_EXT_CTRLS, &ext_ctrls);
}

// ============================================================================
// STANDALONE OPTIMAL-ZONE NERF DETECTOR (LALITH SCORE + ORANGE/YELLOW SPECTRUM)
// ============================================================================
static volatile int64_t g_cooldown_until_us = 0;
static volatile int64_t g_settle_until_us = 0;
static volatile bool g_need_baseline_latch = true;
static int s_baseline_warm_px = 0;

static void parse_incoming_cmd_line(const char *line)
{
    if (strstr(line, "UP") || strstr(line, "ARM") || strstr(line, "POP")) {
        // Settle 300ms after pop-up command, then latch clean baseline
        g_settle_until_us = esp_timer_get_time() + 300000LL;
        g_cooldown_until_us = 0;
        g_need_baseline_latch = true;
    } else if (strstr(line, "DOWN")) {
        g_cooldown_until_us = esp_timer_get_time() + 800000LL;
    }
}

static void uart_rx_task(void *arg)
{
    uint8_t buffer[128];
    int idx = 0;
    int64_t last_byte_time = 0;
    char cmd_u0[48], cmd_u1[48], cmd_u2[48];
    int idx_u0 = 0, idx_u1 = 0, idx_u2 = 0;

    while (1) {
        uint8_t byte;
        int64_t now = esp_timer_get_time();

        while (uart_read_bytes(BRIDGE_UART_NUM, &byte, 1, 0) > 0) {
            if (byte == '\n' || byte == '\r') {
                if (idx_u1 > 0) {
                    cmd_u1[idx_u1] = '\0';
                    parse_incoming_cmd_line(cmd_u1);
                    idx_u1 = 0;
                }
            } else if (idx_u1 < (int)sizeof(cmd_u1) - 1 && byte >= 32 && byte <= 126) {
                cmd_u1[idx_u1++] = (char)byte;
            }
        }

        while (uart_read_bytes(BRIDGE_UART_ALT_NUM, &byte, 1, 0) > 0) {
            if (byte == '\n' || byte == '\r') {
                if (idx_u2 > 0) {
                    cmd_u2[idx_u2] = '\0';
                    parse_incoming_cmd_line(cmd_u2);
                    idx_u2 = 0;
                }
            } else if (idx_u2 < (int)sizeof(cmd_u2) - 1 && byte >= 32 && byte <= 126) {
                cmd_u2[idx_u2++] = (char)byte;
            }
        }

        int len = uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            if (byte == '\n' || byte == '\r') {
                if (idx_u0 > 0) {
                    cmd_u0[idx_u0] = '\0';
                    parse_incoming_cmd_line(cmd_u0);
                    idx_u0 = 0;
                }
            } else if (idx_u0 < (int)sizeof(cmd_u0) - 1 && byte >= 32 && byte <= 126) {
                cmd_u0[idx_u0++] = (char)byte;
            }

            if (idx > 0 && (now - last_byte_time > 100000)) idx = 0;
            last_byte_time = now;

            if (idx == 0) {
                if (byte == 0xCC) buffer[idx++] = byte;
            } else if (idx == 1) {
                if (byte == 0x01) buffer[idx++] = byte;
                else idx = 0;
            } else if (idx >= 2 && idx < 38) {
                buffer[idx++] = byte;
            } else if (idx == 38) {
                buffer[idx] = byte;
                uint8_t check = 0;
                for (int i = 1; i < 38; i++) check ^= buffer[i];
                if (check == buffer[38]) {
                    memcpy(g_homography, &buffer[2], 36);
                    g_is_calibrated = true;
                }
                idx = 0;
            }
        }
    }
}

static void init_bridge_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART_NUM, 4096, 32768, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART_NUM, BRIDGE_TX_PIN, BRIDGE_RX_PIN_PRIMARY, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART_ALT_NUM, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART_ALT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART_ALT_NUM, UART_PIN_NO_CHANGE, BRIDGE_RX_PIN_ALT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void camera_stream_task(void *arg)
{
    struct v4l2_buffer buf = {0};
    uint32_t frame_count = 0;
    int64_t last_transmit_time = 0;
    const int64_t frame_interval_us = 1000000LL / STREAM_MAX_FPS;

    set_camera_exposure_target(60);

    while (true) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(g_video_fd, VIDIOC_DQBUF, &buf) == 0) {
            frame_count++;

            int64_t now_us = esp_timer_get_time();
            if (now_us - last_transmit_time < frame_interval_us) {
                ioctl(g_video_fd, VIDIOC_QBUF, &buf);
                continue;
            }
            last_transmit_time = now_us;

            uint8_t *frame_ptr = g_buffers[buf.index];
            uint32_t out_len = buf.bytesused;

            if (frame_ptr != NULL && out_len > 0) {
                uint16_t *pixels = (uint16_t *)frame_ptr;

                shot_metadata_t meta = {
                    .magic = {0xDE, 0xAD, 0xBE, 0xEF},
                    .laser_found = 0,
                    .zone = 0,
                    .score_ring = 0,
                    .is_calibrated = 1,
                    .laser_x_px = 400,
                    .laser_y_px = 400,
                    .laser_x_mm = 25.0f,
                    .laser_y_mm = 25.0f,
                    .laser_dist_mm = 0.0f
                };

                // Scan ONLY inside the Black Region (center=(400,400), radius=210px)
                // Anything outside the Black Region is a MISS.
                // Inside the Black Region, detect ONLY the Nerf Bullet Orange Tip while rejecting light bounce/glare!
                bool do_latch = (now_us >= g_settle_until_us) && g_need_baseline_latch;
                memset(s_tip_hit_grid, 0, sizeof(s_tip_hit_grid));

                for (int y = 190, gy = 0; y <= 610 && gy < GRID_DIM; y += 3, gy++) {
                    int dy = y - TARGET_CENTER_Y;
                    int dy_sq = dy * dy;
                    const uint16_t *row = &pixels[y * 800];

                    for (int x = 190, gx = 0; x <= 610 && gx < GRID_DIM; x += 3, gx++) {
                        int dx = x - TARGET_CENTER_X;
                        if (dx * dx + dy_sq > OPTIMAL_ZONE_RADIUS_SQ) {
                            if (do_latch) s_dark_target_mask[gy][gx] = 0;
                            continue;
                        }

                        uint16_t p = row[x];
                        int r5 = (p >> 11) & 0x1F;
                        int g6 = (p >> 5)  & 0x3F;
                        int b5 = p & 0x1F;
                        int g5 = g6 >> 1;

                        // Convert to 8-bit 0..255 for exact chromatic and anti-glare checks
                        int r8 = (r5 << 3) | (r5 >> 2);
                        int g8 = (g6 << 2) | (g6 >> 4);
                        int b8 = (b5 << 3) | (b5 >> 2);

                        if (do_latch) {
                            int avg_luma = (r8 + g8 + b8) / 3;
                            int max_ch = (r8 > g8) ? ((r8 > b8) ? r8 : b8) : ((g8 > b8) ? g8 : b8);
                            // Strictly mark ONLY dark black target region pixels (anything brighter or outside is MISS)
                            s_dark_target_mask[gy][gx] = (avg_luma <= 88 && max_ch <= 102) ? 1 : 0;
                            s_base_r[gy][gx] = (uint8_t)r8;
                            s_base_g[gy][gx] = (uint8_t)g8;
                            s_base_b[gy][gx] = (uint8_t)b8;
                        }

                        // Rule 1: Anything outside the Black Region is strictly a MISS!
                        if (s_dark_target_mask[gy][gx] == 0) continue;

                        // Rule 2: Delta from Black-Region Baseline (Anti-Light-Bounce)
                        int dr = r8 - (int)s_base_r[gy][gx];
                        int dg = g8 - (int)s_base_g[gy][gx];
                        int db = b8 - (int)s_base_b[gy][gx];

                        // Light bouncing off black plastic raises R, G, and B together (db > 30 or dr - dg < 20).
                        // A real Nerf Orange Tip has strong Red surge (dr >= 65), suppressed Blue (b8 <= 72, db <= 32),
                        // high saturation ((r8 - b8)/r8 >= 52%), and true Orange hue (R >> G > B).
                        int lalith_score = (r5 * 2) - g5 - b5;
                        bool anti_glare_pass = (
                            b8 <= 72 &&
                            db <= 32 &&
                            dr >= 65 &&
                            (dr - db) >= 55 &&
                            (dr - dg) >= 20 &&
                            (r8 - b8) * 100 >= r8 * 52
                        );

                        bool is_nerf_orange_tip = (
                            r8 >= 140 &&
                            g8 >= 35 && g8 <= 135 &&
                            (r8 - b8) >= 72 &&
                            (r8 - g8) >= 26 && (r8 - g8) <= 135 &&
                            (g8 - b8) >= 12 &&
                            lalith_score >= 12
                        );

                        if (anti_glare_pass && is_nerf_orange_tip) {
                            s_tip_hit_grid[gy][gx] = 1;
                        }
                    }
                }

                // Rule 3: Spatial Contiguity Filter (Nerf Orange Tip forms a compact cluster, NOT scattered glare)
                int warm_nerf_px = 0;
                int64_t sum_x = 0;
                int64_t sum_y = 0;
                int64_t sum_w = 0;

                for (int gy = 1; gy < GRID_DIM - 1; gy++) {
                    for (int gx = 1; gx < GRID_DIM - 1; gx++) {
                        if (!s_tip_hit_grid[gy][gx]) continue;
                        int neighbors =
                            s_tip_hit_grid[gy - 1][gx - 1] + s_tip_hit_grid[gy - 1][gx] + s_tip_hit_grid[gy - 1][gx + 1] +
                            s_tip_hit_grid[gy][gx - 1]     +                              s_tip_hit_grid[gy][gx + 1] +
                            s_tip_hit_grid[gy + 1][gx - 1] + s_tip_hit_grid[gy + 1][gx] + s_tip_hit_grid[gy + 1][gx + 1];
                        // Require at least 2 adjacent orange-tip neighbors in 3x3 grid to eliminate isolated glare specks
                        if (neighbors >= 2) {
                            warm_nerf_px++;
                            int px = 190 + gx * 3;
                            int py = 190 + gy * 3;
                            int weight = neighbors + 1;
                            sum_x += (int64_t)px * weight;
                            sum_y += (int64_t)py * weight;
                            sum_w += weight;
                        }
                    }
                }

                if (now_us >= g_settle_until_us) {
                    if (g_need_baseline_latch) {
                        s_baseline_warm_px = (warm_nerf_px < 20) ? warm_nerf_px : 0;
                        g_need_baseline_latch = false;
                    }

                    if (now_us >= g_cooldown_until_us) {
                        int net_nerf_px = warm_nerf_px - s_baseline_warm_px;
                        if (net_nerf_px < 0) {
                            s_baseline_warm_px = warm_nerf_px;
                            net_nerf_px = 0;
                        }

                        // Trigger HIT when a solid Nerf Orange Tip cluster (>= 10 core cells = ~90 full pixels) lands inside Black Region!
                        if (net_nerf_px >= 10 && sum_w > 0) {
                            int hit_x = (int)(sum_x / sum_w);
                            int hit_y = (int)(sum_y / sum_w);
                            if (hit_x == 400 && hit_y == 400) hit_x = 401;

                            int dx = hit_x - TARGET_CENTER_X;
                            int dy = hit_y - TARGET_CENTER_Y;
                            float mm_x = 25.0f + ((float)dx / (float)OPTIMAL_ZONE_RADIUS_PX) * 25.0f;
                            float mm_y = 25.0f + ((float)dy / (float)OPTIMAL_ZONE_RADIUS_PX) * 25.0f;
                            uint8_t ring = 0, zone = 0;
                            float dist_mm = 0.0f;
                            get_target_score(mm_x, mm_y, &ring, &zone, &dist_mm);

                            meta.laser_found = 1;
                            meta.zone = zone;
                            meta.score_ring = ring;
                            meta.laser_x_px = (uint16_t)hit_x;
                            meta.laser_y_px = (uint16_t)hit_y;
                            meta.laser_x_mm = mm_x;
                            meta.laser_y_mm = mm_y;
                            meta.laser_dist_mm = dist_mm;

                            int reported_pixels = net_nerf_px * 9;
                            if (reported_pixels < 75) reported_pixels = 75;

                            // Dispatch HIT + DOWN over Bridge UART1 (GPIO 21 -> Pop ESP32 GPIO 16)
                            char bridge_msg[96];
                            int msg_len = snprintf(bridge_msg, sizeof(bridge_msg),
                                                   "HIT,1,%d,%d,%d\nDOWN,1\n",
                                                   hit_x, hit_y, reported_pixels);
                            uart_write_bytes(BRIDGE_UART_NUM, bridge_msg, msg_len);

                            // 1.2s cooldown after hit so servo drops cleanly once
                            g_cooldown_until_us = now_us + 1200000LL;
                        }
                    }
                }

                jpeg_encode_cfg_t enc_config = {
                    .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                    .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
                    .image_quality = 35,
                    .width = 800,
                    .height = 800,
                };

                uint32_t jpeg_encoded_size = 0;
                esp_err_t ret = jpeg_encoder_process(g_jpeg_handle, &enc_config,
                                                     frame_ptr, out_len,
                                                     g_jpeg_out_buf, g_jpeg_out_size, &jpeg_encoded_size);
                if (ret == ESP_OK && jpeg_encoded_size > 0) {
                    memcpy(g_jpeg_out_buf + jpeg_encoded_size, &meta, sizeof(meta));
                    send_packet(frame_count, g_jpeg_out_buf, jpeg_encoded_size + sizeof(meta));
                }
            }

            ioctl(g_video_fd, VIDIOC_QBUF, &buf);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = 3000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 65536, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_cfg));
    uart_vfs_dev_use_driver(0);
    uart_vfs_dev_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_LF);

    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL, 1);

    init_bridge_uart();

    esp_video_init_csi_config_t csi_config = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = I2C_MASTER_NUM,
                .scl_pin = I2C_MASTER_SCL_IO,
                .sda_pin = I2C_MASTER_SDA_IO,
            },
            .freq = 100000,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
    };
    esp_video_init_config_t video_cfg = {
        .csi = &csi_config,
    };
    ESP_ERROR_CHECK(esp_video_init(&video_cfg));

    g_video_fd = open("/dev/video0", O_RDWR);
    if (g_video_fd < 0) return;

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = {
            .width = 800,
            .height = 800,
            .pixelformat = V4L2_PIX_FMT_RGB565,
        }
    };
    ioctl(g_video_fd, VIDIOC_S_FMT, &fmt);

    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 5000,
    };
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&encode_eng_cfg, &g_jpeg_handle));

    g_jpeg_out_size = 800 * 800;
    g_jpeg_out_buf = heap_caps_aligned_alloc(64, g_jpeg_out_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    struct v4l2_requestbuffers req = {
        .count = 2,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    ioctl(g_video_fd, VIDIOC_REQBUFS, &req);

    for (int i = 0; i < (int)req.count; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i
        };
        ioctl(g_video_fd, VIDIOC_QUERYBUF, &buf);
        g_buffers[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, g_video_fd, buf.m.offset);
        g_buf_len[i] = buf.length;
        ioctl(g_video_fd, VIDIOC_QBUF, &buf);
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(g_video_fd, VIDIOC_STREAMON, &type);

    xTaskCreatePinnedToCore(camera_stream_task, "camera_stream_task", 12288, NULL, 6, NULL, 0);
}
