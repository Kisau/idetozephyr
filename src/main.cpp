#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <stm32h7xx_hal.h>
#include <zephyr/drivers/uart.h>

// ==========================================
// 🗜️ 引入 JPEG 壓縮庫與記憶體寫入器（僅對此區塊套用 O3 最佳化）
// ==========================================
#pragma GCC push_options
#pragma GCC optimize ("O3", "unroll-loops", "tree-vectorize")
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC pop_options

extern "C" {
    #include "network.h"
    #include "network_data.h"
    #include "main.h" 
    #include "app_postprocess.h" 
}
}
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#pragma pack(push, 1)
struct frame_header {
    uint16_t magic_word;
    uint32_t image_size;   // JPEG payload 的 byte 數（不含 header/boxes）
    uint16_t box_count;    // 緊接在 header 後面的 bbox_data 筆數（0~10，非固定 10 筆）
};

struct bbox_data {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    float    conf;
    uint8_t  label_char;
};
#pragma pack(pop)

static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart8));
#define UART_BAUDRATE 460800U

static const struct device *video_dev;
static AppConfig_TypeDef App_Config;

#define MAX_DETECTIONS 10 
static postprocess_outBuffer_t detections[MAX_DETECTIONS];

#define MAX_FRAME_SIZE  (320 * 240 * 2) 
#define BUFFER_COUNT    2               
#define PROC_BUF_COUNT  2               // AI/JPEG 處理管線緩衝區數量（my_camera_data 用）

// ==========================================
// 🚀 UART 傳輸與影像緩衝區（對齊 32-byte Cache Line）
// ==========================================
#define UART_TX_BUF_CAP 20480
static uint8_t uart_tx_buffer[UART_TX_BUF_CAP] __attribute__((aligned(32), section("SDRAM2")));
#define HEADER_MAX_BLOCK_SIZE (sizeof(struct frame_header) + 10 * sizeof(struct bbox_data))
#define MAX_JPEG_SIZE (UART_TX_BUF_CAP - HEADER_MAX_BLOCK_SIZE)

static uint8_t compressed_jpeg_buffer[30000] __attribute__((aligned(32), section("SDRAM2")));
static uint32_t jpeg_current_size = 0;
static bool jpeg_overflow = false;

static uint8_t raw_video_data[BUFFER_COUNT][MAX_FRAME_SIZE] __attribute__((aligned(32), section("SDRAM2")));
static struct video_buffer video_buf_metadata[BUFFER_COUNT];
static struct video_format cam_fmt;

static uint8_t my_camera_data[PROC_BUF_COUNT][STAI_NETWORK_IN_1_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 
static uint8_t network_context[STAI_NETWORK_CONTEXT_SIZE] __attribute__((aligned(32), section("SDRAM2")));
static stai_network* network = (stai_network*)network_context;
static uint8_t my_output_0[STAI_NETWORK_OUT_1_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 
static uint8_t my_output_1[STAI_NETWORK_OUT_2_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 
static uint8_t my_output_2[STAI_NETWORK_OUT_3_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 

static uint8_t my_activations[STAI_NETWORK_ACTIVATIONS_SIZE_BYTES] __attribute__((aligned(32)));

static uint8_t* in_buffers[1]; 
static uint8_t* act_buffers[1] = { my_activations };
static uint8_t* out_buffers[3] = { my_output_0, my_output_1, my_output_2 };

struct frame_msg {
    int buf_idx;
    uint32_t prep_time; 
};
K_MSGQ_DEFINE(frame_msgq, sizeof(struct frame_msg), 2, 4);

static struct k_sem buf_free_sem[PROC_BUF_COUNT];

#define CAM_THREAD_STACK_SIZE 4096
K_THREAD_STACK_DEFINE(cam_thread_stack, CAM_THREAD_STACK_SIZE);
struct k_thread cam_thread_data;

// ==========================================
// 📡 中斷驅動 UART 傳送
// ==========================================
static const uint8_t *uart_tx_ptr;
static volatile size_t uart_tx_remaining;
static struct k_sem uart_tx_done_sem;

static void uart_isr(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    if (!uart_irq_update(dev)) return;

    if (uart_irq_tx_ready(dev)) {
        if (uart_tx_remaining > 0) {
            int n = uart_fifo_fill(dev, uart_tx_ptr, uart_tx_remaining);
            if (n > 0) {
                uart_tx_ptr += n;
                uart_tx_remaining -= n;
            }
        }
        if (uart_tx_remaining == 0) {
            uart_irq_tx_disable(dev);
            k_sem_give(&uart_tx_done_sem);
        }
    }
}

static void uart_send_blocking(const uint8_t *data, size_t len) {
    uart_tx_ptr = data;
    uart_tx_remaining = len;
    uart_irq_tx_enable(uart_dev);
    k_sem_take(&uart_tx_done_sem, K_FOREVER);
}

static void Enable_SDRAM_Cache(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};
    HAL_MPU_Disable(); 
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0xD0000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE; 
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER15; 
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT); 
    SCB_EnableICache();
    SCB_EnableDCache();
}

int ai_init(void) {
    if (stai_network_init(network) != 0) return -1;
    if (stai_network_set_activations(network, act_buffers, 1) != 0) return -1;
    if (stai_network_set_outputs(network, out_buffers, 3) != 0) return -1;
    return 0;
}

static int setup_camera(void) {
    struct video_format req_fmt;
    req_fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
    req_fmt.width = 320;
    req_fmt.height = 240;
    req_fmt.pitch = 320 * 2;

    if (video_set_format(video_dev, &req_fmt) < 0) {
        video_get_format(video_dev, &cam_fmt);
    } else {
        cam_fmt = req_fmt;
    }

    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct video_buffer *vb = &video_buf_metadata[i];
        vb->buffer = raw_video_data[i];
        vb->size = cam_fmt.size;
        vb->type = VIDEO_BUF_TYPE_OUTPUT; 
        video_enqueue(video_dev, vb);
    }
    return 0;
}

void my_jpeg_write_func(void *context, void *data, int size) {
    ARG_UNUSED(context);
    if (jpeg_current_size + (uint32_t)size <= sizeof(compressed_jpeg_buffer)) {
        memcpy(compressed_jpeg_buffer + jpeg_current_size, data, size);
        jpeg_current_size += (uint32_t)size;
    } else {
        jpeg_overflow = true;
    }
}

#pragma GCC push_options
#pragma GCC optimize ("O3", "unroll-loops")
static int process_camera_frame(const uint8_t *rgb565_buf, uint32_t src_w, uint32_t src_h, int buf_idx) {
    if (src_w < 192 || src_h < 192) return -1;

    uint32_t crop_x = (src_w - 192) / 2;
    uint32_t crop_y = (src_h - 192) / 2;

    uint8_t *ai_ptr = my_camera_data[buf_idx]; 

    for (uint32_t y = 0; y < 192; y++) {
        uint32_t src_y = y + crop_y;
        const uint16_t *row_ptr_16 = (const uint16_t *)(rgb565_buf + ((src_y * src_w) + crop_x) * 2);

        for (uint32_t x = 0; x < 192; x++) {
            uint16_t pixel = *row_ptr_16++;
            
            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5)  & 0x3F;
            uint8_t b =  pixel        & 0x1F;

            *ai_ptr++ = (r << 3) | (r >> 2); 
            *ai_ptr++ = (g << 2) | (g >> 4); 
            *ai_ptr++ = (b << 3) | (b >> 2); 
        }
    }
    return 0;
}
#pragma GCC pop_options

void camera_thread_entry(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    int current_buf_idx = 0; 

    while (1) {
        struct video_buffer *vbuf;
        if (video_dequeue(video_dev, &vbuf, K_MSEC(2000)) < 0) continue;

        if (k_sem_take(&buf_free_sem[current_buf_idx], K_NO_WAIT) != 0) {
            video_enqueue(video_dev, vbuf);
            current_buf_idx = (current_buf_idx + 1) % PROC_BUF_COUNT;
            continue;
        }

        // 🔧 關鍵修正：DCMI DMA 直接寫入 SDRAM，CPU 讀取前必須先 Invalidate D-Cache，避免讀到舊快取
        SCB_InvalidateDCache_by_Addr((uint32_t *)vbuf->buffer, cam_fmt.size);

        uint32_t t_start = k_uptime_get_32();
        bool queued = false;

        if (process_camera_frame((const uint8_t *)vbuf->buffer, cam_fmt.width, cam_fmt.height, current_buf_idx) == 0) {
            uint32_t t_prep = k_uptime_get_32() - t_start;
            struct frame_msg msg = { .buf_idx = current_buf_idx, .prep_time = t_prep };

            if (k_msgq_put(&frame_msgq, &msg, K_NO_WAIT) == 0) {
                queued = true;
            } else {
                LOG_WRN("frame_msgq full, dropping frame idx=%d", current_buf_idx);
            }
        }

        if (!queued) {
            k_sem_give(&buf_free_sem[current_buf_idx]);
        }

        current_buf_idx = (current_buf_idx + 1) % PROC_BUF_COUNT;
        video_enqueue(video_dev, vbuf);
    }
}

int main(void) {
    Enable_SDRAM_Cache();
    k_msleep(1000);
    LOG_INF("Starting STEdgeAI + Zephyr Multi-Thread (UART Streaming Mode)...");

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART not ready!");
        return 0;
    }

    struct uart_config ucfg = {
        .baudrate = UART_BAUDRATE,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    if (uart_configure(uart_dev, &ucfg) != 0) {
        LOG_ERR("UART configure failed!");
        return 0;
    }

    k_sem_init(&uart_tx_done_sem, 0, 1);
    uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
    uart_irq_rx_disable(uart_dev);
    uart_irq_tx_disable(uart_dev);

    if (ai_init() != 0) return 0;
    App_Config.output.pOutBuff = detections;
    App_Config.output.nb_detect = 0;
    app_postprocess_init(&App_Config);

    for (int i = 0; i < PROC_BUF_COUNT; i++) {
        k_sem_init(&buf_free_sem[i], 1, 1);
    }

    video_dev = DEVICE_DT_GET_ANY(st_stm32_dcmi); 
    if (!video_dev) video_dev = DEVICE_DT_GET(DT_NODELABEL(ov5640)); 
    if (!device_is_ready(video_dev) || setup_camera() < 0) return 0;

    if (video_stream_start(video_dev, VIDEO_BUF_TYPE_OUTPUT) < 0) return 0;

    k_thread_create(&cam_thread_data, cam_thread_stack,
                    K_THREAD_STACK_SIZEOF(cam_thread_stack),
                    camera_thread_entry,
                    NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);

    LOG_INF("System Ready. Pipelined Inference & UART Streaming Started.");
    uint32_t frame_count = 0;

    while (1) {
        struct frame_msg msg;
        k_msgq_get(&frame_msgq, &msg, K_FOREVER);
        frame_count++;

        int buf_idx = msg.buf_idx;
        in_buffers[0] = my_camera_data[buf_idx];
        stai_network_set_inputs(network, in_buffers, 1);

        uint32_t t_start = k_uptime_get_32();
        stai_return_code err = stai_network_run(network, (stai_run_mode)0);
        uint32_t t_ai = k_uptime_get_32() - t_start;

        if (err == 0) {
            app_postprocess_run((void**)out_buffers, &App_Config.output, &App_Config.input_static_param);

            struct frame_header hdr;
            hdr.magic_word = 0xAABB;

            struct bbox_data bboxes[10];
            uint16_t valid_boxes = 0;

            if (App_Config.output.nb_detect > 0) {
                for (int i = 0; i < App_Config.output.nb_detect; i++) {
                    float w = App_Config.output.pOutBuff[i].width;
                    float h = App_Config.output.pOutBuff[i].height;
                    float cx = App_Config.output.pOutBuff[i].x_center;
                    float cy = App_Config.output.pOutBuff[i].y_center;
                    float conf = App_Config.output.pOutBuff[i].conf;
                    int class_idx = App_Config.output.pOutBuff[i].class_index;

                    if (class_idx == 0 || conf < 0.60f) continue;

                    if (w <= 1.0f && h <= 1.0f) { cx *= 192.0f; cy *= 192.0f; w *= 192.0f; h *= 192.0f; }
                    int16_t bx = (int16_t)(cx - w / 2.0f);
                    int16_t by = (int16_t)(cy - h / 2.0f);
                    
                    if (valid_boxes < 10) {
                        bboxes[valid_boxes].x = (uint16_t)(bx > 0 ? bx : 0);
                        bboxes[valid_boxes].y = (uint16_t)(by > 0 ? by : 0);
                        bboxes[valid_boxes].w = (uint16_t)w;
                        bboxes[valid_boxes].h = (uint16_t)h;
                        bboxes[valid_boxes].conf = conf;
                        bboxes[valid_boxes].label_char = (class_idx == 1) ? 'C' : '?';
                        valid_boxes++;
                    }
                }
            }

            hdr.box_count = valid_boxes;

            // 🔧 關鍵修正：移除重複壓縮，並在若壓縮大小超過上限時自動降品質重壓，絕不截斷 JPEG
            uint32_t t_compress_start = k_uptime_get_32();
            jpeg_current_size = 0;
            jpeg_overflow = false;
            stbi_write_jpg_to_func(my_jpeg_write_func, NULL, 192, 192, 3, my_camera_data[buf_idx], 70);

            if (jpeg_overflow || jpeg_current_size > MAX_JPEG_SIZE) {
                LOG_WRN("JPEG size (%u) exceeds limit (%u), re-encoding at quality 50...",
                        (unsigned)jpeg_current_size, (unsigned)MAX_JPEG_SIZE);
                jpeg_current_size = 0;
                jpeg_overflow = false;
                stbi_write_jpg_to_func(my_jpeg_write_func, NULL, 192, 192, 3, my_camera_data[buf_idx], 50);
            }

            // 壓縮徹底完成後，才釋放緩衝區給 camera_thread
            k_sem_give(&buf_free_sem[buf_idx]);

            uint32_t t_compress = k_uptime_get_32() - t_compress_start;

            if (jpeg_overflow || jpeg_current_size < 4 || jpeg_current_size > MAX_JPEG_SIZE) {
                LOG_ERR("JPEG size invalid (%u), dropping frame %u", (unsigned)jpeg_current_size, frame_count);
                continue;
            }

            LOG_INF(
                "H7 JPEG size=%u | HEAD=%02X %02X %02X %02X | "
                "TAIL=%02X %02X %02X %02X",
                (unsigned)jpeg_current_size,
                compressed_jpeg_buffer[0],
                compressed_jpeg_buffer[1],
                compressed_jpeg_buffer[2],
                compressed_jpeg_buffer[3],
                compressed_jpeg_buffer[jpeg_current_size - 4],
                compressed_jpeg_buffer[jpeg_current_size - 3],
                compressed_jpeg_buffer[jpeg_current_size - 2],
                compressed_jpeg_buffer[jpeg_current_size - 1]
            );

            hdr.image_size = jpeg_current_size;

            // ==========================================
            // 📡 打包並透過 UART 送出
            // ==========================================
            uint32_t t_uart_start = k_uptime_get_32();

            size_t boxes_bytes = (size_t)valid_boxes * sizeof(struct bbox_data);
            size_t payload_offset = sizeof(hdr) + boxes_bytes;
            size_t total_len = payload_offset + hdr.image_size;

            memcpy(uart_tx_buffer, &hdr, sizeof(hdr));
            if (valid_boxes > 0) {
                memcpy(uart_tx_buffer + sizeof(hdr), bboxes, boxes_bytes);
            }
            memcpy(uart_tx_buffer + payload_offset, compressed_jpeg_buffer, hdr.image_size);

            uart_send_blocking(uart_tx_buffer, total_len);

            uint32_t t_uart = k_uptime_get_32() - t_uart_start;
            if (frame_count % 30 == 0) {
                LOG_INF(" Frame %u -> AI: %u ms | Compress: %u ms | UART TX: %u ms | Bytes: %u | Boxes: %u", 
                        frame_count, t_ai, t_compress, t_uart, (unsigned)total_len, valid_boxes);
            }
                } else {
            k_sem_give(&buf_free_sem[buf_idx]);
            LOG_WRN("[Error] AI inference failed! Code: 0x%08X", (unsigned int)err);
        }
    }
    return 0;
}