#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <stm32h7xx_hal.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

extern "C" {
    #include "network.h"
    #include "network_data.h"
    #include "main.h" 
    #include "app_postprocess.h" 
} 
}
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#pragma pack(push, 1)
struct spi_header {
    uint16_t magic_word;
    uint32_t image_size;
    uint16_t box_count;
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

static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi5));
static struct spi_config spi_cfg;

// 🚨 修正：將 CS 當作獨立的 GPIO 控制，不要依賴 DeviceTree 裡的 cs-gpios
static const struct gpio_dt_spec manual_cs = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpiok)),
    .pin = 1,
    .dt_flags = GPIO_ACTIVE_HIGH  // 設定為 Active High，這樣 set(0) 就是 0V，set(1) 就是 3.3V
};

static const struct device *video_dev;
static AppConfig_TypeDef App_Config;

#define MAX_DETECTIONS 10 
static postprocess_outBuffer_t detections[MAX_DETECTIONS];

#define MAX_FRAME_SIZE  (320 * 240 * 2) 
#define BUFFER_COUNT    2              

// 🚀 DMA 專用無快取跳板緩衝區 (Bounce Buffer)
#define SPI_CHUNK_SIZE 32000
static uint8_t spi_bounce_buffer[SPI_CHUNK_SIZE] __attribute__((section(".nocache")));

static uint8_t spi_img_buffer[2][192 * 192 * 2] __attribute__((section("SDRAM2")));
static uint8_t raw_video_data[BUFFER_COUNT][MAX_FRAME_SIZE] __attribute__((section("SDRAM2")));
static struct video_buffer video_buf_metadata[BUFFER_COUNT];
static struct video_format cam_fmt;

static uint8_t my_camera_data[2][STAI_NETWORK_IN_1_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 
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

#define CAM_THREAD_STACK_SIZE 4096
K_THREAD_STACK_DEFINE(cam_thread_stack, CAM_THREAD_STACK_SIZE);
struct k_thread cam_thread_data;

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

#pragma GCC push_options
#pragma GCC optimize ("O3", "unroll-loops")
static int process_camera_frame(const uint8_t *rgb565_buf, uint32_t src_w, uint32_t src_h, int buf_idx) {
    if (src_w < 192 || src_h < 192) return -1;

    uint32_t crop_x = (src_w - 192) / 2;
    uint32_t crop_y = (src_h - 192) / 2;

    uint8_t *ai_ptr = my_camera_data[buf_idx]; 
    uint16_t *spi_ptr_16 = (uint16_t *)spi_img_buffer[buf_idx]; 

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
            
            *spi_ptr_16++ = pixel;
        }
    }
    return 0;
}
#pragma GCC pop_options

void camera_thread_entry(void *p1, void *p2, void *p3) {
    int current_buf_idx = 0; 

    while (1) {
        struct video_buffer *vbuf;
        if (video_dequeue(video_dev, &vbuf, K_MSEC(2000)) < 0) continue;

        uint32_t t_start = k_uptime_get_32();
        
        if (process_camera_frame(vbuf->buffer, cam_fmt.width, cam_fmt.height, current_buf_idx) == 0) {
            uint32_t t_prep = k_uptime_get_32() - t_start;
            struct frame_msg msg = { .buf_idx = current_buf_idx, .prep_time = t_prep };
            
            while (k_msgq_put(&frame_msgq, &msg, K_NO_WAIT) != 0) {
                k_msgq_purge(&frame_msgq);
            }
            current_buf_idx = (current_buf_idx + 1) % 2;
        }
        video_enqueue(video_dev, vbuf);
    }
}

int main(void) {
    Enable_SDRAM_Cache();
    k_msleep(1000);
    LOG_INF("Starting STEdgeAI + Zephyr Multi-Thread (SPI Output Mode)...");

    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI not ready!");
        return 0;
    }

    // 🚨 修正：手動初始化 CS 腳位 (假設是 PK1)，並預設拉高 (Inactive)
    if (gpio_is_ready_dt(&manual_cs)) {
        gpio_pin_configure_dt(&manual_cs, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&manual_cs, 1); // 實體電壓 3.3V
    } else {
        LOG_ERR("CS GPIO not ready!");
    }

    // 🚨 修正：取消 Zephyr 對 CS 的控制 (.cs = NULL)
  
    spi_cfg.frequency = 4000000U; // 確保與 L4 完全一致的 15MHz
    spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_TRANSFER_MSB;

    if (ai_init() != 0) return 0;
    App_Config.output.pOutBuff = detections;
    App_Config.output.nb_detect = 0;
    app_postprocess_init(&App_Config);

    video_dev = DEVICE_DT_GET_ANY(st_stm32_dcmi); 
    if (!video_dev) video_dev = DEVICE_DT_GET(DT_NODELABEL(ov5640)); 
    if (!device_is_ready(video_dev) || setup_camera() < 0) return 0;

    if (video_stream_start(video_dev, VIDEO_BUF_TYPE_OUTPUT) < 0) return 0;

    k_thread_create(&cam_thread_data, cam_thread_stack,
                    K_THREAD_STACK_SIZEOF(cam_thread_stack),
                    camera_thread_entry,
                    NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);

    LOG_INF("System Ready. Pipelined Inference & SPI TX Started.");
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

            struct spi_header hdr;
            hdr.magic_word = 0xAABB;
            hdr.image_size = 192 * 192 * 2; 

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

            // ==========================================
            // 📡 執行 SPI 分批傳輸 (手動 CS + 跳板策略)
            // ==========================================
            uint32_t t_spi_start = k_uptime_get_32();



            // 1. 手動拉低 CS 腳位
            gpio_pin_set_dt(&manual_cs, 0); 
            k_msleep(5); // 給 L4 充足的時間醒來準備

            // --- 階段 A：傳送固定 138 Bytes 的標頭區塊 ---
            #define HEADER_BLOCK_SIZE (sizeof(struct spi_header) + 10 * sizeof(struct bbox_data))
            uint8_t header_block[HEADER_BLOCK_SIZE] = {0}; // 建立在 Stack 的安全陣列
            
            memcpy(header_block, &hdr, sizeof(hdr));
            if (valid_boxes > 0) {
                memcpy(header_block + sizeof(hdr), bboxes, valid_boxes * sizeof(struct bbox_data));
            }

            struct spi_buf tx_hdr_buf = { .buf = header_block, .len = HEADER_BLOCK_SIZE };
            struct spi_buf_set tx_hdr_set = { .buffers = &tx_hdr_buf, .count = 1 };
            spi_write(spi_dev, &spi_cfg, &tx_hdr_set);

            k_usleep(1000); // 稍微等待 L4 消化標頭

            // --- 階段 B：直接傳送 73KB 影像 (讓 Zephyr 驅動自己慢慢送) ---
            struct spi_buf tx_img_buf = { .buf = spi_img_buffer[buf_idx], .len = hdr.image_size };
            struct spi_buf_set tx_img_set = { .buffers = &tx_img_buf, .count = 1 };
            spi_write(spi_dev, &spi_cfg, &tx_img_set);

            k_usleep(500);

            // 傳輸結束，手動拉高 CS 腳位
            gpio_pin_set_dt(&manual_cs, 1); 

            uint32_t t_spi = k_uptime_get_32() - t_spi_start;

            LOG_INF("⏱️ Frame %d -> Prep: %u ms | AI: %u ms | SPI: %u ms | Boxes: %u", 
                    frame_count, msg.prep_time, t_ai, t_spi, valid_boxes);
                    
        } else {
            LOG_WRN("[Error] AI inference failed! Code: 0x%08X", (unsigned int)err);
        }
    }
    return 0;
}