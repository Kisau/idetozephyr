#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <stm32h7xx_hal.h>
#include <zephyr/drivers/display.h> 

extern "C" {
    #include "network.h"
    #include "network_data.h"
    #include "main.h" 
    #include "app_postprocess.h" 
} 
}
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// ========================================
// 系統設備與全域變數
// ========================================
static const struct device *video_dev;
static const struct device *display_dev; 
static AppConfig_TypeDef App_Config;

#define MAX_DETECTIONS 10 
static postprocess_outBuffer_t detections[MAX_DETECTIONS];

// ==========================================
// 🚨 雙重緩衝區 (Ping-Pong Buffer) 配置
// ==========================================
#define MAX_FRAME_SIZE  (320 * 240 * 2) 
#define BUFFER_COUNT    2              

static uint8_t raw_video_data[BUFFER_COUNT][MAX_FRAME_SIZE] __attribute__((section("SDRAM2")));
static struct video_buffer video_buf_metadata[BUFFER_COUNT];
static struct video_format cam_fmt;

// ✅ 宣告「兩組」陣列輪流使用，防止 Thread 1 寫入時覆蓋到 Thread 2 正在用的資料
static uint8_t lcd_buffer[2][192 * 192 * 2] __attribute__((section("SDRAM2")));
static uint8_t my_camera_data[2][STAI_NETWORK_IN_1_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 
static uint8_t network_context[STAI_NETWORK_CONTEXT_SIZE] __attribute__((aligned(32), section("SDRAM2")));
static stai_network* network = (stai_network*)network_context;
static uint8_t my_output_0[STAI_NETWORK_OUT_1_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 
static uint8_t my_output_1[STAI_NETWORK_OUT_2_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 
static uint8_t my_output_2[STAI_NETWORK_OUT_3_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 

static uint8_t my_activations[STAI_NETWORK_ACTIVATIONS_SIZE_BYTES] __attribute__((aligned(32)));

static uint8_t* in_buffers[1]; // 將在執行緒中動態切換
static uint8_t* act_buffers[1] = { my_activations };
static uint8_t* out_buffers[3] = { my_output_0, my_output_1, my_output_2 };

// ==========================================
// 🚨 訊息佇列與執行緒定義 (IPC)
// ==========================================
// 這個結構用來在兩個執行緒之間傳遞「哪一組 Buffer 準備好了」
struct frame_msg {
    int buf_idx;
    uint32_t prep_time; // 順便把前處理的測速時間傳遞給主執行緒印出
};
K_MSGQ_DEFINE(frame_msgq, sizeof(struct frame_msg), 2, 4);

// 定義相機執行緒的記憶體與資料結構
#define CAM_THREAD_STACK_SIZE 4096
K_THREAD_STACK_DEFINE(cam_thread_stack, CAM_THREAD_STACK_SIZE);
struct k_thread cam_thread_data;

// ==========================================
// 繪圖與設備初始化
// ==========================================
static void draw_box_rgb565(uint8_t *img_buf, uint32_t img_w, uint32_t img_h, 
                            int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img_w || y >= img_h) return;
    if (x + w > img_w) w = img_w - x;
    if (y + h > img_h) h = img_h - y;

    uint8_t color_h = (color >> 8) & 0xFF;
    uint8_t color_l = color & 0xFF;

    for (uint16_t i = 0; i < w; i++) {
        size_t top_idx = ((y * img_w) + (x + i)) * 2;
        size_t bot_idx = (((y + h - 1) * img_w) + (x + i)) * 2;
        img_buf[top_idx] = color_h; img_buf[top_idx + 1] = color_l;
        img_buf[bot_idx] = color_h; img_buf[bot_idx + 1] = color_l;
    }
    for (uint16_t j = 0; j < h; j++) {
        size_t left_idx = (((y + j) * img_w) + x) * 2;
        size_t right_idx = (((y + j) * img_w) + (x + w - 1)) * 2;
        img_buf[left_idx] = color_h; img_buf[left_idx + 1] = color_l;
        img_buf[right_idx] = color_h; img_buf[right_idx + 1] = color_l;
    }
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

static int setup_display(void) {
    display_dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) return -ENODEV;
    display_set_pixel_format(display_dev, PIXEL_FORMAT_RGB_565);
    display_blanking_off(display_dev);
    return 0;
}

int ai_init(void) {
    if (stai_network_init(network) != 0) return -1;
    if (stai_network_set_activations(network, act_buffers, 1) != 0) return -1;
    if (stai_network_set_outputs(network, out_buffers, 3) != 0) return -1;
    // 注意：Input 現在改由迴圈內動態設定
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
// ✅ 新增 buf_idx 參數，決定要把結果寫到哪一個 Buffer
static int process_camera_frame(const uint8_t *rgb565_buf, uint32_t src_w, uint32_t src_h, int buf_idx) {
    if (src_w < 192 || src_h < 192) return -1;

    uint32_t crop_x = (src_w - 192) / 2;
    uint32_t crop_y = (src_h - 192) / 2;

    uint8_t *ai_ptr = my_camera_data[buf_idx]; 
    uint16_t *lcd_ptr_16 = (uint16_t *)lcd_buffer[buf_idx]; 

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

            *lcd_ptr_16++ = pixel;
        }
    }
    return 0;
}
#pragma GCC pop_options

// ==========================================
// 📸 Thread 1: Camera 處理執行緒
// ==========================================
void camera_thread_entry(void *p1, void *p2, void *p3) {
    int current_buf_idx = 0; // 從 0 號緩衝區開始

    while (1) {
        struct video_buffer *vbuf;
        if (video_dequeue(video_dev, &vbuf, K_MSEC(2000)) < 0) continue;

        uint32_t t_start = k_uptime_get_32();
        
        // 影像處理：寫入指定的 current_buf_idx
        if (process_camera_frame(vbuf->buffer, cam_fmt.width, cam_fmt.height, current_buf_idx) == 0) {
            uint32_t t_prep = k_uptime_get_32() - t_start;

            // 打包訊息
            struct frame_msg msg = { .buf_idx = current_buf_idx, .prep_time = t_prep };
            
            // 寄送訊息給 AI 執行緒。如果 AI 算太慢導致佇列塞車，就清空舊的 (Drop Frame機制)
            while (k_msgq_put(&frame_msgq, &msg, K_NO_WAIT) != 0) {
                k_msgq_purge(&frame_msgq);
            }

            // 切換 Ping-Pong Buffer (0變1，1變0)
            current_buf_idx = (current_buf_idx + 1) % 2;
        }
        
        video_enqueue(video_dev, vbuf);
    }
}

// ==========================================
// 🧠 Thread 2: 主程式 (AI 推論與顯示)
// ==========================================
int main(void) {
    Enable_SDRAM_Cache();
    k_msleep(1000);
    LOG_INF("Starting STEdgeAI + Zephyr Multi-Thread Camera...");

    if (ai_init() != 0) return 0;
    App_Config.output.pOutBuff = detections;
    App_Config.output.nb_detect = 0;
    app_postprocess_init(&App_Config);

    video_dev = DEVICE_DT_GET_ANY(st_stm32_dcmi); 
    if (!video_dev) video_dev = DEVICE_DT_GET(DT_NODELABEL(ov5640)); 
    if (!device_is_ready(video_dev) || setup_camera() < 0) return 0;

    setup_display();

    if (video_stream_start(video_dev, VIDEO_BUF_TYPE_OUTPUT) < 0) return 0;

    // 🚀 啟動 Camera 獨立執行緒 (設定優先權為 5)
    k_thread_create(&cam_thread_data, cam_thread_stack,
                    K_THREAD_STACK_SIZEOF(cam_thread_stack),
                    camera_thread_entry,
                    NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);

    LOG_INF("System Ready. Pipelined Inference Started.");
    uint32_t frame_count = 0;

    while (1) {
        struct frame_msg msg;
        
        // 等待 Camera Thread 寄信來 (這會讓出 CPU，不浪費資源)
        k_msgq_get(&frame_msgq, &msg, K_FOREVER);
        frame_count++;

        int buf_idx = msg.buf_idx;

        // 🚨 動態切換 AI 模型輸入指標到準備好的那個 Buffer
        in_buffers[0] = my_camera_data[buf_idx];
        stai_network_set_inputs(network, in_buffers, 1);

        uint32_t t_start = k_uptime_get_32();
        stai_return_code err = stai_network_run(network, (stai_run_mode)0);
        uint32_t t_ai = k_uptime_get_32() - t_start;

        LOG_INF("⏱️ Frame %d Time -> Image Prep: %u ms | AI Inference: %u ms", 
                frame_count, msg.prep_time, t_ai);

        if (err == 0) {
            app_postprocess_run((void**)out_buffers, &App_Config.output, &App_Config.input_static_param);

            if (App_Config.output.nb_detect > 0) {
                int valid_boxes = 0; 
                for (int i = 0; i < App_Config.output.nb_detect; i++) {
                    float w = App_Config.output.pOutBuff[i].width;
                    float h = App_Config.output.pOutBuff[i].height;
                    float cx = App_Config.output.pOutBuff[i].x_center;
                    float cy = App_Config.output.pOutBuff[i].y_center;
                    float conf = App_Config.output.pOutBuff[i].conf;
                    int class_idx = App_Config.output.pOutBuff[i].class_index;

                    if (class_idx == 0 || conf < 0.60f) continue;
                    valid_boxes++;

                    if (w <= 1.0f && h <= 1.0f) {
                        cx *= 192.0f; cy *= 192.0f; w *= 192.0f; h *= 192.0f;
                    }

                    int16_t bx = (int16_t)(cx - w / 2.0f);
                    int16_t by = (int16_t)(cy - h / 2.0f);
                    
                    // 🚨 畫在準備好的那張 LCD Buffer 上
                    draw_box_rgb565(lcd_buffer[buf_idx], 192, 192, bx, by, (uint16_t)w, (uint16_t)h, 0x07E0);
                }
                if (valid_boxes > 0) LOG_INF("[Result] Frame %d -> Successfully drawn %d Cones!", frame_count, valid_boxes);
            }

            if (device_is_ready(display_dev)) {
                struct display_buffer_descriptor buf_desc = {
                    .buf_size = sizeof(lcd_buffer[0]), .width = 192, .height = 192, .pitch = 192
                };
                // 🚨 將正確的 Buffer 推送到螢幕
                display_write(display_dev, 0, 0, &buf_desc, lcd_buffer[buf_idx]);
            }
        }
    }
    return 0;
}