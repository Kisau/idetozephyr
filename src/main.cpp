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
}// <--- 確保這行在這裡把 extern "C" 關閉

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
// Video & LCD Buffer 配置
// ==========================================
#define MAX_FRAME_SIZE  (320 * 240 * 2) 
#define BUFFER_COUNT    2              

// 強制將相機緩衝區放在 SDRAM2
static uint8_t raw_video_data[BUFFER_COUNT][MAX_FRAME_SIZE] __attribute__((section("SDRAM2")));
static struct video_buffer video_buf_metadata[BUFFER_COUNT];
static struct video_format cam_fmt;

// ✅ 新增：專門給 LCD 顯示用的 RGB565 緩衝區 (192x192x2 = 73KB)，放 SDRAM2 省內部空間
static uint8_t lcd_buffer[192 * 192 * 2] __attribute__((section("SDRAM2")));

// ==========================================
// AI 核心記憶體配置 (效能與容量的完美平衡)
// ==========================================
static uint8_t network_context[STAI_NETWORK_CONTEXT_SIZE] __attribute__((aligned(32), section("SDRAM2")));
static stai_network* network = (stai_network*)network_context;

static uint8_t my_camera_data[STAI_NETWORK_IN_1_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 

static uint8_t my_output_0[STAI_NETWORK_OUT_1_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 
static uint8_t my_output_1[STAI_NETWORK_OUT_2_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 
static uint8_t my_output_2[STAI_NETWORK_OUT_3_SIZE_BYTES] __attribute__((aligned(32), section("SDRAM2"))); 

// ✅ 核心運算區保留在內部 SRAM 確保速度
static uint8_t my_activations[STAI_NETWORK_ACTIVATIONS_SIZE_BYTES] __attribute__((aligned(32)));

static uint8_t* in_buffers[1] = { my_camera_data };
static uint8_t* act_buffers[1] = { my_activations };
static uint8_t* out_buffers[3] = { my_output_0, my_output_1, my_output_2 };

// ==========================================
// 繪圖與設備初始化
// ==========================================

static void draw_box_rgb565(uint8_t *img_buf, uint32_t img_w, uint32_t img_h, 
                            int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img_w || y >= img_h) return;
    if (x + w > img_w) w = img_w - x;
    if (y + h > img_h) h = img_h - y;

    uint8_t color_h = (color >> 8) & 0xFF;
    uint8_t color_l = color & 0xFF;

    // 畫水平線 (上與下)
    for (uint16_t i = 0; i < w; i++) {
        size_t top_idx = ((y * img_w) + (x + i)) * 2;
        size_t bot_idx = (((y + h - 1) * img_w) + (x + i)) * 2;
        img_buf[top_idx] = color_h; img_buf[top_idx + 1] = color_l;
        img_buf[bot_idx] = color_h; img_buf[bot_idx + 1] = color_l;
    }
    // 畫垂直線 (左與右)
    for (uint16_t j = 0; j < h; j++) {
        size_t left_idx = (((y + j) * img_w) + x) * 2;
        size_t right_idx = (((y + j) * img_w) + (x + w - 1)) * 2;
        img_buf[left_idx] = color_h; img_buf[left_idx + 1] = color_l;
        img_buf[right_idx] = color_h; img_buf[right_idx + 1] = color_l;
    }
}

static void Enable_SDRAM_Cache(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable(); // 先停用記憶體保護單元

    // 設定外部 SDRAM (0xD0000000 開始的 32MB) 為「可快取 (Cacheable)」
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0xD0000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE; // ✅ 關鍵：開啟快取！
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    
    // 使用最高優先級的 Region 15 來強行覆蓋 Zephyr 的預設設定
    MPU_InitStruct.Number = MPU_REGION_NUMBER15; 
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT); // 重新啟用

    // 確保 CPU 的 D-Cache 和 I-Cache 都有打開
    SCB_EnableICache();
    SCB_EnableDCache();
}
static int setup_display(void) {
    display_dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_WRN("Display not found or not ready.");
        return -ENODEV;
    }
    display_set_pixel_format(display_dev, PIXEL_FORMAT_RGB_565);
    display_blanking_off(display_dev);
    return 0;
}

int ai_init(void) {
    if (stai_network_init(network) != 0) return -1;
    if (stai_network_set_activations(network, act_buffers, 1) != 0) return -1;
    if (stai_network_set_inputs(network, in_buffers, 1) != 0) return -1;
    if (stai_network_set_outputs(network, out_buffers, 3) != 0) return -1;
    return 0;
}

static int setup_camera(void) {
    struct video_format req_fmt;

    req_fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
    req_fmt.width = 320;
    req_fmt.height = 240;
    req_fmt.pitch = 320 * 2;

    int ret = video_set_format(video_dev, &req_fmt);
    if (ret < 0) {
        LOG_ERR("Failed to set video format to 320x240. (Error: %d)", ret);
        video_get_format(video_dev, &cam_fmt);
    } else {
        cam_fmt = req_fmt;
    }

    LOG_INF("Camera Format Set To: %dx%d", cam_fmt.width, cam_fmt.height);

    if (cam_fmt.size > MAX_FRAME_SIZE) {
        LOG_ERR("Camera frame size too large for buffers.");
        return -ENOMEM;
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

static int process_camera_frame(const uint8_t *rgb565_buf, uint32_t src_w, uint32_t src_h) {
    if (src_w < 192 || src_h < 192) return -1;

    uint32_t crop_x = (src_w - 192) / 2;
    uint32_t crop_y = (src_h - 192) / 2;

    uint8_t *ai_ptr = my_camera_data; // AI 用的 RGB888 緩衝區
    
    // 🚨 魔法 1：把 LCD 指標強制轉型為 16-bit！一次寫入兩個 Bytes
    uint16_t *lcd_ptr_16 = (uint16_t *)lcd_buffer; 

    for (uint32_t y = 0; y < 192; y++) {
        uint32_t src_y = y + crop_y;
        
        // 🚨 魔法 2：計算每一行的起點，並轉型為 16-bit 指標！
        // 這樣我們每次讀取都會直接從 SDRAM 抓出完整的 1 個像素 (2 Bytes)
        const uint16_t *row_ptr_16 = (const uint16_t *)(rgb565_buf + ((src_y * src_w) + crop_x) * 2);

        for (uint32_t x = 0; x < 192; x++) {
            
            // 🚨 魔法 3：利用 Little-Endian 天性，一行代碼完成「大小端序反轉」！
            // 當我們用 16-bit 讀取時，硬體會自動把 Byte1 放前面、Byte0 放後面
            // 這完全取代了你原本寫的 (row_start[x*2+1] << 8) | row_start[x*2]
            uint16_t pixel = *row_ptr_16++;

            // 取出 R, G, B
            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5)  & 0x3F;
            uint8_t b =  pixel        & 0x1F;

            // 🚨 魔法 4：消滅除法！改用「位移運算 (Bit-shift)」來做數值放大
            // 這跟 * 255 / 31 的結果幾乎一模一樣，但速度快了幾十倍！
            *ai_ptr++ = (r << 3) | (r >> 2); // 把 5-bit 映射到 8-bit
            *ai_ptr++ = (g << 2) | (g >> 4); // 把 6-bit 映射到 8-bit
            *ai_ptr++ = (b << 3) | (b >> 2); // 把 5-bit 映射到 8-bit

            // 🚨 魔法 5：直接用 16-bit 把像素寫進 LCD，省下一半的寫入時間！
            *lcd_ptr_16++ = pixel;
        }
    }
    return 0;
}

#pragma GCC pop_options
// ==========================================
// 主程式
// ==========================================
int main(void) {
    Enable_SDRAM_Cache();
    k_msleep(1000);
    LOG_INF("Starting STEdgeAI + Zephyr Camera & Display...");

    if (ai_init() != 0) {
        LOG_ERR("AI Init Failed!");
        return 0;
    }
    App_Config.output.pOutBuff = detections;
    App_Config.output.nb_detect = 0;
    app_postprocess_init(&App_Config);

    video_dev = DEVICE_DT_GET_ANY(st_stm32_dcmi); 
    if (!video_dev) video_dev = DEVICE_DT_GET(DT_NODELABEL(ov5640)); 

    if (!device_is_ready(video_dev) || setup_camera() < 0) {
        LOG_ERR("Camera setup failed.");
        return 0;
    }

    // ✅ 初始化 LCD
    setup_display();

    int stream_err = video_stream_start(video_dev, VIDEO_BUF_TYPE_OUTPUT);
    if (stream_err < 0) {
        LOG_ERR("Failed to start video stream! Error: %d", stream_err);
        return 0;
    }

    LOG_INF("System Ready. Running Inference on EVERY frame.");
    LOG_INF("🚀 Current System Core Clock: %u MHz", SystemCoreClock / 1000000);
    uint32_t frame_count = 0;

    while (1) {
        struct video_buffer *vbuf;
        
        int dq_ret = video_dequeue(video_dev, &vbuf, K_MSEC(2000));
        
        if (dq_ret < 0) {
            LOG_WRN("[Error] video_dequeue failed or timeout! Error: %d", dq_ret);
            continue; 
        }

        frame_count++;

        // ==========================================
        // ⏱️ 測速起點 1：影像裁切與轉換
        // ==========================================
        uint32_t t_start = k_uptime_get_32();
        
        int prep_res = process_camera_frame(vbuf->buffer, cam_fmt.width, cam_fmt.height);
        
        uint32_t t_prep = k_uptime_get_32() - t_start; // 計算影像處理耗時

        if (prep_res == 0) {
            
            // 影像處理完就可以先釋放相機 buffer，讓相機繼續拍下一張
            video_enqueue(video_dev, vbuf);
            
            // ==========================================
            // ⏱️ 測速起點 2：AI 神經網路推論
            // ==========================================
            t_start = k_uptime_get_32();
            
            stai_return_code err = stai_network_run(network, (stai_run_mode)0);
            
            uint32_t t_ai = k_uptime_get_32() - t_start; // 計算 AI 推論耗時
            
            // 🚨 印出精準的耗時報告！(這是抓出真兇的關鍵)
            LOG_INF("⏱️ Frame %d Time -> Image Prep: %u ms | AI Inference: %u ms", 
                    frame_count, t_prep, t_ai);

            if (err == 0) {
                app_postprocess_run(
                    (void**)out_buffers, 
                    &App_Config.output, 
                    &App_Config.input_static_param 
                );

                if (App_Config.output.nb_detect > 0) {
                    int valid_boxes = 0; // 用來計算真正是三角錐的數量
                    
                    for (int i = 0; i < App_Config.output.nb_detect; i++) {
                        // 1. 取得原始浮點數資料
                        float w = App_Config.output.pOutBuff[i].width;
                        float h = App_Config.output.pOutBuff[i].height;
                        float cx = App_Config.output.pOutBuff[i].x_center;
                        float cy = App_Config.output.pOutBuff[i].y_center;
                        float conf = App_Config.output.pOutBuff[i].conf;
                        int class_idx = App_Config.output.pOutBuff[i].class_index;

                        // 🚨 兩道最重要防線
                        if (class_idx == 0) continue; // 防線 1：絕對不畫 Class 0 (背景)
                        if (conf < 0.60f) continue;   // 防線 2：信心度低於 60% 的不要畫

                        valid_boxes++;

                        // 2. 驗證：印出過濾後真正的三角錐資料
                        LOG_INF("  -> Obj[%d]: Class=%d (Cone!), Conf=%.2f, Raw_CX=%.3f, Raw_CY=%.3f, W=%.3f, H=%.3f", 
                                i, class_idx, conf, cx, cy, w, h);

                        // 3. 座標還原：如果數值 <= 1.0，代表是比例值，必須乘上影像大小 (192)
                        if (w <= 1.0f && h <= 1.0f) {
                            cx *= 192.0f;
                            cy *= 192.0f;
                            w *= 192.0f;
                            h *= 192.0f;
                        }

                        // 4. 計算左上角座標
                        int16_t bx = (int16_t)(cx - w / 2.0f);
                        int16_t by = (int16_t)(cy - h / 2.0f);
                        uint16_t box_w = (uint16_t)w;
                        uint16_t box_h = (uint16_t)h;

                        // 5. 畫框 (綠色)
                        draw_box_rgb565(lcd_buffer, 192, 192, bx, by, box_w, box_h, 0x07E0);
                    }
                    
                    if (valid_boxes > 0) {
                        LOG_INF("[Result] Frame %d -> Successfully drawn %d Cones!", frame_count, valid_boxes);
                    } else {
                        LOG_INF("[Result] Frame %d -> Boxes found, but all were background or low confidence.", frame_count);
                    }
                } else {
                    LOG_INF("[Result] Frame %d -> No objects.", frame_count);
                }

                if (device_is_ready(display_dev)) {
                    struct display_buffer_descriptor buf_desc = {
                        .buf_size = sizeof(lcd_buffer),
                        .width = 192,
                        .height = 192,
                        .pitch = 192
                    };
                    // 顯示在螢幕最左上角 (x=0, y=0)
                    display_write(display_dev, 0, 0, &buf_desc, lcd_buffer);
                }

            } else {
                LOG_WRN("[Error] AI inference failed! Code: 0x%08X", (unsigned int)err);
            }
        } else {
            video_enqueue(video_dev, vbuf);
        }
    }
    
    return 0;
}