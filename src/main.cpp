#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>


#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/display.h> 
extern "C" {
    #include "network.h"
    #include "network_data.h"
    #include "main.h" 
    #include "app_postprocess.h" 
} 
}// <--- 確保這行在這裡把 extern "C" 關閉

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// ==========================================
// 系統設備與全域變數
// ==========================================
static const struct device *video_dev;
static const struct device *display_dev; // ✅ 新增 LCD 設備
static AppConfig_TypeDef App_Config;

#define MAX_DETECTIONS 10 
static postprocess_outBuffer_t detections[MAX_DETECTIONS];

// ==========================================
// Video & LCD Buffer 配置
// ==========================================
#define MAX_FRAME_SIZE  (320 * 240 * 2) 
#define BUFFER_COUNT    4               

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

// ==========================================
// 影像處理: 將相機畫面裁切為 192x192 並轉 RGB888 (給AI) 與 RGB565 (給LCD)
// ==========================================
static int process_camera_frame(const uint8_t *rgb565_buf, uint32_t src_w, uint32_t src_h) {
    if (src_w < 192 || src_h < 192) {
        LOG_ERR("Resolution too small for 192x192 crop!");
        return -1;
    }

    uint32_t crop_x = (src_w - 192) / 2;
    uint32_t crop_y = (src_h - 192) / 2;
    size_t ai_idx = 0;
    size_t lcd_idx = 0; // ✅ 新增：LCD buffer 索引

    for (uint32_t y = 0; y < 192; y++) {
        for (uint32_t x = 0; x < 192; x++) {
            uint32_t src_x = x + crop_x;
            uint32_t src_y = y + crop_y;
            size_t pixel_idx = (src_y * src_w + src_x) * 2;
            
            uint16_t pixel = (rgb565_buf[pixel_idx] << 8) | rgb565_buf[pixel_idx + 1];

            // 1. 給 AI 用的 RGB888
            my_camera_data[ai_idx++] = ((pixel >> 11) & 0x1F) * 255 / 31; // R
            my_camera_data[ai_idx++] = ((pixel >>  5) & 0x3F) * 255 / 63; // G
            my_camera_data[ai_idx++] = ( pixel        & 0x1F) * 255 / 31; // B

            // 2. ✅ 給 LCD 用的 RGB565 (直接拷貝原始位元組)
            lcd_buffer[lcd_idx++] = rgb565_buf[pixel_idx];
            lcd_buffer[lcd_idx++] = rgb565_buf[pixel_idx + 1];
        }
    }
    return 0;
}

// ==========================================
// 主程式
// ==========================================
int main(void) {
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

    uint32_t frame_count = 0;

    while (1) {
        struct video_buffer *vbuf;
        
        // 為了減少雜訊，我們在這裡就不印 Step 1 的 log 了
        int dq_ret = video_dequeue(video_dev, &vbuf, K_MSEC(2000));
        
        if (dq_ret < 0) {
            LOG_WRN("[Error] video_dequeue failed or timeout! Error: %d", dq_ret);
            continue; 
        }

        frame_count++;

        if (process_camera_frame(vbuf->buffer, cam_fmt.width, cam_fmt.height) == 0) {
            
            
            video_enqueue(video_dev, vbuf);
            
            
            stai_return_code err = stai_network_run(network, (stai_run_mode)0);
            
            if (err == 0) {
                app_postprocess_run(
                    (void**)out_buffers, 
                    &App_Config.output, 
                    &App_Config.input_static_param 
                );

                if (App_Config.output.nb_detect > 0) {
                    LOG_INF("[Result] Frame %d -> Found %d objects!", frame_count, (int)App_Config.output.nb_detect);
                    
                    
                    for (int i = 0; i < App_Config.output.nb_detect; i++) {
                        float w = App_Config.output.pOutBuff[i].width;
                        float h = App_Config.output.pOutBuff[i].height;
                        float cx = App_Config.output.pOutBuff[i].x_center;
                        float cy = App_Config.output.pOutBuff[i].y_center;
                        
                        
                        int16_t bx = (int16_t)(cx - w / 2.0f);
                        int16_t by = (int16_t)(cy - h / 2.0f);

                        
                        
                        draw_box_rgb565(lcd_buffer, 192, 192, bx, by, (uint16_t)w, (uint16_t)h, 0x07E0);
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