# 1. 基本 CMake 與 Zephyr 初始化
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(idetozephyr)

# 2. 🌟 最關鍵的一行：引入你的 lib 資料夾
# 這會執行 lib/CMakeLists.txt 並建立 stm32_ai_lib 模組
add_subdirectory(lib)

# 3. 定義主程式 (app) 的原始碼
target_sources(app PRIVATE src/main.cpp  lib/Src/app_postprocess.c)

# 4. 讓 main.cpp 能夠找到 lib 裡的標頭檔 (.h)
# 雖然 lib 裡有寫，但在這裡再次定義 app 的搜尋路徑是最保險的做法
target_include_directories(app PRIVATE 
    src
    lib/Inc
    lib/Network/Inc
    lib/ST/stedgeai-lib/Inc
    lib/ST/STM32_ImageProcessing_Library/Inc
    lib/ST/STM32_Objdetect_Postprocess/Inc
)

# 5. 強制開啟浮點運算支援 (STM32 AI 核心必備)
zephyr_cc_option(-mfloat-abi=hard -mfpu=fpv5-d16)
