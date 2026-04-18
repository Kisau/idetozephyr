# 1. 宣告 ST AI 核心靜態庫 (指向你的 .a)
add_library(st_ai_core STATIC IMPORTED GLOBAL)
set_target_properties(st_ai_core PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/ST/stedgeai-lib/Lib/GCC/STM32H7/NetworkRuntime1100_CM7_GCC.a
)

# 2. 宣告後處理庫
add_library(st_post_process STATIC IMPORTED GLOBAL)
set_target_properties(st_post_process PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/ST/STM32_Objdetect_Postprocess/lib/objdetect_pp_GCC.a
)

# 3. 將原始碼加入到 app (主程式目標)
# 注意：這裡要把路徑寫清楚，並移除找不到的 ai_render_utils.c
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Network/Src/network.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Network/Src/network_data.c
    # 如果你有其他確定存在的 .c 檔案，再加在這裡
)

# 4. 加入所有標頭檔路徑
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Network/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/ST/stedgeai-lib/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/ST/STM32_ImageProcessing_Library/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/ST/STM32_Objdetect_Postprocess/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/DSP/Include
)

# 5. 連結靜態庫
target_link_libraries(app PUBLIC st_ai_core)
target_link_libraries(app PUBLIC st_post_process)