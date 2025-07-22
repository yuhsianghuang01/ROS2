# CMake 建置系統配置文件
# 此文件用於 ROS2 的自定義訊息包(Message Package)
# wheeltec_robot_msg - 為銀河輪(WheelTec)機器人定義的訊息類型

# 指定 CMake 最低版本要求為 3.5
# CMake 是跨平台的建置工具，用於管理編譯過程
# 版本 3.5 是 ROS2 支援的最低版本，確保相容性
cmake_minimum_required(VERSION 3.5)

# 定義專案名稱為 wheeltec_robot_msg
# 這個名稱會成為 ROS2 包(Package)的名稱，用於：
# 1. 其他包引用此包時的標識符
# 2. 生成的訊息類型的命名空間前綴
# 3. 編譯輸出的目標名稱
project(wheeltec_robot_msg)

# 編譯器標準設定區段
# 設定 C 語言和 C++ 語言的編譯標準版本

# Default to C++14
# 設定 C 語言標準為 C99
# C99 提供了變長陣列、inline 函數、複數類型等功能
# CMAKE_C_STANDARD 變數控制整個專案的 C 標準版本
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
# 設定 C++ 語言標準為 C++14
# C++14 提供了 lambda 表達式改進、auto 推導增強等功能
# 這是 ROS2 建議的最低 C++ 標準，確保現代 C++ 功能可用
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 編譯器警告選項設定
# 針對 GCC 或 Clang 編譯器啟用嚴格的警告選項
# 這些選項有助於提早發現潛在的程式碼問題
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -Wall: 啟用常見警告 (如未使用變數、可能未初始化的變數等)
  # -Wextra: 啟用額外警告 (如符號比較、缺少回傳值等)
  # -Wpedantic: 啟用 ISO C++ 標準的嚴格警告
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ROS2 相依性套件搜尋區段
# 尋找並載入建置此訊息包所需的 ROS2 套件

# 尋找 ament_cmake 套件 (必要)
# ament_cmake 是 ROS2 的建置系統核心，提供：
# 1. 包管理功能 (package.xml 解析)
# 2. 安裝規則 (install targets)
# 3. 匯出資訊 (export information)
# 4. 測試框架整合
find_package(ament_cmake REQUIRED)

# 尋找 rosidl_default_generators 套件 (必要)
# 此套件提供 ROS2 介面定義語言 (IDL) 的程式碼生成器
# 用於將 .msg、.srv、.action 檔案轉換為各語言的實作程式碼
# 支援 C++、Python、C 等多種程式語言
find_package(rosidl_default_generators REQUIRED)

# 尋找 std_msgs 套件 (必要)
# std_msgs 包含 ROS2 的標準訊息類型，如：
# - Header: 時間戳記和座標框架資訊
# - String: 字串訊息
# - Bool, Int32, Float64: 基本資料類型
# - Array 類型: 各種陣列訊息
find_package(std_msgs REQUIRED)

# ROS2 介面程式碼生成區段
# 定義要生成的自定義訊息介面

# rosidl_generate_interfaces 函數用於生成 ROS2 訊息介面
# 此函數會為指定的 .msg 檔案生成對應的程式碼
rosidl_generate_interfaces(${PROJECT_NAME}
  # 指定要處理的訊息檔案
  # "msg/Data.msg" - 位於 msg/ 目錄下的 Data.msg 檔案
  # 生成後的訊息類型會是 wheeltec_robot_msg::msg::Data
  "msg/Data.msg"
  
  # DEPENDENCIES 指定此訊息依賴的其他套件
  # std_msgs - 表示 Data.msg 檔案中可能使用了 std_msgs 中的類型
  # 例如：std_msgs/Header、std_msgs/String 等
  DEPENDENCIES std_msgs
  
  # ADD_LINTER_TESTS 選項
  # 啟用程式碼品質檢查工具，包括：
  # - ament_lint_auto: 自動執行 linting 檢查
  # - ament_lint_common: 通用 linting 規則
  # 有助於維持程式碼品質和一致性
  ADD_LINTER_TESTS
)

# ROS2 套件匯出和完成設定區段

# 匯出執行時期相依性
# rosidl_default_runtime 提供執行時期所需的 ROS2 介面支援
# 當其他套件使用此訊息包時，會自動包含這些相依性
# 確保訊息的序列化、反序列化功能正常運作
ament_export_dependencies(rosidl_default_runtime)

# ament 套件完成標記
# 此指令必須是 CMakeLists.txt 的最後一行
# 執行最終的套件註冊和安裝步驟，包括：
# 1. 生成 package.xml 相關的安裝規則
# 2. 設定套件的匯出資訊
# 3. 建立套件在 ROS2 工作空間中的可見性
# 4. 註冊套件供其他套件 find_package() 使用
ament_package()
