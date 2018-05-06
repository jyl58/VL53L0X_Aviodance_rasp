project(VL53L0X_Aviodance)

cmake_minimum_required(VERSION 3.0)

add_compile_options(-std=c++11 )

SET(api_src_path VL53L0X_rasp/VL53L0X_1.0.2/Api/core/src)
SET(rasp_src_path VL53L0X_rasp/platform/src)

include_directories(
inc/
VL53L0X_rasp/VL53L0X_1.0.2/Api/core/inc
VL53L0X_rasp/platform/inc
inc/mavlink/include/mavlink/v2.0/
/usr/include)

SET(lib_src  ${api_src_path}/vl53l0x_api_calibration.c 
	${api_src_path}/vl53l0x_api_core.c 
	${api_src_path}/vl53l0x_api_ranging.c 
	${api_src_path}/vl53l0x_api_strings.c 
	${api_src_path}/vl53l0x_api.c 
	${rasp_src_path}/vl53l0x_platform.c )
	
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} ${lib_src})
target_link_libraries(${PROJECT_NAME} /usr/lib/libwiringPi.so)

add_executable(vl53l0x_aviodance src/vl53l0x_avoidance.cpp src/main.cpp src/serial.cpp)
target_link_libraries(vl53l0x_aviodance ${PROJECT_NAME})
