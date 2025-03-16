cmake_minimum_required(VERSION 3.10.2)
project(play_video)
find_package( OpenCV REQUIRED )
find_package(Qt5Widgets REQUIRED)

set(CMAKE_AUTOMOC ON) 

set(ONNXRUNTIME_INCLUDE_DIR "/home/vinoth/onnxruntime_cpp/onnxruntime-linux-x64-1.16.0/include")
set(ONNXRUNTIME_LIB_DIR "/home/vinoth/onnxruntime_cpp/onnxruntime-linux-x64-1.16.0/lib")

include_directories( ${OpenCV_INCLUDE_DIRS} ${ONNXRUNTIME_INCLUDE_DIR} )

link_directories(${ONNXRUNTIME_LIB_DIR})


find_package(behaviortree_cpp)
# set(
#     OpenCV_LIBS
#     -lopencv_contrib
#     -lopencv_core
#     -lopencv_highgui
#     -lopencv_imgproc
#     -lopencv_legacy
#     )
add_executable(play_video play_video.cpp )
add_executable(detect det.cpp)
# add_executable(obs obstacle_avoidance.cpp) ////

add_executable(otest otest.cpp)
add_executable(gui gui.cpp)
add_executable(CameraCalib CamCalibration/CameraCalib.cpp)
add_executable(SaveImages CamCalibration/SaveImages.cpp)
add_executable(bt_obs obstacle_avoidance.cpp)
add_executable(serial serial.cpp)
add_executable(nav_v1 navigation_v1.cpp)

#obstacle avoidance
add_executable(vanishingpt obstacleAvoidance/vanishingPointEst.cpp)
add_executable(vanishing obstacleAvoidance/vanishing.cpp)
add_executable(disparity obstacleAvoidance/disparity.cpp)
add_executable(opticalflow obstacleAvoidance/opticalflow.cpp)
target_link_libraries(serial pthread)
target_link_libraries(bt_obs behaviortree_cpp Qt5::Widgets ${OpenCV_LIBS})
target_link_libraries(nav_v1 behaviortree_cpp Qt5::Widgets ${OpenCV_LIBS})
target_link_libraries(otest behaviortree_cpp Qt5::Widgets ${OpenCV_LIBS} onnxruntime)
target_link_libraries( play_video ${OpenCV_LIBS} )
target_link_libraries( detect ${OpenCV_LIBS} )
target_link_libraries( CameraCalib ${OpenCV_LIBS})  
target_link_libraries( SaveImages ${OpenCV_LIBS})
target_link_libraries( vanishingpt ${OpenCV_LIBS} onnxruntime)
target_link_libraries( vanishing ${OpenCV_LIBS})
target_link_libraries( disparity ${OpenCV_LIBS})
target_link_libraries( opticalflow ${OpenCV_LIBS})
# target_link_libraries( gui Qt5::Widgets ${OpenCV_LIBS})
# add_executable(gui main.cpp gui.cpp)
target_link_libraries(gui Qt5::Widgets ${OpenCV_LIBS})
# target_link_libraries( obs Qt5::Widgets ${OpenCV_LIBS}) ////