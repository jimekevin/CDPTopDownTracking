cmake_minimum_required(VERSION 3.17.3)

project(TopDownTracking)

message(STATUS "Compiling for Mac OS")

add_definitions(-DAPPLE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(Qt5_DIR /usr/local/opt/qt/lib/cmake/Qt5)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(GLFW3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
#find_package(libfreenect REQUIRED)
#find_package(freenect2 REQUIRED PATHS /usr/local/Cellar/libfreenect2/0.2.0/lib)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(realsense2 CONFIG REQUIRED)
find_package(Boost REQUIRED)

set(SOURCES
        ${PROJECT_SOURCE_DIR}/TopDownTracking/Config.cpp
        #${PROJECT_SOURCE_DIR}/TopDownTracking/KinectManager_MacOS.cpp
        ${PROJECT_SOURCE_DIR}/TopDownTracking/MainGLWidget.cpp
        ${PROJECT_SOURCE_DIR}/TopDownTracking/MainWindow.cpp
        ${PROJECT_SOURCE_DIR}/TopDownTracking/Map.cpp
        ${PROJECT_SOURCE_DIR}/TopDownTracking/OutputTextBrowser.cpp
        ${PROJECT_SOURCE_DIR}/TopDownTracking/Pipeline.cpp
        ${PROJECT_SOURCE_DIR}/TopDownTracking/RealsenseCameraManager.cpp
        ${PROJECT_SOURCE_DIR}/TopDownTracking/RecordVideoPushButton.cpp
        ${PROJECT_SOURCE_DIR}/TopDownTracking/main.cpp
        #${PROJECT_SOURCE_DIR}/TopDownTracking/glad/src/glad.c
        )

add_executable(TopDownTracking ${SOURCES})

include_directories(
        #${PROJECT_SOURCE_DIR}/TopDownTracking/glad/include
        ${GLFW3_INCLUDE_DIRS}
        ${flextGL_INCLUDE_DIRS}
        #/usr/local/Cellar/libfreenect/0.6.1/include/libfreenect
        #${freenect2_INCLUDE_DIR}
        /usr/local/opt/qt/include
        ${PROJECT_SOURCE_DIR}/TopDownTracking
        ${PROJECT_SOURCE_DIR}/TopDownTracking/config
        ${PROJECT_SOURCE_DIR}/TopDownTracking/pipeline_tasks
        ${PROJECT_SOURCE_DIR}/TopDownTracking/shaders
        )

target_link_libraries(TopDownTracking
        ${OpenCV_LIBS}
        ${OPENGL_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        #${FREENECT_LIBRARIES}
        #${freenect2_LIBRARIES}
        Qt5::Core Qt5::Gui Qt5::Widgets
        realsense2::fw realsense2::realsense2 realsense2::realsense-file
        )
