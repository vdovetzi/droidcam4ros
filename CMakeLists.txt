cmake_minimum_required(VERSION 3.8)
project(droidcam4ros)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS videoio)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB REQUIRED gio-2.0)
pkg_check_modules(LIBAV REQUIRED libswscale libavutil)
pkg_check_modules(JPEG REQUIRED libturbojpeg)
pkg_check_modules(USBMUXD REQUIRED libusbmuxd)

find_library(SPEEX_LIBRARY speex)
find_library(ASOUND_LIBRARY asound)
find_library(PTHREAD_LIBRARY pthread)
find_library(M_LIBRARY m)

####### DroidCam Linux Library #############
add_library(droidcam-lib SHARED
  src/lib/connection.c
  src/lib/decoder.c
  src/lib/queue.c
  src/lib/settings.c
  src/lib/usb.c
  src/lib/av.c
  src/lib/decoder_v4l2.c
  src/lib/resources.c
  src/lib/decoder_snd.c
)
target_include_directories(droidcam-lib PUBLIC
  include/${PROJECT_NAME}/
)

target_include_directories(droidcam-lib PRIVATE
  ${GLIB_INCLUDE_DIRS}
)

target_compile_options(droidcam-lib PRIVATE 
  ${GLIB_CFLAGS}
  ${LIBAV_CFLAGS} 
  ${JPEG_CFLAGS} 
  ${USBMUXD_CFLAGS}
)
target_link_libraries(droidcam-lib
  ${GLIB_LIBRARIES}
  ${LIBAV_LIBRARIES}
  ${JPEG_LIBRARIES}
  ${USBMUXD_LIBRARIES}
  ${SPEEX_LIBRARY}
  ${ASOUND_LIBRARY}
  ${PTHREAD_LIBRARY}
  ${M_LIBRARY}  
)
######### DroidCam Linux Client ############
add_executable(droidcam-cli src/droidcam-cli.c)
target_link_libraries(droidcam-cli PRIVATE 
  droidcam-lib
)
############################################

add_executable(droidcam-publisher src/droidcam-publisher.cpp)
ament_target_dependencies(droidcam-publisher
  rclcpp
  image_transport
  cv_bridge
)
target_link_libraries(droidcam-publisher ${OpenCV_LIBS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  droidcam-cli
  droidcam-publisher
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
