cmake_minimum_required(VERSION 3.1)


set(PROJECT "app")


# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

project(${PROJECT})

# Find includes in corresponding build directories
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
# set(CMAKE_AUTOMOC ON)

ADD_DEFINITIONS(
    -std=c++14 # Or -std=c++0x
    # O
)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")

# Find packages needed to build library
find_package(libusb-1.0 REQUIRED)
# libfreenect support
# libfreenect.h includes libusb.h, so everyone needs this too
include_directories(${LIBUSB_1_INCLUDE_DIRS})
include_directories("/usr/local/Cellar/libusb/1.0.21/include/libusb-1.0/")
include_directories("/usr/local/include/libfreenect")

# # Find packages needed to build library
find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS system filesystem date_time thread REQUIRED)
find_package(PCL 1.7 REQUIRED)


# # libfreenect support
# # libfreenect.h includes libusb.h, so everyone needs this too
 include_directories(${LIBUSB_1_INCLUDE_DIRS})
 include_directories("/usr/include/libusb-1.0")
 include_directories("/usr/local/include/libfreenect")
include_directories( ${Boost_INCLUDE_DIRS} )
 include_directories(${PCL_INCLUDE_DIRS})
include_directories(BEFORE /usr/local/opt/qt5/include)

# add_custom_target(run
#     sudo ./${PROJECT}
#     WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
# )

 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})









# Find the QtWidgets library
find_package(Qt5Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB_RECURSE SRCS src/*.cpp)

# Tell CMake to create the helloworld executable
add_executable(${PROJECT} WIN32 ${SRCS})

target_link_libraries(${PROJECT} ${OpenCV_LIBS})
# Use the Widgets module from Qt 5.
target_link_libraries(${PROJECT} Qt5::Widgets)

target_link_libraries(${PROJECT} ${Boost_LIBRARIES})

 target_link_libraries (${PROJECT} ${PCL_LIBRARIES})
# target_link_libraries (${PROJECT} ${LIBUSB_1_LIB})

target_link_libraries (${PROJECT} ${LIBUSB_1_LIB})

target_link_libraries(${PROJECT} "freenect")


set_target_properties(${PROJECT} PROPERTIES
        CXX_STANDARD_REQUIRED 14
        CXX_EXTENSIONS ON
        )
