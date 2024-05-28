##############################################################################
# CMake
##############################################################################
cmake_minimum_required(VERSION 3.0.2)
project(my_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# 将包含build目录下的头文件，qt会生成moc_中间头文件以及ui_xxx.h 头文件
set(CMAKE_INCLUDE_CURRENT_DIR ON)
##############################################################################
# Catkin
##############################################################################
# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS 
    nav_msgs     # nav_msgs 必须在tf之前，否则会出现 没有规则可制作目标“/opt/ros/melodic_arm_arm/lib/libroscpp.so”的错误
    roscpp 
    tf
    sensor_msgs
    std_msgs
#    image_transport   
    )


find_package(OpenCV REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
    ./include
    ./comm
)

catkin_package()
##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

#######
## 安装多个 Qt 版本时, 可能会导致无法使用特定的 Qt 版本. 需要设置 CMAKE_PREFIX_PATH 为所用 Qt 的安装路径, 但是还需要设置 Qt[Module]_DIR(例如: QtCore_DIR) 等变量. 下面这一段代码自动会修改 ModuleList 中的所有模块, 并自动设置 Qt[Module]_DIR变量; 同时 target_link_libraries.
# set(CMAKE_PREFIX_PATH "/opt/Qt5.12.6/5.12.6/gcc_64")
set(ModuleList 
    Core
    Gui
    Widgets
#    Multimedia
    Network
)

find_package(Qt5 REQUIRED ${ModuleList})

foreach(QtModule ${ModuleList})
    # set(Qt5${QtModule}_DIR ${CMAKE_PREFIX_PATH}/lib/cmake/Qt5${QtModule})
    # message("foreach-> Qt5${QtModule}_DIR : ${Qt5${QtModule}_DIR}")

    set(QT_LIBRARIES ${QT_LIBRARIES} Qt5::${QtModule})
endforeach()
message("${QT_LIBRARIES}")

add_subdirectory(comm)
#######

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QRC_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h basewidget/*.hpp *.h)

#set(QRC_SOURCE_FILES image.qrc)
#qt5_add_resources(QRC_FILES ${QRC_SOURCE_FILES})
#add_library(${PROJECT_NAME}
#${QRC_FILES})

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp basewidget/*.cpp)

##############################################################################
# Binaries
##############################################################################

find_library(cv_bridge_LIBRARIES
  NAMES cv_bridge
  DOC "cv_bridge library")

add_executable(robot_gui ${QRC_FILES} ${QT_SOURCES} ${QT_RESOURCES_CPP} 
                                        ${QT_FORMS_HPP} ${QT_MOC_HPP})

target_link_directories(robot_gui PUBLIC
    ${EXTRA_LINK_DIR}
)

target_link_libraries(robot_gui 
    ${QT_LIBRARIES} 
    ${catkin_LIBRARIES} 
    ${cv_bridge_LIBRARIES}
    ${OpenCV_LIBS}
    client_comm_lib
)
install(TARGETS robot_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############################################################################
#HTML 将html文件拷贝到可执行文件同级目录
#############################################################################
#file(REMOVE_RECURSE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/html)
#file(REMOVE_RECURSE ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/html)
#file(COPY ${PROJECT_SOURCE_DIR}/html  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/html DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#message("PROJECT_SOURCE_DIR:${CMAKE_INSTALL_PREFIX}")

#############################################################################
#DEBUG && RELEASE
#############################################################################
#如果要生成debug版本
#SET(CMAKE_BUILD_TYPE "Debug")
#如果要生成release版本
#SET(CMAKE_BUILD_TYPE "Release")
#添加的支持debug和release支持
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
