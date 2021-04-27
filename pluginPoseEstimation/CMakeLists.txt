
FIND_PACKAGE(PCL REQUIRED)
message(STATUS "PCL version:" ${PCL_VERSION})
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin/lib)
#set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)
#set(pluginInterfaceSegment_SRCS
#	pluginInterfaceSegment.h
#	pluginInterfaceSegment_global.h
#        segment_interface.h
#)

# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)

#find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
#add_definitions(-DMEDIAN_FILTER_PLUGIN_LIBRARY)
#add_library(pluginInterfaceSegment SHARED ${pluginInterfaceSegment_SRCS})#
#qt5_use_modules(pluginInterfaceSegment Core Gui Widgets)
#target_link_libraries(pluginInterfaceSegment
#	${QT_LIBRARIES}
#    ${OpenCV_LIBS}
#    segment_interface
#)
