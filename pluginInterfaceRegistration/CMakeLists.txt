
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin/lib)
#set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)
set(pluginInterfaceRegistration_SRCS
	pluginInterfaceRegistration.h
	pluginInterfaceRegistration_global.h
	MyPointRepresentation.h
)

# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)

#find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
#add_definitions(-DMEDIAN_FILTER_PLUGIN_LIBRARY)
add_library(pluginInterfaceRegistration SHARED ${pluginInterfaceRegistration_SRCS})#
qt5_use_modules(pluginInterfaceRegistration Core Gui Widgets)
target_link_libraries(pluginInterfaceRegistration
	${QT_LIBRARIES}
	${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
