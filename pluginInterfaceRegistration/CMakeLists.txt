
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin/lib)
set(pluginInterfaceRegistration_SRCS
	pluginInterfaceRegistration.h
	pluginInterfaceRegistration_global.h
	MyPointRepresentation.h
)

# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)

add_library(pluginInterfaceRegistration SHARED ${pluginInterfaceRegistration_SRCS})#
qt5_use_modules(pluginInterfaceRegistration Core Gui Widgets)
target_link_libraries(pluginInterfaceRegistration
	${QT_LIBRARIES}
	${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
