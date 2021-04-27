set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin/plugins)
#message(LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH})
set(pluginRegistrationPCL_SRCS
	pluginRegistrationPCL.cpp
        pluginRegistrationPCL.h
)

# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)

add_library(pluginRegistrationPCL SHARED ${pluginRegistrationPCL_SRCS})#
qt5_use_modules(pluginRegistrationPCL Core Gui Widgets)
target_link_libraries(pluginRegistrationPCL
    ${QT_LIBRARIES}
    ${PCL_LIBRARIES}
    pluginInterfaceRegistration
)
