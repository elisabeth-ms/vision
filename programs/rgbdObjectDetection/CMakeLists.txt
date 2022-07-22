option(ENABLE_rgbdObjectDetection "Enable/disable rgbdObjectDetection program" ON)

if(ENABLE_rgbdObjectDetection)

    add_executable(rgbdObjectDetection main.cpp
                                 RgbdObjectDetection.hpp
                                 RgbdObjectDetection.cpp)

    target_link_libraries(rgbdObjectDetection YARP::YARP_os
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        YARP::YARP_sig
                                        ROBOTICSLAB::VisionInterfaces
                                        ${OpenCV_LIBRARIES})

    if(YARP_VERSION VERSION_GREATER_EQUAL 3.5)
        target_compile_definitions(rgbdObjectDetection PRIVATE HAVE_CROP)
        target_link_libraries(rgbdObjectDetection ROBOTICSLAB::YarpCropCallback)
    endif()

    install(TARGETS rgbdObjectDetection
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_rgbdObjectDetection OFF CACHE BOOL "Enable/disable rgbdObjectDetection program" FORCE)

endif()
