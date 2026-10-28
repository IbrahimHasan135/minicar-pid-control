if(NOT CMAKE_SCRIPT_MODE_FILE)
    set_source_files_properties("main.c" PROPERTIES LANGUAGE CXX)
endif()

idf_component_register(
    SRCS
        "main.c"
        "application/Mission.cpp"
        "driver/communication/SerialDriver.cpp"
        "driver/imu/IMUDriver.cpp"
        "driver/motor/MotorEncoderDriver.cpp"
        "driver/telemetry/ConsoleDriver.cpp"
        "logic/SerialTask.cpp"
        "logic/TelemetryTask.cpp"
        "logic/motion_control/MotionCommandTask.cpp"
        "logic/motion_control/MotionControlContext.cpp"
        "logic/motion_control/MotionLoopTask.cpp"
        "service/communication/CommunicationService.cpp"
        "service/control/PIDController.cpp"
        "service/heading/HeadingService.cpp"
        "service/motor/MotorControlService.cpp"
        "service/odometry/OdometryService.cpp"
        "service/telemetry/TelemetryService.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        freertos
        log
)
