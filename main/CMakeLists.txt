idf_component_register(
    SRCS
        "main.c"
        "driver/imu/IMUDriver.cpp"
        "driver/motor/MotorEncoderDriver.cpp"
        "service/control/PIDController.cpp"
        "service/heading/HeadingService.cpp"
        "service/motor/MotorControlService.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_pcnt
        esp_timer
        freertos
        log
)

# main.c sengaja menjadi entry point bench test, tetapi isinya memakai class C++.
set_source_files_properties("main.c" PROPERTIES LANGUAGE CXX)
