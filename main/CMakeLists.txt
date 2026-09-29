idf_component_register(
    SRCS
        "main.c"
        "driver/motor/MotorEncoderDriver.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_gpio
        esp_driver_pcnt
        esp_timer
        freertos
        log
)

# main.c sengaja menjadi entry point bench test, tetapi isinya memakai class C++.
set_source_files_properties("main.c" PROPERTIES LANGUAGE CXX)
