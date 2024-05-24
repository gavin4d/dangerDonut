idf_component_register(SRCS "kalmanFilter.cpp" "orientator.cpp"
                       INCLUDE_DIRS .
                       REQUIRES driver esp_timer "ADXL375")
