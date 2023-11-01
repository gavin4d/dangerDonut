idf_component_register(SRCS "kalmanFilter.cpp" "orientator.cpp"
                       INCLUDE_DIRS .
                       REQUIRES driver esp-dsp arduino "ADXL375")
