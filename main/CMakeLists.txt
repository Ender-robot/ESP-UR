idf_component_register(SRCS "demo.cpp" "datapack.cpp" "ahrs.cpp"
                    PRIV_REQUIRES freertos hardware interface peripheral
                    INCLUDE_DIRS ".")
