idf_component_register(SRCS "midea_ir.c" "esp32_rmt_midea_ir_tx.c"
                            "mesh_main.c"
                            "mqtt_app.c"
                    INCLUDE_DIRS "." "include")

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")