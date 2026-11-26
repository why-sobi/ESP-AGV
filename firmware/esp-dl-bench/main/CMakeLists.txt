# The quantized models from scripts/models/quantize_int8.py, copied under
# unique names (embedded symbols are named after the file name).
set(quantized "${CMAKE_CURRENT_LIST_DIR}/../../../models/esp-quantized-v1")
set(embedded "")
foreach(quant w8a8 w8a16 w16a16)
  foreach(model motor ir motor_gru)
    set(source "${quantized}/${quant}/${model}.espdl")
    if(EXISTS "${source}")
      set(target "${CMAKE_CURRENT_BINARY_DIR}/${model}_${quant}.espdl")
      configure_file("${source}" "${target}" COPYONLY)
      list(APPEND embedded "${target}")
    else()
      message(FATAL_ERROR "Missing ${source}: run scripts/models/quantize_int8.py first")
    endif()
  endforeach()
endforeach()

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_driver_gpio esp_timer
    EMBED_FILES ${embedded}
)
