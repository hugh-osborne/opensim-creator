# Emscripten packaging

# for now, just ensure that the html loader is next to the built js+wasm
add_custom_command(
    TARGET hellotriangle
    PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:hellotriangle> ${CMAKE_CURRENT_SOURCE_DIR}/Emscripten/hellotriangle.html
    COMMAND_EXPAND_LISTS
)
