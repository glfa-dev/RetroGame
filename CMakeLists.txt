cmake_minimum_required(VERSION 3.2)
project(mini_shooter)

set( SDL_STATIC OFF CACHE BOOL "" FORCE )
set( SDL_SHARED ON  CACHE BOOL "" FORCE )

add_subdirectory(SDL2)
include_directories(SDL2/include)
include_directories(stb/)

add_executable(mini_shooter main.c)

target_link_libraries(mini_shooter SDL2 SDL2main)

add_custom_target(copy_assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/assets ${CMAKE_CURRENT_BINARY_DIR}/assets
)

add_dependencies(mini_shooter copy_assets)
