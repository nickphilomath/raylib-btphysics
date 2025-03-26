cmake_minimum_required(VERSION 3.10)

project(3rdPerson LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Manually add Raylib from deps/
add_subdirectory(vendor/raylib)

# Add Bullet Physics
add_subdirectory(vendor/bullet3)

# Find or clone Raylib
# find_package(raylib 2.0 QUIET)
# if (NOT raylib_FOUND)
#   include(FetchContent)
#   FetchContent_Declare(
#     raylib
#     GIT_REPOSITORY https://github.com/raysan5/raylib.git
#     GIT_TAG 4af4483f5fabb686db5f501c8a2947ab83fe84a2
#   )
#   FetchContent_GetProperties(raylib)
#   if (NOT raylib_POPULATED) # Have we downloaded raylib yet?
#     set(FETCHCONTENT_QUIET NO)
#     FetchContent_Populate(raylib)
#     set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # don't build the supplied examples
#     set(BUILD_GAMES    OFF CACHE BOOL "" FORCE) # or games
#     add_subdirectory(${raylib_SOURCE_DIR} ${raylib_BINARY_DIR})
#   endif()
# endif()

# Automatically include all .cpp files from the src/ directory
file(
    GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS 
    
    src/Application.cpp
)

# Add source files
add_executable(${PROJECT_NAME} src/main.cpp ${SRC_FILES})

# Link Raylib
target_link_libraries(${PROJECT_NAME} raylib)

# Link Bullet Physics
target_link_libraries(${PROJECT_NAME}
    BulletDynamics
    BulletCollision
    LinearMath
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    include

    vendor/raylib/src
    vendor/bullet3/src
)

# Set warning flags
# if (MSVC)
#     target_compile_options(${PROJECT_NAME} PRIVATE /W4)
# else()
#     target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
# endif()
