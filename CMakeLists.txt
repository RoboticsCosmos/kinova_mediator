cmake_minimum_required(VERSION 3.0.0)
project(kinova_mediator VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

set(KINOVA_API_VERSION 2.6.0)

add_compile_definitions(_OS_UNIX)

# set(CMAKE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/cmake/${PROJECT_NAME})
set(CMAKE_INSTALL_DIR share/${PROJECT_NAME}/cmake)

message(STATUS "CMAKE_INSTALL_DIR: ${CMAKE_INSTALL_DIR}")

# check if kinova api exists
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/lib/release/libKortexApiCpp.a)
    Set(FETCHCONTENT_QUIET FALSE)
    message(WARNING "kinova_api not found.")
    message("Downloading file...")
    File(DOWNLOAD 
        https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.6.0/linux_x86_64_gcc_5.4.zip
        ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api.zip 
        SHOW_PROGRESS STATUS DOWNLOAD_RESULT)
    list(GET DOWNLOAD_RESULT 0 DOWNLOAD_RESULT_CODE)
    if(NOT DOWNLOAD_RESULT_CODE EQUAL 0)
        message(FATAL_ERROR "Failed downloading! Error: ${DOWNLOAD_RESULT}.")
    endif()
    message("Creating directory...")
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api)
    message("Extracting file...")
    execute_process(COMMAND unzip ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api.zip -d ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api OUTPUT_QUIET)
    message("Removing file...")
    file(REMOVE ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api.zip)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/lib/release/libKortexApiCpp.a)
        message(FATAL_ERROR "Something went wrong with the download and extraction of kinova_api.")
    endif()
    message("Done.")
endif()

# includes
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  # kinova
  ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include
  ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/client
  ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/common
  ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/messages
  ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/client_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/google
)

# export library
add_library(${PROJECT_NAME} SHARED
  src/mediator.cpp
)

# link libraries
target_link_libraries(${PROJECT_NAME}
  ${orocos_kdl_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/lib/release/libKortexApiCpp.a
  pthread
)

# install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# Make the targets accessible from this packages's install tree
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DIR})

# Generate the CMake config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY AnyNewerVersion)

# Generate the version file accompanying the project's config file
configure_package_config_file(${PROJECT_NAME}-config.cmake.in
                              ${PROJECT_NAME}-config.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_DIR})

# Install the config and configversion
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_DIR})