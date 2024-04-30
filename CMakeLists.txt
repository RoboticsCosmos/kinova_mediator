cmake_minimum_required(VERSION 3.0.0)
project(kinova_mediator VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(_OS_UNIX)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

set(KINOVA_API_VERSION 2.6.0)

add_compile_definitions(_OS_UNIX)

set(CMAKE_INSTALL_DIR share/${PROJECT_NAME}/cmake)

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

# install the kinova_api headers to the include directory under kinova_api
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/
  DESTINATION include/kinova_api
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# install the kinova_api library to the lib directory
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/lib/release/libKortexApiCpp.a
  DESTINATION lib
)

list(APPEND local_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/)

# includes
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${local_INCLUDE_DIRS}
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
  ${CMAKE_INSTALL_PREFIX}/lib/libKortexApiCpp.a
  pthread
)

# install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Generate the CMake config file
include(CMakePackageConfigHelpers)
# Generate the version file accompanying the project's config file
configure_package_config_file(${PROJECT_NAME}-config.cmake.in
                              ${PROJECT_NAME}-config.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_DIR})

write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY AnyNewerVersion)

# Install the config and configversion
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_DIR})

# Make the targets accessible from this packages's build tree
export(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
)

# Make the targets accessible from this packages's install tree
install(
  EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DIR})

# Install all public header files
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN
)