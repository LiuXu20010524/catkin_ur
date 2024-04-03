cmake_minimum_required(VERSION 2.8.11)
project(Filter)

#should maybe check whether chosen compiler supports c++11

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")
endif(UNIX)

set(CMAKE_DEBUG_POSTFIX "d")
include(GNUInstallDirs)

add_subdirectory(lib)
add_subdirectory(tests)

# Create ConcurrencyConfig.cmake
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/Filter)

include(CMakePackageConfigHelpers)
configure_package_config_file(FilterConfig.cmake.in
                              ${CMAKE_CURRENT_BINARY_DIR}/FilterConfigForInstall.cmake
                              INSTALL_DESTINATION ${ConfigPackageLocation}
                              PATH_VARS CMAKE_INSTALL_INCLUDEDIR)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/FilterConfigVersion.cmake
                                    VERSION 0.1
                                    COMPATIBILITY ExactVersion )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/FilterConfigForInstall.cmake
              DESTINATION ${ConfigPackageLocation}
              RENAME FilterConfig.cmake)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/FilterConfigVersion.cmake
              DESTINATION ${ConfigPackageLocation})

install(EXPORT Filter_Exports NAMESPACE rtb::
                              DESTINATION ${ConfigPackageLocation})

# Uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

