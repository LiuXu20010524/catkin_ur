#Author: Elena Ceseracciu

set(Filter_HEADERS rtb/Filter/Filter.h
                   rtb/Filter/StateSpaceFilter.h
                   rtb/Filter/TransferFunction.h
                   rtb/Filter/Polynomial.h
                   rtb/Filter/Designer.h)

set(Filter_TEMPLATE_IMPLEMENTATIONS rtb/Filter/Filter.cpp
                                    rtb/Filter/StateSpaceFilter.cpp
                                    rtb/Filter/Polynomial.cpp
                                    rtb/Filter/TransferFunction.cpp
)

set_source_files_properties(${Filter_TEMPLATE_IMPLEMENTATIONS} PROPERTIES HEADER_FILE_ONLY TRUE)

source_group("Header files" FILES ${Filter_HEADERS})
source_group("Implementation files" FILES ${Filter_TEMPLATE_IMPLEMENTATIONS})

add_library(Filter INTERFACE)

target_compile_definitions(Filter INTERFACE "_USE_MATH_DEFINES")

target_include_directories(Filter INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

add_custom_target(Filter_IDE SOURCES ${Filter_HEADERS} ${Filter_TEMPLATE_IMPLEMENTATIONS})

install(FILES ${Filter_HEADERS} ${Filter_TEMPLATE_IMPLEMENTATIONS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rtb/Filter)

install(TARGETS Filter EXPORT Filter_Exports DESTINATION ${CMAKE_INSTALL_LIBDIR})
