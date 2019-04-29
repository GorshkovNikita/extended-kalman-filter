cmake_minimum_required(VERSION 3.10)
project(extended_kalman_filter)

set(CMAKE_CXX_STANDARD 14)
set(sources src/main.cpp)
set(INCLUDE_DIR /usr/include)


if((NOT ${EIGEN_INCLUDE_DIR}) OR (NOT EXISTS ${EIGEN_INCLUDE_DIR}))
    message("Installing Eigen library...")

    execute_process(COMMAND git submodule update --init -- third_party/eigen
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)  # CACHE PATH "Eigen library include directory"

    # todo: need to understand whether i need it or not
#    install(DIRECTORY ${EIGEN_INCLUDE_DIR} DESTINATION ${INCLUDE_DIR})

#    add_library(Eigen INTERFACE)
#    target_include_directories(Eigen INTERFACE
#            $<BUILD_INTERFACE:${EIGEN_INCLUDE_DIR}/Eigen>
#            $<INSTALL_INTERFACE:Eigen>)

#    install(TARGETS Eigen EXPORT my_export_set DESTINATION ${INCLUDE_DIR})

else()
    message("Eigen library already present")
endif()

add_executable(extended_kalman_filter ${sources})
target_include_directories(extended_kalman_filter PRIVATE ${EIGEN_INCLUDE_DIR})
target_link_libraries(extended_kalman_filter)
