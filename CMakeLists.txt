cmake_minimum_required(VERSION 3.6)
project(extended_kalman_filter)

set(CMAKE_CXX_STANDARD 17)
set(sources src/Main.cpp src/EKFProtocol.cpp src/Tools.cpp src/FusionEKF.cpp src/KalmanFilter.cpp)
set(INCLUDE_DIR src) #  /usr/include
set(CMAKE_INSTALL_LIBDIR /usr/lib)
message(${CMAKE_CURRENT_BINARY_DIR})

#set(CXX_FLAGS "-Wall")
#set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

add_definitions(-DLIBUS_NO_SSL -DLIBUS_NO_SSL -DUWS_NO_ZLIB)

if((NOT ${EIGEN_INCLUDE_DIR}) OR (NOT EXISTS ${EIGEN_INCLUDE_DIR}))
    message("Installing Eigen library...")

    execute_process(COMMAND git submodule update --init -- third_party/eigen
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)  # CACHE PATH "Eigen library include directory"

else()
    message("Eigen library already present")
endif()


if((NOT ${JSON_INCLUDE_DIR}) OR (NOT EXISTS ${JSON_INCLUDE_DIR}))
    message("Installing Json library...")

    execute_process(COMMAND git submodule update --init -- third_party/json
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    set(JSON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/json/single_include/nlohmann)

else()
    message("Json library already present")
endif()

if((NOT ${USOCKETS_INCLUDE_DIR}) OR (NOT EXISTS ${USOCKETS_INCLUDE_DIR}))
    message("Installing uSockets library...")

    execute_process(COMMAND git submodule update --init -- third_party/uSockets
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    set(
        USOCKETS_SOURCES
        third_party/uSockets/src/loop.c
        third_party/uSockets/src/ssl.c
        third_party/uSockets/src/context.c
        third_party/uSockets/src/socket.c
        third_party/uSockets/src/eventing/epoll.c
        third_party/uSockets/src/eventing/libuv.c
    )
    set(USOCKETS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/uSockets/src)
#    set(USOCKETS_INCLUDE_DIR_TWO ${CMAKE_CURRENT_SOURCE_DIR}/third_party/uSockets/src/interfaces)

else()
    message("uSockets library already present")
endif()


if((NOT ${UWS_INCLUDE_DIR}) OR (NOT EXISTS ${UWS_INCLUDE_DIR}))
    message("Installing uWebSockets library...")

    execute_process(COMMAND git submodule update --init -- third_party/uWebSockets
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    set(UWS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/uWebSockets/src)

else()
    message("uWebSockets library already present")
endif()


if((NOT ${LIBUV_INCLUDE_DIR}) OR (NOT EXISTS ${LIBUV_INCLUDE_DIR}))
    message("Installing libuv library...")

    execute_process(COMMAND git submodule update --init -- third_party/libuv
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    add_subdirectory(third_party/libuv)
	if (WIN32)
		# todo: работает только для windows, подумать как сделать для linux
		message(${CMAKE_CURRENT_BINARY_DIR})
		configure_file(
				${CMAKE_CURRENT_BINARY_DIR}/third_party/libuv/uv.dll
				${CMAKE_CURRENT_BINARY_DIR}/uv.dll
				COPYONLY
		)
	endif()
    set(LIBUV_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libuv/include)

else()
    message("libuv library already present")
endif()

add_library(uSockets STATIC ${USOCKETS_SOURCES})
add_executable(extended_kalman_filter ${sources})
target_include_directories(uSockets PRIVATE ${USOCKETS_INCLUDE_DIR} ${LIBUV_INCLUDE_DIR})
target_include_directories(
        extended_kalman_filter PRIVATE
        ${INCLUDE_DIR} ${EIGEN_INCLUDE_DIR} ${JSON_INCLUDE_DIR} ${UWS_INCLUDE_DIR} ${USOCKETS_INCLUDE_DIR}
)
target_link_directories(extended_kalman_filter PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/third_party/libuv)
target_link_libraries(extended_kalman_filter uSockets uv) #  ssl z
