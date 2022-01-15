cmake_minimum_required(VERSION 2.8.12)

project(nanopb C)

set(nanoMY_pb_VERSION_STRING nanopb-0.4.6-dev)
set(nanoMY_pb_SOVERSION 0)

string(REPLACE "nanopb-" "" nanoMY_pb_VERSION ${nanoMY_pb_VERSION_STRING})

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(BUILD_STATIC_LIBS "Build static libraries" ON)

option(nanoMY_pb_BUILD_RUNTIME "Build the headers and libraries needed at runtime" ON)
option(nanoMY_pb_BUILD_GENERATOR "Build the protoc plugin for code generation" ON)
option(nanoMY_pb_MSVC_STATIC_RUNTIME "Link static runtime libraries" ON)

find_program(nanoMY_pb_PROTOC_PATH protoc)
if(NOT EXISTS ${nanoMY_pb_PROTOC_PATH})
    message(FATAL_ERROR "protoc compiler not found")
endif()

if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

include(GNUInstallDirs)

if(MSVC AND nanoMY_pb_MSVC_STATIC_RUNTIME)
    foreach(flag_var
            CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
            CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
        if(${flag_var} MATCHES "/MD")
            string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
        endif(${flag_var} MATCHES "/MD")
    endforeach(flag_var)
endif()

if(NOT DEFINED CMAKE_INSTALL_CMAKEDIR)
    set(CMAKE_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/nanopb")
endif()

find_package(Python REQUIRED COMPONENTS Interpreter)
execute_process(
    COMMAND ${Python_EXECUTABLE} -c
        "import os.path, sys, sysconfig; print(os.path.relpath(sysconfig.get_path('purelib'), start=sys.prefix))"
    OUTPUT_VARIABLE PYTHON_INSTDIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(nanoMY_pb_BUILD_GENERATOR)
    set(generator_protos nanopb)

    foreach(generator_proto IN LISTS generator_protos)
        string(REGEX REPLACE "([^;]+)" "${PROJECT_SOURCE_DIR}/generator/proto/\\1.proto" generator_proto_file "${generator_proto}")
        string(REGEX REPLACE "([^;]+)" "\\1_pb2.py" generator_proto_py_file "${generator_proto}")
        add_custom_command(
            OUTPUT ${generator_proto_py_file}
            COMMAND ${nanoMY_pb_PROTOC_PATH} --python_out=${PROJECT_BINARY_DIR} -I${PROJECT_SOURCE_DIR}/generator/proto ${generator_proto_file}
            DEPENDS ${generator_proto_file}
        )
        add_custom_target("generate_${generator_proto_py_file}" ALL DEPENDS ${generator_proto_py_file})
        install(
            FILES ${PROJECT_BINARY_DIR}/${generator_proto_py_file}
                  ${generator_proto_file}
            DESTINATION ${PYTHON_INSTDIR}/proto/
        )
    endforeach()
endif()

install(FILES generator/proto/_utils.py
        DESTINATION ${PYTHON_INSTDIR}/proto/)

if(WIN32)
    install(
        PROGRAMS
            generator/nanoMY_pb_generator.py
            generator/protoc-gen-nanopb.bat
        DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
else()
    install(
        PROGRAMS
            generator/nanoMY_pb_generator.py
            generator/protoc-gen-nanopb
        DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

if(nanoMY_pb_BUILD_RUNTIME)
    if(BUILD_SHARED_LIBS)
        add_library(protobuf-nanopb SHARED
            pb.h
            MY_pb_common.h
            MY_pb_common.c
            MY_pb_encode.h
            MY_pb_encode.c
            MY_pb_decode.h
            MY_pb_decode.c)
        set_target_properties(protobuf-nanopb PROPERTIES
            SOVERSION ${nanoMY_pb_SOVERSION})
        install(TARGETS protobuf-nanopb EXPORT nanopb-targets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
        target_include_directories(protobuf-nanopb INTERFACE
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        )
    endif()

    if(BUILD_STATIC_LIBS)
        add_library(protobuf-nanopb-static STATIC
            pb.h
            MY_pb_common.h
            MY_pb_common.c
            MY_pb_encode.h
            MY_pb_encode.c
            MY_pb_decode.h
            MY_pb_decode.c)
        set_target_properties(protobuf-nanopb-static PROPERTIES
            OUTPUT_NAME protobuf-nanopb)
        install(TARGETS protobuf-nanopb-static EXPORT nanopb-targets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
        target_include_directories(protobuf-nanopb-static INTERFACE
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        )
    endif()

    configure_file(extra/nanopb-config-version.cmake.in
        nanopb-config-version.cmake @ONLY)

    install(EXPORT nanopb-targets
        DESTINATION ${CMAKE_INSTALL_CMAKEDIR}
        NAMESPACE nanopb::)

    install(FILES extra/nanopb-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/nanopb-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_CMAKEDIR})

    install(FILES pb.h MY_pb_common.h MY_pb_encode.h MY_pb_decode.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
