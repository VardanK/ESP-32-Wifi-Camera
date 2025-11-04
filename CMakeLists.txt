cmake_minimum_required(VERSION 3.16.0)

if(NOT DEFINED IDF_PATH)
    set(IDF_PATH "" CACHE PATH "Path to ESP-IDF (used when IDF_PATH env var is absent)")
endif()

if(NOT DEFINED ENV{IDF_PATH} OR "$ENV{IDF_PATH}" STREQUAL "")
    set(_idf_candidates "")

    if(IDF_PATH)
        list(APPEND _idf_candidates "${IDF_PATH}")
    endif()

    list(APPEND _idf_candidates
        "${CMAKE_SOURCE_DIR}/esp-idf"
        "$ENV{HOME}/esp-idf"
        "$ENV{HOME}/esp/esp-idf"
        "$ENV{USERPROFILE}/esp-idf"
        "$ENV{USERPROFILE}/esp/esp-idf"
    )

    if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
        file(GLOB _home_idf_dirs "$ENV{HOME}/esp/*/esp-idf")
        list(APPEND _idf_candidates ${_home_idf_dirs})
    endif()

    if(DEFINED ENV{USERPROFILE} AND NOT "$ENV{USERPROFILE}" STREQUAL "")
        file(GLOB _userprofile_idf_dirs "$ENV{USERPROFILE}/esp/*/esp-idf")
        list(APPEND _idf_candidates ${_userprofile_idf_dirs})
    endif()

    list(REMOVE_DUPLICATES _idf_candidates)

    foreach(_idf_path IN LISTS _idf_candidates)
        if(NOT _idf_path)
            continue()
        endif()
        file(TO_CMAKE_PATH "${_idf_path}" _idf_path_normalized)
        if(EXISTS "${_idf_path_normalized}/tools/cmake/project.cmake")
            set(ENV{IDF_PATH} "${_idf_path_normalized}")
            set(IDF_PATH "${_idf_path_normalized}" CACHE PATH "Path to ESP-IDF" FORCE)
            message(STATUS "Using ESP-IDF located at ${_idf_path_normalized}")
            break()
        endif()
    endforeach()
endif()

if(NOT DEFINED ENV{IDF_PATH} OR "$ENV{IDF_PATH}" STREQUAL "")
    message(FATAL_ERROR
        "ESP-IDF not found. Set the IDF_PATH environment variable or rerun CMake "
        "with -DIDF_PATH=/absolute/path/to/esp-idf (run ESP-IDF's export script first if needed).")
endif()

# Ensure the Xtensa toolchain binaries are on PATH when the ESP-IDF environment
# hasn't been exported in the current shell session.
set(_toolchain_bin_dirs "")
if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
    file(GLOB _idf_tool_bins
        "$ENV{HOME}/.espressif/tools/xtensa-esp32-elf/*/xtensa-esp32-elf/bin"
        "$ENV{HOME}/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin"
    )
    list(APPEND _toolchain_bin_dirs ${_idf_tool_bins})
endif()
if(DEFINED ENV{USERPROFILE} AND NOT "$ENV{USERPROFILE}" STREQUAL "")
    file(GLOB _idf_tool_bins_win
        "$ENV{USERPROFILE}/.espressif/tools/xtensa-esp32-elf/*/xtensa-esp32-elf/bin"
        "$ENV{USERPROFILE}/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin"
    )
    list(APPEND _toolchain_bin_dirs ${_idf_tool_bins_win})
endif()
list(REMOVE_DUPLICATES _toolchain_bin_dirs)

if(WIN32)
    set(_idf_path_sep ";")
else()
    set(_idf_path_sep ":")
endif()

set(_selected_toolchain_bin "")
foreach(_bin_dir IN LISTS _toolchain_bin_dirs)
    if(EXISTS "${_bin_dir}/xtensa-esp32-elf-gcc")
        set(_selected_toolchain_bin "${_bin_dir}")
        string(REPLACE "\\" "\\\\" _bin_dir_escaped "${_bin_dir}")
        if(WIN32)
            set(_path_match_pattern "(^|;)${_bin_dir_escaped}(;|$)")
        else()
            set(_path_match_pattern "(^|:)${_bin_dir_escaped}(:|$)")
        endif()
        if(NOT "$ENV{PATH}" MATCHES "${_path_match_pattern}")
            if("$ENV{PATH}" STREQUAL "")
                set(ENV{PATH} "${_bin_dir}")
            else()
                set(ENV{PATH} "${_bin_dir}${_idf_path_sep}$ENV{PATH}")
            endif()
        endif()
        break()
    endif()
endforeach()

if(_selected_toolchain_bin)
    set(_xtensa_gcc "${_selected_toolchain_bin}/xtensa-esp32-elf-gcc")
    if(EXISTS "${_xtensa_gcc}")
        set(ENV{CC} "${_xtensa_gcc}")
        if(EXISTS "${_selected_toolchain_bin}/xtensa-esp32-elf-g++")
            set(ENV{CXX} "${_selected_toolchain_bin}/xtensa-esp32-elf-g++")
        endif()
    endif()
endif()

if(NOT DEFINED ENV{PYTHON} OR "$ENV{PYTHON}" STREQUAL "" OR NOT EXISTS "$ENV{PYTHON}")
    set(_python_candidates "")
    if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
        file(GLOB _python_envs "$ENV{HOME}/.espressif/python_env/*/bin/python")
        list(APPEND _python_candidates ${_python_envs})
    endif()
    if(DEFINED ENV{USERPROFILE} AND NOT "$ENV{USERPROFILE}" STREQUAL "")
        file(GLOB _python_envs_win "$ENV{USERPROFILE}/.espressif/python_env/*/Scripts/python.exe")
        list(APPEND _python_candidates ${_python_envs_win})
    endif()
    list(REMOVE_DUPLICATES _python_candidates)

    foreach(_python_candidate IN LISTS _python_candidates)
        if(EXISTS "${_python_candidate}")
            set(ENV{PYTHON} "${_python_candidate}")
            set(PYTHON "${_python_candidate}" CACHE FILEPATH "Python interpreter for ESP-IDF" FORCE)
            break()
        endif()
    endforeach()
endif()

if(NOT DEFINED ENV{PYTHON} OR "$ENV{PYTHON}" STREQUAL "")
    find_program(_system_python NAMES python3 python)
    if(_system_python)
        set(ENV{PYTHON} "${_system_python}")
        set(PYTHON "${_system_python}" CACHE FILEPATH "Python interpreter for ESP-IDF" FORCE)
    endif()
endif()

set(_esp_idf_python "$ENV{PYTHON}")

set(ESP_IDF_PATH "$ENV{IDF_PATH}")
set(ESP_IDF_TOOLCHAIN_BIN "${_selected_toolchain_bin}")
set(ESP_IDF_PYTHON "${_esp_idf_python}")

set(_make_program_names "")
if("${CMAKE_GENERATOR}" STREQUAL "Unix Makefiles")
    set(_make_program_names make gmake)
elseif("${CMAKE_GENERATOR}" STREQUAL "Ninja")
    set(_make_program_names ninja ninja-build)
endif()

if(_make_program_names)
    find_program(_make_real NAMES ${_make_program_names} REQUIRED)
    set(ESP_IDF_REAL_MAKE "${_make_real}")
    set(_idf_make_wrapper "${CMAKE_BINARY_DIR}/idf_make_wrapper.sh")
    configure_file("${CMAKE_SOURCE_DIR}/cmake/idf_make_wrapper.in" "${_idf_make_wrapper}" @ONLY)
    file(CHMOD "${_idf_make_wrapper}"
         PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
    set(CMAKE_MAKE_PROGRAM "${_idf_make_wrapper}" CACHE FILEPATH "ESP-IDF make wrapper" FORCE)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(WifiCamera)
