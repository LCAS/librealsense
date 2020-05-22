# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 FRAMOS GmbH.
target_sources(${LRS_TARGET}
        PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/cs-device.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-device.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-sensor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-sensor.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-factory.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-factory.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-timestamp.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-timestamp.h"
        )