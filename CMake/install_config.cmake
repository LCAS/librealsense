# Set CMAKE_INSTALL_* if not defined
set(CMAKECONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${LRS_TARGET}")

add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

include(CMakePackageConfigHelpers)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/realsense2-framosConfigVersion.cmake"
    VERSION ${REALSENSE_VERSION_STRING} COMPATIBILITY AnyNewerVersion)

configure_package_config_file(CMake/realsense2-framosConfig.cmake.in realsense2-framosConfig.cmake
    INSTALL_DESTINATION ${CMAKECONFIG_INSTALL_DIR}
    INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/bin
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR
)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
configure_file(config/librealsense.pc.in config/realsense2-framos.pc @ONLY)

install(TARGETS ${LRS_TARGET}
    EXPORT realsense2-framosTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/librealsense2-framos"
)

#if (UNIX)
#     set(SOURCE_DEST src/librealsense2)
# elseif (WIN32)
#     set(SOURCE_DEST src)
# else()
#     set(SOURCE_DEST .)
# endif()

# install(DIRECTORY ${PROJECT_SOURCE_DIR}/.github DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/CMake DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/common DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/config DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/doc DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/examples DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/scripts DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/src DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/third-party DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/tools DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/unit-tests DESTINATION ${SOURCE_DEST})
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/wrappers DESTINATION ${SOURCE_DEST})
# install(FILES 
#     ${PROJECT_SOURCE_DIR}/.gitignore 
#     ${PROJECT_SOURCE_DIR}/.travis.yml 
#     ${PROJECT_SOURCE_DIR}/appveyor.yml
#     ${PROJECT_SOURCE_DIR}/cmake_uninstall.cmake
#     ${PROJECT_SOURCE_DIR}/CMakeLists.txt
#     ${PROJECT_SOURCE_DIR}/code-of-conduct.md
#     ${PROJECT_SOURCE_DIR}/CONTRIBUTING.md
#     ${PROJECT_SOURCE_DIR}/LICENSE
#     ${PROJECT_SOURCE_DIR}/NOTICE
#     ${PROJECT_SOURCE_DIR}/package.xml
#     ${PROJECT_SOURCE_DIR}/readme.md
#     ${PROJECT_SOURCE_DIR}/d400e_api_extensions.md
#     DESTINATION ${SOURCE_DEST}
# )

if (UNIX)
    install(FILES
        ${PROJECT_SOURCE_DIR}/config/99-realsense-framos-libusb.rules 
        DESTINATION /etc/udev/rules.d/
        CONFIGURATIONS RELEASE
    )
endif()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/librealsense2-framos
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT realsense2-framosTargets
        FILE realsense2-framosTargets.cmake
        NAMESPACE ${LRS_TARGET}::
        DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/realsense2-framosConfig.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/realsense2-framosConfigVersion.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

install(CODE "execute_process(COMMAND ldconfig)")

# Set library pkgconfig file for facilitating 3rd party integration
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config/realsense2-framos.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
)

set(CPACK_PACKAGE_NAME "FRAMOS-librealsense2")
set(CPACK_DEBIAN_PACKAGE_RECOMMENDS "freeglut3")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "FRAMOS GmbH")
set(CPACK_PACKAGE_VERSION_MAJOR 2)
set(CPACK_PACKAGE_VERSION_MINOR 29)
set(CPACK_PACKAGE_VERSION_PATCH 8)

# Workaround for path length limitation of NSIS (260 characters)
if (WIN32)
    set(CPACK_PACKAGE_DIRECTORY C:/temp)
    set(CPACK_OUTPUT_FILE_PREFIX ${PROJECT_BINARY_DIR})
endif()

set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA ${PROJECT_SOURCE_DIR}/postinst)

include(CPack)
