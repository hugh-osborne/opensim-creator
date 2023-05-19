# install-time: set necessary CPack variables
set(CPACK_PACKAGE_NAME "${OSC_PACKAGE_NAME}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VENDOR "${OSC_AUTHOR}")
set(CPACK_PACKAGE_CONTACT "${OSC_AUTHOR} <${OSC_AUTHOR_EMAIL}>")
set(CPACK_PACKAGE_HOMEPAGE_URL "${OSC_REPO_URL}")
set(CPACK_PACKAGE_DESCRIPTION "${OSC_SOFTWARE_DESCRIPTION}")
set(CPACK_PACKAGE_EXECUTABLES "osc;${OSC_PACKAGE_NAME}")

if (WIN32)
    include(cmake/OpenSimCreatorWindowsPackaging.cmake)
elseif(LINUX)
    include(cmake/OpenSimCreatorLinuxPackaging.cmake)
elseif(APPLE)
    include(cmake/OpenSimCreatorApplePackaging.cmake)
endif()
