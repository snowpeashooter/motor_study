# SPDX-License-Identifier: GPL-3.0
AUX_SOURCE_DIRECTORY(. SOURCE_LIST_ENCOS_MOTORS)
add_library(encos_motors
        STATIC
        ${SOURCE_LIST_ENCOS_MOTORS}
)
target_include_directories(encos_motors
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(encos_motors PUBLIC ${PUBLIC_DEPENDENCIES} motors_can)
