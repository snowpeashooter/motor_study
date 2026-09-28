add_library(dummy_motors STATIC dummy_motor_driver.cpp)
target_include_directories(dummy_motors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dummy_motors PUBLIC ${PUBLIC_DEPENDENCIES})
