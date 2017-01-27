file(GLOB SRC
    "*.h"
    "*.hpp"
    "*.cpp"
)

add_executable(${TARGET_TBDCALIBRATION} ${SRC})

target_include_directories(
  ${TARGET_TBDCALIBRATION}
  PRIVATE ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  ${TARGET_TBDCALIBRATION}
  ${OpenCV_LIBS}
)
