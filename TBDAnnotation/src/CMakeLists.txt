file(GLOB_RECURSE SRC
    "*.h"
    "*.hpp"
    "*.cpp"
)

add_executable(${TARGET_TBDANNOTATION} ${SRC})

target_include_directories(
  ${TARGET_TBDANNOTATION}
  PRIVATE ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  ${TARGET_TBDANNOTATION}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${X11_LIBRARIES}
  ${SQLITE3_LIBRARY}
)
