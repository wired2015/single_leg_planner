file (GLOB SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")
file(GLOB HDRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.h")
rock_library(single_leg_planner
SOURCES ${SRCS}
HEADERS ${HDRS}
)
