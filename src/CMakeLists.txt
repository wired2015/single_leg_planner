file(GLOB SRCS "${PROJECT_SOURCE_DIR}/src/*.cpp")
file(GLOB HDRS "${PROJECT_SOURCE_DIR}/src/*.h")

rock_library(single_leg_planner
SOURCES ${SRCS}
HEADERS ${HDRS}
)


