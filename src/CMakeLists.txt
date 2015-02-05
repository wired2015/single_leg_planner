rock_library(single_leg_planner
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(single_leg_planner_bin Main.cpp
    DEPS single_leg_planner)

