rock_library(single_leg_planner
    SOURCES
	sherpaTTIK.cpp
	buildRRTWrapper.cpp
	selectInput.cpp
	rand.cpp
	sherpaTTIKVel.cpp
	rtGetNaN.cpp
	buildRRT.cpp
	eml_rand_mt19937ar_stateful.cpp
	nearestNeighbour.cpp
	buildRRTWrapper_rtwutil.cpp
	log.cpp
	sin.cpp
	buildRRTWrapper_data.cpp
	rt_nonfinite.cpp
	rtGetInf.cpp
	buildRRTWrapper_initialize.cpp
	buildRRTWrapper_terminate.cpp
	exp.cpp
	heuristicSingleLeg.cpp
	buildRRTWrapper_emxAPI.cpp
	buildRRTWrapper_emxutil.cpp
	asin.cpp
    HEADERS
	sherpaTTIK.h
	asin.h
	buildRRTWrapper_emxAPI.h
	heuristicSingleLeg.h
	sin.h
	rt_defines.h
	nearestNeighbour.h
	exp.h
	buildRRTWrapper_terminate.h
	rtGetInf.h
	buildRRTWrapper.h
	buildRRTWrapper_rtwutil.h
	buildRRTWrapper_initialize.h
	rt_nonfinite.h
	selectInput.h
	buildRRTWrapper_types.h
	rand.h
	buildRRT.h
	eml_rand_mt19937ar_stateful.h
	log.h
	rtwtypes.h
	sherpaTTIKVel.h
	buildRRTWrapper_emxutil.h
	rtGetNaN.h
	buildRRTWrapper_data.h
)

rock_executable(single_leg_planner_bin main.cpp
    SOURCES LegPlanner.cpp
    HEADERS LegPlanner.h
    DEPS single_leg_planner)

