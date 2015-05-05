project(omni_kinematics)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

set(OMNI_KINEMATICS_HEADERS
    include/omni_kinematics/common.hpp
    include/omni_kinematics/control.hpp
    include/omni_kinematics/robot.hpp
)

add_library(omni_kinematics SHARED
    src/common.cpp
    src/control.cpp
    src/robot.cpp
)

if ($ENV{OMNI_KINEMATICS_ACC_TEST})
    set_target_properties(omni_kinematics
        PROPERTIES COMPILE_DEFINITIONS OMNI_KINEMATICS_ACC_TEST
    )
endif ($ENV{OMNI_KINEMATICS_ACC_TEST})

install(TARGETS omni_kinematics DESTINATION lib)
install(FILES ${OMNI_KINEMATICS_HEADERS} DESTINATION include/${PROJECT_NAME})
