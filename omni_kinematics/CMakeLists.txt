include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
add_library(omni_kinematics
    src/common.cpp
    src/control.cpp
    src/robot.cpp
)

if ($ENV{OMNI_KINEMATICS_ACC_TEST})
    set_target_properties(omni_kinematics
        PROPERTIES COMPILE_DEFINITIONS OMNI_KINEMATICS_ACC_TEST
    )
endif ($ENV{OMNI_KINEMATICS_ACC_TEST})

