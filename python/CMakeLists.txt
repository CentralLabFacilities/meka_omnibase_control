cmake_minimum_required(VERSION 2.8)
project(meka_omnibase_control-python)

file(GLOB ${PROJECT_NAME}_m3
    "*.py"
)

set(M3SCRIPTS
    meka_omnibase_calibrate.py
    meka_omnibase_caster_tuning.py
)

find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp REQUIRED)

execute_process ( 
   COMMAND ${PYTHON_EXECUTABLE} -c 
   	"import site, sys; sys.stdout.write(site.PREFIXES[-1])" 
   OUTPUT_VARIABLE PYTHON_PREFIX 
) 
file ( TO_CMAKE_PATH "${PYTHON_PREFIX}" PYTHON_PREFIX ) 
execute_process ( 
   COMMAND ${PYTHON_EXECUTABLE} -c 
   	"import site, sys; sys.stdout.write(site.getsitepackages()[-1])" 
   OUTPUT_VARIABLE PYTHON_SITE_DIR 
) 

file ( TO_CMAKE_PATH "${PYTHON_SITE_DIR}" PYTHON_SITE_DIR )
string ( REGEX REPLACE "^${PYTHON_PREFIX}/" "" 
   PYTHON_SITE_DIR "${PYTHON_SITE_DIR}" 
)
set(M3MEKA_PYTHON_DIR ${PYTHON_SITE_DIR} PARENT_SCOPE)
install (
	FILES ${${PROJECT_NAME}_m3} 
	PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ WORLD_EXECUTE
	DESTINATION ${PYTHON_SITE_DIR}/m3
	COMPONENT library
)

install (
	FILES ${M3SCRIPTS} meka_omnibase_control_pb2.py
	PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ WORLD_EXECUTE
	DESTINATION bin
)
