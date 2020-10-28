cmake_minimum_required (VERSION 2.6)
set(CMAKE_MODULE_PATH ../cmake)
project (dds-testing)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include (MacroOpenSplice)
include (osmacros)

find_package (OpenSplice REQUIRED)


include_directories(
  ${PROJECT_SOURCE_DIR}
  ${OpenSplice_INCLUDE_DIRS}
)


set (idls TestData.idl)

foreach(idl ${idls})
	OpenSplice_IDLGEN (${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1.cpp" VARS_1 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1.h" VARS_2 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1_DCPS.h" VARS_4 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1SplDcps.cpp" VARS_7 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1SplDcps.h" VARS_8 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/ccpp_\\1.h" VARS_9 ${idl})
	set(OpenSplice_DATAMODEL ${OpenSplice_DATAMODEL} ${VARS_1} ${VARS_2} ${VARS_3} ${VARS_4} ${VARS_5} ${VARS_6} ${VARS_7} ${VARS_8} ${VARS_9})
endforeach(idl)

set (NODE_SOURCES
        opensplice_node.cpp
)

set (NODE_EXE opensplice_node)
set (DATAMODEL datamodel)

add_executable (${NODE_EXE} ${NODE_SOURCES})
add_library (${DATAMODEL} SHARED ${OpenSplice_DATAMODEL})

add_definitions (
 ${OpenSplice_DEFINITIONS}
 ${DEFINITIONS}
)

target_link_libraries (
  ${DATAMODEL} 
  ${OpenSplice_LIBRARIES}
)

target_link_libraries (${NODE_EXE}
 ${OpenSplice_LIBRARIES}
 ${DATAMODEL} pthread
)

