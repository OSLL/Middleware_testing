MACRO (DEFINE_ZCM_SOURCES zcmfilename)
    GET_FILENAME_COMPONENT(it ${zcmfilename} ABSOLUTE)
    GET_FILENAME_COMPONENT(nfile ${zcmfilename} NAME_WE)
    SET(outsources ./include/${nfile}.hpp)
ENDMACRO(DEFINE_ZCM_SOURCES)

MACRO (ZCM_GEN filename)
    GET_FILENAME_COMPONENT(it ${filename} ABSOLUTE)
    GET_FILENAME_COMPONENT(filename ${filename} NAME)
    DEFINE_ZCM_SOURCES(${ARGV})
    ADD_CUSTOM_COMMAND (
            OUTPUT ${outsources}
            COMMAND zcm-gen -x ${filename}.zcm --cpp-hpath ./include
            DEPENDS ${it}.zcm
            COMMENT "Creating msg_t.hpp"
    )
ENDMACRO (ZCM_GEN)
