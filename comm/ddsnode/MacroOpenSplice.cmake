##############################################################################
# OpenSplice_IDLGEN(idlfilename)
#
# Macro to generate OpenSplice DDS sources from a given idl file with the 
# data structures.
# You must include the extension .idl in the name of the data file.
#
##############################################################################
# Courtersy of Ivan Galvez Junquera <ivgalvez@gmail.com>
##############################################################################


# Macro to create a list with all the generated source files for a given .idl filename
MACRO (DEFINE_OpenSplice_SOURCES idlfilename)
	SET(outsources)
	GET_FILENAME_COMPONENT(it ${idlfilename} ABSOLUTE)
	GET_FILENAME_COMPONENT(nfile ${idlfilename} NAME_WE)
	SET(outsources ${outsources} gen/${nfile}.cpp gen/${nfile}.h)
	SET(outsources ${outsources} gen/${nfile}Dcps.cpp gen/${nfile}Dcps.h)
	SET(outsources ${outsources} gen/${nfile}Dcps_impl.cpp gen/${nfile}Dcps_impl.h)
	SET(outsources ${outsources} gen/${nfile}SplDcps.cpp gen/${nfile}SplDcps.h)
	SET(outsources ${outsources} gen/ccpp_${nfile}.h)
ENDMACRO(DEFINE_OpenSplice_SOURCES)

MACRO (OpenSplice_IDLGEN idlfilename)
        message(!${OpenSplice_IDLGEN_BINARY}!)
	GET_FILENAME_COMPONENT(it ${idlfilename} ABSOLUTE)
	GET_FILENAME_COMPONENT(idlfilename ${idlfilename} NAME)
	DEFINE_OpenSplice_SOURCES(${ARGV})
	ADD_CUSTOM_COMMAND (
		OUTPUT ${outsources}
		COMMAND ${OpenSplice_IDLGEN_BINARY}
                ARGS -I $ENV{OSPL_HOME}/etc/idl/ -l isocpp -d gen ${idlfilename}
		DEPENDS ${it}
	)
ENDMACRO (OpenSplice_IDLGEN)
