###############################################################################
# FILES
###############################################################################

FILE(GLOB CMAKE_TOOLCHAINS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cmake *.dox)

INSTALL(FILES ${CMAKE_TOOLCHAINS} DESTINATION share/ecl/cmake/toolchains)

