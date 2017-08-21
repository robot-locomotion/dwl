# - Run Doxygen given a input file and output folder
#
# This module generates the doxygen documentation. For instances it generates the html, latex and
# xml documentation as defined in the input file
#   find_package(UseDoxygen)
#
# Variables you may define are:
#  DOXYFILE_IN - The Doxygen input file.
#                The Default name file is "${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile.in".
#  DOXYFILE_OUTPUT_DIR - Path where the Doxygen output is stored.
#                        The default path is "${CMAKE_CURRENT_BINARY_DIR}/doc".
#
# Copyright (c) 2014-2018 Carlos Mastalli, <carlos.mastalli@iit.it>
# Redistribution and use is allowed according to the terms of the XXX license.


macro(usedoxygen_set_default name value)
  if(NOT DEFINED "${name}")
    set("${name}" "${value}")
  endif()
endmacro()

find_package(Doxygen)


if(DOXYGEN_FOUND)
    usedoxygen_set_default(DOXYFILE_IN "${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile.in")
    usedoxygen_set_default(DOXYFILE_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/doc/")
    file(MAKE_DIRECTORY ${DOXYFILE_OUTPUT_DIR})

    # Set the output file
    set(DOXYFILE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    set_property(DIRECTORY APPEND PROPERTY
                 ADDITIONAL_MAKE_CLEAN_FILES
                 "${DOXYFILE_OUTPUT_DIR}/html:${DOXYFILE_OUTPUT_DIR}/latex:${DOXYFILE_OUTPUT_DIR}/xml")
                 

    # Request to configure the file
    configure_file(${DOXYFILE_IN} ${DOXYFILE} @ONLY)
    message("Doxygen build started")

    # Generate the doxygen documentation
    add_custom_target( swig_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE}
        COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/doc/doxy2swig.py" -q
                            ${DOXYFILE_OUTPUT_DIR}/xml/index.xml
                            ${CMAKE_CURRENT_SOURCE_DIR}/swig/doc.i
        WORKING_DIRECTORY ${DOXYFILE_OUTPUT_DIR}
        COMMENT "Generating API documentation with Doxygen to ${DOXYFILE_OUTPUT_DIR}"
        VERBATIM)
    
    # Installing the doxygen files
    install(DIRECTORY ${DOXYFILE_OUTPUT_DIR} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/share/dwl)
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif()