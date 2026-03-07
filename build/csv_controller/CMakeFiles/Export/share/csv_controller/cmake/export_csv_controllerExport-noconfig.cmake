#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "csv_controller::csv_controller" for configuration ""
set_property(TARGET csv_controller::csv_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(csv_controller::csv_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcsv_controller.so"
  IMPORTED_SONAME_NOCONFIG "libcsv_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS csv_controller::csv_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_csv_controller::csv_controller "${_IMPORT_PREFIX}/lib/libcsv_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
