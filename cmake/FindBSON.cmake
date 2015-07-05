find_path(BSON_INCLUDE_DIR libbson-1.0/bson.h PATHS "${CMAKE_SOURCE_PREFIX}")
find_library(BSON_LIBRARY NAMES bson-1.0 libbson-1.0 PATHS "${CMAKE_SOURCE_PREFIX}")

set(BSON_INCLUDE_DIR "${BSON_INCLUDE_DIR}/libbson-1.0")

set(BSON_LIBRARIES ${BSON_LIBRARY})
set(BSON_INCLUDE_DIRS ${BSON_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(BSON DEFAULT_MSG BSON_LIBRARY BSON_INCLUDE_DIR)

mark_as_advanced(BSON_INCLUDE_DIR BSON_LIBRARY)