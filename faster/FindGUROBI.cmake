#Taken (and modified) from https://support.gurobi.com/hc/en-us/articles/360039499751-How-do-I-use-CMake-to-build-Gurobi-C-C-projects-

# Locate Gurobi include directory
find_path(GUROBI_INCLUDE_DIR
  NAMES gurobi_c.h
  HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
  PATH_SUFFIXES include
)
# Prepare a list of possible gurobi versioned library names
set(gurobi_library_names gurobi)
foreach(ver RANGE 90 99)
  list(APPEND gurobi_library_names gurobi${ver})
endforeach()

# Define the possible library names
set(gurobi_library_names
    gurobi
    gurobi120
    gurobi_c
    gurobi120_c
)

# Locate the Gurobi C library
find_library(GUROBI_LIBRARY
  NAMES ${gurobi_library_names}
  HINTS $ENV{GUROBI_HOME}
  PATH_SUFFIXES lib
)
message(STATUS "Found Gurobi C library: ${GUROBI_LIBRARY}")

# Locate the Gurobi C++ library
find_library(GUROBI_CXX_LIBRARY
  NAMES gurobi_c++
  HINTS $ENV{GUROBI_HOME}
  PATH_SUFFIXES lib
)
message(STATUS "Found Gurobi C++ library: ${GUROBI_CXX_LIBRARY}")

# Debug version fallback
if(NOT GUROBI_CXX_DEBUG_LIBRARY)
  set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})
endif()

# Mark variables as advanced (optional cleanup)
mark_as_advanced(
  GUROBI_INCLUDE_DIR
  GUROBI_LIBRARY
  GUROBI_CXX_LIBRARY
  GUROBI_CXX_DEBUG_LIBRARY
)

# Set all the output variables
set(GUROBI_INCLUDE_DIRS ${GUROBI_INCLUDE_DIR})
set(GUROBI_LIBRARIES ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

# Result summary
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI REQUIRED_VARS GUROBI_INCLUDE_DIR GUROBI_LIBRARY GUROBI_CXX_LIBRARY)
