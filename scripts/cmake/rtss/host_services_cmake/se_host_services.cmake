set(SE_HOST_SERVICES_DIR            "${ALIF_DEV_SRC_DIR}/se_services")
set(SE_SERVICE_LIB_INC              "${SE_HOST_SERVICES_DIR}/include;${SE_HOST_SERVICES_DIR}/port/include")
include_directories (${SE_SERVICE_LIB_INC})
file (GLOB_RECURSE  SE_SERVICE_LIB_SRC  "${SE_HOST_SERVICES_DIR}/source/*.c" "${SE_HOST_SERVICES_DIR}/port/*.c")

# Removing Linux files from source file list
get_filename_component (rm_app "${SE_HOST_SERVICES_DIR}/source/services_host_handler_linux.c" ABSOLUTE)
list (REMOVE_ITEM SE_SERVICE_LIB_SRC "${rm_app}")

# Creating a Library file for HOST SERVICES
set (SE_HOST_SERVICES_LIB     "SE_HOST_SERVICE")
add_library (${SE_HOST_SERVICES_LIB} STATIC ${SE_SERVICE_LIB_SRC} ${HOST_DRIVERS_SRC})
