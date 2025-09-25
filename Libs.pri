LIB_DIR=$${ROOT_DIR}/lib

exists(LocalLibs.pri) {
    include(LocalLibs.pri)
}

!defined(HELPER_LIB_DIR, var) {
    HELPER_DIR=$${LIB_DIR}/Helper
    HELPER_LIB_DIR = $${HELPER_DIR}/$${BUILD_TYPE}
    HELPER_INCLUDE_DIR = $${HELPER_DIR}/include
}
