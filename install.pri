# Linux and macx install configuration
GLC_LIB_NAME = GLC_lib-3.0

unix {
    # Location of HEADERS and library
    GLC_LIB_DIR = $$(HOME)/$${GLC_LIB_NAME}
    LIB_DIR = /home/carmen/GLC_LIB_install/lib
    INCLUDE_DIR = /home/carmen/GLC_LIB_install/include/$${GLC_LIB_NAME}
    # Adds a -P to preserve link
        QMAKE_COPY_FILE = $${QMAKE_COPY_FILE} -P
}

# Windows Install configuration
win32 {
    # Location of HEADERS and library
    GLC_LIB_DIR = C:/$${GLC_LIB_NAME}
    LIB_DIR = $${GLC_LIB_DIR}/lib
    INCLUDE_DIR = $${GLC_LIB_DIR}/include
}
