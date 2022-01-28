
IF(ENABLE_DEBUG)
    ADD_DEFINITIONS(-DDEBUG_ENABLED)
ENDIF(ENABLE_DEBUG)

set(srcs "main.c"
        "bmp180.c"
        "i2c/i2c.c"
        )
set(incs "."
         "i2c"
        )


idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${incs}")
