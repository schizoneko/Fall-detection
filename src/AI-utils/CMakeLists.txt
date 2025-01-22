set(app_src
    "activation_func.c"
    "parameters.c"
    "forward.c"
)

idf_component_register(
    SRCS ${app_src}
    INCLUDE_DIRS "include"    
)