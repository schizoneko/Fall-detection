set(app_src "activation_func.c" "parameters.c" "forwards.c")

idf_component_register(SRCS ${app_src}
                    PRIV_REQUIRES 
                    INCLUDE_DIRS ".")