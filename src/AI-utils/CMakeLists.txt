set(app_src
    "activation_func.c"
    "parameters.c"
    "forward.c"
)

idf_component_register(
    SRCS ${app_src}          # Sử dụng biến `app_src` ở đây
    INCLUDE_DIRS "include"   # Thư mục chứa các tệp header
)