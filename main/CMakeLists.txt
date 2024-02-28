idf_component_register(
SRCS "main.cpp" 
"ui/ui.c" 
"ui/ui_helpers.c"
"ui/screens/ui_Screen1.c" 
"ui/components/ui_comp_hook.c" 
INCLUDE_DIRS "" "ui"
REQUIRES LovyanGFX lvgl
)

