// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLineProjectAlarm

#include "../ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Screen1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0xABA492), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label1 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, -7);
    lv_obj_set_y(ui_Label1, -190);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "Emergency Notification");

    ui_Checkbox3 = lv_checkbox_create(ui_Screen1);
    lv_checkbox_set_text(ui_Checkbox3, "Test Signal");
    lv_obj_set_width(ui_Checkbox3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox3, -95);
    lv_obj_set_y(ui_Checkbox3, -115);
    lv_obj_set_align(ui_Checkbox3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_Label3 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label3, -106);
    lv_obj_set_y(ui_Label3, -147);
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "Type signal");

    ui_ButtonAlarmOn = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_ButtonAlarmOn, 289);
    lv_obj_set_height(ui_ButtonAlarmOn, 50);
    lv_obj_set_x(ui_ButtonAlarmOn, -3);
    lv_obj_set_y(ui_ButtonAlarmOn, -15);
    lv_obj_set_align(ui_ButtonAlarmOn, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ButtonAlarmOn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_ButtonAlarmOn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label4 = lv_label_create(ui_ButtonAlarmOn);
    lv_obj_set_width(ui_Label4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label4, 3);
    lv_obj_set_y(ui_Label4, 0);
    lv_obj_set_align(ui_Label4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label4, "Alarm ON");

    ui_ButtonAlarmOff = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_ButtonAlarmOff, 289);
    lv_obj_set_height(ui_ButtonAlarmOff, 50);
    lv_obj_set_x(ui_ButtonAlarmOff, -1);
    lv_obj_set_y(ui_ButtonAlarmOff, 51);
    lv_obj_set_align(ui_ButtonAlarmOff, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ButtonAlarmOff, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_ButtonAlarmOff, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label2 = lv_label_create(ui_ButtonAlarmOff);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, 3);
    lv_obj_set_y(ui_Label2, 0);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "Alarm OFF");

    ui_Label5 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label5, -129);
    lv_obj_set_y(ui_Label5, -70);
    lv_obj_set_align(ui_Label5, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label5, "Time:");

    ui_lblTime = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_lblTime, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblTime, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblTime, 9);
    lv_obj_set_y(ui_lblTime, -69);
    lv_obj_set_align(ui_lblTime, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblTime, "Value");

}
