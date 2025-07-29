/*
* Copyright 2025 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include <stdio.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"



void setup_scr_screen_1(lv_ui *ui)
{
    //Write codes screen_1
    ui->screen_1 = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_1, 128, 160);
    lv_obj_set_scrollbar_mode(ui->screen_1, LV_SCROLLBAR_MODE_OFF);

    //Write style for screen_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_1_img_1
    ui->screen_1_img_1 = lv_image_create(ui->screen_1);
    lv_obj_set_pos(ui->screen_1_img_1, 0, 0);
    lv_obj_set_size(ui->screen_1_img_1, 128, 160);
    lv_obj_add_flag(ui->screen_1_img_1, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->screen_1_img_1, &_background2_RGB565A8_128x160);
    lv_image_set_pivot(ui->screen_1_img_1, 50,50);
    lv_image_set_rotation(ui->screen_1_img_1, 0);

    //Write style for screen_1_img_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->screen_1_img_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->screen_1_img_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_1_rlottie_1
    ui->screen_1_rlottie_1 = lv_rlottie_create_from_raw(ui->screen_1, 42, 31, (const void *)lottie_Cat);
    lv_obj_set_pos(ui->screen_1_rlottie_1, 42, 97);
    lv_obj_set_size(ui->screen_1_rlottie_1, 42, 31);

    //Write style for screen_1_rlottie_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_1_rlottie_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_1_imgbtn_1
    ui->screen_1_imgbtn_1 = lv_imagebutton_create(ui->screen_1);
    lv_obj_set_pos(ui->screen_1_imgbtn_1, 7, 10);
    lv_obj_set_size(ui->screen_1_imgbtn_1, 22, 20);
    lv_obj_add_flag(ui->screen_1_imgbtn_1, LV_OBJ_FLAG_CHECKABLE);
    lv_imagebutton_set_src(ui->screen_1_imgbtn_1, LV_IMAGEBUTTON_STATE_RELEASED, &_Chest_Open1_RGB565A8_22x20, NULL, NULL);
    lv_imagebutton_set_src(ui->screen_1_imgbtn_1, LV_IMAGEBUTTON_STATE_PRESSED, &_Chest_Open2_RGB565A8_22x20, NULL, NULL);
    lv_imagebutton_set_src(ui->screen_1_imgbtn_1, LV_IMAGEBUTTON_STATE_CHECKED_RELEASED, &_Chest_Open1_RGB565A8_22x20, NULL, NULL);
    lv_imagebutton_set_src(ui->screen_1_imgbtn_1, LV_IMAGEBUTTON_STATE_CHECKED_PRESSED, &_Chest_Open3_RGB565A8_22x20, NULL, NULL);
    ui->screen_1_imgbtn_1_label = lv_label_create(ui->screen_1_imgbtn_1);
    lv_label_set_text(ui->screen_1_imgbtn_1_label, "");
    lv_label_set_long_mode(ui->screen_1_imgbtn_1_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_1_imgbtn_1_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_1_imgbtn_1, 0, LV_STATE_DEFAULT);

    //Write style for screen_1_imgbtn_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_1_imgbtn_1, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_1_imgbtn_1, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_1_imgbtn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_1_imgbtn_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_1_imgbtn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_1_imgbtn_1, Part: LV_PART_MAIN, State: LV_STATE_PRESSED.
    lv_obj_set_style_image_recolor_opa(ui->screen_1_imgbtn_1, 0, LV_PART_MAIN|LV_STATE_PRESSED);
    lv_obj_set_style_image_opa(ui->screen_1_imgbtn_1, 255, LV_PART_MAIN|LV_STATE_PRESSED);
    lv_obj_set_style_text_color(ui->screen_1_imgbtn_1, lv_color_hex(0xFF33FF), LV_PART_MAIN|LV_STATE_PRESSED);
    lv_obj_set_style_text_font(ui->screen_1_imgbtn_1, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_PRESSED);
    lv_obj_set_style_text_opa(ui->screen_1_imgbtn_1, 255, LV_PART_MAIN|LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(ui->screen_1_imgbtn_1, 0, LV_PART_MAIN|LV_STATE_PRESSED);

    //Write style for screen_1_imgbtn_1, Part: LV_PART_MAIN, State: LV_STATE_CHECKED.
    lv_obj_set_style_image_recolor_opa(ui->screen_1_imgbtn_1, 0, LV_PART_MAIN|LV_STATE_CHECKED);
    lv_obj_set_style_image_opa(ui->screen_1_imgbtn_1, 255, LV_PART_MAIN|LV_STATE_CHECKED);
    lv_obj_set_style_text_color(ui->screen_1_imgbtn_1, lv_color_hex(0xFF33FF), LV_PART_MAIN|LV_STATE_CHECKED);
    lv_obj_set_style_text_font(ui->screen_1_imgbtn_1, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_CHECKED);
    lv_obj_set_style_text_opa(ui->screen_1_imgbtn_1, 255, LV_PART_MAIN|LV_STATE_CHECKED);
    lv_obj_set_style_shadow_width(ui->screen_1_imgbtn_1, 0, LV_PART_MAIN|LV_STATE_CHECKED);

    //Write style for screen_1_imgbtn_1, Part: LV_PART_MAIN, State: LV_IMAGEBUTTON_STATE_RELEASED.
    lv_obj_set_style_image_recolor_opa(ui->screen_1_imgbtn_1, 0, LV_PART_MAIN|LV_IMAGEBUTTON_STATE_RELEASED);
    lv_obj_set_style_image_opa(ui->screen_1_imgbtn_1, 255, LV_PART_MAIN|LV_IMAGEBUTTON_STATE_RELEASED);

    //The custom code of screen_1.


    //Update current screen layout.
    lv_obj_update_layout(ui->screen_1);

}
