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



void setup_scr_screen(lv_ui *ui)
{
    //Write codes screen
    ui->screen = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen, 128, 160);
    lv_obj_set_scrollbar_mode(ui->screen, LV_SCROLLBAR_MODE_OFF);

    //Write style for screen, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_background3
    ui->screen_background3 = lv_image_create(ui->screen);
    lv_obj_set_pos(ui->screen_background3, 0, 0);
    lv_obj_set_size(ui->screen_background3, 128, 160);
    lv_obj_add_flag(ui->screen_background3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui->screen_background3, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->screen_background3, &_pinkPlant_RGB565A8_128x160);
    lv_image_set_pivot(ui->screen_background3, 50,50);
    lv_image_set_rotation(ui->screen_background3, 0);

    //Write style for screen_background3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->screen_background3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->screen_background3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_background2
    ui->screen_background2 = lv_image_create(ui->screen);
    lv_obj_set_pos(ui->screen_background2, 0, 0);
    lv_obj_set_size(ui->screen_background2, 128, 160);
    lv_obj_add_flag(ui->screen_background2, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->screen_background2, &_background_dessert_RGB565A8_128x160);
    lv_image_set_pivot(ui->screen_background2, 50,50);
    lv_image_set_rotation(ui->screen_background2, 0);

    //Write style for screen_background2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->screen_background2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->screen_background2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_cominghome
    ui->screen_cominghome = lv_image_create(ui->screen);
    lv_obj_set_pos(ui->screen_cominghome, 0, 0);
    lv_obj_set_size(ui->screen_cominghome, 128, 160);
    lv_obj_add_flag(ui->screen_cominghome, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->screen_cominghome, &_green_layer_RGB565A8_128x160);
    lv_image_set_pivot(ui->screen_cominghome, 50,50);
    lv_image_set_rotation(ui->screen_cominghome, 0);

    //Write style for screen_cominghome, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->screen_cominghome, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->screen_cominghome, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_animimg_ground2
    ui->screen_animimg_ground2 = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_ground2, 0, 0);
    lv_obj_set_size(ui->screen_animimg_ground2, 128, 160);
    lv_animimg_set_src(ui->screen_animimg_ground2, (const void **) screen_animimg_ground2_imgs, 7);
    lv_animimg_set_duration(ui->screen_animimg_ground2, 250*7);
    lv_animimg_set_repeat_count(ui->screen_animimg_ground2, LV_ANIM_REPEAT_INFINITE);
    lv_animimg_start(ui->screen_animimg_ground2);

    //Write codes screen_background
    ui->screen_background = lv_image_create(ui->screen);
    lv_obj_set_pos(ui->screen_background, 0, 0);
    lv_obj_set_size(ui->screen_background, 128, 160);
    lv_obj_add_flag(ui->screen_background, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui->screen_background, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->screen_background, &_background_RGB565A8_128x160);
    lv_image_set_pivot(ui->screen_background, 50,50);
    lv_image_set_rotation(ui->screen_background, 0);

    //Write style for screen_background, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->screen_background, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->screen_background, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_animimg_ground
    ui->screen_animimg_ground = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_ground, 0, 0);
    lv_obj_set_size(ui->screen_animimg_ground, 128, 160);
    lv_obj_add_flag(ui->screen_animimg_ground, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_ground, (const void **) screen_animimg_ground_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_ground, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_ground, LV_ANIM_REPEAT_INFINITE);
    lv_animimg_start(ui->screen_animimg_ground);

    //Write codes screen_state1
    ui->screen_state1 = lv_image_create(ui->screen);
    lv_obj_set_pos(ui->screen_state1, 51, 8);
    lv_obj_set_size(ui->screen_state1, 25, 25);
    lv_obj_add_flag(ui->screen_state1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui->screen_state1, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->screen_state1, &_state_dryl_RGB565A8_25x25);
    lv_image_set_pivot(ui->screen_state1, 50,50);
    lv_image_set_rotation(ui->screen_state1, 0);

    //Write style for screen_state1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->screen_state1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->screen_state1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_state2
    ui->screen_state2 = lv_image_create(ui->screen);
    lv_obj_set_pos(ui->screen_state2, 51, 7);
    lv_obj_set_size(ui->screen_state2, 25, 25);
    lv_obj_add_flag(ui->screen_state2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui->screen_state2, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->screen_state2, &_state_well_RGB565A8_25x25);
    lv_image_set_pivot(ui->screen_state2, 50,50);
    lv_image_set_rotation(ui->screen_state2, 0);

    //Write style for screen_state2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->screen_state2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->screen_state2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_state3
    ui->screen_state3 = lv_image_create(ui->screen);
    lv_obj_set_pos(ui->screen_state3, 51, 8);
    lv_obj_set_size(ui->screen_state3, 25, 25);
    lv_obj_add_flag(ui->screen_state3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui->screen_state3, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->screen_state3, &_state_flood_RGB565A8_25x25);
    lv_image_set_pivot(ui->screen_state3, 50,50);
    lv_image_set_rotation(ui->screen_state3, 0);

    //Write style for screen_state3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->screen_state3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->screen_state3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_animimg_idel_sleep
    ui->screen_animimg_idel_sleep = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_idel_sleep, 41, 98);
    lv_obj_set_size(ui->screen_animimg_idel_sleep, 46, 36);
    lv_obj_add_flag(ui->screen_animimg_idel_sleep, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_idel_sleep, (const void **) screen_animimg_idel_sleep_imgs, 5);
    lv_animimg_set_duration(ui->screen_animimg_idel_sleep, 250*5);
    lv_animimg_set_repeat_count(ui->screen_animimg_idel_sleep, 1);
    lv_animimg_start(ui->screen_animimg_idel_sleep);

    //Write codes screen_animimg_idel_awake
    ui->screen_animimg_idel_awake = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_idel_awake, 41, 98);
    lv_obj_set_size(ui->screen_animimg_idel_awake, 46, 36);
    lv_obj_add_flag(ui->screen_animimg_idel_awake, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui->screen_animimg_idel_awake, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_idel_awake, (const void **) screen_animimg_idel_awake_imgs, 5);
    lv_animimg_set_duration(ui->screen_animimg_idel_awake, 250*5);
    lv_animimg_set_repeat_count(ui->screen_animimg_idel_awake, 1);
    lv_image_set_src(ui->screen_animimg_idel_awake, screen_animimg_idel_awake_imgs[0]);

    //Write codes screen_animimg_idel
    ui->screen_animimg_idel = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_idel, 41, 98);
    lv_obj_set_size(ui->screen_animimg_idel, 46, 36);
    lv_obj_add_flag(ui->screen_animimg_idel, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_idel, (const void **) screen_animimg_idel_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_idel, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_idel, LV_ANIM_REPEAT_INFINITE);
    lv_animimg_start(ui->screen_animimg_idel);

    //Write codes screen_animimg_idel_flood
    ui->screen_animimg_idel_flood = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_idel_flood, 41, 98);
    lv_obj_set_size(ui->screen_animimg_idel_flood, 46, 36);
    lv_animimg_set_src(ui->screen_animimg_idel_flood, (const void **) screen_animimg_idel_flood_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_idel_flood, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_idel_flood, 2);
    lv_animimg_start(ui->screen_animimg_idel_flood);

    //Write codes screen_animimg_exp_hello
    ui->screen_animimg_exp_hello = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_exp_hello, 68, 81);
    lv_obj_set_size(ui->screen_animimg_exp_hello, 28, 26);
    lv_obj_add_flag(ui->screen_animimg_exp_hello, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_exp_hello, (const void **) screen_animimg_exp_hello_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_exp_hello, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_exp_hello, 2);
    lv_animimg_start(ui->screen_animimg_exp_hello);

    //Write codes screen_animimg_exp_sad
    ui->screen_animimg_exp_sad = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_exp_sad, 68, 81);
    lv_obj_set_size(ui->screen_animimg_exp_sad, 28, 26);
    lv_obj_add_flag(ui->screen_animimg_exp_sad, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_exp_sad, (const void **) screen_animimg_exp_sad_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_exp_sad, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_exp_sad, 2);
    lv_animimg_start(ui->screen_animimg_exp_sad);

    //Write codes screen_animimg_exp_happy
    ui->screen_animimg_exp_happy = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_exp_happy, 68, 81);
    lv_obj_set_size(ui->screen_animimg_exp_happy, 28, 26);
    lv_obj_add_flag(ui->screen_animimg_exp_happy, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_exp_happy, (const void **) screen_animimg_exp_happy_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_exp_happy, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_exp_happy, 2);
    lv_animimg_start(ui->screen_animimg_exp_happy);

    //Write codes screen_animimg_exp_sleep
    ui->screen_animimg_exp_sleep = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_exp_sleep, 68, 81);
    lv_obj_set_size(ui->screen_animimg_exp_sleep, 28, 26);
    lv_obj_add_flag(ui->screen_animimg_exp_sleep, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_exp_sleep, (const void **) screen_animimg_exp_sleep_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_exp_sleep, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_exp_sleep, LV_ANIM_REPEAT_INFINITE);
    lv_animimg_start(ui->screen_animimg_exp_sleep);

    //Write codes screen_animimg_exp_flood
    ui->screen_animimg_exp_flood = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_exp_flood, 68, 81);
    lv_obj_set_size(ui->screen_animimg_exp_flood, 28, 26);
    lv_animimg_set_src(ui->screen_animimg_exp_flood, (const void **) screen_animimg_exp_flood_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_exp_flood, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_exp_flood, 2);
    lv_animimg_start(ui->screen_animimg_exp_flood);

    //Write codes screen_animimg_plant_normal
    ui->screen_animimg_plant_normal = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_plant_normal, 44, 50);
    lv_obj_set_size(ui->screen_animimg_plant_normal, 44, 44);
    lv_obj_add_flag(ui->screen_animimg_plant_normal, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_plant_normal, (const void **) screen_animimg_plant_normal_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_plant_normal, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_plant_normal, 2);
    lv_animimg_start(ui->screen_animimg_plant_normal);

    //Write codes screen_animimg_flood
    ui->screen_animimg_flood = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_flood, 0, 47);
    lv_obj_set_size(ui->screen_animimg_flood, 128, 113);
    lv_animimg_set_src(ui->screen_animimg_flood, (const void **) screen_animimg_flood_imgs, 7);
    lv_animimg_set_duration(ui->screen_animimg_flood, 250*7);
    lv_animimg_set_repeat_count(ui->screen_animimg_flood, LV_ANIM_REPEAT_INFINITE);
    lv_animimg_start(ui->screen_animimg_flood);

    //Write codes screen_animimg_exp_cominghome
    ui->screen_animimg_exp_cominghome = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_exp_cominghome, 68, 81);
    lv_obj_set_size(ui->screen_animimg_exp_cominghome, 28, 26);
    lv_animimg_set_src(ui->screen_animimg_exp_cominghome, (const void **) screen_animimg_exp_cominghome_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_exp_cominghome, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_exp_cominghome, LV_ANIM_REPEAT_INFINITE);
    lv_animimg_start(ui->screen_animimg_exp_cominghome);

    //Write codes screen_animimg_idle_cominghome
    ui->screen_animimg_idle_cominghome = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_idle_cominghome, 41, 98);
    lv_obj_set_size(ui->screen_animimg_idle_cominghome, 46, 36);
    lv_obj_add_flag(ui->screen_animimg_idle_cominghome, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_idle_cominghome, (const void **) screen_animimg_idle_cominghome_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_idle_cominghome, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_idle_cominghome, LV_ANIM_REPEAT_INFINITE);
    lv_animimg_start(ui->screen_animimg_idle_cominghome);

    //Write codes screen_animimg_exp_lost
    ui->screen_animimg_exp_lost = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_exp_lost, 68, 81);
    lv_obj_set_size(ui->screen_animimg_exp_lost, 28, 26);
    lv_obj_add_flag(ui->screen_animimg_exp_lost, LV_OBJ_FLAG_HIDDEN);
    lv_animimg_set_src(ui->screen_animimg_exp_lost, (const void **) screen_animimg_exp_lost_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_exp_lost, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_exp_lost, 2);
    lv_animimg_start(ui->screen_animimg_exp_lost);

    //Write codes screen_animimg_lodding
    ui->screen_animimg_lodding = lv_animimg_create(ui->screen);
    lv_obj_set_pos(ui->screen_animimg_lodding, 0, 0);
    lv_obj_set_size(ui->screen_animimg_lodding, 128, 160);
    lv_animimg_set_src(ui->screen_animimg_lodding, (const void **) screen_animimg_lodding_imgs, 4);
    lv_animimg_set_duration(ui->screen_animimg_lodding, 250*4);
    lv_animimg_set_repeat_count(ui->screen_animimg_lodding, LV_ANIM_REPEAT_INFINITE);
    lv_animimg_start(ui->screen_animimg_lodding);

    //The custom code of screen.


    //Update current screen layout.
    lv_obj_update_layout(ui->screen);

}
