/*
* Copyright 2025 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#ifndef GUI_GUIDER_H
#define GUI_GUIDER_H
#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"


typedef struct
{
  
	lv_obj_t *screen;
	bool screen_del;
	lv_obj_t *screen_background3;
	lv_obj_t *screen_background2;
	lv_obj_t *screen_cominghome;
	lv_obj_t *screen_animimg_ground2;
	lv_obj_t *screen_background;
	lv_obj_t *screen_animimg_ground;
	lv_obj_t *screen_state1;
	lv_obj_t *screen_state2;
	lv_obj_t *screen_state3;
	lv_obj_t *screen_animimg_idel_sleep;
	lv_obj_t *screen_animimg_idel_awake;
	lv_obj_t *screen_animimg_idel;
	lv_obj_t *screen_animimg_idel_flood;
	lv_obj_t *screen_animimg_exp_hello;
	lv_obj_t *screen_animimg_exp_sad;
	lv_obj_t *screen_animimg_exp_happy;
	lv_obj_t *screen_animimg_exp_sleep;
	lv_obj_t *screen_animimg_exp_flood;
	lv_obj_t *screen_animimg_plant_normal;
	lv_obj_t *screen_animimg_flood;
	lv_obj_t *screen_animimg_exp_cominghome;
	lv_obj_t *screen_animimg_idle_cominghome;
	lv_obj_t *screen_animimg_exp_lost;
	lv_obj_t *screen_animimg_lodding;
}lv_ui;

typedef void (*ui_setup_scr_t)(lv_ui * ui);

void ui_init_style(lv_style_t * style);

void ui_load_scr_animation(lv_ui *ui, lv_obj_t ** new_scr, bool new_scr_del, bool * old_scr_del, ui_setup_scr_t setup_scr,
                           lv_screen_load_anim_t anim_type, uint32_t time, uint32_t delay, bool is_clean, bool auto_del);

void ui_animation(void * var, uint32_t duration, int32_t delay, int32_t start_value, int32_t end_value, lv_anim_path_cb_t path_cb,
                  uint32_t repeat_cnt, uint32_t repeat_delay, uint32_t playback_time, uint32_t playback_delay,
                  lv_anim_exec_xcb_t exec_cb, lv_anim_start_cb_t start_cb, lv_anim_completed_cb_t ready_cb, lv_anim_deleted_cb_t deleted_cb);


void init_scr_del_flag(lv_ui *ui);

void setup_bottom_layer(void);

void setup_ui(lv_ui *ui);

void video_play(lv_ui *ui);

void init_keyboard(lv_ui *ui);

extern lv_ui guider_ui;


void setup_scr_screen(lv_ui *ui);
LV_IMAGE_DECLARE(_pinkPlant_RGB565A8_128x160);
LV_IMAGE_DECLARE(_background_dessert_RGB565A8_128x160);
LV_IMAGE_DECLARE(_green_layer_RGB565A8_128x160);
LV_IMAGE_DECLARE(screen_animimg_ground2dessert_cloud1);
LV_IMAGE_DECLARE(screen_animimg_ground2dessert_cloud2);
LV_IMAGE_DECLARE(screen_animimg_ground2dessert_cloud3);
LV_IMAGE_DECLARE(screen_animimg_ground2dessert_cloud4);
LV_IMAGE_DECLARE(screen_animimg_ground2dessert_cloud5);
LV_IMAGE_DECLARE(screen_animimg_ground2dessert_cloud6);
LV_IMAGE_DECLARE(screen_animimg_ground2dessert_cloud7);
LV_IMAGE_DECLARE(_background_RGB565A8_128x160);
LV_IMAGE_DECLARE(screen_animimg_grounddeceration1);
LV_IMAGE_DECLARE(screen_animimg_grounddeceration2);
LV_IMAGE_DECLARE(screen_animimg_grounddeceration3);
LV_IMAGE_DECLARE(screen_animimg_grounddeceration4);
LV_IMAGE_DECLARE(_state_dryl_RGB565A8_25x25);
LV_IMAGE_DECLARE(_state_well_RGB565A8_25x25);
LV_IMAGE_DECLARE(_state_flood_RGB565A8_25x25);
LV_IMAGE_DECLARE(screen_animimg_idel_sleepfallAsleep1);
LV_IMAGE_DECLARE(screen_animimg_idel_sleepfallAsleep2);
LV_IMAGE_DECLARE(screen_animimg_idel_sleepfallAsleep3);
LV_IMAGE_DECLARE(screen_animimg_idel_sleepfallAsleep4);
LV_IMAGE_DECLARE(screen_animimg_idel_sleepfallAsleep5);
LV_IMAGE_DECLARE(screen_animimg_idel_awakeawake1);
LV_IMAGE_DECLARE(screen_animimg_idel_awakeawake2);
LV_IMAGE_DECLARE(screen_animimg_idel_awakeawake3);
LV_IMAGE_DECLARE(screen_animimg_idel_awakeawake4);
LV_IMAGE_DECLARE(screen_animimg_idel_awakeawake5);
LV_IMAGE_DECLARE(screen_animimg_idelidle1);
LV_IMAGE_DECLARE(screen_animimg_idelidle2);
LV_IMAGE_DECLARE(screen_animimg_idelidle3);
LV_IMAGE_DECLARE(screen_animimg_idelidle4);
LV_IMAGE_DECLARE(screen_animimg_idel_floodidle_uncomfort1);
LV_IMAGE_DECLARE(screen_animimg_idel_floodidle_uncomfort2);
LV_IMAGE_DECLARE(screen_animimg_idel_floodidle_uncomfort3);
LV_IMAGE_DECLARE(screen_animimg_idel_floodidle_uncomfort4);
LV_IMAGE_DECLARE(screen_animimg_exp_hellohi_expression1);
LV_IMAGE_DECLARE(screen_animimg_exp_hellohi_expression2);
LV_IMAGE_DECLARE(screen_animimg_exp_hellohi_expression3);
LV_IMAGE_DECLARE(screen_animimg_exp_hellohi_expression4);
LV_IMAGE_DECLARE(screen_animimg_exp_sadsad_expression1);
LV_IMAGE_DECLARE(screen_animimg_exp_sadsad_expression2);
LV_IMAGE_DECLARE(screen_animimg_exp_sadsad_expression3);
LV_IMAGE_DECLARE(screen_animimg_exp_sadsad_expression4);
LV_IMAGE_DECLARE(screen_animimg_exp_happyhappy_expression1);
LV_IMAGE_DECLARE(screen_animimg_exp_happyhappy_expression2);
LV_IMAGE_DECLARE(screen_animimg_exp_happyhappy_expression3);
LV_IMAGE_DECLARE(screen_animimg_exp_happyhappy_expression4);
LV_IMAGE_DECLARE(screen_animimg_exp_sleepsleep_expression1);
LV_IMAGE_DECLARE(screen_animimg_exp_sleepsleep_expression2);
LV_IMAGE_DECLARE(screen_animimg_exp_sleepsleep_expression3);
LV_IMAGE_DECLARE(screen_animimg_exp_sleepsleep_expression4);
LV_IMAGE_DECLARE(screen_animimg_exp_flooduncomfort_expression1);
LV_IMAGE_DECLARE(screen_animimg_exp_flooduncomfort_expression2);
LV_IMAGE_DECLARE(screen_animimg_exp_flooduncomfort_expression3);
LV_IMAGE_DECLARE(screen_animimg_exp_flooduncomfort_expression4);
LV_IMAGE_DECLARE(screen_animimg_plant_normalplant_icon1);
LV_IMAGE_DECLARE(screen_animimg_plant_normalplant_icon2);
LV_IMAGE_DECLARE(screen_animimg_plant_normalplant_icon3);
LV_IMAGE_DECLARE(screen_animimg_plant_normalplant_icon4);
LV_IMAGE_DECLARE(screen_animimg_floodflooding1);
LV_IMAGE_DECLARE(screen_animimg_floodflooding2);
LV_IMAGE_DECLARE(screen_animimg_floodflooding3);
LV_IMAGE_DECLARE(screen_animimg_floodflooding4);
LV_IMAGE_DECLARE(screen_animimg_floodflooding5);
LV_IMAGE_DECLARE(screen_animimg_floodflooding6);
LV_IMAGE_DECLARE(screen_animimg_floodflooding7);
LV_IMAGE_DECLARE(screen_animimg_exp_cominghomebackhome_expression1);
LV_IMAGE_DECLARE(screen_animimg_exp_cominghomebackhome_expression2);
LV_IMAGE_DECLARE(screen_animimg_exp_cominghomebackhome_expression3);
LV_IMAGE_DECLARE(screen_animimg_exp_cominghomebackhome_expression4);
LV_IMAGE_DECLARE(screen_animimg_idle_cominghomewalkLeft1);
LV_IMAGE_DECLARE(screen_animimg_idle_cominghomewalkLeft2);
LV_IMAGE_DECLARE(screen_animimg_idle_cominghomewalkLeft3);
LV_IMAGE_DECLARE(screen_animimg_idle_cominghomewalkLeft4);
LV_IMAGE_DECLARE(screen_animimg_exp_lostlost_expression1);
LV_IMAGE_DECLARE(screen_animimg_exp_lostlost_expression2);
LV_IMAGE_DECLARE(screen_animimg_exp_lostlost_expression3);
LV_IMAGE_DECLARE(screen_animimg_exp_lostlost_expression4);
LV_IMAGE_DECLARE(screen_animimg_loddingloading1);
LV_IMAGE_DECLARE(screen_animimg_loddingloading2);
LV_IMAGE_DECLARE(screen_animimg_loddingloading3);
LV_IMAGE_DECLARE(screen_animimg_loddingloading4);



#ifdef __cplusplus
}
#endif
#endif
