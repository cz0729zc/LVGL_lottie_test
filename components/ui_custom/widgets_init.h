/*
* Copyright 2025 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#ifndef WIDGET_INIT_H
#define WIDGET_INIT_H
#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "gui_guider.h"

__attribute__((unused)) void kb_event_cb(lv_event_t *e);
__attribute__((unused)) void ta_event_cb(lv_event_t *e);
#if LV_USE_ANALOGCLOCK != 0
void clock_count(int *hour, int *min, int *sec);
void digital_clock_count(int * hour, int * minute, int * seconds, char * meridiem);
#endif


extern const lv_image_dsc_t * screen_animimg_ground2_imgs[7];
extern const lv_image_dsc_t * screen_animimg_ground_imgs[4];
extern const lv_image_dsc_t * screen_animimg_idel_sleep_imgs[5];
extern const lv_image_dsc_t * screen_animimg_idel_awake_imgs[5];
extern const lv_image_dsc_t * screen_animimg_idel_imgs[4];
extern const lv_image_dsc_t * screen_animimg_idel_flood_imgs[4];
extern const lv_image_dsc_t * screen_animimg_exp_hello_imgs[4];
extern const lv_image_dsc_t * screen_animimg_exp_sad_imgs[4];
extern const lv_image_dsc_t * screen_animimg_exp_happy_imgs[4];
extern const lv_image_dsc_t * screen_animimg_exp_sleep_imgs[4];
extern const lv_image_dsc_t * screen_animimg_exp_flood_imgs[4];
extern const lv_image_dsc_t * screen_animimg_plant_normal_imgs[4];
extern const lv_image_dsc_t * screen_animimg_flood_imgs[7];
extern const lv_image_dsc_t * screen_animimg_exp_cominghome_imgs[4];
extern const lv_image_dsc_t * screen_animimg_idle_cominghome_imgs[4];
extern const lv_image_dsc_t * screen_animimg_exp_lost_imgs[4];
extern const lv_image_dsc_t * screen_animimg_lodding_imgs[4];


#ifdef __cplusplus
}
#endif
#endif
