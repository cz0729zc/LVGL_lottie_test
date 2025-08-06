/*
* Copyright 2025 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include "gui_guider.h"
#include "widgets_init.h"
#include <stdlib.h>
#include <string.h>


__attribute__((unused)) void kb_event_cb (lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *kb = lv_event_get_target(e);
    if(code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

__attribute__((unused)) void ta_event_cb (lv_event_t *e) {
#if LV_USE_KEYBOARD
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    lv_obj_t * kb = lv_event_get_user_data(e);

    if(code == LV_EVENT_FOCUSED) {
        if(lv_indev_get_type(lv_indev_active()) != LV_INDEV_TYPE_KEYPAD) {
            lv_keyboard_set_textarea(kb, ta);
            lv_obj_remove_flag(kb, LV_OBJ_FLAG_HIDDEN);
        }
    } else if(code == LV_EVENT_READY) {
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_state(ta, LV_STATE_FOCUSED);
        lv_indev_reset(NULL, ta);
    } else if(code == LV_EVENT_DEFOCUSED) {
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
#endif
}

void clock_count(int *hour, int *min, int *sec)
{
    (*sec)++;
    if(*sec == 60)
    {
        *sec = 0;
        (*min)++;
    }
    if(*min == 60)
    {
        *min = 0;
        if(*hour < 12)
        {
            (*hour)++;
        } else {
            (*hour)++;
            *hour = *hour %12;
        }
    }
}

void digital_clock_count(int * hour, int * minute, int * seconds, char * meridiem)
{

    (*seconds)++;
    if(*seconds == 60) {
        *seconds = 0;
        (*minute)++;
    }
    if(*minute == 60) {
        *minute = 0;
        if(*hour < 12) {
            (*hour)++;
        }
        else {
            (*hour)++;
            (*hour) = (*hour) % 12;
        }
    }
    if(*hour == 12 && *seconds == 0 && *minute == 0) {
        if((lv_strcmp(meridiem, "PM") == 0)) {
            lv_strcpy(meridiem, "AM");
        }
        else {
            lv_strcpy(meridiem, "PM");
        }
    }
}


const lv_image_dsc_t * screen_animimg_ground2_imgs[7] = {
    &screen_animimg_ground2dessert_cloud1,
    &screen_animimg_ground2dessert_cloud2,
    &screen_animimg_ground2dessert_cloud3,
    &screen_animimg_ground2dessert_cloud4,
    &screen_animimg_ground2dessert_cloud5,
    &screen_animimg_ground2dessert_cloud6,
    &screen_animimg_ground2dessert_cloud7,
};
const lv_image_dsc_t * screen_animimg_ground_imgs[4] = {
    &screen_animimg_grounddeceration1,
    &screen_animimg_grounddeceration2,
    &screen_animimg_grounddeceration3,
    &screen_animimg_grounddeceration4,
};
const lv_image_dsc_t * screen_animimg_idel_sleep_imgs[5] = {
    &screen_animimg_idel_sleepfallAsleep1,
    &screen_animimg_idel_sleepfallAsleep2,
    &screen_animimg_idel_sleepfallAsleep3,
    &screen_animimg_idel_sleepfallAsleep4,
    &screen_animimg_idel_sleepfallAsleep5,
};
const lv_image_dsc_t * screen_animimg_idel_awake_imgs[5] = {
    &screen_animimg_idel_awakeawake1,
    &screen_animimg_idel_awakeawake2,
    &screen_animimg_idel_awakeawake3,
    &screen_animimg_idel_awakeawake4,
    &screen_animimg_idel_awakeawake5,
};
const lv_image_dsc_t * screen_animimg_idel_imgs[4] = {
    &screen_animimg_idelidle1,
    &screen_animimg_idelidle2,
    &screen_animimg_idelidle3,
    &screen_animimg_idelidle4,
};
const lv_image_dsc_t * screen_animimg_idel_flood_imgs[4] = {
    &screen_animimg_idel_floodidle_uncomfort1,
    &screen_animimg_idel_floodidle_uncomfort2,
    &screen_animimg_idel_floodidle_uncomfort3,
    &screen_animimg_idel_floodidle_uncomfort4,
};
const lv_image_dsc_t * screen_animimg_exp_hello_imgs[4] = {
    &screen_animimg_exp_hellohi_expression1,
    &screen_animimg_exp_hellohi_expression2,
    &screen_animimg_exp_hellohi_expression3,
    &screen_animimg_exp_hellohi_expression4,
};
const lv_image_dsc_t * screen_animimg_exp_sad_imgs[4] = {
    &screen_animimg_exp_sadsad_expression1,
    &screen_animimg_exp_sadsad_expression2,
    &screen_animimg_exp_sadsad_expression3,
    &screen_animimg_exp_sadsad_expression4,
};
const lv_image_dsc_t * screen_animimg_exp_happy_imgs[4] = {
    &screen_animimg_exp_happyhappy_expression1,
    &screen_animimg_exp_happyhappy_expression2,
    &screen_animimg_exp_happyhappy_expression3,
    &screen_animimg_exp_happyhappy_expression4,
};
const lv_image_dsc_t * screen_animimg_exp_sleep_imgs[4] = {
    &screen_animimg_exp_sleepsleep_expression1,
    &screen_animimg_exp_sleepsleep_expression2,
    &screen_animimg_exp_sleepsleep_expression3,
    &screen_animimg_exp_sleepsleep_expression4,
};
const lv_image_dsc_t * screen_animimg_exp_flood_imgs[4] = {
    &screen_animimg_exp_flooduncomfort_expression1,
    &screen_animimg_exp_flooduncomfort_expression2,
    &screen_animimg_exp_flooduncomfort_expression3,
    &screen_animimg_exp_flooduncomfort_expression4,
};
const lv_image_dsc_t * screen_animimg_plant_normal_imgs[4] = {
    &screen_animimg_plant_normalplant_icon1,
    &screen_animimg_plant_normalplant_icon2,
    &screen_animimg_plant_normalplant_icon3,
    &screen_animimg_plant_normalplant_icon4,
};
const lv_image_dsc_t * screen_animimg_flood_imgs[7] = {
    &screen_animimg_floodflooding1,
    &screen_animimg_floodflooding2,
    &screen_animimg_floodflooding3,
    &screen_animimg_floodflooding4,
    &screen_animimg_floodflooding5,
    &screen_animimg_floodflooding6,
    &screen_animimg_floodflooding7,
};
const lv_image_dsc_t * screen_animimg_exp_cominghome_imgs[4] = {
    &screen_animimg_exp_cominghomebackhome_expression1,
    &screen_animimg_exp_cominghomebackhome_expression2,
    &screen_animimg_exp_cominghomebackhome_expression3,
    &screen_animimg_exp_cominghomebackhome_expression4,
};
const lv_image_dsc_t * screen_animimg_idle_cominghome_imgs[4] = {
    &screen_animimg_idle_cominghomewalkLeft1,
    &screen_animimg_idle_cominghomewalkLeft2,
    &screen_animimg_idle_cominghomewalkLeft3,
    &screen_animimg_idle_cominghomewalkLeft4,
};
