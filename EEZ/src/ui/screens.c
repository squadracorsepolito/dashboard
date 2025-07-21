#include "screens.h"
#include "styles.h"
#include "images.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;
uint32_t active_theme_index = 0;

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            lv_obj_set_pos(obj, 0, 0);
            lv_obj_set_size(obj, LV_PCT(100), LV_PCT(100));
            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            add_style_container_basic_style(obj);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // pg_main_lv_bat_v
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_lv_bat_v = obj;
                    lv_obj_set_pos(obj, -193, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -136, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "V");
                }
                {
                    // pg_main_hv_soc_est
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_hv_soc_est = obj;
                    lv_obj_set_pos(obj, 198, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 147, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "%");
                }
                {
                    // pg_main_sx_status_bar
                    lv_obj_t *obj = lv_obj_create(parent_obj);
                    objects.pg_main_sx_status_bar = obj;
                    lv_obj_set_pos(obj, LV_PCT(5), LV_PCT(25));
                    lv_obj_set_size(obj, 36, 140);
                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                }
                {
                    // pg_main_dx_status_bar
                    lv_obj_t *obj = lv_obj_create(parent_obj);
                    objects.pg_main_dx_status_bar = obj;
                    lv_obj_set_pos(obj, LV_PCT(85), LV_PCT(25));
                    lv_obj_set_size(obj, 36, 140);
                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                }
                {
                    // pg_main_sx_map
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_sx_map = obj;
                    lv_obj_set_pos(obj, -198, 110);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "9");
                }
                {
                    // pg_main_dx_map
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_dx_map = obj;
                    lv_obj_set_pos(obj, 186, 112);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "9");
                }
                {
                    // pg_main_lc
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_lc = obj;
                    lv_obj_set_pos(obj, -101, 114);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "LC");
                }
                {
                    // pg_main_tc
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_tc = obj;
                    lv_obj_set_pos(obj, -5, 115);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "TC");
                }
                {
                    // pg_main_tv
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_tv = obj;
                    lv_obj_set_pos(obj, 91, 117);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "TV");
                }
            }
        }
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            lv_obj_set_pos(obj, -12, -12);
            lv_obj_set_size(obj, 313, 209);
            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            add_style_container_basic_style(obj);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -113, -76);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Inv FL");
                }
                {
                    // pg_main_inv_temp_fl
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_inv_temp_fl = obj;
                    lv_obj_set_pos(obj, -55, -77);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -112, -54);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Inv FR");
                }
                {
                    // pg_main_inv_temp_fr
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_inv_temp_fr = obj;
                    lv_obj_set_pos(obj, -56, -55);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -90, -5);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Cool press L");
                }
                {
                    // pg_main_cool_press_l
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_cool_press_l = obj;
                    lv_obj_set_pos(obj, -12, -6);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -90, 17);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Cool press R");
                }
                {
                    // pg_main_cool_press_r
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_cool_press_r = obj;
                    lv_obj_set_pos(obj, -11, 17);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -20, -77);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -20, -54);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 46, -76);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Inv RL");
                }
                {
                    // pg_main_inv_temp_rl
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_inv_temp_rl = obj;
                    lv_obj_set_pos(obj, 105, -75);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 46, -54);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Inv RR");
                }
                {
                    // pg_main_inv_temp_rr
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_inv_temp_rr = obj;
                    lv_obj_set_pos(obj, 105, -53);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 143, -75);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 143, -52);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 26, -6);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Bar");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 26, 17);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Bar");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -109, 57);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Mot FL");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -109, 80);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Mot FR");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 49, 58);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Mot RL");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 49, 81);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Mot RR");
                }
                {
                    // pg_main_motor_temp_fl
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_motor_temp_fl = obj;
                    lv_obj_set_pos(obj, -55, 57);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    // pg_main_motor_temp_fr
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_motor_temp_fr = obj;
                    lv_obj_set_pos(obj, -56, 79);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -20, 57);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -20, 80);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    // pg_main_motor_temp_rl
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_motor_temp_rl = obj;
                    lv_obj_set_pos(obj, 105, 58);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    // pg_main_motor_temp_rr
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_motor_temp_rr = obj;
                    lv_obj_set_pos(obj, 105, 80);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 143, 58);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 143, 81);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
            }
        }
    }
    
    tick_screen_main();
}

void tick_screen_main() {
}



typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

void create_screens() {
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_main();
}
