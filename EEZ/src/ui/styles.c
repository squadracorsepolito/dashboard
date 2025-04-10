#include "styles.h"

#include "ui.h"
#include "screens.h"

//
// Style: label_basic_style
//

void init_style_label_basic_style_MAIN_DEFAULT(lv_style_t *style) {
    lv_style_set_text_color(style, lv_color_hex(0xfffafafa));
    lv_style_set_align(style, LV_ALIGN_CENTER);
    lv_style_set_pad_top(style, 2);
    lv_style_set_pad_bottom(style, 2);
    lv_style_set_pad_left(style, 2);
    lv_style_set_pad_right(style, 2);
    lv_style_set_text_align(style, LV_TEXT_ALIGN_CENTER);
};

lv_style_t *get_style_label_basic_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: label_basic_F40_style
//

void init_style_label_basic_f40_style_MAIN_DEFAULT(lv_style_t *style) {
    init_style_label_basic_style_MAIN_DEFAULT(style);
    
    lv_style_set_text_font(style, &lv_font_montserrat_40);
};

lv_style_t *get_style_label_basic_f40_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_f40_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_f40_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_f40_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_f40_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_f40_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: label_basic_F38_style
//

void init_style_label_basic_f38_style_MAIN_DEFAULT(lv_style_t *style) {
    init_style_label_basic_style_MAIN_DEFAULT(style);
    
    lv_style_set_text_font(style, &lv_font_montserrat_38);
};

lv_style_t *get_style_label_basic_f38_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_f38_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_f38_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_f38_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_f38_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_f38_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: label_basic_F32_style
//

void init_style_label_basic_f32_style_MAIN_DEFAULT(lv_style_t *style) {
    init_style_label_basic_style_MAIN_DEFAULT(style);
    
    lv_style_set_text_font(style, &lv_font_montserrat_32);
};

lv_style_t *get_style_label_basic_f32_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_f32_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_f32_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_f32_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_f32_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_f32_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: label_basic_F30_style
//

void init_style_label_basic_f30_style_MAIN_DEFAULT(lv_style_t *style) {
    init_style_label_basic_style_MAIN_DEFAULT(style);
    
    lv_style_set_text_font(style, &lv_font_montserrat_30);
};

lv_style_t *get_style_label_basic_f30_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_f30_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_f30_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_f30_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_f30_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_f30_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: label_basic_F28_style
//

void init_style_label_basic_f28_style_MAIN_DEFAULT(lv_style_t *style) {
    init_style_label_basic_style_MAIN_DEFAULT(style);
    
    lv_style_set_text_font(style, &lv_font_montserrat_28);
};

lv_style_t *get_style_label_basic_f28_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_f28_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_f28_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_f28_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_f28_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_f28_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: label_basic_F26_style
//

void init_style_label_basic_f26_style_MAIN_DEFAULT(lv_style_t *style) {
    init_style_label_basic_style_MAIN_DEFAULT(style);
    
    lv_style_set_text_font(style, &lv_font_montserrat_26);
};

lv_style_t *get_style_label_basic_f26_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_f26_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_f26_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_f26_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_f26_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_f26_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: label_basic_F24_style
//

void init_style_label_basic_f24_style_MAIN_DEFAULT(lv_style_t *style) {
    init_style_label_basic_style_MAIN_DEFAULT(style);
    
    lv_style_set_text_font(style, &lv_font_montserrat_26);
};

lv_style_t *get_style_label_basic_f24_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_f24_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_f24_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_f24_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_f24_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_f24_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: label_basic_F22_style
//

void init_style_label_basic_f22_style_MAIN_DEFAULT(lv_style_t *style) {
    init_style_label_basic_style_MAIN_DEFAULT(style);
    
    lv_style_set_text_font(style, &lv_font_montserrat_26);
};

lv_style_t *get_style_label_basic_f22_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_f22_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_f22_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_f22_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_f22_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_f22_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: label_basic_F20_style
//

void init_style_label_basic_f20_style_MAIN_DEFAULT(lv_style_t *style) {
    init_style_label_basic_style_MAIN_DEFAULT(style);
    
    lv_style_set_text_font(style, &lv_font_montserrat_26);
};

lv_style_t *get_style_label_basic_f20_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_label_basic_f20_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_label_basic_f20_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_label_basic_f20_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_label_basic_f20_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_label_basic_f20_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: container_basic_style
//

void init_style_container_basic_style_MAIN_DEFAULT(lv_style_t *style) {
    lv_style_set_pad_top(style, 2);
    lv_style_set_pad_bottom(style, 2);
    lv_style_set_pad_left(style, 2);
    lv_style_set_pad_right(style, 2);
    lv_style_set_align(style, LV_ALIGN_CENTER);
};

lv_style_t *get_style_container_basic_style_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = lv_malloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_container_basic_style_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_container_basic_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_container_basic_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_container_basic_style(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_container_basic_style_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
//
//

void add_style(lv_obj_t *obj, int32_t styleIndex) {
    typedef void (*AddStyleFunc)(lv_obj_t *obj);
    static const AddStyleFunc add_style_funcs[] = {
        add_style_label_basic_style,
        add_style_label_basic_f40_style,
        add_style_label_basic_f38_style,
        add_style_label_basic_f32_style,
        add_style_label_basic_f30_style,
        add_style_label_basic_f28_style,
        add_style_label_basic_f26_style,
        add_style_label_basic_f24_style,
        add_style_label_basic_f22_style,
        add_style_label_basic_f20_style,
        add_style_container_basic_style,
    };
    add_style_funcs[styleIndex](obj);
}

void remove_style(lv_obj_t *obj, int32_t styleIndex) {
    typedef void (*RemoveStyleFunc)(lv_obj_t *obj);
    static const RemoveStyleFunc remove_style_funcs[] = {
        remove_style_label_basic_style,
        remove_style_label_basic_f40_style,
        remove_style_label_basic_f38_style,
        remove_style_label_basic_f32_style,
        remove_style_label_basic_f30_style,
        remove_style_label_basic_f28_style,
        remove_style_label_basic_f26_style,
        remove_style_label_basic_f24_style,
        remove_style_label_basic_f22_style,
        remove_style_label_basic_f20_style,
        remove_style_container_basic_style,
    };
    remove_style_funcs[styleIndex](obj);
}

