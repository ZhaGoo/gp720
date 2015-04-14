#include "task_peripheral_handling.h"


typedef void (*KEYFUNC)(INT16U *tick_cnt_ptr);

typedef struct
{
	KEYFUNC key_function;
	KEYFUNC fast_key_fun;
	INT16U key_io;
	INT16U key_cnt;	
}KEYSTATUS;

extern void ap_peripheral_init(void);
extern void ap_peripheral_key_judge(void);
extern void ap_peripheral_adaptor_out_judge(void);
extern void ap_peripheral_key_register(INT8U type);
extern void ap_peripheral_gsensor_data_register(void );
extern void ap_peripheral_motion_detect_judge(void);
extern void ap_peripheral_motion_detect_start(void);
extern void ap_peripheral_motion_detect_stop(void);

extern void ap_peripheral_lcd_backlight_set(INT8U type);
extern void ap_peripheral_night_mode_set(INT8U type);

extern void ap_TFT_backlight_tmr_check(void);
extern void ap_peripheral_tv_detect(void);
extern void ap_peripheral_read_gsensor(void);

extern void ap_peripheral_ad_key_judge(void);
extern void ap_peripheral_battery_check_calculate(void);
extern void ap_peripheral_config_store(void);

extern void ap_peripheral_led_set(INT8U type);
extern void ap_peripheral_led_flash_set(void);
extern void ap_peripheral_led_blink_set(void);

extern INT8U ap_video_record_sts_get(void);

extern void ap_peripheral_hdmi_detect(void);
extern void ap_peripheral_clr_screen_saver_timer(void);

#if TV_DET_ENABLE
extern INT8U tv_plug_status_get(void);
#endif

