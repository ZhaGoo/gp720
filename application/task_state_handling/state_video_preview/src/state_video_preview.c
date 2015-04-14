#include "state_video_preview.h"
#include "ap_video_preview.h"
#include "ap_state_handling.h"
#include "ap_state_config.h"
#include "ap_video_record.h"
#include "ap_display.h"
#include "avi_encoder_app.h"

//+++
INT8U pic_flag;
INT32U photo_check_time_loop_count = 0;
INT32U photo_check_time_preview_count = 0;

#define CONTINUOUS_SHOOTING_COUNT_MAX	5

//	prototypes
void state_video_preview_init(void);
void state_video_preview_exit(void);


extern INT8U ap_state_config_usb_mode_get(void);

void state_photo_check_timer_isr(void)
{
	photo_check_time_loop_count++;

	/*
		當是底下 MSG 馬上跳離
		MSG_APQ_CONNECT_TO_PC / MSG_APQ_MENU_KEY_ACTIVE / MSG_APQ_MODE
	*/	
	if((pic_flag == 2)||(pic_flag == 3)||(pic_flag == 4))
	{
		photo_check_time_loop_count = photo_check_time_preview_count;
	}
	
	if(photo_check_time_loop_count >= photo_check_time_preview_count)
	{
		msgQSend(ApQ, MSG_APQ_CAPTURE_PREVIEW_ON, NULL, NULL, MSG_PRI_NORMAL);	
		timer_stop(TIMER_C);
	}
}

extern INT8U ext_rtc_pwr_on_flag;

void state_video_preview_init(void)
{
	DBG_PRINT("video_preview state init enter\r\n");
	ap_video_preview_init();
}

void state_video_preview_entry(void *para)
{
	EXIT_FLAG_ENUM exit_flag = EXIT_RESUME;
	INT32U msg_id, file_path_addr;
	INT32U *prev_state;
	STAudioConfirm *audio_temp;
	INT8U continuous_shooting_count = 0;
	INT8U photo_check_time_flag;
	INT32U photo_check_time_start;
	INT32U photo_check_time_end;
	INT32U photo_check_time_count_ms;
	INT32U photo_check_ui_setting_ms;

	pic_flag = 0;
	prev_state = para;

	/*
		拍照模式下快速檢視
		0	關
		1	2秒
		2	5秒
	*/
	//連拍不啟動快速檢視

	if(ap_state_handling_storage_id_get() == NO_STORAGE) { //not to use burst when no SD card inserted
		photo_check_time_flag = ap_state_config_preview_get();
	} else if(ap_state_config_burst_get()) {
		photo_check_time_flag = 0;
	} else {
		photo_check_time_flag = ap_state_config_preview_get();
	}

	/*
		從video recording / setting 進來都是將preview buffer 由dummy address 導到display address
	*/

	if(ext_rtc_pwr_on_flag) {
		ext_rtc_pwr_on_flag = 0;
		ap_video_capture_mode_switch(1, STATE_VIDEO_PREVIEW);
	} else {
	  #if (!ENABLE_SAVE_SENSOR_RAW_DATA)
		ap_video_capture_mode_switch(0, STATE_VIDEO_PREVIEW);
	  #else
		ap_video_capture_mode_switch(1, STATE_VIDEO_PREVIEW);
	  #endif
	}

	state_video_preview_init();
	DBG_PRINT("Video_preview_init!\r\n");

	while (exit_flag == EXIT_RESUME) {
		if (msgQReceive(ApQ, &msg_id, (void *) ApQ_para, AP_QUEUE_MSG_MAX_LEN) == STATUS_FAIL) {
			continue;
		}

		switch (msg_id) {
			case EVENT_APQ_ERR_MSG:
				audio_temp = (STAudioConfirm *)ApQ_para;
				if((audio_temp->result == AUDIO_ERR_DEC_FINISH) && (audio_temp->source_type == AUDIO_SRC_TYPE_APP_RS)){
					//gpio_write_io(SPEAKER_EN, DATA_LOW);
				} else {
					audio_confirm_handler((STAudioConfirm *)ApQ_para);
				}
				break;
			case MSG_STORAGE_SERVICE_MOUNT:
				ap_state_handling_storage_id_set(ApQ_para[0]);
        		ap_state_handling_icon_clear_cmd(ICON_INTERNAL_MEMORY, NULL, NULL);
        		ap_state_handling_icon_show_cmd(ICON_SD_CARD, NULL, NULL);
			    ap_state_handling_str_draw_exit();
				left_capture_num = cal_left_capture_num();
				left_capture_num_display(left_capture_num);
        		DBG_PRINT("[Video Preview Mount OK]\r\n");
        		break;
        	case MSG_STORAGE_SERVICE_NO_STORAGE:
        		ap_state_handling_storage_id_set(ApQ_para[0]);
        		ap_state_handling_icon_clear_cmd(ICON_SD_CARD, NULL, NULL);
        		ap_state_handling_icon_show_cmd(ICON_INTERNAL_MEMORY, NULL, NULL);
				left_capture_num = cal_left_capture_num();
				left_capture_num_display(left_capture_num);
        		DBG_PRINT("[Video Preview Mount FAIL]\r\n");
        		break;

        	case MSG_APQ_MENU_KEY_ACTIVE:
				if(pic_flag == 0) {
				    ap_state_handling_str_draw_exit();
        			OSTimeDly(3);
					vid_enc_disable_sensor_clock();
	       		  	OSQPost(StateHandlingQ, (void *) STATE_SETTING);
	        		exit_flag = EXIT_BREAK;
				} else {
					pic_flag = 3;
				}
            	break;

        	case MSG_APQ_NEXT_KEY_ACTIVE:
        	case MSG_APQ_PREV_KEY_ACTIVE:	
				if(pic_flag == 0)
				{
					if(msg_id == MSG_APQ_NEXT_KEY_ACTIVE)
					{
	        			ap_video_record_zoom_inout(0);
	        		}
	        		else
	        		{
	        			ap_video_record_zoom_inout(1);
	        		}
	        	}
        		break;

  	     	case MSG_APQ_MODE:
				if(pic_flag == 0) {
				    ap_state_handling_str_draw_exit();
				    OSTimeDly(3);
					vid_enc_disable_sensor_clock();
      		  		OSQPost(StateHandlingQ, (void *) STATE_BROWSE);
	        		exit_flag = EXIT_BREAK;
	        	} else {
	        		pic_flag = 4;
	        	}
	        	break;

        	case MSG_APQ_FUNCTION_KEY_ACTIVE:
#if KEY_FUNTION_TYPE == SAMPLE2
        		break;
#endif
			case MSG_APQ_CAPTURE_KEY_ACTIVE:	
			case MSG_APQ_CAPTURE_CONTINUOUS_SHOOTING:
				if(pic_flag == 0) {
					ap_state_handling_led_on(); // young 20140930
					if(ap_video_preview_func_key_active() < 0) {
						pic_flag = 0;
					} else {
						pic_flag = 1;
						photo_check_time_start = OSTimeGet();
					}
				}
				break;

        	case MSG_STORAGE_SERVICE_PIC_REPLY:
        		if(ap_video_preview_reply_action((STOR_SERV_FILEINFO *) ApQ_para) < 0) {
        			pic_flag = 0;
        			break;
        		}
        		file_path_addr = ((STOR_SERV_FILEINFO *) ApQ_para)->file_path_addr;
        		break;

        	case MSG_STORAGE_SERVICE_PIC_DONE:
        		ap_video_preview_reply_done(ApQ_para[0], file_path_addr);
     		
				// 沒有啟動快速檢視,直接跑到MSG_APQ_CAPTURE_PREVIEW_ON
				if(photo_check_time_flag)
				{
					photo_check_time_end = OSTimeGet();
					photo_check_ui_setting_ms = ((photo_check_time_flag*3)-1)*1000; // ms

					photo_check_time_count_ms = (photo_check_time_end-photo_check_time_start)*10; //ms

					//當拍照花費時間小於UI設定,才啟動timer
					if(photo_check_time_count_ms < photo_check_ui_setting_ms)
					{
						photo_check_time_loop_count = 0;
						photo_check_time_preview_count = (photo_check_ui_setting_ms-photo_check_time_count_ms)/100;
						timer_msec_setup(TIMER_C, 100, 0, state_photo_check_timer_isr); // 100 ms
		        		break;
					}
				}
			case MSG_APQ_CAPTURE_PREVIEW_ON:			
        		if(pic_flag == 2) { // Connect To PC
				    ap_state_handling_str_draw_exit();
				    OSTimeDly(3);
					vid_enc_disable_sensor_clock();
        			video_encode_preview_off();
	        		ap_state_handling_connect_to_pc(STATE_VIDEO_PREVIEW);
        			break;
        		} else if(pic_flag == 3) { // MEMU Key
				    ap_state_handling_str_draw_exit();
				    OSTimeDly(3);
        		  	OSQPost(StateHandlingQ, (void *) STATE_SETTING);
	        		exit_flag = EXIT_BREAK;
        			break;
        		} else if(pic_flag == 4) { // MODE Key
				    ap_state_handling_str_draw_exit();
				    OSTimeDly(3);
      		  		OSQPost(StateHandlingQ, (void *) STATE_BROWSE);
	        		exit_flag = EXIT_BREAK;
        			break;
        		} else if(pic_flag == 5) { // HDMI insert
				    ap_state_handling_str_draw_exit();
				    OSTimeDly(3);
      		  		OSQPost(StateHandlingQ, (void *) STATE_BROWSE);
					msgQSend(ApQ, MSG_APQ_HDMI_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);
	        		exit_flag = EXIT_BREAK;
        			break;
        		} else if((pic_flag == 6) || (pic_flag == 7)) { // TV plug in/out
					vid_enc_disable_sensor_clock();

					if(pic_flag == 6) {
						ap_state_handling_tv_init();
					} else {
						ap_state_handling_tv_uninit();
					}

			   		ap_video_capture_mode_switch(1, STATE_VIDEO_PREVIEW);
					video_capture_resolution_display();
					left_capture_num = cal_left_capture_num();
					left_capture_num_display(left_capture_num);

					pic_flag = 0;
					break;
        		}
        		
				/* sensor不關,將sensor 送出的資料引導到底層preview flow */
				#if ENABLE_SAVE_SENSOR_RAW_DATA
		   		ap_video_capture_mode_switch(0, STATE_VIDEO_RECORD);
				#else
		   		ap_video_capture_mode_switch(0, STATE_VIDEO_PREVIEW);
		   		#endif
				/*
					拍照模式下連拍
				*/
				if(ap_state_config_burst_get() && (ap_state_handling_storage_id_get() != NO_STORAGE))
				{
					continuous_shooting_count++;
					if(continuous_shooting_count < CONTINUOUS_SHOOTING_COUNT_MAX)
					{
        				msgQSend(ApQ, MSG_APQ_CAPTURE_CONTINUOUS_SHOOTING, NULL, NULL, MSG_PRI_NORMAL);
        			}
        			else
        			{
						continuous_shooting_count = 0;
        			}
				}

        		pic_flag = 0;
			
				ap_state_handling_led_off(); // young 20140930
			break;
 		
        	case MSG_APQ_POWER_KEY_ACTIVE:
        	case MSG_APQ_SYS_RESET:
				video_encode_exit();
				OSTimeDly(10);        		
				ap_state_config_hang_mode_set(0x02);	//STATE_VIDEO_PREIVEW
				if(msg_id == MSG_APQ_POWER_KEY_ACTIVE) {
	        		ap_state_handling_power_off(0);
	        	} else {
	        		ap_state_handling_power_off(1);
	        	}
        		break;

#if C_BATTERY_DETECT == CUSTOM_ON
        	case MSG_APQ_BATTERY_LVL_SHOW:
        		ap_state_handling_battery_icon_show(ApQ_para[0]);
        		break;
        	case MSG_APQ_BATTERY_CHARGED_SHOW:
				ap_state_handling_charge_icon_show(1);
        		break;
        	case MSG_APQ_BATTERY_CHARGED_CLEAR:
				ap_state_handling_charge_icon_show(0);
        		break;        		
        	case MSG_APQ_BATTERY_LOW_SHOW:
        		ap_state_handling_clear_all_icon();
        		OSTimeDly(5);
				ap_state_handling_str_draw_exit();
				ap_state_handling_str_draw(STR_BATTERY_LOW, WARNING_STR_COLOR);
				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_BATTERY_LOW_BLINK_START, NULL, NULL, MSG_PRI_NORMAL);
        		break;
#endif

        	case MSG_APQ_CONNECT_TO_PC:
				if(ap_display_get_device() != DISP_DEV_TFT) break;
				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TV_POLLING_STOP, NULL, NULL, MSG_PRI_NORMAL);
				OSTimeDly(3);
				if(pic_flag == 0) {
				    ap_state_handling_str_draw_exit();
				    OSTimeDly(3);
					vid_enc_disable_sensor_clock();
        			video_encode_preview_off();
	        		ap_state_handling_connect_to_pc(STATE_VIDEO_PREVIEW);
	        	} else {
	        		pic_flag = 2;
	        	}
        		break;

        	case MSG_APQ_DISCONNECT_TO_PC:
				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TV_POLLING_START, NULL, NULL, MSG_PRI_NORMAL);
        		ap_state_handling_disconnect_to_pc();
        		OSTimeDly(100);
				/*
					先將web cam的preview 導出去,再切換成capture 的preview					
				*/
				pic_flag = 0;
				vid_enc_disable_sensor_clock();
	       		video_encode_preview_on();
				/* sensor不關,將sensor 送出的資料引導到底層preview flow */
		   		ap_video_capture_mode_switch(0, STATE_VIDEO_PREVIEW);
        		break;

			case MSG_APQ_NIGHT_MODE_KEY:
				audio_effect_play(EFFECT_CLICK);
				ap_state_handling_night_mode_switch();
				break;

			case MSG_APQ_USER_CONFIG_STORE:
				ap_state_config_store();
				break;

			case MSG_APQ_AUDIO_EFFECT_UP:
			case MSG_APQ_AUDIO_EFFECT_DOWN:
			    break;

			case MSG_APQ_HDMI_PLUG_IN:
				if(pic_flag == 0) {
				    ap_state_handling_str_draw_exit();
				    OSTimeDly(3);
	        		vid_enc_disable_sensor_clock();
      		  		OSQPost(StateHandlingQ, (void *) STATE_BROWSE);
					msgQSend(ApQ, MSG_APQ_HDMI_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);
	        		exit_flag = EXIT_BREAK;
	        	} else {
	        		pic_flag = 5;
	        	}
			break;

			
			//+++ TV_OUT_D1
			case MSG_APQ_TV_PLUG_OUT:
			case MSG_APQ_TV_PLUG_IN:
				video_capture_resolution_clear();
				left_capture_num_str_clear();

				if(pic_flag == 0) {
				    ap_state_handling_str_draw_exit();
				    OSTimeDly(3);
					vid_enc_disable_sensor_clock();
					if(msg_id == MSG_APQ_TV_PLUG_IN) {
						ap_state_handling_tv_init();
	        		} else {
						ap_state_handling_tv_uninit();
	        		}
			   		ap_video_capture_mode_switch(1, STATE_VIDEO_PREVIEW);
					video_capture_resolution_display();
					left_capture_num = cal_left_capture_num();
					left_capture_num_display(left_capture_num);
				} else {
					if(msg_id == MSG_APQ_TV_PLUG_IN) {
		        		pic_flag = 6;
		        	} else {
		        		pic_flag = 7;
		        	}
				}
			break;
			//---

			default:
				ap_state_common_handling(msg_id);
				break;
		}
	}

	state_video_preview_exit();
	if(photo_check_time_flag)
	{
		timer_stop(TIMER_C);
	}
}

void state_video_preview_exit(void)
{
	ap_video_preview_exit();

	DBG_PRINT("Exit video_preview state\r\n");
}
