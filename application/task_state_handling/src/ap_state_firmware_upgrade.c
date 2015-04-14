/*
* Description: This file provides APIs to upgrade firmware from SD card to SPI
*
* Author: Tristan Yang
*
* Date: 2008/02/17
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.00
*/
#include "string.h"
#include "ap_state_firmware_upgrade.h"
#include "ap_state_resource.h"
#include "drv_l2_spifc.h"
#include "ap_state_handling.h"

extern INT32S SPIFC_Flash_erase_block(INT32U addr, INT8U eraseArgs);
extern INT32S SPIFC_Flash_read_page(INT32U addr, INT8U *buf);
extern INT32S SPIFC_Flash_write_page(INT32U addr, INT8U *buf);

#define SPIFC_RESOURCE_OFFSET	0
#define UPGRADE_POS_X 10
#define UPGRADE_POS_Y 100

static void cpu_draw_burn_percent(INT32U val, INT32U target_buffer)
{
	STRING_INFO str_info = {0};
	INT16U temp;

	str_info.language = LCD_EN;
	str_info.font_color = 0xFFFF;	//white
	str_info.pos_x = UPGRADE_POS_X;
	str_info.pos_y = UPGRADE_POS_Y+40;
	str_info.buff_w = TFT_WIDTH;
	str_info.buff_h = TFT_HEIGHT;

	temp = val/10;
	ap_state_resource_char_draw(temp+0x30, (INT16U *) target_buffer, &str_info, RGB565_DRAW, 0);
	temp = val - temp * 10;
	ap_state_resource_char_draw(temp+0x30, (INT16U *) target_buffer, &str_info, RGB565_DRAW, 0);
}

static INT32U spifc_rewrite(int idx, INT8U *frame_buf, INT8U* verify_buf)
{
	INT32U i = 0;
	INT32U ret = -1;
	INT32U time = 5;

	do {	
		for (i=0; i<C_UPGRADE_SPI_WRITE_SIZE; ++i)
		{
			if (verify_buf[i]!=0xFF) {
				DBG_PRINT("0x%x page is not empty\r\n",idx);
				return -1;				// Erase 64KB，整個 PAGE 重新寫過
			}	
		}
		if (SPIFC_Flash_write_page(idx, frame_buf)!=0) {
			DBG_PRINT("Rewrite 0x%x Error!!\r\n");
			return -1;
		}

		if (SPIFC_Flash_read_page(idx, verify_buf)!=0) {
			DBG_PRINT("Reread 0x%x Error!!\r\n");
		}

		if (memcmp(verify_buf,frame_buf,C_UPGRADE_SPI_WRITE_SIZE)==0)
		{
			DBG_PRINT("0x%x rewrite OK\r\n",idx);
			ret = 0;
			break;
		}
	}while(--time != 0);


	return ret;
}


void ap_state_firmware_upgrade(void)
{
	INT32U i, j, k, total_size, complete_size;
	INT32U *firmware_buffer, *verify_buffer;
	INT32U display_buffer;
	INT32U buff_size, *buff_ptr;	
	INT32S verify_size;
	INT16U *ptr; 
	struct stat_t statetest;
	INT16S fd;
	STRING_ASCII_INFO ascii_str;
	//INT8S  prog_str[6];
	INT8U retry=0;
	OS_Q_DATA OSQData;
	
	if (storage_sd_upgrade_file_flag_get() != 2) {
		return;
	}

	fd = open("C:\\gp_cardvr_upgrade.bin", O_RDONLY);
	if (fd < 0) {
		return;
	}

	if (fstat(fd, &statetest)) {
		close(fd);
		return;
	}

	ap_state_handling_tv_uninit();

	total_size = statetest.st_size;
	verify_size = (INT32S)total_size;

	firmware_buffer = (INT32U *) gp_malloc(C_UPGRADE_BUFFER_SIZE);
	if (!firmware_buffer) {
		DBG_PRINT("firmware upgrade allocate firmware_buffer fail\r\n");
		close(fd);
		return;
	}

	verify_buffer = (INT32U *) gp_malloc(C_UPGRADE_SPI_WRITE_SIZE);
	if (!verify_buffer) {
		DBG_PRINT("firmware upgrade allocate verify_buffer fail\r\n");
		close(fd);
		return;
	}

	OSTaskDel(STORAGE_SERVICE_PRIORITY);
	OSTaskDel(USB_DEVICE_PRIORITY);
	OSTaskDel(PERIPHERAL_HANDLING_PRIORITY);

	buff_size = TFT_WIDTH * TFT_HEIGHT * 2;
	display_buffer = (INT32U) gp_malloc_align(buff_size, 64);
	if (!display_buffer) {
		close(fd);
		DBG_PRINT("firmware upgrade allocate display buffer fail\r\n");
		return;
	}
	
	buff_ptr = (INT32U*) display_buffer;
	buff_size >>= 2;
	for (i=0;i<buff_size;i++) {
		*buff_ptr++ = 0;
	}
	
	OSQPost(DisplayTaskQ, (void *) MSG_DISPLAY_TASK_EFFECT_INIT);

	ascii_str.font_color = 0xFFFF;
	ascii_str.font_type = 0;
	ascii_str.buff_w = TFT_WIDTH;
	ascii_str.buff_h = TFT_HEIGHT;
	ascii_str.pos_x = UPGRADE_POS_X;
	ascii_str.pos_y = UPGRADE_POS_Y;
	ascii_str.str_ptr = "Upgrading firmware...";
	ap_state_resource_string_ascii_draw((INT16U *)display_buffer, &ascii_str, RGB565_DRAW);
	
	ascii_str.pos_x = UPGRADE_POS_X;
	ascii_str.pos_y = UPGRADE_POS_Y+20;
	ascii_str.str_ptr = "Do not power off now";
	ap_state_resource_string_ascii_draw((INT16U *)display_buffer, &ascii_str, RGB565_DRAW);
	
	ascii_str.pos_x = UPGRADE_POS_X;
	ascii_str.pos_y = UPGRADE_POS_Y+40;
	ascii_str.str_ptr = "00%";
	ap_state_resource_string_ascii_draw((INT16U *)display_buffer, &ascii_str, RGB565_DRAW);

	OSQPost(DisplayTaskQ, (void *) (display_buffer|MSG_DISPLAY_TASK_JPEG_DRAW));
	OSQQuery(DisplayTaskQ, &OSQData);
	while(OSQData.OSMsg != NULL) {
		OSTimeDly(2);
		OSQQuery(DisplayTaskQ, &OSQData);
	}


	R_SYSTEM_PLLEN  |= 0xC00;	// 把 SPI clock 降低，讀寫比較穩
	complete_size = 0;
	while (complete_size < total_size) {
		INT32U buffer_left;
		INT32S block_size;

	    block_size = read(fd, (INT32U) firmware_buffer, C_UPGRADE_BUFFER_SIZE);
		if (block_size <= 0) {
			break;
		}
		// DBG_PRINT("S:%d\r\n",block_size);
		buffer_left = (total_size - complete_size + (C_UPGRADE_SPI_BLOCK_SIZE-1)) & ~(C_UPGRADE_SPI_BLOCK_SIZE-1);
		if (buffer_left > C_UPGRADE_BUFFER_SIZE) {
			buffer_left = C_UPGRADE_BUFFER_SIZE;
		}
		retry = 0;  // 每個 64KB 有 20 次機會
		while (buffer_left && retry<C_UPGRADE_FAIL_RETRY) {
			INT32U complete_size_bak;
			complete_size &= ~(C_UPGRADE_SPI_BLOCK_SIZE-1);
			complete_size_bak = complete_size;
			if (SPIFC_Flash_erase_block(SPIFC_RESOURCE_OFFSET+complete_size_bak, ERASE_BLOCK_64K)) {
				retry++;
				continue;
			}
			for (j=C_UPGRADE_SPI_BLOCK_SIZE; j; j-=C_UPGRADE_SPI_WRITE_SIZE) {
				if (SPIFC_Flash_write_page(SPIFC_RESOURCE_OFFSET+complete_size_bak, (INT8U *) (firmware_buffer + ((complete_size_bak & (C_UPGRADE_BUFFER_SIZE-1))>>2)))) {
					break;
				}
				complete_size_bak += C_UPGRADE_SPI_WRITE_SIZE;
			}
			
			if ((j == 0)&&(verify_size>0)) {	// verify stage
				complete_size_bak = complete_size;
				
				for (j=C_UPGRADE_SPI_BLOCK_SIZE; j; j-=C_UPGRADE_SPI_WRITE_SIZE) {
					INT32U i;
					INT32U flag = 0;
					for (i=0;i<5;++i)
					{
						if ((SPIFC_Flash_read_page(SPIFC_RESOURCE_OFFSET+complete_size_bak, (INT8U*)verify_buffer) )==0 )
							break;
					}

					flag = 0;
					if (memcmp( (void*)verify_buffer  ,(void *)(firmware_buffer + ((complete_size_bak & (C_UPGRADE_BUFFER_SIZE-1))>>2)), C_UPGRADE_SPI_WRITE_SIZE)!=0) {
						DBG_PRINT("Verify 0x%x (0x%x) Error !!\r\n",complete_size_bak,C_UPGRADE_SPI_WRITE_SIZE);
						flag = 1;
					}
					//else DBG_PRINT("Verify 0x%x (0x%x) OK !!\r\n",complete_size_bak,verify_size);

					if (flag == 1)
					{
						if ( spifc_rewrite(SPIFC_RESOURCE_OFFSET+complete_size_bak, (INT8U*)(firmware_buffer + ((complete_size_bak & (C_UPGRADE_BUFFER_SIZE-1))>>2)),  (INT8U*)verify_buffer)!=0 )
						{
							break;
						}
					}
					
					complete_size_bak += C_UPGRADE_SPI_WRITE_SIZE;
					verify_size-= C_UPGRADE_SPI_WRITE_SIZE;
					if (verify_size <=0) // 檔案結束，沒必要再比下去
					{
						j = 0;
						break;
					}
				}				
			}	

			complete_size = complete_size_bak;
			if (j == 0) {
				buffer_left -= C_UPGRADE_SPI_BLOCK_SIZE;

				if (complete_size < total_size) {
					j = complete_size*100/total_size;
				} else {
					j = 99;//100;
				}
				for (i=0;i<50;i++) {
					ptr = (INT16U *) (display_buffer+((UPGRADE_POS_Y+40+i)*TFT_WIDTH+UPGRADE_POS_X)*2);
					for(k=0;k<22/*50*/;k++) {
						*ptr++ = 0x0;
					}
				}

				/*
				ascii_str.pos_x = UPGRADE_POS_X;
				ascii_str.pos_y = UPGRADE_POS_Y+40;
				sprintf((CHAR*)prog_str,"%d%c",j,'%');
				ascii_str.str_ptr = (CHAR *)prog_str;
				ap_state_resource_string_ascii_draw((INT16U *)display_buffer, &ascii_str, RGB565_DRAW);
				*/

				cpu_draw_burn_percent(j, display_buffer);
				OSQPost(DisplayTaskQ, (void *) (display_buffer|MSG_DISPLAY_TASK_JPEG_DRAW));
				OSQQuery(DisplayTaskQ, &OSQData);
				while(OSQData.OSMsg != NULL) {
					OSTimeDly(2);
					OSQQuery(DisplayTaskQ, &OSQData);
				}
			} else {
				retry++;
			}
		}
		if (retry == C_UPGRADE_FAIL_RETRY) {
			break;
		}
	}
#ifdef SDC_UPGRADE_ERASE_USER_CONFIG	
	SPIFC_Flash_erase_block(SPIFC_RESOURCE_OFFSET+complete_size+C_UPGRADE_SPI_BLOCK_SIZE, ERASE_BLOCK_64K);
#endif
	OSTimeDly(5);
	buff_ptr = (INT32U*) display_buffer;
	for (i=0;i<buff_size;i++) {
		*buff_ptr++ = 0;
	}
	ap_state_resource_exit();
	nvmemory_init();
	ap_state_resource_init();
	
	if (retry != C_UPGRADE_FAIL_RETRY) {
		DBG_PRINT("Upgrade OK\r\n");
		ascii_str.pos_x = UPGRADE_POS_X;
		ascii_str.pos_y = UPGRADE_POS_Y;
		ascii_str.str_ptr = "Remove SD card and";
		ap_state_resource_string_ascii_draw((INT16U *)display_buffer, &ascii_str, RGB565_DRAW);
		
		ascii_str.pos_x = UPGRADE_POS_X;
		ascii_str.pos_y = UPGRADE_POS_Y+20;
		ascii_str.str_ptr = "restart now";
		ap_state_resource_string_ascii_draw((INT16U *)display_buffer, &ascii_str, RGB565_DRAW);
		
		ascii_str.pos_x = UPGRADE_POS_X;
		ascii_str.pos_y = UPGRADE_POS_Y+40;
		ascii_str.str_ptr = "100%";
		ap_state_resource_string_ascii_draw((INT16U *)display_buffer, &ascii_str, RGB565_DRAW);
	}
	else {
		DBG_PRINT("Upgrade Fail\r\n");
		ascii_str.pos_x = UPGRADE_POS_X+10;
		ascii_str.pos_y = UPGRADE_POS_Y;
		ascii_str.str_ptr = "Upgrade fail";
		ap_state_resource_string_ascii_draw((INT16U *)display_buffer, &ascii_str, RGB565_DRAW);
	}

	OSQPost(DisplayTaskQ, (void *) (display_buffer|MSG_DISPLAY_TASK_JPEG_DRAW));
	OSQQuery(DisplayTaskQ, &OSQData);
	while(OSQData.OSMsg != NULL) {
		OSTimeDly(2);
		OSQQuery(DisplayTaskQ, &OSQData);
	}

#if  (PWR_KEY_TYPE == READ_FROM_GPIO)
	i=0;
	while(1) {
		#if KEY_ACTIVE
		  if (gpio_read_io(PW_KEY)) {
		#else
		  if (!gpio_read_io(PW_KEY)) {
		#endif
			  i++;
		  }
		  else {
			  i=0;
		  }
		if (i >= 3) {
			#if KEY_ACTIVE
			  while(gpio_read_io(PW_KEY));
			#else
			  while(!gpio_read_io(PW_KEY));
			#endif
			
			sys_ldo33_off();	// turn off LDO 3.3
		}
		OSTimeDly(5);
	}
 #else  // EVB
	 i=0;
	 while(1) {
		 if ( (PW_KEY==PWR_KEY0)&&(sys_pwr_key0_read()) ) {
			 i++;
		 }
		 else if ( (PW_KEY==PWR_KEY1)&&(sys_pwr_key1_read()) ) {
			 i++;
		 }
		 else {
			 i=0;
		 }

		 if (i >= 3) {
			 if (PW_KEY==PWR_KEY0) while(sys_pwr_key0_read());
			 if (PW_KEY==PWR_KEY1) while(sys_pwr_key1_read());
			 sys_ldo33_off();	 // turn off LDO 3.3
		 }
		 OSTimeDly(5);
	 }
 #endif
}
