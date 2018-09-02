/************************************************************
		Micromouse �v���O����
		2007�N~2012�N	H8/3694f�ł��^�p�����
		2013/03/01		LPC�����ɈڐA					���J�� ���l
		2013/09/20		�W�����̂��ߐF�X�M��				���� �L��
 ************************************************************/
/*
==============================================================
 Name        : main.c
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : main definition

 �X�V����
 2016/2/12�@�[�R�@�ꕔ�R�����g�ǉ�
 2017/6/27 �W�@�A���T�����s�̖������ł�����
==============================================================
 */

#define EXTERN
#include "global.h"

/*===========================================================
		���C���֐�
===========================================================*/
int main(void) {

	//====�ϐ��錾====
	char mode = 0;
	uint16_t i,j;
	
	
	
	//====������====
	R_PG_Clock_Set();					//�N���b�N�ݒ�

	port_Init();						//port�̏�����
	val_Init();							//�e��ϐ��̏�����
	timer_Init();						//�^�C�}�n�̏�����
	sensor_Init();						//�Z���T�n�̏�����
	uart_Init();					//�V���A���ʐM�̏�����	
	
	//timer_start();
/*	while(1){
		
		for(i=10;i<60000;i+=10){
			melody(i,100);
		}
		i = 0x75;
		//R_PG_RSPI_TransferAllData_C0(&i,&test1,1);
		//R_PG_RSPI_StartTransfer_C0(0x47,&test1,1);
		//test_s = test1 + test2;
		uart_printf("Who am I : \r\n");
		uart_printf("0x%x\r\n",test1);
		ms_wait(1000);
		
	}
*/

	duty_r = KR * Kvolt * accel_r / VOLT_BAT;
	duty_l = KL * Kvolt * accel_l / VOLT_BAT;
	i = (1 - duty_r) * 65535;
	j = (1 - duty_l) * 65535;
	
	//uart_printf("R:%lf\tL%lf\ti:%d\tj:%d\r\n",duty_r,duty_l,i,j);
	targ_vel_L = 1.0;
	targ_vel_R = 1.0;
		
	
			//while(1){
				
				set_dir(FORWARD);
				sensor_start();
				drive_start();
				do{
					
				uart_printf("Rmm is %lf duty_r is %lf,duty_oR is %lf, vel_R is %lf\r\n",totalR_mm,duty_r, duty_oR,vel_R);
				//ms_wait(10);
					/*if(volt_bat < 3650 && 1000 < volt_bat){
						melody(1320,2000);
						R_PG_Timer_StopModule_MTU_U0();
						R_PG_Timer_StopModule_CMT_U0();
						while(1){
						}
					}*/
				}while(vel_R < 1 );
					
				uart_printf("END\r\n");
				ms_wait(100);
				do{
					targ_vel_L = 0;
					targ_vel_R = 0;
						
				}while(vel_R > 0);
			
				drive_stop(1);
				R_PG_Timer_HaltCount_CMT_U0_C1();
				ms_wait(10000);				
				
			//}
			if(volt_bat < 3650 && 1000 < volt_bat){
				melody(1320,2000);
				R_PG_Timer_StopModule_MTU_U0();
				R_PG_Timer_StopModule_CMT_U0();
				uart_printf("Voltage Out\r\n");
				while(1){
				}
			}

	while(1){ // Main Loop
		uart_printf("Hello, World! Kvolt = %lf \r\n",Kvolt);	
		//====���[�h�Z���N�g====
		select_mode(&mode);
		ms_wait(100);
		//----�I�����ڂ̎��s----
		switch(mode){
		case 0:	//----��l�����----
			get_base();											//get_base()��sensor.c�Ɋ֐���`���� �@����̂��߂̕Ǌ�l�擾
			//----�����V���A�����M----
			//uart_printf("base_l = %3d, ", base_l);				//UART_printf()��uart.c�Ɋ֐���`����
			//uart_printf("base_r = %3d\r", base_r);
			ms_wait(500);
			uart_printf("START");
			for(i=0;i<300;i++){
				uart_printf("%lf,%lf\r\n",test_valR[i],test_valL[i]);
				ms_wait(100);
			}
			uart_printf("ALL");

			break;
		case 1:	//----�ꎟ�T�����s----
			goal_x = GOAL_X;									//�S�[�����W��ݒ�@�@GOAL_X��global.h�Ƀ}�N����`����
			goal_y = GOAL_Y;									//�S�[�����W��ݒ�@�@GOAL_Y��global.h�Ƀ}�N����`����

			start_ready();
			start_wait();

			get_wall_info();									//�Ǐ��̏�����     get_wall_info()��sensor.c�Ɋ֐���`����
			searchA();											//�S�[�����܂ŒT���C�i�s����@searchA()��search.c�Ɋ֐���`����
			goal_x = goal_y = 0;								//�S�[�����W���X�^�[�g���ɐݒ�
			Wait;												//�ҋ@
			searchA();											//�߂��Ă���

			goal_x = GOAL_X;									//�S�[�����W�ݒ�
			goal_y = GOAL_Y;									//�S�[�����W�ݒ�

			
			break;

			//----�A���T�����s----
		case 2:
			goal_x = GOAL_X;
			goal_y = GOAL_Y;

			
			start_ready();
			start_wait();
			
			searchSA();
			goal_x = goal_y = 0;
			searchSA();
			goal_x = GOAL_X;
			goal_y = GOAL_Y;

			turn_180();									//180�x��]
			turn_dir(DIR_TURN_180);
			break;

			/////////////////////////////////�@�@���̓񎟒T�����s�ƃX�����[�����s�͖�����
			//----�񎟍������s----
		case 3:
		
			
			break;

			//----�X�����[�����s----
		case 4:
			break;
			//////////////////////////////////

			//----���s�e�X�g----
		case 5:
			Wait;
			start_wait();
			set_dir(FORWARD);
			drive_start();
			while(1){
				uart_printf("Driving\r\n");
			}
			test_drive(&mode);									//test_drive()��drive.c�Ɋ֐���`����
			ms_wait(100);
			break;
			//----�G���R�[�_�e�X�g----
		case 6:
			Wait;
			start_wait();
			enc_test();
			ms_wait(100);
			break;
			
			
			//----�Z���T�l, �����m�F----
			//LED�_���͗v�ύX
		default:
			Wait;
			val_Init();
			MF.FLAG.CTRL = 1;	//���䋖��
			R_PG_Timer_StartCount_CMT_U0_C1();
			while(1){
				pins_write(DISP_LEDS, 0, LED_NUM);											//pins_write()��port.c�Ɋ֐���`����
				uart_printf("ad_l: %4d ad_fl:%4d ad_ff:%4d  ad_fr:%4d ad_r:%4d ", ad_l, ad_fl, ad_ff, ad_fr, ad_r);
				uart_printf(" | dif_l: %4d dif_r:%4d\r\n", dif_l, dif_r);
				//----LED��4�̏ꍇ----
				if(ad_fr > WALL_BASE_F){
					// �����Aad_l�ɂȂ��Ă܂�����I�I
					pin_write(DISP_LEDS[0], ON);											//pin_write()��port.c�Ɋ֐���`����
				}
				if(ad_r > WALL_BASE_R){
					pin_write(DISP_LEDS[1], ON);
				}
				if(ad_l > WALL_BASE_L){
					pin_write(DISP_LEDS[2], ON);
				}
				if(ad_fl > WALL_BASE_F){
					pin_write(DISP_LEDS[3], ON);
				}
				ms_wait(1000);

			}
			MF.FLAG.CTRL = 0;
			ms_wait(100);
			break;
		}
		ms_wait(100);
	}

	return 0 ;
}
