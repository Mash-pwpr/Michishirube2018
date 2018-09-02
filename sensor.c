/*
==============================================================
 Name        : sensor.c
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �Z���T�֘A�̊֐������ł��D

  �X�V����
 2016/1/29		�R��	�R�����g�ǉ��Aget_wall_info�֐����̎n�߂�
 	 	 	 	 	pins_write�֐��̑�3������6����LED_NUM�ɕύX
 2016/2/24		�R��	2016�N�x�p�Ƀs���ݒ��ύX
==============================================================
*/

//������͔̏ԍ����͓��{��Ń��[�U�[�}�j���A��Rev.00.15�ɏ���

/*�w�b�_�t�@�C���̓ǂݍ���*/
#include "global.h"

//+++++++++++++++++++++++++++++++++++++++++++++++
//get_base
//	����p�̊�l���擾����
// �����F�Ȃ�
// �߂�l�F���z�I�Ȓl���擾�ł������@1:�ł����@0:�ł��Ȃ�����
//+++++++++++++++++++++++++++++++++++++++++++++++
unsigned char get_base()
{
	unsigned char res = 0;										//���z�I�Ȓl���擾�ł�����

	//----����p�̊���擾----
	base_l = ad_l;										//���݂̍����̃Z���T�l�Ō���
	base_r = ad_r;										//���݂̉E���̃Z���T�l�Ō���

	//----������z�I����LED�_��----
	if((-50 < (int)(base_l - base_r)) && ((int)(base_l - base_r) < 50)){
		//���E�ō���50�ȉ��ł���ꍇ
		pin_write(DISP_LEDS[0], 0x0);					//0�Ԗڂ�LED������
		pins_write(DISP_LEDS, 0x0f, LED_NUM);			//LED��S�_��
		pins_write(DISP_LEDS, 0x00, LED_NUM);			//LED��S����
		res = 1;										//res��1��
	}else{
	}
	//ms_wait(50);

	return res;											//���z�I�Ȓl���擾�ł�������Ԃ�

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//get_wall_info
//	�Ǐ��擾���擾����
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void get_wall_info()
{
	unsigned char tmp = 0;						//�_��������LED�̏ꏊ
	//----�Ǐ��̏�����----
	wall_info = 0x00;									//�Ǐ���������
	pins_write(DISP_LEDS, 0, LED_NUM);					//LED��S����

	//----�O�ǂ�����----
	if(ad_fr > WALL_BASE_F){
		//AD�l��臒l���傫��(=�ǂ������Č������˕Ԃ��Ă��Ă���)�ꍇ
		wall_info |= 0x88;								//�Ǐ����X�V
		tmp = 0x06;										//1�Ԗڂ�2�Ԗڂ�LED��_��������悤�ݒ�
	}
	//----�E�ǂ�����----
	if(ad_r > WALL_BASE_R){
		//AD�l��臒l���傫��(=�ǂ������Č������˕Ԃ��Ă��Ă���)�ꍇ
		wall_info |= 0x44;								//�Ǐ����X�V
		tmp |= 0x01;									//0�Ԗڂ�LED��_��������悤�ݒ�
	}
	//----���ǂ�����----
	if(ad_l > WALL_BASE_L){
		//AD�l��臒l���傫��(=�ǂ������Č������˕Ԃ��Ă��Ă���)�ꍇ
		wall_info |= 0x11;								//�Ǐ����X�V
		tmp |= 0x08;									//3�Ԗڂ�LED��_��������悤�ݒ�
	}

	pins_write(DISP_LEDS, tmp, LED_NUM);				//LED��_��������
}

void enc_test(){
	totalR_mm = totalL_mm = 0;
	
	R_PG_Timer_StartCount_MTU_U0_C1();
	R_PG_Timer_StartCount_MTU_U0_C2();
	
	
	while(1){
		R_PG_Timer_GetCounterValue_MTU_U0_C1(&pulse_l);
		R_PG_Timer_GetCounterValue_MTU_U0_C2(&pulse_r);
		
		if(pulse_sum_r == 1){				//�A���_�[�t���[��
			dif_pulse_r = (pulse_r - 65535) + pulse_pre_r;
			pulse_sum_r = 0;
		}else if(pulse_sum_r == 2){			//�I�[�o�[�t���[��
			dif_pulse_r = pulse_r + (65535 - pulse_pre_r);
			pulse_sum_r = 0;
		}else{
			dif_pulse_r = pulse_r - pulse_pre_r;
		}
		
		if(pulse_sum_l == 1){				//�A���_�[�t���[��
			dif_pulse_l = (pulse_l - 65535) + pulse_pre_l;
			pulse_sum_l = 0;
		}else if(pulse_sum_l == 2){			//�I�[�o�[�t���[��
			dif_pulse_l = pulse_l + (65535 - pulse_pre_l);
			pulse_sum_l = 0;
		}else {
			dif_pulse_l = pulse_l - pulse_pre_l;
		}

		
		totalR_mm += -DIA_WHEEL_mm * (DIA_PINI_mm / DIA_SQUR_mm) * 2 * Pi * (dif_pulse_r % 4096) / 4096;
		totalL_mm += -DIA_WHEEL_mm * (DIA_PINI_mm / DIA_SQUR_mm) * 2 * Pi * (dif_pulse_l % 4096) / 4096;
		
		pulse_pre_r = pulse_r;
		pulse_pre_l = pulse_l;
		
		uart_printf(" pulse_r: %4d  pulse_l:%4d totalR_mm:%4lf totalL_mm:%4lf \r\n", pulse_r, pulse_l, totalR_mm, totalL_mm);	
	}

}

void sensor_start(){
	R_PG_Timer_StartCount_MTU_U0_C1();
	R_PG_Timer_StartCount_MTU_U0_C2();
	R_PG_Timer_StartCount_CMT_U0_C1();	
}