/*
==============================================================
 Name        : init.c
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �������֐������ł��D

 �X�V����
 2015/12/4		�R��	�ꕔ�R�����g�ǉ��A�\���pLED�̐ݒ��ǉ�
 2016/2/6		�[�R	�ꕔ�R�����g�ǉ�
 2016/2/8		�R��	port_Init�֐�����PIN_SET(MDR_M3, 0);��ǉ�
 2016/2/12		�R��	sensor_Init�֐����ɔ�������IOCON�ݒ��ǉ�
 2016/2/24		�R��	2016�N�x�p�Ƀs���ݒ��ύX
==============================================================
 */

#include "global.h"
/*============================================================
		�������֐�
============================================================*/
// �e����o�͂̐ݒ�
void port_Init(void){
	PORTB.PMR.BIT.B6 = 0;
	PORTB.PMR.BIT.B7 = 0;

	PORTB.PDR.BIT.B6 = 0;
	PORTB.PDR.BIT.B7 = 0;
	
	PORTB.PCR.BIT.B6 = 0;
	PORTB.PCR.BIT.B7 = 0;
	
	PORT.PSRA.BIT.PSEL6 = 0;
	PORT.PSRA.BIT.PSEL7 = 0;
	
	//I/O�֌W�̃|�[�g�ݒ�
	R_PG_IO_PORT_Set_P5();		//���^�h���ւ̎w��
	R_PG_IO_PORT_Set_PA();		//���^�h���ւ̎w��
	R_PG_IO_PORT_Set_PC();		//

	PORTB.PDR.BIT.B6 = 1;
	PORTB.PDR.BIT.B7 = 1;
	PORTB.PODR.BIT.B6 = 1;
	PORTB.PODR.BIT.B7 = 1;
				
	PIN_L(MDR_REF);				//���t�@�����X��؂�@�s��2_1�̏o�͂�Low�ɐݒ�

	set_dir(FORWARD);			//�i�s������O��

}


// �Z���T�[�ݒ�
void sensor_Init(void){
	//�Z���T������̃|�[�g�ݒ� �����珇��
	R_PG_ADC_12_Set_S12AD0();
	//�Z���T�����������ݒ�
	R_PG_IO_PORT_Set_PE();
}


// �Z���T�n�C���s�n�C�T���n�@�̕ϐ�������
void val_Init(void){
	//----�Z���T�n----
	tp = 0;
	ad_l = ad_r = ad_fr = ad_fl = 0;
	base_l = base_r = 0;
	pulse_sum_l = pulse_sum_r = 0;
	pulse_pre_l = pulse_pre_r = 0;
	time = 0;
	totalR_pre_mm = totalL_pre_mm = 0;
	totalR_mm = totalL_mm = 0;
	dif_pre_vel_R = dif_pre_vel_L = 0;
		
	accel_r = 0.5;
	accel_l = 0.5;
	
	//----���s�n----
	maxindex = MAXSPEED_S;			//�ō����x������     MAXSPEED_S��global.h�Ƀ}�N����`����
	minindex = MINSPEED_S;			//�Œᑬ�x������     MINSPEED_S��global.h�Ƀ}�N����`����
	MF.FLAGS = 0x80;			//�t���O�N���A����~���  0x80=0b10000000

	//----�T���n----
	goal_x = GOAL_X;        		//GOAL_X��global.h�Ƀ}�N����`����
	goal_y = GOAL_Y;        		//GOAL_Y��global.h�Ƀ}�N����`����
	map_Init();						//�}�b�v�̏�����
	PRELOC.PLANE = 0x00;			//���ݒn�̏�����
	m_dir = 0;				//�}�E�X�����̏�����

	Kvolt = MASS / 2 * DIA_SQUR_mm / DIA_PINI_mm * DIA_WHEEL_mm / Ktolk * Rmotor;
	
}

// �^�C�}������
void timer_Init(void){

	R_PG_Timer_Set_CMT_U0_C0();
	R_PG_Timer_Set_CMT_U0_C1();

	R_PG_Timer_Set_MTU_U0_C0();		//�u�U�[�p�����ݒ�
	R_PG_Timer_Set_MTU_U0_C1();		//�E�G���R�[�_
	R_PG_Timer_Set_MTU_U0_C2();		//���G���R�[�_
	R_PG_Timer_Set_MTU_U0_C3();		//�����[�^
	R_PG_Timer_Set_MTU_U0_C4();		//�E���[�^

}
