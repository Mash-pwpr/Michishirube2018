/*
==============================================================
 Name        : auxiliary.c
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �⏕�֐������ł��D�J���܂��傤�D

 �X�V����
 2015/12/4		�R��	�ꕔ�R�����g�ǉ��Amode_select��LED�\��������mode�̏�����ύX
 2015/1/30		�R��	�ꕔ�R�����g�ǉ��A�\���pLED�̐ݒ��ǉ�
==============================================================
*/

/*�w�b�_�t�@�C���̓ǂݍ���*/
#include "global.h"

//������͔̏ԍ����͓��{��Ń��[�U�[�}�j���A��Rev.00.15�ɏ���

//+++++++++++++++++++++++++++++++++++++++++++++++
//wait
//	ms_wait�͏d�����Ďg�p����(�ȉ��Q��)���Ƃ��o���Ȃ�
//	(���삪�r���Ŏ~�܂�)�B���̂��߁A���荞�݊֐����ł�
//	���m�Ȏ��Ԃ͑���ł��Ȃ���while�����[�v��p���đҋ@����
// ����1�Floop�E�E�E�ҋ@���郋�[�v��
// �߂�l�F����
//+++++++++++++++++++++++++++++++++++++++++++++++
void stay(unsigned int loop){
	while(loop--);					//loop��while������
}
//+++++++++++++++++++++++++++++++++++++++++++++++
//ms_wait
//	�~���b�ҋ@����
// ����1�Fms�E�E�E�ҋ@����[�~���b]
// �߂�l�F����
//+++++++++++++++++++++++++++++++++++++++++++++++
void ms_wait(unsigned int ms){

	//uint16_t timercnt, i;
	// PDG�œ���N���b�N���A�V�X�e���N���b�N20MHz��32�Ŋ����ĕ������Ă���
	R_PG_Timer_StartCount_CMT_U0_C0(); // �J�E���g�X�^�[�g

	/*for(i=0; i<ms; i++){
		R_PG_Timer_SetCounterValue_CMT_U0_C0(0); // �J�E���g�l���[���ɖ߂�
		do{
			R_PG_Timer_GetCounterValue_CMT_U0_C0(&timercnt); // �J�E���g�l���擾
		} while(timercnt < 624 ); // 624�J�E���g�ڂ�1ms�o�������ƂɂȂ�
	}
*/	
	ms_time = 0;
	while(ms_time < ms){
	}
	
	R_PG_Timer_HaltCount_CMT_U0_C0(); // �J�E���g�I��

}

// --��ms_wait���{���Ɗ��荞�݊֐��ŏd�������ꍇ�̗���--
//	�{��							���荞�݊֐�
//	ms_wait�֐����g�p
//	��
//	SysTick�^�C�}�𓮍�J�n
//	��
//	�J�E���^����萔���܂�܂őҋ@
//	��
//	(�ҋ@�����荞�ݔ���)		��	ms_wait�֐����g�p
//								��
//								(SysTick�^�C�}�𓮍�J�n)
//								��
//								�J�E���^����萔���܂�܂őҋ@
//								��
//								�ҋ@�I��
//								��
//								�J�E���^���~�߂�
//								��
//	�J�E���^����萔���܂�܂őҋ@	��	(���荞�ݏI��)
//	��
//	(SysTick�^�C�}�����삵�Ă��Ȃ�����
//	�J�E���^�����܂炸�ȍ~�ҋ@�̂܂ܓ��삹��)


/*------------------------------------------------------------
		���[�h�Z���N�g
------------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//select_mode
//	���[�h�Z���N�g���s��
// ����1�Fmode�E�E�E���[�h�ԍ����i�[����ϐ��̃A�h���X
// �߂�l�F����
//+++++++++++++++++++++++++++++++++++++++++++++++
void select_mode(char *mode){
	uint16_t encR,encL;
	uint16_t nowR = 0;
	uint16_t nowL = 0;
	uint16_t preR,preL;
	
	*mode = 0x00;									//�ϐ��̏�����
	R_PG_Timer_SetCounterValue_MTU_U0_C1(0);
	R_PG_Timer_SetCounterValue_MTU_U0_C2(0);
	
	//====�ϐ�������====
	uart_printf(" mode: 0\r");						//���[�h��UART�ő��M
	
	R_PG_Timer_StartCount_MTU_U0_C1();
	R_PG_Timer_StartCount_MTU_U0_C2();

	//====���[�h�I������====
	do{	
		preR = nowR;
		preL = nowL;
		
		R_PG_Timer_GetCounterValue_MTU_U0_C1(&encL);
		R_PG_Timer_GetCounterValue_MTU_U0_C2(&encR);
		
		nowR = (uint16_t)(encR / 4000);
		nowL = (uint16_t)(encL / 30000);
		
		//ms_wait(50);
		 *mode = nowR;
		//LED�Ō��݂̒l��\��
		pins_write(DISP_LEDS, *mode, 4);			//LED��ActiveLow�̏ꍇ
		if(nowR - preR != 0){
			uart_printf(" mode:%2d\r\n", *mode);
			melody(880,100);
		}
	}while(nowL - preL == 0);
	
	uart_printf("Finish :  This is mode %2d\r\n", *mode);
	melody(1175,500);
	
	R_PG_Timer_HaltCount_MTU_U0_C3();
	R_PG_Timer_HaltCount_MTU_U0_C4();

	R_PG_Timer_SetCounterValue_MTU_U0_C1(0);
	R_PG_Timer_SetCounterValue_MTU_U0_C2(0);
}

void timer_start(){
	R_PG_Timer_StartCount_MTU_U0_C1();
	R_PG_Timer_StartCount_MTU_U0_C2();
	R_PG_Timer_StartCount_MTU_U0_C3();
	R_PG_Timer_StartCount_MTU_U0_C4();

	R_PG_Timer_StartCount_CMT_U0_C1();

}

void melody(uint16_t Hz, uint16_t length){
	float count = (float)1000000/Hz;
	
	count = count*54239;
	count = count/1130;
	
	R_PG_Timer_SetTGR_A_MTU_U0_C0((uint16_t)(count/10));
	R_PG_Timer_SetTGR_B_MTU_U0_C0((uint16_t)count);
	
	R_PG_Timer_StartCount_MTU_U0_C0();
	ms_wait(length);
	R_PG_Timer_HaltCount_MTU_U0_C0();
	
}

void start_wait(){
	R_PG_Timer_StartCount_CMT_U0_C1();
	uart_printf("Ready???\r\n");
	
	while(1){
		uart_printf("ad_l: %4d ad_fl:%4d ad_fr:%4d ad_r:%4d \r\n", ad_l, ad_fl, ad_fr, ad_r);
		if(ad_ff > 1000){
			melody(1320,300);
			melody(1397,300);
			melody(1568,300);
			ms_wait(1000);
			break;
		}
	}
	
	R_PG_Timer_HaltCount_CMT_U0_C1();
	
	pin_write(PE0,0);
	pin_write(PE1,0);
	pin_write(PE2,0);
	pin_write(PE3,0);
	pin_write(PE4,0);

	
}

void start_ready(void){
			turn_R90();
			set_position();
			ms_wait(200);
			turn_L90();
			set_position();
			ms_wait(200);
			
			get_base();

}