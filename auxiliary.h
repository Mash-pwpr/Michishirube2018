/*
==============================================================
 Name        : auxiliary.h
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �⏕�֐������ł��D�J���܂��傤�D

 �X�V����
 2015/1/30		�R��	�ꕔ�R�����g�ǉ�
 2016/2/24		�R��	2016�N�x�p�Ƀs���ݒ��ύX
==============================================================
*/
#include"pinRX631.h"

#ifndef AUXILIARY_H_									//�Ή��t�@�C���ň�x���ǂݍ��܂�Ă��Ȃ��ꍇ�ȉ����`
	#define AUXILIARY_H_								//�ǂݍ��񂾂��Ƃ�\��

/*============================================================
		�e��萔��ϐ��錾
============================================================*/
	//----�X�C�b�`�֘A----
	//Custom
	#define	INC		1	//���Z�{�^��
	#define	RET		1	//����{�^��
	#define	DEC		1	//���Z�{�^��

	//----�ėp----
	#define ON 1
	#define OFF 0

	//----�\���pLED�֘A----
	#ifdef EXTERN										//�Ή��t�@�C����EXTERN����`����Ă���ꍇ
		/*�O���[�o���ϐ��̒�`*/
		uint16_t DISP_LEDS[4] = {PB6, PB7, PC2,PC3};	//�\���pLED�̃s���̔z��
	#else												//�Ή��t�@�C����EXTERN����`����Ă��Ȃ��ꍇ
		/*�O���[�o���ϐ��̐錾*/
		extern uint16_t DISP_LEDS[4];
	#endif

	//----LED�̐�----
	#define LED_NUM 4

	//----���̑�----
	#define DEFWAIT		10000							//wait()�p�萔
	#define Wait		ms_wait(200)					//�����ҋ@����

/*============================================================
		�֐��v���g�^�C�v�錾
============================================================*/
	//====���̑�====
	void stay(unsigned int);
	void ms_wait(unsigned int);
	void select_mode(char *mode);						//���[�h�̓���
	void timer_start();
	void melody(uint16_t,uint16_t);
	void start_wait();
	void start_ready();

#endif /* AUXILIARY_H_ */
