/*
==============================================================
 Name        : sensor.h
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �Z���T�֘A�̊֐������ł��D���ߕ��܂��傤�D

  �X�V����
 2016/1/29		�R��	�R�����g�ǉ�
 2016/2/24		�R��	2016�N�x�p�Ƀs���ݒ��ύX
==============================================================
*/


#include "pinRX631.h"
#include <stdint.h>
#ifndef SENSOR_H_											//�Ή��t�@�C���ň�x���ǂݍ��܂�Ă��Ȃ��ꍇ�ȉ����`
	#define SENSOR_H_										//�ǂݍ��񂾂��Ƃ�\��

/*============================================================
		�e��萔��ϐ��錾
============================================================*/
	//----LED��ON/OFF�o��----
	#define VLED_ON		1
	#define VLED_OFF	0

	#define SHIFT		6									//A-D�ϊ��V�t�g�l

	//====�ϐ�====
	#ifdef EXTERN
		/*�O���[�o���ϐ��̒�`*/
		//----LED�|�[�g----
		uint16_t VLED_FL = PE0;								//�O���������Z���T��������LED�̃|�[�g
		uint16_t VLED_L = PE1;									//�����Z���T��������LED�̃|�[�g
		uint16_t VLED_FF = PE2;
		uint16_t VLED_R = PE3;								//�E���Z���T��������LED�̃|�[�g
		uint16_t VLED_FR = PE4;								//�O�����E���Z���T��������LED�̃|�[�g
		//----���̑�----
		unsigned char tp;											//�^�X�N�|�C���^
		int16_t ad_r, ad_fr, ad_ff, ad_fl, ad_l;						//A-D�l�i�[
		uint16_t ad_res[5],ad_pre_res[5];
		uint16_t base_l, base_r;								//��l���i�[
		int16_t dif_l, dif_r;									//AD�l�Ɗ�Ƃ̍�
		uint16_t volt_bat;									//�d���d���Ď�
	#else
		//----����LED�|�[�g----
		extern uint16_t VLED_R;
		extern uint16_t VLED_FR;
		extern uint16_t VLED_FF;
		extern uint16_t VLED_FL;
		extern uint16_t VLED_L;
		//----���̑�----
		extern unsigned char tp;
		extern int16_t ad_r, ad_fr,ad_ff, ad_fl, ad_l;
		extern uint16_t ad_res[5],ad_pre_res[5];
		extern uint16_t base_l,  base_r;
		extern int16_t dif_l, dif_r;
		extern uint16_t volt_bat;
	#endif

/*============================================================
		�֐��v���g�^�C�v�錾
============================================================*/
	//====�Z���T�n====
	unsigned char get_base();					//�Z���T��l���擾
	void get_wall_info();				//�Ǐ���ǂ�
	void enc_test();
	void sensor_start();
	
#endif /* SENSOR_H_ */
