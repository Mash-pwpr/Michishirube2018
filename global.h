/*
==============================================================
 Name        : global.h
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �O���[�o���ϐ��ł��D�p���X�����͂܂߂ɂ��܂��傤�D


  �X�V����
 2016/1/30	�R��		�R�����g�ǉ�
 2016/2/15	�R��		DR_CENT_BACK��ǉ��A�g���Ă��Ȃ��}�N�����R�����g��
==============================================================
*/

/*�w�b�_�t�@�C���̓ǂݍ���*/
//�e�w�b�_�t�@�C�����ꊇ���ēǂݍ���
#include <stdint.h>

#include "macro.h"
#include "port.h"
#include "auxiliary.h"
#include "drive.h"
#include "sensor.h"
#include "search.h"
#include "uart.h"
#include "init.h"
#include "iodefine.h"

#include "R_PG_IntFuncsExtern.h"
#include "R_PG_RPDL_headers.h"
#include "R_PG_Michishirube.h"
#include "pinRX631.h"

#ifndef GLOBAL_H_			//�Ή��t�@�C���ň�x���ǂݍ��܂�Ă��Ȃ��ꍇ�ȉ����`
#define GLOBAL_H_			//�ǂݍ��񂾂��Ƃ�\��
/*============================================================
		�e��萔�E�ϐ��錾
============================================================*/
/*------------------------------------------------------------
		���s�n
------------------------------------------------------------*/
//----���s�p���X�֘A----
//#define DR_SEC_AD 430		//������1���
//#define DR_SEC_U 430		//����1���
//#define DR_SEC_ADL 360	//��������������1���
#define DR_SEC_HALF 208		//����摖�s�p�p���X�B2�{�����1��敪�� 205
#define DR_ROT_R90 148	//�E90�x��]�p�p���X�� 145
#define	DR_ROT_L90 147  //��90�x��]�p�p���X�� 145
#define DR_ROT_180 294		//180�x��]�p�p���X�� 289
#define DR_CENT_BACK 300	//���ǂɓ��Ă邽�߂ɉ�����p���X��
#define DR_CENT_SET 95		//���ǂ��璆���܂ł̃p���X��
#define CONT 36				//��ᐧ��W��
//----�e�[�u�����x�֘A----
#define MAXSPEED_S 130		//�T�����s���̑��x�̍ő�̗v�f��
#define MINSPEED_S 0		//�T�����s���̑��x�̍ŏ��̗v�f��
#define MAXSPEED_H 287		//�������s���̑��x�̍ő�̗v�f��
#define MINSPEED_H 100		//�������s���̑��x�̍ŏ��̗v�f��

//----�G���R�[�_�EDC���[�^�֘A----
#define DIA_WHEEL_mm 13
#define DIA_PINI_mm 4
#define DIA_SQUR_mm 17.5
#define Pi 3.1415
#define Ktolk 1.98
#define Rmotor 1.07
#define MASS 0.1
#define VOLT_BAT 7.4
#define KR 1
#define KL 1.2
//---PID�Q�C��---
#define KPR 0.1
#define KPL 0.1

#define KDR 0.01
#define KDL 0.01

//----�^�C�}�֘A----
#define DEFGRC 22000		//�f�t�H���g�̃C���^�[�o��

/*------------------------------------------------------------
		�Z���T�nt
------------------------------------------------------------*/
//----�ǔ��f��n----
#define WALL_BASE_F 50		//�O��
#define WALL_BASE_L 55      //����
#define WALL_BASE_R 47		//�E��

//----�����l----
#define SREF_MIN_L -30	//����������-20
#define SREF_MAX_L 280		//���������280
#define SREF_MIN_R -60	//�E��������-20
#define SREF_MAX_R 280 	//�E�������280

/*------------------------------------------------------------
		�T���n
------------------------------------------------------------*/
//----�S�[�����W----
#define GOAL_X 11	//7
#define GOAL_Y 2	//7

/*------------------------------------------------------------
		���p�E�\���̂̒�`
------------------------------------------------------------*/
//���p�E�\���̂Ƃ́A���p�̂ƍ\���̂�g�ݍ��킹�����̂ŁA
//�����̈ꊇ������A�����o�P�ʂł̑�����\�Ȃ��̂ł���B
//�Ⴆ�΁A�ȉ���mouse_flags���p�E�\���̂�MF�ł́A
//MF.FLAGS = 0x10;�ƑS�̂��ꊇ�ύX�ł���ق��A
//MF.FLAG.DECL = 1;�ƃ����o���w�肵�ĒP�̂ŕύX���邱�Ƃ��o����B
//�Q�l�F
//https://www.grapecity.com/tools/support/powernews/column/clang/033/page02.htm

//----�t���O���p�E�\����----
//���Ԃ͊Ԉ���Ă܂���
#ifndef __MOUSE_FLAGS__					//�Ή��t�@�C���ň�x���ǂݍ��܂�Ă��Ȃ��ꍇ�ȉ����`
	#define __MOUSE_FLAGS__				//�ǂݍ��񂾂��Ƃ�\��
	typedef union {						//���p�̂̐錾
		uint16_t FLAGS;
		struct ms_flags{				//�\���̂̐錾
			unsigned char RSV:1;		//�\���r�b�g(B0)		(:1��1�r�b�g���̈Ӗ��A�r�b�g�t�B�[���h)
			unsigned char SCND:1;		//�񎟃t���O(B1)
			unsigned char SLAL:1;		//����t���O(B2)
			unsigned char CTRL:1;		//����t���O(B3)
			unsigned char ACCL:1;		//�����t���O(B4)
			unsigned char DECL:1;		//�����t���O(B5)
			unsigned char DEF:1;		//�葬�t���O(B6)
			unsigned char STOP:1;		//��~�t���O(B7)
			unsigned char R_FRONT:1;	//�\���r�b�g(B8)		
			unsigned char R_BEHIND:1;		//�\���t���O(B9)
			unsigned char L_FRONT:1;		//�\���t���O(B10)
			unsigned char L_BEHIND:1;		//�\���t���O(B11)
			unsigned char S5:1;		//�\���t���O(B12)
			unsigned char S6:1;		//�\���t���O(B13)
			unsigned char S7:1;		//�\���t���O(B14)
			unsigned char S8:1;		//�\���t���O(B15)
		}FLAG;
	} mouse_flags;
#endif

#ifdef EXTERN							//�Ή��t�@�C����EXTERN����`����Ă���ꍇ
	/*�O���[�o���ϐ��̒�`*/
	volatile mouse_flags MF;			//���[�^�X�e�[�^�X���܂Ƃ߂����p�E�\����
#else									//�Ή��t�@�C����EXTERN����`����Ă��Ȃ��ꍇ
	/*�O���[�o���ϐ��̐錾*/
	extern volatile mouse_flags MF;
#endif



//----���ݒn�i�[���p�E�\����----
#ifndef __MAP_COOR__					//�Ή��t�@�C���ň�x���ǂݍ��܂�Ă��Ȃ��ꍇ�ȉ����`
	#define __MAP_COOR__				//�ǂݍ��񂾂��Ƃ�\��
	union map_coor{						//���p�̂̐錾
		unsigned char PLANE;					//YX���W
		struct coor_axis{				//�\���̂̐錾
			unsigned char X:4;					//X���W
			unsigned char Y:4;					//Y���W
		}AXIS;
	};
#endif

#ifdef EXTERN							//�Ή��t�@�C����EXTERN����`����Ă���ꍇ
	/*�O���[�o���ϐ��̒�`*/
	volatile union map_coor PRELOC;		//���ݒn�̍��W���i�[���鋤�p�E�\����
#else									//�Ή��t�@�C����EXTERN����`����Ă��Ȃ��ꍇ
	/*�O���[�o���ϐ��̐錾*/
	extern union map_coor PRELOC;
#endif

#endif /* GLOBAL_H_ */
