/*
==============================================================
 Name        : macro.h
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �}�N���錾�ł��D�ϐ��^�Ƃ��Ŋy�Ɏg����悤�ɂȂ�܂��D

  �X�V����
 2015/12/13		�R��	�R�����g�ǉ�
==============================================================
*/

#ifndef MACRO_H_												//�Ή��t�@�C���ň�x���ǂݍ��܂�Ă��Ȃ��ꍇ�ȉ����`
	#define MACRO_H_											//�ǂݍ��񂾂��Ƃ�\��

	//�����̃v���O�����𓮂����̂ɕK�v�ȃ}�N���B�o�Ă�����ǂޒ��x�ł����ł��B
	#define _BV(bit)			(1<<(bit))						//�w��r�b�g��1�����փr�b�g�V�t�g�����l
	#define cbi(addr,bit)	addr &= ~_BV(bit)					//�w�背�W�X�^�̎w��r�b�g��0�ɂ���
	#define sbi(addr,bit)	addr |= _BV(bit)					//�w�背�W�X�^�̎w��r�b�g��1�ɂ���
	#define bit_is_set(addr,bit)	(addr & _BV(bit) ? 1 : 0)	//�w�背�W�X�^�̎w��r�b�g��1�Ȃ�1�A0�Ȃ�0
	#define bit_is_clear(addr,bit)	(addr & _BV(bit) ? 0 : 1)	//�w�背�W�X�^�̎w��r�b�g��0�Ȃ�1�A1�Ȃ�0
	#define outp(data,addr)	addr = (data)						//�w�背�W�X�^���w��f�[�^�ɂ���
	#define inp(addr)		(addr)								//�w�背�W�X�^�����o��
	#define outb(addr,data)	addr = (data)						//�w�背�W�X�^���w��f�[�^�ɂ���
	#define inb(addr)		(addr)								//�w�背�W�X�^�����o��
	#define PRG_RDB(addr)	(pgm_read_byte(addr))				//�v���O�����������̓ǂݏo���H


	//�^�̒u�������A�����L�̌^����ʓI�Ȍ^�ɒu��������B�ڐA���ɂ͂�����ς���΂����̂Ŋy�ɂȂ�(�S���u��������Ă��Ȃ��C�����邯��...)
	// 8-bit, int8_t=signed char
/*	typedef char			CHAR;
	typedef unsigned char	UCHAR;
	// 16-bit, SHORT�Ȃ�ĂȂ�����
	typedef int16_t			INT;
	typedef uint16_t		UINT;
	// 32-bit
	typedef int32_t			LONG;
	typedef uint32_t		ULONG;
/*
	// 8-bit
	typedef char			CHAR;
	typedef unsigned char	UCHAR;
	typedef unsigned char	BYTE;

	// 16-bit
	typedef short			SHORT;
	typedef unsigned short	USHORT;
	typedef unsigned short	WORD;

	// 32-bit
	typedef long			LONG;
	typedef unsigned long	ULONG;
	typedef unsigned long	DWORD;
*/
	// �u�[�����Z�q
	typedef enum { FALSE = 0, TRUE } BOOL;


	//�����̃v���O�����𓮂����̂ɕK�v�ȃ}�N���B�����g���ĂȂ��ł�
	#ifndef	NOP														//�ҋ@��\���}�N��
	#define	NOP		asm volatile("nop\n"::)							//�A�Z���u������𒼐ڏ����Ă�H
	#endif
	#ifndef SLEEP													//�ҋ@��\���}�N��
	#define	SLEEP	asm volatile("sleep\n"::)						//�A�Z���u������𒼐ڏ����Ă�H
	#endif

	//���̑���`
	#ifndef NULL													//NULL���Ή��t�@�C���Œ�`����Ă��Ȃ��ꍇ
	#define NULL    0												//0�Ƃ��Ē�`����
	#endif

	// Math 2011/03/29
	#define ABS(x)	((x) < 0 ? -(x) : (x))							//��Βl��Ԃ��}�N��

#endif /* MACRO_H_ */
