/*
==============================================================
 Name        : port.h
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �e��s���̓��o�͂���т��̐ݒ���s��

 �X�V����
 2015/12/4		�R��@�R�����g�ǉ�
==============================================================
*/

/*�w�b�_�t�@�C���̓ǂݍ���*/
#include "macro.h"
#include"stdint.h"

#ifndef PORT_H_											//�Ή��t�@�C���ň�x���ǂݍ��܂�Ă��Ȃ��ꍇ�ȉ����`

	#define PORT_H_										//�ǂݍ��񂾂��Ƃ�\��


	enum {												//GPIO�̎q�ԍ��̗񋓌^
		P0_0, P0_1, P0_2, P0_3,
		P0_4, P0_5, P0_6, P0_7,
		P0_8, P0_9, P0_10, P0_11,
		P0_12, P0_13, P0_14, P0_15,

		P1_0, P1_1, P1_2, P1_3,
		P1_4, P1_5, P1_6, P1_7,
		P1_8, P1_9, P1_10, P1_11,
		P1_12, P1_13, P1_14, P1_15,

		P2_0, P2_1, P2_2, P2_3,
		P2_4, P2_5, P2_6, P2_7,
		P2_8, P2_9, P2_10, P2_11,
		P2_12, P2_13, P2_14, P2_15,

		P3_0, P3_1, P3_2, P3_3,
		P3_4, P3_5
	};

	enum{												//�s���̓��o�̗͂񋓌^
		IN,												//����(=0)
		OUT												//�o��(=1)
	};


	/*�}�N���̒�`*/
	#define PIN_H(pin)			pin_write(pin, 1)									//�Ή��s���̏o�͂�High�ɂ���
	#define PIN_L(pin)			pin_write(pin, 0)									//�Ή��s���̏o�͂�Low�ɂ���
	#define PIN_SET(pin, dat)	pin_write(pin, dat)									//�Ή��s���̏o�͂�ݒ�

	#define PIN_DIR(pin, dir)	PIN_SET(pin, dir)									//�Ή��s���̓��o�͂�ݒ�

/*	#define PIN_IS_SET(pin)		bit_is_set(LPC_GPIO[pin>>4]->DATA, pin&0x0f)		//�Ή��s���̓��͂�High������
	#define PIN_IS_CLEAR(pin)	bit_is_clear(LPC_GPIO[pin>>4]->DATA, pin&0x0f)		//�Ή��s���̓��͂�Low������
	#define PIN_STAT(pin)		PIN_IS_SET(pin)										//�Ή��s���̓��͂�Ԃ�
*/

	/*�֐��̃v���g�^�C�v�錾*/
	void pin_write(uint16_t pin, unsigned char dat);
	void pins_write(uint16_t pin[], unsigned char dat, unsigned char cnt);
	void pin_dir(uint16_t pin, unsigned char dir);
	void pins_dir(uint16_t pin[], unsigned char dir, unsigned char cnt);

#endif /* PORT_H_ */
