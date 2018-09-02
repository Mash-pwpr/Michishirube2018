/*
==============================================================
 Name        : port.c
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �e��s���̓��o�͂���т��̐ݒ���s��

 �X�V����
 2015/12/4		�R��@�R�����g�ǉ�
==============================================================
*/



/*�w�b�_�t�@�C���̓ǂݍ���*/
#include "port.h"
#include "r_pdl_io_port.h"
#include "r_pdl_definitions.h"

//+++++++++++++++++++++++++++++++++++++++++++++++
//pin_write
//	�w�肳�ꂽ�s����High�܂���Low�ɂ���
// ����1�Fpin�E�E�E�s���ԍ�(Px_y)
// ����2�Fdat�E�E�EHigh��Low��(0:Low,����ȊO:High)
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void pin_write(uint16_t pin, unsigned char dat){

	R_IO_PORT_Write(pin,dat);

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//pins_write
//	�w�肳�ꂽ�s���z���High�܂���Low�ɂ���
// ����1�Fpin�E�E�E�s���ԍ��̔z��(Px_y)
// ����2�Fdat�E�E�EHigh��Low���̃r�b�g��(0:Low,1:High)
// ����3�Fcnt�E�E�E�s���̌�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void pins_write(uint16_t pin[], unsigned char dat, unsigned char cnt){
	//�ϐ��̒�`
	int i;


	for(i=0; i<cnt; i++) PIN_SET(pin[i], bit_is_set(dat, i));		//�e�s���ɑΉ�����DATA���W�X�^�̏ꏊ��ύX
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//pin_dir
//	�w�肳�ꂽ�s���̓��o�͂�ݒ肷��
// ����1�Fpin�E�E�E�s���ԍ�(Px_y)
// ����2�Fdir�E�E�E���͂��o�͂�(0:����,����ȊO:�o��)
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void pin_dir(uint16_t pin, unsigned char dir){

	R_IO_PORT_Write(pin,dir);
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//pins_dir
//	�w�肳�ꂽ�s���z��̓��o�͂�ݒ肷��
// ����1�Fpin�E�E�E�s���ԍ�(Px_y)
// ����2�Fdir�E�E�E���͂��o�͂��̃r�b�g��(0:����,1:�o��)
// ����3�Fcnt�E�E�E�s���̌�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void pins_dir(uint16_t pin[], unsigned char dir, unsigned char cnt){
	//�ϐ��̒�`
	int i;


	for(i=0; i<cnt; i++) pin_dir(pin[i], dir);						//�e�s����pin_dir�����s

}
