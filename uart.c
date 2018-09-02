/*
==============================================================
 Name        : uart.c
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : �ʐM�p�֐������ł��D

  �X�V����
 2015/12/10		�R��	�R�����g�ǉ��Auart_Init��LCR������ύX
==============================================================
*/

//������͔̏ԍ����͓��{��Ń��[�U�[�}�j���A��Rev.00.15�ɏ���

/*�w�b�_�t�@�C���̓ǂݍ���*/
#include "global.h"
#include <stdio.h>
#include <stdarg.h>															// �ψ������X�g���g�p

//+++++++++++++++++++++++++++++++++++++++++++++++
//uart_Init
//	UART�̏����ݒ���s��
// ����1�Fbaudrate�E�E�E�ʐM�̃{�[���[�g
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void uart_Init(){
	R_PG_SCI_Set_C1();
	R_PG_RSPI_Set_C0();
	R_PG_RSPI_SetCommand_C0();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//UART_printf
//	UART�ŕW���o�͂��s��
//	�Ή��w��q	�F%s,%c,%d,%u,%X,%ld,%lu,%lX
//				�@���l�̓��[�f�B���O�[���ƕ\�������w��\
// ����1�Fformat�E�E�E���M����{��
// ����2�ȍ~:�E�E�E�ψ���(printf���̖{���ȍ~�̈����̂悤�Ȃ���)
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++

void uart_printf(const char *format, ...) {
    va_list arg;
    uint8_t buf[64];
    uint8_t *p;

    va_start(arg, format);

    vsprintf((char*)buf, format, arg);
    p = buf;
    while( *p != '\0' ) {
        R_PG_SCI_SendAllData_C1(p, 1);
        p++;
    }
    va_end(arg);
}

void Spi0IntFunc(){
	//R_PG_RSPI_StopModule_C0();
	while(1){
		uart_printf("ERROR...\r\n");
	}
}
