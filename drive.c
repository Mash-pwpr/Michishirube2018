/*
==============================================================
 Name        : drive.c
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : ���s�֘A�̊֐������ł��D

  �X�V����
 2016/1/29		�R��	�R�����g�ǉ��Atest_drive��ms_wait��WAIT�𓝍�
 2016/2/15		�R��	�p���X�֌W�̃}�N���𐮗�
==============================================================
*/
#include "global.h"

//������͔̏ԍ����͓��{��Ń��[�U�[�}�j���A��Rev.00.15�ɏ���

/*==========================================================
		���s�n�֐�
==========================================================*/
/*
		��{�d�l�Ƃ��āA
		��֐�		������:���s�p���X��
						������:��~���t���O

		�}�E�X�t���O(MF)
			7Bit:�X�g�b�v�t���O
			6Bit:�葬�t���O
			5Bit:�����t���O
			4Bit:�����t���O
			3Bit:����t���O
			2Bit:�X�����[���t���O
			1Bit:�񎟑��s�t���O
			0Bit:�\��r�b�g
		���4Bit�́A��ʂقǗD��x������
*/
/*----------------------------------------------------------
		��ʊ֐�
----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionA
//	����敪�������Ȃ��瑖�s����
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA()
{
	MF.FLAG.CTRL = 1;										//�����L���ɂ���
	driveA(DR_SEC_HALF, 0);									//�����̃p���X���������Ȃ��瑖�s�B���s��͒�~���Ȃ�
	get_wall_info();										//�Ǐ����擾
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionD
//	����敪�������Ȃ��瑖�s����~����
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD()
{
	MF.FLAG.CTRL = 1;										
	driveD(DR_SEC_HALF, 1);									//�����̃p���X���������Ȃ��瑖�s�B���s��͒�~����
	Wait;													
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//a_section
//	1��敪�i��Œ�~����
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void a_section()
{
	half_sectionA();			//����敪�������s
	half_sectionD();										
}

void s_section(){
	half_sectionA();			//����敪�������s
	half_sectionA();										
	
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//a_sectionU
//	������1��敪�i��
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void a_sectionU() {
	MF.FLAG.CTRL = 1;
	driveU(DR_SEC_HALF*2, 0);		//1���̃p���X���������s�B���s��͒�~���Ȃ�
	get_wall_info();			//�Ǐ����擾
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//turn_R90
//	�E��90�x��]����
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R90(){
	MF.FLAG.CTRL = 0;										//����𖳌��ɂ���
	set_dir(TURN_R);										//�E�ɉ�]����悤���[�^�̉�]������ݒ�
 	driveC(DR_ROT_R90, 1);									//�ᑬ�Ŏw��p���X����]�B��]��ɒ�~����
	Wait;													//���S�ɒ�~����܂őҋ@
	set_dir(FORWARD);										//�O�i����悤�Ƀ��[�^�̉�]������ݒ�
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//turn_L90
//	����90�x��]����
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L90()
{
	MF.FLAG.CTRL = 0;										//����𖳌��ɂ���
	set_dir(TURN_L);										//���ɉ�]����悤���[�^�̉�]������ݒ�
	driveC(DR_ROT_L90, 1);									//�葬�Ŏw��p���X����]�B��]��ɒ�~����
	Wait;													//���S�ɒ�~����܂őҋ@
	set_dir(FORWARD);										//�O�i����悤�Ƀ��[�^�̉�]������ݒ�
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//turn_180
//	180�x��]����
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_180()
{
	MF.FLAG.CTRL = 0;										//����𖳌��ɂ���
	set_dir(TURN_R);										//���ɉ�]����悤���[�^�̉�]������ݒ�
	driveC(DR_ROT_180, 1);									//�葬�Ŏw��p���X����]�B��]��ɒ�~����
	Wait;													//���S�ɒ�~����܂őҋ@
	set_dir(FORWARD);										//�O�i����悤�Ƀ��[�^�̉�]������ݒ�
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//set_position
//	�@�̂̐K��ǂɓ��Ăďꏊ����撆���ɍ��킹��
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_position()
{
	MF.FLAG.CTRL = 0;										//����𖳌��ɂ���
	set_dir(BACK);											//��ނ���悤���[�^�̉�]������ݒ�
	driveC(DR_CENT_BACK, 1);								//�K�𓖂Ă���x�Ɍ�ށB��]��ɒ�~����
	set_dir(FORWARD);										//�O�i����悤�Ƀ��[�^�̉�]������ݒ�
	driveC(DR_CENT_SET, 1);									//�葬�Ŏw��p���X����]�B��]��ɒ�~����
  	Wait;												//���S�ɒ�~����܂őҋ@
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveA
//	�w��p���X���������s����
// ����1�Fdist�E�E�E���s����p���X
// ����2�Frs�E�E�E���s���~���邩�@1:����@����ȊO:���Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveA(uint16_t dist, unsigned char rs) {
	//====���s====
	//----���s�J�n----
	MF.FLAGS = 0x10 | (MF.FLAGS & 0x0F);					//�����E�葬�E�X�g�b�v�t���O��0�ɁA�����t���O��1�ɂ���
	drive_start();											//���s�J�n

	//----���s----
	while((pulse_l < dist) && (pulse_r < dist));			//���E�̃��[�^���w��p���X�ȏ�i�ނ܂őҋ@

	//----��~��������Β�~----
	drive_stop(rs);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//driveD
//	�w��p���X���������s���Ē�~����
// ����1�Fdist�E�E�E���s����p���X
// ����2�Frs�E�E�E���s���~���邩�@1:����@����ȊO:���Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveD(uint16_t dist, unsigned char rs) {
	//====�ϐ��錾====
	signed int c_pulse;							//�������s�̃p���X��

	//----�K�v�p���X���̎Z�o----
	c_pulse = dist + minindex - t_cnt_l;					//�������s�̃p���X=�������̃p���X-����������̂ɕK�v�ȃp���X
															//=�������̃p���X-(���݂̑��x�̃p���X-�Œ�̃p���X��)
	if(c_pulse < 0) c_pulse = 0;							//���ɂȂ����Ƃ���0�ɂ���

	//====���s====
	//----���s�J�n----
	MF.FLAGS = 0x00 | (MF.FLAGS & 0x0F);					//�����E�����E�葬�E�X�g�b�v�t���O��0�ɂ���
	drive_start();											//����J�n

	//----���s----
	while((pulse_l < c_pulse) && (pulse_r < c_pulse));		//���E�̃��[�^���葬���̃p���X�ȏ�i�ނ܂őҋ@
	MF.FLAG.DECL = 1;										//�����t���O��1��
	while((pulse_l < dist) && (pulse_r < dist));			//���E�̃��[�^���������̃p���X�ȏ�i�ނ܂őҋ@

	//----��~�[�u----
	drive_stop(rs);											//���s�I���A��~��������Β�~
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//driveAD
//	�w��p���X������(or����)�E�������s���Ē�~����
// ����1�Fdist�E�E�E���s����p���X
// ����2�Frs�E�E�E���s���~���邩�@1:����@����ȊO:���Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveAD(uint16_t dist, unsigned char rs)
{
	//====�ϐ��錾====
	signed int ac_pulse;									//���E�������s�̃p���X��

	//----�������s�p���X���̎Z�o----
	ac_pulse = dist + minindex - t_cnt_l;					//�������s�̃p���X=�������̃p���X-����������̂ɕK�v�ȃp���X
															//=�������̃p���X-(���݂̑��x�̃p���X-�Œ�̃p���X��)
	//====���s====
	//----���s�J�n----
	MF.FLAGS = 0x10 | (MF.FLAGS & 0x0F);					//�����E�葬�E�X�g�b�v�t���O��0�ɁA�����t���O��1�ɂ���
	drive_start();											//���s�J�n

	//----���s----
	while((pulse_l < ac_pulse) && (pulse_r < ac_pulse));	//���E�̃��[�^���葬���̃p���X�ȏ�i�ނ܂őҋ@
	ac_pulse = dist + minindex - t_cnt_l;					//�p���X�̍Čv�Z
	MF.FLAG.DECL = 1;										//�����t���O��1��
	while((pulse_l < dist) && (pulse_r < dist));			//���E�̃��[�^���������̃p���X�ȏ�i�ނ܂őҋ@

	//----��~�[�u----
	drive_stop(rs);											//���s�I���A��~��������Β�~

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//driveU
//	�w��p���X���������s���Ē�~����
// ����1�Fdist�E�E�E���s����p���X
// ����2�Frs�E�E�E���s���~���邩�@1:����@����ȊO:���Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveU(uint16_t dist, unsigned char rs)
{

	//====�������s�J�n====
	MF.FLAGS = 0x00 | (MF.FLAGS & 0x0F);					//�����E�����E�葬�E�X�g�b�v�t���O��0�ɂ���
	drive_start();											//���s�J�n

	//====���s====
	while((pulse_l < dist) && (pulse_r < dist));			//���E�̃��[�^���������̃p���X�ȏ�i�ނ܂őҋ@

	//====���s�I��====
	//----��~��������Β�~----
	drive_stop(rs);											//���s�I���A��~��������Β�~
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//driveC
//	�w��p���X���葬���s���Ē�~����
// ����1�Fdist�E�E�E���s����p���X
// ����2�Frs�E�E�E���s���~���邩�@1:����@����ȊO:���Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC(uint16_t dist, unsigned char rs)
{

	//====��]�J�n====
	MF.FLAGS = 0x40 | (MF.FLAGS & 0x0F);					//�����E�����E�X�g�b�v�t���O��0�ɂ��A�葬�t���O��1�ɂ���
	drive_start();											//���s�J�n

	//====��]====
	while((pulse_l < dist) && (pulse_r < dist));			//���E�̃��[�^���葬���̃p���X�ȏ�i�ނ܂őҋ@

	drive_stop(rs);											//���s�I���A��~��������Β�~

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_start
//	���s���J�n����
// �����F�Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_start(void){
	time = 0;									//���s�����p���X���̏�����	
	R_PG_Timer_StartCount_MTU_U0_C3();  //�����[�^
	R_PG_Timer_StartCount_MTU_U0_C4();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_stop
//	���s�I�����A��~��������Β�~����
// ����1�Frs�E�E�E���s���~���邩�@1:����@����ȊO:���Ȃ�
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_stop(unsigned char rs){
	R_PG_Timer_HaltCount_MTU_U0_C3();
	R_PG_Timer_HaltCount_MTU_U0_C4();

	pin_write(P54,0);
	pin_write(P55,0);
	pin_write(PA4,0);
	pin_write(PA6,0);
	//====��~����====
	if(rs) MF.FLAG.STOP = 1;								//��~����̂ł���΃X�g�b�v�t���O��1��
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//set_dir
//	�i�s������ݒ肷��
// ����1�Fd_dir�E�E�E�ǂ̕����ɐi�s���邩  0���ڂō��A1���ڂŉE�̕����ݒ�
// �߂�l�F�Ȃ�
//		DC�p�ɉ�������
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_dir(unsigned char d_dir)
{
	//====�����[�^====
	switch(d_dir & 0x0f){									//0~3�r�b�g�ڂ����o��
		//----����]----
		case 0x00:									
			MF.FLAG.L_FRONT = 1;						//����O�i�����ɐݒ�
			MF.FLAG.L_BEHIND = 0;
			break;
		//----�t��]----
		case 0x01:								 
			MF.FLAG.L_FRONT = 0;				//������i�����ɐݒ�
			MF.FLAG.L_BEHIND= 1;
			break;
	}
	//====�E���[�^====
	switch(d_dir & 0xf0){									//4~7�r�b�g�ڂ����o��
		case 0x00:											//0x00�̏ꍇ
			MF.FLAG.R_FRONT = 1;					//�E��O�i�����ɐݒ�
			MF.FLAG.R_BEHIND = 0;
			break;
		case 0x10:											//0x10�̏ꍇ
			MF.FLAG.R_FRONT = 0;					//�E����i�����ɐݒ�
			MF.FLAG.R_BEHIND = 1;
			break;
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//test_drive
//	���s�֌W�̃e�X�g������
// ����1�Fmode�E�E�E���[�h�ԍ����i�[����ϐ��̃A�h���X
// �߂�l�F�Ȃ�
//+++++++++++++++++++++++++++++++++++++++++++++++
void test_drive(char *mode){

	while(1){											
		uart_printf("test_drive:\n");						//UART�ő��M
		select_mode(mode);									//���[�h�I����������
		ms_wait(50);
		switch(*mode){										//���[�h�ԍ��ɂ�蕪��
			//----�ʒu�̒���----
			case 0:
				set_position();
				break;

			//----����葬���s----
			case 1:
				sensor_start();
				set_dir(FORWARD);
				drive_start();
				
				while(1){
				//uart_printf(" pulse_r: %4d  pulse_l:%4d  totalR_mm:%4lf totalL_mm:%4lf \r\n", pulse_r, pulse_l, totalR_mm, totalL_mm);
				//uart_printf(" time : %4d time4 : %4d totalR_mm:%4lf totalL_mm:%4lf \r\n", time, time % 4, totalR_mm, totalL_mm);
		
				uart_printf("DR: %4lf  DL:%4lf totalR_mm:%4lf totalL_mm:%4lf \r\n",vel_R,vel_L, totalR_mm, totalL_mm);
				ms_wait(1000);
				}
/*				set_dir(FORWARD);
				driveC(DR_SEC_HALF*2, 1);
				set_dir(FORWARD);
				driveC(DR_SEC_HALF*2, 1);
				set_dir(FORWARD);
				driveC(DR_SEC_HALF*2, 1);
				set_dir(FORWARD);
				driveC(DR_SEC_HALF*2, 1);
				set_dir(FORWARD);
				driveC(DR_SEC_HALF*2, 1);
				set_dir(FORWARD);
				driveC(DR_SEC_HALF*2, 1);
*/				drive_stop(1);
				R_PG_Timer_HaltCount_MTU_U0_C1();
				R_PG_Timer_HaltCount_MTU_U0_C2();
				R_PG_Timer_HaltCount_MTU_U0_C3();
				R_PG_Timer_HaltCount_MTU_U0_C4();
				R_PG_Timer_HaltCount_CMT_U0_C1();

				uart_printf("BREAK \r\n");
				break;

			//----�E90��]----
			case 2:
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				turn_R90();
				break;

			//----��90��]----
			case 3:
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				turn_L90();
				break;

			//----180��]----
			case 4:
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();
				turn_180();

				break;

			//----����������8���----
			case 7:
				minindex = MINSPEED_H;
				maxindex = MAXSPEED_H;
				stay(DEFWAIT);
				MF.FLAG.CTRL = 1;
				driveAD((DR_SEC_HALF*2) * 6, 1);
				break;

			//----�T���p������1���----
			case 8:
				minindex = MINSPEED_S;
				maxindex = MAXSPEED_S;
				stay(DEFWAIT);
				a_section();
				break;

			default:
				
				while(1){
					uart_printf("GoodBy, Drive\r\n");
				}
				return;
		}
		
	}
	uart_printf("drive_Finish: %d\n",*mode);
	
}

