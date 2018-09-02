/*
==============================================================
 Name        : interrupt.c
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : ���荞�݊֐������ł��D(����)lpc11xx.h�Ńv���g�^�C�v�錾����Ă��܂��D

 �X�V����
 2015/12/4		�R��	�R�����g�ǉ��Atp�̏����𕪂���₷���ύX
 2016/2/24		�R��	2016�N�x�p�Ƀs���ݒ��ύX
 2017/10/14		���� ���������̏������򃍃W�b�N�����C��(�J�E���^���I�[�o�[�t���[����)
==============================================================
 */
#include "global.h"

//+++++++++++++++++++++++++++++++++++++++++++++++
//Mtu3IcCmDIntFunction
//���荞�݊֐��A�����[�^�̊Ǘ����s��
// �����F����
// �߂�l�F����
//+++++++++++++++++++++++++++++++++++++++++++++++
void Mtu3IcCmDIntFunc(void){		//====�����[�^���荞��====

	uint16_t paraL0;
	uint16_t paraL1 = 65535;

	pin_write(P54,0);
	pin_write(P55,0);

	duty_l = Kvolt * accel_l / VOLT_BAT + kpvL + kpdL;
	if(duty_l > 1){
		duty_oL = duty_l;
		duty_l = 0.8;
	}

		//====����������====
		//----��������----
/*		if(MF.FLAG.DECL){															//�����t���O��1�ł���ꍇ
			
		}
		//----��������----
		else if(MF.FLAG.ACCL){														//�����t���O��1�ł���ꍇ
			
		}

/*		//----��~���̏���----
		if(MF.FLAG.STOP){															//��~�t���O��1�ł���ꍇ
			paraL1 = 0;
				//----�f�t�H���g�̑��x----
		}else if(MF.FLAG.DEF){														//�葬�t���O��1�ł���ꍇ
			paraL0 = DEFGRC - sen_dl;											//�葬�ɑ΂��Đ���������l�ɐݒ�
			//----���̑��̂Ƃ�----
		}else{
			paraL0 = table[t_cnt_l] - sen_dl;									//�����[�^�C���^�[�o���ݒ�
		}
*/
		paraL0 = (1 - duty_l) * paraL1;									//�o�̓p���X������ݒ�
		test_valL[time]  = duty_l;
		R_PG_Timer_SetTGR_C_MTU_U0_C3(paraL0);
		R_PG_Timer_SetTGR_D_MTU_U0_C3(paraL1);

}

void Mtu3IcCmCIntFunc(void){
	
	if(MF.FLAG.L_FRONT){
		pin_write(P54,1);
		pin_write(P55,0);
	}else if(MF.FLAG.L_BEHIND){
		pin_write(P54,0);
		pin_write(P55,1);
	}
	
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//Mtu4IcCmDIntFunc
//	16�r�b�g�^�C�}�[1�̊��荞�݊֐��A�E���[�^�̊Ǘ����s��
// �����F����
// �߂�l�F����
//+++++++++++++++++++++++++++++++++++++++++++++++
void Mtu4IcCmDIntFunc(void){			//====�E���[�^���荞��====
	uint16_t paraR0 = 40000;
	uint16_t paraR1 = 65535;

	pin_write(PA4,0);
	pin_write(PA6,0);

	duty_r = Kvolt * accel_r / VOLT_BAT + kpvR + kpdR;
	if(duty_r > 1){
		duty_oR = duty_r;
		duty_r = 0.8;
	}
	
	//====����������====
	//----��������----
/*	if(MF.FLAG.DECL){															//�����t���O��1�ł���ꍇ
		if(t_cnt_r > minindex) t_cnt_r--; //�Œᑬ�x�ȏ�̏ꍇ�e�[�u���J�E���^�����炷���ƂŌ���
	}
	//----��������----
	else if(MF.FLAG.ACCL){														//�����t���O��1�ł���ꍇ
		if(t_cnt_r < maxindex) t_cnt_r++; //�Œᑬ�x�ȉ��̏ꍇ�e�[�u���J�E���^�𑝂₷���Ƃŉ���
	}

	//----��~���̏���----
	if(MF.FLAG.STOP){															//��~�t���O��1�ł���ꍇ
		paraR1 = 0;
	
//----�f�t�H���g�̑��x----
	}else if(MF.FLAG.DEF){														//�葬�t���O��1�ł���ꍇ
		paraR0 = DEFGRC - sen_dr;											//�葬�ɑ΂��Đ���������l�ɐݒ�
		//----���̑��̂Ƃ�----
	}else{
		paraR0 = table[t_cnt_r] - sen_dr;									//�E���[�^�C���^�[�o���ݒ�
	}
*/
	paraR0 = (1 - duty_r) * paraR1;									//�o�̓p���X������ݒ�
	
	R_PG_Timer_SetTGR_C_MTU_U0_C4(paraR0);
	R_PG_Timer_SetTGR_D_MTU_U0_C4(paraR1);

}

void Mtu4IcCmCIntFunc(void){
	if(MF.FLAG.R_FRONT){
		pin_write(PA4,0);
		pin_write(PA6,1);
	}else if(MF.FLAG.R_BEHIND){
		pin_write(PA4,1);
		pin_write(PA6,0);
	}
	
	S12AD.ADANS0.WORD = 0x40;
	R_PG_ADC_12_StartConversionSW_S12AD0();					
	R_PG_ADC_12_GetResult_S12AD0(ad_res);
	volt_bat = ad_res[6];
}
/*------------------------------------------------------------
		32bit�^�C�}���荞��
------------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//Cmt0IntFunc
//32�r�b�g�^�C�}�[0�̊��荞�݊֐��A�Z���T�l�̎擾�Ɛ�����s��
// �����F����
// �߂�l�F����
//+++++++++++++++++++++++++++++++++++++++++++++++
void Cmt1IntFunc(void){

	time++;			//���荞�݊J�n��̎��Ԃ�ms�P�ʂő���
	
	switch(tp){
	//----�Z���T����----
	case 0:	
		//���Z���T�l�̎擾
		pin_write(PE1,1);
		stay(100);
		S12AD.ADANS0.WORD = 0x02;
		R_PG_ADC_12_StartConversionSW_S12AD0();					
		R_PG_ADC_12_GetResult_S12AD0(ad_res);
		ad_l = ad_res[1];
		
		//�E�Z���T�l�̎擾
		pin_write(PE3,1);
		stay(100);
		S12AD.ADANS0.WORD = 0x08;
		R_PG_ADC_12_StartConversionSW_S12AD0();
		R_PG_ADC_12_GetResult_S12AD0(ad_res);
		ad_r =  ad_res[3];
				
		break;
	case 1:
		pin_write(PE1,0);
		pin_write(PE3,0);
				
		//���ʃZ���T�l�̎擾
		
		pin_write(PE0,1);								//������LED��ON
		pin_write(PE2,1);								//������LED��ON
		pin_write(PE4,1);
	
		stay(100);
		S12AD.ADANS0.WORD = 0x01;
		R_PG_ADC_12_StartConversionSW_S12AD0();
		R_PG_ADC_12_GetResult_S12AD0(ad_res);
		ad_fl = ad_res[0];
		
		S12AD.ADANS0.WORD = 0x04;
		R_PG_ADC_12_StartConversionSW_S12AD0();
		R_PG_ADC_12_GetResult_S12AD0(ad_res);
		ad_ff = ad_res[2];

		S12AD.ADANS0.WORD = 0x10;
		R_PG_ADC_12_StartConversionSW_S12AD0();
		R_PG_ADC_12_GetResult_S12AD0(ad_res);
		ad_fr = ad_res[4];
		
		break;
	case 2:
		pin_write(PE0,0);								
		pin_write(PE2,0);								
		pin_write(PE4,0);
		
		
//		totalR_pre_mm = totalR_mm;
//		totalL_pre_mm = totalL_mm;
		
		R_PG_Timer_GetCounterValue_MTU_U0_C1(&pulse_l);
		R_PG_Timer_GetCounterValue_MTU_U0_C2(&pulse_r);
		
		if(pulse_sum_r > 0){				//�A���_�[�t���[��
			dif_pulse_r = - (65535 * pulse_sum_r  + pulse_pre_r - pulse_r) ;
			pulse_sum_r = 0;
		}else if(pulse_sum_r < 0){			//�I�[�o�[�t���[��
			dif_pulse_r = 65535 * pulse_sum_r + pulse_pre_r - pulse_r;
			pulse_sum_r = 0;
		}else{
			dif_pulse_r = pulse_r - pulse_pre_r;	//�ʏ�̃p���X�ω�
			pulse_sum_r = 0;
		}
		
		if(pulse_sum_l > 0 ){				//�A���_�[�t���[��
			dif_pulse_l = - (65535 * pulse_sum_l + pulse_pre_l - pulse_l);
			pulse_sum_l = 0;
		}else if(pulse_sum_l < 0){			//�I�[�o�[�t���[��
			dif_pulse_l = 65535 * pulse_sum_l + pulse_pre_l - pulse_l;
			pulse_sum_l = 0;
		}else {
			dif_pulse_l = pulse_l - pulse_pre_l;	//�����傤�̃p���X�ω�
			pulse_sum_l = 0;
		}

		xR = -DIA_WHEEL_mm * (DIA_PINI_mm / DIA_SQUR_mm) * 2 * Pi * (dif_pulse_r % 4096) / 4096;
		xL = -DIA_WHEEL_mm * (DIA_PINI_mm / DIA_SQUR_mm) * 2 * Pi * (dif_pulse_l % 4096) / 4096;

		totalR_mm += xR;
		totalL_mm += xL;
		
		pulse_pre_r = pulse_r;
		pulse_pre_l = pulse_l;
		
		break;
		//----���䏈��----
	case 3:			
		vel_R = xR  / 4;	//4ms�Ԃ̃G���R�[�_�l�̕ω�[mm]�@�ˁ@���x[m/s]
		vel_L = xL  / 4;
		
		test_valR[time/4] = vel_R;
		test_valL[time/4] = vel_L;
/*		test_valR1[time/4] = pulse_r;
		test_valR2[time/4] = dif_pulse_r;
		test_valL[time/4] = vel_L;
		test_valL1[time/4] = pulse_l;
		test_valL2[time/4] = dif_pulse_l;
*/		
		//���xPID����
		dif_vel_R = targ_vel_R - vel_R;
		kpvR = KPR * dif_vel_R;
		kpdR = KDR * (dif_vel_R - dif_pre_vel_R);
		
		dif_pre_vel_R = dif_vel_R;
		
		dif_vel_L = targ_vel_L - vel_L;
		kpvL = KPL * dif_vel_L;
		kpdL = KDL * (dif_vel_L - dif_pre_vel_L);
		
		dif_pre_vel_L = dif_vel_L;

		
		
		
		//����t���O������ΐ���
		if(MF.FLAG.CTRL){
			//��l����̍�������
			dif_l = (int16_t)(ad_l - base_l);
			dif_r = (int16_t)(ad_r - base_r);

			//����̔��f
			if((SREF_MIN_L < dif_l) && (dif_l < SREF_MAX_L))
				sen_dl = CONT * dif_l;	//��ᐧ��l������
			else
				sen_dl = 0;	//�����O��Ă���ΐ���͂����Ȃ�
			if((SREF_MIN_R < dif_r) && (dif_r < SREF_MAX_R))
				sen_dr = CONT * dif_r;
			else
				sen_dr = 0;

		}else{
			//����t���O���Ȃ���ΐ���l0
			sen_dl = sen_dr = 0;
		}
		break;
	}

	//====�^�X�N�|�C���^��i�߂�====
	tp = (tp+1) % 4;

}

void Mtu2UnIntFunc() {
	pulse_sum_r += 1;
}
void Mtu2OvIntFunc() {
	pulse_sum_r -= 1;
}

void Mtu1UnIntFunc() {
	pulse_sum_l += 1;
}
void Mtu1OvIntFunc() {
	pulse_sum_l -= 2;
}

void Cmt0IntFunc(){
	ms_time++;
}

/*void S12ad0IntFunc(void){
	R_PG_ADC_12_GetResult_S12AD0(ad_res);
}*/
