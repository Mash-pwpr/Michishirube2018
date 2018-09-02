/*
==============================================================
 Name        : interrupt.c
 Copyright   : Copyright (C) 早稲田大学マイクロマウスクラブ
 Description : 割り込み関数たちです．(多分)lpc11xx.hでプロトタイプ宣言されています．

 更新履歴
 2015/12/4		山上	コメント追加、tpの処理を分かりやすく変更
 2016/2/24		山上	2016年度用にピン設定を変更
 2017/10/14		嶋崎 加減速時の条件分岐ロジック誤りを修正(カウンタがオーバーフローする)
==============================================================
 */
#include "global.h"

//+++++++++++++++++++++++++++++++++++++++++++++++
//Mtu3IcCmDIntFunction
//割り込み関数、左モータの管理を行う
// 引数：無し
// 戻り値：無し
//+++++++++++++++++++++++++++++++++++++++++++++++
void Mtu3IcCmDIntFunc(void){		//====左モータ割り込み====

	uint16_t paraL0;
	uint16_t paraL1 = 65535;

	pin_write(P54,0);
	pin_write(P55,0);

	duty_l = Kvolt * accel_l / VOLT_BAT + kpvL + kpdL;
	if(duty_l > 1){
		duty_oL = duty_l;
		duty_l = 0.8;
	}

		//====加減速処理====
		//----減速処理----
/*		if(MF.FLAG.DECL){															//減速フラグが1である場合
			
		}
		//----加速処理----
		else if(MF.FLAG.ACCL){														//加速フラグが1である場合
			
		}

/*		//----停止中の処理----
		if(MF.FLAG.STOP){															//停止フラグが1である場合
			paraL1 = 0;
				//----デフォルトの速度----
		}else if(MF.FLAG.DEF){														//定速フラグが1である場合
			paraL0 = DEFGRC - sen_dl;											//定速に対して制御をした値に設定
			//----その他のとき----
		}else{
			paraL0 = table[t_cnt_l] - sen_dl;									//左モータインターバル設定
		}
*/
		paraL0 = (1 - duty_l) * paraL1;									//出力パルス部分を設定
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
//	16ビットタイマー1の割り込み関数、右モータの管理を行う
// 引数：無し
// 戻り値：無し
//+++++++++++++++++++++++++++++++++++++++++++++++
void Mtu4IcCmDIntFunc(void){			//====右モータ割り込み====
	uint16_t paraR0 = 40000;
	uint16_t paraR1 = 65535;

	pin_write(PA4,0);
	pin_write(PA6,0);

	duty_r = Kvolt * accel_r / VOLT_BAT + kpvR + kpdR;
	if(duty_r > 1){
		duty_oR = duty_r;
		duty_r = 0.8;
	}
	
	//====加減速処理====
	//----減速処理----
/*	if(MF.FLAG.DECL){															//減速フラグが1である場合
		if(t_cnt_r > minindex) t_cnt_r--; //最低速度以上の場合テーブルカウンタを減らすことで減速
	}
	//----加速処理----
	else if(MF.FLAG.ACCL){														//加速フラグが1である場合
		if(t_cnt_r < maxindex) t_cnt_r++; //最低速度以下の場合テーブルカウンタを増やすことで加速
	}

	//----停止中の処理----
	if(MF.FLAG.STOP){															//停止フラグが1である場合
		paraR1 = 0;
	
//----デフォルトの速度----
	}else if(MF.FLAG.DEF){														//定速フラグが1である場合
		paraR0 = DEFGRC - sen_dr;											//定速に対して制御をした値に設定
		//----その他のとき----
	}else{
		paraR0 = table[t_cnt_r] - sen_dr;									//右モータインターバル設定
	}
*/
	paraR0 = (1 - duty_r) * paraR1;									//出力パルス部分を設定
	
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
		32bitタイマ割り込み
------------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//Cmt0IntFunc
//32ビットタイマー0の割り込み関数、センサ値の取得と制御を行う
// 引数：無し
// 戻り値：無し
//+++++++++++++++++++++++++++++++++++++++++++++++
void Cmt1IntFunc(void){

	time++;			//割り込み開始後の時間をms単位で測定
	
	switch(tp){
	//----センサ処理----
	case 0:	
		//左センサ値の取得
		pin_write(PE1,1);
		stay(100);
		S12AD.ADANS0.WORD = 0x02;
		R_PG_ADC_12_StartConversionSW_S12AD0();					
		R_PG_ADC_12_GetResult_S12AD0(ad_res);
		ad_l = ad_res[1];
		
		//右センサ値の取得
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
				
		//正面センサ値の取得
		
		pin_write(PE0,1);								//発光部LEDをON
		pin_write(PE2,1);								//発光部LEDをON
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
		
		if(pulse_sum_r > 0){				//アンダーフロー時
			dif_pulse_r = - (65535 * pulse_sum_r  + pulse_pre_r - pulse_r) ;
			pulse_sum_r = 0;
		}else if(pulse_sum_r < 0){			//オーバーフロー時
			dif_pulse_r = 65535 * pulse_sum_r + pulse_pre_r - pulse_r;
			pulse_sum_r = 0;
		}else{
			dif_pulse_r = pulse_r - pulse_pre_r;	//通常のパルス変化
			pulse_sum_r = 0;
		}
		
		if(pulse_sum_l > 0 ){				//アンダーフロー時
			dif_pulse_l = - (65535 * pulse_sum_l + pulse_pre_l - pulse_l);
			pulse_sum_l = 0;
		}else if(pulse_sum_l < 0){			//オーバーフロー時
			dif_pulse_l = 65535 * pulse_sum_l + pulse_pre_l - pulse_l;
			pulse_sum_l = 0;
		}else {
			dif_pulse_l = pulse_l - pulse_pre_l;	//つうじょうのパルス変化
			pulse_sum_l = 0;
		}

		xR = -DIA_WHEEL_mm * (DIA_PINI_mm / DIA_SQUR_mm) * 2 * Pi * (dif_pulse_r % 4096) / 4096;
		xL = -DIA_WHEEL_mm * (DIA_PINI_mm / DIA_SQUR_mm) * 2 * Pi * (dif_pulse_l % 4096) / 4096;

		totalR_mm += xR;
		totalL_mm += xL;
		
		pulse_pre_r = pulse_r;
		pulse_pre_l = pulse_l;
		
		break;
		//----制御処理----
	case 3:			
		vel_R = xR  / 4;	//4ms間のエンコーダ値の変化[mm]　⇒　速度[m/s]
		vel_L = xL  / 4;
		
		test_valR[time/4] = vel_R;
		test_valL[time/4] = vel_L;
/*		test_valR1[time/4] = pulse_r;
		test_valR2[time/4] = dif_pulse_r;
		test_valL[time/4] = vel_L;
		test_valL1[time/4] = pulse_l;
		test_valL2[time/4] = dif_pulse_l;
*/		
		//速度PID制御
		dif_vel_R = targ_vel_R - vel_R;
		kpvR = KPR * dif_vel_R;
		kpdR = KDR * (dif_vel_R - dif_pre_vel_R);
		
		dif_pre_vel_R = dif_vel_R;
		
		dif_vel_L = targ_vel_L - vel_L;
		kpvL = KPL * dif_vel_L;
		kpdL = KDL * (dif_vel_L - dif_pre_vel_L);
		
		dif_pre_vel_L = dif_vel_L;

		
		
		
		//制御フラグがあれば制御
		if(MF.FLAG.CTRL){
			//基準値からの差を見る
			dif_l = (int16_t)(ad_l - base_l);
			dif_r = (int16_t)(ad_r - base_r);

			//制御の判断
			if((SREF_MIN_L < dif_l) && (dif_l < SREF_MAX_L))
				sen_dl = CONT * dif_l;	//比例制御値を決定
			else
				sen_dl = 0;	//基準から外れていれば制御はかけない
			if((SREF_MIN_R < dif_r) && (dif_r < SREF_MAX_R))
				sen_dr = CONT * dif_r;
			else
				sen_dr = 0;

		}else{
			//制御フラグがなければ制御値0
			sen_dl = sen_dr = 0;
		}
		break;
	}

	//====タスクポインタを進める====
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
