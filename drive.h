﻿/*
==============================================================
 Name        : drive.h
 Copyright   : Copyright (C) 早稲田大学マイクロマウスクラブ
 Description : 走行関連の関数たちです．

  更新履歴
 2015/12/11		山上	コメントとSLA7078用のマクロを追加
 2016/2/24		山上	2016年度用にピン設定を変更
==============================================================
*/

/*ヘッダファイルの読み込み*/
#include "port.h"

#ifndef DRIVE_H_

	#define DRIVE_H_									//読み込んだことを表す

/*============================================================
		各種定数･変数宣言
============================================================*/
	//----ドライバIC関連----
	#define MDR_CLK_L	P0_8		//クロック端子左
	#define MDR_CLK_R	P1_9		//クロック端子右
// 上OK

	#define MDR_CW_L	P2_2		//IC分配回路のCW/CCW左
	#define MDR_CW_R	P3_5		//IC分配回路のCW/CCW右
	#define MDR_REF		P2_9		//リファレンス

	#define MDR_M1		P0_6		//励磁切り替えロジック1
	#define MDR_M2		P3_4		//励磁切り替えロジック2
	#define MDR_M3		P2_1		//励磁切り替えロジック3

	/* 励磁切り替えロジック
	・SLA7078MPRT
	  M1 | M2 | M3 |励磁モード
	   L |  L |  L | 2相励磁(Mode8固定)
	   L |  H |  L | 2相励磁(ModeF固定)
	   H |  L |  L | 1-2相励磁(2分割)
	   H |  H |  L | 1-2相励磁(ModeF固定)
	   L |  L |  H |  W1-2相（ 4分割）
	   L |  H |  H | 4W1-2相（16分割）
	   H |  L |  H | 2W1-2相（ 8分割）
	   H |  H |  H | スリープモード2

	・SLA7073MPRT
	  M1 | M2 | 励磁モード
	   L |  L | 4W1-2相（16分割）
	   L |  H | 2W1-2相（ 8分割）
	   H |  L |  W1-2相（ 4分割）
	   H |  H |   1-2相（ 2分割）
	*/



	//----動作方向関連----
	#define MT_FWD_L	1			//CW/CCWで前に進む出力(左)
	#define MT_BACK_L	0			//CW/CCWで後ろに進む出力(左)
	#define MT_FWD_R	1			//CW/CCWで前に進む出力(右)
	#define MT_BACK_R	0		//CW/CCWで後ろに進む出力(右)

	#define FORWARD	0x00			//前進向き
	#define BACK	0x11			//後退
	#define TURN_L	0x01			//回転向き(左)
	#define TURN_R	0x10			//回転向き(右)

	//----方向転換用定数----
	#define DIR_TURN_R90	0x01	//右90度回転
	#define DIR_TURN_L90	0xff	//左90度回転
	#define DIR_TURN_180	0x02	//180度回転


	//====変数====
#ifdef EXTERN										//対応ファイルでEXTERNが定義されている場合
	/*グローバル変数の定義*/
	const uint16_t table[] = {
		#include "table.h"
	};								//table.hに貼り付けた値を保持する配列
	volatile float vel_R, vel_L, xR, xL;				//現在速度，距離変化
	volatile float targ_vel_R,targ_vel_L;				//目標速度
	volatile float dif_vel_R,dif_vel_L;				//速度偏差
	volatile float dif_pre_vel_R, dif_pre_vel_L;			//速度偏差
	volatile float kvpR,kvdR,kviR,kvpL,kvdL,kviL;			//PID制御量
	
	volatile uint16_t t_cnt_l, t_cnt_r,time, ms_time;		//テーブルカウンタ
	volatile uint16_t minindex, maxindex;				//最低速度・最大速度
	volatile uint16_t interval_l, interval_r;			//左右インターバル
	
	volatile float duty_r,duty_l,duty_oR,duty_oL;			//Duty比計算と計算オーバー用のバッファ
	volatile float Kvolt,Kxr;					//加速度えるための電流定数，距離変換のための定数
	volatile float accel_r,accel_l;					//目標加速度？
	volatile float test_valR[2000],test_valL[2000],test_valR1[2000],test_valL1[2000];			//速度ログなどの保存用配列
	volatile uint16_t test_valR2[1000],test_valL2[1000]; 	//エンコーダ獲得値などの保存用配列
	
	
	volatile uint16_t pulse_l, pulse_r, pulse_pre_r, pulse_pre_l;					//左右パルスカウント
	volatile int32_t dif_pulse_r, dif_pulse_l,pulse_sum_l, pulse_sum_r;				//エンコーダ変化値，オバフ用フラグ
	volatile float totalR_mm, totalL_mm,totalR_pre_mm,totalL_pre_mm;						//走行距離[mm]など
	volatile float offsetR_mm, offsetL_mm;
	int16_t	sen_dl, sen_dr;										//壁センサによる比例制御量
	
	
#else									//対応ファイルでEXTERNが定義されていない場合
	/*グローバル変数の宣言*/
	extern const uint16_t table[];				
	extern volatile float dif_pre_vel_R, dif_pre_vel_L;
	extern volatile float vel_R, vel_L, xR, xL;		//両輪速度
	extern volatile float targ_vel_R,targ_vel_L;	        //目標速度
	extern volatile float dif_vel_R, dif_vel_L;
	extern volatile float kvpR,kvdR,kviR,kvpL,kvdL,kviL;
	
	extern volatile uint16_t t_cnt_l, t_cnt_r,time, ms_time;		
	extern volatile uint16_t minindex, maxindex;		//最低速度・最大速度
	extern volatile uint16_t interval_l, interval_r;	//左右インターバル
	
	extern volatile float duty_r,duty_l,duty_oR,duty_oL;
	extern volatile float Kvolt,Kxr;			//加速度えるための電流定数，距離変換のための定数
	extern volatile float accel_r,accel_l;
	extern volatile float test_valR[2000],test_valL[2000],test_valR1[2000],test_valL1[2000];
	extern volatile uint16_t test_valR2[1000],test_valL2[1000]; 
	
	extern volatile uint16_t pulse_l, pulse_r,pulse_pre_r, pulse_pre_l;		//左右エンコーダパルス関係
	extern volatile int32_t dif_pulse_r, dif_pulse_l,pulse_sum_l, pulse_sum_r;
	extern volatile float totalR_mm, totalL_mm, totalR_pre_mm, totalL_pre_mm;
	extern volatile float offsetR_mm, offsetL_mm;
	extern int16_t	sen_dl, sen_dr;					
	
	
#endif

/*============================================================
		関数プロトタイプ宣言
============================================================*/
	//====走行系====
	//----基幹関数----
	void driveA(uint16_t, unsigned char);	//加速走行
	void driveD(uint16_t, unsigned char);	//減速走行
	void driveAD(uint16_t, unsigned char);	//加減速走行
	void driveU(uint16_t, unsigned char);	//等速走行(前の速度を維持)
	void driveC(uint16_t, unsigned char);	//定速走行(あらかじめ決まった速度)
	void set_dir(unsigned char);		//進む方向の設定

	void drive_start();
	void drive_stop(unsigned char);

	//----簡易上位関数----
	void half_sectionA();		//加速半区画
	void half_sectionD();		//減速半区画
	void a_section();		//加減速一区画
	void a_sectionU();		//等速一区画
	void s_section();		//連続区画直線走行
	void turn_R90();		//右90回転
	void turn_L90();		//左90回転
	void turn_180();		//180度回転
	void set_position();		//位置合わせ

	//----走行試験----
	void test_drive(char *mode);	//走行試験
	
	void start_ready();
	

#endif /* DRIVE_H_ */
