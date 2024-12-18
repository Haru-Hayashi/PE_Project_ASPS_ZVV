/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/
#include "ASPS_ZVV_PEV.h"
#include "stdbool.h"
#include "stdio.h"
#include "UserDefined.h"
#include "lib/DSPBoard.h"
#include "lib/InductionMotor.h"
#include "lib/Inverter.h"
#include "lib/Table_Reference.h"
#include "lib/Integrator.h"

// ******* プロトタイプ宣言 ******* //
interrupt void IntrFunc1(void);
interrupt void IntrFunc2(void);

#define SAMPLING_FREQ 40000
#define TS_SAMPLE (25e-6)//(25e-6)
#define FS 40000
#define TS (25e-6)

#pragma SET_DATA_SECTION(".DATA_ON_HIGHER_SPEED")

// 実行時間計算
far float Tact[3] = {0};
far float Tact_ctrl;

// オブジェクト
far IM_tMotor m;
far LPF_tLowPassFilter lpf;
far Inv_InverterParam inv;
far Integral_value inte_values;
far Integral_mod inte_mod;
far Switching_Instance SW_Inst;

// データ読み込み（応答値）
far float iu_res = 0.0;
far float iv_res = 0.0;
far float iw_res = 0.0;
far float Vdc = 0.0;
far float t_res = 0.0;
far float t_res_lpf = 0.0;
far float Vdc_command = 282.0;
far float V_limit = 0.0;
far float ENC_data[2] = { 0 };
far int ENC_delta = 0;
// オフセット調節用
far float iu_set = 0.0;
far float iv_set = 0.0;
far float iw_set = 0.0;
far float t_set = 0.0;
far float Vdc_set = 0.0;
far float data_offset_tmp[8];
far float data_offset[8];
far int offset_count = 0;
far float offset_time = 0.0;

far bool Flag_Vdc_command = 0.0;

// ab座標での応答値
far float ia_res = 0.0;
far float ib_res = 0.0;
far float va_res = 0.0;
far float vb_res = 0.0;

// dq座標での応答値
far float id_res = 0.0;
far float iq_res = 0.0;
far float vd_res = 0.0;
far float vq_res = 0.0;

// 速度関連
far float wre = 0.0;
far float wrm = 0,wrm_filter=0;
far float omega_data[2];


// ASPS関係
#pragma DATA_SECTION(VV_sequence_dss25, ".DATA_ON_HIGHER_SPEED")
far uint8_t VV_sequence_dss25[2][129] = {
        {0, 1, 2, 1, 0, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 7, 2, 1, 2, 3, 2, 7, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 0, 3, 2, 3, 0, 3, 4, 3, 4, 3, 4, 3, 4, 3, 4, 3, 4, 3, 4, 3, 4, 7, 4, 3, 4, 7, 4, 5, 4, 7, 4, 5, 4, 5, 4, 5, 4, 5, 4, 5, 4, 5, 4, 5, 0, 5, 0, 5, 6, 5, 0, 5, 6, 5, 6, 5, 6, 5, 6, 5, 6, 5, 6, 7, 6, 5, 6, 7, 6, 7, 6, 1, 6, 1, 6, 1, 6, 1, 6, 7, 6, 1, 6, 1, 6, 1, 6, 1, 0},
        {1, 11, 2, 7, 2, 5, 3, 8, 4, 6, 4, 5, 6, 5, 7, 4, 9, 4, 9, 2, 10, 2, 4, 2, 9, 2, 7, 3, 9, 4, 7, 5, 6, 6, 5, 6, 4, 8, 3, 7, 2, 5, 2, 16, 3, 10, 3, 9, 3, 6, 3, 6, 4, 5, 5, 4, 5, 4, 8, 4, 7, 2, 6, 2, 12, 2, 11, 2, 6, 2, 7, 4, 8, 4, 6, 5, 5, 6, 5, 8, 4, 10, 3, 8, 2, 12, 2, 12, 3, 5, 1, 6, 4, 8, 5, 7, 6, 5, 7, 4, 7, 3, 7, 2, 5, 2, 10, 1, 9, 2, 8, 3, 9, 3, 7, 4, 7, 5, 2, 1, 3, 6, 4, 6, 4, 9, 4, 19, 3}
        };

far uint8_t VV_sequence[2][MAXIMUM_TABLE_SIZE];
far uint8_t numLines = 0;
far uint8_t Slot_Num = 50;
far uint8_t Table_num = 35;
far uint8_t Vector_Times = 0;
far uint8_t Vector_Number = 0;
far int u = 0, v = 0, w = 0;
far int du = 0, dv = 0, dw = 0;
far float Tc = 0.0;
far float Va_last = 0.0;
far float Vb_last = 0.0;
far float Vu_last = 0.0, Vv_last = 0.0, Vw_last = 0.0;
far float Vun_last = 0.0, Vvn_last = 0.0, Vwn_last = 0.0;
far uint8_t out_swp = 0;
far uint8_t write_step = 0; // 書き込みステップを追跡
far uint8_t is_write_complete = 0; // 書き込みが完了したかどうかを示すフラグ
far uint8_t confirm_number[2] = {0};
far uint8_t confirm[2] = {0};
far bool Flag_zero = 0;
far bool Flag_TableRead[2] = {0};
// bool Nominal_Mode = 0;

// 速度制御パラメータ
far float Kps = 0.0, Kis = 0.0;
far float err_inte1 = 0.0;
far float err_inte2 = 0.0;
far float wrm_ref = 0.0;
// PI_def PI_control;
far float limit_value = 0.0;
far float prop_out = 0.0;
far float inte_out = 0.0;
far float omega_err = 0.0;
far float omega_err_lf = 0.0;
far bool Flag_inte_out_zero = 0;
far bool Flag_omegaControl = 0;

// 不完全積分
far float phi_2d=0;
far float p2_theta = 0.0;
far float theta_phi_s[2];
far float u_sa[2] = 0.0;
far float u_sb[2] = 0.0;
far float u_sa_ref[2] = 0.0;
far float u_sb_ref[2] = 0.0;
far float w_q = 10;
far float w_lpf = 100;
far float theta_rep = 0.0;

// ZVV挿入関係
far float nrm_vi = 0.0;
far float nrm_vi_ref = 0.0;
far float delta = 0.0;
far float theta_err = 0.0;
far float Vtheta_out = 0.0;
far float Vtheta_ref = 0.0;

// 線間電圧、相電圧、電圧積分値計算
far float Vu_inv = 0.0, Vv_inv = 0.0, Vw_inv = 0.0;
far float Vwu = 0.0, Vuv = 0.0, Vvw = 0.0;
far float Vu, Vv, Vw;
far float Vu_ref = 0.0, Vv_ref = 0.0, Vw_ref = 0.0;
far float Vun_ref = 0.0, Vvn_ref = 0.0, Vwn_ref = 0.0;
far float Vest_a = 0.0, Vest_b = 0.0;
far float Pout_a = 0.0, Pout_b = 0.0;
far float Pref_a, Pref_b = 0.0;
far float Perr_a, Perr_b = 0.0;
far float Perr_nrm = 0.0;
far float Hysteresis_Band = 0.0;
far float Vtheta = 0.0;

// スイッチング周波数計算
far uint8_t U_tr[2];
far uint8_t V_tr[2];
far uint8_t W_tr[2];
far float sf_u_ave = 0.0;
far float sf_v_ave = 0.0;
far float sf_w_ave = 0.0;
far float uvw_sw = 0.0;
far float average_time = 0.0;
far int sf_Counter = 0;
far float sf_ave = 0.0;
far float SF_average = 0.0;

// 電圧指令値計算用
far float Vref_nrm = 0.0;
far float Vref_a = 0.0, Vref_b = 0.0;
far float mod = 0.0;
far float f_ref = 0.0;
far float wt = 0.0;
far float dwt = 0.0;
far float Kvf = 0.0;
far float omega_ref = 0.0;

// サーボのアナログ入力
far volatile float data_dac[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
far volatile float omega_load_ref = 0.0;
far volatile float tau_load_ref = 0.0;

// 汎用変数
far int i = 0;
far int j = 0;
far int n = 0;
#pragma SET_DATA_SECTION()


// キャリア同期割り込み:センサ値取得用
#pragma CODE_SECTION(IntrFunc2, ".CODE_ON_HIGHER_SPEED")
interrupt void IntrFunc2(void) {	
	int2_ack(); // 外部割込みアクノリッジの通知（割り込み関数戦闘で必ず呼び出し）

	C6657_timer0_start();   // タイマによる割り込み関数処理速度計測開始

	float data[12] = { 0 };

	PEV_ad_start(SENS_BDN);
	while (PEV_ad_in_st(SENS_BDN)) {}
	PEV_ad_in_grp(SENS_BDN, data);

	// ********** センサオフセット計算 ********** //
	if(offset_time < 5.0){
		for(i = 0; i < 8; ++i){
			data_offset_tmp[i] += data[i];
		}
		offset_count++;
		offset_time = offset_count*TS_SAMPLE;

		for(i = 0; i < 8; ++i){
			data_offset[i] = data_offset_tmp[i]/offset_count;
		}
	}
	
	// *********** AD変換読み取り *********** //
	if(offset_time >= 5.0){
		for(i = 0; i < 8; ++i){
			data_offset_tmp[i] = 0.0;
		}

		iu_res = data[0]-data_offset[0];
		iv_res = data[1]-data_offset[1];
		iw_res = data[2]-data_offset[2];

		Vdc = data[5]-data_offset[5];
		// Vdc = 282.0;
		if(Vdc > 280.0){
			Vdc = 282.0;
		}

		t_res = data[6]-data_offset[6];
	}

	//デバッグするときにVdc=282[V]にする
	if(Flag_Vdc_command == 1){
		Vdc = Vdc_command;
	}

	// ************** センサフィードバックの取得 ************** //
    m.ms.ires.uvw[0] = iu_res;
    m.ms.ires.uvw[1] = iv_res;
    m.ms.ires.uvw[2] = iw_res;
	m.ms.torque = t_res;

   // ************* エンコーダパルス検出と速度計算 ************* //
	// エンコーダ情報読み取り
	DSP_InputEncoderData(FS, SENS_BDN, ENC_data, omega_data, &lpf);
	wrm = omega_data[0];
	wrm_filter = omega_data[1];
	wre = wrm_filter * m.mp.p;
	m.ms.theta_1 = ENC_data[1];
    m.ms.theta_2 = ENC_data[0];
    m.ms.omega_rm = wrm;

	// dq軸二次鎖交磁束位相の演算
    IM_CalcStatus(&m, TS);
	
	// ***** 状態量推定 ***** //
    uvw2ab(m.ms.ires.uvw[0], m.ms.ires.uvw[1], m.ms.ires.uvw[2], &(m.ms.ires.ab[0]), &(m.ms.ires.ab[1]));
    ab2dq(m.ms.ires.ab[0], m.ms.ires.ab[1], m.ms.phi2.phase + TS*m.ms.omega_1, &(m.ms.ires.dq[0]), &(m.ms.ires.dq[1]));
    
	// 変数確認用
    id_res = m.ms.ires.dq[0];
    iq_res = m.ms.ires.dq[1];	

	// ******** 処理時間計算 ******** //
	Tact[0] = (float)C6657_timer0_read() * 4.8 * 1e-3; // (4.8 [ns/count] -> [us]
	C6657_timer0_stop();
    C6657_timer0_clear();

	return;
}

// キャリア同期割り込み:制御用
#pragma CODE_SECTION(IntrFunc1, ".CODE_ON_HIGHER_SPEED")
interrupt void IntrFunc1(void) {
	int1_ack(); // 外部割込みアクノリッジの通知（割り込み関数戦闘で必ず呼び出し）

	C6657_timer0_start();   // タイマによる割り込み関数処理速度計測開始
	
	// ******** DCリンク電圧に同期して速度上昇 ******** //
	if(Flag_omegaControl){
		for(i = 0; i < 20; ++i){
			if((Vdc > 14.1*i) && (Vdc <= 14.1*(i+1))){
				omega_ref = 18.0*i;
			}
		}

		if(Vdc > 275){
			omega_ref = 360;
		}
	}
		
	// **************** ASPSテーブルの書き込み **************** //	
	if(Vector_Number <= 1 && Flag_TableRead[0]){
		Flag_TableRead[1] = 1;
	}
	if(Flag_TableRead[1]){
		Table_Reference(VV_sequence, &Tc, &numLines, Table_num, Slot_Num, &write_step);	
		if(++write_step == WRITE_DIVISION){
			write_step = 0;
			Flag_TableRead[0] = 0;
			Flag_TableRead[1] = 0;
		}
	}

	confirm[0] = VV_sequence[1][confirm_number[0]];
	confirm[1] = VV_sequence[1][confirm_number[1]];
	

	// ****************** テーブル読み出し & ZVV挿入 ***************** //
	if(inte_mod.inte_enable == TRUE){
		// ZVV挿入
		#ifdef NOMINAL_MODE
		Flag_zero = 0;
		#endif
		if(Flag_zero){
			if(out_swp == 1 || out_swp == 3 || out_swp == 5 || out_swp == 0){
				out_swp = 0;
			}else{
				out_swp = 7;
			}
			goto zero;
		}

		// テーブル読み出し
		back:
		if(Vector_Times >= VV_sequence[1][Vector_Number]){
			Vector_Times = 1;
			Vector_Number++;
		}else{
			Vector_Times++;
		}
		if(Vector_Number >= numLines){
			Vector_Number = 0;
		}
		out_swp=VV_sequence[0][Vector_Number];

		// テーブルにもとからあるZVVを除く
		#ifndef NOMINAL_MODE
		if(out_swp == 0 || out_swp == 7){
			Vector_Times = 0;
			Vector_Number++;
			goto back;
		}
		#endif
	}else{
		Vector_Number = 0;
		Vector_Times = 0;
	}
	zero:

	// **************** 出力電圧推定 *************** //
	Inv_UpdateOutputVoltage(&inv, Vdc);
	Vest_a = inv.vout[out_swp].ab[0];
	Vest_b = inv.vout[out_swp].ab[1];

	// ************* スイッチング状態 ************* //
	u = SwitchingState[out_swp][0];
	v = SwitchingState[out_swp][1];
	w = SwitchingState[out_swp][2];
	du = ModulationSwitchingState[out_swp][0];
	dv = ModulationSwitchingState[out_swp][1];
	dw = ModulationSwitchingState[out_swp][2];

	// *************** 電圧指令値計算 ************** //
	Vref_nrm = omega_ref * K_VF;

	Vref_a = Vref_nrm*cos(Vtheta);
	Vref_b = Vref_nrm*cos(Vtheta-0.5*M_PI);

	Vtheta += omega_ref * TS;
	if(Vtheta > M_PI){
		Vtheta -= 2*M_PI;
	}

	// **************** VI計算 **************** //
	// Puseudo_integrator(&inte_values.res, inv.vout[out_swp].ab[0], inv.vout[out_swp].ab[1], TS, 10, omega_ref);
	Puseudo_integrator(&inte_values.res, Vest_a, Vest_b, TS, 10, omega_ref);
	Puseudo_integrator(&inte_values.ref, Vref_a, Vref_b, TS, 10, omega_ref);

	// 変数の確認用
	Pout_a = inte_values.res.y_a[0];
	Pout_b = inte_values.res.y_b[0];
	Pref_a = inte_values.ref.y_a[0];
	Pref_b = inte_values.ref.y_b[0];

	// ************* VI位相差からZVV挿入フラグの判定 ************ //
	// delta = 0.2;  // 位相差許容値(インスペクタ入力)	
	Integral_modulation(&inte_mod, inte_values, delta, &Flag_zero);

	// 変数の確認用
	nrm_vi = inte_mod.nrm_vi;
	nrm_vi_ref = inte_mod.nrm_vi_ref;
	Vtheta_out = inte_mod.theta_vi;
	Vtheta_ref = inte_mod.theta_vi_ref;
	theta_err = inte_mod.theta_vi_err;

	// ********** スイッチング周波数計算 *********** //
	SF_calculation(&SW_Inst, TS);
	SF_average = SW_Inst.SF_average;

	// average_time = 1.0; 
	// sf_Counter++;

    // U_tr[0] = u;
    // V_tr[0] = v;
    // W_tr[0] = w;
    // //u相立ち上がり
    // if(U_tr[0] > U_tr[1]){
    //     uvw_sw++;
    // }
    // //v相立ち上がり
    // if(V_tr[0] > V_tr[1]){
    //     uvw_sw++;
    // }
    // //u相立ち上がり
    // if(W_tr[0] > W_tr[1]){
    //     uvw_sw++;
    // }
    
    // if(TS*sf_Counter > average_time){
    //     sf_ave = (uvw_sw/3)/(sf_Counter*TS);
    //     uvw_sw = 0.0;
    //     sf_Counter = 0.0;
    // }

    // U_tr[1] = U_tr[0];
    // V_tr[1] = V_tr[0];
    // W_tr[1] = W_tr[0];

	// ******** PWMの関数に各相のオンオフ信号を渡す ******** //
	PEV_inverter_set_uvw(CTRL_BDN, du, dv, dw, 0);

	// ******** DACの信号書き込み ******** //
	// アナログトルク指令の値の上限値
	if(tau_load_ref > 2.0){
		tau_load_ref = 2.0;
	}
	// DACの出力
	data_dac[0] = tau_load_ref/1.0;
	DAC_da_out(DAC_BDN, 0 , data_dac[0]);

	// ******** 制御に関する処理時間計算 ******** //
	Tact[1] = (float)C6657_timer0_read() * 4.8 * 1e-3; // (4.8 [ns/count] -> [us]
	C6657_timer0_stop();
    C6657_timer0_clear();

	Tact_ctrl = Tact[1];
	Tact[2] = Tact[0] + Tact[1];
}

void MW_main(void) {
	
	// ***** IMパラメータをセット ***** //
    // {R1 [ohm], R2 [ohm], L1 [H], L2 [H], M [H], p [-], Jm [Nm/(rad/s^2)], Dm [Nm/(rad/s)]}
    static float IM_parameter[] = {
        0.411, 0.382, 90.5e-3, 90.5e-3, 86.7e-3, 2, 0.0183, 0
    };

	// ***** 各種構造体初期化 ***** //
	// *** IM *** //
    IM_Construct(&m, IM_parameter, sizeof(IM_parameter)/sizeof(float), TS);
	// *** センサフィルタ用LPF *** //
    LPF_Construct(&lpf, LPF_Z0, 600, TS);
	// *** 積分器 *** //
	Integral_Construct(&inte_values.res);
	Integral_Construct(&inte_values.ref);
	// *** 電圧積分変調 *** //
    Integral_modulation_Construct(&inte_mod);
	// *** ASPSテーブル *** //
	Init_Table(VV_sequence);
	// *** スイッチング周波数計算 *** //
	SF_calculation_Constract(&SW_Inst);
	
	float range[] = { CURRENT_SENSOR_RANGE, CURRENT_SENSOR_RANGE, CURRENT_SENSOR_RANGE, 5, 5, DCLINK_VOLTAGE_RANGE, TORQUE_SENSOR_RANGE, 31.25};
	//range = range of ch (CH0 = iu, CH1 = iv, CH2 = iw, CH3 = Vdc, CH4 = Vvw, CH5 = Vdc, CH6 = Torque, CH7 = NC)

	// ******* PEVボードの設定 ******* //
    // PEV_init(CTRL_BDN);  　　  // PEVボードの初期化。ソフトウェア実行前に内部パラメータを初期化
    // PEV_init(SENS_BDN);  // PEVボードの初期化。ソフトウェア実行前に内部パラメータを初期化

	// ***** 割り込み設定（拒否・設定） ***** //
	PEV_int_init(CTRL_BDN, 0, 2, 0, 0, 0, 0, 0, 0);			// 割り込み設定 (int1をキャリア割り込み）
	PEV_int_init(SENS_BDN, 0, 0, 2, 0, 0, 0, 0, 0); 	// 割り込み設定 (int2をキャリア割り込み）
	int1_init_vector(IntrFunc1, (CSL_IntcVectId)6, FALSE);	// 外部割込み設定（int1でIntrFunc1を呼び出し）
	int2_init_vector(IntrFunc2, (CSL_IntcVectId)5, FALSE);	// 外部割込み設定（int2でIntrFunc2を呼び出し）

	// ***** キャリア波設定 ***** //
	PEV_inverter_init(CTRL_BDN, FS, DEADTIME);				//キャリア周波数とデッドタイムを設定
	PEV_inverter_init(SENS_BDN, SAMPLING_FREQ, DEADTIME);         // キャリアとデッドタイムの設定 (割込み用なのでデッドタイムは関係なし)

    // ***** キャリア同期設定 ***** //
	PEV_sync_pwm_init(CTRL_BDN, 1, 0);				// 制御用ボードは1:マスタ、同期先信号はNo.0(自身)
	PEV_sync_pwm_init(SENS_BDN, 0, 1);		// センサ用ボードは0:スレーブ、同期先信号はNo.1
	//PEV_sync_ad_init(0,1,0);		// センサ用ボードは0:スレーブ、同期先信号はNo.1
	PEV_sync_pwm_out(CTRL_BDN, 1, 0, 0);	// マスタボードからキャリアの山に同期した信号を同期先信号No.1として出力
	
	// ****************** A/D設定 ******************* //
	PEV_ad_set_range(SENS_BDN, range);	// A/D変換入力電圧レンジ設定
	PEV_ad_set_mode(SENS_BDN, 1);
	PEV_inverter_init_int_timing(CTRL_BDN, 1, 0, 0);     // キャリア同期タイミング設定（山割り込み、オフセット0ns）
	PEV_inverter_init_adtrig_timing(SENS_BDN, 1, 0);

	// *************** DACの設定 *************** //
	DAC_da_set_offset(DAC_BDN, 0, 0);
	DAC_da_set_range(DAC_BDN, 0, 10);
	DAC_da_set_offset(DAC_BDN, 1, 0);
	DAC_da_set_range(DAC_BDN, 1, 10);
	DAC_da_set_offset(DAC_BDN, 2, 0);
	DAC_da_set_range(DAC_BDN, 2, 10);
	DAC_da_set_offset(DAC_BDN, 3, 0);
	DAC_da_set_range(DAC_BDN, 3, 10);
	DAC_da_set_offset(DAC_BDN, 4, 0);
	DAC_da_set_range(DAC_BDN, 4, 10);
	DAC_da_set_offset(DAC_BDN, 5, 0);
	DAC_da_set_range(DAC_BDN, 5, 10);
	DAC_da_set_offset(DAC_BDN, 6, 0);
	DAC_da_set_range(DAC_BDN, 6, 10);
	DAC_da_set_offset(DAC_BDN, 7, 0);
	DAC_da_set_range(DAC_BDN, 7, 10);
	DAC_da_set_offset(DAC_BDN, 8, 0);
	DAC_da_set_range(DAC_BDN, 8, 10);

	// ***** キャリア比較変調設定 ***** //
	PEV_inverter_set_uvw(CTRL_BDN, 0.0, 0.0, 0.0, 0);		//3相指令値の初期化
	PEV_inverter_set_uvw(SENS_BDN, 0.0, 0.0, 0.0, 0);	//3相指令値の初期化(一応)

	// ***** エンコーダ設定 ***** //
	PEV_abz_init_maxcount(SENS_BDN, ENC_MAX);//エンコーダの分解能決定
	PEV_abz_set_mode(SENS_BDN, 0, 0);

	// ***** 割り込み設定（許可・開始） ***** //
    PEV_inverter_enable_int(CTRL_BDN);       //キャリア同期割り込み信号出力開始
    PEV_inverter_enable_int(SENS_BDN);  //キャリア同期割り込み信号出力開始
    PEV_ad_enable_int(CTRL_BDN);             //AD変換完了割り込み許可（一応）
    PEV_ad_enable_int(SENS_BDN);        //AD変換完了割り込み許可

	// ******* DSPボード設定 ******* //
	int1_enable_int();				//外部からの割り込み入力の許可
	int2_enable_int();				//外部からの割り込み入力の許可
	int_enable();					//多重割り込み入力の許可

	C6657_timer0_init(10e6);	// タイマ設定
	C6657_timer1_init(10e6);	// タイマ設定

	//int_disable();	// 全割り込みを禁止

	wait(2000);						// 半キャリア以上待った後，3相PWM信号を出力 (200[us]の無駄時間発生)制御周期に合わせる

	PEV_inverter_start_pwm(CTRL_BDN);	// 3相PWM信号を出力
	PEV_pio_set_bit(SENS_BDN, 13);
									
	while (1);
}

