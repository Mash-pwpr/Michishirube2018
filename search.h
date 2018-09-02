/*
==============================================================
 Name        : search.h
 Copyright   : Copyright (C) ����c��w�}�C�N���}�E�X�N���u
 Description : MicroMouse�̒_�ł��D

  �X�V����
 2015/12/19		�R��@�R�����g�ǉ�
==============================================================
*/


#ifndef SEARCH_H_											//�Ή��t�@�C���ň�x���ǂݍ��܂�Ă��Ȃ��ꍇ�ȉ����`
	#define SEARCH_H_										//�ǂݍ��񂾂��Ƃ�\��
/*============================================================
		�e��萔��ϐ��錾
============================================================*/
	//���H�̐�΍��W�ɂ��āA�X�^�[�g�n�_�������ɂȂ�悤�Ȉʒu���猩���Ƃ��A
	//�������k�A�E�����𓌁A���������A�������𐼂Ƃ��Ē�`���Ă���B
	//m_dir�̏ꍇ�A�k��0x00�A����0x01�A���0x02�A����0x03�ŕ\�����B(turn_dir�֐��Q��)
	//�܂��A�}�b�v�i�[�z��map[][]�ɂ��āA�v�f�͏��4nit�Ɖ���4bit�ɕ�����Ă��āA
	//�k�ǂ�3bit�ځA���ǂ�2bit�ځA��ǂ�1bit�ځA���ǂ�0bit�ڂɁA
	//�ǂ�����ꍇ��1�A�Ȃ��ꍇ��0���i�[����Ă���B
	//����4bit��1�����s�p(���T���ǂ͖����Ɣ���)�ŏ��4bit��2�����s�p(���T���ǂ͗L��Ɣ���)
	//��\���Ă���B(write_map�֐������map_Init�֐��Q��)
	//�Ō�ɁA�ŒZ�o�H�i�[�z��route[]�ɂ��āA�i�s���Ɉړ����L�^����Ă���B
	//�e�v�f�ɋ@�̂��O�i����ۂ�0x88���A�E�܂���ۂ�0x44���AU�^�[������ۂ�0x22���A���܂���ꍇ��0x11���A
	//����ȊO�̏ꍇ�ɂ�0x00���i�[�����B(make_route�֐��Q��)�B�Ȃ��A�i�s�̌o�߂�r_cnt�ŊǗ�����Ă���B

	//====�ϐ�====
	#ifdef EXTERN											//�Ή��t�@�C����EXTERN����`����Ă���ꍇ
		/*�O���[�o���ϐ��̒�`*/
		unsigned char map[16][16];									//�}�b�v�i�[�z��
		unsigned char smap[16][16];									//�����}�b�v�i�[�z��
		unsigned char wall_info;									//�Ǐ��i�[�ϐ�
		unsigned char m_dir;										//�}�E�X�̕���
		unsigned char m_step;										//�����i�[
		uint16_t goal_x, goal_y;								//�S�[�����W
		unsigned char route[256];									//�ŒZ�o�H�i�[�z��
		unsigned char r_cnt;										//�o�H�J�E���^
	#else													//�Ή��t�@�C����EXTERN����`����Ă��Ȃ��ꍇ
		/*�O���[�o���ϐ��̐錾*/
		extern unsigned char map[16][16];							//�}�b�v�i�[�z��
		extern unsigned char smap[16][16];							//�����}�b�v�i�[�z��
		extern unsigned char wall_info;								//�Ǐ��i�[�ϐ�
		extern unsigned char m_dir;									//�}�E�X�̕���
		extern unsigned char m_step;								//�����i�[
		extern uint16_t goal_x, goal_y;							//�S�[�����W
		extern unsigned char route[256];							//�ŒZ�o�H�i�[�z��
		extern unsigned char r_cnt;									//�o�H�J�E���^
	#endif

/*============================================================
		�֐��v���g�^�C�v�錾
============================================================*/
	//====�T���n====
	void searchA();											//�����~�^�T�����s
	void searchSA();										//�A���T�����s
	void adv_pos();											//�}�E�X�̈ʒu����O�i
	void conf_route();										//�����[�g�̊m�F
	void map_Init();										//�}�b�v�f�[�^������
	void write_map();										//�}�b�v��������
	void turn_dir(unsigned char);									//���@�������ύX
	void make_smap();										//�����}�b�v�쐬
	void make_route();										//�ŒZ�o�H����

#endif /* SEARCH_H_ */
