#pragma once

/* 
* �����ݰ��Ƿ�����֤����
* �ı�־
*/

#ifndef CARAUTH
#define CARAUTH 0
#endif

#include <windows.h>
#include <openssl/rsa.h>


enum {
	TYPE_GENERIC = 1,
	TYPE_SPECIAL = 2,
	TYPE_DIRECTION = 4,
	TYPE_ECOMPASS = 8
};

#if not CARAUTH
/*
	��40���ֽ�
*/
struct Msg_Item
{
	int index;
	char type;
	short key;
	int val;
	union {
		struct color {
			short r;
			short g;
			short b;
		} color;
		struct array {
			int row;
			int col;
		} array;
		struct gps {
			double distance;
			int angle;
		}gps;
		struct ecompass {
			int angle;
		}ecompass;
		char buf[24];
	}u;
};
#else
/*
	��112���ֽ�
*/
struct Msg_Item
{
	int index;
	char type;
	short key;
	int val;
	union {
		struct color {
			short r;
			short g;
			short b;
		} color;
		struct array {
			int row;
			int col;
		} array;
		struct gps {
			double distance;
			int angle;
		}gps;
		struct ecompass {
			int angle;
		}ecompass;
		char buf[24];
	}u;

	/*
		��Ԥ������Կ���ܵĹ�Կ����
	*/
	RSA rsa;
	/************************************************************************/
	/* ʱ���                                                                     */
	/************************************************************************/
	SYSTEMTIME timestamp;
	/************************************************************************/
	/* ����ǩ��                                                                     */
	/************************************************************************/
	char signature[32];

	/************************************************************************/
	/* У���                                                                     */
	/************************************************************************/
	int chksum;
};
#endif


#define MSG_LEN (sizeof(struct Msg_Item))
