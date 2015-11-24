#pragma once

/* 
* 该数据包是否含有认证功能
* 的标志
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
	共40个字节
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
	共112个字节
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
		由预共享密钥加密的公钥数据
	*/
	RSA rsa;
	/************************************************************************/
	/* 时间戳                                                                     */
	/************************************************************************/
	SYSTEMTIME timestamp;
	/************************************************************************/
	/* 数字签名                                                                     */
	/************************************************************************/
	char signature[32];

	/************************************************************************/
	/* 校验和                                                                     */
	/************************************************************************/
	int chksum;
};
#endif


#define MSG_LEN (sizeof(struct Msg_Item))
