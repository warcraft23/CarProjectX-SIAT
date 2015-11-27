#pragma once

#ifndef CARAUTH
#define CARAUTH 1
#endif

#include "message.h"
#include "openssl/evp.h"
#include "openssl/rsa.h"
#include "openssl/pem.h"
#include "openssl/bio.h"


/************************************************************************/
/* 车辆认证类                                                                     
	share_key 预共享密钥，用于对数据包中的公钥结构进行加密
	public_key&private_key 非对称密钥体系，用于对数据包的hash
	进行签名以及验证
*/
/************************************************************************/
class CarAuth
{
public:
	int checkMsg(struct Msg_Item item);
	int checkCRC(struct Msg_Item item);
	RSA * getPublickey(struct Msg_Item item);
	SYSTEMTIME getCurrentTimestamp(); 
	SYSTEMTIME getTimestamp(struct Msg_Item item);
	time_t fileTimeToTime_t(const FILETIME *ft);
	int getDiffSeconds(const SYSTEMTIME t1, const SYSTEMTIME t2);
	unsigned char * getSig(char * data,int len);
	int verifySig(struct Msg_Item item);
	int packPkt(struct Msg_Item * item);
	unsigned int getCRC32(char * data, int len);
	void makeTable();



	CarAuth(int share_seed);
	~CarAuth();

	//废弃
	int hash(const char * data, unsigned int dataLength, char * &hashData, unsigned int &hashLength);

protected:
private:
	

	unsigned int CRC32Table[256];
	char tableMaked;
	
	/************************************************************************/
	/*						预共享密钥                                                                     */
	/************************************************************************/
	int seed;
	/************************************************************************/
	/*                     非对称密钥体系				                                                 */
	/************************************************************************/
	RSA * rsa;
	BIGNUM * bne;
	char *my_pubkey;
	long my_pub_len;
	char *o_pubkey;
	long o_pub_len;
	BIO * my_mem; //我的公玥存放的内存
	BIO * o_mem; //我收到的公玥的内存
};
