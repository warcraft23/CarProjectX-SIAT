#pragma once

#ifndef CARAUTH
#define CARAUTH 1
#endif

#include "message.h"
#include <openssl/evp.h>


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
	int hash(const char * data, unsigned int dataLength, char * &hashData, unsigned int &hashLength);
	SYSTEMTIME getCurrentTimestamp();
	SYSTEMTIME getTimestamp(struct Msg_Item item);
	char * encrySig(char * data);
	char * decrySig(struct Msg_Item item);
	int packPkt(struct Msg_Item item, char signature[32]);
	unsigned int getCRC32(char * data, int len);
	void makeTable();


	CarAuth(int share_seed);
	~CarAuth();


protected:
private:
	RSA * rsa;
	int seed;
	BIGNUM *bne;
	unsigned int CRC32Table[32];
	int tableMaked;
	/************************************************************************/
	/*						预共享密钥                                                                     */
	/************************************************************************/

	/************************************************************************/
	/*                     非对称密钥体系				                                                 */
	/************************************************************************/

};
