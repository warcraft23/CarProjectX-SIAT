#pragma once

#ifndef CARAUTH
#define CARAUTH 1
#endif

#include "message.h"
#include <openssl/evp.h>


/************************************************************************/
/* ������֤��                                                                     
	share_key Ԥ������Կ�����ڶ����ݰ��еĹ�Կ�ṹ���м���
	public_key&private_key �ǶԳ���Կ��ϵ�����ڶ����ݰ���hash
	����ǩ���Լ���֤
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
	/*						Ԥ������Կ                                                                     */
	/************************************************************************/

	/************************************************************************/
	/*                     �ǶԳ���Կ��ϵ				                                                 */
	/************************************************************************/

};
