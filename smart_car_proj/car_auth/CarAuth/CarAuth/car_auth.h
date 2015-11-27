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

	//����
	int hash(const char * data, unsigned int dataLength, char * &hashData, unsigned int &hashLength);

protected:
private:
	

	unsigned int CRC32Table[256];
	char tableMaked;
	
	/************************************************************************/
	/*						Ԥ������Կ                                                                     */
	/************************************************************************/
	int seed;
	/************************************************************************/
	/*                     �ǶԳ���Կ��ϵ				                                                 */
	/************************************************************************/
	RSA * rsa;
	BIGNUM * bne;
	char *my_pubkey;
	long my_pub_len;
	char *o_pubkey;
	long o_pub_len;
	BIO * my_mem; //�ҵĹ��h��ŵ��ڴ�
	BIO * o_mem; //���յ��Ĺ��h���ڴ�
};
