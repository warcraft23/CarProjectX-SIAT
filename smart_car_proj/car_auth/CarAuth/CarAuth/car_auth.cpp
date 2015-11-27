#include <iostream>
#include "car_auth.h"
#ifndef DEBUG
	#define DEBUG 1
#endif // !DEBUG



using namespace std;

/*
*	������charָ��dataΪ�׵�len���ȵ����ݵ�CRCֵ
*	���룺data���ݶε���ʼָ�룬len���ݶεĳ���
*	�����ѭ��У��ֵ��ĳunsigned int
*/
unsigned int CarAuth::getCRC32(char * data, int len)
{
	unsigned int ret = 0xFFFFFFFF;
	int i;
	unsigned char * buffer;
	if (!tableMaked) {
		makeTable();
		tableMaked = 1;
	}
	buffer = (unsigned char *)data;
	i = len;

	while (i--) {
		ret = (ret >> 8) ^ CRC32Table[(ret & 0xFF) ^ *buffer++];
	}
	return ret ^ 0xFFFFFFFF;
}

/*
*	����CRC�ı�
*/
void CarAuth::makeTable()
{
	int i, j;
	unsigned long crc;
	for (i = 0; i < 256; i++) {
		crc = i;
		for (j = 0; j < 8; j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0xEDB88320;
			else
				crc >>= 1;
		}
		CRC32Table[i] = crc;
	}
}

CarAuth::CarAuth(int share_seed)
{
	seed = share_seed;
	unsigned long e = RSA_F4;
	int ret = 0;
	tableMaked = 0;


	bne = BN_new();
	ret = BN_set_word(bne, e);
	rsa = RSA_new();

	//���ɱ����ķǶԳ���Կ��
	if (RSA_generate_key_ex(rsa, 2048, bne, NULL) != 1) {
		printf("RSA key generation failed!\n");
	}
	else {
		printf("RSA key generation success!\n");
	}

	//�����Կ���еĹ��h��ת��Ϊ�ַ��������ڴ���
	my_mem = BIO_new(BIO_s_mem());
	if (!PEM_write_bio_RSA_PUBKEY(my_mem, rsa)) {
		cout << "Write RSA Pubkey Failed!" << endl;
	}
	my_pubkey = NULL;
	my_pub_len = BIO_get_mem_data(my_mem, &my_pubkey);
	if (!my_pub_len) {
		cout << "In mypub Get BIO data Failed" << endl;
	}

#if DEBUG
	cout << "hex of n : 0x";
	cout << BN_bn2hex(rsa->n) << endl;
	cout << "hex of e : 0x";
	cout << BN_bn2hex(rsa->e) << endl;
	cout << "hex of d : 0x";
	cout << BN_bn2hex(rsa->d) << endl;
	cout << "hex of p : 0x";
	cout << BN_bn2hex(rsa->p) << endl;
	cout << "hex of q : 0x";
	cout << BN_bn2hex(rsa->q) << endl;
#endif

}

CarAuth::~CarAuth()
{
	BN_free(bne);
	RSA_free(rsa);
	BIO_free(my_mem);
	BIO_free(o_mem);
}

/************************************************************************/
/* ��֤��Ϣ�Ϸ���
	������һ�����ݰ�
	���0Ϊ���Ϸ���1Ϊ�Ϸ�
*/
/************************************************************************/
int CarAuth::checkMsg(struct Msg_Item item)
{
	int timeDiff = 10000; //10s
	//���CRC
	if (!checkCRC(item))
	{
		cout << "CRC does not Match !" << endl;
		return 0;
	}
	//��֤ǩ��
	if (verifySig(item)!=1) {
		cout << "Verify Signature Failed !" << endl;
		return 0;
	}
	//��֤ʱ���
	SYSTEMTIME timestamp = getTimestamp(item);
	SYSTEMTIME localTimestamp = getCurrentTimestamp();
	timestamp.wDayOfWeek = 0;
	localTimestamp.wDayOfWeek = 0;
	int diffSeconds = getDiffSeconds(timestamp, localTimestamp);
	if (diffSeconds <= 0) {
		cout << "TimeStamp is too late !" << endl;
		return 0;
	}
	if (diffSeconds >= timeDiff) {
			cout << "Sending Message costs too much time !" << endl;
			return 0;
	}
	return 1;
}
/************************************************************************/
/* �����Ϣ���е�У������ȷ��                                                               */
/************************************************************************/
int CarAuth::checkCRC(Msg_Item item)
{
	int ret=0;
	unsigned int caculatedCRC = 0;//�������CRC
	unsigned int realCRC = 0;//���ݰ����CRC

	realCRC = item.chksum;
	caculatedCRC = getCRC32((char *)&item, MSG_LEN - sizeof(unsigned int));

	ret = realCRC == caculatedCRC ? 1 : 0;

	return ret;
}
/*
	��ȡ��Ϣ�еĹ�Կָ��
*/
RSA * CarAuth::getPublickey(Msg_Item item)
{
	RSA *o_rsa;
	o_mem = BIO_new_mem_buf(item.pubkey, sizeof(item.pubkey));
	o_rsa = RSA_new();
	o_rsa = PEM_read_bio_RSA_PUBKEY(o_mem, &o_rsa, NULL, NULL);
#if DEBUG
	cout << "hex of n : 0x";
	cout << BN_bn2hex(o_rsa->n) << endl;
	cout << "hex of e : 0x";
	cout << BN_bn2hex(o_rsa->e) << endl;
#endif

	return o_rsa;
}

/************************************************************************/
/* 
	�����������  ����hash ����һ��hashֵ
	����data���������ݣ�hashData������Ľ��
*/
/************************************************************************/
int CarAuth::hash(const char * data, unsigned int dataLength, char *& hashData, unsigned int & hashLength)
{
	OpenSSL_add_all_digests();
	EVP_MD_CTX ctx;
	const EVP_MD * md = EVP_get_digestbyname("sha256");
	if (!md)
	{
		printf("Cannot get message digest algorithm SHA256");
		return -1;
	}
	hashData = (char *)malloc(EVP_MAX_MD_SIZE);
	memset(hashData, 0, EVP_MAX_MD_SIZE);

	EVP_MD_CTX_init(&ctx);
	EVP_DigestInit_ex(&ctx, md, NULL);
	EVP_DigestUpdate(&ctx, data, dataLength);
	EVP_DigestFinal_ex(&ctx,(unsigned char *)hashData,&hashLength);
	EVP_MD_CTX_cleanup(&ctx);

	return 0;
}

/************************************************************************/
/* ��ȡ��ǰʱ�䣬��SYSTEMTIME�ṹ                                                                     */
/************************************************************************/
SYSTEMTIME CarAuth::getCurrentTimestamp()
{
	SYSTEMTIME localTimestamp = { 0 };
	GetLocalTime(&localTimestamp);
	return localTimestamp;
}


/************************************************************************/
/* ��ȡ ��Ϣ�ṹ�е�ʱ���                                                                     */
/************************************************************************/
SYSTEMTIME CarAuth::getTimestamp(Msg_Item item)
{
	SYSTEMTIME timestampInItem = { 0 };
	timestampInItem = item.timestamp;
	return timestampInItem;
}

time_t CarAuth::fileTimeToTime_t(const FILETIME * ft)
{
	ULARGE_INTEGER ui;
	ui.LowPart = ft->dwLowDateTime;
	ui.HighPart = ft->dwHighDateTime;
	return ((LONGLONG)(ui.QuadPart - 116444736000000000) / 10000000);
}

int CarAuth::getDiffSeconds(const SYSTEMTIME t1, const SYSTEMTIME t2)
{
	FILETIME fTime1 = { 0,0 };
	FILETIME fTime2 = { 0,0 };
	SystemTimeToFileTime(&t1, &fTime1);
	SystemTimeToFileTime(&t2, &fTime2);

	time_t tt1 = fileTimeToTime_t(&fTime1);
	time_t tt2 = fileTimeToTime_t(&fTime2);

	return (int)(tt2-tt1);
}

/************************************************************************/
/* ���������˽Կ������ǩ�� data�����ģ�len�����ĳ���*/
/************************************************************************/
unsigned char * CarAuth::getSig(char * data,int len)
{
	unsigned int sig_len = 0;
	//����OpenSSL���㷨�����㰴�����ֲ���
	OpenSSL_add_all_digests();
	
	EVP_MD_CTX ctx;
	//��ʼ��EVP_PKEY��������ΪRSA��������rsa�ṹ��
	EVP_PKEY *pkey = EVP_PKEY_new();
	EVP_PKEY_set1_RSA(pkey, rsa);
	int pkey_size = 0;
	pkey_size = EVP_PKEY_size(pkey);
	unsigned char signature[256];

	//��ʼ��EVP_MD������ΪSHA256
	const EVP_MD *md = EVP_get_digestbyname("sha256");
	if (!md)
	{
		printf("Cannot get message digest algorithm SHA256");
		return nullptr;
	}
	//��ʼǩ������data
	EVP_MD_CTX_init(&ctx);
	EVP_SignInit_ex(&ctx, md,NULL);
	EVP_SignUpdate(&ctx, data, len);
	EVP_SignFinal(&ctx, signature,&sig_len, pkey);
	
//	EVP_MD_CTX_destroy(&ctx);
	EVP_PKEY_free(pkey);
	return signature;
}

/************************************************************************/
/* ������Ϣ���еĹ�Կ����֤�����ݰ�
	1�ǳɹ���0��ʧ�ܣ�-1�Ǵ���
*/
/************************************************************************/
int CarAuth::verifySig(Msg_Item item)
{
	int res = 0;
	unsigned char * signature;
	signature = (unsigned char*)item.signature;//��ȡǩ����ָ��
	int sig_len = 256; //ǩ������

	//����Ϣ�ṹ�л�ȡ��Կ�������ŵ�EVP_PKEY��
	RSA *o_rsa = getPublickey(item);
	EVP_PKEY *pkey = EVP_PKEY_new();
	OpenSSL_add_all_digests();
	EVP_PKEY_set1_RSA(pkey,o_rsa);

	//����ժҪ�����Ļ���
	EVP_MD_CTX ctx;
	const EVP_MD *md = EVP_get_digestbyname("sha256");
	if (!md) {
		printf("Cannot get message digest algorithm SHA256");
		return 0;
	}
	//��ʼ������֤ǩ��
	EVP_MD_CTX_init(&ctx);
	EVP_VerifyInit_ex(&ctx, md, NULL);
	EVP_VerifyUpdate(&ctx, &(item.index), (unsigned int)40);
	res = EVP_VerifyFinal(&ctx, signature, (unsigned int )sig_len, pkey);

	if (res == -1)
		cout << "Error occurs in Verifying the Signature !" << endl;
#if DEBUG
	else if (res == 0) 
		cout << "Signature Does Not Match !" << endl;
	else if (res == 1)
		cout << "Verify Success !" << endl;
	else
		cout << "Something Wrong!" << endl;
#endif

	return res;
}

/************************************************************************/
/* ������ݰ��еĹ�Կ֤�鲿�֣�ʱ������֣�ǩ����������CRC32     */
/************************************************************************/
int CarAuth::packPkt(struct Msg_Item * item)
{
	memcpy(item->pubkey, my_pubkey, my_pub_len);
	item->timestamp = getCurrentTimestamp();
	unsigned char * signature = getSig((char *)&(item->index), 40);
	memcpy(item->signature, signature, 256);
	item->chksum = getCRC32((char *)item, MSG_LEN - sizeof(unsigned int));
	return 0;
}
