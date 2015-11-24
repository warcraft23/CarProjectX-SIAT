#include "car_auth.h"

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
	for (int i = 0; i < 256; i++) {
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
	if (RSA_generate_key_ex(rsa, 1024, bne, NULL) != 1) {
		printf("RSA key generation failed!\n");
	}
	else {
		printf("RSA key generation success!\n");
	}

}

CarAuth::~CarAuth()
{
	BN_free(bne);
	RSA_free(rsa);
}

int CarAuth::checkMsg(struct Msg_Item item)
{
	return 0;
}
int CarAuth::checkCRC(Msg_Item item)
{
	int ret=0;

	return ret;
}
/*
	��ȡ��Ϣ�еĹ�Կָ��
*/
RSA * CarAuth::getPublickey(Msg_Item item)
{
	RSA *rsa =nullptr;
	rsa = &(item.rsa);
	return rsa;
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
