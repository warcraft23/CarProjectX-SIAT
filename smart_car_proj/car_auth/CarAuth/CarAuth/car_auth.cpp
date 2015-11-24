#include "car_auth.h"

/*
*	计算以char指针data为首的len长度的数据的CRC值
*	输入：data数据段的起始指针，len数据段的长度
*	输出：循环校验值，某unsigned int
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
*	制作CRC的表
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

	//生成本车的非对称密钥对
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
	获取消息中的公钥指针
*/
RSA * CarAuth::getPublickey(Msg_Item item)
{
	RSA *rsa =nullptr;
	rsa = &(item.rsa);
	return rsa;
}

/************************************************************************/
/* 
	对输入的数据  进行hash 产生一个hash值
	其中data是输入数据，hashData是输出的结果
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
/* 获取当前时间，以SYSTEMTIME结构                                                                     */
/************************************************************************/
SYSTEMTIME CarAuth::getCurrentTimestamp()
{
	SYSTEMTIME localTimestamp = { 0 };
	GetLocalTime(&localTimestamp);
	return localTimestamp;
}


/************************************************************************/
/* 获取 消息结构中的时间戳                                                                     */
/************************************************************************/
SYSTEMTIME CarAuth::getTimestamp(Msg_Item item)
{
	SYSTEMTIME timestampInItem = { 0 };
	timestampInItem = item.timestamp;
	return timestampInItem;
}
