#include <iostream>
#include "car_auth.h"
#ifndef DEBUG
	#define DEBUG 1
#endif // !DEBUG



using namespace std;

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

	//生成本车的非对称密钥对
	if (RSA_generate_key_ex(rsa, 2048, bne, NULL) != 1) {
		printf("RSA key generation failed!\n");
	}
	else {
		printf("RSA key generation success!\n");
	}

	//获得密钥对中的公玥并转换为字符串存于内存中
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
/* 验证消息合法性
	输入是一个数据包
	输出0为不合法，1为合法
*/
/************************************************************************/
int CarAuth::checkMsg(struct Msg_Item item)
{
	int timeDiff = 10000; //10s
	//检查CRC
	if (!checkCRC(item))
	{
		cout << "CRC does not Match !" << endl;
		return 0;
	}
	//验证签名
	if (verifySig(item)!=1) {
		cout << "Verify Signature Failed !" << endl;
		return 0;
	}
	//验证时间戳
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
/* 检查消息包中的校验码正确性                                                               */
/************************************************************************/
int CarAuth::checkCRC(Msg_Item item)
{
	int ret=0;
	unsigned int caculatedCRC = 0;//算出来的CRC
	unsigned int realCRC = 0;//数据包里的CRC

	realCRC = item.chksum;
	caculatedCRC = getCRC32((char *)&item, MSG_LEN - sizeof(unsigned int));

	ret = realCRC == caculatedCRC ? 1 : 0;

	return ret;
}
/*
	获取消息中的公钥指针
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
/* 利用自身的私钥来计算签名 data是明文，len是明文长度*/
/************************************************************************/
unsigned char * CarAuth::getSig(char * data,int len)
{
	unsigned int sig_len = 0;
	//加载OpenSSL的算法，方便按照名字查找
	OpenSSL_add_all_digests();
	
	EVP_MD_CTX ctx;
	//初始化EVP_PKEY，并设置为RSA，并设置rsa结构体
	EVP_PKEY *pkey = EVP_PKEY_new();
	EVP_PKEY_set1_RSA(pkey, rsa);
	int pkey_size = 0;
	pkey_size = EVP_PKEY_size(pkey);
	unsigned char signature[256];

	//初始化EVP_MD，设置为SHA256
	const EVP_MD *md = EVP_get_digestbyname("sha256");
	if (!md)
	{
		printf("Cannot get message digest algorithm SHA256");
		return nullptr;
	}
	//开始签名明文data
	EVP_MD_CTX_init(&ctx);
	EVP_SignInit_ex(&ctx, md,NULL);
	EVP_SignUpdate(&ctx, data, len);
	EVP_SignFinal(&ctx, signature,&sig_len, pkey);
	
//	EVP_MD_CTX_destroy(&ctx);
	EVP_PKEY_free(pkey);
	return signature;
}

/************************************************************************/
/* 利用消息包中的公钥来验证该数据包
	1是成功，0是失败，-1是错误
*/
/************************************************************************/
int CarAuth::verifySig(Msg_Item item)
{
	int res = 0;
	unsigned char * signature;
	signature = (unsigned char*)item.signature;//获取签名的指针
	int sig_len = 256; //签名长度

	//从消息结构中获取公钥，并附着到EVP_PKEY上
	RSA *o_rsa = getPublickey(item);
	EVP_PKEY *pkey = EVP_PKEY_new();
	OpenSSL_add_all_digests();
	EVP_PKEY_set1_RSA(pkey,o_rsa);

	//设置摘要上下文环境
	EVP_MD_CTX ctx;
	const EVP_MD *md = EVP_get_digestbyname("sha256");
	if (!md) {
		printf("Cannot get message digest algorithm SHA256");
		return 0;
	}
	//初始化并验证签名
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
/* 填充数据包中的公钥证书部分，时间戳部分，签名，最后填充CRC32     */
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
