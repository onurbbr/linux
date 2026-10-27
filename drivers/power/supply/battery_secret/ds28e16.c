/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Maxim DS28E16 secure authenticator found in Xiaomi batteries.
 *
 * The chip answers 1-Wire commands carried over the bit-banged bus provided
 * by the "xiaomi,onewire_gpio" controller. The driver exposes the ROM ID,
 * the page data, the chip status and a challenge-response authentication
 * result through the "batt_verify" power supply, plus the same values as
 * sysfs attributes for the factory tools.
 *
 * The battery identity pages are used by the SM5602 fuel gauge to pick the
 * correct battery profile, and the authentication result gates fast
 * charging.
 *
 * Copyright (C) 2015 Maxim Integrated Products, Inc.
 */

#define pr_fmt(fmt) "[ds28e16] " fmt

#include <crypto/hash.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "ds28e16.h"
#include "onewire_gpio.h"

#define ds_err(fmt, ...) pr_err(fmt, ##__VA_ARGS__)

struct ds28e16_data {
	struct platform_device *pdev;
	struct device *dev;

	struct power_supply *verify_psy;
	struct power_supply_desc verify_psy_d;

	struct delayed_work authentic_work;
	bool batt_verified;
};

static DEFINE_MUTEX(ds_cmd_lock);

static unsigned int attr_trytimes = 1;

static unsigned char session_seed[32] = {
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA
};
static unsigned char S_secret[32] = { 0x0C, 0x99, 0x2B, 0xD3, 0x95, 0xDB, 0xA0,
				      0xB4, 0xEF, 0x07, 0xB3, 0xD8, 0x75, 0xF3,
				      0xC7, 0xAE, 0xDA, 0xC4, 0x41, 0x2F, 0x48,
				      0x93, 0xB5, 0xD9, 0xE1, 0xE5, 0x4B, 0x20,
				      0x9B, 0xF3, 0x77, 0x39 };
static unsigned char challenge[32] = { 0x00 };
static int auth_ANON = 1;
static int auth_BDCONST = 1;
static int pagenumber = 1;

static unsigned short CRC16;
static const short oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1,
				     1, 0, 0, 1, 0, 1, 1, 0 };
static unsigned char last_result_byte = RESULT_SUCCESS;

static unsigned char MANID[2];

/*
 * Cached command results and authentication material. The DS28E16 is a
 * single-instance device, so this state is kept at file scope instead of
 * being threaded through the protocol helpers; ds_cmd_lock serialises the
 * command sequences that touch it.
 */
static unsigned char flag_mi_romid;
static unsigned char flag_mi_status;
static unsigned char flag_mi_page0_data;
static unsigned char flag_mi_page1_data;
static unsigned char flag_mi_counter;
static unsigned char flag_mi_auth_result;
static unsigned char mi_romid[8];
static unsigned char mi_status[8];
static unsigned char mi_page0_data[16];
static unsigned char mi_page1_data[16];
static unsigned char mi_counter[16];
static int mi_auth_result;

/* Retry wrappers defined below the raw command helpers. */
static int ds28el16_Read_RomID_retry(unsigned char *RomID);
static int ds28el16_get_page_status_retry(unsigned char *data);
static int ds28el16_get_page_data_retry(int page, unsigned char *data);
static int DS28E16_cmd_computeS_Secret_retry(int anon, int bdconst, int pg,
					     unsigned char *session_seed);
static int DS28E16_cmd_computeReadPageAuthentication_retry(
	int anon, int pg, unsigned char *challenge, unsigned char *hmac);

/*
 * HMAC-SHA3-256 over the concatenated authentication input, using the 32 byte
 * session secret as the key. The kernel's sha3-256 has the same 136 byte block
 * size as the reference implementation.
 */
static int ds28e16_compute_hmac(const u8 *key, size_t key_len, const u8 *msg,
				size_t msg_len, u8 *mac)
{
	struct crypto_shash *tfm;
	struct shash_desc *desc;
	int ret;

	tfm = crypto_alloc_shash("hmac(sha3-256)", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);

	desc = kzalloc(sizeof(*desc) + crypto_shash_descsize(tfm), GFP_KERNEL);
	if (!desc) {
		crypto_free_shash(tfm);
		return -ENOMEM;
	}

	desc->tfm = tfm;
	ret = crypto_shash_setkey(tfm, key, key_len);
	if (!ret)
		ret = crypto_shash_digest(desc, msg, msg_len, mac);

	kfree_sensitive(desc);
	crypto_free_shash(tfm);

	return ret;
}

static unsigned char crc_low_first(unsigned char *ptr, unsigned char len)
{
	unsigned char i;
	unsigned char crc = 0x00;

	while (len--) {
		crc ^= *ptr++;
		for (i = 0; i < 8; ++i) {
			if (crc & 0x01)
				crc = (crc >> 1) ^ 0x8c;
			else
				crc = (crc >> 1);
		}
	}

	return (crc);
}

static short Read_RomID(unsigned char *RomID)
{
	unsigned char i;
	unsigned char crc = 0x00;

	if ((flag_mi_romid == 2) && (mi_romid[0] != 0x1f)) {
		memcpy(RomID, mi_romid, 8);
		return DS_TRUE;
	}

	if ((ow_reset()) != 0) {
		ds_err("Failed to reset ds28e16!\n");
		ow_reset();
		Software_Reset();
		return ERROR_NO_DEVICE;
	}

	write_byte(CMD_READ_ROM);
	Delay_us(10);
	for (i = 0; i < 8; i++)
		RomID[i] = read_byte();

	crc = crc_low_first(RomID, 7);

	if (crc == RomID[7]) {
		memcpy(mi_romid, RomID, 8);
		if ((mi_romid[0] == FAMILY_CODE) &&
		    (mi_romid[6] == CUSTOM_ID_MSB) &&
		    ((mi_romid[5] & 0xf0) == CUSTOM_ID_LSB))
			flag_mi_romid = 2;
		else
			flag_mi_romid = 1;

		return DS_TRUE;
	} else {
		ow_reset();
		return DS_FALSE;
	}
}

static unsigned short docrc16(unsigned short data)
{
	data = (data ^ (CRC16 & 0xff)) & 0xff;
	CRC16 >>= 8;

	if (oddparity[data & 0xf] ^ oddparity[data >> 4])
		CRC16 ^= 0xc001;

	data <<= 6;
	CRC16 ^= data;
	data <<= 1;
	CRC16 ^= data;

	return CRC16;
}

static int DS28E16_standard_cmd_flow(unsigned char *write_buf, int delay_ms,
				     unsigned char *read_buf, int *read_len,
				     int write_len)
{
	unsigned char buf[128];
	int i;
	int buf_len = 0;
	int expected_read_len = *read_len;

	/*'1 Wire
	'''''''''''''''''''''''
	?<Reset/Presence>
	?<ROM level command Sequence>
	?TX: Start Command
	?TX: Length Byte
	?TX: Memory Command
	?TX: Parameter, TX: Data
	?RX: CRC16
	?TX: Release Byte
	?<Strong pull-up Delay>
	?RX: Dummy Byte
	?RX: Length Byte
	?RX: Result byte
	?RX: CRC16
	?< wait for reset>
	'''''''''''''''''''''''*/
	mutex_lock(&ds_cmd_lock);

	if ((ow_reset()) != 0) {
		ds_err("Failed to reset ds28e16!\n");
		ow_reset();
		Software_Reset();
		mutex_unlock(&ds_cmd_lock);
		return ERROR_NO_DEVICE;
	}

	write_byte(CMD_SKIP_ROM);

	buf[buf_len++] = CMD_START;
	memcpy(&buf[buf_len], write_buf, write_len);
	buf_len += write_len;
	for (i = 0; i < buf_len; i++)
		write_byte(buf[i]);

	buf[buf_len++] = read_byte();
	buf[buf_len++] = read_byte();

	CRC16 = 0;
	for (i = 0; i < buf_len; i++)
		docrc16(buf[i]);

	if (CRC16 != 0xB001) {
		ow_reset();
		mutex_unlock(&ds_cmd_lock);
		ds_err("DS28E16_standard_cmd_flow: crc error!\n");
		return DS_FALSE;
	}

	/* keep the strong pull-up asserted for the requested time */
	if (delay_ms > 0) {
		write_byte(CMD_RELEASE_BYTE);
		Delay_us(1000 * delay_ms);
	}

	read_byte();
	*read_len = read_byte();

	if (expected_read_len != *read_len) {
		ow_reset();
		mutex_unlock(&ds_cmd_lock);
		ds_err("DS28E16_standard_cmd_flow: len error!\n");
		return DS_FALSE;
	}

	buf_len = *read_len + 2;
	for (i = 0; i < buf_len; i++)
		buf[i] = read_byte();

	CRC16 = 0;
	docrc16(*read_len);
	for (i = 0; i < buf_len; i++)
		docrc16(buf[i]);

	if (CRC16 != 0xB001) {
		ow_reset();
		mutex_unlock(&ds_cmd_lock);
		ds_err("DS28E16_standard_cmd_flow: crc2 error!\n");
		return DS_FALSE;
	}

	memcpy(read_buf, buf, *read_len);
	mutex_unlock(&ds_cmd_lock);

	return DS_TRUE;
}

/*
 * 'Read Status' command: return the page protection byte in @data.
 */
static int DS28E16_cmd_readStatus(unsigned char *data)
{
	unsigned char write_buf[255];
	unsigned char read_buf[255];
	int write_len = 0;
	int len_byte = 1;
	int read_len = 7;

	if (flag_mi_status) {
		memcpy(data, mi_status, 8);
		return DS_TRUE;
	}

	last_result_byte = RESULT_FAIL_NONE;
	/*
	?<Start, device address write>
	?TX: Read Page Protection command
	?TX: Length (SMBus) [always 1]
	?TX: Parameter
	?<Stop>
	?<Delay>
	?<Start, device address read>
	?RX: Length (SMBus) [always 1]
	?RX: Protection Data
	?<Stop>
	*/

	write_buf[write_len++] = len_byte;
	write_buf[write_len++] = CMD_READ_STATUS;

	if (DS28E16_standard_cmd_flow(write_buf, DELAY_DS28E16_EE_READ,
				      read_buf, &read_len, write_len)) {
		if (read_buf[0] == RESULT_SUCCESS) {
			last_result_byte = read_buf[0];
			memcpy(data, &read_buf[1], 8);
			flag_mi_status = 1;
			memcpy(mi_status, data, 8);
			MANID[0] = data[4];
			return DS_TRUE;
		}
	}

	ow_reset();
	return DS_FALSE;
}
/*
 * 'Virtual Read Status' command: the response is not decoded, only the
 * pending bytes are clocked out.
 */
static void Virtual_cmd_readStatus(void)
{
	int num;
	ow_reset();
	write_byte(CMD_SKIP_ROM);

	write_byte(0x66);
	write_byte(0x01);
	write_byte(0xaa);
	read_byte();
	read_byte();
	write_byte(CMD_RELEASE_BYTE);
	Delay_us(50 * 1000);

	for (num = 0; num < 11; num++)
		read_byte();

	ow_reset();
}

/*
 * 'Read Memory' command: read page @pg into @data, which must have room
 * for at least 32 bytes.
 */
static int DS28E16_cmd_readMemory(int pg, unsigned char *data)
{
	unsigned char write_buf[3];
	unsigned char read_buf[255];
	int write_len = 0;
	int read_len = 33;
	int length_byte = 2;
	unsigned char pagenum = (unsigned char)pg & 0x03;

	switch (pagenum) {
	case 0x00:
		if (flag_mi_page0_data) {
			memcpy(data, mi_page0_data, 16);
			return DS_TRUE;
		}
		break;
	case 0x01:
		if (flag_mi_page1_data) {
			memcpy(data, mi_page1_data, 16);
			return DS_TRUE;
		}
		break;
	case 0x02:
		if (flag_mi_counter) {
			memcpy(data, mi_counter, 16);
			return DS_TRUE;
		}
		break;
	default:
		return DS_FALSE;
	}

	last_result_byte = RESULT_FAIL_NONE;

	/*
	?<Start, device address write>
	?TX: Read Memory Command
	?TX: Length (SMBus)
	?TX: Parameter
	?<Stop>
	?<Delay>
	?<Start, device address read>
	?RX: Length (SMBus)
	?RX: Result byte
	?RX: Data
	?<Stop>
	*/

	write_buf[write_len++] = length_byte;
	write_buf[write_len++] = CMD_READ_MEM;
	write_buf[write_len++] = pagenum;

	if (DS28E16_standard_cmd_flow(write_buf, DELAY_DS28E16_EE_READ,
				      read_buf, &read_len, write_len)) {
		if (read_len == 33) {
			last_result_byte = read_buf[0];
			if (read_buf[0] == RESULT_SUCCESS) {
				memcpy(data, &read_buf[1], 16);
				if (pagenum == 0x00) {
					flag_mi_page0_data = 1;
					memcpy(mi_page0_data, data, 16);
				}
				if (pagenum == 0x01) {
					flag_mi_page1_data = 1;
					memcpy(mi_page1_data, data, 16);
				}
				return DS_TRUE;
			} else {
				if (read_buf[0] == 0x55 && pagenum == 2) {
					memcpy(data, &read_buf[1], 16);
					flag_mi_counter = 1;
					memcpy(mi_counter, data, 16);
					return DS_TRUE;
				}
			}
		}
	}

	ow_reset();
	return DS_FALSE;
}

/*
 * 'Write Memory' command: write the 32-byte page @pg from @data.
 */
static int DS28E16_cmd_writeMemory(int pg, unsigned char *data)
{
	unsigned char write_buf[255];
	unsigned char read_buf[255];
	int write_len = 0;
	int read_len = 1;
	int len_byte = 18;
	unsigned char pagenum = (unsigned char)pg & 0x03;

	last_result_byte = RESULT_FAIL_NONE;
	/*
	?<Start, device address write>
	?TX: Write Memory Command
	?TX: Length (SMBus) [always 33]
	?TX: Parameter
	?TX: Data
	?<Stop>
	?<Delay>
	?<Start, device address read>
	?RX: Length (SMBus) [always 1]
	?RX: Result byte
	?<Stop>
	*/

	write_buf[write_len++] = len_byte;
	write_buf[write_len++] = CMD_WRITE_MEM;
	write_buf[write_len++] = pagenum;
	memcpy(&write_buf[write_len], data, 16);
	write_len += 16;

	if (DS28E16_standard_cmd_flow(write_buf, DELAY_DS28E16_EE_WRITE,
				      read_buf, &read_len, write_len)) {
		if (read_len == 1) {
			last_result_byte = read_buf[0];
			if (read_buf[0] == RESULT_SUCCESS) {
				if (pagenum == 0x00) {
					flag_mi_page0_data = 0;
					memset(mi_page0_data, 0x00, 16);
				}
				if (pagenum == 0x01) {
					flag_mi_page1_data = 0;
					memset(mi_page1_data, 0x00, 16);
				}
				if (pagenum == 0x02) {
					flag_mi_counter = 0;
					memset(mi_counter, 0x00, 16);
				}
				return DS_TRUE;
			}
		}
	}

	ow_reset();
	return DS_FALSE;
}

/*
 * 'Compute and Read Page Authentication' command.
 *
 * @anon selects the anonymous variant, @pg is the page (0, 1 or 2) and
 * @challenge is the 32-byte input.  The 32-byte MAC is returned in @hmac.
 */
static int DS28E16_cmd_computeReadPageAuthentication(int anon, int pg,
						     unsigned char *challenge,
						     unsigned char *hmac)
{
	unsigned char write_buf[255];
	unsigned char read_buf[255];
	int write_len = 0;
	int read_len = 33;
	int len_byte = 35;

	last_result_byte = RESULT_FAIL_NONE;

	write_buf[write_len++] = len_byte;
	write_buf[write_len++] = CMD_COMP_READ_AUTH;
	write_buf[write_len] = pg & 0x03;
	if (anon)
		write_buf[write_len] = (write_buf[write_len] | 0xE0);

	write_len++;
	write_buf[write_len++] = 0x02;
	memcpy(&write_buf[write_len], challenge, 32);
	write_len += 32;

	if (DS28E16_standard_cmd_flow(write_buf, DELAY_DS28E16_EE_WRITE,
				      read_buf, &read_len, write_len)) {
		last_result_byte = read_buf[0];
		if (read_buf[0] == RESULT_SUCCESS) {
			memcpy(hmac, &read_buf[1], 32);
			return DS_TRUE;
		}
	}

	ow_reset();
	return DS_FALSE;
}

/*
 * 'Compute Secret S' command: derive the session secret from the 32-byte
 * seed in @partial.  @anon and @bdconst select the anonymous and bind
 * variants, @pg is the page.
 */
static int DS28E16_cmd_computeS_Secret(int anon, int bdconst, int pg,
				       unsigned char *partial)
{
	unsigned char write_buf[40];
	unsigned char read_buf[40];
	int write_len = 0;
	int read_len = 1;
	int len_byte = 35;
	int param = pg & 0x03;

	last_result_byte = RESULT_FAIL_NONE;

	write_buf[write_len++] = len_byte;
	write_buf[write_len++] = CMD_COMP_S_SECRET;

	if (bdconst)
		param = param | 0x04;
	if (anon)
		param = param | 0xE0;

	write_buf[write_len] = param;
	write_len++;
	write_buf[write_len++] = 0x08;
	memcpy(&write_buf[write_len], partial, 32);
	write_len += 32;

	if (DS28E16_standard_cmd_flow(write_buf, DELAY_DS28E16_EE_WRITE,
				      read_buf, &read_len, write_len)) {
		last_result_byte = read_buf[0];
		if (read_buf[0] == RESULT_SUCCESS)
			return DS_TRUE;
	}

	ow_reset();
	return DS_FALSE;
}

static int AuthenticateDS28E16(int anon, int bdconst, int S_Secret_PageNum,
			       int PageNum, unsigned char *Challenge,
			       unsigned char *Secret_Seeds,
			       unsigned char *S_Secret)
{
	unsigned char PageData[32], MAC_Read_Value[32], CAL_MAC[32];
	unsigned char status_chip[16];
	unsigned char MAC_Computer_Datainput[128];
	int i = 0;
	int msg_len = 0;
	unsigned char flag = DS_FALSE;

	if (mi_auth_result == DS_TRUE)
		return mi_auth_result;
	{
		if (ds28el16_Read_RomID_retry(mi_romid) != DS_TRUE) {
			ow_reset();
			return ERROR_R_ROMID;
		}

		if (ds28el16_get_page_status_retry(status_chip) == DS_TRUE) {
			MANID[0] = status_chip[4];
		} else {
			ow_reset();
			return ERROR_R_STATUS;
		}
	}

	/* let the chip derive its session secret */
	flag = DS28E16_cmd_computeS_Secret_retry(
		anon, bdconst, S_Secret_PageNum, Secret_Seeds);
	if (flag == DS_FALSE) {
		ds_err("DS28E16_cmd_computeS_Secret error");
		ow_reset();
		return ERROR_S_SECRET;
	}

	/* then let it compute the page MAC over that session secret */
	flag = DS28E16_cmd_computeReadPageAuthentication_retry(
		anon, PageNum, Challenge, MAC_Read_Value);
	if (flag == DS_FALSE) {
		ds_err("DS28E16_cmd_computeReadPageAuthentication error");
		ow_reset();
		return ERROR_COMPUTE_MAC;
	}

	/* read back the page that the MAC covers */
	flag = ds28el16_get_page_data_retry(PageNum, PageData);
	if (flag != DS_TRUE) {
		ds_err("DS28E16_cmd_readMemory error");
		ow_reset();
		return ERROR_R_PAGEDATA;
	}

	/* ROM ID, or all-ones for the anonymous variant */
	if (anon != ANONYMOUS)
		memcpy(MAC_Computer_Datainput, mi_romid, 8);
	else
		memset(MAC_Computer_Datainput, 0xff, 8);
	msg_len += 8;

	/* page data padded with sixteen zero bytes */
	memcpy(&MAC_Computer_Datainput[msg_len], PageData, 16);
	msg_len += 16;
	memset(&MAC_Computer_Datainput[msg_len], 0x00, 16);
	msg_len += 16;

	/* the challenge itself */
	memcpy(&MAC_Computer_Datainput[msg_len], Challenge, 32);
	msg_len += 32;

	/* bind-data page number is stored with its high bits masked off */
	MAC_Computer_Datainput[msg_len] = PageNum & 0x03;
	MAC_Computer_Datainput[msg_len] = MAC_Computer_Datainput[msg_len] &
					  0x03;
	msg_len += 1;

	/* manufacturer ID */
	memcpy(&MAC_Computer_Datainput[msg_len], MANID, 2);
	msg_len += 2;

	/* calculate the MAC */
	if (ds28e16_compute_hmac(S_secret, sizeof(S_secret),
				 MAC_Computer_Datainput, msg_len, CAL_MAC)) {
		mi_auth_result = ERROR_COMPUTE_MAC;
		return ERROR_COMPUTE_MAC;
	}

	for (i = 0; i < 32; i++) {
		if (CAL_MAC[i] != MAC_Read_Value[i])
			break;
	}

	if (i != 32) {
		flag_mi_auth_result = 1;
		mi_auth_result = ERROR_UNMATCH_MAC;
		flag_mi_page1_data = 0;
		return ERROR_UNMATCH_MAC;
	} else {
		flag_mi_auth_result = 1;
		mi_auth_result = DS_TRUE;
		return DS_TRUE;
	}
}

/*
 * A failed transaction costs a 400 ms recovery plus retries, which stalls
 * boot when no chip answers, so gate on a ~100 us presence check first.
 */
static inline int ds28e16_has_presence(void)
{
	return ow_reset() == 0;
}

static int ds28el16_Read_RomID_retry(unsigned char *RomID)
{
	int i;

	if (!ds28e16_has_presence())
		return DS_FALSE;

	for (i = 0; (i < 5) && ((mi_romid[0] == 0x1f) || (mi_romid[0] == 0x00));
	     i++) {
		Virtual_cmd_readStatus();
		Read_RomID(RomID);
		Delay_us(100);
	}

	for (i = 0; i < GET_ROM_ID_RETRY; i++) {
		if (Read_RomID(RomID) == DS_TRUE) {
			return DS_TRUE;
		}
	}
	return DS_FALSE;
}

static int ds28el16_get_page_status_retry(unsigned char *data)
{
	int i;

	if (!ds28e16_has_presence())
		return DS_FALSE;

	for (i = 0; i < GET_BLOCK_STATUS_RETRY; i++) {
		if (DS28E16_cmd_readStatus(data) == DS_TRUE)
			return DS_TRUE;
	}

	return DS_FALSE;
}

static int ds28el16_get_page_data_retry(int page, unsigned char *data)
{
	int i;

	if (page >= MAX_PAGENUM)
		return DS_FALSE;

	if (!ds28e16_has_presence())
		return DS_FALSE;

	for (i = 0; i < GET_USER_MEMORY_RETRY; i++) {
		if (DS28E16_cmd_readMemory(page, data) == DS_TRUE)
			return DS_TRUE;
	}

	return DS_FALSE;
}

static int DS28E16_cmd_computeS_Secret_retry(int anon, int bdconst, int pg,
					     unsigned char *partial)
{
	int i;

	if (pg >= MAX_PAGENUM)
		return DS_FALSE;

	if (!ds28e16_has_presence())
		return DS_FALSE;

	for (i = 0; i < GET_S_SECRET_RETRY; i++) {
		if (DS28E16_cmd_computeS_Secret(anon, bdconst, pg, partial) ==
		    DS_TRUE)
			return DS_TRUE;
	}

	return DS_FALSE;
}

static int DS28E16_cmd_computeReadPageAuthentication_retry(
	int anon, int pg, unsigned char *challenge, unsigned char *hmac)
{
	int i;

	if (pg >= MAX_PAGENUM)
		return DS_FALSE;

	if (!ds28e16_has_presence())
		return DS_FALSE;

	for (i = 0; i < GET_MAC_RETRY; i++) {
		if (DS28E16_cmd_computeReadPageAuthentication(
			    anon, pg, challenge, hmac) == DS_TRUE)
			return DS_TRUE;
	}

	return DS_FALSE;
}


/* All power supply functions here */

static enum power_supply_property verify_props[] = {
	POWER_SUPPLY_PROP_ROMID,
	POWER_SUPPLY_PROP_DS_STATUS,
	POWER_SUPPLY_PROP_PAGENUMBER,
	POWER_SUPPLY_PROP_PAGEDATA,
	POWER_SUPPLY_PROP_AUTHEN_RESULT,
	POWER_SUPPLY_PROP_SESSION_SEED,
	POWER_SUPPLY_PROP_S_SECRET,
	POWER_SUPPLY_PROP_CHALLENGE,
	POWER_SUPPLY_PROP_AUTH_ANON,
	POWER_SUPPLY_PROP_AUTH_BDCONST,
	POWER_SUPPLY_PROP_PAGE0_DATA,
	POWER_SUPPLY_PROP_PAGE1_DATA,
	POWER_SUPPLY_PROP_VERIFY_MODEL_NAME,
	POWER_SUPPLY_PROP_CHIP_OK,
};

static int verify_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	int ret;
	unsigned char buf[50];
	struct ds28e16_data *data = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VERIFY_MODEL_NAME:
		ret = ds28el16_Read_RomID_retry(mi_romid);
		if (ret == DS_TRUE)
			val->strval = "ds28e16";
		else
			val->strval = "unknown";
		break;
	case POWER_SUPPLY_PROP_AUTHEN_RESULT:
		if (!data->batt_verified) {
			ret = AuthenticateDS28E16(auth_ANON, auth_BDCONST, 0,
						  pagenumber, challenge,
						  session_seed, S_secret);
			if (ret == DS_TRUE) {
				val->intval = true;
				data->batt_verified = true;
			} else {
				val->intval = false;
				data->batt_verified = false;
			}
		} else {
			val->intval = true;
		}
		break;
	case POWER_SUPPLY_PROP_PAGENUMBER:
		val->intval = pagenumber;
		break;
	case POWER_SUPPLY_PROP_ROMID:
		ret = ds28el16_Read_RomID_retry(mi_romid);
		memcpy(val->arrayval, mi_romid, 8);
		if (ret != DS_TRUE)
			return -EAGAIN;
		break;
	case POWER_SUPPLY_PROP_CHIP_OK:
		ret = ds28el16_Read_RomID_retry(mi_romid);
		if ((mi_romid[0] == FAMILY_CODE) &&
		    (mi_romid[6] == CUSTOM_ID_MSB) &&
		    ((mi_romid[5] & 0xf0) == CUSTOM_ID_LSB))
			val->intval = true;
		else
			val->intval = false;
		break;
	case POWER_SUPPLY_PROP_DS_STATUS:
		ret = ds28el16_get_page_status_retry(buf);
		memcpy(val->arrayval, buf, 8);
		if (ret != DS_TRUE)
			return -EAGAIN;
		break;
	case POWER_SUPPLY_PROP_PAGEDATA:
		ret = ds28el16_get_page_data_retry(pagenumber, buf);
		memcpy(val->arrayval, buf, 16);
		if (ret != DS_TRUE)
			return -EAGAIN;
		break;
	case POWER_SUPPLY_PROP_PAGE0_DATA:
		ret = ds28el16_get_page_data_retry(0, buf);
		memcpy(val->arrayval, buf, 16);
		if (ret != DS_TRUE)
			return -EAGAIN;
		break;
	case POWER_SUPPLY_PROP_PAGE1_DATA:
		ret = ds28el16_get_page_data_retry(1, buf);
		memcpy(val->arrayval, buf, 16);
		if (ret != DS_TRUE)
			return -EAGAIN;
		break;
	/* Raw auth-material accessors (props 84-88). Downstream had no GET
	 * cases for these, so every psy uevent/poll logged "unsupported
	 * property" and factory tooling could only use the ds_* sysfs
	 * files. They mirror the same globals the sysfs stores write. */
	case POWER_SUPPLY_PROP_SESSION_SEED:
		memcpy(val->arrayval, session_seed, 32);
		break;
	case POWER_SUPPLY_PROP_S_SECRET:
		memcpy(val->arrayval, S_secret, 32);
		break;
	case POWER_SUPPLY_PROP_CHALLENGE:
		memcpy(val->arrayval, challenge, 32);
		break;
	case POWER_SUPPLY_PROP_AUTH_ANON:
		val->intval = auth_ANON;
		break;
	case POWER_SUPPLY_PROP_AUTH_BDCONST:
		val->intval = auth_BDCONST;
		break;
	default:
		ds_err("unsupported property %d\n", psp);
		return -ENODATA;
	}

	return 0;
}

static int verify_set_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_PAGENUMBER:
		pagenumber = val->intval;
		break;
	case POWER_SUPPLY_PROP_AUTH_ANON:
		auth_ANON = val->intval;
		break;
	case POWER_SUPPLY_PROP_AUTH_BDCONST:
		auth_BDCONST = val->intval;
		break;
	default:
		ds_err("unsupported property %d\n", prop);
		return -ENODATA;
	}

	return 0;
}

static int verify_prop_is_writeable(struct power_supply *psy,
				    enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_PAGENUMBER:
	case POWER_SUPPLY_PROP_PAGEDATA:
	case POWER_SUPPLY_PROP_SESSION_SEED:
	case POWER_SUPPLY_PROP_S_SECRET:
	case POWER_SUPPLY_PROP_CHALLENGE:
	case POWER_SUPPLY_PROP_AUTH_ANON:
	case POWER_SUPPLY_PROP_AUTH_BDCONST:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int verify_psy_register(struct ds28e16_data *ds)
{
	struct power_supply_config verify_psy_cfg = {};

	ds->verify_psy_d.name = "batt_verify";
	ds->verify_psy_d.type = POWER_SUPPLY_TYPE_BATT_VERIFY;
	ds->verify_psy_d.properties = verify_props;
	ds->verify_psy_d.num_properties = ARRAY_SIZE(verify_props);
	ds->verify_psy_d.get_property = verify_get_property;
	ds->verify_psy_d.set_property = verify_set_property;
	ds->verify_psy_d.property_is_writeable = verify_prop_is_writeable;

	verify_psy_cfg.drv_data = ds;
	/* Mainline struct power_supply_config has no of_node (only fwnode);
	 * the psy is matched by name ("batt_verify") like downstream. */
	verify_psy_cfg.num_supplicants = 0;
	ds->verify_psy = devm_power_supply_register(ds->dev, &ds->verify_psy_d,
						    &verify_psy_cfg);
	if (IS_ERR(ds->verify_psy)) {
		ds_err("Failed to register verify_psy");
		return PTR_ERR(ds->verify_psy);
	}

	return 0;
}

	/* read the key material from the file */
static ssize_t ds28e16_ds_Auth_Result_status_read(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	int result;

	result = AuthenticateDS28E16(auth_ANON, auth_BDCONST, 0, pagenumber,
				     challenge, session_seed, S_secret);
	if (result == ERROR_R_STATUS)
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed : ERROR_R_STATUS!\n");
	else if (result == ERROR_UNMATCH_MAC)
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed : MAC is not match!\n");
	else if (result == ERROR_R_ROMID)
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed : ERROR_R_ROMID!\n");
	else if (result == ERROR_COMPUTE_MAC)
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed : ERROR_COMPUTE_MAC!\n");
	else if (result == ERROR_S_SECRET)
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed : ERROR_S_SECRET!\n");
	else if (result == DS_TRUE)
		return scnprintf(buf, PAGE_SIZE, "Authenticate success!!!\n");
	else
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed : other reason.\n");
}

static ssize_t ds28e16_ds_romid_status_read(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	short status;
	unsigned char RomID[10] = { 0x00 };
	int i = 0;
	int count = 0;

	for (i = 0; i < attr_trytimes; i++) {
		status = ds28el16_Read_RomID_retry(RomID);

		if (status == DS_TRUE) {
			count++;
		} else {
		}
		Delay_us(1000);
	}
	return scnprintf(
		buf, PAGE_SIZE,
		"Success = %d\nRomID = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
		count, RomID[0], RomID[1], RomID[2], RomID[3], RomID[4],
		RomID[5], RomID[6], RomID[7]);
}

static ssize_t ds28e16_ds_pagenumber_status_read(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%02x\n", pagenumber);
}

static ssize_t ds28e16_ds_pagenumber_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int buf_int;

	if (sscanf(buf, "%d", &buf_int) != 1)
		return -EINVAL;

	if ((buf_int >= 0) && (buf_int <= 3))
		pagenumber = buf_int;

	return count;
}

static ssize_t ds28e16_ds_pagedata_status_read(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	int result;
	unsigned char pagedata[16] = { 0x00 };
	int i = 0;
	int count = 0;

	for (i = 0; i < attr_trytimes; i++) {
		result = ds28el16_get_page_data_retry(pagenumber, pagedata);

		if (result == DS_TRUE) {
			count++;
		} else {
		}
		Delay_us(1000);
	}
	return scnprintf(
		buf, PAGE_SIZE,
		"Success = %d\npagedata = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
		count, pagedata[0], pagedata[1], pagedata[2], pagedata[3],
		pagedata[4], pagedata[5], pagedata[6], pagedata[7], pagedata[8],
		pagedata[9], pagedata[10], pagedata[11], pagedata[12],
		pagedata[13], pagedata[14], pagedata[15]);
}

static ssize_t ds28e16_ds_pagedata_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int result;
	unsigned char pagedata[16] = { 0x00 };

	if (sscanf(buf,
		   "%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx",
		   &pagedata[0], &pagedata[1], &pagedata[2], &pagedata[3],
		   &pagedata[4], &pagedata[5], &pagedata[6], &pagedata[7],
		   &pagedata[8], &pagedata[9], &pagedata[10], &pagedata[11],
		   &pagedata[12], &pagedata[13], &pagedata[14],
		   &pagedata[15]) != 16)
		return -EINVAL;

	result = DS28E16_cmd_writeMemory(pagenumber, pagedata);
	if (result != DS_TRUE)
		return -EIO;

	return count;
}

static ssize_t ds28e16_ds_time_status_read(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", attr_trytimes);
}

static ssize_t ds28e16_ds_time_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int buf_int;

	if (sscanf(buf, "%d", &buf_int) != 1)
		return -EINVAL;

	if (buf_int > 0)
		attr_trytimes = buf_int;

	return count;
}

static ssize_t
ds28e16_ds_session_seed_status_read(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return scnprintf(
		buf, PAGE_SIZE,
		"%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n",
		session_seed[0], session_seed[1], session_seed[2],
		session_seed[3], session_seed[4], session_seed[5],
		session_seed[6], session_seed[7], session_seed[8],
		session_seed[9], session_seed[10], session_seed[11],
		session_seed[12], session_seed[13], session_seed[14],
		session_seed[15], session_seed[16], session_seed[17],
		session_seed[18], session_seed[19], session_seed[20],
		session_seed[21], session_seed[22], session_seed[23],
		session_seed[24], session_seed[25], session_seed[26],
		session_seed[27], session_seed[28], session_seed[29],
		session_seed[30], session_seed[31]);
}

static ssize_t ds28e16_ds_session_seed_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	if (sscanf(buf,
		   "%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx",
		   &session_seed[0], &session_seed[1], &session_seed[2],
		   &session_seed[3], &session_seed[4], &session_seed[5],
		   &session_seed[6], &session_seed[7], &session_seed[8],
		   &session_seed[9], &session_seed[10], &session_seed[11],
		   &session_seed[12], &session_seed[13], &session_seed[14],
		   &session_seed[15], &session_seed[16], &session_seed[17],
		   &session_seed[18], &session_seed[19], &session_seed[20],
		   &session_seed[21], &session_seed[22], &session_seed[23],
		   &session_seed[24], &session_seed[25], &session_seed[26],
		   &session_seed[27], &session_seed[28], &session_seed[29],
		   &session_seed[30], &session_seed[31]) != 32)
		return -EINVAL;

	return count;
}

static ssize_t ds28e16_ds_challenge_status_read(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return scnprintf(
		buf, PAGE_SIZE,
		"%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n",
		challenge[0], challenge[1], challenge[2], challenge[3],
		challenge[4], challenge[5], challenge[6], challenge[7],
		challenge[8], challenge[9], challenge[10], challenge[11],
		challenge[12], challenge[13], challenge[14], challenge[15],
		challenge[16], challenge[17], challenge[18], challenge[19],
		challenge[20], challenge[21], challenge[22], challenge[23],
		challenge[24], challenge[25], challenge[26], challenge[27],
		challenge[28], challenge[29], challenge[30], challenge[31]);
}

static ssize_t ds28e16_ds_challenge_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	if (sscanf(buf,
		   "%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx",
		   &challenge[0], &challenge[1], &challenge[2], &challenge[3],
		   &challenge[4], &challenge[5], &challenge[6], &challenge[7],
		   &challenge[8], &challenge[9], &challenge[10], &challenge[11],
		   &challenge[12], &challenge[13], &challenge[14],
		   &challenge[15], &challenge[16], &challenge[17],
		   &challenge[18], &challenge[19], &challenge[20],
		   &challenge[21], &challenge[22], &challenge[23],
		   &challenge[24], &challenge[25], &challenge[26],
		   &challenge[27], &challenge[28], &challenge[29],
		   &challenge[30], &challenge[31]) != 32)
		return -EINVAL;

	return count;
}

static ssize_t ds28e16_ds_S_secret_status_read(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	return scnprintf(
		buf, PAGE_SIZE,
		"%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n",
		S_secret[0], S_secret[1], S_secret[2], S_secret[3], S_secret[4],
		S_secret[5], S_secret[6], S_secret[7], S_secret[8], S_secret[9],
		S_secret[10], S_secret[11], S_secret[12], S_secret[13],
		S_secret[14], S_secret[15], S_secret[16], S_secret[17],
		S_secret[18], S_secret[19], S_secret[20], S_secret[21],
		S_secret[22], S_secret[23], S_secret[24], S_secret[25],
		S_secret[26], S_secret[27], S_secret[28], S_secret[29],
		S_secret[30], S_secret[31]);
}

static ssize_t ds28e16_ds_S_secret_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	if (sscanf(buf,
		   "%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx",
		   &S_secret[0], &S_secret[1], &S_secret[2], &S_secret[3],
		   &S_secret[4], &S_secret[5], &S_secret[6], &S_secret[7],
		   &S_secret[8], &S_secret[9], &S_secret[10], &S_secret[11],
		   &S_secret[12], &S_secret[13], &S_secret[14], &S_secret[15],
		   &S_secret[16], &S_secret[17], &S_secret[18], &S_secret[19],
		   &S_secret[20], &S_secret[21], &S_secret[22], &S_secret[23],
		   &S_secret[24], &S_secret[25], &S_secret[26], &S_secret[27],
		   &S_secret[28], &S_secret[29], &S_secret[30],
		   &S_secret[31]) != 32)
		return -EINVAL;

	return count;
}

static ssize_t ds28e16_ds_auth_ANON_status_read(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%02x\n", auth_ANON);
}

static ssize_t ds28e16_ds_auth_ANON_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	if (sscanf(buf, "%d", &auth_ANON) != 1)
		return -EINVAL;

	return count;
}

static ssize_t
ds28e16_ds_auth_BDCONST_status_read(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%02x\n", auth_BDCONST);
}

static ssize_t ds28e16_ds_auth_BDCONST_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	if (sscanf(buf, "%d", &auth_BDCONST) != 1)
		return -EINVAL;

	return count;
}

static ssize_t ds28e16_ds_readstatus_status_read(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	int result;
	unsigned char status[16] = { 0x00 };
	int i = 0;
	int count = 0;

	for (i = 0; i < attr_trytimes; i++) {
		result = ds28el16_get_page_status_retry(status);

		if (result == DS_TRUE) {
			count++;
		} else {
		}
		Delay_us(1000);
	}
	return scnprintf(
		buf, PAGE_SIZE,
		"Success = %d\nStatus = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
		count, status[0], status[1], status[2], status[3], status[4],
		status[5], status[6], status[7], status[8], status[9],
		status[10], status[11], status[12], status[13], status[14],
		status[15]);
}

static DEVICE_ATTR(ds_readstatus, S_IRUGO, ds28e16_ds_readstatus_status_read,
		   NULL);
static DEVICE_ATTR(ds_romid, S_IRUGO, ds28e16_ds_romid_status_read, NULL);
static DEVICE_ATTR(ds_pagenumber, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e16_ds_pagenumber_status_read,
		   ds28e16_ds_pagenumber_store);
static DEVICE_ATTR(ds_pagedata, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e16_ds_pagedata_status_read, ds28e16_ds_pagedata_store);
static DEVICE_ATTR(ds_time, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e16_ds_time_status_read, ds28e16_ds_time_store);
static DEVICE_ATTR(ds_session_seed, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e16_ds_session_seed_status_read,
		   ds28e16_ds_session_seed_store);
static DEVICE_ATTR(ds_challenge, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e16_ds_challenge_status_read,
		   ds28e16_ds_challenge_store);
static DEVICE_ATTR(ds_S_secret, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e16_ds_S_secret_status_read, ds28e16_ds_S_secret_store);
static DEVICE_ATTR(ds_auth_ANON, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e16_ds_auth_ANON_status_read,
		   ds28e16_ds_auth_ANON_store);
static DEVICE_ATTR(ds_auth_BDCONST, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e16_ds_auth_BDCONST_status_read,
		   ds28e16_ds_auth_BDCONST_store);
static DEVICE_ATTR(ds_Auth_Result, S_IRUGO, ds28e16_ds_Auth_Result_status_read,
		   NULL);

static struct attribute *ds_attributes[] = {
	&dev_attr_ds_readstatus.attr,  &dev_attr_ds_romid.attr,
	&dev_attr_ds_pagenumber.attr,  &dev_attr_ds_pagedata.attr,
	&dev_attr_ds_time.attr,	       &dev_attr_ds_session_seed.attr,
	&dev_attr_ds_challenge.attr,   &dev_attr_ds_S_secret.attr,
	&dev_attr_ds_auth_ANON.attr,   &dev_attr_ds_auth_BDCONST.attr,
	&dev_attr_ds_Auth_Result.attr, NULL,
};

static const struct attribute_group ds_attr_group = {
	.attrs = ds_attributes,
};

static int authentic_period_ms = 5000;
#define AUTHENTIC_COUNT_MAX 5
static int retry_authentic;
static void authentic_work(struct work_struct *work)
{
	struct ds28e16_data *ds =
		container_of(work, struct ds28e16_data, authentic_work.work);
	union power_supply_propval pval = {};
	int ret;

	ret = power_supply_get_property(ds->verify_psy,
					POWER_SUPPLY_PROP_AUTHEN_RESULT, &pval);
	if (ret)
		return;

	if (pval.intval == true) {
		ds->batt_verified = true;
		return;
	}

	ds->batt_verified = false;
	if (++retry_authentic < AUTHENTIC_COUNT_MAX)
		schedule_delayed_work(&ds->authentic_work,
				      msecs_to_jiffies(authentic_period_ms));
}

static int ds28e16_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ds28e16_data *ds;
	int ret;

	/*
	 * The authenticator is only reachable once the 1-Wire controller has
	 * probed, which is an independent platform device with no phandle
	 * linking the two.
	 */
	if (!onewire_gpio_is_ready())
		return -EPROBE_DEFER;

	ds = devm_kzalloc(dev, sizeof(*ds), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

	ds->dev = dev;
	ds->pdev = pdev;
	platform_set_drvdata(pdev, ds);
	INIT_DELAYED_WORK(&ds->authentic_work, authentic_work);

	ret = verify_psy_register(ds);
	if (ret)
		return ret;

	ret = sysfs_create_group(&dev->kobj, &ds_attr_group);
	if (ret) {
		dev_err(dev, "failed to create sysfs attributes: %d\n", ret);
		return ret;
	}

	/* Give the 1-Wire bus time to settle before the first authentication. */
	schedule_delayed_work(&ds->authentic_work, msecs_to_jiffies(500));

	return 0;
}

static void ds28e16_remove(struct platform_device *pdev)
{
	struct ds28e16_data *ds = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&ds->authentic_work);
	sysfs_remove_group(&ds->dev->kobj, &ds_attr_group);
}

static const struct of_device_id ds28e16_dt_match[] = {
	{ .compatible = "maxim,ds28e16" },
	{}
};
MODULE_DEVICE_TABLE(of, ds28e16_dt_match);

static struct platform_driver ds28e16_driver = {
	.driver = {
		.name = "maxim_ds28e16",
		.of_match_table = ds28e16_dt_match,
	},
	.probe = ds28e16_probe,
	.remove = ds28e16_remove,
};
module_platform_driver(ds28e16_driver);

MODULE_AUTHOR("Xiaomi Inc.");
MODULE_DESCRIPTION("Maxim DS28E16 battery authenticator");
MODULE_LICENSE("GPL");
