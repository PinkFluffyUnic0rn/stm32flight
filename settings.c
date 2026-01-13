#include "stm32f4xx_hal.h"
#include <string.h>

#include "irc.h"
#include "util.h"

#include "settings.h"

struct settings St;

int writesettings(int slot)
{
	struct settings s[6];
	uint32_t sz;
	uint32_t *pt;
	uint32_t addr;
	int j;

	memcpy(s, (void *) (USER_FLASH), sizeof(struct settings) * 6);

	__disable_irq();
	HAL_FLASH_Unlock();

	FLASH_Erase_Sector(FLASH_SECTOR_11,  VOLTAGE_RANGE_3);

	memcpy(s + slot, &St, sizeof(struct settings));

	sz = sizeof(struct settings) * 6;
	pt = (uint32_t *) s;

	addr = USER_FLASH;
	for (j = 0; j < sz / 4; ++j) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, pt[j]);
		addr += 4;
	}

	HAL_FLASH_Lock();
	__enable_irq();

	return 0;
}

int validatesettings()
{
	int i;

	if (!irc_ispowervalid(St.irc.power))
		St.irc.power = IRCDEFPOWER;
	if (!irc_isfreqvalid(St.irc.power))
		St.irc.freq = IRCDEFFREQ;

	if (St.mtr.lt < 0 || St.mtr.lt > 3)	St.mtr.lt = LTDEFNUM;
	if (St.mtr.lb < 0 || St.mtr.lb > 3)	St.mtr.lb = LBDEFNUM;
	if (St.mtr.rb < 0 || St.mtr.rb > 3)	St.mtr.rb = RBDEFNUM;
	if (St.mtr.rt < 0 || St.mtr.rt > 3)	St.mtr.rt = RTDEFNUM;

	if (St.log.freq <= 0 || St.log.freq > LOG_MAXFREQ)
		St.log.freq = LOGDEFFREQ;

	if (St.log.recsize < 0
			|| St.log.recsize > LOG_MAXRECSIZE
			|| !ispow2(St.log.recsize)) {
		St.log.recsize = LOGDEFRECSIZE;
	}

	for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
		if (St.log.fieldid[i] < 0)
			St.log.fieldid[i] = LOGFIELDDEFPOS;
	}

	return 0;
}

int readsettings(int slot)
{
	memcpy(&St, (void *) (USER_FLASH
			+ slot * sizeof(struct settings)),
		sizeof(struct settings));

	validatesettings();

	return 0;
}
