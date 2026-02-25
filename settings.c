#include "mcudef.h"

#include <string.h>

#include "irc.h"
#include "util.h"

#include "settings.h"

struct settings St;

int writesettings(int slot)
{
	struct settings s[6];
	uint32_t sz;
	char *pt;
	int j;

	// read all settings slots from MCU's flash to temporary storage
	memcpy(s, (void *) (USER_FLASH), sizeof(struct settings) * 6);

	// disable interrupts and unlock MCU's flash
	__disable_irq();
	HAL_FLASH_Unlock();

	// erase MCU's flash sector containing settings
#ifdef STM32F4xx
	FLASH_Erase_Sector(FLASH_SECTOR_11,  VOLTAGE_RANGE_3);
#elif STM32H7xx
	FLASH_Erase_Sector(FLASH_SECTOR_7, FLASH_BANK_1, FLASH_VOLTAGE_RANGE_3);
#endif

	// copy current running settings to slot in temporary storage
	memcpy(s + slot, &St, sizeof(struct settings));

	// write temporary storage back to flash
	sz = sizeof(struct settings) * 6;
	pt = (char *) s;

#ifdef STM32F4xx
	for (j = 0; j < sz; j += 4) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
			USER_FLASH + j, *((uint32_t *) (pt + j)));
	}
#elif STM32H7xx
	for (j = 0; j < sz; j += 32) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
			USER_FLASH + j, (uint32_t) (pt + j));
	}
#endif

	// lock MCU's flash and enable interrupts
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
