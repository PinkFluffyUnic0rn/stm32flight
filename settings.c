#include "stm32f4xx_hal.h"
#include <string.h>

#include "irc.h"
#include "util.h"

#include "settings.h"

struct settings st;

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

	memcpy(s + slot, &st, sizeof(struct settings));

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

	if (!irc_ispowervalid(st.irc.power))
		st.irc.power = IRCDEFPOWER;
	if (!irc_isfreqvalid(st.irc.power))
		st.irc.freq = IRCDEFFREQ;

	if (st.mtr.lt < 0 || st.mtr.lt > 3)	st.mtr.lt = LTDEFNUM;
	if (st.mtr.lb < 0 || st.mtr.lb > 3)	st.mtr.lb = LBDEFNUM;
	if (st.mtr.rb < 0 || st.mtr.rb > 3)	st.mtr.rb = RBDEFNUM;
	if (st.mtr.rt < 0 || st.mtr.rt > 3)	st.mtr.rt = RTDEFNUM;

	if (st.log.freq <= 0 || st.log.freq > LOG_MAXFREQ)
		st.log.freq = LOGDEFFREQ;

	if (st.log.recsize < 0
			|| st.log.recsize > LOG_MAXRECSIZE
			|| !ispow2(st.log.recsize)) {
		st.log.recsize = LOGDEFRECSIZE;
	}

	for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
		if (st.log.fieldid[i] < 0)
			st.log.fieldid[i] = LOGFIELDDEFPOS;
	}

	return 0;
}

int readsettings(int slot)
{
	memcpy(&st, (void *) (USER_FLASH
			+ slot * sizeof(struct settings)),
		sizeof(struct settings));

	validatesettings();

	return 0;
}
