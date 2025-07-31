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

	if (!irc_ispowervalid(st.ircpower))
		st.ircpower = IRCDEFPOWER;
	if (!irc_isfreqvalid(st.ircpower))
		st.ircfreq = IRCDEFFREQ;

	if (st.lt < 0 || st.lt > 3)	st.lt = LTDEFNUM;
	if (st.lb < 0 || st.lb > 3)	st.lb = LBDEFNUM;
	if (st.rb < 0 || st.rb > 3)	st.rb = RBDEFNUM;
	if (st.rt < 0 || st.rt > 3)	st.rt = RTDEFNUM;

	if (st.logfreq <= 0 || st.logfreq > LOG_MAXFREQ)
		st.logfreq = LOGDEFFREQ;

	if (st.logrecsize < 0
			|| st.logrecsize > LOG_MAXRECSIZE
			|| !ispow2(st.logrecsize)) {
		st.logrecsize = LOGDEFRECSIZE;
	}

	for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
		if (st.fieldid[i] < 0)
			st.fieldid[i] = LOGFIELDDEFPOS;
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
