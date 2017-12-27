/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 *
 */
#include <asm/bootinfo.h>

#include <loongson.h>

void __init mach_prom_init_machtype(void)
{
	/* We share the same kernel image file among Lemote 2F family
	 * of machines, and provide the machtype= kernel command line
	 * to users to indicate their machine, this command line will
	 * be passed by the latest PMON automatically. and fortunately,
	 * up to now, we can get the machine type from the PMON_VER=
	 * commandline directly except the NAS machine, In the old
	 * machines, this will help the users a lot.
	 *
	 * If no "machtype=" passed, get machine type from "PMON_VER=".
	 *	PMON_VER=LM8089		Lemote 8.9'' netbook
	 *		 LM8101		Lemote 10.1'' netbook
	 *	(The above two netbooks have the same kernel support)
	 *		 LM6XXX		Lemote FuLoong(2F) box series
	 *		 LM9XXX		Lemote LynLoong PC series
	 */
	if (strstr(mips_cmdline, "PMON_VER=LM")) {
		if (strstr(mips_cmdline, "PMON_VER=LM8"))
			mips_machtype = MACH_LEMOTE_YL2F89;
		else if (strstr(mips_cmdline, "PMON_VER=LM6"))
			mips_machtype = MACH_LEMOTE_FL2F;
		else if (strstr(mips_cmdline, "PMON_VER=LM9"))
			mips_machtype = MACH_LEMOTE_LL2F;
		else
			mips_machtype = MACH_LEMOTE_NAS;

		strcat(mips_cmdline, " machtype=");
		strcat(mips_cmdline, get_system_type());
		strcat(mips_cmdline, " ");
	}
}
