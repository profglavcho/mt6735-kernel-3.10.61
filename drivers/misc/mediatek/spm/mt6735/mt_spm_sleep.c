#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/aee.h>
#include <linux/i2c.h>
#include <linux/of_fdt.h>
#include <asm/setup.h>

#ifndef CONFIG_ARM64
#include <mach/irqs.h>
#else
#include <linux/irqchip/mt-gic.h>
#endif
#include <mach/mt_cirq.h>
#include <mach/mt_spm_sleep.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_cpuidle.h>
#include <mach/wd_api.h>
#include <mach/eint.h>
#include <mach/mtk_ccci_helper.h>
#include <mach/mt_cpufreq.h>
#include <mach/upmu_common.h>
#ifdef CONFIG_MD32_SUPPORT
#include <mach/md32_helper.h>
#endif
#include "mach/mt_pmic_wrap.h"
#if 0
#include <cust_gpio_usage.h>
#endif
//FIXME: for K2 fpga early porting
#if 0
#include <mach/mt_dramc.h>
#endif
#include <mt_i2c.h>

#include "mt_spm_internal.h"
#include "../../pmic_wrap/mt6735/pwrap_hal.h"
#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>


/**************************************
 * only for internal debug
 **************************************/
#ifdef CONFIG_MTK_LDVT
#define SPM_PWAKE_EN            0
#define SPM_PCMWDT_EN           0
#define SPM_BYPASS_SYSPWREQ     1
#else
#define SPM_PWAKE_EN            1
#define SPM_PCMWDT_EN           1
#define SPM_BYPASS_SYSPWREQ     0
#endif
#define SPM_PCMTIMER_DIS        0

#define K2_MCUCFG_BASE          (0xF0200000)      //0x1020_0000
#define MP0_AXI_CONFIG          (K2_MCUCFG_BASE + 0x2C) 
#define MP1_AXI_CONFIG          (K2_MCUCFG_BASE + 0x22C) 
#define ACINACTM                (1<<4)

#define I2C_CHANNEL 2

int spm_dormant_sta = MT_CPU_DORMANT_RESET;
int spm_ap_mdsrc_req_cnt = 0;
u32 spm_suspend_flag = 0;

struct wake_status suspend_info[20];
u32 log_wakesta_cnt = 0;
u32 log_wakesta_index = 0;
u8 spm_snapshot_golden_setting = 0;

/**********************************************************
 * PCM code for suspend
 **********************************************************/
#if defined(CONFIG_ARCH_MT6735)
static const u32 suspend_binary[] = {
	0x81429801, 0xd80001a5, 0x17c07c1f, 0x18c0001f, 0x10001138, 0x1910001f,
	0x10001138, 0xa1108404, 0xe0c00004, 0x1a00001f, 0x10006604, 0xc0c03e00,
	0xe2200007, 0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4, 0xa0940402,
	0x1b00001f, 0x10000001, 0x18d0001f, 0x10006310, 0x80c38c01, 0xd8200403,
	0xe2000002, 0x80c10001, 0x1b00001f, 0xbf7ce7ff, 0xd8200403, 0x17c07c1f,
	0x1b00001f, 0x7f7ce7ff, 0xf0000000, 0x17c07c1f, 0x1a00001f, 0x100062c4,
	0x1890001f, 0x100062c4, 0x80b40402, 0x81429801, 0xd8000685, 0xe2000002,
	0x18c0001f, 0x10001138, 0x1910001f, 0x10001138, 0x81308404, 0xe0c00004,
	0x1a00001f, 0x10006604, 0xc0c03e00, 0xe2200007, 0x1b00001f, 0x00801001,
	0x18d0001f, 0x10006310, 0x80c40c01, 0xd8200843, 0x17c07c1f, 0x80c10001,
	0x1b00001f, 0x6ffcf7ff, 0xd8000843, 0x17c07c1f, 0x1b00001f, 0xaffce7ff,
	0xf0000000, 0x17c07c1f, 0x81f58407, 0x81f68407, 0x1800001f, 0x17cf0f3f,
	0x1b80001f, 0x20000000, 0x1800001f, 0x17cf0f16, 0xa1d28407, 0x81f20407,
	0x1a00001f, 0x10006604, 0xc0c03e00, 0xe2200004, 0xc0c03e00, 0xe2200005,
	0x81409801, 0xd8000b65, 0x17c07c1f, 0x18c0001f, 0x10006234, 0xc0c02ac0,
	0x1200041f, 0x80310400, 0x1b80001f, 0x2000000a, 0xa0110400, 0x81f00407,
	0xa1dd0407, 0x18c0001f, 0x100062d8, 0xc0c02d80, 0x1200041f, 0x81fd0407,
	0xc2803ee0, 0x1290041f, 0x1b00001f, 0x6ffcf7ff, 0xf0000000, 0x17c07c1f,
	0x1b00001f, 0x2f7ce7ff, 0x1b80001f, 0x20000004, 0xd80014ec, 0x17c07c1f,
	0x1a00001f, 0x10006604, 0xc0c03e00, 0xe2200002, 0xc0c03e00, 0xe2200003,
	0x1880001f, 0x10006320, 0xc0c030a0, 0xe080000f, 0xd8201043, 0x17c07c1f,
	0x1b00001f, 0x6ffcf7ff, 0xd00014e0, 0x17c07c1f, 0xe080001f, 0x81409801,
	0xd80012c5, 0x17c07c1f, 0x18c0001f, 0x10214094, 0x1910001f, 0x1020e374,
	0xe0c00004, 0x18c0001f, 0x10214098, 0x1910001f, 0x1020e378, 0xe0c00004,
	0x1910001f, 0x10213378, 0x18c0001f, 0x10006234, 0xc0c02ca0, 0x17c07c1f,
	0x18c0001f, 0x100062d8, 0xc0c02ea0, 0x17c07c1f, 0xc2803ee0, 0x1290841f,
	0xa1d20407, 0x81f28407, 0xa1d68407, 0x1800001f, 0x17cf0f3f, 0x1800001f,
	0x17ff0f3f, 0x19c0001f, 0x001c2397, 0x1b00001f, 0x2f7cefff, 0xf0000000,
	0x17c07c1f, 0x18c0001f, 0x102135cc, 0x1910001f, 0x102135cc, 0x813f8404,
	0x81411801, 0xd80016e5, 0xe0c00004, 0x18c0001f, 0x10006240, 0xe0e00016,
	0xe0e0001e, 0xe0e0000e, 0xe0e0000f, 0x803e0400, 0x1b80001f, 0x20000222,
	0x80380400, 0x1b80001f, 0x20000280, 0x803b0400, 0x1b80001f, 0x2000001a,
	0x803d0400, 0x1b80001f, 0x20000208, 0x80340400, 0x80310400, 0xe8208000,
	0x10210044, 0x00000100, 0x1b80001f, 0x20000068, 0x1b80001f, 0x2000000a,
	0x18c0001f, 0x10006240, 0xe0e0000d, 0xd8001d65, 0x17c07c1f, 0x1b80001f,
	0x20000020, 0x18c0001f, 0x102130f0, 0x1910001f, 0x102130f0, 0xa9000004,
	0x10000000, 0xe0c00004, 0x1b80001f, 0x2000000a, 0x89000004, 0xefffffff,
	0xe0c00004, 0x18c0001f, 0x102140f4, 0x1910001f, 0x102140f4, 0xa9000004,
	0x02000000, 0xe0c00004, 0x1b80001f, 0x2000000a, 0x89000004, 0xfdffffff,
	0xe0c00004, 0x81fa0407, 0x81f08407, 0xe8208000, 0x10006354, 0x001fdae3,
	0xa1d80407, 0xa1de8407, 0xa1df0407, 0xc2803ee0, 0x1291041f, 0x1b00001f,
	0xbf7ce7ff, 0x1950001f, 0x100062c4, 0x80c41401, 0xd8001fc3, 0x17c07c1f,
	0x1b00001f, 0xaffce7ff, 0xf0000000, 0x17c07c1f, 0x1b80001f, 0x20000fdf,
	0x1a50001f, 0x10006608, 0x80c9a401, 0x810ba401, 0x10920c1f, 0xa0979002,
	0x80ca2401, 0xa0938c02, 0x8080080d, 0xd8202442, 0x17c07c1f, 0x81f08407,
	0xa1d80407, 0xa1de8407, 0xa1df0407, 0x1b00001f, 0x2f7ce7ff, 0x1b80001f,
	0x20000004, 0xd8002a8c, 0x17c07c1f, 0x1b00001f, 0xbf7ce7ff, 0x1950001f,
	0x100062c4, 0x80c41401, 0xd8002a83, 0x17c07c1f, 0x1b00001f, 0xaffce7ff,
	0xd0002a80, 0x17c07c1f, 0x81f80407, 0x81fe8407, 0x81ff0407, 0x1880001f,
	0x10006320, 0xc0c02f40, 0xe080000f, 0xd80021a3, 0x17c07c1f, 0xe080001f,
	0xa1da0407, 0xe8208000, 0x10210048, 0x00000100, 0x1b80001f, 0x20000068,
	0xa0110400, 0xa0140400, 0xa0180400, 0xa01b0400, 0xa01d0400, 0x1b80001f,
	0x20000068, 0xa01e0400, 0x1b80001f, 0x20000104, 0x81411801, 0xd8002925,
	0x17c07c1f, 0x18c0001f, 0x10006240, 0xc0c02ea0, 0x17c07c1f, 0x18c0001f,
	0x102135cc, 0x1910001f, 0x102135cc, 0xa11f8404, 0xe0c00004, 0xc2803ee0,
	0x1291841f, 0x1b00001f, 0x6ffcf7ff, 0x1950001f, 0x100062c4, 0x80c41401,
	0xd8202a83, 0x17c07c1f, 0x1b00001f, 0x7f7ce7ff, 0xf0000000, 0x17c07c1f,
	0xe0f07f16, 0x1380201f, 0xe0f07f1e, 0x1380201f, 0xe0f07f0e, 0x1b80001f,
	0x20000100, 0xe0f07f0c, 0xe0f07f0d, 0xe0f07e0d, 0xe0f07c0d, 0xe0f0780d,
	0xe0f0700d, 0xf0000000, 0x17c07c1f, 0xe0f07f0d, 0xe0f07f1d, 0xe0f07f1f,
	0xe0f07f1e, 0xe0f07f12, 0xf0000000, 0x17c07c1f, 0xe0e00016, 0x1380201f,
	0xe0e0001e, 0x1380201f, 0xe0e0000e, 0xe0e0000c, 0xe0e0000d, 0xf0000000,
	0x17c07c1f, 0xe0e0000f, 0xe0e0001e, 0xe0e00012, 0xf0000000, 0x17c07c1f,
	0x1112841f, 0xa1d08407, 0xd8203004, 0x80eab401, 0xd8002f83, 0x01200404,
	0x1a00001f, 0x10006814, 0xe2000003, 0xf0000000, 0x17c07c1f, 0xa1d00407,
	0x1b80001f, 0x20000100, 0x80ea3401, 0x1a00001f, 0x10006814, 0xe2000003,
	0xf0000000, 0x17c07c1f, 0x1092041f, 0x81499801, 0xd8203345, 0x17c07c1f,
	0xd8203702, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0x60000000,
	0xd8003242, 0x00a00402, 0x814a1801, 0xd82034a5, 0x17c07c1f, 0xd8203702,
	0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0x80000000, 0xd80033a2,
	0x00a00402, 0x814a9801, 0xd8203605, 0x17c07c1f, 0xd8203702, 0x17c07c1f,
	0x18d0001f, 0x40000000, 0x18d0001f, 0xc0000000, 0xd8003502, 0x00a00402,
	0xd8203702, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0x40000000,
	0xd8003602, 0x00a00402, 0xf0000000, 0x17c07c1f, 0x81441801, 0xd8203965,
	0x17c07c1f, 0x1a00001f, 0x10006604, 0xc0c03e00, 0xe2200004, 0xc0c031c0,
	0x17c07c1f, 0xc0c03e00, 0xe2200003, 0xc0c031c0, 0x17c07c1f, 0xc0c03e00,
	0xe2200002, 0xc0c031c0, 0x17c07c1f, 0x1a00001f, 0x100062c4, 0x1890001f,
	0x100062c4, 0xa0908402, 0xe2000002, 0x1b00001f, 0x00801001, 0xf0000000,
	0x17c07c1f, 0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4, 0x80b08402,
	0xe2000002, 0x81441801, 0xd8203d85, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0xc0c03e00, 0xe2200003, 0xc0c031c0, 0x17c07c1f, 0xc0c03e00, 0xe2200004,
	0xc0c031c0, 0x17c07c1f, 0xc0c03e00, 0xe2200005, 0xc0c031c0, 0x17c07c1f,
	0x1b00001f, 0x00000801, 0xf0000000, 0x17c07c1f, 0x18d0001f, 0x10006604,
	0x10cf8c1f, 0xd8203e03, 0x17c07c1f, 0xf0000000, 0x17c07c1f, 0x18c0001f,
	0x10006b18, 0x1910001f, 0x10006b18, 0xa1002804, 0xe0c00004, 0xf0000000,
	0x17c07c1f, 0x17c07c1f, 0x1840001f, 0x00000001, 0xa1d48407, 0x1990001f,
	0x10006b08, 0xe8208000, 0x10006b18, 0x00000000, 0x81441801, 0xd82044c5,
	0x17c07c1f, 0x1910001f, 0x100062c4, 0x80809001, 0x18c0001f, 0x2f7ce7ff,
	0x81600801, 0xa0d59403, 0xa0d60803, 0x80849001, 0x1a00001f, 0x10006b0c,
	0x1950001f, 0x10006b0c, 0xa0908805, 0xe2000002, 0x80841001, 0x81600801,
	0xa0db9403, 0xa0de0803, 0x13000c1f, 0xe8208000, 0x10006310, 0x0b160c38,
	0x1b80001f, 0xd00f0000, 0xd0004500, 0x17c07c1f, 0x1b00001f, 0x2f7ce7ff,
	0x81469801, 0xd8204665, 0x17c07c1f, 0x1b80001f, 0xd00f0000, 0x8880000c,
	0x2f7ce7ff, 0xd8006582, 0x17c07c1f, 0xd00046a0, 0x17c07c1f, 0x1b80001f,
	0x500f0000, 0xe8208000, 0x10006354, 0x001fdae3, 0xc0c07140, 0x81401801,
	0xd8004bc5, 0x17c07c1f, 0x81f60407, 0x18c0001f, 0x10006200, 0xc0c06fc0,
	0x12807c1f, 0xe8208000, 0x1000625c, 0x00000001, 0x1b80001f, 0x20000080,
	0xc0c06fc0, 0x1280041f, 0x18c0001f, 0x10006208, 0xc0c06fc0, 0x12807c1f,
	0xe8208000, 0x10006244, 0x00000001, 0x1b80001f, 0x20000080, 0xc0c06fc0,
	0x1280041f, 0x18c0001f, 0x10006290, 0xc0c06fc0, 0x12807c1f, 0xc0c06fc0,
	0x1280041f, 0x18c0001f, 0x100062dc, 0xe0c00001, 0xc2803ee0, 0x1292041f,
	0x81469801, 0xd8004ca5, 0x17c07c1f, 0x8880000c, 0x2f7ce7ff, 0xd80060e2,
	0x17c07c1f, 0xc0c071e0, 0x17c07c1f, 0x18c0001f, 0x10006294, 0xe0f07fff,
	0xe0e00fff, 0xe0e000ff, 0x81449801, 0xd8004fa5, 0x17c07c1f, 0x1a00001f,
	0x10006604, 0xe2200001, 0xc0c03e00, 0x17c07c1f, 0xe2200006, 0x18c0001f,
	0x10001130, 0x1910001f, 0x10001130, 0xa1108404, 0xe0c00004, 0xc0c03e00,
	0x17c07c1f, 0xa1d38407, 0xa1d98407, 0xa0108400, 0xa0120400, 0xa0148400,
	0xa0150400, 0xa0158400, 0xa01b8400, 0xa01c0400, 0xa01c8400, 0xa0188400,
	0xa0190400, 0xa0198400, 0x81441801, 0xd8205445, 0x17c07c1f, 0x1910001f,
	0x10006b0c, 0x1a00001f, 0x100062c4, 0x1950001f, 0x100062c4, 0x80809001,
	0x81748405, 0xa1548805, 0xe2000005, 0x1a00001f, 0x10006b0c, 0x80c01801,
	0x81308404, 0xa1108c04, 0xe2000004, 0x1b00001f, 0x2f7ce7ff, 0x80c09401,
	0xc8e03743, 0x17c07c1f, 0xe8208000, 0x10006310, 0x0b1603f8, 0x1950001f,
	0x100062c4, 0x80841401, 0x81600801, 0x18c0001f, 0xbffce7ff, 0xa0de0803,
	0xa0db9403, 0x13000c1f, 0x1b80001f, 0x90100000, 0x80c00001, 0xc8c00883,
	0x17c07c1f, 0x80c10001, 0xc8c01523, 0x17c07c1f, 0x1b00001f, 0x2f7ce7ff,
	0x18c0001f, 0x10006294, 0xe0e001fe, 0xe0e003fc, 0xe0e007f8, 0xe0e00ff0,
	0x1b80001f, 0x20000020, 0xe0f07ff0, 0xe0f07f00, 0x81449801, 0xd8005aa5,
	0x17c07c1f, 0x1a00001f, 0x10006604, 0xe2200006, 0xc0c03e00, 0x17c07c1f,
	0xe2200000, 0x18c0001f, 0x10001130, 0x1910001f, 0x10001130, 0x81308404,
	0xe0c00004, 0xc0c03e00, 0x17c07c1f, 0x1b80001f, 0x200016a8, 0x1880001f,
	0x1000110c, 0xe0a00618, 0x1880001f, 0x10001114, 0xe0a00618, 0x1880001f,
	0x1000111c, 0xe0a00618, 0x1880001f, 0x10001124, 0xe0a00618, 0x18d0001f,
	0x10006b68, 0x1880001f, 0x10001110, 0xe0800003, 0x1880001f, 0x10001118,
	0x11080c1f, 0xe0800004, 0x18d0001f, 0x10006b6c, 0x1880001f, 0x10001120,
	0xe0800003, 0x1880001f, 0x10001128, 0x11080c1f, 0xe0800004, 0x80388400,
	0x80390400, 0x80398400, 0x1b80001f, 0x20000300, 0x803b8400, 0x803c0400,
	0x803c8400, 0x1b80001f, 0x20000300, 0x80348400, 0x80350400, 0x80358400,
	0x1b80001f, 0x20000104, 0x10007c1f, 0x81f38407, 0x81f98407, 0x81f90407,
	0x81f40407, 0x81401801, 0xd8006585, 0x17c07c1f, 0x18c0001f, 0x100062dc,
	0xe0c0001f, 0x18c0001f, 0x10006290, 0x1212841f, 0xc0c06d40, 0x12807c1f,
	0xc0c06d40, 0x1280041f, 0x18c0001f, 0x10006208, 0x1212841f, 0xc0c06d40,
	0x12807c1f, 0xe8208000, 0x10006244, 0x00000000, 0x1b80001f, 0x20000080,
	0xc0c06d40, 0x1280041f, 0x18c0001f, 0x10006200, 0x1212841f, 0xc0c06d40,
	0x12807c1f, 0xe8208000, 0x1000625c, 0x00000000, 0x1b80001f, 0x20000080,
	0xc0c06d40, 0x1280041f, 0xe8208000, 0x10006824, 0x000f0000, 0x81f48407,
	0xa1d60407, 0x81f10407, 0xa1db0407, 0xc2803ee0, 0x1293841f, 0x81441801,
	0xd8206cc5, 0x17c07c1f, 0x18c0001f, 0x10006b14, 0xe0c0000c, 0x18c0001f,
	0x10006b68, 0x1950001f, 0x100063c0, 0xe0c00005, 0x1910001f, 0x100062c4,
	0x80849001, 0x1a00001f, 0x10006b0c, 0x1910001f, 0x10006b0c, 0xa0908804,
	0xe2000002, 0x18c0001f, 0x00001001, 0x80841001, 0x81600801, 0xa0de0803,
	0xa0db9403, 0x13000c1f, 0xe8208000, 0x10006310, 0x0b160c38, 0x1b80001f,
	0x900a0000, 0x88900001, 0x10006814, 0xd8206922, 0x17c07c1f, 0x1910001f,
	0x10006b0c, 0x1a00001f, 0x100062c4, 0x1950001f, 0x100062c4, 0x80809001,
	0x81748405, 0xa1548805, 0x80801001, 0x81750405, 0xa1550805, 0xe2000005,
	0x1ac0001f, 0x55aa55aa, 0x10007c1f, 0xf0000000, 0xd8006e4a, 0x17c07c1f,
	0xe2e00036, 0x17c07c1f, 0x17c07c1f, 0xe2e0003e, 0x1380201f, 0xe2e0003c,
	0xd8206f8a, 0x17c07c1f, 0x1b80001f, 0x20000018, 0xe2e0007c, 0x1b80001f,
	0x20000003, 0xe2e0005c, 0xe2e0004c, 0xe2e0004d, 0xf0000000, 0x17c07c1f,
	0xd800706a, 0x17c07c1f, 0xe2e0004f, 0xe2e0006f, 0xe2e0002f, 0xd820710a,
	0x17c07c1f, 0xe2e0002e, 0xe2e0003e, 0xe2e00032, 0xf0000000, 0x17c07c1f,
	0xa1d10407, 0x1b80001f, 0x20000020, 0xf0000000, 0x17c07c1f, 0xa1d40407,
	0x1391841f, 0xa1d90407, 0xf0000000, 0x17c07c1f
};
static struct pcm_desc suspend_pcm = {
	.version	= "pcm_suspend_v34.5_20150120",
	.base		= suspend_binary,
	.size		= 916,
	.sess		= 2,
	.replace	= 0,
	.vec0		= EVENT_VEC(23, 1, 0, 0),	/* FUNC_MD_VRF18_WAKEUP */
	.vec1		= EVENT_VEC(28, 1, 0, 34),	/* FUNC_MD_VRF18_SLEEP */
	.vec2		= EVENT_VEC(11, 1, 0, 68),	/* FUNC_26M_WAKEUP */
	.vec3		= EVENT_VEC(12, 1, 0, 108),	/* FUNC_26M_SLEEP */
	.vec4		= EVENT_VEC(30, 1, 0, 169),	/* FUNC_APSRC_WAKEUP */
	.vec5		= EVENT_VEC(31, 1, 0, 256),	/* FUNC_APSRC_SLEEP */
	.vec6		= EVENT_VEC(11, 1, 0, 442),	/* FUNC_VCORE_HIGH */
	.vec7		= EVENT_VEC(12, 1, 0, 469),	/* FUNC_VCORE_LOW */
};
#elif defined(CONFIG_ARCH_MT6735M)
static const u32 suspend_binary[] = {
	0x81429801, 0xd80001c5, 0x17c07c1f, 0x1a00001f, 0x10006604, 0xe2200007,
	0x18c0001f, 0x10001124, 0x1910001f, 0x10001124, 0x81308404, 0xe0c00004,
	0xc0c03da0, 0x17c07c1f, 0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4,
	0xa0940402, 0xe2000002, 0x80c10001, 0x1b00001f, 0xbf7ce7ff, 0xd8200363,
	0x17c07c1f, 0x1b00001f, 0x7f7ce7ff, 0xf0000000, 0x17c07c1f, 0x1a00001f,
	0x100062c4, 0x1890001f, 0x100062c4, 0x80b40402, 0xe2000002, 0x81429801,
	0xd8000625, 0x17c07c1f, 0x1a00001f, 0x10006604, 0xe2200007, 0x18c0001f,
	0x10001124, 0x1910001f, 0x10001124, 0xa1108404, 0xe0c00004, 0xc0c03da0,
	0x17c07c1f, 0x80c10001, 0x1b00001f, 0x6ffcf7ff, 0xd8000703, 0x17c07c1f,
	0x1b00001f, 0xaffce7ff, 0xf0000000, 0x17c07c1f, 0x81f58407, 0x81f68407,
	0x1800001f, 0x17cf0f3f, 0x1b80001f, 0x20000000, 0x1800001f, 0x17cf0f16,
	0xa1d28407, 0x81f20407, 0x1a00001f, 0x10006604, 0xc0c03da0, 0xe2200004,
	0xc0c03da0, 0xe2200005, 0x81409801, 0xd8000a25, 0x17c07c1f, 0x18c0001f,
	0x10006234, 0xc0c029a0, 0x1200041f, 0x80310400, 0x1b80001f, 0x2000000a,
	0xa0110400, 0x81f00407, 0xa1dd0407, 0x18c0001f, 0x100062d8, 0xc0c02c60,
	0x1200041f, 0x81fd0407, 0xc2803e80, 0x1290041f, 0x1b00001f, 0x6ffcf7ff,
	0xf0000000, 0x17c07c1f, 0x1b00001f, 0x2f7ce7ff, 0x1b80001f, 0x20000004,
	0xd80013ac, 0x17c07c1f, 0x1a00001f, 0x10006604, 0xc0c03da0, 0xe2200002,
	0xc0c03da0, 0xe2200003, 0x1880001f, 0x10006320, 0xc0c02f80, 0xe080000f,
	0xd8200f03, 0x17c07c1f, 0x1b00001f, 0x6ffcf7ff, 0xd00013a0, 0x17c07c1f,
	0xe080001f, 0x81409801, 0xd8001185, 0x17c07c1f, 0x18c0001f, 0x10214094,
	0x1910001f, 0x1020e374, 0xe0c00004, 0x18c0001f, 0x10214098, 0x1910001f,
	0x1020e378, 0xe0c00004, 0x1910001f, 0x10213378, 0x18c0001f, 0x10006234,
	0xc0c02b80, 0x17c07c1f, 0x18c0001f, 0x100062d8, 0xc0c02d80, 0x17c07c1f,
	0xc2803e80, 0x1290841f, 0xa1d20407, 0x81f28407, 0xa1d68407, 0x1800001f,
	0x17cf0f3f, 0x1800001f, 0x17ff0f3f, 0x19c0001f, 0x001c2397, 0x1b00001f,
	0x2f7cefff, 0xf0000000, 0x17c07c1f, 0x81411801, 0xd80015c5, 0x17c07c1f,
	0x18c0001f, 0x102135cc, 0x1910001f, 0x102135cc, 0x813f8404, 0xe0c00004,
	0x18c0001f, 0x10006240, 0xe0e00016, 0xe0e0001e, 0xe0e0000e, 0xe0e0000f,
	0x803e0400, 0x1b80001f, 0x20000222, 0x80380400, 0x1b80001f, 0x20000280,
	0x803b0400, 0x1b80001f, 0x2000001a, 0x803d0400, 0x1b80001f, 0x20000208,
	0x80340400, 0x80310400, 0xe8208000, 0x10210044, 0x00000100, 0x1b80001f,
	0x20000068, 0x1b80001f, 0x2000000a, 0x18c0001f, 0x10006240, 0xe0e0000d,
	0xd8001c45, 0x17c07c1f, 0x1b80001f, 0x20000020, 0x18c0001f, 0x102130f0,
	0x1910001f, 0x102130f0, 0xa9000004, 0x10000000, 0xe0c00004, 0x1b80001f,
	0x2000000a, 0x89000004, 0xefffffff, 0xe0c00004, 0x18c0001f, 0x102140f4,
	0x1910001f, 0x102140f4, 0xa9000004, 0x02000000, 0xe0c00004, 0x1b80001f,
	0x2000000a, 0x89000004, 0xfdffffff, 0xe0c00004, 0x81fa0407, 0x81f08407,
	0xe8208000, 0x10006354, 0x001fdae3, 0xa1d80407, 0xa1de8407, 0xa1df0407,
	0xc2803e80, 0x1291041f, 0x1b00001f, 0xbf7ce7ff, 0x1950001f, 0x100062c4,
	0x80c41401, 0xd8001ea3, 0x17c07c1f, 0x1b00001f, 0xaffce7ff, 0xf0000000,
	0x17c07c1f, 0x1b80001f, 0x20000fdf, 0x1a50001f, 0x10006608, 0x80c9a401,
	0x810ba401, 0x10920c1f, 0xa0979002, 0x80ca2401, 0xa0938c02, 0x8080080d,
	0xd8202322, 0x17c07c1f, 0x81f08407, 0xa1d80407, 0xa1de8407, 0xa1df0407,
	0x1b00001f, 0x2f7ce7ff, 0x1b80001f, 0x20000004, 0xd800296c, 0x17c07c1f,
	0x1b00001f, 0xbf7ce7ff, 0x1950001f, 0x100062c4, 0x80c41401, 0xd8002963,
	0x17c07c1f, 0x1b00001f, 0xaffce7ff, 0xd0002960, 0x17c07c1f, 0x81f80407,
	0x81fe8407, 0x81ff0407, 0x1880001f, 0x10006320, 0xc0c02e20, 0xe080000f,
	0xd8002083, 0x17c07c1f, 0xe080001f, 0xa1da0407, 0xe8208000, 0x10210048,
	0x00000100, 0x1b80001f, 0x20000068, 0xa0110400, 0xa0140400, 0xa0180400,
	0xa01b0400, 0xa01d0400, 0x1b80001f, 0x20000068, 0xa01e0400, 0x1b80001f,
	0x20000104, 0x81411801, 0xd8002805, 0x17c07c1f, 0x18c0001f, 0x10006240,
	0xc0c02d80, 0x17c07c1f, 0x18c0001f, 0x102135cc, 0x1910001f, 0x102135cc,
	0xa11f8404, 0xe0c00004, 0xc2803e80, 0x1291841f, 0x1b00001f, 0x6ffcf7ff,
	0x1950001f, 0x100062c4, 0x80c41401, 0xd8202963, 0x17c07c1f, 0x1b00001f,
	0x7f7ce7ff, 0xf0000000, 0x17c07c1f, 0xe0f07f16, 0x1380201f, 0xe0f07f1e,
	0x1380201f, 0xe0f07f0e, 0x1b80001f, 0x20000100, 0xe0f07f0c, 0xe0f07f0d,
	0xe0f07e0d, 0xe0f07c0d, 0xe0f0780d, 0xe0f0700d, 0xf0000000, 0x17c07c1f,
	0xe0f07f0d, 0xe0f07f1d, 0xe0f07f1f, 0xe0f07f1e, 0xe0f07f12, 0xf0000000,
	0x17c07c1f, 0xe0e00016, 0x1380201f, 0xe0e0001e, 0x1380201f, 0xe0e0000e,
	0xe0e0000c, 0xe0e0000d, 0xf0000000, 0x17c07c1f, 0xe0e0000f, 0xe0e0001e,
	0xe0e00012, 0xf0000000, 0x17c07c1f, 0x1112841f, 0xa1d08407, 0xd8202ee4,
	0x80eab401, 0xd8002e63, 0x01200404, 0x1a00001f, 0x10006814, 0xe2000003,
	0xf0000000, 0x17c07c1f, 0xa1d00407, 0x1b80001f, 0x20000100, 0x80ea3401,
	0x1a00001f, 0x10006814, 0xe2000003, 0xf0000000, 0x17c07c1f, 0x1092041f,
	0x81499801, 0xd8203225, 0x17c07c1f, 0xd82035e2, 0x17c07c1f, 0x18d0001f,
	0x40000000, 0x18d0001f, 0x60000000, 0xd8003122, 0x00a00402, 0x814a1801,
	0xd8203385, 0x17c07c1f, 0xd82035e2, 0x17c07c1f, 0x18d0001f, 0x40000000,
	0x18d0001f, 0x80000000, 0xd8003282, 0x00a00402, 0x814a9801, 0xd82034e5,
	0x17c07c1f, 0xd82035e2, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f,
	0xc0000000, 0xd80033e2, 0x00a00402, 0xd82035e2, 0x17c07c1f, 0x18d0001f,
	0x40000000, 0x18d0001f, 0x40000000, 0xd80034e2, 0x00a00402, 0xf0000000,
	0x17c07c1f, 0x81441801, 0xd82038a5, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0xe2200004, 0xc0c03da0, 0x17c07c1f, 0xc0c030a0, 0x17c07c1f, 0xe2200003,
	0xc0c03da0, 0x17c07c1f, 0xc0c030a0, 0x17c07c1f, 0xe2200002, 0xc0c03da0,
	0x17c07c1f, 0xc0c030a0, 0x17c07c1f, 0x1a00001f, 0x100062c4, 0x1890001f,
	0x100062c4, 0xa0908402, 0xe2000002, 0x1b00001f, 0x00001001, 0xf0000000,
	0x17c07c1f, 0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4, 0x80b08402,
	0xe2000002, 0x81441801, 0xd8203d25, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0xe2200003, 0xc0c03da0, 0x17c07c1f, 0xc0c030a0, 0x17c07c1f, 0xe2200004,
	0xc0c03da0, 0x17c07c1f, 0xc0c030a0, 0x17c07c1f, 0xe2200005, 0xc0c03da0,
	0x17c07c1f, 0xc0c030a0, 0x17c07c1f, 0x1b00001f, 0x00000801, 0xf0000000,
	0x17c07c1f, 0x18d0001f, 0x10006604, 0x10cf8c1f, 0xd8203da3, 0x17c07c1f,
	0xf0000000, 0x17c07c1f, 0x18c0001f, 0x10006b18, 0x1910001f, 0x10006b18,
	0xa1002804, 0xe0c00004, 0xf0000000, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x1840001f, 0x00000001, 0xa1d48407, 0x1990001f,
	0x10006b08, 0xe8208000, 0x10006b18, 0x00000000, 0x1b00001f, 0x2f7ce7ff,
	0x81469801, 0xd82042a5, 0x17c07c1f, 0x1b80001f, 0xd00f0000, 0x8880000c,
	0x2f7ce7ff, 0xd8005e82, 0x17c07c1f, 0xd00042e0, 0x17c07c1f, 0x1b80001f,
	0x500f0000, 0xe8208000, 0x10006354, 0x001fdae3, 0xc0c06780, 0x81401801,
	0xd8004805, 0x17c07c1f, 0x81f60407, 0x18c0001f, 0x10006200, 0xc0c06600,
	0x12807c1f, 0xe8208000, 0x1000625c, 0x00000001, 0x1b80001f, 0x20000080,
	0xc0c06600, 0x1280041f, 0x18c0001f, 0x10006208, 0xc0c06600, 0x12807c1f,
	0xe8208000, 0x10006244, 0x00000001, 0x1b80001f, 0x20000080, 0xc0c06600,
	0x1280041f, 0x18c0001f, 0x10006290, 0xc0c06600, 0x12807c1f, 0xc0c06600,
	0x1280041f, 0x18c0001f, 0x100062dc, 0xe0c00001, 0xc2803e80, 0x1292041f,
	0x81469801, 0xd80048e5, 0x17c07c1f, 0x8880000c, 0x2f7ce7ff, 0xd80059e2,
	0x17c07c1f, 0xc0c06820, 0x17c07c1f, 0x18c0001f, 0x10006294, 0xe0f07fff,
	0xe0e00fff, 0xe0e000ff, 0x81449801, 0xd8004be5, 0x17c07c1f, 0x1a00001f,
	0x10006604, 0xe2200001, 0xc0c03da0, 0x17c07c1f, 0xe2200006, 0x18c0001f,
	0x1000111c, 0x1910001f, 0x1000111c, 0xa1108404, 0xe0c00004, 0xc0c03da0,
	0x17c07c1f, 0xa1d38407, 0xa1d98407, 0xa0108400, 0xa0120400, 0xa0148400,
	0xa0150400, 0xa0158400, 0xa01b8400, 0xa01c0400, 0xa01c8400, 0xa0188400,
	0xa0190400, 0xa0198400, 0xe8208000, 0x10006310, 0x0b1603f8, 0x1b00001f,
	0xbf7ce7ff, 0x1b80001f, 0x90100000, 0x80c00001, 0xc8c00743, 0x17c07c1f,
	0x1910001f, 0x100062c4, 0x80c41001, 0xc8e00003, 0x17c07c1f, 0x80c10001,
	0xc8c013e3, 0x17c07c1f, 0x1b00001f, 0x2f7ce7ff, 0x18c0001f, 0x10006294,
	0xe0e001fe, 0xe0e003fc, 0xe0e007f8, 0xe0e00ff0, 0x1b80001f, 0x20000020,
	0xe0f07ff0, 0xe0f07f00, 0x81449801, 0xd80053a5, 0x17c07c1f, 0x1a00001f,
	0x10006604, 0xe2200006, 0xc0c03da0, 0x17c07c1f, 0xe2200000, 0x18c0001f,
	0x1000111c, 0x1910001f, 0x1000111c, 0x81308404, 0xe0c00004, 0xc0c03da0,
	0x17c07c1f, 0x1b80001f, 0x200016a8, 0x1880001f, 0x100010f8, 0xe0a00618,
	0x1880001f, 0x10001100, 0xe0a00618, 0x1880001f, 0x10001108, 0xe0a00618,
	0x1880001f, 0x10001110, 0xe0a00618, 0x18d0001f, 0x10006b68, 0x1880001f,
	0x100010fc, 0xe0800003, 0x1880001f, 0x10001104, 0x11080c1f, 0xe0800004,
	0x18d0001f, 0x10006b6c, 0x1880001f, 0x1000110c, 0xe0800003, 0x1880001f,
	0x10001114, 0x11080c1f, 0xe0800004, 0x80388400, 0x80390400, 0x80398400,
	0x1b80001f, 0x20000300, 0x803b8400, 0x803c0400, 0x803c8400, 0x1b80001f,
	0x20000300, 0x80348400, 0x80350400, 0x80358400, 0x1b80001f, 0x20000104,
	0x10007c1f, 0x81f38407, 0x81f98407, 0x81f90407, 0x81f40407, 0x81401801,
	0xd8005e85, 0x17c07c1f, 0x18c0001f, 0x100062dc, 0xe0c0001f, 0x18c0001f,
	0x10006290, 0x1212841f, 0xc0c06380, 0x12807c1f, 0xc0c06380, 0x1280041f,
	0x18c0001f, 0x10006208, 0x1212841f, 0xc0c06380, 0x12807c1f, 0xe8208000,
	0x10006244, 0x00000000, 0x1b80001f, 0x20000080, 0xc0c06380, 0x1280041f,
	0x18c0001f, 0x10006200, 0x1212841f, 0xc0c06380, 0x12807c1f, 0xe8208000,
	0x1000625c, 0x00000000, 0x1b80001f, 0x20000080, 0xc0c06380, 0x1280041f,
	0xe8208000, 0x10006824, 0x000f0000, 0x81f48407, 0xa1d60407, 0x81f10407,
	0xa1db0407, 0xc2803e80, 0x1293841f, 0x81441801, 0xd8206305, 0x17c07c1f,
	0x18c0001f, 0x10006b14, 0xe0c0000c, 0x18c0001f, 0x10006b68, 0x1950001f,
	0x100063c0, 0xe0c00005, 0xe8208000, 0x10006310, 0x0b160c08, 0x1b00001f,
	0x00001001, 0x1b80001f, 0x900a0000, 0x88900001, 0x10006814, 0xd8206162,
	0x17c07c1f, 0x1910001f, 0x100062c4, 0x80809001, 0xc8a03622, 0x17c07c1f,
	0x1ac0001f, 0x55aa55aa, 0x10007c1f, 0xf0000000, 0xd800648a, 0x17c07c1f,
	0xe2e00036, 0x17c07c1f, 0x17c07c1f, 0xe2e0003e, 0x1380201f, 0xe2e0003c,
	0xd82065ca, 0x17c07c1f, 0x1b80001f, 0x20000018, 0xe2e0007c, 0x1b80001f,
	0x20000003, 0xe2e0005c, 0xe2e0004c, 0xe2e0004d, 0xf0000000, 0x17c07c1f,
	0xd80066aa, 0x17c07c1f, 0xe2e0004f, 0xe2e0006f, 0xe2e0002f, 0xd820674a,
	0x17c07c1f, 0xe2e0002e, 0xe2e0003e, 0xe2e00032, 0xf0000000, 0x17c07c1f,
	0xa1d10407, 0x1b80001f, 0x20000020, 0xf0000000, 0x17c07c1f, 0xa1d40407,
	0x1391841f, 0xa1d90407, 0xf0000000, 0x17c07c1f
};
static struct pcm_desc suspend_pcm = {
	.version	= "pcm_suspend_v0.2_20150113",
	.base		= suspend_binary,
	.size		= 838,
	.sess		= 2,
	.replace	= 0,
	.vec0		= EVENT_VEC(23, 1, 0, 0),	/* FUNC_MD_VRF18_WAKEUP */
	.vec1		= EVENT_VEC(28, 1, 0, 29),	/* FUNC_MD_VRF18_SLEEP */
	.vec2		= EVENT_VEC(11, 1, 0, 58),	/* FUNC_26M_WAKEUP */
	.vec3		= EVENT_VEC(12, 1, 0, 98),	/* FUNC_26M_SLEEP */
	.vec4		= EVENT_VEC(30, 1, 0, 159),	/* FUNC_APSRC_WAKEUP */
	.vec5		= EVENT_VEC(31, 1, 0, 247),	/* FUNC_APSRC_SLEEP */
	.vec6		= EVENT_VEC(11, 1, 0, 433),	/* FUNC_VCORE_HIGH */
	.vec7		= EVENT_VEC(12, 1, 0, 463),	/* FUNC_VCORE_LOW */
};

#elif defined(CONFIG_ARCH_MT6753)
//~~~Do something in D3
#else
//ERROR
#endif


/**************************************
 * SW code for suspend
 **************************************/
#define SPM_SYSCLK_SETTLE       99      /* 3ms */

#define WAIT_UART_ACK_TIMES     10      /* 10 * 10us */

#define SPM_WAKE_PERIOD         600     /* sec */

#define WAKE_SRC_FOR_SUSPEND                                                          \
    (WAKE_SRC_KP | WAKE_SRC_EINT |  WAKE_SRC_CONN_WDT  |  WAKE_SRC_CCIF0_MD | WAKE_SRC_CCIF1_MD | WAKE_SRC_CONN2AP |                 \
     WAKE_SRC_USB_CD | WAKE_SRC_USB_PDN |  WAKE_SRC_SEJ |          \
     /*WAKE_SRC_SYSPWREQ |*/ WAKE_SRC_MD_WDT | WAKE_SRC_C2K_WDT | WAKE_SRC_CLDMA_MD)

#define WAKE_SRC_FOR_MD32  0                                          \
    //(WAKE_SRC_AUD_MD32)

#define spm_is_wakesrc_invalid(wakesrc)     (!!((u32)(wakesrc) & 0xc0003803))

#ifdef CONFIG_MTK_RAM_CONSOLE
#define SPM_AEE_RR_REC 1
#else
#define SPM_AEE_RR_REC 0
#endif

#if SPM_AEE_RR_REC
enum spm_suspend_step
{
	SPM_SUSPEND_ENTER=0,
	SPM_SUSPEND_ENTER_WFI,
	SPM_SUSPEND_LEAVE_WFI,
	SPM_SUSPEND_LEAVE
};
extern void aee_rr_rec_spm_suspend_val(u32 val);
extern u32 aee_rr_curr_spm_suspend_val(void);
#endif

//FIXME: Denali early porting
#if 0
int __attribute__((weak)) exec_ccci_kern_func_by_md_id(int md_id, unsigned int id, char *buf, unsigned int len){}
int __attribute__((weak)) get_dynamic_period(int first_use, int first_wakeup_time, int battery_capacity_level){}
#else
extern int get_dynamic_period(int first_use, int first_wakeup_time, int battery_capacity_level);
#endif

extern int mt_irq_mask_all(struct mtk_irq_mask *mask);
extern int mt_irq_mask_restore(struct mtk_irq_mask *mask);
extern void mt_irq_unmask_for_sleep(unsigned int irq);


extern int request_uart_to_sleep(void);
extern int request_uart_to_wakeup(void);
extern void mtk_uart_restore(void);
extern void dump_uart_reg(void);



static struct pwr_ctrl suspend_ctrl = {
	.wake_src		= WAKE_SRC_FOR_SUSPEND,
	.wake_src_md32		= WAKE_SRC_FOR_MD32,
	.r0_ctrl_en		= 1,
	.r7_ctrl_en		= 1,
	.infra_dcm_lock		= 1,
	.wfi_op			= WFI_OP_AND,

    .ca7top_idle_mask   = 0,
    .ca15top_idle_mask  = 0,
    .mcusys_idle_mask   = 0,
    .disp_req_mask      = 0,
    .mfg_req_mask       = 0,
    .md1_req_mask       = 0,
    .md2_req_mask       = 0,
    .md32_req_mask      = 0,
    .md_apsrc_sel       = 0,
    .md2_apsrc_sel      = 0,
    .lte_mask           = 1,
    .conn_mask          = 0,
#if 0
    .ccif0_to_ap_mask = 1,
    .ccif0_to_md_mask = 1,
    .ccif1_to_ap_mask = 1,
    .ccif1_to_md_mask = 1,
    .ccifmd_md1_event_mask = 1,
    .ccifmd_md2_event_mask = 1,
#endif
    //.pcm_f26m_req = 1,
 
    .ca7_wfi0_en    = 1,
    .ca7_wfi1_en    = 1,
    .ca7_wfi2_en    = 1,
    .ca7_wfi3_en    = 1,    
    .ca15_wfi0_en   = 1,
    .ca15_wfi1_en   = 1,
    .ca15_wfi2_en   = 1,
    .ca15_wfi3_en   = 1,

#if SPM_BYPASS_SYSPWREQ
	.syspwreq_mask		= 1,
#endif
};

struct spm_lp_scen __spm_suspend = {
	.pcmdesc	= &suspend_pcm,
	.pwrctrl	= &suspend_ctrl,
    .wakestatus = &suspend_info[0],
};

void spm_i2c_control(u32 channel, bool onoff)
{
	 return;
#if 0
    //static int pdn = 0;
    static bool i2c_onoff = 0;
   
#ifdef CONFIG_OF
    void __iomem *base;
#else
    u32 base;//, i2c_clk;
#endif
    switch(channel)
    {
        case 0:
            base = SPM_I2C0_BASE;
            //i2c_clk = MT_CG_INFRA_I2C0;
            break;
        case 1:
            base = SPM_I2C1_BASE;
            //i2c_clk = MT_CG_INFRA_I2C1;
            break;
        case 2:
            base = SPM_I2C2_BASE;
            //i2c_clk = MT_CG_INFRA_I2C2;
	          break;
        default:
            base = SPM_I2C2_BASE;
            break;
    }

    if ((1 == onoff) && (0 == i2c_onoff))
    {
       i2c_onoff = 1;
#if 0
#if 1
        pdn = spm_read(INFRA_PDN_STA0) & (1U << i2c_clk);
        spm_write(INFRA_PDN_CLR0, pdn);                /* power on I2C */
#else
        pdn = clock_is_on(i2c_clk);
        if (!pdn)
            enable_clock(i2c_clk, "spm_i2c");
#endif
#endif
        spm_write(base + OFFSET_CONTROL,     0x0);    /* init I2C_CONTROL */
        spm_write(base + OFFSET_TRANSAC_LEN, 0x1);    /* init I2C_TRANSAC_LEN */
        spm_write(base + OFFSET_EXT_CONF,    0x0);    /* init I2C_EXT_CONF */
        spm_write(base + OFFSET_IO_CONFIG,   0x0);    /* init I2C_IO_CONFIG */
        spm_write(base + OFFSET_HS,          0x102);  /* init I2C_HS */
    }
    else
    if ((0 == onoff) && (1 == i2c_onoff))
    {
        i2c_onoff = 0;
#if 0
#if 1
        spm_write(INFRA_PDN_SET0, pdn);                /* restore I2C power */
#else
        if (!pdn)
            disable_clock(i2c_clk, "spm_i2c");      
#endif
#endif
    }
    else
        ASSERT(1);
#endif
}


extern unsigned int mt_get_clk_mem_sel(void);
/*
static bool spm_set_suspend_pcm_ver(u32 *suspend_flags)
{
    u32 flag;

    flag = *suspend_flags;

    if(mt_get_clk_mem_sel()==MEMPLL3PLL)
    {
        __spm_suspend.pcmdesc = &suspend_pcm_3pll;
        flag |= SPM_VCORE_DVS_DIS;
    }
    else if(mt_get_clk_mem_sel()==MEMPLL1PLL)

    {
        __spm_suspend.pcmdesc = &suspend_pcm_1pll;
        flag &= ~SPM_VCORE_DVS_DIS;
    }
    else
        return false;

    *suspend_flags = flag;
    return true;

}
*/
static void spm_suspend_pre_process(struct pwr_ctrl *pwrctrl)
{
#if 0
    u32 rdata1 = 0, rdata2 = 0;
#endif

    /* set PMIC WRAP table for suspend power control */
    mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_SUSPEND);
    
    spm_i2c_control(I2C_CHANNEL, 1);

#if 0
    /* for infra pdn (emi driving) */
    spm_write(0xF0004000, spm_read(0xF0004000) | (1 << 24));

    /* MEMPLL control for SPM */
    spm_write(0xF000F5C8, 0x3010F030);
    spm_write(0xF000F5CC, 0x50101010);
#endif
    //spm_write(0xF0001070 , spm_read(0xF0001070) | (1 << 21)); // 26:26 enable 
    //spm_write(0xF0000204 , spm_read(0xF0000204) | (1 << 0));  // BUS 26MHz enable 
    //spm_write(0xF0001108 , 0x0);

#ifdef CONFIG_MD32_SUPPORT
    //spm_write(MD32_BASE+0x2C, (spm_read(MD32_BASE+0x2C) & ~0xFFFF) | 0xcafe);
#endif

#if 0    
    pwrap_read(0x2c2, &rdata1);
    pwrap_write(0x2c2, 0x0123);
    pwrap_read(0x2c2, &rdata2);
    if(rdata2 != 0x0123)
    {
        spm_crit2("suspend pmic wrapper 0x2c2, rdata1 = 0x%x, rdata2 = 0x%x\n", rdata1, rdata2);
        BUG();
    }
#endif
}

static void spm_suspend_post_process(struct pwr_ctrl *pwrctrl)
{
#if 0
    u32 rdata1 = 0, rdata2 = 0;
          
    pwrap_read(0x2c2, &rdata1);
    pwrap_write(0x2c2, 0x3210);
    pwrap_read(0x2c2, &rdata2);
    if(rdata2 != 0x3210)
    {
        spm_crit2("resume pmic wrapper 0x2c2, rdata1 = 0x%x, rdata2 = 0x%x\n", rdata1, rdata2);
        BUG();
    }
#endif

#ifdef CONFIG_MD32_SUPPORT
    //spm_write(MD32_BASE+0x2C, spm_read(MD32_BASE+0x2C) & ~0xFFFF);
#endif

    /* set PMIC WRAP table for normal power control */
    mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_NORMAL);

    spm_i2c_control(I2C_CHANNEL, 0);
}

static void spm_set_sysclk_settle(void)
{
    u32 md_settle, settle;

    /* get MD SYSCLK settle */
    spm_write(SPM_CLK_CON, spm_read(SPM_CLK_CON) | CC_SYSSETTLE_SEL);
    spm_write(SPM_CLK_SETTLE, 0);
    md_settle = spm_read(SPM_CLK_SETTLE);

    /* SYSCLK settle = MD SYSCLK settle but set it again for MD PDN */
    spm_write(SPM_CLK_SETTLE, SPM_SYSCLK_SETTLE - md_settle);
    settle = spm_read(SPM_CLK_SETTLE);

    spm_crit2("md_settle = %u, settle = %u\n", md_settle, settle);
}

static void spm_kick_pcm_to_run(struct pwr_ctrl *pwrctrl)
{
    /* enable PCM WDT (normal mode) to start count if needed */
#if SPM_PCMWDT_EN
    {
        u32 con1;
        con1 = spm_read(SPM_PCM_CON1) & ~(CON1_PCM_WDT_WAKE_MODE | CON1_PCM_WDT_EN);
        spm_write(SPM_PCM_CON1, CON1_CFG_KEY | con1);

        if (spm_read(SPM_PCM_TIMER_VAL) > PCM_TIMER_MAX)
            spm_write(SPM_PCM_TIMER_VAL, PCM_TIMER_MAX);
        spm_write(SPM_PCM_WDT_TIMER_VAL, spm_read(SPM_PCM_TIMER_VAL) + PCM_WDT_TIMEOUT);
        spm_write(SPM_PCM_CON1, con1 | CON1_CFG_KEY | CON1_PCM_WDT_EN);
    }
#endif

#if SPM_PCMTIMER_DIS
    {
    	  u32 con1; //CONFIG_MTK_LDVT is enable? Otherwise why con1: /mt_spm_sleep.c:523:9: error: 'con1' undeclared (first use in this function)
        con1 = spm_read(SPM_PCM_CON1) & ~(CON1_PCM_TIMER_EN);
        spm_write(SPM_PCM_CON1, con1 | CON1_CFG_KEY);
    }
#endif

    /* init PCM_PASR_DPD_0 for DPD */
    spm_write(SPM_PCM_PASR_DPD_0, 0);

//FIXME: for K2 fpga early porting
#if 0
    /* make MD32 work in suspend: fscp_ck = CLK26M */
    clkmux_sel(MT_MUX_SCP, 0, "SPM-Sleep");
#endif

    __spm_kick_pcm_to_run(pwrctrl);
}

static void spm_trigger_wfi_for_sleep(struct pwr_ctrl *pwrctrl)
{
//FIXME: for K2 fpga early porting
#if 0
    sync_hw_gating_value();     /* for Vcore DVFS */
#endif

    if (is_cpu_pdn(pwrctrl->pcm_flags)) {
        spm_dormant_sta = mt_cpu_dormant(CPU_SHUTDOWN_MODE/* | DORMANT_SKIP_WFI*/);
        switch (spm_dormant_sta)
        {
            case MT_CPU_DORMANT_RESET:
                break;
            case MT_CPU_DORMANT_ABORT:
                break;
            case MT_CPU_DORMANT_BREAK:
                break;
            case MT_CPU_DORMANT_BYPASS:
                break;
        }
    } else {
        spm_dormant_sta = -1;
		//spm_write(MP0_AXI_CONFIG, spm_read(MP0_AXI_CONFIG) | ACINACTM);
        wfi_with_sync();
      //  spm_write(MP0_AXI_CONFIG, spm_read(MP0_AXI_CONFIG) & ~ACINACTM);
    }

    if (is_infra_pdn(pwrctrl->pcm_flags))
        mtk_uart_restore();
}

static void spm_clean_after_wakeup(void)
{
    /* disable PCM WDT to stop count if needed */
#if SPM_PCMWDT_EN
    spm_write(SPM_PCM_CON1, CON1_CFG_KEY | (spm_read(SPM_PCM_CON1) & ~CON1_PCM_WDT_EN));
#endif

#if SPM_PCMTIMER_DIS
    spm_write(SPM_PCM_CON1, CON1_CFG_KEY | (spm_read(SPM_PCM_CON1) | CON1_PCM_TIMER_EN));
#endif

    __spm_clean_after_wakeup();

//FIXME: for K2 fpga early porting
#if 0
    /* restore clock mux: fscp_ck = SYSPLL1_D2 */
    clkmux_sel(MT_MUX_SCP, 1, "SPM-Sleep");
#endif
}

static wake_reason_t spm_output_wake_reason(struct wake_status *wakesta, struct pcm_desc *pcmdesc)
{
	wake_reason_t wr;
    u32 md32_flag = 0;
    u32 md32_flag2 = 0;

	wr = __spm_output_wake_reason(wakesta, pcmdesc, true);

#if 1
    memcpy(&suspend_info[log_wakesta_cnt], wakesta, sizeof(struct wake_status));
    suspend_info[log_wakesta_cnt].log_index = log_wakesta_index;

    if (10 <= log_wakesta_cnt)
    {
        log_wakesta_cnt = 0;
        spm_snapshot_golden_setting = 0;
    }
    else
    {
        log_wakesta_cnt++;
        log_wakesta_index++;
    }
#if 0
    else
    {
        if (2 != spm_snapshot_golden_setting)
        {
            if ((0x90100000 == wakesta->event_reg) && (0x140001f == wakesta->debug_flag))
                spm_snapshot_golden_setting = 1;
        }
    }
#endif
    
    
    if (0xFFFFFFF0 <= log_wakesta_index)
        log_wakesta_index = 0;
#endif

    #ifdef CONFIG_MD32_SUPPORT
    md32_flag = spm_read(MD32_BASE+0x2C);
    md32_flag2 = spm_read(MD32_BASE+0x30);
    #endif
    spm_crit2("suspend dormant state = %d, md32_flag = 0x%x, md32_flag2 = %d\n", spm_dormant_sta, md32_flag, md32_flag2);
    if (0 != spm_ap_mdsrc_req_cnt)
        spm_crit2("warning: spm_ap_mdsrc_req_cnt = %d, r7[ap_mdsrc_req] = 0x%x\n", spm_ap_mdsrc_req_cnt, spm_read(SPM_POWER_ON_VAL1) & (1<<17));

    if (wakesta->r12 & WAKE_SRC_EINT)
        mt_eint_print_status();

#if 0
    if (wakesta->debug_flag & (1 << 18))
    {
        spm_crit2("MD32 suspned pmic wrapper error");
        BUG();
    }

    if (wakesta->debug_flag & (1 << 19))
    {
        spm_crit2("MD32 resume pmic wrapper error");
        BUG();
    }
#endif

    if (wakesta->r12 & WAKE_SRC_CLDMA_MD)
        exec_ccci_kern_func_by_md_id(0, ID_GET_MD_WAKEUP_SRC, NULL, 0);

	return wr;
}

#if SPM_PWAKE_EN
static u32 spm_get_wake_period(int pwake_time, wake_reason_t last_wr)
{
    int period = SPM_WAKE_PERIOD;

    if (pwake_time < 0) {
        /* use FG to get the period of 1% battery decrease */
        period = get_dynamic_period(last_wr != WR_PCM_TIMER ? 1 : 0, SPM_WAKE_PERIOD, 1);
        if (period <= 0) {
            spm_warn("CANNOT GET PERIOD FROM FUEL GAUGE\n");
            period = SPM_WAKE_PERIOD;
        }
    } else {
        period = pwake_time;
        spm_crit2("pwake = %d\n", pwake_time);
    }

    if (period > 36 * 3600)     /* max period is 36.4 hours */
        period = 36 * 3600;

    return period;
}
#endif

/*
 * wakesrc: WAKE_SRC_XXX
 * enable : enable or disable @wakesrc
 * replace: if true, will replace the default setting
 */
int spm_set_sleep_wakesrc(u32 wakesrc, bool enable, bool replace)
{
    unsigned long flags;

    if (spm_is_wakesrc_invalid(wakesrc))
        return -EINVAL;

    spin_lock_irqsave(&__spm_lock, flags);
    if (enable) {
        if (replace)
            __spm_suspend.pwrctrl->wake_src = wakesrc;
        else
            __spm_suspend.pwrctrl->wake_src |= wakesrc;
    } else {
        if (replace)
            __spm_suspend.pwrctrl->wake_src = 0;
        else
            __spm_suspend.pwrctrl->wake_src &= ~wakesrc;
    }
    spin_unlock_irqrestore(&__spm_lock, flags);

    return 0;
}

/*
 * wakesrc: WAKE_SRC_XXX
 */
u32 spm_get_sleep_wakesrc(void)
{
    return __spm_suspend.pwrctrl->wake_src;
}

#if 0
static void uart_mhl_gpio_sleep_ctrl(bool suspend)
{
    static int mhl_ws_gpio_mode= 0;
    static int mhl_ck_gpio_mode= 0;
    static int mhl_dat_gpio_mode= 0;
    if(suspend == true)
    {
#ifdef GPIO_MHL_I2S_OUT_WS_PIN
        mhl_ws_gpio_mode = mt_get_gpio_mode(GPIO_MHL_I2S_OUT_WS_PIN);
        mhl_ck_gpio_mode = mt_get_gpio_mode(GPIO_MHL_I2S_OUT_CK_PIN);
        mhl_dat_gpio_mode = mt_get_gpio_mode(GPIO_MHL_I2S_OUT_DAT_PIN);
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_MODE_GPIO);
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_MODE_01);
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_MODE_02);

		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_PULL_DOWN);

		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_PULL_DOWN);

		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_PULL_DOWN);

#endif

    }
    else
    {
#ifdef GPIO_MHL_I2S_OUT_WS_PIN
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_WS_PIN, mhl_ws_gpio_mode);
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_CK_PIN, mhl_ck_gpio_mode);
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_DAT_PIN, mhl_dat_gpio_mode);
#endif
    }
}
#endif

#if SPM_AEE_RR_REC
void spm_suspend_aee_init(void)
{
    aee_rr_rec_spm_suspend_val(0);
}
#endif

wake_reason_t spm_go_to_sleep(u32 spm_flags, u32 spm_data)
{
    u32 sec = 2;
    int wd_ret;
    struct wake_status wakesta;
    unsigned long flags;
    unsigned long temp_a, temp_b;
    struct mtk_irq_mask mask;
    struct wd_api *wd_api;
    static wake_reason_t last_wr = WR_NONE;
    struct pcm_desc *pcmdesc;
    struct pwr_ctrl *pwrctrl;
    
#if SPM_AEE_RR_REC
    spm_suspend_aee_init();
    aee_rr_rec_spm_suspend_val(1<<SPM_SUSPEND_ENTER);
#endif   
/*
    if(spm_set_suspend_pcm_ver(&spm_flags)==false) {
        spm_crit2("mempll setting error %x\n",mt_get_clk_mem_sel());
        last_wr = WR_UNKNOWN;
        return last_wr;
    }
*/
    pcmdesc = __spm_suspend.pcmdesc;
    pwrctrl = __spm_suspend.pwrctrl;

    set_pwrctrl_pcm_flags(pwrctrl, spm_flags);
    set_pwrctrl_pcm_data(pwrctrl, spm_data);

#if SPM_PWAKE_EN
    sec = spm_get_wake_period(-1 /* FIXME */, last_wr);
#endif
    pwrctrl->timer_val = sec * 32768;

    wd_ret = get_wd_api(&wd_api);
    if (!wd_ret)
        wd_api->wd_suspend_notify();

//FIXME: wait for golden setting
#if 0
    {
        extern int snapshot_golden_setting(const char *func, const unsigned int line);
        extern bool is_already_snap_shot;

        if (!is_already_snap_shot)
            snapshot_golden_setting(__FUNCTION__, 0);
    }
#endif

    //spm_suspend_pre_process(pwrctrl);

    spin_lock_irqsave(&__spm_lock, flags);
    mt_irq_mask_all(&mask);
    mt_irq_unmask_for_sleep(SPM_IRQ0_ID);

    mt_cirq_clone_gic();
    mt_cirq_enable();

    spm_set_sysclk_settle();
    
    spm_crit2("sec = %u, wakesrc = 0x%x (%u)(%u)\n",
              sec, pwrctrl->wake_src, is_cpu_pdn(pwrctrl->pcm_flags), is_infra_pdn(pwrctrl->pcm_flags));

    if (request_uart_to_sleep()) {
        last_wr = WR_UART_BUSY;
        goto RESTORE_IRQ;
    }

#if 0
    ///only for mhl i2s/uart share gpio issue on k2 platform
    uart_mhl_gpio_sleep_ctrl(true);
#endif

    __spm_reset_and_init_pcm(pcmdesc);

    __spm_kick_im_to_fetch(pcmdesc);

    __spm_init_pcm_register();

    __spm_init_event_vector(pcmdesc);

    __spm_set_power_control(pwrctrl);

    __spm_set_wakeup_event(pwrctrl);

    spm_kick_pcm_to_run(pwrctrl);
    
#if SPM_AEE_RR_REC
    aee_rr_rec_spm_suspend_val(aee_rr_curr_spm_suspend_val()|(1<<SPM_SUSPEND_ENTER_WFI));
#endif
    mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_SUSPEND);
     
    //pmic software workaround: D1/D2/D3 same pmic
    //extern U32 pmic_read_interface (U32 RegNum, U32 *val, U32 MASK, U32 SHIFT);
    //extern U32 pmic_config_interface (U32 RegNum, U32 val, U32 MASK, U32 SHIFT);
    temp_a = spm_read(PMIC_WRAP_DVFS_WDATA5);
    temp_b = spm_read(PMIC_WRAP_DVFS_WDATA4);
    spm_write(SPM_PCM_PASR_DPD_3, (temp_a << 16)|temp_b);
    temp_a = spm_read(PMIC_WRAP_DVFS_WDATA3);
    temp_b = spm_read(PMIC_WRAP_DVFS_WDATA2);
    spm_write(SPM_PCM_PASR_DPD_2, (temp_a << 16)|temp_b);
    pmic_read_interface(0x450, &temp_a, 0xFFFF, 0);
    pmic_read_interface(0x44c, &temp_b, 0xFFFF, 0);
    spm_write(PMIC_WRAP_DVFS_ADR2, 0x450);
    spm_write(PMIC_WRAP_DVFS_WDATA2, (temp_a & ~(0x3 << 6)) | 0x2 << 6);
    spm_write(PMIC_WRAP_DVFS_ADR3, 0x44C);
    spm_write(PMIC_WRAP_DVFS_WDATA3, temp_b & ~(0x3 <<7));
    spm_write(PMIC_WRAP_DVFS_ADR4, 0x450);
    spm_write(PMIC_WRAP_DVFS_WDATA4, temp_a);
    spm_write(PMIC_WRAP_DVFS_ADR5, 0x44C);
    spm_write(PMIC_WRAP_DVFS_WDATA5, temp_b);
  
    spm_trigger_wfi_for_sleep(pwrctrl);
    mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_NORMAL);
    
#if SPM_AEE_RR_REC
    aee_rr_rec_spm_suspend_val(aee_rr_curr_spm_suspend_val()|(1<<SPM_SUSPEND_LEAVE_WFI));
#endif

    __spm_get_wakeup_status(&wakesta);

    spm_clean_after_wakeup();

#if 0
    ///only for mhl i2s/uart share gpio issue on k2 platform
    uart_mhl_gpio_sleep_ctrl(false);
#endif

    request_uart_to_wakeup();

    last_wr = spm_output_wake_reason(&wakesta, pcmdesc);

RESTORE_IRQ:
    mt_cirq_flush();
    mt_cirq_disable();

    mt_irq_mask_restore(&mask);
    spin_unlock_irqrestore(&__spm_lock, flags);

    //spm_suspend_post_process(pwrctrl);

    if (!wd_ret)
        wd_api->wd_resume_notify();
#if SPM_AEE_RR_REC
    aee_rr_rec_spm_suspend_val(aee_rr_curr_spm_suspend_val()|(1<<SPM_SUSPEND_LEAVE));
#endif

#ifdef SPM_VCORE_EN
    spm_go_to_vcore_dvfs(SPM_VCORE_DVFS_EN, 0);
#endif

    return last_wr;
}

bool spm_is_md_sleep(void)
{
    return !( (spm_read(SPM_PCM_REG13_DATA) & R13_MD1_SRCLKENA) | (spm_read(SPM_PCM_REG13_DATA) & R13_MD2_SRCLKENA));
}

bool spm_is_md1_sleep(void)
{
    return !(spm_read(SPM_PCM_REG13_DATA) & R13_MD1_SRCLKENA);
}

bool spm_is_md2_sleep(void)
{
    return !(spm_read(SPM_PCM_REG13_DATA) & R13_MD2_SRCLKENA);
}

bool spm_is_conn_sleep(void)
{
    return !(spm_read(SPM_PCM_REG13_DATA) & R13_CONN_SRCLKENA);
}

void spm_set_wakeup_src_check(void)
{
    /* clean wakeup event raw status */
    spm_write(SPM_SLEEP_WAKEUP_EVENT_MASK, 0xFFFFFFFF);

    /* set wakeup event */
    spm_write(SPM_SLEEP_WAKEUP_EVENT_MASK, ~WAKE_SRC_FOR_SUSPEND);
}

bool spm_check_wakeup_src(void)
{
    u32 wakeup_src;

    /* check wanek event raw status */
    wakeup_src = spm_read(SPM_SLEEP_ISR_RAW_STA);
    
    if (wakeup_src)
    {
        spm_crit2("WARNING: spm_check_wakeup_src = 0x%x", wakeup_src);
        return 1;
    }
    else
        return 0;
}

void spm_poweron_config_set(void)
{
    unsigned long flags;

    spin_lock_irqsave(&__spm_lock, flags);
    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));
    spin_unlock_irqrestore(&__spm_lock, flags);
}

void spm_md32_sram_con(u32 value)
{
    unsigned long flags;

    spin_lock_irqsave(&__spm_lock, flags);
    /* enable register control */
    spm_write(SPM_MD32_SRAM_CON, value);
    spin_unlock_irqrestore(&__spm_lock, flags);
}

//FIXME: for K2 fpga early porting
#if 0
#define hw_spin_lock_for_ddrdfs()           \
do {                                        \
    spm_write(0xF0050090, 0x8000);          \
} while (!(spm_read(0xF0050090) & 0x8000))

#define hw_spin_unlock_for_ddrdfs()         \
    spm_write(0xF0050090, 0x8000)
#else
#define hw_spin_lock_for_ddrdfs()
#define hw_spin_unlock_for_ddrdfs()
#endif

void spm_ap_mdsrc_req(u8 set)
{
    unsigned long flags;
    u32 i = 0;
    u32 md_sleep = 0;

    if (set)
    {   
    spin_lock_irqsave(&__spm_lock, flags);

        if (spm_ap_mdsrc_req_cnt < 0)
        {
            spm_crit2("warning: set = %d, spm_ap_mdsrc_req_cnt = %d\n", set, spm_ap_mdsrc_req_cnt); 
            //goto AP_MDSRC_REC_CNT_ERR;
            spin_unlock_irqrestore(&__spm_lock, flags);
        }
        else
        {
        spm_ap_mdsrc_req_cnt++;

            hw_spin_lock_for_ddrdfs();
        spm_write(SPM_POWER_ON_VAL1, spm_read(SPM_POWER_ON_VAL1) | (1 << 17));
            hw_spin_unlock_for_ddrdfs();
    
            spin_unlock_irqrestore(&__spm_lock, flags);
    
        /* if md_apsrc_req = 1'b0, wait 26M settling time (3ms) */
        if (0 == (spm_read(SPM_PCM_REG13_DATA) & R13_MD1_APSRC_REQ))
        {
            md_sleep = 1;
            mdelay(3);
        }

        /* Check ap_mdsrc_ack = 1'b1 */
        while(0 == (spm_read(SPM_PCM_REG13_DATA) & R13_AP_MD1SRC_ACK))
        {
            if (10 > i++)
            {
                mdelay(1);
            }
            else
            {
                spm_crit2("WARNING: MD SLEEP = %d, spm_ap_mdsrc_req CAN NOT polling AP_MD1SRC_ACK\n", md_sleep);
                    //goto AP_MDSRC_REC_CNT_ERR;
                    break;
                }
            }
        }        
    }
    else
    {
        spin_lock_irqsave(&__spm_lock, flags);

        spm_ap_mdsrc_req_cnt--;

        if (spm_ap_mdsrc_req_cnt < 0)
        {
            spm_crit2("warning: set = %d, spm_ap_mdsrc_req_cnt = %d\n", set, spm_ap_mdsrc_req_cnt); 
            //goto AP_MDSRC_REC_CNT_ERR;
        }
        else
        {
        if (0 == spm_ap_mdsrc_req_cnt)
        {
                hw_spin_lock_for_ddrdfs();
            spm_write(SPM_POWER_ON_VAL1, spm_read(SPM_POWER_ON_VAL1) & ~(1 << 17));
                hw_spin_unlock_for_ddrdfs();
        }
    }

    spin_unlock_irqrestore(&__spm_lock, flags);
}

//AP_MDSRC_REC_CNT_ERR:
//    spin_unlock_irqrestore(&__spm_lock, flags);
}

void spm_output_sleep_option(void)
{
    spm_notice("PWAKE_EN:%d, PCMWDT_EN:%d, BYPASS_SYSPWREQ:%d, I2C_CHANNEL:%d\n",
               SPM_PWAKE_EN, SPM_PCMWDT_EN, SPM_BYPASS_SYSPWREQ, I2C_CHANNEL);
}



MODULE_DESCRIPTION("SPM-Sleep Driver v0.1");
