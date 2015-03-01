#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include <mach/mt_vcore_dvfs.h>
#include <mach/mt_cpufreq.h>

#include "mt_spm_internal.h"

#define PER_OPP_DVS_US		(100 + 50)

#define VCORE_STA_UHPM		(VCORE_STA_1 | VCORE_STA_0)
#define VCORE_STA_HPM		(VCORE_STA_1)

#define get_vcore_sta()		(spm_read(SPM_SLEEP_DVFS_STA) & (VCORE_STA_1 | VCORE_STA_0))

static const u32 vcore_dvfs_binary[] = {
	0x1840001f, 0x00000001, 0x1990001f, 0x10006b08, 0xe8208000, 0x10006b18,
	0x00000000, 0x1910001f, 0x100062c4, 0x1a00001f, 0x10006b0c, 0x1950001f,
	0x10006b0c, 0x80849001, 0xa1508805, 0x80851001, 0xa1500805, 0xe2000005,
	0x1910001f, 0x100062c4, 0x10c0041f, 0x80801001, 0x81600801, 0xa0df1403,
	0xa0df8803, 0xd80003e2, 0x17c07c1f, 0x80809001, 0x81600801, 0xa0d59403,
	0xa0d60803, 0x80841001, 0x81600801, 0xa0db9403, 0xa0de0803, 0x13000c1f,
	0xe8208000, 0x10006310, 0x0b1603f8, 0x1b80001f, 0x90100000, 0x88900001,
	0x10006814, 0xd8200242, 0x17c07c1f, 0x1910001f, 0x10006b0c, 0x1a00001f,
	0x100062c4, 0x1950001f, 0x100062c4, 0x80809001, 0x81748405, 0xa1548805,
	0x80801001, 0x81750405, 0xa1550805, 0xe2000005, 0x1ac0001f, 0x55aa55aa,
	0x1940001f, 0xaa55aa55, 0x1b80001f, 0x00001000, 0xf0000000, 0x81429801,
	0xd80009c5, 0x17c07c1f, 0x1a00001f, 0x10006604, 0x18c0001f, 0x10001138,
	0x1910001f, 0x10001138, 0xa1108404, 0xe0c00004, 0xc0c02200, 0xe2200007,
	0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4, 0xa0940402, 0xe2000002,
	0x1b00001f, 0x50000001, 0x1890001f, 0x100062c4, 0x80800402, 0xd8200ba2,
	0x17c07c1f, 0x1b00001f, 0x90000001, 0xf0000000, 0x17c07c1f, 0x1a00001f,
	0x100062c4, 0x1890001f, 0x100062c4, 0x81740402, 0xe2000005, 0x81429801,
	0xd8000e45, 0x17c07c1f, 0x1a00001f, 0x10006604, 0x18c0001f, 0x10001138,
	0x1910001f, 0x10001138, 0x81308404, 0xe0c00004, 0xc0c02200, 0xe2200007,
	0x1b00001f, 0x40801001, 0x80800402, 0xd8200f22, 0x17c07c1f, 0x1b00001f,
	0x80800001, 0xf0000000, 0x17c07c1f, 0x81441801, 0xd8201185, 0x17c07c1f,
	0x1a00001f, 0x10006604, 0xc0c02200, 0xe2200004, 0xc0c022e0, 0x17c07c1f,
	0xc0c02200, 0xe2200003, 0xc0c022e0, 0x17c07c1f, 0xc0c02200, 0xe2200002,
	0xc0c022e0, 0x17c07c1f, 0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4,
	0xa0908402, 0xe2000002, 0x1b00001f, 0x50001001, 0xf0000000, 0x17c07c1f,
	0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4, 0x80b08402, 0xe2000002,
	0x81441801, 0xd82015a5, 0x17c07c1f, 0x1a00001f, 0x10006604, 0xc0c02200,
	0xe2200003, 0xc0c022e0, 0x17c07c1f, 0xc0c02200, 0xe2200004, 0xc0c022e0,
	0x17c07c1f, 0xc0c02200, 0xe2200005, 0xc0c022e0, 0x17c07c1f, 0x1b00001f,
	0x00000801, 0xf0000000, 0x17c07c1f, 0x81441801, 0xd8201a65, 0x17c07c1f,
	0x1890001f, 0x10006b0c, 0x81400402, 0xd80017c5, 0x17c07c1f, 0x18d0001f,
	0x100063e8, 0x13000c1f, 0xd0001c00, 0x17c07c1f, 0x1880001f, 0x10006608,
	0x18d0001f, 0x10006608, 0x80f98403, 0x80fb8403, 0xe0800003, 0x1a00001f,
	0x10006604, 0xc0c02200, 0xe2200001, 0xc0c022e0, 0x17c07c1f, 0xc0c02200,
	0xe2200000, 0xc0c022e0, 0x17c07c1f, 0xc0c02200, 0xe2200006, 0xc0c022e0,
	0x17c07c1f, 0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4, 0xa1400402,
	0xe2000005, 0x1b00001f, 0x90000001, 0x80840801, 0xd8001c02, 0x17c07c1f,
	0x1b00001f, 0x80800001, 0xf0000000, 0x17c07c1f, 0x1a00001f, 0x100062c4,
	0x1890001f, 0x100062c4, 0x81400402, 0xd82021c5, 0x17c07c1f, 0x81441801,
	0xd8202025, 0x17c07c1f, 0x1a00001f, 0x10006604, 0xc0c02200, 0xe2200000,
	0xc0c022e0, 0x17c07c1f, 0xc0c02200, 0xe2200001, 0xc0c022e0, 0x17c07c1f,
	0xc0c02200, 0xe2200002, 0xc0c022e0, 0x17c07c1f, 0x1880001f, 0x10006608,
	0x18d0001f, 0x10006608, 0xa0d98403, 0xa0db8403, 0xe0800003, 0x1a00001f,
	0x100062c4, 0x1890001f, 0x100062c4, 0x81200402, 0xe2000004, 0x1b00001f,
	0x50000001, 0x80840801, 0xd80021c2, 0x17c07c1f, 0x1b00001f, 0x40801001,
	0xf0000000, 0x17c07c1f, 0x18d0001f, 0x10006604, 0x10cf8c1f, 0xd8202203,
	0x17c07c1f, 0xf0000000, 0x17c07c1f, 0x1092041f, 0x81499801, 0xd8202465,
	0x17c07c1f, 0xd8202822, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f,
	0x60000000, 0xd8002362, 0x00a00402, 0x814a1801, 0xd82025c5, 0x17c07c1f,
	0xd8202822, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0x80000000,
	0xd80024c2, 0x00a00402, 0x814a9801, 0xd8202725, 0x17c07c1f, 0xd8202822,
	0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0xc0000000, 0xd8002622,
	0x00a00402, 0xd8202822, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f,
	0x40000000, 0xd8002722, 0x00a00402, 0xf0000000, 0x17c07c1f
};
static struct pcm_desc vcore_dvfs_pcm = {
	.version	= "pcm_vcore_dvfs_v0.5.2_20150122",
	.base		= vcore_dvfs_binary,
	.size		= 323,
	.sess		= 1,
	.replace	= 1,
	.vec0		= EVENT_VEC(23, 1, 0, 65),	/* FUNC_MD_VRF18_WAKEUP */
	.vec1		= EVENT_VEC(28, 1, 0, 95),	/* FUNC_MD_VRF18_SLEEP */
	.vec2		= EVENT_VEC(11, 1, 0, 123),	/* FUNC_26M_WAKEUP */
	.vec3		= EVENT_VEC(12, 1, 0, 150),	/* FUNC_26M_SLEEP */
	.vec4		= EVENT_VEC(30, 1, 0, 177),	/* FUNC_UHPM_WAKEUP */
	.vec5		= EVENT_VEC(31, 1, 0, 226),	/* FUNC_UHPM_SLEEP */
};

static struct pwr_ctrl vcore_dvfs_ctrl = {
	.md1_req_mask		= 0,
	.conn_mask		= 0,
	.md_vrf18_req_mask_b	= 0,

	/* mask to avoid affecting apsrc_state */
	.md2_req_mask		= 1,
	.ccif0_to_ap_mask	= 1,
	.ccif0_to_md_mask	= 1,
	.ccif1_to_ap_mask	= 1,
	.ccif1_to_md_mask	= 1,
	.ccifmd_md1_event_mask	= 1,
	.ccifmd_md2_event_mask	= 1,
	.gce_req_mask		= 1,
	.disp_req_mask		= 1,
	.mfg_req_mask		= 1,
};

struct spm_lp_scen __spm_vcore_dvfs = {
	.pcmdesc	= &vcore_dvfs_pcm,
	.pwrctrl	= &vcore_dvfs_ctrl,
};

char *spm_dump_vcore_dvs_regs(char *p)
{
	if (p) {
		p += sprintf(p, "SLEEP_DVFS_STA: 0x%x\n", spm_read(SPM_SLEEP_DVFS_STA));
		p += sprintf(p, "PCM_SRC_REQ   : 0x%x\n", spm_read(SPM_PCM_SRC_REQ));
		p += sprintf(p, "PCM_REG13_DATA: 0x%x\n", spm_read(SPM_PCM_REG13_DATA));
		p += sprintf(p, "PCM_REG12_MASK: 0x%x\n", spm_read(SPM_PCM_REG12_MASK));
		p += sprintf(p, "PCM_REG12_DATA: 0x%x\n", spm_read(SPM_PCM_REG12_DATA));
		p += sprintf(p, "AP_STANBY_CON : 0x%x\n", spm_read(SPM_AP_STANBY_CON));
		p += sprintf(p, "PCM_FSM_STA   : 0x%x\n", spm_read(SPM_PCM_FSM_STA));
	} else {
		spm_err("[VcoreFS] SLEEP_DVFS_STA: 0x%x\n", spm_read(SPM_SLEEP_DVFS_STA));
		spm_err("[VcoreFS] PCM_SRC_REQ   : 0x%x\n", spm_read(SPM_PCM_SRC_REQ));
		spm_err("[VcoreFS] PCM_REG13_DATA: 0x%x\n", spm_read(SPM_PCM_REG13_DATA));
		spm_err("[VcoreFS] PCM_REG12_MASK: 0x%x\n", spm_read(SPM_PCM_REG12_MASK));
		spm_err("[VcoreFS] PCM_REG12_DATA: 0x%x\n", spm_read(SPM_PCM_REG12_DATA));
		spm_err("[VcoreFS] AP_STANBY_CON : 0x%x\n", spm_read(SPM_AP_STANBY_CON));
		spm_err("[VcoreFS] PCM_FSM_STA   : 0x%x\n", spm_read(SPM_PCM_FSM_STA));
	}

	return p;
}

#define wait_pcm_complete_dvs(condition, timeout)	\
({							\
	int i = 0;					\
	while (!(condition)) {				\
		if (i >= (timeout)) {			\
			i = -EBUSY;			\
			break;				\
		}					\
		udelay(1);				\
		i++;					\
	}						\
	i;						\
})

int spm_set_vcore_dvs_voltage(unsigned int opp)
{
	int r;
	unsigned long flags;

	spin_lock_irqsave(&__spm_lock, flags);
	switch (opp) {
		case OPP_0:
			spm_write(SPM_PCM_SRC_REQ, spm_read(SPM_PCM_SRC_REQ) | SR_PCM_APSRC_REQ | SR_PCM_F26M_REQ);
			r = wait_pcm_complete_dvs(get_vcore_sta() == VCORE_STA_UHPM,
						  3 * PER_OPP_DVS_US /* 1.15->1.05->1.15->1.25V */);
			break;
		case OPP_1:
			spm_write(SPM_PCM_SRC_REQ, (spm_read(SPM_PCM_SRC_REQ) & ~SR_PCM_APSRC_REQ) | SR_PCM_F26M_REQ);
			r = wait_pcm_complete_dvs(get_vcore_sta() == VCORE_STA_HPM,
						  2 * PER_OPP_DVS_US /* 1.15->1.05->1.15V */);
			break;
		case OPP_2:
			spm_write(SPM_PCM_SRC_REQ, spm_read(SPM_PCM_SRC_REQ) & ~(SR_PCM_APSRC_REQ | SR_PCM_F26M_REQ));
			r = 0;		/* unrequest, no need to wait */
			break;
		default:
			spm_crit("[VcoreFS] *** FAILED: OPP INDEX IS INCORRECT ***\n");
			spin_unlock_irqrestore(&__spm_lock, flags);
			return -EINVAL;
	}

	if (r >= 0) {	/* DVS pass */
		r = 0;
	} else {
		spm_dump_vcore_dvs_regs(NULL);
		BUG();	/* FIXME */
	}
	spin_unlock_irqrestore(&__spm_lock, flags);

	return r;
}

void spm_go_to_vcore_dvfs(u32 spm_flags, u32 spm_data)
{
	struct pcm_desc *pcmdesc = __spm_vcore_dvfs.pcmdesc;
	struct pwr_ctrl *pwrctrl = __spm_vcore_dvfs.pwrctrl;
	unsigned long flags;

	spin_lock_irqsave(&__spm_lock, flags);

	mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_NORMAL);

	set_pwrctrl_pcm_flags(pwrctrl, spm_flags);

	__spm_reset_and_init_pcm(pcmdesc);

	__spm_kick_im_to_fetch(pcmdesc);

	__spm_init_pcm_register();

	__spm_init_event_vector(pcmdesc);

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	__spm_kick_pcm_to_run(pwrctrl);

	spin_unlock_irqrestore(&__spm_lock, flags);
}

MODULE_DESCRIPTION("SPM-VCORE_DVFS Driver v0.1");
