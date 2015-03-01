#include "cmdq_device.h"
#include "cmdq_core.h"
#include <mach/mt_irq.h>

/* device tree */
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>

typedef struct CmdqModuleBaseVA {
	long MMSYS_CONFIG;
	long MDP_RDMA;
	long MDP_RSZ0;
	long MDP_RSZ1;
	long MDP_WDMA;
	long MDP_WROT;
	long MDP_TDSHP;
	long MM_MUTEX;
	long VENC;
} CmdqModuleBaseVA;

typedef struct CmdqDeviceStruct {
	struct device *pDev;
	long regBaseVA;		/* considering 64 bit kernel, use long */
	long regBasePA;
	uint32_t irqId;
	uint32_t irqSecId;
} CmdqDeviceStruct;

static CmdqModuleBaseVA gCmdqModuleBaseVA;
static CmdqDeviceStruct gCmdqDev;

struct device *cmdq_dev_get(void)
{
	return gCmdqDev.pDev;
}

const uint32_t cmdq_dev_get_irq_id(void)
{
	return gCmdqDev.irqId;
}

const uint32_t cmdq_dev_get_irq_secure_id(void)
{
	return gCmdqDev.irqSecId;
}

const long cmdq_dev_get_module_base_VA_GCE(void)
{
	return gCmdqDev.regBaseVA;
}

const long cmdq_dev_get_module_base_PA_GCE(void)
{
	return gCmdqDev.regBasePA;
}

const long cmdq_dev_get_module_base_VA_MMSYS_CONFIG(void)
{
	return gCmdqModuleBaseVA.MMSYS_CONFIG;
}

const long cmdq_dev_get_module_base_VA_MDP_RDMA(void)
{
	return gCmdqModuleBaseVA.MDP_RDMA;
}

const long cmdq_dev_get_module_base_VA_MDP_RSZ0(void)
{
	return gCmdqModuleBaseVA.MDP_RSZ0;
}

const long cmdq_dev_get_module_base_VA_MDP_RSZ1(void)
{
	return gCmdqModuleBaseVA.MDP_RSZ1;
}

const long cmdq_dev_get_module_base_VA_MDP_WDMA(void)
{
	return gCmdqModuleBaseVA.MDP_WDMA;
}

const long cmdq_dev_get_module_base_VA_MDP_WROT(void)
{
	return gCmdqModuleBaseVA.MDP_WROT;
}

const long cmdq_dev_get_module_base_VA_MDP_TDSHP(void)
{
	return gCmdqModuleBaseVA.MDP_TDSHP;
}

const long cmdq_dev_get_module_base_VA_MM_MUTEX(void)
{
	return gCmdqModuleBaseVA.MM_MUTEX;
}

const long cmdq_dev_get_module_base_VA_VENC(void)
{
	return gCmdqModuleBaseVA.VENC;
}

const long cmdq_dev_alloc_module_base_VA_by_name(const char *name)
{
	unsigned long VA;
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, name);
	VA = (unsigned long)of_iomap(node, 0);
	CMDQ_LOG("DEV: VA(%s): 0x%lx\n", name, VA);

	return VA;
}

void cmdq_dev_free_module_base_VA(const long VA)
{
	iounmap((void*)VA);
}

void cmdq_dev_init_module_base_VA(void)
{
	memset(&gCmdqModuleBaseVA, 0, sizeof(CmdqModuleBaseVA));

#ifdef CMDQ_OF_SUPPORT
	gCmdqModuleBaseVA.MMSYS_CONFIG =
	    cmdq_dev_alloc_module_base_VA_by_name("mediatek,MMSYS_CONFIG");
	gCmdqModuleBaseVA.MDP_RDMA = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_RDMA");
	gCmdqModuleBaseVA.MDP_RSZ0 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_RSZ0");
	gCmdqModuleBaseVA.MDP_RSZ1 = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_RSZ1");
	gCmdqModuleBaseVA.MDP_WDMA = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_WDMA");
	gCmdqModuleBaseVA.MDP_WROT = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_WROT");
	gCmdqModuleBaseVA.MDP_TDSHP = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MDP_TDSHP");
	gCmdqModuleBaseVA.MM_MUTEX = cmdq_dev_alloc_module_base_VA_by_name("mediatek,MM_MUTEX");
	gCmdqModuleBaseVA.VENC = cmdq_dev_alloc_module_base_VA_by_name("mediatek,VENC");
#else
	gCmdqModuleBaseVA.MMSYS_CONFIG = MMSYS_CONFIG_BASE;
	gCmdqModuleBaseVA.MDP_RDMA = MDP_RDMA_BASE;
	gCmdqModuleBaseVA.MDP_RSZ0 = MDP_RSZ0_BASE;
	gCmdqModuleBaseVA.MDP_RSZ1 = MDP_RSZ1_BASE;
	gCmdqModuleBaseVA.MDP_WDMA = MDP_WDMA_BASE;
	gCmdqModuleBaseVA.MDP_WROT = MDP_WROT_BASE;
	gCmdqModuleBaseVA.MDP_TDSHP = MDP_TDSHP_BASE;
	gCmdqModuleBaseVA.MM_MUTEX = MMSYS_MUTEX_BASE;
	gCmdqModuleBaseVA.VENC = VENC_BASE;
#endif
}

void cmdq_dev_deinit_module_base_VA(void)
{
#ifdef CMDQ_OF_SUPPORT
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MMSYS_CONFIG());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_RDMA());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_RSZ0());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_RSZ1());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_WDMA());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_WROT());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MDP_TDSHP());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_MM_MUTEX());
	cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_VENC());

	memset(&gCmdqModuleBaseVA, 0, sizeof(CmdqModuleBaseVA));
#else
	/* do nothing, registers' IOMAP will be destoryed by platform */
#endif
}

void cmdq_dev_init(struct platform_device *pDevice)
{
	struct device_node *node = pDevice->dev.of_node;

	/* init cmdq device dependent data */
	do {
		memset(&gCmdqDev, 0x0, sizeof(CmdqDeviceStruct));

		gCmdqDev.pDev = &pDevice->dev;
#ifdef CMDQ_OF_SUPPORT
		gCmdqDev.regBaseVA = (unsigned long)of_iomap(node, 0);
		gCmdqDev.regBasePA = (0L | 0x10217000);
		gCmdqDev.irqId = irq_of_parse_and_map(node, 0);
		gCmdqDev.irqSecId = irq_of_parse_and_map(node, 1);
#else
		gCmdqDev.regBaseVA = (0L | GCE_BASE);
		gCmdqDev.regBasePA = (0L | 0x10217000);
		gCmdqDev.irqId = CQ_DMA_IRQ_BIT_ID;
		gCmdqDev.irqSecId = CQ_DMA_SEC_IRQ_BIT_ID;
#endif

		CMDQ_LOG
		    ("[CMDQ] platform_dev: dev: %p, PA: %lx, VA: %lx, irqId: %d,  irqSecId:%d\n",
		     gCmdqDev.pDev, gCmdqDev.regBasePA, gCmdqDev.regBaseVA, gCmdqDev.irqId,
		     gCmdqDev.irqSecId);
	} while (0);

	/* init module VA */
	cmdq_dev_init_module_base_VA();
}

void cmdq_dev_deinit(void)
{
	cmdq_dev_deinit_module_base_VA();

	/* deinit cmdq device dependent data */
	do {
#ifdef CMDQ_OF_SUPPORT
		cmdq_dev_free_module_base_VA(cmdq_dev_get_module_base_VA_GCE());
		gCmdqDev.regBaseVA = 0;
#else
		/* do nothing */
#endif
	} while (0);
}
