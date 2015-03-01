#ifndef __SMI_COMMON_H__
#define __SMI_COMMON_H__

#undef pr_fmt
#define pr_fmt(fmt) "[SMI]" fmt

#include <linux/aee.h>

#define SMIMSG(string, args...) if(1){\
 pr_warn("[pid=%d]"string, current->tgid, ##args); \
 }
#define SMIMSG2(string, args...) pr_warn(string, ##args)
#define SMITMP(string, args...) pr_warn("[pid=%d]"string, current->tgid, ##args)
#define SMIERR(string, args...) do{\
	pr_err("error: "string, ##args); \
	aee_kernel_warning(SMI_LOG_TAG, "error: "string, ##args);  \
}while(0)

#define smi_aee_print(string, args...) do{\
    char smi_name[100];\
    snprintf(smi_name,100, "["SMI_LOG_TAG"]"string, ##args); \
  aee_kernel_warning(smi_name, "["SMI_LOG_TAG"]error:"string,##args);  \
}while(0)


#define SMI_ERROR_ADDR  0
#define SMI_LARB_NR     4

#define SMI_LARB0_PORT_NUM  7
#define SMI_LARB1_PORT_NUM  7
#define SMI_LARB2_PORT_NUM  21
#define SMI_LARB3_PORT_NUM  13
#define SMI_LARB4_PORT_NUM  4

// Please use the function to instead gLarbBaseAddr to prevent the NULL pointer access error 
// when the corrosponding larb is not exist
// extern unsigned int gLarbBaseAddr[SMI_LARB_NR];
extern unsigned long get_larb_base_addr(int larb_id);
extern char *smi_port_name[][21];

extern void smi_dumpDebugMsg(void);

#define SMI_CLIENT_DISP 0
#define SMI_CLIENT_WFD 1

#define SMI_EVENT_DIRECT_LINK  ( 0x1 << 0 )
#define SMI_EVENT_DECOUPLE     ( 0x1 << 1 )
#define SMI_EVENT_OVL_CASCADE  ( 0x1 << 2 )
#define SMI_EVENT_OVL1_EXTERNAL  ( 0x1 << 3 )


extern void smi_client_status_change_notify(int module, int mode);
// module:
//      0: DISP
//      1: WFD
// mode:
//      DISP:
//      SMI_EVENT_DIRECT_LINK - directlink mode
//      SMI_EVENT_DECOUPLE - decouple mode
//      SMI_EVENT_OVL_CASCADE - OVL cascade
//      SMI_EVENT_OVL1_EXTERNAL - OVL 1 for external display

extern void SMI_DBG_Init(void);

#endif

