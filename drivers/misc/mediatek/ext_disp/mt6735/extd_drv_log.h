#ifndef __EXTD_DRV_LOG_H__
#define __EXTD_DRV_LOG_H__

///for kernel
#include <linux/xlog.h>

/* HDMI log functions*/
#define HDMI_LOG(fmt, arg...) \
    do { \
        xlog_printk(ANDROID_LOG_WARN, "[EXTD][HDMI]", fmt, ##arg); \
    }while (0)

#define HDMI_FUNC()    \
    do { \
        xlog_printk(ANDROID_LOG_WARN, "[EXTD][HDMI]", "%s\n", __func__); \
    }while (0)


/*external display fence log fuctions*/
#define EXT_FENCE_FUNC() \
    do { \
        xlog_printk(ANDROID_LOG_WARN, "[EXTD][fence]", " %s\n", __func__); \
    }while (0)
#define EXTD_FENCE_LOG(fmt, arg...) \
    do { \
        xlog_printk(ANDROID_LOG_WARN, "[EXTD][fence]", fmt, ##arg); \
    }while (0)
#define EXTD_FENCE_ERR(fmt, arg...) \
    do { \
        xlog_printk(ANDROID_LOG_ERROR, "[EXTD][fence]", fmt, ##arg); \
    }while (0)
#define EXTD_FENCE_LOG_D(fmt, arg...) \
    do { \
        xlog_printk(ANDROID_LOG_DEBUG, "[EXTD][fence]", fmt, ##arg); \
    }while (0)
#define EXTD_FENCE_LOG_D_IF(con,fmt, arg...) \
    do { \
        if(con) xlog_printk(ANDROID_LOG_DEBUG, "[EXTD][fence]", fmt, ##arg); \
    }while (0)

/*external display lcm log fuctions*/
#define EXT_LCM_LOG(fmt, arg...) \
    do { \
        xlog_printk(ANDROID_LOG_WARN, "[EXTD][LCM]", fmt, ##arg); \
    }while (0)
#define EXT_LCM_ERR(fmt, arg...) \
    do { \
        xlog_printk(ANDROID_LOG_ERROR, "[EXTD][LCM]", fmt, ##arg); \
    }while (0)
#define EXT_LCM_FUNC() \
    do { \
        xlog_printk(ANDROID_LOG_WARN, "[EXTD][LCM]", " %s\n", __func__); \
    }while (0)

/*external display log functions*/
#define EXT_DISP_LOG(fmt, arg...) \
    do { \
        xlog_printk(ANDROID_LOG_DEBUG, "[EXTD][DISP]", fmt, ##arg); \
    }while (0)
#define EXT_DISP_ERR(fmt, arg...) \
    do { \
        xlog_printk(ANDROID_LOG_ERROR, "[EXTD][DISP]", fmt, ##arg); \
    }while (0)
#define EXT_DISP_FUNC() \
    do { \
        xlog_printk(ANDROID_LOG_WARN, "[EXTD][DISP] ", "%s\n", __func__); \
    }while (0)

#endif
