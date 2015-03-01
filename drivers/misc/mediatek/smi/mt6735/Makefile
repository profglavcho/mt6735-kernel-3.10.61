include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y += smi_debug.o
obj-y += mmdvfs_mgr.o

ifeq ($(CONFIG_ARCH_MT6735),y)
obj-y += smi_common.o
endif

ifeq ($(CONFIG_ARCH_MT6735M),y)
obj-y += smi_common_6735m.o
endif

ifeq ($(CONFIG_ARCH_MT6753),y)
obj-y += smi_common.o
endif