#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/ccs1200/tirtos_tivac_2_16_01_14/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/tidrivers_tivac_2_16_01_13/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/bios_6_45_02_31/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/ndk_2_25_00_09/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/uia_2_00_05_50/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/ns_1_11_00_10/packages;C:/ti/ccs1200/ccs/ccs_base;C:/Users/naleslie/Desktop/labs/ece3849_lab4_starter/.config
override XDCROOT = C:/ti/ccs1200/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/ccs1200/tirtos_tivac_2_16_01_14/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/tidrivers_tivac_2_16_01_13/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/bios_6_45_02_31/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/ndk_2_25_00_09/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/uia_2_00_05_50/packages;C:/ti/ccs1200/tirtos_tivac_2_16_01_14/products/ns_1_11_00_10/packages;C:/ti/ccs1200/ccs/ccs_base;C:/Users/naleslie/Desktop/labs/ece3849_lab4_starter/.config;C:/ti/ccs1200/xdctools_3_32_00_06_core/packages;..
HOSTOS = Windows
endif
