# This makefile is included from vendor/intel/common/wifi/WifiRules.mk
$(eval $(call build_kernel_module,$(call my-dir)/,rtl8723bs,CONFIG_RTL8723B=m CONFIG_BT_COEXIST=y))
