#
# Copyright (C) 2019 OpenWrt.org
#

SUBTARGET:=rtl8196e
BOARDNAME:=RTL8196e based boards
ARCH_PACKAGES:=realtek_lx43
CPU_TYPE:=lx43
KERNEL_PATCHVER:=4.14

define Target/Description
        Build firmware images for Realtek RTL8196E based boards.
endef


