#
# Copyright (C) 2019 OpenWrt.org
#

SUBTARGET:=rtl8197d
BOARDNAME:=RTL8197d based boards
ARCH_PACKAGES:=realtek_lx53
CPU_TYPE:=lx53
KERNEL_PATCHVER:=4.14

define Target/Description
        Build firmware images for Realtek RTL8197D based boards.
endef


