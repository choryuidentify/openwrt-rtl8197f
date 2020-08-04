#
# Copyright (C) 2008-2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

ARCH:=mips
BOARD:=realtek
BOARDNAME:=Realtek
SUBTARGETS:=rtl8197d rtl8196e rtl8197f
FEATURES:=squashfs gpio
MAINTAINER:=Anlix <contato@anlix.io>

KERNEL_PATCHVER:=4.14

define Target/Description
	Build firmware images for Realtek based boards.
endef

include $(INCLUDE_DIR)/target.mk

DEFAULT_PACKAGES += \
        kmod-gpio-button-hotplug swconfig

$(eval $(call BuildTarget))
