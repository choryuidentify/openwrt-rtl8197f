#!/bin/sh
#
# Copyright (C) 2009-2011 OpenWrt.org
#

realtek_board_detect() {
	local machine
	local name

	machine=$(awk 'BEGIN{FS="[ \t]+:[ \t]"} /machine/ {print $2}' /proc/cpuinfo)

	case "$machine" in
	"GENERIC")
		name="rlx"
		;;
	*"DIR-815 D1")
		name="dir-815-d1"
		;;
	*"GWR-300N V1")
		name="gwr-300n-v1"
		;;
	*"RE172 V1")
		name="re172-v1"
		;;
	*"RE708 V1")
		name="re708-v1"
		;;
	*"GWR1200AC V1")
		name="gwr1200ac-v1"
		;;
	*"GWR1200AC V2")
		name="gwr1200ac-v2"
		;;
	*"ACTIONRG1200 V1")
		name="actionrg1200-v1"
		;;
	*"ACTIONRF1200 V1")
		name="actionrf1200-v1"
		;;
	esac

	# use generic board detect if no name is set
	[ -z "$name" ] && return

	[ -e "/tmp/sysinfo/" ] || mkdir -p "/tmp/sysinfo/"

	echo "$name" > /tmp/sysinfo/board_name
	echo "$machine" > /tmp/sysinfo/model
}

