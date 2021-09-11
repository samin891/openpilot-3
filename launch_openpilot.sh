#!/usr/bin/bash

ALIAS_CHECK=$(/usr/bin/grep gitpull /system/comma/home/.bash_profile)
GET_PROP=$(getprop persist.sys.timezone)

if [ "$ALIAS_CHECK" == "" ]; then
    sleep 3
    mount -o remount,rw /system
    echo "alias gi='/data/openpilot/selfdrive/assets/addon/script/gitpull.sh'" >> /system/comma/home/.bash_profile
    mount -o remount,r /system
fi

if [ "$GET_PROP" != "Asia/Seoul" ]; then
    setprop persist.sys.timezone Asia/Seoul
fi

export PASSIVE="0"
exec ./launch_chffrplus.sh

