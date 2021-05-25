#!/bin/bash

userresources=$HOME/.Xresources
usermodmap=$HOME/.Xmodmap
sysresources=/etc/X11/xinit/.Xresources
sysmodmap=/etc/X11/xinit/.Xmodmap

# merge in defaults and keymaps

if [ -f $sysresources ]; then
    xrdb -merge $sysresources
fi

if [ -f $sysmodmap ]; then
    xmodmap $sysmodmap
fi

if [ -f "$userresources" ]; then
    xrdb -merge "$userresources"
fi

if [ -f "$usermodmap" ]; then
    xmodmap "$usermodmap"
fi

# start some nice programs

if [ -d /etc/X11/xinit/xinitrc.d ] ; then
 for f in /etc/X11/xinit/xinitrc.d/?*.sh ; do
  [ -x "$f" ] && . "$f"
 done
 unset f
fi

nohup picom &> ~/logs/picom.log &
nohup sh -c "sleep 3; npm --prefix ~/dev/cstv-placard start" &> ~/logs/placard.log &
nohup sh -c "FLASK_APP=~/dev/cstv-controller/cstv_controller/api.py python3 -m flask run -h 0.0.0.0" &> ~/logs/controller.log &

exec python3 ~/dev/cstv-wm/wallwm.py &> ~/logs/wm.log
