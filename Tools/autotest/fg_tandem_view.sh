#!/bin/sh

AUTOTESTDIR=$(dirname $0)

nice fgfs \
    --generic=socket,in,10,,5003,udp,MAVLink \
    --generic=socket,out,20,,5001,udp,MAVLink \
    --aircraft=ch47 \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --airport=PHNL \
    --geometry=650x550 \
    --bpp=32 \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
    $*