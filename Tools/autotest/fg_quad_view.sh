#!/bin/sh

AUTOTESTDIR=$(dirname $0)

nice fgfs \
    --native-fdm=socket,in,1,,5003,udp \
    --native-fdm=socket,out,1,,5001,udp \
    --aircraft=arducopter \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --airport=YKRY \
    --geometry=650x550 \
    --bpp=32 \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-sound \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
    $*
