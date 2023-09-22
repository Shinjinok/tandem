#!/bin/sh
#https://wiki.flightgear.org/Property_Tree/Native_Protocol_Slaving

AUTOTESTDIR=$(dirname $0)

nice fgfs \
    --native-ctrls=socket,in,10,127.0.0.1,5600,udp \
    --native-ctrls=socket,out,10,127.0.0.1,5700,udp \
    --native-fdm=socket,out,10,127.0.0.1,5500,udp \
    --aircraft=UH1 \
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
