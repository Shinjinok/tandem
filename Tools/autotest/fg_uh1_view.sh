#!/bin/sh
#https://wiki.flightgear.org/Property_Tree/Native_Protocol_Slaving

AUTOTESTDIR=$(dirname $0)

nice fgfs \
    --native=socket,in,10,,5503,udp \
    --native=socket,out,10,,5502,udp \
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
