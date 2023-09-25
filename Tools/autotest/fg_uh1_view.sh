#!/bin/sh
#https://wiki.flightgear.org/Property_Tree/Native_Protocol_Slaving
#./sim_vehicle.py -v ArduCopter -f heli --model flightaxis:127.0.0.1

AUTOTESTDIR=$(dirname $0)

nice fgfs \
    --native-fdm=socket,in,1,,5001,udp \
    --native-fdm=socket,out,1,,5000,udp \
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
