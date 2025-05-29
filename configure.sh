#!/bin/bash

mkdir -p build
cd build
mkdir -p release
cd release
cmake -DRTAUDIO_API_JACK=OFF -DRTMIDI_API_JACK=OFF -DCMAKE_BUILD_TYPE=Release -Wno-deprecated -DBUILD_EXAMPLES=0 ../..