#!/bin/sh

ffplay -vf "transpose=2, scale=2*iw:-1" -i rtp://127.0.0.1:5000 -protocol_whitelist file,udp,crypto,data,rtp -fflags nobuffer