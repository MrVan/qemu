#!/bin/sh
./bin/qemu-system-unicore32 -M sep6200 -serial stdio -d in_asm,op,out_asm
