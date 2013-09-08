#!/bin/sh
./bin/qemu-system-mipsel -M sep612 -initrd ramdisk.img  -kernel smp-vmlinux -smp 2 -append "console=ttyS0" -serial stdio
