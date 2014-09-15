#!/bin/bash

prog="$1"

if [ $# -ne 1 ] || [ ! -f "${prog}" ]; then
    echo "Usage: $0 FILE.bin" 1>&2
    exit 1
fi

dfu-util -d 0483:df11 -a 0 -D "${prog}" -s 0x08000000
