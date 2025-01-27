#!/bin/bash

fpc src/ufr.pp
sudo cp src/ufr.o src/ufr.ppu   /usr/lib/x86_64-linux-gnu/fpc/3.2.2/units/x86_64-linux/rtl-extra/

# fpc examples/pub_ii.pas