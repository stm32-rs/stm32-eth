#!/usr/bin/env bash

export PATH=~/.cargo/bin:$PATH

xargo build --target=thumbv7em-none-eabihf --release \
      --features="board_stm32f429x" \
      --example=pktgen \
    || exit 1

killall openocd
sleep 0.1

BIN=target/thumbv7em-none-eabihf/release/examples/pktgen
openocd \
    -f /usr/share/openocd/scripts/interface/stlink-v2-1.cfg \
    -f /usr/share/openocd/scripts/target/stm32f4x.cfg \
    -c init \
    -c "reset halt" \
    -c "flash write_image erase $BIN" \
    -c "reset run" &
sleep 3
arm-none-eabi-gdb $BIN
