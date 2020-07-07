#!/usr/bin/env bash

export PATH=~/.cargo/bin:$PATH

EXAMPLE=$1
if [[ "$EXAMPLE" = "" ]]; then
    echo "Run with one of the examples:"
    for f in examples/*.rs; do
        fn=`basename $f | sed -e 's/.rs//'`
        echo "- $fn"
    done
    exit 1
fi

cargo build --target=thumbv7em-none-eabihf --release \
      --features="stm32f429 smoltcp-phy smoltcp-log smoltcp-verbose" \
      --example=$EXAMPLE \
    || exit 1

killall openocd
sleep 0.1

BIN=target/thumbv7em-none-eabihf/release/examples/$EXAMPLE
openocd \
    -f /usr/share/openocd/scripts/interface/stlink-v2-1.cfg \
    -f /usr/share/openocd/scripts/target/stm32f4x.cfg \
    -c init \
    -c "reset halt" \
    -c "flash write_image erase $BIN" \
    -c "reset run" &
sleep 3
arm-none-eabi-gdb $BIN
