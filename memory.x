MEMORY
{
  FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 2048K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 112K
  RAM2 (xrw)      : ORIGIN = 0x20010000, LENGTH = 16K
  RAM3 (xrw)      : ORIGIN = 0x20020000, LENGTH = 64K
  CCMRAM (rw)     : ORIGIN = 0x10000000, LENGTH = 64K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
_eheap = _stack_start - 4K;
