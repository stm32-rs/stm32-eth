
./working:     file format elf32-littlearm
./working
architecture: armv7e-m, flags 0x00000112:
EXEC_P, HAS_SYMS, D_PAGED
start address 0x080001f9

Program Header:
    LOAD off    0x00010000 vaddr 0x08000000 paddr 0x08000000 align 2**16
         filesz 0x000001f8 memsz 0x000001f8 flags r--
    LOAD off    0x000101f8 vaddr 0x080001f8 paddr 0x080001f8 align 2**16
         filesz 0x00009740 memsz 0x00009740 flags r-x
    LOAD off    0x00019938 vaddr 0x08009938 paddr 0x08009938 align 2**16
         filesz 0x00001918 memsz 0x00001918 flags r--
    LOAD off    0x00020000 vaddr 0x20000000 paddr 0x0800b250 align 2**16
         filesz 0x00000038 memsz 0x00000038 flags rw-
    LOAD off    0x00030038 vaddr 0x20000038 paddr 0x20000038 align 2**16
         filesz 0x00000000 memsz 0x0000043c flags rw-
   STACK off    0x00000000 vaddr 0x00000000 paddr 0x00000000 align 2**0
         filesz 0x00000000 memsz 0x00000000 flags rw-
private flags = 0x5000400: [Version5 EABI] [hard-float ABI]

Sections:
Idx Name          Size      VMA       LMA       File off  Algn
  0 .vector_table 000001f8  08000000  08000000  00010000  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
  1 .text         00009740  080001f8  080001f8  000101f8  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
  2 .rodata       00001918  08009938  08009938  00019938  2**3
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
  3 .data         00000038  20000000  0800b250  00020000  2**2
                  CONTENTS, ALLOC, LOAD, DATA
  4 .gnu.sgstubs  00000000  0800b2a0  0800b2a0  00020040  2**5
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
  5 .bss          0000003c  20000038  20000038  00030038  2**3
                  ALLOC
  6 .uninit       00000400  20000074  20000074  00030038  2**2
                  ALLOC
  7 .defmt        00000042  00000000  00000000  00030038  2**0
                  CONTENTS, READONLY
  8 .debug_loc    00024d0f  00000000  00000000  0003007a  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
  9 .debug_abbrev 000015ad  00000000  00000000  00054d89  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 10 .debug_info   0006fc85  00000000  00000000  00056336  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 11 .debug_aranges 00001600  00000000  00000000  000c5fbb  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 12 .debug_ranges 0000ab30  00000000  00000000  000c75bb  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 13 .debug_str    0006aef4  00000000  00000000  000d20eb  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 14 .comment      0000008b  00000000  00000000  0013cfdf  2**0
                  CONTENTS, READONLY
 15 .ARM.attributes 00000038  00000000  00000000  0013d06a  2**0
                  CONTENTS, READONLY
 16 .debug_frame  00000eac  00000000  00000000  0013d0a4  2**2
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 17 .debug_line   00015a10  00000000  00000000  0013df50  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 18 .debug_pubnames 000002be  00000000  00000000  00153960  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 19 .debug_pubtypes 00000047  00000000  00000000  00153c1e  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
SYMBOL TABLE:
00000000 l    df *ABS*	00000000 ip.db5b21dddbb36540-cgu.3
08009918 l     F .text	00000018 HardFaultTrampoline
08000250 l     F .text	0000010c smoltcp::wire::arp::Repr::emit
08009998 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.8
080037fe l     F .text	00000032 core::slice::index::slice_index_fail
08009968 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.5
08009948 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.3
08009958 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.4
08009938 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.2
08009978 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.6
08009988 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.7
080099a8 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.9
080038fc l     F .text	00000028 core::panicking::panic_bounds_check
080099b8 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.10
0800035c l     F .text	00000dc0 smoltcp::iface::interface::InterfaceInner::dispatch_ip
08004476 l     F .text	00000026 defmt::export::acquire_header_and_release
08008426 l     F .text	0000013e smoltcp::iface::route::Routes::lookup
0800b00c l     O .rodata	0000002c .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.7
0800b038 l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.8
08003924 l     F .text	0000000c core::panicking::panic
0800815a l     F .text	000002cc smoltcp::wire::ip::checksum::data
08007c98 l     F .text	0000017e smoltcp::wire::ipv4::Repr::emit
0800aae0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.41
080062c4 l     F .text	00000224 smoltcp::wire::tcp::TcpOption::emit
0800ab70 l     O .rodata	00000024 .Lanon.ee555cc9a30366a9abf098ddf788bd84.55
0800ab94 l     O .rodata	00000024 .Lanon.ee555cc9a30366a9abf098ddf788bd84.56
08004454 l     F .text	00000022 defmt::export::acquire_and_header
080043b0 l     F .text	000000a4 defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format
0800a518 l     O .rodata	00000036 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.22
0800a550 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.24
0800b0b8 l     O .rodata	0000001d .Lanon.24a1e35971d85806a98c30bf10e26011.21
0800b0d8 l     O .rodata	00000010 .Lanon.24a1e35971d85806a98c30bf10e26011.22
0800a9ec l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.15
0800a9fc l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.17
0800ae0c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.15
0800ae4c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.21
0800adcc l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.5
0800adfc l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.11
0800adec l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.9
0800af58 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.12
0800af88 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.15
0800ab10 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.44
0800af78 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.14
0800ab00 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.43
0800af98 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.19
0800ab40 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.47
0800ab30 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.46
0800ab50 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.48
0800afa8 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.29
08004118 l     F .text	0000000e core::slice::copy_from_slice_impl::len_mismatch_fail
0800af68 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.13
0800abb8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.58
0800ace0 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.91
0800acc0 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.87
0800acd0 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.88
0800ab20 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.45
0800aaf0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.42
0800ad10 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.96
0800ad30 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.98
0800ad00 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.95
0800ad20 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.97
0800b098 l     O .rodata	00000010 .Lanon.24a1e35971d85806a98c30bf10e26011.17
0800606e l     F .text	00000086 core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>
0800aa0c l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.19
080037ea l     F .text	00000014 core::option::unwrap_failed
0800addc l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.8
0800ae3c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.19
0800ae5c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.26
0800ae1c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.16
0800b058 l     O .rodata	00000010 .Lanon.24a1e35971d85806a98c30bf10e26011.5
0800acf0 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.92
0800111c l     F .text	000023e4 smoltcp::iface::interface::Interface::poll
0800b0a8 l     O .rodata	00000010 .Lanon.24a1e35971d85806a98c30bf10e26011.19
08008564 l     F .text	00000372 smoltcp::iface::neighbor::Cache::fill
08007ab2 l     F .text	000001b6 smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply
08007f4a l     F .text	00000092 smoltcp::socket::udp::Socket::accepts
08007fdc l     F .text	0000017e smoltcp::socket::udp::Socket::process
08006240 l     F .text	00000084 <core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold
080065a8 l     F .text	0000121e smoltcp::socket::tcp::Socket::process
080077c6 l     F .text	00000096 smoltcp::socket::tcp::Socket::rst_reply
080064e8 l     F .text	000000c0 smoltcp::socket::tcp::Socket::seq_to_transmit
0800ac78 l     O .rodata	00000038 .Lanon.ee555cc9a30366a9abf098ddf788bd84.70
0800acb0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.71
080038e0 l     F .text	0000001c core::panicking::panic_fmt
0800b0f8 l     O .rodata	00000041 .Lanon.6cf9266b5dcbf9a66a3291302e8633f7.2
0800b13c l     O .rodata	00000010 .Lanon.6cf9266b5dcbf9a66a3291302e8633f7.4
08004152 l     F .text	00000026 core::option::expect_failed
0800b14c l     O .rodata	00000042 .Lanon.6cf9266b5dcbf9a66a3291302e8633f7.5
0800b190 l     O .rodata	00000010 .Lanon.6cf9266b5dcbf9a66a3291302e8633f7.6
0800b068 l     O .rodata	00000010 .Lanon.24a1e35971d85806a98c30bf10e26011.7
0800a9bc l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.20
0800a9dc l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.23
0800a560 l     O .rodata	00000016 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.25
0800a578 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.26
0800a988 l     O .rodata	00000022 .Lanon.02e056d4c889a085f771c01290e74e2f.18
0800a9ac l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.19
0800a958 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.13
0800a9cc l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.22
0800aa9c l     O .rodata	00000033 .Lanon.ee555cc9a30366a9abf098ddf788bd84.23
0800aad0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.25
0800a968 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.15
0800b078 l     O .rodata	00000010 .Lanon.24a1e35971d85806a98c30bf10e26011.14
0800ae2c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.17
0800aebc l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.12
080041b6 l     F .text	00000014 core::panicking::panic_const::panic_const_rem_by_zero
0800b088 l     O .rodata	00000010 .Lanon.24a1e35971d85806a98c30bf10e26011.16
0800a978 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.17
0800ae6c l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.21
0800ae7c l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.22
0800ae8c l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.23
0800ae9c l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.24
0800adac l     O .rodata	00000010 .Lanon.ee56ee3d56b2a23f692cb60d33ec53f4.8
0800ac48 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.67
0800aa2c l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.2
0800ac28 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.65
0800ac38 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.66
0800ab60 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.54
08003500 l     F .text	0000010c core::fmt::write
0800360c l     F .text	0000019c core::fmt::Formatter::pad_integral
080037a8 l     F .text	00000042 core::fmt::Formatter::pad_integral::write_prefix
0800a650 l     O .rodata	0000002b .Lanon.dc388a968c23fa524085dd94a83f5751.251
08003830 l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
0800385c l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
080038b4 l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
08003888 l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
08004016 l     F .text	00000102 core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt
080060f4 l     F .text	00000088 __rustc::rust_begin_unwind
08003930 l     F .text	00000014 <&T as core::fmt::Display>::fmt
08003944 l     F .text	00000652 core::fmt::Formatter::pad
08003f96 l     F .text	00000014 core::panicking::panic_const::panic_const_div_by_zero
0800a67b l     O .rodata	00000019 .Lanon.dc388a968c23fa524085dd94a83f5751.271
08003faa l     F .text	00000010 <&T as core::fmt::Debug>::fmt
08003fba l     F .text	0000005c core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt
0800a588 l     O .rodata	000000c8 .Lanon.dc388a968c23fa524085dd94a83f5751.14
08004126 l     F .text	0000002c core::slice::copy_from_slice_impl::len_mismatch_fail::do_panic::runtime
08004178 l     F .text	0000003e core::result::unwrap_failed
0800a694 l     O .rodata	00000039 .Lanon.dc388a968c23fa524085dd94a83f5751.273
080041ca l     F .text	000000ce <core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt
0800a6d4 l     O .rodata	00000018 .Lanon.dc388a968c23fa524085dd94a83f5751.363
0800a6ec l     O .rodata	00000010 .Lanon.dc388a968c23fa524085dd94a83f5751.365
0800a910 l     O .rodata	0000002b .Lanon.02e056d4c889a085f771c01290e74e2f.8
0800a6fc l     O .rodata	00000010 .Lanon.dc388a968c23fa524085dd94a83f5751.366
0800a70c l     O .rodata	00000010 .Lanon.dc388a968c23fa524085dd94a83f5751.369
08004298 l     F .text	0000001a <core::fmt::Error as core::fmt::Debug>::fmt
0800a6cd l     O .rodata	00000005 .Lanon.dc388a968c23fa524085dd94a83f5751.303
080042b2 l     F .text	00000032 <core::net::display_buffer::DisplayBuffer<_> as core::fmt::Write>::write_str
080042e4 l     F .text	000000a8 core::fmt::Write::write_char
0800438c l     F .text	00000018 core::fmt::Write::write_fmt
0800449c l     F .text	00000010 <defmt::export::FmtWrite as core::fmt::Write>::write_str
080044ac l     F .text	00000088 core::fmt::Write::write_char
08004534 l     F .text	00000018 core::fmt::Write::write_fmt
0800a72c l     O .rodata	00000018 .Lanon.ea5d5760b73fa5b90415fc99172c09b2.1
0800454c l     F .text	00000082 defmt_rtt::channel::Channel::blocking_write
080045ce l     F .text	0000005c defmt_rtt::channel::Channel::nonblocking_write
20000040 l     O .bss	00000034 .L_MergedGlobals
0800a745 l     O .rodata	0000001e .Lanon.1d905ed0c947f0695d8369569f054732.0
0800a764 l     O .rodata	00000010 .Lanon.1d905ed0c947f0695d8369569f054732.2
0800a774 l     O .rodata	00000010 .Lanon.1d905ed0c947f0695d8369569f054732.4
08004a9c l     F .text	000015b8 ip::__cortex_m_rt_main
0800b1d8 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.6
0800b0e8 l     O .rodata	00000010 .Lanon.24a1e35971d85806a98c30bf10e26011.24
0800a900 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.7
0800a93c l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.9
08007c68 l     F .text	0000002e <smoltcp::wire::ip::Address as core::fmt::Display>::fmt
0800adbc l     O .rodata	00000010 .Lanon.ee56ee3d56b2a23f692cb60d33ec53f4.14
0800a7a8 l     O .rodata	00000150 .Lanon.02e056d4c889a085f771c01290e74e2f.5
0800a8f8 l     O .rodata	00000006 .Lanon.02e056d4c889a085f771c01290e74e2f.6
080099c8 l     O .rodata	00000023 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.11
0800a454 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.13
0800a4a0 l     O .rodata	00000027 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.16
0800a4c8 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.17
0800a464 l     O .rodata	00000029 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.14
0800a4d8 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.18
0800a4f8 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.20
0800a508 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.21
0800af04 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.17
0800b1e8 l     O .rodata	00000027 .Lanon.4bc4785c9ada05daa616a70a22937c0f.10
0800b210 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.11
0800b1a0 l     O .rodata	00000028 .Lanon.4bc4785c9ada05daa616a70a22937c0f.3
0800b220 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.12
0800b230 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.13
0800b240 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.14
0800a490 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.15
0800b1c8 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.5
0800a4e8 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.19
0800a784 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.2
0800a794 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.3
08006054 l     F .text	0000001a <stm32_eth::mac::WrongClock as core::fmt::Debug>::fmt
0800a94c l     O .rodata	0000000a .Lanon.02e056d4c889a085f771c01290e74e2f.10
20000038 l     O .bss	00000001 panic_probe::imp::panic::PANICKED
0800aa1c l     O .rodata	00000010 .Lanon.6fd57bafcc6bbd1a0f0c6a4a6d894bad.0
0800a744 l     O .rodata	00000001 .Lanon.ea5d5760b73fa5b90415fc99172c09b2.2
0800617c l     F .text	00000018 panic_probe::hard_fault
08006194 l     F .text	000000ac <&T as core::fmt::Display>::fmt
0800a71c l     O .rodata	0000000c .Lanon.dc388a968c23fa524085dd94a83f5751.408
0800a728 l     O .rodata	00000002 .Lanon.dc388a968c23fa524085dd94a83f5751.409
0800ac58 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.68
0800ac68 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.69
0800abd8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.60
0800ac08 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.63
0800aa8c l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.6
0800aa7c l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.4
0800abf8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.62
0800ac18 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.64
0800abc8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.59
0800abe8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.61
0800ad40 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.29
08007e7a l     F .text	00000066 defmt::export::fmt
08007e4c l     F .text	0000002e defmt::export::fmt
0800ad60 l     O .rodata	00000001 .Lanon.c1140054cd359bfdcd454e5b7dad9548.32
08007e16 l     F .text	00000036 defmt::export::fmt
08007a64 l     F .text	0000004e smoltcp::socket::tcp::Socket::immediate_ack_to_transmit
0800785c l     F .text	00000208 smoltcp::socket::tcp::Socket::ack_reply
0800ad50 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.31
0800ad9c l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.36
0800aecc l     O .rodata	00000025 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.14
0800aef4 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.15
0800aeac l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.11
0800ad61 l     O .rodata	00000028 .Lanon.c1140054cd359bfdcd454e5b7dad9548.33
0800ad8c l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.35
0800b048 l     O .rodata	00000010 .Lanon.d4b94c958ea1f8000f4d53dc8b85c559.8
08007ee0 l     F .text	0000006a smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with
0800af48 l     O .rodata	00000010 .Lanon.c1510acb27196742b5d924b6a5d99da6.17
0800afb8 l     O .rodata	00000023 .Lanon.f0ac2cabe1edd51d584ebf3fae05aeec.16
0800afdc l     O .rodata	00000010 .Lanon.f0ac2cabe1edd51d584ebf3fae05aeec.18
0800afec l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.4
0800af14 l     O .rodata	00000022 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.20
0800af38 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.22
0800affc l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.6
20000000 l     O .data	00000006 defmt_rtt::handle::NAME
20000074 l     O .uninit	00000400 defmt_rtt::handle::BUFFER
20000040 l     O .bss	00000001 defmt_rtt::CS_RESTORE
20000041 l     O .bss	00000001 defmt_rtt::TAKEN
20000042 l     O .bss	00000001 defmt_rtt::ENCODER
20000043 l     O .bss	00000001 defmt_rtt::ENCODER
20000044 l     O .bss	00000001 defmt_rtt::ENCODER
20000045 l     O .bss	00000001 cortex_m::peripheral::TAKEN
20000048 l     O .bss	00000008 ip::TIME
20000050 l     O .bss	0000000c stm32_eth::dma::EthernetDMA::rx_waker::WAKER
2000005c l     O .bss	0000000c stm32_eth::dma::EthernetDMA::tx_waker::WAKER
20000068 l     O .bss	0000000c stm32_eth::ptp::EthernetPTP::waker::WAKER
00000000 l    df *ABS*	00000000 lib.f2c86790-cgu.0
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.189
080088fa l     F .text	000000ae .hidden __aeabi_memclr8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.250
080089a8 l     F .text	0000009c .hidden __aeabi_memcpy4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.012
08008a44 l     F .text	000002e6 .hidden compiler_builtins::mem::memcpy
08008d2a l     F .text	00000618 .hidden compiler_builtins::mem::memmove
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.275
08009342 l     F .text	0000000c .hidden __aeabi_memmove8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.279
0800934e l     F .text	000000ae .hidden __aeabi_memclr4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.288
080093fc l     F .text	0000000c .hidden __aeabi_memcpy
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.320
08009408 l     F .text	0000009c .hidden __aeabi_memcpy8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.373
080094a4 l     F .text	000000d0 .hidden __aeabi_memclr
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.413
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.362
08009588 l     F .text	00000030 .hidden __udivmoddi4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.017
080095b8 l     F .text	00000360 .hidden compiler_builtins::int::specialized_div_rem::u64_div_rem
08000004 g     O .vector_table	00000004 __RESET_VECTOR
080001f8 g     F .text	0000003e Reset
08000008 g     O .vector_table	00000038 __EXCEPTIONS
080043a4 g     F .text	00000000 DefaultHandler
08000040 g     O .vector_table	000001b8 __INTERRUPTS
0800462a g     F .text	0000009e _defmt_acquire
080046c8 g     F .text	000000f0 _defmt_release
080043aa g     F .text	00000006 __defmt_default_timestamp
08009930 g     F .text	00000000 HardFault
080043aa g     F .text	00000000 __pre_init
08004a94 g     F .text	00000008 main
00000022 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"Enable the `proto-ipv4-fragmentation` feature for fragmentation support.","disambiguator":"5448626954131158173","crate_name":"smoltcp"}
00000024 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"address {} not in neighbor cache, sending ARP request","disambiguator":"12066069798907769471","crate_name":"smoltcp"}
00000007 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=__internal_FormatSequence}","disambiguator":"8408127155438382822","crate_name":"defmt"}
080047b8 g     F .text	0000019c _defmt_write
0000002f g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_error","data":"EthRxToken: RX would block","disambiguator":"7442772949220635545","crate_name":"stm32_eth"}
0000002a g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_debug","data":"Failed to receive packet: {}","disambiguator":"11700474013454779557","crate_name":"stm32_eth"}
00000041 g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_derived","data":"Truncated|DmaError|WouldBlock","disambiguator":"12700258740049707074","crate_name":"stm32_eth"}
00000026 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"arp: non-unicast source address","disambiguator":"1901301005436061336","crate_name":"smoltcp"}
00000028 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"arp: unknown operation code","disambiguator":"12440323745247841462","crate_name":"smoltcp"}
00000029 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"non-unicast or unspecified source address","disambiguator":"6766392025179650790","crate_name":"smoltcp"}
00000027 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"arp: source IP address not in same network as us","disambiguator":"8412082944703312747","crate_name":"smoltcp"}
00000023 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"Failed to send response: {:?}","disambiguator":"11552764603065337685","crate_name":"smoltcp"}
0000003e g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_derived","data":"NoRoute|NeighborPending","disambiguator":"17160438304021938019","crate_name":"smoltcp"}
0000000a g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"{}:{}:{}:{}: parsed window scaling factor >14, setting to 14","disambiguator":"8158328147869412070","crate_name":"smoltcp"}
0000003f g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":"{:?}","disambiguator":"10833894435547768834","crate_name":"smoltcp"}
00000004 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=u16}","disambiguator":"8239863257445217352","crate_name":"defmt"}
00000009 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"sACK with >3 blocks, truncating to 3","disambiguator":"15703149373311970542","crate_name":"smoltcp"}
00000019 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"timeout exceeded","disambiguator":"2000748738221331933","crate_name":"smoltcp"}
00000025 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"failed to transmit IP: device exhausted","disambiguator":"9389076079625704121","crate_name":"smoltcp"}
00000015 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"retransmitting at t+{}","disambiguator":"10495559254336700149","crate_name":"smoltcp"}
00000040 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":"{}.{:03}s","disambiguator":"5292232958402097832","crate_name":"smoltcp"}
00000005 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=u64}","disambiguator":"2959124217660265969","crate_name":"defmt"}
08009574  w    F .text	00000014 __aeabi_uldivmod
080088f0 g     F .text	00000006 __primask_r
080088d6 g     F .text	00000004 __cpsid
080088da g     F .text	00000004 __cpsie
080043a4 g     F .text	00000006 DefaultHandler_
080043aa g     F .text	00000006 DefaultPreInit
08009930 g     F .text	00000006 HardFault_
00000032 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_write","data":"{}.{}.{}.{}","disambiguator":"14742771906021295210","crate_name":"defmt"}
00000002 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=u8}","disambiguator":"16912398427198947700","crate_name":"defmt"}
080043aa g     F .text	00000000 _defmt_timestamp
20000008 g     O .data	00000030 _SEGGER_RTT
08004954 g     F .text	0000010e ETH
08004a62 g     F .text	00000032 SysTick
080088de g     F .text	0000000c __delay
080088ea g     F .text	00000006 __dsb
0000002b g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Enabling ethernet...","disambiguator":"10404236363425019797","crate_name":"ip"}
0000002c g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Listening at {}:80...","disambiguator":"3686426055769507929","crate_name":"ip"}
0000002d g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Transmitted hello! Closing socket...","disambiguator":"11635372039424199218","crate_name":"ip"}
0000002e g     O .defmt	00000001 {"package":"panic-probe","tag":"defmt_error","data":"{}","disambiguator":"3281125884690968605","crate_name":"panic_probe"}
00000001 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=__internal_Display}","disambiguator":"14158116662542268607","crate_name":"defmt"}
080088f6 g     F .text	00000004 __udf
0000001d g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"unacceptable RST|ACK in response to initial SYN","disambiguator":"12010015513314917541","crate_name":"smoltcp"}
0000001e g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"unacceptable SYN|ACK in response to initial SYN","disambiguator":"2348323649613492752","crate_name":"smoltcp"}
0000000e g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"expecting a SYN|ACK, received an ACK with the right ack_number, ignoring.","disambiguator":"2859068138203652258","crate_name":"smoltcp"}
0000000d g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"expecting a SYN|ACK","disambiguator":"6334425642103298512","crate_name":"smoltcp"}
00000011 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"non-zero-length segment with zero receive window, will only send an ACK","disambiguator":"6676005152151262999","crate_name":"smoltcp"}
0000001b g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"unacceptable ACK in response to SYN|ACK","disambiguator":"3989064690652622681","crate_name":"smoltcp"}
0000001c g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"unacceptable RST (expecting RST|ACK) in response to initial SYN","disambiguator":"12592444965224528503","crate_name":"smoltcp"}
00000012 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"received a keep-alive or window probe packet, will send an ACK","disambiguator":"5323319599099390235","crate_name":"smoltcp"}
00000010 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"expecting an ACK","disambiguator":"3625140730523199000","crate_name":"smoltcp"}
00000016 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"segment not in receive window ({}..{} not intersecting {}..{}), will send challenge ACK","disambiguator":"15410474912840077765","crate_name":"smoltcp"}
0000000f g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"expecting a SYN|ACK, received an ACK with the wrong ack_number, sending RST.","disambiguator":"11067998957997480673","crate_name":"smoltcp"}
00000021 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"zero-length segment not inside zero-length window, will send an ACK.","disambiguator":"17620149882924763407","crate_name":"smoltcp"}
0000001a g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"unacceptable ACK ({} not in {}...{})","disambiguator":"1850591223312697649","crate_name":"smoltcp"}
00000020 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"zero-length segment not inside window, will send an ACK.","disambiguator":"3646786452687534024","crate_name":"smoltcp"}
0000001f g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"unexpected packet {}","disambiguator":"12823780021606470319","crate_name":"smoltcp"}
0000003c g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":"TCP src={} dst={}","disambiguator":"1030315939857595793","crate_name":"smoltcp"}
00000037 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":" psh","disambiguator":"9481317621797126725","crate_name":"smoltcp"}
00000038 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":" rst","disambiguator":"10013211076911544330","crate_name":"smoltcp"}
0000003a g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":" syn","disambiguator":"10838285941484427392","crate_name":"smoltcp"}
00000034 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":" fin","disambiguator":"17565740076167230561","crate_name":"smoltcp"}
00000039 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":" seq={}","disambiguator":"4537005165886293247","crate_name":"smoltcp"}
0000003d g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":"{}","disambiguator":"5146805914163525621","crate_name":"smoltcp"}
00000008 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=u32}","disambiguator":"11041816430350983556","crate_name":"defmt"}
00000033 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":" ack={}","disambiguator":"3786093232229792722","crate_name":"smoltcp"}
0000003b g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":" win={}","disambiguator":"3473245040867372688","crate_name":"smoltcp"}
00000035 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":" len={}","disambiguator":"9057084416632356930","crate_name":"smoltcp"}
00000006 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=usize}","disambiguator":"8559056741933462175","crate_name":"defmt"}
00000036 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_write","data":" mss={}","disambiguator":"10911176379477047732","crate_name":"smoltcp"}
0000000c g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"duplicate ACK ({} not in {}...{})","disambiguator":"12380727397643179295","crate_name":"smoltcp"}
00000013 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"received duplicate ACK for seq {} (duplicate nr {}{})","disambiguator":"5695652426937161345","crate_name":"smoltcp"}
00000018 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"started fast retransmit","disambiguator":"3241634950913198545","crate_name":"smoltcp"}
00000014 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"reset duplicate ACK count","disambiguator":"8376999530309766497","crate_name":"smoltcp"}
0000000b g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"assembler: too many holes to add {} octets at offset {}","disambiguator":"11422860642586057134","crate_name":"smoltcp"}
00000017 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"sending sACK option with current assembler ranges","disambiguator":"17647954063314049668","crate_name":"smoltcp"}
00000003 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=str}","disambiguator":"11239111700719193807","crate_name":"defmt"}
080043a4 g     F .text	00000000 NonMaskableInt
080043a4 g     F .text	00000000 MemoryManagement
080043a4 g     F .text	00000000 BusFault
080043a4 g     F .text	00000000 UsageFault
080043a4 g     F .text	00000000 SVCall
080043a4 g     F .text	00000000 DebugMonitor
080043a4 g     F .text	00000000 PendSV
00000030 g     O .defmt	00000001 _defmt_encoding_ = rzcobs
00000031 g     O .defmt	00000001 _defmt_version_ = 4
080043a4 g     F .text	00000000 WWDG
080043a4 g     F .text	00000000 PVD
080043a4 g     F .text	00000000 TAMP_STAMP
080043a4 g     F .text	00000000 RTC_WKUP
080043a4 g     F .text	00000000 FLASH
080043a4 g     F .text	00000000 RCC
080043a4 g     F .text	00000000 EXTI0
080043a4 g     F .text	00000000 EXTI1
080043a4 g     F .text	00000000 EXTI2
080043a4 g     F .text	00000000 EXTI3
080043a4 g     F .text	00000000 EXTI4
080043a4 g     F .text	00000000 DMA1_STREAM0
080043a4 g     F .text	00000000 DMA1_STREAM1
080043a4 g     F .text	00000000 DMA1_STREAM2
080043a4 g     F .text	00000000 DMA1_STREAM3
080043a4 g     F .text	00000000 DMA1_STREAM4
080043a4 g     F .text	00000000 DMA1_STREAM5
080043a4 g     F .text	00000000 DMA1_STREAM6
080043a4 g     F .text	00000000 ADC
080043a4 g     F .text	00000000 CAN1_TX
080043a4 g     F .text	00000000 CAN1_RX0
080043a4 g     F .text	00000000 CAN1_RX1
080043a4 g     F .text	00000000 CAN1_SCE
080043a4 g     F .text	00000000 EXTI9_5
080043a4 g     F .text	00000000 TIM1_BRK_TIM9
080043a4 g     F .text	00000000 TIM1_UP_TIM10
080043a4 g     F .text	00000000 TIM1_TRG_COM_TIM11
080043a4 g     F .text	00000000 TIM1_CC
080043a4 g     F .text	00000000 TIM2
080043a4 g     F .text	00000000 TIM3
080043a4 g     F .text	00000000 TIM4
080043a4 g     F .text	00000000 I2C1_EV
080043a4 g     F .text	00000000 I2C1_ER
080043a4 g     F .text	00000000 I2C2_EV
080043a4 g     F .text	00000000 I2C2_ER
080043a4 g     F .text	00000000 SPI1
080043a4 g     F .text	00000000 SPI2
080043a4 g     F .text	00000000 USART1
080043a4 g     F .text	00000000 USART2
080043a4 g     F .text	00000000 USART3
080043a4 g     F .text	00000000 EXTI15_10
080043a4 g     F .text	00000000 RTC_ALARM
080043a4 g     F .text	00000000 OTG_FS_WKUP
080043a4 g     F .text	00000000 TIM8_BRK_TIM12
080043a4 g     F .text	00000000 TIM8_UP_TIM13
080043a4 g     F .text	00000000 TIM8_TRG_COM_TIM14
080043a4 g     F .text	00000000 TIM8_CC
080043a4 g     F .text	00000000 DMA1_STREAM7
080043a4 g     F .text	00000000 FMC
080043a4 g     F .text	00000000 SDMMC1
080043a4 g     F .text	00000000 TIM5
080043a4 g     F .text	00000000 SPI3
080043a4 g     F .text	00000000 UART4
080043a4 g     F .text	00000000 UART5
080043a4 g     F .text	00000000 TIM6_DAC
080043a4 g     F .text	00000000 TIM7
080043a4 g     F .text	00000000 DMA2_STREAM0
080043a4 g     F .text	00000000 DMA2_STREAM1
080043a4 g     F .text	00000000 DMA2_STREAM2
080043a4 g     F .text	00000000 DMA2_STREAM3
080043a4 g     F .text	00000000 DMA2_STREAM4
080043a4 g     F .text	00000000 ETH_WKUP
080043a4 g     F .text	00000000 CAN2_TX
080043a4 g     F .text	00000000 CAN2_RX0
080043a4 g     F .text	00000000 CAN2_RX1
080043a4 g     F .text	00000000 CAN2_SCE
080043a4 g     F .text	00000000 OTG_FS
080043a4 g     F .text	00000000 DMA2_STREAM5
080043a4 g     F .text	00000000 DMA2_STREAM6
080043a4 g     F .text	00000000 DMA2_STREAM7
080043a4 g     F .text	00000000 USART6
080043a4 g     F .text	00000000 I2C3_EV
080043a4 g     F .text	00000000 I2C3_ER
080043a4 g     F .text	00000000 OTG_HS_EP1_OUT
080043a4 g     F .text	00000000 OTG_HS_EP1_IN
080043a4 g     F .text	00000000 OTG_HS_WKUP
080043a4 g     F .text	00000000 OTG_HS
080043a4 g     F .text	00000000 DCMI
080043a4 g     F .text	00000000 CRYP
080043a4 g     F .text	00000000 HASH_RNG
080043a4 g     F .text	00000000 FPU
080043a4 g     F .text	00000000 UART7
080043a4 g     F .text	00000000 UART8
080043a4 g     F .text	00000000 SPI4
080043a4 g     F .text	00000000 SPI5
080043a4 g     F .text	00000000 SPI6
080043a4 g     F .text	00000000 SAI1
080043a4 g     F .text	00000000 LTDC
080043a4 g     F .text	00000000 LTDC_ER
080043a4 g     F .text	00000000 DMA2D
080043a4 g     F .text	00000000 SAI2
080043a4 g     F .text	00000000 QUADSPI
080043a4 g     F .text	00000000 LP_TIMER1
080043a4 g     F .text	00000000 HDMI_CEC
080043a4 g     F .text	00000000 I2C4_EV
080043a4 g     F .text	00000000 I2C4_ER
080043a4 g     F .text	00000000 SPDIFRX
080043a4 g     F .text	00000000 DSIHOST
080043a4 g     F .text	00000000 DFSDM1_FLT0
080043a4 g     F .text	00000000 DFSDM1_FLT1
080043a4 g     F .text	00000000 DFSDM1_FLT2
080043a4 g     F .text	00000000 DFSDM1_FLT3
080043a4 g     F .text	00000000 SDMMC2
080043a4 g     F .text	00000000 CAN3_TX
080043a4 g     F .text	00000000 CAN3_RX0
080043a4 g     F .text	00000000 CAN3_RX1
080043a4 g     F .text	00000000 CAN3_SCE
080043a4 g     F .text	00000000 JPEG
080043a4 g     F .text	00000000 MDIOS
20000038 g       .bss	00000000 __sbss
20000074 g       .bss	00000000 __ebss
20000000 g       .data	00000000 __sdata
20000038 g       .data	00000000 __edata
0800b250 g       *ABS*	00000000 __sidata
20000046 g     O .bss	00000001 DEVICE_PERIPHERALS
20008000 g       *ABS*	00000000 _stack_start
080001f8 g       .vector_table	00000000 _stext
20000474 g       .uninit	00000000 __sheap
08000008 g       .vector_table	00000000 __reset_vector
08000040 g       .vector_table	00000000 __eexceptions
00000042 g       .defmt	00000000 __DEFMT_MARKER_END
20000474 g       .uninit	00000000 __euninit
08000000 g       .vector_table	00000000 __vector_table
080001f8 g       .text	00000000 __stext
08009938 g       .text	00000000 __etext
08009938 g       .rodata	00000000 __srodata
0800b250 g       .rodata	00000000 __erodata
0800b2a0 g       .gnu.sgstubs	00000000 __veneer_base
0800b2a0 g       .gnu.sgstubs	00000000 __veneer_limit
20000074 g       .uninit	00000000 __suninit
00000009 g       .defmt	00000000 __DEFMT_MARKER_TRACE_START
00000009 g       .defmt	00000000 __DEFMT_MARKER_TRACE_END
00000009 g       .defmt	00000000 __DEFMT_MARKER_DEBUG_START
0000002b g       .defmt	00000000 __DEFMT_MARKER_DEBUG_END
0000002b g       .defmt	00000000 __DEFMT_MARKER_INFO_START
0000002e g       .defmt	00000000 __DEFMT_MARKER_INFO_END
0000002e g       .defmt	00000000 __DEFMT_MARKER_WARN_START
0000002e g       .defmt	00000000 __DEFMT_MARKER_WARN_END
0000002e g       .defmt	00000000 __DEFMT_MARKER_ERROR_START
00000030 g       .defmt	00000000 __DEFMT_MARKER_ERROR_END



Disassembly of section .text:

<Reset>:
	f004 f8d7 	bl	<DefaultPreInit>
	480e      	ldr	r0, [pc, #56]	@ (<Reset+0x40>)
	490f      	ldr	r1, [pc, #60]	@ (<Reset+0x44>)
	2200      	movs	r2, #0
	4281      	cmp	r1, r0
	d001      	beq.n	<Reset+0x12>
	c004      	stmia	r0!, {r2}
	e7fb      	b.n	<Reset+0xa>
	480d      	ldr	r0, [pc, #52]	@ (<Reset+0x48>)
	490d      	ldr	r1, [pc, #52]	@ (<Reset+0x4c>)
	4a0e      	ldr	r2, [pc, #56]	@ (<Reset+0x50>)
	4281      	cmp	r1, r0
	d002      	beq.n	<Reset+0x22>
	ca08      	ldmia	r2!, {r3}
	c008      	stmia	r0!, {r3}
	e7fa      	b.n	<Reset+0x18>
	480c      	ldr	r0, [pc, #48]	@ (<Reset+0x54>)
	f44f 0170 	mov.w	r1, #15728640	@ 0xf00000
	6802      	ldr	r2, [r0, #0]
	ea42 0201 	orr.w	r2, r2, r1
	6002      	str	r2, [r0, #0]
	f3bf 8f4f 	dsb	sy
	f3bf 8f6f 	isb	sy
	f004 fc30 	bl	<main>
	de00      	udf	#0
	0000      	movs	r0, r0
	20000038 	.word	0x20000038
	20000074 	.word	0x20000074
	20000000 	.word	0x20000000
	20000038 	.word	0x20000038
	0800b250 	.word	0x0800b250
	e000ed88 	.word	0xe000ed88

<smoltcp::wire::arp::Repr::emit>:
	b5b0      	push	{r4, r5, r7, lr}
	af02      	add	r7, sp, #8
	2a01      	cmp	r2, #1
	d949      	bls.n	<smoltcp::wire::arp::Repr::emit+0x9c>
	f8d0 e00a 	ldr.w	lr, [r0, #10]
	f44f 7580 	mov.w	r5, #256	@ 0x100
	f8d0 c014 	ldr.w	ip, [r0, #20]
	2a03      	cmp	r2, #3
	8843      	ldrh	r3, [r0, #2]
	8804      	ldrh	r4, [r0, #0]
	800d      	strh	r5, [r1, #0]
	d946      	bls.n	<smoltcp::wire::arp::Repr::emit+0xac>
	2508      	movs	r5, #8
	2a04      	cmp	r2, #4
	804d      	strh	r5, [r1, #2]
	d062      	beq.n	<smoltcp::wire::arp::Repr::emit+0xec>
	2506      	movs	r5, #6
	2a05      	cmp	r2, #5
	710d      	strb	r5, [r1, #4]
	d966      	bls.n	<smoltcp::wire::arp::Repr::emit+0xfc>
	2504      	movs	r5, #4
	2a07      	cmp	r2, #7
	714d      	strb	r5, [r1, #5]
	d942      	bls.n	<smoltcp::wire::arp::Repr::emit+0xbc>
	b324      	cbz	r4, <smoltcp::wire::arp::Repr::emit+0x82>
	2c01      	cmp	r4, #1
	bf08      	it	eq
	2302      	moveq	r3, #2
	ba5b      	rev16	r3, r3
	2a0d      	cmp	r2, #13
	80cb      	strh	r3, [r1, #6]
	d922      	bls.n	<smoltcp::wire::arp::Repr::emit+0x8c>
	1d03      	adds	r3, r0, #4
	2a11      	cmp	r2, #17
	681c      	ldr	r4, [r3, #0]
	889b      	ldrh	r3, [r3, #4]
	818b      	strh	r3, [r1, #12]
	608c      	str	r4, [r1, #8]
	d93b      	bls.n	<smoltcp::wire::arp::Repr::emit+0xcc>
	2a17      	cmp	r2, #23
	f8c1 e00e 	str.w	lr, [r1, #14]
	d93f      	bls.n	<smoltcp::wire::arp::Repr::emit+0xdc>
	300e      	adds	r0, #14
	2a1b      	cmp	r2, #27
	6803      	ldr	r3, [r0, #0]
	8880      	ldrh	r0, [r0, #4]
	82c8      	strh	r0, [r1, #22]
	f8c1 3012 	str.w	r3, [r1, #18]
	bf84      	itt	hi
	f8c1 c018 	strhi.w	ip, [r1, #24]
	bdb0      	pophi	{r4, r5, r7, pc}
	f649 1398 	movw	r3, #39320	@ 0x9998
	2018      	movs	r0, #24
	f6c0 0300 	movt	r3, #2048	@ 0x800
	211c      	movs	r1, #28
	f003 fa96 	bl	<core::slice::index::slice_index_fail>
	2301      	movs	r3, #1
	ba5b      	rev16	r3, r3
	2a0d      	cmp	r2, #13
	80cb      	strh	r3, [r1, #6]
	d8dc      	bhi.n	<smoltcp::wire::arp::Repr::emit+0x46>
	f649 1368 	movw	r3, #39272	@ 0x9968
	2008      	movs	r0, #8
	f6c0 0300 	movt	r3, #2048	@ 0x800
	210e      	movs	r1, #14
	f003 fa89 	bl	<core::slice::index::slice_index_fail>
	f649 1348 	movw	r3, #39240	@ 0x9948
	2000      	movs	r0, #0
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2102      	movs	r1, #2
	f003 fa81 	bl	<core::slice::index::slice_index_fail>
	f649 1358 	movw	r3, #39256	@ 0x9958
	2002      	movs	r0, #2
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2104      	movs	r1, #4
	f003 fa79 	bl	<core::slice::index::slice_index_fail>
	f649 1338 	movw	r3, #39224	@ 0x9938
	2006      	movs	r0, #6
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2108      	movs	r1, #8
	f003 fa71 	bl	<core::slice::index::slice_index_fail>
	f649 1378 	movw	r3, #39288	@ 0x9978
	200e      	movs	r0, #14
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2112      	movs	r1, #18
	f003 fa69 	bl	<core::slice::index::slice_index_fail>
	f649 1388 	movw	r3, #39304	@ 0x9988
	2012      	movs	r0, #18
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2118      	movs	r1, #24
	f003 fa61 	bl	<core::slice::index::slice_index_fail>
	f649 12a8 	movw	r2, #39336	@ 0x99a8
	2004      	movs	r0, #4
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2104      	movs	r1, #4
	f003 fad8 	bl	<core::panicking::panic_bounds_check>
	f649 12b8 	movw	r2, #39352	@ 0x99b8
	2005      	movs	r0, #5
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2105      	movs	r1, #5
	f003 fad0 	bl	<core::panicking::panic_bounds_check>

<smoltcp::iface::interface::InterfaceInner::dispatch_ip>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b0b1      	sub	sp, #196	@ 0xc4
	f8d3 8058 	ldr.w	r8, [r3, #88]	@ 0x58
	f1b8 0f00 	cmp.w	r8, #0
	f000 856e 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xaf2>
	920b      	str	r2, [sp, #44]	@ 0x2c
	f118 0601 	adds.w	r6, r8, #1
	6d5a      	ldr	r2, [r3, #84]	@ 0x54
	920f      	str	r2, [sp, #60]	@ 0x3c
	6dda      	ldr	r2, [r3, #92]	@ 0x5c
	9211      	str	r2, [sp, #68]	@ 0x44
	f893 2062 	ldrb.w	r2, [r3, #98]	@ 0x62
	9208      	str	r2, [sp, #32]
	f893 2061 	ldrb.w	r2, [r3, #97]	@ 0x61
	920d      	str	r2, [sp, #52]	@ 0x34
	f893 2060 	ldrb.w	r2, [r3, #96]	@ 0x60
	920c      	str	r2, [sp, #48]	@ 0x30
	6802      	ldr	r2, [r0, #0]
	68c4      	ldr	r4, [r0, #12]
	920a      	str	r2, [sp, #40]	@ 0x28
	6842      	ldr	r2, [r0, #4]
	9209      	str	r2, [sp, #36]	@ 0x24
	6882      	ldr	r2, [r0, #8]
	920e      	str	r2, [sp, #56]	@ 0x38
	d044      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd0>
	f8d0 90f0 	ldr.w	r9, [r0, #240]	@ 0xf0
	9410      	str	r4, [sp, #64]	@ 0x40
	f1b9 0f00 	cmp.w	r9, #0
	eb09 0a89 	add.w	sl, r9, r9, lsl #2
	d028      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xa8>
	eb00 060a 	add.w	r6, r0, sl
	f100 05f4 	add.w	r5, r0, #244	@ 0xf4
	f106 0ef4 	add.w	lr, r6, #244	@ 0xf4
	f04f 3cff 	mov.w	ip, #4294967295	@ 0xffffffff
	e00e      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x86>
	2600      	movs	r6, #0
	f004 041f 	and.w	r4, r4, #31
	ea06 060b 	and.w	r6, r6, fp
	fa2c f404 	lsr.w	r4, ip, r4
	ba24      	rev	r4, r4
	4334      	orrs	r4, r6
	45a0      	cmp	r8, r4
	f000 80e9 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x252>
	3505      	adds	r5, #5
	4575      	cmp	r5, lr
	d010      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xa8>
	792c      	ldrb	r4, [r5, #4]
	f8d5 b000 	ldr.w	fp, [r5]
	2c00      	cmp	r4, #0
	d0eb      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x68>
	f1a4 061f 	sub.w	r6, r4, #31
	b2f6      	uxtb	r6, r6
	2e02      	cmp	r6, #2
	d3f2      	bcc.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x80>
	4266      	negs	r6, r4
	f006 061f 	and.w	r6, r6, #31
	fa0c f606 	lsl.w	r6, ip, r6
	ba36      	rev	r6, r6
	e7e0      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x6a>
	f008 06f0 	and.w	r6, r8, #240	@ 0xf0
	2ee0      	cmp	r6, #224	@ 0xe0
	d12d      	bne.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x10c>
	f3c8 2a06 	ubfx	sl, r8, #8, #7
	ea4f 6c18 	mov.w	ip, r8, lsr #24
	ea4f 4b18 	mov.w	fp, r8, lsr #16
	f04f 0901 	mov.w	r9, #1
	2500      	movs	r5, #0
	265e      	movs	r6, #94	@ 0x5e
	9c10      	ldr	r4, [sp, #64]	@ 0x40
	680a      	ldr	r2, [r1, #0]
	2a02      	cmp	r2, #2
	f000 80d1 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x270>
	e00d      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xec>
	f04f 0cff 	mov.w	ip, #255	@ 0xff
	f04f 0bff 	mov.w	fp, #255	@ 0xff
	f04f 0aff 	mov.w	sl, #255	@ 0xff
	26ff      	movs	r6, #255	@ 0xff
	25ff      	movs	r5, #255	@ 0xff
	f04f 09ff 	mov.w	r9, #255	@ 0xff
	680a      	ldr	r2, [r1, #0]
	2a02      	cmp	r2, #2
	f000 80c2 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x270>
	9607      	str	r6, [sp, #28]
	f1a4 020e 	sub.w	r2, r4, #14
	9e11      	ldr	r6, [sp, #68]	@ 0x44
	f106 0e14 	add.w	lr, r6, #20
	4596      	cmp	lr, r2
	d927      	bls.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x14c>
	f240 0022 	movw	r0, #34	@ 0x22
	f2c0 0000 	movt	r0, #0
	f004 f809 	bl	<defmt::export::acquire_header_and_release>
	f000 bc0c 	b.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x924>
	e9d0 2b04 	ldrd	r2, fp, [r0, #16]
	f100 04f4 	add.w	r4, r0, #244	@ 0xf4
	f04f 3cff 	mov.w	ip, #4294967295	@ 0xffffffff
	9207      	str	r2, [sp, #28]
	e00e      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x13a>
	4276      	negs	r6, r6
	f006 061f 	and.w	r6, r6, #31
	fa0c f606 	lsl.w	r6, ip, r6
	ba36      	rev	r6, r6
	6825      	ldr	r5, [r4, #0]
	3405      	adds	r4, #5
	f1aa 0a05 	sub.w	sl, sl, #5
	ea85 0508 	eor.w	r5, r5, r8
	422e      	tst	r6, r5
	f000 80b8 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x2aa>
	f1ba 0f00 	cmp.w	sl, #0
	f000 809c 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x27a>
	7926      	ldrb	r6, [r4, #4]
	2e00      	cmp	r6, #0
	d1e9      	bne.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x11c>
	2600      	movs	r6, #0
	e7ed      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x128>
	9505      	str	r5, [sp, #20]
	688d      	ldr	r5, [r1, #8]
	e9d5 1601 	ldrd	r1, r6, [r5, #4]
	428e      	cmp	r6, r1
	f080 85da 	bcs.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd0e>
	f44f 62c6 	mov.w	r2, #1584	@ 0x630
	e9cd ba02 	strd	fp, sl, [sp, #8]
	4372      	muls	r2, r6
	f8d5 a000 	ldr.w	sl, [r5]
	f8cd c010 	str.w	ip, [sp, #16]
	9410      	str	r4, [sp, #64]	@ 0x40
	f85a 2002 	ldr.w	r2, [sl, r2]
	2a00      	cmp	r2, #0
	f100 85d2 	bmi.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd1c>
	9a11      	ldr	r2, [sp, #68]	@ 0x44
	9606      	str	r6, [sp, #24]
	f102 0b22 	add.w	fp, r2, #34	@ 0x22
	1c72      	adds	r2, r6, #1
	fbb2 fcf1 	udiv	ip, r2, r1
	fb0c 2111 	mls	r1, ip, r1, r2
	60a9      	str	r1, [r5, #8]
	f240 51f3 	movw	r1, #1523	@ 0x5f3
	458b      	cmp	fp, r1
	f080 84b9 	bcs.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb08>
	46dc      	mov	ip, fp
	f1bb 0f0b 	cmp.w	fp, #11
	f240 84bf 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb1e>
	9e06      	ldr	r6, [sp, #24]
	f44f 62c6 	mov.w	r2, #1584	@ 0x630
	f500 71b4 	add.w	r1, r0, #360	@ 0x168
	46ab      	mov	fp, r5
	f1bc 0f0d 	cmp.w	ip, #13
	fb06 a202 	mla	r2, r6, r2, sl
	f102 0538 	add.w	r5, r2, #56	@ 0x38
	680a      	ldr	r2, [r1, #0]
	8889      	ldrh	r1, [r1, #4]
	8169      	strh	r1, [r5, #10]
	9904      	ldr	r1, [sp, #16]
	7169      	strb	r1, [r5, #5]
	9902      	ldr	r1, [sp, #8]
	7129      	strb	r1, [r5, #4]
	9903      	ldr	r1, [sp, #12]
	70e9      	strb	r1, [r5, #3]
	9907      	ldr	r1, [sp, #28]
	70a9      	strb	r1, [r5, #2]
	9905      	ldr	r1, [sp, #20]
	f8c5 2006 	str.w	r2, [r5, #6]
	7069      	strb	r1, [r5, #1]
	f885 9000 	strb.w	r9, [r5]
	f240 84a9 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb30>
	f8dd 9044 	ldr.w	r9, [sp, #68]	@ 0x44
	2108      	movs	r1, #8
	9c0d      	ldr	r4, [sp, #52]	@ 0x34
	f1be 0f00 	cmp.w	lr, #0
	81a9      	strh	r1, [r5, #12]
	f000 85a1 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd32>
	7a00      	ldrb	r0, [r0, #8]
	f105 010e 	add.w	r1, r5, #14
	2245      	movs	r2, #69	@ 0x45
	f1be 0f01 	cmp.w	lr, #1
	700a      	strb	r2, [r1, #0]
	f000 85a0 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd42>
	2200      	movs	r2, #0
	f1be 0f03 	cmp.w	lr, #3
	73ea      	strb	r2, [r5, #15]
	f240 849a 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb42>
	fa9e f28e 	rev.w	r2, lr
	f1be 0f05 	cmp.w	lr, #5
	ea4f 4212 	mov.w	r2, r2, lsr #16
	822a      	strh	r2, [r5, #16]
	f240 849a 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb54>
	2200      	movs	r2, #0
	f1be 0f07 	cmp.w	lr, #7
	826a      	strh	r2, [r5, #18]
	f240 849d 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb66>
	2240      	movs	r2, #64	@ 0x40
	f1be 0f08 	cmp.w	lr, #8
	82aa      	strh	r2, [r5, #20]
	f000 858d 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd52>
	9a0c      	ldr	r2, [sp, #48]	@ 0x30
	75aa      	strb	r2, [r5, #22]
	e8df f004 	tbb	[pc, r4]
	07979797 	.word	0x07979797
	8e929088 	.word	0x8e929088
	86948c96 	.word	0x86948c96
	008a      	.short	0x008a
	2406      	movs	r4, #6
	e08d      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
	f04f 0cff 	mov.w	ip, #255	@ 0xff
	f04f 0bff 	mov.w	fp, #255	@ 0xff
	f04f 0aff 	mov.w	sl, #255	@ 0xff
	26ff      	movs	r6, #255	@ 0xff
	25ff      	movs	r5, #255	@ 0xff
	f04f 09ff 	mov.w	r9, #255	@ 0xff
	9c10      	ldr	r4, [sp, #64]	@ 0x40
	680a      	ldr	r2, [r1, #0]
	2a02      	cmp	r2, #2
	f47f af3e 	bne.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xec>
	4648      	mov	r0, r9
	b031      	add	sp, #196	@ 0xc4
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f500 7680 	add.w	r6, r0, #256	@ 0x100
	4682      	mov	sl, r0
	a813      	add	r0, sp, #76	@ 0x4c
	e9cd 2b00 	strd	r2, fp, [sp]
	460d      	mov	r5, r1
	4631      	mov	r1, r6
	4642      	mov	r2, r8
	461c      	mov	r4, r3
	f007 ff1c 	bl	<smoltcp::iface::route::Routes::lookup>
	f89d 004c 	ldrb.w	r0, [sp, #76]	@ 0x4c
	4629      	mov	r1, r5
	4623      	mov	r3, r4
	b948      	cbnz	r0, <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x2b0>
	f04f 0900 	mov.w	r9, #0
	4648      	mov	r0, r9
	b031      	add	sp, #196	@ 0xc4
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	4682      	mov	sl, r0
	f8cd 804d 	str.w	r8, [sp, #77]	@ 0x4d
	f89d 0050 	ldrb.w	r0, [sp, #80]	@ 0x50
	f8bd 604e 	ldrh.w	r6, [sp, #78]	@ 0x4e
	f89d e04d 	ldrb.w	lr, [sp, #77]	@ 0x4d
	ea46 4200 	orr.w	r2, r6, r0, lsl #16
	ea4e 2602 	orr.w	r6, lr, r2, lsl #8
	1c70      	adds	r0, r6, #1
	d006      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x2d6>
	f00e 00f0 	and.w	r0, lr, #240	@ 0xf0
	28e0      	cmp	r0, #224	@ 0xe0
	bf18      	it	ne
	ea52 000e 	orrsne.w	r0, r2, lr
	d10a      	bne.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x2ec>
	f24b 000c 	movw	r0, #45068	@ 0xb00c
	f24b 0238 	movw	r2, #45112	@ 0xb038
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	212c      	movs	r1, #44	@ 0x2c
	f003 f96e 	bl	<core::panicking::panic>
	f8da 40e0 	ldr.w	r4, [sl, #224]	@ 0xe0
	4650      	mov	r0, sl
	e9cd 2b05 	strd	r2, fp, [sp, #20]
	eba4 0484 	sub.w	r4, r4, r4, lsl #2
	ea4f 0ac4 	mov.w	sl, r4, lsl #3
	2400      	movs	r4, #0
	eb1a 0b04 	adds.w	fp, sl, r4
	f000 832f 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
	1905      	adds	r5, r0, r4
	f8d5 c020 	ldr.w	ip, [r5, #32]
	45b4      	cmp	ip, r6
	f000 82d2 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8b8>
	f11b 0218 	adds.w	r2, fp, #24
	f000 8325 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
	6baa      	ldr	r2, [r5, #56]	@ 0x38
	42b2      	cmp	r2, r6
	f000 82cc 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8bc>
	f11b 0230 	adds.w	r2, fp, #48	@ 0x30
	f000 831d 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
	6d2a      	ldr	r2, [r5, #80]	@ 0x50
	42b2      	cmp	r2, r6
	f000 82ff 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x932>
	f11b 0248 	adds.w	r2, fp, #72	@ 0x48
	f000 8315 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
	6eaa      	ldr	r2, [r5, #104]	@ 0x68
	3460      	adds	r4, #96	@ 0x60
	42b2      	cmp	r2, r6
	d1dd      	bne.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x300>
	1902      	adds	r2, r0, r4
	f102 0508 	add.w	r5, r2, #8
	e2f3      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x934>
	243c      	movs	r4, #60	@ 0x3c
	e00e      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
	2411      	movs	r4, #17
	e00c      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
	9c08      	ldr	r4, [sp, #32]
	e00a      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
	243a      	movs	r4, #58	@ 0x3a
	e008      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
	2432      	movs	r4, #50	@ 0x32
	e006      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
	242b      	movs	r4, #43	@ 0x2b
	e004      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
	242c      	movs	r4, #44	@ 0x2c
	e002      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
	243b      	movs	r4, #59	@ 0x3b
	e000      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
	2433      	movs	r4, #51	@ 0x33
	f1be 0f09 	cmp.w	lr, #9
	f240 84f6 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd62>
	f1be 0f0f 	cmp.w	lr, #15
	75ec      	strb	r4, [r5, #23]
	f240 83fc 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb78>
	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
	f119 0f14 	cmn.w	r9, #20
	f8c5 201a 	str.w	r2, [r5, #26]
	f080 83fe 	bcs.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb8a>
	f010 0ffd 	tst.w	r0, #253	@ 0xfd
	f8cd c034 	str.w	ip, [sp, #52]	@ 0x34
	f8c5 801e 	str.w	r8, [r5, #30]
	d10b      	bne.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x3b4>
	2000      	movs	r0, #0
	461c      	mov	r4, r3
	8328      	strh	r0, [r5, #24]
	4608      	mov	r0, r1
	2114      	movs	r1, #20
	f007 fd2a 	bl	<smoltcp::wire::ip::checksum::data>
	43c0      	mvns	r0, r0
	4623      	mov	r3, r4
	ba00      	rev	r0, r0
	0c00      	lsrs	r0, r0, #16
	e000      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x3b6>
	2000      	movs	r0, #0
	8328      	strh	r0, [r5, #24]
	f105 0a22 	add.w	sl, r5, #34	@ 0x22
	6818      	ldr	r0, [r3, #0]
	3802      	subs	r0, #2
	bf38      	it	cc
	2002      	movcc	r0, #2
	2800      	cmp	r0, #0
	d071      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x4ac>
	2801      	cmp	r0, #1
	f040 8081 	bne.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x4d0>
	f1b9 0f01 	cmp.w	r9, #1
	f240 83ec 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbae>
	e9d3 1201 	ldrd	r1, r2, [r3, #4]
	f1b9 0f03 	cmp.w	r9, #3
	89d8      	ldrh	r0, [r3, #14]
	899b      	ldrh	r3, [r3, #12]
	ba1b      	rev	r3, r3
	ea4f 4313 	mov.w	r3, r3, lsr #16
	f8aa 3000 	strh.w	r3, [sl]
	f240 83f1 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbd2>
	ba40      	rev16	r0, r0
	f1b9 0f05 	cmp.w	r9, #5
	84a8      	strh	r0, [r5, #36]	@ 0x24
	f240 83fd 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbf6>
	f102 0008 	add.w	r0, r2, #8
	ba03      	rev	r3, r0
	b286      	uxth	r6, r0
	b290      	uxth	r0, r2
	0c1b      	lsrs	r3, r3, #16
	84eb      	strh	r3, [r5, #38]	@ 0x26
	f64f 73f7 	movw	r3, #65527	@ 0xfff7
	4298      	cmp	r0, r3
	f200 83c4 	bhi.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb9c>
	45b1      	cmp	r9, r6
	f0c0 83c1 	bcc.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb9c>
	f1a6 0008 	sub.w	r0, r6, #8
	4290      	cmp	r0, r2
	f040 840d 	bne.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc3e>
	980e      	ldr	r0, [sp, #56]	@ 0x38
	0a04      	lsrs	r4, r0, #8
	f105 002a 	add.w	r0, r5, #42	@ 0x2a
	f008 fe38 	bl	<__aeabi_memcpy>
	f014 0ffd 	tst.w	r4, #253	@ 0xfd
	f040 80f5 	bne.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x622>
	f1b9 0f07 	cmp.w	r9, #7
	f240 8407 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc4e>
	2000      	movs	r0, #0
	f64f 79ff 	movw	r9, #65535	@ 0xffff
	8528      	strh	r0, [r5, #40]	@ 0x28
	fa98 f088 	rev.w	r0, r8
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	990f      	ldr	r1, [sp, #60]	@ 0x3c
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	ba09      	rev	r1, r1
	b280      	uxth	r0, r0
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	eb01 4111 	add.w	r1, r1, r1, lsr #16
	fa10 f081 	uxtah	r0, r0, r1
	4430      	add	r0, r6
	3011      	adds	r0, #17
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	4631      	mov	r1, r6
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	b284      	uxth	r4, r0
	4650      	mov	r0, sl
	f007 fcb8 	bl	<smoltcp::wire::ip::checksum::data>
	fa14 f080 	uxtah	r0, r4, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	43c1      	mvns	r1, r0
	b280      	uxth	r0, r0
	ba09      	rev	r1, r1
	4548      	cmp	r0, r9
	bf18      	it	ne
	0c08      	lsrne	r0, r1, #16
	8528      	strh	r0, [r5, #40]	@ 0x28
	e20b      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c4>
	f1b9 0f02 	cmp.w	r9, #2
	f0c0 8465 	bcc.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd7e>
	980e      	ldr	r0, [sp, #56]	@ 0x38
	ea4f 6810 	mov.w	r8, r0, lsr #24
	7918      	ldrb	r0, [r3, #4]
	e8df f010 	tbh	[pc, r0, lsl #1]
	01070004 	.word	0x01070004
	00df00b8 	.word	0x00df00b8
	e9d3 1003 	ldrd	r1, r0, [r3, #12]
	2608      	movs	r6, #8
	e101      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x6d4>
	a813      	add	r0, sp, #76	@ 0x4c
	4619      	mov	r1, r3
	2254      	movs	r2, #84	@ 0x54
	f008 f8b9 	bl	<__aeabi_memcpy4>
	980a      	ldr	r0, [sp, #40]	@ 0x28
	b380      	cbz	r0, <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x540>
	f8bd 0084 	ldrh.w	r0, [sp, #132]	@ 0x84
	2318      	movs	r3, #24
	f89d 2094 	ldrb.w	r2, [sp, #148]	@ 0x94
	f89d 109c 	ldrb.w	r1, [sp, #156]	@ 0x9c
	2800      	cmp	r0, #0
	f8dd c054 	ldr.w	ip, [sp, #84]	@ 0x54
	f8dd e060 	ldr.w	lr, [sp, #96]	@ 0x60
	9e1b      	ldr	r6, [sp, #108]	@ 0x6c
	9c1e      	ldr	r4, [sp, #120]	@ 0x78
	bf08      	it	eq
	2314      	moveq	r3, #20
	f362 0300 	bfi	r3, r2, #0, #1
	f362 0341 	bfi	r3, r2, #1, #1
	2c00      	cmp	r4, #0
	eb03 0041 	add.w	r0, r3, r1, lsl #1
	eb0e 010c 	add.w	r1, lr, ip
	4431      	add	r1, r6
	bf18      	it	ne
	300a      	addne	r0, #10
	2300      	movs	r3, #0
	eb00 02c1 	add.w	r2, r0, r1, lsl #3
	ebb3 0fc1 	cmp.w	r3, r1, lsl #3
	bf18      	it	ne
	1c90      	addne	r0, r2, #2
	9910      	ldr	r1, [sp, #64]	@ 0x40
	3003      	adds	r0, #3
	f020 0003 	bic.w	r0, r0, #3
	1a08      	subs	r0, r1, r0
	9909      	ldr	r1, [sp, #36]	@ 0x24
	3814      	subs	r0, #20
	4348      	muls	r0, r1
	f8bd 109a 	ldrh.w	r1, [sp, #154]	@ 0x9a
	4288      	cmp	r0, r1
	bf38      	it	cc
	f8ad 009a 	strhcc.w	r0, [sp, #154]	@ 0x9a
	f1b9 0f01 	cmp.w	r9, #1
	f240 833c 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbc0>
	f8bd 0096 	ldrh.w	r0, [sp, #150]	@ 0x96
	f1b9 0f03 	cmp.w	r9, #3
	ba00      	rev	r0, r0
	ea4f 4010 	mov.w	r0, r0, lsr #16
	f8aa 0000 	strh.w	r0, [sl]
	f240 8343 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbe4>
	f8bd 0098 	ldrh.w	r0, [sp, #152]	@ 0x98
	f1b9 0f07 	cmp.w	r9, #7
	ba00      	rev	r0, r0
	ea4f 4010 	mov.w	r0, r0, lsr #16
	84a8      	strh	r0, [r5, #36]	@ 0x24
	f240 834b 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc08>
	9e13      	ldr	r6, [sp, #76]	@ 0x4c
	f1b9 0f0b 	cmp.w	r9, #11
	9824      	ldr	r0, [sp, #144]	@ 0x90
	ba00      	rev	r0, r0
	f8c5 0026 	str.w	r0, [r5, #38]	@ 0x26
	f240 834b 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc1a>
	2e00      	cmp	r6, #0
	4631      	mov	r1, r6
	f8cd a030 	str.w	sl, [sp, #48]	@ 0x30
	f8cd b040 	str.w	fp, [sp, #64]	@ 0x40
	9814      	ldr	r0, [sp, #80]	@ 0x50
	bf18      	it	ne
	ba01      	revne	r1, r0
	f1b9 0f0f 	cmp.w	r9, #15
	f8c5 102a 	str.w	r1, [r5, #42]	@ 0x2a
	f240 8345 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc2c>
	f8bd 109a 	ldrh.w	r1, [sp, #154]	@ 0x9a
	980e      	ldr	r0, [sp, #56]	@ 0x38
	ba09      	rev	r1, r1
	f89d a09c 	ldrb.w	sl, [sp, #156]	@ 0x9c
	9c15      	ldr	r4, [sp, #84]	@ 0x54
	ea4f 4c10 	mov.w	ip, r0, lsr #16
	f8bd 0084 	ldrh.w	r0, [sp, #132]	@ 0x84
	0c09      	lsrs	r1, r1, #16
	9a18      	ldr	r2, [sp, #96]	@ 0x60
	9b1b      	ldr	r3, [sp, #108]	@ 0x6c
	2800      	cmp	r0, #0
	f8dd 9078 	ldr.w	r9, [sp, #120]	@ 0x78
	f89d b094 	ldrb.w	fp, [sp, #148]	@ 0x94
	8629      	strh	r1, [r5, #48]	@ 0x30
	f04f 0118 	mov.w	r1, #24
	bf08      	it	eq
	2114      	moveq	r1, #20
	f1b9 0f00 	cmp.w	r9, #0
	f36b 0100 	bfi	r1, fp, #0, #1
	f36b 0141 	bfi	r1, fp, #1, #1
	eb01 014a 	add.w	r1, r1, sl, lsl #1
	bf18      	it	ne
	310a      	addne	r1, #10
	e9cd 2409 	strd	r2, r4, [sp, #36]	@ 0x24
	4422      	add	r2, r4
	9308      	str	r3, [sp, #32]
	441a      	add	r2, r3
	2400      	movs	r4, #0
	eb01 03c2 	add.w	r3, r1, r2, lsl #3
	ebb4 0fc2 	cmp.w	r4, r2, lsl #3
	bf18      	it	ne
	1c99      	addne	r1, r3, #2
	220c      	movs	r2, #12
	eb02 0181 	add.w	r1, r2, r1, lsl #2
	f001 02f0 	and.w	r2, r1, #240	@ 0xf0
	a913      	add	r1, sp, #76	@ 0x4c
	f101 0408 	add.w	r4, r1, #8
	f89d 109d 	ldrb.w	r1, [sp, #157]	@ 0x9d
	e8df f001 	tbb	[pc, r1]
	038d      	.short	0x038d
	00858b88 	.word	0x00858b88
	f502 6200 	add.w	r2, r2, #2048	@ 0x800
	e086      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x730>
	f1b9 0f07 	cmp.w	r9, #7
	f240 8312 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc4e>
	2000      	movs	r0, #0
	8528      	strh	r0, [r5, #40]	@ 0x28
	e149      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c4>
	7959      	ldrb	r1, [r3, #5]
	2003      	movs	r0, #3
	f885 0022 	strb.w	r0, [r5, #34]	@ 0x22
	e9d3 0602 	ldrd	r0, r6, [r3, #8]
	2910      	cmp	r1, #16
	799a      	ldrb	r2, [r3, #6]
	9010      	str	r0, [sp, #64]	@ 0x40
	bf38      	it	cc
	460a      	movcc	r2, r1
	f1b9 0f08 	cmp.w	r9, #8
	f885 2023 	strb.w	r2, [r5, #35]	@ 0x23
	f0c0 8310 	bcc.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc72>
	f103 0010 	add.w	r0, r3, #16
	f1a9 0408 	sub.w	r4, r9, #8
	9b0e      	ldr	r3, [sp, #56]	@ 0x38
	f105 012a 	add.w	r1, r5, #42	@ 0x2a
	4622      	mov	r2, r4
	f007 f96b 	bl	<smoltcp::wire::ipv4::Repr::emit>
	2c14      	cmp	r4, #20
	f0c0 832f 	bcc.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xcca>
	f1a9 021c 	sub.w	r2, r9, #28
	42b2      	cmp	r2, r6
	f040 833c 	bne.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xcee>
	f105 003e 	add.w	r0, r5, #62	@ 0x3e
	9910      	ldr	r1, [sp, #64]	@ 0x40
	e040      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x700>
	7959      	ldrb	r1, [r3, #5]
	200b      	movs	r0, #11
	f885 0022 	strb.w	r0, [r5, #34]	@ 0x22
	e9d3 0402 	ldrd	r0, r4, [r3, #8]
	2902      	cmp	r1, #2
	799a      	ldrb	r2, [r3, #6]
	9010      	str	r0, [sp, #64]	@ 0x40
	bf38      	it	cc
	460a      	movcc	r2, r1
	f1b9 0f08 	cmp.w	r9, #8
	f885 2023 	strb.w	r2, [r5, #35]	@ 0x23
	f0c0 82e9 	bcc.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc72>
	f103 0010 	add.w	r0, r3, #16
	f1a9 0608 	sub.w	r6, r9, #8
	9b0e      	ldr	r3, [sp, #56]	@ 0x38
	f105 012a 	add.w	r1, r5, #42	@ 0x2a
	4632      	mov	r2, r6
	f007 f944 	bl	<smoltcp::wire::ipv4::Repr::emit>
	2e14      	cmp	r6, #20
	f0c0 8311 	bcc.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xcdc>
	f1a9 001c 	sub.w	r0, r9, #28
	42a0      	cmp	r0, r4
	f040 831e 	bne.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd00>
	f105 003e 	add.w	r0, r5, #62	@ 0x3e
	9910      	ldr	r1, [sp, #64]	@ 0x40
	4622      	mov	r2, r4
	e018      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x700>
	e9d3 1003 	ldrd	r1, r0, [r3, #12]
	2600      	movs	r6, #0
	891a      	ldrh	r2, [r3, #8]
	88db      	ldrh	r3, [r3, #6]
	f1b9 0f05 	cmp.w	r9, #5
	846e      	strh	r6, [r5, #34]	@ 0x22
	f240 82d1 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc84>
	ba5b      	rev16	r3, r3
	f1b9 0f07 	cmp.w	r9, #7
	84eb      	strh	r3, [r5, #38]	@ 0x26
	f240 82d4 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc96>
	ba52      	rev16	r2, r2
	852a      	strh	r2, [r5, #40]	@ 0x28
	f1a9 0208 	sub.w	r2, r9, #8
	4290      	cmp	r0, r2
	bf38      	it	cc
	4602      	movcc	r2, r0
	f105 002a 	add.w	r0, r5, #42	@ 0x2a
	f008 fcce 	bl	<__aeabi_memcpy>
	2000      	movs	r0, #0
	f018 0ffd 	tst.w	r8, #253	@ 0xfd
	d107      	bne.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x71c>
	84a8      	strh	r0, [r5, #36]	@ 0x24
	4650      	mov	r0, sl
	4649      	mov	r1, r9
	f007 fb74 	bl	<smoltcp::wire::ip::checksum::data>
	43c0      	mvns	r0, r0
	ba00      	rev	r0, r0
	0c00      	lsrs	r0, r0, #16
	84a8      	strh	r0, [r5, #36]	@ 0x24
	e0d1      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c4>
	f502 6280 	add.w	r2, r2, #1024	@ 0x400
	e004      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x730>
	f502 7200 	add.w	r2, r2, #512	@ 0x200
	e001      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x730>
	f502 7280 	add.w	r2, r2, #256	@ 0x100
	4611      	mov	r1, r2
	2e00      	cmp	r6, #0
	bf18      	it	ne
	f501 5180 	addne.w	r1, r1, #4096	@ 0x1000
	85e9      	strh	r1, [r5, #46]	@ 0x2e
	213c      	movs	r1, #60	@ 0x3c
	ea01 0192 	and.w	r1, r1, r2, lsr #2
	b2d2      	uxtb	r2, r2
	2a50      	cmp	r2, #80	@ 0x50
	bf24      	itt	cs
	9a11      	ldrcs	r2, [sp, #68]	@ 0x44
	428a      	cmpcs	r2, r1
	d207      	bcs.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x75e>
	9a11      	ldr	r2, [sp, #68]	@ 0x44
	f64a 23e0 	movw	r3, #43744	@ 0xaae0
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2014      	movs	r0, #20
	f002 fea2 	bl	<core::slice::index::slice_index_fail>
	f105 0336 	add.w	r3, r5, #54	@ 0x36
	f1a1 0214 	sub.w	r2, r1, #20
	f8cd c038 	str.w	ip, [sp, #56]	@ 0x38
	b1d0      	cbz	r0, <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7a2>
	f8bd 0086 	ldrh.w	r0, [sp, #134]	@ 0x86
	2104      	movs	r1, #4
	f8ad 00a4 	strh.w	r0, [sp, #164]	@ 0xa4
	a828      	add	r0, sp, #160	@ 0xa0
	9128      	str	r1, [sp, #160]	@ 0xa0
	4619      	mov	r1, r3
	f005 fbf4 	bl	<smoltcp::wire::tcp::TcpOption::emit>
	4603      	mov	r3, r0
	460a      	mov	r2, r1
	f1bb 0f00 	cmp.w	fp, #0
	d10e      	bne.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7a8>
	f8dd b040 	ldr.w	fp, [sp, #64]	@ 0x40
	f1ba 0f00 	cmp.w	sl, #0
	d01a      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7ca>
	f64a 3470 	movw	r4, #43888	@ 0xab70
	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
	f6c0 0400 	movt	r4, #2048	@ 0x800
	e01d      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7de>
	f1bb 0f00 	cmp.w	fp, #0
	d0f0      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x78a>
	f89d 0095 	ldrb.w	r0, [sp, #149]	@ 0x95
	2105      	movs	r1, #5
	f88d 00a4 	strb.w	r0, [sp, #164]	@ 0xa4
	a828      	add	r0, sp, #160	@ 0xa0
	9128      	str	r1, [sp, #160]	@ 0xa0
	4619      	mov	r1, r3
	f005 fbd6 	bl	<smoltcp::wire::tcp::TcpOption::emit>
	4603      	mov	r3, r0
	460a      	mov	r2, r1
	f8dd b040 	ldr.w	fp, [sp, #64]	@ 0x40
	f1ba 0f00 	cmp.w	sl, #0
	d1e4      	bne.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x794>
	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
	b166      	cbz	r6, <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7ea>
	980a      	ldr	r0, [sp, #40]	@ 0x28
	2800      	cmp	r0, #0
	bf04      	itt	eq
	9809      	ldreq	r0, [sp, #36]	@ 0x24
	2800      	cmpeq	r0, #0
	f000 8185 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xae8>
	4620      	mov	r0, r4
	4619      	mov	r1, r3
	f005 fbc1 	bl	<smoltcp::wire::tcp::TcpOption::emit>
	4603      	mov	r3, r0
	460a      	mov	r2, r1
	f1b9 0f00 	cmp.w	r9, #0
	d00b      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x808>
	e9dd 011f 	ldrd	r0, r1, [sp, #124]	@ 0x7c
	e9cd 0129 	strd	r0, r1, [sp, #164]	@ 0xa4
	2008      	movs	r0, #8
	9028      	str	r0, [sp, #160]	@ 0xa0
	a828      	add	r0, sp, #160	@ 0xa0
	4619      	mov	r1, r3
	f005 fbb2 	bl	<smoltcp::wire::tcp::TcpOption::emit>
	4603      	mov	r3, r0
	460a      	mov	r2, r1
	9c11      	ldr	r4, [sp, #68]	@ 0x44
	b132      	cbz	r2, <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x81a>
	f64a 3094 	movw	r0, #43924	@ 0xab94
	4619      	mov	r1, r3
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f005 fba7 	bl	<smoltcp::wire::tcp::TcpOption::emit>
	2c13      	cmp	r4, #19
	f240 8244 	bls.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xca8>
	8de8      	ldrh	r0, [r5, #46]	@ 0x2e
	2100      	movs	r1, #0
	86a9      	strh	r1, [r5, #52]	@ 0x34
	213c      	movs	r1, #60	@ 0x3c
	ea01 0090 	and.w	r0, r1, r0, lsr #2
	4284      	cmp	r4, r0
	f0c0 8244 	bcc.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xcba>
	9a23      	ldr	r2, [sp, #140]	@ 0x8c
	1a26      	subs	r6, r4, r0
	42b2      	cmp	r2, r6
	f200 8212 	bhi.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc60>
	9922      	ldr	r1, [sp, #136]	@ 0x88
	4450      	add	r0, sl
	f008 fc2e 	bl	<__aeabi_memcpy>
	980e      	ldr	r0, [sp, #56]	@ 0x38
	f010 0ffd 	tst.w	r0, #253	@ 0xfd
	d139      	bne.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c0>
	2000      	movs	r0, #0
	8668      	strh	r0, [r5, #50]	@ 0x32
	fa98 f088 	rev.w	r0, r8
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	990f      	ldr	r1, [sp, #60]	@ 0x3c
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	ba09      	rev	r1, r1
	b280      	uxth	r0, r0
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	eb01 4111 	add.w	r1, r1, r1, lsr #16
	fa10 f081 	uxtah	r0, r0, r1
	1da1      	adds	r1, r4, #6
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	eb01 4111 	add.w	r1, r1, r1, lsr #16
	fa10 f081 	uxtah	r0, r0, r1
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	4621      	mov	r1, r4
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	b286      	uxth	r6, r0
	4650      	mov	r0, sl
	f007 faae 	bl	<smoltcp::wire::ip::checksum::data>
	fa16 f080 	uxtah	r0, r6, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	43c0      	mvns	r0, r0
	ba00      	rev	r0, r0
	0c00      	lsrs	r0, r0, #16
	e004      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c2>
	3520      	adds	r5, #32
	e03b      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x934>
	3538      	adds	r5, #56	@ 0x38
	e039      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x934>
	2000      	movs	r0, #0
	8668      	strh	r0, [r5, #50]	@ 0x32
	f8dd c02c 	ldr.w	ip, [sp, #44]	@ 0x2c
	f8db 1004 	ldr.w	r1, [fp, #4]
	9806      	ldr	r0, [sp, #24]
	4288      	cmp	r0, r1
	f080 824f 	bcs.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd72>
	f44f 61c6 	mov.w	r1, #1584	@ 0x630
	f8db 2000 	ldr.w	r2, [fp]
	fb00 2301 	mla	r3, r0, r1, r2
	e9d3 650a 	ldrd	r6, r5, [r3, #40]	@ 0x28
	4341      	muls	r1, r0
	f640 70ff 	movw	r0, #4095	@ 0xfff
	685c      	ldr	r4, [r3, #4]
	ea24 0000 	bic.w	r0, r4, r0
	9c0d      	ldr	r4, [sp, #52]	@ 0x34
	4320      	orrs	r0, r4
	6058      	str	r0, [r3, #4]
	2001      	movs	r0, #1
	609e      	str	r6, [r3, #8]
	e9c3 0c08 	strd	r0, ip, [r3, #32]
	60dd      	str	r5, [r3, #12]
	f3bf 8f5f 	dmb	sy
	f893 0030 	ldrb.w	r0, [r3, #48]	@ 0x30
	2300      	movs	r3, #0
	f2cf 23d0 	movt	r3, #62160	@ 0xf2d0
	ea43 5040 	orr.w	r0, r3, r0, lsl #21
	5050      	str	r0, [r2, r1]
	f249 0004 	movw	r0, #36868	@ 0x9004
	f3bf 8f5f 	dmb	sy
	f2c4 0002 	movt	r0, #16386	@ 0x4002
	2100      	movs	r1, #0
	6001      	str	r1, [r0, #0]
	f04f 0902 	mov.w	r9, #2
	4648      	mov	r0, r9
	b031      	add	sp, #196	@ 0xc4
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	3550      	adds	r5, #80	@ 0x50
	e9d5 2602 	ldrd	r2, r6, [r5, #8]
	9c07      	ldr	r4, [sp, #28]
	1aa2      	subs	r2, r4, r2
	9a06      	ldr	r2, [sp, #24]
	41b2      	sbcs	r2, r6
	da11      	bge.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
	7c6a      	ldrb	r2, [r5, #17]
	f895 9010 	ldrb.w	r9, [r5, #16]
	7cae      	ldrb	r6, [r5, #18]
	f895 a013 	ldrb.w	sl, [r5, #19]
	f895 c015 	ldrb.w	ip, [r5, #21]
	f895 b014 	ldrb.w	fp, [r5, #20]
	4615      	mov	r5, r2
	9c10      	ldr	r4, [sp, #64]	@ 0x40
	680a      	ldr	r2, [r1, #0]
	2a02      	cmp	r2, #2
	f43f ac87 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x270>
	f7ff bbc3 	b.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xec>
	e9d0 233a 	ldrd	r2, r3, [r0, #232]	@ 0xe8
	9e07      	ldr	r6, [sp, #28]
	1ab2      	subs	r2, r6, r2
	9a06      	ldr	r2, [sp, #24]
	419a      	sbcs	r2, r3
	f2c0 80b2 	blt.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xada>
	4682      	mov	sl, r0
	9805      	ldr	r0, [sp, #20]
	f8ad 0049 	strh.w	r0, [sp, #73]	@ 0x49
	460c      	mov	r4, r1
	f88d e048 	strb.w	lr, [sp, #72]	@ 0x48
	0c00      	lsrs	r0, r0, #16
	f88d 004b 	strb.w	r0, [sp, #75]	@ 0x4b
	f240 0024 	movw	r0, #36	@ 0x24
	f2c0 0000 	movt	r0, #0
	f003 fbb1 	bl	<defmt::export::acquire_and_header>
	f240 0007 	movw	r0, #7
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 004c 	strh.w	r0, [sp, #76]	@ 0x4c
	a813      	add	r0, sp, #76	@ 0x4c
	f003 fd59 	bl	<_defmt_write>
	a812      	add	r0, sp, #72	@ 0x48
	f003 fb52 	bl	<defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>
	a813      	add	r0, sp, #76	@ 0x4c
	2600      	movs	r6, #0
	2102      	movs	r1, #2
	f8ad 604c 	strh.w	r6, [sp, #76]	@ 0x4c
	f003 fd4f 	bl	<_defmt_write>
	f003 fcd5 	bl	<_defmt_release>
	f1b9 0f00 	cmp.w	r9, #0
	f43f ac69 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x29c>
	f8da 20f4 	ldr.w	r2, [sl, #244]	@ 0xf4
	f04f 35ff 	mov.w	r5, #4294967295	@ 0xffffffff
	9912      	ldr	r1, [sp, #72]	@ 0x48
	f8ad 604c 	strh.w	r6, [sp, #76]	@ 0x4c
	f8ba 616c 	ldrh.w	r6, [sl, #364]	@ 0x16c
	9118      	str	r1, [sp, #96]	@ 0x60
	0e11      	lsrs	r1, r2, #24
	f8da 3168 	ldr.w	r3, [sl, #360]	@ 0x168
	f8ad 6054 	strh.w	r6, [sp, #84]	@ 0x54
	f88d 1059 	strb.w	r1, [sp, #89]	@ 0x59
	0a11      	lsrs	r1, r2, #8
	68a6      	ldr	r6, [r4, #8]
	f8cd 505a 	str.w	r5, [sp, #90]	@ 0x5a
	f8ad 505e 	strh.w	r5, [sp, #94]	@ 0x5e
	9314      	str	r3, [sp, #80]	@ 0x50
	f88d 2056 	strb.w	r2, [sp, #86]	@ 0x56
	f8ad 1057 	strh.w	r1, [sp, #87]	@ 0x57
	e9d4 9800 	ldrd	r9, r8, [r4]
	e9d6 1401 	ldrd	r1, r4, [r6, #4]
	428c      	cmp	r4, r1
	f080 81bf 	bcs.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd8e>
	f44f 6cc6 	mov.w	ip, #1584	@ 0x630
	6833      	ldr	r3, [r6, #0]
	fb04 f50c 	mul.w	r5, r4, ip
	595d      	ldr	r5, [r3, r5]
	2d00      	cmp	r5, #0
	f100 81bd 	bmi.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd9c>
	1c65      	adds	r5, r4, #1
	f50a 70b4 	add.w	r0, sl, #360	@ 0x168
	fbb5 f2f1 	udiv	r2, r5, r1
	fb02 5111 	mls	r1, r2, r1, r5
	fb04 320c 	mla	r2, r4, ip, r3
	60b1      	str	r1, [r6, #8]
	6801      	ldr	r1, [r0, #0]
	8880      	ldrh	r0, [r0, #4]
	f8a2 0042 	strh.w	r0, [r2, #66]	@ 0x42
	f04f 30ff 	mov.w	r0, #4294967295	@ 0xffffffff
	6390      	str	r0, [r2, #56]	@ 0x38
	8790      	strh	r0, [r2, #60]	@ 0x3c
	f44f 60c1 	mov.w	r0, #1544	@ 0x608
	f8c2 103e 	str.w	r1, [r2, #62]	@ 0x3e
	f102 0146 	add.w	r1, r2, #70	@ 0x46
	f8a2 0044 	strh.w	r0, [r2, #68]	@ 0x44
	a813      	add	r0, sp, #76	@ 0x4c
	221c      	movs	r2, #28
	f7ff fa4b 	bl	<smoltcp::wire::arp::Repr::emit>
	6871      	ldr	r1, [r6, #4]
	428c      	cmp	r4, r1
	f080 81a6 	bcs.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xdb2>
	f44f 60c6 	mov.w	r0, #1584	@ 0x630
	6831      	ldr	r1, [r6, #0]
	fb04 1000 	mla	r0, r4, r0, r1
	f1b9 0f00 	cmp.w	r9, #0
	d006      	beq.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xa84>
	2101      	movs	r1, #1
	e9c0 1808 	strd	r1, r8, [r0, #32]
	2100      	movs	r1, #0
	f2cf 21d0 	movt	r1, #62160	@ 0xf2d0
	e006      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xa92>
	2100      	movs	r1, #0
	6201      	str	r1, [r0, #32]
	2100      	movs	r1, #0
	f2cf 21d0 	movt	r1, #62160	@ 0xf2d0
	f101 417e 	add.w	r1, r1, #4261412864	@ 0xfe000000
	6842      	ldr	r2, [r0, #4]
	242a      	movs	r4, #42	@ 0x2a
	e9d0 360a 	ldrd	r3, r6, [r0, #40]	@ 0x28
	4655      	mov	r5, sl
	f364 020b 	bfi	r2, r4, #0, #12
	6042      	str	r2, [r0, #4]
	6083      	str	r3, [r0, #8]
	60c6      	str	r6, [r0, #12]
	f3bf 8f5f 	dmb	sy
	f890 2030 	ldrb.w	r2, [r0, #48]	@ 0x30
	ea41 5142 	orr.w	r1, r1, r2, lsl #21
	6001      	str	r1, [r0, #0]
	f249 0004 	movw	r0, #36868	@ 0x9004
	2100      	movs	r1, #0
	f2c4 0002 	movt	r0, #16386	@ 0x4002
	f3bf 8f5f 	dmb	sy
	6001      	str	r1, [r0, #0]
	f244 2240 	movw	r2, #16960	@ 0x4240
	e9da 0104 	ldrd	r0, r1, [sl, #16]
	f2c0 020f 	movt	r2, #15
	1880      	adds	r0, r0, r2
	f141 0100 	adc.w	r1, r1, #0
	e9c5 013a 	strd	r0, r1, [r5, #232]	@ 0xe8
	f04f 0901 	mov.w	r9, #1
	4648      	mov	r0, r9
	b031      	add	sp, #196	@ 0xc4
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	9808      	ldr	r0, [sp, #32]
	2800      	cmp	r0, #0
	f43f ae7d 	beq.w	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7ea>
	e675      	b.n	<smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7de>
	f24a 5018 	movw	r0, #42264	@ 0xa518
	f24a 5250 	movw	r2, #42320	@ 0xa550
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2136      	movs	r1, #54	@ 0x36
	f002 fd60 	bl	<core::panicking::panic>
	f24b 00b8 	movw	r0, #45240	@ 0xb0b8
	f24b 02d8 	movw	r2, #45272	@ 0xb0d8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	211d      	movs	r1, #29
	f002 fd55 	bl	<core::panicking::panic>
	f64a 13ec 	movw	r3, #43500	@ 0xa9ec
	2006      	movs	r0, #6
	f6c0 0300 	movt	r3, #2048	@ 0x800
	210c      	movs	r1, #12
	4662      	mov	r2, ip
	f002 fcb9 	bl	<core::slice::index::slice_index_fail>
	f64a 13fc 	movw	r3, #43516	@ 0xa9fc
	200c      	movs	r0, #12
	f6c0 0300 	movt	r3, #2048	@ 0x800
	210e      	movs	r1, #14
	4662      	mov	r2, ip
	f002 fcb0 	bl	<core::slice::index::slice_index_fail>
	f64a 630c 	movw	r3, #44556	@ 0xae0c
	2002      	movs	r0, #2
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2104      	movs	r1, #4
	4672      	mov	r2, lr
	f002 fca7 	bl	<core::slice::index::slice_index_fail>
	f64a 634c 	movw	r3, #44620	@ 0xae4c
	2004      	movs	r0, #4
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2106      	movs	r1, #6
	4672      	mov	r2, lr
	f002 fc9e 	bl	<core::slice::index::slice_index_fail>
	f64a 53cc 	movw	r3, #44492	@ 0xadcc
	2006      	movs	r0, #6
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2108      	movs	r1, #8
	4672      	mov	r2, lr
	f002 fc95 	bl	<core::slice::index::slice_index_fail>
	f64a 53fc 	movw	r3, #44540	@ 0xadfc
	200c      	movs	r0, #12
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2110      	movs	r1, #16
	4672      	mov	r2, lr
	f002 fc8c 	bl	<core::slice::index::slice_index_fail>
	f64a 53ec 	movw	r3, #44524	@ 0xadec
	2010      	movs	r0, #16
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2114      	movs	r1, #20
	4672      	mov	r2, lr
	f002 fc83 	bl	<core::slice::index::slice_index_fail>
	f64a 7358 	movw	r3, #44888	@ 0xaf58
	2008      	movs	r0, #8
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4631      	mov	r1, r6
	464a      	mov	r2, r9
	f002 fc7a 	bl	<core::slice::index::slice_index_fail>
	f64a 7388 	movw	r3, #44936	@ 0xaf88
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2000      	movs	r0, #0
	2102      	movs	r1, #2
	464a      	mov	r2, r9
	f002 fc71 	bl	<core::slice::index::slice_index_fail>
	f64a 3310 	movw	r3, #43792	@ 0xab10
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2000      	movs	r0, #0
	2102      	movs	r1, #2
	464a      	mov	r2, r9
	f002 fc68 	bl	<core::slice::index::slice_index_fail>
	f64a 7378 	movw	r3, #44920	@ 0xaf78
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2002      	movs	r0, #2
	2104      	movs	r1, #4
	464a      	mov	r2, r9
	f002 fc5f 	bl	<core::slice::index::slice_index_fail>
	f64a 3300 	movw	r3, #43776	@ 0xab00
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2002      	movs	r0, #2
	2104      	movs	r1, #4
	464a      	mov	r2, r9
	f002 fc56 	bl	<core::slice::index::slice_index_fail>
	f64a 7398 	movw	r3, #44952	@ 0xaf98
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2004      	movs	r0, #4
	2106      	movs	r1, #6
	464a      	mov	r2, r9
	f002 fc4d 	bl	<core::slice::index::slice_index_fail>
	f64a 3340 	movw	r3, #43840	@ 0xab40
	2004      	movs	r0, #4
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2108      	movs	r1, #8
	464a      	mov	r2, r9
	f002 fc44 	bl	<core::slice::index::slice_index_fail>
	f64a 3330 	movw	r3, #43824	@ 0xab30
	2008      	movs	r0, #8
	f6c0 0300 	movt	r3, #2048	@ 0x800
	210c      	movs	r1, #12
	464a      	mov	r2, r9
	f002 fc3b 	bl	<core::slice::index::slice_index_fail>
	9a11      	ldr	r2, [sp, #68]	@ 0x44
	f64a 3350 	movw	r3, #43856	@ 0xab50
	f6c0 0300 	movt	r3, #2048	@ 0x800
	200e      	movs	r0, #14
	2110      	movs	r1, #16
	f002 fc32 	bl	<core::slice::index::slice_index_fail>
	f64a 73a8 	movw	r3, #44968	@ 0xafa8
	4611      	mov	r1, r2
	f6c0 0300 	movt	r3, #2048	@ 0x800
	461a      	mov	r2, r3
	f003 f8b7 	bl	<core::slice::copy_from_slice_impl::len_mismatch_fail>
	f64a 7368 	movw	r3, #44904	@ 0xaf68
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2006      	movs	r0, #6
	2108      	movs	r1, #8
	464a      	mov	r2, r9
	f002 fc21 	bl	<core::slice::index::slice_index_fail>
	f64a 33b8 	movw	r3, #43960	@ 0xabb8
	4611      	mov	r1, r2
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2000      	movs	r0, #0
	4632      	mov	r2, r6
	f002 fc18 	bl	<core::slice::index::slice_index_fail>
	f64a 43e0 	movw	r3, #44256	@ 0xace0
	2008      	movs	r0, #8
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4649      	mov	r1, r9
	464a      	mov	r2, r9
	f002 fc0f 	bl	<core::slice::index::slice_index_fail>
	f64a 43c0 	movw	r3, #44224	@ 0xacc0
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2004      	movs	r0, #4
	2106      	movs	r1, #6
	464a      	mov	r2, r9
	f002 fc06 	bl	<core::slice::index::slice_index_fail>
	f64a 43d0 	movw	r3, #44240	@ 0xacd0
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2006      	movs	r0, #6
	2108      	movs	r1, #8
	464a      	mov	r2, r9
	f002 fbfd 	bl	<core::slice::index::slice_index_fail>
	f64a 3320 	movw	r3, #43808	@ 0xab20
	2012      	movs	r0, #18
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2114      	movs	r1, #20
	4622      	mov	r2, r4
	f002 fbf4 	bl	<core::slice::index::slice_index_fail>
	f64a 23f0 	movw	r3, #43760	@ 0xaaf0
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4621      	mov	r1, r4
	4622      	mov	r2, r4
	f002 fbec 	bl	<core::slice::index::slice_index_fail>
	f64a 5310 	movw	r3, #44304	@ 0xad10
	2014      	movs	r0, #20
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4621      	mov	r1, r4
	4622      	mov	r2, r4
	f002 fbe3 	bl	<core::slice::index::slice_index_fail>
	f64a 5330 	movw	r3, #44336	@ 0xad30
	2014      	movs	r0, #20
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4631      	mov	r1, r6
	4632      	mov	r2, r6
	f002 fbda 	bl	<core::slice::index::slice_index_fail>
	f64a 5300 	movw	r3, #44288	@ 0xad00
	4610      	mov	r0, r2
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4631      	mov	r1, r6
	461a      	mov	r2, r3
	f003 f85e 	bl	<core::slice::copy_from_slice_impl::len_mismatch_fail>
	f64a 5220 	movw	r2, #44320	@ 0xad20
	4621      	mov	r1, r4
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f003 f857 	bl	<core::slice::copy_from_slice_impl::len_mismatch_fail>
	f24b 0298 	movw	r2, #45208	@ 0xb098
	4630      	mov	r0, r6
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f002 fc42 	bl	<core::panicking::panic_bounds_check>
	2002      	movs	r0, #2
	9013      	str	r0, [sp, #76]	@ 0x4c
	a813      	add	r0, sp, #76	@ 0x4c
	f004 fff6 	bl	<core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>
	f64a 200c 	movw	r0, #43532	@ 0xaa0c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f002 fbae 	bl	<core::option::unwrap_failed>
	f64a 52dc 	movw	r2, #44508	@ 0xaddc
	2000      	movs	r0, #0
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2100      	movs	r1, #0
	f002 fc2f 	bl	<core::panicking::panic_bounds_check>
	f64a 623c 	movw	r2, #44604	@ 0xae3c
	2001      	movs	r0, #1
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2101      	movs	r1, #1
	f002 fc27 	bl	<core::panicking::panic_bounds_check>
	f64a 625c 	movw	r2, #44636	@ 0xae5c
	2008      	movs	r0, #8
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2108      	movs	r1, #8
	f002 fc1f 	bl	<core::panicking::panic_bounds_check>
	f64a 621c 	movw	r2, #44572	@ 0xae1c
	2009      	movs	r0, #9
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2109      	movs	r1, #9
	f002 fc17 	bl	<core::panicking::panic_bounds_check>
	f24b 0258 	movw	r2, #45144	@ 0xb058
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f002 fc11 	bl	<core::panicking::panic_bounds_check>
	f64a 42f0 	movw	r2, #44272	@ 0xacf0
	2001      	movs	r0, #1
	f6c0 0200 	movt	r2, #2048	@ 0x800
	4649      	mov	r1, r9
	f002 fc09 	bl	<core::panicking::panic_bounds_check>
	f24b 0298 	movw	r2, #45208	@ 0xb098
	4620      	mov	r0, r4
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f002 fc02 	bl	<core::panicking::panic_bounds_check>
	2002      	movs	r0, #2
	9028      	str	r0, [sp, #160]	@ 0xa0
	a828      	add	r0, sp, #160	@ 0xa0
	f004 ffb6 	bl	<core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>
	f64a 200c 	movw	r0, #43532	@ 0xaa0c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f002 fb6e 	bl	<core::option::unwrap_failed>
	f24b 0258 	movw	r2, #45144	@ 0xb058
	4620      	mov	r0, r4
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f002 fbf0 	bl	<core::panicking::panic_bounds_check>

<smoltcp::iface::interface::Interface::poll>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	f5ad 7d5d 	sub.w	sp, sp, #884	@ 0x374
	aebe      	add	r6, sp, #760	@ 0x2f8
	f500 7bb8 	add.w	fp, r0, #368	@ 0x170
	1c71      	adds	r1, r6, #1
	913f      	str	r1, [sp, #252]	@ 0xfc
	a98c      	add	r1, sp, #560	@ 0x230
	e9c0 2304 	strd	r2, r3, [r0, #16]
	f101 0510 	add.w	r5, r1, #16
	9536      	str	r5, [sp, #216]	@ 0xd8
	f106 0554 	add.w	r5, r6, #84	@ 0x54
	9532      	str	r5, [sp, #200]	@ 0xc8
	ada5      	add	r5, sp, #660	@ 0x294
	904a      	str	r0, [sp, #296]	@ 0x128
	f105 0454 	add.w	r4, r5, #84	@ 0x54
	9431      	str	r4, [sp, #196]	@ 0xc4
	f101 040d 	add.w	r4, r1, #13
	3124      	adds	r1, #36	@ 0x24
	9133      	str	r1, [sp, #204]	@ 0xcc
	a954      	add	r1, sp, #336	@ 0x150
	3102      	adds	r1, #2
	913a      	str	r1, [sp, #232]	@ 0xe8
	f500 7180 	add.w	r1, r0, #256	@ 0x100
	9135      	str	r1, [sp, #212]	@ 0xd4
	f100 0120 	add.w	r1, r0, #32
	913e      	str	r1, [sp, #248]	@ 0xf8
	f100 01f4 	add.w	r1, r0, #244	@ 0xf4
	9144      	str	r1, [sp, #272]	@ 0x110
	f500 71b4 	add.w	r1, r0, #360	@ 0x168
	a857      	add	r0, sp, #348	@ 0x15c
	9146      	str	r1, [sp, #280]	@ 0x118
	f100 011c 	add.w	r1, r0, #28
	9142      	str	r1, [sp, #264]	@ 0x108
	f100 0113 	add.w	r1, r0, #19
	3006      	adds	r0, #6
	9040      	str	r0, [sp, #256]	@ 0x100
	f105 000f 	add.w	r0, r5, #15
	9039      	str	r0, [sp, #228]	@ 0xe4
	1ca8      	adds	r0, r5, #2
	9038      	str	r0, [sp, #224]	@ 0xe0
	f106 001c 	add.w	r0, r6, #28
	903d      	str	r0, [sp, #244]	@ 0xf4
	f106 0013 	add.w	r0, r6, #19
	903c      	str	r0, [sp, #240]	@ 0xf0
	1db0      	adds	r0, r6, #6
	903b      	str	r0, [sp, #236]	@ 0xec
	f8d7 a008 	ldr.w	sl, [r7, #8]
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	9434      	str	r4, [sp, #208]	@ 0xd0
	f249 0408 	movw	r4, #36872	@ 0x9008
	f10a 080c 	add.w	r8, sl, #12
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	f04f 0c01 	mov.w	ip, #1
	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
	e9cd 2347 	strd	r2, r3, [sp, #284]	@ 0x11c
	9141      	str	r1, [sp, #260]	@ 0x104
	f8cd 810c 	str.w	r8, [sp, #268]	@ 0x10c
	f8cd b124 	str.w	fp, [sp, #292]	@ 0x124
	2000      	movs	r0, #0
	9045      	str	r0, [sp, #276]	@ 0x114
	2000      	movs	r0, #0
	9037      	str	r0, [sp, #220]	@ 0xdc
	e9da 1004 	ldrd	r1, r0, [sl, #16]
	4288      	cmp	r0, r1
	d317      	bcc.n	<smoltcp::iface::interface::Interface::poll+0xee>
	f24b 02a8 	movw	r2, #45224	@ 0xb0a8
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f002 fb8b 	bl	<core::panicking::panic_bounds_check>
	f240 002f 	movw	r0, #47	@ 0x2f
	4664      	mov	r4, ip
	f2c0 0000 	movt	r0, #0
	4675      	mov	r5, lr
	f003 f940 	bl	<defmt::export::acquire_header_and_release>
	46ae      	mov	lr, r5
	46a4      	mov	ip, r4
	f249 0408 	movw	r4, #36872	@ 0x9008
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	e9da 1004 	ldrd	r1, r0, [sl, #16]
	4288      	cmp	r0, r1
	d2e7      	bcs.n	<smoltcp::iface::interface::Interface::poll+0xbe>
	f44f 61c6 	mov.w	r1, #1584	@ 0x630
	4348      	muls	r0, r1
	f8d8 1000 	ldr.w	r1, [r8]
	5808      	ldr	r0, [r1, r0]
	2800      	cmp	r0, #0
	f101 8242 	bmi.w	<smoltcp::iface::interface::Interface::poll+0x1584>
	68e0      	ldr	r0, [r4, #12]
	f3c0 4042 	ubfx	r0, r0, #17, #3
	2807      	cmp	r0, #7
	d877      	bhi.n	<smoltcp::iface::interface::Interface::poll+0x1fa>
	fa0c f000 	lsl.w	r0, ip, r0
	f010 0faa 	tst.w	r0, #170	@ 0xaa
	d072      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1fa>
	e9da 1001 	ldrd	r1, r0, [sl, #4]
	4288      	cmp	r0, r1
	f082 80d0 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x22be>
	fb00 f20e 	mul.w	r2, r0, lr
	f8da 3000 	ldr.w	r3, [sl]
	589a      	ldr	r2, [r3, r2]
	2a00      	cmp	r2, #0
	f101 822b 	bmi.w	<smoltcp::iface::interface::Interface::poll+0x1584>
	f8da 2018 	ldr.w	r2, [sl, #24]
	2500      	movs	r5, #0
	f8cd 8138 	str.w	r8, [sp, #312]	@ 0x138
	1c56      	adds	r6, r2, #1
	f8ca 6018 	str.w	r6, [sl, #24]
	68fe      	ldr	r6, [r7, #12]
	954c      	str	r5, [sp, #304]	@ 0x130
	f8cd 914c 	str.w	r9, [sp, #332]	@ 0x14c
	9652      	str	r6, [sp, #328]	@ 0x148
	924b      	str	r2, [sp, #300]	@ 0x12c
	68e6      	ldr	r6, [r4, #12]
	9d4a      	ldr	r5, [sp, #296]	@ 0x128
	e9cd bb50 	strd	fp, fp, [sp, #320]	@ 0x140
	f3c6 4642 	ubfx	r6, r6, #17, #3
	954f      	str	r5, [sp, #316]	@ 0x13c
	2e07      	cmp	r6, #7
	d858      	bhi.n	<smoltcp::iface::interface::Interface::poll+0x20e>
	fa0c f606 	lsl.w	r6, ip, r6
	f016 0faa 	tst.w	r6, #170	@ 0xaa
	d053      	beq.n	<smoltcp::iface::interface::Interface::poll+0x20e>
	fb00 340e 	mla	r4, r0, lr, r3
	6823      	ldr	r3, [r4, #0]
	2b00      	cmp	r3, #0
	d4ac      	bmi.n	<smoltcp::iface::interface::Interface::poll+0xca>
	6823      	ldr	r3, [r4, #0]
	f413 4300 	ands.w	r3, r3, #32768	@ 0x8000
	d106      	bne.n	<smoltcp::iface::interface::Interface::poll+0x186>
	6826      	ldr	r6, [r4, #0]
	05b6      	lsls	r6, r6, #22
	bf44      	itt	mi
	6826      	ldrmi	r6, [r4, #0]
	ea5f 56c6 	movsmi.w	r6, r6, lsl #23
	d44d      	bmi.n	<smoltcp::iface::interface::Interface::poll+0x222>
	6a20      	ldr	r0, [r4, #32]
	2800      	cmp	r0, #0
	f002 802e 	beq.w	<smoltcp::iface::interface::Interface::poll+0x21ea>
	e9d4 0109 	ldrd	r0, r1, [r4, #36]	@ 0x24
	2900      	cmp	r1, #0
	60a0      	str	r0, [r4, #8]
	f002 8033 	beq.w	<smoltcp::iface::interface::Interface::poll+0x2200>
	6ae0      	ldr	r0, [r4, #44]	@ 0x2c
	60e0      	str	r0, [r4, #12]
	f04f 4000 	mov.w	r0, #2147483648	@ 0x80000000
	f3bf 8f5f 	dmb	sy
	6020      	str	r0, [r4, #0]
	0bdc      	lsrs	r4, r3, #15
	2c02      	cmp	r4, #2
	f3bf 8f5f 	dmb	sy
	d08b      	beq.n	<smoltcp::iface::interface::Interface::poll+0xca>
	f240 002a 	movw	r0, #42	@ 0x2a
	46c1      	mov	r9, r8
	f2c0 0000 	movt	r0, #0
	46d8      	mov	r8, fp
	4666      	mov	r6, ip
	46f3      	mov	fp, lr
	f003 f8b9 	bl	<defmt::export::acquire_and_header>
	f240 0041 	movw	r0, #65	@ 0x41
	adbe      	add	r5, sp, #760	@ 0x2f8
	f2c0 0000 	movt	r0, #0
	2102      	movs	r1, #2
	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
	4628      	mov	r0, r5
	f003 fa60 	bl	<_defmt_write>
	4628      	mov	r0, r5
	2101      	movs	r1, #1
	f88d 42f8 	strb.w	r4, [sp, #760]	@ 0x2f8
	f003 fa5a 	bl	<_defmt_write>
	f003 f9e0 	bl	<_defmt_release>
	a84b      	add	r0, sp, #300	@ 0x12c
	46de      	mov	lr, fp
	46c3      	mov	fp, r8
	46c8      	mov	r8, r9
	4681      	mov	r9, r0
	46b4      	mov	ip, r6
	e771      	b.n	<smoltcp::iface::interface::Interface::poll+0xde>
	f8c4 c000 	str.w	ip, [r4]
	f8c4 c000 	str.w	ip, [r4]
	e9da 1001 	ldrd	r1, r0, [sl, #4]
	4288      	cmp	r0, r1
	d389      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x11e>
	f002 b858 	b.w	<smoltcp::iface::interface::Interface::poll+0x22be>
	f8c4 c000 	str.w	ip, [r4]
	f8c4 c000 	str.w	ip, [r4]
	fb00 340e 	mla	r4, r0, lr, r3
	6823      	ldr	r3, [r4, #0]
	2b00      	cmp	r3, #0
	d5a7      	bpl.n	<smoltcp::iface::interface::Interface::poll+0x170>
	e753      	b.n	<smoltcp::iface::interface::Interface::poll+0xca>
	f8d4 c000 	ldr.w	ip, [r4]
	f8d4 e000 	ldr.w	lr, [r4]
	2500      	movs	r5, #0
	69e3      	ldr	r3, [r4, #28]
	f8d4 8018 	ldr.w	r8, [r4, #24]
	f028 4600 	bic.w	r6, r8, #2147483648	@ 0x80000000
	ea46 76c3 	orr.w	r6, r6, r3, lsl #31
	f1d6 0b00 	rsbs	fp, r6, #0
	eb65 0a53 	sbc.w	sl, r5, r3, lsr #1
	f1b8 0f00 	cmp.w	r8, #0
	bf5c      	itt	pl
	46b3      	movpl	fp, r6
	ea4f 0a53 	movpl.w	sl, r3, lsr #1
	ea5f 630e 	movs.w	r3, lr, lsl #24
	f8d7 8008 	ldr.w	r8, [r7, #8]
	bf41      	itttt	mi
	6823      	ldrmi	r3, [r4, #0]
	ea5f 53c3 	movsmi.w	r3, r3, lsl #23
	f04f 0900 	movmi.w	r9, #0
	f04f 0e01 	movmi.w	lr, #1
	bf5c      	itt	pl
	f04f 0e00 	movpl.w	lr, #0
	f04f 0900 	movpl.w	r9, #0
	e9d8 3501 	ldrd	r3, r5, [r8, #4]
	3501      	adds	r5, #1
	e9c4 e90e 	strd	lr, r9, [r4, #56]	@ 0x38
	fbb5 f6f1 	udiv	r6, r5, r1
	4298      	cmp	r0, r3
	fb06 5111 	mls	r1, r6, r1, r5
	f04f 0501 	mov.w	r5, #1
	e9c4 520c 	strd	r5, r2, [r4, #48]	@ 0x30
	e9c4 ba10 	strd	fp, sl, [r4, #64]	@ 0x40
	f8c8 1008 	str.w	r1, [r8, #8]
	f082 8027 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x22e6>
	46c2      	mov	sl, r8
	9d43      	ldr	r5, [sp, #268]	@ 0x10c
	f8dd 8124 	ldr.w	r8, [sp, #292]	@ 0x124
	f3cc 410d 	ubfx	r1, ip, #16, #14
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
	f240 52f3 	movw	r2, #1523	@ 0x5f3
	4291      	cmp	r1, r2
	f081 87b0 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x2216>
	f8da 2000 	ldr.w	r2, [sl]
	fb00 260e 	mla	r6, r0, lr, r2
	b179      	cbz	r1, <smoltcp::iface::interface::Interface::poll+0x2e0>
	f106 0248 	add.w	r2, r6, #72	@ 0x48
	f249 0408 	movw	r4, #36872	@ 0x9008
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	f04f 0c01 	mov.w	ip, #1
	4610      	mov	r0, r2
	46c3      	mov	fp, r8
	290e      	cmp	r1, #14
	bf38      	it	cc
	2000      	movcc	r0, #0
	d20a      	bcs.n	<smoltcp::iface::interface::Interface::poll+0x2f2>
	46a8      	mov	r8, r5
	e0c1      	b.n	<smoltcp::iface::interface::Interface::poll+0x464>
	f249 0408 	movw	r4, #36872	@ 0x9008
	46c3      	mov	fp, r8
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	f04f 0c01 	mov.w	ip, #1
	46a8      	mov	r8, r5
	e0b8      	b.n	<smoltcp::iface::interface::Interface::poll+0x464>
	9b4b      	ldr	r3, [sp, #300]	@ 0x12c
	46b1      	mov	r9, r6
	932e      	str	r3, [sp, #184]	@ 0xb8
	68fb      	ldr	r3, [r7, #12]
	e9d3 3600 	ldrd	r3, r6, [r3]
	e9cd 632f 	strd	r6, r3, [sp, #188]	@ 0xbc
	6813      	ldr	r3, [r2, #0]
	07de      	lsls	r6, r3, #31
	d113      	bne.n	<smoltcp::iface::interface::Interface::poll+0x330>
	9e46      	ldr	r6, [sp, #280]	@ 0x118
	46cb      	mov	fp, r9
	8892      	ldrh	r2, [r2, #4]
	46b1      	mov	r9, r6
	88b6      	ldrh	r6, [r6, #4]
	f8d9 9000 	ldr.w	r9, [r9]
	4072      	eors	r2, r6
	ea83 0309 	eor.w	r3, r3, r9
	46d9      	mov	r9, fp
	b292      	uxth	r2, r2
	46c3      	mov	fp, r8
	431a      	orrs	r2, r3
	d004      	beq.n	<smoltcp::iface::interface::Interface::poll+0x330>
	464e      	mov	r6, r9
	46a8      	mov	r8, r5
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	e099      	b.n	<smoltcp::iface::interface::Interface::poll+0x464>
	f8b9 2054 	ldrh.w	r2, [r9, #84]	@ 0x54
	464e      	mov	r6, r9
	ba12      	rev	r2, r2
	0c12      	lsrs	r2, r2, #16
	f5b2 6f00 	cmp.w	r2, #2048	@ 0x800
	f000 8082 	beq.w	<smoltcp::iface::interface::Interface::poll+0x446>
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	46a8      	mov	r8, r5
	f640 0006 	movw	r0, #2054	@ 0x806
	4282      	cmp	r2, r0
	f040 8089 	bne.w	<smoltcp::iface::interface::Interface::poll+0x464>
	f1a1 020e 	sub.w	r2, r1, #14
	2a08      	cmp	r2, #8
	f0c0 8084 	bcc.w	<smoltcp::iface::interface::Interface::poll+0x464>
	f896 005a 	ldrb.w	r0, [r6, #90]	@ 0x5a
	f896 105b 	ldrb.w	r1, [r6, #91]	@ 0x5b
	4408      	add	r0, r1
	3004      	adds	r0, #4
	ebb2 0f40 	cmp.w	r2, r0, lsl #1
	d37a      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x464>
	f896 005a 	ldrb.w	r0, [r6, #90]	@ 0x5a
	f896 105b 	ldrb.w	r1, [r6, #91]	@ 0x5b
	180b      	adds	r3, r1, r0
	3304      	adds	r3, #4
	ebb2 0f43 	cmp.w	r2, r3, lsl #1
	d371      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x464>
	2904      	cmp	r1, #4
	bf08      	it	eq
	2806      	cmpeq	r0, #6
	d16d      	bne.n	<smoltcp::iface::interface::Interface::poll+0x464>
	f8b6 3056 	ldrh.w	r3, [r6, #86]	@ 0x56
	f5b3 7f80 	cmp.w	r3, #256	@ 0x100
	bf04      	itt	eq
	f8b6 3058 	ldrheq.w	r3, [r6, #88]	@ 0x58
	2b08      	cmpeq	r3, #8
	d164      	bne.n	<smoltcp::iface::interface::Interface::poll+0x464>
	2a0d      	cmp	r2, #13
	f241 87b0 	bls.w	<smoltcp::iface::interface::Interface::poll+0x2300>
	310e      	adds	r1, #14
	4291      	cmp	r1, r2
	f201 87b4 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x2310>
	eb01 0c00 	add.w	ip, r1, r0
	962d      	str	r6, [sp, #180]	@ 0xb4
	4594      	cmp	ip, r2
	f201 87b5 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x231e>
	2a1b      	cmp	r2, #27
	f241 87ba 	bls.w	<smoltcp::iface::interface::Interface::poll+0x232e>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	464d      	mov	r5, r9
	e9d0 1204 	ldrd	r1, r2, [r0, #16]
	912c      	str	r1, [sp, #176]	@ 0xb0
	992d      	ldr	r1, [sp, #180]	@ 0xb4
	922a      	str	r2, [sp, #168]	@ 0xa8
	f8d0 00f0 	ldr.w	r0, [r0, #240]	@ 0xf0
	f8d1 205f 	ldr.w	r2, [r1, #95]	@ 0x5f
	922e      	str	r2, [sp, #184]	@ 0xb8
	f891 205e 	ldrb.w	r2, [r1, #94]	@ 0x5e
	eb00 0980 	add.w	r9, r0, r0, lsl #2
	922f      	str	r2, [sp, #188]	@ 0xbc
	6e4a      	ldr	r2, [r1, #100]	@ 0x64
	f8b1 405c 	ldrh.w	r4, [r1, #92]	@ 0x5c
	f891 e063 	ldrb.w	lr, [r1, #99]	@ 0x63
	f8d1 606e 	ldr.w	r6, [r1, #110]	@ 0x6e
	9944      	ldr	r1, [sp, #272]	@ 0x110
	9230      	str	r2, [sp, #192]	@ 0xc0
	464a      	mov	r2, r9
	eb01 0309 	add.w	r3, r1, r9
	2a00      	cmp	r2, #0
	f000 80b9 	beq.w	<smoltcp::iface::interface::Interface::poll+0x56c>
	4608      	mov	r0, r1
	46b4      	mov	ip, r6
	f850 6b05 	ldr.w	r6, [r0], #5
	4566      	cmp	r6, ip
	4666      	mov	r6, ip
	f000 80b6 	beq.w	<smoltcp::iface::interface::Interface::poll+0x576>
	4298      	cmp	r0, r3
	f000 80ae 	beq.w	<smoltcp::iface::interface::Interface::poll+0x56c>
	f8d1 0005 	ldr.w	r0, [r1, #5]
	42b0      	cmp	r0, r6
	f000 80ae 	beq.w	<smoltcp::iface::interface::Interface::poll+0x576>
	f101 000a 	add.w	r0, r1, #10
	4298      	cmp	r0, r3
	f000 80a4 	beq.w	<smoltcp::iface::interface::Interface::poll+0x56c>
	f8d1 000a 	ldr.w	r0, [r1, #10]
	42b0      	cmp	r0, r6
	f000 80a4 	beq.w	<smoltcp::iface::interface::Interface::poll+0x576>
	f101 000f 	add.w	r0, r1, #15
	4298      	cmp	r0, r3
	f000 809a 	beq.w	<smoltcp::iface::interface::Interface::poll+0x56c>
	f8d1 000f 	ldr.w	r0, [r1, #15]
	3a14      	subs	r2, #20
	3114      	adds	r1, #20
	42b0      	cmp	r0, r6
	d1d7      	bne.n	<smoltcp::iface::interface::Interface::poll+0x3f4>
	e097      	b.n	<smoltcp::iface::interface::Interface::poll+0x576>
	f1a1 020e 	sub.w	r2, r1, #14
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	46a8      	mov	r8, r5
	2a14      	cmp	r2, #20
	d307      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x464>
	f100 050e 	add.w	r5, r0, #14
	213c      	movs	r1, #60	@ 0x3c
	7828      	ldrb	r0, [r5, #0]
	ea01 0380 	and.w	r3, r1, r0, lsl #2
	429a      	cmp	r2, r3
	d218      	bcs.n	<smoltcp::iface::interface::Interface::poll+0x496>
	6a30      	ldr	r0, [r6, #32]
	2800      	cmp	r0, #0
	f001 86bf 	beq.w	<smoltcp::iface::interface::Interface::poll+0x21ea>
	e9d6 0109 	ldrd	r0, r1, [r6, #36]	@ 0x24
	2900      	cmp	r1, #0
	60b0      	str	r0, [r6, #8]
	f001 86c4 	beq.w	<smoltcp::iface::interface::Interface::poll+0x2200>
	6af0      	ldr	r0, [r6, #44]	@ 0x2c
	60f0      	str	r0, [r6, #12]
	f04f 4000 	mov.w	r0, #2147483648	@ 0x80000000
	f3bf 8f5f 	dmb	sy
	6030      	str	r0, [r6, #0]
	f3bf 8f5f 	dmb	sy
	e9da 1004 	ldrd	r1, r0, [sl, #16]
	4288      	cmp	r0, r1
	f4ff ae2d 	bcc.w	<smoltcp::iface::interface::Interface::poll+0xee>
	e613      	b.n	<smoltcp::iface::interface::Interface::poll+0xbe>
	f8b6 1058 	ldrh.w	r1, [r6, #88]	@ 0x58
	ba09      	rev	r1, r1
	ebb3 4f11 	cmp.w	r3, r1, lsr #16
	d8e0      	bhi.n	<smoltcp::iface::interface::Interface::poll+0x464>
	0c09      	lsrs	r1, r1, #16
	428a      	cmp	r2, r1
	d3dd      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x464>
	f8b6 1058 	ldrh.w	r1, [r6, #88]	@ 0x58
	ba09      	rev	r1, r1
	ebb3 4f11 	cmp.w	r3, r1, lsr #16
	d8d7      	bhi.n	<smoltcp::iface::interface::Interface::poll+0x464>
	0c09      	lsrs	r1, r1, #16
	428a      	cmp	r2, r1
	912c      	str	r1, [sp, #176]	@ 0xb0
	d3d3      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x464>
	f000 01f0 	and.w	r1, r0, #240	@ 0xf0
	2940      	cmp	r1, #64	@ 0x40
	d1cf      	bne.n	<smoltcp::iface::interface::Interface::poll+0x464>
	f8b6 1052 	ldrh.w	r1, [r6, #82]	@ 0x52
	9127      	str	r1, [sp, #156]	@ 0x9c
	f8d6 104e 	ldr.w	r1, [r6, #78]	@ 0x4e
	9128      	str	r1, [sp, #160]	@ 0xa0
	994a      	ldr	r1, [sp, #296]	@ 0x128
	7a09      	ldrb	r1, [r1, #8]
	2901      	cmp	r1, #1
	d812      	bhi.n	<smoltcp::iface::interface::Interface::poll+0x4fe>
	0080      	lsls	r0, r0, #2
	9329      	str	r3, [sp, #164]	@ 0xa4
	b2c1      	uxtb	r1, r0
	428a      	cmp	r2, r1
	922a      	str	r2, [sp, #168]	@ 0xa8
	f0c1 86f2 	bcc.w	<smoltcp::iface::interface::Interface::poll+0x22ca>
	4628      	mov	r0, r5
	f006 fda9 	bl	<smoltcp::wire::ip::checksum::data>
	e9dd 3229 	ldrd	r3, r2, [sp, #164]	@ 0xa4
	43c0      	mvns	r0, r0
	f04f 0c01 	mov.w	ip, #1
	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
	0400      	lsls	r0, r0, #16
	d1b2      	bne.n	<smoltcp::iface::interface::Interface::poll+0x464>
	f8b6 005c 	ldrh.w	r0, [r6, #92]	@ 0x5c
	f06f 01c0 	mvn.w	r1, #192	@ 0xc0
	4208      	tst	r0, r1
	d1ac      	bne.n	<smoltcp::iface::interface::Interface::poll+0x464>
	9524      	str	r5, [sp, #144]	@ 0x90
	922a      	str	r2, [sp, #168]	@ 0xa8
	f896 005f 	ldrb.w	r0, [r6, #95]	@ 0x5f
	f8d6 2066 	ldr.w	r2, [r6, #102]	@ 0x66
	f8d6 5062 	ldr.w	r5, [r6, #98]	@ 0x62
	283c      	cmp	r0, #60	@ 0x3c
	962d      	str	r6, [sp, #180]	@ 0xb4
	f04f 060c 	mov.w	r6, #12
	f200 808c 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x63e>
	e8df f000 	tbb	[pc, r0]
	1f1f      	.short	0x1f1f
	8a8a8a1f 	.word	0x8a8a8a1f
	8a8a8a83 	.word	0x8a8a8a83
	8a8a8a8a 	.word	0x8a8a8a8a
	858a8a8a 	.word	0x858a8a8a
	8a8a8a8a 	.word	0x8a8a8a8a
	8a8a8a8a 	.word	0x8a8a8a8a
	8a8a8a8a 	.word	0x8a8a8a8a
	8a8a8a8a 	.word	0x8a8a8a8a
	8a8a8a8a 	.word	0x8a8a8a8a
	8a8a8a8a 	.word	0x8a8a8a8a
	8a7f7b8a 	.word	0x8a7f7b8a
	8a8a8a8a 	.word	0x8a8a8a8a
	8a8a8981 	.word	0x8a8a8981
	8a8a8a8a 	.word	0x8a8a8a8a
	007d7987 	.word	0x007d7987
	4606      	mov	r6, r0
	e068      	b.n	<smoltcp::iface::interface::Interface::poll+0x63e>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	f890 016e 	ldrb.w	r0, [r0, #366]	@ 0x16e
	2801      	cmp	r0, #1
	d118      	bne.n	<smoltcp::iface::interface::Interface::poll+0x5a8>
	f5b4 7f80 	cmp.w	r4, #256	@ 0x100
	bf18      	it	ne
	f5b4 7f00 	cmpne.w	r4, #512	@ 0x200
	d10c      	bne.n	<smoltcp::iface::interface::Interface::poll+0x59c>
	9930      	ldr	r1, [sp, #192]	@ 0xc0
	f001 00f0 	and.w	r0, r1, #240	@ 0xf0
	28e0      	cmp	r0, #224	@ 0xe0
	bf1c      	itt	ne
	1c48      	addne	r0, r1, #1
	2801      	cmpne	r0, #1
	d815      	bhi.n	<smoltcp::iface::interface::Interface::poll+0x5be>
	f240 0026 	movw	r0, #38	@ 0x26
	f2c0 0000 	movt	r0, #0
	e003      	b.n	<smoltcp::iface::interface::Interface::poll+0x5a4>
	f240 0028 	movw	r0, #40	@ 0x28
	f2c0 0000 	movt	r0, #0
	f002 fed9 	bl	<defmt::export::acquire_header_and_release>
	f249 0408 	movw	r4, #36872	@ 0x9008
	f04f 0c01 	mov.w	ip, #1
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
	46a9      	mov	r9, r5
	e752      	b.n	<smoltcp::iface::interface::Interface::poll+0x464>
	e9dd 202e 	ldrd	r2, r0, [sp, #184]	@ 0xb8
	ea4f 210e 	mov.w	r1, lr, lsl #8
	ea41 6112 	orr.w	r1, r1, r2, lsr #24
	ea40 2002 	orr.w	r0, r0, r2, lsl #8
	43c9      	mvns	r1, r1
	b289      	uxth	r1, r1
	ea61 0000 	orn	r0, r1, r0
	2800      	cmp	r0, #0
	d0db      	beq.n	<smoltcp::iface::interface::Interface::poll+0x592>
	982f      	ldr	r0, [sp, #188]	@ 0xbc
	f010 0001 	ands.w	r0, r0, #1
	d1d7      	bne.n	<smoltcp::iface::interface::Interface::poll+0x592>
	9844      	ldr	r0, [sp, #272]	@ 0x110
	f04f 3cff 	mov.w	ip, #4294967295	@ 0xffffffff
	4623      	mov	r3, r4
	e00e      	b.n	<smoltcp::iface::interface::Interface::poll+0x60a>
	4249      	negs	r1, r1
	f001 011f 	and.w	r1, r1, #31
	fa0c f101 	lsl.w	r1, ip, r1
	ba0a      	rev	r2, r1
	6801      	ldr	r1, [r0, #0]
	3005      	adds	r0, #5
	9c30      	ldr	r4, [sp, #192]	@ 0xc0
	f1a9 0905 	sub.w	r9, r9, #5
	4061      	eors	r1, r4
	420a      	tst	r2, r1
	f000 80f4 	beq.w	<smoltcp::iface::interface::Interface::poll+0x7f2>
	f1b9 0f00 	cmp.w	r9, #0
	f000 80eb 	beq.w	<smoltcp::iface::interface::Interface::poll+0x7e8>
	7901      	ldrb	r1, [r0, #4]
	2900      	cmp	r1, #0
	d1e9      	bne.n	<smoltcp::iface::interface::Interface::poll+0x5ec>
	2200      	movs	r2, #0
	e7ed      	b.n	<smoltcp::iface::interface::Interface::poll+0x5f8>
	260a      	movs	r6, #10
	e00e      	b.n	<smoltcp::iface::interface::Interface::poll+0x63e>
	2605      	movs	r6, #5
	e00c      	b.n	<smoltcp::iface::interface::Interface::poll+0x63e>
	260b      	movs	r6, #11
	e00a      	b.n	<smoltcp::iface::interface::Interface::poll+0x63e>
	2606      	movs	r6, #6
	e008      	b.n	<smoltcp::iface::interface::Interface::poll+0x63e>
	2607      	movs	r6, #7
	e006      	b.n	<smoltcp::iface::interface::Interface::poll+0x63e>
	2603      	movs	r6, #3
	e004      	b.n	<smoltcp::iface::interface::Interface::poll+0x63e>
	2604      	movs	r6, #4
	e002      	b.n	<smoltcp::iface::interface::Interface::poll+0x63e>
	2609      	movs	r6, #9
	e000      	b.n	<smoltcp::iface::interface::Interface::poll+0x63e>
	2608      	movs	r6, #8
	992c      	ldr	r1, [sp, #176]	@ 0xb0
	9329      	str	r3, [sp, #164]	@ 0xa4
	1ac9      	subs	r1, r1, r3
	9123      	str	r1, [sp, #140]	@ 0x8c
	992d      	ldr	r1, [sp, #180]	@ 0xb4
	922b      	str	r2, [sp, #172]	@ 0xac
	f891 305e 	ldrb.w	r3, [r1, #94]	@ 0x5e
	9926      	ldr	r1, [sp, #152]	@ 0x98
	f021 01ff 	bic.w	r1, r1, #255	@ 0xff
	4401      	add	r1, r0
	f005 00f0 	and.w	r0, r5, #240	@ 0xf0
	28e0      	cmp	r0, #224	@ 0xe0
	e9cd 5125 	strd	r5, r1, [sp, #148]	@ 0x94
	bf18      	it	ne
	f115 0001 	addsne.w	r0, r5, #1
	d10f      	bne.n	<smoltcp::iface::interface::Interface::poll+0x688>
	f240 0029 	movw	r0, #41	@ 0x29
	4664      	mov	r4, ip
	f2c0 0000 	movt	r0, #0
	4675      	mov	r5, lr
	f002 fe71 	bl	<defmt::export::acquire_header_and_release>
	46a4      	mov	ip, r4
	f249 0408 	movw	r4, #36872	@ 0x9008
	46ae      	mov	lr, r5
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
	e6ed      	b.n	<smoltcp::iface::interface::Interface::poll+0x464>
	e9cd 3621 	strd	r3, r6, [sp, #132]	@ 0x84
	b375      	cbz	r5, <smoltcp::iface::interface::Interface::poll+0x6ec>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	f8d0 00f0 	ldr.w	r0, [r0, #240]	@ 0xf0
	b330      	cbz	r0, <smoltcp::iface::interface::Interface::poll+0x6e4>
	9944      	ldr	r1, [sp, #272]	@ 0x110
	eb00 0080 	add.w	r0, r0, r0, lsl #2
	4408      	add	r0, r1
	e00f      	b.n	<smoltcp::iface::interface::Interface::poll+0x6c0>
	2600      	movs	r6, #0
	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
	f002 021f 	and.w	r2, r2, #31
	4033      	ands	r3, r6
	fa24 f202 	lsr.w	r2, r4, r2
	ba12      	rev	r2, r2
	431a      	orrs	r2, r3
	4295      	cmp	r5, r2
	f000 8091 	beq.w	<smoltcp::iface::interface::Interface::poll+0x7dc>
	3105      	adds	r1, #5
	4281      	cmp	r1, r0
	d011      	beq.n	<smoltcp::iface::interface::Interface::poll+0x6e4>
	790a      	ldrb	r2, [r1, #4]
	680b      	ldr	r3, [r1, #0]
	2a00      	cmp	r2, #0
	d0eb      	beq.n	<smoltcp::iface::interface::Interface::poll+0x6a0>
	f1a2 061f 	sub.w	r6, r2, #31
	b2f6      	uxtb	r6, r6
	2e02      	cmp	r6, #2
	d3f3      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x6ba>
	4256      	negs	r6, r2
	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
	f006 061f 	and.w	r6, r6, #31
	fa04 f606 	lsl.w	r6, r4, r6
	ba36      	rev	r6, r6
	e7e0      	b.n	<smoltcp::iface::interface::Interface::poll+0x6a6>
	fab5 f085 	clz	r0, r5
	ea4f 1c50 	mov.w	ip, r0, lsr #5
	982d      	ldr	r0, [sp, #180]	@ 0xb4
	9c29      	ldr	r4, [sp, #164]	@ 0xa4
	9a2a      	ldr	r2, [sp, #168]	@ 0xa8
	f8b0 0058 	ldrh.w	r0, [r0, #88]	@ 0x58
	9b24      	ldr	r3, [sp, #144]	@ 0x90
	ba00      	rev	r0, r0
	0c01      	lsrs	r1, r0, #16
	ebb4 4f10 	cmp.w	r4, r0, lsr #16
	f201 8624 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x234c>
	428a      	cmp	r2, r1
	f0c1 8621 	bcc.w	<smoltcp::iface::interface::Interface::poll+0x234c>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	191e      	adds	r6, r3, r4
	9b44      	ldr	r3, [sp, #272]	@ 0x110
	e9cd 6629 	strd	r6, r6, [sp, #164]	@ 0xa4
	f8d0 e0f0 	ldr.w	lr, [r0, #240]	@ 0xf0
	1b08      	subs	r0, r1, r4
	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
	902c      	str	r0, [sp, #176]	@ 0xb0
	f8cd c080 	str.w	ip, [sp, #128]	@ 0x80
	eb0e 028e 	add.w	r2, lr, lr, lsl #2
	1899      	adds	r1, r3, r2
	b322      	cbz	r2, <smoltcp::iface::interface::Interface::poll+0x776>
	4618      	mov	r0, r3
	9d2b      	ldr	r5, [sp, #172]	@ 0xac
	f850 6b05 	ldr.w	r6, [r0], #5
	42ae      	cmp	r6, r5
	f000 8126 	beq.w	<smoltcp::iface::interface::Interface::poll+0x986>
	4288      	cmp	r0, r1
	d01b      	beq.n	<smoltcp::iface::interface::Interface::poll+0x776>
	f8d3 0005 	ldr.w	r0, [r3, #5]
	9e2b      	ldr	r6, [sp, #172]	@ 0xac
	42b0      	cmp	r0, r6
	f000 811e 	beq.w	<smoltcp::iface::interface::Interface::poll+0x986>
	f103 000a 	add.w	r0, r3, #10
	4288      	cmp	r0, r1
	d011      	beq.n	<smoltcp::iface::interface::Interface::poll+0x776>
	f8d3 000a 	ldr.w	r0, [r3, #10]
	9e2b      	ldr	r6, [sp, #172]	@ 0xac
	42b0      	cmp	r0, r6
	f000 8114 	beq.w	<smoltcp::iface::interface::Interface::poll+0x986>
	f103 000f 	add.w	r0, r3, #15
	4288      	cmp	r0, r1
	d007      	beq.n	<smoltcp::iface::interface::Interface::poll+0x776>
	f8d3 000f 	ldr.w	r0, [r3, #15]
	3a14      	subs	r2, #20
	9e2b      	ldr	r6, [sp, #172]	@ 0xac
	3314      	adds	r3, #20
	42b0      	cmp	r0, r6
	d1da      	bne.n	<smoltcp::iface::interface::Interface::poll+0x72a>
	e107      	b.n	<smoltcp::iface::interface::Interface::poll+0x986>
	982b      	ldr	r0, [sp, #172]	@ 0xac
	9d25      	ldr	r5, [sp, #148]	@ 0x94
	f110 0c01 	adds.w	ip, r0, #1
	f000 810d 	beq.w	<smoltcp::iface::interface::Interface::poll+0x99c>
	9a2b      	ldr	r2, [sp, #172]	@ 0xac
	20e0      	movs	r0, #224	@ 0xe0
	f2c0 1000 	movt	r0, #256	@ 0x100
	4282      	cmp	r2, r0
	f000 8106 	beq.w	<smoltcp::iface::interface::Interface::poll+0x99c>
	f1be 0f00 	cmp.w	lr, #0
	f000 80d4 	beq.w	<smoltcp::iface::interface::Interface::poll+0x940>
	9b44      	ldr	r3, [sp, #272]	@ 0x110
	e00f      	b.n	<smoltcp::iface::interface::Interface::poll+0x7bc>
	2000      	movs	r0, #0
	4010      	ands	r0, r2
	f006 021f 	and.w	r2, r6, #31
	fa24 f202 	lsr.w	r2, r4, r2
	ba12      	rev	r2, r2
	4310      	orrs	r0, r2
	9a2b      	ldr	r2, [sp, #172]	@ 0xac
	4282      	cmp	r2, r0
	f000 80e3 	beq.w	<smoltcp::iface::interface::Interface::poll+0x97a>
	3305      	adds	r3, #5
	428b      	cmp	r3, r1
	f000 80c2 	beq.w	<smoltcp::iface::interface::Interface::poll+0x940>
	791e      	ldrb	r6, [r3, #4]
	681a      	ldr	r2, [r3, #0]
	2e00      	cmp	r6, #0
	d0eb      	beq.n	<smoltcp::iface::interface::Interface::poll+0x79c>
	f1a6 001f 	sub.w	r0, r6, #31
	b2c0      	uxtb	r0, r0
	2802      	cmp	r0, #2
	d3f2      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x7b4>
	4270      	negs	r0, r6
	f000 001f 	and.w	r0, r0, #31
	fa04 f000 	lsl.w	r0, r4, r0
	ba00      	rev	r0, r0
	e7e0      	b.n	<smoltcp::iface::interface::Interface::poll+0x79e>
	2d00      	cmp	r5, #0
	f47f af43 	bne.w	<smoltcp::iface::interface::Interface::poll+0x668>
	f04f 0c01 	mov.w	ip, #1
	e781      	b.n	<smoltcp::iface::interface::Interface::poll+0x6ec>
	f240 0027 	movw	r0, #39	@ 0x27
	f2c0 0000 	movt	r0, #0
	e6d8      	b.n	<smoltcp::iface::interface::Interface::poll+0x5a4>
	983f      	ldr	r0, [sp, #252]	@ 0xfc
	461c      	mov	r4, r3
	992e      	ldr	r1, [sp, #184]	@ 0xb8
	46b3      	mov	fp, r6
	9b2a      	ldr	r3, [sp, #168]	@ 0xa8
	4675      	mov	r5, lr
	6001      	str	r1, [r0, #0]
	f880 e004 	strb.w	lr, [r0, #4]
	0e09      	lsrs	r1, r1, #24
	982f      	ldr	r0, [sp, #188]	@ 0xbc
	f88d 02f8 	strb.w	r0, [sp, #760]	@ 0x2f8
	982c      	ldr	r0, [sp, #176]	@ 0xb0
	e9cd 0300 	strd	r0, r3, [sp]
	fa5f f08e 	uxtb.w	r0, lr
	ea41 2300 	orr.w	r3, r1, r0, lsl #8
	9abe      	ldr	r2, [sp, #760]	@ 0x2f8
	983e      	ldr	r0, [sp, #248]	@ 0xf8
	9930      	ldr	r1, [sp, #192]	@ 0xc0
	f006 fe12 	bl	<smoltcp::iface::neighbor::Cache::fill>
	f5b4 7f80 	cmp.w	r4, #256	@ 0x100
	f040 8098 	bne.w	<smoltcp::iface::interface::Interface::poll+0x95c>
	9946      	ldr	r1, [sp, #280]	@ 0x118
	9a3a      	ldr	r2, [sp, #232]	@ 0xe8
	f8cd b29e 	str.w	fp, [sp, #670]	@ 0x29e
	6808      	ldr	r0, [r1, #0]
	8889      	ldrh	r1, [r1, #4]
	8091      	strh	r1, [r2, #4]
	6010      	str	r0, [r2, #0]
	9a38      	ldr	r2, [sp, #224]	@ 0xe0
	e9dd 0154 	ldrd	r0, r1, [sp, #336]	@ 0x150
	6010      	str	r0, [r2, #0]
	9839      	ldr	r0, [sp, #228]	@ 0xe4
	6051      	str	r1, [r2, #4]
	992e      	ldr	r1, [sp, #184]	@ 0xb8
	6001      	str	r1, [r0, #0]
	e9da 1404 	ldrd	r1, r4, [sl, #16]
	7105      	strb	r5, [r0, #4]
	2001      	movs	r0, #1
	f8ad 0294 	strh.w	r0, [sp, #660]	@ 0x294
	428c      	cmp	r4, r1
	982f      	ldr	r0, [sp, #188]	@ 0xbc
	f88d 02a2 	strb.w	r0, [sp, #674]	@ 0x2a2
	9830      	ldr	r0, [sp, #192]	@ 0xc0
	90aa      	str	r0, [sp, #680]	@ 0x2a8
	f081 8582 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x236c>
	f44f 60c6 	mov.w	r0, #1584	@ 0x630
	fb04 f600 	mul.w	r6, r4, r0
	9843      	ldr	r0, [sp, #268]	@ 0x10c
	6800      	ldr	r0, [r0, #0]
	5982      	ldr	r2, [r0, r6]
	2a00      	cmp	r2, #0
	f101 857f 	bmi.w	<smoltcp::iface::interface::Interface::poll+0x237a>
	1c62      	adds	r2, r4, #1
	f8d7 8008 	ldr.w	r8, [r7, #8]
	fbb2 f3f1 	udiv	r3, r2, r1
	fb03 2111 	mls	r1, r3, r1, r2
	f44f 62c6 	mov.w	r2, #1584	@ 0x630
	9b2e      	ldr	r3, [sp, #184]	@ 0xb8
	fb04 0002 	mla	r0, r4, r2, r0
	9a46      	ldr	r2, [sp, #280]	@ 0x118
	f8c8 1014 	str.w	r1, [r8, #20]
	6811      	ldr	r1, [r2, #0]
	8892      	ldrh	r2, [r2, #4]
	f8c0 103e 	str.w	r1, [r0, #62]	@ 0x3e
	f44f 61c1 	mov.w	r1, #1544	@ 0x608
	f8a0 1044 	strh.w	r1, [r0, #68]	@ 0x44
	992f      	ldr	r1, [sp, #188]	@ 0xbc
	f8a0 2042 	strh.w	r2, [r0, #66]	@ 0x42
	221c      	movs	r2, #28
	f880 1038 	strb.w	r1, [r0, #56]	@ 0x38
	f100 0146 	add.w	r1, r0, #70	@ 0x46
	f880 503d 	strb.w	r5, [r0, #61]	@ 0x3d
	f8c0 3039 	str.w	r3, [r0, #57]	@ 0x39
	a8a5      	add	r0, sp, #660	@ 0x294
	f7fe fc36 	bl	<smoltcp::wire::arp::Repr::emit>
	f8d8 1010 	ldr.w	r1, [r8, #16]
	428c      	cmp	r4, r1
	f081 855f 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x2390>
	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
	f44f 61c6 	mov.w	r1, #1584	@ 0x630
	f04f 0900 	mov.w	r9, #0
	f8d7 a008 	ldr.w	sl, [r7, #8]
	f8d8 0000 	ldr.w	r0, [r8]
	fb04 0101 	mla	r1, r4, r1, r0
	242a      	movs	r4, #42	@ 0x2a
	684a      	ldr	r2, [r1, #4]
	e9d1 3c0a 	ldrd	r3, ip, [r1, #40]	@ 0x28
	f364 020b 	bfi	r2, r4, #0, #12
	604a      	str	r2, [r1, #4]
	608b      	str	r3, [r1, #8]
	2200      	movs	r2, #0
	f8c1 9020 	str.w	r9, [r1, #32]
	f249 0408 	movw	r4, #36872	@ 0x9008
	f8c1 c00c 	str.w	ip, [r1, #12]
	f2cf 02d0 	movt	r2, #61648	@ 0xf0d0
	f3bf 8f5f 	dmb	sy
	f891 1030 	ldrb.w	r1, [r1, #48]	@ 0x30
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	ea42 5141 	orr.w	r1, r2, r1, lsl #21
	5181      	str	r1, [r0, r6]
	f3bf 8f5f 	dmb	sy
	2001      	movs	r0, #1
	f844 9c04 	str.w	r9, [r4, #-4]
	901c      	str	r0, [sp, #112]	@ 0x70
	9830      	ldr	r0, [sp, #192]	@ 0xc0
	901b      	str	r0, [sp, #108]	@ 0x6c
	982e      	ldr	r0, [sp, #184]	@ 0xb8
	f8cd b074 	str.w	fp, [sp, #116]	@ 0x74
	e9cd 501e 	strd	r5, r0, [sp, #120]	@ 0x78
	982f      	ldr	r0, [sp, #188]	@ 0xbc
	9045      	str	r0, [sp, #276]	@ 0x114
	9849      	ldr	r0, [sp, #292]	@ 0x124
	e014      	b.n	<smoltcp::iface::interface::Interface::poll+0x96a>
	982b      	ldr	r0, [sp, #172]	@ 0xac
	f000 00f0 	and.w	r0, r0, #240	@ 0xf0
	28e0      	cmp	r0, #224	@ 0xe0
	bf1e      	ittt	ne
	984a      	ldrne	r0, [sp, #296]	@ 0x128
	f890 016e 	ldrbne.w	r0, [r0, #366]	@ 0x16e
	ea5f 70c0 	movsne.w	r0, r0, lsl #31
	f040 81e2 	bne.w	<smoltcp::iface::interface::Interface::poll+0xd1c>
	f8d7 a008 	ldr.w	sl, [r7, #8]
	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
	f249 0408 	movw	r4, #36872	@ 0x9008
	9849      	ldr	r0, [sp, #292]	@ 0x124
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	f04f 0c01 	mov.w	ip, #1
	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
	4683      	mov	fp, r0
	e684      	b.n	<smoltcp::iface::interface::Interface::poll+0x684>
	f8d7 a008 	ldr.w	sl, [r7, #8]
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
	982b      	ldr	r0, [sp, #172]	@ 0xac
	9d25      	ldr	r5, [sp, #148]	@ 0x94
	f000 00f0 	and.w	r0, r0, #240	@ 0xf0
	28e0      	cmp	r0, #224	@ 0xe0
	d004      	beq.n	<smoltcp::iface::interface::Interface::poll+0x99c>
	982b      	ldr	r0, [sp, #172]	@ 0xac
	3001      	adds	r0, #1
	2802      	cmp	r0, #2
	f080 816c 	bcs.w	<smoltcp::iface::interface::Interface::poll+0xc74>
	9922      	ldr	r1, [sp, #136]	@ 0x88
	2904      	cmp	r1, #4
	f000 80b8 	beq.w	<smoltcp::iface::interface::Interface::poll+0xb14>
	2903      	cmp	r1, #3
	d052      	beq.n	<smoltcp::iface::interface::Interface::poll+0xa4e>
	2901      	cmp	r1, #1
	f040 80cb 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb44>
	982c      	ldr	r0, [sp, #176]	@ 0xb0
	9929      	ldr	r1, [sp, #164]	@ 0xa4
	2808      	cmp	r0, #8
	bf38      	it	cc
	2100      	movcc	r1, #0
	2807      	cmp	r0, #7
	9129      	str	r1, [sp, #164]	@ 0xa4
	f240 80b9 	bls.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	7ac0      	ldrb	r0, [r0, #11]
	2801      	cmp	r0, #1
	d807      	bhi.n	<smoltcp::iface::interface::Interface::poll+0x9d8>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	992c      	ldr	r1, [sp, #176]	@ 0xb0
	f006 fb37 	bl	<smoltcp::wire::ip::checksum::data>
	43c0      	mvns	r0, r0
	0400      	lsls	r0, r0, #16
	f040 80ad 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	7800      	ldrb	r0, [r0, #0]
	280b      	cmp	r0, #11
	f200 80a8 	bhi.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	e8df f010 	tbh	[pc, r0, lsl #1]
	026e      	.short	0x026e
	00a600a6 	.word	0x00a600a6
	00a6000c 	.word	0x00a6000c
	00a600a6 	.word	0x00a600a6
	028100a6 	.word	0x028100a6
	00a600a6 	.word	0x00a600a6
	000c      	.short	0x000c
	982c      	ldr	r0, [sp, #176]	@ 0xb0
	f1a0 0108 	sub.w	r1, r0, #8
	2914      	cmp	r1, #20
	f0c0 8094 	bcc.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	9829      	ldr	r0, [sp, #164]	@ 0xa4
	233c      	movs	r3, #60	@ 0x3c
	3008      	adds	r0, #8
	7802      	ldrb	r2, [r0, #0]
	ea03 0282 	and.w	r2, r3, r2, lsl #2
	4291      	cmp	r1, r2
	f0c0 808b 	bcc.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	9b2a      	ldr	r3, [sp, #168]	@ 0xa8
	895b      	ldrh	r3, [r3, #10]
	ba1b      	rev	r3, r3
	ebb2 4f13 	cmp.w	r2, r3, lsr #16
	f200 8084 	bhi.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	0c1b      	lsrs	r3, r3, #16
	4299      	cmp	r1, r3
	bf24      	itt	cs
	1a89      	subcs	r1, r1, r2
	2908      	cmpcs	r1, #8
	d37d      	bcc.n	<smoltcp::iface::interface::Interface::poll+0xb32>
	992a      	ldr	r1, [sp, #168]	@ 0xa8
	4410      	add	r0, r2
	901a      	str	r0, [sp, #104]	@ 0x68
	2004      	movs	r0, #4
	9057      	str	r0, [sp, #348]	@ 0x15c
	9817      	ldr	r0, [sp, #92]	@ 0x5c
	7849      	ldrb	r1, [r1, #1]
	f020 00ff 	bic.w	r0, r0, #255	@ 0xff
	4408      	add	r0, r1
	9017      	str	r0, [sp, #92]	@ 0x5c
	e073      	b.n	<smoltcp::iface::interface::Interface::poll+0xb36>
	982c      	ldr	r0, [sp, #176]	@ 0xb0
	2814      	cmp	r0, #20
	d36e      	bcc.n	<smoltcp::iface::interface::Interface::poll+0xb32>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	213c      	movs	r1, #60	@ 0x3c
	8986      	ldrh	r6, [r0, #12]
	b2f0      	uxtb	r0, r6
	2850      	cmp	r0, #80	@ 0x50
	f04f 0000 	mov.w	r0, #0
	ea01 0596 	and.w	r5, r1, r6, lsr #2
	bf38      	it	cc
	2001      	movcc	r0, #1
	992c      	ldr	r1, [sp, #176]	@ 0xb0
	42a9      	cmp	r1, r5
	f04f 0100 	mov.w	r1, #0
	bf38      	it	cc
	2101      	movcc	r1, #1
	4308      	orrs	r0, r1
	2801      	cmp	r0, #1
	d05a      	beq.n	<smoltcp::iface::interface::Interface::poll+0xb32>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	8800      	ldrh	r0, [r0, #0]
	2800      	cmp	r0, #0
	bf1e      	ittt	ne
	982a      	ldrne	r0, [sp, #168]	@ 0xa8
	8840      	ldrhne	r0, [r0, #2]
	2800      	cmpne	r0, #0
	d052      	beq.n	<smoltcp::iface::interface::Interface::poll+0xb32>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	7a80      	ldrb	r0, [r0, #10]
	2801      	cmp	r0, #1
	d832      	bhi.n	<smoltcp::iface::interface::Interface::poll+0xafa>
	982b      	ldr	r0, [sp, #172]	@ 0xac
	ba00      	rev	r0, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	9925      	ldr	r1, [sp, #148]	@ 0x94
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	ba09      	rev	r1, r1
	b280      	uxth	r0, r0
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	eb01 4111 	add.w	r1, r1, r1, lsr #16
	fa10 f081 	uxtah	r0, r0, r1
	992c      	ldr	r1, [sp, #176]	@ 0xb0
	1d8b      	adds	r3, r1, #6
	0c1a      	lsrs	r2, r3, #16
	fa12 f283 	uxtah	r2, r2, r3
	eb02 4212 	add.w	r2, r2, r2, lsr #16
	fa10 f082 	uxtah	r0, r0, r2
	0c02      	lsrs	r2, r0, #16
	fa12 f080 	uxtah	r0, r2, r0
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	b284      	uxth	r4, r0
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	f006 faac 	bl	<smoltcp::wire::ip::checksum::data>
	fa14 f080 	uxtah	r0, r4, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	43c0      	mvns	r0, r0
	0400      	lsls	r0, r0, #16
	d11b      	bne.n	<smoltcp::iface::interface::Interface::poll+0xb32>
	05b0      	lsls	r0, r6, #22
	f100 8270 	bmi.w	<smoltcp::iface::interface::Interface::poll+0xfe0>
	05f0      	lsls	r0, r6, #23
	f100 8273 	bmi.w	<smoltcp::iface::interface::Interface::poll+0xfec>
	0570      	lsls	r0, r6, #21
	f04f 0004 	mov.w	r0, #4
	bf58      	it	pl
	f3c6 20c0 	ubfxpl	r0, r6, #11, #1
	e26f      	b.n	<smoltcp::iface::interface::Interface::poll+0xff4>
	982c      	ldr	r0, [sp, #176]	@ 0xb0
	2808      	cmp	r0, #8
	d30b      	bcc.n	<smoltcp::iface::interface::Interface::poll+0xb32>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	8880      	ldrh	r0, [r0, #4]
	ba00      	rev	r0, r0
	ea4f 4910 	mov.w	r9, r0, lsr #16
	f1b9 0f08 	cmp.w	r9, #8
	bf24      	itt	cs
	982c      	ldrcs	r0, [sp, #176]	@ 0xb0
	4548      	cmpcs	r0, r9
	f080 813d 	bcs.w	<smoltcp::iface::interface::Interface::poll+0xdac>
	2004      	movs	r0, #4
	9057      	str	r0, [sp, #348]	@ 0x15c
	f8d7 a008 	ldr.w	sl, [r7, #8]
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
	e021      	b.n	<smoltcp::iface::interface::Interface::poll+0xb88>
	9826      	ldr	r0, [sp, #152]	@ 0x98
	462a      	mov	r2, r5
	f8ad 01da 	strh.w	r0, [sp, #474]	@ 0x1da
	9821      	ldr	r0, [sp, #132]	@ 0x84
	f88d 01d8 	strb.w	r0, [sp, #472]	@ 0x1d8
	9823      	ldr	r0, [sp, #140]	@ 0x8c
	9b2b      	ldr	r3, [sp, #172]	@ 0xac
	f88d 11d9 	strb.w	r1, [sp, #473]	@ 0x1d9
	9075      	str	r0, [sp, #468]	@ 0x1d4
	f240 2002 	movw	r0, #514	@ 0x202
	992c      	ldr	r1, [sp, #176]	@ 0xb0
	f8ad 01c0 	strh.w	r0, [sp, #448]	@ 0x1c0
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	f5b1 7f04 	cmp.w	r1, #528	@ 0x210
	9071      	str	r0, [sp, #452]	@ 0x1c4
	f44f 7004 	mov.w	r0, #528	@ 0x210
	e9cd 5373 	strd	r5, r3, [sp, #460]	@ 0x1cc
	bf28      	it	cs
	4601      	movcs	r1, r0
	9172      	str	r1, [sp, #456]	@ 0x1c8
	a870      	add	r0, sp, #448	@ 0x1c0
	994a      	ldr	r1, [sp, #296]	@ 0x128
	9000      	str	r0, [sp, #0]
	a857      	add	r0, sp, #348	@ 0x15c
	f005 ff07 	bl	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>
	9c57      	ldr	r4, [sp, #348]	@ 0x15c
	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
	2c04      	cmp	r4, #4
	d10b      	bne.n	<smoltcp::iface::interface::Interface::poll+0xba8>
	f249 0408 	movw	r4, #36872	@ 0x9008
	f04f 0c01 	mov.w	ip, #1
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
	f8dd b124 	ldr.w	fp, [sp, #292]	@ 0x124
	f7ff bc5e 	b.w	<smoltcp::iface::interface::Interface::poll+0x464>
	9940      	ldr	r1, [sp, #256]	@ 0x100
	f8dd 016a 	ldr.w	r0, [sp, #362]	@ 0x16a
	901d      	str	r0, [sp, #116]	@ 0x74
	f89d 016e 	ldrb.w	r0, [sp, #366]	@ 0x16e
	9045      	str	r0, [sp, #276]	@ 0x114
	6808      	ldr	r0, [r1, #0]
	6849      	ldr	r1, [r1, #4]
	9a41      	ldr	r2, [sp, #260]	@ 0x104
	f8bd 5160 	ldrh.w	r5, [sp, #352]	@ 0x160
	e9cd 0154 	strd	r0, r1, [sp, #336]	@ 0x150
	a8a5      	add	r0, sp, #660	@ 0x294
	9942      	ldr	r1, [sp, #264]	@ 0x108
	7913      	ldrb	r3, [r2, #4]
	6812      	ldr	r2, [r2, #0]
	921f      	str	r2, [sp, #124]	@ 0x7c
	2248      	movs	r2, #72	@ 0x48
	931e      	str	r3, [sp, #120]	@ 0x78
	9e5d      	ldr	r6, [sp, #372]	@ 0x174
	f006 fe5a 	bl	<__aeabi_memcpy4>
	2c05      	cmp	r4, #5
	e9cd 651b 	strd	r6, r5, [sp, #108]	@ 0x6c
	d03c      	beq.n	<smoltcp::iface::interface::Interface::poll+0xc5a>
	9a3b      	ldr	r2, [sp, #236]	@ 0xec
	e9dd 0154 	ldrd	r0, r1, [sp, #336]	@ 0x150
	f8ad 52fc 	strh.w	r5, [sp, #764]	@ 0x2fc
	6010      	str	r0, [r2, #0]
	983c      	ldr	r0, [sp, #240]	@ 0xf0
	6051      	str	r1, [r2, #4]
	2248      	movs	r2, #72	@ 0x48
	991f      	ldr	r1, [sp, #124]	@ 0x7c
	6001      	str	r1, [r0, #0]
	991e      	ldr	r1, [sp, #120]	@ 0x78
	7101      	strb	r1, [r0, #4]
	a9a5      	add	r1, sp, #660	@ 0x294
	9845      	ldr	r0, [sp, #276]	@ 0x114
	f88d 030a 	strb.w	r0, [sp, #778]	@ 0x30a
	981d      	ldr	r0, [sp, #116]	@ 0x74
	f8cd 0306 	str.w	r0, [sp, #774]	@ 0x306
	983d      	ldr	r0, [sp, #244]	@ 0xf4
	94be      	str	r4, [sp, #760]	@ 0x2f8
	96c4      	str	r6, [sp, #784]	@ 0x310
	f006 fe3d 	bl	<__aeabi_memcpy4>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	a94c      	add	r1, sp, #304	@ 0x130
	abbe      	add	r3, sp, #760	@ 0x2f8
	2200      	movs	r2, #0
	f7fe fb11 	bl	<smoltcp::iface::interface::InterfaceInner::dispatch_ip>
	b2c4      	uxtb	r4, r0
	2c02      	cmp	r4, #2
	d01a      	beq.n	<smoltcp::iface::interface::Interface::poll+0xc5a>
	f240 0023 	movw	r0, #35	@ 0x23
	f2c0 0000 	movt	r0, #0
	f002 fb84 	bl	<defmt::export::acquire_and_header>
	f240 003e 	movw	r0, #62	@ 0x3e
	aebe      	add	r6, sp, #760	@ 0x2f8
	f2c0 0000 	movt	r0, #0
	2102      	movs	r1, #2
	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
	4630      	mov	r0, r6
	f002 fd2b 	bl	<_defmt_write>
	f004 0001 	and.w	r0, r4, #1
	f88d 02f8 	strb.w	r0, [sp, #760]	@ 0x2f8
	4630      	mov	r0, r6
	2101      	movs	r1, #1
	f002 fd23 	bl	<_defmt_write>
	f002 fca9 	bl	<_defmt_release>
	f249 0408 	movw	r4, #36872	@ 0x9008
	f04f 0c01 	mov.w	ip, #1
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
	f8dd b124 	ldr.w	fp, [sp, #292]	@ 0x124
	f7ff bbf8 	b.w	<smoltcp::iface::interface::Interface::poll+0x464>
	f1be 0f00 	cmp.w	lr, #0
	d023      	beq.n	<smoltcp::iface::interface::Interface::poll+0xcc2>
	9944      	ldr	r1, [sp, #272]	@ 0x110
	eb0e 008e 	add.w	r0, lr, lr, lsl #2
	4408      	add	r0, r1
	e00e      	b.n	<smoltcp::iface::interface::Interface::poll+0xca2>
	2600      	movs	r6, #0
	f002 021f 	and.w	r2, r2, #31
	4033      	ands	r3, r6
	fa24 f202 	lsr.w	r2, r4, r2
	ba12      	rev	r2, r2
	431a      	orrs	r2, r3
	9b2b      	ldr	r3, [sp, #172]	@ 0xac
	4293      	cmp	r3, r2
	f43f ae80 	beq.w	<smoltcp::iface::interface::Interface::poll+0x99c>
	3105      	adds	r1, #5
	4281      	cmp	r1, r0
	d00f      	beq.n	<smoltcp::iface::interface::Interface::poll+0xcc2>
	790a      	ldrb	r2, [r1, #4]
	680b      	ldr	r3, [r1, #0]
	2a00      	cmp	r2, #0
	d0ec      	beq.n	<smoltcp::iface::interface::Interface::poll+0xc84>
	f1a2 061f 	sub.w	r6, r2, #31
	b2f6      	uxtb	r6, r6
	2e02      	cmp	r6, #2
	d3f3      	bcc.n	<smoltcp::iface::interface::Interface::poll+0xc9c>
	4256      	negs	r6, r2
	f006 061f 	and.w	r6, r6, #31
	fa04 f606 	lsl.w	r6, r4, r6
	ba36      	rev	r6, r6
	e7e1      	b.n	<smoltcp::iface::interface::Interface::poll+0xc86>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	2200      	movs	r2, #0
	e9d0 1c04 	ldrd	r1, ip, [r0, #16]
	f8d0 00e0 	ldr.w	r0, [r0, #224]	@ 0xe0
	eba0 0080 	sub.w	r0, r0, r0, lsl #2
	00c3      	lsls	r3, r0, #3
	1898      	adds	r0, r3, r2
	f43f ae61 	beq.w	<smoltcp::iface::interface::Interface::poll+0x99c>
	9e4a      	ldr	r6, [sp, #296]	@ 0x128
	4416      	add	r6, r2
	6a34      	ldr	r4, [r6, #32]
	42ac      	cmp	r4, r5
	f000 814c 	beq.w	<smoltcp::iface::interface::Interface::poll+0xf7e>
	f110 0418 	adds.w	r4, r0, #24
	f43f ae57 	beq.w	<smoltcp::iface::interface::Interface::poll+0x99c>
	6bb4      	ldr	r4, [r6, #56]	@ 0x38
	42ac      	cmp	r4, r5
	f000 8147 	beq.w	<smoltcp::iface::interface::Interface::poll+0xf84>
	f110 0430 	adds.w	r4, r0, #48	@ 0x30
	f43f ae4f 	beq.w	<smoltcp::iface::interface::Interface::poll+0x99c>
	6d34      	ldr	r4, [r6, #80]	@ 0x50
	42ac      	cmp	r4, r5
	f000 8142 	beq.w	<smoltcp::iface::interface::Interface::poll+0xf8a>
	3048      	adds	r0, #72	@ 0x48
	f43f ae48 	beq.w	<smoltcp::iface::interface::Interface::poll+0x99c>
	6eb0      	ldr	r0, [r6, #104]	@ 0x68
	3260      	adds	r2, #96	@ 0x60
	42a8      	cmp	r0, r5
	d1df      	bne.n	<smoltcp::iface::interface::Interface::poll+0xcd4>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	4410      	add	r0, r2
	3008      	adds	r0, #8
	e138      	b.n	<smoltcp::iface::interface::Interface::poll+0xf8e>
	f1bc 0f02 	cmp.w	ip, #2
	f4ff ae1a 	bcc.w	<smoltcp::iface::interface::Interface::poll+0x958>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	9a2b      	ldr	r2, [sp, #172]	@ 0xac
	e9d0 0104 	ldrd	r0, r1, [r0, #16]
	e9cd 0100 	strd	r0, r1, [sp]
	a8be      	add	r0, sp, #760	@ 0x2f8
	9935      	ldr	r1, [sp, #212]	@ 0xd4
	f006 fae9 	bl	<smoltcp::iface::route::Routes::lookup>
	f89d 02f8 	ldrb.w	r0, [sp, #760]	@ 0x2f8
	2801      	cmp	r0, #1
	f47f ae0b 	bne.w	<smoltcp::iface::interface::Interface::poll+0x958>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	9e44      	ldr	r6, [sp, #272]	@ 0x110
	f8dd 12f9 	ldr.w	r1, [sp, #761]	@ 0x2f9
	f8d0 e0f0 	ldr.w	lr, [r0, #240]	@ 0xf0
	eb0e 028e 	add.w	r2, lr, lr, lsl #2
	18b3      	adds	r3, r6, r2
	2a00      	cmp	r2, #0
	f43f adff 	beq.w	<smoltcp::iface::interface::Interface::poll+0x958>
	4630      	mov	r0, r6
	f850 4b05 	ldr.w	r4, [r0], #5
	428c      	cmp	r4, r1
	d01a      	beq.n	<smoltcp::iface::interface::Interface::poll+0xd9a>
	4298      	cmp	r0, r3
	f43f adf7 	beq.w	<smoltcp::iface::interface::Interface::poll+0x958>
	f8d6 0005 	ldr.w	r0, [r6, #5]
	4288      	cmp	r0, r1
	d013      	beq.n	<smoltcp::iface::interface::Interface::poll+0xd9a>
	f106 000a 	add.w	r0, r6, #10
	4298      	cmp	r0, r3
	f43f adee 	beq.w	<smoltcp::iface::interface::Interface::poll+0x958>
	f8d6 000a 	ldr.w	r0, [r6, #10]
	4288      	cmp	r0, r1
	d00a      	beq.n	<smoltcp::iface::interface::Interface::poll+0xd9a>
	f106 000f 	add.w	r0, r6, #15
	4298      	cmp	r0, r3
	f43f ade5 	beq.w	<smoltcp::iface::interface::Interface::poll+0x958>
	f8d6 000f 	ldr.w	r0, [r6, #15]
	3a14      	subs	r2, #20
	3614      	adds	r6, #20
	4288      	cmp	r0, r1
	d1dc      	bne.n	<smoltcp::iface::interface::Interface::poll+0xd54>
	f8d7 a008 	ldr.w	sl, [r7, #8]
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
	e5ec      	b.n	<smoltcp::iface::interface::Interface::poll+0x986>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	8846      	ldrh	r6, [r0, #2]
	2e00      	cmp	r6, #0
	f43f aebe 	beq.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	7a40      	ldrb	r0, [r0, #9]
	2801      	cmp	r0, #1
	d837      	bhi.n	<smoltcp::iface::interface::Interface::poll+0xe2e>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	88c0      	ldrh	r0, [r0, #6]
	b3a0      	cbz	r0, <smoltcp::iface::interface::Interface::poll+0xe2e>
	982b      	ldr	r0, [sp, #172]	@ 0xac
	ba00      	rev	r0, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	f109 0111 	add.w	r1, r9, #17
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	b280      	uxth	r0, r0
	eb01 4111 	add.w	r1, r1, r1, lsr #16
	fa10 f081 	uxtah	r0, r0, r1
	9925      	ldr	r1, [sp, #148]	@ 0x94
	ba09      	rev	r1, r1
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	0c0a      	lsrs	r2, r1, #16
	fa12 f181 	uxtah	r1, r2, r1
	eb01 4111 	add.w	r1, r1, r1, lsr #16
	fa10 f081 	uxtah	r0, r0, r1
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	4649      	mov	r1, r9
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	b284      	uxth	r4, r0
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	f006 f913 	bl	<smoltcp::wire::ip::checksum::data>
	fa14 f080 	uxtah	r0, r4, r0
	0c01      	lsrs	r1, r0, #16
	fa11 f080 	uxtah	r0, r1, r0
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	43c0      	mvns	r0, r0
	0400      	lsls	r0, r0, #16
	f47f ae82 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	e9dd 252f 	ldrd	r2, r5, [sp, #188]	@ 0xbc
	f44f 71a8 	mov.w	r1, #336	@ 0x150
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	fa96 fa96 	rev16.w	sl, r6
	fb02 5401 	mla	r4, r2, r1, r5
	8800      	ldrh	r0, [r0, #0]
	ba00      	rev	r0, r0
	ea4f 4810 	mov.w	r8, r0, lsr #16
	e00c      	b.n	<smoltcp::iface::interface::Interface::poll+0xe64>
	4605      	mov	r5, r0
	4608      	mov	r0, r1
	f100 0608 	add.w	r6, r0, #8
	994a      	ldr	r1, [sp, #296]	@ 0x128
	9a2b      	ldr	r2, [sp, #172]	@ 0xac
	4653      	mov	r3, sl
	4630      	mov	r0, r6
	f005 ffe8 	bl	<smoltcp::socket::udp::Socket::accepts>
	2800      	cmp	r0, #0
	f040 80b0 	bne.w	<smoltcp::iface::interface::Interface::poll+0xfc4>
	42a5      	cmp	r5, r4
	d066      	beq.n	<smoltcp::iface::interface::Interface::poll+0xf36>
	e9d5 0200 	ldrd	r0, r2, [r5]
	4629      	mov	r1, r5
	f080 0002 	eor.w	r0, r0, #2
	4310      	orrs	r0, r2
	f505 70a8 	add.w	r0, r5, #336	@ 0x150
	d0e7      	beq.n	<smoltcp::iface::interface::Interface::poll+0xe4a>
	42a0      	cmp	r0, r4
	d05b      	beq.n	<smoltcp::iface::interface::Interface::poll+0xf36>
	e9d1 2354 	ldrd	r2, r3, [r1, #336]	@ 0x150
	f082 0202 	eor.w	r2, r2, #2
	431a      	orrs	r2, r3
	f501 7228 	add.w	r2, r1, #672	@ 0x2a0
	d101      	bne.n	<smoltcp::iface::interface::Interface::poll+0xe92>
	4615      	mov	r5, r2
	e7dd      	b.n	<smoltcp::iface::interface::Interface::poll+0xe4e>
	42a2      	cmp	r2, r4
	d04f      	beq.n	<smoltcp::iface::interface::Interface::poll+0xf36>
	e9d1 03a8 	ldrd	r0, r3, [r1, #672]	@ 0x2a0
	f080 0002 	eor.w	r0, r0, #2
	4318      	orrs	r0, r3
	f501 707c 	add.w	r0, r1, #1008	@ 0x3f0
	d102      	bne.n	<smoltcp::iface::interface::Interface::poll+0xeac>
	4605      	mov	r5, r0
	4610      	mov	r0, r2
	e7d0      	b.n	<smoltcp::iface::interface::Interface::poll+0xe4e>
	42a0      	cmp	r0, r4
	d042      	beq.n	<smoltcp::iface::interface::Interface::poll+0xf36>
	e9d1 23fc 	ldrd	r2, r3, [r1, #1008]	@ 0x3f0
	f501 65a8 	add.w	r5, r1, #1344	@ 0x540
	f082 0202 	eor.w	r2, r2, #2
	431a      	orrs	r2, r3
	d1d1      	bne.n	<smoltcp::iface::interface::Interface::poll+0xe64>
	e7c5      	b.n	<smoltcp::iface::interface::Interface::poll+0xe4e>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	7840      	ldrb	r0, [r0, #1]
	2800      	cmp	r0, #0
	f47f ae33 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	9a2a      	ldr	r2, [sp, #168]	@ 0xa8
	2104      	movs	r1, #4
	9157      	str	r1, [sp, #348]	@ 0x15c
	88d0      	ldrh	r0, [r2, #6]
	8891      	ldrh	r1, [r2, #4]
	ba00      	rev	r0, r0
	9a1a      	ldr	r2, [sp, #104]	@ 0x68
	eac2 4220 	pkhtb	r2, r2, r0, asr #16
	ba08      	rev	r0, r1
	921a      	str	r2, [sp, #104]	@ 0x68
	0c00      	lsrs	r0, r0, #16
	9017      	str	r0, [sp, #92]	@ 0x5c
	e626      	b.n	<smoltcp::iface::interface::Interface::poll+0xb36>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	7840      	ldrb	r0, [r0, #1]
	2800      	cmp	r0, #0
	f47f ae20 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	982c      	ldr	r0, [sp, #176]	@ 0xb0
	2101      	movs	r1, #1
	9a2a      	ldr	r2, [sp, #168]	@ 0xa8
	3808      	subs	r0, #8
	90c1      	str	r0, [sp, #772]	@ 0x304
	9829      	ldr	r0, [sp, #164]	@ 0xa4
	9b2b      	ldr	r3, [sp, #172]	@ 0xac
	3008      	adds	r0, #8
	90c0      	str	r0, [sp, #768]	@ 0x300
	88d0      	ldrh	r0, [r2, #6]
	f88d 12f8 	strb.w	r1, [sp, #760]	@ 0x2f8
	ba04      	rev	r4, r0
	8891      	ldrh	r1, [r2, #4]
	9a25      	ldr	r2, [sp, #148]	@ 0x94
	0c20      	lsrs	r0, r4, #16
	f8ad 02fc 	strh.w	r0, [sp, #764]	@ 0x2fc
	ba08      	rev	r0, r1
	994a      	ldr	r1, [sp, #296]	@ 0x128
	0c00      	lsrs	r0, r0, #16
	9017      	str	r0, [sp, #92]	@ 0x5c
	f8ad 02fa 	strh.w	r0, [sp, #762]	@ 0x2fa
	a8be      	add	r0, sp, #760	@ 0x2f8
	9000      	str	r0, [sp, #0]
	a857      	add	r0, sp, #348	@ 0x15c
	f005 fd35 	bl	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>
	981a      	ldr	r0, [sp, #104]	@ 0x68
	eac0 4024 	pkhtb	r0, r0, r4, asr #16
	901a      	str	r0, [sp, #104]	@ 0x68
	e5ff      	b.n	<smoltcp::iface::interface::Interface::poll+0xb36>
	9826      	ldr	r0, [sp, #152]	@ 0x98
	f8ad 0312 	strh.w	r0, [sp, #786]	@ 0x312
	2004      	movs	r0, #4
	f88d 0311 	strb.w	r0, [sp, #785]	@ 0x311
	9821      	ldr	r0, [sp, #132]	@ 0x84
	f88d 0310 	strb.w	r0, [sp, #784]	@ 0x310
	9823      	ldr	r0, [sp, #140]	@ 0x8c
	90c3      	str	r0, [sp, #780]	@ 0x30c
	f240 3002 	movw	r0, #770	@ 0x302
	992c      	ldr	r1, [sp, #176]	@ 0xb0
	9b2b      	ldr	r3, [sp, #172]	@ 0xac
	9a25      	ldr	r2, [sp, #148]	@ 0x94
	f5b1 7f04 	cmp.w	r1, #528	@ 0x210
	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	90bf      	str	r0, [sp, #764]	@ 0x2fc
	f44f 7004 	mov.w	r0, #528	@ 0x210
	93c2      	str	r3, [sp, #776]	@ 0x308
	92c1      	str	r2, [sp, #772]	@ 0x304
	bf28      	it	cs
	4601      	movcs	r1, r0
	91c0      	str	r1, [sp, #768]	@ 0x300
	a8be      	add	r0, sp, #760	@ 0x2f8
	994a      	ldr	r1, [sp, #296]	@ 0x128
	9000      	str	r0, [sp, #0]
	a857      	add	r0, sp, #348	@ 0x15c
	f005 fd0d 	bl	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>
	e5db      	b.n	<smoltcp::iface::interface::Interface::poll+0xb36>
	f106 0020 	add.w	r0, r6, #32
	e004      	b.n	<smoltcp::iface::interface::Interface::poll+0xf8e>
	f106 0038 	add.w	r0, r6, #56	@ 0x38
	e001      	b.n	<smoltcp::iface::interface::Interface::poll+0xf8e>
	f106 0050 	add.w	r0, r6, #80	@ 0x50
	9e27      	ldr	r6, [sp, #156]	@ 0x9c
	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
	8a82      	ldrh	r2, [r0, #20]
	6903      	ldr	r3, [r0, #16]
	b2b6      	uxth	r6, r6
	4072      	eors	r2, r6
	9e28      	ldr	r6, [sp, #160]	@ 0xa0
	f8d7 a008 	ldr.w	sl, [r7, #8]
	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
	4073      	eors	r3, r6
	9d25      	ldr	r5, [sp, #148]	@ 0x94
	431a      	orrs	r2, r3
	f47f acf6 	bne.w	<smoltcp::iface::interface::Interface::poll+0x99c>
	f248 7200 	movw	r2, #34560	@ 0x8700
	f2c0 3293 	movt	r2, #915	@ 0x393
	1889      	adds	r1, r1, r2
	f14c 0200 	adc.w	r2, ip, #0
	e9c0 1202 	strd	r1, r2, [r0, #8]
	e4eb      	b.n	<smoltcp::iface::interface::Interface::poll+0x99c>
	9929      	ldr	r1, [sp, #164]	@ 0xa4
	f1a9 0008 	sub.w	r0, r9, #8
	9a25      	ldr	r2, [sp, #148]	@ 0x94
	3108      	adds	r1, #8
	9b2b      	ldr	r3, [sp, #172]	@ 0xac
	9002      	str	r0, [sp, #8]
	4630      	mov	r0, r6
	e9cd 8100 	strd	r8, r1, [sp]
	992e      	ldr	r1, [sp, #184]	@ 0xb8
	f005 ff71 	bl	<smoltcp::socket::udp::Socket::process>
	e5a8      	b.n	<smoltcp::iface::interface::Interface::poll+0xb32>
	f416 6fa0 	tst.w	r6, #1280	@ 0x500
	f47f ada5 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	2002      	movs	r0, #2
	e003      	b.n	<smoltcp::iface::interface::Interface::poll+0xff4>
	0570      	lsls	r0, r6, #21
	f53f ada0 	bmi.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	2003      	movs	r0, #3
	9016      	str	r0, [sp, #88]	@ 0x58
	04f0      	lsls	r0, r6, #19
	902e      	str	r0, [sp, #184]	@ 0xb8
	d402      	bmi.n	<smoltcp::iface::interface::Interface::poll+0x1002>
	2000      	movs	r0, #0
	9015      	str	r0, [sp, #84]	@ 0x54
	e005      	b.n	<smoltcp::iface::interface::Interface::poll+0x100e>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	6880      	ldr	r0, [r0, #8]
	ba00      	rev	r0, r0
	9006      	str	r0, [sp, #24]
	2001      	movs	r0, #1
	9015      	str	r0, [sp, #84]	@ 0x54
	3d14      	subs	r5, #20
	f000 81a5 	beq.w	<smoltcp::iface::interface::Interface::poll+0x135e>
	9829      	ldr	r0, [sp, #164]	@ 0xa4
	f04f 0900 	mov.w	r9, #0
	f04f 0a00 	mov.w	sl, #0
	f100 0814 	add.w	r8, r0, #20
	2000      	movs	r0, #0
	9019      	str	r0, [sp, #100]	@ 0x64
	2000      	movs	r0, #0
	9022      	str	r0, [sp, #136]	@ 0x88
	2000      	movs	r0, #0
	9024      	str	r0, [sp, #144]	@ 0x90
	9018      	str	r0, [sp, #96]	@ 0x60
	e9cd 0027 	strd	r0, r0, [sp, #156]	@ 0x9c
	e002      	b.n	<smoltcp::iface::interface::Interface::poll+0x103c>
	2d00      	cmp	r5, #0
	f000 81a0 	beq.w	<smoltcp::iface::interface::Interface::poll+0x137c>
	f898 0000 	ldrb.w	r0, [r8]
	b150      	cbz	r0, <smoltcp::iface::interface::Interface::poll+0x1058>
	2801      	cmp	r0, #1
	d134      	bne.n	<smoltcp::iface::interface::Interface::poll+0x10b0>
	f04f 0b01 	mov.w	fp, #1
	2103      	movs	r1, #3
	2000      	movs	r0, #0
	2200      	movs	r2, #0
	f04f 0c00 	mov.w	ip, #0
	2300      	movs	r3, #0
	e005      	b.n	<smoltcp::iface::interface::Interface::poll+0x1064>
	f04f 0b01 	mov.w	fp, #1
	2102      	movs	r1, #2
	4602      	mov	r2, r0
	4684      	mov	ip, r0
	4603      	mov	r3, r0
	455d      	cmp	r5, fp
	f0c1 8178 	bcc.w	<smoltcp::iface::interface::Interface::poll+0x235a>
	290a      	cmp	r1, #10
	f43f ad61 	beq.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	021b      	lsls	r3, r3, #8
	2901      	cmp	r1, #1
	b29b      	uxth	r3, r3
	ea42 0203 	orr.w	r2, r2, r3
	f04f 0305 	mov.w	r3, #5
	44d8      	add	r8, fp
	bf88      	it	hi
	1e8b      	subhi	r3, r1, #2
	eba5 050b 	sub.w	r5, r5, fp
	ea42 020c 	orr.w	r2, r2, ip
	e8df f013 	tbh	[pc, r3, lsl #1]
	000b0176 	.word	0x000b0176
	004b000c 	.word	0x004b000c
	00d90008 	.word	0x00d90008
	000b00eb 	.word	0x000b00eb
	2001      	movs	r0, #1
	9019      	str	r0, [sp, #100]	@ 0x64
	e7c7      	b.n	<smoltcp::iface::interface::Interface::poll+0x1036>
	e7c6      	b.n	<smoltcp::iface::interface::Interface::poll+0x1036>
	2001      	movs	r0, #1
	920c      	str	r2, [sp, #48]	@ 0x30
	9022      	str	r0, [sp, #136]	@ 0x88
	e7c2      	b.n	<smoltcp::iface::interface::Interface::poll+0x1036>
	2d01      	cmp	r5, #1
	f43f ad3e 	beq.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	f898 2001 	ldrb.w	r2, [r8, #1]
	2100      	movs	r1, #0
	2300      	movs	r3, #0
	f108 0602 	add.w	r6, r8, #2
	fa5f fb82 	uxtb.w	fp, r2
	455d      	cmp	r5, fp
	bf38      	it	cc
	2101      	movcc	r1, #1
	f1bb 0f02 	cmp.w	fp, #2
	bf38      	it	cc
	2301      	movcc	r3, #1
	4319      	orrs	r1, r3
	4631      	mov	r1, r6
	bf18      	it	ne
	2100      	movne	r1, #0
	f47f ad29 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	f06f 0301 	mvn.w	r3, #1
	fa53 f482 	uxtab	r4, r3, r2
	1e82      	subs	r2, r0, #2
	2a06      	cmp	r2, #6
	f200 812e 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x134c>
	e8df f012 	tbh	[pc, r2, lsl #1]
	00c20007 	.word	0x00c20007
	00d600ce 	.word	0x00d600ce
	012c012c 	.word	0x012c012c
	0111      	.short	0x0111
	f1bb 0f04 	cmp.w	fp, #4
	f47f ad14 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	2c01      	cmp	r4, #1
	f241 8147 	bls.w	<smoltcp::iface::interface::Interface::poll+0x239e>
	8830      	ldrh	r0, [r6, #0]
	2104      	movs	r1, #4
	2200      	movs	r2, #0
	f04f 0b04 	mov.w	fp, #4
	ba00      	rev	r0, r0
	0e03      	lsrs	r3, r0, #24
	f3c0 4c07 	ubfx	ip, r0, #16, #8
	2000      	movs	r0, #0
	e79e      	b.n	<smoltcp::iface::interface::Interface::poll+0x1064>
	f1bc 0f0e 	cmp.w	ip, #14
	f240 80a1 	bls.w	<smoltcp::iface::interface::Interface::poll+0x1270>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	8841      	ldrh	r1, [r0, #2]
	8800      	ldrh	r0, [r0, #0]
	9004      	str	r0, [sp, #16]
	f240 000a 	movw	r0, #10
	f2c0 0000 	movt	r0, #0
	9105      	str	r1, [sp, #20]
	f002 f8fa 	bl	<defmt::export::acquire_and_header>
	ae8c      	add	r6, sp, #560	@ 0x230
	f240 0907 	movw	r9, #7
	f2c0 0900 	movt	r9, #0
	2102      	movs	r1, #2
	4630      	mov	r0, r6
	f8ad 9230 	strh.w	r9, [sp, #560]	@ 0x230
	f002 faa1 	bl	<_defmt_write>
	acda      	add	r4, sp, #872	@ 0x368
	f240 0a3f 	movw	sl, #63	@ 0x3f
	9825      	ldr	r0, [sp, #148]	@ 0x94
	f2c0 0a00 	movt	sl, #0
	908c      	str	r0, [sp, #560]	@ 0x230
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f8ad a368 	strh.w	sl, [sp, #872]	@ 0x368
	f002 fa94 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f8ad 9368 	strh.w	r9, [sp, #872]	@ 0x368
	f002 fa8e 	bl	<_defmt_write>
	4630      	mov	r0, r6
	f002 f887 	bl	<defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>
	f04f 0b00 	mov.w	fp, #0
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f8ad b368 	strh.w	fp, [sp, #872]	@ 0x368
	f002 fa83 	bl	<_defmt_write>
	4630      	mov	r0, r6
	2102      	movs	r1, #2
	f8ad b230 	strh.w	fp, [sp, #560]	@ 0x230
	f002 fa7d 	bl	<_defmt_write>
	f240 0004 	movw	r0, #4
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 0230 	strh.w	r0, [sp, #560]	@ 0x230
	4630      	mov	r0, r6
	f002 fa73 	bl	<_defmt_write>
	9804      	ldr	r0, [sp, #16]
	2102      	movs	r1, #2
	ba00      	rev	r0, r0
	0c00      	lsrs	r0, r0, #16
	f8ad 0230 	strh.w	r0, [sp, #560]	@ 0x230
	4630      	mov	r0, r6
	f002 fa6a 	bl	<_defmt_write>
	4630      	mov	r0, r6
	2102      	movs	r1, #2
	f8ad 9230 	strh.w	r9, [sp, #560]	@ 0x230
	f002 fa64 	bl	<_defmt_write>
	982b      	ldr	r0, [sp, #172]	@ 0xac
	2102      	movs	r1, #2
	908c      	str	r0, [sp, #560]	@ 0x230
	4620      	mov	r0, r4
	f8ad a368 	strh.w	sl, [sp, #872]	@ 0x368
	f002 fa5c 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f8ad 9368 	strh.w	r9, [sp, #872]	@ 0x368
	f002 fa56 	bl	<_defmt_write>
	4630      	mov	r0, r6
	f002 f84f 	bl	<defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f8ad b368 	strh.w	fp, [sp, #872]	@ 0x368
	f002 fa4d 	bl	<_defmt_write>
	4630      	mov	r0, r6
	2102      	movs	r1, #2
	f8ad b230 	strh.w	fp, [sp, #560]	@ 0x230
	f002 fa47 	bl	<_defmt_write>
	f240 0004 	movw	r0, #4
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 0230 	strh.w	r0, [sp, #560]	@ 0x230
	4630      	mov	r0, r6
	f002 fa3d 	bl	<_defmt_write>
	9805      	ldr	r0, [sp, #20]
	2102      	movs	r1, #2
	ba00      	rev	r0, r0
	0c00      	lsrs	r0, r0, #16
	f8ad 0230 	strh.w	r0, [sp, #560]	@ 0x230
	4630      	mov	r0, r6
	f002 fa34 	bl	<_defmt_write>
	f002 f9ba 	bl	<_defmt_release>
	f04f 0901 	mov.w	r9, #1
	f04f 0a0e 	mov.w	sl, #14
	e6f9      	b.n	<smoltcp::iface::interface::Interface::poll+0x1036>
	9b37      	ldr	r3, [sp, #220]	@ 0xdc
	e9cd 4213 	strd	r4, r2, [sp, #76]	@ 0x4c
	9124      	str	r1, [sp, #144]	@ 0x90
	ea40 2003 	orr.w	r0, r0, r3, lsl #8
	9027      	str	r0, [sp, #156]	@ 0x9c
	9808      	ldr	r0, [sp, #32]
	9012      	str	r0, [sp, #72]	@ 0x48
	9809      	ldr	r0, [sp, #36]	@ 0x24
	9018      	str	r0, [sp, #96]	@ 0x60
	980b      	ldr	r0, [sp, #44]	@ 0x2c
	900d      	str	r0, [sp, #52]	@ 0x34
	9807      	ldr	r0, [sp, #28]
	900f      	str	r0, [sp, #60]	@ 0x3c
	980a      	ldr	r0, [sp, #40]	@ 0x28
	900e      	str	r0, [sp, #56]	@ 0x38
	e6e7      	b.n	<smoltcp::iface::interface::Interface::poll+0x1036>
	2001      	movs	r0, #1
	e9cd 4210 	strd	r4, r2, [sp, #64]	@ 0x40
	9028      	str	r0, [sp, #160]	@ 0xa0
	e6e2      	b.n	<smoltcp::iface::interface::Interface::poll+0x1036>
	f04f 0901 	mov.w	r9, #1
	46e2      	mov	sl, ip
	e6de      	b.n	<smoltcp::iface::interface::Interface::poll+0x1036>
	f1bb 0f03 	cmp.w	fp, #3
	f47f ac59 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	f04f 0b03 	mov.w	fp, #3
	f896 c000 	ldrb.w	ip, [r6]
	2105      	movs	r1, #5
	2000      	movs	r0, #0
	2200      	movs	r2, #0
	e6e1      	b.n	<smoltcp::iface::interface::Interface::poll+0x1054>
	f1bb 0f02 	cmp.w	fp, #2
	f47f ac4d 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	f04f 0b02 	mov.w	fp, #2
	2106      	movs	r1, #6
	e6d5      	b.n	<smoltcp::iface::interface::Interface::poll+0x104c>
	f1bb 0f0a 	cmp.w	fp, #10
	f4ff ac45 	bcc.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	f00b 0007 	and.w	r0, fp, #7
	2802      	cmp	r0, #2
	f47f ac40 	bne.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	f1bb 0f1a 	cmp.w	fp, #26
	d905      	bls.n	<smoltcp::iface::interface::Interface::poll+0x12c4>
	f240 0009 	movw	r0, #9
	f2c0 0000 	movt	r0, #0
	f002 f84b 	bl	<defmt::export::acquire_header_and_release>
	2000      	movs	r0, #0
	9933      	ldr	r1, [sp, #204]	@ 0xcc
	9092      	str	r0, [sp, #584]	@ 0x248
	aada      	add	r2, sp, #872	@ 0x368
	908f      	str	r0, [sp, #572]	@ 0x23c
	908c      	str	r0, [sp, #560]	@ 0x230
	90dc      	str	r0, [sp, #880]	@ 0x370
	a88c      	add	r0, sp, #560	@ 0x230
	e9cd 64da 	strd	r6, r4, [sp, #872]	@ 0x368
	f003 ff24 	bl	<<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold>
	9a34      	ldr	r2, [sp, #208]	@ 0xd0
	e9dd 108c 	ldrd	r1, r0, [sp, #560]	@ 0x230
	fa5f fc80 	uxtb.w	ip, r0
	9c8e      	ldr	r4, [sp, #568]	@ 0x238
	7893      	ldrb	r3, [r2, #2]
	8816      	ldrh	r6, [r2, #0]
	9a94      	ldr	r2, [sp, #592]	@ 0x250
	ea46 4303 	orr.w	r3, r6, r3, lsl #16
	920b      	str	r2, [sp, #44]	@ 0x2c
	f64f 72ff 	movw	r2, #65535	@ 0xffff
	9337      	str	r3, [sp, #220]	@ 0xdc
	ea20 0202 	bic.w	r2, r0, r2
	0a03      	lsrs	r3, r0, #8
	9893      	ldr	r0, [sp, #588]	@ 0x24c
	9008      	str	r0, [sp, #32]
	9892      	ldr	r0, [sp, #584]	@ 0x248
	9009      	str	r0, [sp, #36]	@ 0x24
	9891      	ldr	r0, [sp, #580]	@ 0x244
	9e90      	ldr	r6, [sp, #576]	@ 0x240
	900a      	str	r0, [sp, #40]	@ 0x28
	f89d 023c 	ldrb.w	r0, [sp, #572]	@ 0x23c
	9607      	str	r6, [sp, #28]
	e6a6      	b.n	<smoltcp::iface::interface::Interface::poll+0x1064>
	f1bb 0f0a 	cmp.w	fp, #10
	d117      	bne.n	<smoltcp::iface::interface::Interface::poll+0x134c>
	2c03      	cmp	r4, #3
	f241 8047 	bls.w	<smoltcp::iface::interface::Interface::poll+0x23b0>
	2c07      	cmp	r4, #7
	f241 804d 	bls.w	<smoltcp::iface::interface::Interface::poll+0x23c2>
	f8d8 0002 	ldr.w	r0, [r8, #2]
	f64f 72ff 	movw	r2, #65535	@ 0xffff
	f8d8 1006 	ldr.w	r1, [r8, #6]
	f04f 0b0a 	mov.w	fp, #10
	ba00      	rev	r0, r0
	ba0c      	rev	r4, r1
	ea20 0202 	bic.w	r2, r0, r2
	0a03      	lsrs	r3, r0, #8
	fa5f fc80 	uxtb.w	ip, r0
	2108      	movs	r1, #8
	2000      	movs	r0, #0
	e68b      	b.n	<smoltcp::iface::interface::Interface::poll+0x1064>
	f64f 72ff 	movw	r2, #65535	@ 0xffff
	0a0b      	lsrs	r3, r1, #8
	ea21 0202 	bic.w	r2, r1, r2
	fa5f fc81 	uxtb.w	ip, r1
	2109      	movs	r1, #9
	e682      	b.n	<smoltcp::iface::interface::Interface::poll+0x1064>
	2000      	movs	r0, #0
	f04f 0a00 	mov.w	sl, #0
	9028      	str	r0, [sp, #160]	@ 0xa0
	f04f 0900 	mov.w	r9, #0
	2000      	movs	r0, #0
	9018      	str	r0, [sp, #96]	@ 0x60
	9027      	str	r0, [sp, #156]	@ 0x9c
	9024      	str	r0, [sp, #144]	@ 0x90
	2000      	movs	r0, #0
	9022      	str	r0, [sp, #136]	@ 0x88
	2000      	movs	r0, #0
	9019      	str	r0, [sp, #100]	@ 0x64
	e001      	b.n	<smoltcp::iface::interface::Interface::poll+0x1380>
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	8986      	ldrh	r6, [r0, #12]
	203c      	movs	r0, #60	@ 0x3c
	992c      	ldr	r1, [sp, #176]	@ 0xb0
	ea00 0096 	and.w	r0, r0, r6, lsr #2
	4281      	cmp	r1, r0
	f0c1 8023 	bcc.w	<smoltcp::iface::interface::Interface::poll+0x23d4>
	e9dd 322f 	ldrd	r3, r2, [sp, #188]	@ 0xbc
	f44f 71a8 	mov.w	r1, #336	@ 0x150
	f88d a225 	strb.w	sl, [sp, #549]	@ 0x225
	fb03 2201 	mla	r2, r3, r1, r2
	9916      	ldr	r1, [sp, #88]	@ 0x58
	f88d 122d 	strb.w	r1, [sp, #557]	@ 0x22d
	9919      	ldr	r1, [sp, #100]	@ 0x64
	9b06      	ldr	r3, [sp, #24]
	f001 0101 	and.w	r1, r1, #1
	f88d 122c 	strb.w	r1, [sp, #556]	@ 0x22c
	f009 0101 	and.w	r1, r9, #1
	f88d 1224 	strb.w	r1, [sp, #548]	@ 0x224
	992c      	ldr	r1, [sp, #176]	@ 0xb0
	9378      	str	r3, [sp, #480]	@ 0x1e0
	1a09      	subs	r1, r1, r0
	9187      	str	r1, [sp, #540]	@ 0x21c
	9929      	ldr	r1, [sp, #164]	@ 0xa4
	9b15      	ldr	r3, [sp, #84]	@ 0x54
	4408      	add	r0, r1
	9086      	str	r0, [sp, #536]	@ 0x218
	980c      	ldr	r0, [sp, #48]	@ 0x30
	f8ad 0216 	strh.w	r0, [sp, #534]	@ 0x216
	9822      	ldr	r0, [sp, #136]	@ 0x88
	f8ad 0214 	strh.w	r0, [sp, #532]	@ 0x214
	9810      	ldr	r0, [sp, #64]	@ 0x40
	9084      	str	r0, [sp, #528]	@ 0x210
	9811      	ldr	r0, [sp, #68]	@ 0x44
	9083      	str	r0, [sp, #524]	@ 0x20c
	9828      	ldr	r0, [sp, #160]	@ 0xa0
	9082      	str	r0, [sp, #520]	@ 0x208
	980d      	ldr	r0, [sp, #52]	@ 0x34
	9081      	str	r0, [sp, #516]	@ 0x204
	9812      	ldr	r0, [sp, #72]	@ 0x48
	9080      	str	r0, [sp, #512]	@ 0x200
	9818      	ldr	r0, [sp, #96]	@ 0x60
	907f      	str	r0, [sp, #508]	@ 0x1fc
	980e      	ldr	r0, [sp, #56]	@ 0x38
	907e      	str	r0, [sp, #504]	@ 0x1f8
	980f      	ldr	r0, [sp, #60]	@ 0x3c
	907d      	str	r0, [sp, #500]	@ 0x1f4
	9827      	ldr	r0, [sp, #156]	@ 0x9c
	907c      	str	r0, [sp, #496]	@ 0x1f0
	9813      	ldr	r0, [sp, #76]	@ 0x4c
	907b      	str	r0, [sp, #492]	@ 0x1ec
	9814      	ldr	r0, [sp, #80]	@ 0x50
	907a      	str	r0, [sp, #488]	@ 0x1e8
	9824      	ldr	r0, [sp, #144]	@ 0x90
	9079      	str	r0, [sp, #484]	@ 0x1e4
	982a      	ldr	r0, [sp, #168]	@ 0xa8
	9377      	str	r3, [sp, #476]	@ 0x1dc
	6846      	ldr	r6, [r0, #4]
	8801      	ldrh	r1, [r0, #0]
	89c4      	ldrh	r4, [r0, #14]
	ba36      	rev	r6, r6
	f8b0 e002 	ldrh.w	lr, [r0, #2]
	ba09      	rev	r1, r1
	9688      	str	r6, [sp, #544]	@ 0x220
	ba26      	rev	r6, r4
	fa9e f39e 	rev16.w	r3, lr
	ea4f 4c11 	mov.w	ip, r1, lsr #16
	0c36      	lsrs	r6, r6, #16
	f8ad 3228 	strh.w	r3, [sp, #552]	@ 0x228
	f8ad 622a 	strh.w	r6, [sp, #554]	@ 0x22a
	f8ad c226 	strh.w	ip, [sp, #550]	@ 0x226
	9830      	ldr	r0, [sp, #192]	@ 0xc0
	4290      	cmp	r0, r2
	f000 8080 	beq.w	<smoltcp::iface::interface::Interface::poll+0x1538>
	9830      	ldr	r0, [sp, #192]	@ 0xc0
	e9d0 6400 	ldrd	r6, r4, [r0]
	f500 71a8 	add.w	r1, r0, #336	@ 0x150
	3e02      	subs	r6, #2
	f174 0600 	sbcs.w	r6, r4, #0
	d202      	bcs.n	<smoltcp::iface::interface::Interface::poll+0x1450>
	460c      	mov	r4, r1
	9930      	ldr	r1, [sp, #192]	@ 0xc0
	e029      	b.n	<smoltcp::iface::interface::Interface::poll+0x14a4>
	4291      	cmp	r1, r2
	d071      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1538>
	9d30      	ldr	r5, [sp, #192]	@ 0xc0
	e9d5 4054 	ldrd	r4, r0, [r5, #336]	@ 0x150
	f505 7628 	add.w	r6, r5, #672	@ 0x2a0
	3c02      	subs	r4, #2
	f170 0000 	sbcs.w	r0, r0, #0
	d201      	bcs.n	<smoltcp::iface::interface::Interface::poll+0x146a>
	4634      	mov	r4, r6
	e01c      	b.n	<smoltcp::iface::interface::Interface::poll+0x14a4>
	4296      	cmp	r6, r2
	d064      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1538>
	9930      	ldr	r1, [sp, #192]	@ 0xc0
	e9d1 04a8 	ldrd	r0, r4, [r1, #672]	@ 0x2a0
	f501 717c 	add.w	r1, r1, #1008	@ 0x3f0
	3802      	subs	r0, #2
	f174 0000 	sbcs.w	r0, r4, #0
	d202      	bcs.n	<smoltcp::iface::interface::Interface::poll+0x1486>
	460c      	mov	r4, r1
	4631      	mov	r1, r6
	e00e      	b.n	<smoltcp::iface::interface::Interface::poll+0x14a4>
	4291      	cmp	r1, r2
	d056      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1538>
	9d30      	ldr	r5, [sp, #192]	@ 0xc0
	e9d5 04fc 	ldrd	r0, r4, [r5, #1008]	@ 0x3f0
	f505 65a8 	add.w	r5, r5, #1344	@ 0x540
	9530      	str	r5, [sp, #192]	@ 0xc0
	f1d0 0001 	rsbs	r0, r0, #1
	f04f 0000 	mov.w	r0, #0
	41a0      	sbcs	r0, r4
	462c      	mov	r4, r5
	d3c5      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x1430>
	f891 6133 	ldrb.w	r6, [r1, #307]	@ 0x133
	9430      	str	r4, [sp, #192]	@ 0xc0
	2e00      	cmp	r6, #0
	d0c0      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1430>
	2e01      	cmp	r6, #1
	d103      	bne.n	<smoltcp::iface::interface::Interface::poll+0x14ba>
	982e      	ldr	r0, [sp, #184]	@ 0xb8
	9430      	str	r4, [sp, #192]	@ 0xc0
	2800      	cmp	r0, #0
	d4ba      	bmi.n	<smoltcp::iface::interface::Interface::poll+0x1430>
	f8b1 011c 	ldrh.w	r0, [r1, #284]	@ 0x11c
	b1a0      	cbz	r0, <smoltcp::iface::interface::Interface::poll+0x14ea>
	f8d1 011e 	ldr.w	r0, [r1, #286]	@ 0x11e
	9d2b      	ldr	r5, [sp, #172]	@ 0xac
	9430      	str	r4, [sp, #192]	@ 0xc0
	4285      	cmp	r5, r0
	d1b1      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1430>
	f8b1 0122 	ldrh.w	r0, [r1, #290]	@ 0x122
	9430      	str	r4, [sp, #192]	@ 0xc0
	4283      	cmp	r3, r0
	d1ac      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1430>
	f8d1 0124 	ldr.w	r0, [r1, #292]	@ 0x124
	9d25      	ldr	r5, [sp, #148]	@ 0x94
	9430      	str	r4, [sp, #192]	@ 0xc0
	4285      	cmp	r5, r0
	d1a6      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1430>
	f8b1 0128 	ldrh.w	r0, [r1, #296]	@ 0x128
	4584      	cmp	ip, r0
	e012      	b.n	<smoltcp::iface::interface::Interface::poll+0x1510>
	f8d1 00b3 	ldr.w	r0, [r1, #179]	@ 0xb3
	9d2b      	ldr	r5, [sp, #172]	@ 0xac
	f891 60b2 	ldrb.w	r6, [r1, #178]	@ 0xb2
	1a28      	subs	r0, r5, r0
	bf18      	it	ne
	2001      	movne	r0, #1
	4206      	tst	r6, r0
	9430      	str	r4, [sp, #192]	@ 0xc0
	d197      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1430>
	f1be 0f00 	cmp.w	lr, #0
	9430      	str	r4, [sp, #192]	@ 0xc0
	f43f af93 	beq.w	<smoltcp::iface::interface::Interface::poll+0x1430>
	f8b1 00b0 	ldrh.w	r0, [r1, #176]	@ 0xb0
	4283      	cmp	r3, r0
	9430      	str	r4, [sp, #192]	@ 0xc0
	f47f af8d 	bne.w	<smoltcp::iface::interface::Interface::poll+0x1430>
	a877      	add	r0, sp, #476	@ 0x1dc
	9a2b      	ldr	r2, [sp, #172]	@ 0xac
	9b25      	ldr	r3, [sp, #148]	@ 0x94
	e9cd 2000 	strd	r2, r0, [sp]
	a88c      	add	r0, sp, #560	@ 0x230
	9a4a      	ldr	r2, [sp, #296]	@ 0x128
	f003 ffb2 	bl	<smoltcp::socket::tcp::Socket::process>
	9890      	ldr	r0, [sp, #576]	@ 0x240
	2802      	cmp	r0, #2
	f43f ab01 	beq.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	ab8c      	add	r3, sp, #560	@ 0x230
	9e31      	ldr	r6, [sp, #196]	@ 0xc4
	aca5      	add	r4, sp, #660	@ 0x294
	e017      	b.n	<smoltcp::iface::interface::Interface::poll+0x1568>
	9816      	ldr	r0, [sp, #88]	@ 0x58
	992b      	ldr	r1, [sp, #172]	@ 0xac
	3804      	subs	r0, #4
	fab0 f080 	clz	r0, r0
	fab1 f181 	clz	r1, r1
	0949      	lsrs	r1, r1, #5
	0940      	lsrs	r0, r0, #5
	4308      	orrs	r0, r1
	9920      	ldr	r1, [sp, #128]	@ 0x80
	4308      	orrs	r0, r1
	2801      	cmp	r0, #1
	f43f aaee 	beq.w	<smoltcp::iface::interface::Interface::poll+0xb32>
	9925      	ldr	r1, [sp, #148]	@ 0x94
	a88c      	add	r0, sp, #560	@ 0x230
	9a2b      	ldr	r2, [sp, #172]	@ 0xac
	ab77      	add	r3, sp, #476	@ 0x1dc
	f005 f8a4 	bl	<smoltcp::socket::tcp::Socket::rst_reply>
	ab8c      	add	r3, sp, #560	@ 0x230
	acbe      	add	r4, sp, #760	@ 0x2f8
	9e32      	ldr	r6, [sp, #200]	@ 0xc8
	cb0f      	ldmia	r3, {r0, r1, r2, r3}
	c60f      	stmia	r6!, {r0, r1, r2, r3}
	9936      	ldr	r1, [sp, #216]	@ 0xd8
	4620      	mov	r0, r4
	2254      	movs	r2, #84	@ 0x54
	f006 f98b 	bl	<__aeabi_memcpy4>
	a857      	add	r0, sp, #348	@ 0x15c
	4621      	mov	r1, r4
	2264      	movs	r2, #100	@ 0x64
	f006 f986 	bl	<__aeabi_memcpy4>
	f7ff bad9 	b.w	<smoltcp::iface::interface::Interface::poll+0xb36>
	68f8      	ldr	r0, [r7, #12]
	f44f 71a8 	mov.w	r1, #336	@ 0x150
	e9d0 5000 	ldrd	r5, r0, [r0]
	fb00 5001 	mla	r0, r0, r1, r5
	e9dd 2147 	ldrd	r2, r1, [sp, #284]	@ 0x11c
	9049      	str	r0, [sp, #292]	@ 0x124
	984a      	ldr	r0, [sp, #296]	@ 0x128
	e9c0 2104 	strd	r2, r1, [r0, #16]
	a8be      	add	r0, sp, #760	@ 0x2f8
	3008      	adds	r0, #8
	9038      	str	r0, [sp, #224]	@ 0xe0
	9849      	ldr	r0, [sp, #292]	@ 0x124
	4285      	cmp	r5, r0
	f000 861a 	beq.w	<smoltcp::iface::interface::Interface::poll+0x21e0>
	46a9      	mov	r9, r5
	e8f5 0154 	ldrd	r0, r1, [r5], #336	@ 0x150
	f080 0003 	eor.w	r0, r0, #3
	4308      	orrs	r0, r1
	d0f4      	beq.n	<smoltcp::iface::interface::Interface::poll+0x15a4>
	f899 0138 	ldrb.w	r0, [r9, #312]	@ 0x138
	2801      	cmp	r0, #1
	f040 8089 	bne.w	<smoltcp::iface::interface::Interface::poll+0x16d6>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	46a8      	mov	r8, r5
	f8d9 2139 	ldr.w	r2, [r9, #313]	@ 0x139
	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
	e9d0 ab04 	ldrd	sl, fp, [r0, #16]
	f8d0 00f0 	ldr.w	r0, [r0, #240]	@ 0xf0
	e9d9 c550 	ldrd	ip, r5, [r9, #320]	@ 0x140
	9944      	ldr	r1, [sp, #272]	@ 0x110
	eb00 0080 	add.w	r0, r0, r0, lsl #2
	e00b      	b.n	<smoltcp::iface::interface::Interface::poll+0x15fc>
	425b      	negs	r3, r3
	f003 031f 	and.w	r3, r3, #31
	fa04 f303 	lsl.w	r3, r4, r3
	ba1b      	rev	r3, r3
	680e      	ldr	r6, [r1, #0]
	3105      	adds	r1, #5
	3805      	subs	r0, #5
	4056      	eors	r6, r2
	4233      	tst	r3, r6
	d016      	beq.n	<smoltcp::iface::interface::Interface::poll+0x162a>
	b120      	cbz	r0, <smoltcp::iface::interface::Interface::poll+0x1608>
	790b      	ldrb	r3, [r1, #4]
	2b00      	cmp	r3, #0
	d1ef      	bne.n	<smoltcp::iface::interface::Interface::poll+0x15e4>
	2300      	movs	r3, #0
	e7f3      	b.n	<smoltcp::iface::interface::Interface::poll+0x15f0>
	1c50      	adds	r0, r2, #1
	f000 85cd 	beq.w	<smoltcp::iface::interface::Interface::poll+0x21a8>
	9935      	ldr	r1, [sp, #212]	@ 0xd4
	a8be      	add	r0, sp, #760	@ 0x2f8
	e9cd ab00 	strd	sl, fp, [sp]
	4664      	mov	r4, ip
	f005 fe77 	bl	<smoltcp::iface::route::Routes::lookup>
	f89d 02f8 	ldrb.w	r0, [sp, #760]	@ 0x2f8
	46a4      	mov	ip, r4
	2801      	cmp	r0, #1
	d150      	bne.n	<smoltcp::iface::interface::Interface::poll+0x16c8>
	f8dd 22f9 	ldr.w	r2, [sp, #761]	@ 0x2f9
	1c50      	adds	r0, r2, #1
	f000 85bc 	beq.w	<smoltcp::iface::interface::Interface::poll+0x21a8>
	f002 00f0 	and.w	r0, r2, #240	@ 0xf0
	28e0      	cmp	r0, #224	@ 0xe0
	bf1c      	itt	ne
	b2d0      	uxtbne	r0, r2
	ea50 2012 	orrsne.w	r0, r0, r2, lsr #8
	f000 85b3 	beq.w	<smoltcp::iface::interface::Interface::poll+0x21a8>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	f8d0 00e0 	ldr.w	r0, [r0, #224]	@ 0xe0
	eba0 0080 	sub.w	r0, r0, r0, lsl #2
	00c1      	lsls	r1, r0, #3
	2000      	movs	r0, #0
	180e      	adds	r6, r1, r0
	d039      	beq.n	<smoltcp::iface::interface::Interface::poll+0x16c8>
	9b4a      	ldr	r3, [sp, #296]	@ 0x128
	4403      	add	r3, r0
	6a1c      	ldr	r4, [r3, #32]
	4294      	cmp	r4, r2
	d01d      	beq.n	<smoltcp::iface::interface::Interface::poll+0x169a>
	f116 0418 	adds.w	r4, r6, #24
	d031      	beq.n	<smoltcp::iface::interface::Interface::poll+0x16c8>
	6b9c      	ldr	r4, [r3, #56]	@ 0x38
	4294      	cmp	r4, r2
	d025      	beq.n	<smoltcp::iface::interface::Interface::poll+0x16b6>
	f116 0430 	adds.w	r4, r6, #48	@ 0x30
	d02b      	beq.n	<smoltcp::iface::interface::Interface::poll+0x16c8>
	6d1c      	ldr	r4, [r3, #80]	@ 0x50
	4294      	cmp	r4, r2
	f000 8189 	beq.w	<smoltcp::iface::interface::Interface::poll+0x198a>
	3648      	adds	r6, #72	@ 0x48
	d025      	beq.n	<smoltcp::iface::interface::Interface::poll+0x16c8>
	6e9b      	ldr	r3, [r3, #104]	@ 0x68
	3060      	adds	r0, #96	@ 0x60
	4293      	cmp	r3, r2
	d1e5      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1650>
	994a      	ldr	r1, [sp, #296]	@ 0x128
	4408      	add	r0, r1
	3008      	adds	r0, #8
	e9d0 0102 	ldrd	r0, r1, [r0, #8]
	ebba 0000 	subs.w	r0, sl, r0
	eb7b 0001 	sbcs.w	r0, fp, r1
	db09      	blt.n	<smoltcp::iface::interface::Interface::poll+0x16ac>
	e016      	b.n	<smoltcp::iface::interface::Interface::poll+0x16c8>
	f103 0020 	add.w	r0, r3, #32
	e9d0 0102 	ldrd	r0, r1, [r0, #8]
	ebba 0000 	subs.w	r0, sl, r0
	eb7b 0001 	sbcs.w	r0, fp, r1
	da0d      	bge.n	<smoltcp::iface::interface::Interface::poll+0x16c8>
	2000      	movs	r0, #0
	4645      	mov	r5, r8
	f889 0138 	strb.w	r0, [r9, #312]	@ 0x138
	e00f      	b.n	<smoltcp::iface::interface::Interface::poll+0x16d6>
	f103 0038 	add.w	r0, r3, #56	@ 0x38
	e9d0 0102 	ldrd	r0, r1, [r0, #8]
	ebba 0000 	subs.w	r0, sl, r0
	eb7b 0001 	sbcs.w	r0, fp, r1
	dbf1      	blt.n	<smoltcp::iface::interface::Interface::poll+0x16ac>
	ebba 000c 	subs.w	r0, sl, ip
	eb7b 0005 	sbcs.w	r0, fp, r5
	4645      	mov	r5, r8
	f6ff af67 	blt.w	<smoltcp::iface::interface::Interface::poll+0x15a4>
	e9d9 0100 	ldrd	r0, r1, [r9]
	f080 0002 	eor.w	r0, r0, #2
	4308      	orrs	r0, r1
	d140      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1764>
	f899 0050 	ldrb.w	r0, [r9, #80]	@ 0x50
	f8d9 b03c 	ldr.w	fp, [r9, #60]	@ 0x3c
	f899 e051 	ldrb.w	lr, [r9, #81]	@ 0x51
	2800      	cmp	r0, #0
	bf08      	it	eq
	f04f 0e40 	moveq.w	lr, #64	@ 0x40
	f1bb 0f00 	cmp.w	fp, #0
	f000 80d7 	beq.w	<smoltcp::iface::interface::Interface::poll+0x18ac>
	f8d9 1034 	ldr.w	r1, [r9, #52]	@ 0x34
	2900      	cmp	r1, #0
	f000 85e9 	beq.w	<smoltcp::iface::interface::Interface::poll+0x22da>
	f8d9 0038 	ldr.w	r0, [r9, #56]	@ 0x38
	4288      	cmp	r0, r1
	f080 85f1 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x22f4>
	f8d9 a030 	ldr.w	sl, [r9, #48]	@ 0x30
	eb00 0480 	add.w	r4, r0, r0, lsl #2
	e9d9 2302 	ldrd	r2, r3, [r9, #8]
	9246      	str	r2, [sp, #280]	@ 0x118
	eb0a 0284 	add.w	r2, sl, r4, lsl #2
	933f      	str	r3, [sp, #252]	@ 0xfc
	7b92      	ldrb	r2, [r2, #14]
	2a02      	cmp	r2, #2
	d12b      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1784>
	f109 0844 	add.w	r8, r9, #68	@ 0x44
	46ac      	mov	ip, r5
	e898 0124 	ldmia.w	r8, {r2, r5, r8}
	1b53      	subs	r3, r2, r5
	4543      	cmp	r3, r8
	bf28      	it	cs
	4643      	movcs	r3, r8
	195e      	adds	r6, r3, r5
	f080 858d 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x225e>
	4296      	cmp	r6, r2
	f200 858a 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x225e>
	f85a 6024 	ldr.w	r6, [sl, r4, lsl #2]
	42b3      	cmp	r3, r6
	bf38      	it	cc
	461e      	movcc	r6, r3
	2a00      	cmp	r2, #0
	d061      	beq.n	<smoltcp::iface::interface::Interface::poll+0x181c>
	1973      	adds	r3, r6, r5
	fbb3 f4f2 	udiv	r4, r3, r2
	fb04 3212 	mls	r2, r4, r2, r3
	e05c      	b.n	<smoltcp::iface::interface::Interface::poll+0x181e>
	f8b9 011c 	ldrh.w	r0, [r9, #284]	@ 0x11c
	2800      	cmp	r0, #0
	f43f af1b 	beq.w	<smoltcp::iface::interface::Interface::poll+0x15a4>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	e9d9 2314 	ldrd	r2, r3, [r9, #80]	@ 0x50
	e9d0 1004 	ldrd	r1, r0, [r0, #16]
	431a      	orrs	r2, r3
	9548      	str	r5, [sp, #288]	@ 0x120
	d005      	beq.n	<smoltcp::iface::interface::Interface::poll+0x178a>
	e9d9 2316 	ldrd	r2, r3, [r9, #88]	@ 0x58
	e00c      	b.n	<smoltcp::iface::interface::Interface::poll+0x179e>
	f8dd a118 	ldr.w	sl, [sp, #280]	@ 0x118
	e05a      	b.n	<smoltcp::iface::interface::Interface::poll+0x1840>
	2201      	movs	r2, #1
	f8c9 005c 	str.w	r0, [r9, #92]	@ 0x5c
	f8c9 2050 	str.w	r2, [r9, #80]	@ 0x50
	2200      	movs	r2, #0
	e9c9 2115 	strd	r2, r1, [r9, #84]	@ 0x54
	460a      	mov	r2, r1
	4603      	mov	r3, r0
	e9d9 540e 	ldrd	r5, r4, [r9, #56]	@ 0x38
	f8d9 6030 	ldr.w	r6, [r9, #48]	@ 0x30
	1952      	adds	r2, r2, r5
	4163      	adcs	r3, r4
	07f6      	lsls	r6, r6, #31
	d00c      	beq.n	<smoltcp::iface::interface::Interface::poll+0x17c8>
	1a89      	subs	r1, r1, r2
	4198      	sbcs	r0, r3
	db09      	blt.n	<smoltcp::iface::interface::Interface::poll+0x17c8>
	f240 0019 	movw	r0, #25
	f2c0 0000 	movt	r0, #0
	f001 fdcd 	bl	<defmt::export::acquire_header_and_release>
	2000      	movs	r0, #0
	f889 0133 	strb.w	r0, [r9, #307]	@ 0x133
	e171      	b.n	<smoltcp::iface::interface::Interface::poll+0x1aac>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	68c1      	ldr	r1, [r0, #12]
	4648      	mov	r0, r9
	f003 fdfd 	bl	<smoltcp::socket::tcp::Socket::seq_to_transmit>
	2800      	cmp	r0, #0
	f040 816a 	bne.w	<smoltcp::iface::interface::Interface::poll+0x1aac>
	f8d9 0080 	ldr.w	r0, [r9, #128]	@ 0x80
	2802      	cmp	r0, #2
	f000 80df 	beq.w	<smoltcp::iface::interface::Interface::poll+0x19a0>
	2801      	cmp	r0, #1
	f040 8162 	bne.w	<smoltcp::iface::interface::Interface::poll+0x1aac>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	e9d9 3222 	ldrd	r3, r2, [r9, #136]	@ 0x88
	e9d0 1004 	ldrd	r1, r0, [r0, #16]
	1ace      	subs	r6, r1, r3
	eb70 0602 	sbcs.w	r6, r0, r2
	f2c0 8158 	blt.w	<smoltcp::iface::interface::Interface::poll+0x1aac>
	1ac9      	subs	r1, r1, r3
	e9d9 6424 	ldrd	r6, r4, [r9, #144]	@ 0x90
	4190      	sbcs	r0, r2
	ea81 71e0 	eor.w	r1, r1, r0, asr #31
	ea80 72e0 	eor.w	r2, r0, r0, asr #31
	ebb1 71e0 	subs.w	r1, r1, r0, asr #31
	eb62 70e0 	sbc.w	r0, r2, r0, asr #31
	1875      	adds	r5, r6, r1
	eb44 0600 	adc.w	r6, r4, r0
	e0c3      	b.n	<smoltcp::iface::interface::Interface::poll+0x19a4>
	2200      	movs	r2, #0
	3001      	adds	r0, #1
	f8dd a118 	ldr.w	sl, [sp, #280]	@ 0x118
	fbb0 f3f1 	udiv	r3, r0, r1
	f1bb 0b01 	subs.w	fp, fp, #1
	fb03 0011 	mls	r0, r3, r1, r0
	eba8 0106 	sub.w	r1, r8, r6
	4665      	mov	r5, ip
	e9c9 2112 	strd	r2, r1, [r9, #72]	@ 0x48
	e9c9 0b0e 	strd	r0, fp, [r9, #56]	@ 0x38
	d035      	beq.n	<smoltcp::iface::interface::Interface::poll+0x18ac>
	f8d9 3034 	ldr.w	r3, [r9, #52]	@ 0x34
	2b00      	cmp	r3, #0
	f000 8548 	beq.w	<smoltcp::iface::interface::Interface::poll+0x22da>
	f8d9 0038 	ldr.w	r0, [r9, #56]	@ 0x38
	9548      	str	r5, [sp, #288]	@ 0x120
	4298      	cmp	r0, r3
	f080 8574 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x233e>
	e9d9 5811 	ldrd	r5, r8, [r9, #68]	@ 0x44
	f8d9 404c 	ldr.w	r4, [r9, #76]	@ 0x4c
	eba5 0608 	sub.w	r6, r5, r8
	42a6      	cmp	r6, r4
	bf28      	it	cs
	4626      	movcs	r6, r4
	eb16 0108 	adds.w	r1, r6, r8
	f080 84dc 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x2228>
	42a9      	cmp	r1, r5
	f200 84d9 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x2228>
	f8d9 2030 	ldr.w	r2, [r9, #48]	@ 0x30
	eb00 0180 	add.w	r1, r0, r0, lsl #2
	f8cd b114 	str.w	fp, [sp, #276]	@ 0x114
	eb02 0b81 	add.w	fp, r2, r1, lsl #2
	f89b 200e 	ldrb.w	r2, [fp, #14]
	2a02      	cmp	r2, #2
	f000 8500 	beq.w	<smoltcp::iface::interface::Interface::poll+0x2290>
	f8db 1000 	ldr.w	r1, [fp]
	42b1      	cmp	r1, r6
	f200 84cf 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x2238>
	f8d9 c040 	ldr.w	ip, [r9, #64]	@ 0x40
	07d2      	lsls	r2, r2, #31
	aa40      	add	r2, sp, #256	@ 0x100
	c219      	stmia	r2!, {r0, r3, r4}
	d006      	beq.n	<smoltcp::iface::interface::Interface::poll+0x18b4>
	f8db 300f 	ldr.w	r3, [fp, #15]
	e014      	b.n	<smoltcp::iface::interface::Interface::poll+0x18d6>
	2203      	movs	r2, #3
	f04f 0c00 	mov.w	ip, #0
	e2eb      	b.n	<smoltcp::iface::interface::Interface::poll+0x1e8c>
	ea5f 32ca 	movs.w	r2, sl, lsl #15
	d408      	bmi.n	<smoltcp::iface::interface::Interface::poll+0x18cc>
	9a4a      	ldr	r2, [sp, #296]	@ 0x128
	f8d2 20f0 	ldr.w	r2, [r2, #240]	@ 0xf0
	2a00      	cmp	r2, #0
	f000 82c7 	beq.w	<smoltcp::iface::interface::Interface::poll+0x1e54>
	9a44      	ldr	r2, [sp, #272]	@ 0x110
	6813      	ldr	r3, [r2, #0]
	e004      	b.n	<smoltcp::iface::interface::Interface::poll+0x18d6>
	983f      	ldr	r0, [sp, #252]	@ 0xfc
	ea4f 621a 	mov.w	r2, sl, lsr #24
	ea42 2300 	orr.w	r3, r2, r0, lsl #8
	eb0c 0008 	add.w	r0, ip, r8
	e9db 2a01 	ldrd	r2, sl, [fp, #4]
	f8bb c00c 	ldrh.w	ip, [fp, #12]
	2404      	movs	r4, #4
	e9cd 01c1 	strd	r0, r1, [sp, #772]	@ 0x304
	2003      	movs	r0, #3
	e9cd 20bf 	strd	r2, r0, [sp, #764]	@ 0x2fc
	984a      	ldr	r0, [sp, #296]	@ 0x128
	90be      	str	r0, [sp, #760]	@ 0x2f8
	68b8      	ldr	r0, [r7, #8]
	f88d 4361 	strb.w	r4, [sp, #865]	@ 0x361
	f101 0408 	add.w	r4, r1, #8
	e9d0 1004 	ldrd	r1, r0, [r0, #16]
	e9cd 3ad5 	strd	r3, sl, [sp, #852]	@ 0x354
	9b46      	ldr	r3, [sp, #280]	@ 0x118
	4288      	cmp	r0, r1
	f88d e360 	strb.w	lr, [sp, #864]	@ 0x360
	94d7      	str	r4, [sp, #860]	@ 0x35c
	f8cd a11c 	str.w	sl, [sp, #284]	@ 0x11c
	f8ad c30e 	strh.w	ip, [sp, #782]	@ 0x30e
	f8ad 330c 	strh.w	r3, [sp, #780]	@ 0x30c
	f4be abd0 	bcs.w	<smoltcp::iface::interface::Interface::poll+0xbe>
	f44f 61c6 	mov.w	r1, #1584	@ 0x630
	9b43      	ldr	r3, [sp, #268]	@ 0x10c
	4348      	muls	r0, r1
	6819      	ldr	r1, [r3, #0]
	5808      	ldr	r0, [r1, r0]
	2800      	cmp	r0, #0
	d415      	bmi.n	<smoltcp::iface::interface::Interface::poll+0x195a>
	2000      	movs	r0, #0
	93a7      	str	r3, [sp, #668]	@ 0x29c
	90a5      	str	r0, [sp, #660]	@ 0x294
	a9a5      	add	r1, sp, #660	@ 0x294
	984a      	ldr	r0, [sp, #296]	@ 0x128
	9b38      	ldr	r3, [sp, #224]	@ 0xe0
	f7fd fc81 	bl	<smoltcp::iface::interface::InterfaceInner::dispatch_ip>
	b2c0      	uxtb	r0, r0
	2802      	cmp	r0, #2
	d112      	bne.n	<smoltcp::iface::interface::Interface::poll+0x196a>
	f8db 1000 	ldr.w	r1, [fp]
	f04f 0c01 	mov.w	ip, #1
	9c42      	ldr	r4, [sp, #264]	@ 0x108
	9845      	ldr	r0, [sp, #276]	@ 0x114
	42b1      	cmp	r1, r6
	f240 8285 	bls.w	<smoltcp::iface::interface::Interface::poll+0x1e60>
	f000 bc8a 	b.w	<smoltcp::iface::interface::Interface::poll+0x226e>
	f240 0025 	movw	r0, #37	@ 0x25
	f2c0 0000 	movt	r0, #0
	f001 fcfa 	bl	<defmt::export::acquire_header_and_release>
	2200      	movs	r2, #0
	e000      	b.n	<smoltcp::iface::interface::Interface::poll+0x196c>
	2201      	movs	r2, #1
	9c42      	ldr	r4, [sp, #264]	@ 0x108
	2100      	movs	r1, #0
	9845      	ldr	r0, [sp, #276]	@ 0x114
	f04f 0c01 	mov.w	ip, #1
	2d00      	cmp	r5, #0
	f000 8276 	beq.w	<smoltcp::iface::interface::Interface::poll+0x1e68>
	eb01 0308 	add.w	r3, r1, r8
	fbb3 f6f5 	udiv	r6, r3, r5
	fb06 3315 	mls	r3, r6, r5, r3
	e26f      	b.n	<smoltcp::iface::interface::Interface::poll+0x1e6a>
	f103 0050 	add.w	r0, r3, #80	@ 0x50
	e9d0 0102 	ldrd	r0, r1, [r0, #8]
	ebba 0000 	subs.w	r0, sl, r0
	eb7b 0001 	sbcs.w	r0, fp, r1
	f6ff ae87 	blt.w	<smoltcp::iface::interface::Interface::poll+0x16ac>
	e693      	b.n	<smoltcp::iface::interface::Interface::poll+0x16c8>
	2500      	movs	r5, #0
	2600      	movs	r6, #0
	f240 0015 	movw	r0, #21
	f2c0 0000 	movt	r0, #0
	f001 fcc4 	bl	<defmt::export::acquire_and_header>
	f240 0007 	movw	r0, #7
	acbe      	add	r4, sp, #760	@ 0x2f8
	f2c0 0000 	movt	r0, #0
	2102      	movs	r1, #2
	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
	4620      	mov	r0, r4
	f001 fe6b 	bl	<_defmt_write>
	f240 0040 	movw	r0, #64	@ 0x40
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
	4620      	mov	r0, r4
	f001 fe61 	bl	<_defmt_write>
	f240 0805 	movw	r8, #5
	4620      	mov	r0, r4
	f2c0 0800 	movt	r8, #0
	2102      	movs	r1, #2
	f8ad 82f8 	strh.w	r8, [sp, #760]	@ 0x2f8
	f001 fe57 	bl	<_defmt_write>
	f244 2240 	movw	r2, #16960	@ 0x4240
	4628      	mov	r0, r5
	4631      	mov	r1, r6
	f2c0 020f 	movt	r2, #15
	2300      	movs	r3, #0
	f006 fd2c 	bl	<__aeabi_uldivmod>
	e9cd 01be 	strd	r0, r1, [sp, #760]	@ 0x2f8
	4620      	mov	r0, r4
	2108      	movs	r1, #8
	f001 fe48 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f8ad 82f8 	strh.w	r8, [sp, #760]	@ 0x2f8
	f001 fe42 	bl	<_defmt_write>
	4628      	mov	r0, r5
	4631      	mov	r1, r6
	f44f 727a 	mov.w	r2, #1000	@ 0x3e8
	2300      	movs	r3, #0
	f006 fd19 	bl	<__aeabi_uldivmod>
	f44f 727a 	mov.w	r2, #1000	@ 0x3e8
	2300      	movs	r3, #0
	f006 fd14 	bl	<__aeabi_uldivmod>
	4620      	mov	r0, r4
	2108      	movs	r1, #8
	e9cd 23be 	strd	r2, r3, [sp, #760]	@ 0x2f8
	f001 fe30 	bl	<_defmt_write>
	2500      	movs	r5, #0
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f8ad 52f8 	strh.w	r5, [sp, #760]	@ 0x2f8
	f001 fe29 	bl	<_defmt_write>
	f001 fdaf 	bl	<_defmt_release>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	e9d9 6312 	ldrd	r6, r3, [r9, #72]	@ 0x48
	e9d0 0104 	ldrd	r0, r1, [r0, #16]
	f8d9 4100 	ldr.w	r4, [r9, #256]	@ 0x100
	1980      	adds	r0, r0, r6
	f999 2028 	ldrsb.w	r2, [r9, #40]	@ 0x28
	f8c9 4108 	str.w	r4, [r9, #264]	@ 0x108
	f04f 0401 	mov.w	r4, #1
	4159      	adcs	r1, r3
	e9d9 ce10 	ldrd	ip, lr, [r9, #64]	@ 0x40
	fa82 f254 	uqadd8	r2, r2, r4
	e9c9 5500 	strd	r5, r5, [r9]
	e9c9 0124 	strd	r0, r1, [r9, #144]	@ 0x90
	b2d0      	uxtb	r0, r2
	2803      	cmp	r0, #3
	f889 2028 	strb.w	r2, [r9, #40]	@ 0x28
	e9c9 5520 	strd	r5, r5, [r9, #128]	@ 0x80
	e9c9 ce22 	strd	ip, lr, [r9, #136]	@ 0x88
	d30e      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x1aac>
	f8d9 0020 	ldr.w	r0, [r9, #32]
	2100      	movs	r1, #0
	f889 1028 	strb.w	r1, [r9, #40]	@ 0x28
	f242 7210 	movw	r2, #10000	@ 0x2710
	0041      	lsls	r1, r0, #1
	4291      	cmp	r1, r2
	f242 7110 	movw	r1, #10000	@ 0x2710
	bf38      	it	cc
	0041      	lslcc	r1, r0, #1
	f8c9 1020 	str.w	r1, [r9, #32]
	984a      	ldr	r0, [sp, #296]	@ 0x128
	68c1      	ldr	r1, [r0, #12]
	4648      	mov	r0, r9
	f003 fc8b 	bl	<smoltcp::socket::tcp::Socket::seq_to_transmit>
	2800      	cmp	r0, #0
	d03e      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1b38>
	f8b9 011c 	ldrh.w	r0, [r9, #284]	@ 0x11c
	07c0      	lsls	r0, r0, #31
	f000 83e0 	beq.w	<smoltcp::iface::interface::Interface::poll+0x2284>
	f899 112a 	ldrb.w	r1, [r9, #298]	@ 0x12a
	f44f 7a50 	mov.w	sl, #832	@ 0x340
	f8d9 40c4 	ldr.w	r4, [r9, #196]	@ 0xc4
	2900      	cmp	r1, #0
	f899 012b 	ldrb.w	r0, [r9, #299]	@ 0x12b
	bf18      	it	ne
	f500 7a40 	addne.w	sl, r0, #768	@ 0x300
	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
	f340 836d 	ble.w	<smoltcp::iface::interface::Interface::poll+0x21be>
	f8d9 011e 	ldr.w	r0, [r9, #286]	@ 0x11e
	903c      	str	r0, [sp, #240]	@ 0xf0
	f8d9 0124 	ldr.w	r0, [r9, #292]	@ 0x124
	9041      	str	r0, [sp, #260]	@ 0x104
	f8b9 0122 	ldrh.w	r0, [r9, #290]	@ 0x122
	903b      	str	r0, [sp, #236]	@ 0xec
	f8b9 0128 	ldrh.w	r0, [r9, #296]	@ 0x128
	f899 1134 	ldrb.w	r1, [r9, #308]	@ 0x134
	903a      	str	r0, [sp, #232]	@ 0xe8
	f8d9 00bc 	ldr.w	r0, [r9, #188]	@ 0xbc
	f001 011f 	and.w	r1, r1, #31
	e9d9 5641 	ldrd	r5, r6, [r9, #260]	@ 0x104
	1b00      	subs	r0, r0, r4
	f8d9 2114 	ldr.w	r2, [r9, #276]	@ 0x114
	fa20 f101 	lsr.w	r1, r0, r1
	f64f 70ff 	movw	r0, #65535	@ 0xffff
	4281      	cmp	r1, r0
	bf28      	it	cs
	4601      	movcs	r1, r0
	2a00      	cmp	r2, #0
	9142      	str	r1, [sp, #264]	@ 0x108
	923d      	str	r2, [sp, #244]	@ 0xf4
	f000 80c1 	beq.w	<smoltcp::iface::interface::Interface::poll+0x1cac>
	f8d9 0118 	ldr.w	r0, [r9, #280]	@ 0x118
	9037      	str	r0, [sp, #220]	@ 0xdc
	4790      	blx	r2
	9036      	str	r0, [sp, #216]	@ 0xd8
	2001      	movs	r0, #1
	e0ba      	b.n	<smoltcp::iface::interface::Interface::poll+0x1cae>
	f8d9 1098 	ldr.w	r1, [r9, #152]	@ 0x98
	2901      	cmp	r1, #1
	d10f      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1b60>
	f8d9 00c4 	ldr.w	r0, [r9, #196]	@ 0xc4
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f340 8339 	ble.w	<smoltcp::iface::interface::Interface::poll+0x21be>
	f8d9 3104 	ldr.w	r3, [r9, #260]	@ 0x104
	f8d9 209c 	ldr.w	r2, [r9, #156]	@ 0x9c
	4418      	add	r0, r3
	1a10      	subs	r0, r2, r0
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f340 814f 	ble.w	<smoltcp::iface::interface::Interface::poll+0x1dfe>
	f899 0133 	ldrb.w	r0, [r9, #307]	@ 0x133
	1e82      	subs	r2, r0, #2
	2a04      	cmp	r2, #4
	d834      	bhi.n	<smoltcp::iface::interface::Interface::poll+0x1bd4>
	f899 2134 	ldrb.w	r2, [r9, #308]	@ 0x134
	f8d9 30bc 	ldr.w	r3, [r9, #188]	@ 0xbc
	f8d9 50c4 	ldr.w	r5, [r9, #196]	@ 0xc4
	f002 021f 	and.w	r2, r2, #31
	1b5b      	subs	r3, r3, r5
	fa23 f602 	lsr.w	r6, r3, r2
	f64f 73ff 	movw	r3, #65535	@ 0xffff
	429e      	cmp	r6, r3
	bf38      	it	cc
	4633      	movcc	r3, r6
	b319      	cbz	r1, <smoltcp::iface::interface::Interface::poll+0x1bd4>
	f1b5 3fff 	cmp.w	r5, #4294967295	@ 0xffffffff
	bfc2      	ittt	gt
	f8b9 112e 	ldrhgt.w	r1, [r9, #302]	@ 0x12e
	4091      	lslgt	r1, r2
	f1b1 3fff 	cmpgt.w	r1, #4294967295	@ 0xffffffff
	f340 830f 	ble.w	<smoltcp::iface::interface::Interface::poll+0x21be>
	f8d9 409c 	ldr.w	r4, [r9, #156]	@ 0x9c
	f8d9 c104 	ldr.w	ip, [r9, #260]	@ 0x104
	4421      	add	r1, r4
	4465      	add	r5, ip
	1b49      	subs	r1, r1, r5
	f1b1 3fff 	cmp.w	r1, #4294967295	@ 0xffffffff
	f340 8373 	ble.w	<smoltcp::iface::interface::Interface::poll+0x229c>
	b16e      	cbz	r6, <smoltcp::iface::interface::Interface::poll+0x1bd4>
	40d1      	lsrs	r1, r2
	f64f 72ff 	movw	r2, #65535	@ 0xffff
	4291      	cmp	r1, r2
	bf28      	it	cs
	4611      	movcs	r1, r2
	ebb1 0f53 	cmp.w	r1, r3, lsr #1
	f04f 0100 	mov.w	r1, #0
	bf98      	it	ls
	2101      	movls	r1, #1
	b121      	cbz	r1, <smoltcp::iface::interface::Interface::poll+0x1bdc>
	e772      	b.n	<smoltcp::iface::interface::Interface::poll+0x1aba>
	2100      	movs	r1, #0
	2900      	cmp	r1, #0
	f47f af6f 	bne.w	<smoltcp::iface::interface::Interface::poll+0x1aba>
	2800      	cmp	r0, #0
	f43f af6c 	beq.w	<smoltcp::iface::interface::Interface::poll+0x1aba>
	4648      	mov	r0, r9
	994a      	ldr	r1, [sp, #296]	@ 0x128
	f850 5f80 	ldr.w	r5, [r0, #128]!
	f100 0c04 	add.w	ip, r0, #4
	e9d1 2e04 	ldrd	r2, lr, [r1, #16]
	e89c 1050 	ldmia.w	ip, {r4, r6, ip}
	ea55 0304 	orrs.w	r3, r5, r4
	d108      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1c0e>
	07f3      	lsls	r3, r6, #31
	d006      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1c0e>
	e9d0 3104 	ldrd	r3, r1, [r0, #16]
	1ad3      	subs	r3, r2, r3
	eb7e 0101 	sbcs.w	r1, lr, r1
	f6bf af56 	bge.w	<smoltcp::iface::interface::Interface::poll+0x1aba>
	f085 0103 	eor.w	r1, r5, #3
	9d48      	ldr	r5, [sp, #288]	@ 0x120
	4321      	orrs	r1, r4
	f47f acc5 	bne.w	<smoltcp::iface::interface::Interface::poll+0x15a4>
	1b91      	subs	r1, r2, r6
	eb7e 010c 	sbcs.w	r1, lr, ip
	f6ff acc0 	blt.w	<smoltcp::iface::interface::Interface::poll+0x15a4>
	462e      	mov	r6, r5
	2500      	movs	r5, #0
	e9c0 5500 	strd	r5, r5, [r0]
	f44f 7196 	mov.w	r1, #300	@ 0x12c
	e9c0 5502 	strd	r5, r5, [r0, #8]
	2064      	movs	r0, #100	@ 0x64
	e9c9 1008 	strd	r1, r0, [r9, #32]
	f109 00d0 	add.w	r0, r9, #208	@ 0xd0
	2128      	movs	r1, #40	@ 0x28
	e9c9 5500 	strd	r5, r5, [r9]
	f889 5133 	strb.w	r5, [r9, #307]	@ 0x133
	f8c9 5018 	str.w	r5, [r9, #24]
	f889 5130 	strb.w	r5, [r9, #304]	@ 0x130
	e9c9 5530 	strd	r5, r5, [r9, #192]	@ 0xc0
	f889 5028 	strb.w	r5, [r9, #40]	@ 0x28
	f8d9 40bc 	ldr.w	r4, [r9, #188]	@ 0xbc
	f8a9 511c 	strh.w	r5, [r9, #284]	@ 0x11c
	f889 50b2 	strb.w	r5, [r9, #178]	@ 0xb2
	f8a9 50b0 	strh.w	r5, [r9, #176]	@ 0xb0
	f8a9 512e 	strh.w	r5, [r9, #302]	@ 0x12e
	f8c9 5098 	str.w	r5, [r9, #152]	@ 0x98
	f889 512c 	strb.w	r5, [r9, #300]	@ 0x12c
	e9c9 5540 	strd	r5, r5, [r9, #256]	@ 0x100
	e9c9 5542 	strd	r5, r5, [r9, #264]	@ 0x108
	f005 fdaf 	bl	<__aeabi_memclr8>
	f44f 7006 	mov.w	r0, #536	@ 0x218
	e9c9 553e 	strd	r5, r5, [r9, #248]	@ 0xf8
	f8c9 0110 	str.w	r0, [r9, #272]	@ 0x110
	fab4 f084 	clz	r0, r4
	f1c0 0110 	rsb	r1, r0, #16
	2210      	movs	r2, #16
	e9c9 551c 	strd	r5, r5, [r9, #112]	@ 0x70
	4282      	cmp	r2, r0
	e9c9 5514 	strd	r5, r5, [r9, #80]	@ 0x50
	bf38      	it	cc
	4629      	movcc	r1, r5
	4635      	mov	r5, r6
	f889 1134 	strb.w	r1, [r9, #308]	@ 0x134
	e47b      	b.n	<smoltcp::iface::interface::Interface::poll+0x15a4>
	2000      	movs	r0, #0
	9039      	str	r0, [sp, #228]	@ 0xe4
	1960      	adds	r0, r4, r5
	f899 4133 	ldrb.w	r4, [r9, #307]	@ 0x133
	f04f 0800 	mov.w	r8, #0
	903f      	str	r0, [sp, #252]	@ 0xfc
	2001      	movs	r0, #1
	963e      	str	r6, [sp, #248]	@ 0xf8
	2200      	movs	r2, #0
	9045      	str	r0, [sp, #276]	@ 0x114
	2001      	movs	r0, #1
	9040      	str	r0, [sp, #256]	@ 0x100
	2000      	movs	r0, #0
	2300      	movs	r3, #0
	9046      	str	r0, [sp, #280]	@ 0x118
	f04f 0b01 	mov.w	fp, #1
	f04f 0e00 	mov.w	lr, #0
	9d48      	ldr	r5, [sp, #288]	@ 0x120
	e8df f014 	tbh	[pc, r4, lsl #1]
	000b00a3 	.word	0x000b00a3
	00510051 	.word	0x00510051
	000c000c 	.word	0x000c000c
	000c00f0 	.word	0x000c00f0
	000c000c 	.word	0x000c000c
	00f0      	.short	0x00f0
	e457      	b.n	<smoltcp::iface::interface::Interface::poll+0x15a4>
	f8d9 010c 	ldr.w	r0, [r9, #268]	@ 0x10c
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f340 825f 	ble.w	<smoltcp::iface::interface::Interface::poll+0x21be>
	f8d9 2100 	ldr.w	r2, [r9, #256]	@ 0x100
	994a      	ldr	r1, [sp, #296]	@ 0x128
	f8d9 3108 	ldr.w	r3, [r9, #264]	@ 0x108
	4410      	add	r0, r2
	f8d9 5110 	ldr.w	r5, [r9, #272]	@ 0x110
	1ac0      	subs	r0, r0, r3
	68ce      	ldr	r6, [r1, #12]
	ea20 71e0 	bic.w	r1, r0, r0, asr #31
	f1a6 0036 	sub.w	r0, r6, #54	@ 0x36
	428d      	cmp	r5, r1
	bf38      	it	cc
	4629      	movcc	r1, r5
	4288      	cmp	r0, r1
	bf38      	it	cc
	4601      	movcc	r1, r0
	1a9b      	subs	r3, r3, r2
	f100 82b7 	bmi.w	<smoltcp::iface::interface::Interface::poll+0x229c>
	f8d9 20cc 	ldr.w	r2, [r9, #204]	@ 0xcc
	2a00      	cmp	r2, #0
	d03f      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1db6>
	f8d9 00d0 	ldr.w	r0, [r9, #208]	@ 0xd0
	4418      	add	r0, r3
	fbb0 f6f2 	udiv	r6, r0, r2
	fb06 0012 	mls	r0, r6, r2, r0
	f8d9 50d4 	ldr.w	r5, [r9, #212]	@ 0xd4
	429d      	cmp	r5, r3
	d239      	bcs.n	<smoltcp::iface::interface::Interface::poll+0x1dc0>
	2001      	movs	r0, #1
	9040      	str	r0, [sp, #256]	@ 0x100
	eb08 0003 	add.w	r0, r8, r3
	42a8      	cmp	r0, r5
	d14b      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1df0>
	3c04      	subs	r4, #4
	2001      	movs	r0, #1
	2200      	movs	r2, #0
	2c05      	cmp	r4, #5
	9045      	str	r0, [sp, #276]	@ 0x114
	d848      	bhi.n	<smoltcp::iface::interface::Interface::poll+0x1df6>
	2100      	movs	r1, #0
	2300      	movs	r3, #0
	9146      	str	r1, [sp, #280]	@ 0x118
	e8df f004 	tbb	[pc, r4]
	03a0      	.short	0x03a0
	0303a0a7 	.word	0x0303a0a7
	2003      	movs	r0, #3
	9046      	str	r0, [sp, #280]	@ 0x118
	2001      	movs	r0, #1
	9045      	str	r0, [sp, #276]	@ 0x114
	e056      	b.n	<smoltcp::iface::interface::Interface::poll+0x1e2c>
	f8d9 00bc 	ldr.w	r0, [r9, #188]	@ 0xbc
	f8d9 10c4 	ldr.w	r1, [r9, #196]	@ 0xc4
	f8d9 2100 	ldr.w	r2, [r9, #256]	@ 0x100
	1a41      	subs	r1, r0, r1
	f64f 70ff 	movw	r0, #65535	@ 0xffff
	923e      	str	r2, [sp, #248]	@ 0xf8
	4281      	cmp	r1, r0
	bf28      	it	cs
	4601      	movcs	r1, r0
	2001      	movs	r0, #1
	2c02      	cmp	r4, #2
	9142      	str	r1, [sp, #264]	@ 0x108
	d148      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1e32>
	9040      	str	r0, [sp, #256]	@ 0x100
	2201      	movs	r2, #1
	f899 0134 	ldrb.w	r0, [r9, #308]	@ 0x134
	2301      	movs	r3, #1
	9032      	str	r0, [sp, #200]	@ 0xc8
	2000      	movs	r0, #0
	9045      	str	r0, [sp, #276]	@ 0x114
	2002      	movs	r0, #2
	9046      	str	r0, [sp, #280]	@ 0x118
	e049      	b.n	<smoltcp::iface::interface::Interface::poll+0x1e4a>
	2000      	movs	r0, #0
	f8d9 50d4 	ldr.w	r5, [r9, #212]	@ 0xd4
	429d      	cmp	r5, r3
	d3c5      	bcc.n	<smoltcp::iface::interface::Interface::poll+0x1d4c>
	eba5 0803 	sub.w	r8, r5, r3
	4541      	cmp	r1, r8
	bf38      	it	cc
	4688      	movcc	r8, r1
	1a11      	subs	r1, r2, r0
	4588      	cmp	r8, r1
	bf28      	it	cs
	4688      	movcs	r8, r1
	eb18 0100 	adds.w	r1, r8, r0
	f080 826c 	bcs.w	<smoltcp::iface::interface::Interface::poll+0x22b2>
	4291      	cmp	r1, r2
	f200 8269 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x22b2>
	f8d9 10c8 	ldr.w	r1, [r9, #200]	@ 0xc8
	4408      	add	r0, r1
	9040      	str	r0, [sp, #256]	@ 0x100
	eb08 0003 	add.w	r0, r8, r3
	42a8      	cmp	r0, r5
	d0b3      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1d58>
	2001      	movs	r0, #1
	2200      	movs	r2, #0
	9045      	str	r0, [sp, #276]	@ 0x114
	2300      	movs	r3, #0
	2000      	movs	r0, #0
	9046      	str	r0, [sp, #280]	@ 0x118
	e05e      	b.n	<smoltcp::iface::interface::Interface::poll+0x1ebc>
	e9d9 021c 	ldrd	r0, r2, [r9, #112]	@ 0x70
	f080 0001 	eor.w	r0, r0, #1
	4310      	orrs	r0, r2
	f47f ae57 	bne.w	<smoltcp::iface::interface::Interface::poll+0x1aba>
	9b4a      	ldr	r3, [sp, #296]	@ 0x128
	e9d9 021e 	ldrd	r0, r2, [r9, #120]	@ 0x78
	e9d3 3604 	ldrd	r3, r6, [r3, #16]
	1a18      	subs	r0, r3, r0
	eb76 0002 	sbcs.w	r0, r6, r2
	f6bf ae4d 	bge.w	<smoltcp::iface::interface::Interface::poll+0x1aba>
	e69e      	b.n	<smoltcp::iface::interface::Interface::poll+0x1b60>
	2004      	movs	r0, #4
	9046      	str	r0, [sp, #280]	@ 0x118
	2001      	movs	r0, #1
	9045      	str	r0, [sp, #276]	@ 0x114
	9040      	str	r0, [sp, #256]	@ 0x100
	f04f 0b00 	mov.w	fp, #0
	e044      	b.n	<smoltcp::iface::interface::Interface::poll+0x1ebc>
	9045      	str	r0, [sp, #276]	@ 0x114
	f899 0134 	ldrb.w	r0, [r9, #308]	@ 0x134
	9032      	str	r0, [sp, #200]	@ 0xc8
	2002      	movs	r0, #2
	9046      	str	r0, [sp, #280]	@ 0x118
	2001      	movs	r0, #1
	f899 212c 	ldrb.w	r2, [r9, #300]	@ 0x12c
	f899 3131 	ldrb.w	r3, [r9, #305]	@ 0x131
	9040      	str	r0, [sp, #256]	@ 0x100
	f04f 0b00 	mov.w	fp, #0
	f04f 0e01 	mov.w	lr, #1
	e033      	b.n	<smoltcp::iface::interface::Interface::poll+0x1ebc>
	f04f 0c00 	mov.w	ip, #0
	9845      	ldr	r0, [sp, #276]	@ 0x114
	42b1      	cmp	r1, r6
	f200 8207 	bhi.w	<smoltcp::iface::interface::Interface::poll+0x226e>
	2202      	movs	r2, #2
	2d00      	cmp	r5, #0
	f47f ad8a 	bne.w	<smoltcp::iface::interface::Interface::poll+0x197c>
	2300      	movs	r3, #0
	9d48      	ldr	r5, [sp, #288]	@ 0x120
	1a61      	subs	r1, r4, r1
	2a02      	cmp	r2, #2
	e9c9 3112 	strd	r3, r1, [r9, #72]	@ 0x48
	d10a      	bne.n	<smoltcp::iface::interface::Interface::poll+0x1e8c>
	9940      	ldr	r1, [sp, #256]	@ 0x100
	9b41      	ldr	r3, [sp, #260]	@ 0x104
	3101      	adds	r1, #1
	fbb1 f2f3 	udiv	r2, r1, r3
	fb02 1113 	mls	r1, r2, r3, r1
	1e42      	subs	r2, r0, #1
	e9c9 120e 	strd	r1, r2, [r9, #56]	@ 0x38
	2202      	movs	r2, #2
	2a02      	cmp	r2, #2
	f04f 0102 	mov.w	r1, #2
	bf28      	it	cs
	460a      	movcs	r2, r1
	2a02      	cmp	r2, #2
	f43f ab84 	beq.w	<smoltcp::iface::interface::Interface::poll+0x15a4>
	07d1      	lsls	r1, r2, #31
	f000 819f 	beq.w	<smoltcp::iface::interface::Interface::poll+0x21e0>
	9a47      	ldr	r2, [sp, #284]	@ 0x11c
	f1bc 0f00 	cmp.w	ip, #0
	f040 80f3 	bne.w	<smoltcp::iface::interface::Interface::poll+0x2092>
	e1cc      	b.n	<smoltcp::iface::interface::Interface::poll+0x2248>
	f1b8 0f00 	cmp.w	r8, #0
	f000 815c 	beq.w	<smoltcp::iface::interface::Interface::poll+0x216e>
	2001      	movs	r0, #1
	e9cd 0045 	strd	r0, r0, [sp, #276]	@ 0x114
	f109 0680 	add.w	r6, r9, #128	@ 0x80
	984a      	ldr	r0, [sp, #296]	@ 0x128
	9247      	str	r2, [sp, #284]	@ 0x11c
	469c      	mov	ip, r3
	ce70      	ldmia	r6, {r4, r5, r6}
	e9d9 1224 	ldrd	r1, r2, [r9, #144]	@ 0x90
	4325      	orrs	r5, r4
	e9d0 3004 	ldrd	r3, r0, [r0, #16]
	fab5 f585 	clz	r5, r5
	1a59      	subs	r1, r3, r1
	ea4f 1555 	mov.w	r5, r5, lsr #5
	4190      	sbcs	r0, r2
	ea06 0605 	and.w	r6, r6, r5
	f04f 0000 	mov.w	r0, #0
	fab8 f188 	clz	r1, r8
	bfa8      	it	ge
	2001      	movge	r0, #1
	4030      	ands	r0, r6
	0949      	lsrs	r1, r1, #5
	ea00 000b 	and.w	r0, r0, fp
	ea10 0201 	ands.w	r2, r0, r1
	9234      	str	r2, [sp, #208]	@ 0xd0
	d00e      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1f1c>
	f8dd b0f8 	ldr.w	fp, [sp, #248]	@ 0xf8
	f649 4490 	movw	r4, #40080	@ 0x9c90
	9a43      	ldr	r2, [sp, #268]	@ 0x10c
	f04f 0801 	mov.w	r8, #1
	9d48      	ldr	r5, [sp, #288]	@ 0x120
	f1ab 0b01 	sub.w	fp, fp, #1
	f6c0 0400 	movt	r4, #2048	@ 0x800
	2000      	movs	r0, #0
	2314      	movs	r3, #20
	e01c      	b.n	<smoltcp::iface::interface::Interface::poll+0x1f56>
	f8dd b0f8 	ldr.w	fp, [sp, #248]	@ 0xf8
	f1b8 0f00 	cmp.w	r8, #0
	d008      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1f38>
	f8d9 0100 	ldr.w	r0, [r9, #256]	@ 0x100
	f8d9 1108 	ldr.w	r1, [r9, #264]	@ 0x108
	1a08      	subs	r0, r1, r0
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f340 81b2 	ble.w	<smoltcp::iface::interface::Interface::poll+0x229c>
	f1be 0f00 	cmp.w	lr, #0
	d006      	beq.n	<smoltcp::iface::interface::Interface::poll+0x1f4c>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	2318      	movs	r3, #24
	8980      	ldrh	r0, [r0, #12]
	3836      	subs	r0, #54	@ 0x36
	9033      	str	r0, [sp, #204]	@ 0xcc
	2001      	movs	r0, #1
	e001      	b.n	<smoltcp::iface::interface::Interface::poll+0x1f50>
	2000      	movs	r0, #0
	2314      	movs	r3, #20
	9a43      	ldr	r2, [sp, #268]	@ 0x10c
	9d48      	ldr	r5, [sp, #288]	@ 0x120
	9c40      	ldr	r4, [sp, #256]	@ 0x100
	9946      	ldr	r1, [sp, #280]	@ 0x118
	4666      	mov	r6, ip
	f88d 1351 	strb.w	r1, [sp, #849]	@ 0x351
	9942      	ldr	r1, [sp, #264]	@ 0x108
	f8ad 134e 	strh.w	r1, [sp, #846]	@ 0x34e
	993a      	ldr	r1, [sp, #232]	@ 0xe8
	f8ad 134c 	strh.w	r1, [sp, #844]	@ 0x34c
	993b      	ldr	r1, [sp, #236]	@ 0xec
	f8ad 134a 	strh.w	r1, [sp, #842]	@ 0x34a
	9932      	ldr	r1, [sp, #200]	@ 0xc8
	f88d 1349 	strb.w	r1, [sp, #841]	@ 0x349
	a9cf      	add	r1, sp, #828	@ 0x33c
	f8ad 0338 	strh.w	r0, [sp, #824]	@ 0x338
	9837      	ldr	r0, [sp, #220]	@ 0xdc
	e881 0910 	stmia.w	r1, {r4, r8, fp}
	9933      	ldr	r1, [sp, #204]	@ 0xcc
	90cd      	str	r0, [sp, #820]	@ 0x334
	9836      	ldr	r0, [sp, #216]	@ 0xd8
	f8ad 133a 	strh.w	r1, [sp, #826]	@ 0x33a
	90cc      	str	r0, [sp, #816]	@ 0x330
	9839      	ldr	r0, [sp, #228]	@ 0xe4
	993f      	ldr	r1, [sp, #252]	@ 0xfc
	f88d c350 	strb.w	ip, [sp, #848]	@ 0x350
	f8dd c11c 	ldr.w	ip, [sp, #284]	@ 0x11c
	90cb      	str	r0, [sp, #812]	@ 0x32c
	2000      	movs	r0, #0
	91c1      	str	r1, [sp, #772]	@ 0x304
	f1bc 0f00 	cmp.w	ip, #0
	9945      	ldr	r1, [sp, #276]	@ 0x114
	91c0      	str	r1, [sp, #768]	@ 0x300
	9c41      	ldr	r4, [sp, #260]	@ 0x104
	993c      	ldr	r1, [sp, #240]	@ 0xf0
	90c8      	str	r0, [sp, #800]	@ 0x320
	90c5      	str	r0, [sp, #788]	@ 0x314
	90c2      	str	r0, [sp, #776]	@ 0x308
	90bf      	str	r0, [sp, #764]	@ 0x2fc
	984a      	ldr	r0, [sp, #296]	@ 0x128
	f88d c348 	strb.w	ip, [sp, #840]	@ 0x348
	f8cd a360 	str.w	sl, [sp, #864]	@ 0x360
	94d6      	str	r4, [sp, #856]	@ 0x358
	91d5      	str	r1, [sp, #852]	@ 0x354
	90be      	str	r0, [sp, #760]	@ 0x2f8
	bf18      	it	ne
	f043 0303 	orrne.w	r3, r3, #3
	993d      	ldr	r1, [sp, #244]	@ 0xf4
	eb03 0046 	add.w	r0, r3, r6, lsl #1
	2900      	cmp	r1, #0
	bf18      	it	ne
	300a      	addne	r0, #10
	3003      	adds	r0, #3
	f020 0003 	bic.w	r0, r0, #3
	4440      	add	r0, r8
	90d7      	str	r0, [sp, #860]	@ 0x35c
	68b8      	ldr	r0, [r7, #8]
	e9d0 1004 	ldrd	r1, r0, [r0, #16]
	4288      	cmp	r0, r1
	f4be a869 	bcs.w	<smoltcp::iface::interface::Interface::poll+0xbe>
	f44f 61c6 	mov.w	r1, #1584	@ 0x630
	4348      	muls	r0, r1
	6811      	ldr	r1, [r2, #0]
	5808      	ldr	r0, [r1, r0]
	2800      	cmp	r0, #0
	f100 80ec 	bmi.w	<smoltcp::iface::interface::Interface::poll+0x21d4>
	2000      	movs	r0, #0
	9b38      	ldr	r3, [sp, #224]	@ 0xe0
	90a5      	str	r0, [sp, #660]	@ 0x294
	a9a5      	add	r1, sp, #660	@ 0x294
	984a      	ldr	r0, [sp, #296]	@ 0x128
	92a7      	str	r2, [sp, #668]	@ 0x29c
	2200      	movs	r2, #0
	f7fd f919 	bl	<smoltcp::iface::interface::InterfaceInner::dispatch_ip>
	b2c0      	uxtb	r0, r0
	2802      	cmp	r0, #2
	d13d      	bne.n	<smoltcp::iface::interface::Interface::poll+0x2090>
	e9d9 0120 	ldrd	r0, r1, [r9, #128]	@ 0x80
	4308      	orrs	r0, r1
	d10c      	bne.n	<smoltcp::iface::interface::Interface::poll+0x2036>
	9e4a      	ldr	r6, [sp, #296]	@ 0x128
	f109 0340 	add.w	r3, r9, #64	@ 0x40
	f109 0c88 	add.w	ip, r9, #136	@ 0x88
	e9d6 6504 	ldrd	r6, r5, [r6, #16]
	cb0f      	ldmia	r3, {r0, r1, r2, r3}
	1992      	adds	r2, r2, r6
	416b      	adcs	r3, r5
	9d48      	ldr	r5, [sp, #288]	@ 0x120
	e88c 000f 	stmia.w	ip, {r0, r1, r2, r3}
	2000      	movs	r0, #0
	9447      	str	r4, [sp, #284]	@ 0x11c
	e9c9 001c 	strd	r0, r0, [r9, #112]	@ 0x70
	9834      	ldr	r0, [sp, #208]	@ 0xd0
	2800      	cmp	r0, #0
	f47f aaaf 	bne.w	<smoltcp::iface::interface::Interface::poll+0x15a4>
	9846      	ldr	r0, [sp, #280]	@ 0x118
	f000 0106 	and.w	r1, r0, #6
	4640      	mov	r0, r8
	2902      	cmp	r1, #2
	bf08      	it	eq
	3001      	addeq	r0, #1
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f340 80b1 	ble.w	<smoltcp::iface::interface::Interface::poll+0x21be>
	9a42      	ldr	r2, [sp, #264]	@ 0x108
	4458      	add	r0, fp
	f8a9 212e 	strh.w	r2, [r9, #302]	@ 0x12e
	2902      	cmp	r1, #2
	9a3f      	ldr	r2, [sp, #252]	@ 0xfc
	f04f 0100 	mov.w	r1, #0
	9b45      	ldr	r3, [sp, #276]	@ 0x114
	f8c9 0108 	str.w	r0, [r9, #264]	@ 0x108
	e9c9 3226 	strd	r3, r2, [r9, #152]	@ 0x98
	bf08      	it	eq
	f04f 31ff 	moveq.w	r1, #4294967295	@ 0xffffffff
	4588      	cmp	r8, r1
	d11c      	bne.n	<smoltcp::iface::interface::Interface::poll+0x20ba>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	68c1      	ldr	r1, [r0, #12]
	4648      	mov	r0, r9
	f003 f9a1 	bl	<smoltcp::socket::tcp::Socket::seq_to_transmit>
	f8dd 8104 	ldr.w	r8, [sp, #260]	@ 0x104
	e060      	b.n	<smoltcp::iface::interface::Interface::poll+0x2152>
	4622      	mov	r2, r4
	984a      	ldr	r0, [sp, #296]	@ 0x128
	9247      	str	r2, [sp, #284]	@ 0x11c
	e9d0 0104 	ldrd	r0, r1, [r0, #16]
	f8c9 2139 	str.w	r2, [r9, #313]	@ 0x139
	2201      	movs	r2, #1
	f889 2138 	strb.w	r2, [r9, #312]	@ 0x138
	f244 2240 	movw	r2, #16960	@ 0x4240
	f2c0 020f 	movt	r2, #15
	1880      	adds	r0, r0, r2
	f141 0100 	adc.w	r1, r1, #0
	e9c9 0150 	strd	r0, r1, [r9, #320]	@ 0x140
	f7ff ba75 	b.w	<smoltcp::iface::interface::Interface::poll+0x15a4>
	994a      	ldr	r1, [sp, #296]	@ 0x128
	f8dd 8104 	ldr.w	r8, [sp, #260]	@ 0x104
	e9d1 2104 	ldrd	r2, r1, [r1, #16]
	f8d9 3018 	ldr.w	r3, [r9, #24]
	2b01      	cmp	r3, #1
	d104      	bne.n	<smoltcp::iface::interface::Interface::poll+0x20d6>
	f8d9 301c 	ldr.w	r3, [r9, #28]
	1ac3      	subs	r3, r0, r3
	2b01      	cmp	r3, #1
	db0e      	blt.n	<smoltcp::iface::interface::Interface::poll+0x20f4>
	e9d9 3600 	ldrd	r3, r6, [r9]
	2401      	movs	r4, #1
	e9c9 4006 	strd	r4, r0, [r9, #24]
	4333      	orrs	r3, r6
	d107      	bne.n	<smoltcp::iface::interface::Interface::poll+0x20f4>
	2301      	movs	r3, #1
	e9c9 1003 	strd	r1, r0, [r9, #12]
	f8c9 3000 	str.w	r3, [r9]
	2300      	movs	r3, #0
	e9c9 3201 	strd	r3, r2, [r9, #4]
	984a      	ldr	r0, [sp, #296]	@ 0x128
	68c1      	ldr	r1, [r0, #12]
	4648      	mov	r0, r9
	f003 f967 	bl	<smoltcp::socket::tcp::Socket::seq_to_transmit>
	bb40      	cbnz	r0, <smoltcp::iface::interface::Interface::poll+0x2152>
	984a      	ldr	r0, [sp, #296]	@ 0x128
	f8d9 2080 	ldr.w	r2, [r9, #128]	@ 0x80
	e9d0 1004 	ldrd	r1, r0, [r0, #16]
	e8df f002 	tbb	[pc, r2]
	3502      	.short	0x3502
	2202      	.short	0x2202
	e9d9 2308 	ldrd	r2, r3, [r9, #32]
	009e      	lsls	r6, r3, #2
	2e05      	cmp	r6, #5
	f04f 0605 	mov.w	r6, #5
	bf88      	it	hi
	009e      	lslhi	r6, r3, #2
	4432      	add	r2, r6
	2a0a      	cmp	r2, #10
	bf98      	it	ls
	220a      	movls	r2, #10
	f242 7310 	movw	r3, #10000	@ 0x2710
	429a      	cmp	r2, r3
	bf28      	it	cs
	461a      	movcs	r2, r3
	f44f 737a 	mov.w	r3, #1000	@ 0x3e8
	435a      	muls	r2, r3
	2301      	movs	r3, #1
	f8c9 3080 	str.w	r3, [r9, #128]	@ 0x80
	2300      	movs	r3, #0
	1889      	adds	r1, r1, r2
	f140 0000 	adc.w	r0, r0, #0
	e9c9 3121 	strd	r3, r1, [r9, #132]	@ 0x84
	f109 018c 	add.w	r1, r9, #140	@ 0x8c
	c10d      	stmia	r1!, {r0, r2, r3}
	f899 0133 	ldrb.w	r0, [r9, #307]	@ 0x133
	f8cd 811c 	str.w	r8, [sp, #284]	@ 0x11c
	2800      	cmp	r0, #0
	f47f aa22 	bne.w	<smoltcp::iface::interface::Interface::poll+0x15a4>
	2000      	movs	r0, #0
	f8a9 011c 	strh.w	r0, [r9, #284]	@ 0x11c
	9841      	ldr	r0, [sp, #260]	@ 0x104
	9047      	str	r0, [sp, #284]	@ 0x11c
	f7ff ba1b 	b.w	<smoltcp::iface::interface::Interface::poll+0x15a4>
	2001      	movs	r0, #1
	f04f 0800 	mov.w	r8, #0
	9045      	str	r0, [sp, #276]	@ 0x114
	e63f      	b.n	<smoltcp::iface::interface::Interface::poll+0x1df8>
	e9d9 2322 	ldrd	r2, r3, [r9, #136]	@ 0x88
	1a8a      	subs	r2, r1, r2
	eb70 0203 	sbcs.w	r2, r0, r3
	dbe6      	blt.n	<smoltcp::iface::interface::Interface::poll+0x2152>
	e9d9 2324 	ldrd	r2, r3, [r9, #144]	@ 0x90
	005e      	lsls	r6, r3, #1
	1889      	adds	r1, r1, r2
	4158      	adcs	r0, r3
	2301      	movs	r3, #1
	f8c9 3080 	str.w	r3, [r9, #128]	@ 0x80
	2300      	movs	r3, #0
	e9c9 3121 	strd	r3, r1, [r9, #132]	@ 0x84
	f109 018c 	add.w	r1, r9, #140	@ 0x8c
	ea46 76d2 	orr.w	r6, r6, r2, lsr #31
	0052      	lsls	r2, r2, #1
	c145      	stmia	r1!, {r0, r2, r6}
	e7d4      	b.n	<smoltcp::iface::interface::Interface::poll+0x2152>
	f24b 000c 	movw	r0, #45068	@ 0xb00c
	f24b 0238 	movw	r2, #45112	@ 0xb038
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	212c      	movs	r1, #44	@ 0x2c
	f000 fb25 	bl	<core::panicking::panic>
	f64a 4078 	movw	r0, #44152	@ 0xac78
	f64a 42b0 	movw	r2, #44208	@ 0xacb0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2171      	movs	r1, #113	@ 0x71
	f000 faf8 	bl	<core::panicking::panic_fmt>
	f240 0025 	movw	r0, #37	@ 0x25
	f2c0 0000 	movt	r0, #0
	f001 f8bd 	bl	<defmt::export::acquire_header_and_release>
	f50d 7d5d 	add.w	sp, sp, #884	@ 0x374
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f24b 00f8 	movw	r0, #45304	@ 0xb0f8
	f24b 123c 	movw	r2, #45372	@ 0xb13c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2141      	movs	r1, #65	@ 0x41
	f000 ff1b 	bl	<core::option::expect_failed>
	f24b 104c 	movw	r0, #45388	@ 0xb14c
	f24b 1290 	movw	r2, #45456	@ 0xb190
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2142      	movs	r1, #66	@ 0x42
	f000 ff10 	bl	<core::option::expect_failed>
	f24b 0368 	movw	r3, #45160	@ 0xb068
	2000      	movs	r0, #0
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f240 52f2 	movw	r2, #1522	@ 0x5f2
	f000 fa5d 	bl	<core::slice::index::slice_index_fail>
	f64a 13bc 	movw	r3, #43452	@ 0xa9bc
	4640      	mov	r0, r8
	f6c0 0300 	movt	r3, #2048	@ 0x800
	462a      	mov	r2, r5
	f000 fa55 	bl	<core::slice::index::slice_index_fail>
	f64a 13dc 	movw	r3, #43484	@ 0xa9dc
	2000      	movs	r0, #0
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4632      	mov	r2, r6
	f000 fa4d 	bl	<core::slice::index::slice_index_fail>
	f24a 5060 	movw	r0, #42336	@ 0xa560
	f24a 5278 	movw	r2, #42360	@ 0xa578
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2116      	movs	r1, #22
	f000 feec 	bl	<core::option::expect_failed>
	f64a 13bc 	movw	r3, #43452	@ 0xa9bc
	4628      	mov	r0, r5
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4631      	mov	r1, r6
	f000 fa3a 	bl	<core::slice::index::slice_index_fail>
	f64a 1088 	movw	r0, #43400	@ 0xa988
	f64a 12ac 	movw	r2, #43436	@ 0xa9ac
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2122      	movs	r1, #34	@ 0x22
	f000 fac2 	bl	<core::panicking::panic>
	f64a 1058 	movw	r0, #43352	@ 0xa958
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f000 fa1f 	bl	<core::option::unwrap_failed>
	f64a 10cc 	movw	r0, #43468	@ 0xa9cc
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f000 fa19 	bl	<core::option::unwrap_failed>
	f64a 209c 	movw	r0, #43676	@ 0xaa9c
	f64a 22d0 	movw	r2, #43728	@ 0xaad0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2167      	movs	r1, #103	@ 0x67
	f000 fa89 	bl	<core::panicking::panic_fmt>
	f64a 1368 	movw	r3, #43368	@ 0xa968
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f000 fa12 	bl	<core::slice::index::slice_index_fail>
	f24b 0278 	movw	r2, #45176	@ 0xb078
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f000 fa8b 	bl	<core::panicking::panic_bounds_check>
	9a2a      	ldr	r2, [sp, #168]	@ 0xa8
	f64a 632c 	movw	r3, #44588	@ 0xae2c
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2000      	movs	r0, #0
	f000 fa04 	bl	<core::slice::index::slice_index_fail>
	f64a 60bc 	movw	r0, #44732	@ 0xaebc
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f000 feda 	bl	<core::panicking::panic_const::panic_const_rem_by_zero>
	f24b 0288 	movw	r2, #45192	@ 0xb088
	4619      	mov	r1, r3
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f000 fa76 	bl	<core::panicking::panic_bounds_check>
	f64a 1278 	movw	r2, #43384	@ 0xa978
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f000 fa70 	bl	<core::panicking::panic_bounds_check>
	f64a 636c 	movw	r3, #44652	@ 0xae6c
	2008      	movs	r0, #8
	f6c0 0300 	movt	r3, #2048	@ 0x800
	210e      	movs	r1, #14
	f000 f9e9 	bl	<core::slice::index::slice_index_fail>
	f64a 637c 	movw	r3, #44668	@ 0xae7c
	200e      	movs	r0, #14
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f000 f9e2 	bl	<core::slice::index::slice_index_fail>
	f64a 638c 	movw	r3, #44684	@ 0xae8c
	4608      	mov	r0, r1
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4661      	mov	r1, ip
	f000 f9da 	bl	<core::slice::index::slice_index_fail>
	f64a 639c 	movw	r3, #44700	@ 0xae9c
	2018      	movs	r0, #24
	f6c0 0300 	movt	r3, #2048	@ 0x800
	211c      	movs	r1, #28
	f000 f9d2 	bl	<core::slice::index::slice_index_fail>
	f64a 1278 	movw	r2, #43384	@ 0xa978
	4619      	mov	r1, r3
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f000 fa4a 	bl	<core::panicking::panic_bounds_check>
	f64a 53ac 	movw	r3, #44460	@ 0xadac
	4620      	mov	r0, r4
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f000 f9c4 	bl	<core::slice::index::slice_index_fail>
	f64a 4348 	movw	r3, #44104	@ 0xac48
	4658      	mov	r0, fp
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4629      	mov	r1, r5
	462a      	mov	r2, r5
	f000 f9bb 	bl	<core::slice::index::slice_index_fail>
	f24b 0298 	movw	r2, #45208	@ 0xb098
	4620      	mov	r0, r4
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f000 fa33 	bl	<core::panicking::panic_bounds_check>
	2002      	movs	r0, #2
	908c      	str	r0, [sp, #560]	@ 0x230
	a88c      	add	r0, sp, #560	@ 0x230
	f002 fde7 	bl	<core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>
	f64a 200c 	movw	r0, #43532	@ 0xaa0c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f000 f99f 	bl	<core::option::unwrap_failed>
	f24b 0258 	movw	r2, #45144	@ 0xb058
	4620      	mov	r0, r4
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f000 fa21 	bl	<core::panicking::panic_bounds_check>
	f64a 232c 	movw	r3, #43564	@ 0xaa2c
	2000      	movs	r0, #0
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2102      	movs	r1, #2
	4622      	mov	r2, r4
	f000 f999 	bl	<core::slice::index::slice_index_fail>
	f64a 4328 	movw	r3, #44072	@ 0xac28
	2000      	movs	r0, #0
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2104      	movs	r1, #4
	4622      	mov	r2, r4
	f000 f990 	bl	<core::slice::index::slice_index_fail>
	f64a 4338 	movw	r3, #44088	@ 0xac38
	2004      	movs	r0, #4
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2108      	movs	r1, #8
	4622      	mov	r2, r4
	f000 f987 	bl	<core::slice::index::slice_index_fail>
	992c      	ldr	r1, [sp, #176]	@ 0xb0
	f64a 3360 	movw	r3, #43872	@ 0xab60
	f6c0 0300 	movt	r3, #2048	@ 0x800
	460a      	mov	r2, r1
	f000 f97f 	bl	<core::slice::index::slice_index_fail>

<core::fmt::write>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b085      	sub	sp, #20
	4606      	mov	r6, r0
	469b      	mov	fp, r3
	468a      	mov	sl, r1
	07d8      	lsls	r0, r3, #31
	d164      	bne.n	<core::fmt::write+0xde>
	7815      	ldrb	r5, [r2, #0]
	2000      	movs	r0, #0
	2d00      	cmp	r5, #0
	d06e      	beq.n	<core::fmt::write+0xfa>
	f8da 900c 	ldr.w	r9, [sl, #12]
	f04f 0800 	mov.w	r8, #0
	e002      	b.n	<core::fmt::write+0x2c>
	7815      	ldrb	r5, [r2, #0]
	2d00      	cmp	r5, #0
	d06a      	beq.n	<core::fmt::write+0x102>
	1c54      	adds	r4, r2, #1
	b268      	sxtb	r0, r5
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	dd07      	ble.n	<core::fmt::write+0x46>
	4630      	mov	r0, r6
	4621      	mov	r1, r4
	462a      	mov	r2, r5
	47c8      	blx	r9
	2800      	cmp	r0, #0
	d15a      	bne.n	<core::fmt::write+0xf8>
	1962      	adds	r2, r4, r5
	e7ef      	b.n	<core::fmt::write+0x26>
	2d80      	cmp	r5, #128	@ 0x80
	d009      	beq.n	<core::fmt::write+0x5e>
	2dc0      	cmp	r5, #192	@ 0xc0
	d112      	bne.n	<core::fmt::write+0x74>
	f85b 0038 	ldr.w	r0, [fp, r8, lsl #3]
	2100      	movs	r1, #0
	9104      	str	r1, [sp, #16]
	2120      	movs	r1, #32
	f2c6 0100 	movt	r1, #24576	@ 0x6000
	e031      	b.n	<core::fmt::write+0xc2>
	f8b2 4001 	ldrh.w	r4, [r2, #1]
	1cd5      	adds	r5, r2, #3
	4630      	mov	r0, r6
	4629      	mov	r1, r5
	4622      	mov	r2, r4
	47c8      	blx	r9
	2800      	cmp	r0, #0
	d143      	bne.n	<core::fmt::write+0xf8>
	192a      	adds	r2, r5, r4
	e7d8      	b.n	<core::fmt::write+0x26>
	07e8      	lsls	r0, r5, #31
	d103      	bne.n	<core::fmt::write+0x80>
	2120      	movs	r1, #32
	f2c6 0100 	movt	r1, #24576	@ 0x6000
	e002      	b.n	<core::fmt::write+0x86>
	f8d2 1001 	ldr.w	r1, [r2, #1]
	1d54      	adds	r4, r2, #5
	07a8      	lsls	r0, r5, #30
	bf4c      	ite	mi
	f834 2b02 	ldrhmi.w	r2, [r4], #2
	2200      	movpl	r2, #0
	0768      	lsls	r0, r5, #29
	bf4c      	ite	mi
	f834 3b02 	ldrhmi.w	r3, [r4], #2
	2300      	movpl	r3, #0
	0728      	lsls	r0, r5, #28
	bf48      	it	mi
	f834 8b02 	ldrhmi.w	r8, [r4], #2
	06e8      	lsls	r0, r5, #27
	bf44      	itt	mi
	eb0b 00c2 	addmi.w	r0, fp, r2, lsl #3
	8882      	ldrhmi	r2, [r0, #4]
	06a8      	lsls	r0, r5, #26
	bf44      	itt	mi
	eb0b 00c3 	addmi.w	r0, fp, r3, lsl #3
	8883      	ldrhmi	r3, [r0, #4]
	f85b 0038 	ldr.w	r0, [fp, r8, lsl #3]
	f8ad 3012 	strh.w	r3, [sp, #18]
	f8ad 2010 	strh.w	r2, [sp, #16]
	9103      	str	r1, [sp, #12]
	eb0b 01c8 	add.w	r1, fp, r8, lsl #3
	f8cd a008 	str.w	sl, [sp, #8]
	684a      	ldr	r2, [r1, #4]
	a901      	add	r1, sp, #4
	9601      	str	r6, [sp, #4]
	4790      	blx	r2
	b980      	cbnz	r0, <core::fmt::write+0xf8>
	f108 0801 	add.w	r8, r8, #1
	4622      	mov	r2, r4
	e7a3      	b.n	<core::fmt::write+0x26>
	ea4f 035b 	mov.w	r3, fp, lsr #1
	4611      	mov	r1, r2
	f8da c00c 	ldr.w	ip, [sl, #12]
	4630      	mov	r0, r6
	461a      	mov	r2, r3
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
	4760      	bx	ip
	2001      	movs	r0, #1
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	2000      	movs	r0, #0
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<core::fmt::Formatter::pad_integral>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b085      	sub	sp, #20
	6886      	ldr	r6, [r0, #8]
	460c      	mov	r4, r1
	f8b0 800c 	ldrh.w	r8, [r0, #12]
	4692      	mov	sl, r2
	f416 1100 	ands.w	r1, r6, #2097152	@ 0x200000
	f04f 0b2b 	mov.w	fp, #43	@ 0x2b
	f3c6 53c0 	ubfx	r3, r6, #23, #1
	bf08      	it	eq
	f44f 1b88 	moveq.w	fp, #1114112	@ 0x110000
	eb02 5951 	add.w	r9, r2, r1, lsr #21
	45c1      	cmp	r9, r8
	d211      	bcs.n	<core::fmt::Formatter::pad_integral+0x52>
	01f1      	lsls	r1, r6, #7
	d41d      	bmi.n	<core::fmt::Formatter::pad_integral+0x6e>
	f3c6 7141 	ubfx	r1, r6, #29, #2
	9304      	str	r3, [sp, #16]
	9403      	str	r4, [sp, #12]
	eba8 0309 	sub.w	r3, r8, r9
	f36f 565f 	bfc	r6, #21, #11
	f04f 0800 	mov.w	r8, #0
	e8df f001 	tbb	[pc, r1]
	025b      	.short	0x025b
	0258      	.short	0x0258
	4698      	mov	r8, r3
	e056      	b.n	<core::fmt::Formatter::pad_integral+0x100>
	4626      	mov	r6, r4
	e9d0 4500 	ldrd	r4, r5, [r0]
	4629      	mov	r1, r5
	465a      	mov	r2, fp
	4620      	mov	r0, r4
	f000 f89d 	bl	<core::fmt::Formatter::pad_integral::write_prefix>
	b300      	cbz	r0, <core::fmt::Formatter::pad_integral+0xa6>
	2001      	movs	r0, #1
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	e9d0 6102 	ldrd	r6, r1, [r0, #8]
	465a      	mov	r2, fp
	f8cd a010 	str.w	sl, [sp, #16]
	9102      	str	r1, [sp, #8]
	2100      	movs	r1, #0
	f6c9 71e0 	movt	r1, #40928	@ 0x9fe0
	e9d0 a500 	ldrd	sl, r5, [r0]
	4031      	ands	r1, r6
	9003      	str	r0, [sp, #12]
	f041 5100 	orr.w	r1, r1, #536870912	@ 0x20000000
	f041 0130 	orr.w	r1, r1, #48	@ 0x30
	6081      	str	r1, [r0, #8]
	4650      	mov	r0, sl
	4629      	mov	r1, r5
	f000 f881 	bl	<core::fmt::Formatter::pad_integral::write_prefix>
	b170      	cbz	r0, <core::fmt::Formatter::pad_integral+0xba>
	2001      	movs	r0, #1
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	68eb      	ldr	r3, [r5, #12]
	4620      	mov	r0, r4
	4631      	mov	r1, r6
	4652      	mov	r2, sl
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
	4718      	bx	r3
	eba8 0009 	sub.w	r0, r8, r9
	9601      	str	r6, [sp, #4]
	46a3      	mov	fp, r4
	2600      	movs	r6, #0
	b284      	uxth	r4, r0
	b2b0      	uxth	r0, r6
	42a0      	cmp	r0, r4
	d20b      	bcs.n	<core::fmt::Formatter::pad_integral+0xe4>
	692a      	ldr	r2, [r5, #16]
	4650      	mov	r0, sl
	2130      	movs	r1, #48	@ 0x30
	4790      	blx	r2
	3601      	adds	r6, #1
	2800      	cmp	r0, #0
	d0f5      	beq.n	<core::fmt::Formatter::pad_integral+0xc6>
	2001      	movs	r0, #1
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	9a04      	ldr	r2, [sp, #16]
	4650      	mov	r0, sl
	68eb      	ldr	r3, [r5, #12]
	4659      	mov	r1, fp
	4798      	blx	r3
	b398      	cbz	r0, <core::fmt::Formatter::pad_integral+0x158>
	2001      	movs	r0, #1
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	b299      	uxth	r1, r3
	ea4f 0851 	mov.w	r8, r1, lsr #1
	e9d0 9500 	ldrd	r9, r5, [r0]
	2400      	movs	r4, #0
	9302      	str	r3, [sp, #8]
	fa1f f088 	uxth.w	r0, r8
	b2a1      	uxth	r1, r4
	4281      	cmp	r1, r0
	d20b      	bcs.n	<core::fmt::Formatter::pad_integral+0x12a>
	692a      	ldr	r2, [r5, #16]
	4648      	mov	r0, r9
	4631      	mov	r1, r6
	4790      	blx	r2
	3401      	adds	r4, #1
	2800      	cmp	r0, #0
	d0f3      	beq.n	<core::fmt::Formatter::pad_integral+0x108>
	2001      	movs	r0, #1
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	9b04      	ldr	r3, [sp, #16]
	4648      	mov	r0, r9
	4629      	mov	r1, r5
	465a      	mov	r2, fp
	f000 f833 	bl	<core::fmt::Formatter::pad_integral::write_prefix>
	b120      	cbz	r0, <core::fmt::Formatter::pad_integral+0x142>
	2001      	movs	r0, #1
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	9903      	ldr	r1, [sp, #12]
	4648      	mov	r0, r9
	68eb      	ldr	r3, [r5, #12]
	4652      	mov	r2, sl
	4798      	blx	r3
	b170      	cbz	r0, <core::fmt::Formatter::pad_integral+0x16c>
	2001      	movs	r0, #1
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	9803      	ldr	r0, [sp, #12]
	e9dd 1201 	ldrd	r1, r2, [sp, #4]
	e9c0 1202 	strd	r1, r2, [r0, #8]
	2000      	movs	r0, #0
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	9802      	ldr	r0, [sp, #8]
	2400      	movs	r4, #0
	eba0 0008 	sub.w	r0, r0, r8
	fa1f f880 	uxth.w	r8, r0
	b2a0      	uxth	r0, r4
	4540      	cmp	r0, r8
	d209      	bcs.n	<core::fmt::Formatter::pad_integral+0x192>
	692a      	ldr	r2, [r5, #16]
	4648      	mov	r0, r9
	4631      	mov	r1, r6
	4790      	blx	r2
	3401      	adds	r4, #1
	4601      	mov	r1, r0
	2001      	movs	r0, #1
	2900      	cmp	r1, #0
	d0f3      	beq.n	<core::fmt::Formatter::pad_integral+0x178>
	e769      	b.n	<core::fmt::Formatter::pad_integral+0x66>
	2000      	movs	r0, #0
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<core::fmt::Formatter::pad_integral::write_prefix>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	f84d bd04 	str.w	fp, [sp, #-4]!
	461c      	mov	r4, r3
	460d      	mov	r5, r1
	f5b2 1f88 	cmp.w	r2, #1114112	@ 0x110000
	d00a      	beq.n	<core::fmt::Formatter::pad_integral::write_prefix+0x28>
	692b      	ldr	r3, [r5, #16]
	4611      	mov	r1, r2
	4606      	mov	r6, r0
	4798      	blx	r3
	4601      	mov	r1, r0
	4630      	mov	r0, r6
	b119      	cbz	r1, <core::fmt::Formatter::pad_integral::write_prefix+0x28>
	2001      	movs	r0, #1
	f85d bb04 	ldr.w	fp, [sp], #4
	bdf0      	pop	{r4, r5, r6, r7, pc}
	b13c      	cbz	r4, <core::fmt::Formatter::pad_integral::write_prefix+0x3a>
	68eb      	ldr	r3, [r5, #12]
	4621      	mov	r1, r4
	2200      	movs	r2, #0
	f85d bb04 	ldr.w	fp, [sp], #4
	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
	4718      	bx	r3
	2000      	movs	r0, #0
	f85d bb04 	ldr.w	fp, [sp], #4
	bdf0      	pop	{r4, r5, r6, r7, pc}

<core::option::unwrap_failed>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	4602      	mov	r2, r0
	f24a 6050 	movw	r0, #42576	@ 0xa650
	f6c0 0000 	movt	r0, #2048	@ 0x800
	212b      	movs	r1, #43	@ 0x2b
	f000 f893 	bl	<core::panicking::panic>

<core::slice::index::slice_index_fail>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	4290      	cmp	r0, r2
	d903      	bls.n	<core::slice::index::slice_index_fail+0x10>
	4611      	mov	r1, r2
	461a      	mov	r2, r3
	f000 f811 	bl	<core::slice::index::slice_index_fail::do_panic::runtime>
	4291      	cmp	r1, r2
	d904      	bls.n	<core::slice::index::slice_index_fail+0x1e>
	4608      	mov	r0, r1
	4611      	mov	r1, r2
	461a      	mov	r2, r3
	f000 f820 	bl	<core::slice::index::slice_index_fail::do_panic::runtime>
	4288      	cmp	r0, r1
	d902      	bls.n	<core::slice::index::slice_index_fail+0x28>
	461a      	mov	r2, r3
	f000 f847 	bl	<core::slice::index::slice_index_fail::do_panic::runtime>
	4608      	mov	r0, r1
	4611      	mov	r1, r2
	461a      	mov	r2, r3
	f000 f82c 	bl	<core::slice::index::slice_index_fail::do_panic::runtime>

<core::slice::index::slice_index_fail::do_panic::runtime>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b086      	sub	sp, #24
	e9cd 0100 	strd	r0, r1, [sp]
	f244 0017 	movw	r0, #16407	@ 0x4017
	a901      	add	r1, sp, #4
	f6c0 0000 	movt	r0, #2048	@ 0x800
	9005      	str	r0, [sp, #20]
	e9cd 0103 	strd	r0, r1, [sp, #12]
	4668      	mov	r0, sp
	9002      	str	r0, [sp, #8]
	f24a 10e3 	movw	r0, #41443	@ 0xa1e3
	a902      	add	r1, sp, #8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f000 f842 	bl	<core::panicking::panic_fmt>

<core::slice::index::slice_index_fail::do_panic::runtime>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b086      	sub	sp, #24
	e9cd 0100 	strd	r0, r1, [sp]
	f244 0017 	movw	r0, #16407	@ 0x4017
	a901      	add	r1, sp, #4
	f6c0 0000 	movt	r0, #2048	@ 0x800
	9005      	str	r0, [sp, #20]
	e9cd 0103 	strd	r0, r1, [sp, #12]
	4668      	mov	r0, sp
	9002      	str	r0, [sp, #8]
	f24a 103a 	movw	r0, #41274	@ 0xa13a
	a902      	add	r1, sp, #8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f000 f82c 	bl	<core::panicking::panic_fmt>

<core::slice::index::slice_index_fail::do_panic::runtime>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b086      	sub	sp, #24
	e9cd 0100 	strd	r0, r1, [sp]
	f244 0017 	movw	r0, #16407	@ 0x4017
	a901      	add	r1, sp, #4
	f6c0 0000 	movt	r0, #2048	@ 0x800
	9005      	str	r0, [sp, #20]
	e9cd 0103 	strd	r0, r1, [sp, #12]
	4668      	mov	r0, sp
	9002      	str	r0, [sp, #8]
	f24a 103a 	movw	r0, #41274	@ 0xa13a
	a902      	add	r1, sp, #8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f000 f816 	bl	<core::panicking::panic_fmt>

<core::slice::index::slice_index_fail::do_panic::runtime>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b086      	sub	sp, #24
	e9cd 0100 	strd	r0, r1, [sp]
	f244 0017 	movw	r0, #16407	@ 0x4017
	a901      	add	r1, sp, #4
	f6c0 0000 	movt	r0, #2048	@ 0x800
	9005      	str	r0, [sp, #20]
	e9cd 0103 	strd	r0, r1, [sp, #12]
	4668      	mov	r0, sp
	9002      	str	r0, [sp, #8]
	f649 60a0 	movw	r0, #40608	@ 0x9ea0
	a902      	add	r1, sp, #8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f000 f800 	bl	<core::panicking::panic_fmt>

<core::panicking::panic_fmt>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b086      	sub	sp, #24
	e9cd 0101 	strd	r0, r1, [sp, #4]
	2001      	movs	r0, #1
	f8ad 0014 	strh.w	r0, [sp, #20]
	a801      	add	r0, sp, #4
	9003      	str	r0, [sp, #12]
	a803      	add	r0, sp, #12
	9204      	str	r2, [sp, #16]
	f002 fbfc 	bl	<__rustc::rust_begin_unwind>

<core::panicking::panic_bounds_check>:
	b5fe      	push	{r1, r2, r3, r4, r5, r6, r7, lr}
	af06      	add	r7, sp, #24
	e9cd 0100 	strd	r0, r1, [sp]
	4669      	mov	r1, sp
	4805      	ldr	r0, [pc, #20]	@ (<core::panicking::panic_bounds_check+0x20>)
	9005      	str	r0, [sp, #20]
	e9cd 0103 	strd	r0, r1, [sp, #12]
	a801      	add	r0, sp, #4
	9002      	str	r0, [sp, #8]
	a902      	add	r1, sp, #8
	4802      	ldr	r0, [pc, #8]	@ (<core::panicking::panic_bounds_check+0x24>)
	f7ff ffe3 	bl	<core::panicking::panic_fmt>
	bf00      	nop
	08004017 	.word	0x08004017
	08009fec 	.word	0x08009fec

<core::panicking::panic>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	0049      	lsls	r1, r1, #1
	3101      	adds	r1, #1
	f7ff ffd8 	bl	<core::panicking::panic_fmt>

<<&T as core::fmt::Display>::fmt>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	460b      	mov	r3, r1
	e9d0 1200 	ldrd	r1, r2, [r0]
	4618      	mov	r0, r3
	e8bd 4080 	ldmia.w	sp!, {r7, lr}
	f000 b800 	b.w	<core::fmt::Formatter::pad>

<core::fmt::Formatter::pad>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b08d      	sub	sp, #52	@ 0x34
	f8d0 b008 	ldr.w	fp, [r0, #8]
	4690      	mov	r8, r2
	460c      	mov	r4, r1
	f01b 5fc0 	tst.w	fp, #402653184	@ 0x18000000
	f000 8311 	beq.w	<core::fmt::Formatter::pad+0x63c>
	ea5f 01cb 	movs.w	r1, fp, lsl #3
	d43c      	bmi.n	<core::fmt::Formatter::pad+0x9a>
	f1b8 0f10 	cmp.w	r8, #16
	d25a      	bcs.n	<core::fmt::Formatter::pad+0xdc>
	f1b8 0f00 	cmp.w	r8, #0
	d079      	beq.n	<core::fmt::Formatter::pad+0x120>
	f008 0c03 	and.w	ip, r8, #3
	4681      	mov	r9, r0
	ea5f 0198 	movs.w	r1, r8, lsr #2
	f04f 0200 	mov.w	r2, #0
	f000 82df 	beq.w	<core::fmt::Formatter::pad+0x5fc>
	f001 0103 	and.w	r1, r1, #3
	46a6      	mov	lr, r4
	f06f 0603 	mvn.w	r6, #3
	2300      	movs	r3, #0
	eba2 0a81 	sub.w	sl, r2, r1, lsl #2
	1c62      	adds	r2, r4, #1
	1991      	adds	r1, r2, r6
	3604      	adds	r6, #4
	f991 4003 	ldrsb.w	r4, [r1, #3]
	f991 5006 	ldrsb.w	r5, [r1, #6]
	f991 0005 	ldrsb.w	r0, [r1, #5]
	f114 0f41 	cmn.w	r4, #65	@ 0x41
	f991 1004 	ldrsb.w	r1, [r1, #4]
	bfc8      	it	gt
	3301      	addgt	r3, #1
	f111 0f41 	cmn.w	r1, #65	@ 0x41
	bfc8      	it	gt
	3301      	addgt	r3, #1
	f110 0f41 	cmn.w	r0, #65	@ 0x41
	bfc8      	it	gt
	3301      	addgt	r3, #1
	f115 0f41 	cmn.w	r5, #65	@ 0x41
	eb0a 0006 	add.w	r0, sl, r6
	bfc8      	it	gt
	3301      	addgt	r3, #1
	3004      	adds	r0, #4
	d1e1      	bne.n	<core::fmt::Formatter::pad+0x50>
	1d32      	adds	r2, r6, #4
	4674      	mov	r4, lr
	f1bc 0f00 	cmp.w	ip, #0
	f040 82b6 	bne.w	<core::fmt::Formatter::pad+0x604>
	e2cc      	b.n	<core::fmt::Formatter::pad+0x634>
	f8b0 c00e 	ldrh.w	ip, [r0, #14]
	f1bc 0f00 	cmp.w	ip, #0
	d02c      	beq.n	<core::fmt::Formatter::pad+0xfe>
	eb04 0208 	add.w	r2, r4, r8
	f04f 0800 	mov.w	r8, #0
	4626      	mov	r6, r4
	4661      	mov	r1, ip
	e007      	b.n	<core::fmt::Formatter::pad+0xc2>
	2ef0      	cmp	r6, #240	@ 0xf0
	bf2c      	ite	cs
	1d1e      	addcs	r6, r3, #4
	1cde      	addcc	r6, r3, #3
	1af3      	subs	r3, r6, r3
	3901      	subs	r1, #1
	4498      	add	r8, r3
	d01f      	beq.n	<core::fmt::Formatter::pad+0x102>
	4296      	cmp	r6, r2
	d01e      	beq.n	<core::fmt::Formatter::pad+0x104>
	4633      	mov	r3, r6
	f916 5b01 	ldrsb.w	r5, [r6], #1
	f1b5 3fff 	cmp.w	r5, #4294967295	@ 0xffffffff
	dcf3      	bgt.n	<core::fmt::Formatter::pad+0xba>
	b2ee      	uxtb	r6, r5
	2ee0      	cmp	r6, #224	@ 0xe0
	d2ec      	bcs.n	<core::fmt::Formatter::pad+0xb2>
	1c9e      	adds	r6, r3, #2
	e7ee      	b.n	<core::fmt::Formatter::pad+0xba>
	9000      	str	r0, [sp, #0]
	1ce0      	adds	r0, r4, #3
	4622      	mov	r2, r4
	f020 0403 	bic.w	r4, r0, #3
	ebb4 0e02 	subs.w	lr, r4, r2
	f8cd 800c 	str.w	r8, [sp, #12]
	eba8 080e 	sub.w	r8, r8, lr
	9201      	str	r2, [sp, #4]
	f008 0c03 	and.w	ip, r8, #3
	d10b      	bne.n	<core::fmt::Formatter::pad+0x112>
	2100      	movs	r1, #0
	e057      	b.n	<core::fmt::Formatter::pad+0x1ae>
	f04f 0800 	mov.w	r8, #0
	2100      	movs	r1, #0
	ebac 0301 	sub.w	r3, ip, r1
	8981      	ldrh	r1, [r0, #12]
	428b      	cmp	r3, r1
	f0c0 8232 	bcc.w	<core::fmt::Formatter::pad+0x574>
	e294      	b.n	<core::fmt::Formatter::pad+0x63c>
	1b11      	subs	r1, r2, r4
	f111 0f04 	cmn.w	r1, #4
	d90a      	bls.n	<core::fmt::Formatter::pad+0x130>
	2200      	movs	r2, #0
	2100      	movs	r1, #0
	e02c      	b.n	<core::fmt::Formatter::pad+0x17a>
	2300      	movs	r3, #0
	f04f 0800 	mov.w	r8, #0
	8981      	ldrh	r1, [r0, #12]
	428b      	cmp	r3, r1
	f0c0 8223 	bcc.w	<core::fmt::Formatter::pad+0x574>
	e285      	b.n	<core::fmt::Formatter::pad+0x63c>
	f102 0901 	add.w	r9, r2, #1
	2100      	movs	r1, #0
	f06f 0503 	mvn.w	r5, #3
	eb09 0205 	add.w	r2, r9, r5
	f992 6003 	ldrsb.w	r6, [r2, #3]
	f992 0006 	ldrsb.w	r0, [r2, #6]
	f992 3005 	ldrsb.w	r3, [r2, #5]
	f116 0f41 	cmn.w	r6, #65	@ 0x41
	f992 2004 	ldrsb.w	r2, [r2, #4]
	bfc8      	it	gt
	3101      	addgt	r1, #1
	f112 0f41 	cmn.w	r2, #65	@ 0x41
	bfc8      	it	gt
	3101      	addgt	r1, #1
	f113 0f41 	cmn.w	r3, #65	@ 0x41
	bfc8      	it	gt
	3101      	addgt	r1, #1
	f110 0f41 	cmn.w	r0, #65	@ 0x41
	f105 0004 	add.w	r0, r5, #4
	bfc8      	it	gt
	3101      	addgt	r1, #1
	f115 0208 	adds.w	r2, r5, #8
	4605      	mov	r5, r0
	d1df      	bne.n	<core::fmt::Formatter::pad+0x13a>
	9801      	ldr	r0, [sp, #4]
	5680      	ldrsb	r0, [r0, r2]
	f110 0f41 	cmn.w	r0, #65	@ 0x41
	bfc8      	it	gt
	3101      	addgt	r1, #1
	f1be 0f01 	cmp.w	lr, #1
	d010      	beq.n	<core::fmt::Formatter::pad+0x1ae>
	9801      	ldr	r0, [sp, #4]
	4402      	add	r2, r0
	f992 0001 	ldrsb.w	r0, [r2, #1]
	f110 0f41 	cmn.w	r0, #65	@ 0x41
	bfc8      	it	gt
	3101      	addgt	r1, #1
	f1be 0f02 	cmp.w	lr, #2
	d005      	beq.n	<core::fmt::Formatter::pad+0x1ae>
	f992 0002 	ldrsb.w	r0, [r2, #2]
	f110 0f41 	cmn.w	r0, #65	@ 0x41
	bfc8      	it	gt
	3101      	addgt	r1, #1
	f64f 72fc 	movw	r2, #65532	@ 0xfffc
	ea4f 0e98 	mov.w	lr, r8, lsr #2
	f6c1 72ff 	movt	r2, #8191	@ 0x1fff
	2000      	movs	r0, #0
	f1bc 0f00 	cmp.w	ip, #0
	d01f      	beq.n	<core::fmt::Formatter::pad+0x202>
	f102 42c0 	add.w	r2, r2, #1610612736	@ 0x60000000
	ea02 0208 	and.w	r2, r2, r8
	4422      	add	r2, r4
	f992 3000 	ldrsb.w	r3, [r2]
	f113 0f41 	cmn.w	r3, #65	@ 0x41
	bfc8      	it	gt
	2001      	movgt	r0, #1
	f8dd 800c 	ldr.w	r8, [sp, #12]
	f1bc 0f01 	cmp.w	ip, #1
	d011      	beq.n	<core::fmt::Formatter::pad+0x206>
	f992 3001 	ldrsb.w	r3, [r2, #1]
	f113 0f41 	cmn.w	r3, #65	@ 0x41
	bfc8      	it	gt
	3001      	addgt	r0, #1
	f1bc 0f02 	cmp.w	ip, #2
	d008      	beq.n	<core::fmt::Formatter::pad+0x206>
	f992 2002 	ldrsb.w	r2, [r2, #2]
	f112 0f41 	cmn.w	r2, #65	@ 0x41
	bfc8      	it	gt
	3001      	addgt	r0, #1
	e001      	b.n	<core::fmt::Formatter::pad+0x206>
	f8dd 800c 	ldr.w	r8, [sp, #12]
	1843      	adds	r3, r0, r1
	f8cd b008 	str.w	fp, [sp, #8]
	e014      	b.n	<core::fmt::Formatter::pad+0x238>
	f04f 0900 	mov.w	r9, #0
	fa3f f189 	uxtb16	r1, r9
	fa3f f299 	uxtb16	r2, r9, ror #8
	4411      	add	r1, r2
	9d07      	ldr	r5, [sp, #28]
	9b08      	ldr	r3, [sp, #32]
	eb01 4101 	add.w	r1, r1, r1, lsl #16
	ebae 0e05 	sub.w	lr, lr, r5
	eb0c 0485 	add.w	r4, ip, r5, lsl #2
	f015 0003 	ands.w	r0, r5, #3
	eb03 4311 	add.w	r3, r3, r1, lsr #16
	f040 816c 	bne.w	<core::fmt::Formatter::pad+0x510>
	f1be 0f00 	cmp.w	lr, #0
	f000 8166 	beq.w	<core::fmt::Formatter::pad+0x50c>
	f1be 0fc0 	cmp.w	lr, #192	@ 0xc0
	4671      	mov	r1, lr
	f44f 707c 	mov.w	r0, #1008	@ 0x3f0
	9308      	str	r3, [sp, #32]
	bf28      	it	cs
	21c0      	movcs	r1, #192	@ 0xc0
	ea10 0081 	ands.w	r0, r0, r1, lsl #2
	46a4      	mov	ip, r4
	9107      	str	r1, [sp, #28]
	d0d9      	beq.n	<core::fmt::Formatter::pad+0x20e>
	eb0c 0400 	add.w	r4, ip, r0
	0088      	lsls	r0, r1, #2
	3810      	subs	r0, #16
	2101      	movs	r1, #1
	f10c 0a10 	add.w	sl, ip, #16
	f8cd e018 	str.w	lr, [sp, #24]
	eb01 1110 	add.w	r1, r1, r0, lsr #4
	2830      	cmp	r0, #48	@ 0x30
	e9cd 1c04 	strd	r1, ip, [sp, #16]
	d203      	bcs.n	<core::fmt::Formatter::pad+0x280>
	f04f 0900 	mov.w	r9, #0
	4663      	mov	r3, ip
	e0c5      	b.n	<core::fmt::Formatter::pad+0x40c>
	f64f 70fc 	movw	r0, #65532	@ 0xfffc
	f04f 0900 	mov.w	r9, #0
	f6c1 70ff 	movt	r0, #8191	@ 0x1fff
	ea01 0e00 	and.w	lr, r1, r0
	46d0      	mov	r8, sl
	4666      	mov	r6, ip
	9409      	str	r4, [sp, #36]	@ 0x24
	4645      	mov	r5, r8
	45a0      	cmp	r8, r4
	bf18      	it	ne
	3510      	addne	r5, #16
	462a      	mov	r2, r5
	42a5      	cmp	r5, r4
	bf18      	it	ne
	3210      	addne	r2, #16
	4613      	mov	r3, r2
	42a2      	cmp	r2, r4
	bf18      	it	ne
	3310      	addne	r3, #16
	469a      	mov	sl, r3
	42a3      	cmp	r3, r4
	bf18      	it	ne
	f10a 0a10 	addne.w	sl, sl, #16
	e9d2 4c00 	ldrd	r4, ip, [r2]
	f1be 0e04 	subs.w	lr, lr, #4
	6890      	ldr	r0, [r2, #8]
	900b      	str	r0, [sp, #44]	@ 0x2c
	68d0      	ldr	r0, [r2, #12]
	ea6f 0204 	mvn.w	r2, r4
	ea4f 12d2 	mov.w	r2, r2, lsr #7
	900c      	str	r0, [sp, #48]	@ 0x30
	ea42 1294 	orr.w	r2, r2, r4, lsr #6
	f022 30fe 	bic.w	r0, r2, #4278124286	@ 0xfefefefe
	e9d6 2400 	ldrd	r2, r4, [r6]
	900a      	str	r0, [sp, #40]	@ 0x28
	e9d6 0602 	ldrd	r0, r6, [r6, #8]
	ea6f 0b02 	mvn.w	fp, r2
	ea4f 11db 	mov.w	r1, fp, lsr #7
	ea41 1192 	orr.w	r1, r1, r2, lsr #6
	ea6f 0204 	mvn.w	r2, r4
	ea4f 12d2 	mov.w	r2, r2, lsr #7
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	ea42 1294 	orr.w	r2, r2, r4, lsr #6
	4449      	add	r1, r9
	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
	4411      	add	r1, r2
	ea6f 0200 	mvn.w	r2, r0
	ea4f 12d2 	mov.w	r2, r2, lsr #7
	ea42 1090 	orr.w	r0, r2, r0, lsr #6
	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	ea6f 0106 	mvn.w	r1, r6
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	ea41 1196 	orr.w	r1, r1, r6, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	eb01 0900 	add.w	r9, r1, r0
	e898 0056 	ldmia.w	r8, {r1, r2, r4, r6}
	46d0      	mov	r8, sl
	ea6f 0001 	mvn.w	r0, r1
	ea4f 10d0 	mov.w	r0, r0, lsr #7
	ea40 1091 	orr.w	r0, r0, r1, lsr #6
	ea6f 0102 	mvn.w	r1, r2
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
	ea41 1192 	orr.w	r1, r1, r2, lsr #6
	4448      	add	r0, r9
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	ea6f 0104 	mvn.w	r1, r4
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	ea41 1194 	orr.w	r1, r1, r4, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	ea6f 0106 	mvn.w	r1, r6
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	ea41 1196 	orr.w	r1, r1, r6, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	cd36      	ldmia	r5, {r1, r2, r4, r5}
	ea6f 0601 	mvn.w	r6, r1
	ea4f 16d6 	mov.w	r6, r6, lsr #7
	ea46 1191 	orr.w	r1, r6, r1, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	461e      	mov	r6, r3
	4408      	add	r0, r1
	ea6f 0102 	mvn.w	r1, r2
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	ea41 1192 	orr.w	r1, r1, r2, lsr #6
	9a0b      	ldr	r2, [sp, #44]	@ 0x2c
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	ea6f 0104 	mvn.w	r1, r4
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	ea41 1194 	orr.w	r1, r1, r4, lsr #6
	9c09      	ldr	r4, [sp, #36]	@ 0x24
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	ea6f 0105 	mvn.w	r1, r5
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	ea41 1195 	orr.w	r1, r1, r5, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	990a      	ldr	r1, [sp, #40]	@ 0x28
	4408      	add	r0, r1
	ea6f 010c 	mvn.w	r1, ip
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	ea41 119c 	orr.w	r1, r1, ip, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	ea6f 0102 	mvn.w	r1, r2
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	ea41 1192 	orr.w	r1, r1, r2, lsr #6
	9a0c      	ldr	r2, [sp, #48]	@ 0x30
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	ea6f 0102 	mvn.w	r1, r2
	ea4f 11d1 	mov.w	r1, r1, lsr #7
	ea41 1192 	orr.w	r1, r1, r2, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	eb01 0900 	add.w	r9, r1, r0
	f47f af45 	bne.w	<core::fmt::Formatter::pad+0x296>
	9804      	ldr	r0, [sp, #16]
	e9dd b802 	ldrd	fp, r8, [sp, #8]
	e9dd ce05 	ldrd	ip, lr, [sp, #20]
	f010 0003 	ands.w	r0, r0, #3
	f43f aefa 	beq.w	<core::fmt::Formatter::pad+0x212>
	e893 0046 	ldmia.w	r3, {r1, r2, r6}
	45a2      	cmp	sl, r4
	68db      	ldr	r3, [r3, #12]
	ea6f 0501 	mvn.w	r5, r1
	ea4f 15d5 	mov.w	r5, r5, lsr #7
	ea45 1191 	orr.w	r1, r5, r1, lsr #6
	ea6f 0502 	mvn.w	r5, r2
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	ea4f 15d5 	mov.w	r5, r5, lsr #7
	ea45 1292 	orr.w	r2, r5, r2, lsr #6
	4449      	add	r1, r9
	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
	4411      	add	r1, r2
	ea6f 0206 	mvn.w	r2, r6
	ea4f 12d2 	mov.w	r2, r2, lsr #7
	ea42 1296 	orr.w	r2, r2, r6, lsr #6
	4656      	mov	r6, sl
	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
	bf18      	it	ne
	3610      	addne	r6, #16
	4411      	add	r1, r2
	43da      	mvns	r2, r3
	09d2      	lsrs	r2, r2, #7
	2801      	cmp	r0, #1
	ea42 1293 	orr.w	r2, r2, r3, lsr #6
	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
	eb02 0901 	add.w	r9, r2, r1
	f43f aecd 	beq.w	<core::fmt::Formatter::pad+0x212>
	e89a 002e 	ldmia.w	sl, {r1, r2, r3, r5}
	2802      	cmp	r0, #2
	ea6f 0401 	mvn.w	r4, r1
	ea4f 14d4 	mov.w	r4, r4, lsr #7
	ea44 1191 	orr.w	r1, r4, r1, lsr #6
	ea6f 0402 	mvn.w	r4, r2
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	ea4f 14d4 	mov.w	r4, r4, lsr #7
	ea44 1292 	orr.w	r2, r4, r2, lsr #6
	4449      	add	r1, r9
	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
	4411      	add	r1, r2
	ea6f 0203 	mvn.w	r2, r3
	ea4f 12d2 	mov.w	r2, r2, lsr #7
	ea42 1293 	orr.w	r2, r2, r3, lsr #6
	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
	4411      	add	r1, r2
	ea6f 0205 	mvn.w	r2, r5
	ea4f 12d2 	mov.w	r2, r2, lsr #7
	ea42 1295 	orr.w	r2, r2, r5, lsr #6
	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
	eb02 0901 	add.w	r9, r2, r1
	f43f aea3 	beq.w	<core::fmt::Formatter::pad+0x212>
	e896 000f 	ldmia.w	r6, {r0, r1, r2, r3}
	43c6      	mvns	r6, r0
	09f6      	lsrs	r6, r6, #7
	ea46 1090 	orr.w	r0, r6, r0, lsr #6
	43ce      	mvns	r6, r1
	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
	09f6      	lsrs	r6, r6, #7
	ea46 1191 	orr.w	r1, r6, r1, lsr #6
	4448      	add	r0, r9
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	43d1      	mvns	r1, r2
	09c9      	lsrs	r1, r1, #7
	ea41 1192 	orr.w	r1, r1, r2, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	4408      	add	r0, r1
	43d9      	mvns	r1, r3
	09c9      	lsrs	r1, r1, #7
	ea41 1193 	orr.w	r1, r1, r3, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	eb01 0900 	add.w	r9, r1, r0
	e682      	b.n	<core::fmt::Formatter::pad+0x212>
	9c01      	ldr	r4, [sp, #4]
	e02d      	b.n	<core::fmt::Formatter::pad+0x56c>
	f005 06fc 	and.w	r6, r5, #252	@ 0xfc
	9c01      	ldr	r4, [sp, #4]
	2801      	cmp	r0, #1
	f85c 1026 	ldr.w	r1, [ip, r6, lsl #2]
	ea6f 0201 	mvn.w	r2, r1
	ea4f 12d2 	mov.w	r2, r2, lsr #7
	ea42 1191 	orr.w	r1, r2, r1, lsr #6
	f021 32fe 	bic.w	r2, r1, #4278124286	@ 0xfefefefe
	d015      	beq.n	<core::fmt::Formatter::pad+0x55a>
	eb0c 0586 	add.w	r5, ip, r6, lsl #2
	2802      	cmp	r0, #2
	6869      	ldr	r1, [r5, #4]
	ea6f 0601 	mvn.w	r6, r1
	ea4f 16d6 	mov.w	r6, r6, lsr #7
	ea46 1191 	orr.w	r1, r6, r1, lsr #6
	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
	440a      	add	r2, r1
	d007      	beq.n	<core::fmt::Formatter::pad+0x55a>
	68a8      	ldr	r0, [r5, #8]
	43c1      	mvns	r1, r0
	09c9      	lsrs	r1, r1, #7
	ea41 1090 	orr.w	r0, r1, r0, lsr #6
	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
	4402      	add	r2, r0
	fa3f f082 	uxtb16	r0, r2
	fa3f f192 	uxtb16	r1, r2, ror #8
	4408      	add	r0, r1
	eb00 4000 	add.w	r0, r0, r0, lsl #16
	eb03 4310 	add.w	r3, r3, r0, lsr #16
	9800      	ldr	r0, [sp, #0]
	8981      	ldrh	r1, [r0, #12]
	428b      	cmp	r3, r1
	d263      	bcs.n	<core::fmt::Formatter::pad+0x63c>
	f3cb 7241 	ubfx	r2, fp, #29, #2
	f8cd 800c 	str.w	r8, [sp, #12]
	eba1 0803 	sub.w	r8, r1, r3
	f36f 5b5f 	bfc	fp, #21, #11
	f04f 0900 	mov.w	r9, #0
	46a2      	mov	sl, r4
	e8df f002 	tbb	[pc, r2]
	0208      	.short	0x0208
	0804      	.short	0x0804
	46c1      	mov	r9, r8
	e003      	b.n	<core::fmt::Formatter::pad+0x59e>
	fa1f f188 	uxth.w	r1, r8
	ea4f 0951 	mov.w	r9, r1, lsr #1
	e9d0 5600 	ldrd	r5, r6, [r0]
	2400      	movs	r4, #0
	fa1f f089 	uxth.w	r0, r9
	b2a1      	uxth	r1, r4
	4281      	cmp	r1, r0
	d207      	bcs.n	<core::fmt::Formatter::pad+0x5be>
	6932      	ldr	r2, [r6, #16]
	4628      	mov	r0, r5
	4659      	mov	r1, fp
	4790      	blx	r2
	3401      	adds	r4, #1
	2800      	cmp	r0, #0
	d0f3      	beq.n	<core::fmt::Formatter::pad+0x5a4>
	e014      	b.n	<core::fmt::Formatter::pad+0x5e8>
	9a03      	ldr	r2, [sp, #12]
	4628      	mov	r0, r5
	68f3      	ldr	r3, [r6, #12]
	4651      	mov	r1, sl
	4798      	blx	r3
	b970      	cbnz	r0, <core::fmt::Formatter::pad+0x5e8>
	eba8 0009 	sub.w	r0, r8, r9
	2400      	movs	r4, #0
	fa1f f880 	uxth.w	r8, r0
	b2a0      	uxth	r0, r4
	4540      	cmp	r0, r8
	d20b      	bcs.n	<core::fmt::Formatter::pad+0x5f2>
	6932      	ldr	r2, [r6, #16]
	4628      	mov	r0, r5
	4659      	mov	r1, fp
	4790      	blx	r2
	3401      	adds	r4, #1
	2800      	cmp	r0, #0
	d0f5      	beq.n	<core::fmt::Formatter::pad+0x5d4>
	2001      	movs	r0, #1
	b00d      	add	sp, #52	@ 0x34
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	2000      	movs	r0, #0
	b00d      	add	sp, #52	@ 0x34
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	2300      	movs	r3, #0
	f1bc 0f00 	cmp.w	ip, #0
	d017      	beq.n	<core::fmt::Formatter::pad+0x634>
	56a0      	ldrsb	r0, [r4, r2]
	f110 0f41 	cmn.w	r0, #65	@ 0x41
	bfc8      	it	gt
	3301      	addgt	r3, #1
	f1bc 0f01 	cmp.w	ip, #1
	d00f      	beq.n	<core::fmt::Formatter::pad+0x634>
	4422      	add	r2, r4
	f992 0001 	ldrsb.w	r0, [r2, #1]
	f110 0f41 	cmn.w	r0, #65	@ 0x41
	bfc8      	it	gt
	3301      	addgt	r3, #1
	f1bc 0f02 	cmp.w	ip, #2
	d005      	beq.n	<core::fmt::Formatter::pad+0x634>
	f992 0002 	ldrsb.w	r0, [r2, #2]
	f110 0f41 	cmn.w	r0, #65	@ 0x41
	bfc8      	it	gt
	3301      	addgt	r3, #1
	4648      	mov	r0, r9
	8981      	ldrh	r1, [r0, #12]
	428b      	cmp	r3, r1
	d39b      	bcc.n	<core::fmt::Formatter::pad+0x574>
	e9d0 0100 	ldrd	r0, r1, [r0]
	4642      	mov	r2, r8
	68cb      	ldr	r3, [r1, #12]
	4621      	mov	r1, r4
	b00d      	add	sp, #52	@ 0x34
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
	4718      	bx	r3

<core::panicking::panic_const::panic_const_div_by_zero>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	4602      	mov	r2, r0
	f24a 607b 	movw	r0, #42619	@ 0xa67b
	f6c0 0000 	movt	r0, #2048	@ 0x800
	2133      	movs	r1, #51	@ 0x33
	f7ff fc9b 	bl	<core::panicking::panic_fmt>

<<&T as core::fmt::Debug>::fmt>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	e9d0 0200 	ldrd	r0, r2, [r0]
	68d2      	ldr	r2, [r2, #12]
	e8bd 4080 	ldmia.w	sp!, {r7, lr}
	4710      	bx	r2

<core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b082      	sub	sp, #8
	7802      	ldrb	r2, [r0, #0]
	f24a 5c88 	movw	ip, #42376	@ 0xa588
	460b      	mov	r3, r1
	f6c0 0c00 	movt	ip, #2048	@ 0x800
	2a0a      	cmp	r2, #10
	d30d      	bcc.n	<core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x32>
	2029      	movs	r0, #41	@ 0x29
	4350      	muls	r0, r2
	0b01      	lsrs	r1, r0, #12
	2064      	movs	r0, #100	@ 0x64
	fb01 2010 	mls	r0, r1, r0, r2
	b2c0      	uxtb	r0, r0
	f83c 0010 	ldrh.w	r0, [ip, r0, lsl #1]
	f827 0c02 	strh.w	r0, [r7, #-2]
	2001      	movs	r0, #1
	b91a      	cbnz	r2, <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x38>
	e003      	b.n	<core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x3a>
	2003      	movs	r0, #3
	4611      	mov	r1, r2
	b102      	cbz	r2, <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x3a>
	b139      	cbz	r1, <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x4a>
	f001 017f 	and.w	r1, r1, #127	@ 0x7f
	3801      	subs	r0, #1
	1efa      	subs	r2, r7, #3
	eb0c 0141 	add.w	r1, ip, r1, lsl #1
	7849      	ldrb	r1, [r1, #1]
	5411      	strb	r1, [r2, r0]
	1ef9      	subs	r1, r7, #3
	f1c0 0203 	rsb	r2, r0, #3
	4401      	add	r1, r0
	4618      	mov	r0, r3
	f7ff fafd 	bl	<core::fmt::Formatter::pad_integral>
	b002      	add	sp, #8
	bd80      	pop	{r7, pc}

<core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	f84d bd04 	str.w	fp, [sp, #-4]!
	b084      	sub	sp, #16
	6803      	ldr	r3, [r0, #0]
	f24a 5e88 	movw	lr, #42376	@ 0xa588
	468c      	mov	ip, r1
	f6c0 0e00 	movt	lr, #2048	@ 0x800
	08d8      	lsrs	r0, r3, #3
	287c      	cmp	r0, #124	@ 0x7c
	d943      	bls.n	<core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xa4>
	f241 7059 	movw	r0, #5977	@ 0x1759
	f2cd 10b7 	movt	r0, #53687	@ 0xd1b7
	fba3 0100 	umull	r0, r1, r3, r0
	f242 7010 	movw	r0, #10000	@ 0x2710
	0b49      	lsrs	r1, r1, #13
	fb01 3510 	mls	r5, r1, r0, r3
	b2aa      	uxth	r2, r5
	0894      	lsrs	r4, r2, #2
	f241 427b 	movw	r2, #5243	@ 0x147b
	4354      	muls	r4, r2
	0c66      	lsrs	r6, r4, #17
	2464      	movs	r4, #100	@ 0x64
	fb06 5514 	mls	r5, r6, r4, r5
	f83e 6016 	ldrh.w	r6, [lr, r6, lsl #1]
	f827 6c14 	strh.w	r6, [r7, #-20]
	b2ad      	uxth	r5, r5
	f83e 5015 	ldrh.w	r5, [lr, r5, lsl #1]
	f827 5c12 	strh.w	r5, [r7, #-18]
	f249 657f 	movw	r5, #38527	@ 0x967f
	f2c0 0598 	movt	r5, #152	@ 0x98
	42ab      	cmp	r3, r5
	d936      	bls.n	<core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xd0>
	f648 55b9 	movw	r5, #36281	@ 0x8db9
	f2c0 0506 	movt	r5, #6
	fba1 5605 	umull	r5, r6, r1, r5
	fb06 1010 	mls	r0, r6, r0, r1
	fb00 f102 	mul.w	r1, r0, r2
	f643 3289 	movw	r2, #15241	@ 0x3b89
	f2c5 52e6 	movt	r2, #21990	@ 0x55e6
	0cc9      	lsrs	r1, r1, #19
	fb01 0014 	mls	r0, r1, r4, r0
	f83e 1011 	ldrh.w	r1, [lr, r1, lsl #1]
	fba3 2402 	umull	r2, r4, r3, r2
	f827 1c18 	strh.w	r1, [r7, #-24]
	2202      	movs	r2, #2
	b280      	uxth	r0, r0
	f83e 0010 	ldrh.w	r0, [lr, r0, lsl #1]
	0e61      	lsrs	r1, r4, #25
	f827 0c16 	strh.w	r0, [r7, #-22]
	2909      	cmp	r1, #9
	d804      	bhi.n	<core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xac>
	e018      	b.n	<core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xd6>
	220a      	movs	r2, #10
	4619      	mov	r1, r3
	2909      	cmp	r1, #9
	d914      	bls.n	<core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xd6>
	b288      	uxth	r0, r1
	f241 447b 	movw	r4, #5243	@ 0x147b
	0880      	lsrs	r0, r0, #2
	3a02      	subs	r2, #2
	4360      	muls	r0, r4
	2464      	movs	r4, #100	@ 0x64
	0c40      	lsrs	r0, r0, #17
	fb00 1114 	mls	r1, r0, r4, r1
	f1a7 041a 	sub.w	r4, r7, #26
	b289      	uxth	r1, r1
	f83e 1011 	ldrh.w	r1, [lr, r1, lsl #1]
	52a1      	strh	r1, [r4, r2]
	b92b      	cbnz	r3, <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xda>
	e005      	b.n	<core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xdc>
	2206      	movs	r2, #6
	2909      	cmp	r1, #9
	d8ea      	bhi.n	<core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xac>
	4608      	mov	r0, r1
	b103      	cbz	r3, <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xdc>
	b130      	cbz	r0, <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xea>
	eb0e 0040 	add.w	r0, lr, r0, lsl #1
	3a01      	subs	r2, #1
	f1a7 011a 	sub.w	r1, r7, #26
	7840      	ldrb	r0, [r0, #1]
	5488      	strb	r0, [r1, r2]
	f1a7 001a 	sub.w	r0, r7, #26
	1881      	adds	r1, r0, r2
	f1c2 020a 	rsb	r2, r2, #10
	4660      	mov	r0, ip
	f7ff fa7e 	bl	<core::fmt::Formatter::pad_integral>
	b004      	add	sp, #16
	f85d bb04 	ldr.w	fp, [sp], #4
	bdf0      	pop	{r4, r5, r6, r7, pc}

<core::slice::copy_from_slice_impl::len_mismatch_fail>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	4603      	mov	r3, r0
	4608      	mov	r0, r1
	4619      	mov	r1, r3
	f000 f800 	bl	<core::slice::copy_from_slice_impl::len_mismatch_fail::do_panic::runtime>

<core::slice::copy_from_slice_impl::len_mismatch_fail::do_panic::runtime>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b086      	sub	sp, #24
	e9cd 0100 	strd	r0, r1, [sp]
	f244 0017 	movw	r0, #16407	@ 0x4017
	a901      	add	r1, sp, #4
	f6c0 0000 	movt	r0, #2048	@ 0x800
	9005      	str	r0, [sp, #20]
	e9cd 0103 	strd	r0, r1, [sp, #12]
	4668      	mov	r0, sp
	9002      	str	r0, [sp, #8]
	f649 5022 	movw	r0, #40226	@ 0x9d22
	a902      	add	r1, sp, #8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7ff fbc7 	bl	<core::panicking::panic_fmt>

<core::option::expect_failed>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b084      	sub	sp, #16
	e9cd 0100 	strd	r0, r1, [sp]
	f643 1031 	movw	r0, #14641	@ 0x3931
	f6c0 0000 	movt	r0, #2048	@ 0x800
	a902      	add	r1, sp, #8
	9003      	str	r0, [sp, #12]
	4668      	mov	r0, sp
	9002      	str	r0, [sp, #8]
	f649 50d9 	movw	r0, #40409	@ 0x9dd9
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7ff fbb4 	bl	<core::panicking::panic_fmt>

<core::result::unwrap_failed>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b088      	sub	sp, #32
	469c      	mov	ip, r3
	232b      	movs	r3, #43	@ 0x2b
	e9cd 0300 	strd	r0, r3, [sp]
	f643 70ab 	movw	r0, #16299	@ 0x3fab
	f6c0 0000 	movt	r0, #2048	@ 0x800
	e9cd 1202 	strd	r1, r2, [sp, #8]
	9007      	str	r0, [sp, #28]
	a802      	add	r0, sp, #8
	9006      	str	r0, [sp, #24]
	f643 1031 	movw	r0, #14641	@ 0x3931
	f6c0 0000 	movt	r0, #2048	@ 0x800
	a904      	add	r1, sp, #16
	9005      	str	r0, [sp, #20]
	4668      	mov	r0, sp
	9004      	str	r0, [sp, #16]
	f649 10eb 	movw	r0, #39403	@ 0x99eb
	f6c0 0000 	movt	r0, #2048	@ 0x800
	4662      	mov	r2, ip
	f7ff fb95 	bl	<core::panicking::panic_fmt>

<core::panicking::panic_const::panic_const_rem_by_zero>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	4602      	mov	r2, r0
	f24a 6094 	movw	r0, #42644	@ 0xa694
	f6c0 0000 	movt	r0, #2048	@ 0x800
	2173      	movs	r1, #115	@ 0x73
	f7ff fb8b 	bl	<core::panicking::panic_fmt>

<<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt>:
	b5b0      	push	{r4, r5, r7, lr}
	af02      	add	r7, sp, #8
	b090      	sub	sp, #64	@ 0x40
	460c      	mov	r4, r1
	7ac9      	ldrb	r1, [r1, #11]
	6800      	ldr	r0, [r0, #0]
	f011 0f18 	tst.w	r1, #24
	9001      	str	r0, [sp, #4]
	d028      	beq.n	<<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt+0x66>
	2000      	movs	r0, #0
	a901      	add	r1, sp, #4
	9002      	str	r0, [sp, #8]
	f643 70bb 	movw	r0, #16315	@ 0x3fbb
	1cca      	adds	r2, r1, #3
	f6c0 0000 	movt	r0, #2048	@ 0x800
	ad02      	add	r5, sp, #8
	e9cd 1007 	strd	r1, r0, [sp, #28]
	e9cd 020c 	strd	r0, r2, [sp, #48]	@ 0x30
	1c8a      	adds	r2, r1, #2
	ab07      	add	r3, sp, #28
	900e      	str	r0, [sp, #56]	@ 0x38
	e9cd 020a 	strd	r0, r2, [sp, #40]	@ 0x28
	1c4a      	adds	r2, r1, #1
	f24a 61d4 	movw	r1, #42708	@ 0xa6d4
	9209      	str	r2, [sp, #36]	@ 0x24
	f24a 1271 	movw	r2, #41329	@ 0xa171
	f6c0 0100 	movt	r1, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	4628      	mov	r0, r5
	f7ff f972 	bl	<core::fmt::write>
	bb60      	cbnz	r0, <<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt+0xae>
	9a02      	ldr	r2, [sp, #8]
	2a10      	cmp	r2, #16
	d220      	bcs.n	<<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt+0x9c>
	1d29      	adds	r1, r5, #4
	4620      	mov	r0, r4
	f7ff fb8c 	bl	<core::fmt::Formatter::pad>
	b010      	add	sp, #64	@ 0x40
	bdb0      	pop	{r4, r5, r7, pc}
	ab01      	add	r3, sp, #4
	f643 72bb 	movw	r2, #16315	@ 0x3fbb
	1cdd      	adds	r5, r3, #3
	f6c0 0200 	movt	r2, #2048	@ 0x800
	e9d4 0100 	ldrd	r0, r1, [r4]
	e9cd 250c 	strd	r2, r5, [sp, #48]	@ 0x30
	1c9d      	adds	r5, r3, #2
	920e      	str	r2, [sp, #56]	@ 0x38
	e9cd 250a 	strd	r2, r5, [sp, #40]	@ 0x28
	1c5d      	adds	r5, r3, #1
	e9cd 3207 	strd	r3, r2, [sp, #28]
	f24a 1271 	movw	r2, #41329	@ 0xa171
	ab07      	add	r3, sp, #28
	f6c0 0200 	movt	r2, #2048	@ 0x800
	9509      	str	r5, [sp, #36]	@ 0x24
	f7ff f94f 	bl	<core::fmt::write>
	b010      	add	sp, #64	@ 0x40
	bdb0      	pop	{r4, r5, r7, pc}
	f24a 63ec 	movw	r3, #42732	@ 0xa6ec
	4611      	mov	r1, r2
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2000      	movs	r0, #0
	220f      	movs	r2, #15
	f7ff fac3 	bl	<core::slice::index::slice_index_fail>
	f64a 1010 	movw	r0, #43280	@ 0xa910
	f24a 62fc 	movw	r2, #42748	@ 0xa6fc
	f24a 730c 	movw	r3, #42764	@ 0xa70c
	f1a7 0109 	sub.w	r1, r7, #9
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f7ff ff70 	bl	<core::result::unwrap_failed>

<<core::fmt::Error as core::fmt::Debug>::fmt>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	e9d1 0200 	ldrd	r0, r2, [r1]
	f24a 61cd 	movw	r1, #42701	@ 0xa6cd
	68d3      	ldr	r3, [r2, #12]
	f6c0 0100 	movt	r1, #2048	@ 0x800
	2205      	movs	r2, #5
	e8bd 4080 	ldmia.w	sp!, {r7, lr}
	4718      	bx	r3

<<core::net::display_buffer::DisplayBuffer<_> as core::fmt::Write>::write_str>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	f84d bd04 	str.w	fp, [sp, #-4]!
	6803      	ldr	r3, [r0, #0]
	2400      	movs	r4, #0
	189e      	adds	r6, r3, r2
	f144 0500 	adc.w	r5, r4, #0
	2e0f      	cmp	r6, #15
	bf88      	it	hi
	2401      	movhi	r4, #1
	432c      	orrs	r4, r5
	d106      	bne.n	<<core::net::display_buffer::DisplayBuffer<_> as core::fmt::Write>::write_str+0x2a>
	4403      	add	r3, r0
	4605      	mov	r5, r0
	3304      	adds	r3, #4
	4618      	mov	r0, r3
	f005 f891 	bl	<__aeabi_memcpy>
	602e      	str	r6, [r5, #0]
	4620      	mov	r0, r4
	f85d bb04 	ldr.w	fp, [sp], #4
	bdf0      	pop	{r4, r5, r6, r7, pc}

<core::fmt::Write::write_char>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	f84d bd04 	str.w	fp, [sp, #-4]!
	b082      	sub	sp, #8
	2200      	movs	r2, #0
	2980      	cmp	r1, #128	@ 0x80
	9201      	str	r2, [sp, #4]
	d203      	bcs.n	<core::fmt::Write::write_char+0x1a>
	f88d 1004 	strb.w	r1, [sp, #4]
	2201      	movs	r2, #1
	e02f      	b.n	<core::fmt::Write::write_char+0x7a>
	f64f 7cfe 	movw	ip, #65534	@ 0xfffe
	460b      	mov	r3, r1
	f2c0 3cff 	movt	ip, #1023	@ 0x3ff
	098a      	lsrs	r2, r1, #6
	f36c 139f 	bfi	r3, ip, #6, #26
	f5b1 6f00 	cmp.w	r1, #2048	@ 0x800
	d207      	bcs.n	<core::fmt::Write::write_char+0x40>
	f042 01c0 	orr.w	r1, r2, #192	@ 0xc0
	f88d 3005 	strb.w	r3, [sp, #5]
	f88d 1004 	strb.w	r1, [sp, #4]
	2202      	movs	r2, #2
	e01c      	b.n	<core::fmt::Write::write_char+0x7a>
	f36c 129f 	bfi	r2, ip, #6, #26
	0b0c      	lsrs	r4, r1, #12
	f5b1 3f80 	cmp.w	r1, #65536	@ 0x10000
	d207      	bcs.n	<core::fmt::Write::write_char+0x5c>
	f88d 2005 	strb.w	r2, [sp, #5]
	2203      	movs	r2, #3
	f044 01e0 	orr.w	r1, r4, #224	@ 0xe0
	f88d 3006 	strb.w	r3, [sp, #6]
	e00c      	b.n	<core::fmt::Write::write_char+0x76>
	f06f 050f 	mvn.w	r5, #15
	ea45 4191 	orr.w	r1, r5, r1, lsr #18
	f36c 149f 	bfi	r4, ip, #6, #26
	f88d 2006 	strb.w	r2, [sp, #6]
	f88d 3007 	strb.w	r3, [sp, #7]
	2204      	movs	r2, #4
	f88d 4005 	strb.w	r4, [sp, #5]
	f88d 1004 	strb.w	r1, [sp, #4]
	6801      	ldr	r1, [r0, #0]
	2300      	movs	r3, #0
	188d      	adds	r5, r1, r2
	f143 0400 	adc.w	r4, r3, #0
	2d0f      	cmp	r5, #15
	bf88      	it	hi
	2301      	movhi	r3, #1
	431c      	orrs	r4, r3
	d107      	bne.n	<core::fmt::Write::write_char+0x9e>
	4401      	add	r1, r0
	4606      	mov	r6, r0
	1d0b      	adds	r3, r1, #4
	a901      	add	r1, sp, #4
	4618      	mov	r0, r3
	f005 f83e 	bl	<__aeabi_memcpy>
	6035      	str	r5, [r6, #0]
	4620      	mov	r0, r4
	b002      	add	sp, #8
	f85d bb04 	ldr.w	fp, [sp], #4
	bdf0      	pop	{r4, r5, r6, r7, pc}

<core::fmt::Write::write_fmt>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	4613      	mov	r3, r2
	460a      	mov	r2, r1
	f24a 61d4 	movw	r1, #42708	@ 0xa6d4
	f6c0 0100 	movt	r1, #2048	@ 0x800
	e8bd 4080 	ldmia.w	sp!, {r7, lr}
	f7ff b8ae 	b.w	<core::fmt::write>

<DefaultHandler_>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	e7fe      	b.n	<DefaultHandler_+0x4>

<DefaultPreInit>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	bd80      	pop	{r7, pc}

<defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
	b086      	sub	sp, #24
	7805      	ldrb	r5, [r0, #0]
	2102      	movs	r1, #2
	7844      	ldrb	r4, [r0, #1]
	f890 9002 	ldrb.w	r9, [r0, #2]
	f890 8003 	ldrb.w	r8, [r0, #3]
	f240 0032 	movw	r0, #50	@ 0x32
	f2c0 0000 	movt	r0, #0
	f827 0c2a 	strh.w	r0, [r7, #-42]
	f1a7 002a 	sub.w	r0, r7, #42	@ 0x2a
	f000 f9ee 	bl	<_defmt_write>
	f240 0602 	movw	r6, #2
	a802      	add	r0, sp, #8
	f2c0 0600 	movt	r6, #0
	2102      	movs	r1, #2
	f8ad 6008 	strh.w	r6, [sp, #8]
	f000 f9e4 	bl	<_defmt_write>
	f1a7 0025 	sub.w	r0, r7, #37	@ 0x25
	2101      	movs	r1, #1
	f807 5c25 	strb.w	r5, [r7, #-37]
	f000 f9dd 	bl	<_defmt_write>
	a803      	add	r0, sp, #12
	2102      	movs	r1, #2
	f8ad 600c 	strh.w	r6, [sp, #12]
	f000 f9d7 	bl	<_defmt_write>
	f1a7 0021 	sub.w	r0, r7, #33	@ 0x21
	2101      	movs	r1, #1
	f807 4c21 	strb.w	r4, [r7, #-33]
	f000 f9d0 	bl	<_defmt_write>
	a804      	add	r0, sp, #16
	2102      	movs	r1, #2
	f8ad 6010 	strh.w	r6, [sp, #16]
	f000 f9ca 	bl	<_defmt_write>
	f1a7 001d 	sub.w	r0, r7, #29
	2101      	movs	r1, #1
	f807 9c1d 	strb.w	r9, [r7, #-29]
	f000 f9c3 	bl	<_defmt_write>
	a805      	add	r0, sp, #20
	2102      	movs	r1, #2
	f8ad 6014 	strh.w	r6, [sp, #20]
	f000 f9bd 	bl	<_defmt_write>
	f1a7 0019 	sub.w	r0, r7, #25
	2101      	movs	r1, #1
	f807 8c19 	strb.w	r8, [r7, #-25]
	f000 f9b6 	bl	<_defmt_write>
	b006      	add	sp, #24
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<defmt::export::acquire_and_header>:
	b5d0      	push	{r4, r6, r7, lr}
	af02      	add	r7, sp, #8
	b082      	sub	sp, #8
	4604      	mov	r4, r0
	f000 f8e5 	bl	<_defmt_acquire>
	f1a7 000a 	sub.w	r0, r7, #10
	2102      	movs	r1, #2
	f827 4c0a 	strh.w	r4, [r7, #-10]
	f000 f9a5 	bl	<_defmt_write>
	f7ff ff9c 	bl	<DefaultPreInit>
	b002      	add	sp, #8
	bdd0      	pop	{r4, r6, r7, pc}

<defmt::export::acquire_header_and_release>:
	b5d0      	push	{r4, r6, r7, lr}
	af02      	add	r7, sp, #8
	b082      	sub	sp, #8
	4604      	mov	r4, r0
	f000 f8d4 	bl	<_defmt_acquire>
	f1a7 000a 	sub.w	r0, r7, #10
	2102      	movs	r1, #2
	f827 4c0a 	strh.w	r4, [r7, #-10]
	f000 f994 	bl	<_defmt_write>
	f7ff ff8b 	bl	<DefaultPreInit>
	f000 f918 	bl	<_defmt_release>
	b002      	add	sp, #8
	bdd0      	pop	{r4, r6, r7, pc}

<<defmt::export::FmtWrite as core::fmt::Write>::write_str>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	4608      	mov	r0, r1
	4611      	mov	r1, r2
	f000 f988 	bl	<_defmt_write>
	2000      	movs	r0, #0
	bd80      	pop	{r7, pc}

<core::fmt::Write::write_char>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b082      	sub	sp, #8
	2000      	movs	r0, #0
	2980      	cmp	r1, #128	@ 0x80
	9001      	str	r0, [sp, #4]
	d203      	bcs.n	<core::fmt::Write::write_char+0x16>
	f88d 1004 	strb.w	r1, [sp, #4]
	2101      	movs	r1, #1
	e032      	b.n	<core::fmt::Write::write_char+0x7c>
	f64f 7cfe 	movw	ip, #65534	@ 0xfffe
	460a      	mov	r2, r1
	f2c0 3cff 	movt	ip, #1023	@ 0x3ff
	0988      	lsrs	r0, r1, #6
	f36c 129f 	bfi	r2, ip, #6, #26
	f5b1 6f00 	cmp.w	r1, #2048	@ 0x800
	d207      	bcs.n	<core::fmt::Write::write_char+0x3c>
	f040 00c0 	orr.w	r0, r0, #192	@ 0xc0
	f88d 2005 	strb.w	r2, [sp, #5]
	f88d 0004 	strb.w	r0, [sp, #4]
	2102      	movs	r1, #2
	e01f      	b.n	<core::fmt::Write::write_char+0x7c>
	f36c 109f 	bfi	r0, ip, #6, #26
	ea4f 3e11 	mov.w	lr, r1, lsr #12
	f5b1 3f80 	cmp.w	r1, #65536	@ 0x10000
	d209      	bcs.n	<core::fmt::Write::write_char+0x5e>
	2103      	movs	r1, #3
	f88d 0005 	strb.w	r0, [sp, #5]
	f04e 00e0 	orr.w	r0, lr, #224	@ 0xe0
	f88d 2006 	strb.w	r2, [sp, #6]
	f88d 0004 	strb.w	r0, [sp, #4]
	e00e      	b.n	<core::fmt::Write::write_char+0x7c>
	f06f 030f 	mvn.w	r3, #15
	ea43 4191 	orr.w	r1, r3, r1, lsr #18
	f36c 1e9f 	bfi	lr, ip, #6, #26
	f88d 1004 	strb.w	r1, [sp, #4]
	f88d 2007 	strb.w	r2, [sp, #7]
	2104      	movs	r1, #4
	f88d 0006 	strb.w	r0, [sp, #6]
	f88d e005 	strb.w	lr, [sp, #5]
	a801      	add	r0, sp, #4
	f000 f945 	bl	<_defmt_write>
	2000      	movs	r0, #0
	b002      	add	sp, #8
	bd80      	pop	{r7, pc}

<core::fmt::Write::write_fmt>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	4613      	mov	r3, r2
	460a      	mov	r2, r1
	f24a 712c 	movw	r1, #42796	@ 0xa72c
	f6c0 0100 	movt	r1, #2048	@ 0x800
	e8bd 4080 	ldmia.w	sp!, {r7, lr}
	f7fe bfda 	b.w	<core::fmt::write>

<defmt_rtt::channel::Channel::blocking_write>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
	b152      	cbz	r2, <defmt_rtt::channel::Channel::blocking_write+0x20>
	4605      	mov	r5, r0
	4614      	mov	r4, r2
	6900      	ldr	r0, [r0, #16]
	68ea      	ldr	r2, [r5, #12]
	f3bf 8f5f 	dmb	sy
	4290      	cmp	r0, r2
	d907      	bls.n	<defmt_rtt::channel::Channel::blocking_write+0x2a>
	43d3      	mvns	r3, r2
	4418      	add	r0, r3
	b968      	cbnz	r0, <defmt_rtt::channel::Channel::blocking_write+0x3c>
	2400      	movs	r4, #0
	4620      	mov	r0, r4
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	2800      	cmp	r0, #0
	bf06      	itte	eq
	f240 30ff 	movweq	r0, #1023	@ 0x3ff
	1a80      	subeq	r0, r0, r2
	f5c2 6080 	rsbne	r0, r2, #1024	@ 0x400
	2800      	cmp	r0, #0
	d0f1      	beq.n	<defmt_rtt::channel::Channel::blocking_write+0x20>
	42a0      	cmp	r0, r4
	bf38      	it	cc
	4604      	movcc	r4, r0
	18a6      	adds	r6, r4, r2
	f5b6 6f80 	cmp.w	r6, #1024	@ 0x400
	d90d      	bls.n	<defmt_rtt::channel::Channel::blocking_write+0x66>
	6868      	ldr	r0, [r5, #4]
	f5c2 6980 	rsb	r9, r2, #1024	@ 0x400
	4688      	mov	r8, r1
	4410      	add	r0, r2
	464a      	mov	r2, r9
	f004 ff2b 	bl	<__aeabi_memcpy>
	6868      	ldr	r0, [r5, #4]
	eb08 0109 	add.w	r1, r8, r9
	eba4 0209 	sub.w	r2, r4, r9
	e002      	b.n	<defmt_rtt::channel::Channel::blocking_write+0x6c>
	6868      	ldr	r0, [r5, #4]
	4410      	add	r0, r2
	4622      	mov	r2, r4
	f004 ff20 	bl	<__aeabi_memcpy>
	f36f 269f 	bfc	r6, #10, #22
	f3bf 8f5f 	dmb	sy
	60ee      	str	r6, [r5, #12]
	4620      	mov	r0, r4
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<defmt_rtt::channel::Channel::nonblocking_write>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
	4614      	mov	r4, r2
	68c2      	ldr	r2, [r0, #12]
	f5b4 6f80 	cmp.w	r4, #1024	@ 0x400
	bf28      	it	cs
	f44f 6480 	movcs.w	r4, #1024	@ 0x400
	1916      	adds	r6, r2, r4
	4605      	mov	r5, r0
	f5b6 6f80 	cmp.w	r6, #1024	@ 0x400
	f3bf 8f5f 	dmb	sy
	d90d      	bls.n	<defmt_rtt::channel::Channel::nonblocking_write+0x40>
	6868      	ldr	r0, [r5, #4]
	f5c2 6980 	rsb	r9, r2, #1024	@ 0x400
	4688      	mov	r8, r1
	4410      	add	r0, r2
	464a      	mov	r2, r9
	f004 fefd 	bl	<__aeabi_memcpy>
	6868      	ldr	r0, [r5, #4]
	eb08 0109 	add.w	r1, r8, r9
	eba4 0209 	sub.w	r2, r4, r9
	e002      	b.n	<defmt_rtt::channel::Channel::nonblocking_write+0x46>
	6868      	ldr	r0, [r5, #4]
	4410      	add	r0, r2
	4622      	mov	r2, r4
	f004 fef2 	bl	<__aeabi_memcpy>
	f3bf 8f5f 	dmb	sy
	f36f 269f 	bfc	r6, #10, #22
	60ee      	str	r6, [r5, #12]
	4620      	mov	r0, r4
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<_defmt_acquire>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	f84d bd04 	str.w	fp, [sp, #-4]!
	b082      	sub	sp, #8
	f004 f95c 	bl	<__primask_r>
	4604      	mov	r4, r0
	f004 f94c 	bl	<__cpsid>
	f240 0040 	movw	r0, #64	@ 0x40
	f2c2 0000 	movt	r0, #8192	@ 0x2000
	7841      	ldrb	r1, [r0, #1]
	bb59      	cbnz	r1, <_defmt_acquire+0x78>
	2101      	movs	r1, #1
	ea21 0304 	bic.w	r3, r1, r4
	7041      	strb	r1, [r0, #1]
	7882      	ldrb	r2, [r0, #2]
	7003      	strb	r3, [r0, #0]
	bb02      	cbnz	r2, <_defmt_acquire+0x70>
	7081      	strb	r1, [r0, #2]
	f240 0508 	movw	r5, #8
	2000      	movs	r0, #0
	f2c2 0500 	movt	r5, #8192	@ 0x2000
	f807 0c11 	strb.w	r0, [r7, #-17]
	f244 56cf 	movw	r6, #17871	@ 0x45cf
	6ae8      	ldr	r0, [r5, #44]	@ 0x2c
	f1a7 0411 	sub.w	r4, r7, #17
	f244 514d 	movw	r1, #17741	@ 0x454d
	f6c0 0600 	movt	r6, #2048	@ 0x800
	f000 0003 	and.w	r0, r0, #3
	f6c0 0100 	movt	r1, #2048	@ 0x800
	2802      	cmp	r0, #2
	bf08      	it	eq
	460e      	moveq	r6, r1
	f105 0018 	add.w	r0, r5, #24
	4621      	mov	r1, r4
	2201      	movs	r2, #1
	47b0      	blx	r6
	2800      	cmp	r0, #0
	d0f8      	beq.n	<_defmt_acquire+0x5e>
	2801      	cmp	r0, #1
	d10e      	bne.n	<_defmt_acquire+0x8e>
	b002      	add	sp, #8
	f85d bb04 	ldr.w	fp, [sp], #4
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f24a 7045 	movw	r0, #42821	@ 0xa745
	f24a 7264 	movw	r2, #42852	@ 0xa764
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	213d      	movs	r1, #61	@ 0x3d
	f7ff f914 	bl	<core::panicking::panic_fmt>
	f24a 7374 	movw	r3, #42868	@ 0xa774
	2101      	movs	r1, #1
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2201      	movs	r2, #1
	f7ff f89b 	bl	<core::slice::index::slice_index_fail>

<_defmt_release>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0700 	stmdb	sp!, {r8, r9, sl}
	b082      	sub	sp, #8
	f240 0840 	movw	r8, #64	@ 0x40
	f240 0608 	movw	r6, #8
	f2c2 0800 	movt	r8, #8192	@ 0x2000
	f244 5a4d 	movw	sl, #17741	@ 0x454d
	f898 0003 	ldrb.w	r0, [r8, #3]
	f244 55cf 	movw	r5, #17871	@ 0x45cf
	f2c2 0600 	movt	r6, #8192	@ 0x2000
	f6c0 0a00 	movt	sl, #2048	@ 0x800
	f6c0 0500 	movt	r5, #2048	@ 0x800
	b3a0      	cbz	r0, <_defmt_release+0x9a>
	2807      	cmp	r0, #7
	d21b      	bcs.n	<_defmt_release+0x6c>
	f04f 32ff 	mov.w	r2, #4294967295	@ 0xffffffff
	f898 1004 	ldrb.w	r1, [r8, #4]
	fa02 f000 	lsl.w	r0, r2, r0
	f1a7 091a 	sub.w	r9, r7, #26
	4308      	orrs	r0, r1
	462c      	mov	r4, r5
	f000 007f 	and.w	r0, r0, #127	@ 0x7f
	f807 0c1a 	strb.w	r0, [r7, #-26]
	6af0      	ldr	r0, [r6, #44]	@ 0x2c
	f000 0003 	and.w	r0, r0, #3
	2802      	cmp	r0, #2
	bf08      	it	eq
	4654      	moveq	r4, sl
	f106 0018 	add.w	r0, r6, #24
	4649      	mov	r1, r9
	2201      	movs	r2, #1
	47a0      	blx	r4
	2800      	cmp	r0, #0
	d0f8      	beq.n	<_defmt_release+0x5c>
	e014      	b.n	<_defmt_release+0x96>
	3079      	adds	r0, #121	@ 0x79
	f1a7 091b 	sub.w	r9, r7, #27
	f040 0080 	orr.w	r0, r0, #128	@ 0x80
	f807 0c1b 	strb.w	r0, [r7, #-27]
	6af0      	ldr	r0, [r6, #44]	@ 0x2c
	462c      	mov	r4, r5
	f000 0003 	and.w	r0, r0, #3
	2802      	cmp	r0, #2
	bf08      	it	eq
	4654      	moveq	r4, sl
	f106 0018 	add.w	r0, r6, #24
	4649      	mov	r1, r9
	2201      	movs	r2, #1
	47a0      	blx	r4
	2800      	cmp	r0, #0
	d0f8      	beq.n	<_defmt_release+0x88>
	2801      	cmp	r0, #1
	d122      	bne.n	<_defmt_release+0xe0>
	2000      	movs	r0, #0
	f1a7 0419 	sub.w	r4, r7, #25
	f807 0c19 	strb.w	r0, [r7, #-25]
	6af0      	ldr	r0, [r6, #44]	@ 0x2c
	f000 0003 	and.w	r0, r0, #3
	2802      	cmp	r0, #2
	bf08      	it	eq
	4655      	moveq	r5, sl
	f106 0018 	add.w	r0, r6, #24
	4621      	mov	r1, r4
	2201      	movs	r2, #1
	47a8      	blx	r5
	2800      	cmp	r0, #0
	d0f8      	beq.n	<_defmt_release+0xb0>
	2801      	cmp	r0, #1
	d10e      	bne.n	<_defmt_release+0xe0>
	2000      	movs	r0, #0
	f8a8 0003 	strh.w	r0, [r8, #3]
	f888 0001 	strb.w	r0, [r8, #1]
	f898 0000 	ldrb.w	r0, [r8]
	07c0      	lsls	r0, r0, #31
	bf18      	it	ne
	f004 f89d 	blne	<__cpsie>
	b002      	add	sp, #8
	e8bd 0700 	ldmia.w	sp!, {r8, r9, sl}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f24a 7374 	movw	r3, #42868	@ 0xa774
	2101      	movs	r1, #1
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2201      	movs	r2, #1
	f7ff f823 	bl	<core::slice::index::slice_index_fail>

<_defmt_write>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b083      	sub	sp, #12
	2900      	cmp	r1, #0
	f000 80ba 	beq.w	<_defmt_write+0x184>
	f240 0840 	movw	r8, #64	@ 0x40
	4604      	mov	r4, r0
	f2c2 0800 	movt	r8, #8192	@ 0x2000
	eb00 0a01 	add.w	sl, r0, r1
	f898 0003 	ldrb.w	r0, [r8, #3]
	f240 0908 	movw	r9, #8
	f244 5bcf 	movw	fp, #17871	@ 0x45cf
	f2c2 0900 	movt	r9, #8192	@ 0x2000
	f6c0 0b00 	movt	fp, #2048	@ 0x800
	e008      	b.n	<_defmt_write+0x46>
	2801      	cmp	r0, #1
	f040 80a9 	bne.w	<_defmt_write+0x18c>
	2000      	movs	r0, #0
	f8a8 0003 	strh.w	r0, [r8, #3]
	4554      	cmp	r4, sl
	f000 809f 	beq.w	<_defmt_write+0x184>
	f814 1b01 	ldrb.w	r1, [r4], #1
	b2c2      	uxtb	r2, r0
	2a07      	cmp	r2, #7
	d21f      	bcs.n	<_defmt_write+0x90>
	2900      	cmp	r1, #0
	d057      	beq.n	<_defmt_write+0x104>
	f807 1c21 	strb.w	r1, [r7, #-33]
	465d      	mov	r5, fp
	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
	f000 0003 	and.w	r0, r0, #3
	2802      	cmp	r0, #2
	f244 504d 	movw	r0, #17741	@ 0x454d
	f6c0 0000 	movt	r0, #2048	@ 0x800
	bf08      	it	eq
	4605      	moveq	r5, r0
	f109 0018 	add.w	r0, r9, #24
	f1a7 0121 	sub.w	r1, r7, #33	@ 0x21
	2201      	movs	r2, #1
	47a8      	blx	r5
	2800      	cmp	r0, #0
	d0f7      	beq.n	<_defmt_write+0x70>
	2801      	cmp	r0, #1
	f040 8083 	bne.w	<_defmt_write+0x18c>
	f898 0003 	ldrb.w	r0, [r8, #3]
	f898 1004 	ldrb.w	r1, [r8, #4]
	e041      	b.n	<_defmt_write+0x114>
	2900      	cmp	r1, #0
	d05d      	beq.n	<_defmt_write+0x150>
	f807 1c1e 	strb.w	r1, [r7, #-30]
	465d      	mov	r5, fp
	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
	f000 0003 	and.w	r0, r0, #3
	2802      	cmp	r0, #2
	f244 504d 	movw	r0, #17741	@ 0x454d
	f6c0 0000 	movt	r0, #2048	@ 0x800
	bf08      	it	eq
	4605      	moveq	r5, r0
	f109 0618 	add.w	r6, r9, #24
	f1a7 011e 	sub.w	r1, r7, #30
	2201      	movs	r2, #1
	4630      	mov	r0, r6
	47a8      	blx	r5
	2800      	cmp	r0, #0
	d0f6      	beq.n	<_defmt_write+0xb0>
	2801      	cmp	r0, #1
	d162      	bne.n	<_defmt_write+0x18c>
	f898 0003 	ldrb.w	r0, [r8, #3]
	3001      	adds	r0, #1
	f888 0003 	strb.w	r0, [r8, #3]
	b2c1      	uxtb	r1, r0
	2986      	cmp	r1, #134	@ 0x86
	d1b4      	bne.n	<_defmt_write+0x40>
	20ff      	movs	r0, #255	@ 0xff
	465d      	mov	r5, fp
	f807 0c1d 	strb.w	r0, [r7, #-29]
	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
	f000 0003 	and.w	r0, r0, #3
	2802      	cmp	r0, #2
	f244 504d 	movw	r0, #17741	@ 0x454d
	f6c0 0000 	movt	r0, #2048	@ 0x800
	bf08      	it	eq
	4605      	moveq	r5, r0
	f1a7 011d 	sub.w	r1, r7, #29
	4630      	mov	r0, r6
	2201      	movs	r2, #1
	47a8      	blx	r5
	2800      	cmp	r0, #0
	d0f8      	beq.n	<_defmt_write+0xf4>
	e797      	b.n	<_defmt_write+0x34>
	f898 1004 	ldrb.w	r1, [r8, #4]
	2301      	movs	r3, #1
	fa03 f202 	lsl.w	r2, r3, r2
	4311      	orrs	r1, r2
	f888 1004 	strb.w	r1, [r8, #4]
	3001      	adds	r0, #1
	f888 0003 	strb.w	r0, [r8, #3]
	b2c2      	uxtb	r2, r0
	2a07      	cmp	r2, #7
	d18f      	bne.n	<_defmt_write+0x40>
	060a      	lsls	r2, r1, #24
	d08d      	beq.n	<_defmt_write+0x40>
	f88d 1008 	strb.w	r1, [sp, #8]
	465d      	mov	r5, fp
	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
	f000 0003 	and.w	r0, r0, #3
	2802      	cmp	r0, #2
	f244 504d 	movw	r0, #17741	@ 0x454d
	f6c0 0000 	movt	r0, #2048	@ 0x800
	bf08      	it	eq
	4605      	moveq	r5, r0
	f109 0018 	add.w	r0, r9, #24
	a902      	add	r1, sp, #8
	2201      	movs	r2, #1
	47a8      	blx	r5
	2800      	cmp	r0, #0
	d0f8      	beq.n	<_defmt_write+0x140>
	e771      	b.n	<_defmt_write+0x34>
	3079      	adds	r0, #121	@ 0x79
	465d      	mov	r5, fp
	f040 0080 	orr.w	r0, r0, #128	@ 0x80
	f807 0c1f 	strb.w	r0, [r7, #-31]
	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
	f000 0003 	and.w	r0, r0, #3
	2802      	cmp	r0, #2
	f244 504d 	movw	r0, #17741	@ 0x454d
	f6c0 0000 	movt	r0, #2048	@ 0x800
	bf08      	it	eq
	4605      	moveq	r5, r0
	f109 0018 	add.w	r0, r9, #24
	f1a7 011f 	sub.w	r1, r7, #31
	2201      	movs	r2, #1
	47a8      	blx	r5
	2800      	cmp	r0, #0
	d0f7      	beq.n	<_defmt_write+0x172>
	e757      	b.n	<_defmt_write+0x34>
	b003      	add	sp, #12
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f24a 7374 	movw	r3, #42868	@ 0xa774
	2101      	movs	r1, #1
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2201      	movs	r2, #1
	f7fe ff55 	bl	<core::slice::index::slice_index_fail>

<ETH>:
	b5b0      	push	{r4, r5, r7, lr}
	af02      	add	r7, sp, #8
	f003 ffca 	bl	<__primask_r>
	4604      	mov	r4, r0
	f003 ffba 	bl	<__cpsid>
	07e0      	lsls	r0, r4, #31
	bf08      	it	eq
	f003 ffb8 	bleq	<__cpsie>
	f249 0014 	movw	r0, #36884	@ 0x9014
	2141      	movs	r1, #65	@ 0x41
	f2c4 0002 	movt	r0, #16386	@ 0x4002
	f240 0440 	movw	r4, #64	@ 0x40
	6805      	ldr	r5, [r0, #0]
	f2c0 0101 	movt	r1, #1
	f2c2 0400 	movt	r4, #8192	@ 0x2000
	6001      	str	r1, [r0, #0]
	07e8      	lsls	r0, r5, #31
	d01d      	beq.n	<ETH+0x70>
	f3bf 8f5f 	dmb	sy
	e854 0f09 	ldrex	r0, [r4, #36]	@ 0x24
	f040 0102 	orr.w	r1, r0, #2
	e844 1209 	strex	r2, r1, [r4, #36]	@ 0x24
	2a00      	cmp	r2, #0
	d1f7      	bne.n	<ETH+0x38>
	f3bf 8f5f 	dmb	sy
	b980      	cbnz	r0, <ETH+0x70>
	e9d4 1007 	ldrd	r1, r0, [r4, #28]
	2200      	movs	r2, #0
	61e2      	str	r2, [r4, #28]
	f3bf 8f5f 	dmb	sy
	e854 2f09 	ldrex	r2, [r4, #36]	@ 0x24
	f022 0202 	bic.w	r2, r2, #2
	e844 2309 	strex	r3, r2, [r4, #36]	@ 0x24
	2b00      	cmp	r3, #0
	d1f7      	bne.n	<ETH+0x5a>
	b109      	cbz	r1, <ETH+0x70>
	6849      	ldr	r1, [r1, #4]
	4788      	blx	r1
	0668      	lsls	r0, r5, #25
	d51e      	bpl.n	<ETH+0xb2>
	f3bf 8f5f 	dmb	sy
	e854 0f06 	ldrex	r0, [r4, #24]
	f040 0102 	orr.w	r1, r0, #2
	e844 1206 	strex	r2, r1, [r4, #24]
	2a00      	cmp	r2, #0
	d1f7      	bne.n	<ETH+0x78>
	f3bf 8f5f 	dmb	sy
	b988      	cbnz	r0, <ETH+0xb2>
	e9d4 1004 	ldrd	r1, r0, [r4, #16]
	2200      	movs	r2, #0
	6122      	str	r2, [r4, #16]
	f3bf 8f5f 	dmb	sy
	e854 2f06 	ldrex	r2, [r4, #24]
	f022 0202 	bic.w	r2, r2, #2
	e844 2306 	strex	r3, r2, [r4, #24]
	2b00      	cmp	r3, #0
	d1f7      	bne.n	<ETH+0x9a>
	2900      	cmp	r1, #0
	bf1c      	itt	ne
	6849      	ldrne	r1, [r1, #4]
	4788      	blxne	r1
	f248 0138 	movw	r1, #32824	@ 0x8038
	f2c4 0102 	movt	r1, #16386	@ 0x4002
	6808      	ldr	r0, [r1, #0]
	0580      	lsls	r0, r0, #22
	bf44      	itt	mi
	f44f 7000 	movmi.w	r0, #512	@ 0x200
	6048      	strmi	r0, [r1, #4]
	f3bf 8f5f 	dmb	sy
	e854 0f0c 	ldrex	r0, [r4, #48]	@ 0x30
	f040 0202 	orr.w	r2, r0, #2
	e844 230c 	strex	r3, r2, [r4, #48]	@ 0x30
	2b00      	cmp	r3, #0
	d1f7      	bne.n	<ETH+0xca>
	f3bf 8f5f 	dmb	sy
	b110      	cbz	r0, <ETH+0xe6>
	f8d1 06f0 	ldr.w	r0, [r1, #1776]	@ 0x6f0
	bdb0      	pop	{r4, r5, r7, pc}
	e9d4 200a 	ldrd	r2, r0, [r4, #40]	@ 0x28
	2300      	movs	r3, #0
	62a3      	str	r3, [r4, #40]	@ 0x28
	f3bf 8f5f 	dmb	sy
	e854 3f0c 	ldrex	r3, [r4, #48]	@ 0x30
	f023 0302 	bic.w	r3, r3, #2
	e844 350c 	strex	r5, r3, [r4, #48]	@ 0x30
	2d00      	cmp	r5, #0
	d1f7      	bne.n	<ETH+0xf2>
	2a00      	cmp	r2, #0
	d0ec      	beq.n	<ETH+0xe0>
	6851      	ldr	r1, [r2, #4]
	e8bd 40b0 	ldmia.w	sp!, {r4, r5, r7, lr}
	4708      	bx	r1

<SysTick>:
	b5d0      	push	{r4, r6, r7, lr}
	af02      	add	r7, sp, #8
	f003 ff43 	bl	<__primask_r>
	4604      	mov	r4, r0
	f003 ff33 	bl	<__cpsid>
	f240 0040 	movw	r0, #64	@ 0x40
	f2c2 0000 	movt	r0, #8192	@ 0x2000
	e9d0 1202 	ldrd	r1, r2, [r0, #8]
	3101      	adds	r1, #1
	f142 0200 	adc.w	r2, r2, #0
	e9c0 1202 	strd	r1, r2, [r0, #8]
	07e0      	lsls	r0, r4, #31
	bf18      	it	ne
	bdd0      	popne	{r4, r6, r7, pc}
	e8bd 40d0 	ldmia.w	sp!, {r4, r6, r7, lr}
	f003 bf23 	b.w	<__cpsie>

<main>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	f000 f800 	bl	<ip::__cortex_m_rt_main>

<ip::__cortex_m_rt_main>:
	b5b0      	push	{r4, r5, r7, lr}
	af02      	add	r7, sp, #8
	f5ad 5d01 	sub.w	sp, sp, #8256	@ 0x2040
	f003 ff24 	bl	<__primask_r>
	4604      	mov	r4, r0
	f003 ff14 	bl	<__cpsid>
	f240 0540 	movw	r5, #64	@ 0x40
	07e1      	lsls	r1, r4, #31
	f2c2 0500 	movt	r5, #8192	@ 0x2000
	79a8      	ldrb	r0, [r5, #6]
	d107      	bne.n	<ip::__cortex_m_rt_main+0x30>
	07c0      	lsls	r0, r0, #31
	f041 82b2 	bne.w	<ip::__cortex_m_rt_main+0x158a>
	2001      	movs	r0, #1
	71a8      	strb	r0, [r5, #6]
	f003 ff08 	bl	<__cpsie>
	e004      	b.n	<ip::__cortex_m_rt_main+0x3a>
	07c0      	lsls	r0, r0, #31
	f041 82ac 	bne.w	<ip::__cortex_m_rt_main+0x158e>
	2001      	movs	r0, #1
	71a8      	strb	r0, [r5, #6]
	f003 ff0b 	bl	<__primask_r>
	4604      	mov	r4, r0
	f003 fefb 	bl	<__cpsid>
	7968      	ldrb	r0, [r5, #5]
	07e1      	lsls	r1, r4, #31
	d107      	bne.n	<ip::__cortex_m_rt_main+0x5a>
	2800      	cmp	r0, #0
	f041 82a5 	bne.w	<ip::__cortex_m_rt_main+0x159a>
	2001      	movs	r0, #1
	7168      	strb	r0, [r5, #5]
	f003 fef3 	bl	<__cpsie>
	e004      	b.n	<ip::__cortex_m_rt_main+0x64>
	2800      	cmp	r0, #0
	f041 829f 	bne.w	<ip::__cortex_m_rt_main+0x159e>
	2001      	movs	r0, #1
	7168      	strb	r0, [r5, #5]
	f10d 0e18 	add.w	lr, sp, #24
	f64d 0c00 	movw	ip, #55296	@ 0xd800
	f24f 0480 	movw	r4, #61568	@ 0xf080
	f64c 4300 	movw	r3, #52224	@ 0xcc00
	f245 0903 	movw	r9, #20483	@ 0x5003
	f24a 0b07 	movw	fp, #40967	@ 0xa007
	f50e 5097 	add.w	r0, lr, #4832	@ 0x12e0
	f24a 1e20 	movw	lr, #41248	@ 0xa120
	f2c0 0e07 	movt	lr, #7
	f2c0 5cb8 	movt	ip, #1464	@ 0x5b8
	f04f 0a02 	mov.w	sl, #2
	f04f 4800 	mov.w	r8, #2147483648	@ 0x80000000
	f2c0 24fa 	movt	r4, #762	@ 0x2fa
	f6c1 13bf 	movt	r3, #6591	@ 0x19bf
	f2cf 498e 	movt	r9, #62606	@ 0xf48e
	f6ce 1b1c 	movt	fp, #59676	@ 0xe91c
	f8c0 eaf0 	str.w	lr, [r0, #2800]	@ 0xaf0
	f8cd cca0 	str.w	ip, [sp, #3232]	@ 0xca0
	e001      	b.n	<ip::__cortex_m_rt_main+0xb2>
	f10a 0a01 	add.w	sl, sl, #1
	f1ba 0f09 	cmp.w	sl, #9
	f001 8248 	beq.w	<ip::__cortex_m_rt_main+0x154a>
	f1ba 0f04 	cmp.w	sl, #4
	d3f6      	bcc.n	<ip::__cortex_m_rt_main+0xae>
	fbb8 f0fa 	udiv	r0, r8, sl
	f44f 72d8 	mov.w	r2, #432	@ 0x1b0
	3001      	adds	r0, #1
	0840      	lsrs	r0, r0, #1
	fba0 010e 	umull	r0, r1, r0, lr
	e003      	b.n	<ip::__cortex_m_rt_main+0xda>
	1e55      	subs	r5, r2, #1
	2a33      	cmp	r2, #51	@ 0x33
	462a      	mov	r2, r5
	d3e9      	bcc.n	<ip::__cortex_m_rt_main+0xae>
	fba0 5602 	umull	r5, r6, r0, r2
	fb01 6602 	mla	r6, r1, r2, r6
	0ead      	lsrs	r5, r5, #26
	ea45 1586 	orr.w	r5, r5, r6, lsl #6
	42a5      	cmp	r5, r4
	d3e0      	bcc.n	<ip::__cortex_m_rt_main+0xae>
	f025 050f 	bic.w	r5, r5, #15
	429d      	cmp	r5, r3
	d8ee      	bhi.n	<ip::__cortex_m_rt_main+0xd2>
	eb05 060b 	add.w	r6, r5, fp
	2e0f      	cmp	r6, #15
	bf24      	itt	cs
	444d      	addcs	r5, r9
	2d07      	cmpcs	r5, #7
	d2e7      	bcs.n	<ip::__cortex_m_rt_main+0xd2>
	ea5f 600a 	movs.w	r0, sl, lsl #24
	d105      	bne.n	<ip::__cortex_m_rt_main+0x114>
	f24b 10d8 	movw	r0, #45528	@ 0xb1d8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7ff f9f3 	bl	<core::panicking::panic_const::panic_const_div_by_zero>
	fa5f f08a 	uxtb.w	r0, sl
	f04f 4100 	mov.w	r1, #2147483648	@ 0x80000000
	fbb1 f0f0 	udiv	r0, r1, r0
	b293      	uxth	r3, r2
	3001      	adds	r0, #1
	2e0f      	cmp	r6, #15
	ea4f 0050 	mov.w	r0, r0, lsr #1
	fba0 010e 	umull	r0, r1, r0, lr
	fba0 0503 	umull	r0, r5, r0, r3
	fb01 5103 	mla	r1, r1, r3, r5
	f04f 051d 	mov.w	r5, #29
	bf38      	it	cc
	251c      	movcc	r5, #28
	f085 041f 	eor.w	r4, r5, #31
	0f80      	lsrs	r0, r0, #30
	ea40 0081 	orr.w	r0, r0, r1, lsl #2
	0f8b      	lsrs	r3, r1, #30
	40ab      	lsls	r3, r5
	0841      	lsrs	r1, r0, #1
	40a8      	lsls	r0, r5
	40e1      	lsrs	r1, r4
	0e80      	lsrs	r0, r0, #26
	ea40 1081 	orr.w	r0, r0, r1, lsl #6
	430b      	orrs	r3, r1
	f020 010c 	bic.w	r1, r0, #12
	f24e 6001 	movw	r0, #58881	@ 0xe601
	f6c0 40df 	movt	r0, #3295	@ 0xcdf
	0e9b      	lsrs	r3, r3, #26
	1a08      	subs	r0, r1, r0
	f173 0000 	sbcs.w	r0, r3, #0
	f081 81b9 	bcs.w	<ip::__cortex_m_rt_main+0x14e4>
	ee00 1a10 	vmov	s0, r1
	4561      	cmp	r1, ip
	bf38      	it	cc
	468c      	movcc	ip, r1
	eeb8 1a40 	vcvt.f32.u32	s2, s0
	ee00 ca10 	vmov	s0, ip
	eeb8 0a40 	vcvt.f32.u32	s0, s0
	eeb1 2a41 	vneg.f32	s4, s2
	ee82 2a00 	vdiv.f32	s4, s4, s0
	eebd 3ac2 	vcvt.s32.f32	s6, s4
	eebf 0a00 	vmov.f32	s0, #240	@ 0xbf800000 -1.0
	eeb8 3ac3 	vcvt.f32.s32	s6, s6
	eeb4 2a43 	vcmp.f32	s4, s6
	ee33 4a00 	vadd.f32	s8, s6, s0
	eef1 fa10 	vmrs	APSR_nzcv, fpscr
	bf48      	it	mi
	eeb0 3a44 	vmovmi.f32	s6, s8
	eeb1 2a43 	vneg.f32	s4, s6
	eebc 2ac2 	vcvt.u32.f32	s4, s4
	ee12 0a10 	vmov	r0, s4
	2801      	cmp	r0, #1
	d011      	beq.n	<ip::__cortex_m_rt_main+0x1e2>
	2802      	cmp	r0, #2
	d00a      	beq.n	<ip::__cortex_m_rt_main+0x1d8>
	2800      	cmp	r0, #0
	f001 8199 	beq.w	<ip::__cortex_m_rt_main+0x14fa>
	1ec3      	subs	r3, r0, #3
	2b03      	cmp	r3, #3
	d26f      	bcs.n	<ip::__cortex_m_rt_main+0x2ae>
	eeb1 2a00 	vmov.f32	s4, #16	@ 0x40800000  4.0
	f04f 0e90 	mov.w	lr, #144	@ 0x90
	e008      	b.n	<ip::__cortex_m_rt_main+0x1ea>
	eeb0 2a00 	vmov.f32	s4, #0	@ 0x40000000  2.0
	f04f 0e80 	mov.w	lr, #128	@ 0x80
	e003      	b.n	<ip::__cortex_m_rt_main+0x1ea>
	eeb7 2a00 	vmov.f32	s4, #112	@ 0x3f800000  1.0
	f04f 0e00 	mov.w	lr, #0
	ee81 1a02 	vdiv.f32	s2, s2, s4
	f249 5c00 	movw	ip, #38144	@ 0x9500
	f64f 1080 	movw	r0, #63872	@ 0xf980
	f24f 3300 	movw	r3, #62208	@ 0xf300
	f6c0 2cba 	movt	ip, #2746	@ 0xaba
	f10c 0901 	add.w	r9, ip, #1
	f2c0 3037 	movt	r0, #823	@ 0x337
	f2c0 636f 	movt	r3, #1647	@ 0x66f
	eebd 2ac1 	vcvt.s32.f32	s4, s2
	eeb8 2ac2 	vcvt.f32.s32	s4, s4
	eeb4 1a42 	vcmp.f32	s2, s4
	ee32 3a00 	vadd.f32	s6, s4, s0
	eef1 fa10 	vmrs	APSR_nzcv, fpscr
	bf48      	it	mi
	eeb0 2a43 	vmovmi.f32	s4, s6
	eebc 1ac2 	vcvt.u32.f32	s2, s4
	4549      	cmp	r1, r9
	bf3c      	itt	cc
	f24a 5040 	movwcc	r0, #42304	@ 0xa540
	f2c0 20ae 	movtcc	r0, #686	@ 0x2ae
	bf3c      	itt	cc
	f644 2380 	movwcc	r3, #19072	@ 0x4a80
	f2c0 535d 	movtcc	r3, #1373	@ 0x55d
	ee11 5a10 	vmov	r5, s2
	eeb8 1a41 	vcvt.f32.u32	s2, s2
	eeb1 1a41 	vneg.f32	s2, s2
	4285      	cmp	r5, r0
	bf38      	it	cc
	4628      	movcc	r0, r5
	ee02 0a10 	vmov	s4, r0
	eeb8 2a42 	vcvt.f32.u32	s4, s4
	ee81 2a02 	vdiv.f32	s4, s2, s4
	eebd 3ac2 	vcvt.s32.f32	s6, s4
	eeb8 3ac3 	vcvt.f32.s32	s6, s6
	eeb4 2a43 	vcmp.f32	s4, s6
	ee33 4a00 	vadd.f32	s8, s6, s0
	eef1 fa10 	vmrs	APSR_nzcv, fpscr
	bf48      	it	mi
	eeb0 3a44 	vmovmi.f32	s6, s8
	eeb1 2a43 	vneg.f32	s4, s6
	429d      	cmp	r5, r3
	bf38      	it	cc
	462b      	movcc	r3, r5
	eebc 2ac2 	vcvt.u32.f32	s4, s4
	ee12 0a10 	vmov	r0, s4
	2801      	cmp	r0, #1
	d00d      	beq.n	<ip::__cortex_m_rt_main+0x2a8>
	2802      	cmp	r0, #2
	d008      	beq.n	<ip::__cortex_m_rt_main+0x2a2>
	2800      	cmp	r0, #0
	f001 813d 	beq.w	<ip::__cortex_m_rt_main+0x1510>
	1ec4      	subs	r4, r0, #3
	2c04      	cmp	r4, #4
	d210      	bcs.n	<ip::__cortex_m_rt_main+0x2be>
	f44f 58a0 	mov.w	r8, #5120	@ 0x1400
	e014      	b.n	<ip::__cortex_m_rt_main+0x2cc>
	f44f 5880 	mov.w	r8, #4096	@ 0x1000
	e011      	b.n	<ip::__cortex_m_rt_main+0x2cc>
	f04f 0800 	mov.w	r8, #0
	e00e      	b.n	<ip::__cortex_m_rt_main+0x2cc>
	1f83      	subs	r3, r0, #6
	2b06      	cmp	r3, #6
	d272      	bcs.n	<ip::__cortex_m_rt_main+0x39a>
	eeb2 2a00 	vmov.f32	s4, #32	@ 0x41000000  8.0
	f04f 0ea0 	mov.w	lr, #160	@ 0xa0
	e795      	b.n	<ip::__cortex_m_rt_main+0x1ea>
	3807      	subs	r0, #7
	f44f 58e0 	mov.w	r8, #7168	@ 0x1c00
	2806      	cmp	r0, #6
	bf38      	it	cc
	f44f 58c0 	movcc.w	r8, #6144	@ 0x1800
	ee02 3a10 	vmov	s4, r3
	eeb8 2a42 	vcvt.f32.u32	s4, s4
	ee81 1a02 	vdiv.f32	s2, s2, s4
	eebd 2ac1 	vcvt.s32.f32	s4, s2
	eeb8 2ac2 	vcvt.f32.s32	s4, s4
	eeb4 1a42 	vcmp.f32	s2, s4
	ee32 0a00 	vadd.f32	s0, s4, s0
	eef1 fa10 	vmrs	APSR_nzcv, fpscr
	bf48      	it	mi
	eeb0 2a40 	vmovmi.f32	s4, s0
	eeb1 0a42 	vneg.f32	s0, s4
	9507      	str	r5, [sp, #28]
	eebc 0ac0 	vcvt.u32.f32	s0, s0
	ee10 0a10 	vmov	r0, s0
	2801      	cmp	r0, #1
	d019      	beq.n	<ip::__cortex_m_rt_main+0x338>
	2802      	cmp	r0, #2
	d00e      	beq.n	<ip::__cortex_m_rt_main+0x326>
	2800      	cmp	r0, #0
	f001 810c 	beq.w	<ip::__cortex_m_rt_main+0x1526>
	1ec3      	subs	r3, r0, #3
	2b04      	cmp	r3, #4
	d21b      	bcs.n	<ip::__cortex_m_rt_main+0x34c>
	f44f 4b20 	mov.w	fp, #40960	@ 0xa000
	f24c 3081 	movw	r0, #50049	@ 0xc381
	f2c0 10c9 	movt	r0, #457	@ 0x1c9
	4281      	cmp	r1, r0
	d311      	bcc.n	<ip::__cortex_m_rt_main+0x348>
	e01f      	b.n	<ip::__cortex_m_rt_main+0x366>
	f44f 4b00 	mov.w	fp, #32768	@ 0x8000
	f24c 3081 	movw	r0, #50049	@ 0xc381
	f2c0 10c9 	movt	r0, #457	@ 0x1c9
	4281      	cmp	r1, r0
	d308      	bcc.n	<ip::__cortex_m_rt_main+0x348>
	e016      	b.n	<ip::__cortex_m_rt_main+0x366>
	f04f 0b00 	mov.w	fp, #0
	f24c 3081 	movw	r0, #50049	@ 0xc381
	f2c0 10c9 	movt	r0, #457	@ 0x1c9
	4281      	cmp	r1, r0
	d20e      	bcs.n	<ip::__cortex_m_rt_main+0x366>
	2300      	movs	r3, #0
	e043      	b.n	<ip::__cortex_m_rt_main+0x3d4>
	3807      	subs	r0, #7
	f44f 4b60 	mov.w	fp, #57344	@ 0xe000
	2806      	cmp	r0, #6
	bf38      	it	cc
	f44f 4b40 	movcc.w	fp, #49152	@ 0xc000
	f24c 3081 	movw	r0, #50049	@ 0xc381
	f2c0 10c9 	movt	r0, #457	@ 0x1c9
	4281      	cmp	r1, r0
	d3f0      	bcc.n	<ip::__cortex_m_rt_main+0x348>
	f248 7000 	movw	r0, #34560	@ 0x8700
	f2c0 3093 	movt	r0, #915	@ 0x393
	3001      	adds	r0, #1
	4281      	cmp	r1, r0
	d201      	bcs.n	<ip::__cortex_m_rt_main+0x378>
	2301      	movs	r3, #1
	e02d      	b.n	<ip::__cortex_m_rt_main+0x3d4>
	f644 2080 	movw	r0, #19072	@ 0x4a80
	f2c0 505d 	movt	r0, #1373	@ 0x55d
	3001      	adds	r0, #1
	4281      	cmp	r1, r0
	d201      	bcs.n	<ip::__cortex_m_rt_main+0x38a>
	2302      	movs	r3, #2
	e024      	b.n	<ip::__cortex_m_rt_main+0x3d4>
	f640 6001 	movw	r0, #3585	@ 0xe01
	f2c0 7027 	movt	r0, #1831	@ 0x727
	4281      	cmp	r1, r0
	d20b      	bcs.n	<ip::__cortex_m_rt_main+0x3ae>
	2303      	movs	r3, #3
	e01c      	b.n	<ip::__cortex_m_rt_main+0x3d4>
	f1a0 030c 	sub.w	r3, r0, #12
	2b1c      	cmp	r3, #28
	f080 850f 	bcs.w	<ip::__cortex_m_rt_main+0xdc2>
	eeb3 2a00 	vmov.f32	s4, #48	@ 0x41800000  16.0
	f04f 0eb0 	mov.w	lr, #176	@ 0xb0
	e71d      	b.n	<ip::__cortex_m_rt_main+0x1ea>
	f24d 1080 	movw	r0, #53632	@ 0xd180
	2305      	movs	r3, #5
	f6c0 00f0 	movt	r0, #2288	@ 0x8f0
	3001      	adds	r0, #1
	4281      	cmp	r1, r0
	bf38      	it	cc
	2304      	movcc	r3, #4
	4549      	cmp	r1, r9
	d307      	bcc.n	<ip::__cortex_m_rt_main+0x3d4>
	f645 0081 	movw	r0, #22657	@ 0x5881
	2307      	movs	r3, #7
	f6c0 4084 	movt	r0, #3204	@ 0xc84
	4281      	cmp	r1, r0
	bf38      	it	cc
	2306      	movcc	r3, #6
	f643 0910 	movw	r9, #14352	@ 0x3810
	f2c4 0902 	movt	r9, #16386	@ 0x4002
	f859 0c10 	ldr.w	r0, [r9, #-16]
	f040 0001 	orr.w	r0, r0, #1
	f849 0c10 	str.w	r0, [r9, #-16]
	f859 0c10 	ldr.w	r0, [r9, #-16]
	0780      	lsls	r0, r0, #30
	d40c      	bmi.n	<ip::__cortex_m_rt_main+0x40a>
	f859 0c10 	ldr.w	r0, [r9, #-16]
	0780      	lsls	r0, r0, #30
	bf5c      	itt	pl
	f859 0c10 	ldrpl.w	r0, [r9, #-16]
	ea5f 7080 	movspl.w	r0, r0, lsl #30
	d403      	bmi.n	<ip::__cortex_m_rt_main+0x40a>
	f859 0c10 	ldr.w	r0, [r9, #-16]
	0780      	lsls	r0, r0, #30
	d5ee      	bpl.n	<ip::__cortex_m_rt_main+0x3e8>
	f859 0c08 	ldr.w	r0, [r9, #-8]
	2500      	movs	r5, #0
	f020 0003 	bic.w	r0, r0, #3
	f849 0c08 	str.w	r0, [r9, #-8]
	f859 0c10 	ldr.w	r0, [r9, #-16]
	f440 2080 	orr.w	r0, r0, #262144	@ 0x40000
	f849 0c10 	str.w	r0, [r9, #-16]
	f859 0c10 	ldr.w	r0, [r9, #-16]
	f440 3080 	orr.w	r0, r0, #65536	@ 0x10000
	f849 0c10 	str.w	r0, [r9, #-16]
	f647 2001 	movw	r0, #31233	@ 0x7a01
	f6c0 2003 	movt	r0, #2563	@ 0xa03
	4281      	cmp	r1, r0
	f244 4001 	movw	r0, #17409	@ 0x4401
	bf38      	it	cc
	2501      	movcc	r5, #1
	f6c0 0095 	movt	r0, #2197	@ 0x895
	4281      	cmp	r1, r0
	bf38      	it	cc
	2502      	movcc	r5, #2
	f859 0c10 	ldr.w	r0, [r9, #-16]
	0380      	lsls	r0, r0, #14
	d40c      	bmi.n	<ip::__cortex_m_rt_main+0x46e>
	f859 0c10 	ldr.w	r0, [r9, #-16]
	0380      	lsls	r0, r0, #14
	bf5c      	itt	pl
	f859 0c10 	ldrpl.w	r0, [r9, #-16]
	ea5f 3080 	movspl.w	r0, r0, lsl #14
	d403      	bmi.n	<ip::__cortex_m_rt_main+0x46e>
	f859 0c10 	ldr.w	r0, [r9, #-16]
	0380      	lsls	r0, r0, #14
	d5ee      	bpl.n	<ip::__cortex_m_rt_main+0x44c>
	f859 0c10 	ldr.w	r0, [r9, #-16]
	2e0f      	cmp	r6, #15
	f020 7080 	bic.w	r0, r0, #16777216	@ 0x1000000
	f849 0c10 	str.w	r0, [r9, #-16]
	f647 70c0 	movw	r0, #32704	@ 0x7fc0
	f859 4c0c 	ldr.w	r4, [r9, #-12]
	ea00 1082 	and.w	r0, r0, r2, lsl #6
	f248 0200 	movw	r2, #32768	@ 0x8000
	f2cf 02bc 	movt	r2, #61628	@ 0xf0bc
	ea02 0204 	and.w	r2, r2, r4
	f00a 043f 	and.w	r4, sl, #63	@ 0x3f
	4422      	add	r2, r4
	4410      	add	r0, r2
	bf38      	it	cc
	f500 3080 	addcc.w	r0, r0, #65536	@ 0x10000
	2d02      	cmp	r5, #2
	f100 7010 	add.w	r0, r0, #37748736	@ 0x2400000
	f849 0c0c 	str.w	r0, [r9, #-12]
	f8d9 0030 	ldr.w	r0, [r9, #48]	@ 0x30
	f040 5080 	orr.w	r0, r0, #268435456	@ 0x10000000
	f8c9 0030 	str.w	r0, [r9, #48]	@ 0x30
	f247 0004 	movw	r0, #28676	@ 0x7004
	f2c4 0000 	movt	r0, #16384	@ 0x4000
	f850 2c04 	ldr.w	r2, [r0, #-4]
	d003      	beq.n	<ip::__cortex_m_rt_main+0x4ce>
	2d01      	cmp	r5, #1
	d105      	bne.n	<ip::__cortex_m_rt_main+0x4d6>
	2402      	movs	r4, #2
	e000      	b.n	<ip::__cortex_m_rt_main+0x4d0>
	2401      	movs	r4, #1
	f364 328f 	bfi	r2, r4, #14, #2
	e001      	b.n	<ip::__cortex_m_rt_main+0x4da>
	f442 4240 	orr.w	r2, r2, #49152	@ 0xc000
	f840 2c04 	str.w	r2, [r0, #-4]
	f859 2c10 	ldr.w	r2, [r9, #-16]
	f042 7280 	orr.w	r2, r2, #16777216	@ 0x1000000
	f849 2c10 	str.w	r2, [r9, #-16]
	f859 2c10 	ldr.w	r2, [r9, #-16]
	0192      	lsls	r2, r2, #6
	d40c      	bmi.n	<ip::__cortex_m_rt_main+0x50c>
	f859 2c10 	ldr.w	r2, [r9, #-16]
	0192      	lsls	r2, r2, #6
	bf5c      	itt	pl
	f859 2c10 	ldrpl.w	r2, [r9, #-16]
	ea5f 1282 	movspl.w	r2, r2, lsl #6
	d403      	bmi.n	<ip::__cortex_m_rt_main+0x50c>
	f859 2c10 	ldr.w	r2, [r9, #-16]
	0192      	lsls	r2, r2, #6
	d5ee      	bpl.n	<ip::__cortex_m_rt_main+0x4ea>
	4561      	cmp	r1, ip
	d925      	bls.n	<ip::__cortex_m_rt_main+0x55c>
	f850 1c04 	ldr.w	r1, [r0, #-4]
	f441 3180 	orr.w	r1, r1, #65536	@ 0x10000
	f840 1c04 	str.w	r1, [r0, #-4]
	6801      	ldr	r1, [r0, #0]
	03c9      	lsls	r1, r1, #15
	d409      	bmi.n	<ip::__cortex_m_rt_main+0x536>
	6801      	ldr	r1, [r0, #0]
	03c9      	lsls	r1, r1, #15
	bf5c      	itt	pl
	6801      	ldrpl	r1, [r0, #0]
	ea5f 31c1 	movspl.w	r1, r1, lsl #15
	d402      	bmi.n	<ip::__cortex_m_rt_main+0x536>
	6801      	ldr	r1, [r0, #0]
	03c9      	lsls	r1, r1, #15
	d5f2      	bpl.n	<ip::__cortex_m_rt_main+0x51c>
	f850 1c04 	ldr.w	r1, [r0, #-4]
	f441 3100 	orr.w	r1, r1, #131072	@ 0x20000
	f840 1c04 	str.w	r1, [r0, #-4]
	6801      	ldr	r1, [r0, #0]
	0389      	lsls	r1, r1, #14
	d409      	bmi.n	<ip::__cortex_m_rt_main+0x55c>
	6801      	ldr	r1, [r0, #0]
	0389      	lsls	r1, r1, #14
	bf5c      	itt	pl
	6801      	ldrpl	r1, [r0, #0]
	ea5f 3181 	movspl.w	r1, r1, lsl #14
	d402      	bmi.n	<ip::__cortex_m_rt_main+0x55c>
	6801      	ldr	r1, [r0, #0]
	0389      	lsls	r1, r1, #14
	d5f2      	bpl.n	<ip::__cortex_m_rt_main+0x542>
	f859 0c08 	ldr.w	r0, [r9, #-8]
	f64f 71ff 	movw	r1, #65535	@ 0xffff
	f2c0 019f 	movt	r1, #159	@ 0x9f
	f64f 42f0 	movw	r2, #64752	@ 0xfcf0
	4008      	ands	r0, r1
	f849 0c08 	str.w	r0, [r9, #-8]
	f8c9 33f0 	str.w	r3, [r9, #1008]	@ 0x3f0
	ea48 000e 	orr.w	r0, r8, lr
	f859 1c08 	ldr.w	r1, [r9, #-8]
	ea40 000b 	orr.w	r0, r0, fp
	4391      	bics	r1, r2
	4308      	orrs	r0, r1
	f849 0c08 	str.w	r0, [r9, #-8]
	f859 0c08 	ldr.w	r0, [r9, #-8]
	2102      	movs	r1, #2
	f361 0001 	bfi	r0, r1, #0, #2
	f849 0c08 	str.w	r0, [r9, #-8]
	f859 0c08 	ldr.w	r0, [r9, #-8]
	f000 000c 	and.w	r0, r0, #12
	2808      	cmp	r0, #8
	d011      	beq.n	<ip::__cortex_m_rt_main+0x5c8>
	f859 0c08 	ldr.w	r0, [r9, #-8]
	f000 000c 	and.w	r0, r0, #12
	2808      	cmp	r0, #8
	bf1e      	ittt	ne
	f859 0c08 	ldrne.w	r0, [r9, #-8]
	f000 000c 	andne.w	r0, r0, #12
	2808      	cmpne	r0, #8
	d005      	beq.n	<ip::__cortex_m_rt_main+0x5c8>
	f859 0c08 	ldr.w	r0, [r9, #-8]
	f000 000c 	and.w	r0, r0, #12
	2808      	cmp	r0, #8
	d1e7      	bne.n	<ip::__cortex_m_rt_main+0x598>
	2010      	movs	r0, #16
	f003 fc3a 	bl	<__delay>
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 0001 	orr.w	r0, r0, #1
	f8c9 0020 	str.w	r0, [r9, #32]
	f003 fc38 	bl	<__dsb>
	f8d9 0000 	ldr.w	r0, [r9]
	f040 0001 	orr.w	r0, r0, #1
	f8c9 0000 	str.w	r0, [r9]
	f8d9 0000 	ldr.w	r0, [r9]
	f020 0001 	bic.w	r0, r0, #1
	f8c9 0000 	str.w	r0, [r9]
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 0002 	orr.w	r0, r0, #2
	f8c9 0020 	str.w	r0, [r9, #32]
	f003 fc24 	bl	<__dsb>
	f8d9 0000 	ldr.w	r0, [r9]
	f040 0002 	orr.w	r0, r0, #2
	f8c9 0000 	str.w	r0, [r9]
	f8d9 0000 	ldr.w	r0, [r9]
	f020 0002 	bic.w	r0, r0, #2
	f8c9 0000 	str.w	r0, [r9]
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 0004 	orr.w	r0, r0, #4
	f8c9 0020 	str.w	r0, [r9, #32]
	f003 fc10 	bl	<__dsb>
	f8d9 0000 	ldr.w	r0, [r9]
	f040 0004 	orr.w	r0, r0, #4
	f8c9 0000 	str.w	r0, [r9]
	f8d9 0000 	ldr.w	r0, [r9]
	f020 0004 	bic.w	r0, r0, #4
	f8c9 0000 	str.w	r0, [r9]
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 0040 	orr.w	r0, r0, #64	@ 0x40
	f8c9 0020 	str.w	r0, [r9, #32]
	f003 fbfc 	bl	<__dsb>
	f8d9 0000 	ldr.w	r0, [r9]
	f24e 0210 	movw	r2, #57360	@ 0xe010
	f2ce 0200 	movt	r2, #57344	@ 0xe000
	f64f 71fe 	movw	r1, #65534	@ 0xfffe
	f040 0040 	orr.w	r0, r0, #64	@ 0x40
	f8c9 0000 	str.w	r0, [r9]
	f8d9 0000 	ldr.w	r0, [r9]
	f2c0 01ff 	movt	r1, #255	@ 0xff
	f020 0040 	bic.w	r0, r0, #64	@ 0x40
	f8c9 0000 	str.w	r0, [r9]
	68d0      	ldr	r0, [r2, #12]
	4008      	ands	r0, r1
	f649 119a 	movw	r1, #39322	@ 0x999a
	f6c1 1199 	movt	r1, #6553	@ 0x1999
	fba0 0101 	umull	r0, r1, r0, r1
	6051      	str	r1, [r2, #4]
	6810      	ldr	r0, [r2, #0]
	f040 0001 	orr.w	r0, r0, #1
	6010      	str	r0, [r2, #0]
	6810      	ldr	r0, [r2, #0]
	f040 0002 	orr.w	r0, r0, #2
	6010      	str	r0, [r2, #0]
	f240 002b 	movw	r0, #43	@ 0x2b
	f2c0 0000 	movt	r0, #0
	f7ff f997 	bl	<defmt::export::acquire_header_and_release>
	2500      	movs	r5, #0
	f640 0400 	movw	r4, #2048	@ 0x800
	f2c4 0502 	movt	r5, #16386	@ 0x4002
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	68e8      	ldr	r0, [r5, #12]
	f640 0208 	movw	r2, #2056	@ 0x808
	f2c4 0202 	movt	r2, #16386	@ 0x4002
	f641 0a00 	movw	sl, #6144	@ 0x1800
	f020 000c 	bic.w	r0, r0, #12
	60e8      	str	r0, [r5, #12]
	6828      	ldr	r0, [r5, #0]
	f2c4 0a02 	movt	sl, #16386	@ 0x4002
	230b      	movs	r3, #11
	2602      	movs	r6, #2
	f020 000c 	bic.w	r0, r0, #12
	6028      	str	r0, [r5, #0]
	68e8      	ldr	r0, [r5, #12]
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f04f 0800 	mov.w	r8, #0
	f50d 6b4a 	add.w	fp, sp, #3232	@ 0xca0
	f420 4040 	bic.w	r0, r0, #49152	@ 0xc000
	60e8      	str	r0, [r5, #12]
	6828      	ldr	r0, [r5, #0]
	f420 4040 	bic.w	r0, r0, #49152	@ 0xc000
	6028      	str	r0, [r5, #0]
	f64f 400c 	movw	r0, #64524	@ 0xfc0c
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
	5821      	ldr	r1, [r4, r0]
	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
	5021      	str	r1, [r4, r0]
	f8d5 1400 	ldr.w	r1, [r5, #1024]	@ 0x400
	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
	f8c5 1400 	str.w	r1, [r5, #1024]	@ 0x400
	6851      	ldr	r1, [r2, #4]
	f421 7140 	bic.w	r1, r1, #768	@ 0x300
	6051      	str	r1, [r2, #4]
	6821      	ldr	r1, [r4, #0]
	f421 7140 	bic.w	r1, r1, #768	@ 0x300
	6021      	str	r1, [r4, #0]
	6851      	ldr	r1, [r2, #4]
	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
	6051      	str	r1, [r2, #4]
	6821      	ldr	r1, [r4, #0]
	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
	6021      	str	r1, [r4, #0]
	f8da 100c 	ldr.w	r1, [sl, #12]
	f421 0140 	bic.w	r1, r1, #12582912	@ 0xc00000
	f8ca 100c 	str.w	r1, [sl, #12]
	f8da 1000 	ldr.w	r1, [sl]
	f421 0140 	bic.w	r1, r1, #12582912	@ 0xc00000
	f8ca 1000 	str.w	r1, [sl]
	f8da 100c 	ldr.w	r1, [sl, #12]
	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
	f8ca 100c 	str.w	r1, [sl, #12]
	f8da 1000 	ldr.w	r1, [sl]
	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
	f8ca 1000 	str.w	r1, [sl]
	6a29      	ldr	r1, [r5, #32]
	f363 210b 	bfi	r1, r3, #8, #4
	6229      	str	r1, [r5, #32]
	6829      	ldr	r1, [r5, #0]
	f366 1105 	bfi	r1, r6, #4, #2
	6029      	str	r1, [r5, #0]
	68a9      	ldr	r1, [r5, #8]
	f041 0130 	orr.w	r1, r1, #48	@ 0x30
	60a9      	str	r1, [r5, #8]
	6991      	ldr	r1, [r2, #24]
	f363 1107 	bfi	r1, r3, #4, #4
	6191      	str	r1, [r2, #24]
	6821      	ldr	r1, [r4, #0]
	f366 0183 	bfi	r1, r6, #2, #2
	6021      	str	r1, [r4, #0]
	6811      	ldr	r1, [r2, #0]
	f041 010c 	orr.w	r1, r1, #12
	6011      	str	r1, [r2, #0]
	f64f 4118 	movw	r1, #64536	@ 0xfc18
	f44f 1200 	mov.w	r2, #2097152	@ 0x200000
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
	5062      	str	r2, [r4, r1]
	5821      	ldr	r1, [r4, r0]
	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
	5021      	str	r1, [r4, r0]
	f64f 4004 	movw	r0, #64516	@ 0xfc04
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
	5821      	ldr	r1, [r4, r0]
	f021 0120 	bic.w	r1, r1, #32
	5021      	str	r1, [r4, r0]
	2101      	movs	r1, #1
	f8d5 0400 	ldr.w	r0, [r5, #1024]	@ 0x400
	f361 208b 	bfi	r0, r1, #10, #2
	f8c5 0400 	str.w	r0, [r5, #1024]	@ 0x400
	f8ce 82ec 	str.w	r8, [lr, #748]	@ 0x2ec
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f50b 60cd 	add.w	r0, fp, #1640	@ 0x668
	f240 51f2 	movw	r1, #1522	@ 0x5f2
	f8ce 82e8 	str.w	r8, [lr, #744]	@ 0x2e8
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8cd 8660 	str.w	r8, [sp, #1632]	@ 0x660
	f8ce 82e4 	str.w	r8, [lr, #740]	@ 0x2e4
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8cd 8664 	str.w	r8, [sp, #1636]	@ 0x664
	f8ce 82e0 	str.w	r8, [lr, #736]	@ 0x2e0
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8cd 8668 	str.w	r8, [sp, #1640]	@ 0x668
	f8ce 82dc 	str.w	r8, [lr, #732]	@ 0x2dc
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8cd 866c 	str.w	r8, [sp, #1644]	@ 0x66c
	f8ce 82d8 	str.w	r8, [lr, #728]	@ 0x2d8
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8cd 8670 	str.w	r8, [sp, #1648]	@ 0x670
	f8ce 82d4 	str.w	r8, [lr, #724]	@ 0x2d4
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8cd 8674 	str.w	r8, [sp, #1652]	@ 0x674
	f8cd 8678 	str.w	r8, [sp, #1656]	@ 0x678
	f8cd 867c 	str.w	r8, [sp, #1660]	@ 0x67c
	f8ce 82d0 	str.w	r8, [lr, #720]	@ 0x2d0
	9006      	str	r0, [sp, #24]
	f003 fb15 	bl	<__aeabi_memclr8>
	ae08      	add	r6, sp, #32
	f240 51f2 	movw	r1, #1522	@ 0x5f2
	f506 60d1 	add.w	r0, r6, #1672	@ 0x688
	9005      	str	r0, [sp, #20]
	f003 fb0d 	bl	<__aeabi_memclr8>
	4630      	mov	r0, r6
	2124      	movs	r1, #36	@ 0x24
	e9cd 8816 	strd	r8, r8, [sp, #88]	@ 0x58
	f8cd 8050 	str.w	r8, [sp, #80]	@ 0x50
	f8cd 8048 	str.w	r8, [sp, #72]	@ 0x48
	f003 fb03 	bl	<__aeabi_memclr8>
	f106 0048 	add.w	r0, r6, #72	@ 0x48
	f240 51f2 	movw	r1, #1522	@ 0x5f2
	9003      	str	r0, [sp, #12]
	f003 fafc 	bl	<__aeabi_memclr8>
	4658      	mov	r0, fp
	2124      	movs	r1, #36	@ 0x24
	f8cd 869c 	str.w	r8, [sp, #1692]	@ 0x69c
	f8cd 8698 	str.w	r8, [sp, #1688]	@ 0x698
	f8cd 8690 	str.w	r8, [sp, #1680]	@ 0x690
	f8cd 8688 	str.w	r8, [sp, #1672]	@ 0x688
	f8cd 8680 	str.w	r8, [sp, #1664]	@ 0x680
	f88d 8cd0 	strb.w	r8, [sp, #3280]	@ 0xcd0
	f8cd 8ccc 	str.w	r8, [sp, #3276]	@ 0xccc
	f8cd 8cc8 	str.w	r8, [sp, #3272]	@ 0xcc8
	f003 fae8 	bl	<__aeabi_memclr8>
	f10b 0038 	add.w	r0, fp, #56	@ 0x38
	f240 51f2 	movw	r1, #1522	@ 0x5f2
	9004      	str	r0, [sp, #16]
	f003 fae1 	bl	<__aeabi_memclr8>
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	210b      	movs	r1, #11
	2302      	movs	r3, #2
	220b      	movs	r2, #11
	f88e 8300 	strb.w	r8, [lr, #768]	@ 0x300
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8ce 82fc 	str.w	r8, [lr, #764]	@ 0x2fc
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8ce 82f8 	str.w	r8, [lr, #760]	@ 0x2f8
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8ce 82f0 	str.w	r8, [lr, #752]	@ 0x2f0
	6a28      	ldr	r0, [r5, #32]
	f361 1007 	bfi	r0, r1, #4, #4
	6228      	str	r0, [r5, #32]
	6828      	ldr	r0, [r5, #0]
	f363 0083 	bfi	r0, r3, #2, #2
	6028      	str	r0, [r5, #0]
	68a8      	ldr	r0, [r5, #8]
	f040 000c 	orr.w	r0, r0, #12
	60a8      	str	r0, [r5, #8]
	6a28      	ldr	r0, [r5, #32]
	f361 701f 	bfi	r0, r1, #28, #4
	6228      	str	r0, [r5, #32]
	6828      	ldr	r0, [r5, #0]
	f363 308f 	bfi	r0, r3, #14, #2
	6028      	str	r0, [r5, #0]
	68a8      	ldr	r0, [r5, #8]
	f440 4040 	orr.w	r0, r0, #49152	@ 0xc000
	60a8      	str	r0, [r5, #8]
	f8da 0024 	ldr.w	r0, [sl, #36]	@ 0x24
	f362 300f 	bfi	r0, r2, #12, #4
	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
	f8da 0000 	ldr.w	r0, [sl]
	f363 5097 	bfi	r0, r3, #22, #2
	f8ca 0000 	str.w	r0, [sl]
	f8da 0008 	ldr.w	r0, [sl, #8]
	f440 0040 	orr.w	r0, r0, #12582912	@ 0xc00000
	f8ca 0008 	str.w	r0, [sl, #8]
	f8da 0024 	ldr.w	r0, [sl, #36]	@ 0x24
	f362 5017 	bfi	r0, r2, #20, #4
	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
	f8da 0000 	ldr.w	r0, [sl]
	f363 609b 	bfi	r0, r3, #26, #2
	f8ca 0000 	str.w	r0, [sl]
	f8da 0008 	ldr.w	r0, [sl, #8]
	f040 6040 	orr.w	r0, r0, #201326592	@ 0xc000000
	f8ca 0008 	str.w	r0, [sl, #8]
	f64f 4024 	movw	r0, #64548	@ 0xfc24
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
	5821      	ldr	r1, [r4, r0]
	f362 5117 	bfi	r1, r2, #20, #4
	5021      	str	r1, [r4, r0]
	f8d5 0400 	ldr.w	r0, [r5, #1024]	@ 0x400
	f363 609b 	bfi	r0, r3, #26, #2
	f8c5 0400 	str.w	r0, [r5, #1024]	@ 0x400
	f64f 4008 	movw	r0, #64520	@ 0xfc08
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
	5821      	ldr	r1, [r4, r0]
	f041 6140 	orr.w	r1, r1, #201326592	@ 0xc000000
	5021      	str	r1, [r4, r0]
	f640 0108 	movw	r1, #2056	@ 0x808
	f2c4 0102 	movt	r1, #16386	@ 0x4002
	6988      	ldr	r0, [r1, #24]
	f362 4013 	bfi	r0, r2, #16, #4
	6188      	str	r0, [r1, #24]
	6820      	ldr	r0, [r4, #0]
	f363 2009 	bfi	r0, r3, #8, #2
	6020      	str	r0, [r4, #0]
	6808      	ldr	r0, [r1, #0]
	f440 7040 	orr.w	r0, r0, #768	@ 0x300
	6008      	str	r0, [r1, #0]
	6988      	ldr	r0, [r1, #24]
	f362 5017 	bfi	r0, r2, #20, #4
	6188      	str	r0, [r1, #24]
	6820      	ldr	r0, [r4, #0]
	f363 208b 	bfi	r0, r3, #10, #2
	6020      	str	r0, [r4, #0]
	6808      	ldr	r0, [r1, #0]
	f440 6040 	orr.w	r0, r0, #3072	@ 0xc00
	6008      	str	r0, [r1, #0]
	f003 fa59 	bl	<__primask_r>
	4680      	mov	r8, r0
	f003 fa49 	bl	<__cpsid>
	f8d9 0034 	ldr.w	r0, [r9, #52]	@ 0x34
	ea5f 71c8 	movs.w	r1, r8, lsl #31
	f440 4080 	orr.w	r0, r0, #16384	@ 0x4000
	f8c9 0034 	str.w	r0, [r9, #52]	@ 0x34
	f8d9 0020 	ldr.w	r0, [r9, #32]
	d126      	bne.n	<ip::__cortex_m_rt_main+0xa0c>
	0180      	lsls	r0, r0, #6
	f8dd 801c 	ldr.w	r8, [sp, #28]
	bf42      	ittt	mi
	f8d9 0020 	ldrmi.w	r0, [r9, #32]
	f020 7000 	bicmi.w	r0, r0, #33554432	@ 0x2000000
	f8c9 0020 	strmi.w	r0, [r9, #32]
	f643 0004 	movw	r0, #14340	@ 0x3804
	f2c4 0001 	movt	r0, #16385	@ 0x4001
	6801      	ldr	r1, [r0, #0]
	f441 0100 	orr.w	r1, r1, #8388608	@ 0x800000
	6001      	str	r1, [r0, #0]
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 6060 	orr.w	r0, r0, #234881024	@ 0xe000000
	f8c9 0020 	str.w	r0, [r9, #32]
	f8d9 0000 	ldr.w	r0, [r9]
	f040 7000 	orr.w	r0, r0, #33554432	@ 0x2000000
	f8c9 0000 	str.w	r0, [r9]
	f8d9 0000 	ldr.w	r0, [r9]
	f020 7000 	bic.w	r0, r0, #33554432	@ 0x2000000
	f8c9 0000 	str.w	r0, [r9]
	f003 fa1a 	bl	<__cpsie>
	e023      	b.n	<ip::__cortex_m_rt_main+0xa54>
	0180      	lsls	r0, r0, #6
	f8dd 801c 	ldr.w	r8, [sp, #28]
	bf42      	ittt	mi
	f8d9 0020 	ldrmi.w	r0, [r9, #32]
	f020 7000 	bicmi.w	r0, r0, #33554432	@ 0x2000000
	f8c9 0020 	strmi.w	r0, [r9, #32]
	f643 0004 	movw	r0, #14340	@ 0x3804
	f2c4 0001 	movt	r0, #16385	@ 0x4001
	6801      	ldr	r1, [r0, #0]
	f441 0100 	orr.w	r1, r1, #8388608	@ 0x800000
	6001      	str	r1, [r0, #0]
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 6060 	orr.w	r0, r0, #234881024	@ 0xe000000
	f8c9 0020 	str.w	r0, [r9, #32]
	f8d9 0000 	ldr.w	r0, [r9]
	f040 7000 	orr.w	r0, r0, #33554432	@ 0x2000000
	f8c9 0000 	str.w	r0, [r9]
	f8d9 0000 	ldr.w	r0, [r9]
	f020 7000 	bic.w	r0, r0, #33554432	@ 0x2000000
	f8c9 0000 	str.w	r0, [r9]
	f249 0418 	movw	r4, #36888	@ 0x9018
	e9dd 5c05 	ldrd	r5, ip, [sp, #20]
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	e9dd 6e03 	ldrd	r6, lr, [sp, #12]
	f854 0c18 	ldr.w	r0, [r4, #-24]
	f50d 6a4a 	add.w	sl, sp, #3232	@ 0xca0
	f10d 0b20 	add.w	fp, sp, #32
	f040 0001 	orr.w	r0, r0, #1
	f844 0c18 	str.w	r0, [r4, #-24]
	a806      	add	r0, sp, #24
	f500 5997 	add.w	r9, r0, #4832	@ 0x12e0
	f854 0c18 	ldr.w	r0, [r4, #-24]
	07c0      	lsls	r0, r0, #31
	d00c      	beq.n	<ip::__cortex_m_rt_main+0xaa0>
	f854 0c18 	ldr.w	r0, [r4, #-24]
	07c0      	lsls	r0, r0, #31
	bf1c      	itt	ne
	f854 0c18 	ldrne.w	r0, [r4, #-24]
	ea5f 70c0 	movsne.w	r0, r0, lsl #31
	d003      	beq.n	<ip::__cortex_m_rt_main+0xaa0>
	f854 0c18 	ldr.w	r0, [r4, #-24]
	07c0      	lsls	r0, r0, #31
	d1ee      	bne.n	<ip::__cortex_m_rt_main+0xa7e>
	6820      	ldr	r0, [r4, #0]
	217f      	movs	r1, #127	@ 0x7f
	f6cf 5100 	movt	r1, #64768	@ 0xfd00
	f240 52f2 	movw	r2, #1522	@ 0x5f2
	f040 60e4 	orr.w	r0, r0, #119537664	@ 0x7200000
	f50b 63c8 	add.w	r3, fp, #1600	@ 0x640
	f040 0084 	orr.w	r0, r0, #132	@ 0x84
	6020      	str	r0, [r4, #0]
	f854 0c18 	ldr.w	r0, [r4, #-24]
	4008      	ands	r0, r1
	f246 0180 	movw	r1, #24704	@ 0x6080
	f2c0 21c1 	movt	r1, #705	@ 0x2c1
	4308      	orrs	r0, r1
	f44f 4180 	mov.w	r1, #16384	@ 0x4000
	f844 0c18 	str.w	r0, [r4, #-24]
	9109      	str	r1, [sp, #36]	@ 0x24
	960a      	str	r6, [sp, #40]	@ 0x28
	9809      	ldr	r0, [sp, #36]	@ 0x24
	9611      	str	r6, [sp, #68]	@ 0x44
	f362 000b 	bfi	r0, r2, #0, #12
	9009      	str	r0, [sp, #36]	@ 0x24
	2001      	movs	r0, #1
	930b      	str	r3, [sp, #44]	@ 0x2c
	960a      	str	r6, [sp, #40]	@ 0x28
	9010      	str	r0, [sp, #64]	@ 0x40
	e9cd 0312 	strd	r0, r3, [sp, #72]	@ 0x48
	930b      	str	r3, [sp, #44]	@ 0x2c
	f04f 4300 	mov.w	r3, #2147483648	@ 0x80000000
	f3bf 8f5f 	dmb	sy
	9308      	str	r3, [sp, #32]
	f3bf 8f5f 	dmb	sy
	f8cd 1664 	str.w	r1, [sp, #1636]	@ 0x664
	f8cd 5668 	str.w	r5, [sp, #1640]	@ 0x668
	f8dd 1664 	ldr.w	r1, [sp, #1636]	@ 0x664
	f362 010b 	bfi	r1, r2, #0, #12
	f8cd 1664 	str.w	r1, [sp, #1636]	@ 0x664
	2100      	movs	r1, #0
	f8cd 166c 	str.w	r1, [sp, #1644]	@ 0x66c
	f8dd 2664 	ldr.w	r2, [sp, #1636]	@ 0x664
	f442 4200 	orr.w	r2, r2, #32768	@ 0x8000
	f8cd 2664 	str.w	r2, [sp, #1636]	@ 0x664
	f8cd 5668 	str.w	r5, [sp, #1640]	@ 0x668
	f8cd 5684 	str.w	r5, [sp, #1668]	@ 0x684
	f248 7500 	movw	r5, #34560	@ 0x8700
	f8cd 0680 	str.w	r0, [sp, #1664]	@ 0x680
	f2c4 0502 	movt	r5, #16386	@ 0x4002
	f8cd 168c 	str.w	r1, [sp, #1676]	@ 0x68c
	f8cd 0688 	str.w	r0, [sp, #1672]	@ 0x688
	f8cd 166c 	str.w	r1, [sp, #1644]	@ 0x66c
	f3bf 8f5f 	dmb	sy
	f8cd 3660 	str.w	r3, [sp, #1632]	@ 0x660
	f3bf 8f5f 	dmb	sy
	f844 bc0c 	str.w	fp, [r4, #-12]
	6822      	ldr	r2, [r4, #0]
	f042 0202 	orr.w	r2, r2, #2
	6022      	str	r2, [r4, #0]
	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
	f844 0c10 	str.w	r0, [r4, #-16]
	f844 0c10 	str.w	r0, [r4, #-16]
	f8cd 1ca0 	str.w	r1, [sp, #3232]	@ 0xca0
	f8cd 1ca4 	str.w	r1, [sp, #3236]	@ 0xca4
	f8cd 1ca8 	str.w	r1, [sp, #3240]	@ 0xca8
	f8cd 1cac 	str.w	r1, [sp, #3244]	@ 0xcac
	f8cd 1cb0 	str.w	r1, [sp, #3248]	@ 0xcb0
	f8cd 1cb4 	str.w	r1, [sp, #3252]	@ 0xcb4
	f8cd 1cb8 	str.w	r1, [sp, #3256]	@ 0xcb8
	f8cd 1cbc 	str.w	r1, [sp, #3260]	@ 0xcbc
	f8c2 12d0 	str.w	r1, [r2, #720]	@ 0x2d0
	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
	f8c2 12d4 	str.w	r1, [r2, #724]	@ 0x2d4
	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
	f8c2 12d8 	str.w	r1, [r2, #728]	@ 0x2d8
	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
	f8c2 12dc 	str.w	r1, [r2, #732]	@ 0x2dc
	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
	f8c2 12e0 	str.w	r1, [r2, #736]	@ 0x2e0
	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
	f8c2 12e4 	str.w	r1, [r2, #740]	@ 0x2e4
	f50a 62c6 	add.w	r2, sl, #1584	@ 0x630
	f8cd 2ccc 	str.w	r2, [sp, #3276]	@ 0xccc
	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
	f8c2 12e8 	str.w	r1, [r2, #744]	@ 0x2e8
	f8cd ecc8 	str.w	lr, [sp, #3272]	@ 0xcc8
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f88e 0300 	strb.w	r0, [lr, #768]	@ 0x300
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8ce 12ec 	str.w	r1, [lr, #748]	@ 0x2ec
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8c9 c000 	str.w	ip, [r9]
	f8ce 12fc 	str.w	r1, [lr, #764]	@ 0x2fc
	f44f 7100 	mov.w	r1, #512	@ 0x200
	f844 ac08 	str.w	sl, [r4, #-8]
	f3bf 8f5f 	dmb	sy
	6820      	ldr	r0, [r4, #0]
	f440 5000 	orr.w	r0, r0, #8192	@ 0x2000
	6020      	str	r0, [r4, #0]
	f64f 103c 	movw	r0, #63804	@ 0xf93c
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
	5029      	str	r1, [r5, r0]
	ea5f 0058 	movs.w	r0, r8, lsr #1
	d105      	bne.n	<ip::__cortex_m_rt_main+0xc12>
	f24b 00e8 	movw	r0, #45288	@ 0xb0e8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fe fc74 	bl	<core::panicking::panic_const::panic_const_div_by_zero>
	f06f 4100 	mvn.w	r1, #2147483648	@ 0x80000000
	eb01 0298 	add.w	r2, r1, r8, lsr #2
	fbb2 f2f0 	udiv	r2, r2, r0
	f248 7604 	movw	r6, #34564	@ 0x8704
	fbb1 f0f2 	udiv	r0, r1, r2
	f04f 31ff 	mov.w	r1, #4294967295	@ 0xffffffff
	fba0 0101 	umull	r0, r1, r0, r1
	f242 1303 	movw	r3, #8451	@ 0x2103
	b2d2      	uxtb	r2, r2
	f2c4 0602 	movt	r6, #16386	@ 0x4002
	602b      	str	r3, [r5, #0]
	2300      	movs	r3, #0
	6032      	str	r2, [r6, #0]
	4642      	mov	r2, r8
	f003 ff4a 	bl	<__aeabi_uldivmod>
	6170      	str	r0, [r6, #20]
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	d509      	bpl.n	<ip::__cortex_m_rt_main+0xc60>
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	bf44      	itt	mi
	6828      	ldrmi	r0, [r5, #0]
	ea5f 6080 	movsmi.w	r0, r0, lsl #26
	d502      	bpl.n	<ip::__cortex_m_rt_main+0xc60>
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	d4f2      	bmi.n	<ip::__cortex_m_rt_main+0xc46>
	6828      	ldr	r0, [r5, #0]
	f040 0020 	orr.w	r0, r0, #32
	6028      	str	r0, [r5, #0]
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	d509      	bpl.n	<ip::__cortex_m_rt_main+0xc82>
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	bf44      	itt	mi
	6828      	ldrmi	r0, [r5, #0]
	ea5f 6080 	movsmi.w	r0, r0, lsl #26
	d502      	bpl.n	<ip::__cortex_m_rt_main+0xc82>
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	d4f2      	bmi.n	<ip::__cortex_m_rt_main+0xc68>
	2000      	movs	r0, #0
	60f0      	str	r0, [r6, #12]
	6130      	str	r0, [r6, #16]
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	d509      	bpl.n	<ip::__cortex_m_rt_main+0xca2>
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	bf44      	itt	mi
	6828      	ldrmi	r0, [r5, #0]
	ea5f 7040 	movsmi.w	r0, r0, lsl #29
	d502      	bpl.n	<ip::__cortex_m_rt_main+0xca2>
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	d4f2      	bmi.n	<ip::__cortex_m_rt_main+0xc88>
	6828      	ldr	r0, [r5, #0]
	f040 0004 	orr.w	r0, r0, #4
	6028      	str	r0, [r5, #0]
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	d509      	bpl.n	<ip::__cortex_m_rt_main+0xcc4>
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	bf44      	itt	mi
	6828      	ldrmi	r0, [r5, #0]
	ea5f 7040 	movsmi.w	r0, r0, lsl #29
	d502      	bpl.n	<ip::__cortex_m_rt_main+0xcc4>
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	d4f2      	bmi.n	<ip::__cortex_m_rt_main+0xcaa>
	f647 0040 	movw	r0, #30784	@ 0x7840
	f2c0 107d 	movt	r0, #381	@ 0x17d
	4580      	cmp	r8, r0
	d260      	bcs.n	<ip::__cortex_m_rt_main+0xd92>
	6820      	ldr	r0, [r4, #0]
	f420 5000 	bic.w	r0, r0, #8192	@ 0x2000
	6020      	str	r0, [r4, #0]
	f854 0c04 	ldr.w	r0, [r4, #-4]
	f3c0 5002 	ubfx	r0, r0, #20, #3
	3801      	subs	r0, #1
	2802      	cmp	r0, #2
	d814      	bhi.n	<ip::__cortex_m_rt_main+0xd10>
	f854 0c04 	ldr.w	r0, [r4, #-4]
	f3c0 5002 	ubfx	r0, r0, #20, #3
	3801      	subs	r0, #1
	2802      	cmp	r0, #2
	bf9f      	itttt	ls
	f854 0c04 	ldrls.w	r0, [r4, #-4]
	f3c0 5002 	ubfxls	r0, r0, #20, #3
	3801      	subls	r0, #1
	2802      	cmpls	r0, #2
	d806      	bhi.n	<ip::__cortex_m_rt_main+0xd10>
	f854 0c04 	ldr.w	r0, [r4, #-4]
	f3c0 5002 	ubfx	r0, r0, #20, #3
	3801      	subs	r0, #1
	2803      	cmp	r0, #3
	d3e3      	bcc.n	<ip::__cortex_m_rt_main+0xcd8>
	6820      	ldr	r0, [r4, #0]
	f020 0002 	bic.w	r0, r0, #2
	6020      	str	r0, [r4, #0]
	2001      	movs	r0, #1
	f854 1c04 	ldr.w	r1, [r4, #-4]
	f3c1 4142 	ubfx	r1, r1, #17, #3
	2907      	cmp	r1, #7
	d825      	bhi.n	<ip::__cortex_m_rt_main+0xd72>
	fa00 f101 	lsl.w	r1, r0, r1
	f011 0faa 	tst.w	r1, #170	@ 0xaa
	d020      	beq.n	<ip::__cortex_m_rt_main+0xd72>
	f854 1c04 	ldr.w	r1, [r4, #-4]
	f3c1 4142 	ubfx	r1, r1, #17, #3
	2907      	cmp	r1, #7
	d81a      	bhi.n	<ip::__cortex_m_rt_main+0xd72>
	fa00 f101 	lsl.w	r1, r0, r1
	f011 0faa 	tst.w	r1, #170	@ 0xaa
	d015      	beq.n	<ip::__cortex_m_rt_main+0xd72>
	f854 1c04 	ldr.w	r1, [r4, #-4]
	f3c1 4142 	ubfx	r1, r1, #17, #3
	2907      	cmp	r1, #7
	d80f      	bhi.n	<ip::__cortex_m_rt_main+0xd72>
	fa00 f101 	lsl.w	r1, r0, r1
	f011 0faa 	tst.w	r1, #170	@ 0xaa
	d00a      	beq.n	<ip::__cortex_m_rt_main+0xd72>
	f854 1c04 	ldr.w	r1, [r4, #-4]
	f3c1 4142 	ubfx	r1, r1, #17, #3
	2907      	cmp	r1, #7
	d804      	bhi.n	<ip::__cortex_m_rt_main+0xd72>
	fa00 f101 	lsl.w	r1, r0, r1
	f011 0faa 	tst.w	r1, #170	@ 0xaa
	d1d3      	bne.n	<ip::__cortex_m_rt_main+0xd1a>
	f64a 1010 	movw	r0, #43280	@ 0xa910
	f64a 1200 	movw	r2, #43264	@ 0xa900
	f64a 133c 	movw	r3, #43324	@ 0xa93c
	f1a7 0111 	sub.w	r1, r7, #17
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f7fe fca5 	bl	<core::result::unwrap_failed>
	f640 60c0 	movw	r0, #3776	@ 0xec0
	f2c0 2016 	movt	r0, #534	@ 0x216
	4580      	cmp	r8, r0
	d201      	bcs.n	<ip::__cortex_m_rt_main+0xda2>
	2008      	movs	r0, #8
	e022      	b.n	<ip::__cortex_m_rt_main+0xde8>
	f248 7000 	movw	r0, #34560	@ 0x8700
	f2c0 3093 	movt	r0, #915	@ 0x393
	4580      	cmp	r8, r0
	d201      	bcs.n	<ip::__cortex_m_rt_main+0xdb2>
	200c      	movs	r0, #12
	e01a      	b.n	<ip::__cortex_m_rt_main+0xde8>
	f24e 1000 	movw	r0, #57600	@ 0xe100
	f2c0 50f5 	movt	r0, #1525	@ 0x5f5
	4580      	cmp	r8, r0
	d20c      	bcs.n	<ip::__cortex_m_rt_main+0xdd8>
	2000      	movs	r0, #0
	e012      	b.n	<ip::__cortex_m_rt_main+0xde8>
	f1a0 0328 	sub.w	r3, r0, #40	@ 0x28
	2b38      	cmp	r3, #56	@ 0x38
	f080 8341 	bcs.w	<ip::__cortex_m_rt_main+0x144e>
	ed9f 2acf 	vldr	s4, [pc, #828]	@ <ip::__cortex_m_rt_main+0x110c>
	f04f 0ec0 	mov.w	lr, #192	@ 0xc0
	f7ff ba09 	b.w	<ip::__cortex_m_rt_main+0x1ea>
	f24d 1180 	movw	r1, #53632	@ 0xd180
	2010      	movs	r0, #16
	f6c0 01f0 	movt	r1, #2288	@ 0x8f0
	4588      	cmp	r8, r1
	bf38      	it	cc
	2004      	movcc	r0, #4
	f64f 1110 	movw	r1, #63760	@ 0xf910
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
	f50d 56c9 	add.w	r6, sp, #6432	@ 0x1920
	586a      	ldr	r2, [r5, r1]
	f022 021c 	bic.w	r2, r2, #28
	4310      	orrs	r0, r2
	5068      	str	r0, [r5, r1]
	f64f 1000 	movw	r0, #63744	@ 0xf900
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
	f644 628c 	movw	r2, #20108	@ 0x4e8c
	5829      	ldr	r1, [r5, r0]
	f2c0 2200 	movt	r2, #512	@ 0x200
	4311      	orrs	r1, r2
	5029      	str	r1, [r5, r0]
	f64f 1104 	movw	r1, #63748	@ 0xf904
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
	586a      	ldr	r2, [r5, r1]
	f042 4200 	orr.w	r2, r2, #2147483648	@ 0x80000000
	f042 0201 	orr.w	r2, r2, #1
	506a      	str	r2, [r5, r1]
	f64f 1118 	movw	r1, #63768	@ 0xf918
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
	586a      	ldr	r2, [r5, r1]
	f2c0 1200 	movt	r2, #256	@ 0x100
	506a      	str	r2, [r5, r1]
	f64f 210c 	movw	r1, #64012	@ 0xfa0c
	2260      	movs	r2, #96	@ 0x60
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
	f2c0 0202 	movt	r2, #2
	506a      	str	r2, [r5, r1]
	f64f 2110 	movw	r1, #64016	@ 0xfa10
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
	f44f 1203 	mov.w	r2, #2146304	@ 0x20c000
	506a      	str	r2, [r5, r1]
	586a      	ldr	r2, [r5, r1]
	f442 1200 	orr.w	r2, r2, #2097152	@ 0x200000
	506a      	str	r2, [r5, r1]
	f04f 3203 	mov.w	r2, #50529027	@ 0x3030303
	5829      	ldr	r1, [r5, r0]
	f441 4190 	orr.w	r1, r1, #18432	@ 0x4800
	5029      	str	r1, [r5, r0]
	2500      	movs	r5, #0
	2002      	movs	r0, #2
	f509 61c2 	add.w	r1, r9, #1552	@ 0x610
	e881 0421 	stmia.w	r1, {r0, r5, sl}
	f24e 0110 	movw	r1, #57360	@ 0xe010
	f2ce 0100 	movt	r1, #57344	@ 0xe000
	f8c9 5624 	str.w	r5, [r9, #1572]	@ 0x624
	f8c9 5620 	str.w	r5, [r9, #1568]	@ 0x620
	f8c9 061c 	str.w	r0, [r9, #1564]	@ 0x61c
	f8c9 b60c 	str.w	fp, [r9, #1548]	@ 0x60c
	6860      	ldr	r0, [r4, #4]
	f440 3080 	orr.w	r0, r0, #65536	@ 0x10000
	f040 0041 	orr.w	r0, r0, #65	@ 0x41
	6060      	str	r0, [r4, #4]
	f04f 5000 	mov.w	r0, #536870912	@ 0x20000000
	2401      	movs	r4, #1
	f8c1 00f4 	str.w	r0, [r1, #244]	@ 0xf4
	f64e 71be 	movw	r1, #61374	@ 0xefbe
	f64e 7039 	movw	r0, #61241	@ 0xef39
	f8ae 1a8c 	strh.w	r1, [lr, #2700]	@ 0xa8c
	2100      	movs	r1, #0
	f2c7 5090 	movt	r0, #30096	@ 0x7590
	f6ca 51de 	movt	r1, #44510	@ 0xadde
	f8c9 0640 	str.w	r0, [r9, #1600]	@ 0x640
	f8c9 1790 	str.w	r1, [r9, #1936]	@ 0x790
	f240 51f2 	movw	r1, #1522	@ 0x5f2
	f106 0020 	add.w	r0, r6, #32
	f8c9 1634 	str.w	r1, [r9, #1588]	@ 0x634
	21c0      	movs	r1, #192	@ 0xc0
	f8c9 5710 	str.w	r5, [r9, #1808]	@ 0x710
	f8c9 5714 	str.w	r5, [r9, #1812]	@ 0x714
	f8c9 5718 	str.w	r5, [r9, #1816]	@ 0x718
	f8c9 462c 	str.w	r4, [r9, #1580]	@ 0x62c
	f8c9 2630 	str.w	r2, [r9, #1584]	@ 0x630
	f8c9 5638 	str.w	r5, [r9, #1592]	@ 0x638
	f8c9 563c 	str.w	r5, [r9, #1596]	@ 0x63c
	f8c9 5644 	str.w	r5, [r9, #1604]	@ 0x644
	f8c9 4628 	str.w	r4, [r9, #1576]	@ 0x628
	f002 ffb1 	bl	<__aeabi_memclr8>
	f506 7080 	add.w	r0, r6, #256	@ 0x100
	2164      	movs	r1, #100	@ 0x64
	f002 ffac 	bl	<__aeabi_memclr8>
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	2018      	movs	r0, #24
	f8c9 4718 	str.w	r4, [r9, #1816]	@ 0x718
	f88e 0a18 	strb.w	r0, [lr, #2584]	@ 0xa18
	f64f 70fe 	movw	r0, #65534	@ 0xfffe
	f2c0 00ff 	movt	r0, #255	@ 0xff
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	300c      	adds	r0, #12
	f8c9 071c 	str.w	r0, [r9, #1820]	@ 0x71c
	f000 01f0 	and.w	r1, r0, #240	@ 0xf0
	f88e 5a8e 	strb.w	r5, [lr, #2702]	@ 0xa8e
	29e0      	cmp	r1, #224	@ 0xe0
	f8c9 5708 	str.w	r5, [r9, #1800]	@ 0x708
	bf18      	it	ne
	f110 0101 	addsne.w	r1, r0, #1
	d11b      	bne.n	<ip::__cortex_m_rt_main+0xf74>
	f8c9 08f0 	str.w	r0, [r9, #2288]	@ 0x8f0
	f647 4069 	movw	r0, #31849	@ 0x7c69
	f10d 0e08 	add.w	lr, sp, #8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f50e 51df 	add.w	r1, lr, #7136	@ 0x1be0
	f10d 0e08 	add.w	lr, sp, #8
	f64a 52bc 	movw	r2, #44476	@ 0xadbc
	f8c9 1af0 	str.w	r1, [r9, #2800]	@ 0xaf0
	f8c9 0af4 	str.w	r0, [r9, #2804]	@ 0xaf4
	f649 205b 	movw	r0, #39515	@ 0x9a5b
	f50e 51ef 	add.w	r1, lr, #7648	@ 0x1de0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd ff68 	bl	<core::panicking::panic_fmt>
	f10d 0e10 	add.w	lr, sp, #16
	f24a 71a8 	movw	r1, #42920	@ 0xa7a8
	f50e 58d4 	add.w	r8, lr, #6784	@ 0x1a80
	f6c0 0100 	movt	r1, #2048	@ 0x800
	f44f 72a8 	mov.w	r2, #336	@ 0x150
	4640      	mov	r0, r8
	f003 fcef 	bl	<__aeabi_memcpy8>
	f10d 0e08 	add.w	lr, sp, #8
	f44f 7100 	mov.w	r1, #512	@ 0x200
	f50e 56df 	add.w	r6, lr, #7136	@ 0x1be0
	f8c9 88e8 	str.w	r8, [r9, #2280]	@ 0x8e8
	f8c9 48ec 	str.w	r4, [r9, #2284]	@ 0x8ec
	f44f 7b00 	mov.w	fp, #512	@ 0x200
	4630      	mov	r0, r6
	f003 fc83 	bl	<__aeabi_memclr4>
	f10d 0e08 	add.w	lr, sp, #8
	f44f 7100 	mov.w	r1, #512	@ 0x200
	f50e 50ef 	add.w	r0, lr, #7648	@ 0x1de0
	f003 fc7b 	bl	<__aeabi_memclr4>
	f8d9 0798 	ldr.w	r0, [r9, #1944]	@ 0x798
	f04f 0a00 	mov.w	sl, #0
	f8d9 179c 	ldr.w	r1, [r9, #1948]	@ 0x79c
	f080 0003 	eor.w	r0, r0, #3
	e947 aa0e 	strd	sl, sl, [r7, #-56]	@ 0x38
	4308      	orrs	r0, r1
	e947 aa10 	strd	sl, sl, [r7, #-64]	@ 0x40
	e947 aa12 	strd	sl, sl, [r7, #-72]	@ 0x48
	e947 aa14 	strd	sl, sl, [r7, #-80]	@ 0x50
	e947 aa0a 	strd	sl, sl, [r7, #-40]	@ 0x28
	e947 aa08 	strd	sl, sl, [r7, #-32]
	e947 aa18 	strd	sl, sl, [r7, #-96]	@ 0x60
	e947 aa16 	strd	sl, sl, [r7, #-88]	@ 0x58
	f040 8223 	bne.w	<ip::__cortex_m_rt_main+0x1438>
	f242 7010 	movw	r0, #10000	@ 0x2710
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f44f 7196 	mov.w	r1, #300	@ 0x12c
	f8c9 0800 	str.w	r0, [r9, #2048]	@ 0x800
	2064      	movs	r0, #100	@ 0x64
	f8c9 17b8 	str.w	r1, [r9, #1976]	@ 0x7b8
	f1a7 0130 	sub.w	r1, r7, #48	@ 0x30
	f88e aab8 	strb.w	sl, [lr, #2744]	@ 0xab8
	f8c9 47f8 	str.w	r4, [r9, #2040]	@ 0x7f8
	46de      	mov	lr, fp
	f8c9 a7fc 	str.w	sl, [r9, #2044]	@ 0x7fc
	46b3      	mov	fp, r6
	f8c9 a804 	str.w	sl, [r9, #2052]	@ 0x804
	f108 0c78 	add.w	ip, r8, #120	@ 0x78
	f8c9 a808 	str.w	sl, [r9, #2056]	@ 0x808
	f8c9 a80c 	str.w	sl, [r9, #2060]	@ 0x80c
	f8c9 a7e8 	str.w	sl, [r9, #2024]	@ 0x7e8
	f8c9 a7ec 	str.w	sl, [r9, #2028]	@ 0x7ec
	f8c9 a7d8 	str.w	sl, [r9, #2008]	@ 0x7d8
	f8c9 a7dc 	str.w	sl, [r9, #2012]	@ 0x7dc
	f8c9 a7c8 	str.w	sl, [r9, #1992]	@ 0x7c8
	f8c9 a7cc 	str.w	sl, [r9, #1996]	@ 0x7cc
	f8c9 a798 	str.w	sl, [r9, #1944]	@ 0x798
	f8c9 a79c 	str.w	sl, [r9, #1948]	@ 0x79c
	f8c9 07bc 	str.w	r0, [r9, #1980]	@ 0x7bc
	f8c9 a7b0 	str.w	sl, [r9, #1968]	@ 0x7b0
	e891 007d 	ldmia.w	r1, {r0, r2, r3, r4, r5, r6}
	f1a7 0150 	sub.w	r1, r7, #80	@ 0x50
	e88c 007d 	stmia.w	ip, {r0, r2, r3, r4, r5, r6}
	a802      	add	r0, sp, #8
	f500 50ef 	add.w	r0, r0, #7648	@ 0x1de0
	f8c9 e854 	str.w	lr, [r9, #2132]	@ 0x854
	f8c9 e864 	str.w	lr, [r9, #2148]	@ 0x864
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8c9 0860 	str.w	r0, [r9, #2144]	@ 0x860
	f108 00d8 	add.w	r0, r8, #216	@ 0xd8
	f88e ab42 	strb.w	sl, [lr, #2882]	@ 0xb42
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8c9 b850 	str.w	fp, [r9, #2128]	@ 0x850
	f240 0840 	movw	r8, #64	@ 0x40
	f8ae ab40 	strh.w	sl, [lr, #2880]	@ 0xb40
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8c9 a858 	str.w	sl, [r9, #2136]	@ 0x858
	f2c2 0800 	movt	r8, #8192	@ 0x2000
	f8c9 a85c 	str.w	sl, [r9, #2140]	@ 0x85c
	f8c9 a868 	str.w	sl, [r9, #2152]	@ 0x868
	f8c9 a86c 	str.w	sl, [r9, #2156]	@ 0x86c
	f8c9 a840 	str.w	sl, [r9, #2112]	@ 0x840
	f8c9 a838 	str.w	sl, [r9, #2104]	@ 0x838
	f8c9 a830 	str.w	sl, [r9, #2096]	@ 0x830
	c96c      	ldmia	r1!, {r2, r3, r5, r6}
	c06c      	stmia	r0!, {r2, r3, r5, r6}
	e891 006c 	ldmia.w	r1, {r2, r3, r5, r6}
	c06c      	stmia	r0!, {r2, r3, r5, r6}
	f1a7 0360 	sub.w	r3, r7, #96	@ 0x60
	2601      	movs	r6, #1
	f88e abc8 	strb.w	sl, [lr, #3016]	@ 0xbc8
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8c9 68ca 	str.w	r6, [r9, #2250]	@ 0x8ca
	f44f 7606 	mov.w	r6, #536	@ 0x218
	cb0f      	ldmia	r3, {r0, r1, r2, r3}
	f8c9 68a8 	str.w	r6, [r9, #2216]	@ 0x8a8
	464e      	mov	r6, r9
	f8c9 a8c6 	str.w	sl, [r9, #2246]	@ 0x8c6
	f8c9 a8c2 	str.w	sl, [r9, #2242]	@ 0x8c2
	f8c9 a8e0 	str.w	sl, [r9, #2272]	@ 0x8e0
	f8ae abac 	strh.w	sl, [lr, #2988]	@ 0xbac
	f8c9 a8b0 	str.w	sl, [r9, #2224]	@ 0x8b0
	f8c9 a890 	str.w	sl, [r9, #2192]	@ 0x890
	f8c9 a894 	str.w	sl, [r9, #2196]	@ 0x894
	f8c9 0898 	str.w	r0, [r9, #2200]	@ 0x898
	f8c9 189c 	str.w	r1, [r9, #2204]	@ 0x89c
	f8c9 28a0 	str.w	r2, [r9, #2208]	@ 0x8a0
	f8c9 38a4 	str.w	r3, [r9, #2212]	@ 0x8a4
	f8c9 a8ac 	str.w	sl, [r9, #2220]	@ 0x8ac
	e09b      	b.n	<ip::__cortex_m_rt_main+0x1244>
	42800000 	.word	0x42800000
	2064      	movs	r0, #100	@ 0x64
	f44f 7196 	mov.w	r1, #300	@ 0x12c
	e9c5 1008 	strd	r1, r0, [r5, #32]
	f105 00d0 	add.w	r0, r5, #208	@ 0xd0
	2128      	movs	r1, #40	@ 0x28
	e9c5 aa00 	strd	sl, sl, [r5]
	f8c5 a018 	str.w	sl, [r5, #24]
	e9c5 aa20 	strd	sl, sl, [r5, #128]	@ 0x80
	e9c5 aa22 	strd	sl, sl, [r5, #136]	@ 0x88
	f885 a130 	strb.w	sl, [r5, #304]	@ 0x130
	e9c5 aa30 	strd	sl, sl, [r5, #192]	@ 0xc0
	f885 a028 	strb.w	sl, [r5, #40]	@ 0x28
	f8d5 40bc 	ldr.w	r4, [r5, #188]	@ 0xbc
	f8a5 a12e 	strh.w	sl, [r5, #302]	@ 0x12e
	f8c5 a098 	str.w	sl, [r5, #152]	@ 0x98
	f885 a12c 	strb.w	sl, [r5, #300]	@ 0x12c
	e9c5 aa40 	strd	sl, sl, [r5, #256]	@ 0x100
	e9c5 aa42 	strd	sl, sl, [r5, #264]	@ 0x108
	f002 fe83 	bl	<__aeabi_memclr8>
	2050      	movs	r0, #80	@ 0x50
	2210      	movs	r2, #16
	e9c5 0a2c 	strd	r0, sl, [r5, #176]	@ 0xb0
	f44f 7006 	mov.w	r0, #536	@ 0x218
	f8c5 0110 	str.w	r0, [r5, #272]	@ 0x110
	fab4 f084 	clz	r0, r4
	f1c0 0110 	rsb	r1, r0, #16
	4282      	cmp	r2, r0
	f240 002c 	movw	r0, #44	@ 0x2c
	f04f 0901 	mov.w	r9, #1
	f2c0 0000 	movt	r0, #0
	e9c5 aa3e 	strd	sl, sl, [r5, #248]	@ 0xf8
	e9c5 aa1c 	strd	sl, sl, [r5, #112]	@ 0x70
	e9c5 aa14 	strd	sl, sl, [r5, #80]	@ 0x50
	f885 9133 	strb.w	r9, [r5, #307]	@ 0x133
	f8a5 a11c 	strh.w	sl, [r5, #284]	@ 0x11c
	bf38      	it	cc
	4651      	movcc	r1, sl
	f885 1134 	strb.w	r1, [r5, #308]	@ 0x134
	f7fe fc0d 	bl	<defmt::export::acquire_and_header>
	f240 0007 	movw	r0, #7
	f1a7 0414 	sub.w	r4, r7, #20
	f2c0 0000 	movt	r0, #0
	2102      	movs	r1, #2
	f827 0c14 	strh.w	r0, [r7, #-20]
	4620      	mov	r0, r4
	f7fe fdb3 	bl	<_defmt_write>
	f240 0032 	movw	r0, #50	@ 0x32
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f827 0c14 	strh.w	r0, [r7, #-20]
	4620      	mov	r0, r4
	f7fe fda9 	bl	<_defmt_write>
	f240 0502 	movw	r5, #2
	4620      	mov	r0, r4
	f2c0 0500 	movt	r5, #0
	2102      	movs	r1, #2
	f827 5c14 	strh.w	r5, [r7, #-20]
	f7fe fd9f 	bl	<_defmt_write>
	200a      	movs	r0, #10
	2101      	movs	r1, #1
	f807 0c14 	strb.w	r0, [r7, #-20]
	4620      	mov	r0, r4
	f7fe fd98 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f827 5c14 	strh.w	r5, [r7, #-20]
	f7fe fd92 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2101      	movs	r1, #1
	f807 ac14 	strb.w	sl, [r7, #-20]
	f7fe fd8c 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f827 5c14 	strh.w	r5, [r7, #-20]
	f7fe fd86 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2101      	movs	r1, #1
	f807 ac14 	strb.w	sl, [r7, #-20]
	f7fe fd80 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f827 5c14 	strh.w	r5, [r7, #-20]
	f7fe fd7a 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2101      	movs	r1, #1
	f807 9c14 	strb.w	r9, [r7, #-20]
	f7fe fd74 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f827 ac14 	strh.w	sl, [r7, #-20]
	f7fe fd6e 	bl	<_defmt_write>
	f7fe fcf4 	bl	<_defmt_release>
	f002 fe06 	bl	<__primask_r>
	4604      	mov	r4, r0
	f002 fdf6 	bl	<__cpsid>
	07e0      	lsls	r0, r4, #31
	e9d8 5902 	ldrd	r5, r9, [r8, #8]
	bf08      	it	eq
	f002 fdf2 	bleq	<__cpsie>
	f002 fdfb 	bl	<__primask_r>
	4604      	mov	r4, r0
	f002 fdeb 	bl	<__cpsid>
	07e0      	lsls	r0, r4, #31
	bf08      	it	eq
	f002 fde9 	bleq	<__cpsie>
	f44f 717a 	mov.w	r1, #1000	@ 0x3e8
	f10d 0e04 	add.w	lr, sp, #4
	fba5 b001 	umull	fp, r0, r5, r1
	fb09 0401 	mla	r4, r9, r1, r0
	f50d 50df 	add.w	r0, sp, #7136	@ 0x1be0
	f50e 51c8 	add.w	r1, lr, #6400	@ 0x1900
	e9cd 1000 	strd	r1, r0, [sp]
	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
	465a      	mov	r2, fp
	4623      	mov	r3, r4
	f7fb f9f6 	bl	<smoltcp::iface::interface::Interface::poll>
	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
	2900      	cmp	r1, #0
	f000 816f 	beq.w	<ip::__cortex_m_rt_main+0x157c>
	f8d6 58e8 	ldr.w	r5, [r6, #2280]	@ 0x8e8
	6828      	ldr	r0, [r5, #0]
	43c1      	mvns	r1, r0
	0789      	lsls	r1, r1, #30
	f000 80ff 	beq.w	<ip::__cortex_m_rt_main+0x14aa>
	2802      	cmp	r0, #2
	f000 8107 	beq.w	<ip::__cortex_m_rt_main+0x14c0>
	f895 0133 	ldrb.w	r0, [r5, #307]	@ 0x133
	2800      	cmp	r0, #0
	bf18      	it	ne
	280a      	cmpne	r0, #10
	f43f af28 	beq.w	<ip::__cortex_m_rt_main+0x1110>
	2807      	cmp	r0, #7
	bf18      	it	ne
	2804      	cmpne	r0, #4
	d1bd      	bne.n	<ip::__cortex_m_rt_main+0x1244>
	f8d5 60d4 	ldr.w	r6, [r5, #212]	@ 0xd4
	2e00      	cmp	r6, #0
	bf08      	it	eq
	f8c5 a0d0 	streq.w	sl, [r5, #208]	@ 0xd0
	f8d5 90cc 	ldr.w	r9, [r5, #204]	@ 0xcc
	f1b9 0f00 	cmp.w	r9, #0
	d007      	beq.n	<ip::__cortex_m_rt_main+0x12ee>
	f8d5 00d0 	ldr.w	r0, [r5, #208]	@ 0xd0
	4430      	add	r0, r6
	fbb0 f1f9 	udiv	r1, r0, r9
	fb01 0019 	mls	r0, r1, r9, r0
	e000      	b.n	<ip::__cortex_m_rt_main+0x12f0>
	2000      	movs	r0, #0
	eba9 0806 	sub.w	r8, r9, r6
	eba9 0100 	sub.w	r1, r9, r0
	4541      	cmp	r1, r8
	bf38      	it	cc
	4688      	movcc	r8, r1
	eb18 0100 	adds.w	r1, r8, r0
	f080 80e8 	bcs.w	<ip::__cortex_m_rt_main+0x14d6>
	4549      	cmp	r1, r9
	f200 80e5 	bhi.w	<ip::__cortex_m_rt_main+0x14d6>
	f8d5 10c8 	ldr.w	r1, [r5, #200]	@ 0xc8
	f1b8 0f06 	cmp.w	r8, #6
	bf28      	it	cs
	f04f 0806 	movcs.w	r8, #6
	9106      	str	r1, [sp, #24]
	4408      	add	r0, r1
	f64a 01f8 	movw	r1, #43256	@ 0xa8f8
	f6c0 0100 	movt	r1, #2048	@ 0x800
	4642      	mov	r2, r8
	f003 fb1a 	bl	<__aeabi_memcpy>
	eb18 0206 	adds.w	r2, r8, r6
	bf08      	it	eq
	f8c5 a0d0 	streq.w	sl, [r5, #208]	@ 0xd0
	f1b9 0f00 	cmp.w	r9, #0
	9407      	str	r4, [sp, #28]
	d007      	beq.n	<ip::__cortex_m_rt_main+0x134e>
	f8d5 00d0 	ldr.w	r0, [r5, #208]	@ 0xd0
	4410      	add	r0, r2
	fbb0 f1f9 	udiv	r1, r0, r9
	fb01 0019 	mls	r0, r1, r9, r0
	e000      	b.n	<ip::__cortex_m_rt_main+0x1350>
	2000      	movs	r0, #0
	eba9 0402 	sub.w	r4, r9, r2
	eba9 0100 	sub.w	r1, r9, r0
	42a1      	cmp	r1, r4
	bf38      	it	cc
	460c      	movcc	r4, r1
	1821      	adds	r1, r4, r0
	f080 80b9 	bcs.w	<ip::__cortex_m_rt_main+0x14d6>
	4549      	cmp	r1, r9
	f200 80b6 	bhi.w	<ip::__cortex_m_rt_main+0x14d6>
	f1c8 0106 	rsb	r1, r8, #6
	46b1      	mov	r9, r6
	42a1      	cmp	r1, r4
	bf38      	it	cc
	460c      	movcc	r4, r1
	9906      	ldr	r1, [sp, #24]
	4616      	mov	r6, r2
	4622      	mov	r2, r4
	4408      	add	r0, r1
	f64a 01f8 	movw	r1, #43256	@ 0xa8f8
	f6c0 0100 	movt	r1, #2048	@ 0x800
	4441      	add	r1, r8
	f003 faea 	bl	<__aeabi_memcpy>
	19a0      	adds	r0, r4, r6
	f1b9 0f00 	cmp.w	r9, #0
	f8c5 00d4 	str.w	r0, [r5, #212]	@ 0xd4
	d104      	bne.n	<ip::__cortex_m_rt_main+0x13a2>
	ea54 0008 	orrs.w	r0, r4, r8
	bf18      	it	ne
	e9c5 aa14 	strdne	sl, sl, [r5, #80]	@ 0x50
	9807      	ldr	r0, [sp, #28]
	f51b 747a 	adds.w	r4, fp, #1000	@ 0x3e8
	f10d 0e18 	add.w	lr, sp, #24
	f240 0840 	movw	r8, #64	@ 0x40
	f140 0500 	adc.w	r5, r0, #0
	f50e 5697 	add.w	r6, lr, #4832	@ 0x12e0
	f50d 59df 	add.w	r9, sp, #7136	@ 0x1be0
	f2c2 0800 	movt	r8, #8192	@ 0x2000
	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
	2900      	cmp	r1, #0
	f000 80b9 	beq.w	<ip::__cortex_m_rt_main+0x153c>
	f8d6 08e8 	ldr.w	r0, [r6, #2280]	@ 0x8e8
	6801      	ldr	r1, [r0, #0]
	43ca      	mvns	r2, r1
	0792      	lsls	r2, r2, #30
	d045      	beq.n	<ip::__cortex_m_rt_main+0x1462>
	2902      	cmp	r1, #2
	d04e      	beq.n	<ip::__cortex_m_rt_main+0x1478>
	f8d0 20d4 	ldr.w	r2, [r0, #212]	@ 0xd4
	b182      	cbz	r2, <ip::__cortex_m_rt_main+0x1402>
	f10d 0e04 	add.w	lr, sp, #4
	4622      	mov	r2, r4
	f50e 50c8 	add.w	r0, lr, #6400	@ 0x1900
	e9cd 0900 	strd	r0, r9, [sp]
	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
	462b      	mov	r3, r5
	f7fb f944 	bl	<smoltcp::iface::interface::Interface::poll>
	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
	2900      	cmp	r1, #0
	d1e4      	bne.n	<ip::__cortex_m_rt_main+0x13ca>
	e09c      	b.n	<ip::__cortex_m_rt_main+0x153c>
	f001 0103 	and.w	r1, r1, #3
	2903      	cmp	r1, #3
	d04f      	beq.n	<ip::__cortex_m_rt_main+0x14aa>
	2902      	cmp	r1, #2
	d058      	beq.n	<ip::__cortex_m_rt_main+0x14c0>
	f880 a133 	strb.w	sl, [r0, #307]	@ 0x133
	f240 002d 	movw	r0, #45	@ 0x2d
	f2c0 0000 	movt	r0, #0
	f7fe fade 	bl	<defmt::export::acquire_header_and_release>
	f10d 0e04 	add.w	lr, sp, #4
	9b07      	ldr	r3, [sp, #28]
	f50e 50c8 	add.w	r0, lr, #6400	@ 0x1900
	e9cd 0900 	strd	r0, r9, [sp]
	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
	465a      	mov	r2, fp
	f7fb f925 	bl	<smoltcp::iface::interface::Interface::poll>
	e705      	b.n	<ip::__cortex_m_rt_main+0x1244>
	f649 10c8 	movw	r0, #39368	@ 0x99c8
	f24a 4254 	movw	r2, #42068	@ 0xa454
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2147      	movs	r1, #71	@ 0x47
	f7fd fcfb 	bl	<core::panicking::panic_fmt>
	f1a0 0360 	sub.w	r3, r0, #96	@ 0x60
	2b60      	cmp	r3, #96	@ 0x60
	d21b      	bcs.n	<ip::__cortex_m_rt_main+0x148e>
	ed9f 2a55 	vldr	s4, [pc, #340]	@ <ip::__cortex_m_rt_main+0x15ac>
	f04f 0ed0 	mov.w	lr, #208	@ 0xd0
	f7fe bec4 	b.w	<ip::__cortex_m_rt_main+0x1ea>
	f24a 40a0 	movw	r0, #42144	@ 0xa4a0
	f24a 42c8 	movw	r2, #42184	@ 0xa4c8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	214f      	movs	r1, #79	@ 0x4f
	f7fd fce6 	bl	<core::panicking::panic_fmt>
	f24a 4064 	movw	r0, #42084	@ 0xa464
	f24a 42d8 	movw	r2, #42200	@ 0xa4d8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2129      	movs	r1, #41	@ 0x29
	f7fe f914 	bl	<core::option::expect_failed>
	38c0      	subs	r0, #192	@ 0xc0
	a347      	add	r3, pc, #284	@ (adr r3, <ip::__cortex_m_rt_main+0x15b0>)
	f04f 0ef0 	mov.w	lr, #240	@ 0xf0
	28c0      	cmp	r0, #192	@ 0xc0
	bf38      	it	cc
	3304      	addcc	r3, #4
	ed93 2a00 	vldr	s4, [r3]
	bf38      	it	cc
	f04f 0ee0 	movcc.w	lr, #224	@ 0xe0
	f7fe bea0 	b.w	<ip::__cortex_m_rt_main+0x1ea>
	f24a 40a0 	movw	r0, #42144	@ 0xa4a0
	f24a 42f8 	movw	r2, #42232	@ 0xa4f8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	214f      	movs	r1, #79	@ 0x4f
	f7fd fcc2 	bl	<core::panicking::panic_fmt>
	f24a 4064 	movw	r0, #42084	@ 0xa464
	f24a 5208 	movw	r2, #42248	@ 0xa508
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2129      	movs	r1, #41	@ 0x29
	f7fe f8f0 	bl	<core::option::expect_failed>
	f64a 7304 	movw	r3, #44804	@ 0xaf04
	464a      	mov	r2, r9
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f7fd fc3f 	bl	<core::slice::index::slice_index_fail>
	f24b 10e8 	movw	r0, #45544	@ 0xb1e8
	f24b 2210 	movw	r2, #45584	@ 0xb210
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2127      	movs	r1, #39	@ 0x27
	f7fd fcc7 	bl	<core::panicking::panic>
	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
	f24b 2220 	movw	r2, #45600	@ 0xb220
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2128      	movs	r1, #40	@ 0x28
	f7fd fcbc 	bl	<core::panicking::panic>
	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
	f24b 2230 	movw	r2, #45616	@ 0xb230
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2128      	movs	r1, #40	@ 0x28
	f7fd fcb1 	bl	<core::panicking::panic>
	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
	f24b 2240 	movw	r2, #45632	@ 0xb240
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2128      	movs	r1, #40	@ 0x28
	f7fd fca6 	bl	<core::panicking::panic>
	f24a 4290 	movw	r2, #42128	@ 0xa490
	2000      	movs	r0, #0
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd fc8b 	bl	<core::panicking::panic_bounds_check>
	f10d 0e08 	add.w	lr, sp, #8
	f244 0017 	movw	r0, #16407	@ 0x4017
	f50e 51ef 	add.w	r1, lr, #7648	@ 0x1de0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	900b      	str	r0, [sp, #44]	@ 0x2c
	f24b 12c8 	movw	r2, #45512	@ 0xb1c8
	e9cd 0109 	strd	r0, r1, [sp, #36]	@ 0x24
	f50d 604a 	add.w	r0, sp, #3232	@ 0xca0
	9008      	str	r0, [sp, #32]
	f24a 4009 	movw	r0, #41993	@ 0xa409
	a908      	add	r1, sp, #32
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd fc64 	bl	<core::panicking::panic_fmt>
	f24a 42e8 	movw	r2, #42216	@ 0xa4e8
	2000      	movs	r0, #0
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd fc6b 	bl	<core::panicking::panic_bounds_check>
	f002 fc58 	bl	<__cpsie>
	f24a 7084 	movw	r0, #42884	@ 0xa784
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fd fbda 	bl	<core::option::unwrap_failed>
	f002 fc50 	bl	<__cpsie>
	f24a 7094 	movw	r0, #42900	@ 0xa794
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fd fbd2 	bl	<core::option::unwrap_failed>
	bf00      	nop
	43000000 	.word	0x43000000
	44000000 	.word	0x44000000
	43800000 	.word	0x43800000

<<stm32_eth::mac::WrongClock as core::fmt::Debug>::fmt>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	e9d1 0200 	ldrd	r0, r2, [r1]
	f64a 114c 	movw	r1, #43340	@ 0xa94c
	68d3      	ldr	r3, [r2, #12]
	f6c0 0100 	movt	r1, #2048	@ 0x800
	220a      	movs	r2, #10
	e8bd 4080 	ldmia.w	sp!, {r7, lr}
	4718      	bx	r3

<core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>:
	b5b0      	push	{r4, r5, r7, lr}
	af02      	add	r7, sp, #8
	6802      	ldr	r2, [r0, #0]
	2a02      	cmp	r2, #2
	bf08      	it	eq
	bdb0      	popeq	{r4, r5, r7, pc}
	e9d0 3c02 	ldrd	r3, ip, [r0, #8]
	6859      	ldr	r1, [r3, #4]
	458c      	cmp	ip, r1
	d230      	bcs.n	<core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>+0x78>
	f44f 61c6 	mov.w	r1, #1584	@ 0x630
	681b      	ldr	r3, [r3, #0]
	fb0c 3101 	mla	r1, ip, r1, r3
	f8d0 e010 	ldr.w	lr, [r0, #16]
	2500      	movs	r5, #0
	f2cf 05d0 	movt	r5, #61648	@ 0xf0d0
	07d2      	lsls	r2, r2, #31
	d007      	beq.n	<core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>+0x3e>
	6840      	ldr	r0, [r0, #4]
	f04f 0c01 	mov.w	ip, #1
	f105 7500 	add.w	r5, r5, #33554432	@ 0x2000000
	e9c1 c008 	strd	ip, r0, [r1, #32]
	e001      	b.n	<core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>+0x42>
	2000      	movs	r0, #0
	6208      	str	r0, [r1, #32]
	6848      	ldr	r0, [r1, #4]
	f640 74ff 	movw	r4, #4095	@ 0xfff
	e9d1 320a 	ldrd	r3, r2, [r1, #40]	@ 0x28
	43a0      	bics	r0, r4
	ea40 000e 	orr.w	r0, r0, lr
	6048      	str	r0, [r1, #4]
	608b      	str	r3, [r1, #8]
	60ca      	str	r2, [r1, #12]
	f3bf 8f5f 	dmb	sy
	f891 0030 	ldrb.w	r0, [r1, #48]	@ 0x30
	ea45 5040 	orr.w	r0, r5, r0, lsl #21
	6008      	str	r0, [r1, #0]
	f249 0004 	movw	r0, #36868	@ 0x9004
	2100      	movs	r1, #0
	f2c4 0002 	movt	r0, #16386	@ 0x4002
	f3bf 8f5f 	dmb	sy
	6001      	str	r1, [r0, #0]
	bdb0      	pop	{r4, r5, r7, pc}
	f24b 0258 	movw	r2, #45144	@ 0xb058
	4660      	mov	r0, ip
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd fc04 	bl	<core::panicking::panic_bounds_check>

<__rustc::rust_begin_unwind>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b086      	sub	sp, #24
	4604      	mov	r4, r0
	f002 fbeb 	bl	<__cpsid>
	f240 0038 	movw	r0, #56	@ 0x38
	f2c2 0000 	movt	r0, #8192	@ 0x2000
	7801      	ldrb	r1, [r0, #0]
	bba9      	cbnz	r1, <__rustc::rust_begin_unwind+0x84>
	2101      	movs	r1, #1
	7001      	strb	r1, [r0, #0]
	f240 002e 	movw	r0, #46	@ 0x2e
	f2c0 0000 	movt	r0, #0
	9400      	str	r4, [sp, #0]
	f7fe f99b 	bl	<defmt::export::acquire_and_header>
	f240 0001 	movw	r0, #1
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 000c 	strh.w	r0, [sp, #12]
	a803      	add	r0, sp, #12
	f7fe fb43 	bl	<_defmt_write>
	f64a 201c 	movw	r0, #43548	@ 0xaa1c
	f24a 712c 	movw	r1, #42796	@ 0xa72c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f649 52d9 	movw	r2, #40409	@ 0x9dd9
	9002      	str	r0, [sp, #8]
	4668      	mov	r0, sp
	9001      	str	r0, [sp, #4]
	f643 70ab 	movw	r0, #16299	@ 0x3fab
	f6c0 0000 	movt	r0, #2048	@ 0x800
	ab03      	add	r3, sp, #12
	9004      	str	r0, [sp, #16]
	a801      	add	r0, sp, #4
	9003      	str	r0, [sp, #12]
	1e78      	subs	r0, r7, #1
	f6c0 0100 	movt	r1, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd f9cd 	bl	<core::fmt::write>
	f24a 7044 	movw	r0, #42820	@ 0xa744
	2101      	movs	r1, #1
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fe fb22 	bl	<_defmt_write>
	f7fe faa8 	bl	<_defmt_release>
	f000 f800 	bl	<panic_probe::hard_fault>

<panic_probe::hard_fault>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	f64e 5024 	movw	r0, #60708	@ 0xed24
	f2ce 0000 	movt	r0, #57344	@ 0xe000
	6801      	ldr	r1, [r0, #0]
	f421 2180 	bic.w	r1, r1, #262144	@ 0x40000
	6001      	str	r1, [r0, #0]
	f002 fbb1 	bl	<__udf>

<<&T as core::fmt::Display>::fmt>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
	b088      	sub	sp, #32
	e9d1 5400 	ldrd	r5, r4, [r1]
	f24a 711c 	movw	r1, #42780	@ 0xa71c
	6800      	ldr	r0, [r0, #0]
	f6c0 0100 	movt	r1, #2048	@ 0x800
	f8d4 900c 	ldr.w	r9, [r4, #12]
	220c      	movs	r2, #12
	e9d0 8600 	ldrd	r8, r6, [r0]
	4628      	mov	r0, r5
	47c8      	blx	r9
	b120      	cbz	r0, <<&T as core::fmt::Display>::fmt+0x32>
	2001      	movs	r0, #1
	b008      	add	sp, #32
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	e9d6 0100 	ldrd	r0, r1, [r6]
	f24a 4227 	movw	r2, #42023	@ 0xa427
	e9cd 0100 	strd	r0, r1, [sp]
	f244 0017 	movw	r0, #16407	@ 0x4017
	f106 010c 	add.w	r1, r6, #12
	f6c0 0000 	movt	r0, #2048	@ 0x800
	9007      	str	r0, [sp, #28]
	ab02      	add	r3, sp, #8
	e9cd 0105 	strd	r0, r1, [sp, #20]
	f106 0008 	add.w	r0, r6, #8
	9004      	str	r0, [sp, #16]
	f643 1031 	movw	r0, #14641	@ 0x3931
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	9003      	str	r0, [sp, #12]
	4668      	mov	r0, sp
	9002      	str	r0, [sp, #8]
	4628      	mov	r0, r5
	4621      	mov	r1, r4
	f7fd f97d 	bl	<core::fmt::write>
	b120      	cbz	r0, <<&T as core::fmt::Display>::fmt+0x7e>
	2001      	movs	r0, #1
	b008      	add	sp, #32
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f24a 7128 	movw	r1, #42792	@ 0xa728
	4628      	mov	r0, r5
	f6c0 0100 	movt	r1, #2048	@ 0x800
	2202      	movs	r2, #2
	47c8      	blx	r9
	b120      	cbz	r0, <<&T as core::fmt::Display>::fmt+0x98>
	2001      	movs	r0, #1
	b008      	add	sp, #32
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	e9d8 2300 	ldrd	r2, r3, [r8]
	4628      	mov	r0, r5
	4621      	mov	r1, r4
	f7fd f964 	bl	<core::fmt::write>
	b008      	add	sp, #32
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold>:
	b5b0      	push	{r4, r5, r7, lr}
	af02      	add	r7, sp, #8
	4288      	cmp	r0, r1
	d02c      	beq.n	<<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x62>
	f64a 23ab 	movw	r3, #43691	@ 0xaaab
	1a09      	subs	r1, r1, r0
	f6ca 23aa 	movt	r3, #43690	@ 0xaaaa
	6815      	ldr	r5, [r2, #0]
	fba1 1303 	umull	r1, r3, r1, r3
	4611      	mov	r1, r2
	688c      	ldr	r4, [r1, #8]
	6852      	ldr	r2, [r2, #4]
	f105 0e04 	add.w	lr, r5, #4
	08d9      	lsrs	r1, r3, #3
	1d03      	adds	r3, r0, #4
	00e0      	lsls	r0, r4, #3
	e007      	b.n	<<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x3a>
	2400      	movs	r4, #0
	f843 4c04 	str.w	r4, [r3, #-4]
	3008      	adds	r0, #8
	e8e3 5c03 	strd	r5, ip, [r3], #12
	3901      	subs	r1, #1
	d013      	beq.n	<<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x62>
	4290      	cmp	r0, r2
	d2f5      	bcs.n	<<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x2a>
	f100 0c04 	add.w	ip, r0, #4
	4594      	cmp	ip, r2
	d817      	bhi.n	<<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x76>
	1dc4      	adds	r4, r0, #7
	4294      	cmp	r4, r2
	d20b      	bcs.n	<<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x64>
	eb0e 0400 	add.w	r4, lr, r0
	f85e 5000 	ldr.w	r5, [lr, r0]
	f854 4c04 	ldr.w	r4, [r4, #-4]
	fa95 fc85 	rev.w	ip, r5
	ba25      	rev	r5, r4
	2401      	movs	r4, #1
	e7e4      	b.n	<<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x2c>
	bdb0      	pop	{r4, r5, r7, pc}
	f64a 4358 	movw	r3, #44120	@ 0xac58
	f100 0108 	add.w	r1, r0, #8
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4660      	mov	r0, ip
	f7fd faa4 	bl	<core::slice::index::slice_index_fail>
	f64a 4368 	movw	r3, #44136	@ 0xac68
	4661      	mov	r1, ip
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f7fd fa9d 	bl	<core::slice::index::slice_index_fail>

<smoltcp::wire::tcp::TcpOption::emit>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	f84d bd04 	str.w	fp, [sp, #-4]!
	6803      	ldr	r3, [r0, #0]
	2405      	movs	r4, #5
	2504      	movs	r5, #4
	2b01      	cmp	r3, #1
	bf88      	it	hi
	1e9c      	subhi	r4, r3, #2
	e8df f004 	tbb	[pc, r4]
	13251904 	.word	0x13251904
	17391f0f 	.word	0x17391f0f
	b13a      	cbz	r2, <smoltcp::wire::tcp::TcpOption::emit+0x32>
	4608      	mov	r0, r1
	460c      	mov	r4, r1
	4611      	mov	r1, r2
	4615      	mov	r5, r2
	f003 f8d9 	bl	<__aeabi_memclr>
	4621      	mov	r1, r4
	462a      	mov	r2, r5
	2501      	movs	r5, #1
	e07d      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x132>
	2502      	movs	r5, #2
	2a01      	cmp	r2, #1
	d814      	bhi.n	<smoltcp::wire::tcp::TcpOption::emit+0x66>
	e028      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x90>
	2503      	movs	r5, #3
	2a01      	cmp	r2, #1
	d810      	bhi.n	<smoltcp::wire::tcp::TcpOption::emit+0x66>
	e024      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x90>
	6886      	ldr	r6, [r0, #8]
	e00a      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x60>
	2a00      	cmp	r2, #0
	f000 80da 	beq.w	<smoltcp::wire::tcp::TcpOption::emit+0x204>
	2501      	movs	r5, #1
	700d      	strb	r5, [r1, #0]
	e06d      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x132>
	68c6      	ldr	r6, [r0, #12]
	6985      	ldr	r5, [r0, #24]
	441e      	add	r6, r3
	442e      	add	r6, r5
	00f6      	lsls	r6, r6, #3
	1cb5      	adds	r5, r6, #2
	2a01      	cmp	r2, #1
	d914      	bls.n	<smoltcp::wire::tcp::TcpOption::emit+0x90>
	3c02      	subs	r4, #2
	704d      	strb	r5, [r1, #1]
	e8df f004 	tbb	[pc, r4]
	4a03      	.short	0x4a03
	521a312e 	.word	0x521a312e
	2302      	movs	r3, #2
	700b      	strb	r3, [r1, #0]
	f002 033e 	and.w	r3, r2, #62	@ 0x3e
	2b02      	cmp	r3, #2
	f000 80a7 	beq.w	<smoltcp::wire::tcp::TcpOption::emit+0x1d0>
	8880      	ldrh	r0, [r0, #4]
	ba40      	rev16	r0, r0
	8048      	strh	r0, [r1, #2]
	e053      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x132>
	250a      	movs	r5, #10
	2a01      	cmp	r2, #1
	d8ea      	bhi.n	<smoltcp::wire::tcp::TcpOption::emit+0x66>
	f64a 33d8 	movw	r3, #43992	@ 0xabd8
	2001      	movs	r0, #1
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4611      	mov	r1, r2
	461a      	mov	r2, r3
	f7fd facb 	bl	<core::panicking::panic_bounds_check>
	2608      	movs	r6, #8
	e9d0 3001 	ldrd	r3, r0, [r0, #4]
	700e      	strb	r6, [r1, #0]
	1f96      	subs	r6, r2, #6
	f116 0f04 	cmn.w	r6, #4
	bf3f      	itttt	cc
	ba1b      	revcc	r3, r3
	f8c1 3002 	strcc.w	r3, [r1, #2]
	f1a2 030a 	subcc.w	r3, r2, #10
	f113 0f04 	cmncc.w	r3, #4
	d278      	bcs.n	<smoltcp::wire::tcp::TcpOption::emit+0x1b4>
	ba00      	rev	r0, r0
	f8c1 0006 	str.w	r0, [r1, #6]
	e033      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x132>
	2004      	movs	r0, #4
	7008      	strb	r0, [r1, #0]
	e030      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x132>
	2605      	movs	r6, #5
	700e      	strb	r6, [r1, #0]
	b3e3      	cbz	r3, <smoltcp::wire::tcp::TcpOption::emit+0x150>
	1f93      	subs	r3, r2, #6
	f113 0f04 	cmn.w	r3, #4
	bf3f      	itttt	cc
	e9d0 6301 	ldrdcc	r6, r3, [r0, #4]
	ba36      	revcc	r6, r6
	f8c1 6002 	strcc.w	r6, [r1, #2]
	f1a2 060a 	subcc.w	r6, r2, #10
	bf38      	it	cc
	f116 0f04 	cmncc.w	r6, #4
	d25f      	bcs.n	<smoltcp::wire::tcp::TcpOption::emit+0x1b4>
	ba1b      	rev	r3, r3
	f8c1 3006 	str.w	r3, [r1, #6]
	2301      	movs	r3, #1
	68c6      	ldr	r6, [r0, #12]
	bb56      	cbnz	r6, <smoltcp::wire::tcp::TcpOption::emit+0x156>
	e041      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x186>
	7900      	ldrb	r0, [r0, #4]
	2303      	movs	r3, #3
	2a02      	cmp	r2, #2
	700b      	strb	r3, [r1, #0]
	f000 8083 	beq.w	<smoltcp::wire::tcp::TcpOption::emit+0x214>
	7088      	strb	r0, [r1, #2]
	e00f      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x132>
	e9d0 c301 	ldrd	ip, r3, [r0, #4]
	7b00      	ldrb	r0, [r0, #12]
	7008      	strb	r0, [r1, #0]
	1e90      	subs	r0, r2, #2
	4298      	cmp	r0, r3
	d162      	bne.n	<smoltcp::wire::tcp::TcpOption::emit+0x1e6>
	1c88      	adds	r0, r1, #2
	4614      	mov	r4, r2
	460e      	mov	r6, r1
	4661      	mov	r1, ip
	461a      	mov	r2, r3
	f003 f805 	bl	<__aeabi_memcpy>
	4631      	mov	r1, r6
	4622      	mov	r2, r4
	42aa      	cmp	r2, r5
	bf21      	itttt	cs
	1948      	addcs	r0, r1, r5
	1b51      	subcs	r1, r2, r5
	f85d bb04 	ldrcs.w	fp, [sp], #4
	bdf0      	popcs	{r4, r5, r6, r7, pc}
	f64a 4308 	movw	r3, #44040	@ 0xac08
	4628      	mov	r0, r5
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4611      	mov	r1, r2
	f7fd f9f5 	bl	<core::slice::index::slice_index_fail>
	2300      	movs	r3, #0
	68c6      	ldr	r6, [r0, #12]
	b1be      	cbz	r6, <smoltcp::wire::tcp::TcpOption::emit+0x186>
	00dc      	lsls	r4, r3, #3
	f104 0c02 	add.w	ip, r4, #2
	4562      	cmp	r2, ip
	d349      	bcc.n	<smoltcp::wire::tcp::TcpOption::emit+0x1f4>
	eba2 060c 	sub.w	r6, r2, ip
	2e04      	cmp	r6, #4
	bf21      	itttt	cs
	e9d0 6e04 	ldrdcs	r6, lr, [r0, #16]
	ba36      	revcs	r6, r6
	f841 600c 	strcs.w	r6, [r1, ip]
	f044 0406 	orrcs.w	r4, r4, #6
	bf24      	itt	cs
	1b16      	subcs	r6, r2, r4
	2e04      	cmpcs	r6, #4
	d31a      	bcc.n	<smoltcp::wire::tcp::TcpOption::emit+0x1b4>
	3301      	adds	r3, #1
	fa9e f68e 	rev.w	r6, lr
	510e      	str	r6, [r1, r4]
	6986      	ldr	r6, [r0, #24]
	2e00      	cmp	r6, #0
	d0d2      	beq.n	<smoltcp::wire::tcp::TcpOption::emit+0x132>
	00db      	lsls	r3, r3, #3
	f103 0c02 	add.w	ip, r3, #2
	4562      	cmp	r2, ip
	d32e      	bcc.n	<smoltcp::wire::tcp::TcpOption::emit+0x1f4>
	eba2 060c 	sub.w	r6, r2, ip
	2e04      	cmp	r6, #4
	bf21      	itttt	cs
	e9d0 6007 	ldrdcs	r6, r0, [r0, #28]
	ba36      	revcs	r6, r6
	f841 600c 	strcs.w	r6, [r1, ip]
	f043 0306 	orrcs.w	r3, r3, #6
	bf24      	itt	cs
	1ad6      	subcs	r6, r2, r3
	2e04      	cmpcs	r6, #4
	d20a      	bcs.n	<smoltcp::wire::tcp::TcpOption::emit+0x1ca>
	f64a 203c 	movw	r0, #43580	@ 0xaa3c
	f64a 228c 	movw	r2, #43660	@ 0xaa8c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2120      	movs	r1, #32
	f7fd fa4b 	bl	<core::panicking::panic>
	ba00      	rev	r0, r0
	50c8      	str	r0, [r1, r3]
	e7b0      	b.n	<smoltcp::wire::tcp::TcpOption::emit+0x132>
	f64a 205c 	movw	r0, #43612	@ 0xaa5c
	f64a 227c 	movw	r2, #43644	@ 0xaa7c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2120      	movs	r1, #32
	f7fd fa3d 	bl	<core::panicking::panic>
	f64a 32f8 	movw	r2, #44024	@ 0xabf8
	4619      	mov	r1, r3
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd fe30 	bl	<core::slice::copy_from_slice_impl::len_mismatch_fail>
	f64a 4318 	movw	r3, #44056	@ 0xac18
	4660      	mov	r0, ip
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4611      	mov	r1, r2
	f7fd f99b 	bl	<core::slice::index::slice_index_fail>
	f64a 32c8 	movw	r2, #43976	@ 0xabc8
	2000      	movs	r0, #0
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2100      	movs	r1, #0
	f7fd fa12 	bl	<core::panicking::panic_bounds_check>
	f64a 32e8 	movw	r2, #44008	@ 0xabe8
	2002      	movs	r0, #2
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2102      	movs	r1, #2
	f7fd fa0a 	bl	<core::panicking::panic_bounds_check>

<smoltcp::socket::tcp::Socket::seq_to_transmit>:
	b5b0      	push	{r4, r5, r7, lr}
	af02      	add	r7, sp, #8
	f8b0 211c 	ldrh.w	r2, [r0, #284]	@ 0x11c
	07d2      	lsls	r2, r2, #31
	d053      	beq.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0xb4>
	f8d0 3110 	ldr.w	r3, [r0, #272]	@ 0x110
	3936      	subs	r1, #54	@ 0x36
	f8d0 5100 	ldr.w	r5, [r0, #256]	@ 0x100
	f8d0 e108 	ldr.w	lr, [r0, #264]	@ 0x108
	428b      	cmp	r3, r1
	f890 2133 	ldrb.w	r2, [r0, #307]	@ 0x133
	bf38      	it	cc
	4619      	movcc	r1, r3
	45ae      	cmp	lr, r5
	bf04      	itt	eq
	f002 030e 	andeq.w	r3, r2, #14
	2b02      	cmpeq	r3, #2
	d024      	beq.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0x7a>
	f8d0 40d4 	ldr.w	r4, [r0, #212]	@ 0xd4
	f8d0 310c 	ldr.w	r3, [r0, #268]	@ 0x10c
	429c      	cmp	r4, r3
	bf38      	it	cc
	4623      	movcc	r3, r4
	f1b3 3fff 	cmp.w	r3, #4294967295	@ 0xffffffff
	dd2c      	ble.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0x9e>
	442b      	add	r3, r5
	2a09      	cmp	r2, #9
	eba3 0c0e 	sub.w	ip, r3, lr
	d817      	bhi.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0x7e>
	2301      	movs	r3, #1
	fa03 f202 	lsl.w	r2, r3, r2
	f412 7f48 	tst.w	r2, #800	@ 0x320
	d011      	beq.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0x7e>
	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
	dd1e      	ble.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0x9e>
	1960      	adds	r0, r4, r5
	ebae 0000 	sub.w	r0, lr, r0
	fab0 f080 	clz	r0, r0
	0940      	lsrs	r0, r0, #5
	2100      	movs	r1, #0
	f1bc 0f00 	cmp.w	ip, #0
	bfc8      	it	gt
	2101      	movgt	r1, #1
	4308      	orrs	r0, r1
	bdb0      	pop	{r4, r5, r7, pc}
	2001      	movs	r0, #1
	bdb0      	pop	{r4, r5, r7, pc}
	2200      	movs	r2, #0
	45ae      	cmp	lr, r5
	d00a      	beq.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0x9a>
	ea2c 73ec 	bic.w	r3, ip, ip, asr #31
	428b      	cmp	r3, r1
	d206      	bcs.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0x9a>
	f890 0132 	ldrb.w	r0, [r0, #306]	@ 0x132
	07c0      	lsls	r0, r0, #31
	4610      	mov	r0, r2
	bf18      	it	ne
	bdb0      	popne	{r4, r5, r7, pc}
	e7e8      	b.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0x6c>
	4610      	mov	r0, r2
	e7e6      	b.n	<smoltcp::socket::tcp::Socket::seq_to_transmit+0x6c>
	f64a 4078 	movw	r0, #44152	@ 0xac78
	f64a 42b0 	movw	r2, #44208	@ 0xacb0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2171      	movs	r1, #113	@ 0x71
	f7fd f9a2 	bl	<core::panicking::panic_fmt>
	f64a 5040 	movw	r0, #44352	@ 0xad40
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fd f921 	bl	<core::option::unwrap_failed>

<smoltcp::socket::tcp::Socket::process>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b091      	sub	sp, #68	@ 0x44
	f891 9133 	ldrb.w	r9, [r1, #307]	@ 0x133
	4682      	mov	sl, r0
	200c      	movs	r0, #12
	68fd      	ldr	r5, [r7, #12]
	469b      	mov	fp, r3
	f8d7 8008 	ldr.w	r8, [r7, #8]
	fa20 f009 	lsr.w	r0, r0, r9
	f000 0301 	and.w	r3, r0, #1
	f44f 7048 	mov.w	r0, #800	@ 0x320
	f895 4051 	ldrb.w	r4, [r5, #81]	@ 0x51
	fa20 f609 	lsr.w	r6, r0, r9
	e9d5 0c00 	ldrd	r0, ip, [r5]
	f1b9 0f02 	cmp.w	r9, #2
	d113      	bne.n	<smoltcp::socket::tcp::Socket::process+0x60>
	b35c      	cbz	r4, <smoltcp::socket::tcp::Socket::process+0x92>
	2c02      	cmp	r4, #2
	d01c      	beq.n	<smoltcp::socket::tcp::Socket::process+0x78>
	2c04      	cmp	r4, #4
	d133      	bne.n	<smoltcp::socket::tcp::Socket::process+0xaa>
	2800      	cmp	r0, #0
	f000 80d2 	beq.w	<smoltcp::socket::tcp::Socket::process+0x1ec>
	4686      	mov	lr, r0
	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
	3001      	adds	r0, #1
	4584      	cmp	ip, r0
	4670      	mov	r0, lr
	d031      	beq.n	<smoltcp::socket::tcp::Socket::process+0xba>
	f240 001d 	movw	r0, #29
	f2c0 0000 	movt	r0, #0
	e101      	b.n	<smoltcp::socket::tcp::Socket::process+0x264>
	2c04      	cmp	r4, #4
	d02a      	beq.n	<smoltcp::socket::tcp::Socket::process+0xba>
	f1b9 0f01 	cmp.w	r9, #1
	d024      	beq.n	<smoltcp::socket::tcp::Socket::process+0xb4>
	f1b9 0f02 	cmp.w	r9, #2
	f040 809c 	bne.w	<smoltcp::socket::tcp::Socket::process+0x1aa>
	b174      	cbz	r4, <smoltcp::socket::tcp::Socket::process+0x92>
	2c02      	cmp	r4, #2
	d118      	bne.n	<smoltcp::socket::tcp::Socket::process+0xaa>
	b1f8      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0xba>
	4686      	mov	lr, r0
	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
	3001      	adds	r0, #1
	4584      	cmp	ip, r0
	4670      	mov	r0, lr
	d018      	beq.n	<smoltcp::socket::tcp::Socket::process+0xba>
	f240 001e 	movw	r0, #30
	f2c0 0000 	movt	r0, #0
	e12c      	b.n	<smoltcp::socket::tcp::Socket::process+0x2ec>
	b150      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0xaa>
	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
	3001      	adds	r0, #1
	4584      	cmp	ip, r0
	f040 8122 	bne.w	<smoltcp::socket::tcp::Socket::process+0x2e4>
	f240 000e 	movw	r0, #14
	f2c0 0000 	movt	r0, #0
	e0dc      	b.n	<smoltcp::socket::tcp::Socket::process+0x264>
	f240 000d 	movw	r0, #13
	f2c0 0000 	movt	r0, #0
	e0d7      	b.n	<smoltcp::socket::tcp::Socket::process+0x264>
	2800      	cmp	r0, #0
	f041 8070 	bne.w	<smoltcp::socket::tcp::Socket::process+0x119a>
	900d      	str	r0, [sp, #52]	@ 0x34
	f8d1 00c4 	ldr.w	r0, [r1, #196]	@ 0xc4
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	dd0e      	ble.n	<smoltcp::socket::tcp::Socket::process+0xe4>
	e9cd 6305 	strd	r6, r3, [sp, #20]
	f8cd 801c 	str.w	r8, [sp, #28]
	f8d1 80bc 	ldr.w	r8, [r1, #188]	@ 0xbc
	f1b8 3fff 	cmp.w	r8, #4294967295	@ 0xffffffff
	bfc1      	itttt	gt
	920a      	strgt	r2, [sp, #40]	@ 0x28
	6c2b      	ldrgt	r3, [r5, #64]	@ 0x40
	930b      	strgt	r3, [sp, #44]	@ 0x2c
	f1b3 3fff 	cmpgt.w	r3, #4294967295	@ 0xffffffff
	dc0a      	bgt.n	<smoltcp::socket::tcp::Socket::process+0xfa>
	f64a 4078 	movw	r0, #44152	@ 0xac78
	f64a 42b0 	movw	r2, #44208	@ 0xacb0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2171      	movs	r1, #113	@ 0x71
	f7fd f91f 	bl	<core::panicking::panic_fmt>
	f8d1 6104 	ldr.w	r6, [r1, #260]	@ 0x104
	46a6      	mov	lr, r4
	6c6c      	ldr	r4, [r5, #68]	@ 0x44
	f1a9 0301 	sub.w	r3, r9, #1
	f8cd c03c 	str.w	ip, [sp, #60]	@ 0x3c
	eb00 0c06 	add.w	ip, r0, r6
	2b02      	cmp	r3, #2
	f8cd a030 	str.w	sl, [sp, #48]	@ 0x30
	950e      	str	r5, [sp, #56]	@ 0x38
	f8cd b020 	str.w	fp, [sp, #32]
	d239      	bcs.n	<smoltcp::socket::tcp::Socket::process+0x190>
	9603      	str	r6, [sp, #12]
	46e2      	mov	sl, ip
	2501      	movs	r5, #1
	2300      	movs	r3, #0
	f04f 0c00 	mov.w	ip, #0
	4676      	mov	r6, lr
	9409      	str	r4, [sp, #36]	@ 0x24
	980d      	ldr	r0, [sp, #52]	@ 0x34
	f04f 0b00 	mov.w	fp, #0
	2e04      	cmp	r6, #4
	f000 81f0 	beq.w	<smoltcp::socket::tcp::Socket::process+0x518>
	2400      	movs	r4, #0
	f04f 0800 	mov.w	r8, #0
	2800      	cmp	r0, #0
	f000 81ed 	beq.w	<smoltcp::socket::tcp::Socket::process+0x51e>
	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
	9a06      	ldr	r2, [sp, #24]
	4410      	add	r0, r2
	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
	1a14      	subs	r4, r2, r0
	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
	f340 80fc 	ble.w	<smoltcp::socket::tcp::Socket::process+0x350>
	9805      	ldr	r0, [sp, #20]
	07c0      	lsls	r0, r0, #31
	d00b      	beq.n	<smoltcp::socket::tcp::Socket::process+0x176>
	f8d1 00d4 	ldr.w	r0, [r1, #212]	@ 0xd4
	3001      	adds	r0, #1
	42a0      	cmp	r0, r4
	eba0 0004 	sub.w	r0, r0, r4
	fab0 f080 	clz	r0, r0
	bf08      	it	eq
	3c01      	subeq	r4, #1
	ea4f 1850 	mov.w	r8, r0, lsr #5
	f8d1 0108 	ldr.w	r0, [r1, #264]	@ 0x108
	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
	1a80      	subs	r0, r0, r2
	fab0 f080 	clz	r0, r0
	ea4f 1b50 	mov.w	fp, r0, lsr #5
	6808      	ldr	r0, [r1, #0]
	07c0      	lsls	r0, r0, #31
	f040 80e6 	bne.w	<smoltcp::socket::tcp::Socket::process+0x35a>
	e1c6      	b.n	<smoltcp::socket::tcp::Socket::process+0x51e>
	9b0b      	ldr	r3, [sp, #44]	@ 0x2c
	eb08 0b06 	add.w	fp, r8, r6
	191d      	adds	r5, r3, r4
	b36b      	cbz	r3, <smoltcp::socket::tcp::Socket::process+0x1f6>
	4540      	cmp	r0, r8
	d178      	bne.n	<smoltcp::socket::tcp::Socket::process+0x290>
	f240 0011 	movw	r0, #17
	460e      	mov	r6, r1
	f2c0 0000 	movt	r0, #0
	e02f      	b.n	<smoltcp::socket::tcp::Socket::process+0x20a>
	2800      	cmp	r0, #0
	d056      	beq.n	<smoltcp::socket::tcp::Socket::process+0x25c>
	469e      	mov	lr, r3
	f1b9 0f03 	cmp.w	r9, #3
	900d      	str	r0, [sp, #52]	@ 0x34
	f040 811d 	bne.w	<smoltcp::socket::tcp::Socket::process+0x3f4>
	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
	4673      	mov	r3, lr
	3001      	adds	r0, #1
	4584      	cmp	ip, r0
	980d      	ldr	r0, [sp, #52]	@ 0x34
	f43f af78 	beq.w	<smoltcp::socket::tcp::Socket::process+0xba>
	f240 001b 	movw	r0, #27
	f2c0 0000 	movt	r0, #0
	f7fd fe7c 	bl	<defmt::export::acquire_header_and_release>
	4650      	mov	r0, sl
	4659      	mov	r1, fp
	4642      	mov	r2, r8
	462b      	mov	r3, r5
	b011      	add	sp, #68	@ 0x44
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
	f001 b819 	b.w	<smoltcp::socket::tcp::Socket::rst_reply>
	f240 001c 	movw	r0, #28
	f2c0 0000 	movt	r0, #0
	e036      	b.n	<smoltcp::socket::tcp::Socket::process+0x264>
	f1ac 0301 	sub.w	r3, ip, #1
	429c      	cmp	r4, r3
	f040 80ef 	bne.w	<smoltcp::socket::tcp::Socket::process+0x3de>
	f240 0012 	movw	r0, #18
	460e      	mov	r6, r1
	f2c0 0000 	movt	r0, #0
	f7fd fe60 	bl	<defmt::export::acquire_header_and_release>
	980a      	ldr	r0, [sp, #40]	@ 0x28
	4631      	mov	r1, r6
	f8dd e038 	ldr.w	lr, [sp, #56]	@ 0x38
	e9d0 4504 	ldrd	r4, r5, [r0, #16]
	980c      	ldr	r0, [sp, #48]	@ 0x30
	9a08      	ldr	r2, [sp, #32]
	f896 3133 	ldrb.w	r3, [r6, #307]	@ 0x133
	2b0a      	cmp	r3, #10
	d10e      	bne.n	<smoltcp::socket::tcp::Socket::process+0x244>
	f249 6380 	movw	r3, #38528	@ 0x9680
	f04f 0c00 	mov.w	ip, #0
	f2c0 0398 	movt	r3, #152	@ 0x98
	f04f 0803 	mov.w	r8, #3
	191b      	adds	r3, r3, r4
	e9c1 8c20 	strd	r8, ip, [r1, #128]	@ 0x80
	f145 0600 	adc.w	r6, r5, #0
	e9c1 3622 	strd	r3, r6, [r1, #136]	@ 0x88
	e9d1 363e 	ldrd	r3, r6, [r1, #248]	@ 0xf8
	1ae3      	subs	r3, r4, r3
	eb75 0306 	sbcs.w	r3, r5, r6
	da12      	bge.n	<smoltcp::socket::tcp::Socket::process+0x276>
	2102      	movs	r1, #2
	6101      	str	r1, [r0, #16]
	b011      	add	sp, #68	@ 0x44
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f240 0010 	movw	r0, #16
	f2c0 0000 	movt	r0, #0
	f7fd fe33 	bl	<defmt::export::acquire_header_and_release>
	2002      	movs	r0, #2
	f8ca 0010 	str.w	r0, [sl, #16]
	b011      	add	sp, #68	@ 0x44
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f244 2340 	movw	r3, #16960	@ 0x4240
	f2c0 030f 	movt	r3, #15
	18e6      	adds	r6, r4, r3
	f145 0300 	adc.w	r3, r5, #0
	e9c1 633e 	strd	r6, r3, [r1, #248]	@ 0xf8
	f8c7 e008 	str.w	lr, [r7, #8]
	f000 bf6b 	b.w	<smoltcp::socket::tcp::Socket::process+0x1166>
	ebac 0004 	sub.w	r0, ip, r4
	2800      	cmp	r0, #0
	dc04      	bgt.n	<smoltcp::socket::tcp::Socket::process+0x2a2>
	4623      	mov	r3, r4
	ebb4 020b 	subs.w	r2, r4, fp
	f100 8109 	bmi.w	<smoltcp::socket::tcp::Socket::process+0x4b4>
	ebac 0205 	sub.w	r2, ip, r5
	f1b2 3fff 	cmp.w	r2, #4294967295	@ 0xffffffff
	bfdc      	itt	le
	eba5 020b 	suble.w	r2, r5, fp
	2a01      	cmple	r2, #1
	f2c0 80fb 	blt.w	<smoltcp::socket::tcp::Socket::process+0x4ac>
	f240 0016 	movw	r0, #22
	460e      	mov	r6, r1
	f2c0 0000 	movt	r0, #0
	46e0      	mov	r8, ip
	f7fd fdf3 	bl	<defmt::export::acquire_and_header>
	4620      	mov	r0, r4
	f001 fb03 	bl	<defmt::export::fmt>
	4628      	mov	r0, r5
	f001 fb00 	bl	<defmt::export::fmt>
	4640      	mov	r0, r8
	f001 fafd 	bl	<defmt::export::fmt>
	4658      	mov	r0, fp
	f001 fafa 	bl	<defmt::export::fmt>
	f7fd ff1f 	bl	<_defmt_release>
	e794      	b.n	<smoltcp::socket::tcp::Socket::process+0x20e>
	f240 000f 	movw	r0, #15
	f2c0 0000 	movt	r0, #0
	4666      	mov	r6, ip
	f7fd fdee 	bl	<defmt::export::acquire_header_and_release>
	2000      	movs	r0, #0
	2101      	movs	r1, #1
	f88a 0058 	strb.w	r0, [sl, #88]	@ 0x58
	f04f 6280 	mov.w	r2, #67108864	@ 0x4000000
	f8aa 0048 	strh.w	r0, [sl, #72]	@ 0x48
	f8ca 003c 	str.w	r0, [sl, #60]	@ 0x3c
	f8ca 0030 	str.w	r0, [sl, #48]	@ 0x30
	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
	f8ca 0018 	str.w	r0, [sl, #24]
	f8ca 0050 	str.w	r0, [sl, #80]	@ 0x50
	f8ca 0010 	str.w	r0, [sl, #16]
	f44f 7050 	mov.w	r0, #832	@ 0x340
	f8aa 000c 	strh.w	r0, [sl, #12]
	f8d5 004a 	ldr.w	r0, [r5, #74]	@ 0x4a
	f8ca 104c 	str.w	r1, [sl, #76]	@ 0x4c
	2114      	movs	r1, #20
	f8ca 205e 	str.w	r2, [sl, #94]	@ 0x5e
	ea4f 4030 	mov.w	r0, r0, ror #16
	f8ca 6054 	str.w	r6, [sl, #84]	@ 0x54
	f8ca 8000 	str.w	r8, [sl]
	f8ca b004 	str.w	fp, [sl, #4]
	f8ca 1008 	str.w	r1, [sl, #8]
	f8ca 005a 	str.w	r0, [sl, #90]	@ 0x5a
	b011      	add	sp, #68	@ 0x44
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	2400      	movs	r4, #0
	6808      	ldr	r0, [r1, #0]
	07c0      	lsls	r0, r0, #31
	f000 80e2 	beq.w	<smoltcp::socket::tcp::Socket::process+0x51e>
	6908      	ldr	r0, [r1, #16]
	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
	1a10      	subs	r0, r2, r0
	f100 80dd 	bmi.w	<smoltcp::socket::tcp::Socket::process+0x51e>
	e9cd 6500 	strd	r6, r5, [sp]
	f8cd c010 	str.w	ip, [sp, #16]
	9302      	str	r3, [sp, #8]
	980a      	ldr	r0, [sp, #40]	@ 0x28
	e9d1 3502 	ldrd	r3, r5, [r1, #8]
	e9d0 0c04 	ldrd	r0, ip, [r0, #16]
	e9d1 6208 	ldrd	r6, r2, [r1, #32]
	e9cd 6205 	strd	r6, r2, [sp, #20]
	2200      	movs	r2, #0
	1ac0      	subs	r0, r0, r3
	e9c1 2200 	strd	r2, r2, [r1]
	f881 2028 	strb.w	r2, [r1, #40]	@ 0x28
	eb6c 0205 	sbc.w	r2, ip, r5
	460d      	mov	r5, r1
	ea80 70e2 	eor.w	r0, r0, r2, asr #31
	ea82 73e2 	eor.w	r3, r2, r2, asr #31
	ebb0 70e2 	subs.w	r0, r0, r2, asr #31
	eb63 72e2 	sbc.w	r2, r3, r2, asr #31
	2300      	movs	r3, #0
	4611      	mov	r1, r2
	f44f 727a 	mov.w	r2, #1000	@ 0x3e8
	f002 fe0f 	bl	<__aeabi_uldivmod>
	9a05      	ldr	r2, [sp, #20]
	4629      	mov	r1, r5
	ebc2 02c2 	rsb	r2, r2, r2, lsl #3
	4402      	add	r2, r0
	3207      	adds	r2, #7
	08d2      	lsrs	r2, r2, #3
	1a10      	subs	r0, r2, r0
	622a      	str	r2, [r5, #32]
	bf48      	it	mi
	4240      	negmi	r0, r0
	9a06      	ldr	r2, [sp, #24]
	9b02      	ldr	r3, [sp, #8]
	e9dd 6500 	ldrd	r6, r5, [sp]
	eb02 0242 	add.w	r2, r2, r2, lsl #1
	f8dd c010 	ldr.w	ip, [sp, #16]
	4410      	add	r0, r2
	3003      	adds	r0, #3
	0880      	lsrs	r0, r0, #2
	6248      	str	r0, [r1, #36]	@ 0x24
	e09f      	b.n	<smoltcp::socket::tcp::Socket::process+0x51e>
	4540      	cmp	r0, r8
	d156      	bne.n	<smoltcp::socket::tcp::Socket::process+0x490>
	4663      	mov	r3, ip
	45a4      	cmp	ip, r4
	d065      	beq.n	<smoltcp::socket::tcp::Socket::process+0x4b4>
	f240 0021 	movw	r0, #33	@ 0x21
	460e      	mov	r6, r1
	f2c0 0000 	movt	r0, #0
	e70a      	b.n	<smoltcp::socket::tcp::Socket::process+0x20a>
	f006 0001 	and.w	r0, r6, #1
	f8d1 30d4 	ldr.w	r3, [r1, #212]	@ 0xd4
	4470      	add	r0, lr
	f8cd 801c 	str.w	r8, [sp, #28]
	4418      	add	r0, r3
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f77f ae6c 	ble.w	<smoltcp::socket::tcp::Socket::process+0xe4>
	f8d1 3100 	ldr.w	r3, [r1, #256]	@ 0x100
	eb03 080e 	add.w	r8, r3, lr
	4418      	add	r0, r3
	900f      	str	r0, [sp, #60]	@ 0x3c
	ebbc 0008 	subs.w	r0, ip, r8
	f100 830f 	bmi.w	<smoltcp::socket::tcp::Socket::process+0xa3e>
	f8cd 8038 	str.w	r8, [sp, #56]	@ 0x38
	4673      	mov	r3, lr
	980f      	ldr	r0, [sp, #60]	@ 0x3c
	f8dd 801c 	ldr.w	r8, [sp, #28]
	ebac 0000 	sub.w	r0, ip, r0
	2800      	cmp	r0, #0
	980d      	ldr	r0, [sp, #52]	@ 0x34
	f77f ae41 	ble.w	<smoltcp::socket::tcp::Socket::process+0xba>
	f240 001a 	movw	r0, #26
	4688      	mov	r8, r1
	f2c0 0000 	movt	r0, #0
	4664      	mov	r4, ip
	4691      	mov	r9, r2
	f7fd fd31 	bl	<defmt::export::acquire_and_header>
	4620      	mov	r0, r4
	f001 fa41 	bl	<defmt::export::fmt>
	980e      	ldr	r0, [sp, #56]	@ 0x38
	f001 fa3e 	bl	<defmt::export::fmt>
	980f      	ldr	r0, [sp, #60]	@ 0x3c
	f001 fa3b 	bl	<defmt::export::fmt>
	f7fd fe60 	bl	<_defmt_release>
	e9d9 2004 	ldrd	r2, r0, [r9, #16]
	e9d8 363e 	ldrd	r3, r6, [r8, #248]	@ 0xf8
	1ad3      	subs	r3, r2, r3
	eb70 0306 	sbcs.w	r3, r0, r6
	f6ff aefb 	blt.w	<smoltcp::socket::tcp::Socket::process+0x268>
	f244 2340 	movw	r3, #16960	@ 0x4240
	4641      	mov	r1, r8
	f2c0 030f 	movt	r3, #15
	18d2      	adds	r2, r2, r3
	f140 0000 	adc.w	r0, r0, #0
	e9c1 203e 	strd	r2, r0, [r1, #248]	@ 0xf8
	4650      	mov	r0, sl
	60bd      	str	r5, [r7, #8]
	465a      	mov	r2, fp
	f000 be6b 	b.w	<smoltcp::socket::tcp::Socket::process+0x1166>
	ebac 0004 	sub.w	r0, ip, r4
	2800      	cmp	r0, #0
	dc03      	bgt.n	<smoltcp::socket::tcp::Socket::process+0x4a0>
	4623      	mov	r3, r4
	ebb4 000b 	subs.w	r0, r4, fp
	d409      	bmi.n	<smoltcp::socket::tcp::Socket::process+0x4b4>
	f240 0020 	movw	r0, #32
	460e      	mov	r6, r1
	f2c0 0000 	movt	r0, #0
	e6ae      	b.n	<smoltcp::socket::tcp::Socket::process+0x20a>
	4623      	mov	r3, r4
	2800      	cmp	r0, #0
	bfc8      	it	gt
	4663      	movgt	r3, ip
	2001      	movs	r0, #1
	e9c1 0428 	strd	r0, r4, [r1, #160]	@ 0xa0
	1b18      	subs	r0, r3, r4
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f340 8660 	ble.w	<smoltcp::socket::tcp::Socket::process+0x1184>
	ebbb 0205 	subs.w	r2, fp, r5
	9603      	str	r6, [sp, #12]
	bf48      	it	mi
	465d      	movmi	r5, fp
	1b2a      	subs	r2, r5, r4
	f1b2 3fff 	cmp.w	r2, #4294967295	@ 0xffffffff
	f340 8656 	ble.w	<smoltcp::socket::tcp::Socket::process+0x1184>
	46e2      	mov	sl, ip
	f8dd c038 	ldr.w	ip, [sp, #56]	@ 0x38
	4282      	cmp	r2, r0
	f0c0 8666 	bcc.w	<smoltcp::socket::tcp::Socket::process+0x11b0>
	9d0b      	ldr	r5, [sp, #44]	@ 0x2c
	42aa      	cmp	r2, r5
	f200 8662 	bhi.w	<smoltcp::socket::tcp::Socket::process+0x11b0>
	eba3 030a 	sub.w	r3, r3, sl
	9409      	str	r4, [sp, #36]	@ 0x24
	f1b3 3fff 	cmp.w	r3, #4294967295	@ 0xffffffff
	f340 8645 	ble.w	<smoltcp::socket::tcp::Socket::process+0x1184>
	461d      	mov	r5, r3
	f8dc 303c 	ldr.w	r3, [ip, #60]	@ 0x3c
	eba2 0c00 	sub.w	ip, r2, r0
	4676      	mov	r6, lr
	4418      	add	r0, r3
	462b      	mov	r3, r5
	4605      	mov	r5, r0
	980d      	ldr	r0, [sp, #52]	@ 0x34
	f04f 0b00 	mov.w	fp, #0
	2e04      	cmp	r6, #4
	f47f ae10 	bne.w	<smoltcp::socket::tcp::Socket::process+0x138>
	2400      	movs	r4, #0
	f04f 0800 	mov.w	r8, #0
	1e72      	subs	r2, r6, #1
	bf18      	it	ne
	4632      	movne	r2, r6
	9809      	ldr	r0, [sp, #36]	@ 0x24
	ebba 0000 	subs.w	r0, sl, r0
	4610      	mov	r0, r2
	bf48      	it	mi
	2000      	movmi	r0, #0
	2a03      	cmp	r2, #3
	bf18      	it	ne
	4610      	movne	r0, r2
	f1b9 0f01 	cmp.w	r9, #1
	d039      	beq.n	<smoltcp::socket::tcp::Socket::process+0x5b0>
	f1b9 0f03 	cmp.w	r9, #3
	b2c0      	uxtb	r0, r0
	bf08      	it	eq
	2804      	cmpeq	r0, #4
	f000 8082 	beq.w	<smoltcp::socket::tcp::Socket::process+0x64e>
	e8df f010 	tbh	[pc, r0, lsl #1]
	0005      	.short	0x0005
	00980039 	.word	0x00980039
	008c00eb 	.word	0x008c00eb
	f1a9 0003 	sub.w	r0, r9, #3
	2806      	cmp	r0, #6
	d82f      	bhi.n	<smoltcp::socket::tcp::Socket::process+0x5c0>
	e9cd 5301 	strd	r5, r3, [sp, #4]
	f8cd c010 	str.w	ip, [sp, #16]
	9406      	str	r4, [sp, #24]
	e8df f010 	tbh	[pc, r0, lsl #1]
	02c9      	.short	0x02c9
	02a902e7 	.word	0x02a902e7
	00070007 	.word	0x00070007
	0435030a 	.word	0x0435030a
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	4634      	mov	r4, r6
	6c8b      	ldr	r3, [r1, #72]	@ 0x48
	f04f 0e00 	mov.w	lr, #0
	e9da 5004 	ldrd	r5, r0, [sl, #16]
	6cce      	ldr	r6, [r1, #76]	@ 0x4c
	195b      	adds	r3, r3, r5
	f8d1 c040 	ldr.w	ip, [r1, #64]	@ 0x40
	4170      	adcs	r0, r6
	6c4a      	ldr	r2, [r1, #68]	@ 0x44
	4626      	mov	r6, r4
	f8c1 e080 	str.w	lr, [r1, #128]	@ 0x80
	f8c1 c088 	str.w	ip, [r1, #136]	@ 0x88
	f8c1 e084 	str.w	lr, [r1, #132]	@ 0x84
	f8c1 208c 	str.w	r2, [r1, #140]	@ 0x8c
	f8c1 3090 	str.w	r3, [r1, #144]	@ 0x90
	e2c2      	b.n	<smoltcp::socket::tcp::Socket::process+0xb36>
	b2c0      	uxtb	r0, r0
	e8df f010 	tbh	[pc, r0, lsl #1]
	0005      	.short	0x0005
	00ef0005 	.word	0x00ef0005
	005d0005 	.word	0x005d0005
	f240 001f 	movw	r0, #31
	f2c0 0000 	movt	r0, #0
	f7fd fc70 	bl	<defmt::export::acquire_and_header>
	f240 0907 	movw	r9, #7
	a810      	add	r0, sp, #64	@ 0x40
	f2c0 0900 	movt	r9, #0
	2102      	movs	r1, #2
	f8ad 9040 	strh.w	r9, [sp, #64]	@ 0x40
	f7fd fe18 	bl	<_defmt_write>
	f240 003c 	movw	r0, #60	@ 0x3c
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fe0e 	bl	<_defmt_write>
	9c0e      	ldr	r4, [sp, #56]	@ 0x38
	f240 0a04 	movw	sl, #4
	a810      	add	r0, sp, #64	@ 0x40
	f2c0 0a00 	movt	sl, #0
	2102      	movs	r1, #2
	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
	f8b4 504a 	ldrh.w	r5, [r4, #74]	@ 0x4a
	f7fd fe01 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
	f7fd fdfb 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8b4 504c 	ldrh.w	r5, [r4, #76]	@ 0x4c
	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
	f7fd fdf3 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
	f7fd fded 	bl	<_defmt_write>
	e8df f016 	tbh	[pc, r6, lsl #1]
	0149      	.short	0x0149
	013a0005 	.word	0x013a0005
	0135013f 	.word	0x0135013f
	f240 0037 	movw	r0, #55	@ 0x37
	f2c0 0000 	movt	r0, #0
	e138      	b.n	<smoltcp::socket::tcp::Socket::process+0x8c0>
	f8b1 20b0 	ldrh.w	r2, [r1, #176]	@ 0xb0
	2a00      	cmp	r2, #0
	f43f af79 	beq.w	<smoltcp::socket::tcp::Socket::process+0x54a>
	2001      	movs	r0, #1
	f881 0133 	strb.w	r0, [r1, #307]	@ 0x133
	2000      	movs	r0, #0
	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
	e004      	b.n	<smoltcp::socket::tcp::Socket::process+0x670>
	2000      	movs	r0, #0
	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
	f881 0133 	strb.w	r0, [r1, #307]	@ 0x133
	990c      	ldr	r1, [sp, #48]	@ 0x30
	2002      	movs	r0, #2
	6108      	str	r0, [r1, #16]
	b011      	add	sp, #68	@ 0x44
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f1b9 0f01 	cmp.w	r9, #1
	f000 8087 	beq.w	<smoltcp::socket::tcp::Socket::process+0x794>
	f1b9 0f02 	cmp.w	r9, #2
	d199      	bne.n	<smoltcp::socket::tcp::Socket::process+0x5c0>
	f8cd c010 	str.w	ip, [sp, #16]
	9501      	str	r5, [sp, #4]
	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
	f8dd a024 	ldr.w	sl, [sp, #36]	@ 0x24
	f8bb 0038 	ldrh.w	r0, [fp, #56]	@ 0x38
	b128      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0x6ac>
	f8bb 003a 	ldrh.w	r0, [fp, #58]	@ 0x3a
	2800      	cmp	r0, #0
	d0e3      	beq.n	<smoltcp::socket::tcp::Socket::process+0x670>
	f8c1 0110 	str.w	r0, [r1, #272]	@ 0x110
	46b0      	mov	r8, r6
	f10a 0601 	add.w	r6, sl, #1
	f89b 0048 	ldrb.w	r0, [fp, #72]	@ 0x48
	2501      	movs	r5, #1
	f8c1 6104 	str.w	r6, [r1, #260]	@ 0x104
	f04f 0e00 	mov.w	lr, #0
	f8d1 6100 	ldr.w	r6, [r1, #256]	@ 0x100
	2800      	cmp	r0, #0
	9406      	str	r4, [sp, #24]
	9302      	str	r3, [sp, #8]
	f89b 2049 	ldrb.w	r2, [fp, #73]	@ 0x49
	f89b 3050 	ldrb.w	r3, [fp, #80]	@ 0x50
	f881 212d 	strb.w	r2, [r1, #301]	@ 0x12d
	f106 0201 	add.w	r2, r6, #1
	f8c1 2108 	str.w	r2, [r1, #264]	@ 0x108
	f04f 0204 	mov.w	r2, #4
	e9c1 5a26 	strd	r5, sl, [r1, #152]	@ 0x98
	f881 3131 	strb.w	r3, [r1, #305]	@ 0x131
	f881 012c 	strb.w	r0, [r1, #300]	@ 0x12c
	bf04      	itt	eq
	2000      	moveq	r0, #0
	f881 0134 	strbeq.w	r0, [r1, #308]	@ 0x134
	f8db 002c 	ldr.w	r0, [fp, #44]	@ 0x2c
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	2800      	cmp	r0, #0
	bf04      	itt	eq
	2000      	moveq	r0, #0
	f8c1 0114 	streq.w	r0, [r1, #276]	@ 0x114
	9b0d      	ldr	r3, [sp, #52]	@ 0x34
	e9d1 4612 	ldrd	r4, r6, [r1, #72]	@ 0x48
	e9da 5004 	ldrd	r5, r0, [sl, #16]
	2b00      	cmp	r3, #0
	e9d1 c910 	ldrd	ip, r9, [r1, #64]	@ 0x40
	bf08      	it	eq
	2203      	moveq	r2, #3
	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
	192a      	adds	r2, r5, r4
	e0b0      	b.n	<smoltcp::socket::tcp::Socket::process+0x886>
	f1a9 0003 	sub.w	r0, r9, #3
	2803      	cmp	r0, #3
	f63f af49 	bhi.w	<smoltcp::socket::tcp::Socket::process+0x5c0>
	e9cd 5301 	strd	r5, r3, [sp, #4]
	f8cd c010 	str.w	ip, [sp, #16]
	9406      	str	r4, [sp, #24]
	e8df f010 	tbh	[pc, r0, lsl #1]
	00040004 	.word	0x00040004
	01b10195 	.word	0x01b10195
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	4634      	mov	r4, r6
	2607      	movs	r6, #7
	9803      	ldr	r0, [sp, #12]
	e9da 8e04 	ldrd	r8, lr, [sl, #16]
	f04f 0c00 	mov.w	ip, #0
	6c8b      	ldr	r3, [r1, #72]	@ 0x48
	f881 6133 	strb.w	r6, [r1, #307]	@ 0x133
	2601      	movs	r6, #1
	6cca      	ldr	r2, [r1, #76]	@ 0x4c
	eb13 0308 	adds.w	r3, r3, r8
	f881 6130 	strb.w	r6, [r1, #304]	@ 0x130
	f100 0601 	add.w	r6, r0, #1
	6c0d      	ldr	r5, [r1, #64]	@ 0x40
	eb42 020e 	adc.w	r2, r2, lr
	6c48      	ldr	r0, [r1, #68]	@ 0x44
	f8c1 6104 	str.w	r6, [r1, #260]	@ 0x104
	4626      	mov	r6, r4
	f8c1 c080 	str.w	ip, [r1, #128]	@ 0x80
	f8c1 5088 	str.w	r5, [r1, #136]	@ 0x88
	f8c1 3090 	str.w	r3, [r1, #144]	@ 0x90
	f8c1 c084 	str.w	ip, [r1, #132]	@ 0x84
	f8c1 008c 	str.w	r0, [r1, #140]	@ 0x8c
	f8c1 2094 	str.w	r2, [r1, #148]	@ 0x94
	e212      	b.n	<smoltcp::socket::tcp::Socket::process+0xbba>
	f8cd c010 	str.w	ip, [sp, #16]
	e9cd 5301 	strd	r5, r3, [sp, #4]
	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
	e9dd 3a08 	ldrd	r3, sl, [sp, #32]
	f8bb 0038 	ldrh.w	r0, [fp, #56]	@ 0x38
	b130      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0x7b8>
	f8bb 003a 	ldrh.w	r0, [fp, #58]	@ 0x3a
	2800      	cmp	r0, #0
	f43f af5e 	beq.w	<smoltcp::socket::tcp::Socket::process+0x670>
	f8c1 0110 	str.w	r0, [r1, #272]	@ 0x110
	f8dd e028 	ldr.w	lr, [sp, #40]	@ 0x28
	2001      	movs	r0, #1
	f8c1 3124 	str.w	r3, [r1, #292]	@ 0x124
	46b0      	mov	r8, r6
	9b07      	ldr	r3, [sp, #28]
	f10a 0601 	add.w	r6, sl, #1
	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
	f8c1 311e 	str.w	r3, [r1, #286]	@ 0x11e
	f8bb 304a 	ldrh.w	r3, [fp, #74]	@ 0x4a
	e9de 0206 	ldrd	r0, r2, [lr, #24]
	f8a1 3128 	strh.w	r3, [r1, #296]	@ 0x128
	f246 131d 	movw	r3, #24861	@ 0x611d
	f8bb 504c 	ldrh.w	r5, [fp, #76]	@ 0x4c
	f6c3 4339 	movt	r3, #15417	@ 0x3c39
	f8c1 6104 	str.w	r6, [r1, #260]	@ 0x104
	fb02 fc03 	mul.w	ip, r2, r3
	f8a1 5122 	strh.w	r5, [r1, #290]	@ 0x122
	fba0 6503 	umull	r6, r5, r0, r3
	f64f 43ec 	movw	r3, #64748	@ 0xfcec
	f6cb 332e 	movt	r3, #47918	@ 0xbb2e
	f89b 2050 	ldrb.w	r2, [fp, #80]	@ 0x50
	fb00 5003 	mla	r0, r0, r3, r5
	f881 2131 	strb.w	r2, [r1, #305]	@ 0x131
	f64e 7239 	movw	r2, #61241	@ 0xef39
	f2c7 5290 	movt	r2, #30096	@ 0x7590
	f89b 5049 	ldrb.w	r5, [fp, #73]	@ 0x49
	1992      	adds	r2, r2, r6
	f04f 061d 	mov.w	r6, #29
	f881 512d 	strb.w	r5, [r1, #301]	@ 0x12d
	eb40 000c 	adc.w	r0, r0, ip
	f89b 3048 	ldrb.w	r3, [fp, #72]	@ 0x48
	f881 312c 	strb.w	r3, [r1, #300]	@ 0x12c
	eba6 7650 	sub.w	r6, r6, r0, lsr #29
	e9ce 2006 	strd	r2, r0, [lr, #24]
	f086 051f 	eor.w	r5, r6, #31
	0040      	lsls	r0, r0, #1
	40f2      	lsrs	r2, r6
	2b00      	cmp	r3, #0
	fa00 f005 	lsl.w	r0, r0, r5
	f04f 0e00 	mov.w	lr, #0
	ea40 0002 	orr.w	r0, r0, r2
	f8c1 0108 	str.w	r0, [r1, #264]	@ 0x108
	f8c1 0100 	str.w	r0, [r1, #256]	@ 0x100
	bf04      	itt	eq
	2000      	moveq	r0, #0
	f881 0134 	strbeq.w	r0, [r1, #308]	@ 0x134
	f8db 002c 	ldr.w	r0, [fp, #44]	@ 0x2c
	2203      	movs	r2, #3
	9406      	str	r4, [sp, #24]
	2800      	cmp	r0, #0
	bf04      	itt	eq
	2000      	moveq	r0, #0
	f8c1 0114 	streq.w	r0, [r1, #276]	@ 0x114
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	e9d1 3612 	ldrd	r3, r6, [r1, #72]	@ 0x48
	e9da 5004 	ldrd	r5, r0, [sl, #16]
	e9d1 c910 	ldrd	ip, r9, [r1, #64]	@ 0x40
	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
	18ea      	adds	r2, r5, r3
	4170      	adcs	r0, r6
	f8c1 e080 	str.w	lr, [r1, #128]	@ 0x80
	f8c1 c088 	str.w	ip, [r1, #136]	@ 0x88
	4646      	mov	r6, r8
	f8c1 2090 	str.w	r2, [r1, #144]	@ 0x90
	f8c1 e084 	str.w	lr, [r1, #132]	@ 0x84
	f8c1 908c 	str.w	r9, [r1, #140]	@ 0x8c
	f8c1 0094 	str.w	r0, [r1, #148]	@ 0x94
	e18c      	b.n	<smoltcp::socket::tcp::Socket::process+0xbbe>
	f240 0038 	movw	r0, #56	@ 0x38
	f2c0 0000 	movt	r0, #0
	e008      	b.n	<smoltcp::socket::tcp::Socket::process+0x8c0>
	f240 003a 	movw	r0, #58	@ 0x3a
	f2c0 0000 	movt	r0, #0
	e003      	b.n	<smoltcp::socket::tcp::Socket::process+0x8c0>
	f240 0034 	movw	r0, #52	@ 0x34
	f2c0 0000 	movt	r0, #0
	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f7fd fca2 	bl	<_defmt_write>
	f240 0039 	movw	r0, #57	@ 0x39
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fc98 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad 9040 	strh.w	r9, [sp, #64]	@ 0x40
	f7fd fc92 	bl	<_defmt_write>
	f240 0b3d 	movw	fp, #61	@ 0x3d
	a810      	add	r0, sp, #64	@ 0x40
	f2c0 0b00 	movt	fp, #0
	2102      	movs	r1, #2
	f8ad b040 	strh.w	fp, [sp, #64]	@ 0x40
	f7fd fc88 	bl	<_defmt_write>
	f240 0508 	movw	r5, #8
	a810      	add	r0, sp, #64	@ 0x40
	f2c0 0500 	movt	r5, #0
	2102      	movs	r1, #2
	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
	f7fd fc7e 	bl	<_defmt_write>
	9809      	ldr	r0, [sp, #36]	@ 0x24
	2104      	movs	r1, #4
	9010      	str	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fc78 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2600      	movs	r6, #0
	2102      	movs	r1, #2
	f8ad 6040 	strh.w	r6, [sp, #64]	@ 0x40
	f7fd fc71 	bl	<_defmt_write>
	f8dd 8030 	ldr.w	r8, [sp, #48]	@ 0x30
	980d      	ldr	r0, [sp, #52]	@ 0x34
	b338      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0x986>
	f240 0033 	movw	r0, #51	@ 0x33
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fc63 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad 9040 	strh.w	r9, [sp, #64]	@ 0x40
	f7fd fc5d 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad b040 	strh.w	fp, [sp, #64]	@ 0x40
	f7fd fc57 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
	f7fd fc51 	bl	<_defmt_write>
	980f      	ldr	r0, [sp, #60]	@ 0x3c
	2104      	movs	r1, #4
	9010      	str	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fc4b 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad 6040 	strh.w	r6, [sp, #64]	@ 0x40
	f7fd fc45 	bl	<_defmt_write>
	f240 003b 	movw	r0, #59	@ 0x3b
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	2602      	movs	r6, #2
	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fc3a 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8b4 504e 	ldrh.w	r5, [r4, #78]	@ 0x4e
	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
	f7fd fc32 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
	f7fd fc2c 	bl	<_defmt_write>
	f240 0035 	movw	r0, #53	@ 0x35
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fc22 	bl	<_defmt_write>
	f240 0006 	movw	r0, #6
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fc18 	bl	<_defmt_write>
	980b      	ldr	r0, [sp, #44]	@ 0x2c
	2104      	movs	r1, #4
	9010      	str	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fc12 	bl	<_defmt_write>
	8f20      	ldrh	r0, [r4, #56]	@ 0x38
	2801      	cmp	r0, #1
	d116      	bne.n	<smoltcp::socket::tcp::Socket::process+0xa20>
	f240 0036 	movw	r0, #54	@ 0x36
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	8f65      	ldrh	r5, [r4, #58]	@ 0x3a
	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fc04 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
	f7fd fbfe 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
	f7fd fbf8 	bl	<_defmt_write>
	2000      	movs	r0, #0
	2102      	movs	r1, #2
	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd fbf1 	bl	<_defmt_write>
	f7fd fb77 	bl	<_defmt_release>
	f8c8 6010 	str.w	r6, [r8, #16]
	b011      	add	sp, #68	@ 0x44
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f240 000c 	movw	r0, #12
	4665      	mov	r5, ip
	f2c0 0000 	movt	r0, #0
	f7fd fa30 	bl	<defmt::export::acquire_and_header>
	4628      	mov	r0, r5
	f000 ff40 	bl	<defmt::export::fmt>
	4640      	mov	r0, r8
	f000 ff3d 	bl	<defmt::export::fmt>
	980f      	ldr	r0, [sp, #60]	@ 0x3c
	f000 ff3a 	bl	<defmt::export::fmt>
	f7fd fb5f 	bl	<_defmt_release>
	f7ff bc01 	b.w	<smoltcp::socket::tcp::Socket::process+0x268>
	2001      	movs	r0, #1
	f1b8 0f00 	cmp.w	r8, #0
	f881 0130 	strb.w	r0, [r1, #304]	@ 0x130
	9803      	ldr	r0, [sp, #12]
	f100 0001 	add.w	r0, r0, #1
	f8c1 0104 	str.w	r0, [r1, #260]	@ 0x104
	f040 8086 	bne.w	<smoltcp::socket::tcp::Socket::process+0xb8a>
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	2208      	movs	r2, #8
	e9d1 3412 	ldrd	r3, r4, [r1, #72]	@ 0x48
	f04f 0e00 	mov.w	lr, #0
	e9da 5004 	ldrd	r5, r0, [sl, #16]
	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
	18ea      	adds	r2, r5, r3
	e9d1 c810 	ldrd	ip, r8, [r1, #64]	@ 0x40
	4160      	adcs	r0, r4
	e041      	b.n	<smoltcp::socket::tcp::Socket::process+0xb22>
	250a      	movs	r5, #10
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	f881 5133 	strb.w	r5, [r1, #307]	@ 0x133
	2501      	movs	r5, #1
	f881 5130 	strb.w	r5, [r1, #304]	@ 0x130
	2000      	movs	r0, #0
	9d03      	ldr	r5, [sp, #12]
	2403      	movs	r4, #3
	e9da 2304 	ldrd	r2, r3, [sl, #16]
	3501      	adds	r5, #1
	f8c1 5104 	str.w	r5, [r1, #260]	@ 0x104
	e06d      	b.n	<smoltcp::socket::tcp::Socket::process+0xb9c>
	f1b8 0f00 	cmp.w	r8, #0
	bf1c      	itt	ne
	2006      	movne	r0, #6
	f881 0133 	strbne.w	r0, [r1, #307]	@ 0x133
	f1bb 0f00 	cmp.w	fp, #0
	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	d071      	beq.n	<smoltcp::socket::tcp::Socket::process+0xbbe>
	6c8b      	ldr	r3, [r1, #72]	@ 0x48
	4634      	mov	r4, r6
	e9da 5004 	ldrd	r5, r0, [sl, #16]
	f04f 0e00 	mov.w	lr, #0
	6cce      	ldr	r6, [r1, #76]	@ 0x4c
	195b      	adds	r3, r3, r5
	e9d1 c210 	ldrd	ip, r2, [r1, #64]	@ 0x40
	4170      	adcs	r0, r6
	4626      	mov	r6, r4
	e9c1 ee20 	strd	lr, lr, [r1, #128]	@ 0x80
	e9c1 c222 	strd	ip, r2, [r1, #136]	@ 0x88
	e9c1 3024 	strd	r3, r0, [r1, #144]	@ 0x90
	e05e      	b.n	<smoltcp::socket::tcp::Socket::process+0xbbe>
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	4634      	mov	r4, r6
	6c8b      	ldr	r3, [r1, #72]	@ 0x48
	2204      	movs	r2, #4
	e9da 5004 	ldrd	r5, r0, [sl, #16]
	f04f 0e00 	mov.w	lr, #0
	6cce      	ldr	r6, [r1, #76]	@ 0x4c
	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
	18ea      	adds	r2, r5, r3
	4170      	adcs	r0, r6
	e9d1 c810 	ldrd	ip, r8, [r1, #64]	@ 0x40
	4626      	mov	r6, r4
	f8c1 e080 	str.w	lr, [r1, #128]	@ 0x80
	f8c1 c088 	str.w	ip, [r1, #136]	@ 0x88
	f8c1 e084 	str.w	lr, [r1, #132]	@ 0x84
	f8c1 808c 	str.w	r8, [r1, #140]	@ 0x8c
	f8c1 2090 	str.w	r2, [r1, #144]	@ 0x90
	f8c1 0094 	str.w	r0, [r1, #148]	@ 0x94
	e03e      	b.n	<smoltcp::socket::tcp::Socket::process+0xbba>
	f8d1 0080 	ldr.w	r0, [r1, #128]	@ 0x80
	f04f 0c00 	mov.w	ip, #0
	1ec2      	subs	r2, r0, #3
	f112 0f02 	cmn.w	r2, #2
	f04f 0200 	mov.w	r2, #0
	bf38      	it	cc
	2201      	movcc	r2, #1
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	ea52 020b 	orrs.w	r2, r2, fp
	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
	d02e      	beq.n	<smoltcp::socket::tcp::Socket::process+0xbbe>
	4634      	mov	r4, r6
	e9d1 6512 	ldrd	r6, r5, [r1, #72]	@ 0x48
	e9da 0204 	ldrd	r0, r2, [sl, #16]
	e9d1 e310 	ldrd	lr, r3, [r1, #64]	@ 0x40
	1980      	adds	r0, r0, r6
	4626      	mov	r6, r4
	416a      	adcs	r2, r5
	e9c1 cc20 	strd	ip, ip, [r1, #128]	@ 0x80
	e9c1 e322 	strd	lr, r3, [r1, #136]	@ 0x88
	e9c1 0224 	strd	r0, r2, [r1, #144]	@ 0x90
	e01d      	b.n	<smoltcp::socket::tcp::Socket::process+0xbbe>
	f1b8 0f00 	cmp.w	r8, #0
	f000 813d 	beq.w	<smoltcp::socket::tcp::Socket::process+0xe04>
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	250a      	movs	r5, #10
	2000      	movs	r0, #0
	2403      	movs	r4, #3
	e9da 2304 	ldrd	r2, r3, [sl, #16]
	f881 5133 	strb.w	r5, [r1, #307]	@ 0x133
	f249 6580 	movw	r5, #38528	@ 0x9680
	f8c1 4080 	str.w	r4, [r1, #128]	@ 0x80
	f2c0 0598 	movt	r5, #152	@ 0x98
	1952      	adds	r2, r2, r5
	f143 0300 	adc.w	r3, r3, #0
	f8c1 2088 	str.w	r2, [r1, #136]	@ 0x88
	f8c1 0084 	str.w	r0, [r1, #132]	@ 0x84
	f8c1 308c 	str.w	r3, [r1, #140]	@ 0x8c
	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
	e9da 3904 	ldrd	r3, r9, [sl, #16]
	2200      	movs	r2, #0
	2001      	movs	r0, #1
	2e02      	cmp	r6, #2
	e9c1 0214 	strd	r0, r2, [r1, #80]	@ 0x50
	930a      	str	r3, [sp, #40]	@ 0x28
	e9c1 3916 	strd	r3, r9, [r1, #88]	@ 0x58
	d007      	beq.n	<smoltcp::socket::tcp::Socket::process+0xbe4>
	f891 212c 	ldrb.w	r2, [r1, #300]	@ 0x12c
	f891 012d 	ldrb.w	r0, [r1, #301]	@ 0x12d
	2a00      	cmp	r2, #0
	bf18      	it	ne
	f000 021f 	andne.w	r2, r0, #31
	f8bb 304e 	ldrh.w	r3, [fp, #78]	@ 0x4e
	9d06      	ldr	r5, [sp, #24]
	f8d1 010c 	ldr.w	r0, [r1, #268]	@ 0x10c
	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
	fa03 f202 	lsl.w	r2, r3, r2
	9c04      	ldr	r4, [sp, #16]
	f8c1 210c 	str.w	r2, [r1, #268]	@ 0x10c
	b1ad      	cbz	r5, <smoltcp::socket::tcp::Socket::process+0xc2a>
	f8d1 30d4 	ldr.w	r3, [r1, #212]	@ 0xd4
	42ab      	cmp	r3, r5
	f0c0 82dc 	bcc.w	<smoltcp::socket::tcp::Socket::process+0x11c0>
	1b5e      	subs	r6, r3, r5
	f8d1 30cc 	ldr.w	r3, [r1, #204]	@ 0xcc
	f8c1 60d4 	str.w	r6, [r1, #212]	@ 0xd4
	b13b      	cbz	r3, <smoltcp::socket::tcp::Socket::process+0xc24>
	f8d1 60d0 	ldr.w	r6, [r1, #208]	@ 0xd0
	442e      	add	r6, r5
	fbb6 f5f3 	udiv	r5, r6, r3
	fb05 6313 	mls	r3, r5, r3, r6
	e000      	b.n	<smoltcp::socket::tcp::Socket::process+0xc26>
	2300      	movs	r3, #0
	f8c1 30d0 	str.w	r3, [r1, #208]	@ 0xd0
	9b0d      	ldr	r3, [sp, #52]	@ 0x34
	2b00      	cmp	r3, #0
	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
	d06e      	beq.n	<smoltcp::socket::tcp::Socket::process+0xd10>
	9e0b      	ldr	r6, [sp, #44]	@ 0x2c
	2e00      	cmp	r6, #0
	d14f      	bne.n	<smoltcp::socket::tcp::Socket::process+0xcd8>
	f8d1 30a8 	ldr.w	r3, [r1, #168]	@ 0xa8
	2b00      	cmp	r3, #0
	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
	d04a      	beq.n	<smoltcp::socket::tcp::Socket::process+0xcd8>
	f8d1 30ac 	ldr.w	r3, [r1, #172]	@ 0xac
	9e0f      	ldr	r6, [sp, #60]	@ 0x3c
	42b3      	cmp	r3, r6
	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
	bf08      	it	eq
	4282      	cmpeq	r2, r0
	d142      	bne.n	<smoltcp::socket::tcp::Socket::process+0xcd8>
	f8d1 0108 	ldr.w	r0, [r1, #264]	@ 0x108
	1a18      	subs	r0, r3, r0
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	dc3c      	bgt.n	<smoltcp::socket::tcp::Socket::process+0xcd8>
	f991 0135 	ldrsb.w	r0, [r1, #309]	@ 0x135
	f04f 0801 	mov.w	r8, #1
	4625      	mov	r5, r4
	460c      	mov	r4, r1
	fa80 f658 	uqadd8	r6, r0, r8
	f240 0013 	movw	r0, #19
	f2c0 0000 	movt	r0, #0
	f881 6135 	strb.w	r6, [r1, #309]	@ 0x135
	f7fd f917 	bl	<defmt::export::acquire_and_header>
	980f      	ldr	r0, [sp, #60]	@ 0x3c
	f000 fe27 	bl	<defmt::export::fmt>
	f894 0135 	ldrb.w	r0, [r4, #309]	@ 0x135
	f000 fe0c 	bl	<defmt::export::fmt>
	b2f1      	uxtb	r1, r6
	29ff      	cmp	r1, #255	@ 0xff
	f06f 01fe 	mvn.w	r1, #254	@ 0xfe
	f64a 5060 	movw	r0, #44384	@ 0xad60
	fa51 f186 	uxtab	r1, r1, r6
	f6c0 0000 	movt	r0, #2048	@ 0x800
	fab1 f181 	clz	r1, r1
	bf18      	it	ne
	4640      	movne	r0, r8
	0949      	lsrs	r1, r1, #5
	f000 fde0 	bl	<defmt::export::fmt>
	f7fd fa37 	bl	<_defmt_release>
	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
	4621      	mov	r1, r4
	f894 0135 	ldrb.w	r0, [r4, #309]	@ 0x135
	2803      	cmp	r0, #3
	d11f      	bne.n	<smoltcp::socket::tcp::Socket::process+0xcfe>
	2000      	movs	r0, #0
	2202      	movs	r2, #2
	e9c1 2020 	strd	r2, r0, [r1, #128]	@ 0x80
	f240 0018 	movw	r0, #24
	f2c0 0000 	movt	r0, #0
	f7fd f8fe 	bl	<defmt::export::acquire_header_and_release>
	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
	4621      	mov	r1, r4
	e012      	b.n	<smoltcp::socket::tcp::Socket::process+0xcfe>
	f891 0135 	ldrb.w	r0, [r1, #309]	@ 0x135
	4625      	mov	r5, r4
	b158      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0xcf8>
	2000      	movs	r0, #0
	460c      	mov	r4, r1
	f881 0135 	strb.w	r0, [r1, #309]	@ 0x135
	f240 0014 	movw	r0, #20
	f2c0 0000 	movt	r0, #0
	f7fd f8ed 	bl	<defmt::export::acquire_header_and_release>
	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
	4621      	mov	r1, r4
	2001      	movs	r0, #1
	e9c1 032a 	strd	r0, r3, [r1, #168]	@ 0xa8
	f8d1 0108 	ldr.w	r0, [r1, #264]	@ 0x108
	462c      	mov	r4, r5
	f8c1 3100 	str.w	r3, [r1, #256]	@ 0x100
	1ac0      	subs	r0, r0, r3
	bf48      	it	mi
	f8c1 3108 	strmi.w	r3, [r1, #264]	@ 0x108
	980e      	ldr	r0, [sp, #56]	@ 0x38
	9b02      	ldr	r3, [sp, #8]
	6ac0      	ldr	r0, [r0, #44]	@ 0x2c
	2800      	cmp	r0, #0
	bf1e      	ittt	ne
	980e      	ldrne	r0, [sp, #56]	@ 0x38
	6b00      	ldrne	r0, [r0, #48]	@ 0x30
	f8c1 0118 	strne.w	r0, [r1, #280]	@ 0x118
	2c00      	cmp	r4, #0
	f43f aaa0 	beq.w	<smoltcp::socket::tcp::Socket::process+0x268>
	4688      	mov	r8, r1
	9404      	str	r4, [sp, #16]
	f858 6fd8 	ldr.w	r6, [r8, #216]!
	f8cd 9024 	str.w	r9, [sp, #36]	@ 0x24
	f8d8 0004 	ldr.w	r0, [r8, #4]
	900f      	str	r0, [sp, #60]	@ 0x3c
	b933      	cbnz	r3, <smoltcp::socket::tcp::Socket::process+0xd4a>
	42a6      	cmp	r6, r4
	d904      	bls.n	<smoltcp::socket::tcp::Socket::process+0xd4a>
	46a4      	mov	ip, r4
	1b30      	subs	r0, r6, r4
	f8c8 0000 	str.w	r0, [r8]
	e15a      	b.n	<smoltcp::socket::tcp::Socket::process+0x1000>
	b398      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0xdb4>
	eb06 0e00 	add.w	lr, r6, r0
	4573      	cmp	r3, lr
	d934      	bls.n	<smoltcp::socket::tcp::Socket::process+0xdbe>
	f8d1 50e4 	ldr.w	r5, [r1, #228]	@ 0xe4
	eba3 090e 	sub.w	r9, r3, lr
	f101 0be0 	add.w	fp, r1, #224	@ 0xe0
	b355      	cbz	r5, <smoltcp::socket::tcp::Socket::process+0xdb8>
	f8db 3000 	ldr.w	r3, [fp]
	eb03 0c05 	add.w	ip, r3, r5
	45e1      	cmp	r9, ip
	d942      	bls.n	<smoltcp::socket::tcp::Socket::process+0xdf4>
	f8d1 50ec 	ldr.w	r5, [r1, #236]	@ 0xec
	eba9 090c 	sub.w	r9, r9, ip
	f101 0be8 	add.w	fp, r1, #232	@ 0xe8
	2d00      	cmp	r5, #0
	d050      	beq.n	<smoltcp::socket::tcp::Socket::process+0xe20>
	f8db 3000 	ldr.w	r3, [fp]
	eb03 0c05 	add.w	ip, r3, r5
	45e1      	cmp	r9, ip
	d94e      	bls.n	<smoltcp::socket::tcp::Socket::process+0xe28>
	f8d1 50f4 	ldr.w	r5, [r1, #244]	@ 0xf4
	eba9 090c 	sub.w	r9, r9, ip
	f101 0bf0 	add.w	fp, r1, #240	@ 0xf0
	2d00      	cmp	r5, #0
	d042      	beq.n	<smoltcp::socket::tcp::Socket::process+0xe20>
	f8db 3000 	ldr.w	r3, [fp]
	eb03 0c05 	add.w	ip, r3, r5
	45e1      	cmp	r9, ip
	d853      	bhi.n	<smoltcp::socket::tcp::Socket::process+0xe4e>
	2203      	movs	r2, #3
	920d      	str	r2, [sp, #52]	@ 0x34
	2201      	movs	r2, #1
	9206      	str	r2, [sp, #24]
	2200      	movs	r2, #0
	920b      	str	r2, [sp, #44]	@ 0x2c
	e040      	b.n	<smoltcp::socket::tcp::Socket::process+0xe36>
	4699      	mov	r9, r3
	46c3      	mov	fp, r8
	e9cb 9400 	strd	r9, r4, [fp]
	e0fb      	b.n	<smoltcp::socket::tcp::Socket::process+0xfb6>
	2200      	movs	r2, #0
	4699      	mov	r9, r3
	9206      	str	r2, [sp, #24]
	2201      	movs	r2, #1
	920b      	str	r2, [sp, #44]	@ 0x2c
	4605      	mov	r5, r0
	9205      	str	r2, [sp, #20]
	2200      	movs	r2, #0
	4633      	mov	r3, r6
	46f4      	mov	ip, lr
	920d      	str	r2, [sp, #52]	@ 0x34
	46c3      	mov	fp, r8
	e02f      	b.n	<smoltcp::socket::tcp::Socket::process+0xe38>
	f1b8 0f00 	cmp.w	r8, #0
	f43f abce 	beq.w	<smoltcp::socket::tcp::Socket::process+0x57c>
	2000      	movs	r0, #0
	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
	f881 0133 	strb.w	r0, [r1, #307]	@ 0x133
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	e6e4      	b.n	<smoltcp::socket::tcp::Socket::process+0xbbe>
	2201      	movs	r2, #1
	920b      	str	r2, [sp, #44]	@ 0x2c
	2200      	movs	r2, #0
	9206      	str	r2, [sp, #24]
	2201      	movs	r2, #1
	9205      	str	r2, [sp, #20]
	920d      	str	r2, [sp, #52]	@ 0x34
	e019      	b.n	<smoltcp::socket::tcp::Socket::process+0xe38>
	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
	f04f 0e00 	mov.w	lr, #0
	e9d1 3412 	ldrd	r3, r4, [r1, #72]	@ 0x48
	e9da 5004 	ldrd	r5, r0, [sl, #16]
	e9d1 c210 	ldrd	ip, r2, [r1, #64]	@ 0x40
	195b      	adds	r3, r3, r5
	4160      	adcs	r0, r4
	f7ff bbbd 	b.w	<smoltcp::socket::tcp::Socket::process+0x59a>
	9b02      	ldr	r3, [sp, #8]
	e9cb 9400 	strd	r9, r4, [fp]
	e0c6      	b.n	<smoltcp::socket::tcp::Socket::process+0xfb6>
	2202      	movs	r2, #2
	920d      	str	r2, [sp, #52]	@ 0x34
	2200      	movs	r2, #0
	9206      	str	r2, [sp, #24]
	2201      	movs	r2, #1
	920b      	str	r2, [sp, #44]	@ 0x2c
	2200      	movs	r2, #0
	9205      	str	r2, [sp, #20]
	eb09 0a04 	add.w	sl, r9, r4
	4599      	cmp	r9, r3
	d231      	bcs.n	<smoltcp::socket::tcp::Socket::process+0xea4>
	459a      	cmp	sl, r3
	d231      	bcs.n	<smoltcp::socket::tcp::Socket::process+0xea8>
	f8d1 00f4 	ldr.w	r0, [r1, #244]	@ 0xf4
	2800      	cmp	r0, #0
	f000 8096 	beq.w	<smoltcp::socket::tcp::Socket::process+0xf7a>
	f240 000b 	movw	r0, #11
	f2c0 0000 	movt	r0, #0
	f7fd f829 	bl	<defmt::export::acquire_and_header>
	f240 0506 	movw	r5, #6
	a810      	add	r0, sp, #64	@ 0x40
	f2c0 0500 	movt	r5, #0
	2102      	movs	r1, #2
	4626      	mov	r6, r4
	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
	2402      	movs	r4, #2
	f7fd f9cf 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2104      	movs	r1, #4
	9610      	str	r6, [sp, #64]	@ 0x40
	f7fd f9ca 	bl	<_defmt_write>
	a810      	add	r0, sp, #64	@ 0x40
	2102      	movs	r1, #2
	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
	f7fd f9c4 	bl	<_defmt_write>
	9802      	ldr	r0, [sp, #8]
	2104      	movs	r1, #4
	9010      	str	r0, [sp, #64]	@ 0x40
	a810      	add	r0, sp, #64	@ 0x40
	f7fd f9be 	bl	<_defmt_write>
	f7fd f944 	bl	<_defmt_release>
	980c      	ldr	r0, [sp, #48]	@ 0x30
	6104      	str	r4, [r0, #16]
	b011      	add	sp, #68	@ 0x44
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	4699      	mov	r9, r3
	e003      	b.n	<smoltcp::socket::tcp::Socket::process+0xeb0>
	ebac 0509 	sub.w	r5, ip, r9
	e9cb 9500 	strd	r9, r5, [fp]
	f8dd e034 	ldr.w	lr, [sp, #52]	@ 0x34
	980b      	ldr	r0, [sp, #44]	@ 0x2c
	f10e 0c01 	add.w	ip, lr, #1
	b1b0      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0xeea>
	4672      	mov	r2, lr
	4666      	mov	r6, ip
	eb01 03c2 	add.w	r3, r1, r2, lsl #3
	f8d3 40e4 	ldr.w	r4, [r3, #228]	@ 0xe4
	b18c      	cbz	r4, <smoltcp::socket::tcp::Socket::process+0xeee>
	f8d3 30e0 	ldr.w	r3, [r3, #224]	@ 0xe0
	442b      	add	r3, r5
	eb03 0009 	add.w	r0, r3, r9
	4582      	cmp	sl, r0
	d30b      	bcc.n	<smoltcp::socket::tcp::Socket::process+0xef0>
	191d      	adds	r5, r3, r4
	3201      	adds	r2, #1
	3601      	adds	r6, #1
	2a03      	cmp	r2, #3
	f8cb 5004 	str.w	r5, [fp, #4]
	d1ec      	bne.n	<smoltcp::socket::tcp::Socket::process+0xec0>
	2604      	movs	r6, #4
	e002      	b.n	<smoltcp::socket::tcp::Socket::process+0xef0>
	9b02      	ldr	r3, [sp, #8]
	e02e      	b.n	<smoltcp::socket::tcp::Socket::process+0xf4c>
	1c56      	adds	r6, r2, #1
	9b02      	ldr	r3, [sp, #8]
	ea6f 020e 	mvn.w	r2, lr
	1992      	adds	r2, r2, r6
	d028      	beq.n	<smoltcp::socket::tcp::Socket::process+0xf4c>
	f8cd c02c 	str.w	ip, [sp, #44]	@ 0x2c
	eb01 09ce 	add.w	r9, r1, lr, lsl #3
	eb01 0cc6 	add.w	ip, r1, r6, lsl #3
	f08e 0203 	eor.w	r2, lr, #3
	2500      	movs	r5, #0
	9302      	str	r3, [sp, #8]
	e008      	b.n	<smoltcp::socket::tcp::Socket::process+0xf22>
	eb0c 00c5 	add.w	r0, ip, r5, lsl #3
	e9d0 3e36 	ldrd	r3, lr, [r0, #216]	@ 0xd8
	3501      	adds	r5, #1
	e9c4 3e38 	strd	r3, lr, [r4, #224]	@ 0xe0
	42aa      	cmp	r2, r5
	d00f      	beq.n	<smoltcp::socket::tcp::Socket::process+0xf42>
	eb09 04c5 	add.w	r4, r9, r5, lsl #3
	f8d4 00e4 	ldr.w	r0, [r4, #228]	@ 0xe4
	b150      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0xf42>
	1970      	adds	r0, r6, r5
	2803      	cmp	r0, #3
	d9ee      	bls.n	<smoltcp::socket::tcp::Socket::process+0xf10>
	2300      	movs	r3, #0
	f04f 0e00 	mov.w	lr, #0
	3501      	adds	r5, #1
	e9c4 3e38 	strd	r3, lr, [r4, #224]	@ 0xe0
	42aa      	cmp	r2, r5
	d1ef      	bne.n	<smoltcp::socket::tcp::Socket::process+0xf22>
	e9db 9500 	ldrd	r9, r5, [fp]
	9b02      	ldr	r3, [sp, #8]
	f8dd c02c 	ldr.w	ip, [sp, #44]	@ 0x2c
	eb05 0209 	add.w	r2, r5, r9
	4592      	cmp	sl, r2
	d930      	bls.n	<smoltcp::socket::tcp::Socket::process+0xfb6>
	ebaa 0202 	sub.w	r2, sl, r2
	1950      	adds	r0, r2, r5
	f8cb 0004 	str.w	r0, [fp, #4]
	9806      	ldr	r0, [sp, #24]
	bb48      	cbnz	r0, <smoltcp::socket::tcp::Socket::process+0xfb6>
	eb08 00cc 	add.w	r0, r8, ip, lsl #3
	461e      	mov	r6, r3
	6843      	ldr	r3, [r0, #4]
	2b00      	cmp	r3, #0
	4633      	mov	r3, r6
	bf1f      	itttt	ne
	6803      	ldrne	r3, [r0, #0]
	1a9a      	subne	r2, r3, r2
	4633      	movne	r3, r6
	6002      	strne	r2, [r0, #0]
	e01d      	b.n	<smoltcp::socket::tcp::Socket::process+0xfb6>
	9806      	ldr	r0, [sp, #24]
	2800      	cmp	r0, #0
	f040 8144 	bne.w	<smoltcp::socket::tcp::Socket::process+0x120a>
	e9d1 033a 	ldrd	r0, r3, [r1, #232]	@ 0xe8
	e9c1 033c 	strd	r0, r3, [r1, #240]	@ 0xf0
	9b02      	ldr	r3, [sp, #8]
	9805      	ldr	r0, [sp, #20]
	b140      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0xfa2>
	e9d1 0238 	ldrd	r0, r2, [r1, #224]	@ 0xe0
	4573      	cmp	r3, lr
	e9c1 023a 	strd	r0, r2, [r1, #232]	@ 0xe8
	bf9c      	itt	ls
	980f      	ldrls	r0, [sp, #60]	@ 0x3c
	e9c1 6038 	strdls	r6, r0, [r1, #224]	@ 0xe0
	f8db 0008 	ldr.w	r0, [fp, #8]
	f8cb 4004 	str.w	r4, [fp, #4]
	eba0 000a 	sub.w	r0, r0, sl
	f8cb 9000 	str.w	r9, [fp]
	f8cb 0008 	str.w	r0, [fp, #8]
	f8d1 00d8 	ldr.w	r0, [r1, #216]	@ 0xd8
	f04f 0c00 	mov.w	ip, #0
	b9e8      	cbnz	r0, <smoltcp::socket::tcp::Socket::process+0xffc>
	f8d1 00dc 	ldr.w	r0, [r1, #220]	@ 0xdc
	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
	b1d0      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0x1000>
	461d      	mov	r5, r3
	e9d1 2338 	ldrd	r2, r3, [r1, #224]	@ 0xe0
	e9c1 2336 	strd	r2, r3, [r1, #216]	@ 0xd8
	2b00      	cmp	r3, #0
	bf1e      	ittt	ne
	e9d1 233a 	ldrdne	r2, r3, [r1, #232]	@ 0xe8
	e9c1 2338 	strdne	r2, r3, [r1, #224]	@ 0xe0
	2b00      	cmpne	r3, #0
	d008      	beq.n	<smoltcp::socket::tcp::Socket::process+0xff6>
	e9d1 233c 	ldrd	r2, r3, [r1, #240]	@ 0xf0
	2600      	movs	r6, #0
	f101 0ce8 	add.w	ip, r1, #232	@ 0xe8
	f8c1 60f4 	str.w	r6, [r1, #244]	@ 0xf4
	e88c 004c 	stmia.w	ip, {r2, r3, r6}
	4684      	mov	ip, r0
	462b      	mov	r3, r5
	e001      	b.n	<smoltcp::socket::tcp::Socket::process+0x1000>
	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
	f8d1 40bc 	ldr.w	r4, [r1, #188]	@ 0xbc
	f8d1 50c4 	ldr.w	r5, [r1, #196]	@ 0xc4
	b144      	cbz	r4, <smoltcp::socket::tcp::Socket::process+0x101c>
	f8d1 00c0 	ldr.w	r0, [r1, #192]	@ 0xc0
	18ea      	adds	r2, r5, r3
	4410      	add	r0, r2
	fbb0 f2f4 	udiv	r2, r0, r4
	fb02 0014 	mls	r0, r2, r4, r0
	e000      	b.n	<smoltcp::socket::tcp::Socket::process+0x101e>
	2000      	movs	r0, #0
	eba4 0805 	sub.w	r8, r4, r5
	f8cd c034 	str.w	ip, [sp, #52]	@ 0x34
	4598      	cmp	r8, r3
	d205      	bcs.n	<smoltcp::socket::tcp::Socket::process+0x1036>
	46d1      	mov	r9, sl
	469b      	mov	fp, r3
	468a      	mov	sl, r1
	2600      	movs	r6, #0
	2001      	movs	r0, #1
	e015      	b.n	<smoltcp::socket::tcp::Socket::process+0x1062>
	9a04      	ldr	r2, [sp, #16]
	eba8 0603 	sub.w	r6, r8, r3
	42b2      	cmp	r2, r6
	bf38      	it	cc
	4616      	movcc	r6, r2
	1a22      	subs	r2, r4, r0
	4296      	cmp	r6, r2
	bf28      	it	cs
	4616      	movcs	r6, r2
	1832      	adds	r2, r6, r0
	f080 80c3 	bcs.w	<smoltcp::socket::tcp::Socket::process+0x11d6>
	42a2      	cmp	r2, r4
	f200 80c0 	bhi.w	<smoltcp::socket::tcp::Socket::process+0x11d6>
	46d1      	mov	r9, sl
	468a      	mov	sl, r1
	f8d1 10b8 	ldr.w	r1, [r1, #184]	@ 0xb8
	469b      	mov	fp, r3
	4408      	add	r0, r1
	9901      	ldr	r1, [sp, #4]
	4632      	mov	r2, r6
	f001 fef5 	bl	<__aeabi_memcpy>
	eb06 010b 	add.w	r1, r6, fp
	b14c      	cbz	r4, <smoltcp::socket::tcp::Socket::process+0x1084>
	f8da 00c0 	ldr.w	r0, [sl, #192]	@ 0xc0
	194a      	adds	r2, r1, r5
	4653      	mov	r3, sl
	4410      	add	r0, r2
	fbb0 f2f4 	udiv	r2, r0, r4
	fb02 0014 	mls	r0, r2, r4, r0
	e001      	b.n	<smoltcp::socket::tcp::Socket::process+0x1088>
	2000      	movs	r0, #0
	4653      	mov	r3, sl
	4588      	cmp	r8, r1
	d203      	bcs.n	<smoltcp::socket::tcp::Socket::process+0x1094>
	461c      	mov	r4, r3
	2200      	movs	r2, #0
	2001      	movs	r0, #1
	e015      	b.n	<smoltcp::socket::tcp::Socket::process+0x10c0>
	9a04      	ldr	r2, [sp, #16]
	1b95      	subs	r5, r2, r6
	eba8 0201 	sub.w	r2, r8, r1
	4295      	cmp	r5, r2
	eba4 0100 	sub.w	r1, r4, r0
	bf38      	it	cc
	462a      	movcc	r2, r5
	428a      	cmp	r2, r1
	bf28      	it	cs
	460a      	movcs	r2, r1
	1811      	adds	r1, r2, r0
	f080 809a 	bcs.w	<smoltcp::socket::tcp::Socket::process+0x11e6>
	42a1      	cmp	r1, r4
	f200 8097 	bhi.w	<smoltcp::socket::tcp::Socket::process+0x11e6>
	f8d3 10b8 	ldr.w	r1, [r3, #184]	@ 0xb8
	461c      	mov	r4, r3
	4408      	add	r0, r1
	9901      	ldr	r1, [sp, #4]
	4431      	add	r1, r6
	f001 fec6 	bl	<__aeabi_memcpy>
	9b0d      	ldr	r3, [sp, #52]	@ 0x34
	4621      	mov	r1, r4
	b153      	cbz	r3, <smoltcp::socket::tcp::Socket::process+0x10e4>
	f8d1 20bc 	ldr.w	r2, [r1, #188]	@ 0xbc
	f8d1 00c4 	ldr.w	r0, [r1, #196]	@ 0xc4
	1a12      	subs	r2, r2, r0
	4293      	cmp	r3, r2
	f200 808b 	bhi.w	<smoltcp::socket::tcp::Socket::process+0x11f4>
	4418      	add	r0, r3
	f8c1 00c4 	str.w	r0, [r1, #196]	@ 0xc4
	6e08      	ldr	r0, [r1, #96]	@ 0x60
	07c0      	lsls	r0, r0, #31
	d034      	beq.n	<smoltcp::socket::tcp::Socket::process+0x1154>
	f8d1 0098 	ldr.w	r0, [r1, #152]	@ 0x98
	2801      	cmp	r0, #1
	d130      	bne.n	<smoltcp::socket::tcp::Socket::process+0x1154>
	f8d1 00c4 	ldr.w	r0, [r1, #196]	@ 0xc4
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f77e aff3 	ble.w	<smoltcp::socket::tcp::Socket::process+0xe4>
	f8d1 3104 	ldr.w	r3, [r1, #260]	@ 0x104
	f8d1 209c 	ldr.w	r2, [r1, #156]	@ 0x9c
	4418      	add	r0, r3
	1a10      	subs	r0, r2, r0
	d523      	bpl.n	<smoltcp::socket::tcp::Socket::process+0x1154>
	6f08      	ldr	r0, [r1, #112]	@ 0x70
	2802      	cmp	r0, #2
	d00a      	beq.n	<smoltcp::socket::tcp::Socket::process+0x1128>
	2801      	cmp	r0, #1
	d10b      	bne.n	<smoltcp::socket::tcp::Socket::process+0x112e>
	4608      	mov	r0, r1
	460c      	mov	r4, r1
	f000 f9cf 	bl	<smoltcp::socket::tcp::Socket::immediate_ack_to_transmit>
	b180      	cbz	r0, <smoltcp::socket::tcp::Socket::process+0x1142>
	2000      	movs	r0, #0
	2202      	movs	r2, #2
	4621      	mov	r1, r4
	e011      	b.n	<smoltcp::socket::tcp::Socket::process+0x114c>
	2000      	movs	r0, #0
	2202      	movs	r2, #2
	e00e      	b.n	<smoltcp::socket::tcp::Socket::process+0x114c>
	e9d1 201a 	ldrd	r2, r0, [r1, #104]	@ 0x68
	9b0a      	ldr	r3, [sp, #40]	@ 0x28
	189b      	adds	r3, r3, r2
	9a09      	ldr	r2, [sp, #36]	@ 0x24
	eb40 0602 	adc.w	r6, r0, r2
	2000      	movs	r0, #0
	2201      	movs	r2, #1
	e004      	b.n	<smoltcp::socket::tcp::Socket::process+0x114c>
	4621      	mov	r1, r4
	e9d4 201c 	ldrd	r2, r0, [r4, #112]	@ 0x70
	e9d4 361e 	ldrd	r3, r6, [r4, #120]	@ 0x78
	e9c1 201c 	strd	r2, r0, [r1, #112]	@ 0x70
	e9c1 361e 	strd	r3, r6, [r1, #120]	@ 0x78
	f8d1 00dc 	ldr.w	r0, [r1, #220]	@ 0xdc
	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
	4310      	orrs	r0, r2
	d00b      	beq.n	<smoltcp::socket::tcp::Socket::process+0x1176>
	980e      	ldr	r0, [sp, #56]	@ 0x38
	60b8      	str	r0, [r7, #8]
	4648      	mov	r0, r9
	9a08      	ldr	r2, [sp, #32]
	9b07      	ldr	r3, [sp, #28]
	b011      	add	sp, #68	@ 0x44
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
	f000 b89f 	b.w	<smoltcp::socket::tcp::Socket::ack_reply>
	2002      	movs	r0, #2
	f8c9 0010 	str.w	r0, [r9, #16]
	b011      	add	sp, #68	@ 0x44
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f64a 209c 	movw	r0, #43676	@ 0xaa9c
	f64a 22d0 	movw	r2, #43728	@ 0xaad0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2167      	movs	r1, #103	@ 0x67
	f7fc f8cf 	bl	<core::panicking::panic_fmt>
	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
	f64a 5250 	movw	r2, #44368	@ 0xad50
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2128      	movs	r1, #40	@ 0x28
	f7fc f8e6 	bl	<core::panicking::panic>
	4611      	mov	r1, r2
	9a0b      	ldr	r2, [sp, #44]	@ 0x2c
	f64a 539c 	movw	r3, #44444	@ 0xad9c
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f7fc f84b 	bl	<core::slice::index::slice_index_fail>
	f64a 60cc 	movw	r0, #44748	@ 0xaecc
	f64a 62f4 	movw	r2, #44788	@ 0xaef4
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2125      	movs	r1, #37	@ 0x25
	f7fc f8d3 	bl	<core::panicking::panic>
	f64a 63ac 	movw	r3, #44716	@ 0xaeac
	4611      	mov	r1, r2
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4622      	mov	r2, r4
	f7fc f838 	bl	<core::slice::index::slice_index_fail>
	f64a 63ac 	movw	r3, #44716	@ 0xaeac
	4622      	mov	r2, r4
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f7fc f831 	bl	<core::slice::index::slice_index_fail>
	f64a 5061 	movw	r0, #44385	@ 0xad61
	f64a 528c 	movw	r2, #44428	@ 0xad8c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2128      	movs	r1, #40	@ 0x28
	f7fc f8b9 	bl	<core::panicking::panic>
	f24b 0248 	movw	r2, #45128	@ 0xb048
	2004      	movs	r0, #4
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2104      	movs	r1, #4
	e9cb 9400 	strd	r9, r4, [fp]
	f7fc f89b 	bl	<core::panicking::panic_bounds_check>

<smoltcp::socket::tcp::Socket::rst_reply>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	f84d 8d04 	str.w	r8, [sp, #-4]!
	f8b3 c04c 	ldrh.w	ip, [r3, #76]	@ 0x4c
	f8b3 e04a 	ldrh.w	lr, [r3, #74]	@ 0x4a
	681c      	ldr	r4, [r3, #0]
	2c01      	cmp	r4, #1
	d102      	bne.n	<smoltcp::socket::tcp::Socket::rst_reply+0x1c>
	685d      	ldr	r5, [r3, #4]
	2400      	movs	r4, #0
	e010      	b.n	<smoltcp::socket::tcp::Socket::rst_reply+0x3e>
	f893 4051 	ldrb.w	r4, [r3, #81]	@ 0x51
	2c02      	cmp	r4, #2
	d10a      	bne.n	<smoltcp::socket::tcp::Socket::rst_reply+0x3a>
	6c1c      	ldr	r4, [r3, #64]	@ 0x40
	3401      	adds	r4, #1
	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
	dd28      	ble.n	<smoltcp::socket::tcp::Socket::rst_reply+0x80>
	6c5b      	ldr	r3, [r3, #68]	@ 0x44
	2500      	movs	r5, #0
	eb04 0803 	add.w	r8, r4, r3
	2401      	movs	r4, #1
	e001      	b.n	<smoltcp::socket::tcp::Socket::rst_reply+0x3e>
	2500      	movs	r5, #0
	2400      	movs	r4, #0
	f04f 6680 	mov.w	r6, #67108864	@ 0x4000000
	2301      	movs	r3, #1
	f8c0 605e 	str.w	r6, [r0, #94]	@ 0x5e
	2600      	movs	r6, #0
	e9c0 3613 	strd	r3, r6, [r0, #76]	@ 0x4c
	f44f 7350 	mov.w	r3, #832	@ 0x340
	8183      	strh	r3, [r0, #12]
	2314      	movs	r3, #20
	f8a0 e05c 	strh.w	lr, [r0, #92]	@ 0x5c
	f8a0 c05a 	strh.w	ip, [r0, #90]	@ 0x5a
	f880 6058 	strb.w	r6, [r0, #88]	@ 0x58
	6545      	str	r5, [r0, #84]	@ 0x54
	f8a0 6048 	strh.w	r6, [r0, #72]	@ 0x48
	63c6      	str	r6, [r0, #60]	@ 0x3c
	6306      	str	r6, [r0, #48]	@ 0x30
	6246      	str	r6, [r0, #36]	@ 0x24
	e9c0 4804 	strd	r4, r8, [r0, #16]
	6186      	str	r6, [r0, #24]
	e9c0 2100 	strd	r2, r1, [r0]
	6083      	str	r3, [r0, #8]
	f85d 8b04 	ldr.w	r8, [sp], #4
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f64a 4078 	movw	r0, #44152	@ 0xac78
	f64a 42b0 	movw	r2, #44208	@ 0xacb0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2171      	movs	r1, #113	@ 0x71
	f7fc f842 	bl	<core::panicking::panic_fmt>

<smoltcp::socket::tcp::Socket::ack_reply>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b089      	sub	sp, #36	@ 0x24
	4604      	mov	r4, r0
	68b8      	ldr	r0, [r7, #8]
	4689      	mov	r9, r1
	6ac1      	ldr	r1, [r0, #44]	@ 0x2c
	f8b0 b04c 	ldrh.w	fp, [r0, #76]	@ 0x4c
	f8b0 504a 	ldrh.w	r5, [r0, #74]	@ 0x4a
	07c9      	lsls	r1, r1, #31
	bf1c      	itt	ne
	f8d9 1114 	ldrne.w	r1, [r9, #276]	@ 0x114
	2900      	cmpne	r1, #0
	f040 8085 	bne.w	<smoltcp::socket::tcp::Socket::ack_reply+0x132>
	2600      	movs	r6, #0
	2117      	movs	r1, #23
	f8d9 00c4 	ldr.w	r0, [r9, #196]	@ 0xc4
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f340 808d 	ble.w	<smoltcp::socket::tcp::Socket::ack_reply+0x152>
	e9cd 6106 	strd	r6, r1, [sp, #24]
	f04f 0c01 	mov.w	ip, #1
	e9d9 1641 	ldrd	r1, r6, [r9, #260]	@ 0x104
	f64f 78ff 	movw	r8, #65535	@ 0xffff
	9508      	str	r5, [sp, #32]
	9605      	str	r6, [sp, #20]
	eb00 0a01 	add.w	sl, r0, r1
	f8d9 50bc 	ldr.w	r5, [r9, #188]	@ 0xbc
	f899 6134 	ldrb.w	r6, [r9, #308]	@ 0x134
	1a28      	subs	r0, r5, r0
	f899 e131 	ldrb.w	lr, [r9, #305]	@ 0x131
	f006 011f 	and.w	r1, r6, #31
	e9c9 ca26 	strd	ip, sl, [r9, #152]	@ 0x98
	40c8      	lsrs	r0, r1
	4540      	cmp	r0, r8
	bf38      	it	cc
	4680      	movcc	r8, r0
	f1be 0f00 	cmp.w	lr, #0
	f8a9 812e 	strh.w	r8, [r9, #302]	@ 0x12e
	f000 8093 	beq.w	<smoltcp::socket::tcp::Socket::ack_reply+0x1a0>
	f240 0017 	movw	r0, #23
	e9cd 3201 	strd	r3, r2, [sp, #4]
	f2c0 0000 	movt	r0, #0
	f7fc fdc8 	bl	<defmt::export::acquire_header_and_release>
	f8d9 00a0 	ldr.w	r0, [r9, #160]	@ 0xa0
	2801      	cmp	r0, #1
	d147      	bne.n	<smoltcp::socket::tcp::Socket::ack_reply+0x122>
	f8d9 c0a4 	ldr.w	ip, [r9, #164]	@ 0xa4
	f109 00d8 	add.w	r0, r9, #216	@ 0xd8
	2600      	movs	r6, #0
	2200      	movs	r2, #0
	e000      	b.n	<smoltcp::socket::tcp::Socket::ack_reply+0xa2>
	3201      	adds	r2, #1
	2a03      	cmp	r2, #3
	d83d      	bhi.n	<smoltcp::socket::tcp::Socket::ack_reply+0x122>
	eb00 03c2 	add.w	r3, r0, r2, lsl #3
	f850 1032 	ldr.w	r1, [r0, r2, lsl #3]
	685d      	ldr	r5, [r3, #4]
	1873      	adds	r3, r6, r1
	195e      	adds	r6, r3, r5
	42b3      	cmp	r3, r6
	d325      	bcc.n	<smoltcp::socket::tcp::Socket::ack_reply+0x104>
	1c51      	adds	r1, r2, #1
	2904      	cmp	r1, #4
	d031      	beq.n	<smoltcp::socket::tcp::Socket::ack_reply+0x122>
	eb00 05c1 	add.w	r5, r0, r1, lsl #3
	f850 6031 	ldr.w	r6, [r0, r1, lsl #3]
	686d      	ldr	r5, [r5, #4]
	4433      	add	r3, r6
	195e      	adds	r6, r3, r5
	42b3      	cmp	r3, r6
	d30b      	bcc.n	<smoltcp::socket::tcp::Socket::ack_reply+0xe8>
	1c91      	adds	r1, r2, #2
	2904      	cmp	r1, #4
	d025      	beq.n	<smoltcp::socket::tcp::Socket::ack_reply+0x122>
	eb00 05c1 	add.w	r5, r0, r1, lsl #3
	f850 6031 	ldr.w	r6, [r0, r1, lsl #3]
	686d      	ldr	r5, [r5, #4]
	4433      	add	r3, r6
	195e      	adds	r6, r3, r5
	42b3      	cmp	r3, r6
	d201      	bcs.n	<smoltcp::socket::tcp::Socket::ack_reply+0xec>
	460a      	mov	r2, r1
	e00b      	b.n	<smoltcp::socket::tcp::Socket::ack_reply+0x104>
	3203      	adds	r2, #3
	2a04      	cmp	r2, #4
	d017      	beq.n	<smoltcp::socket::tcp::Socket::ack_reply+0x122>
	eb00 06c2 	add.w	r6, r0, r2, lsl #3
	f850 1032 	ldr.w	r1, [r0, r2, lsl #3]
	6876      	ldr	r6, [r6, #4]
	440b      	add	r3, r1
	441e      	add	r6, r3
	42b3      	cmp	r3, r6
	d20e      	bcs.n	<smoltcp::socket::tcp::Socket::ack_reply+0x122>
	eb0a 0503 	add.w	r5, sl, r3
	4565      	cmp	r5, ip
	d8c9      	bhi.n	<smoltcp::socket::tcp::Socket::ack_reply+0xa0>
	eb0a 0106 	add.w	r1, sl, r6
	4561      	cmp	r1, ip
	d3c5      	bcc.n	<smoltcp::socket::tcp::Socket::ack_reply+0xa0>
	f04f 090a 	mov.w	r9, #10
	f04f 0c01 	mov.w	ip, #1
	e9dd 3201 	ldrd	r3, r2, [sp, #4]
	e042      	b.n	<smoltcp::socket::tcp::Socket::ack_reply+0x1a8>
	e9d9 0136 	ldrd	r0, r1, [r9, #216]	@ 0xd8
	4401      	add	r1, r0
	4288      	cmp	r0, r1
	d21d      	bcs.n	<smoltcp::socket::tcp::Socket::ack_reply+0x168>
	e9dd 3201 	ldrd	r3, r2, [sp, #4]
	e02e      	b.n	<smoltcp::socket::tcp::Socket::ack_reply+0x190>
	6b00      	ldr	r0, [r0, #48]	@ 0x30
	4690      	mov	r8, r2
	9004      	str	r0, [sp, #16]
	461e      	mov	r6, r3
	4788      	blx	r1
	4633      	mov	r3, r6
	4642      	mov	r2, r8
	2601      	movs	r6, #1
	2121      	movs	r1, #33	@ 0x21
	9003      	str	r0, [sp, #12]
	f8d9 00c4 	ldr.w	r0, [r9, #196]	@ 0xc4
	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
	f73f af73 	bgt.w	<smoltcp::socket::tcp::Socket::ack_reply+0x38>
	f64a 4078 	movw	r0, #44152	@ 0xac78
	f64a 42b0 	movw	r2, #44208	@ 0xacb0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2171      	movs	r1, #113	@ 0x71
	f7fb ff8e 	bl	<core::panicking::panic_fmt>
	e9d9 1238 	ldrd	r1, r2, [r9, #224]	@ 0xe0
	4408      	add	r0, r1
	1881      	adds	r1, r0, r2
	e9dd 3201 	ldrd	r3, r2, [sp, #4]
	4288      	cmp	r0, r1
	d30b      	bcc.n	<smoltcp::socket::tcp::Socket::ack_reply+0x190>
	e9d9 163a 	ldrd	r1, r6, [r9, #232]	@ 0xe8
	4408      	add	r0, r1
	1981      	adds	r1, r0, r6
	4288      	cmp	r0, r1
	d305      	bcc.n	<smoltcp::socket::tcp::Socket::ack_reply+0x190>
	e9d9 163c 	ldrd	r1, r6, [r9, #240]	@ 0xf0
	4408      	add	r0, r1
	1981      	adds	r1, r0, r6
	4288      	cmp	r0, r1
	d207      	bcs.n	<smoltcp::socket::tcp::Socket::ack_reply+0x1a0>
	4451      	add	r1, sl
	eb0a 0500 	add.w	r5, sl, r0
	f04f 0c01 	mov.w	ip, #1
	f04f 090a 	mov.w	r9, #10
	e003      	b.n	<smoltcp::socket::tcp::Socket::ack_reply+0x1a8>
	f04f 0c00 	mov.w	ip, #0
	f04f 0900 	mov.w	r9, #0
	9808      	ldr	r0, [sp, #32]
	2600      	movs	r6, #0
	f8a4 005c 	strh.w	r0, [r4, #92]	@ 0x5c
	f04f 0e01 	mov.w	lr, #1
	9805      	ldr	r0, [sp, #20]
	6560      	str	r0, [r4, #84]	@ 0x54
	9806      	ldr	r0, [sp, #24]
	63e0      	str	r0, [r4, #60]	@ 0x3c
	9803      	ldr	r0, [sp, #12]
	6420      	str	r0, [r4, #64]	@ 0x40
	9804      	ldr	r0, [sp, #16]
	6460      	str	r0, [r4, #68]	@ 0x44
	9807      	ldr	r0, [sp, #28]
	e9c4 3200 	strd	r3, r2, [r4]
	f104 0208 	add.w	r2, r4, #8
	4448      	add	r0, r9
	e9c4 1608 	strd	r1, r6, [r4, #32]
	f000 007c 	and.w	r0, r0, #124	@ 0x7c
	f44f 7150 	mov.w	r1, #832	@ 0x340
	f8a4 6060 	strh.w	r6, [r4, #96]	@ 0x60
	f8a4 805e 	strh.w	r8, [r4, #94]	@ 0x5e
	f8a4 b05a 	strh.w	fp, [r4, #90]	@ 0x5a
	f884 6058 	strb.w	r6, [r4, #88]	@ 0x58
	e9c4 e613 	strd	lr, r6, [r4, #76]	@ 0x4c
	f8a4 6048 	strh.w	r6, [r4, #72]	@ 0x48
	6326      	str	r6, [r4, #48]	@ 0x30
	e882 4003 	stmia.w	r2, {r0, r1, lr}
	e9c4 ac05 	strd	sl, ip, [r4, #20]
	61e5      	str	r5, [r4, #28]
	b009      	add	sp, #36	@ 0x24
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<smoltcp::socket::tcp::Socket::immediate_ack_to_transmit>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	f8d0 1098 	ldr.w	r1, [r0, #152]	@ 0x98
	2901      	cmp	r1, #1
	bf1c      	itt	ne
	2000      	movne	r0, #0
	bd80      	popne	{r7, pc}
	f8d0 1110 	ldr.w	r1, [r0, #272]	@ 0x110
	f1b1 3fff 	cmp.w	r1, #4294967295	@ 0xffffffff
	bfc1      	itttt	gt
	f8d0 20c4 	ldrgt.w	r2, [r0, #196]	@ 0xc4
	f1b2 3fff 	cmpgt.w	r2, #4294967295	@ 0xffffffff
	f8d0 309c 	ldrgt.w	r3, [r0, #156]	@ 0x9c
	f8d0 0104 	ldrgt.w	r0, [r0, #260]	@ 0x104
	bfc1      	itttt	gt
	4410      	addgt	r0, r2
	4419      	addgt	r1, r3
	1a08      	subgt	r0, r1, r0
	0fc0      	lsrgt	r0, r0, #31
	bfc8      	it	gt
	bd80      	popgt	{r7, pc}
	f64a 4078 	movw	r0, #44152	@ 0xac78
	f64a 42b0 	movw	r2, #44208	@ 0xacb0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2171      	movs	r1, #113	@ 0x71
	f7fb ff17 	bl	<core::panicking::panic_fmt>

<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	f002 06f0 	and.w	r6, r2, #240	@ 0xf0
	2ee0      	cmp	r6, #224	@ 0xe0
	d002      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
	1c56      	adds	r6, r2, #1
	2e02      	cmp	r6, #2
	d204      	bcs.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x20>
	2104      	movs	r1, #4
	6001      	str	r1, [r0, #0]
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f8d1 c0f0 	ldr.w	ip, [r1, #240]	@ 0xf0
	f1bc 0f00 	cmp.w	ip, #0
	eb0c 068c 	add.w	r6, ip, ip, lsl #2
	440e      	add	r6, r1
	f106 0ef4 	add.w	lr, r6, #244	@ 0xf4
	d023      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x7c>
	f101 04f4 	add.w	r4, r1, #244	@ 0xf4
	f04f 38ff 	mov.w	r8, #4294967295	@ 0xffffffff
	e00d      	b.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x5a>
	2500      	movs	r5, #0
	f006 061f 	and.w	r6, r6, #31
	ea05 0509 	and.w	r5, r5, r9
	fa28 f606 	lsr.w	r6, r8, r6
	ba36      	rev	r6, r6
	4335      	orrs	r5, r6
	42aa      	cmp	r2, r5
	d0e0      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
	3405      	adds	r4, #5
	4574      	cmp	r4, lr
	d010      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x7c>
	7926      	ldrb	r6, [r4, #4]
	f8d4 9000 	ldr.w	r9, [r4]
	2e00      	cmp	r6, #0
	d0ec      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x3e>
	f1a6 051f 	sub.w	r5, r6, #31
	b2ed      	uxtb	r5, r5
	2d02      	cmp	r5, #2
	d3f2      	bcc.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x54>
	4275      	negs	r5, r6
	f005 051f 	and.w	r5, r5, #31
	fa08 f505 	lsl.w	r5, r8, r5
	ba2d      	rev	r5, r5
	e7e1      	b.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x40>
	f8d7 9008 	ldr.w	r9, [r7, #8]
	f003 04f0 	and.w	r4, r3, #240	@ 0xf0
	2ce0      	cmp	r4, #224	@ 0xe0
	d003      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x90>
	f113 0801 	adds.w	r8, r3, #1
	d070      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x170>
	bb33      	cbnz	r3, <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xde>
	f1bc 0f00 	cmp.w	ip, #0
	d0bf      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
	f101 05f4 	add.w	r5, r1, #244	@ 0xf4
	f04f 38ff 	mov.w	r8, #4294967295	@ 0xffffffff
	e00d      	b.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xbc>
	2600      	movs	r6, #0
	f004 041f 	and.w	r4, r4, #31
	ea06 060a 	and.w	r6, r6, sl
	fa28 f404 	lsr.w	r4, r8, r4
	ba24      	rev	r4, r4
	4334      	orrs	r4, r6
	42a3      	cmp	r3, r4
	d05c      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x170>
	3505      	adds	r5, #5
	4575      	cmp	r5, lr
	d0ac      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
	792c      	ldrb	r4, [r5, #4]
	f8d5 a000 	ldr.w	sl, [r5]
	2c00      	cmp	r4, #0
	d0ec      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xa0>
	f1a4 061f 	sub.w	r6, r4, #31
	b2f6      	uxtb	r6, r6
	2e02      	cmp	r6, #2
	d3f2      	bcc.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xb6>
	4266      	negs	r6, r4
	f006 061f 	and.w	r6, r6, #31
	fa08 f606 	lsl.w	r6, r8, r6
	ba36      	rev	r6, r6
	e7e1      	b.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xa2>
	f1bc 0f00 	cmp.w	ip, #0
	d023      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x12c>
	f101 06f4 	add.w	r6, r1, #244	@ 0xf4
	f04f 3aff 	mov.w	sl, #4294967295	@ 0xffffffff
	e00d      	b.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x10a>
	2500      	movs	r5, #0
	f004 041f 	and.w	r4, r4, #31
	ea05 050b 	and.w	r5, r5, fp
	fa2a f404 	lsr.w	r4, sl, r4
	ba24      	rev	r4, r4
	432c      	orrs	r4, r5
	42a3      	cmp	r3, r4
	d032      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16a>
	3605      	adds	r6, #5
	4576      	cmp	r6, lr
	d010      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x12c>
	7934      	ldrb	r4, [r6, #4]
	f8d6 b000 	ldr.w	fp, [r6]
	2c00      	cmp	r4, #0
	d0ec      	beq.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xee>
	f1a4 051f 	sub.w	r5, r4, #31
	b2ed      	uxtb	r5, r5
	2d02      	cmp	r5, #2
	d3f2      	bcc.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x104>
	4265      	negs	r5, r4
	f005 051f 	and.w	r5, r5, #31
	fa0a f505 	lsl.w	r5, sl, r5
	ba2d      	rev	r5, r5
	e7e1      	b.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xf0>
	2102      	movs	r1, #2
	f899 8000 	ldrb.w	r8, [r9]
	6001      	str	r1, [r0, #0]
	1d04      	adds	r4, r0, #4
	e9d9 ec02 	ldrd	lr, ip, [r9, #8]
	f1b8 0f02 	cmp.w	r8, #2
	e8b9 0062 	ldmia.w	r9!, {r1, r5, r6}
	c462      	stmia	r4!, {r1, r5, r6}
	e899 0462 	ldmia.w	r9, {r1, r5, r6, sl}
	e884 0462 	stmia.w	r4, {r1, r5, r6, sl}
	f44f 71a0 	mov.w	r1, #320	@ 0x140
	f8a0 1060 	strh.w	r1, [r0, #96]	@ 0x60
	f10e 011c 	add.w	r1, lr, #28
	bf38      	it	cc
	f10c 0108 	addcc.w	r1, ip, #8
	6543      	str	r3, [r0, #84]	@ 0x54
	6582      	str	r2, [r0, #88]	@ 0x58
	65c1      	str	r1, [r0, #92]	@ 0x5c
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f1b8 0f00 	cmp.w	r8, #0
	d18f      	bne.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x90>
	f899 3000 	ldrb.w	r3, [r9]
	2b01      	cmp	r3, #1
	f47f af4e 	bne.w	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
	f1bc 0f00 	cmp.w	ip, #0
	f43f af4a 	beq.w	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
	2302      	movs	r3, #2
	f8d1 c0f4 	ldr.w	ip, [r1, #244]	@ 0xf4
	6003      	str	r3, [r0, #0]
	1d06      	adds	r6, r0, #4
	f8d9 e00c 	ldr.w	lr, [r9, #12]
	e8b9 0032 	ldmia.w	r9!, {r1, r4, r5}
	c632      	stmia	r6!, {r1, r4, r5}
	e899 003a 	ldmia.w	r9, {r1, r3, r4, r5}
	c63a      	stmia	r6!, {r1, r3, r4, r5}
	f44f 71a0 	mov.w	r1, #320	@ 0x140
	f8a0 1060 	strh.w	r1, [r0, #96]	@ 0x60
	f10e 0108 	add.w	r1, lr, #8
	f8c0 c054 	str.w	ip, [r0, #84]	@ 0x54
	6582      	str	r2, [r0, #88]	@ 0x58
	65c1      	str	r1, [r0, #92]	@ 0x5c
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<<smoltcp::wire::ip::Address as core::fmt::Display>::fmt>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b084      	sub	sp, #16
	f244 12cb 	movw	r2, #16843	@ 0x41cb
	6800      	ldr	r0, [r0, #0]
	f6c0 0200 	movt	r2, #2048	@ 0x800
	9001      	str	r0, [sp, #4]
	9203      	str	r2, [sp, #12]
	aa01      	add	r2, sp, #4
	e9d1 0100 	ldrd	r0, r1, [r1]
	ab02      	add	r3, sp, #8
	9202      	str	r2, [sp, #8]
	f649 52d9 	movw	r2, #40409	@ 0x9dd9
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fb fc37 	bl	<core::fmt::write>
	b004      	add	sp, #16
	bd80      	pop	{r7, pc}
	d4d4      	bmi.n	<smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x190>

<smoltcp::wire::ipv4::Repr::emit>:
	b5d0      	push	{r4, r6, r7, lr}
	af02      	add	r7, sp, #8
	2a00      	cmp	r2, #0
	f000 809a 	beq.w	<smoltcp::wire::ipv4::Repr::emit+0x13e>
	f04f 0c45 	mov.w	ip, #69	@ 0x45
	2a01      	cmp	r2, #1
	f881 c000 	strb.w	ip, [r1]
	f000 809b 	beq.w	<smoltcp::wire::ipv4::Repr::emit+0x14e>
	f04f 0c00 	mov.w	ip, #0
	2a03      	cmp	r2, #3
	f881 c001 	strb.w	ip, [r1, #1]
	d964      	bls.n	<smoltcp::wire::ipv4::Repr::emit+0xee>
	f8b0 c008 	ldrh.w	ip, [r0, #8]
	2a05      	cmp	r2, #5
	f10c 0c14 	add.w	ip, ip, #20
	fa9c fc8c 	rev.w	ip, ip
	ea4f 4c1c 	mov.w	ip, ip, lsr #16
	f8a1 c002 	strh.w	ip, [r1, #2]
	d960      	bls.n	<smoltcp::wire::ipv4::Repr::emit+0xfe>
	f04f 0c00 	mov.w	ip, #0
	2a07      	cmp	r2, #7
	f8a1 c004 	strh.w	ip, [r1, #4]
	d962      	bls.n	<smoltcp::wire::ipv4::Repr::emit+0x10e>
	f04f 0c40 	mov.w	ip, #64	@ 0x40
	2a08      	cmp	r2, #8
	f8a1 c006 	strh.w	ip, [r1, #6]
	f000 8084 	beq.w	<smoltcp::wire::ipv4::Repr::emit+0x15e>
	f890 c00d 	ldrb.w	ip, [r0, #13]
	f890 e00c 	ldrb.w	lr, [r0, #12]
	f881 e008 	strb.w	lr, [r1, #8]
	e8df f00c 	tbb	[pc, ip]
	2424      	.short	0x2424
	190d0724 	.word	0x190d0724
	1322161c 	.word	0x1322161c
	00100a1f 	.word	0x00100a1f
	f04f 0c06 	mov.w	ip, #6
	e019      	b.n	<smoltcp::wire::ipv4::Repr::emit+0xae>
	f04f 0c3c 	mov.w	ip, #60	@ 0x3c
	e016      	b.n	<smoltcp::wire::ipv4::Repr::emit+0xae>
	f04f 0c11 	mov.w	ip, #17
	e013      	b.n	<smoltcp::wire::ipv4::Repr::emit+0xae>
	f890 c00e 	ldrb.w	ip, [r0, #14]
	e010      	b.n	<smoltcp::wire::ipv4::Repr::emit+0xae>
	f04f 0c3a 	mov.w	ip, #58	@ 0x3a
	e00d      	b.n	<smoltcp::wire::ipv4::Repr::emit+0xae>
	f04f 0c32 	mov.w	ip, #50	@ 0x32
	e00a      	b.n	<smoltcp::wire::ipv4::Repr::emit+0xae>
	f04f 0c2b 	mov.w	ip, #43	@ 0x2b
	e007      	b.n	<smoltcp::wire::ipv4::Repr::emit+0xae>
	f04f 0c2c 	mov.w	ip, #44	@ 0x2c
	e004      	b.n	<smoltcp::wire::ipv4::Repr::emit+0xae>
	f04f 0c3b 	mov.w	ip, #59	@ 0x3b
	e001      	b.n	<smoltcp::wire::ipv4::Repr::emit+0xae>
	f04f 0c33 	mov.w	ip, #51	@ 0x33
	2a09      	cmp	r2, #9
	d95d      	bls.n	<smoltcp::wire::ipv4::Repr::emit+0x16e>
	2a0f      	cmp	r2, #15
	f881 c009 	strb.w	ip, [r1, #9]
	d931      	bls.n	<smoltcp::wire::ipv4::Repr::emit+0x11e>
	f8d0 c000 	ldr.w	ip, [r0]
	2a13      	cmp	r2, #19
	f8c1 c00c 	str.w	ip, [r1, #12]
	d933      	bls.n	<smoltcp::wire::ipv4::Repr::emit+0x12e>
	6840      	ldr	r0, [r0, #4]
	f013 0ffd 	tst.w	r3, #253	@ 0xfd
	6108      	str	r0, [r1, #16]
	f04f 0000 	mov.w	r0, #0
	d10a      	bne.n	<smoltcp::wire::ipv4::Repr::emit+0xea>
	8148      	strh	r0, [r1, #10]
	4608      	mov	r0, r1
	460c      	mov	r4, r1
	2114      	movs	r1, #20
	f000 f9f1 	bl	<smoltcp::wire::ip::checksum::data>
	43c0      	mvns	r0, r0
	ba00      	rev	r0, r0
	0c00      	lsrs	r0, r0, #16
	8160      	strh	r0, [r4, #10]
	bdd0      	pop	{r4, r6, r7, pc}
	8148      	strh	r0, [r1, #10]
	bdd0      	pop	{r4, r6, r7, pc}
	f64a 630c 	movw	r3, #44556	@ 0xae0c
	2002      	movs	r0, #2
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2104      	movs	r1, #4
	f7fb fd34 	bl	<core::slice::index::slice_index_fail>
	f64a 634c 	movw	r3, #44620	@ 0xae4c
	2004      	movs	r0, #4
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2106      	movs	r1, #6
	f7fb fd2c 	bl	<core::slice::index::slice_index_fail>
	f64a 53cc 	movw	r3, #44492	@ 0xadcc
	2006      	movs	r0, #6
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2108      	movs	r1, #8
	f7fb fd24 	bl	<core::slice::index::slice_index_fail>
	f64a 53fc 	movw	r3, #44540	@ 0xadfc
	200c      	movs	r0, #12
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2110      	movs	r1, #16
	f7fb fd1c 	bl	<core::slice::index::slice_index_fail>
	f64a 53ec 	movw	r3, #44524	@ 0xadec
	2010      	movs	r0, #16
	f6c0 0300 	movt	r3, #2048	@ 0x800
	2114      	movs	r1, #20
	f7fb fd14 	bl	<core::slice::index::slice_index_fail>
	f64a 52dc 	movw	r2, #44508	@ 0xaddc
	2000      	movs	r0, #0
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2100      	movs	r1, #0
	f7fb fd8b 	bl	<core::panicking::panic_bounds_check>
	f64a 623c 	movw	r2, #44604	@ 0xae3c
	2001      	movs	r0, #1
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2101      	movs	r1, #1
	f7fb fd83 	bl	<core::panicking::panic_bounds_check>
	f64a 625c 	movw	r2, #44636	@ 0xae5c
	2008      	movs	r0, #8
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2108      	movs	r1, #8
	f7fb fd7b 	bl	<core::panicking::panic_bounds_check>
	f64a 621c 	movw	r2, #44572	@ 0xae1c
	2009      	movs	r0, #9
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2109      	movs	r1, #9
	f7fb fd73 	bl	<core::panicking::panic_bounds_check>

<defmt::export::fmt>:
	b5b0      	push	{r4, r5, r7, lr}
	af02      	add	r7, sp, #8
	b082      	sub	sp, #8
	4605      	mov	r5, r0
	f240 0003 	movw	r0, #3
	f2c0 0000 	movt	r0, #0
	460c      	mov	r4, r1
	f827 0c0e 	strh.w	r0, [r7, #-14]
	f1a7 000e 	sub.w	r0, r7, #14
	2102      	movs	r1, #2
	f7fc fcc1 	bl	<_defmt_write>
	a801      	add	r0, sp, #4
	2104      	movs	r1, #4
	9401      	str	r4, [sp, #4]
	f7fc fcbc 	bl	<_defmt_write>
	4628      	mov	r0, r5
	4621      	mov	r1, r4
	f7fc fcb8 	bl	<_defmt_write>
	b002      	add	sp, #8
	bdb0      	pop	{r4, r5, r7, pc}

<defmt::export::fmt>:
	b5d0      	push	{r4, r6, r7, lr}
	af02      	add	r7, sp, #8
	b082      	sub	sp, #8
	4604      	mov	r4, r0
	f240 0002 	movw	r0, #2
	f2c0 0000 	movt	r0, #0
	2102      	movs	r1, #2
	f8ad 0004 	strh.w	r0, [sp, #4]
	a801      	add	r0, sp, #4
	f7fc fca8 	bl	<_defmt_write>
	f1a7 0009 	sub.w	r0, r7, #9
	2101      	movs	r1, #1
	f807 4c09 	strb.w	r4, [r7, #-9]
	f7fc fca1 	bl	<_defmt_write>
	b002      	add	sp, #8
	bdd0      	pop	{r4, r6, r7, pc}

<defmt::export::fmt>:
	b5d0      	push	{r4, r6, r7, lr}
	af02      	add	r7, sp, #8
	b084      	sub	sp, #16
	4604      	mov	r4, r0
	f240 0007 	movw	r0, #7
	f2c0 0000 	movt	r0, #0
	2102      	movs	r1, #2
	f827 0c16 	strh.w	r0, [r7, #-22]
	f1a7 0016 	sub.w	r0, r7, #22
	f7fc fc90 	bl	<_defmt_write>
	f240 003d 	movw	r0, #61	@ 0x3d
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f8ad 0004 	strh.w	r0, [sp, #4]
	a801      	add	r0, sp, #4
	f7fc fc86 	bl	<_defmt_write>
	f240 0008 	movw	r0, #8
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f827 0c12 	strh.w	r0, [r7, #-18]
	f1a7 0012 	sub.w	r0, r7, #18
	f7fc fc7b 	bl	<_defmt_write>
	a802      	add	r0, sp, #8
	2104      	movs	r1, #4
	9402      	str	r4, [sp, #8]
	f7fc fc76 	bl	<_defmt_write>
	2000      	movs	r0, #0
	2102      	movs	r1, #2
	f827 0c0a 	strh.w	r0, [r7, #-10]
	f1a7 000a 	sub.w	r0, r7, #10
	f7fc fc6e 	bl	<_defmt_write>
	b004      	add	sp, #16
	bdd0      	pop	{r4, r6, r7, pc}

<smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	f84d bd04 	str.w	fp, [sp, #-4]!
	68ce      	ldr	r6, [r1, #12]
	2e00      	cmp	r6, #0
	bf04      	itt	eq
	2500      	moveq	r5, #0
	608d      	streq	r5, [r1, #8]
	684c      	ldr	r4, [r1, #4]
	b134      	cbz	r4, <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x24>
	688d      	ldr	r5, [r1, #8]
	4435      	add	r5, r6
	fbb5 f3f4 	udiv	r3, r5, r4
	fb03 5c14 	mls	ip, r3, r4, r5
	e001      	b.n	<smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x28>
	f04f 0c00 	mov.w	ip, #0
	1ba5      	subs	r5, r4, r6
	eba4 030c 	sub.w	r3, r4, ip
	42ab      	cmp	r3, r5
	bf38      	it	cc
	461d      	movcc	r5, r3
	eb15 0e0c 	adds.w	lr, r5, ip
	d20e      	bcs.n	<smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x58>
	45a6      	cmp	lr, r4
	d80c      	bhi.n	<smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x58>
	680b      	ldr	r3, [r1, #0]
	4295      	cmp	r5, r2
	bf38      	it	cc
	462a      	movcc	r2, r5
	6082      	str	r2, [r0, #8]
	4463      	add	r3, ip
	e9c0 2300 	strd	r2, r3, [r0]
	1990      	adds	r0, r2, r6
	60c8      	str	r0, [r1, #12]
	f85d bb04 	ldr.w	fp, [sp], #4
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f64a 7304 	movw	r3, #44804	@ 0xaf04
	4660      	mov	r0, ip
	f6c0 0300 	movt	r3, #2048	@ 0x800
	4671      	mov	r1, lr
	4622      	mov	r2, r4
	f7fb fc5a 	bl	<core::slice::index::slice_index_fail>

<smoltcp::socket::udp::Socket::accepts>:
	4684      	mov	ip, r0
	8800      	ldrh	r0, [r0, #0]
	b29b      	uxth	r3, r3
	4298      	cmp	r0, r3
	d10d      	bne.n	<smoltcp::socket::udp::Socket::accepts+0x26>
	f89c 3002 	ldrb.w	r3, [ip, #2]
	2001      	movs	r0, #1
	2b01      	cmp	r3, #1
	bf18      	it	ne
	4770      	bxne	lr
	f8dc 3003 	ldr.w	r3, [ip, #3]
	4293      	cmp	r3, r2
	bf18      	it	ne
	f112 0301 	addsne.w	r3, r2, #1
	d102      	bne.n	<smoltcp::socket::udp::Socket::accepts+0x2a>
	4770      	bx	lr
	2000      	movs	r0, #0
	4770      	bx	lr
	b5d0      	push	{r4, r6, r7, lr}
	af02      	add	r7, sp, #8
	f8d1 00f0 	ldr.w	r0, [r1, #240]	@ 0xf0
	b328      	cbz	r0, <smoltcp::socket::udp::Socket::accepts+0x80>
	eb00 0080 	add.w	r0, r0, r0, lsl #2
	f04f 3cff 	mov.w	ip, #4294967295	@ 0xffffffff
	4408      	add	r0, r1
	31f4      	adds	r1, #244	@ 0xf4
	f100 0ef4 	add.w	lr, r0, #244	@ 0xf4
	e00c      	b.n	<smoltcp::socket::udp::Socket::accepts+0x60>
	2400      	movs	r4, #0
	f003 031f 	and.w	r3, r3, #31
	4020      	ands	r0, r4
	fa2c f303 	lsr.w	r3, ip, r3
	ba1b      	rev	r3, r3
	4318      	orrs	r0, r3
	4282      	cmp	r2, r0
	d019      	beq.n	<smoltcp::socket::udp::Socket::accepts+0x8e>
	3105      	adds	r1, #5
	4571      	cmp	r1, lr
	d00f      	beq.n	<smoltcp::socket::udp::Socket::accepts+0x80>
	790b      	ldrb	r3, [r1, #4]
	6808      	ldr	r0, [r1, #0]
	2b00      	cmp	r3, #0
	d0ee      	beq.n	<smoltcp::socket::udp::Socket::accepts+0x46>
	f1a3 041f 	sub.w	r4, r3, #31
	b2e4      	uxtb	r4, r4
	2c02      	cmp	r4, #2
	d3f3      	bcc.n	<smoltcp::socket::udp::Socket::accepts+0x5a>
	425c      	negs	r4, r3
	f004 041f 	and.w	r4, r4, #31
	fa0c f404 	lsl.w	r4, ip, r4
	ba24      	rev	r4, r4
	e7e3      	b.n	<smoltcp::socket::udp::Socket::accepts+0x48>
	f002 00f0 	and.w	r0, r2, #240	@ 0xf0
	38e0      	subs	r0, #224	@ 0xe0
	fab0 f080 	clz	r0, r0
	0940      	lsrs	r0, r0, #5
	bdd0      	pop	{r4, r6, r7, pc}
	2001      	movs	r0, #1
	bdd0      	pop	{r4, r6, r7, pc}

<smoltcp::socket::udp::Socket::process>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b085      	sub	sp, #20
	f8d7 a010 	ldr.w	sl, [r7, #16]
	69c5      	ldr	r5, [r0, #28]
	4555      	cmp	r5, sl
	d34f      	bcc.n	<smoltcp::socket::udp::Socket::process+0xb4>
	f8d0 b00c 	ldr.w	fp, [r0, #12]
	f8d0 c014 	ldr.w	ip, [r0, #20]
	45e3      	cmp	fp, ip
	d049      	beq.n	<smoltcp::socket::udp::Socket::process+0xb4>
	6a44      	ldr	r4, [r0, #36]	@ 0x24
	e9d7 9802 	ldrd	r9, r8, [r7, #8]
	eba5 0e04 	sub.w	lr, r5, r4
	2c00      	cmp	r4, #0
	bf04      	itt	eq
	2600      	moveq	r6, #0
	6206      	streq	r6, [r0, #32]
	b135      	cbz	r5, <smoltcp::socket::udp::Socket::process+0x42>
	6a06      	ldr	r6, [r0, #32]
	4434      	add	r4, r6
	fbb4 f6f5 	udiv	r6, r4, r5
	fb06 4415 	mls	r4, r6, r5, r4
	e000      	b.n	<smoltcp::socket::udp::Socket::process+0x44>
	2400      	movs	r4, #0
	1b2c      	subs	r4, r5, r4
	4675      	mov	r5, lr
	4574      	cmp	r4, lr
	bf38      	it	cc
	4625      	movcc	r5, r4
	45d6      	cmp	lr, sl
	d330      	bcc.n	<smoltcp::socket::udp::Socket::process+0xb4>
	4554      	cmp	r4, sl
	d22c      	bcs.n	<smoltcp::socket::udp::Socket::process+0xb0>
	ebae 0605 	sub.w	r6, lr, r5
	4556      	cmp	r6, sl
	d32a      	bcc.n	<smoltcp::socket::udp::Socket::process+0xb4>
	f1bb 0f00 	cmp.w	fp, #0
	e9cd 1300 	strd	r1, r3, [sp]
	f000 8084 	beq.w	<smoltcp::socket::udp::Socket::process+0x172>
	6903      	ldr	r3, [r0, #16]
	4614      	mov	r4, r2
	6882      	ldr	r2, [r0, #8]
	f100 0118 	add.w	r1, r0, #24
	4463      	add	r3, ip
	fbb3 f6fb 	udiv	r6, r3, fp
	fb06 331b 	mls	r3, r6, fp, r3
	f10c 0601 	add.w	r6, ip, #1
	6146      	str	r6, [r0, #20]
	4606      	mov	r6, r0
	eb03 0383 	add.w	r3, r3, r3, lsl #2
	f842 5023 	str.w	r5, [r2, r3, lsl #2]
	eb02 0283 	add.w	r2, r2, r3, lsl #2
	2302      	movs	r3, #2
	7393      	strb	r3, [r2, #14]
	aa02      	add	r2, sp, #8
	4610      	mov	r0, r2
	462a      	mov	r2, r5
	f7ff ff32 	bl	<smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with>
	4630      	mov	r0, r6
	f8d6 b00c 	ldr.w	fp, [r6, #12]
	f8d6 c014 	ldr.w	ip, [r6, #20]
	4622      	mov	r2, r4
	e9dd 1300 	ldrd	r1, r3, [sp]
	45e3      	cmp	fp, ip
	d103      	bne.n	<smoltcp::socket::udp::Socket::process+0xbc>
	b005      	add	sp, #20
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f1bb 0f00 	cmp.w	fp, #0
	d057      	beq.n	<smoltcp::socket::udp::Socket::process+0x172>
	6905      	ldr	r5, [r0, #16]
	6884      	ldr	r4, [r0, #8]
	4465      	add	r5, ip
	f8d0 e024 	ldr.w	lr, [r0, #36]	@ 0x24
	fbb5 f6fb 	udiv	r6, r5, fp
	fb06 551b 	mls	r5, r6, fp, r5
	f10c 0601 	add.w	r6, ip, #1
	6146      	str	r6, [r0, #20]
	f1be 0f00 	cmp.w	lr, #0
	eb05 0585 	add.w	r5, r5, r5, lsl #2
	eb04 0685 	add.w	r6, r4, r5, lsl #2
	f844 a025 	str.w	sl, [r4, r5, lsl #2]
	f8c6 300f 	str.w	r3, [r6, #15]
	f04f 0301 	mov.w	r3, #1
	73b3      	strb	r3, [r6, #14]
	f8a6 900c 	strh.w	r9, [r6, #12]
	e9c6 1201 	strd	r1, r2, [r6, #4]
	bf04      	itt	eq
	2100      	moveq	r1, #0
	6201      	streq	r1, [r0, #32]
	69c2      	ldr	r2, [r0, #28]
	b132      	cbz	r2, <smoltcp::socket::udp::Socket::process+0x114>
	6a01      	ldr	r1, [r0, #32]
	4471      	add	r1, lr
	fbb1 f3f2 	udiv	r3, r1, r2
	fb03 1512 	mls	r5, r3, r2, r1
	e000      	b.n	<smoltcp::socket::udp::Socket::process+0x116>
	2500      	movs	r5, #0
	eba2 030e 	sub.w	r3, r2, lr
	1b51      	subs	r1, r2, r5
	4299      	cmp	r1, r3
	bf38      	it	cc
	460b      	movcc	r3, r1
	1959      	adds	r1, r3, r5
	d21e      	bcs.n	<smoltcp::socket::udp::Socket::process+0x164>
	4291      	cmp	r1, r2
	d81c      	bhi.n	<smoltcp::socket::udp::Socket::process+0x164>
	6982      	ldr	r2, [r0, #24]
	4553      	cmp	r3, sl
	4651      	mov	r1, sl
	bf38      	it	cc
	4619      	movcc	r1, r3
	459a      	cmp	sl, r3
	eb01 060e 	add.w	r6, r1, lr
	6246      	str	r6, [r0, #36]	@ 0x24
	bf9f      	itttt	ls
	1950      	addls	r0, r2, r5
	4641      	movls	r1, r8
	4652      	movls	r2, sl
	b005      	addls	sp, #20
	bf9e      	ittt	ls
	e8bd 0f00 	ldmials.w	sp!, {r8, r9, sl, fp}
	e8bd 40f0 	ldmials.w	sp!, {r4, r5, r6, r7, lr}
	f001 b966 	bls.w	<__aeabi_memcpy>
	f64a 7248 	movw	r2, #44872	@ 0xaf48
	4608      	mov	r0, r1
	f6c0 0200 	movt	r2, #2048	@ 0x800
	4651      	mov	r1, sl
	f7fb ffec 	bl	<core::slice::copy_from_slice_impl::len_mismatch_fail>
	f64a 7304 	movw	r3, #44804	@ 0xaf04
	4628      	mov	r0, r5
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f7fb fb58 	bl	<core::slice::index::slice_index_fail>
	f64a 60bc 	movw	r0, #44732	@ 0xaebc
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fc f82e 	bl	<core::panicking::panic_const::panic_const_rem_by_zero>

<smoltcp::wire::ip::checksum::data>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
	2920      	cmp	r1, #32
	f0c0 80b5 	bcc.w	<smoltcp::wire::ip::checksum::data+0x178>
	f1a1 0c20 	sub.w	ip, r1, #32
	ea5f 125c 	movs.w	r2, ip, lsr #5
	f000 80b1 	beq.w	<smoltcp::wire::ip::checksum::data+0x17c>
	3201      	adds	r2, #1
	f022 0301 	bic.w	r3, r2, #1
	2200      	movs	r2, #0
	f830 6b40 	ldrh.w	r6, [r0], #64
	3940      	subs	r1, #64	@ 0x40
	3b02      	subs	r3, #2
	f830 4c3e 	ldrh.w	r4, [r0, #-62]
	ba36      	rev	r6, r6
	f830 5c3c 	ldrh.w	r5, [r0, #-60]
	ba24      	rev	r4, r4
	eb02 4216 	add.w	r2, r2, r6, lsr #16
	f830 6c3a 	ldrh.w	r6, [r0, #-58]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c38 	ldrh.w	r5, [r0, #-56]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c36 	ldrh.w	r6, [r0, #-54]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c34 	ldrh.w	r5, [r0, #-52]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c32 	ldrh.w	r6, [r0, #-50]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c30 	ldrh.w	r5, [r0, #-48]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c2e 	ldrh.w	r6, [r0, #-46]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c2c 	ldrh.w	r5, [r0, #-44]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c2a 	ldrh.w	r6, [r0, #-42]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c28 	ldrh.w	r5, [r0, #-40]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c26 	ldrh.w	r6, [r0, #-38]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c24 	ldrh.w	r5, [r0, #-36]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c22 	ldrh.w	r6, [r0, #-34]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c20 	ldrh.w	r5, [r0, #-32]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c1e 	ldrh.w	r6, [r0, #-30]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c1c 	ldrh.w	r5, [r0, #-28]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c1a 	ldrh.w	r6, [r0, #-26]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c18 	ldrh.w	r5, [r0, #-24]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c16 	ldrh.w	r6, [r0, #-22]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c14 	ldrh.w	r5, [r0, #-20]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c12 	ldrh.w	r6, [r0, #-18]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c10 	ldrh.w	r5, [r0, #-16]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c0e 	ldrh.w	r6, [r0, #-14]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c0c 	ldrh.w	r5, [r0, #-12]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 6c0a 	ldrh.w	r6, [r0, #-10]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 5c08 	ldrh.w	r5, [r0, #-8]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba34      	rev	r4, r6
	f830 9c06 	ldrh.w	r9, [r0, #-6]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	ba2c      	rev	r4, r5
	f830 8c04 	ldrh.w	r8, [r0, #-4]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	fa99 f489 	rev.w	r4, r9
	f830 ec02 	ldrh.w	lr, [r0, #-2]
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	fa98 f488 	rev.w	r4, r8
	fa9e f68e 	rev.w	r6, lr
	eb02 4214 	add.w	r2, r2, r4, lsr #16
	eb02 4216 	add.w	r2, r2, r6, lsr #16
	f47f af59 	bne.w	<smoltcp::wire::ip::checksum::data+0x22>
	ea5f 638c 	movs.w	r3, ip, lsl #26
	d506      	bpl.n	<smoltcp::wire::ip::checksum::data+0x184>
	e058      	b.n	<smoltcp::wire::ip::checksum::data+0x22a>
	2200      	movs	r2, #0
	e056      	b.n	<smoltcp::wire::ip::checksum::data+0x22a>
	2200      	movs	r2, #0
	ea5f 638c 	movs.w	r3, ip, lsl #26
	d452      	bmi.n	<smoltcp::wire::ip::checksum::data+0x22a>
	f830 6b20 	ldrh.w	r6, [r0], #32
	3920      	subs	r1, #32
	f830 3c1e 	ldrh.w	r3, [r0, #-30]
	ba36      	rev	r6, r6
	f830 5c1c 	ldrh.w	r5, [r0, #-28]
	ba1b      	rev	r3, r3
	eb02 4216 	add.w	r2, r2, r6, lsr #16
	f830 6c1a 	ldrh.w	r6, [r0, #-26]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba2b      	rev	r3, r5
	f830 5c18 	ldrh.w	r5, [r0, #-24]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba33      	rev	r3, r6
	f830 6c16 	ldrh.w	r6, [r0, #-22]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba2b      	rev	r3, r5
	f830 5c14 	ldrh.w	r5, [r0, #-20]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba33      	rev	r3, r6
	f830 6c12 	ldrh.w	r6, [r0, #-18]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba2b      	rev	r3, r5
	f830 5c10 	ldrh.w	r5, [r0, #-16]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba33      	rev	r3, r6
	f830 6c0e 	ldrh.w	r6, [r0, #-14]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba2b      	rev	r3, r5
	f830 5c0c 	ldrh.w	r5, [r0, #-12]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba33      	rev	r3, r6
	f830 6c0a 	ldrh.w	r6, [r0, #-10]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba2b      	rev	r3, r5
	f830 5c08 	ldrh.w	r5, [r0, #-8]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba33      	rev	r3, r6
	f830 4c06 	ldrh.w	r4, [r0, #-6]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba2b      	rev	r3, r5
	f830 ec04 	ldrh.w	lr, [r0, #-4]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	ba23      	rev	r3, r4
	f830 cc02 	ldrh.w	ip, [r0, #-2]
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	fa9e f38e 	rev.w	r3, lr
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	fa9c f38c 	rev.w	r3, ip
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	2902      	cmp	r1, #2
	d306      	bcc.n	<smoltcp::wire::ip::checksum::data+0x23c>
	1e8d      	subs	r5, r1, #2
	43eb      	mvns	r3, r5
	f013 0f06 	tst.w	r3, #6
	d103      	bne.n	<smoltcp::wire::ip::checksum::data+0x240>
	4603      	mov	r3, r0
	e01a      	b.n	<smoltcp::wire::ip::checksum::data+0x272>
	4603      	mov	r3, r0
	e02f      	b.n	<smoltcp::wire::ip::checksum::data+0x2a0>
	4603      	mov	r3, r0
	f015 0f06 	tst.w	r5, #6
	f833 6b02 	ldrh.w	r6, [r3], #2
	ba36      	rev	r6, r6
	eb02 4216 	add.w	r2, r2, r6, lsr #16
	d00c      	beq.n	<smoltcp::wire::ip::checksum::data+0x26c>
	8843      	ldrh	r3, [r0, #2]
	ba1b      	rev	r3, r3
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	f005 0306 	and.w	r3, r5, #6
	2b02      	cmp	r3, #2
	d12b      	bne.n	<smoltcp::wire::ip::checksum::data+0x2ba>
	1d03      	adds	r3, r0, #4
	3904      	subs	r1, #4
	2d06      	cmp	r5, #6
	d203      	bcs.n	<smoltcp::wire::ip::checksum::data+0x272>
	e019      	b.n	<smoltcp::wire::ip::checksum::data+0x2a0>
	4629      	mov	r1, r5
	2d06      	cmp	r5, #6
	d316      	bcc.n	<smoltcp::wire::ip::checksum::data+0x2a0>
	f833 0b08 	ldrh.w	r0, [r3], #8
	3908      	subs	r1, #8
	2901      	cmp	r1, #1
	ba00      	rev	r0, r0
	f833 4c06 	ldrh.w	r4, [r3, #-6]
	f833 5c04 	ldrh.w	r5, [r3, #-4]
	eb02 4010 	add.w	r0, r2, r0, lsr #16
	ba22      	rev	r2, r4
	f833 6c02 	ldrh.w	r6, [r3, #-2]
	eb00 4012 	add.w	r0, r0, r2, lsr #16
	ba2a      	rev	r2, r5
	eb00 4012 	add.w	r0, r0, r2, lsr #16
	ba32      	rev	r2, r6
	eb00 4212 	add.w	r2, r0, r2, lsr #16
	d8e8      	bhi.n	<smoltcp::wire::ip::checksum::data+0x272>
	2900      	cmp	r1, #0
	bf1c      	itt	ne
	7818      	ldrbne	r0, [r3, #0]
	eb02 2200 	addne.w	r2, r2, r0, lsl #8
	0c10      	lsrs	r0, r2, #16
	fa10 f082 	uxtah	r0, r0, r2
	eb00 4010 	add.w	r0, r0, r0, lsr #16
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	8883      	ldrh	r3, [r0, #4]
	3906      	subs	r1, #6
	ba1b      	rev	r3, r3
	eb02 4213 	add.w	r2, r2, r3, lsr #16
	1d83      	adds	r3, r0, #6
	2d06      	cmp	r5, #6
	d2d3      	bcs.n	<smoltcp::wire::ip::checksum::data+0x272>
	e7e9      	b.n	<smoltcp::wire::ip::checksum::data+0x2a0>

<smoltcp::iface::route::Routes::lookup>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b081      	sub	sp, #4
	1c53      	adds	r3, r2, #1
	d007      	beq.n	<smoltcp::iface::route::Routes::lookup+0x1e>
	f002 03f0 	and.w	r3, r2, #240	@ 0xf0
	2be0      	cmp	r3, #224	@ 0xe0
	bf1c      	itt	ne
	b2d3      	uxtbne	r3, r2
	ea53 2312 	orrsne.w	r3, r3, r2, lsr #8
	d10a      	bne.n	<smoltcp::iface::route::Routes::lookup+0x34>
	f64a 70b8 	movw	r0, #44984	@ 0xafb8
	f64a 72dc 	movw	r2, #45020	@ 0xafdc
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2123      	movs	r1, #35	@ 0x23
	f7fb fa65 	bl	<core::panicking::panic>
	6e0b      	ldr	r3, [r1, #96]	@ 0x60
	2b00      	cmp	r3, #0
	d072      	beq.n	<smoltcp::iface::route::Routes::lookup+0x120>
	eb03 0343 	add.w	r3, r3, r3, lsl #1
	e9d7 ec02 	ldrd	lr, ip, [r7, #8]
	2600      	movs	r6, #0
	f04f 3aff 	mov.w	sl, #4294967295	@ 0xffffffff
	eb01 1903 	add.w	r9, r1, r3, lsl #4
	ea4f 1803 	mov.w	r8, r3, lsl #4
	f1a8 0330 	sub.w	r3, r8, #48	@ 0x30
	9300      	str	r3, [sp, #0]
	e007      	b.n	<smoltcp::iface::route::Routes::lookup+0x68>
	2400      	movs	r4, #0
	6a5d      	ldr	r5, [r3, #36]	@ 0x24
	4055      	eors	r5, r2
	422c      	tst	r4, r5
	d01d      	beq.n	<smoltcp::iface::route::Routes::lookup+0x9e>
	3630      	adds	r6, #48	@ 0x30
	45b0      	cmp	r8, r6
	d05b      	beq.n	<smoltcp::iface::route::Routes::lookup+0x120>
	198b      	adds	r3, r1, r6
	691c      	ldr	r4, [r3, #16]
	07e4      	lsls	r4, r4, #31
	d006      	beq.n	<smoltcp::iface::route::Routes::lookup+0x7e>
	e9d3 4506 	ldrd	r4, r5, [r3, #24]
	ebb4 040e 	subs.w	r4, r4, lr
	eb75 040c 	sbcs.w	r4, r5, ip
	dbf1      	blt.n	<smoltcp::iface::route::Routes::lookup+0x62>
	f893 b028 	ldrb.w	fp, [r3, #40]	@ 0x28
	f1bb 0f00 	cmp.w	fp, #0
	d0e7      	beq.n	<smoltcp::iface::route::Routes::lookup+0x58>
	f1cb 0400 	rsb	r4, fp, #0
	f004 041f 	and.w	r4, r4, #31
	fa0a f404 	lsl.w	r4, sl, r4
	ba24      	rev	r4, r4
	6a5d      	ldr	r5, [r3, #36]	@ 0x24
	4055      	eors	r5, r2
	422c      	tst	r4, r5
	d1e1      	bne.n	<smoltcp::iface::route::Routes::lookup+0x62>
	f103 0130 	add.w	r1, r3, #48	@ 0x30
	4549      	cmp	r1, r9
	d042      	beq.n	<smoltcp::iface::route::Routes::lookup+0x12c>
	9900      	ldr	r1, [sp, #0]
	f64a 25ab 	movw	r5, #43691	@ 0xaaab
	f6ca 25aa 	movt	r5, #43690	@ 0xaaaa
	f04f 38ff 	mov.w	r8, #4294967295	@ 0xffffffff
	1b8c      	subs	r4, r1, r6
	f103 0158 	add.w	r1, r3, #88	@ 0x58
	fba4 4505 	umull	r4, r5, r4, r5
	096d      	lsrs	r5, r5, #5
	e00a      	b.n	<smoltcp::iface::route::Routes::lookup+0xd8>
	fa5f f68b 	uxtb.w	r6, fp
	42a6      	cmp	r6, r4
	bf8c      	ite	hi
	4634      	movhi	r4, r6
	f1a1 0328 	subls.w	r3, r1, #40	@ 0x28
	46a3      	mov	fp, r4
	3130      	adds	r1, #48	@ 0x30
	3d01      	subs	r5, #1
	d029      	beq.n	<smoltcp::iface::route::Routes::lookup+0x12c>
	f851 4c18 	ldr.w	r4, [r1, #-24]
	07e4      	lsls	r4, r4, #31
	d006      	beq.n	<smoltcp::iface::route::Routes::lookup+0xee>
	e951 4604 	ldrd	r4, r6, [r1, #-16]
	ebb4 040e 	subs.w	r4, r4, lr
	eb76 040c 	sbcs.w	r4, r6, ip
	dbf1      	blt.n	<smoltcp::iface::route::Routes::lookup+0xd2>
	780c      	ldrb	r4, [r1, #0]
	b16c      	cbz	r4, <smoltcp::iface::route::Routes::lookup+0x10e>
	4266      	negs	r6, r4
	f006 061f 	and.w	r6, r6, #31
	fa08 f606 	lsl.w	r6, r8, r6
	fa96 f986 	rev.w	r9, r6
	f851 6c04 	ldr.w	r6, [r1, #-4]
	4056      	eors	r6, r2
	ea19 0f06 	tst.w	r9, r6
	d1e2      	bne.n	<smoltcp::iface::route::Routes::lookup+0xd2>
	e7d9      	b.n	<smoltcp::iface::route::Routes::lookup+0xc2>
	f04f 0900 	mov.w	r9, #0
	f851 6c04 	ldr.w	r6, [r1, #-4]
	4056      	eors	r6, r2
	ea19 0f06 	tst.w	r9, r6
	d1d9      	bne.n	<smoltcp::iface::route::Routes::lookup+0xd2>
	e7d0      	b.n	<smoltcp::iface::route::Routes::lookup+0xc2>
	2100      	movs	r1, #0
	7001      	strb	r1, [r0, #0]
	b001      	add	sp, #4
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	6a19      	ldr	r1, [r3, #32]
	f8c0 1001 	str.w	r1, [r0, #1]
	2101      	movs	r1, #1
	7001      	strb	r1, [r0, #0]
	b001      	add	sp, #4
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<smoltcp::iface::neighbor::Cache::fill>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b089      	sub	sp, #36	@ 0x24
	f8d0 60c0 	ldr.w	r6, [r0, #192]	@ 0xc0
	f248 7400 	movw	r4, #34560	@ 0x8700
	f8d7 c008 	ldr.w	ip, [r7, #8]
	f2c0 3493 	movt	r4, #915	@ 0x393
	68fd      	ldr	r5, [r7, #12]
	fa1f fe83 	uxth.w	lr, r3
	eb06 0846 	add.w	r8, r6, r6, lsl #1
	eb1c 0a04 	adds.w	sl, ip, r4
	eba6 0386 	sub.w	r3, r6, r6, lsl #2
	f145 0c00 	adc.w	ip, r5, #0
	eb00 0bc8 	add.w	fp, r0, r8, lsl #3
	9607      	str	r6, [sp, #28]
	ea4f 09c3 	mov.w	r9, r3, lsl #3
	2300      	movs	r3, #0
	eb19 0403 	adds.w	r4, r9, r3
	d01a      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x78>
	58c6      	ldr	r6, [r0, r3]
	18c5      	adds	r5, r0, r3
	428e      	cmp	r6, r1
	f000 8082 	beq.w	<smoltcp::iface::neighbor::Cache::fill+0x150>
	f114 0618 	adds.w	r6, r4, #24
	d012      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x78>
	69ae      	ldr	r6, [r5, #24]
	428e      	cmp	r6, r1
	d078      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x14a>
	f114 0630 	adds.w	r6, r4, #48	@ 0x30
	d00c      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x78>
	6b2e      	ldr	r6, [r5, #48]	@ 0x30
	428e      	cmp	r6, r1
	d074      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x14e>
	3448      	adds	r4, #72	@ 0x48
	d007      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x78>
	6cac      	ldr	r4, [r5, #72]	@ 0x48
	3360      	adds	r3, #96	@ 0x60
	428c      	cmp	r4, r1
	d1e5      	bne.n	<smoltcp::iface::neighbor::Cache::fill+0x3c>
	4418      	add	r0, r3
	f1a0 0518 	sub.w	r5, r0, #24
	e06b      	b.n	<smoltcp::iface::neighbor::Cache::fill+0x150>
	9b07      	ldr	r3, [sp, #28]
	2b07      	cmp	r3, #7
	d80a      	bhi.n	<smoltcp::iface::neighbor::Cache::fill+0x94>
	f8cb a008 	str.w	sl, [fp, #8]
	f8cb 1000 	str.w	r1, [fp]
	e9cb 2e04 	strd	r2, lr, [fp, #16]
	f8d0 10c0 	ldr.w	r1, [r0, #192]	@ 0xc0
	f8cb c00c 	str.w	ip, [fp, #12]
	e146      	b.n	<smoltcp::iface::neighbor::Cache::fill+0x322>
	f06f 032f 	mvn.w	r3, #47	@ 0x2f
	f64a 24ab 	movw	r4, #43691	@ 0xaaab
	eb03 03c8 	add.w	r3, r3, r8, lsl #3
	f6ca 24aa 	movt	r4, #43690	@ 0xaaaa
	f8cd b020 	str.w	fp, [sp, #32]
	fba3 4304 	umull	r4, r3, r3, r4
	9104      	str	r1, [sp, #16]
	e9d0 4502 	ldrd	r4, r5, [r0, #8]
	f8cd a008 	str.w	sl, [sp, #8]
	f8cd c00c 	str.w	ip, [sp, #12]
	f8cd e004 	str.w	lr, [sp, #4]
	e9cd 0205 	strd	r0, r2, [sp, #20]
	ea6f 1613 	mvn.w	r6, r3, lsr #4
	07b6      	lsls	r6, r6, #30
	f100 0618 	add.w	r6, r0, #24
	d105      	bne.n	<smoltcp::iface::neighbor::Cache::fill+0xda>
	46b3      	mov	fp, r6
	4603      	mov	r3, r0
	4606      	mov	r6, r0
	46a2      	mov	sl, r4
	46a9      	mov	r9, r5
	e058      	b.n	<smoltcp::iface::neighbor::Cache::fill+0x18c>
	e9d0 b808 	ldrd	fp, r8, [r0, #32]
	ea4f 1a13 	mov.w	sl, r3, lsr #4
	4601      	mov	r1, r0
	4650      	mov	r0, sl
	ebb4 030b 	subs.w	r3, r4, fp
	46c1      	mov	r9, r8
	eb75 0308 	sbcs.w	r3, r5, r8
	46da      	mov	sl, fp
	bfb8      	it	lt
	46a9      	movlt	r9, r5
	bfb8      	it	lt
	46a2      	movlt	sl, r4
	ebbb 0304 	subs.w	r3, fp, r4
	f101 0b30 	add.w	fp, r1, #48	@ 0x30
	eb78 0305 	sbcs.w	r3, r8, r5
	4633      	mov	r3, r6
	bfa8      	it	ge
	460b      	movge	r3, r1
	0784      	lsls	r4, r0, #30
	d03d      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x18c>
	e9d1 460e 	ldrd	r4, r6, [r1, #56]	@ 0x38
	f000 0003 	and.w	r0, r0, #3
	ebba 0504 	subs.w	r5, sl, r4
	eb79 0506 	sbcs.w	r5, r9, r6
	46a0      	mov	r8, r4
	4635      	mov	r5, r6
	bfbc      	itt	lt
	464d      	movlt	r5, r9
	46d0      	movlt	r8, sl
	ebb4 040a 	subs.w	r4, r4, sl
	eb76 0409 	sbcs.w	r4, r6, r9
	f101 0648 	add.w	r6, r1, #72	@ 0x48
	bfb8      	it	lt
	465b      	movlt	r3, fp
	2801      	cmp	r0, #1
	d113      	bne.n	<smoltcp::iface::neighbor::Cache::fill+0x166>
	4658      	mov	r0, fp
	46b3      	mov	fp, r6
	46c2      	mov	sl, r8
	46a9      	mov	r9, r5
	4606      	mov	r6, r0
	e020      	b.n	<smoltcp::iface::neighbor::Cache::fill+0x18c>
	3518      	adds	r5, #24
	e000      	b.n	<smoltcp::iface::neighbor::Cache::fill+0x150>
	3530      	adds	r5, #48	@ 0x30
	f8c5 e014 	str.w	lr, [r5, #20]
	612a      	str	r2, [r5, #16]
	f8c5 c00c 	str.w	ip, [r5, #12]
	f8c5 a008 	str.w	sl, [r5, #8]
	b009      	add	sp, #36	@ 0x24
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	e9d1 2b14 	ldrd	r2, fp, [r1, #80]	@ 0x50
	46d9      	mov	r9, fp
	ebb8 0402 	subs.w	r4, r8, r2
	eb75 040b 	sbcs.w	r4, r5, fp
	4692      	mov	sl, r2
	bfbc      	itt	lt
	46a9      	movlt	r9, r5
	46c2      	movlt	sl, r8
	ebb2 0208 	subs.w	r2, r2, r8
	eb7b 0205 	sbcs.w	r2, fp, r5
	f101 0b60 	add.w	fp, r1, #96	@ 0x60
	bfb8      	it	lt
	4633      	movlt	r3, r6
	f04f 0800 	mov.w	r8, #0
	e9d6 4508 	ldrd	r4, r5, [r6, #32]
	eb0b 0608 	add.w	r6, fp, r8
	f108 0860 	add.w	r8, r8, #96	@ 0x60
	ebb4 020a 	subs.w	r2, r4, sl
	eb75 0209 	sbcs.w	r2, r5, r9
	bfb8      	it	lt
	4633      	movlt	r3, r6
	ebba 0204 	subs.w	r2, sl, r4
	eb79 0205 	sbcs.w	r2, r9, r5
	bfbc      	itt	lt
	464d      	movlt	r5, r9
	4654      	movlt	r4, sl
	e9d6 9208 	ldrd	r9, r2, [r6, #32]
	e9d6 ac0e 	ldrd	sl, ip, [r6, #56]	@ 0x38
	ebb9 0e04 	subs.w	lr, r9, r4
	eb72 0005 	sbcs.w	r0, r2, r5
	bfb8      	it	lt
	f106 0318 	addlt.w	r3, r6, #24
	ebb4 0009 	subs.w	r0, r4, r9
	eb75 0002 	sbcs.w	r0, r5, r2
	bfa4      	itt	ge
	4615      	movge	r5, r2
	464c      	movge	r4, r9
	ebba 0004 	subs.w	r0, sl, r4
	eb7c 0005 	sbcs.w	r0, ip, r5
	bfb8      	it	lt
	f106 0330 	addlt.w	r3, r6, #48	@ 0x30
	ebb4 000a 	subs.w	r0, r4, sl
	eb75 000c 	sbcs.w	r0, r5, ip
	bfa8      	it	ge
	4665      	movge	r5, ip
	e9d6 2014 	ldrd	r2, r0, [r6, #80]	@ 0x50
	bfa8      	it	ge
	4654      	movge	r4, sl
	3648      	adds	r6, #72	@ 0x48
	1b11      	subs	r1, r2, r4
	eb70 0105 	sbcs.w	r1, r0, r5
	bfb8      	it	lt
	4633      	movlt	r3, r6
	1aa1      	subs	r1, r4, r2
	eb75 0100 	sbcs.w	r1, r5, r0
	bfa4      	itt	ge
	4605      	movge	r5, r0
	4614      	movge	r4, r2
	9908      	ldr	r1, [sp, #32]
	eb0b 0008 	add.w	r0, fp, r8
	46a2      	mov	sl, r4
	46a9      	mov	r9, r5
	4288      	cmp	r0, r1
	d1b6      	bne.n	<smoltcp::iface::neighbor::Cache::fill+0x190>
	9905      	ldr	r1, [sp, #20]
	681c      	ldr	r4, [r3, #0]
	2300      	movs	r3, #0
	f8dd 8018 	ldr.w	r8, [sp, #24]
	460d      	mov	r5, r1
	9a04      	ldr	r2, [sp, #16]
	9e08      	ldr	r6, [sp, #32]
	6828      	ldr	r0, [r5, #0]
	42a0      	cmp	r0, r4
	d023      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x280>
	3518      	adds	r5, #24
	42b5      	cmp	r5, r6
	d012      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x264>
	6828      	ldr	r0, [r5, #0]
	42a0      	cmp	r0, r4
	d015      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x270>
	3518      	adds	r5, #24
	42b5      	cmp	r5, r6
	d00c      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x264>
	6828      	ldr	r0, [r5, #0]
	42a0      	cmp	r0, r4
	d012      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x276>
	3518      	adds	r5, #24
	42b5      	cmp	r5, r6
	d006      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x264>
	6828      	ldr	r0, [r5, #0]
	42a0      	cmp	r0, r4
	d00f      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x27c>
	3518      	adds	r5, #24
	3304      	adds	r3, #4
	42b5      	cmp	r5, r6
	d1e6      	bne.n	<smoltcp::iface::neighbor::Cache::fill+0x232>
	f64a 70ec 	movw	r0, #45036	@ 0xafec
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fb f80b 	bl	<core::option::unwrap_failed>
	f043 0301 	orr.w	r3, r3, #1
	e004      	b.n	<smoltcp::iface::neighbor::Cache::fill+0x280>
	f043 0302 	orr.w	r3, r3, #2
	e001      	b.n	<smoltcp::iface::neighbor::Cache::fill+0x280>
	f043 0303 	orr.w	r3, r3, #3
	9807      	ldr	r0, [sp, #28]
	4283      	cmp	r3, r0
	d254      	bcs.n	<smoltcp::iface::neighbor::Cache::fill+0x330>
	eb03 0043 	add.w	r0, r3, r3, lsl #1
	f1a6 0318 	sub.w	r3, r6, #24
	4615      	mov	r5, r2
	460c      	mov	r4, r1
	eb01 00c0 	add.w	r0, r1, r0, lsl #3
	4619      	mov	r1, r3
	2218      	movs	r2, #24
	f000 fda0 	bl	<__aeabi_memmove8>
	f8d4 20c0 	ldr.w	r2, [r4, #192]	@ 0xc0
	4629      	mov	r1, r5
	4620      	mov	r0, r4
	46c4      	mov	ip, r8
	f1a2 0e01 	sub.w	lr, r2, #1
	f8c4 e0c0 	str.w	lr, [r4, #192]	@ 0xc0
	eba2 0282 	sub.w	r2, r2, r2, lsl #2
	eb0e 034e 	add.w	r3, lr, lr, lsl #1
	00d2      	lsls	r2, r2, #3
	eb04 09c3 	add.w	r9, r4, r3, lsl #3
	2400      	movs	r4, #0
	1916      	adds	r6, r2, r4
	f116 0518 	adds.w	r5, r6, #24
	d01a      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x2fe>
	5903      	ldr	r3, [r0, r4]
	1905      	adds	r5, r0, r4
	428b      	cmp	r3, r1
	d03d      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x34c>
	f116 0330 	adds.w	r3, r6, #48	@ 0x30
	d013      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x2fe>
	69ab      	ldr	r3, [r5, #24]
	428b      	cmp	r3, r1
	d034      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x346>
	f116 0348 	adds.w	r3, r6, #72	@ 0x48
	d00d      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x2fe>
	6b2b      	ldr	r3, [r5, #48]	@ 0x30
	428b      	cmp	r3, r1
	d030      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x34a>
	f116 0360 	adds.w	r3, r6, #96	@ 0x60
	d007      	beq.n	<smoltcp::iface::neighbor::Cache::fill+0x2fe>
	6cab      	ldr	r3, [r5, #72]	@ 0x48
	3460      	adds	r4, #96	@ 0x60
	428b      	cmp	r3, r1
	d1e4      	bne.n	<smoltcp::iface::neighbor::Cache::fill+0x2c0>
	4420      	add	r0, r4
	f1a0 0518 	sub.w	r5, r0, #24
	e026      	b.n	<smoltcp::iface::neighbor::Cache::fill+0x34c>
	f1be 0f07 	cmp.w	lr, #7
	d82b      	bhi.n	<smoltcp::iface::neighbor::Cache::fill+0x35c>
	9a02      	ldr	r2, [sp, #8]
	f8c9 2008 	str.w	r2, [r9, #8]
	f8c9 1000 	str.w	r1, [r9]
	9901      	ldr	r1, [sp, #4]
	f8c9 c010 	str.w	ip, [r9, #16]
	9a03      	ldr	r2, [sp, #12]
	f8c9 1014 	str.w	r1, [r9, #20]
	f8d0 10c0 	ldr.w	r1, [r0, #192]	@ 0xc0
	f8c9 200c 	str.w	r2, [r9, #12]
	3101      	adds	r1, #1
	f8c0 10c0 	str.w	r1, [r0, #192]	@ 0xc0
	b009      	add	sp, #36	@ 0x24
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f64a 7014 	movw	r0, #44820	@ 0xaf14
	f64a 7238 	movw	r2, #44856	@ 0xaf38
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2122      	movs	r1, #34	@ 0x22
	f7fb f83d 	bl	<core::panicking::panic>
	3518      	adds	r5, #24
	e000      	b.n	<smoltcp::iface::neighbor::Cache::fill+0x34c>
	3530      	adds	r5, #48	@ 0x30
	9801      	ldr	r0, [sp, #4]
	6168      	str	r0, [r5, #20]
	9803      	ldr	r0, [sp, #12]
	60e8      	str	r0, [r5, #12]
	9802      	ldr	r0, [sp, #8]
	f8c5 8010 	str.w	r8, [r5, #16]
	60a8      	str	r0, [r5, #8]
	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
	f64a 72fc 	movw	r2, #45052	@ 0xaffc
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2128      	movs	r1, #40	@ 0x28
	f7fb f827 	bl	<core::panicking::panic>

<__cpsid>:
	b672      	cpsid	i
	4770      	bx	lr

<__cpsie>:
	b662      	cpsie	i
	4770      	bx	lr

<__delay>:
	2101      	movs	r1, #1
	eb01 0050 	add.w	r0, r1, r0, lsr #1
	3801      	subs	r0, #1
	d1fd      	bne.n	<__delay+0x6>
	4770      	bx	lr

<__dsb>:
	f3bf 8f4f 	dsb	sy
	4770      	bx	lr

<__primask_r>:
	f3ef 8010 	mrs	r0, PRIMASK
	4770      	bx	lr

<__udf>:
	de00      	udf	#0
	defe      	udf	#254	@ 0xfe

<__aeabi_memclr8>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	2904      	cmp	r1, #4
	d305      	bcc.n	<__aeabi_memclr8+0x14>
	1f0b      	subs	r3, r1, #4
	43da      	mvns	r2, r3
	f012 0f0c 	tst.w	r2, #12
	d102      	bne.n	<__aeabi_memclr8+0x18>
	e01a      	b.n	<__aeabi_memclr8+0x4a>
	4602      	mov	r2, r0
	e024      	b.n	<__aeabi_memclr8+0x62>
	f04f 0c00 	mov.w	ip, #0
	4602      	mov	r2, r0
	f842 cb04 	str.w	ip, [r2], #4
	f013 0f0c 	tst.w	r3, #12
	d008      	beq.n	<__aeabi_memclr8+0x3a>
	f003 020c 	and.w	r2, r3, #12
	f8c0 c004 	str.w	ip, [r0, #4]
	2a04      	cmp	r2, #4
	d105      	bne.n	<__aeabi_memclr8+0x40>
	3908      	subs	r1, #8
	3008      	adds	r0, #8
	e006      	b.n	<__aeabi_memclr8+0x48>
	4619      	mov	r1, r3
	4610      	mov	r0, r2
	e004      	b.n	<__aeabi_memclr8+0x4a>
	2200      	movs	r2, #0
	390c      	subs	r1, #12
	6082      	str	r2, [r0, #8]
	300c      	adds	r0, #12
	4602      	mov	r2, r0
	2b0c      	cmp	r3, #12
	d309      	bcc.n	<__aeabi_memclr8+0x62>
	2300      	movs	r3, #0
	4602      	mov	r2, r0
	3910      	subs	r1, #16
	e9c2 3300 	strd	r3, r3, [r2]
	e9c2 3302 	strd	r3, r3, [r2, #8]
	3210      	adds	r2, #16
	2903      	cmp	r1, #3
	d8f7      	bhi.n	<__aeabi_memclr8+0x52>
	1850      	adds	r0, r2, r1
	4282      	cmp	r2, r0
	d221      	bcs.n	<__aeabi_memclr8+0xac>
	f1a1 0c01 	sub.w	ip, r1, #1
	f011 0303 	ands.w	r3, r1, #3
	d00c      	beq.n	<__aeabi_memclr8+0x8c>
	f04f 0e00 	mov.w	lr, #0
	4611      	mov	r1, r2
	f801 eb01 	strb.w	lr, [r1], #1
	2b01      	cmp	r3, #1
	d00a      	beq.n	<__aeabi_memclr8+0x96>
	2b02      	cmp	r3, #2
	f882 e001 	strb.w	lr, [r2, #1]
	d103      	bne.n	<__aeabi_memclr8+0x90>
	1c91      	adds	r1, r2, #2
	e004      	b.n	<__aeabi_memclr8+0x96>
	4611      	mov	r1, r2
	e002      	b.n	<__aeabi_memclr8+0x96>
	2100      	movs	r1, #0
	7091      	strb	r1, [r2, #2]
	1cd1      	adds	r1, r2, #3
	f1bc 0f03 	cmp.w	ip, #3
	bf38      	it	cc
	bd80      	popcc	{r7, pc}
	3904      	subs	r1, #4
	2200      	movs	r2, #0
	f841 2f04 	str.w	r2, [r1, #4]!
	1d0b      	adds	r3, r1, #4
	4283      	cmp	r3, r0
	d1fa      	bne.n	<__aeabi_memclr8+0xa2>
	bd80      	pop	{r7, pc}

<__aeabi_memcpy4>:
	2a04      	cmp	r2, #4
	d309      	bcc.n	<__aeabi_memcpy4+0x18>
	b5d0      	push	{r4, r6, r7, lr}
	af02      	add	r7, sp, #8
	f1a2 0e04 	sub.w	lr, r2, #4
	ea6f 030e 	mvn.w	r3, lr
	f013 0f0c 	tst.w	r3, #12
	d106      	bne.n	<__aeabi_memcpy4+0x24>
	e023      	b.n	<__aeabi_memcpy4+0x60>
	460b      	mov	r3, r1
	4684      	mov	ip, r0
	4660      	mov	r0, ip
	4619      	mov	r1, r3
	f000 b83c 	b.w	<compiler_builtins::mem::memcpy>
	460b      	mov	r3, r1
	4684      	mov	ip, r0
	f853 4b04 	ldr.w	r4, [r3], #4
	f01e 0f0c 	tst.w	lr, #12
	f84c 4b04 	str.w	r4, [ip], #4
	d009      	beq.n	<__aeabi_memcpy4+0x4a>
	684b      	ldr	r3, [r1, #4]
	6043      	str	r3, [r0, #4]
	f00e 030c 	and.w	r3, lr, #12
	2b04      	cmp	r3, #4
	d107      	bne.n	<__aeabi_memcpy4+0x52>
	3a08      	subs	r2, #8
	3108      	adds	r1, #8
	3008      	adds	r0, #8
	e008      	b.n	<__aeabi_memcpy4+0x5c>
	4672      	mov	r2, lr
	4660      	mov	r0, ip
	4619      	mov	r1, r3
	e006      	b.n	<__aeabi_memcpy4+0x60>
	688b      	ldr	r3, [r1, #8]
	3a0c      	subs	r2, #12
	6083      	str	r3, [r0, #8]
	310c      	adds	r1, #12
	300c      	adds	r0, #12
	4684      	mov	ip, r0
	460b      	mov	r3, r1
	f1be 0f0c 	cmp.w	lr, #12
	d314      	bcc.n	<__aeabi_memcpy4+0x90>
	4684      	mov	ip, r0
	460b      	mov	r3, r1
	6818      	ldr	r0, [r3, #0]
	3a10      	subs	r2, #16
	f8cc 0000 	str.w	r0, [ip]
	2a03      	cmp	r2, #3
	6858      	ldr	r0, [r3, #4]
	f8cc 0004 	str.w	r0, [ip, #4]
	6898      	ldr	r0, [r3, #8]
	f8cc 0008 	str.w	r0, [ip, #8]
	68d8      	ldr	r0, [r3, #12]
	f103 0310 	add.w	r3, r3, #16
	f8cc 000c 	str.w	r0, [ip, #12]
	f10c 0c10 	add.w	ip, ip, #16
	d8ec      	bhi.n	<__aeabi_memcpy4+0x6a>
	e8bd 40d0 	ldmia.w	sp!, {r4, r6, r7, lr}
	4660      	mov	r0, ip
	4619      	mov	r1, r3
	f000 b800 	b.w	<compiler_builtins::mem::memcpy>

<compiler_builtins::mem::memcpy>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b08c      	sub	sp, #48	@ 0x30
	2a10      	cmp	r2, #16
	d31e      	bcc.n	<compiler_builtins::mem::memcpy+0x4c>
	4243      	negs	r3, r0
	f003 0803 	and.w	r8, r3, #3
	eb00 0308 	add.w	r3, r0, r8
	4298      	cmp	r0, r3
	d233      	bcs.n	<compiler_builtins::mem::memcpy+0x84>
	f1a8 0c01 	sub.w	ip, r8, #1
	4606      	mov	r6, r0
	460c      	mov	r4, r1
	f1b8 0f00 	cmp.w	r8, #0
	d01a      	beq.n	<compiler_builtins::mem::memcpy+0x60>
	460c      	mov	r4, r1
	4606      	mov	r6, r0
	f814 eb01 	ldrb.w	lr, [r4], #1
	f1b8 0f01 	cmp.w	r8, #1
	f806 eb01 	strb.w	lr, [r6], #1
	d011      	beq.n	<compiler_builtins::mem::memcpy+0x60>
	784e      	ldrb	r6, [r1, #1]
	f1b8 0f02 	cmp.w	r8, #2
	7046      	strb	r6, [r0, #1]
	d108      	bne.n	<compiler_builtins::mem::memcpy+0x58>
	1c8c      	adds	r4, r1, #2
	1c86      	adds	r6, r0, #2
	e009      	b.n	<compiler_builtins::mem::memcpy+0x60>
	4684      	mov	ip, r0
	eb0c 0302 	add.w	r3, ip, r2
	459c      	cmp	ip, r3
	d340      	bcc.n	<compiler_builtins::mem::memcpy+0xd8>
	e06c      	b.n	<compiler_builtins::mem::memcpy+0x132>
	788e      	ldrb	r6, [r1, #2]
	1ccc      	adds	r4, r1, #3
	7086      	strb	r6, [r0, #2]
	1cc6      	adds	r6, r0, #3
	f1bc 0f03 	cmp.w	ip, #3
	d30e      	bcc.n	<compiler_builtins::mem::memcpy+0x84>
	3c04      	subs	r4, #4
	3e04      	subs	r6, #4
	f814 5f04 	ldrb.w	r5, [r4, #4]!
	f806 5f04 	strb.w	r5, [r6, #4]!
	7865      	ldrb	r5, [r4, #1]
	7075      	strb	r5, [r6, #1]
	78a5      	ldrb	r5, [r4, #2]
	70b5      	strb	r5, [r6, #2]
	78e5      	ldrb	r5, [r4, #3]
	70f5      	strb	r5, [r6, #3]
	1d35      	adds	r5, r6, #4
	429d      	cmp	r5, r3
	d1f2      	bne.n	<compiler_builtins::mem::memcpy+0x6a>
	eba2 0e08 	sub.w	lr, r2, r8
	eb01 0408 	add.w	r4, r1, r8
	f02e 0203 	bic.w	r2, lr, #3
	f014 0603 	ands.w	r6, r4, #3
	eb03 0c02 	add.w	ip, r3, r2
	d159      	bne.n	<compiler_builtins::mem::memcpy+0x14e>
	4563      	cmp	r3, ip
	d215      	bcs.n	<compiler_builtins::mem::memcpy+0xca>
	4621      	mov	r1, r4
	680d      	ldr	r5, [r1, #0]
	f843 5b04 	str.w	r5, [r3], #4
	4563      	cmp	r3, ip
	d20f      	bcs.n	<compiler_builtins::mem::memcpy+0xca>
	684d      	ldr	r5, [r1, #4]
	f843 5b04 	str.w	r5, [r3], #4
	4563      	cmp	r3, ip
	bf3e      	ittt	cc
	688d      	ldrcc	r5, [r1, #8]
	f843 5b04 	strcc.w	r5, [r3], #4
	4563      	cmpcc	r3, ip
	d205      	bcs.n	<compiler_builtins::mem::memcpy+0xca>
	68cd      	ldr	r5, [r1, #12]
	3110      	adds	r1, #16
	f843 5b04 	str.w	r5, [r3], #4
	4563      	cmp	r3, ip
	d3ea      	bcc.n	<compiler_builtins::mem::memcpy+0xa0>
	18a1      	adds	r1, r4, r2
	f00e 0203 	and.w	r2, lr, #3
	eb0c 0302 	add.w	r3, ip, r2
	459c      	cmp	ip, r3
	d22c      	bcs.n	<compiler_builtins::mem::memcpy+0x132>
	f1a2 0e01 	sub.w	lr, r2, #1
	f012 0403 	ands.w	r4, r2, #3
	d013      	beq.n	<compiler_builtins::mem::memcpy+0x10a>
	460a      	mov	r2, r1
	4665      	mov	r5, ip
	f812 6b01 	ldrb.w	r6, [r2], #1
	2c01      	cmp	r4, #1
	f805 6b01 	strb.w	r6, [r5], #1
	d00d      	beq.n	<compiler_builtins::mem::memcpy+0x10e>
	784a      	ldrb	r2, [r1, #1]
	2c02      	cmp	r4, #2
	f88c 2001 	strb.w	r2, [ip, #1]
	d11e      	bne.n	<compiler_builtins::mem::memcpy+0x13a>
	1c8a      	adds	r2, r1, #2
	f10c 0502 	add.w	r5, ip, #2
	f1be 0f03 	cmp.w	lr, #3
	d205      	bcs.n	<compiler_builtins::mem::memcpy+0x114>
	e013      	b.n	<compiler_builtins::mem::memcpy+0x132>
	4665      	mov	r5, ip
	460a      	mov	r2, r1
	f1be 0f03 	cmp.w	lr, #3
	d30e      	bcc.n	<compiler_builtins::mem::memcpy+0x132>
	1f11      	subs	r1, r2, #4
	1f2a      	subs	r2, r5, #4
	f811 6f04 	ldrb.w	r6, [r1, #4]!
	f802 6f04 	strb.w	r6, [r2, #4]!
	784e      	ldrb	r6, [r1, #1]
	7056      	strb	r6, [r2, #1]
	788e      	ldrb	r6, [r1, #2]
	7096      	strb	r6, [r2, #2]
	78ce      	ldrb	r6, [r1, #3]
	70d6      	strb	r6, [r2, #3]
	1d16      	adds	r6, r2, #4
	429e      	cmp	r6, r3
	d1f2      	bne.n	<compiler_builtins::mem::memcpy+0x118>
	b00c      	add	sp, #48	@ 0x30
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	788a      	ldrb	r2, [r1, #2]
	f10c 0503 	add.w	r5, ip, #3
	f88c 2002 	strb.w	r2, [ip, #2]
	1cca      	adds	r2, r1, #3
	f1be 0f03 	cmp.w	lr, #3
	d2e3      	bcs.n	<compiler_builtins::mem::memcpy+0x114>
	e7f1      	b.n	<compiler_builtins::mem::memcpy+0x132>
	f04f 0900 	mov.w	r9, #0
	f1c6 0a04 	rsb	sl, r6, #4
	ad0b      	add	r5, sp, #44	@ 0x2c
	f8cd 902c 	str.w	r9, [sp, #44]	@ 0x2c
	eb05 0b06 	add.w	fp, r5, r6
	ea5f 75ca 	movs.w	r5, sl, lsl #31
	bf1e      	ittt	ne
	7825      	ldrbne	r5, [r4, #0]
	f88b 5000 	strbne.w	r5, [fp]
	f04f 0901 	movne.w	r9, #1
	1ba5      	subs	r5, r4, r6
	9507      	str	r5, [sp, #28]
	00f5      	lsls	r5, r6, #3
	9501      	str	r5, [sp, #4]
	ea5f 758a 	movs.w	r5, sl, lsl #30
	bf44      	itt	mi
	f834 5009 	ldrhmi.w	r5, [r4, r9]
	f82b 5009 	strhmi.w	r5, [fp, r9]
	1d1d      	adds	r5, r3, #4
	f8dd b02c 	ldr.w	fp, [sp, #44]	@ 0x2c
	4565      	cmp	r5, ip
	9d01      	ldr	r5, [sp, #4]
	46aa      	mov	sl, r5
	f1c5 0500 	rsb	r5, r5, #0
	d25c      	bcs.n	<compiler_builtins::mem::memcpy+0x252>
	4273      	negs	r3, r6
	9500      	str	r5, [sp, #0]
	eb01 0903 	add.w	r9, r1, r3
	f005 0118 	and.w	r1, r5, #24
	4605      	mov	r5, r0
	9107      	str	r1, [sp, #28]
	f8cd 9010 	str.w	r9, [sp, #16]
	44c1      	add	r9, r8
	9b07      	ldr	r3, [sp, #28]
	fa2b fb0a 	lsr.w	fp, fp, sl
	f8d9 1004 	ldr.w	r1, [r9, #4]
	9108      	str	r1, [sp, #32]
	fa01 f303 	lsl.w	r3, r1, r3
	eb05 0108 	add.w	r1, r5, r8
	ea43 030b 	orr.w	r3, r3, fp
	468b      	mov	fp, r1
	f84b 3b08 	str.w	r3, [fp], #8
	45e3      	cmp	fp, ip
	d242      	bcs.n	<compiler_builtins::mem::memcpy+0x256>
	9b08      	ldr	r3, [sp, #32]
	9503      	str	r5, [sp, #12]
	f8cd 9018 	str.w	r9, [sp, #24]
	fa23 f50a 	lsr.w	r5, r3, sl
	f8d9 9008 	ldr.w	r9, [r9, #8]
	9b07      	ldr	r3, [sp, #28]
	9105      	str	r1, [sp, #20]
	fa09 f303 	lsl.w	r3, r9, r3
	432b      	orrs	r3, r5
	604b      	str	r3, [r1, #4]
	f101 030c 	add.w	r3, r1, #12
	4563      	cmp	r3, ip
	d236      	bcs.n	<compiler_builtins::mem::memcpy+0x262>
	9d06      	ldr	r5, [sp, #24]
	68ed      	ldr	r5, [r5, #12]
	9508      	str	r5, [sp, #32]
	fa29 f50a 	lsr.w	r5, r9, sl
	9502      	str	r5, [sp, #8]
	e9dd 5107 	ldrd	r5, r1, [sp, #28]
	fa01 f905 	lsl.w	r9, r1, r5
	9905      	ldr	r1, [sp, #20]
	9d02      	ldr	r5, [sp, #8]
	3110      	adds	r1, #16
	ea45 0509 	orr.w	r5, r5, r9
	4561      	cmp	r1, ip
	f8cb 5000 	str.w	r5, [fp]
	d22d      	bcs.n	<compiler_builtins::mem::memcpy+0x276>
	9906      	ldr	r1, [sp, #24]
	9d07      	ldr	r5, [sp, #28]
	f8dd 9010 	ldr.w	r9, [sp, #16]
	f8d1 b010 	ldr.w	fp, [r1, #16]
	9908      	ldr	r1, [sp, #32]
	f109 0910 	add.w	r9, r9, #16
	fa0b f505 	lsl.w	r5, fp, r5
	fa21 f10a 	lsr.w	r1, r1, sl
	4329      	orrs	r1, r5
	9d03      	ldr	r5, [sp, #12]
	6019      	str	r1, [r3, #0]
	3510      	adds	r5, #16
	eb05 0308 	add.w	r3, r5, r8
	1d19      	adds	r1, r3, #4
	4561      	cmp	r1, ip
	d3b0      	bcc.n	<compiler_builtins::mem::memcpy+0x1a8>
	eb09 0108 	add.w	r1, r9, r8
	9107      	str	r1, [sp, #28]
	f8dd a000 	ldr.w	sl, [sp]
	e018      	b.n	<compiler_builtins::mem::memcpy+0x284>
	46aa      	mov	sl, r5
	e016      	b.n	<compiler_builtins::mem::memcpy+0x284>
	460b      	mov	r3, r1
	f109 0104 	add.w	r1, r9, #4
	9107      	str	r1, [sp, #28]
	3304      	adds	r3, #4
	e00c      	b.n	<compiler_builtins::mem::memcpy+0x27c>
	9906      	ldr	r1, [sp, #24]
	46cb      	mov	fp, r9
	f8dd a000 	ldr.w	sl, [sp]
	3108      	adds	r1, #8
	9107      	str	r1, [sp, #28]
	9905      	ldr	r1, [sp, #20]
	f101 0308 	add.w	r3, r1, #8
	e006      	b.n	<compiler_builtins::mem::memcpy+0x284>
	9906      	ldr	r1, [sp, #24]
	310c      	adds	r1, #12
	9107      	str	r1, [sp, #28]
	f8dd a000 	ldr.w	sl, [sp]
	f8dd b020 	ldr.w	fp, [sp, #32]
	2500      	movs	r5, #0
	2e01      	cmp	r6, #1
	f88d 5028 	strb.w	r5, [sp, #40]	@ 0x28
	f807 5c26 	strb.w	r5, [r7, #-38]
	d105      	bne.n	<compiler_builtins::mem::memcpy+0x29e>
	f10d 0828 	add.w	r8, sp, #40	@ 0x28
	f04f 0900 	mov.w	r9, #0
	2100      	movs	r1, #0
	e009      	b.n	<compiler_builtins::mem::memcpy+0x2b2>
	9907      	ldr	r1, [sp, #28]
	f1a7 0826 	sub.w	r8, r7, #38	@ 0x26
	790d      	ldrb	r5, [r1, #4]
	7949      	ldrb	r1, [r1, #5]
	f88d 5028 	strb.w	r5, [sp, #40]	@ 0x28
	ea4f 2901 	mov.w	r9, r1, lsl #8
	2102      	movs	r1, #2
	07e6      	lsls	r6, r4, #31
	d101      	bne.n	<compiler_builtins::mem::memcpy+0x2ba>
	2100      	movs	r1, #0
	e009      	b.n	<compiler_builtins::mem::memcpy+0x2ce>
	9d07      	ldr	r5, [sp, #28]
	3504      	adds	r5, #4
	5c69      	ldrb	r1, [r5, r1]
	f888 1000 	strb.w	r1, [r8]
	f817 1c26 	ldrb.w	r1, [r7, #-38]
	f89d 5028 	ldrb.w	r5, [sp, #40]	@ 0x28
	0409      	lsls	r1, r1, #16
	ea41 0109 	orr.w	r1, r1, r9
	9e01      	ldr	r6, [sp, #4]
	4329      	orrs	r1, r5
	f00a 0518 	and.w	r5, sl, #24
	40a9      	lsls	r1, r5
	fa2b f606 	lsr.w	r6, fp, r6
	4331      	orrs	r1, r6
	6019      	str	r1, [r3, #0]
	e6f1      	b.n	<compiler_builtins::mem::memcpy+0xca>

<compiler_builtins::mem::memmove>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
	b08e      	sub	sp, #56	@ 0x38
	1a43      	subs	r3, r0, r1
	4293      	cmp	r3, r2
	d22b      	bcs.n	<compiler_builtins::mem::memmove+0x68>
	eb01 0902 	add.w	r9, r1, r2
	eb00 0c02 	add.w	ip, r0, r2
	2a10      	cmp	r2, #16
	d34a      	bcc.n	<compiler_builtins::mem::memmove+0xb2>
	f00c 0a03 	and.w	sl, ip, #3
	f02c 0503 	bic.w	r5, ip, #3
	f1ca 0b00 	rsb	fp, sl, #0
	4565      	cmp	r5, ip
	d264      	bcs.n	<compiler_builtins::mem::memmove+0xf6>
	f1aa 0e01 	sub.w	lr, sl, #1
	f1ba 0f00 	cmp.w	sl, #0
	d049      	beq.n	<compiler_builtins::mem::memmove+0xca>
	46c8      	mov	r8, r9
	4663      	mov	r3, ip
	f818 4d01 	ldrb.w	r4, [r8, #-1]!
	f1ba 0f01 	cmp.w	sl, #1
	f803 4d01 	strb.w	r4, [r3, #-1]!
	d042      	beq.n	<compiler_builtins::mem::memmove+0xce>
	46c8      	mov	r8, r9
	4663      	mov	r3, ip
	f818 4d02 	ldrb.w	r4, [r8, #-2]!
	f1ba 0f02 	cmp.w	sl, #2
	f803 4d02 	strb.w	r4, [r3, #-2]!
	bf1f      	itttt	ne
	46c8      	movne	r8, r9
	f818 4d03 	ldrbne.w	r4, [r8, #-3]!
	4663      	movne	r3, ip
	f803 4d03 	strbne.w	r4, [r3, #-3]!
	e032      	b.n	<compiler_builtins::mem::memmove+0xce>
	2a10      	cmp	r2, #16
	d327      	bcc.n	<compiler_builtins::mem::memmove+0xbc>
	4243      	negs	r3, r0
	f003 0803 	and.w	r8, r3, #3
	eb00 0308 	add.w	r3, r0, r8
	4298      	cmp	r0, r3
	f080 80c3 	bcs.w	<compiler_builtins::mem::memmove+0x202>
	f1a8 0c01 	sub.w	ip, r8, #1
	4606      	mov	r6, r0
	460d      	mov	r5, r1
	f1b8 0f00 	cmp.w	r8, #0
	f000 80a9 	beq.w	<compiler_builtins::mem::memmove+0x1de>
	460d      	mov	r5, r1
	4606      	mov	r6, r0
	f815 eb01 	ldrb.w	lr, [r5], #1
	f1b8 0f01 	cmp.w	r8, #1
	f806 eb01 	strb.w	lr, [r6], #1
	f000 809f 	beq.w	<compiler_builtins::mem::memmove+0x1de>
	784e      	ldrb	r6, [r1, #1]
	f1b8 0f02 	cmp.w	r8, #2
	7046      	strb	r6, [r0, #1]
	f040 8095 	bne.w	<compiler_builtins::mem::memmove+0x1d6>
	1c8d      	adds	r5, r1, #2
	1c86      	adds	r6, r0, #2
	e095      	b.n	<compiler_builtins::mem::memmove+0x1de>
	4663      	mov	r3, ip
	1a99      	subs	r1, r3, r2
	4299      	cmp	r1, r3
	d35b      	bcc.n	<compiler_builtins::mem::memmove+0x172>
	e0f9      	b.n	<compiler_builtins::mem::memmove+0x2b0>
	4684      	mov	ip, r0
	eb0c 0302 	add.w	r3, ip, r2
	459c      	cmp	ip, r3
	f0c0 80c7 	bcc.w	<compiler_builtins::mem::memmove+0x256>
	e0f2      	b.n	<compiler_builtins::mem::memmove+0x2b0>
	4663      	mov	r3, ip
	46c8      	mov	r8, r9
	f1be 0f03 	cmp.w	lr, #3
	d310      	bcc.n	<compiler_builtins::mem::memmove+0xf6>
	f1a8 0604 	sub.w	r6, r8, #4
	78f4      	ldrb	r4, [r6, #3]
	f803 4c01 	strb.w	r4, [r3, #-1]
	78b4      	ldrb	r4, [r6, #2]
	f803 4c02 	strb.w	r4, [r3, #-2]
	7874      	ldrb	r4, [r6, #1]
	f803 4c03 	strb.w	r4, [r3, #-3]
	f816 4904 	ldrb.w	r4, [r6], #-4
	f803 4d04 	strb.w	r4, [r3, #-4]!
	429d      	cmp	r5, r3
	d3f0      	bcc.n	<compiler_builtins::mem::memmove+0xd8>
	eba2 0e0a 	sub.w	lr, r2, sl
	eb09 0a0b 	add.w	sl, r9, fp
	f02e 0403 	bic.w	r4, lr, #3
	1b2b      	subs	r3, r5, r4
	f1c4 0800 	rsb	r8, r4, #0
	f01a 0403 	ands.w	r4, sl, #3
	f040 80de 	bne.w	<compiler_builtins::mem::memmove+0x2cc>
	42ab      	cmp	r3, r5
	d226      	bcs.n	<compiler_builtins::mem::memmove+0x162>
	f1a9 0110 	sub.w	r1, r9, #16
	f1ab 0904 	sub.w	r9, fp, #4
	eb01 060b 	add.w	r6, r1, fp
	eb0c 050b 	add.w	r5, ip, fp
	68f4      	ldr	r4, [r6, #12]
	f845 4c04 	str.w	r4, [r5, #-4]
	eb0c 0409 	add.w	r4, ip, r9
	42a3      	cmp	r3, r4
	d217      	bcs.n	<compiler_builtins::mem::memmove+0x162>
	68b2      	ldr	r2, [r6, #8]
	f845 2c08 	str.w	r2, [r5, #-8]
	1f22      	subs	r2, r4, #4
	4293      	cmp	r3, r2
	bf3f      	itttt	cc
	6872      	ldrcc	r2, [r6, #4]
	f845 2c0c 	strcc.w	r2, [r5, #-12]
	f1a4 0208 	subcc.w	r2, r4, #8
	4293      	cmpcc	r3, r2
	d20a      	bcs.n	<compiler_builtins::mem::memmove+0x162>
	f851 200b 	ldr.w	r2, [r1, fp]
	f1ac 0c10 	sub.w	ip, ip, #16
	3910      	subs	r1, #16
	f845 2c10 	str.w	r2, [r5, #-16]
	eb0c 020b 	add.w	r2, ip, fp
	4293      	cmp	r3, r2
	d3dc      	bcc.n	<compiler_builtins::mem::memmove+0x11c>
	eb0a 0908 	add.w	r9, sl, r8
	f00e 0203 	and.w	r2, lr, #3
	1a99      	subs	r1, r3, r2
	4299      	cmp	r1, r3
	f080 809f 	bcs.w	<compiler_builtins::mem::memmove+0x2b0>
	f1a2 0c01 	sub.w	ip, r2, #1
	f012 0403 	ands.w	r4, r2, #3
	d013      	beq.n	<compiler_builtins::mem::memmove+0x1a4>
	464d      	mov	r5, r9
	461a      	mov	r2, r3
	f815 6d01 	ldrb.w	r6, [r5, #-1]!
	2c01      	cmp	r4, #1
	f802 6d01 	strb.w	r6, [r2, #-1]!
	d00d      	beq.n	<compiler_builtins::mem::memmove+0x1a8>
	464d      	mov	r5, r9
	461a      	mov	r2, r3
	f815 6d02 	ldrb.w	r6, [r5, #-2]!
	2c02      	cmp	r4, #2
	f802 6d02 	strb.w	r6, [r2, #-2]!
	d005      	beq.n	<compiler_builtins::mem::memmove+0x1a8>
	f819 2d03 	ldrb.w	r2, [r9, #-3]!
	f803 2d03 	strb.w	r2, [r3, #-3]!
	464d      	mov	r5, r9
	461a      	mov	r2, r3
	f1bc 0f03 	cmp.w	ip, #3
	f0c0 8080 	bcc.w	<compiler_builtins::mem::memmove+0x2b0>
	1eab      	subs	r3, r5, #2
	785e      	ldrb	r6, [r3, #1]
	f802 6c01 	strb.w	r6, [r2, #-1]
	781e      	ldrb	r6, [r3, #0]
	f802 6c02 	strb.w	r6, [r2, #-2]
	f813 6c01 	ldrb.w	r6, [r3, #-1]
	f802 6c03 	strb.w	r6, [r2, #-3]
	f813 6c02 	ldrb.w	r6, [r3, #-2]
	3b04      	subs	r3, #4
	f802 6d04 	strb.w	r6, [r2, #-4]!
	4291      	cmp	r1, r2
	d3ee      	bcc.n	<compiler_builtins::mem::memmove+0x1b2>
	e06c      	b.n	<compiler_builtins::mem::memmove+0x2b0>
	788e      	ldrb	r6, [r1, #2]
	1ccd      	adds	r5, r1, #3
	7086      	strb	r6, [r0, #2]
	1cc6      	adds	r6, r0, #3
	f1bc 0f03 	cmp.w	ip, #3
	d30e      	bcc.n	<compiler_builtins::mem::memmove+0x202>
	3d04      	subs	r5, #4
	3e04      	subs	r6, #4
	f815 4f04 	ldrb.w	r4, [r5, #4]!
	f806 4f04 	strb.w	r4, [r6, #4]!
	786c      	ldrb	r4, [r5, #1]
	7074      	strb	r4, [r6, #1]
	78ac      	ldrb	r4, [r5, #2]
	70b4      	strb	r4, [r6, #2]
	78ec      	ldrb	r4, [r5, #3]
	70f4      	strb	r4, [r6, #3]
	1d34      	adds	r4, r6, #4
	429c      	cmp	r4, r3
	d1f2      	bne.n	<compiler_builtins::mem::memmove+0x1e8>
	eba2 0208 	sub.w	r2, r2, r8
	eb01 0508 	add.w	r5, r1, r8
	f022 0603 	bic.w	r6, r2, #3
	f015 0903 	ands.w	r9, r5, #3
	eb03 0c06 	add.w	ip, r3, r6
	d16d      	bne.n	<compiler_builtins::mem::memmove+0x2f4>
	4563      	cmp	r3, ip
	d215      	bcs.n	<compiler_builtins::mem::memmove+0x248>
	4629      	mov	r1, r5
	680c      	ldr	r4, [r1, #0]
	f843 4b04 	str.w	r4, [r3], #4
	4563      	cmp	r3, ip
	d20f      	bcs.n	<compiler_builtins::mem::memmove+0x248>
	684c      	ldr	r4, [r1, #4]
	f843 4b04 	str.w	r4, [r3], #4
	4563      	cmp	r3, ip
	bf3e      	ittt	cc
	688c      	ldrcc	r4, [r1, #8]
	f843 4b04 	strcc.w	r4, [r3], #4
	4563      	cmpcc	r3, ip
	d205      	bcs.n	<compiler_builtins::mem::memmove+0x248>
	68cc      	ldr	r4, [r1, #12]
	3110      	adds	r1, #16
	f843 4b04 	str.w	r4, [r3], #4
	4563      	cmp	r3, ip
	d3ea      	bcc.n	<compiler_builtins::mem::memmove+0x21e>
	19a9      	adds	r1, r5, r6
	f002 0203 	and.w	r2, r2, #3
	eb0c 0302 	add.w	r3, ip, r2
	459c      	cmp	ip, r3
	d22c      	bcs.n	<compiler_builtins::mem::memmove+0x2b0>
	f1a2 0e01 	sub.w	lr, r2, #1
	f012 0403 	ands.w	r4, r2, #3
	d013      	beq.n	<compiler_builtins::mem::memmove+0x288>
	460a      	mov	r2, r1
	4665      	mov	r5, ip
	f812 6b01 	ldrb.w	r6, [r2], #1
	2c01      	cmp	r4, #1
	f805 6b01 	strb.w	r6, [r5], #1
	d00d      	beq.n	<compiler_builtins::mem::memmove+0x28c>
	784a      	ldrb	r2, [r1, #1]
	2c02      	cmp	r4, #2
	f88c 2001 	strb.w	r2, [ip, #1]
	d11e      	bne.n	<compiler_builtins::mem::memmove+0x2b8>
	1c8a      	adds	r2, r1, #2
	f10c 0502 	add.w	r5, ip, #2
	f1be 0f03 	cmp.w	lr, #3
	d205      	bcs.n	<compiler_builtins::mem::memmove+0x292>
	e013      	b.n	<compiler_builtins::mem::memmove+0x2b0>
	4665      	mov	r5, ip
	460a      	mov	r2, r1
	f1be 0f03 	cmp.w	lr, #3
	d30e      	bcc.n	<compiler_builtins::mem::memmove+0x2b0>
	1f11      	subs	r1, r2, #4
	1f2a      	subs	r2, r5, #4
	f811 6f04 	ldrb.w	r6, [r1, #4]!
	f802 6f04 	strb.w	r6, [r2, #4]!
	784e      	ldrb	r6, [r1, #1]
	7056      	strb	r6, [r2, #1]
	788e      	ldrb	r6, [r1, #2]
	7096      	strb	r6, [r2, #2]
	78ce      	ldrb	r6, [r1, #3]
	70d6      	strb	r6, [r2, #3]
	1d16      	adds	r6, r2, #4
	429e      	cmp	r6, r3
	d1f2      	bne.n	<compiler_builtins::mem::memmove+0x296>
	b00e      	add	sp, #56	@ 0x38
	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	788a      	ldrb	r2, [r1, #2]
	f10c 0503 	add.w	r5, ip, #3
	f88c 2002 	strb.w	r2, [ip, #2]
	1cca      	adds	r2, r1, #3
	f1be 0f03 	cmp.w	lr, #3
	d2e3      	bcs.n	<compiler_builtins::mem::memmove+0x292>
	e7f1      	b.n	<compiler_builtins::mem::memmove+0x2b0>
	ebaa 0604 	sub.w	r6, sl, r4
	f04f 0900 	mov.w	r9, #0
	9607      	str	r6, [sp, #28]
	00e6      	lsls	r6, r4, #3
	2c01      	cmp	r4, #1
	f88d 9028 	strb.w	r9, [sp, #40]	@ 0x28
	f807 9c2e 	strb.w	r9, [r7, #-46]
	9606      	str	r6, [sp, #24]
	9402      	str	r4, [sp, #8]
	f040 808e 	bne.w	<compiler_builtins::mem::memmove+0x406>
	ae0a      	add	r6, sp, #40	@ 0x28
	9605      	str	r6, [sp, #20]
	2600      	movs	r6, #0
	9608      	str	r6, [sp, #32]
	e099      	b.n	<compiler_builtins::mem::memmove+0x428>
	f04f 0b00 	mov.w	fp, #0
	f1c9 0a04 	rsb	sl, r9, #4
	ac0d      	add	r4, sp, #52	@ 0x34
	f8cd b034 	str.w	fp, [sp, #52]	@ 0x34
	eb04 0e09 	add.w	lr, r4, r9
	ea5f 74ca 	movs.w	r4, sl, lsl #31
	bf1e      	ittt	ne
	782c      	ldrbne	r4, [r5, #0]
	f88e 4000 	strbne.w	r4, [lr]
	f04f 0b01 	movne.w	fp, #1
	eba5 0409 	sub.w	r4, r5, r9
	9407      	str	r4, [sp, #28]
	ea4f 04c9 	mov.w	r4, r9, lsl #3
	9401      	str	r4, [sp, #4]
	ea5f 748a 	movs.w	r4, sl, lsl #30
	bf44      	itt	mi
	f835 400b 	ldrhmi.w	r4, [r5, fp]
	f82e 400b 	strhmi.w	r4, [lr, fp]
	9c0d      	ldr	r4, [sp, #52]	@ 0x34
	f8dd a004 	ldr.w	sl, [sp, #4]
	9408      	str	r4, [sp, #32]
	1d1c      	adds	r4, r3, #4
	4564      	cmp	r4, ip
	f1ca 0400 	rsb	r4, sl, #0
	9400      	str	r4, [sp, #0]
	f080 8105 	bcs.w	<compiler_builtins::mem::memmove+0x550>
	f1c9 0300 	rsb	r3, r9, #0
	f8dd b020 	ldr.w	fp, [sp, #32]
	eb01 0e03 	add.w	lr, r1, r3
	f004 0118 	and.w	r1, r4, #24
	4604      	mov	r4, r0
	9107      	str	r1, [sp, #28]
	f8cd e010 	str.w	lr, [sp, #16]
	44c6      	add	lr, r8
	9b07      	ldr	r3, [sp, #28]
	fa2b fb0a 	lsr.w	fp, fp, sl
	f8de 1004 	ldr.w	r1, [lr, #4]
	9108      	str	r1, [sp, #32]
	9403      	str	r4, [sp, #12]
	fa01 f303 	lsl.w	r3, r1, r3
	eb04 0108 	add.w	r1, r4, r8
	ea43 030b 	orr.w	r3, r3, fp
	468b      	mov	fp, r1
	f84b 3b08 	str.w	r3, [fp], #8
	45e3      	cmp	fp, ip
	f080 80c6 	bcs.w	<compiler_builtins::mem::memmove+0x512>
	9b08      	ldr	r3, [sp, #32]
	f8cd e018 	str.w	lr, [sp, #24]
	f8de e008 	ldr.w	lr, [lr, #8]
	fa23 f40a 	lsr.w	r4, r3, sl
	9b07      	ldr	r3, [sp, #28]
	9105      	str	r1, [sp, #20]
	fa0e f303 	lsl.w	r3, lr, r3
	4323      	orrs	r3, r4
	604b      	str	r3, [r1, #4]
	f101 030c 	add.w	r3, r1, #12
	4563      	cmp	r3, ip
	f080 80be 	bcs.w	<compiler_builtins::mem::memmove+0x526>
	9c06      	ldr	r4, [sp, #24]
	68e4      	ldr	r4, [r4, #12]
	9408      	str	r4, [sp, #32]
	fa2e f40a 	lsr.w	r4, lr, sl
	9402      	str	r4, [sp, #8]
	e9dd 4107 	ldrd	r4, r1, [sp, #28]
	fa01 fe04 	lsl.w	lr, r1, r4
	9905      	ldr	r1, [sp, #20]
	9c02      	ldr	r4, [sp, #8]
	3110      	adds	r1, #16
	ea44 040e 	orr.w	r4, r4, lr
	4561      	cmp	r1, ip
	f8cb 4000 	str.w	r4, [fp]
	f080 80bc 	bcs.w	<compiler_builtins::mem::memmove+0x54a>
	9906      	ldr	r1, [sp, #24]
	9c07      	ldr	r4, [sp, #28]
	f8dd e010 	ldr.w	lr, [sp, #16]
	f8d1 b010 	ldr.w	fp, [r1, #16]
	9908      	ldr	r1, [sp, #32]
	f10e 0e10 	add.w	lr, lr, #16
	fa0b f404 	lsl.w	r4, fp, r4
	fa21 f10a 	lsr.w	r1, r1, sl
	4321      	orrs	r1, r4
	9c03      	ldr	r4, [sp, #12]
	6019      	str	r1, [r3, #0]
	3410      	adds	r4, #16
	eb04 0308 	add.w	r3, r4, r8
	1d19      	adds	r1, r3, #4
	4561      	cmp	r1, ip
	d3ad      	bcc.n	<compiler_builtins::mem::memmove+0x35a>
	eb0e 0108 	add.w	r1, lr, r8
	9107      	str	r1, [sp, #28]
	e0a6      	b.n	<compiler_builtins::mem::memmove+0x554>
	9e07      	ldr	r6, [sp, #28]
	f896 9000 	ldrb.w	r9, [r6]
	7876      	ldrb	r6, [r6, #1]
	9608      	str	r6, [sp, #32]
	ea5f 76ca 	movs.w	r6, sl, lsl #31
	f88d 9028 	strb.w	r9, [sp, #40]	@ 0x28
	d101      	bne.n	<compiler_builtins::mem::memmove+0x41e>
	2400      	movs	r4, #0
	e00e      	b.n	<compiler_builtins::mem::memmove+0x43c>
	f04f 0902 	mov.w	r9, #2
	f1a7 062e 	sub.w	r6, r7, #46	@ 0x2e
	9605      	str	r6, [sp, #20]
	9e07      	ldr	r6, [sp, #28]
	9c05      	ldr	r4, [sp, #20]
	f816 6009 	ldrb.w	r6, [r6, r9]
	7026      	strb	r6, [r4, #0]
	f817 6c2e 	ldrb.w	r6, [r7, #-46]
	f89d 9028 	ldrb.w	r9, [sp, #40]	@ 0x28
	0434      	lsls	r4, r6, #16
	9e08      	ldr	r6, [sp, #32]
	ea44 2606 	orr.w	r6, r4, r6, lsl #8
	9c02      	ldr	r4, [sp, #8]
	ea46 0609 	orr.w	r6, r6, r9
	9605      	str	r6, [sp, #20]
	1d1e      	adds	r6, r3, #4
	9608      	str	r6, [sp, #32]
	42ae      	cmp	r6, r5
	9e06      	ldr	r6, [sp, #24]
	f1c6 0900 	rsb	r9, r6, #0
	d259      	bcs.n	<compiler_builtins::mem::memmove+0x50c>
	1b12      	subs	r2, r2, r4
	f8cd 9004 	str.w	r9, [sp, #4]
	4411      	add	r1, r2
	f1ab 0204 	sub.w	r2, fp, #4
	9203      	str	r2, [sp, #12]
	f009 0218 	and.w	r2, r9, #24
	9207      	str	r2, [sp, #28]
	9a05      	ldr	r2, [sp, #20]
	eb01 090b 	add.w	r9, r1, fp
	9104      	str	r1, [sp, #16]
	9907      	ldr	r1, [sp, #28]
	eb0c 060b 	add.w	r6, ip, fp
	f859 4c04 	ldr.w	r4, [r9, #-4]
	9d06      	ldr	r5, [sp, #24]
	fa02 f101 	lsl.w	r1, r2, r1
	fa24 f205 	lsr.w	r2, r4, r5
	4311      	orrs	r1, r2
	f846 1c04 	str.w	r1, [r6, #-4]
	9903      	ldr	r1, [sp, #12]
	eb0c 0201 	add.w	r2, ip, r1
	9908      	ldr	r1, [sp, #32]
	4291      	cmp	r1, r2
	d240      	bcs.n	<compiler_builtins::mem::memmove+0x51c>
	f859 1c08 	ldr.w	r1, [r9, #-8]
	9105      	str	r1, [sp, #20]
	9907      	ldr	r1, [sp, #28]
	408c      	lsls	r4, r1
	9905      	ldr	r1, [sp, #20]
	fa21 f505 	lsr.w	r5, r1, r5
	9908      	ldr	r1, [sp, #32]
	4325      	orrs	r5, r4
	f846 5c08 	str.w	r5, [r6, #-8]
	1f15      	subs	r5, r2, #4
	42a9      	cmp	r1, r5
	d23e      	bcs.n	<compiler_builtins::mem::memmove+0x536>
	9907      	ldr	r1, [sp, #28]
	9d05      	ldr	r5, [sp, #20]
	f859 4c0c 	ldr.w	r4, [r9, #-12]
	fa05 f101 	lsl.w	r1, r5, r1
	9d06      	ldr	r5, [sp, #24]
	fa24 f505 	lsr.w	r5, r4, r5
	4329      	orrs	r1, r5
	f846 1c0c 	str.w	r1, [r6, #-12]
	f1a2 0508 	sub.w	r5, r2, #8
	9908      	ldr	r1, [sp, #32]
	42a9      	cmp	r1, r5
	d275      	bcs.n	<compiler_builtins::mem::memmove+0x5c6>
	9907      	ldr	r1, [sp, #28]
	f1ac 0c10 	sub.w	ip, ip, #16
	f859 9c10 	ldr.w	r9, [r9, #-16]
	eb0c 050b 	add.w	r5, ip, fp
	9a06      	ldr	r2, [sp, #24]
	fa04 f101 	lsl.w	r1, r4, r1
	fa29 f202 	lsr.w	r2, r9, r2
	4311      	orrs	r1, r2
	f846 1c10 	str.w	r1, [r6, #-16]
	9904      	ldr	r1, [sp, #16]
	9a08      	ldr	r2, [sp, #32]
	3910      	subs	r1, #16
	42aa      	cmp	r2, r5
	464a      	mov	r2, r9
	d3b4      	bcc.n	<compiler_builtins::mem::memmove+0x46e>
	4459      	add	r1, fp
	4693      	mov	fp, r2
	9107      	str	r1, [sp, #28]
	e01b      	b.n	<compiler_builtins::mem::memmove+0x544>
	f8dd b014 	ldr.w	fp, [sp, #20]
	e062      	b.n	<compiler_builtins::mem::memmove+0x5d8>
	460b      	mov	r3, r1
	f10e 0104 	add.w	r1, lr, #4
	3304      	adds	r3, #4
	e018      	b.n	<compiler_builtins::mem::memmove+0x54e>
	e9dd 6103 	ldrd	r6, r1, [sp, #12]
	4615      	mov	r5, r2
	4431      	add	r1, r6
	e053      	b.n	<compiler_builtins::mem::memmove+0x5ce>
	9906      	ldr	r1, [sp, #24]
	46f3      	mov	fp, lr
	3108      	adds	r1, #8
	9107      	str	r1, [sp, #28]
	9905      	ldr	r1, [sp, #20]
	f101 0308 	add.w	r3, r1, #8
	e00e      	b.n	<compiler_builtins::mem::memmove+0x554>
	e9dd 6203 	ldrd	r6, r2, [sp, #12]
	f8dd b014 	ldr.w	fp, [sp, #20]
	4432      	add	r2, r6
	3a04      	subs	r2, #4
	9207      	str	r2, [sp, #28]
	e9dd 9401 	ldrd	r9, r4, [sp, #4]
	e046      	b.n	<compiler_builtins::mem::memmove+0x5d8>
	9906      	ldr	r1, [sp, #24]
	310c      	adds	r1, #12
	9107      	str	r1, [sp, #28]
	f8dd b020 	ldr.w	fp, [sp, #32]
	2400      	movs	r4, #0
	f1b9 0f01 	cmp.w	r9, #1
	f88d 402c 	strb.w	r4, [sp, #44]	@ 0x2c
	f807 4c2a 	strb.w	r4, [r7, #-42]
	d106      	bne.n	<compiler_builtins::mem::memmove+0x572>
	f10d 092c 	add.w	r9, sp, #44	@ 0x2c
	f04f 0800 	mov.w	r8, #0
	f04f 0e00 	mov.w	lr, #0
	e00a      	b.n	<compiler_builtins::mem::memmove+0x588>
	9907      	ldr	r1, [sp, #28]
	f1a7 092a 	sub.w	r9, r7, #42	@ 0x2a
	f04f 0e02 	mov.w	lr, #2
	790c      	ldrb	r4, [r1, #4]
	7949      	ldrb	r1, [r1, #5]
	f88d 402c 	strb.w	r4, [sp, #44]	@ 0x2c
	ea4f 2801 	mov.w	r8, r1, lsl #8
	07e9      	lsls	r1, r5, #31
	d102      	bne.n	<compiler_builtins::mem::memmove+0x592>
	f04f 0e00 	mov.w	lr, #0
	e00b      	b.n	<compiler_builtins::mem::memmove+0x5aa>
	9907      	ldr	r1, [sp, #28]
	3104      	adds	r1, #4
	f811 100e 	ldrb.w	r1, [r1, lr]
	f889 1000 	strb.w	r1, [r9]
	f817 1c2a 	ldrb.w	r1, [r7, #-42]
	f89d 402c 	ldrb.w	r4, [sp, #44]	@ 0x2c
	ea4f 4e01 	mov.w	lr, r1, lsl #16
	9901      	ldr	r1, [sp, #4]
	fa2b f901 	lsr.w	r9, fp, r1
	ea48 010e 	orr.w	r1, r8, lr
	4321      	orrs	r1, r4
	9c00      	ldr	r4, [sp, #0]
	f004 0418 	and.w	r4, r4, #24
	40a1      	lsls	r1, r4
	ea41 0109 	orr.w	r1, r1, r9
	6019      	str	r1, [r3, #0]
	e640      	b.n	<compiler_builtins::mem::memmove+0x248>
	e9dd 2103 	ldrd	r2, r1, [sp, #12]
	4411      	add	r1, r2
	3908      	subs	r1, #8
	9107      	str	r1, [sp, #28]
	46a3      	mov	fp, r4
	f8dd 9004 	ldr.w	r9, [sp, #4]
	9c02      	ldr	r4, [sp, #8]
	a90c      	add	r1, sp, #48	@ 0x30
	2600      	movs	r6, #0
	eb01 0c04 	add.w	ip, r1, r4
	9907      	ldr	r1, [sp, #28]
	960c      	str	r6, [sp, #48]	@ 0x30
	4421      	add	r1, r4
	1f0a      	subs	r2, r1, #4
	f1c4 0104 	rsb	r1, r4, #4
	07cc      	lsls	r4, r1, #31
	bf1e      	ittt	ne
	7814      	ldrbne	r4, [r2, #0]
	f88c 4000 	strbne.w	r4, [ip]
	2601      	movne	r6, #1
	0789      	lsls	r1, r1, #30
	bf44      	itt	mi
	5b91      	ldrhmi	r1, [r2, r6]
	f82c 1006 	strhmi.w	r1, [ip, r6]
	990c      	ldr	r1, [sp, #48]	@ 0x30
	9a06      	ldr	r2, [sp, #24]
	40d1      	lsrs	r1, r2
	f009 0218 	and.w	r2, r9, #24
	fa0b f202 	lsl.w	r2, fp, r2
	4311      	orrs	r1, r2
	f845 1c04 	str.w	r1, [r5, #-4]
	e5a4      	b.n	<compiler_builtins::mem::memmove+0x162>

<__aeabi_memmove8>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	e8bd 4080 	ldmia.w	sp!, {r7, lr}
	f7ff bcee 	b.w	<compiler_builtins::mem::memmove>

<__aeabi_memclr4>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	2904      	cmp	r1, #4
	d305      	bcc.n	<__aeabi_memclr4+0x14>
	1f0b      	subs	r3, r1, #4
	43da      	mvns	r2, r3
	f012 0f0c 	tst.w	r2, #12
	d102      	bne.n	<__aeabi_memclr4+0x18>
	e01a      	b.n	<__aeabi_memclr4+0x4a>
	4602      	mov	r2, r0
	e024      	b.n	<__aeabi_memclr4+0x62>
	f04f 0c00 	mov.w	ip, #0
	4602      	mov	r2, r0
	f842 cb04 	str.w	ip, [r2], #4
	f013 0f0c 	tst.w	r3, #12
	d008      	beq.n	<__aeabi_memclr4+0x3a>
	f003 020c 	and.w	r2, r3, #12
	f8c0 c004 	str.w	ip, [r0, #4]
	2a04      	cmp	r2, #4
	d105      	bne.n	<__aeabi_memclr4+0x40>
	3908      	subs	r1, #8
	3008      	adds	r0, #8
	e006      	b.n	<__aeabi_memclr4+0x48>
	4619      	mov	r1, r3
	4610      	mov	r0, r2
	e004      	b.n	<__aeabi_memclr4+0x4a>
	2200      	movs	r2, #0
	390c      	subs	r1, #12
	6082      	str	r2, [r0, #8]
	300c      	adds	r0, #12
	4602      	mov	r2, r0
	2b0c      	cmp	r3, #12
	d309      	bcc.n	<__aeabi_memclr4+0x62>
	2300      	movs	r3, #0
	4602      	mov	r2, r0
	3910      	subs	r1, #16
	e9c2 3300 	strd	r3, r3, [r2]
	e9c2 3302 	strd	r3, r3, [r2, #8]
	3210      	adds	r2, #16
	2903      	cmp	r1, #3
	d8f7      	bhi.n	<__aeabi_memclr4+0x52>
	1850      	adds	r0, r2, r1
	4282      	cmp	r2, r0
	d221      	bcs.n	<__aeabi_memclr4+0xac>
	f1a1 0c01 	sub.w	ip, r1, #1
	f011 0303 	ands.w	r3, r1, #3
	d00c      	beq.n	<__aeabi_memclr4+0x8c>
	f04f 0e00 	mov.w	lr, #0
	4611      	mov	r1, r2
	f801 eb01 	strb.w	lr, [r1], #1
	2b01      	cmp	r3, #1
	d00a      	beq.n	<__aeabi_memclr4+0x96>
	2b02      	cmp	r3, #2
	f882 e001 	strb.w	lr, [r2, #1]
	d103      	bne.n	<__aeabi_memclr4+0x90>
	1c91      	adds	r1, r2, #2
	e004      	b.n	<__aeabi_memclr4+0x96>
	4611      	mov	r1, r2
	e002      	b.n	<__aeabi_memclr4+0x96>
	2100      	movs	r1, #0
	7091      	strb	r1, [r2, #2]
	1cd1      	adds	r1, r2, #3
	f1bc 0f03 	cmp.w	ip, #3
	bf38      	it	cc
	bd80      	popcc	{r7, pc}
	3904      	subs	r1, #4
	2200      	movs	r2, #0
	f841 2f04 	str.w	r2, [r1, #4]!
	1d0b      	adds	r3, r1, #4
	4283      	cmp	r3, r0
	d1fa      	bne.n	<__aeabi_memclr4+0xa2>
	bd80      	pop	{r7, pc}

<__aeabi_memcpy>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	e8bd 4080 	ldmia.w	sp!, {r7, lr}
	f7ff bb1e 	b.w	<compiler_builtins::mem::memcpy>

<__aeabi_memcpy8>:
	2a04      	cmp	r2, #4
	d309      	bcc.n	<__aeabi_memcpy8+0x18>
	b5d0      	push	{r4, r6, r7, lr}
	af02      	add	r7, sp, #8
	f1a2 0e04 	sub.w	lr, r2, #4
	ea6f 030e 	mvn.w	r3, lr
	f013 0f0c 	tst.w	r3, #12
	d106      	bne.n	<__aeabi_memcpy8+0x24>
	e023      	b.n	<__aeabi_memcpy8+0x60>
	460b      	mov	r3, r1
	4684      	mov	ip, r0
	4660      	mov	r0, ip
	4619      	mov	r1, r3
	f7ff bb0c 	b.w	<compiler_builtins::mem::memcpy>
	460b      	mov	r3, r1
	4684      	mov	ip, r0
	f853 4b04 	ldr.w	r4, [r3], #4
	f01e 0f0c 	tst.w	lr, #12
	f84c 4b04 	str.w	r4, [ip], #4
	d009      	beq.n	<__aeabi_memcpy8+0x4a>
	684b      	ldr	r3, [r1, #4]
	6043      	str	r3, [r0, #4]
	f00e 030c 	and.w	r3, lr, #12
	2b04      	cmp	r3, #4
	d107      	bne.n	<__aeabi_memcpy8+0x52>
	3a08      	subs	r2, #8
	3108      	adds	r1, #8
	3008      	adds	r0, #8
	e008      	b.n	<__aeabi_memcpy8+0x5c>
	4672      	mov	r2, lr
	4660      	mov	r0, ip
	4619      	mov	r1, r3
	e006      	b.n	<__aeabi_memcpy8+0x60>
	688b      	ldr	r3, [r1, #8]
	3a0c      	subs	r2, #12
	6083      	str	r3, [r0, #8]
	310c      	adds	r1, #12
	300c      	adds	r0, #12
	4684      	mov	ip, r0
	460b      	mov	r3, r1
	f1be 0f0c 	cmp.w	lr, #12
	d314      	bcc.n	<__aeabi_memcpy8+0x90>
	4684      	mov	ip, r0
	460b      	mov	r3, r1
	6818      	ldr	r0, [r3, #0]
	3a10      	subs	r2, #16
	f8cc 0000 	str.w	r0, [ip]
	2a03      	cmp	r2, #3
	6858      	ldr	r0, [r3, #4]
	f8cc 0004 	str.w	r0, [ip, #4]
	6898      	ldr	r0, [r3, #8]
	f8cc 0008 	str.w	r0, [ip, #8]
	68d8      	ldr	r0, [r3, #12]
	f103 0310 	add.w	r3, r3, #16
	f8cc 000c 	str.w	r0, [ip, #12]
	f10c 0c10 	add.w	ip, ip, #16
	d8ec      	bhi.n	<__aeabi_memcpy8+0x6a>
	e8bd 40d0 	ldmia.w	sp!, {r4, r6, r7, lr}
	4660      	mov	r0, ip
	4619      	mov	r1, r3
	f7ff bad0 	b.w	<compiler_builtins::mem::memcpy>

<__aeabi_memclr>:
	b5b0      	push	{r4, r5, r7, lr}
	af02      	add	r7, sp, #8
	2910      	cmp	r1, #16
	d33f      	bcc.n	<__aeabi_memclr+0x88>
	4242      	negs	r2, r0
	f002 0503 	and.w	r5, r2, #3
	1942      	adds	r2, r0, r5
	4290      	cmp	r0, r2
	d220      	bcs.n	<__aeabi_memclr+0x56>
	f1a5 0c01 	sub.w	ip, r5, #1
	b18d      	cbz	r5, <__aeabi_memclr+0x3e>
	2300      	movs	r3, #0
	4686      	mov	lr, r0
	f80e 3b01 	strb.w	r3, [lr], #1
	2d01      	cmp	r5, #1
	d007      	beq.n	<__aeabi_memclr+0x36>
	7043      	strb	r3, [r0, #1]
	2d02      	cmp	r5, #2
	bf1a      	itte	ne
	7083      	strbne	r3, [r0, #2]
	f100 0e03 	addne.w	lr, r0, #3
	f100 0e02 	addeq.w	lr, r0, #2
	f1bc 0f03 	cmp.w	ip, #3
	d204      	bcs.n	<__aeabi_memclr+0x46>
	e00b      	b.n	<__aeabi_memclr+0x56>
	4686      	mov	lr, r0
	f1bc 0f03 	cmp.w	ip, #3
	d307      	bcc.n	<__aeabi_memclr+0x56>
	f1ae 0004 	sub.w	r0, lr, #4
	2300      	movs	r3, #0
	f840 3f04 	str.w	r3, [r0, #4]!
	1d04      	adds	r4, r0, #4
	4294      	cmp	r4, r2
	d1fa      	bne.n	<__aeabi_memclr+0x4c>
	1b49      	subs	r1, r1, r5
	f021 0003 	bic.w	r0, r1, #3
	4410      	add	r0, r2
	4282      	cmp	r2, r0
	d210      	bcs.n	<__aeabi_memclr+0x84>
	2300      	movs	r3, #0
	f842 3b04 	str.w	r3, [r2], #4
	4282      	cmp	r2, r0
	d20b      	bcs.n	<__aeabi_memclr+0x84>
	f842 3b04 	str.w	r3, [r2], #4
	4282      	cmp	r2, r0
	bf3c      	itt	cc
	f842 3b04 	strcc.w	r3, [r2], #4
	4282      	cmpcc	r2, r0
	d203      	bcs.n	<__aeabi_memclr+0x84>
	f842 3b04 	str.w	r3, [r2], #4
	4282      	cmp	r2, r0
	d3ef      	bcc.n	<__aeabi_memclr+0x64>
	f001 0103 	and.w	r1, r1, #3
	1842      	adds	r2, r0, r1
	4290      	cmp	r0, r2
	d21f      	bcs.n	<__aeabi_memclr+0xce>
	1e4b      	subs	r3, r1, #1
	f011 0403 	ands.w	r4, r1, #3
	d00c      	beq.n	<__aeabi_memclr+0xb0>
	f04f 0c00 	mov.w	ip, #0
	4601      	mov	r1, r0
	f801 cb01 	strb.w	ip, [r1], #1
	2c01      	cmp	r4, #1
	d00a      	beq.n	<__aeabi_memclr+0xba>
	2c02      	cmp	r4, #2
	f880 c001 	strb.w	ip, [r0, #1]
	d103      	bne.n	<__aeabi_memclr+0xb4>
	1c81      	adds	r1, r0, #2
	e004      	b.n	<__aeabi_memclr+0xba>
	4601      	mov	r1, r0
	e002      	b.n	<__aeabi_memclr+0xba>
	2100      	movs	r1, #0
	7081      	strb	r1, [r0, #2]
	1cc1      	adds	r1, r0, #3
	2b03      	cmp	r3, #3
	bf38      	it	cc
	bdb0      	popcc	{r4, r5, r7, pc}
	1f08      	subs	r0, r1, #4
	2100      	movs	r1, #0
	f840 1f04 	str.w	r1, [r0, #4]!
	1d03      	adds	r3, r0, #4
	4293      	cmp	r3, r2
	d1fa      	bne.n	<__aeabi_memclr+0xc4>
	bdb0      	pop	{r4, r5, r7, pc}

<__aeabi_uldivmod>:
	b510      	push	{r4, lr}
	b084      	sub	sp, #16
	ac02      	add	r4, sp, #8
	9400      	str	r4, [sp, #0]
	f000 f804 	bl	<__udivmoddi4>
	9a02      	ldr	r2, [sp, #8]
	9b03      	ldr	r3, [sp, #12]
	b004      	add	sp, #16
	bd10      	pop	{r4, pc}

<__udivmoddi4>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	b086      	sub	sp, #24
	4684      	mov	ip, r0
	a802      	add	r0, sp, #8
	e9cd 2300 	strd	r2, r3, [sp]
	4662      	mov	r2, ip
	460b      	mov	r3, r1
	f000 f80d 	bl	<compiler_builtins::int::specialized_div_rem::u64_div_rem>
	f8d7 c008 	ldr.w	ip, [r7, #8]
	e9dd 0102 	ldrd	r0, r1, [sp, #8]
	f1bc 0f00 	cmp.w	ip, #0
	bf1c      	itt	ne
	e9dd 3204 	ldrdne	r3, r2, [sp, #16]
	e9cc 3200 	strdne	r3, r2, [ip]
	b006      	add	sp, #24
	bd80      	pop	{r7, pc}

<compiler_builtins::int::specialized_div_rem::u64_div_rem>:
	b5f0      	push	{r4, r5, r6, r7, lr}
	af03      	add	r7, sp, #12
	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
	e9d7 e802 	ldrd	lr, r8, [r7, #8]
	f1be 0f00 	cmp.w	lr, #0
	d072      	beq.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0xf8>
	f1b8 0f00 	cmp.w	r8, #0
	d16f      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0xf8>
	2b00      	cmp	r3, #0
	f000 80fc 	beq.w	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x216>
	4573      	cmp	r3, lr
	f080 8107 	bcs.w	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x232>
	fab3 f183 	clz	r1, r3
	fabe f68e 	clz	r6, lr
	1a71      	subs	r1, r6, r1
	f101 0620 	add.w	r6, r1, #32
	bf08      	it	eq
	261f      	moveq	r6, #31
	f1c6 0520 	rsb	r5, r6, #32
	fa08 f106 	lsl.w	r1, r8, r6
	fa0e f806 	lsl.w	r8, lr, r6
	fa2e f505 	lsr.w	r5, lr, r5
	4329      	orrs	r1, r5
	f1b6 0520 	subs.w	r5, r6, #32
	f006 061f 	and.w	r6, r6, #31
	bf58      	it	pl
	fa0e f105 	lslpl.w	r1, lr, r5
	f04f 0501 	mov.w	r5, #1
	fa05 fc06 	lsl.w	ip, r5, r6
	f04f 0500 	mov.w	r5, #0
	bf58      	it	pl
	f04f 0800 	movpl.w	r8, #0
	e008      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x7c>
	4622      	mov	r2, r4
	4633      	mov	r3, r6
	ea4f 1418 	mov.w	r4, r8, lsr #4
	ea44 7801 	orr.w	r8, r4, r1, lsl #28
	ea4f 1c1c 	mov.w	ip, ip, lsr #4
	0909      	lsrs	r1, r1, #4
	ebb2 0408 	subs.w	r4, r2, r8
	eb63 0601 	sbc.w	r6, r3, r1
	2e00      	cmp	r6, #0
	d403      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x90>
	ea45 050c 	orr.w	r5, r5, ip
	d102      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x94>
	e02d      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0xec>
	4614      	mov	r4, r2
	461e      	mov	r6, r3
	ea5f 0351 	movs.w	r3, r1, lsr #1
	ea4f 0238 	mov.w	r2, r8, rrx
	1aa2      	subs	r2, r4, r2
	eb66 0303 	sbc.w	r3, r6, r3
	2b00      	cmp	r3, #0
	d404      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0xb0>
	ea45 055c 	orr.w	r5, r5, ip, lsr #1
	4614      	mov	r4, r2
	d102      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0xb4>
	e01d      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0xec>
	4622      	mov	r2, r4
	4633      	mov	r3, r6
	ea4f 0498 	mov.w	r4, r8, lsr #2
	ea44 7481 	orr.w	r4, r4, r1, lsl #30
	1b14      	subs	r4, r2, r4
	eb63 0691 	sbc.w	r6, r3, r1, lsr #2
	2e00      	cmp	r6, #0
	d403      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0xce>
	ea45 059c 	orr.w	r5, r5, ip, lsr #2
	d102      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0xd2>
	e00e      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0xec>
	4614      	mov	r4, r2
	461e      	mov	r6, r3
	ea4f 02d8 	mov.w	r2, r8, lsr #3
	ea42 7241 	orr.w	r2, r2, r1, lsl #29
	1aa2      	subs	r2, r4, r2
	eb66 03d1 	sbc.w	r3, r6, r1, lsr #3
	2b00      	cmp	r3, #0
	d4c2      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x6a>
	ea45 05dc 	orr.w	r5, r5, ip, lsr #3
	4614      	mov	r4, r2
	d1c0      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x6e>
	fbb4 f1fe 	udiv	r1, r4, lr
	fb01 461e 	mls	r6, r1, lr, r4
	4329      	orrs	r1, r5
	e092      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x21e>
	ebb2 060e 	subs.w	r6, r2, lr
	f04f 0100 	mov.w	r1, #0
	eb73 0608 	sbcs.w	r6, r3, r8
	d37c      	bcc.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x200>
	2b00      	cmp	r3, #0
	d07a      	beq.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x200>
	fab3 f183 	clz	r1, r3
	fab8 f688 	clz	r6, r8
	1a71      	subs	r1, r6, r1
	f001 063f 	and.w	r6, r1, #63	@ 0x3f
	f001 011f 	and.w	r1, r1, #31
	f1c6 0420 	rsb	r4, r6, #32
	fa08 f506 	lsl.w	r5, r8, r6
	fa0e fc06 	lsl.w	ip, lr, r6
	fa2e f404 	lsr.w	r4, lr, r4
	432c      	orrs	r4, r5
	f1b6 0520 	subs.w	r5, r6, #32
	bf58      	it	pl
	fa0e f405 	lslpl.w	r4, lr, r5
	f04f 0501 	mov.w	r5, #1
	fa05 f901 	lsl.w	r9, r5, r1
	f04f 0100 	mov.w	r1, #0
	bf58      	it	pl
	f04f 0c00 	movpl.w	ip, #0
	e008      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x15e>
	4632      	mov	r2, r6
	462b      	mov	r3, r5
	ea4f 161c 	mov.w	r6, ip, lsr #4
	ea46 7c04 	orr.w	ip, r6, r4, lsl #28
	ea4f 1919 	mov.w	r9, r9, lsr #4
	0924      	lsrs	r4, r4, #4
	ebb2 060c 	subs.w	r6, r2, ip
	eb73 0504 	sbcs.w	r5, r3, r4
	d407      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x178>
	ea41 0109 	orr.w	r1, r1, r9
	ebb6 020e 	subs.w	r2, r6, lr
	eb75 0208 	sbcs.w	r2, r5, r8
	d202      	bcs.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x17c>
	e03a      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ee>
	4616      	mov	r6, r2
	461d      	mov	r5, r3
	ea5f 0354 	movs.w	r3, r4, lsr #1
	ea4f 023c 	mov.w	r2, ip, rrx
	1ab2      	subs	r2, r6, r2
	eb75 0303 	sbcs.w	r3, r5, r3
	d409      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1a0>
	ebb2 050e 	subs.w	r5, r2, lr
	ea41 0159 	orr.w	r1, r1, r9, lsr #1
	eb73 0508 	sbcs.w	r5, r3, r8
	4616      	mov	r6, r2
	461d      	mov	r5, r3
	d202      	bcs.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1a4>
	e026      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ee>
	4632      	mov	r2, r6
	462b      	mov	r3, r5
	ea4f 059c 	mov.w	r5, ip, lsr #2
	ea45 7584 	orr.w	r5, r5, r4, lsl #30
	1b56      	subs	r6, r2, r5
	eb63 0594 	sbc.w	r5, r3, r4, lsr #2
	2d00      	cmp	r5, #0
	d407      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1c6>
	ea41 0199 	orr.w	r1, r1, r9, lsr #2
	ebb6 020e 	subs.w	r2, r6, lr
	eb75 0208 	sbcs.w	r2, r5, r8
	d202      	bcs.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ca>
	e013      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ee>
	4616      	mov	r6, r2
	461d      	mov	r5, r3
	ea4f 02dc 	mov.w	r2, ip, lsr #3
	ea42 7244 	orr.w	r2, r2, r4, lsl #29
	1ab2      	subs	r2, r6, r2
	eb65 03d4 	sbc.w	r3, r5, r4, lsr #3
	2b00      	cmp	r3, #0
	d4b7      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x14c>
	ebb2 050e 	subs.w	r5, r2, lr
	ea41 01d9 	orr.w	r1, r1, r9, lsr #3
	eb73 0508 	sbcs.w	r5, r3, r8
	4616      	mov	r6, r2
	461d      	mov	r5, r3
	d2b0      	bcs.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x150>
	f04f 0c00 	mov.w	ip, #0
	e9c0 1c00 	strd	r1, ip, [r0]
	e9c0 6502 	strd	r6, r5, [r0, #8]
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	f04f 0c00 	mov.w	ip, #0
	4616      	mov	r6, r2
	461d      	mov	r5, r3
	e9c0 1c00 	strd	r1, ip, [r0]
	e9c0 6502 	strd	r6, r5, [r0, #8]
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	fbb2 f1fe 	udiv	r1, r2, lr
	fb01 261e 	mls	r6, r1, lr, r2
	f04f 0c00 	mov.w	ip, #0
	2500      	movs	r5, #0
	e9c0 1c00 	strd	r1, ip, [r0]
	e9c0 6502 	strd	r6, r5, [r0, #8]
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	d10d      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x250>
	fbb2 f1f3 	udiv	r1, r2, r3
	2500      	movs	r5, #0
	fb01 2613 	mls	r6, r1, r3, r2
	f04f 0c01 	mov.w	ip, #1
	e9c0 1c00 	strd	r1, ip, [r0]
	e9c0 6502 	strd	r6, r5, [r0, #8]
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	fbb3 fcfe 	udiv	ip, r3, lr
	f5be 3f80 	cmp.w	lr, #65536	@ 0x10000
	fb0c 351e 	mls	r5, ip, lr, r3
	d21a      	bcs.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x294>
	0429      	lsls	r1, r5, #16
	2500      	movs	r5, #0
	ea41 4112 	orr.w	r1, r1, r2, lsr #16
	fbb1 f3fe 	udiv	r3, r1, lr
	fb03 f10e 	mul.w	r1, r3, lr
	ea4c 4c13 	orr.w	ip, ip, r3, lsr #16
	ebc1 4112 	rsb	r1, r1, r2, lsr #16
	eac2 4101 	pkhbt	r1, r2, r1, lsl #16
	fbb1 f2fe 	udiv	r2, r1, lr
	fb02 161e 	mls	r6, r2, lr, r1
	ea42 4103 	orr.w	r1, r2, r3, lsl #16
	e9c0 1c00 	strd	r1, ip, [r0]
	e9c0 6502 	strd	r6, r5, [r0, #8]
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	ebb2 010e 	subs.w	r1, r2, lr
	eb75 0108 	sbcs.w	r1, r5, r8
	d208      	bcs.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2b0>
	2100      	movs	r1, #0
	4616      	mov	r6, r2
	e9c0 1c00 	strd	r1, ip, [r0]
	e9c0 6502 	strd	r6, r5, [r0, #8]
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}
	ea4f 71c8 	mov.w	r1, r8, lsl #31
	ea41 035e 	orr.w	r3, r1, lr, lsr #1
	ea4f 79ce 	mov.w	r9, lr, lsl #31
	f04f 4800 	mov.w	r8, #2147483648	@ 0x80000000
	2100      	movs	r1, #0
	e008      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2d6>
	4622      	mov	r2, r4
	4635      	mov	r5, r6
	ea4f 1419 	mov.w	r4, r9, lsr #4
	ea44 7903 	orr.w	r9, r4, r3, lsl #28
	ea4f 1818 	mov.w	r8, r8, lsr #4
	091b      	lsrs	r3, r3, #4
	ebb2 0409 	subs.w	r4, r2, r9
	eb65 0603 	sbc.w	r6, r5, r3
	2e00      	cmp	r6, #0
	d403      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2ea>
	ea41 0108 	orr.w	r1, r1, r8
	d102      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2ee>
	e02d      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x346>
	4614      	mov	r4, r2
	462e      	mov	r6, r5
	ea5f 0553 	movs.w	r5, r3, lsr #1
	ea4f 0239 	mov.w	r2, r9, rrx
	1aa2      	subs	r2, r4, r2
	eb66 0505 	sbc.w	r5, r6, r5
	2d00      	cmp	r5, #0
	d404      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x30a>
	ea41 0158 	orr.w	r1, r1, r8, lsr #1
	4614      	mov	r4, r2
	d102      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x30e>
	e01d      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x346>
	4622      	mov	r2, r4
	4635      	mov	r5, r6
	ea4f 0499 	mov.w	r4, r9, lsr #2
	ea44 7483 	orr.w	r4, r4, r3, lsl #30
	1b14      	subs	r4, r2, r4
	eb65 0693 	sbc.w	r6, r5, r3, lsr #2
	2e00      	cmp	r6, #0
	d403      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x328>
	ea41 0198 	orr.w	r1, r1, r8, lsr #2
	d102      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x32c>
	e00e      	b.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x346>
	4614      	mov	r4, r2
	462e      	mov	r6, r5
	ea4f 02d9 	mov.w	r2, r9, lsr #3
	ea42 7243 	orr.w	r2, r2, r3, lsl #29
	1aa2      	subs	r2, r4, r2
	eb66 05d3 	sbc.w	r5, r6, r3, lsr #3
	2d00      	cmp	r5, #0
	d4c2      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2c4>
	ea41 01d8 	orr.w	r1, r1, r8, lsr #3
	4614      	mov	r4, r2
	d1c0      	bne.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2c8>
	fbb4 f2fe 	udiv	r2, r4, lr
	2500      	movs	r5, #0
	fb02 461e 	mls	r6, r2, lr, r4
	4311      	orrs	r1, r2
	e9c0 1c00 	strd	r1, ip, [r0]
	e9c0 6502 	strd	r6, r5, [r0, #8]
	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
	bdf0      	pop	{r4, r5, r6, r7, pc}

<HardFaultTrampoline>:
	4670      	mov	r0, lr
	2104      	movs	r1, #4
	4208      	tst	r0, r1
	d103      	bne.n	<HardFaultTrampoline+0x10>
	f3ef 8008 	mrs	r0, MSP
	f000 b804 	b.w	<HardFault_>
	f3ef 8009 	mrs	r0, PSP
	f000 b800 	b.w	<HardFault_>

<HardFault_>:
	b580      	push	{r7, lr}
	466f      	mov	r7, sp
	e7fe      	b.n	<HardFault_+0x4>
	d4d4      	bmi.n	<compiler_builtins::int::specialized_div_rem::u64_div_rem+0x32a>
