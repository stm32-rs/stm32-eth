
./broken:     file format elf32-littlearm
./broken
architecture: armv7e-m, flags 0x00000112:
EXEC_P, HAS_SYMS, D_PAGED
start address 0x080001f9

Program Header:
    LOAD off    0x00010000 vaddr 0x08000000 paddr 0x08000000 align 2**16
         filesz 0x000001f8 memsz 0x000001f8 flags r--
    LOAD off    0x000101f8 vaddr 0x080001f8 paddr 0x080001f8 align 2**16
         filesz 0x0000973c memsz 0x0000973c flags r-x
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
  1 .text         0000973c  080001f8  080001f8  000101f8  2**2
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
  8 .debug_loc    00024dc2  00000000  00000000  0003007a  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
  9 .debug_abbrev 000015ad  00000000  00000000  00054e3c  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 10 .debug_info   0006faf7  00000000  00000000  000563e9  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 11 .debug_aranges 00001600  00000000  00000000  000c5ee0  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 12 .debug_ranges 0000ab60  00000000  00000000  000c74e0  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 13 .debug_str    0006ae3a  00000000  00000000  000d2040  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 14 .comment      0000008b  00000000  00000000  0013ce7a  2**0
                  CONTENTS, READONLY
 15 .ARM.attributes 00000038  00000000  00000000  0013cf05  2**0
                  CONTENTS, READONLY
 16 .debug_frame  00000eac  00000000  00000000  0013cf40  2**2
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 17 .debug_line   00015a25  00000000  00000000  0013ddec  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 18 .debug_pubnames 000002be  00000000  00000000  00153811  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 19 .debug_pubtypes 00000047  00000000  00000000  00153acf  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
SYMBOL TABLE:
00000000 l    df *ABS*	00000000 ip.db5b21dddbb36540-cgu.3
08009914 l     F .text	00000018 HardFaultTrampoline
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
08008422 l     F .text	0000013e smoltcp::iface::route::Routes::lookup
0800b00c l     O .rodata	0000002c .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.7
0800b038 l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.8
08003924 l     F .text	0000000c core::panicking::panic
08008156 l     F .text	000002cc smoltcp::wire::ip::checksum::data
08007c94 l     F .text	0000017e smoltcp::wire::ipv4::Repr::emit
0800aae0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.41
080062c0 l     F .text	00000224 smoltcp::wire::tcp::TcpOption::emit
0800ab70 l     O .rodata	00000024 .Lanon.ee555cc9a30366a9abf098ddf788bd84.55
0800ab94 l     O .rodata	00000024 .Lanon.ee555cc9a30366a9abf098ddf788bd84.56
08004454 l     F .text	00000022 defmt::export::acquire_and_header
080043b0 l     F .text	000000a4 defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format
0800a518 l     O .rodata	00000036 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.22
0800a550 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.24
0800b0b8 l     O .rodata	0000001d .Lanon.ed157fc5cbdd1a6edf19cdf9648d5188.21
0800b0d8 l     O .rodata	00000010 .Lanon.ed157fc5cbdd1a6edf19cdf9648d5188.22
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
0800b098 l     O .rodata	00000010 .Lanon.ed157fc5cbdd1a6edf19cdf9648d5188.17
0800606a l     F .text	00000086 core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>
0800aa0c l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.19
080037ea l     F .text	00000014 core::option::unwrap_failed
0800addc l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.8
0800ae3c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.19
0800ae5c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.26
0800ae1c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.16
0800b058 l     O .rodata	00000010 .Lanon.ed157fc5cbdd1a6edf19cdf9648d5188.5
0800acf0 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.92
0800111c l     F .text	000023e4 smoltcp::iface::interface::Interface::poll
0800b0a8 l     O .rodata	00000010 .Lanon.ed157fc5cbdd1a6edf19cdf9648d5188.19
08008560 l     F .text	00000372 smoltcp::iface::neighbor::Cache::fill
08007aae l     F .text	000001b6 smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply
08007f46 l     F .text	00000092 smoltcp::socket::udp::Socket::accepts
08007fd8 l     F .text	0000017e smoltcp::socket::udp::Socket::process
0800623c l     F .text	00000084 <core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold
080065a4 l     F .text	0000121e smoltcp::socket::tcp::Socket::process
080077c2 l     F .text	00000096 smoltcp::socket::tcp::Socket::rst_reply
080064e4 l     F .text	000000c0 smoltcp::socket::tcp::Socket::seq_to_transmit
0800ac78 l     O .rodata	00000038 .Lanon.ee555cc9a30366a9abf098ddf788bd84.70
0800acb0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.71
080038e0 l     F .text	0000001c core::panicking::panic_fmt
0800b0f8 l     O .rodata	00000041 .Lanon.e5fdbc5d2fbccefff69a0640183fd610.2
0800b13c l     O .rodata	00000010 .Lanon.e5fdbc5d2fbccefff69a0640183fd610.4
08004152 l     F .text	00000026 core::option::expect_failed
0800b14c l     O .rodata	00000042 .Lanon.e5fdbc5d2fbccefff69a0640183fd610.5
0800b190 l     O .rodata	00000010 .Lanon.e5fdbc5d2fbccefff69a0640183fd610.6
0800b068 l     O .rodata	00000010 .Lanon.ed157fc5cbdd1a6edf19cdf9648d5188.7
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
0800b078 l     O .rodata	00000010 .Lanon.ed157fc5cbdd1a6edf19cdf9648d5188.14
0800ae2c l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.17
0800aebc l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.12
080041b6 l     F .text	00000014 core::panicking::panic_const::panic_const_rem_by_zero
0800b088 l     O .rodata	00000010 .Lanon.ed157fc5cbdd1a6edf19cdf9648d5188.16
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
080060f0 l     F .text	00000088 __rustc::rust_begin_unwind
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
08004a9c l     F .text	000015b4 ip::__cortex_m_rt_main
0800b1d8 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.6
0800b0e8 l     O .rodata	00000010 .Lanon.ed157fc5cbdd1a6edf19cdf9648d5188.24
0800a900 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.7
0800a93c l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.9
08007c64 l     F .text	0000002e <smoltcp::wire::ip::Address as core::fmt::Display>::fmt
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
08006050 l     F .text	0000001a <stm32_eth::mac::WrongClock as core::fmt::Debug>::fmt
0800a94c l     O .rodata	0000000a .Lanon.02e056d4c889a085f771c01290e74e2f.10
20000038 l     O .bss	00000001 panic_probe::imp::panic::PANICKED
0800aa1c l     O .rodata	00000010 .Lanon.6fd57bafcc6bbd1a0f0c6a4a6d894bad.0
0800a744 l     O .rodata	00000001 .Lanon.ea5d5760b73fa5b90415fc99172c09b2.2
08006178 l     F .text	00000018 panic_probe::hard_fault
08006190 l     F .text	000000ac <&T as core::fmt::Display>::fmt
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
08007e76 l     F .text	00000066 defmt::export::fmt
08007e48 l     F .text	0000002e defmt::export::fmt
0800ad60 l     O .rodata	00000001 .Lanon.c1140054cd359bfdcd454e5b7dad9548.32
08007e12 l     F .text	00000036 defmt::export::fmt
08007a60 l     F .text	0000004e smoltcp::socket::tcp::Socket::immediate_ack_to_transmit
08007858 l     F .text	00000208 smoltcp::socket::tcp::Socket::ack_reply
0800ad50 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.31
0800ad9c l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.36
0800aecc l     O .rodata	00000025 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.14
0800aef4 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.15
0800aeac l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.11
0800ad61 l     O .rodata	00000028 .Lanon.c1140054cd359bfdcd454e5b7dad9548.33
0800ad8c l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.35
0800b048 l     O .rodata	00000010 .Lanon.d4b94c958ea1f8000f4d53dc8b85c559.8
08007edc l     F .text	0000006a smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with
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
080088f6 l     F .text	000000ae .hidden __aeabi_memclr8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.250
080089a4 l     F .text	0000009c .hidden __aeabi_memcpy4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.012
08008a40 l     F .text	000002e6 .hidden compiler_builtins::mem::memcpy
08008d26 l     F .text	00000618 .hidden compiler_builtins::mem::memmove
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.275
0800933e l     F .text	0000000c .hidden __aeabi_memmove8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.279
0800934a l     F .text	000000ae .hidden __aeabi_memclr4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.288
080093f8 l     F .text	0000000c .hidden __aeabi_memcpy
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.320
08009404 l     F .text	0000009c .hidden __aeabi_memcpy8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.373
080094a0 l     F .text	000000d0 .hidden __aeabi_memclr
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.413
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.362
08009584 l     F .text	00000030 .hidden __udivmoddi4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.017
080095b4 l     F .text	00000360 .hidden compiler_builtins::int::specialized_div_rem::u64_div_rem
08000004 g     O .vector_table	00000004 __RESET_VECTOR
080001f8 g     F .text	0000003e Reset
08000008 g     O .vector_table	00000038 __EXCEPTIONS
080043a4 g     F .text	00000000 DefaultHandler
08000040 g     O .vector_table	000001b8 __INTERRUPTS
0800462a g     F .text	0000009e _defmt_acquire
080046c8 g     F .text	000000f0 _defmt_release
080043aa g     F .text	00000006 __defmt_default_timestamp
0800992c g     F .text	00000000 HardFault
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
08009570  w    F .text	00000014 __aeabi_uldivmod
080088ec g     F .text	00000006 __primask_r
080088d2 g     F .text	00000004 __cpsid
080088d6 g     F .text	00000004 __cpsie
080043a4 g     F .text	00000006 DefaultHandler_
080043aa g     F .text	00000006 DefaultPreInit
0800992c g     F .text	00000006 HardFault_
00000032 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_write","data":"{}.{}.{}.{}","disambiguator":"14742771906021295210","crate_name":"defmt"}
00000002 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=u8}","disambiguator":"16912398427198947700","crate_name":"defmt"}
080043aa g     F .text	00000000 _defmt_timestamp
20000008 g     O .data	00000030 _SEGGER_RTT
08004954 g     F .text	0000010e ETH
08004a62 g     F .text	00000032 SysTick
080088da g     F .text	0000000c __delay
080088e6 g     F .text	00000006 __dsb
0000002b g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Enabling ethernet...","disambiguator":"10404236363425019797","crate_name":"ip"}
0000002c g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Listening at {}:80...","disambiguator":"3686426055769507929","crate_name":"ip"}
0000002d g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Transmitted hello! Closing socket...","disambiguator":"11635372039424199218","crate_name":"ip"}
0000002e g     O .defmt	00000001 {"package":"panic-probe","tag":"defmt_error","data":"{}","disambiguator":"3281125884690968605","crate_name":"panic_probe"}
00000001 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=__internal_Display}","disambiguator":"14158116662542268607","crate_name":"defmt"}
080088f2 g     F .text	00000004 __udf
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
08009934 g       .text	00000000 __etext
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

080001f8 <Reset>:
 80001f8:	f004 f8d7 	bl	80043aa <DefaultPreInit>
 80001fc:	480e      	ldr	r0, [pc, #56]	@ (8000238 <Reset+0x40>)
 80001fe:	490f      	ldr	r1, [pc, #60]	@ (800023c <Reset+0x44>)
 8000200:	2200      	movs	r2, #0
 8000202:	4281      	cmp	r1, r0
 8000204:	d001      	beq.n	800020a <Reset+0x12>
 8000206:	c004      	stmia	r0!, {r2}
 8000208:	e7fb      	b.n	8000202 <Reset+0xa>
 800020a:	480d      	ldr	r0, [pc, #52]	@ (8000240 <Reset+0x48>)
 800020c:	490d      	ldr	r1, [pc, #52]	@ (8000244 <Reset+0x4c>)
 800020e:	4a0e      	ldr	r2, [pc, #56]	@ (8000248 <Reset+0x50>)
 8000210:	4281      	cmp	r1, r0
 8000212:	d002      	beq.n	800021a <Reset+0x22>
 8000214:	ca08      	ldmia	r2!, {r3}
 8000216:	c008      	stmia	r0!, {r3}
 8000218:	e7fa      	b.n	8000210 <Reset+0x18>
 800021a:	480c      	ldr	r0, [pc, #48]	@ (800024c <Reset+0x54>)
 800021c:	f44f 0170 	mov.w	r1, #15728640	@ 0xf00000
 8000220:	6802      	ldr	r2, [r0, #0]
 8000222:	ea42 0201 	orr.w	r2, r2, r1
 8000226:	6002      	str	r2, [r0, #0]
 8000228:	f3bf 8f4f 	dsb	sy
 800022c:	f3bf 8f6f 	isb	sy
 8000230:	f004 fc30 	bl	8004a94 <main>
 8000234:	de00      	udf	#0
 8000236:	0000      	movs	r0, r0
 8000238:	20000038 	.word	0x20000038
 800023c:	20000074 	.word	0x20000074
 8000240:	20000000 	.word	0x20000000
 8000244:	20000038 	.word	0x20000038
 8000248:	0800b250 	.word	0x0800b250
 800024c:	e000ed88 	.word	0xe000ed88

08000250 <smoltcp::wire::arp::Repr::emit>:
 8000250:	b5b0      	push	{r4, r5, r7, lr}
 8000252:	af02      	add	r7, sp, #8
 8000254:	2a01      	cmp	r2, #1
 8000256:	d949      	bls.n	80002ec <smoltcp::wire::arp::Repr::emit+0x9c>
 8000258:	f8d0 e00a 	ldr.w	lr, [r0, #10]
 800025c:	f44f 7580 	mov.w	r5, #256	@ 0x100
 8000260:	f8d0 c014 	ldr.w	ip, [r0, #20]
 8000264:	2a03      	cmp	r2, #3
 8000266:	8843      	ldrh	r3, [r0, #2]
 8000268:	8804      	ldrh	r4, [r0, #0]
 800026a:	800d      	strh	r5, [r1, #0]
 800026c:	d946      	bls.n	80002fc <smoltcp::wire::arp::Repr::emit+0xac>
 800026e:	2508      	movs	r5, #8
 8000270:	2a04      	cmp	r2, #4
 8000272:	804d      	strh	r5, [r1, #2]
 8000274:	d062      	beq.n	800033c <smoltcp::wire::arp::Repr::emit+0xec>
 8000276:	2506      	movs	r5, #6
 8000278:	2a05      	cmp	r2, #5
 800027a:	710d      	strb	r5, [r1, #4]
 800027c:	d966      	bls.n	800034c <smoltcp::wire::arp::Repr::emit+0xfc>
 800027e:	2504      	movs	r5, #4
 8000280:	2a07      	cmp	r2, #7
 8000282:	714d      	strb	r5, [r1, #5]
 8000284:	d942      	bls.n	800030c <smoltcp::wire::arp::Repr::emit+0xbc>
 8000286:	b324      	cbz	r4, 80002d2 <smoltcp::wire::arp::Repr::emit+0x82>
 8000288:	2c01      	cmp	r4, #1
 800028a:	bf08      	it	eq
 800028c:	2302      	moveq	r3, #2
 800028e:	ba5b      	rev16	r3, r3
 8000290:	2a0d      	cmp	r2, #13
 8000292:	80cb      	strh	r3, [r1, #6]
 8000294:	d922      	bls.n	80002dc <smoltcp::wire::arp::Repr::emit+0x8c>
 8000296:	1d03      	adds	r3, r0, #4
 8000298:	2a11      	cmp	r2, #17
 800029a:	681c      	ldr	r4, [r3, #0]
 800029c:	889b      	ldrh	r3, [r3, #4]
 800029e:	818b      	strh	r3, [r1, #12]
 80002a0:	608c      	str	r4, [r1, #8]
 80002a2:	d93b      	bls.n	800031c <smoltcp::wire::arp::Repr::emit+0xcc>
 80002a4:	2a17      	cmp	r2, #23
 80002a6:	f8c1 e00e 	str.w	lr, [r1, #14]
 80002aa:	d93f      	bls.n	800032c <smoltcp::wire::arp::Repr::emit+0xdc>
 80002ac:	300e      	adds	r0, #14
 80002ae:	2a1b      	cmp	r2, #27
 80002b0:	6803      	ldr	r3, [r0, #0]
 80002b2:	8880      	ldrh	r0, [r0, #4]
 80002b4:	82c8      	strh	r0, [r1, #22]
 80002b6:	f8c1 3012 	str.w	r3, [r1, #18]
 80002ba:	bf84      	itt	hi
 80002bc:	f8c1 c018 	strhi.w	ip, [r1, #24]
 80002c0:	bdb0      	pophi	{r4, r5, r7, pc}
 80002c2:	f649 1398 	movw	r3, #39320	@ 0x9998
 80002c6:	2018      	movs	r0, #24
 80002c8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80002cc:	211c      	movs	r1, #28
 80002ce:	f003 fa96 	bl	80037fe <core::slice::index::slice_index_fail>
 80002d2:	2301      	movs	r3, #1
 80002d4:	ba5b      	rev16	r3, r3
 80002d6:	2a0d      	cmp	r2, #13
 80002d8:	80cb      	strh	r3, [r1, #6]
 80002da:	d8dc      	bhi.n	8000296 <smoltcp::wire::arp::Repr::emit+0x46>
 80002dc:	f649 1368 	movw	r3, #39272	@ 0x9968
 80002e0:	2008      	movs	r0, #8
 80002e2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80002e6:	210e      	movs	r1, #14
 80002e8:	f003 fa89 	bl	80037fe <core::slice::index::slice_index_fail>
 80002ec:	f649 1348 	movw	r3, #39240	@ 0x9948
 80002f0:	2000      	movs	r0, #0
 80002f2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80002f6:	2102      	movs	r1, #2
 80002f8:	f003 fa81 	bl	80037fe <core::slice::index::slice_index_fail>
 80002fc:	f649 1358 	movw	r3, #39256	@ 0x9958
 8000300:	2002      	movs	r0, #2
 8000302:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000306:	2104      	movs	r1, #4
 8000308:	f003 fa79 	bl	80037fe <core::slice::index::slice_index_fail>
 800030c:	f649 1338 	movw	r3, #39224	@ 0x9938
 8000310:	2006      	movs	r0, #6
 8000312:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000316:	2108      	movs	r1, #8
 8000318:	f003 fa71 	bl	80037fe <core::slice::index::slice_index_fail>
 800031c:	f649 1378 	movw	r3, #39288	@ 0x9978
 8000320:	200e      	movs	r0, #14
 8000322:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000326:	2112      	movs	r1, #18
 8000328:	f003 fa69 	bl	80037fe <core::slice::index::slice_index_fail>
 800032c:	f649 1388 	movw	r3, #39304	@ 0x9988
 8000330:	2012      	movs	r0, #18
 8000332:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000336:	2118      	movs	r1, #24
 8000338:	f003 fa61 	bl	80037fe <core::slice::index::slice_index_fail>
 800033c:	f649 12a8 	movw	r2, #39336	@ 0x99a8
 8000340:	2004      	movs	r0, #4
 8000342:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000346:	2104      	movs	r1, #4
 8000348:	f003 fad8 	bl	80038fc <core::panicking::panic_bounds_check>
 800034c:	f649 12b8 	movw	r2, #39352	@ 0x99b8
 8000350:	2005      	movs	r0, #5
 8000352:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000356:	2105      	movs	r1, #5
 8000358:	f003 fad0 	bl	80038fc <core::panicking::panic_bounds_check>

0800035c <smoltcp::iface::interface::InterfaceInner::dispatch_ip>:
 800035c:	b5f0      	push	{r4, r5, r6, r7, lr}
 800035e:	af03      	add	r7, sp, #12
 8000360:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8000364:	b0b1      	sub	sp, #196	@ 0xc4
 8000366:	f8d3 8058 	ldr.w	r8, [r3, #88]	@ 0x58
 800036a:	f1b8 0f00 	cmp.w	r8, #0
 800036e:	f000 856e 	beq.w	8000e4e <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xaf2>
 8000372:	920b      	str	r2, [sp, #44]	@ 0x2c
 8000374:	f118 0601 	adds.w	r6, r8, #1
 8000378:	6d5a      	ldr	r2, [r3, #84]	@ 0x54
 800037a:	920f      	str	r2, [sp, #60]	@ 0x3c
 800037c:	6dda      	ldr	r2, [r3, #92]	@ 0x5c
 800037e:	9211      	str	r2, [sp, #68]	@ 0x44
 8000380:	f893 2062 	ldrb.w	r2, [r3, #98]	@ 0x62
 8000384:	9208      	str	r2, [sp, #32]
 8000386:	f893 2061 	ldrb.w	r2, [r3, #97]	@ 0x61
 800038a:	920d      	str	r2, [sp, #52]	@ 0x34
 800038c:	f893 2060 	ldrb.w	r2, [r3, #96]	@ 0x60
 8000390:	920c      	str	r2, [sp, #48]	@ 0x30
 8000392:	6802      	ldr	r2, [r0, #0]
 8000394:	68c4      	ldr	r4, [r0, #12]
 8000396:	920a      	str	r2, [sp, #40]	@ 0x28
 8000398:	6842      	ldr	r2, [r0, #4]
 800039a:	9209      	str	r2, [sp, #36]	@ 0x24
 800039c:	6882      	ldr	r2, [r0, #8]
 800039e:	920e      	str	r2, [sp, #56]	@ 0x38
 80003a0:	d044      	beq.n	800042c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd0>
 80003a2:	f8d0 90f0 	ldr.w	r9, [r0, #240]	@ 0xf0
 80003a6:	9410      	str	r4, [sp, #64]	@ 0x40
 80003a8:	f1b9 0f00 	cmp.w	r9, #0
 80003ac:	eb09 0a89 	add.w	sl, r9, r9, lsl #2
 80003b0:	d028      	beq.n	8000404 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xa8>
 80003b2:	eb00 060a 	add.w	r6, r0, sl
 80003b6:	f100 05f4 	add.w	r5, r0, #244	@ 0xf4
 80003ba:	f106 0ef4 	add.w	lr, r6, #244	@ 0xf4
 80003be:	f04f 3cff 	mov.w	ip, #4294967295	@ 0xffffffff
 80003c2:	e00e      	b.n	80003e2 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x86>
 80003c4:	2600      	movs	r6, #0
 80003c6:	f004 041f 	and.w	r4, r4, #31
 80003ca:	ea06 060b 	and.w	r6, r6, fp
 80003ce:	fa2c f404 	lsr.w	r4, ip, r4
 80003d2:	ba24      	rev	r4, r4
 80003d4:	4334      	orrs	r4, r6
 80003d6:	45a0      	cmp	r8, r4
 80003d8:	f000 80e9 	beq.w	80005ae <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x252>
 80003dc:	3505      	adds	r5, #5
 80003de:	4575      	cmp	r5, lr
 80003e0:	d010      	beq.n	8000404 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xa8>
 80003e2:	792c      	ldrb	r4, [r5, #4]
 80003e4:	f8d5 b000 	ldr.w	fp, [r5]
 80003e8:	2c00      	cmp	r4, #0
 80003ea:	d0eb      	beq.n	80003c4 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x68>
 80003ec:	f1a4 061f 	sub.w	r6, r4, #31
 80003f0:	b2f6      	uxtb	r6, r6
 80003f2:	2e02      	cmp	r6, #2
 80003f4:	d3f2      	bcc.n	80003dc <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x80>
 80003f6:	4266      	negs	r6, r4
 80003f8:	f006 061f 	and.w	r6, r6, #31
 80003fc:	fa0c f606 	lsl.w	r6, ip, r6
 8000400:	ba36      	rev	r6, r6
 8000402:	e7e0      	b.n	80003c6 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x6a>
 8000404:	f008 06f0 	and.w	r6, r8, #240	@ 0xf0
 8000408:	2ee0      	cmp	r6, #224	@ 0xe0
 800040a:	d12d      	bne.n	8000468 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x10c>
 800040c:	f3c8 2a06 	ubfx	sl, r8, #8, #7
 8000410:	ea4f 6c18 	mov.w	ip, r8, lsr #24
 8000414:	ea4f 4b18 	mov.w	fp, r8, lsr #16
 8000418:	f04f 0901 	mov.w	r9, #1
 800041c:	2500      	movs	r5, #0
 800041e:	265e      	movs	r6, #94	@ 0x5e
 8000420:	9c10      	ldr	r4, [sp, #64]	@ 0x40
 8000422:	680a      	ldr	r2, [r1, #0]
 8000424:	2a02      	cmp	r2, #2
 8000426:	f000 80d1 	beq.w	80005cc <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x270>
 800042a:	e00d      	b.n	8000448 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xec>
 800042c:	f04f 0cff 	mov.w	ip, #255	@ 0xff
 8000430:	f04f 0bff 	mov.w	fp, #255	@ 0xff
 8000434:	f04f 0aff 	mov.w	sl, #255	@ 0xff
 8000438:	26ff      	movs	r6, #255	@ 0xff
 800043a:	25ff      	movs	r5, #255	@ 0xff
 800043c:	f04f 09ff 	mov.w	r9, #255	@ 0xff
 8000440:	680a      	ldr	r2, [r1, #0]
 8000442:	2a02      	cmp	r2, #2
 8000444:	f000 80c2 	beq.w	80005cc <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x270>
 8000448:	9607      	str	r6, [sp, #28]
 800044a:	f1a4 020e 	sub.w	r2, r4, #14
 800044e:	9e11      	ldr	r6, [sp, #68]	@ 0x44
 8000450:	f106 0e14 	add.w	lr, r6, #20
 8000454:	4596      	cmp	lr, r2
 8000456:	d927      	bls.n	80004a8 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x14c>
 8000458:	f240 0022 	movw	r0, #34	@ 0x22
 800045c:	f2c0 0000 	movt	r0, #0
 8000460:	f004 f809 	bl	8004476 <defmt::export::acquire_header_and_release>
 8000464:	f000 bc0c 	b.w	8000c80 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x924>
 8000468:	e9d0 2b04 	ldrd	r2, fp, [r0, #16]
 800046c:	f100 04f4 	add.w	r4, r0, #244	@ 0xf4
 8000470:	f04f 3cff 	mov.w	ip, #4294967295	@ 0xffffffff
 8000474:	9207      	str	r2, [sp, #28]
 8000476:	e00e      	b.n	8000496 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x13a>
 8000478:	4276      	negs	r6, r6
 800047a:	f006 061f 	and.w	r6, r6, #31
 800047e:	fa0c f606 	lsl.w	r6, ip, r6
 8000482:	ba36      	rev	r6, r6
 8000484:	6825      	ldr	r5, [r4, #0]
 8000486:	3405      	adds	r4, #5
 8000488:	f1aa 0a05 	sub.w	sl, sl, #5
 800048c:	ea85 0508 	eor.w	r5, r5, r8
 8000490:	422e      	tst	r6, r5
 8000492:	f000 80b8 	beq.w	8000606 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x2aa>
 8000496:	f1ba 0f00 	cmp.w	sl, #0
 800049a:	f000 809c 	beq.w	80005d6 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x27a>
 800049e:	7926      	ldrb	r6, [r4, #4]
 80004a0:	2e00      	cmp	r6, #0
 80004a2:	d1e9      	bne.n	8000478 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x11c>
 80004a4:	2600      	movs	r6, #0
 80004a6:	e7ed      	b.n	8000484 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x128>
 80004a8:	9505      	str	r5, [sp, #20]
 80004aa:	688d      	ldr	r5, [r1, #8]
 80004ac:	e9d5 1601 	ldrd	r1, r6, [r5, #4]
 80004b0:	428e      	cmp	r6, r1
 80004b2:	f080 85da 	bcs.w	800106a <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd0e>
 80004b6:	f44f 62c6 	mov.w	r2, #1584	@ 0x630
 80004ba:	e9cd ba02 	strd	fp, sl, [sp, #8]
 80004be:	4372      	muls	r2, r6
 80004c0:	f8d5 a000 	ldr.w	sl, [r5]
 80004c4:	f8cd c010 	str.w	ip, [sp, #16]
 80004c8:	9410      	str	r4, [sp, #64]	@ 0x40
 80004ca:	f85a 2002 	ldr.w	r2, [sl, r2]
 80004ce:	2a00      	cmp	r2, #0
 80004d0:	f100 85d2 	bmi.w	8001078 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd1c>
 80004d4:	9a11      	ldr	r2, [sp, #68]	@ 0x44
 80004d6:	9606      	str	r6, [sp, #24]
 80004d8:	f102 0b22 	add.w	fp, r2, #34	@ 0x22
 80004dc:	1c72      	adds	r2, r6, #1
 80004de:	fbb2 fcf1 	udiv	ip, r2, r1
 80004e2:	fb0c 2111 	mls	r1, ip, r1, r2
 80004e6:	60a9      	str	r1, [r5, #8]
 80004e8:	f240 51f3 	movw	r1, #1523	@ 0x5f3
 80004ec:	458b      	cmp	fp, r1
 80004ee:	f080 84b9 	bcs.w	8000e64 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb08>
 80004f2:	46dc      	mov	ip, fp
 80004f4:	f1bb 0f0b 	cmp.w	fp, #11
 80004f8:	f240 84bf 	bls.w	8000e7a <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb1e>
 80004fc:	9e06      	ldr	r6, [sp, #24]
 80004fe:	f44f 62c6 	mov.w	r2, #1584	@ 0x630
 8000502:	f500 71b4 	add.w	r1, r0, #360	@ 0x168
 8000506:	46ab      	mov	fp, r5
 8000508:	f1bc 0f0d 	cmp.w	ip, #13
 800050c:	fb06 a202 	mla	r2, r6, r2, sl
 8000510:	f102 0538 	add.w	r5, r2, #56	@ 0x38
 8000514:	680a      	ldr	r2, [r1, #0]
 8000516:	8889      	ldrh	r1, [r1, #4]
 8000518:	8169      	strh	r1, [r5, #10]
 800051a:	9904      	ldr	r1, [sp, #16]
 800051c:	7169      	strb	r1, [r5, #5]
 800051e:	9902      	ldr	r1, [sp, #8]
 8000520:	7129      	strb	r1, [r5, #4]
 8000522:	9903      	ldr	r1, [sp, #12]
 8000524:	70e9      	strb	r1, [r5, #3]
 8000526:	9907      	ldr	r1, [sp, #28]
 8000528:	70a9      	strb	r1, [r5, #2]
 800052a:	9905      	ldr	r1, [sp, #20]
 800052c:	f8c5 2006 	str.w	r2, [r5, #6]
 8000530:	7069      	strb	r1, [r5, #1]
 8000532:	f885 9000 	strb.w	r9, [r5]
 8000536:	f240 84a9 	bls.w	8000e8c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb30>
 800053a:	f8dd 9044 	ldr.w	r9, [sp, #68]	@ 0x44
 800053e:	2108      	movs	r1, #8
 8000540:	9c0d      	ldr	r4, [sp, #52]	@ 0x34
 8000542:	f1be 0f00 	cmp.w	lr, #0
 8000546:	81a9      	strh	r1, [r5, #12]
 8000548:	f000 85a1 	beq.w	800108e <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd32>
 800054c:	7a00      	ldrb	r0, [r0, #8]
 800054e:	f105 010e 	add.w	r1, r5, #14
 8000552:	2245      	movs	r2, #69	@ 0x45
 8000554:	f1be 0f01 	cmp.w	lr, #1
 8000558:	700a      	strb	r2, [r1, #0]
 800055a:	f000 85a0 	beq.w	800109e <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd42>
 800055e:	2200      	movs	r2, #0
 8000560:	f1be 0f03 	cmp.w	lr, #3
 8000564:	73ea      	strb	r2, [r5, #15]
 8000566:	f240 849a 	bls.w	8000e9e <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb42>
 800056a:	fa9e f28e 	rev.w	r2, lr
 800056e:	f1be 0f05 	cmp.w	lr, #5
 8000572:	ea4f 4212 	mov.w	r2, r2, lsr #16
 8000576:	822a      	strh	r2, [r5, #16]
 8000578:	f240 849a 	bls.w	8000eb0 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb54>
 800057c:	2200      	movs	r2, #0
 800057e:	f1be 0f07 	cmp.w	lr, #7
 8000582:	826a      	strh	r2, [r5, #18]
 8000584:	f240 849d 	bls.w	8000ec2 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb66>
 8000588:	2240      	movs	r2, #64	@ 0x40
 800058a:	f1be 0f08 	cmp.w	lr, #8
 800058e:	82aa      	strh	r2, [r5, #20]
 8000590:	f000 858d 	beq.w	80010ae <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd52>
 8000594:	9a0c      	ldr	r2, [sp, #48]	@ 0x30
 8000596:	75aa      	strb	r2, [r5, #22]
 8000598:	e8df f004 	tbb	[pc, r4]
 800059c:	07979797 	.word	0x07979797
 80005a0:	8e929088 	.word	0x8e929088
 80005a4:	86948c96 	.word	0x86948c96
 80005a8:	008a      	.short	0x008a
 80005aa:	2406      	movs	r4, #6
 80005ac:	e08d      	b.n	80006ca <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
 80005ae:	f04f 0cff 	mov.w	ip, #255	@ 0xff
 80005b2:	f04f 0bff 	mov.w	fp, #255	@ 0xff
 80005b6:	f04f 0aff 	mov.w	sl, #255	@ 0xff
 80005ba:	26ff      	movs	r6, #255	@ 0xff
 80005bc:	25ff      	movs	r5, #255	@ 0xff
 80005be:	f04f 09ff 	mov.w	r9, #255	@ 0xff
 80005c2:	9c10      	ldr	r4, [sp, #64]	@ 0x40
 80005c4:	680a      	ldr	r2, [r1, #0]
 80005c6:	2a02      	cmp	r2, #2
 80005c8:	f47f af3e 	bne.w	8000448 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xec>
 80005cc:	4648      	mov	r0, r9
 80005ce:	b031      	add	sp, #196	@ 0xc4
 80005d0:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80005d4:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80005d6:	f500 7680 	add.w	r6, r0, #256	@ 0x100
 80005da:	4682      	mov	sl, r0
 80005dc:	a813      	add	r0, sp, #76	@ 0x4c
 80005de:	e9cd 2b00 	strd	r2, fp, [sp]
 80005e2:	460d      	mov	r5, r1
 80005e4:	4631      	mov	r1, r6
 80005e6:	4642      	mov	r2, r8
 80005e8:	461c      	mov	r4, r3
 80005ea:	f007 ff1a 	bl	8008422 <smoltcp::iface::route::Routes::lookup>
 80005ee:	f89d 004c 	ldrb.w	r0, [sp, #76]	@ 0x4c
 80005f2:	4629      	mov	r1, r5
 80005f4:	4623      	mov	r3, r4
 80005f6:	b948      	cbnz	r0, 800060c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x2b0>
 80005f8:	f04f 0900 	mov.w	r9, #0
 80005fc:	4648      	mov	r0, r9
 80005fe:	b031      	add	sp, #196	@ 0xc4
 8000600:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8000604:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8000606:	4682      	mov	sl, r0
 8000608:	f8cd 804d 	str.w	r8, [sp, #77]	@ 0x4d
 800060c:	f89d 0050 	ldrb.w	r0, [sp, #80]	@ 0x50
 8000610:	f8bd 604e 	ldrh.w	r6, [sp, #78]	@ 0x4e
 8000614:	f89d e04d 	ldrb.w	lr, [sp, #77]	@ 0x4d
 8000618:	ea46 4200 	orr.w	r2, r6, r0, lsl #16
 800061c:	ea4e 2602 	orr.w	r6, lr, r2, lsl #8
 8000620:	1c70      	adds	r0, r6, #1
 8000622:	d006      	beq.n	8000632 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x2d6>
 8000624:	f00e 00f0 	and.w	r0, lr, #240	@ 0xf0
 8000628:	28e0      	cmp	r0, #224	@ 0xe0
 800062a:	bf18      	it	ne
 800062c:	ea52 000e 	orrsne.w	r0, r2, lr
 8000630:	d10a      	bne.n	8000648 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x2ec>
 8000632:	f24b 000c 	movw	r0, #45068	@ 0xb00c
 8000636:	f24b 0238 	movw	r2, #45112	@ 0xb038
 800063a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800063e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000642:	212c      	movs	r1, #44	@ 0x2c
 8000644:	f003 f96e 	bl	8003924 <core::panicking::panic>
 8000648:	f8da 40e0 	ldr.w	r4, [sl, #224]	@ 0xe0
 800064c:	4650      	mov	r0, sl
 800064e:	e9cd 2b05 	strd	r2, fp, [sp, #20]
 8000652:	eba4 0484 	sub.w	r4, r4, r4, lsl #2
 8000656:	ea4f 0ac4 	mov.w	sl, r4, lsl #3
 800065a:	2400      	movs	r4, #0
 800065c:	eb1a 0b04 	adds.w	fp, sl, r4
 8000660:	f000 832f 	beq.w	8000cc2 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
 8000664:	1905      	adds	r5, r0, r4
 8000666:	f8d5 c020 	ldr.w	ip, [r5, #32]
 800066a:	45b4      	cmp	ip, r6
 800066c:	f000 82d2 	beq.w	8000c14 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8b8>
 8000670:	f11b 0218 	adds.w	r2, fp, #24
 8000674:	f000 8325 	beq.w	8000cc2 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
 8000678:	6baa      	ldr	r2, [r5, #56]	@ 0x38
 800067a:	42b2      	cmp	r2, r6
 800067c:	f000 82cc 	beq.w	8000c18 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8bc>
 8000680:	f11b 0230 	adds.w	r2, fp, #48	@ 0x30
 8000684:	f000 831d 	beq.w	8000cc2 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
 8000688:	6d2a      	ldr	r2, [r5, #80]	@ 0x50
 800068a:	42b2      	cmp	r2, r6
 800068c:	f000 82ff 	beq.w	8000c8e <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x932>
 8000690:	f11b 0248 	adds.w	r2, fp, #72	@ 0x48
 8000694:	f000 8315 	beq.w	8000cc2 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
 8000698:	6eaa      	ldr	r2, [r5, #104]	@ 0x68
 800069a:	3460      	adds	r4, #96	@ 0x60
 800069c:	42b2      	cmp	r2, r6
 800069e:	d1dd      	bne.n	800065c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x300>
 80006a0:	1902      	adds	r2, r0, r4
 80006a2:	f102 0508 	add.w	r5, r2, #8
 80006a6:	e2f3      	b.n	8000c90 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x934>
 80006a8:	243c      	movs	r4, #60	@ 0x3c
 80006aa:	e00e      	b.n	80006ca <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
 80006ac:	2411      	movs	r4, #17
 80006ae:	e00c      	b.n	80006ca <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
 80006b0:	9c08      	ldr	r4, [sp, #32]
 80006b2:	e00a      	b.n	80006ca <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
 80006b4:	243a      	movs	r4, #58	@ 0x3a
 80006b6:	e008      	b.n	80006ca <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
 80006b8:	2432      	movs	r4, #50	@ 0x32
 80006ba:	e006      	b.n	80006ca <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
 80006bc:	242b      	movs	r4, #43	@ 0x2b
 80006be:	e004      	b.n	80006ca <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
 80006c0:	242c      	movs	r4, #44	@ 0x2c
 80006c2:	e002      	b.n	80006ca <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
 80006c4:	243b      	movs	r4, #59	@ 0x3b
 80006c6:	e000      	b.n	80006ca <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x36e>
 80006c8:	2433      	movs	r4, #51	@ 0x33
 80006ca:	f1be 0f09 	cmp.w	lr, #9
 80006ce:	f240 84f6 	bls.w	80010be <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd62>
 80006d2:	f1be 0f0f 	cmp.w	lr, #15
 80006d6:	75ec      	strb	r4, [r5, #23]
 80006d8:	f240 83fc 	bls.w	8000ed4 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb78>
 80006dc:	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
 80006de:	f119 0f14 	cmn.w	r9, #20
 80006e2:	f8c5 201a 	str.w	r2, [r5, #26]
 80006e6:	f080 83fe 	bcs.w	8000ee6 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb8a>
 80006ea:	f010 0ffd 	tst.w	r0, #253	@ 0xfd
 80006ee:	f8cd c034 	str.w	ip, [sp, #52]	@ 0x34
 80006f2:	f8c5 801e 	str.w	r8, [r5, #30]
 80006f6:	d10b      	bne.n	8000710 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x3b4>
 80006f8:	2000      	movs	r0, #0
 80006fa:	461c      	mov	r4, r3
 80006fc:	8328      	strh	r0, [r5, #24]
 80006fe:	4608      	mov	r0, r1
 8000700:	2114      	movs	r1, #20
 8000702:	f007 fd28 	bl	8008156 <smoltcp::wire::ip::checksum::data>
 8000706:	43c0      	mvns	r0, r0
 8000708:	4623      	mov	r3, r4
 800070a:	ba00      	rev	r0, r0
 800070c:	0c00      	lsrs	r0, r0, #16
 800070e:	e000      	b.n	8000712 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x3b6>
 8000710:	2000      	movs	r0, #0
 8000712:	8328      	strh	r0, [r5, #24]
 8000714:	f105 0a22 	add.w	sl, r5, #34	@ 0x22
 8000718:	6818      	ldr	r0, [r3, #0]
 800071a:	3802      	subs	r0, #2
 800071c:	bf38      	it	cc
 800071e:	2002      	movcc	r0, #2
 8000720:	2800      	cmp	r0, #0
 8000722:	d071      	beq.n	8000808 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x4ac>
 8000724:	2801      	cmp	r0, #1
 8000726:	f040 8081 	bne.w	800082c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x4d0>
 800072a:	f1b9 0f01 	cmp.w	r9, #1
 800072e:	f240 83ec 	bls.w	8000f0a <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbae>
 8000732:	e9d3 1201 	ldrd	r1, r2, [r3, #4]
 8000736:	f1b9 0f03 	cmp.w	r9, #3
 800073a:	89d8      	ldrh	r0, [r3, #14]
 800073c:	899b      	ldrh	r3, [r3, #12]
 800073e:	ba1b      	rev	r3, r3
 8000740:	ea4f 4313 	mov.w	r3, r3, lsr #16
 8000744:	f8aa 3000 	strh.w	r3, [sl]
 8000748:	f240 83f1 	bls.w	8000f2e <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbd2>
 800074c:	ba40      	rev16	r0, r0
 800074e:	f1b9 0f05 	cmp.w	r9, #5
 8000752:	84a8      	strh	r0, [r5, #36]	@ 0x24
 8000754:	f240 83fd 	bls.w	8000f52 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbf6>
 8000758:	f102 0008 	add.w	r0, r2, #8
 800075c:	ba03      	rev	r3, r0
 800075e:	b286      	uxth	r6, r0
 8000760:	b290      	uxth	r0, r2
 8000762:	0c1b      	lsrs	r3, r3, #16
 8000764:	84eb      	strh	r3, [r5, #38]	@ 0x26
 8000766:	f64f 73f7 	movw	r3, #65527	@ 0xfff7
 800076a:	4298      	cmp	r0, r3
 800076c:	f200 83c4 	bhi.w	8000ef8 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb9c>
 8000770:	45b1      	cmp	r9, r6
 8000772:	f0c0 83c1 	bcc.w	8000ef8 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xb9c>
 8000776:	f1a6 0008 	sub.w	r0, r6, #8
 800077a:	4290      	cmp	r0, r2
 800077c:	f040 840d 	bne.w	8000f9a <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc3e>
 8000780:	980e      	ldr	r0, [sp, #56]	@ 0x38
 8000782:	0a04      	lsrs	r4, r0, #8
 8000784:	f105 002a 	add.w	r0, r5, #42	@ 0x2a
 8000788:	f008 fe36 	bl	80093f8 <__aeabi_memcpy>
 800078c:	f014 0ffd 	tst.w	r4, #253	@ 0xfd
 8000790:	f040 80f5 	bne.w	800097e <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x622>
 8000794:	f1b9 0f07 	cmp.w	r9, #7
 8000798:	f240 8407 	bls.w	8000faa <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc4e>
 800079c:	2000      	movs	r0, #0
 800079e:	f64f 79ff 	movw	r9, #65535	@ 0xffff
 80007a2:	8528      	strh	r0, [r5, #40]	@ 0x28
 80007a4:	fa98 f088 	rev.w	r0, r8
 80007a8:	0c01      	lsrs	r1, r0, #16
 80007aa:	fa11 f080 	uxtah	r0, r1, r0
 80007ae:	0c01      	lsrs	r1, r0, #16
 80007b0:	fa11 f080 	uxtah	r0, r1, r0
 80007b4:	990f      	ldr	r1, [sp, #60]	@ 0x3c
 80007b6:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 80007ba:	ba09      	rev	r1, r1
 80007bc:	b280      	uxth	r0, r0
 80007be:	0c0a      	lsrs	r2, r1, #16
 80007c0:	fa12 f181 	uxtah	r1, r2, r1
 80007c4:	0c0a      	lsrs	r2, r1, #16
 80007c6:	fa12 f181 	uxtah	r1, r2, r1
 80007ca:	eb01 4111 	add.w	r1, r1, r1, lsr #16
 80007ce:	fa10 f081 	uxtah	r0, r0, r1
 80007d2:	4430      	add	r0, r6
 80007d4:	3011      	adds	r0, #17
 80007d6:	0c01      	lsrs	r1, r0, #16
 80007d8:	fa11 f080 	uxtah	r0, r1, r0
 80007dc:	4631      	mov	r1, r6
 80007de:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 80007e2:	b284      	uxth	r4, r0
 80007e4:	4650      	mov	r0, sl
 80007e6:	f007 fcb6 	bl	8008156 <smoltcp::wire::ip::checksum::data>
 80007ea:	fa14 f080 	uxtah	r0, r4, r0
 80007ee:	0c01      	lsrs	r1, r0, #16
 80007f0:	fa11 f080 	uxtah	r0, r1, r0
 80007f4:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 80007f8:	43c1      	mvns	r1, r0
 80007fa:	b280      	uxth	r0, r0
 80007fc:	ba09      	rev	r1, r1
 80007fe:	4548      	cmp	r0, r9
 8000800:	bf18      	it	ne
 8000802:	0c08      	lsrne	r0, r1, #16
 8000804:	8528      	strh	r0, [r5, #40]	@ 0x28
 8000806:	e20b      	b.n	8000c20 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c4>
 8000808:	f1b9 0f02 	cmp.w	r9, #2
 800080c:	f0c0 8465 	bcc.w	80010da <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd7e>
 8000810:	980e      	ldr	r0, [sp, #56]	@ 0x38
 8000812:	ea4f 6810 	mov.w	r8, r0, lsr #24
 8000816:	7918      	ldrb	r0, [r3, #4]
 8000818:	e8df f010 	tbh	[pc, r0, lsl #1]
 800081c:	01070004 	.word	0x01070004
 8000820:	00df00b8 	.word	0x00df00b8
 8000824:	e9d3 1003 	ldrd	r1, r0, [r3, #12]
 8000828:	2608      	movs	r6, #8
 800082a:	e101      	b.n	8000a30 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x6d4>
 800082c:	a813      	add	r0, sp, #76	@ 0x4c
 800082e:	4619      	mov	r1, r3
 8000830:	2254      	movs	r2, #84	@ 0x54
 8000832:	f008 f8b7 	bl	80089a4 <__aeabi_memcpy4>
 8000836:	980a      	ldr	r0, [sp, #40]	@ 0x28
 8000838:	b380      	cbz	r0, 800089c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x540>
 800083a:	f8bd 0084 	ldrh.w	r0, [sp, #132]	@ 0x84
 800083e:	2318      	movs	r3, #24
 8000840:	f89d 2094 	ldrb.w	r2, [sp, #148]	@ 0x94
 8000844:	f89d 109c 	ldrb.w	r1, [sp, #156]	@ 0x9c
 8000848:	2800      	cmp	r0, #0
 800084a:	f8dd c054 	ldr.w	ip, [sp, #84]	@ 0x54
 800084e:	f8dd e060 	ldr.w	lr, [sp, #96]	@ 0x60
 8000852:	9e1b      	ldr	r6, [sp, #108]	@ 0x6c
 8000854:	9c1e      	ldr	r4, [sp, #120]	@ 0x78
 8000856:	bf08      	it	eq
 8000858:	2314      	moveq	r3, #20
 800085a:	f362 0300 	bfi	r3, r2, #0, #1
 800085e:	f362 0341 	bfi	r3, r2, #1, #1
 8000862:	2c00      	cmp	r4, #0
 8000864:	eb03 0041 	add.w	r0, r3, r1, lsl #1
 8000868:	eb0e 010c 	add.w	r1, lr, ip
 800086c:	4431      	add	r1, r6
 800086e:	bf18      	it	ne
 8000870:	300a      	addne	r0, #10
 8000872:	2300      	movs	r3, #0
 8000874:	eb00 02c1 	add.w	r2, r0, r1, lsl #3
 8000878:	ebb3 0fc1 	cmp.w	r3, r1, lsl #3
 800087c:	bf18      	it	ne
 800087e:	1c90      	addne	r0, r2, #2
 8000880:	9910      	ldr	r1, [sp, #64]	@ 0x40
 8000882:	3003      	adds	r0, #3
 8000884:	f020 0003 	bic.w	r0, r0, #3
 8000888:	1a08      	subs	r0, r1, r0
 800088a:	9909      	ldr	r1, [sp, #36]	@ 0x24
 800088c:	3814      	subs	r0, #20
 800088e:	4348      	muls	r0, r1
 8000890:	f8bd 109a 	ldrh.w	r1, [sp, #154]	@ 0x9a
 8000894:	4288      	cmp	r0, r1
 8000896:	bf38      	it	cc
 8000898:	f8ad 009a 	strhcc.w	r0, [sp, #154]	@ 0x9a
 800089c:	f1b9 0f01 	cmp.w	r9, #1
 80008a0:	f240 833c 	bls.w	8000f1c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbc0>
 80008a4:	f8bd 0096 	ldrh.w	r0, [sp, #150]	@ 0x96
 80008a8:	f1b9 0f03 	cmp.w	r9, #3
 80008ac:	ba00      	rev	r0, r0
 80008ae:	ea4f 4010 	mov.w	r0, r0, lsr #16
 80008b2:	f8aa 0000 	strh.w	r0, [sl]
 80008b6:	f240 8343 	bls.w	8000f40 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xbe4>
 80008ba:	f8bd 0098 	ldrh.w	r0, [sp, #152]	@ 0x98
 80008be:	f1b9 0f07 	cmp.w	r9, #7
 80008c2:	ba00      	rev	r0, r0
 80008c4:	ea4f 4010 	mov.w	r0, r0, lsr #16
 80008c8:	84a8      	strh	r0, [r5, #36]	@ 0x24
 80008ca:	f240 834b 	bls.w	8000f64 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc08>
 80008ce:	9e13      	ldr	r6, [sp, #76]	@ 0x4c
 80008d0:	f1b9 0f0b 	cmp.w	r9, #11
 80008d4:	9824      	ldr	r0, [sp, #144]	@ 0x90
 80008d6:	ba00      	rev	r0, r0
 80008d8:	f8c5 0026 	str.w	r0, [r5, #38]	@ 0x26
 80008dc:	f240 834b 	bls.w	8000f76 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc1a>
 80008e0:	2e00      	cmp	r6, #0
 80008e2:	4631      	mov	r1, r6
 80008e4:	f8cd a030 	str.w	sl, [sp, #48]	@ 0x30
 80008e8:	f8cd b040 	str.w	fp, [sp, #64]	@ 0x40
 80008ec:	9814      	ldr	r0, [sp, #80]	@ 0x50
 80008ee:	bf18      	it	ne
 80008f0:	ba01      	revne	r1, r0
 80008f2:	f1b9 0f0f 	cmp.w	r9, #15
 80008f6:	f8c5 102a 	str.w	r1, [r5, #42]	@ 0x2a
 80008fa:	f240 8345 	bls.w	8000f88 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc2c>
 80008fe:	f8bd 109a 	ldrh.w	r1, [sp, #154]	@ 0x9a
 8000902:	980e      	ldr	r0, [sp, #56]	@ 0x38
 8000904:	ba09      	rev	r1, r1
 8000906:	f89d a09c 	ldrb.w	sl, [sp, #156]	@ 0x9c
 800090a:	9c15      	ldr	r4, [sp, #84]	@ 0x54
 800090c:	ea4f 4c10 	mov.w	ip, r0, lsr #16
 8000910:	f8bd 0084 	ldrh.w	r0, [sp, #132]	@ 0x84
 8000914:	0c09      	lsrs	r1, r1, #16
 8000916:	9a18      	ldr	r2, [sp, #96]	@ 0x60
 8000918:	9b1b      	ldr	r3, [sp, #108]	@ 0x6c
 800091a:	2800      	cmp	r0, #0
 800091c:	f8dd 9078 	ldr.w	r9, [sp, #120]	@ 0x78
 8000920:	f89d b094 	ldrb.w	fp, [sp, #148]	@ 0x94
 8000924:	8629      	strh	r1, [r5, #48]	@ 0x30
 8000926:	f04f 0118 	mov.w	r1, #24
 800092a:	bf08      	it	eq
 800092c:	2114      	moveq	r1, #20
 800092e:	f1b9 0f00 	cmp.w	r9, #0
 8000932:	f36b 0100 	bfi	r1, fp, #0, #1
 8000936:	f36b 0141 	bfi	r1, fp, #1, #1
 800093a:	eb01 014a 	add.w	r1, r1, sl, lsl #1
 800093e:	bf18      	it	ne
 8000940:	310a      	addne	r1, #10
 8000942:	e9cd 2409 	strd	r2, r4, [sp, #36]	@ 0x24
 8000946:	4422      	add	r2, r4
 8000948:	9308      	str	r3, [sp, #32]
 800094a:	441a      	add	r2, r3
 800094c:	2400      	movs	r4, #0
 800094e:	eb01 03c2 	add.w	r3, r1, r2, lsl #3
 8000952:	ebb4 0fc2 	cmp.w	r4, r2, lsl #3
 8000956:	bf18      	it	ne
 8000958:	1c99      	addne	r1, r3, #2
 800095a:	220c      	movs	r2, #12
 800095c:	eb02 0181 	add.w	r1, r2, r1, lsl #2
 8000960:	f001 02f0 	and.w	r2, r1, #240	@ 0xf0
 8000964:	a913      	add	r1, sp, #76	@ 0x4c
 8000966:	f101 0408 	add.w	r4, r1, #8
 800096a:	f89d 109d 	ldrb.w	r1, [sp, #157]	@ 0x9d
 800096e:	e8df f001 	tbb	[pc, r1]
 8000972:	038d      	.short	0x038d
 8000974:	00858b88 	.word	0x00858b88
 8000978:	f502 6200 	add.w	r2, r2, #2048	@ 0x800
 800097c:	e086      	b.n	8000a8c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x730>
 800097e:	f1b9 0f07 	cmp.w	r9, #7
 8000982:	f240 8312 	bls.w	8000faa <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc4e>
 8000986:	2000      	movs	r0, #0
 8000988:	8528      	strh	r0, [r5, #40]	@ 0x28
 800098a:	e149      	b.n	8000c20 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c4>
 800098c:	7959      	ldrb	r1, [r3, #5]
 800098e:	2003      	movs	r0, #3
 8000990:	f885 0022 	strb.w	r0, [r5, #34]	@ 0x22
 8000994:	e9d3 0602 	ldrd	r0, r6, [r3, #8]
 8000998:	2910      	cmp	r1, #16
 800099a:	799a      	ldrb	r2, [r3, #6]
 800099c:	9010      	str	r0, [sp, #64]	@ 0x40
 800099e:	bf38      	it	cc
 80009a0:	460a      	movcc	r2, r1
 80009a2:	f1b9 0f08 	cmp.w	r9, #8
 80009a6:	f885 2023 	strb.w	r2, [r5, #35]	@ 0x23
 80009aa:	f0c0 8310 	bcc.w	8000fce <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc72>
 80009ae:	f103 0010 	add.w	r0, r3, #16
 80009b2:	f1a9 0408 	sub.w	r4, r9, #8
 80009b6:	9b0e      	ldr	r3, [sp, #56]	@ 0x38
 80009b8:	f105 012a 	add.w	r1, r5, #42	@ 0x2a
 80009bc:	4622      	mov	r2, r4
 80009be:	f007 f969 	bl	8007c94 <smoltcp::wire::ipv4::Repr::emit>
 80009c2:	2c14      	cmp	r4, #20
 80009c4:	f0c0 832f 	bcc.w	8001026 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xcca>
 80009c8:	f1a9 021c 	sub.w	r2, r9, #28
 80009cc:	42b2      	cmp	r2, r6
 80009ce:	f040 833c 	bne.w	800104a <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xcee>
 80009d2:	f105 003e 	add.w	r0, r5, #62	@ 0x3e
 80009d6:	9910      	ldr	r1, [sp, #64]	@ 0x40
 80009d8:	e040      	b.n	8000a5c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x700>
 80009da:	7959      	ldrb	r1, [r3, #5]
 80009dc:	200b      	movs	r0, #11
 80009de:	f885 0022 	strb.w	r0, [r5, #34]	@ 0x22
 80009e2:	e9d3 0402 	ldrd	r0, r4, [r3, #8]
 80009e6:	2902      	cmp	r1, #2
 80009e8:	799a      	ldrb	r2, [r3, #6]
 80009ea:	9010      	str	r0, [sp, #64]	@ 0x40
 80009ec:	bf38      	it	cc
 80009ee:	460a      	movcc	r2, r1
 80009f0:	f1b9 0f08 	cmp.w	r9, #8
 80009f4:	f885 2023 	strb.w	r2, [r5, #35]	@ 0x23
 80009f8:	f0c0 82e9 	bcc.w	8000fce <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc72>
 80009fc:	f103 0010 	add.w	r0, r3, #16
 8000a00:	f1a9 0608 	sub.w	r6, r9, #8
 8000a04:	9b0e      	ldr	r3, [sp, #56]	@ 0x38
 8000a06:	f105 012a 	add.w	r1, r5, #42	@ 0x2a
 8000a0a:	4632      	mov	r2, r6
 8000a0c:	f007 f942 	bl	8007c94 <smoltcp::wire::ipv4::Repr::emit>
 8000a10:	2e14      	cmp	r6, #20
 8000a12:	f0c0 8311 	bcc.w	8001038 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xcdc>
 8000a16:	f1a9 001c 	sub.w	r0, r9, #28
 8000a1a:	42a0      	cmp	r0, r4
 8000a1c:	f040 831e 	bne.w	800105c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd00>
 8000a20:	f105 003e 	add.w	r0, r5, #62	@ 0x3e
 8000a24:	9910      	ldr	r1, [sp, #64]	@ 0x40
 8000a26:	4622      	mov	r2, r4
 8000a28:	e018      	b.n	8000a5c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x700>
 8000a2a:	e9d3 1003 	ldrd	r1, r0, [r3, #12]
 8000a2e:	2600      	movs	r6, #0
 8000a30:	891a      	ldrh	r2, [r3, #8]
 8000a32:	88db      	ldrh	r3, [r3, #6]
 8000a34:	f1b9 0f05 	cmp.w	r9, #5
 8000a38:	846e      	strh	r6, [r5, #34]	@ 0x22
 8000a3a:	f240 82d1 	bls.w	8000fe0 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc84>
 8000a3e:	ba5b      	rev16	r3, r3
 8000a40:	f1b9 0f07 	cmp.w	r9, #7
 8000a44:	84eb      	strh	r3, [r5, #38]	@ 0x26
 8000a46:	f240 82d4 	bls.w	8000ff2 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc96>
 8000a4a:	ba52      	rev16	r2, r2
 8000a4c:	852a      	strh	r2, [r5, #40]	@ 0x28
 8000a4e:	f1a9 0208 	sub.w	r2, r9, #8
 8000a52:	4290      	cmp	r0, r2
 8000a54:	bf38      	it	cc
 8000a56:	4602      	movcc	r2, r0
 8000a58:	f105 002a 	add.w	r0, r5, #42	@ 0x2a
 8000a5c:	f008 fccc 	bl	80093f8 <__aeabi_memcpy>
 8000a60:	2000      	movs	r0, #0
 8000a62:	f018 0ffd 	tst.w	r8, #253	@ 0xfd
 8000a66:	d107      	bne.n	8000a78 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x71c>
 8000a68:	84a8      	strh	r0, [r5, #36]	@ 0x24
 8000a6a:	4650      	mov	r0, sl
 8000a6c:	4649      	mov	r1, r9
 8000a6e:	f007 fb72 	bl	8008156 <smoltcp::wire::ip::checksum::data>
 8000a72:	43c0      	mvns	r0, r0
 8000a74:	ba00      	rev	r0, r0
 8000a76:	0c00      	lsrs	r0, r0, #16
 8000a78:	84a8      	strh	r0, [r5, #36]	@ 0x24
 8000a7a:	e0d1      	b.n	8000c20 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c4>
 8000a7c:	f502 6280 	add.w	r2, r2, #1024	@ 0x400
 8000a80:	e004      	b.n	8000a8c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x730>
 8000a82:	f502 7200 	add.w	r2, r2, #512	@ 0x200
 8000a86:	e001      	b.n	8000a8c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x730>
 8000a88:	f502 7280 	add.w	r2, r2, #256	@ 0x100
 8000a8c:	4611      	mov	r1, r2
 8000a8e:	2e00      	cmp	r6, #0
 8000a90:	bf18      	it	ne
 8000a92:	f501 5180 	addne.w	r1, r1, #4096	@ 0x1000
 8000a96:	85e9      	strh	r1, [r5, #46]	@ 0x2e
 8000a98:	213c      	movs	r1, #60	@ 0x3c
 8000a9a:	ea01 0192 	and.w	r1, r1, r2, lsr #2
 8000a9e:	b2d2      	uxtb	r2, r2
 8000aa0:	2a50      	cmp	r2, #80	@ 0x50
 8000aa2:	bf24      	itt	cs
 8000aa4:	9a11      	ldrcs	r2, [sp, #68]	@ 0x44
 8000aa6:	428a      	cmpcs	r2, r1
 8000aa8:	d207      	bcs.n	8000aba <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x75e>
 8000aaa:	9a11      	ldr	r2, [sp, #68]	@ 0x44
 8000aac:	f64a 23e0 	movw	r3, #43744	@ 0xaae0
 8000ab0:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ab4:	2014      	movs	r0, #20
 8000ab6:	f002 fea2 	bl	80037fe <core::slice::index::slice_index_fail>
 8000aba:	f105 0336 	add.w	r3, r5, #54	@ 0x36
 8000abe:	f1a1 0214 	sub.w	r2, r1, #20
 8000ac2:	f8cd c038 	str.w	ip, [sp, #56]	@ 0x38
 8000ac6:	b1d0      	cbz	r0, 8000afe <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7a2>
 8000ac8:	f8bd 0086 	ldrh.w	r0, [sp, #134]	@ 0x86
 8000acc:	2104      	movs	r1, #4
 8000ace:	f8ad 00a4 	strh.w	r0, [sp, #164]	@ 0xa4
 8000ad2:	a828      	add	r0, sp, #160	@ 0xa0
 8000ad4:	9128      	str	r1, [sp, #160]	@ 0xa0
 8000ad6:	4619      	mov	r1, r3
 8000ad8:	f005 fbf2 	bl	80062c0 <smoltcp::wire::tcp::TcpOption::emit>
 8000adc:	4603      	mov	r3, r0
 8000ade:	460a      	mov	r2, r1
 8000ae0:	f1bb 0f00 	cmp.w	fp, #0
 8000ae4:	d10e      	bne.n	8000b04 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7a8>
 8000ae6:	f8dd b040 	ldr.w	fp, [sp, #64]	@ 0x40
 8000aea:	f1ba 0f00 	cmp.w	sl, #0
 8000aee:	d01a      	beq.n	8000b26 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7ca>
 8000af0:	f64a 3470 	movw	r4, #43888	@ 0xab70
 8000af4:	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
 8000af8:	f6c0 0400 	movt	r4, #2048	@ 0x800
 8000afc:	e01d      	b.n	8000b3a <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7de>
 8000afe:	f1bb 0f00 	cmp.w	fp, #0
 8000b02:	d0f0      	beq.n	8000ae6 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x78a>
 8000b04:	f89d 0095 	ldrb.w	r0, [sp, #149]	@ 0x95
 8000b08:	2105      	movs	r1, #5
 8000b0a:	f88d 00a4 	strb.w	r0, [sp, #164]	@ 0xa4
 8000b0e:	a828      	add	r0, sp, #160	@ 0xa0
 8000b10:	9128      	str	r1, [sp, #160]	@ 0xa0
 8000b12:	4619      	mov	r1, r3
 8000b14:	f005 fbd4 	bl	80062c0 <smoltcp::wire::tcp::TcpOption::emit>
 8000b18:	4603      	mov	r3, r0
 8000b1a:	460a      	mov	r2, r1
 8000b1c:	f8dd b040 	ldr.w	fp, [sp, #64]	@ 0x40
 8000b20:	f1ba 0f00 	cmp.w	sl, #0
 8000b24:	d1e4      	bne.n	8000af0 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x794>
 8000b26:	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
 8000b2a:	b166      	cbz	r6, 8000b46 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7ea>
 8000b2c:	980a      	ldr	r0, [sp, #40]	@ 0x28
 8000b2e:	2800      	cmp	r0, #0
 8000b30:	bf04      	itt	eq
 8000b32:	9809      	ldreq	r0, [sp, #36]	@ 0x24
 8000b34:	2800      	cmpeq	r0, #0
 8000b36:	f000 8185 	beq.w	8000e44 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xae8>
 8000b3a:	4620      	mov	r0, r4
 8000b3c:	4619      	mov	r1, r3
 8000b3e:	f005 fbbf 	bl	80062c0 <smoltcp::wire::tcp::TcpOption::emit>
 8000b42:	4603      	mov	r3, r0
 8000b44:	460a      	mov	r2, r1
 8000b46:	f1b9 0f00 	cmp.w	r9, #0
 8000b4a:	d00b      	beq.n	8000b64 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x808>
 8000b4c:	e9dd 011f 	ldrd	r0, r1, [sp, #124]	@ 0x7c
 8000b50:	e9cd 0129 	strd	r0, r1, [sp, #164]	@ 0xa4
 8000b54:	2008      	movs	r0, #8
 8000b56:	9028      	str	r0, [sp, #160]	@ 0xa0
 8000b58:	a828      	add	r0, sp, #160	@ 0xa0
 8000b5a:	4619      	mov	r1, r3
 8000b5c:	f005 fbb0 	bl	80062c0 <smoltcp::wire::tcp::TcpOption::emit>
 8000b60:	4603      	mov	r3, r0
 8000b62:	460a      	mov	r2, r1
 8000b64:	9c11      	ldr	r4, [sp, #68]	@ 0x44
 8000b66:	b132      	cbz	r2, 8000b76 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x81a>
 8000b68:	f64a 3094 	movw	r0, #43924	@ 0xab94
 8000b6c:	4619      	mov	r1, r3
 8000b6e:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8000b72:	f005 fba5 	bl	80062c0 <smoltcp::wire::tcp::TcpOption::emit>
 8000b76:	2c13      	cmp	r4, #19
 8000b78:	f240 8244 	bls.w	8001004 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xca8>
 8000b7c:	8de8      	ldrh	r0, [r5, #46]	@ 0x2e
 8000b7e:	2100      	movs	r1, #0
 8000b80:	86a9      	strh	r1, [r5, #52]	@ 0x34
 8000b82:	213c      	movs	r1, #60	@ 0x3c
 8000b84:	ea01 0090 	and.w	r0, r1, r0, lsr #2
 8000b88:	4284      	cmp	r4, r0
 8000b8a:	f0c0 8244 	bcc.w	8001016 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xcba>
 8000b8e:	9a23      	ldr	r2, [sp, #140]	@ 0x8c
 8000b90:	1a26      	subs	r6, r4, r0
 8000b92:	42b2      	cmp	r2, r6
 8000b94:	f200 8212 	bhi.w	8000fbc <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xc60>
 8000b98:	9922      	ldr	r1, [sp, #136]	@ 0x88
 8000b9a:	4450      	add	r0, sl
 8000b9c:	f008 fc2c 	bl	80093f8 <__aeabi_memcpy>
 8000ba0:	980e      	ldr	r0, [sp, #56]	@ 0x38
 8000ba2:	f010 0ffd 	tst.w	r0, #253	@ 0xfd
 8000ba6:	d139      	bne.n	8000c1c <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c0>
 8000ba8:	2000      	movs	r0, #0
 8000baa:	8668      	strh	r0, [r5, #50]	@ 0x32
 8000bac:	fa98 f088 	rev.w	r0, r8
 8000bb0:	0c01      	lsrs	r1, r0, #16
 8000bb2:	fa11 f080 	uxtah	r0, r1, r0
 8000bb6:	0c01      	lsrs	r1, r0, #16
 8000bb8:	fa11 f080 	uxtah	r0, r1, r0
 8000bbc:	990f      	ldr	r1, [sp, #60]	@ 0x3c
 8000bbe:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8000bc2:	ba09      	rev	r1, r1
 8000bc4:	b280      	uxth	r0, r0
 8000bc6:	0c0a      	lsrs	r2, r1, #16
 8000bc8:	fa12 f181 	uxtah	r1, r2, r1
 8000bcc:	0c0a      	lsrs	r2, r1, #16
 8000bce:	fa12 f181 	uxtah	r1, r2, r1
 8000bd2:	eb01 4111 	add.w	r1, r1, r1, lsr #16
 8000bd6:	fa10 f081 	uxtah	r0, r0, r1
 8000bda:	1da1      	adds	r1, r4, #6
 8000bdc:	0c0a      	lsrs	r2, r1, #16
 8000bde:	fa12 f181 	uxtah	r1, r2, r1
 8000be2:	eb01 4111 	add.w	r1, r1, r1, lsr #16
 8000be6:	fa10 f081 	uxtah	r0, r0, r1
 8000bea:	0c01      	lsrs	r1, r0, #16
 8000bec:	fa11 f080 	uxtah	r0, r1, r0
 8000bf0:	4621      	mov	r1, r4
 8000bf2:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8000bf6:	b286      	uxth	r6, r0
 8000bf8:	4650      	mov	r0, sl
 8000bfa:	f007 faac 	bl	8008156 <smoltcp::wire::ip::checksum::data>
 8000bfe:	fa16 f080 	uxtah	r0, r6, r0
 8000c02:	0c01      	lsrs	r1, r0, #16
 8000c04:	fa11 f080 	uxtah	r0, r1, r0
 8000c08:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8000c0c:	43c0      	mvns	r0, r0
 8000c0e:	ba00      	rev	r0, r0
 8000c10:	0c00      	lsrs	r0, r0, #16
 8000c12:	e004      	b.n	8000c1e <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x8c2>
 8000c14:	3520      	adds	r5, #32
 8000c16:	e03b      	b.n	8000c90 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x934>
 8000c18:	3538      	adds	r5, #56	@ 0x38
 8000c1a:	e039      	b.n	8000c90 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x934>
 8000c1c:	2000      	movs	r0, #0
 8000c1e:	8668      	strh	r0, [r5, #50]	@ 0x32
 8000c20:	f8dd c02c 	ldr.w	ip, [sp, #44]	@ 0x2c
 8000c24:	f8db 1004 	ldr.w	r1, [fp, #4]
 8000c28:	9806      	ldr	r0, [sp, #24]
 8000c2a:	4288      	cmp	r0, r1
 8000c2c:	f080 824f 	bcs.w	80010ce <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd72>
 8000c30:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 8000c34:	f8db 2000 	ldr.w	r2, [fp]
 8000c38:	fb00 2301 	mla	r3, r0, r1, r2
 8000c3c:	e9d3 650a 	ldrd	r6, r5, [r3, #40]	@ 0x28
 8000c40:	4341      	muls	r1, r0
 8000c42:	f640 70ff 	movw	r0, #4095	@ 0xfff
 8000c46:	685c      	ldr	r4, [r3, #4]
 8000c48:	ea24 0000 	bic.w	r0, r4, r0
 8000c4c:	9c0d      	ldr	r4, [sp, #52]	@ 0x34
 8000c4e:	4320      	orrs	r0, r4
 8000c50:	6058      	str	r0, [r3, #4]
 8000c52:	2001      	movs	r0, #1
 8000c54:	609e      	str	r6, [r3, #8]
 8000c56:	e9c3 0c08 	strd	r0, ip, [r3, #32]
 8000c5a:	60dd      	str	r5, [r3, #12]
 8000c5c:	f3bf 8f5f 	dmb	sy
 8000c60:	f893 0030 	ldrb.w	r0, [r3, #48]	@ 0x30
 8000c64:	2300      	movs	r3, #0
 8000c66:	f2cf 23d0 	movt	r3, #62160	@ 0xf2d0
 8000c6a:	ea43 5040 	orr.w	r0, r3, r0, lsl #21
 8000c6e:	5050      	str	r0, [r2, r1]
 8000c70:	f249 0004 	movw	r0, #36868	@ 0x9004
 8000c74:	f3bf 8f5f 	dmb	sy
 8000c78:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 8000c7c:	2100      	movs	r1, #0
 8000c7e:	6001      	str	r1, [r0, #0]
 8000c80:	f04f 0902 	mov.w	r9, #2
 8000c84:	4648      	mov	r0, r9
 8000c86:	b031      	add	sp, #196	@ 0xc4
 8000c88:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8000c8c:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8000c8e:	3550      	adds	r5, #80	@ 0x50
 8000c90:	e9d5 2602 	ldrd	r2, r6, [r5, #8]
 8000c94:	9c07      	ldr	r4, [sp, #28]
 8000c96:	1aa2      	subs	r2, r4, r2
 8000c98:	9a06      	ldr	r2, [sp, #24]
 8000c9a:	41b2      	sbcs	r2, r6
 8000c9c:	da11      	bge.n	8000cc2 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x966>
 8000c9e:	7c6a      	ldrb	r2, [r5, #17]
 8000ca0:	f895 9010 	ldrb.w	r9, [r5, #16]
 8000ca4:	7cae      	ldrb	r6, [r5, #18]
 8000ca6:	f895 a013 	ldrb.w	sl, [r5, #19]
 8000caa:	f895 c015 	ldrb.w	ip, [r5, #21]
 8000cae:	f895 b014 	ldrb.w	fp, [r5, #20]
 8000cb2:	4615      	mov	r5, r2
 8000cb4:	9c10      	ldr	r4, [sp, #64]	@ 0x40
 8000cb6:	680a      	ldr	r2, [r1, #0]
 8000cb8:	2a02      	cmp	r2, #2
 8000cba:	f43f ac87 	beq.w	80005cc <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x270>
 8000cbe:	f7ff bbc3 	b.w	8000448 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xec>
 8000cc2:	e9d0 233a 	ldrd	r2, r3, [r0, #232]	@ 0xe8
 8000cc6:	9e07      	ldr	r6, [sp, #28]
 8000cc8:	1ab2      	subs	r2, r6, r2
 8000cca:	9a06      	ldr	r2, [sp, #24]
 8000ccc:	419a      	sbcs	r2, r3
 8000cce:	f2c0 80b2 	blt.w	8000e36 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xada>
 8000cd2:	4682      	mov	sl, r0
 8000cd4:	9805      	ldr	r0, [sp, #20]
 8000cd6:	f8ad 0049 	strh.w	r0, [sp, #73]	@ 0x49
 8000cda:	460c      	mov	r4, r1
 8000cdc:	f88d e048 	strb.w	lr, [sp, #72]	@ 0x48
 8000ce0:	0c00      	lsrs	r0, r0, #16
 8000ce2:	f88d 004b 	strb.w	r0, [sp, #75]	@ 0x4b
 8000ce6:	f240 0024 	movw	r0, #36	@ 0x24
 8000cea:	f2c0 0000 	movt	r0, #0
 8000cee:	f003 fbb1 	bl	8004454 <defmt::export::acquire_and_header>
 8000cf2:	f240 0007 	movw	r0, #7
 8000cf6:	2102      	movs	r1, #2
 8000cf8:	f2c0 0000 	movt	r0, #0
 8000cfc:	f8ad 004c 	strh.w	r0, [sp, #76]	@ 0x4c
 8000d00:	a813      	add	r0, sp, #76	@ 0x4c
 8000d02:	f003 fd59 	bl	80047b8 <_defmt_write>
 8000d06:	a812      	add	r0, sp, #72	@ 0x48
 8000d08:	f003 fb52 	bl	80043b0 <defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>
 8000d0c:	a813      	add	r0, sp, #76	@ 0x4c
 8000d0e:	2600      	movs	r6, #0
 8000d10:	2102      	movs	r1, #2
 8000d12:	f8ad 604c 	strh.w	r6, [sp, #76]	@ 0x4c
 8000d16:	f003 fd4f 	bl	80047b8 <_defmt_write>
 8000d1a:	f003 fcd5 	bl	80046c8 <_defmt_release>
 8000d1e:	f1b9 0f00 	cmp.w	r9, #0
 8000d22:	f43f ac69 	beq.w	80005f8 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x29c>
 8000d26:	f8da 20f4 	ldr.w	r2, [sl, #244]	@ 0xf4
 8000d2a:	f04f 35ff 	mov.w	r5, #4294967295	@ 0xffffffff
 8000d2e:	9912      	ldr	r1, [sp, #72]	@ 0x48
 8000d30:	f8ad 604c 	strh.w	r6, [sp, #76]	@ 0x4c
 8000d34:	f8ba 616c 	ldrh.w	r6, [sl, #364]	@ 0x16c
 8000d38:	9118      	str	r1, [sp, #96]	@ 0x60
 8000d3a:	0e11      	lsrs	r1, r2, #24
 8000d3c:	f8da 3168 	ldr.w	r3, [sl, #360]	@ 0x168
 8000d40:	f8ad 6054 	strh.w	r6, [sp, #84]	@ 0x54
 8000d44:	f88d 1059 	strb.w	r1, [sp, #89]	@ 0x59
 8000d48:	0a11      	lsrs	r1, r2, #8
 8000d4a:	68a6      	ldr	r6, [r4, #8]
 8000d4c:	f8cd 505a 	str.w	r5, [sp, #90]	@ 0x5a
 8000d50:	f8ad 505e 	strh.w	r5, [sp, #94]	@ 0x5e
 8000d54:	9314      	str	r3, [sp, #80]	@ 0x50
 8000d56:	f88d 2056 	strb.w	r2, [sp, #86]	@ 0x56
 8000d5a:	f8ad 1057 	strh.w	r1, [sp, #87]	@ 0x57
 8000d5e:	e9d4 9800 	ldrd	r9, r8, [r4]
 8000d62:	e9d6 1401 	ldrd	r1, r4, [r6, #4]
 8000d66:	428c      	cmp	r4, r1
 8000d68:	f080 81bf 	bcs.w	80010ea <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd8e>
 8000d6c:	f44f 6cc6 	mov.w	ip, #1584	@ 0x630
 8000d70:	6833      	ldr	r3, [r6, #0]
 8000d72:	fb04 f50c 	mul.w	r5, r4, ip
 8000d76:	595d      	ldr	r5, [r3, r5]
 8000d78:	2d00      	cmp	r5, #0
 8000d7a:	f100 81bd 	bmi.w	80010f8 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xd9c>
 8000d7e:	1c65      	adds	r5, r4, #1
 8000d80:	f50a 70b4 	add.w	r0, sl, #360	@ 0x168
 8000d84:	fbb5 f2f1 	udiv	r2, r5, r1
 8000d88:	fb02 5111 	mls	r1, r2, r1, r5
 8000d8c:	fb04 320c 	mla	r2, r4, ip, r3
 8000d90:	60b1      	str	r1, [r6, #8]
 8000d92:	6801      	ldr	r1, [r0, #0]
 8000d94:	8880      	ldrh	r0, [r0, #4]
 8000d96:	f8a2 0042 	strh.w	r0, [r2, #66]	@ 0x42
 8000d9a:	f04f 30ff 	mov.w	r0, #4294967295	@ 0xffffffff
 8000d9e:	6390      	str	r0, [r2, #56]	@ 0x38
 8000da0:	8790      	strh	r0, [r2, #60]	@ 0x3c
 8000da2:	f44f 60c1 	mov.w	r0, #1544	@ 0x608
 8000da6:	f8c2 103e 	str.w	r1, [r2, #62]	@ 0x3e
 8000daa:	f102 0146 	add.w	r1, r2, #70	@ 0x46
 8000dae:	f8a2 0044 	strh.w	r0, [r2, #68]	@ 0x44
 8000db2:	a813      	add	r0, sp, #76	@ 0x4c
 8000db4:	221c      	movs	r2, #28
 8000db6:	f7ff fa4b 	bl	8000250 <smoltcp::wire::arp::Repr::emit>
 8000dba:	6871      	ldr	r1, [r6, #4]
 8000dbc:	428c      	cmp	r4, r1
 8000dbe:	f080 81a6 	bcs.w	800110e <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xdb2>
 8000dc2:	f44f 60c6 	mov.w	r0, #1584	@ 0x630
 8000dc6:	6831      	ldr	r1, [r6, #0]
 8000dc8:	fb04 1000 	mla	r0, r4, r0, r1
 8000dcc:	f1b9 0f00 	cmp.w	r9, #0
 8000dd0:	d006      	beq.n	8000de0 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xa84>
 8000dd2:	2101      	movs	r1, #1
 8000dd4:	e9c0 1808 	strd	r1, r8, [r0, #32]
 8000dd8:	2100      	movs	r1, #0
 8000dda:	f2cf 21d0 	movt	r1, #62160	@ 0xf2d0
 8000dde:	e006      	b.n	8000dee <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0xa92>
 8000de0:	2100      	movs	r1, #0
 8000de2:	6201      	str	r1, [r0, #32]
 8000de4:	2100      	movs	r1, #0
 8000de6:	f2cf 21d0 	movt	r1, #62160	@ 0xf2d0
 8000dea:	f101 417e 	add.w	r1, r1, #4261412864	@ 0xfe000000
 8000dee:	6842      	ldr	r2, [r0, #4]
 8000df0:	242a      	movs	r4, #42	@ 0x2a
 8000df2:	e9d0 360a 	ldrd	r3, r6, [r0, #40]	@ 0x28
 8000df6:	4655      	mov	r5, sl
 8000df8:	f364 020b 	bfi	r2, r4, #0, #12
 8000dfc:	6042      	str	r2, [r0, #4]
 8000dfe:	6083      	str	r3, [r0, #8]
 8000e00:	60c6      	str	r6, [r0, #12]
 8000e02:	f3bf 8f5f 	dmb	sy
 8000e06:	f890 2030 	ldrb.w	r2, [r0, #48]	@ 0x30
 8000e0a:	ea41 5142 	orr.w	r1, r1, r2, lsl #21
 8000e0e:	6001      	str	r1, [r0, #0]
 8000e10:	f249 0004 	movw	r0, #36868	@ 0x9004
 8000e14:	2100      	movs	r1, #0
 8000e16:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 8000e1a:	f3bf 8f5f 	dmb	sy
 8000e1e:	6001      	str	r1, [r0, #0]
 8000e20:	f244 2240 	movw	r2, #16960	@ 0x4240
 8000e24:	e9da 0104 	ldrd	r0, r1, [sl, #16]
 8000e28:	f2c0 020f 	movt	r2, #15
 8000e2c:	1880      	adds	r0, r0, r2
 8000e2e:	f141 0100 	adc.w	r1, r1, #0
 8000e32:	e9c5 013a 	strd	r0, r1, [r5, #232]	@ 0xe8
 8000e36:	f04f 0901 	mov.w	r9, #1
 8000e3a:	4648      	mov	r0, r9
 8000e3c:	b031      	add	sp, #196	@ 0xc4
 8000e3e:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8000e42:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8000e44:	9808      	ldr	r0, [sp, #32]
 8000e46:	2800      	cmp	r0, #0
 8000e48:	f43f ae7d 	beq.w	8000b46 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7ea>
 8000e4c:	e675      	b.n	8000b3a <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7de>
 8000e4e:	f24a 5018 	movw	r0, #42264	@ 0xa518
 8000e52:	f24a 5250 	movw	r2, #42320	@ 0xa550
 8000e56:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8000e5a:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000e5e:	2136      	movs	r1, #54	@ 0x36
 8000e60:	f002 fd60 	bl	8003924 <core::panicking::panic>
 8000e64:	f24b 00b8 	movw	r0, #45240	@ 0xb0b8
 8000e68:	f24b 02d8 	movw	r2, #45272	@ 0xb0d8
 8000e6c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8000e70:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000e74:	211d      	movs	r1, #29
 8000e76:	f002 fd55 	bl	8003924 <core::panicking::panic>
 8000e7a:	f64a 13ec 	movw	r3, #43500	@ 0xa9ec
 8000e7e:	2006      	movs	r0, #6
 8000e80:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000e84:	210c      	movs	r1, #12
 8000e86:	4662      	mov	r2, ip
 8000e88:	f002 fcb9 	bl	80037fe <core::slice::index::slice_index_fail>
 8000e8c:	f64a 13fc 	movw	r3, #43516	@ 0xa9fc
 8000e90:	200c      	movs	r0, #12
 8000e92:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000e96:	210e      	movs	r1, #14
 8000e98:	4662      	mov	r2, ip
 8000e9a:	f002 fcb0 	bl	80037fe <core::slice::index::slice_index_fail>
 8000e9e:	f64a 630c 	movw	r3, #44556	@ 0xae0c
 8000ea2:	2002      	movs	r0, #2
 8000ea4:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ea8:	2104      	movs	r1, #4
 8000eaa:	4672      	mov	r2, lr
 8000eac:	f002 fca7 	bl	80037fe <core::slice::index::slice_index_fail>
 8000eb0:	f64a 634c 	movw	r3, #44620	@ 0xae4c
 8000eb4:	2004      	movs	r0, #4
 8000eb6:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000eba:	2106      	movs	r1, #6
 8000ebc:	4672      	mov	r2, lr
 8000ebe:	f002 fc9e 	bl	80037fe <core::slice::index::slice_index_fail>
 8000ec2:	f64a 53cc 	movw	r3, #44492	@ 0xadcc
 8000ec6:	2006      	movs	r0, #6
 8000ec8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ecc:	2108      	movs	r1, #8
 8000ece:	4672      	mov	r2, lr
 8000ed0:	f002 fc95 	bl	80037fe <core::slice::index::slice_index_fail>
 8000ed4:	f64a 53fc 	movw	r3, #44540	@ 0xadfc
 8000ed8:	200c      	movs	r0, #12
 8000eda:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ede:	2110      	movs	r1, #16
 8000ee0:	4672      	mov	r2, lr
 8000ee2:	f002 fc8c 	bl	80037fe <core::slice::index::slice_index_fail>
 8000ee6:	f64a 53ec 	movw	r3, #44524	@ 0xadec
 8000eea:	2010      	movs	r0, #16
 8000eec:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ef0:	2114      	movs	r1, #20
 8000ef2:	4672      	mov	r2, lr
 8000ef4:	f002 fc83 	bl	80037fe <core::slice::index::slice_index_fail>
 8000ef8:	f64a 7358 	movw	r3, #44888	@ 0xaf58
 8000efc:	2008      	movs	r0, #8
 8000efe:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f02:	4631      	mov	r1, r6
 8000f04:	464a      	mov	r2, r9
 8000f06:	f002 fc7a 	bl	80037fe <core::slice::index::slice_index_fail>
 8000f0a:	f64a 7388 	movw	r3, #44936	@ 0xaf88
 8000f0e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f12:	2000      	movs	r0, #0
 8000f14:	2102      	movs	r1, #2
 8000f16:	464a      	mov	r2, r9
 8000f18:	f002 fc71 	bl	80037fe <core::slice::index::slice_index_fail>
 8000f1c:	f64a 3310 	movw	r3, #43792	@ 0xab10
 8000f20:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f24:	2000      	movs	r0, #0
 8000f26:	2102      	movs	r1, #2
 8000f28:	464a      	mov	r2, r9
 8000f2a:	f002 fc68 	bl	80037fe <core::slice::index::slice_index_fail>
 8000f2e:	f64a 7378 	movw	r3, #44920	@ 0xaf78
 8000f32:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f36:	2002      	movs	r0, #2
 8000f38:	2104      	movs	r1, #4
 8000f3a:	464a      	mov	r2, r9
 8000f3c:	f002 fc5f 	bl	80037fe <core::slice::index::slice_index_fail>
 8000f40:	f64a 3300 	movw	r3, #43776	@ 0xab00
 8000f44:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f48:	2002      	movs	r0, #2
 8000f4a:	2104      	movs	r1, #4
 8000f4c:	464a      	mov	r2, r9
 8000f4e:	f002 fc56 	bl	80037fe <core::slice::index::slice_index_fail>
 8000f52:	f64a 7398 	movw	r3, #44952	@ 0xaf98
 8000f56:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f5a:	2004      	movs	r0, #4
 8000f5c:	2106      	movs	r1, #6
 8000f5e:	464a      	mov	r2, r9
 8000f60:	f002 fc4d 	bl	80037fe <core::slice::index::slice_index_fail>
 8000f64:	f64a 3340 	movw	r3, #43840	@ 0xab40
 8000f68:	2004      	movs	r0, #4
 8000f6a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f6e:	2108      	movs	r1, #8
 8000f70:	464a      	mov	r2, r9
 8000f72:	f002 fc44 	bl	80037fe <core::slice::index::slice_index_fail>
 8000f76:	f64a 3330 	movw	r3, #43824	@ 0xab30
 8000f7a:	2008      	movs	r0, #8
 8000f7c:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f80:	210c      	movs	r1, #12
 8000f82:	464a      	mov	r2, r9
 8000f84:	f002 fc3b 	bl	80037fe <core::slice::index::slice_index_fail>
 8000f88:	9a11      	ldr	r2, [sp, #68]	@ 0x44
 8000f8a:	f64a 3350 	movw	r3, #43856	@ 0xab50
 8000f8e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f92:	200e      	movs	r0, #14
 8000f94:	2110      	movs	r1, #16
 8000f96:	f002 fc32 	bl	80037fe <core::slice::index::slice_index_fail>
 8000f9a:	f64a 73a8 	movw	r3, #44968	@ 0xafa8
 8000f9e:	4611      	mov	r1, r2
 8000fa0:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fa4:	461a      	mov	r2, r3
 8000fa6:	f003 f8b7 	bl	8004118 <core::slice::copy_from_slice_impl::len_mismatch_fail>
 8000faa:	f64a 7368 	movw	r3, #44904	@ 0xaf68
 8000fae:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fb2:	2006      	movs	r0, #6
 8000fb4:	2108      	movs	r1, #8
 8000fb6:	464a      	mov	r2, r9
 8000fb8:	f002 fc21 	bl	80037fe <core::slice::index::slice_index_fail>
 8000fbc:	f64a 33b8 	movw	r3, #43960	@ 0xabb8
 8000fc0:	4611      	mov	r1, r2
 8000fc2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fc6:	2000      	movs	r0, #0
 8000fc8:	4632      	mov	r2, r6
 8000fca:	f002 fc18 	bl	80037fe <core::slice::index::slice_index_fail>
 8000fce:	f64a 43e0 	movw	r3, #44256	@ 0xace0
 8000fd2:	2008      	movs	r0, #8
 8000fd4:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fd8:	4649      	mov	r1, r9
 8000fda:	464a      	mov	r2, r9
 8000fdc:	f002 fc0f 	bl	80037fe <core::slice::index::slice_index_fail>
 8000fe0:	f64a 43c0 	movw	r3, #44224	@ 0xacc0
 8000fe4:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fe8:	2004      	movs	r0, #4
 8000fea:	2106      	movs	r1, #6
 8000fec:	464a      	mov	r2, r9
 8000fee:	f002 fc06 	bl	80037fe <core::slice::index::slice_index_fail>
 8000ff2:	f64a 43d0 	movw	r3, #44240	@ 0xacd0
 8000ff6:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ffa:	2006      	movs	r0, #6
 8000ffc:	2108      	movs	r1, #8
 8000ffe:	464a      	mov	r2, r9
 8001000:	f002 fbfd 	bl	80037fe <core::slice::index::slice_index_fail>
 8001004:	f64a 3320 	movw	r3, #43808	@ 0xab20
 8001008:	2012      	movs	r0, #18
 800100a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800100e:	2114      	movs	r1, #20
 8001010:	4622      	mov	r2, r4
 8001012:	f002 fbf4 	bl	80037fe <core::slice::index::slice_index_fail>
 8001016:	f64a 23f0 	movw	r3, #43760	@ 0xaaf0
 800101a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800101e:	4621      	mov	r1, r4
 8001020:	4622      	mov	r2, r4
 8001022:	f002 fbec 	bl	80037fe <core::slice::index::slice_index_fail>
 8001026:	f64a 5310 	movw	r3, #44304	@ 0xad10
 800102a:	2014      	movs	r0, #20
 800102c:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8001030:	4621      	mov	r1, r4
 8001032:	4622      	mov	r2, r4
 8001034:	f002 fbe3 	bl	80037fe <core::slice::index::slice_index_fail>
 8001038:	f64a 5330 	movw	r3, #44336	@ 0xad30
 800103c:	2014      	movs	r0, #20
 800103e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8001042:	4631      	mov	r1, r6
 8001044:	4632      	mov	r2, r6
 8001046:	f002 fbda 	bl	80037fe <core::slice::index::slice_index_fail>
 800104a:	f64a 5300 	movw	r3, #44288	@ 0xad00
 800104e:	4610      	mov	r0, r2
 8001050:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8001054:	4631      	mov	r1, r6
 8001056:	461a      	mov	r2, r3
 8001058:	f003 f85e 	bl	8004118 <core::slice::copy_from_slice_impl::len_mismatch_fail>
 800105c:	f64a 5220 	movw	r2, #44320	@ 0xad20
 8001060:	4621      	mov	r1, r4
 8001062:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8001066:	f003 f857 	bl	8004118 <core::slice::copy_from_slice_impl::len_mismatch_fail>
 800106a:	f24b 0298 	movw	r2, #45208	@ 0xb098
 800106e:	4630      	mov	r0, r6
 8001070:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8001074:	f002 fc42 	bl	80038fc <core::panicking::panic_bounds_check>
 8001078:	2002      	movs	r0, #2
 800107a:	9013      	str	r0, [sp, #76]	@ 0x4c
 800107c:	a813      	add	r0, sp, #76	@ 0x4c
 800107e:	f004 fff4 	bl	800606a <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>
 8001082:	f64a 200c 	movw	r0, #43532	@ 0xaa0c
 8001086:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800108a:	f002 fbae 	bl	80037ea <core::option::unwrap_failed>
 800108e:	f64a 52dc 	movw	r2, #44508	@ 0xaddc
 8001092:	2000      	movs	r0, #0
 8001094:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8001098:	2100      	movs	r1, #0
 800109a:	f002 fc2f 	bl	80038fc <core::panicking::panic_bounds_check>
 800109e:	f64a 623c 	movw	r2, #44604	@ 0xae3c
 80010a2:	2001      	movs	r0, #1
 80010a4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010a8:	2101      	movs	r1, #1
 80010aa:	f002 fc27 	bl	80038fc <core::panicking::panic_bounds_check>
 80010ae:	f64a 625c 	movw	r2, #44636	@ 0xae5c
 80010b2:	2008      	movs	r0, #8
 80010b4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010b8:	2108      	movs	r1, #8
 80010ba:	f002 fc1f 	bl	80038fc <core::panicking::panic_bounds_check>
 80010be:	f64a 621c 	movw	r2, #44572	@ 0xae1c
 80010c2:	2009      	movs	r0, #9
 80010c4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010c8:	2109      	movs	r1, #9
 80010ca:	f002 fc17 	bl	80038fc <core::panicking::panic_bounds_check>
 80010ce:	f24b 0258 	movw	r2, #45144	@ 0xb058
 80010d2:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010d6:	f002 fc11 	bl	80038fc <core::panicking::panic_bounds_check>
 80010da:	f64a 42f0 	movw	r2, #44272	@ 0xacf0
 80010de:	2001      	movs	r0, #1
 80010e0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010e4:	4649      	mov	r1, r9
 80010e6:	f002 fc09 	bl	80038fc <core::panicking::panic_bounds_check>
 80010ea:	f24b 0298 	movw	r2, #45208	@ 0xb098
 80010ee:	4620      	mov	r0, r4
 80010f0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010f4:	f002 fc02 	bl	80038fc <core::panicking::panic_bounds_check>
 80010f8:	2002      	movs	r0, #2
 80010fa:	9028      	str	r0, [sp, #160]	@ 0xa0
 80010fc:	a828      	add	r0, sp, #160	@ 0xa0
 80010fe:	f004 ffb4 	bl	800606a <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>
 8001102:	f64a 200c 	movw	r0, #43532	@ 0xaa0c
 8001106:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800110a:	f002 fb6e 	bl	80037ea <core::option::unwrap_failed>
 800110e:	f24b 0258 	movw	r2, #45144	@ 0xb058
 8001112:	4620      	mov	r0, r4
 8001114:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8001118:	f002 fbf0 	bl	80038fc <core::panicking::panic_bounds_check>

0800111c <smoltcp::iface::interface::Interface::poll>:
 800111c:	b5f0      	push	{r4, r5, r6, r7, lr}
 800111e:	af03      	add	r7, sp, #12
 8001120:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8001124:	f5ad 7d5d 	sub.w	sp, sp, #884	@ 0x374
 8001128:	aebe      	add	r6, sp, #760	@ 0x2f8
 800112a:	f500 7bb8 	add.w	fp, r0, #368	@ 0x170
 800112e:	1c71      	adds	r1, r6, #1
 8001130:	913f      	str	r1, [sp, #252]	@ 0xfc
 8001132:	a98c      	add	r1, sp, #560	@ 0x230
 8001134:	e9c0 2304 	strd	r2, r3, [r0, #16]
 8001138:	f101 0510 	add.w	r5, r1, #16
 800113c:	9536      	str	r5, [sp, #216]	@ 0xd8
 800113e:	f106 0554 	add.w	r5, r6, #84	@ 0x54
 8001142:	9532      	str	r5, [sp, #200]	@ 0xc8
 8001144:	ada5      	add	r5, sp, #660	@ 0x294
 8001146:	904a      	str	r0, [sp, #296]	@ 0x128
 8001148:	f105 0454 	add.w	r4, r5, #84	@ 0x54
 800114c:	9431      	str	r4, [sp, #196]	@ 0xc4
 800114e:	f101 040d 	add.w	r4, r1, #13
 8001152:	3124      	adds	r1, #36	@ 0x24
 8001154:	9133      	str	r1, [sp, #204]	@ 0xcc
 8001156:	a954      	add	r1, sp, #336	@ 0x150
 8001158:	3102      	adds	r1, #2
 800115a:	913a      	str	r1, [sp, #232]	@ 0xe8
 800115c:	f500 7180 	add.w	r1, r0, #256	@ 0x100
 8001160:	9135      	str	r1, [sp, #212]	@ 0xd4
 8001162:	f100 0120 	add.w	r1, r0, #32
 8001166:	913e      	str	r1, [sp, #248]	@ 0xf8
 8001168:	f100 01f4 	add.w	r1, r0, #244	@ 0xf4
 800116c:	9144      	str	r1, [sp, #272]	@ 0x110
 800116e:	f500 71b4 	add.w	r1, r0, #360	@ 0x168
 8001172:	a857      	add	r0, sp, #348	@ 0x15c
 8001174:	9146      	str	r1, [sp, #280]	@ 0x118
 8001176:	f100 011c 	add.w	r1, r0, #28
 800117a:	9142      	str	r1, [sp, #264]	@ 0x108
 800117c:	f100 0113 	add.w	r1, r0, #19
 8001180:	3006      	adds	r0, #6
 8001182:	9040      	str	r0, [sp, #256]	@ 0x100
 8001184:	f105 000f 	add.w	r0, r5, #15
 8001188:	9039      	str	r0, [sp, #228]	@ 0xe4
 800118a:	1ca8      	adds	r0, r5, #2
 800118c:	9038      	str	r0, [sp, #224]	@ 0xe0
 800118e:	f106 001c 	add.w	r0, r6, #28
 8001192:	903d      	str	r0, [sp, #244]	@ 0xf4
 8001194:	f106 0013 	add.w	r0, r6, #19
 8001198:	903c      	str	r0, [sp, #240]	@ 0xf0
 800119a:	1db0      	adds	r0, r6, #6
 800119c:	903b      	str	r0, [sp, #236]	@ 0xec
 800119e:	f8d7 a008 	ldr.w	sl, [r7, #8]
 80011a2:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 80011a6:	9434      	str	r4, [sp, #208]	@ 0xd0
 80011a8:	f249 0408 	movw	r4, #36872	@ 0x9008
 80011ac:	f10a 080c 	add.w	r8, sl, #12
 80011b0:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 80011b4:	f04f 0c01 	mov.w	ip, #1
 80011b8:	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
 80011bc:	e9cd 2347 	strd	r2, r3, [sp, #284]	@ 0x11c
 80011c0:	9141      	str	r1, [sp, #260]	@ 0x104
 80011c2:	f8cd 810c 	str.w	r8, [sp, #268]	@ 0x10c
 80011c6:	f8cd b124 	str.w	fp, [sp, #292]	@ 0x124
 80011ca:	2000      	movs	r0, #0
 80011cc:	9045      	str	r0, [sp, #276]	@ 0x114
 80011ce:	2000      	movs	r0, #0
 80011d0:	9037      	str	r0, [sp, #220]	@ 0xdc
 80011d2:	e9da 1004 	ldrd	r1, r0, [sl, #16]
 80011d6:	4288      	cmp	r0, r1
 80011d8:	d317      	bcc.n	800120a <smoltcp::iface::interface::Interface::poll+0xee>
 80011da:	f24b 02a8 	movw	r2, #45224	@ 0xb0a8
 80011de:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80011e2:	f002 fb8b 	bl	80038fc <core::panicking::panic_bounds_check>
 80011e6:	f240 002f 	movw	r0, #47	@ 0x2f
 80011ea:	4664      	mov	r4, ip
 80011ec:	f2c0 0000 	movt	r0, #0
 80011f0:	4675      	mov	r5, lr
 80011f2:	f003 f940 	bl	8004476 <defmt::export::acquire_header_and_release>
 80011f6:	46ae      	mov	lr, r5
 80011f8:	46a4      	mov	ip, r4
 80011fa:	f249 0408 	movw	r4, #36872	@ 0x9008
 80011fe:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 8001202:	e9da 1004 	ldrd	r1, r0, [sl, #16]
 8001206:	4288      	cmp	r0, r1
 8001208:	d2e7      	bcs.n	80011da <smoltcp::iface::interface::Interface::poll+0xbe>
 800120a:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 800120e:	4348      	muls	r0, r1
 8001210:	f8d8 1000 	ldr.w	r1, [r8]
 8001214:	5808      	ldr	r0, [r1, r0]
 8001216:	2800      	cmp	r0, #0
 8001218:	f101 8242 	bmi.w	80026a0 <smoltcp::iface::interface::Interface::poll+0x1584>
 800121c:	68e0      	ldr	r0, [r4, #12]
 800121e:	f3c0 4042 	ubfx	r0, r0, #17, #3
 8001222:	2807      	cmp	r0, #7
 8001224:	d87b      	bhi.n	800131e <smoltcp::iface::interface::Interface::poll+0x202>
 8001226:	fa0c f000 	lsl.w	r0, ip, r0
 800122a:	f010 0faa 	tst.w	r0, #170	@ 0xaa
 800122e:	bf08      	it	eq
 8001230:	f8c4 c000 	streq.w	ip, [r4]
 8001234:	e9da 1001 	ldrd	r1, r0, [sl, #4]
 8001238:	4288      	cmp	r0, r1
 800123a:	f082 80ce 	bcs.w	80033da <smoltcp::iface::interface::Interface::poll+0x22be>
 800123e:	fb00 f20e 	mul.w	r2, r0, lr
 8001242:	f8da 3000 	ldr.w	r3, [sl]
 8001246:	589a      	ldr	r2, [r3, r2]
 8001248:	2a00      	cmp	r2, #0
 800124a:	f101 8229 	bmi.w	80026a0 <smoltcp::iface::interface::Interface::poll+0x1584>
 800124e:	f8da 2018 	ldr.w	r2, [sl, #24]
 8001252:	2500      	movs	r5, #0
 8001254:	f8cd 8138 	str.w	r8, [sp, #312]	@ 0x138
 8001258:	1c56      	adds	r6, r2, #1
 800125a:	f8ca 6018 	str.w	r6, [sl, #24]
 800125e:	68fe      	ldr	r6, [r7, #12]
 8001260:	954c      	str	r5, [sp, #304]	@ 0x130
 8001262:	f8cd 914c 	str.w	r9, [sp, #332]	@ 0x14c
 8001266:	9652      	str	r6, [sp, #328]	@ 0x148
 8001268:	924b      	str	r2, [sp, #300]	@ 0x12c
 800126a:	68e6      	ldr	r6, [r4, #12]
 800126c:	9d4a      	ldr	r5, [sp, #296]	@ 0x128
 800126e:	e9cd bb50 	strd	fp, fp, [sp, #320]	@ 0x140
 8001272:	f3c6 4642 	ubfx	r6, r6, #17, #3
 8001276:	954f      	str	r5, [sp, #316]	@ 0x13c
 8001278:	2e07      	cmp	r6, #7
 800127a:	d858      	bhi.n	800132e <smoltcp::iface::interface::Interface::poll+0x212>
 800127c:	fa0c f606 	lsl.w	r6, ip, r6
 8001280:	f016 0faa 	tst.w	r6, #170	@ 0xaa
 8001284:	bf08      	it	eq
 8001286:	f8c4 c000 	streq.w	ip, [r4]
 800128a:	fb00 340e 	mla	r4, r0, lr, r3
 800128e:	6823      	ldr	r3, [r4, #0]
 8001290:	2b00      	cmp	r3, #0
 8001292:	d4a8      	bmi.n	80011e6 <smoltcp::iface::interface::Interface::poll+0xca>
 8001294:	6823      	ldr	r3, [r4, #0]
 8001296:	f413 4300 	ands.w	r3, r3, #32768	@ 0x8000
 800129a:	d106      	bne.n	80012aa <smoltcp::iface::interface::Interface::poll+0x18e>
 800129c:	6826      	ldr	r6, [r4, #0]
 800129e:	05b6      	lsls	r6, r6, #22
 80012a0:	bf44      	itt	mi
 80012a2:	6826      	ldrmi	r6, [r4, #0]
 80012a4:	ea5f 56c6 	movsmi.w	r6, r6, lsl #23
 80012a8:	d449      	bmi.n	800133e <smoltcp::iface::interface::Interface::poll+0x222>
 80012aa:	6a20      	ldr	r0, [r4, #32]
 80012ac:	2800      	cmp	r0, #0
 80012ae:	f002 802a 	beq.w	8003306 <smoltcp::iface::interface::Interface::poll+0x21ea>
 80012b2:	e9d4 0109 	ldrd	r0, r1, [r4, #36]	@ 0x24
 80012b6:	2900      	cmp	r1, #0
 80012b8:	60a0      	str	r0, [r4, #8]
 80012ba:	f002 802f 	beq.w	800331c <smoltcp::iface::interface::Interface::poll+0x2200>
 80012be:	6ae0      	ldr	r0, [r4, #44]	@ 0x2c
 80012c0:	60e0      	str	r0, [r4, #12]
 80012c2:	f04f 4000 	mov.w	r0, #2147483648	@ 0x80000000
 80012c6:	f3bf 8f5f 	dmb	sy
 80012ca:	6020      	str	r0, [r4, #0]
 80012cc:	0bdc      	lsrs	r4, r3, #15
 80012ce:	2c02      	cmp	r4, #2
 80012d0:	f3bf 8f5f 	dmb	sy
 80012d4:	d087      	beq.n	80011e6 <smoltcp::iface::interface::Interface::poll+0xca>
 80012d6:	f240 002a 	movw	r0, #42	@ 0x2a
 80012da:	46c1      	mov	r9, r8
 80012dc:	f2c0 0000 	movt	r0, #0
 80012e0:	46d8      	mov	r8, fp
 80012e2:	4666      	mov	r6, ip
 80012e4:	46f3      	mov	fp, lr
 80012e6:	f003 f8b5 	bl	8004454 <defmt::export::acquire_and_header>
 80012ea:	f240 0041 	movw	r0, #65	@ 0x41
 80012ee:	adbe      	add	r5, sp, #760	@ 0x2f8
 80012f0:	f2c0 0000 	movt	r0, #0
 80012f4:	2102      	movs	r1, #2
 80012f6:	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
 80012fa:	4628      	mov	r0, r5
 80012fc:	f003 fa5c 	bl	80047b8 <_defmt_write>
 8001300:	4628      	mov	r0, r5
 8001302:	2101      	movs	r1, #1
 8001304:	f88d 42f8 	strb.w	r4, [sp, #760]	@ 0x2f8
 8001308:	f003 fa56 	bl	80047b8 <_defmt_write>
 800130c:	f003 f9dc 	bl	80046c8 <_defmt_release>
 8001310:	a84b      	add	r0, sp, #300	@ 0x12c
 8001312:	46de      	mov	lr, fp
 8001314:	46c3      	mov	fp, r8
 8001316:	46c8      	mov	r8, r9
 8001318:	4681      	mov	r9, r0
 800131a:	46b4      	mov	ip, r6
 800131c:	e76d      	b.n	80011fa <smoltcp::iface::interface::Interface::poll+0xde>
 800131e:	f8c4 c000 	str.w	ip, [r4]
 8001322:	e9da 1001 	ldrd	r1, r0, [sl, #4]
 8001326:	4288      	cmp	r0, r1
 8001328:	d389      	bcc.n	800123e <smoltcp::iface::interface::Interface::poll+0x122>
 800132a:	f002 b856 	b.w	80033da <smoltcp::iface::interface::Interface::poll+0x22be>
 800132e:	f8c4 c000 	str.w	ip, [r4]
 8001332:	fb00 340e 	mla	r4, r0, lr, r3
 8001336:	6823      	ldr	r3, [r4, #0]
 8001338:	2b00      	cmp	r3, #0
 800133a:	d5ab      	bpl.n	8001294 <smoltcp::iface::interface::Interface::poll+0x178>
 800133c:	e753      	b.n	80011e6 <smoltcp::iface::interface::Interface::poll+0xca>
 800133e:	f8d4 c000 	ldr.w	ip, [r4]
 8001342:	f8d4 e000 	ldr.w	lr, [r4]
 8001346:	2500      	movs	r5, #0
 8001348:	69e3      	ldr	r3, [r4, #28]
 800134a:	f8d4 8018 	ldr.w	r8, [r4, #24]
 800134e:	f028 4600 	bic.w	r6, r8, #2147483648	@ 0x80000000
 8001352:	ea46 76c3 	orr.w	r6, r6, r3, lsl #31
 8001356:	f1d6 0b00 	rsbs	fp, r6, #0
 800135a:	eb65 0a53 	sbc.w	sl, r5, r3, lsr #1
 800135e:	f1b8 0f00 	cmp.w	r8, #0
 8001362:	bf5c      	itt	pl
 8001364:	46b3      	movpl	fp, r6
 8001366:	ea4f 0a53 	movpl.w	sl, r3, lsr #1
 800136a:	ea5f 630e 	movs.w	r3, lr, lsl #24
 800136e:	f8d7 8008 	ldr.w	r8, [r7, #8]
 8001372:	bf41      	itttt	mi
 8001374:	6823      	ldrmi	r3, [r4, #0]
 8001376:	ea5f 53c3 	movsmi.w	r3, r3, lsl #23
 800137a:	f04f 0900 	movmi.w	r9, #0
 800137e:	f04f 0e01 	movmi.w	lr, #1
 8001382:	bf5c      	itt	pl
 8001384:	f04f 0e00 	movpl.w	lr, #0
 8001388:	f04f 0900 	movpl.w	r9, #0
 800138c:	e9d8 3501 	ldrd	r3, r5, [r8, #4]
 8001390:	3501      	adds	r5, #1
 8001392:	e9c4 e90e 	strd	lr, r9, [r4, #56]	@ 0x38
 8001396:	fbb5 f6f1 	udiv	r6, r5, r1
 800139a:	4298      	cmp	r0, r3
 800139c:	fb06 5111 	mls	r1, r6, r1, r5
 80013a0:	f04f 0501 	mov.w	r5, #1
 80013a4:	e9c4 520c 	strd	r5, r2, [r4, #48]	@ 0x30
 80013a8:	e9c4 ba10 	strd	fp, sl, [r4, #64]	@ 0x40
 80013ac:	f8c8 1008 	str.w	r1, [r8, #8]
 80013b0:	f082 8027 	bcs.w	8003402 <smoltcp::iface::interface::Interface::poll+0x22e6>
 80013b4:	46c2      	mov	sl, r8
 80013b6:	9d43      	ldr	r5, [sp, #268]	@ 0x10c
 80013b8:	f8dd 8124 	ldr.w	r8, [sp, #292]	@ 0x124
 80013bc:	f3cc 410d 	ubfx	r1, ip, #16, #14
 80013c0:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 80013c4:	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
 80013c8:	f240 52f3 	movw	r2, #1523	@ 0x5f3
 80013cc:	4291      	cmp	r1, r2
 80013ce:	f081 87b0 	bcs.w	8003332 <smoltcp::iface::interface::Interface::poll+0x2216>
 80013d2:	f8da 2000 	ldr.w	r2, [sl]
 80013d6:	fb00 260e 	mla	r6, r0, lr, r2
 80013da:	b179      	cbz	r1, 80013fc <smoltcp::iface::interface::Interface::poll+0x2e0>
 80013dc:	f106 0248 	add.w	r2, r6, #72	@ 0x48
 80013e0:	f249 0408 	movw	r4, #36872	@ 0x9008
 80013e4:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 80013e8:	f04f 0c01 	mov.w	ip, #1
 80013ec:	4610      	mov	r0, r2
 80013ee:	46c3      	mov	fp, r8
 80013f0:	290e      	cmp	r1, #14
 80013f2:	bf38      	it	cc
 80013f4:	2000      	movcc	r0, #0
 80013f6:	d20a      	bcs.n	800140e <smoltcp::iface::interface::Interface::poll+0x2f2>
 80013f8:	46a8      	mov	r8, r5
 80013fa:	e0c1      	b.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80013fc:	f249 0408 	movw	r4, #36872	@ 0x9008
 8001400:	46c3      	mov	fp, r8
 8001402:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 8001406:	f04f 0c01 	mov.w	ip, #1
 800140a:	46a8      	mov	r8, r5
 800140c:	e0b8      	b.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 800140e:	9b4b      	ldr	r3, [sp, #300]	@ 0x12c
 8001410:	46b1      	mov	r9, r6
 8001412:	932e      	str	r3, [sp, #184]	@ 0xb8
 8001414:	68fb      	ldr	r3, [r7, #12]
 8001416:	e9d3 3600 	ldrd	r3, r6, [r3]
 800141a:	e9cd 632f 	strd	r6, r3, [sp, #188]	@ 0xbc
 800141e:	6813      	ldr	r3, [r2, #0]
 8001420:	07de      	lsls	r6, r3, #31
 8001422:	d113      	bne.n	800144c <smoltcp::iface::interface::Interface::poll+0x330>
 8001424:	9e46      	ldr	r6, [sp, #280]	@ 0x118
 8001426:	46cb      	mov	fp, r9
 8001428:	8892      	ldrh	r2, [r2, #4]
 800142a:	46b1      	mov	r9, r6
 800142c:	88b6      	ldrh	r6, [r6, #4]
 800142e:	f8d9 9000 	ldr.w	r9, [r9]
 8001432:	4072      	eors	r2, r6
 8001434:	ea83 0309 	eor.w	r3, r3, r9
 8001438:	46d9      	mov	r9, fp
 800143a:	b292      	uxth	r2, r2
 800143c:	46c3      	mov	fp, r8
 800143e:	431a      	orrs	r2, r3
 8001440:	d004      	beq.n	800144c <smoltcp::iface::interface::Interface::poll+0x330>
 8001442:	464e      	mov	r6, r9
 8001444:	46a8      	mov	r8, r5
 8001446:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 800144a:	e099      	b.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 800144c:	f8b9 2054 	ldrh.w	r2, [r9, #84]	@ 0x54
 8001450:	464e      	mov	r6, r9
 8001452:	ba12      	rev	r2, r2
 8001454:	0c12      	lsrs	r2, r2, #16
 8001456:	f5b2 6f00 	cmp.w	r2, #2048	@ 0x800
 800145a:	f000 8082 	beq.w	8001562 <smoltcp::iface::interface::Interface::poll+0x446>
 800145e:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 8001462:	46a8      	mov	r8, r5
 8001464:	f640 0006 	movw	r0, #2054	@ 0x806
 8001468:	4282      	cmp	r2, r0
 800146a:	f040 8089 	bne.w	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 800146e:	f1a1 020e 	sub.w	r2, r1, #14
 8001472:	2a08      	cmp	r2, #8
 8001474:	f0c0 8084 	bcc.w	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 8001478:	f896 005a 	ldrb.w	r0, [r6, #90]	@ 0x5a
 800147c:	f896 105b 	ldrb.w	r1, [r6, #91]	@ 0x5b
 8001480:	4408      	add	r0, r1
 8001482:	3004      	adds	r0, #4
 8001484:	ebb2 0f40 	cmp.w	r2, r0, lsl #1
 8001488:	d37a      	bcc.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 800148a:	f896 005a 	ldrb.w	r0, [r6, #90]	@ 0x5a
 800148e:	f896 105b 	ldrb.w	r1, [r6, #91]	@ 0x5b
 8001492:	180b      	adds	r3, r1, r0
 8001494:	3304      	adds	r3, #4
 8001496:	ebb2 0f43 	cmp.w	r2, r3, lsl #1
 800149a:	d371      	bcc.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 800149c:	2904      	cmp	r1, #4
 800149e:	bf08      	it	eq
 80014a0:	2806      	cmpeq	r0, #6
 80014a2:	d16d      	bne.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80014a4:	f8b6 3056 	ldrh.w	r3, [r6, #86]	@ 0x56
 80014a8:	f5b3 7f80 	cmp.w	r3, #256	@ 0x100
 80014ac:	bf04      	itt	eq
 80014ae:	f8b6 3058 	ldrheq.w	r3, [r6, #88]	@ 0x58
 80014b2:	2b08      	cmpeq	r3, #8
 80014b4:	d164      	bne.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80014b6:	2a0d      	cmp	r2, #13
 80014b8:	f241 87b0 	bls.w	800341c <smoltcp::iface::interface::Interface::poll+0x2300>
 80014bc:	310e      	adds	r1, #14
 80014be:	4291      	cmp	r1, r2
 80014c0:	f201 87b4 	bhi.w	800342c <smoltcp::iface::interface::Interface::poll+0x2310>
 80014c4:	eb01 0c00 	add.w	ip, r1, r0
 80014c8:	962d      	str	r6, [sp, #180]	@ 0xb4
 80014ca:	4594      	cmp	ip, r2
 80014cc:	f201 87b5 	bhi.w	800343a <smoltcp::iface::interface::Interface::poll+0x231e>
 80014d0:	2a1b      	cmp	r2, #27
 80014d2:	f241 87ba 	bls.w	800344a <smoltcp::iface::interface::Interface::poll+0x232e>
 80014d6:	984a      	ldr	r0, [sp, #296]	@ 0x128
 80014d8:	464d      	mov	r5, r9
 80014da:	e9d0 1204 	ldrd	r1, r2, [r0, #16]
 80014de:	912c      	str	r1, [sp, #176]	@ 0xb0
 80014e0:	992d      	ldr	r1, [sp, #180]	@ 0xb4
 80014e2:	922a      	str	r2, [sp, #168]	@ 0xa8
 80014e4:	f8d0 00f0 	ldr.w	r0, [r0, #240]	@ 0xf0
 80014e8:	f8d1 205f 	ldr.w	r2, [r1, #95]	@ 0x5f
 80014ec:	922e      	str	r2, [sp, #184]	@ 0xb8
 80014ee:	f891 205e 	ldrb.w	r2, [r1, #94]	@ 0x5e
 80014f2:	eb00 0980 	add.w	r9, r0, r0, lsl #2
 80014f6:	922f      	str	r2, [sp, #188]	@ 0xbc
 80014f8:	6e4a      	ldr	r2, [r1, #100]	@ 0x64
 80014fa:	f8b1 405c 	ldrh.w	r4, [r1, #92]	@ 0x5c
 80014fe:	f891 e063 	ldrb.w	lr, [r1, #99]	@ 0x63
 8001502:	f8d1 606e 	ldr.w	r6, [r1, #110]	@ 0x6e
 8001506:	9944      	ldr	r1, [sp, #272]	@ 0x110
 8001508:	9230      	str	r2, [sp, #192]	@ 0xc0
 800150a:	464a      	mov	r2, r9
 800150c:	eb01 0309 	add.w	r3, r1, r9
 8001510:	2a00      	cmp	r2, #0
 8001512:	f000 80b9 	beq.w	8001688 <smoltcp::iface::interface::Interface::poll+0x56c>
 8001516:	4608      	mov	r0, r1
 8001518:	46b4      	mov	ip, r6
 800151a:	f850 6b05 	ldr.w	r6, [r0], #5
 800151e:	4566      	cmp	r6, ip
 8001520:	4666      	mov	r6, ip
 8001522:	f000 80b6 	beq.w	8001692 <smoltcp::iface::interface::Interface::poll+0x576>
 8001526:	4298      	cmp	r0, r3
 8001528:	f000 80ae 	beq.w	8001688 <smoltcp::iface::interface::Interface::poll+0x56c>
 800152c:	f8d1 0005 	ldr.w	r0, [r1, #5]
 8001530:	42b0      	cmp	r0, r6
 8001532:	f000 80ae 	beq.w	8001692 <smoltcp::iface::interface::Interface::poll+0x576>
 8001536:	f101 000a 	add.w	r0, r1, #10
 800153a:	4298      	cmp	r0, r3
 800153c:	f000 80a4 	beq.w	8001688 <smoltcp::iface::interface::Interface::poll+0x56c>
 8001540:	f8d1 000a 	ldr.w	r0, [r1, #10]
 8001544:	42b0      	cmp	r0, r6
 8001546:	f000 80a4 	beq.w	8001692 <smoltcp::iface::interface::Interface::poll+0x576>
 800154a:	f101 000f 	add.w	r0, r1, #15
 800154e:	4298      	cmp	r0, r3
 8001550:	f000 809a 	beq.w	8001688 <smoltcp::iface::interface::Interface::poll+0x56c>
 8001554:	f8d1 000f 	ldr.w	r0, [r1, #15]
 8001558:	3a14      	subs	r2, #20
 800155a:	3114      	adds	r1, #20
 800155c:	42b0      	cmp	r0, r6
 800155e:	d1d7      	bne.n	8001510 <smoltcp::iface::interface::Interface::poll+0x3f4>
 8001560:	e097      	b.n	8001692 <smoltcp::iface::interface::Interface::poll+0x576>
 8001562:	f1a1 020e 	sub.w	r2, r1, #14
 8001566:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 800156a:	46a8      	mov	r8, r5
 800156c:	2a14      	cmp	r2, #20
 800156e:	d307      	bcc.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 8001570:	f100 050e 	add.w	r5, r0, #14
 8001574:	213c      	movs	r1, #60	@ 0x3c
 8001576:	7828      	ldrb	r0, [r5, #0]
 8001578:	ea01 0380 	and.w	r3, r1, r0, lsl #2
 800157c:	429a      	cmp	r2, r3
 800157e:	d218      	bcs.n	80015b2 <smoltcp::iface::interface::Interface::poll+0x496>
 8001580:	6a30      	ldr	r0, [r6, #32]
 8001582:	2800      	cmp	r0, #0
 8001584:	f001 86bf 	beq.w	8003306 <smoltcp::iface::interface::Interface::poll+0x21ea>
 8001588:	e9d6 0109 	ldrd	r0, r1, [r6, #36]	@ 0x24
 800158c:	2900      	cmp	r1, #0
 800158e:	60b0      	str	r0, [r6, #8]
 8001590:	f001 86c4 	beq.w	800331c <smoltcp::iface::interface::Interface::poll+0x2200>
 8001594:	6af0      	ldr	r0, [r6, #44]	@ 0x2c
 8001596:	60f0      	str	r0, [r6, #12]
 8001598:	f04f 4000 	mov.w	r0, #2147483648	@ 0x80000000
 800159c:	f3bf 8f5f 	dmb	sy
 80015a0:	6030      	str	r0, [r6, #0]
 80015a2:	f3bf 8f5f 	dmb	sy
 80015a6:	e9da 1004 	ldrd	r1, r0, [sl, #16]
 80015aa:	4288      	cmp	r0, r1
 80015ac:	f4ff ae2d 	bcc.w	800120a <smoltcp::iface::interface::Interface::poll+0xee>
 80015b0:	e613      	b.n	80011da <smoltcp::iface::interface::Interface::poll+0xbe>
 80015b2:	f8b6 1058 	ldrh.w	r1, [r6, #88]	@ 0x58
 80015b6:	ba09      	rev	r1, r1
 80015b8:	ebb3 4f11 	cmp.w	r3, r1, lsr #16
 80015bc:	d8e0      	bhi.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80015be:	0c09      	lsrs	r1, r1, #16
 80015c0:	428a      	cmp	r2, r1
 80015c2:	d3dd      	bcc.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80015c4:	f8b6 1058 	ldrh.w	r1, [r6, #88]	@ 0x58
 80015c8:	ba09      	rev	r1, r1
 80015ca:	ebb3 4f11 	cmp.w	r3, r1, lsr #16
 80015ce:	d8d7      	bhi.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80015d0:	0c09      	lsrs	r1, r1, #16
 80015d2:	428a      	cmp	r2, r1
 80015d4:	912c      	str	r1, [sp, #176]	@ 0xb0
 80015d6:	d3d3      	bcc.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80015d8:	f000 01f0 	and.w	r1, r0, #240	@ 0xf0
 80015dc:	2940      	cmp	r1, #64	@ 0x40
 80015de:	d1cf      	bne.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80015e0:	f8b6 1052 	ldrh.w	r1, [r6, #82]	@ 0x52
 80015e4:	9127      	str	r1, [sp, #156]	@ 0x9c
 80015e6:	f8d6 104e 	ldr.w	r1, [r6, #78]	@ 0x4e
 80015ea:	9128      	str	r1, [sp, #160]	@ 0xa0
 80015ec:	994a      	ldr	r1, [sp, #296]	@ 0x128
 80015ee:	7a09      	ldrb	r1, [r1, #8]
 80015f0:	2901      	cmp	r1, #1
 80015f2:	d812      	bhi.n	800161a <smoltcp::iface::interface::Interface::poll+0x4fe>
 80015f4:	0080      	lsls	r0, r0, #2
 80015f6:	9329      	str	r3, [sp, #164]	@ 0xa4
 80015f8:	b2c1      	uxtb	r1, r0
 80015fa:	428a      	cmp	r2, r1
 80015fc:	922a      	str	r2, [sp, #168]	@ 0xa8
 80015fe:	f0c1 86f2 	bcc.w	80033e6 <smoltcp::iface::interface::Interface::poll+0x22ca>
 8001602:	4628      	mov	r0, r5
 8001604:	f006 fda7 	bl	8008156 <smoltcp::wire::ip::checksum::data>
 8001608:	e9dd 3229 	ldrd	r3, r2, [sp, #164]	@ 0xa4
 800160c:	43c0      	mvns	r0, r0
 800160e:	f04f 0c01 	mov.w	ip, #1
 8001612:	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
 8001616:	0400      	lsls	r0, r0, #16
 8001618:	d1b2      	bne.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 800161a:	f8b6 005c 	ldrh.w	r0, [r6, #92]	@ 0x5c
 800161e:	f06f 01c0 	mvn.w	r1, #192	@ 0xc0
 8001622:	4208      	tst	r0, r1
 8001624:	d1ac      	bne.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 8001626:	9524      	str	r5, [sp, #144]	@ 0x90
 8001628:	922a      	str	r2, [sp, #168]	@ 0xa8
 800162a:	f896 005f 	ldrb.w	r0, [r6, #95]	@ 0x5f
 800162e:	f8d6 2066 	ldr.w	r2, [r6, #102]	@ 0x66
 8001632:	f8d6 5062 	ldr.w	r5, [r6, #98]	@ 0x62
 8001636:	283c      	cmp	r0, #60	@ 0x3c
 8001638:	962d      	str	r6, [sp, #180]	@ 0xb4
 800163a:	f04f 060c 	mov.w	r6, #12
 800163e:	f200 808c 	bhi.w	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 8001642:	e8df f000 	tbb	[pc, r0]
 8001646:	1f1f      	.short	0x1f1f
 8001648:	8a8a8a1f 	.word	0x8a8a8a1f
 800164c:	8a8a8a83 	.word	0x8a8a8a83
 8001650:	8a8a8a8a 	.word	0x8a8a8a8a
 8001654:	858a8a8a 	.word	0x858a8a8a
 8001658:	8a8a8a8a 	.word	0x8a8a8a8a
 800165c:	8a8a8a8a 	.word	0x8a8a8a8a
 8001660:	8a8a8a8a 	.word	0x8a8a8a8a
 8001664:	8a8a8a8a 	.word	0x8a8a8a8a
 8001668:	8a8a8a8a 	.word	0x8a8a8a8a
 800166c:	8a8a8a8a 	.word	0x8a8a8a8a
 8001670:	8a7f7b8a 	.word	0x8a7f7b8a
 8001674:	8a8a8a8a 	.word	0x8a8a8a8a
 8001678:	8a8a8981 	.word	0x8a8a8981
 800167c:	8a8a8a8a 	.word	0x8a8a8a8a
 8001680:	007d7987 	.word	0x007d7987
 8001684:	4606      	mov	r6, r0
 8001686:	e068      	b.n	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 8001688:	984a      	ldr	r0, [sp, #296]	@ 0x128
 800168a:	f890 016e 	ldrb.w	r0, [r0, #366]	@ 0x16e
 800168e:	2801      	cmp	r0, #1
 8001690:	d118      	bne.n	80016c4 <smoltcp::iface::interface::Interface::poll+0x5a8>
 8001692:	f5b4 7f80 	cmp.w	r4, #256	@ 0x100
 8001696:	bf18      	it	ne
 8001698:	f5b4 7f00 	cmpne.w	r4, #512	@ 0x200
 800169c:	d10c      	bne.n	80016b8 <smoltcp::iface::interface::Interface::poll+0x59c>
 800169e:	9930      	ldr	r1, [sp, #192]	@ 0xc0
 80016a0:	f001 00f0 	and.w	r0, r1, #240	@ 0xf0
 80016a4:	28e0      	cmp	r0, #224	@ 0xe0
 80016a6:	bf1c      	itt	ne
 80016a8:	1c48      	addne	r0, r1, #1
 80016aa:	2801      	cmpne	r0, #1
 80016ac:	d815      	bhi.n	80016da <smoltcp::iface::interface::Interface::poll+0x5be>
 80016ae:	f240 0026 	movw	r0, #38	@ 0x26
 80016b2:	f2c0 0000 	movt	r0, #0
 80016b6:	e003      	b.n	80016c0 <smoltcp::iface::interface::Interface::poll+0x5a4>
 80016b8:	f240 0028 	movw	r0, #40	@ 0x28
 80016bc:	f2c0 0000 	movt	r0, #0
 80016c0:	f002 fed9 	bl	8004476 <defmt::export::acquire_header_and_release>
 80016c4:	f249 0408 	movw	r4, #36872	@ 0x9008
 80016c8:	f04f 0c01 	mov.w	ip, #1
 80016cc:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 80016d0:	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
 80016d4:	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
 80016d6:	46a9      	mov	r9, r5
 80016d8:	e752      	b.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80016da:	e9dd 202e 	ldrd	r2, r0, [sp, #184]	@ 0xb8
 80016de:	ea4f 210e 	mov.w	r1, lr, lsl #8
 80016e2:	ea41 6112 	orr.w	r1, r1, r2, lsr #24
 80016e6:	ea40 2002 	orr.w	r0, r0, r2, lsl #8
 80016ea:	43c9      	mvns	r1, r1
 80016ec:	b289      	uxth	r1, r1
 80016ee:	ea61 0000 	orn	r0, r1, r0
 80016f2:	2800      	cmp	r0, #0
 80016f4:	d0db      	beq.n	80016ae <smoltcp::iface::interface::Interface::poll+0x592>
 80016f6:	982f      	ldr	r0, [sp, #188]	@ 0xbc
 80016f8:	f010 0001 	ands.w	r0, r0, #1
 80016fc:	d1d7      	bne.n	80016ae <smoltcp::iface::interface::Interface::poll+0x592>
 80016fe:	9844      	ldr	r0, [sp, #272]	@ 0x110
 8001700:	f04f 3cff 	mov.w	ip, #4294967295	@ 0xffffffff
 8001704:	4623      	mov	r3, r4
 8001706:	e00e      	b.n	8001726 <smoltcp::iface::interface::Interface::poll+0x60a>
 8001708:	4249      	negs	r1, r1
 800170a:	f001 011f 	and.w	r1, r1, #31
 800170e:	fa0c f101 	lsl.w	r1, ip, r1
 8001712:	ba0a      	rev	r2, r1
 8001714:	6801      	ldr	r1, [r0, #0]
 8001716:	3005      	adds	r0, #5
 8001718:	9c30      	ldr	r4, [sp, #192]	@ 0xc0
 800171a:	f1a9 0905 	sub.w	r9, r9, #5
 800171e:	4061      	eors	r1, r4
 8001720:	420a      	tst	r2, r1
 8001722:	f000 80f4 	beq.w	800190e <smoltcp::iface::interface::Interface::poll+0x7f2>
 8001726:	f1b9 0f00 	cmp.w	r9, #0
 800172a:	f000 80eb 	beq.w	8001904 <smoltcp::iface::interface::Interface::poll+0x7e8>
 800172e:	7901      	ldrb	r1, [r0, #4]
 8001730:	2900      	cmp	r1, #0
 8001732:	d1e9      	bne.n	8001708 <smoltcp::iface::interface::Interface::poll+0x5ec>
 8001734:	2200      	movs	r2, #0
 8001736:	e7ed      	b.n	8001714 <smoltcp::iface::interface::Interface::poll+0x5f8>
 8001738:	260a      	movs	r6, #10
 800173a:	e00e      	b.n	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 800173c:	2605      	movs	r6, #5
 800173e:	e00c      	b.n	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 8001740:	260b      	movs	r6, #11
 8001742:	e00a      	b.n	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 8001744:	2606      	movs	r6, #6
 8001746:	e008      	b.n	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 8001748:	2607      	movs	r6, #7
 800174a:	e006      	b.n	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 800174c:	2603      	movs	r6, #3
 800174e:	e004      	b.n	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 8001750:	2604      	movs	r6, #4
 8001752:	e002      	b.n	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 8001754:	2609      	movs	r6, #9
 8001756:	e000      	b.n	800175a <smoltcp::iface::interface::Interface::poll+0x63e>
 8001758:	2608      	movs	r6, #8
 800175a:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 800175c:	9329      	str	r3, [sp, #164]	@ 0xa4
 800175e:	1ac9      	subs	r1, r1, r3
 8001760:	9123      	str	r1, [sp, #140]	@ 0x8c
 8001762:	992d      	ldr	r1, [sp, #180]	@ 0xb4
 8001764:	922b      	str	r2, [sp, #172]	@ 0xac
 8001766:	f891 305e 	ldrb.w	r3, [r1, #94]	@ 0x5e
 800176a:	9926      	ldr	r1, [sp, #152]	@ 0x98
 800176c:	f021 01ff 	bic.w	r1, r1, #255	@ 0xff
 8001770:	4401      	add	r1, r0
 8001772:	f005 00f0 	and.w	r0, r5, #240	@ 0xf0
 8001776:	28e0      	cmp	r0, #224	@ 0xe0
 8001778:	e9cd 5125 	strd	r5, r1, [sp, #148]	@ 0x94
 800177c:	bf18      	it	ne
 800177e:	f115 0001 	addsne.w	r0, r5, #1
 8001782:	d10f      	bne.n	80017a4 <smoltcp::iface::interface::Interface::poll+0x688>
 8001784:	f240 0029 	movw	r0, #41	@ 0x29
 8001788:	4664      	mov	r4, ip
 800178a:	f2c0 0000 	movt	r0, #0
 800178e:	4675      	mov	r5, lr
 8001790:	f002 fe71 	bl	8004476 <defmt::export::acquire_header_and_release>
 8001794:	46a4      	mov	ip, r4
 8001796:	f249 0408 	movw	r4, #36872	@ 0x9008
 800179a:	46ae      	mov	lr, r5
 800179c:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 80017a0:	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
 80017a2:	e6ed      	b.n	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 80017a4:	e9cd 3621 	strd	r3, r6, [sp, #132]	@ 0x84
 80017a8:	b375      	cbz	r5, 8001808 <smoltcp::iface::interface::Interface::poll+0x6ec>
 80017aa:	984a      	ldr	r0, [sp, #296]	@ 0x128
 80017ac:	f8d0 00f0 	ldr.w	r0, [r0, #240]	@ 0xf0
 80017b0:	b330      	cbz	r0, 8001800 <smoltcp::iface::interface::Interface::poll+0x6e4>
 80017b2:	9944      	ldr	r1, [sp, #272]	@ 0x110
 80017b4:	eb00 0080 	add.w	r0, r0, r0, lsl #2
 80017b8:	4408      	add	r0, r1
 80017ba:	e00f      	b.n	80017dc <smoltcp::iface::interface::Interface::poll+0x6c0>
 80017bc:	2600      	movs	r6, #0
 80017be:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 80017c2:	f002 021f 	and.w	r2, r2, #31
 80017c6:	4033      	ands	r3, r6
 80017c8:	fa24 f202 	lsr.w	r2, r4, r2
 80017cc:	ba12      	rev	r2, r2
 80017ce:	431a      	orrs	r2, r3
 80017d0:	4295      	cmp	r5, r2
 80017d2:	f000 8091 	beq.w	80018f8 <smoltcp::iface::interface::Interface::poll+0x7dc>
 80017d6:	3105      	adds	r1, #5
 80017d8:	4281      	cmp	r1, r0
 80017da:	d011      	beq.n	8001800 <smoltcp::iface::interface::Interface::poll+0x6e4>
 80017dc:	790a      	ldrb	r2, [r1, #4]
 80017de:	680b      	ldr	r3, [r1, #0]
 80017e0:	2a00      	cmp	r2, #0
 80017e2:	d0eb      	beq.n	80017bc <smoltcp::iface::interface::Interface::poll+0x6a0>
 80017e4:	f1a2 061f 	sub.w	r6, r2, #31
 80017e8:	b2f6      	uxtb	r6, r6
 80017ea:	2e02      	cmp	r6, #2
 80017ec:	d3f3      	bcc.n	80017d6 <smoltcp::iface::interface::Interface::poll+0x6ba>
 80017ee:	4256      	negs	r6, r2
 80017f0:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 80017f4:	f006 061f 	and.w	r6, r6, #31
 80017f8:	fa04 f606 	lsl.w	r6, r4, r6
 80017fc:	ba36      	rev	r6, r6
 80017fe:	e7e0      	b.n	80017c2 <smoltcp::iface::interface::Interface::poll+0x6a6>
 8001800:	fab5 f085 	clz	r0, r5
 8001804:	ea4f 1c50 	mov.w	ip, r0, lsr #5
 8001808:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 800180a:	9c29      	ldr	r4, [sp, #164]	@ 0xa4
 800180c:	9a2a      	ldr	r2, [sp, #168]	@ 0xa8
 800180e:	f8b0 0058 	ldrh.w	r0, [r0, #88]	@ 0x58
 8001812:	9b24      	ldr	r3, [sp, #144]	@ 0x90
 8001814:	ba00      	rev	r0, r0
 8001816:	0c01      	lsrs	r1, r0, #16
 8001818:	ebb4 4f10 	cmp.w	r4, r0, lsr #16
 800181c:	f201 8624 	bhi.w	8003468 <smoltcp::iface::interface::Interface::poll+0x234c>
 8001820:	428a      	cmp	r2, r1
 8001822:	f0c1 8621 	bcc.w	8003468 <smoltcp::iface::interface::Interface::poll+0x234c>
 8001826:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8001828:	191e      	adds	r6, r3, r4
 800182a:	9b44      	ldr	r3, [sp, #272]	@ 0x110
 800182c:	e9cd 6629 	strd	r6, r6, [sp, #164]	@ 0xa4
 8001830:	f8d0 e0f0 	ldr.w	lr, [r0, #240]	@ 0xf0
 8001834:	1b08      	subs	r0, r1, r4
 8001836:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 800183a:	902c      	str	r0, [sp, #176]	@ 0xb0
 800183c:	f8cd c080 	str.w	ip, [sp, #128]	@ 0x80
 8001840:	eb0e 028e 	add.w	r2, lr, lr, lsl #2
 8001844:	1899      	adds	r1, r3, r2
 8001846:	b322      	cbz	r2, 8001892 <smoltcp::iface::interface::Interface::poll+0x776>
 8001848:	4618      	mov	r0, r3
 800184a:	9d2b      	ldr	r5, [sp, #172]	@ 0xac
 800184c:	f850 6b05 	ldr.w	r6, [r0], #5
 8001850:	42ae      	cmp	r6, r5
 8001852:	f000 8126 	beq.w	8001aa2 <smoltcp::iface::interface::Interface::poll+0x986>
 8001856:	4288      	cmp	r0, r1
 8001858:	d01b      	beq.n	8001892 <smoltcp::iface::interface::Interface::poll+0x776>
 800185a:	f8d3 0005 	ldr.w	r0, [r3, #5]
 800185e:	9e2b      	ldr	r6, [sp, #172]	@ 0xac
 8001860:	42b0      	cmp	r0, r6
 8001862:	f000 811e 	beq.w	8001aa2 <smoltcp::iface::interface::Interface::poll+0x986>
 8001866:	f103 000a 	add.w	r0, r3, #10
 800186a:	4288      	cmp	r0, r1
 800186c:	d011      	beq.n	8001892 <smoltcp::iface::interface::Interface::poll+0x776>
 800186e:	f8d3 000a 	ldr.w	r0, [r3, #10]
 8001872:	9e2b      	ldr	r6, [sp, #172]	@ 0xac
 8001874:	42b0      	cmp	r0, r6
 8001876:	f000 8114 	beq.w	8001aa2 <smoltcp::iface::interface::Interface::poll+0x986>
 800187a:	f103 000f 	add.w	r0, r3, #15
 800187e:	4288      	cmp	r0, r1
 8001880:	d007      	beq.n	8001892 <smoltcp::iface::interface::Interface::poll+0x776>
 8001882:	f8d3 000f 	ldr.w	r0, [r3, #15]
 8001886:	3a14      	subs	r2, #20
 8001888:	9e2b      	ldr	r6, [sp, #172]	@ 0xac
 800188a:	3314      	adds	r3, #20
 800188c:	42b0      	cmp	r0, r6
 800188e:	d1da      	bne.n	8001846 <smoltcp::iface::interface::Interface::poll+0x72a>
 8001890:	e107      	b.n	8001aa2 <smoltcp::iface::interface::Interface::poll+0x986>
 8001892:	982b      	ldr	r0, [sp, #172]	@ 0xac
 8001894:	9d25      	ldr	r5, [sp, #148]	@ 0x94
 8001896:	f110 0c01 	adds.w	ip, r0, #1
 800189a:	f000 810d 	beq.w	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 800189e:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 80018a0:	20e0      	movs	r0, #224	@ 0xe0
 80018a2:	f2c0 1000 	movt	r0, #256	@ 0x100
 80018a6:	4282      	cmp	r2, r0
 80018a8:	f000 8106 	beq.w	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 80018ac:	f1be 0f00 	cmp.w	lr, #0
 80018b0:	f000 80d4 	beq.w	8001a5c <smoltcp::iface::interface::Interface::poll+0x940>
 80018b4:	9b44      	ldr	r3, [sp, #272]	@ 0x110
 80018b6:	e00f      	b.n	80018d8 <smoltcp::iface::interface::Interface::poll+0x7bc>
 80018b8:	2000      	movs	r0, #0
 80018ba:	4010      	ands	r0, r2
 80018bc:	f006 021f 	and.w	r2, r6, #31
 80018c0:	fa24 f202 	lsr.w	r2, r4, r2
 80018c4:	ba12      	rev	r2, r2
 80018c6:	4310      	orrs	r0, r2
 80018c8:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 80018ca:	4282      	cmp	r2, r0
 80018cc:	f000 80e3 	beq.w	8001a96 <smoltcp::iface::interface::Interface::poll+0x97a>
 80018d0:	3305      	adds	r3, #5
 80018d2:	428b      	cmp	r3, r1
 80018d4:	f000 80c2 	beq.w	8001a5c <smoltcp::iface::interface::Interface::poll+0x940>
 80018d8:	791e      	ldrb	r6, [r3, #4]
 80018da:	681a      	ldr	r2, [r3, #0]
 80018dc:	2e00      	cmp	r6, #0
 80018de:	d0eb      	beq.n	80018b8 <smoltcp::iface::interface::Interface::poll+0x79c>
 80018e0:	f1a6 001f 	sub.w	r0, r6, #31
 80018e4:	b2c0      	uxtb	r0, r0
 80018e6:	2802      	cmp	r0, #2
 80018e8:	d3f2      	bcc.n	80018d0 <smoltcp::iface::interface::Interface::poll+0x7b4>
 80018ea:	4270      	negs	r0, r6
 80018ec:	f000 001f 	and.w	r0, r0, #31
 80018f0:	fa04 f000 	lsl.w	r0, r4, r0
 80018f4:	ba00      	rev	r0, r0
 80018f6:	e7e0      	b.n	80018ba <smoltcp::iface::interface::Interface::poll+0x79e>
 80018f8:	2d00      	cmp	r5, #0
 80018fa:	f47f af43 	bne.w	8001784 <smoltcp::iface::interface::Interface::poll+0x668>
 80018fe:	f04f 0c01 	mov.w	ip, #1
 8001902:	e781      	b.n	8001808 <smoltcp::iface::interface::Interface::poll+0x6ec>
 8001904:	f240 0027 	movw	r0, #39	@ 0x27
 8001908:	f2c0 0000 	movt	r0, #0
 800190c:	e6d8      	b.n	80016c0 <smoltcp::iface::interface::Interface::poll+0x5a4>
 800190e:	983f      	ldr	r0, [sp, #252]	@ 0xfc
 8001910:	461c      	mov	r4, r3
 8001912:	992e      	ldr	r1, [sp, #184]	@ 0xb8
 8001914:	46b3      	mov	fp, r6
 8001916:	9b2a      	ldr	r3, [sp, #168]	@ 0xa8
 8001918:	4675      	mov	r5, lr
 800191a:	6001      	str	r1, [r0, #0]
 800191c:	f880 e004 	strb.w	lr, [r0, #4]
 8001920:	0e09      	lsrs	r1, r1, #24
 8001922:	982f      	ldr	r0, [sp, #188]	@ 0xbc
 8001924:	f88d 02f8 	strb.w	r0, [sp, #760]	@ 0x2f8
 8001928:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 800192a:	e9cd 0300 	strd	r0, r3, [sp]
 800192e:	fa5f f08e 	uxtb.w	r0, lr
 8001932:	ea41 2300 	orr.w	r3, r1, r0, lsl #8
 8001936:	9abe      	ldr	r2, [sp, #760]	@ 0x2f8
 8001938:	983e      	ldr	r0, [sp, #248]	@ 0xf8
 800193a:	9930      	ldr	r1, [sp, #192]	@ 0xc0
 800193c:	f006 fe10 	bl	8008560 <smoltcp::iface::neighbor::Cache::fill>
 8001940:	f5b4 7f80 	cmp.w	r4, #256	@ 0x100
 8001944:	f040 8098 	bne.w	8001a78 <smoltcp::iface::interface::Interface::poll+0x95c>
 8001948:	9946      	ldr	r1, [sp, #280]	@ 0x118
 800194a:	9a3a      	ldr	r2, [sp, #232]	@ 0xe8
 800194c:	f8cd b29e 	str.w	fp, [sp, #670]	@ 0x29e
 8001950:	6808      	ldr	r0, [r1, #0]
 8001952:	8889      	ldrh	r1, [r1, #4]
 8001954:	8091      	strh	r1, [r2, #4]
 8001956:	6010      	str	r0, [r2, #0]
 8001958:	9a38      	ldr	r2, [sp, #224]	@ 0xe0
 800195a:	e9dd 0154 	ldrd	r0, r1, [sp, #336]	@ 0x150
 800195e:	6010      	str	r0, [r2, #0]
 8001960:	9839      	ldr	r0, [sp, #228]	@ 0xe4
 8001962:	6051      	str	r1, [r2, #4]
 8001964:	992e      	ldr	r1, [sp, #184]	@ 0xb8
 8001966:	6001      	str	r1, [r0, #0]
 8001968:	e9da 1404 	ldrd	r1, r4, [sl, #16]
 800196c:	7105      	strb	r5, [r0, #4]
 800196e:	2001      	movs	r0, #1
 8001970:	f8ad 0294 	strh.w	r0, [sp, #660]	@ 0x294
 8001974:	428c      	cmp	r4, r1
 8001976:	982f      	ldr	r0, [sp, #188]	@ 0xbc
 8001978:	f88d 02a2 	strb.w	r0, [sp, #674]	@ 0x2a2
 800197c:	9830      	ldr	r0, [sp, #192]	@ 0xc0
 800197e:	90aa      	str	r0, [sp, #680]	@ 0x2a8
 8001980:	f081 8582 	bcs.w	8003488 <smoltcp::iface::interface::Interface::poll+0x236c>
 8001984:	f44f 60c6 	mov.w	r0, #1584	@ 0x630
 8001988:	fb04 f600 	mul.w	r6, r4, r0
 800198c:	9843      	ldr	r0, [sp, #268]	@ 0x10c
 800198e:	6800      	ldr	r0, [r0, #0]
 8001990:	5982      	ldr	r2, [r0, r6]
 8001992:	2a00      	cmp	r2, #0
 8001994:	f101 857f 	bmi.w	8003496 <smoltcp::iface::interface::Interface::poll+0x237a>
 8001998:	1c62      	adds	r2, r4, #1
 800199a:	f8d7 8008 	ldr.w	r8, [r7, #8]
 800199e:	fbb2 f3f1 	udiv	r3, r2, r1
 80019a2:	fb03 2111 	mls	r1, r3, r1, r2
 80019a6:	f44f 62c6 	mov.w	r2, #1584	@ 0x630
 80019aa:	9b2e      	ldr	r3, [sp, #184]	@ 0xb8
 80019ac:	fb04 0002 	mla	r0, r4, r2, r0
 80019b0:	9a46      	ldr	r2, [sp, #280]	@ 0x118
 80019b2:	f8c8 1014 	str.w	r1, [r8, #20]
 80019b6:	6811      	ldr	r1, [r2, #0]
 80019b8:	8892      	ldrh	r2, [r2, #4]
 80019ba:	f8c0 103e 	str.w	r1, [r0, #62]	@ 0x3e
 80019be:	f44f 61c1 	mov.w	r1, #1544	@ 0x608
 80019c2:	f8a0 1044 	strh.w	r1, [r0, #68]	@ 0x44
 80019c6:	992f      	ldr	r1, [sp, #188]	@ 0xbc
 80019c8:	f8a0 2042 	strh.w	r2, [r0, #66]	@ 0x42
 80019cc:	221c      	movs	r2, #28
 80019ce:	f880 1038 	strb.w	r1, [r0, #56]	@ 0x38
 80019d2:	f100 0146 	add.w	r1, r0, #70	@ 0x46
 80019d6:	f880 503d 	strb.w	r5, [r0, #61]	@ 0x3d
 80019da:	f8c0 3039 	str.w	r3, [r0, #57]	@ 0x39
 80019de:	a8a5      	add	r0, sp, #660	@ 0x294
 80019e0:	f7fe fc36 	bl	8000250 <smoltcp::wire::arp::Repr::emit>
 80019e4:	f8d8 1010 	ldr.w	r1, [r8, #16]
 80019e8:	428c      	cmp	r4, r1
 80019ea:	f081 855f 	bcs.w	80034ac <smoltcp::iface::interface::Interface::poll+0x2390>
 80019ee:	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
 80019f2:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 80019f6:	f04f 0900 	mov.w	r9, #0
 80019fa:	f8d7 a008 	ldr.w	sl, [r7, #8]
 80019fe:	f8d8 0000 	ldr.w	r0, [r8]
 8001a02:	fb04 0101 	mla	r1, r4, r1, r0
 8001a06:	242a      	movs	r4, #42	@ 0x2a
 8001a08:	684a      	ldr	r2, [r1, #4]
 8001a0a:	e9d1 3c0a 	ldrd	r3, ip, [r1, #40]	@ 0x28
 8001a0e:	f364 020b 	bfi	r2, r4, #0, #12
 8001a12:	604a      	str	r2, [r1, #4]
 8001a14:	608b      	str	r3, [r1, #8]
 8001a16:	2200      	movs	r2, #0
 8001a18:	f8c1 9020 	str.w	r9, [r1, #32]
 8001a1c:	f249 0408 	movw	r4, #36872	@ 0x9008
 8001a20:	f8c1 c00c 	str.w	ip, [r1, #12]
 8001a24:	f2cf 02d0 	movt	r2, #61648	@ 0xf0d0
 8001a28:	f3bf 8f5f 	dmb	sy
 8001a2c:	f891 1030 	ldrb.w	r1, [r1, #48]	@ 0x30
 8001a30:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 8001a34:	ea42 5141 	orr.w	r1, r2, r1, lsl #21
 8001a38:	5181      	str	r1, [r0, r6]
 8001a3a:	f3bf 8f5f 	dmb	sy
 8001a3e:	2001      	movs	r0, #1
 8001a40:	f844 9c04 	str.w	r9, [r4, #-4]
 8001a44:	901c      	str	r0, [sp, #112]	@ 0x70
 8001a46:	9830      	ldr	r0, [sp, #192]	@ 0xc0
 8001a48:	901b      	str	r0, [sp, #108]	@ 0x6c
 8001a4a:	982e      	ldr	r0, [sp, #184]	@ 0xb8
 8001a4c:	f8cd b074 	str.w	fp, [sp, #116]	@ 0x74
 8001a50:	e9cd 501e 	strd	r5, r0, [sp, #120]	@ 0x78
 8001a54:	982f      	ldr	r0, [sp, #188]	@ 0xbc
 8001a56:	9045      	str	r0, [sp, #276]	@ 0x114
 8001a58:	9849      	ldr	r0, [sp, #292]	@ 0x124
 8001a5a:	e014      	b.n	8001a86 <smoltcp::iface::interface::Interface::poll+0x96a>
 8001a5c:	982b      	ldr	r0, [sp, #172]	@ 0xac
 8001a5e:	f000 00f0 	and.w	r0, r0, #240	@ 0xf0
 8001a62:	28e0      	cmp	r0, #224	@ 0xe0
 8001a64:	bf1e      	ittt	ne
 8001a66:	984a      	ldrne	r0, [sp, #296]	@ 0x128
 8001a68:	f890 016e 	ldrbne.w	r0, [r0, #366]	@ 0x16e
 8001a6c:	ea5f 70c0 	movsne.w	r0, r0, lsl #31
 8001a70:	f040 81e2 	bne.w	8001e38 <smoltcp::iface::interface::Interface::poll+0xd1c>
 8001a74:	f8d7 a008 	ldr.w	sl, [r7, #8]
 8001a78:	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
 8001a7c:	f249 0408 	movw	r4, #36872	@ 0x9008
 8001a80:	9849      	ldr	r0, [sp, #292]	@ 0x124
 8001a82:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 8001a86:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 8001a8a:	f04f 0c01 	mov.w	ip, #1
 8001a8e:	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
 8001a92:	4683      	mov	fp, r0
 8001a94:	e684      	b.n	80017a0 <smoltcp::iface::interface::Interface::poll+0x684>
 8001a96:	f8d7 a008 	ldr.w	sl, [r7, #8]
 8001a9a:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 8001a9e:	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
 8001aa2:	982b      	ldr	r0, [sp, #172]	@ 0xac
 8001aa4:	9d25      	ldr	r5, [sp, #148]	@ 0x94
 8001aa6:	f000 00f0 	and.w	r0, r0, #240	@ 0xf0
 8001aaa:	28e0      	cmp	r0, #224	@ 0xe0
 8001aac:	d004      	beq.n	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 8001aae:	982b      	ldr	r0, [sp, #172]	@ 0xac
 8001ab0:	3001      	adds	r0, #1
 8001ab2:	2802      	cmp	r0, #2
 8001ab4:	f080 816c 	bcs.w	8001d90 <smoltcp::iface::interface::Interface::poll+0xc74>
 8001ab8:	9922      	ldr	r1, [sp, #136]	@ 0x88
 8001aba:	2904      	cmp	r1, #4
 8001abc:	f000 80b8 	beq.w	8001c30 <smoltcp::iface::interface::Interface::poll+0xb14>
 8001ac0:	2903      	cmp	r1, #3
 8001ac2:	d052      	beq.n	8001b6a <smoltcp::iface::interface::Interface::poll+0xa4e>
 8001ac4:	2901      	cmp	r1, #1
 8001ac6:	f040 80cb 	bne.w	8001c60 <smoltcp::iface::interface::Interface::poll+0xb44>
 8001aca:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001acc:	9929      	ldr	r1, [sp, #164]	@ 0xa4
 8001ace:	2808      	cmp	r0, #8
 8001ad0:	bf38      	it	cc
 8001ad2:	2100      	movcc	r1, #0
 8001ad4:	2807      	cmp	r0, #7
 8001ad6:	9129      	str	r1, [sp, #164]	@ 0xa4
 8001ad8:	f240 80b9 	bls.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001adc:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8001ade:	7ac0      	ldrb	r0, [r0, #11]
 8001ae0:	2801      	cmp	r0, #1
 8001ae2:	d807      	bhi.n	8001af4 <smoltcp::iface::interface::Interface::poll+0x9d8>
 8001ae4:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001ae6:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 8001ae8:	f006 fb35 	bl	8008156 <smoltcp::wire::ip::checksum::data>
 8001aec:	43c0      	mvns	r0, r0
 8001aee:	0400      	lsls	r0, r0, #16
 8001af0:	f040 80ad 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001af4:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001af6:	7800      	ldrb	r0, [r0, #0]
 8001af8:	280b      	cmp	r0, #11
 8001afa:	f200 80a8 	bhi.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001afe:	e8df f010 	tbh	[pc, r0, lsl #1]
 8001b02:	026e      	.short	0x026e
 8001b04:	00a600a6 	.word	0x00a600a6
 8001b08:	00a6000c 	.word	0x00a6000c
 8001b0c:	00a600a6 	.word	0x00a600a6
 8001b10:	028100a6 	.word	0x028100a6
 8001b14:	00a600a6 	.word	0x00a600a6
 8001b18:	000c      	.short	0x000c
 8001b1a:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001b1c:	f1a0 0108 	sub.w	r1, r0, #8
 8001b20:	2914      	cmp	r1, #20
 8001b22:	f0c0 8094 	bcc.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001b26:	9829      	ldr	r0, [sp, #164]	@ 0xa4
 8001b28:	233c      	movs	r3, #60	@ 0x3c
 8001b2a:	3008      	adds	r0, #8
 8001b2c:	7802      	ldrb	r2, [r0, #0]
 8001b2e:	ea03 0282 	and.w	r2, r3, r2, lsl #2
 8001b32:	4291      	cmp	r1, r2
 8001b34:	f0c0 808b 	bcc.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001b38:	9b2a      	ldr	r3, [sp, #168]	@ 0xa8
 8001b3a:	895b      	ldrh	r3, [r3, #10]
 8001b3c:	ba1b      	rev	r3, r3
 8001b3e:	ebb2 4f13 	cmp.w	r2, r3, lsr #16
 8001b42:	f200 8084 	bhi.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001b46:	0c1b      	lsrs	r3, r3, #16
 8001b48:	4299      	cmp	r1, r3
 8001b4a:	bf24      	itt	cs
 8001b4c:	1a89      	subcs	r1, r1, r2
 8001b4e:	2908      	cmpcs	r1, #8
 8001b50:	d37d      	bcc.n	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001b52:	992a      	ldr	r1, [sp, #168]	@ 0xa8
 8001b54:	4410      	add	r0, r2
 8001b56:	901a      	str	r0, [sp, #104]	@ 0x68
 8001b58:	2004      	movs	r0, #4
 8001b5a:	9057      	str	r0, [sp, #348]	@ 0x15c
 8001b5c:	9817      	ldr	r0, [sp, #92]	@ 0x5c
 8001b5e:	7849      	ldrb	r1, [r1, #1]
 8001b60:	f020 00ff 	bic.w	r0, r0, #255	@ 0xff
 8001b64:	4408      	add	r0, r1
 8001b66:	9017      	str	r0, [sp, #92]	@ 0x5c
 8001b68:	e073      	b.n	8001c52 <smoltcp::iface::interface::Interface::poll+0xb36>
 8001b6a:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001b6c:	2814      	cmp	r0, #20
 8001b6e:	d36e      	bcc.n	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001b70:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001b72:	213c      	movs	r1, #60	@ 0x3c
 8001b74:	8986      	ldrh	r6, [r0, #12]
 8001b76:	b2f0      	uxtb	r0, r6
 8001b78:	2850      	cmp	r0, #80	@ 0x50
 8001b7a:	f04f 0000 	mov.w	r0, #0
 8001b7e:	ea01 0596 	and.w	r5, r1, r6, lsr #2
 8001b82:	bf38      	it	cc
 8001b84:	2001      	movcc	r0, #1
 8001b86:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 8001b88:	42a9      	cmp	r1, r5
 8001b8a:	f04f 0100 	mov.w	r1, #0
 8001b8e:	bf38      	it	cc
 8001b90:	2101      	movcc	r1, #1
 8001b92:	4308      	orrs	r0, r1
 8001b94:	2801      	cmp	r0, #1
 8001b96:	d05a      	beq.n	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001b98:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001b9a:	8800      	ldrh	r0, [r0, #0]
 8001b9c:	2800      	cmp	r0, #0
 8001b9e:	bf1e      	ittt	ne
 8001ba0:	982a      	ldrne	r0, [sp, #168]	@ 0xa8
 8001ba2:	8840      	ldrhne	r0, [r0, #2]
 8001ba4:	2800      	cmpne	r0, #0
 8001ba6:	d052      	beq.n	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001ba8:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8001baa:	7a80      	ldrb	r0, [r0, #10]
 8001bac:	2801      	cmp	r0, #1
 8001bae:	d832      	bhi.n	8001c16 <smoltcp::iface::interface::Interface::poll+0xafa>
 8001bb0:	982b      	ldr	r0, [sp, #172]	@ 0xac
 8001bb2:	ba00      	rev	r0, r0
 8001bb4:	0c01      	lsrs	r1, r0, #16
 8001bb6:	fa11 f080 	uxtah	r0, r1, r0
 8001bba:	0c01      	lsrs	r1, r0, #16
 8001bbc:	fa11 f080 	uxtah	r0, r1, r0
 8001bc0:	9925      	ldr	r1, [sp, #148]	@ 0x94
 8001bc2:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001bc6:	ba09      	rev	r1, r1
 8001bc8:	b280      	uxth	r0, r0
 8001bca:	0c0a      	lsrs	r2, r1, #16
 8001bcc:	fa12 f181 	uxtah	r1, r2, r1
 8001bd0:	0c0a      	lsrs	r2, r1, #16
 8001bd2:	fa12 f181 	uxtah	r1, r2, r1
 8001bd6:	eb01 4111 	add.w	r1, r1, r1, lsr #16
 8001bda:	fa10 f081 	uxtah	r0, r0, r1
 8001bde:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 8001be0:	1d8b      	adds	r3, r1, #6
 8001be2:	0c1a      	lsrs	r2, r3, #16
 8001be4:	fa12 f283 	uxtah	r2, r2, r3
 8001be8:	eb02 4212 	add.w	r2, r2, r2, lsr #16
 8001bec:	fa10 f082 	uxtah	r0, r0, r2
 8001bf0:	0c02      	lsrs	r2, r0, #16
 8001bf2:	fa12 f080 	uxtah	r0, r2, r0
 8001bf6:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001bfa:	b284      	uxth	r4, r0
 8001bfc:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001bfe:	f006 faaa 	bl	8008156 <smoltcp::wire::ip::checksum::data>
 8001c02:	fa14 f080 	uxtah	r0, r4, r0
 8001c06:	0c01      	lsrs	r1, r0, #16
 8001c08:	fa11 f080 	uxtah	r0, r1, r0
 8001c0c:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001c10:	43c0      	mvns	r0, r0
 8001c12:	0400      	lsls	r0, r0, #16
 8001c14:	d11b      	bne.n	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001c16:	05b0      	lsls	r0, r6, #22
 8001c18:	f100 8270 	bmi.w	80020fc <smoltcp::iface::interface::Interface::poll+0xfe0>
 8001c1c:	05f0      	lsls	r0, r6, #23
 8001c1e:	f100 8273 	bmi.w	8002108 <smoltcp::iface::interface::Interface::poll+0xfec>
 8001c22:	0570      	lsls	r0, r6, #21
 8001c24:	f04f 0004 	mov.w	r0, #4
 8001c28:	bf58      	it	pl
 8001c2a:	f3c6 20c0 	ubfxpl	r0, r6, #11, #1
 8001c2e:	e26f      	b.n	8002110 <smoltcp::iface::interface::Interface::poll+0xff4>
 8001c30:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001c32:	2808      	cmp	r0, #8
 8001c34:	d30b      	bcc.n	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001c36:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001c38:	8880      	ldrh	r0, [r0, #4]
 8001c3a:	ba00      	rev	r0, r0
 8001c3c:	ea4f 4910 	mov.w	r9, r0, lsr #16
 8001c40:	f1b9 0f08 	cmp.w	r9, #8
 8001c44:	bf24      	itt	cs
 8001c46:	982c      	ldrcs	r0, [sp, #176]	@ 0xb0
 8001c48:	4548      	cmpcs	r0, r9
 8001c4a:	f080 813d 	bcs.w	8001ec8 <smoltcp::iface::interface::Interface::poll+0xdac>
 8001c4e:	2004      	movs	r0, #4
 8001c50:	9057      	str	r0, [sp, #348]	@ 0x15c
 8001c52:	f8d7 a008 	ldr.w	sl, [r7, #8]
 8001c56:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 8001c5a:	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
 8001c5e:	e021      	b.n	8001ca4 <smoltcp::iface::interface::Interface::poll+0xb88>
 8001c60:	9826      	ldr	r0, [sp, #152]	@ 0x98
 8001c62:	462a      	mov	r2, r5
 8001c64:	f8ad 01da 	strh.w	r0, [sp, #474]	@ 0x1da
 8001c68:	9821      	ldr	r0, [sp, #132]	@ 0x84
 8001c6a:	f88d 01d8 	strb.w	r0, [sp, #472]	@ 0x1d8
 8001c6e:	9823      	ldr	r0, [sp, #140]	@ 0x8c
 8001c70:	9b2b      	ldr	r3, [sp, #172]	@ 0xac
 8001c72:	f88d 11d9 	strb.w	r1, [sp, #473]	@ 0x1d9
 8001c76:	9075      	str	r0, [sp, #468]	@ 0x1d4
 8001c78:	f240 2002 	movw	r0, #514	@ 0x202
 8001c7c:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 8001c7e:	f8ad 01c0 	strh.w	r0, [sp, #448]	@ 0x1c0
 8001c82:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001c84:	f5b1 7f04 	cmp.w	r1, #528	@ 0x210
 8001c88:	9071      	str	r0, [sp, #452]	@ 0x1c4
 8001c8a:	f44f 7004 	mov.w	r0, #528	@ 0x210
 8001c8e:	e9cd 5373 	strd	r5, r3, [sp, #460]	@ 0x1cc
 8001c92:	bf28      	it	cs
 8001c94:	4601      	movcs	r1, r0
 8001c96:	9172      	str	r1, [sp, #456]	@ 0x1c8
 8001c98:	a870      	add	r0, sp, #448	@ 0x1c0
 8001c9a:	994a      	ldr	r1, [sp, #296]	@ 0x128
 8001c9c:	9000      	str	r0, [sp, #0]
 8001c9e:	a857      	add	r0, sp, #348	@ 0x15c
 8001ca0:	f005 ff05 	bl	8007aae <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>
 8001ca4:	9c57      	ldr	r4, [sp, #348]	@ 0x15c
 8001ca6:	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
 8001ca8:	2c04      	cmp	r4, #4
 8001caa:	d10b      	bne.n	8001cc4 <smoltcp::iface::interface::Interface::poll+0xba8>
 8001cac:	f249 0408 	movw	r4, #36872	@ 0x9008
 8001cb0:	f04f 0c01 	mov.w	ip, #1
 8001cb4:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 8001cb8:	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
 8001cbc:	f8dd b124 	ldr.w	fp, [sp, #292]	@ 0x124
 8001cc0:	f7ff bc5e 	b.w	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 8001cc4:	9940      	ldr	r1, [sp, #256]	@ 0x100
 8001cc6:	f8dd 016a 	ldr.w	r0, [sp, #362]	@ 0x16a
 8001cca:	901d      	str	r0, [sp, #116]	@ 0x74
 8001ccc:	f89d 016e 	ldrb.w	r0, [sp, #366]	@ 0x16e
 8001cd0:	9045      	str	r0, [sp, #276]	@ 0x114
 8001cd2:	6808      	ldr	r0, [r1, #0]
 8001cd4:	6849      	ldr	r1, [r1, #4]
 8001cd6:	9a41      	ldr	r2, [sp, #260]	@ 0x104
 8001cd8:	f8bd 5160 	ldrh.w	r5, [sp, #352]	@ 0x160
 8001cdc:	e9cd 0154 	strd	r0, r1, [sp, #336]	@ 0x150
 8001ce0:	a8a5      	add	r0, sp, #660	@ 0x294
 8001ce2:	9942      	ldr	r1, [sp, #264]	@ 0x108
 8001ce4:	7913      	ldrb	r3, [r2, #4]
 8001ce6:	6812      	ldr	r2, [r2, #0]
 8001ce8:	921f      	str	r2, [sp, #124]	@ 0x7c
 8001cea:	2248      	movs	r2, #72	@ 0x48
 8001cec:	931e      	str	r3, [sp, #120]	@ 0x78
 8001cee:	9e5d      	ldr	r6, [sp, #372]	@ 0x174
 8001cf0:	f006 fe58 	bl	80089a4 <__aeabi_memcpy4>
 8001cf4:	2c05      	cmp	r4, #5
 8001cf6:	e9cd 651b 	strd	r6, r5, [sp, #108]	@ 0x6c
 8001cfa:	d03c      	beq.n	8001d76 <smoltcp::iface::interface::Interface::poll+0xc5a>
 8001cfc:	9a3b      	ldr	r2, [sp, #236]	@ 0xec
 8001cfe:	e9dd 0154 	ldrd	r0, r1, [sp, #336]	@ 0x150
 8001d02:	f8ad 52fc 	strh.w	r5, [sp, #764]	@ 0x2fc
 8001d06:	6010      	str	r0, [r2, #0]
 8001d08:	983c      	ldr	r0, [sp, #240]	@ 0xf0
 8001d0a:	6051      	str	r1, [r2, #4]
 8001d0c:	2248      	movs	r2, #72	@ 0x48
 8001d0e:	991f      	ldr	r1, [sp, #124]	@ 0x7c
 8001d10:	6001      	str	r1, [r0, #0]
 8001d12:	991e      	ldr	r1, [sp, #120]	@ 0x78
 8001d14:	7101      	strb	r1, [r0, #4]
 8001d16:	a9a5      	add	r1, sp, #660	@ 0x294
 8001d18:	9845      	ldr	r0, [sp, #276]	@ 0x114
 8001d1a:	f88d 030a 	strb.w	r0, [sp, #778]	@ 0x30a
 8001d1e:	981d      	ldr	r0, [sp, #116]	@ 0x74
 8001d20:	f8cd 0306 	str.w	r0, [sp, #774]	@ 0x306
 8001d24:	983d      	ldr	r0, [sp, #244]	@ 0xf4
 8001d26:	94be      	str	r4, [sp, #760]	@ 0x2f8
 8001d28:	96c4      	str	r6, [sp, #784]	@ 0x310
 8001d2a:	f006 fe3b 	bl	80089a4 <__aeabi_memcpy4>
 8001d2e:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8001d30:	a94c      	add	r1, sp, #304	@ 0x130
 8001d32:	abbe      	add	r3, sp, #760	@ 0x2f8
 8001d34:	2200      	movs	r2, #0
 8001d36:	f7fe fb11 	bl	800035c <smoltcp::iface::interface::InterfaceInner::dispatch_ip>
 8001d3a:	b2c4      	uxtb	r4, r0
 8001d3c:	2c02      	cmp	r4, #2
 8001d3e:	d01a      	beq.n	8001d76 <smoltcp::iface::interface::Interface::poll+0xc5a>
 8001d40:	f240 0023 	movw	r0, #35	@ 0x23
 8001d44:	f2c0 0000 	movt	r0, #0
 8001d48:	f002 fb84 	bl	8004454 <defmt::export::acquire_and_header>
 8001d4c:	f240 003e 	movw	r0, #62	@ 0x3e
 8001d50:	aebe      	add	r6, sp, #760	@ 0x2f8
 8001d52:	f2c0 0000 	movt	r0, #0
 8001d56:	2102      	movs	r1, #2
 8001d58:	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
 8001d5c:	4630      	mov	r0, r6
 8001d5e:	f002 fd2b 	bl	80047b8 <_defmt_write>
 8001d62:	f004 0001 	and.w	r0, r4, #1
 8001d66:	f88d 02f8 	strb.w	r0, [sp, #760]	@ 0x2f8
 8001d6a:	4630      	mov	r0, r6
 8001d6c:	2101      	movs	r1, #1
 8001d6e:	f002 fd23 	bl	80047b8 <_defmt_write>
 8001d72:	f002 fca9 	bl	80046c8 <_defmt_release>
 8001d76:	f249 0408 	movw	r4, #36872	@ 0x9008
 8001d7a:	f04f 0c01 	mov.w	ip, #1
 8001d7e:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 8001d82:	f44f 6ec8 	mov.w	lr, #1600	@ 0x640
 8001d86:	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
 8001d88:	f8dd b124 	ldr.w	fp, [sp, #292]	@ 0x124
 8001d8c:	f7ff bbf8 	b.w	8001580 <smoltcp::iface::interface::Interface::poll+0x464>
 8001d90:	f1be 0f00 	cmp.w	lr, #0
 8001d94:	d023      	beq.n	8001dde <smoltcp::iface::interface::Interface::poll+0xcc2>
 8001d96:	9944      	ldr	r1, [sp, #272]	@ 0x110
 8001d98:	eb0e 008e 	add.w	r0, lr, lr, lsl #2
 8001d9c:	4408      	add	r0, r1
 8001d9e:	e00e      	b.n	8001dbe <smoltcp::iface::interface::Interface::poll+0xca2>
 8001da0:	2600      	movs	r6, #0
 8001da2:	f002 021f 	and.w	r2, r2, #31
 8001da6:	4033      	ands	r3, r6
 8001da8:	fa24 f202 	lsr.w	r2, r4, r2
 8001dac:	ba12      	rev	r2, r2
 8001dae:	431a      	orrs	r2, r3
 8001db0:	9b2b      	ldr	r3, [sp, #172]	@ 0xac
 8001db2:	4293      	cmp	r3, r2
 8001db4:	f43f ae80 	beq.w	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 8001db8:	3105      	adds	r1, #5
 8001dba:	4281      	cmp	r1, r0
 8001dbc:	d00f      	beq.n	8001dde <smoltcp::iface::interface::Interface::poll+0xcc2>
 8001dbe:	790a      	ldrb	r2, [r1, #4]
 8001dc0:	680b      	ldr	r3, [r1, #0]
 8001dc2:	2a00      	cmp	r2, #0
 8001dc4:	d0ec      	beq.n	8001da0 <smoltcp::iface::interface::Interface::poll+0xc84>
 8001dc6:	f1a2 061f 	sub.w	r6, r2, #31
 8001dca:	b2f6      	uxtb	r6, r6
 8001dcc:	2e02      	cmp	r6, #2
 8001dce:	d3f3      	bcc.n	8001db8 <smoltcp::iface::interface::Interface::poll+0xc9c>
 8001dd0:	4256      	negs	r6, r2
 8001dd2:	f006 061f 	and.w	r6, r6, #31
 8001dd6:	fa04 f606 	lsl.w	r6, r4, r6
 8001dda:	ba36      	rev	r6, r6
 8001ddc:	e7e1      	b.n	8001da2 <smoltcp::iface::interface::Interface::poll+0xc86>
 8001dde:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8001de0:	2200      	movs	r2, #0
 8001de2:	e9d0 1c04 	ldrd	r1, ip, [r0, #16]
 8001de6:	f8d0 00e0 	ldr.w	r0, [r0, #224]	@ 0xe0
 8001dea:	eba0 0080 	sub.w	r0, r0, r0, lsl #2
 8001dee:	00c3      	lsls	r3, r0, #3
 8001df0:	1898      	adds	r0, r3, r2
 8001df2:	f43f ae61 	beq.w	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 8001df6:	9e4a      	ldr	r6, [sp, #296]	@ 0x128
 8001df8:	4416      	add	r6, r2
 8001dfa:	6a34      	ldr	r4, [r6, #32]
 8001dfc:	42ac      	cmp	r4, r5
 8001dfe:	f000 814c 	beq.w	800209a <smoltcp::iface::interface::Interface::poll+0xf7e>
 8001e02:	f110 0418 	adds.w	r4, r0, #24
 8001e06:	f43f ae57 	beq.w	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 8001e0a:	6bb4      	ldr	r4, [r6, #56]	@ 0x38
 8001e0c:	42ac      	cmp	r4, r5
 8001e0e:	f000 8147 	beq.w	80020a0 <smoltcp::iface::interface::Interface::poll+0xf84>
 8001e12:	f110 0430 	adds.w	r4, r0, #48	@ 0x30
 8001e16:	f43f ae4f 	beq.w	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 8001e1a:	6d34      	ldr	r4, [r6, #80]	@ 0x50
 8001e1c:	42ac      	cmp	r4, r5
 8001e1e:	f000 8142 	beq.w	80020a6 <smoltcp::iface::interface::Interface::poll+0xf8a>
 8001e22:	3048      	adds	r0, #72	@ 0x48
 8001e24:	f43f ae48 	beq.w	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 8001e28:	6eb0      	ldr	r0, [r6, #104]	@ 0x68
 8001e2a:	3260      	adds	r2, #96	@ 0x60
 8001e2c:	42a8      	cmp	r0, r5
 8001e2e:	d1df      	bne.n	8001df0 <smoltcp::iface::interface::Interface::poll+0xcd4>
 8001e30:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8001e32:	4410      	add	r0, r2
 8001e34:	3008      	adds	r0, #8
 8001e36:	e138      	b.n	80020aa <smoltcp::iface::interface::Interface::poll+0xf8e>
 8001e38:	f1bc 0f02 	cmp.w	ip, #2
 8001e3c:	f4ff ae1a 	bcc.w	8001a74 <smoltcp::iface::interface::Interface::poll+0x958>
 8001e40:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8001e42:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 8001e44:	e9d0 0104 	ldrd	r0, r1, [r0, #16]
 8001e48:	e9cd 0100 	strd	r0, r1, [sp]
 8001e4c:	a8be      	add	r0, sp, #760	@ 0x2f8
 8001e4e:	9935      	ldr	r1, [sp, #212]	@ 0xd4
 8001e50:	f006 fae7 	bl	8008422 <smoltcp::iface::route::Routes::lookup>
 8001e54:	f89d 02f8 	ldrb.w	r0, [sp, #760]	@ 0x2f8
 8001e58:	2801      	cmp	r0, #1
 8001e5a:	f47f ae0b 	bne.w	8001a74 <smoltcp::iface::interface::Interface::poll+0x958>
 8001e5e:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8001e60:	9e44      	ldr	r6, [sp, #272]	@ 0x110
 8001e62:	f8dd 12f9 	ldr.w	r1, [sp, #761]	@ 0x2f9
 8001e66:	f8d0 e0f0 	ldr.w	lr, [r0, #240]	@ 0xf0
 8001e6a:	eb0e 028e 	add.w	r2, lr, lr, lsl #2
 8001e6e:	18b3      	adds	r3, r6, r2
 8001e70:	2a00      	cmp	r2, #0
 8001e72:	f43f adff 	beq.w	8001a74 <smoltcp::iface::interface::Interface::poll+0x958>
 8001e76:	4630      	mov	r0, r6
 8001e78:	f850 4b05 	ldr.w	r4, [r0], #5
 8001e7c:	428c      	cmp	r4, r1
 8001e7e:	d01a      	beq.n	8001eb6 <smoltcp::iface::interface::Interface::poll+0xd9a>
 8001e80:	4298      	cmp	r0, r3
 8001e82:	f43f adf7 	beq.w	8001a74 <smoltcp::iface::interface::Interface::poll+0x958>
 8001e86:	f8d6 0005 	ldr.w	r0, [r6, #5]
 8001e8a:	4288      	cmp	r0, r1
 8001e8c:	d013      	beq.n	8001eb6 <smoltcp::iface::interface::Interface::poll+0xd9a>
 8001e8e:	f106 000a 	add.w	r0, r6, #10
 8001e92:	4298      	cmp	r0, r3
 8001e94:	f43f adee 	beq.w	8001a74 <smoltcp::iface::interface::Interface::poll+0x958>
 8001e98:	f8d6 000a 	ldr.w	r0, [r6, #10]
 8001e9c:	4288      	cmp	r0, r1
 8001e9e:	d00a      	beq.n	8001eb6 <smoltcp::iface::interface::Interface::poll+0xd9a>
 8001ea0:	f106 000f 	add.w	r0, r6, #15
 8001ea4:	4298      	cmp	r0, r3
 8001ea6:	f43f ade5 	beq.w	8001a74 <smoltcp::iface::interface::Interface::poll+0x958>
 8001eaa:	f8d6 000f 	ldr.w	r0, [r6, #15]
 8001eae:	3a14      	subs	r2, #20
 8001eb0:	3614      	adds	r6, #20
 8001eb2:	4288      	cmp	r0, r1
 8001eb4:	d1dc      	bne.n	8001e70 <smoltcp::iface::interface::Interface::poll+0xd54>
 8001eb6:	f8d7 a008 	ldr.w	sl, [r7, #8]
 8001eba:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 8001ebe:	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
 8001ec2:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 8001ec6:	e5ec      	b.n	8001aa2 <smoltcp::iface::interface::Interface::poll+0x986>
 8001ec8:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001eca:	8846      	ldrh	r6, [r0, #2]
 8001ecc:	2e00      	cmp	r6, #0
 8001ece:	f43f aebe 	beq.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001ed2:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8001ed4:	7a40      	ldrb	r0, [r0, #9]
 8001ed6:	2801      	cmp	r0, #1
 8001ed8:	d837      	bhi.n	8001f4a <smoltcp::iface::interface::Interface::poll+0xe2e>
 8001eda:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001edc:	88c0      	ldrh	r0, [r0, #6]
 8001ede:	b3a0      	cbz	r0, 8001f4a <smoltcp::iface::interface::Interface::poll+0xe2e>
 8001ee0:	982b      	ldr	r0, [sp, #172]	@ 0xac
 8001ee2:	ba00      	rev	r0, r0
 8001ee4:	0c01      	lsrs	r1, r0, #16
 8001ee6:	fa11 f080 	uxtah	r0, r1, r0
 8001eea:	0c01      	lsrs	r1, r0, #16
 8001eec:	fa11 f080 	uxtah	r0, r1, r0
 8001ef0:	f109 0111 	add.w	r1, r9, #17
 8001ef4:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001ef8:	0c0a      	lsrs	r2, r1, #16
 8001efa:	fa12 f181 	uxtah	r1, r2, r1
 8001efe:	b280      	uxth	r0, r0
 8001f00:	eb01 4111 	add.w	r1, r1, r1, lsr #16
 8001f04:	fa10 f081 	uxtah	r0, r0, r1
 8001f08:	9925      	ldr	r1, [sp, #148]	@ 0x94
 8001f0a:	ba09      	rev	r1, r1
 8001f0c:	0c0a      	lsrs	r2, r1, #16
 8001f0e:	fa12 f181 	uxtah	r1, r2, r1
 8001f12:	0c0a      	lsrs	r2, r1, #16
 8001f14:	fa12 f181 	uxtah	r1, r2, r1
 8001f18:	eb01 4111 	add.w	r1, r1, r1, lsr #16
 8001f1c:	fa10 f081 	uxtah	r0, r0, r1
 8001f20:	0c01      	lsrs	r1, r0, #16
 8001f22:	fa11 f080 	uxtah	r0, r1, r0
 8001f26:	4649      	mov	r1, r9
 8001f28:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001f2c:	b284      	uxth	r4, r0
 8001f2e:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001f30:	f006 f911 	bl	8008156 <smoltcp::wire::ip::checksum::data>
 8001f34:	fa14 f080 	uxtah	r0, r4, r0
 8001f38:	0c01      	lsrs	r1, r0, #16
 8001f3a:	fa11 f080 	uxtah	r0, r1, r0
 8001f3e:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001f42:	43c0      	mvns	r0, r0
 8001f44:	0400      	lsls	r0, r0, #16
 8001f46:	f47f ae82 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001f4a:	e9dd 252f 	ldrd	r2, r5, [sp, #188]	@ 0xbc
 8001f4e:	f44f 71a8 	mov.w	r1, #336	@ 0x150
 8001f52:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001f54:	fa96 fa96 	rev16.w	sl, r6
 8001f58:	fb02 5401 	mla	r4, r2, r1, r5
 8001f5c:	8800      	ldrh	r0, [r0, #0]
 8001f5e:	ba00      	rev	r0, r0
 8001f60:	ea4f 4810 	mov.w	r8, r0, lsr #16
 8001f64:	e00c      	b.n	8001f80 <smoltcp::iface::interface::Interface::poll+0xe64>
 8001f66:	4605      	mov	r5, r0
 8001f68:	4608      	mov	r0, r1
 8001f6a:	f100 0608 	add.w	r6, r0, #8
 8001f6e:	994a      	ldr	r1, [sp, #296]	@ 0x128
 8001f70:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 8001f72:	4653      	mov	r3, sl
 8001f74:	4630      	mov	r0, r6
 8001f76:	f005 ffe6 	bl	8007f46 <smoltcp::socket::udp::Socket::accepts>
 8001f7a:	2800      	cmp	r0, #0
 8001f7c:	f040 80b0 	bne.w	80020e0 <smoltcp::iface::interface::Interface::poll+0xfc4>
 8001f80:	42a5      	cmp	r5, r4
 8001f82:	d066      	beq.n	8002052 <smoltcp::iface::interface::Interface::poll+0xf36>
 8001f84:	e9d5 0200 	ldrd	r0, r2, [r5]
 8001f88:	4629      	mov	r1, r5
 8001f8a:	f080 0002 	eor.w	r0, r0, #2
 8001f8e:	4310      	orrs	r0, r2
 8001f90:	f505 70a8 	add.w	r0, r5, #336	@ 0x150
 8001f94:	d0e7      	beq.n	8001f66 <smoltcp::iface::interface::Interface::poll+0xe4a>
 8001f96:	42a0      	cmp	r0, r4
 8001f98:	d05b      	beq.n	8002052 <smoltcp::iface::interface::Interface::poll+0xf36>
 8001f9a:	e9d1 2354 	ldrd	r2, r3, [r1, #336]	@ 0x150
 8001f9e:	f082 0202 	eor.w	r2, r2, #2
 8001fa2:	431a      	orrs	r2, r3
 8001fa4:	f501 7228 	add.w	r2, r1, #672	@ 0x2a0
 8001fa8:	d101      	bne.n	8001fae <smoltcp::iface::interface::Interface::poll+0xe92>
 8001faa:	4615      	mov	r5, r2
 8001fac:	e7dd      	b.n	8001f6a <smoltcp::iface::interface::Interface::poll+0xe4e>
 8001fae:	42a2      	cmp	r2, r4
 8001fb0:	d04f      	beq.n	8002052 <smoltcp::iface::interface::Interface::poll+0xf36>
 8001fb2:	e9d1 03a8 	ldrd	r0, r3, [r1, #672]	@ 0x2a0
 8001fb6:	f080 0002 	eor.w	r0, r0, #2
 8001fba:	4318      	orrs	r0, r3
 8001fbc:	f501 707c 	add.w	r0, r1, #1008	@ 0x3f0
 8001fc0:	d102      	bne.n	8001fc8 <smoltcp::iface::interface::Interface::poll+0xeac>
 8001fc2:	4605      	mov	r5, r0
 8001fc4:	4610      	mov	r0, r2
 8001fc6:	e7d0      	b.n	8001f6a <smoltcp::iface::interface::Interface::poll+0xe4e>
 8001fc8:	42a0      	cmp	r0, r4
 8001fca:	d042      	beq.n	8002052 <smoltcp::iface::interface::Interface::poll+0xf36>
 8001fcc:	e9d1 23fc 	ldrd	r2, r3, [r1, #1008]	@ 0x3f0
 8001fd0:	f501 65a8 	add.w	r5, r1, #1344	@ 0x540
 8001fd4:	f082 0202 	eor.w	r2, r2, #2
 8001fd8:	431a      	orrs	r2, r3
 8001fda:	d1d1      	bne.n	8001f80 <smoltcp::iface::interface::Interface::poll+0xe64>
 8001fdc:	e7c5      	b.n	8001f6a <smoltcp::iface::interface::Interface::poll+0xe4e>
 8001fde:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8001fe0:	7840      	ldrb	r0, [r0, #1]
 8001fe2:	2800      	cmp	r0, #0
 8001fe4:	f47f ae33 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8001fe8:	9a2a      	ldr	r2, [sp, #168]	@ 0xa8
 8001fea:	2104      	movs	r1, #4
 8001fec:	9157      	str	r1, [sp, #348]	@ 0x15c
 8001fee:	88d0      	ldrh	r0, [r2, #6]
 8001ff0:	8891      	ldrh	r1, [r2, #4]
 8001ff2:	ba00      	rev	r0, r0
 8001ff4:	9a1a      	ldr	r2, [sp, #104]	@ 0x68
 8001ff6:	eac2 4220 	pkhtb	r2, r2, r0, asr #16
 8001ffa:	ba08      	rev	r0, r1
 8001ffc:	921a      	str	r2, [sp, #104]	@ 0x68
 8001ffe:	0c00      	lsrs	r0, r0, #16
 8002000:	9017      	str	r0, [sp, #92]	@ 0x5c
 8002002:	e626      	b.n	8001c52 <smoltcp::iface::interface::Interface::poll+0xb36>
 8002004:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8002006:	7840      	ldrb	r0, [r0, #1]
 8002008:	2800      	cmp	r0, #0
 800200a:	f47f ae20 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 800200e:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8002010:	2101      	movs	r1, #1
 8002012:	9a2a      	ldr	r2, [sp, #168]	@ 0xa8
 8002014:	3808      	subs	r0, #8
 8002016:	90c1      	str	r0, [sp, #772]	@ 0x304
 8002018:	9829      	ldr	r0, [sp, #164]	@ 0xa4
 800201a:	9b2b      	ldr	r3, [sp, #172]	@ 0xac
 800201c:	3008      	adds	r0, #8
 800201e:	90c0      	str	r0, [sp, #768]	@ 0x300
 8002020:	88d0      	ldrh	r0, [r2, #6]
 8002022:	f88d 12f8 	strb.w	r1, [sp, #760]	@ 0x2f8
 8002026:	ba04      	rev	r4, r0
 8002028:	8891      	ldrh	r1, [r2, #4]
 800202a:	9a25      	ldr	r2, [sp, #148]	@ 0x94
 800202c:	0c20      	lsrs	r0, r4, #16
 800202e:	f8ad 02fc 	strh.w	r0, [sp, #764]	@ 0x2fc
 8002032:	ba08      	rev	r0, r1
 8002034:	994a      	ldr	r1, [sp, #296]	@ 0x128
 8002036:	0c00      	lsrs	r0, r0, #16
 8002038:	9017      	str	r0, [sp, #92]	@ 0x5c
 800203a:	f8ad 02fa 	strh.w	r0, [sp, #762]	@ 0x2fa
 800203e:	a8be      	add	r0, sp, #760	@ 0x2f8
 8002040:	9000      	str	r0, [sp, #0]
 8002042:	a857      	add	r0, sp, #348	@ 0x15c
 8002044:	f005 fd33 	bl	8007aae <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>
 8002048:	981a      	ldr	r0, [sp, #104]	@ 0x68
 800204a:	eac0 4024 	pkhtb	r0, r0, r4, asr #16
 800204e:	901a      	str	r0, [sp, #104]	@ 0x68
 8002050:	e5ff      	b.n	8001c52 <smoltcp::iface::interface::Interface::poll+0xb36>
 8002052:	9826      	ldr	r0, [sp, #152]	@ 0x98
 8002054:	f8ad 0312 	strh.w	r0, [sp, #786]	@ 0x312
 8002058:	2004      	movs	r0, #4
 800205a:	f88d 0311 	strb.w	r0, [sp, #785]	@ 0x311
 800205e:	9821      	ldr	r0, [sp, #132]	@ 0x84
 8002060:	f88d 0310 	strb.w	r0, [sp, #784]	@ 0x310
 8002064:	9823      	ldr	r0, [sp, #140]	@ 0x8c
 8002066:	90c3      	str	r0, [sp, #780]	@ 0x30c
 8002068:	f240 3002 	movw	r0, #770	@ 0x302
 800206c:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 800206e:	9b2b      	ldr	r3, [sp, #172]	@ 0xac
 8002070:	9a25      	ldr	r2, [sp, #148]	@ 0x94
 8002072:	f5b1 7f04 	cmp.w	r1, #528	@ 0x210
 8002076:	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
 800207a:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 800207c:	90bf      	str	r0, [sp, #764]	@ 0x2fc
 800207e:	f44f 7004 	mov.w	r0, #528	@ 0x210
 8002082:	93c2      	str	r3, [sp, #776]	@ 0x308
 8002084:	92c1      	str	r2, [sp, #772]	@ 0x304
 8002086:	bf28      	it	cs
 8002088:	4601      	movcs	r1, r0
 800208a:	91c0      	str	r1, [sp, #768]	@ 0x300
 800208c:	a8be      	add	r0, sp, #760	@ 0x2f8
 800208e:	994a      	ldr	r1, [sp, #296]	@ 0x128
 8002090:	9000      	str	r0, [sp, #0]
 8002092:	a857      	add	r0, sp, #348	@ 0x15c
 8002094:	f005 fd0b 	bl	8007aae <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>
 8002098:	e5db      	b.n	8001c52 <smoltcp::iface::interface::Interface::poll+0xb36>
 800209a:	f106 0020 	add.w	r0, r6, #32
 800209e:	e004      	b.n	80020aa <smoltcp::iface::interface::Interface::poll+0xf8e>
 80020a0:	f106 0038 	add.w	r0, r6, #56	@ 0x38
 80020a4:	e001      	b.n	80020aa <smoltcp::iface::interface::Interface::poll+0xf8e>
 80020a6:	f106 0050 	add.w	r0, r6, #80	@ 0x50
 80020aa:	9e27      	ldr	r6, [sp, #156]	@ 0x9c
 80020ac:	f50d 7996 	add.w	r9, sp, #300	@ 0x12c
 80020b0:	8a82      	ldrh	r2, [r0, #20]
 80020b2:	6903      	ldr	r3, [r0, #16]
 80020b4:	b2b6      	uxth	r6, r6
 80020b6:	4072      	eors	r2, r6
 80020b8:	9e28      	ldr	r6, [sp, #160]	@ 0xa0
 80020ba:	f8d7 a008 	ldr.w	sl, [r7, #8]
 80020be:	f8dd 810c 	ldr.w	r8, [sp, #268]	@ 0x10c
 80020c2:	4073      	eors	r3, r6
 80020c4:	9d25      	ldr	r5, [sp, #148]	@ 0x94
 80020c6:	431a      	orrs	r2, r3
 80020c8:	f47f acf6 	bne.w	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 80020cc:	f248 7200 	movw	r2, #34560	@ 0x8700
 80020d0:	f2c0 3293 	movt	r2, #915	@ 0x393
 80020d4:	1889      	adds	r1, r1, r2
 80020d6:	f14c 0200 	adc.w	r2, ip, #0
 80020da:	e9c0 1202 	strd	r1, r2, [r0, #8]
 80020de:	e4eb      	b.n	8001ab8 <smoltcp::iface::interface::Interface::poll+0x99c>
 80020e0:	9929      	ldr	r1, [sp, #164]	@ 0xa4
 80020e2:	f1a9 0008 	sub.w	r0, r9, #8
 80020e6:	9a25      	ldr	r2, [sp, #148]	@ 0x94
 80020e8:	3108      	adds	r1, #8
 80020ea:	9b2b      	ldr	r3, [sp, #172]	@ 0xac
 80020ec:	9002      	str	r0, [sp, #8]
 80020ee:	4630      	mov	r0, r6
 80020f0:	e9cd 8100 	strd	r8, r1, [sp]
 80020f4:	992e      	ldr	r1, [sp, #184]	@ 0xb8
 80020f6:	f005 ff6f 	bl	8007fd8 <smoltcp::socket::udp::Socket::process>
 80020fa:	e5a8      	b.n	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 80020fc:	f416 6fa0 	tst.w	r6, #1280	@ 0x500
 8002100:	f47f ada5 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8002104:	2002      	movs	r0, #2
 8002106:	e003      	b.n	8002110 <smoltcp::iface::interface::Interface::poll+0xff4>
 8002108:	0570      	lsls	r0, r6, #21
 800210a:	f53f ada0 	bmi.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 800210e:	2003      	movs	r0, #3
 8002110:	9016      	str	r0, [sp, #88]	@ 0x58
 8002112:	04f0      	lsls	r0, r6, #19
 8002114:	902e      	str	r0, [sp, #184]	@ 0xb8
 8002116:	d402      	bmi.n	800211e <smoltcp::iface::interface::Interface::poll+0x1002>
 8002118:	2000      	movs	r0, #0
 800211a:	9015      	str	r0, [sp, #84]	@ 0x54
 800211c:	e005      	b.n	800212a <smoltcp::iface::interface::Interface::poll+0x100e>
 800211e:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8002120:	6880      	ldr	r0, [r0, #8]
 8002122:	ba00      	rev	r0, r0
 8002124:	9006      	str	r0, [sp, #24]
 8002126:	2001      	movs	r0, #1
 8002128:	9015      	str	r0, [sp, #84]	@ 0x54
 800212a:	3d14      	subs	r5, #20
 800212c:	f000 81a5 	beq.w	800247a <smoltcp::iface::interface::Interface::poll+0x135e>
 8002130:	9829      	ldr	r0, [sp, #164]	@ 0xa4
 8002132:	f04f 0900 	mov.w	r9, #0
 8002136:	f04f 0a00 	mov.w	sl, #0
 800213a:	f100 0814 	add.w	r8, r0, #20
 800213e:	2000      	movs	r0, #0
 8002140:	9019      	str	r0, [sp, #100]	@ 0x64
 8002142:	2000      	movs	r0, #0
 8002144:	9022      	str	r0, [sp, #136]	@ 0x88
 8002146:	2000      	movs	r0, #0
 8002148:	9024      	str	r0, [sp, #144]	@ 0x90
 800214a:	9018      	str	r0, [sp, #96]	@ 0x60
 800214c:	e9cd 0027 	strd	r0, r0, [sp, #156]	@ 0x9c
 8002150:	e002      	b.n	8002158 <smoltcp::iface::interface::Interface::poll+0x103c>
 8002152:	2d00      	cmp	r5, #0
 8002154:	f000 81a0 	beq.w	8002498 <smoltcp::iface::interface::Interface::poll+0x137c>
 8002158:	f898 0000 	ldrb.w	r0, [r8]
 800215c:	b150      	cbz	r0, 8002174 <smoltcp::iface::interface::Interface::poll+0x1058>
 800215e:	2801      	cmp	r0, #1
 8002160:	d134      	bne.n	80021cc <smoltcp::iface::interface::Interface::poll+0x10b0>
 8002162:	f04f 0b01 	mov.w	fp, #1
 8002166:	2103      	movs	r1, #3
 8002168:	2000      	movs	r0, #0
 800216a:	2200      	movs	r2, #0
 800216c:	f04f 0c00 	mov.w	ip, #0
 8002170:	2300      	movs	r3, #0
 8002172:	e005      	b.n	8002180 <smoltcp::iface::interface::Interface::poll+0x1064>
 8002174:	f04f 0b01 	mov.w	fp, #1
 8002178:	2102      	movs	r1, #2
 800217a:	4602      	mov	r2, r0
 800217c:	4684      	mov	ip, r0
 800217e:	4603      	mov	r3, r0
 8002180:	455d      	cmp	r5, fp
 8002182:	f0c1 8178 	bcc.w	8003476 <smoltcp::iface::interface::Interface::poll+0x235a>
 8002186:	290a      	cmp	r1, #10
 8002188:	f43f ad61 	beq.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 800218c:	021b      	lsls	r3, r3, #8
 800218e:	2901      	cmp	r1, #1
 8002190:	b29b      	uxth	r3, r3
 8002192:	ea42 0203 	orr.w	r2, r2, r3
 8002196:	f04f 0305 	mov.w	r3, #5
 800219a:	44d8      	add	r8, fp
 800219c:	bf88      	it	hi
 800219e:	1e8b      	subhi	r3, r1, #2
 80021a0:	eba5 050b 	sub.w	r5, r5, fp
 80021a4:	ea42 020c 	orr.w	r2, r2, ip
 80021a8:	e8df f013 	tbh	[pc, r3, lsl #1]
 80021ac:	000b0176 	.word	0x000b0176
 80021b0:	004b000c 	.word	0x004b000c
 80021b4:	00d90008 	.word	0x00d90008
 80021b8:	000b00eb 	.word	0x000b00eb
 80021bc:	2001      	movs	r0, #1
 80021be:	9019      	str	r0, [sp, #100]	@ 0x64
 80021c0:	e7c7      	b.n	8002152 <smoltcp::iface::interface::Interface::poll+0x1036>
 80021c2:	e7c6      	b.n	8002152 <smoltcp::iface::interface::Interface::poll+0x1036>
 80021c4:	2001      	movs	r0, #1
 80021c6:	920c      	str	r2, [sp, #48]	@ 0x30
 80021c8:	9022      	str	r0, [sp, #136]	@ 0x88
 80021ca:	e7c2      	b.n	8002152 <smoltcp::iface::interface::Interface::poll+0x1036>
 80021cc:	2d01      	cmp	r5, #1
 80021ce:	f43f ad3e 	beq.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 80021d2:	f898 2001 	ldrb.w	r2, [r8, #1]
 80021d6:	2100      	movs	r1, #0
 80021d8:	2300      	movs	r3, #0
 80021da:	f108 0602 	add.w	r6, r8, #2
 80021de:	fa5f fb82 	uxtb.w	fp, r2
 80021e2:	455d      	cmp	r5, fp
 80021e4:	bf38      	it	cc
 80021e6:	2101      	movcc	r1, #1
 80021e8:	f1bb 0f02 	cmp.w	fp, #2
 80021ec:	bf38      	it	cc
 80021ee:	2301      	movcc	r3, #1
 80021f0:	4319      	orrs	r1, r3
 80021f2:	4631      	mov	r1, r6
 80021f4:	bf18      	it	ne
 80021f6:	2100      	movne	r1, #0
 80021f8:	f47f ad29 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 80021fc:	f06f 0301 	mvn.w	r3, #1
 8002200:	fa53 f482 	uxtab	r4, r3, r2
 8002204:	1e82      	subs	r2, r0, #2
 8002206:	2a06      	cmp	r2, #6
 8002208:	f200 812e 	bhi.w	8002468 <smoltcp::iface::interface::Interface::poll+0x134c>
 800220c:	e8df f012 	tbh	[pc, r2, lsl #1]
 8002210:	00c20007 	.word	0x00c20007
 8002214:	00d600ce 	.word	0x00d600ce
 8002218:	012c012c 	.word	0x012c012c
 800221c:	0111      	.short	0x0111
 800221e:	f1bb 0f04 	cmp.w	fp, #4
 8002222:	f47f ad14 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8002226:	2c01      	cmp	r4, #1
 8002228:	f241 8147 	bls.w	80034ba <smoltcp::iface::interface::Interface::poll+0x239e>
 800222c:	8830      	ldrh	r0, [r6, #0]
 800222e:	2104      	movs	r1, #4
 8002230:	2200      	movs	r2, #0
 8002232:	f04f 0b04 	mov.w	fp, #4
 8002236:	ba00      	rev	r0, r0
 8002238:	0e03      	lsrs	r3, r0, #24
 800223a:	f3c0 4c07 	ubfx	ip, r0, #16, #8
 800223e:	2000      	movs	r0, #0
 8002240:	e79e      	b.n	8002180 <smoltcp::iface::interface::Interface::poll+0x1064>
 8002242:	f1bc 0f0e 	cmp.w	ip, #14
 8002246:	f240 80a1 	bls.w	800238c <smoltcp::iface::interface::Interface::poll+0x1270>
 800224a:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 800224c:	8841      	ldrh	r1, [r0, #2]
 800224e:	8800      	ldrh	r0, [r0, #0]
 8002250:	9004      	str	r0, [sp, #16]
 8002252:	f240 000a 	movw	r0, #10
 8002256:	f2c0 0000 	movt	r0, #0
 800225a:	9105      	str	r1, [sp, #20]
 800225c:	f002 f8fa 	bl	8004454 <defmt::export::acquire_and_header>
 8002260:	ae8c      	add	r6, sp, #560	@ 0x230
 8002262:	f240 0907 	movw	r9, #7
 8002266:	f2c0 0900 	movt	r9, #0
 800226a:	2102      	movs	r1, #2
 800226c:	4630      	mov	r0, r6
 800226e:	f8ad 9230 	strh.w	r9, [sp, #560]	@ 0x230
 8002272:	f002 faa1 	bl	80047b8 <_defmt_write>
 8002276:	acda      	add	r4, sp, #872	@ 0x368
 8002278:	f240 0a3f 	movw	sl, #63	@ 0x3f
 800227c:	9825      	ldr	r0, [sp, #148]	@ 0x94
 800227e:	f2c0 0a00 	movt	sl, #0
 8002282:	908c      	str	r0, [sp, #560]	@ 0x230
 8002284:	4620      	mov	r0, r4
 8002286:	2102      	movs	r1, #2
 8002288:	f8ad a368 	strh.w	sl, [sp, #872]	@ 0x368
 800228c:	f002 fa94 	bl	80047b8 <_defmt_write>
 8002290:	4620      	mov	r0, r4
 8002292:	2102      	movs	r1, #2
 8002294:	f8ad 9368 	strh.w	r9, [sp, #872]	@ 0x368
 8002298:	f002 fa8e 	bl	80047b8 <_defmt_write>
 800229c:	4630      	mov	r0, r6
 800229e:	f002 f887 	bl	80043b0 <defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>
 80022a2:	f04f 0b00 	mov.w	fp, #0
 80022a6:	4620      	mov	r0, r4
 80022a8:	2102      	movs	r1, #2
 80022aa:	f8ad b368 	strh.w	fp, [sp, #872]	@ 0x368
 80022ae:	f002 fa83 	bl	80047b8 <_defmt_write>
 80022b2:	4630      	mov	r0, r6
 80022b4:	2102      	movs	r1, #2
 80022b6:	f8ad b230 	strh.w	fp, [sp, #560]	@ 0x230
 80022ba:	f002 fa7d 	bl	80047b8 <_defmt_write>
 80022be:	f240 0004 	movw	r0, #4
 80022c2:	2102      	movs	r1, #2
 80022c4:	f2c0 0000 	movt	r0, #0
 80022c8:	f8ad 0230 	strh.w	r0, [sp, #560]	@ 0x230
 80022cc:	4630      	mov	r0, r6
 80022ce:	f002 fa73 	bl	80047b8 <_defmt_write>
 80022d2:	9804      	ldr	r0, [sp, #16]
 80022d4:	2102      	movs	r1, #2
 80022d6:	ba00      	rev	r0, r0
 80022d8:	0c00      	lsrs	r0, r0, #16
 80022da:	f8ad 0230 	strh.w	r0, [sp, #560]	@ 0x230
 80022de:	4630      	mov	r0, r6
 80022e0:	f002 fa6a 	bl	80047b8 <_defmt_write>
 80022e4:	4630      	mov	r0, r6
 80022e6:	2102      	movs	r1, #2
 80022e8:	f8ad 9230 	strh.w	r9, [sp, #560]	@ 0x230
 80022ec:	f002 fa64 	bl	80047b8 <_defmt_write>
 80022f0:	982b      	ldr	r0, [sp, #172]	@ 0xac
 80022f2:	2102      	movs	r1, #2
 80022f4:	908c      	str	r0, [sp, #560]	@ 0x230
 80022f6:	4620      	mov	r0, r4
 80022f8:	f8ad a368 	strh.w	sl, [sp, #872]	@ 0x368
 80022fc:	f002 fa5c 	bl	80047b8 <_defmt_write>
 8002300:	4620      	mov	r0, r4
 8002302:	2102      	movs	r1, #2
 8002304:	f8ad 9368 	strh.w	r9, [sp, #872]	@ 0x368
 8002308:	f002 fa56 	bl	80047b8 <_defmt_write>
 800230c:	4630      	mov	r0, r6
 800230e:	f002 f84f 	bl	80043b0 <defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>
 8002312:	4620      	mov	r0, r4
 8002314:	2102      	movs	r1, #2
 8002316:	f8ad b368 	strh.w	fp, [sp, #872]	@ 0x368
 800231a:	f002 fa4d 	bl	80047b8 <_defmt_write>
 800231e:	4630      	mov	r0, r6
 8002320:	2102      	movs	r1, #2
 8002322:	f8ad b230 	strh.w	fp, [sp, #560]	@ 0x230
 8002326:	f002 fa47 	bl	80047b8 <_defmt_write>
 800232a:	f240 0004 	movw	r0, #4
 800232e:	2102      	movs	r1, #2
 8002330:	f2c0 0000 	movt	r0, #0
 8002334:	f8ad 0230 	strh.w	r0, [sp, #560]	@ 0x230
 8002338:	4630      	mov	r0, r6
 800233a:	f002 fa3d 	bl	80047b8 <_defmt_write>
 800233e:	9805      	ldr	r0, [sp, #20]
 8002340:	2102      	movs	r1, #2
 8002342:	ba00      	rev	r0, r0
 8002344:	0c00      	lsrs	r0, r0, #16
 8002346:	f8ad 0230 	strh.w	r0, [sp, #560]	@ 0x230
 800234a:	4630      	mov	r0, r6
 800234c:	f002 fa34 	bl	80047b8 <_defmt_write>
 8002350:	f002 f9ba 	bl	80046c8 <_defmt_release>
 8002354:	f04f 0901 	mov.w	r9, #1
 8002358:	f04f 0a0e 	mov.w	sl, #14
 800235c:	e6f9      	b.n	8002152 <smoltcp::iface::interface::Interface::poll+0x1036>
 800235e:	9b37      	ldr	r3, [sp, #220]	@ 0xdc
 8002360:	e9cd 4213 	strd	r4, r2, [sp, #76]	@ 0x4c
 8002364:	9124      	str	r1, [sp, #144]	@ 0x90
 8002366:	ea40 2003 	orr.w	r0, r0, r3, lsl #8
 800236a:	9027      	str	r0, [sp, #156]	@ 0x9c
 800236c:	9808      	ldr	r0, [sp, #32]
 800236e:	9012      	str	r0, [sp, #72]	@ 0x48
 8002370:	9809      	ldr	r0, [sp, #36]	@ 0x24
 8002372:	9018      	str	r0, [sp, #96]	@ 0x60
 8002374:	980b      	ldr	r0, [sp, #44]	@ 0x2c
 8002376:	900d      	str	r0, [sp, #52]	@ 0x34
 8002378:	9807      	ldr	r0, [sp, #28]
 800237a:	900f      	str	r0, [sp, #60]	@ 0x3c
 800237c:	980a      	ldr	r0, [sp, #40]	@ 0x28
 800237e:	900e      	str	r0, [sp, #56]	@ 0x38
 8002380:	e6e7      	b.n	8002152 <smoltcp::iface::interface::Interface::poll+0x1036>
 8002382:	2001      	movs	r0, #1
 8002384:	e9cd 4210 	strd	r4, r2, [sp, #64]	@ 0x40
 8002388:	9028      	str	r0, [sp, #160]	@ 0xa0
 800238a:	e6e2      	b.n	8002152 <smoltcp::iface::interface::Interface::poll+0x1036>
 800238c:	f04f 0901 	mov.w	r9, #1
 8002390:	46e2      	mov	sl, ip
 8002392:	e6de      	b.n	8002152 <smoltcp::iface::interface::Interface::poll+0x1036>
 8002394:	f1bb 0f03 	cmp.w	fp, #3
 8002398:	f47f ac59 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 800239c:	f04f 0b03 	mov.w	fp, #3
 80023a0:	f896 c000 	ldrb.w	ip, [r6]
 80023a4:	2105      	movs	r1, #5
 80023a6:	2000      	movs	r0, #0
 80023a8:	2200      	movs	r2, #0
 80023aa:	e6e1      	b.n	8002170 <smoltcp::iface::interface::Interface::poll+0x1054>
 80023ac:	f1bb 0f02 	cmp.w	fp, #2
 80023b0:	f47f ac4d 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 80023b4:	f04f 0b02 	mov.w	fp, #2
 80023b8:	2106      	movs	r1, #6
 80023ba:	e6d5      	b.n	8002168 <smoltcp::iface::interface::Interface::poll+0x104c>
 80023bc:	f1bb 0f0a 	cmp.w	fp, #10
 80023c0:	f4ff ac45 	bcc.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 80023c4:	f00b 0007 	and.w	r0, fp, #7
 80023c8:	2802      	cmp	r0, #2
 80023ca:	f47f ac40 	bne.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 80023ce:	f1bb 0f1a 	cmp.w	fp, #26
 80023d2:	d905      	bls.n	80023e0 <smoltcp::iface::interface::Interface::poll+0x12c4>
 80023d4:	f240 0009 	movw	r0, #9
 80023d8:	f2c0 0000 	movt	r0, #0
 80023dc:	f002 f84b 	bl	8004476 <defmt::export::acquire_header_and_release>
 80023e0:	2000      	movs	r0, #0
 80023e2:	9933      	ldr	r1, [sp, #204]	@ 0xcc
 80023e4:	9092      	str	r0, [sp, #584]	@ 0x248
 80023e6:	aada      	add	r2, sp, #872	@ 0x368
 80023e8:	908f      	str	r0, [sp, #572]	@ 0x23c
 80023ea:	908c      	str	r0, [sp, #560]	@ 0x230
 80023ec:	90dc      	str	r0, [sp, #880]	@ 0x370
 80023ee:	a88c      	add	r0, sp, #560	@ 0x230
 80023f0:	e9cd 64da 	strd	r6, r4, [sp, #872]	@ 0x368
 80023f4:	f003 ff22 	bl	800623c <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold>
 80023f8:	9a34      	ldr	r2, [sp, #208]	@ 0xd0
 80023fa:	e9dd 108c 	ldrd	r1, r0, [sp, #560]	@ 0x230
 80023fe:	fa5f fc80 	uxtb.w	ip, r0
 8002402:	9c8e      	ldr	r4, [sp, #568]	@ 0x238
 8002404:	7893      	ldrb	r3, [r2, #2]
 8002406:	8816      	ldrh	r6, [r2, #0]
 8002408:	9a94      	ldr	r2, [sp, #592]	@ 0x250
 800240a:	ea46 4303 	orr.w	r3, r6, r3, lsl #16
 800240e:	920b      	str	r2, [sp, #44]	@ 0x2c
 8002410:	f64f 72ff 	movw	r2, #65535	@ 0xffff
 8002414:	9337      	str	r3, [sp, #220]	@ 0xdc
 8002416:	ea20 0202 	bic.w	r2, r0, r2
 800241a:	0a03      	lsrs	r3, r0, #8
 800241c:	9893      	ldr	r0, [sp, #588]	@ 0x24c
 800241e:	9008      	str	r0, [sp, #32]
 8002420:	9892      	ldr	r0, [sp, #584]	@ 0x248
 8002422:	9009      	str	r0, [sp, #36]	@ 0x24
 8002424:	9891      	ldr	r0, [sp, #580]	@ 0x244
 8002426:	9e90      	ldr	r6, [sp, #576]	@ 0x240
 8002428:	900a      	str	r0, [sp, #40]	@ 0x28
 800242a:	f89d 023c 	ldrb.w	r0, [sp, #572]	@ 0x23c
 800242e:	9607      	str	r6, [sp, #28]
 8002430:	e6a6      	b.n	8002180 <smoltcp::iface::interface::Interface::poll+0x1064>
 8002432:	f1bb 0f0a 	cmp.w	fp, #10
 8002436:	d117      	bne.n	8002468 <smoltcp::iface::interface::Interface::poll+0x134c>
 8002438:	2c03      	cmp	r4, #3
 800243a:	f241 8047 	bls.w	80034cc <smoltcp::iface::interface::Interface::poll+0x23b0>
 800243e:	2c07      	cmp	r4, #7
 8002440:	f241 804d 	bls.w	80034de <smoltcp::iface::interface::Interface::poll+0x23c2>
 8002444:	f8d8 0002 	ldr.w	r0, [r8, #2]
 8002448:	f64f 72ff 	movw	r2, #65535	@ 0xffff
 800244c:	f8d8 1006 	ldr.w	r1, [r8, #6]
 8002450:	f04f 0b0a 	mov.w	fp, #10
 8002454:	ba00      	rev	r0, r0
 8002456:	ba0c      	rev	r4, r1
 8002458:	ea20 0202 	bic.w	r2, r0, r2
 800245c:	0a03      	lsrs	r3, r0, #8
 800245e:	fa5f fc80 	uxtb.w	ip, r0
 8002462:	2108      	movs	r1, #8
 8002464:	2000      	movs	r0, #0
 8002466:	e68b      	b.n	8002180 <smoltcp::iface::interface::Interface::poll+0x1064>
 8002468:	f64f 72ff 	movw	r2, #65535	@ 0xffff
 800246c:	0a0b      	lsrs	r3, r1, #8
 800246e:	ea21 0202 	bic.w	r2, r1, r2
 8002472:	fa5f fc81 	uxtb.w	ip, r1
 8002476:	2109      	movs	r1, #9
 8002478:	e682      	b.n	8002180 <smoltcp::iface::interface::Interface::poll+0x1064>
 800247a:	2000      	movs	r0, #0
 800247c:	f04f 0a00 	mov.w	sl, #0
 8002480:	9028      	str	r0, [sp, #160]	@ 0xa0
 8002482:	f04f 0900 	mov.w	r9, #0
 8002486:	2000      	movs	r0, #0
 8002488:	9018      	str	r0, [sp, #96]	@ 0x60
 800248a:	9027      	str	r0, [sp, #156]	@ 0x9c
 800248c:	9024      	str	r0, [sp, #144]	@ 0x90
 800248e:	2000      	movs	r0, #0
 8002490:	9022      	str	r0, [sp, #136]	@ 0x88
 8002492:	2000      	movs	r0, #0
 8002494:	9019      	str	r0, [sp, #100]	@ 0x64
 8002496:	e001      	b.n	800249c <smoltcp::iface::interface::Interface::poll+0x1380>
 8002498:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 800249a:	8986      	ldrh	r6, [r0, #12]
 800249c:	203c      	movs	r0, #60	@ 0x3c
 800249e:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 80024a0:	ea00 0096 	and.w	r0, r0, r6, lsr #2
 80024a4:	4281      	cmp	r1, r0
 80024a6:	f0c1 8023 	bcc.w	80034f0 <smoltcp::iface::interface::Interface::poll+0x23d4>
 80024aa:	e9dd 322f 	ldrd	r3, r2, [sp, #188]	@ 0xbc
 80024ae:	f44f 71a8 	mov.w	r1, #336	@ 0x150
 80024b2:	f88d a225 	strb.w	sl, [sp, #549]	@ 0x225
 80024b6:	fb03 2201 	mla	r2, r3, r1, r2
 80024ba:	9916      	ldr	r1, [sp, #88]	@ 0x58
 80024bc:	f88d 122d 	strb.w	r1, [sp, #557]	@ 0x22d
 80024c0:	9919      	ldr	r1, [sp, #100]	@ 0x64
 80024c2:	9b06      	ldr	r3, [sp, #24]
 80024c4:	f001 0101 	and.w	r1, r1, #1
 80024c8:	f88d 122c 	strb.w	r1, [sp, #556]	@ 0x22c
 80024cc:	f009 0101 	and.w	r1, r9, #1
 80024d0:	f88d 1224 	strb.w	r1, [sp, #548]	@ 0x224
 80024d4:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 80024d6:	9378      	str	r3, [sp, #480]	@ 0x1e0
 80024d8:	1a09      	subs	r1, r1, r0
 80024da:	9187      	str	r1, [sp, #540]	@ 0x21c
 80024dc:	9929      	ldr	r1, [sp, #164]	@ 0xa4
 80024de:	9b15      	ldr	r3, [sp, #84]	@ 0x54
 80024e0:	4408      	add	r0, r1
 80024e2:	9086      	str	r0, [sp, #536]	@ 0x218
 80024e4:	980c      	ldr	r0, [sp, #48]	@ 0x30
 80024e6:	f8ad 0216 	strh.w	r0, [sp, #534]	@ 0x216
 80024ea:	9822      	ldr	r0, [sp, #136]	@ 0x88
 80024ec:	f8ad 0214 	strh.w	r0, [sp, #532]	@ 0x214
 80024f0:	9810      	ldr	r0, [sp, #64]	@ 0x40
 80024f2:	9084      	str	r0, [sp, #528]	@ 0x210
 80024f4:	9811      	ldr	r0, [sp, #68]	@ 0x44
 80024f6:	9083      	str	r0, [sp, #524]	@ 0x20c
 80024f8:	9828      	ldr	r0, [sp, #160]	@ 0xa0
 80024fa:	9082      	str	r0, [sp, #520]	@ 0x208
 80024fc:	980d      	ldr	r0, [sp, #52]	@ 0x34
 80024fe:	9081      	str	r0, [sp, #516]	@ 0x204
 8002500:	9812      	ldr	r0, [sp, #72]	@ 0x48
 8002502:	9080      	str	r0, [sp, #512]	@ 0x200
 8002504:	9818      	ldr	r0, [sp, #96]	@ 0x60
 8002506:	907f      	str	r0, [sp, #508]	@ 0x1fc
 8002508:	980e      	ldr	r0, [sp, #56]	@ 0x38
 800250a:	907e      	str	r0, [sp, #504]	@ 0x1f8
 800250c:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 800250e:	907d      	str	r0, [sp, #500]	@ 0x1f4
 8002510:	9827      	ldr	r0, [sp, #156]	@ 0x9c
 8002512:	907c      	str	r0, [sp, #496]	@ 0x1f0
 8002514:	9813      	ldr	r0, [sp, #76]	@ 0x4c
 8002516:	907b      	str	r0, [sp, #492]	@ 0x1ec
 8002518:	9814      	ldr	r0, [sp, #80]	@ 0x50
 800251a:	907a      	str	r0, [sp, #488]	@ 0x1e8
 800251c:	9824      	ldr	r0, [sp, #144]	@ 0x90
 800251e:	9079      	str	r0, [sp, #484]	@ 0x1e4
 8002520:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 8002522:	9377      	str	r3, [sp, #476]	@ 0x1dc
 8002524:	6846      	ldr	r6, [r0, #4]
 8002526:	8801      	ldrh	r1, [r0, #0]
 8002528:	89c4      	ldrh	r4, [r0, #14]
 800252a:	ba36      	rev	r6, r6
 800252c:	f8b0 e002 	ldrh.w	lr, [r0, #2]
 8002530:	ba09      	rev	r1, r1
 8002532:	9688      	str	r6, [sp, #544]	@ 0x220
 8002534:	ba26      	rev	r6, r4
 8002536:	fa9e f39e 	rev16.w	r3, lr
 800253a:	ea4f 4c11 	mov.w	ip, r1, lsr #16
 800253e:	0c36      	lsrs	r6, r6, #16
 8002540:	f8ad 3228 	strh.w	r3, [sp, #552]	@ 0x228
 8002544:	f8ad 622a 	strh.w	r6, [sp, #554]	@ 0x22a
 8002548:	f8ad c226 	strh.w	ip, [sp, #550]	@ 0x226
 800254c:	9830      	ldr	r0, [sp, #192]	@ 0xc0
 800254e:	4290      	cmp	r0, r2
 8002550:	f000 8080 	beq.w	8002654 <smoltcp::iface::interface::Interface::poll+0x1538>
 8002554:	9830      	ldr	r0, [sp, #192]	@ 0xc0
 8002556:	e9d0 6400 	ldrd	r6, r4, [r0]
 800255a:	f500 71a8 	add.w	r1, r0, #336	@ 0x150
 800255e:	3e02      	subs	r6, #2
 8002560:	f174 0600 	sbcs.w	r6, r4, #0
 8002564:	d202      	bcs.n	800256c <smoltcp::iface::interface::Interface::poll+0x1450>
 8002566:	460c      	mov	r4, r1
 8002568:	9930      	ldr	r1, [sp, #192]	@ 0xc0
 800256a:	e029      	b.n	80025c0 <smoltcp::iface::interface::Interface::poll+0x14a4>
 800256c:	4291      	cmp	r1, r2
 800256e:	d071      	beq.n	8002654 <smoltcp::iface::interface::Interface::poll+0x1538>
 8002570:	9d30      	ldr	r5, [sp, #192]	@ 0xc0
 8002572:	e9d5 4054 	ldrd	r4, r0, [r5, #336]	@ 0x150
 8002576:	f505 7628 	add.w	r6, r5, #672	@ 0x2a0
 800257a:	3c02      	subs	r4, #2
 800257c:	f170 0000 	sbcs.w	r0, r0, #0
 8002580:	d201      	bcs.n	8002586 <smoltcp::iface::interface::Interface::poll+0x146a>
 8002582:	4634      	mov	r4, r6
 8002584:	e01c      	b.n	80025c0 <smoltcp::iface::interface::Interface::poll+0x14a4>
 8002586:	4296      	cmp	r6, r2
 8002588:	d064      	beq.n	8002654 <smoltcp::iface::interface::Interface::poll+0x1538>
 800258a:	9930      	ldr	r1, [sp, #192]	@ 0xc0
 800258c:	e9d1 04a8 	ldrd	r0, r4, [r1, #672]	@ 0x2a0
 8002590:	f501 717c 	add.w	r1, r1, #1008	@ 0x3f0
 8002594:	3802      	subs	r0, #2
 8002596:	f174 0000 	sbcs.w	r0, r4, #0
 800259a:	d202      	bcs.n	80025a2 <smoltcp::iface::interface::Interface::poll+0x1486>
 800259c:	460c      	mov	r4, r1
 800259e:	4631      	mov	r1, r6
 80025a0:	e00e      	b.n	80025c0 <smoltcp::iface::interface::Interface::poll+0x14a4>
 80025a2:	4291      	cmp	r1, r2
 80025a4:	d056      	beq.n	8002654 <smoltcp::iface::interface::Interface::poll+0x1538>
 80025a6:	9d30      	ldr	r5, [sp, #192]	@ 0xc0
 80025a8:	e9d5 04fc 	ldrd	r0, r4, [r5, #1008]	@ 0x3f0
 80025ac:	f505 65a8 	add.w	r5, r5, #1344	@ 0x540
 80025b0:	9530      	str	r5, [sp, #192]	@ 0xc0
 80025b2:	f1d0 0001 	rsbs	r0, r0, #1
 80025b6:	f04f 0000 	mov.w	r0, #0
 80025ba:	41a0      	sbcs	r0, r4
 80025bc:	462c      	mov	r4, r5
 80025be:	d3c5      	bcc.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 80025c0:	f891 6133 	ldrb.w	r6, [r1, #307]	@ 0x133
 80025c4:	9430      	str	r4, [sp, #192]	@ 0xc0
 80025c6:	2e00      	cmp	r6, #0
 80025c8:	d0c0      	beq.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 80025ca:	2e01      	cmp	r6, #1
 80025cc:	d103      	bne.n	80025d6 <smoltcp::iface::interface::Interface::poll+0x14ba>
 80025ce:	982e      	ldr	r0, [sp, #184]	@ 0xb8
 80025d0:	9430      	str	r4, [sp, #192]	@ 0xc0
 80025d2:	2800      	cmp	r0, #0
 80025d4:	d4ba      	bmi.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 80025d6:	f8b1 011c 	ldrh.w	r0, [r1, #284]	@ 0x11c
 80025da:	b1a0      	cbz	r0, 8002606 <smoltcp::iface::interface::Interface::poll+0x14ea>
 80025dc:	f8d1 011e 	ldr.w	r0, [r1, #286]	@ 0x11e
 80025e0:	9d2b      	ldr	r5, [sp, #172]	@ 0xac
 80025e2:	9430      	str	r4, [sp, #192]	@ 0xc0
 80025e4:	4285      	cmp	r5, r0
 80025e6:	d1b1      	bne.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 80025e8:	f8b1 0122 	ldrh.w	r0, [r1, #290]	@ 0x122
 80025ec:	9430      	str	r4, [sp, #192]	@ 0xc0
 80025ee:	4283      	cmp	r3, r0
 80025f0:	d1ac      	bne.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 80025f2:	f8d1 0124 	ldr.w	r0, [r1, #292]	@ 0x124
 80025f6:	9d25      	ldr	r5, [sp, #148]	@ 0x94
 80025f8:	9430      	str	r4, [sp, #192]	@ 0xc0
 80025fa:	4285      	cmp	r5, r0
 80025fc:	d1a6      	bne.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 80025fe:	f8b1 0128 	ldrh.w	r0, [r1, #296]	@ 0x128
 8002602:	4584      	cmp	ip, r0
 8002604:	e012      	b.n	800262c <smoltcp::iface::interface::Interface::poll+0x1510>
 8002606:	f8d1 00b3 	ldr.w	r0, [r1, #179]	@ 0xb3
 800260a:	9d2b      	ldr	r5, [sp, #172]	@ 0xac
 800260c:	f891 60b2 	ldrb.w	r6, [r1, #178]	@ 0xb2
 8002610:	1a28      	subs	r0, r5, r0
 8002612:	bf18      	it	ne
 8002614:	2001      	movne	r0, #1
 8002616:	4206      	tst	r6, r0
 8002618:	9430      	str	r4, [sp, #192]	@ 0xc0
 800261a:	d197      	bne.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 800261c:	f1be 0f00 	cmp.w	lr, #0
 8002620:	9430      	str	r4, [sp, #192]	@ 0xc0
 8002622:	f43f af93 	beq.w	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 8002626:	f8b1 00b0 	ldrh.w	r0, [r1, #176]	@ 0xb0
 800262a:	4283      	cmp	r3, r0
 800262c:	9430      	str	r4, [sp, #192]	@ 0xc0
 800262e:	f47f af8d 	bne.w	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 8002632:	a877      	add	r0, sp, #476	@ 0x1dc
 8002634:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 8002636:	9b25      	ldr	r3, [sp, #148]	@ 0x94
 8002638:	e9cd 2000 	strd	r2, r0, [sp]
 800263c:	a88c      	add	r0, sp, #560	@ 0x230
 800263e:	9a4a      	ldr	r2, [sp, #296]	@ 0x128
 8002640:	f003 ffb0 	bl	80065a4 <smoltcp::socket::tcp::Socket::process>
 8002644:	9890      	ldr	r0, [sp, #576]	@ 0x240
 8002646:	2802      	cmp	r0, #2
 8002648:	f43f ab01 	beq.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 800264c:	ab8c      	add	r3, sp, #560	@ 0x230
 800264e:	9e31      	ldr	r6, [sp, #196]	@ 0xc4
 8002650:	aca5      	add	r4, sp, #660	@ 0x294
 8002652:	e017      	b.n	8002684 <smoltcp::iface::interface::Interface::poll+0x1568>
 8002654:	9816      	ldr	r0, [sp, #88]	@ 0x58
 8002656:	992b      	ldr	r1, [sp, #172]	@ 0xac
 8002658:	3804      	subs	r0, #4
 800265a:	fab0 f080 	clz	r0, r0
 800265e:	fab1 f181 	clz	r1, r1
 8002662:	0949      	lsrs	r1, r1, #5
 8002664:	0940      	lsrs	r0, r0, #5
 8002666:	4308      	orrs	r0, r1
 8002668:	9920      	ldr	r1, [sp, #128]	@ 0x80
 800266a:	4308      	orrs	r0, r1
 800266c:	2801      	cmp	r0, #1
 800266e:	f43f aaee 	beq.w	8001c4e <smoltcp::iface::interface::Interface::poll+0xb32>
 8002672:	9925      	ldr	r1, [sp, #148]	@ 0x94
 8002674:	a88c      	add	r0, sp, #560	@ 0x230
 8002676:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 8002678:	ab77      	add	r3, sp, #476	@ 0x1dc
 800267a:	f005 f8a2 	bl	80077c2 <smoltcp::socket::tcp::Socket::rst_reply>
 800267e:	ab8c      	add	r3, sp, #560	@ 0x230
 8002680:	acbe      	add	r4, sp, #760	@ 0x2f8
 8002682:	9e32      	ldr	r6, [sp, #200]	@ 0xc8
 8002684:	cb0f      	ldmia	r3, {r0, r1, r2, r3}
 8002686:	c60f      	stmia	r6!, {r0, r1, r2, r3}
 8002688:	9936      	ldr	r1, [sp, #216]	@ 0xd8
 800268a:	4620      	mov	r0, r4
 800268c:	2254      	movs	r2, #84	@ 0x54
 800268e:	f006 f989 	bl	80089a4 <__aeabi_memcpy4>
 8002692:	a857      	add	r0, sp, #348	@ 0x15c
 8002694:	4621      	mov	r1, r4
 8002696:	2264      	movs	r2, #100	@ 0x64
 8002698:	f006 f984 	bl	80089a4 <__aeabi_memcpy4>
 800269c:	f7ff bad9 	b.w	8001c52 <smoltcp::iface::interface::Interface::poll+0xb36>
 80026a0:	68f8      	ldr	r0, [r7, #12]
 80026a2:	f44f 71a8 	mov.w	r1, #336	@ 0x150
 80026a6:	e9d0 5000 	ldrd	r5, r0, [r0]
 80026aa:	fb00 5001 	mla	r0, r0, r1, r5
 80026ae:	e9dd 2147 	ldrd	r2, r1, [sp, #284]	@ 0x11c
 80026b2:	9049      	str	r0, [sp, #292]	@ 0x124
 80026b4:	984a      	ldr	r0, [sp, #296]	@ 0x128
 80026b6:	e9c0 2104 	strd	r2, r1, [r0, #16]
 80026ba:	a8be      	add	r0, sp, #760	@ 0x2f8
 80026bc:	3008      	adds	r0, #8
 80026be:	9038      	str	r0, [sp, #224]	@ 0xe0
 80026c0:	9849      	ldr	r0, [sp, #292]	@ 0x124
 80026c2:	4285      	cmp	r5, r0
 80026c4:	f000 861a 	beq.w	80032fc <smoltcp::iface::interface::Interface::poll+0x21e0>
 80026c8:	46a9      	mov	r9, r5
 80026ca:	e8f5 0154 	ldrd	r0, r1, [r5], #336	@ 0x150
 80026ce:	f080 0003 	eor.w	r0, r0, #3
 80026d2:	4308      	orrs	r0, r1
 80026d4:	d0f4      	beq.n	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 80026d6:	f899 0138 	ldrb.w	r0, [r9, #312]	@ 0x138
 80026da:	2801      	cmp	r0, #1
 80026dc:	f040 8089 	bne.w	80027f2 <smoltcp::iface::interface::Interface::poll+0x16d6>
 80026e0:	984a      	ldr	r0, [sp, #296]	@ 0x128
 80026e2:	46a8      	mov	r8, r5
 80026e4:	f8d9 2139 	ldr.w	r2, [r9, #313]	@ 0x139
 80026e8:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 80026ec:	e9d0 ab04 	ldrd	sl, fp, [r0, #16]
 80026f0:	f8d0 00f0 	ldr.w	r0, [r0, #240]	@ 0xf0
 80026f4:	e9d9 c550 	ldrd	ip, r5, [r9, #320]	@ 0x140
 80026f8:	9944      	ldr	r1, [sp, #272]	@ 0x110
 80026fa:	eb00 0080 	add.w	r0, r0, r0, lsl #2
 80026fe:	e00b      	b.n	8002718 <smoltcp::iface::interface::Interface::poll+0x15fc>
 8002700:	425b      	negs	r3, r3
 8002702:	f003 031f 	and.w	r3, r3, #31
 8002706:	fa04 f303 	lsl.w	r3, r4, r3
 800270a:	ba1b      	rev	r3, r3
 800270c:	680e      	ldr	r6, [r1, #0]
 800270e:	3105      	adds	r1, #5
 8002710:	3805      	subs	r0, #5
 8002712:	4056      	eors	r6, r2
 8002714:	4233      	tst	r3, r6
 8002716:	d016      	beq.n	8002746 <smoltcp::iface::interface::Interface::poll+0x162a>
 8002718:	b120      	cbz	r0, 8002724 <smoltcp::iface::interface::Interface::poll+0x1608>
 800271a:	790b      	ldrb	r3, [r1, #4]
 800271c:	2b00      	cmp	r3, #0
 800271e:	d1ef      	bne.n	8002700 <smoltcp::iface::interface::Interface::poll+0x15e4>
 8002720:	2300      	movs	r3, #0
 8002722:	e7f3      	b.n	800270c <smoltcp::iface::interface::Interface::poll+0x15f0>
 8002724:	1c50      	adds	r0, r2, #1
 8002726:	f000 85cd 	beq.w	80032c4 <smoltcp::iface::interface::Interface::poll+0x21a8>
 800272a:	9935      	ldr	r1, [sp, #212]	@ 0xd4
 800272c:	a8be      	add	r0, sp, #760	@ 0x2f8
 800272e:	e9cd ab00 	strd	sl, fp, [sp]
 8002732:	4664      	mov	r4, ip
 8002734:	f005 fe75 	bl	8008422 <smoltcp::iface::route::Routes::lookup>
 8002738:	f89d 02f8 	ldrb.w	r0, [sp, #760]	@ 0x2f8
 800273c:	46a4      	mov	ip, r4
 800273e:	2801      	cmp	r0, #1
 8002740:	d150      	bne.n	80027e4 <smoltcp::iface::interface::Interface::poll+0x16c8>
 8002742:	f8dd 22f9 	ldr.w	r2, [sp, #761]	@ 0x2f9
 8002746:	1c50      	adds	r0, r2, #1
 8002748:	f000 85bc 	beq.w	80032c4 <smoltcp::iface::interface::Interface::poll+0x21a8>
 800274c:	f002 00f0 	and.w	r0, r2, #240	@ 0xf0
 8002750:	28e0      	cmp	r0, #224	@ 0xe0
 8002752:	bf1c      	itt	ne
 8002754:	b2d0      	uxtbne	r0, r2
 8002756:	ea50 2012 	orrsne.w	r0, r0, r2, lsr #8
 800275a:	f000 85b3 	beq.w	80032c4 <smoltcp::iface::interface::Interface::poll+0x21a8>
 800275e:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8002760:	f8d0 00e0 	ldr.w	r0, [r0, #224]	@ 0xe0
 8002764:	eba0 0080 	sub.w	r0, r0, r0, lsl #2
 8002768:	00c1      	lsls	r1, r0, #3
 800276a:	2000      	movs	r0, #0
 800276c:	180e      	adds	r6, r1, r0
 800276e:	d039      	beq.n	80027e4 <smoltcp::iface::interface::Interface::poll+0x16c8>
 8002770:	9b4a      	ldr	r3, [sp, #296]	@ 0x128
 8002772:	4403      	add	r3, r0
 8002774:	6a1c      	ldr	r4, [r3, #32]
 8002776:	4294      	cmp	r4, r2
 8002778:	d01d      	beq.n	80027b6 <smoltcp::iface::interface::Interface::poll+0x169a>
 800277a:	f116 0418 	adds.w	r4, r6, #24
 800277e:	d031      	beq.n	80027e4 <smoltcp::iface::interface::Interface::poll+0x16c8>
 8002780:	6b9c      	ldr	r4, [r3, #56]	@ 0x38
 8002782:	4294      	cmp	r4, r2
 8002784:	d025      	beq.n	80027d2 <smoltcp::iface::interface::Interface::poll+0x16b6>
 8002786:	f116 0430 	adds.w	r4, r6, #48	@ 0x30
 800278a:	d02b      	beq.n	80027e4 <smoltcp::iface::interface::Interface::poll+0x16c8>
 800278c:	6d1c      	ldr	r4, [r3, #80]	@ 0x50
 800278e:	4294      	cmp	r4, r2
 8002790:	f000 8189 	beq.w	8002aa6 <smoltcp::iface::interface::Interface::poll+0x198a>
 8002794:	3648      	adds	r6, #72	@ 0x48
 8002796:	d025      	beq.n	80027e4 <smoltcp::iface::interface::Interface::poll+0x16c8>
 8002798:	6e9b      	ldr	r3, [r3, #104]	@ 0x68
 800279a:	3060      	adds	r0, #96	@ 0x60
 800279c:	4293      	cmp	r3, r2
 800279e:	d1e5      	bne.n	800276c <smoltcp::iface::interface::Interface::poll+0x1650>
 80027a0:	994a      	ldr	r1, [sp, #296]	@ 0x128
 80027a2:	4408      	add	r0, r1
 80027a4:	3008      	adds	r0, #8
 80027a6:	e9d0 0102 	ldrd	r0, r1, [r0, #8]
 80027aa:	ebba 0000 	subs.w	r0, sl, r0
 80027ae:	eb7b 0001 	sbcs.w	r0, fp, r1
 80027b2:	db09      	blt.n	80027c8 <smoltcp::iface::interface::Interface::poll+0x16ac>
 80027b4:	e016      	b.n	80027e4 <smoltcp::iface::interface::Interface::poll+0x16c8>
 80027b6:	f103 0020 	add.w	r0, r3, #32
 80027ba:	e9d0 0102 	ldrd	r0, r1, [r0, #8]
 80027be:	ebba 0000 	subs.w	r0, sl, r0
 80027c2:	eb7b 0001 	sbcs.w	r0, fp, r1
 80027c6:	da0d      	bge.n	80027e4 <smoltcp::iface::interface::Interface::poll+0x16c8>
 80027c8:	2000      	movs	r0, #0
 80027ca:	4645      	mov	r5, r8
 80027cc:	f889 0138 	strb.w	r0, [r9, #312]	@ 0x138
 80027d0:	e00f      	b.n	80027f2 <smoltcp::iface::interface::Interface::poll+0x16d6>
 80027d2:	f103 0038 	add.w	r0, r3, #56	@ 0x38
 80027d6:	e9d0 0102 	ldrd	r0, r1, [r0, #8]
 80027da:	ebba 0000 	subs.w	r0, sl, r0
 80027de:	eb7b 0001 	sbcs.w	r0, fp, r1
 80027e2:	dbf1      	blt.n	80027c8 <smoltcp::iface::interface::Interface::poll+0x16ac>
 80027e4:	ebba 000c 	subs.w	r0, sl, ip
 80027e8:	eb7b 0005 	sbcs.w	r0, fp, r5
 80027ec:	4645      	mov	r5, r8
 80027ee:	f6ff af67 	blt.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 80027f2:	e9d9 0100 	ldrd	r0, r1, [r9]
 80027f6:	f080 0002 	eor.w	r0, r0, #2
 80027fa:	4308      	orrs	r0, r1
 80027fc:	d140      	bne.n	8002880 <smoltcp::iface::interface::Interface::poll+0x1764>
 80027fe:	f899 0050 	ldrb.w	r0, [r9, #80]	@ 0x50
 8002802:	f8d9 b03c 	ldr.w	fp, [r9, #60]	@ 0x3c
 8002806:	f899 e051 	ldrb.w	lr, [r9, #81]	@ 0x51
 800280a:	2800      	cmp	r0, #0
 800280c:	bf08      	it	eq
 800280e:	f04f 0e40 	moveq.w	lr, #64	@ 0x40
 8002812:	f1bb 0f00 	cmp.w	fp, #0
 8002816:	f000 80d7 	beq.w	80029c8 <smoltcp::iface::interface::Interface::poll+0x18ac>
 800281a:	f8d9 1034 	ldr.w	r1, [r9, #52]	@ 0x34
 800281e:	2900      	cmp	r1, #0
 8002820:	f000 85e9 	beq.w	80033f6 <smoltcp::iface::interface::Interface::poll+0x22da>
 8002824:	f8d9 0038 	ldr.w	r0, [r9, #56]	@ 0x38
 8002828:	4288      	cmp	r0, r1
 800282a:	f080 85f1 	bcs.w	8003410 <smoltcp::iface::interface::Interface::poll+0x22f4>
 800282e:	f8d9 a030 	ldr.w	sl, [r9, #48]	@ 0x30
 8002832:	eb00 0480 	add.w	r4, r0, r0, lsl #2
 8002836:	e9d9 2302 	ldrd	r2, r3, [r9, #8]
 800283a:	9246      	str	r2, [sp, #280]	@ 0x118
 800283c:	eb0a 0284 	add.w	r2, sl, r4, lsl #2
 8002840:	933f      	str	r3, [sp, #252]	@ 0xfc
 8002842:	7b92      	ldrb	r2, [r2, #14]
 8002844:	2a02      	cmp	r2, #2
 8002846:	d12b      	bne.n	80028a0 <smoltcp::iface::interface::Interface::poll+0x1784>
 8002848:	f109 0844 	add.w	r8, r9, #68	@ 0x44
 800284c:	46ac      	mov	ip, r5
 800284e:	e898 0124 	ldmia.w	r8, {r2, r5, r8}
 8002852:	1b53      	subs	r3, r2, r5
 8002854:	4543      	cmp	r3, r8
 8002856:	bf28      	it	cs
 8002858:	4643      	movcs	r3, r8
 800285a:	195e      	adds	r6, r3, r5
 800285c:	f080 858d 	bcs.w	800337a <smoltcp::iface::interface::Interface::poll+0x225e>
 8002860:	4296      	cmp	r6, r2
 8002862:	f200 858a 	bhi.w	800337a <smoltcp::iface::interface::Interface::poll+0x225e>
 8002866:	f85a 6024 	ldr.w	r6, [sl, r4, lsl #2]
 800286a:	42b3      	cmp	r3, r6
 800286c:	bf38      	it	cc
 800286e:	461e      	movcc	r6, r3
 8002870:	2a00      	cmp	r2, #0
 8002872:	d061      	beq.n	8002938 <smoltcp::iface::interface::Interface::poll+0x181c>
 8002874:	1973      	adds	r3, r6, r5
 8002876:	fbb3 f4f2 	udiv	r4, r3, r2
 800287a:	fb04 3212 	mls	r2, r4, r2, r3
 800287e:	e05c      	b.n	800293a <smoltcp::iface::interface::Interface::poll+0x181e>
 8002880:	f8b9 011c 	ldrh.w	r0, [r9, #284]	@ 0x11c
 8002884:	2800      	cmp	r0, #0
 8002886:	f43f af1b 	beq.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 800288a:	984a      	ldr	r0, [sp, #296]	@ 0x128
 800288c:	e9d9 2314 	ldrd	r2, r3, [r9, #80]	@ 0x50
 8002890:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 8002894:	431a      	orrs	r2, r3
 8002896:	9548      	str	r5, [sp, #288]	@ 0x120
 8002898:	d005      	beq.n	80028a6 <smoltcp::iface::interface::Interface::poll+0x178a>
 800289a:	e9d9 2316 	ldrd	r2, r3, [r9, #88]	@ 0x58
 800289e:	e00c      	b.n	80028ba <smoltcp::iface::interface::Interface::poll+0x179e>
 80028a0:	f8dd a118 	ldr.w	sl, [sp, #280]	@ 0x118
 80028a4:	e05a      	b.n	800295c <smoltcp::iface::interface::Interface::poll+0x1840>
 80028a6:	2201      	movs	r2, #1
 80028a8:	f8c9 005c 	str.w	r0, [r9, #92]	@ 0x5c
 80028ac:	f8c9 2050 	str.w	r2, [r9, #80]	@ 0x50
 80028b0:	2200      	movs	r2, #0
 80028b2:	e9c9 2115 	strd	r2, r1, [r9, #84]	@ 0x54
 80028b6:	460a      	mov	r2, r1
 80028b8:	4603      	mov	r3, r0
 80028ba:	e9d9 540e 	ldrd	r5, r4, [r9, #56]	@ 0x38
 80028be:	f8d9 6030 	ldr.w	r6, [r9, #48]	@ 0x30
 80028c2:	1952      	adds	r2, r2, r5
 80028c4:	4163      	adcs	r3, r4
 80028c6:	07f6      	lsls	r6, r6, #31
 80028c8:	d00c      	beq.n	80028e4 <smoltcp::iface::interface::Interface::poll+0x17c8>
 80028ca:	1a89      	subs	r1, r1, r2
 80028cc:	4198      	sbcs	r0, r3
 80028ce:	db09      	blt.n	80028e4 <smoltcp::iface::interface::Interface::poll+0x17c8>
 80028d0:	f240 0019 	movw	r0, #25
 80028d4:	f2c0 0000 	movt	r0, #0
 80028d8:	f001 fdcd 	bl	8004476 <defmt::export::acquire_header_and_release>
 80028dc:	2000      	movs	r0, #0
 80028de:	f889 0133 	strb.w	r0, [r9, #307]	@ 0x133
 80028e2:	e171      	b.n	8002bc8 <smoltcp::iface::interface::Interface::poll+0x1aac>
 80028e4:	984a      	ldr	r0, [sp, #296]	@ 0x128
 80028e6:	68c1      	ldr	r1, [r0, #12]
 80028e8:	4648      	mov	r0, r9
 80028ea:	f003 fdfb 	bl	80064e4 <smoltcp::socket::tcp::Socket::seq_to_transmit>
 80028ee:	2800      	cmp	r0, #0
 80028f0:	f040 816a 	bne.w	8002bc8 <smoltcp::iface::interface::Interface::poll+0x1aac>
 80028f4:	f8d9 0080 	ldr.w	r0, [r9, #128]	@ 0x80
 80028f8:	2802      	cmp	r0, #2
 80028fa:	f000 80df 	beq.w	8002abc <smoltcp::iface::interface::Interface::poll+0x19a0>
 80028fe:	2801      	cmp	r0, #1
 8002900:	f040 8162 	bne.w	8002bc8 <smoltcp::iface::interface::Interface::poll+0x1aac>
 8002904:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8002906:	e9d9 3222 	ldrd	r3, r2, [r9, #136]	@ 0x88
 800290a:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 800290e:	1ace      	subs	r6, r1, r3
 8002910:	eb70 0602 	sbcs.w	r6, r0, r2
 8002914:	f2c0 8158 	blt.w	8002bc8 <smoltcp::iface::interface::Interface::poll+0x1aac>
 8002918:	1ac9      	subs	r1, r1, r3
 800291a:	e9d9 6424 	ldrd	r6, r4, [r9, #144]	@ 0x90
 800291e:	4190      	sbcs	r0, r2
 8002920:	ea81 71e0 	eor.w	r1, r1, r0, asr #31
 8002924:	ea80 72e0 	eor.w	r2, r0, r0, asr #31
 8002928:	ebb1 71e0 	subs.w	r1, r1, r0, asr #31
 800292c:	eb62 70e0 	sbc.w	r0, r2, r0, asr #31
 8002930:	1875      	adds	r5, r6, r1
 8002932:	eb44 0600 	adc.w	r6, r4, r0
 8002936:	e0c3      	b.n	8002ac0 <smoltcp::iface::interface::Interface::poll+0x19a4>
 8002938:	2200      	movs	r2, #0
 800293a:	3001      	adds	r0, #1
 800293c:	f8dd a118 	ldr.w	sl, [sp, #280]	@ 0x118
 8002940:	fbb0 f3f1 	udiv	r3, r0, r1
 8002944:	f1bb 0b01 	subs.w	fp, fp, #1
 8002948:	fb03 0011 	mls	r0, r3, r1, r0
 800294c:	eba8 0106 	sub.w	r1, r8, r6
 8002950:	4665      	mov	r5, ip
 8002952:	e9c9 2112 	strd	r2, r1, [r9, #72]	@ 0x48
 8002956:	e9c9 0b0e 	strd	r0, fp, [r9, #56]	@ 0x38
 800295a:	d035      	beq.n	80029c8 <smoltcp::iface::interface::Interface::poll+0x18ac>
 800295c:	f8d9 3034 	ldr.w	r3, [r9, #52]	@ 0x34
 8002960:	2b00      	cmp	r3, #0
 8002962:	f000 8548 	beq.w	80033f6 <smoltcp::iface::interface::Interface::poll+0x22da>
 8002966:	f8d9 0038 	ldr.w	r0, [r9, #56]	@ 0x38
 800296a:	9548      	str	r5, [sp, #288]	@ 0x120
 800296c:	4298      	cmp	r0, r3
 800296e:	f080 8574 	bcs.w	800345a <smoltcp::iface::interface::Interface::poll+0x233e>
 8002972:	e9d9 5811 	ldrd	r5, r8, [r9, #68]	@ 0x44
 8002976:	f8d9 404c 	ldr.w	r4, [r9, #76]	@ 0x4c
 800297a:	eba5 0608 	sub.w	r6, r5, r8
 800297e:	42a6      	cmp	r6, r4
 8002980:	bf28      	it	cs
 8002982:	4626      	movcs	r6, r4
 8002984:	eb16 0108 	adds.w	r1, r6, r8
 8002988:	f080 84dc 	bcs.w	8003344 <smoltcp::iface::interface::Interface::poll+0x2228>
 800298c:	42a9      	cmp	r1, r5
 800298e:	f200 84d9 	bhi.w	8003344 <smoltcp::iface::interface::Interface::poll+0x2228>
 8002992:	f8d9 2030 	ldr.w	r2, [r9, #48]	@ 0x30
 8002996:	eb00 0180 	add.w	r1, r0, r0, lsl #2
 800299a:	f8cd b114 	str.w	fp, [sp, #276]	@ 0x114
 800299e:	eb02 0b81 	add.w	fp, r2, r1, lsl #2
 80029a2:	f89b 200e 	ldrb.w	r2, [fp, #14]
 80029a6:	2a02      	cmp	r2, #2
 80029a8:	f000 8500 	beq.w	80033ac <smoltcp::iface::interface::Interface::poll+0x2290>
 80029ac:	f8db 1000 	ldr.w	r1, [fp]
 80029b0:	42b1      	cmp	r1, r6
 80029b2:	f200 84cf 	bhi.w	8003354 <smoltcp::iface::interface::Interface::poll+0x2238>
 80029b6:	f8d9 c040 	ldr.w	ip, [r9, #64]	@ 0x40
 80029ba:	07d2      	lsls	r2, r2, #31
 80029bc:	aa40      	add	r2, sp, #256	@ 0x100
 80029be:	c219      	stmia	r2!, {r0, r3, r4}
 80029c0:	d006      	beq.n	80029d0 <smoltcp::iface::interface::Interface::poll+0x18b4>
 80029c2:	f8db 300f 	ldr.w	r3, [fp, #15]
 80029c6:	e014      	b.n	80029f2 <smoltcp::iface::interface::Interface::poll+0x18d6>
 80029c8:	2203      	movs	r2, #3
 80029ca:	f04f 0c00 	mov.w	ip, #0
 80029ce:	e2eb      	b.n	8002fa8 <smoltcp::iface::interface::Interface::poll+0x1e8c>
 80029d0:	ea5f 32ca 	movs.w	r2, sl, lsl #15
 80029d4:	d408      	bmi.n	80029e8 <smoltcp::iface::interface::Interface::poll+0x18cc>
 80029d6:	9a4a      	ldr	r2, [sp, #296]	@ 0x128
 80029d8:	f8d2 20f0 	ldr.w	r2, [r2, #240]	@ 0xf0
 80029dc:	2a00      	cmp	r2, #0
 80029de:	f000 82c7 	beq.w	8002f70 <smoltcp::iface::interface::Interface::poll+0x1e54>
 80029e2:	9a44      	ldr	r2, [sp, #272]	@ 0x110
 80029e4:	6813      	ldr	r3, [r2, #0]
 80029e6:	e004      	b.n	80029f2 <smoltcp::iface::interface::Interface::poll+0x18d6>
 80029e8:	983f      	ldr	r0, [sp, #252]	@ 0xfc
 80029ea:	ea4f 621a 	mov.w	r2, sl, lsr #24
 80029ee:	ea42 2300 	orr.w	r3, r2, r0, lsl #8
 80029f2:	eb0c 0008 	add.w	r0, ip, r8
 80029f6:	e9db 2a01 	ldrd	r2, sl, [fp, #4]
 80029fa:	f8bb c00c 	ldrh.w	ip, [fp, #12]
 80029fe:	2404      	movs	r4, #4
 8002a00:	e9cd 01c1 	strd	r0, r1, [sp, #772]	@ 0x304
 8002a04:	2003      	movs	r0, #3
 8002a06:	e9cd 20bf 	strd	r2, r0, [sp, #764]	@ 0x2fc
 8002a0a:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8002a0c:	90be      	str	r0, [sp, #760]	@ 0x2f8
 8002a0e:	68b8      	ldr	r0, [r7, #8]
 8002a10:	f88d 4361 	strb.w	r4, [sp, #865]	@ 0x361
 8002a14:	f101 0408 	add.w	r4, r1, #8
 8002a18:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 8002a1c:	e9cd 3ad5 	strd	r3, sl, [sp, #852]	@ 0x354
 8002a20:	9b46      	ldr	r3, [sp, #280]	@ 0x118
 8002a22:	4288      	cmp	r0, r1
 8002a24:	f88d e360 	strb.w	lr, [sp, #864]	@ 0x360
 8002a28:	94d7      	str	r4, [sp, #860]	@ 0x35c
 8002a2a:	f8cd a11c 	str.w	sl, [sp, #284]	@ 0x11c
 8002a2e:	f8ad c30e 	strh.w	ip, [sp, #782]	@ 0x30e
 8002a32:	f8ad 330c 	strh.w	r3, [sp, #780]	@ 0x30c
 8002a36:	f4be abd0 	bcs.w	80011da <smoltcp::iface::interface::Interface::poll+0xbe>
 8002a3a:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 8002a3e:	9b43      	ldr	r3, [sp, #268]	@ 0x10c
 8002a40:	4348      	muls	r0, r1
 8002a42:	6819      	ldr	r1, [r3, #0]
 8002a44:	5808      	ldr	r0, [r1, r0]
 8002a46:	2800      	cmp	r0, #0
 8002a48:	d415      	bmi.n	8002a76 <smoltcp::iface::interface::Interface::poll+0x195a>
 8002a4a:	2000      	movs	r0, #0
 8002a4c:	93a7      	str	r3, [sp, #668]	@ 0x29c
 8002a4e:	90a5      	str	r0, [sp, #660]	@ 0x294
 8002a50:	a9a5      	add	r1, sp, #660	@ 0x294
 8002a52:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8002a54:	9b38      	ldr	r3, [sp, #224]	@ 0xe0
 8002a56:	f7fd fc81 	bl	800035c <smoltcp::iface::interface::InterfaceInner::dispatch_ip>
 8002a5a:	b2c0      	uxtb	r0, r0
 8002a5c:	2802      	cmp	r0, #2
 8002a5e:	d112      	bne.n	8002a86 <smoltcp::iface::interface::Interface::poll+0x196a>
 8002a60:	f8db 1000 	ldr.w	r1, [fp]
 8002a64:	f04f 0c01 	mov.w	ip, #1
 8002a68:	9c42      	ldr	r4, [sp, #264]	@ 0x108
 8002a6a:	9845      	ldr	r0, [sp, #276]	@ 0x114
 8002a6c:	42b1      	cmp	r1, r6
 8002a6e:	f240 8285 	bls.w	8002f7c <smoltcp::iface::interface::Interface::poll+0x1e60>
 8002a72:	f000 bc8a 	b.w	800338a <smoltcp::iface::interface::Interface::poll+0x226e>
 8002a76:	f240 0025 	movw	r0, #37	@ 0x25
 8002a7a:	f2c0 0000 	movt	r0, #0
 8002a7e:	f001 fcfa 	bl	8004476 <defmt::export::acquire_header_and_release>
 8002a82:	2200      	movs	r2, #0
 8002a84:	e000      	b.n	8002a88 <smoltcp::iface::interface::Interface::poll+0x196c>
 8002a86:	2201      	movs	r2, #1
 8002a88:	9c42      	ldr	r4, [sp, #264]	@ 0x108
 8002a8a:	2100      	movs	r1, #0
 8002a8c:	9845      	ldr	r0, [sp, #276]	@ 0x114
 8002a8e:	f04f 0c01 	mov.w	ip, #1
 8002a92:	2d00      	cmp	r5, #0
 8002a94:	f000 8276 	beq.w	8002f84 <smoltcp::iface::interface::Interface::poll+0x1e68>
 8002a98:	eb01 0308 	add.w	r3, r1, r8
 8002a9c:	fbb3 f6f5 	udiv	r6, r3, r5
 8002aa0:	fb06 3315 	mls	r3, r6, r5, r3
 8002aa4:	e26f      	b.n	8002f86 <smoltcp::iface::interface::Interface::poll+0x1e6a>
 8002aa6:	f103 0050 	add.w	r0, r3, #80	@ 0x50
 8002aaa:	e9d0 0102 	ldrd	r0, r1, [r0, #8]
 8002aae:	ebba 0000 	subs.w	r0, sl, r0
 8002ab2:	eb7b 0001 	sbcs.w	r0, fp, r1
 8002ab6:	f6ff ae87 	blt.w	80027c8 <smoltcp::iface::interface::Interface::poll+0x16ac>
 8002aba:	e693      	b.n	80027e4 <smoltcp::iface::interface::Interface::poll+0x16c8>
 8002abc:	2500      	movs	r5, #0
 8002abe:	2600      	movs	r6, #0
 8002ac0:	f240 0015 	movw	r0, #21
 8002ac4:	f2c0 0000 	movt	r0, #0
 8002ac8:	f001 fcc4 	bl	8004454 <defmt::export::acquire_and_header>
 8002acc:	f240 0007 	movw	r0, #7
 8002ad0:	acbe      	add	r4, sp, #760	@ 0x2f8
 8002ad2:	f2c0 0000 	movt	r0, #0
 8002ad6:	2102      	movs	r1, #2
 8002ad8:	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
 8002adc:	4620      	mov	r0, r4
 8002ade:	f001 fe6b 	bl	80047b8 <_defmt_write>
 8002ae2:	f240 0040 	movw	r0, #64	@ 0x40
 8002ae6:	2102      	movs	r1, #2
 8002ae8:	f2c0 0000 	movt	r0, #0
 8002aec:	f8ad 02f8 	strh.w	r0, [sp, #760]	@ 0x2f8
 8002af0:	4620      	mov	r0, r4
 8002af2:	f001 fe61 	bl	80047b8 <_defmt_write>
 8002af6:	f240 0805 	movw	r8, #5
 8002afa:	4620      	mov	r0, r4
 8002afc:	f2c0 0800 	movt	r8, #0
 8002b00:	2102      	movs	r1, #2
 8002b02:	f8ad 82f8 	strh.w	r8, [sp, #760]	@ 0x2f8
 8002b06:	f001 fe57 	bl	80047b8 <_defmt_write>
 8002b0a:	f244 2240 	movw	r2, #16960	@ 0x4240
 8002b0e:	4628      	mov	r0, r5
 8002b10:	4631      	mov	r1, r6
 8002b12:	f2c0 020f 	movt	r2, #15
 8002b16:	2300      	movs	r3, #0
 8002b18:	f006 fd2a 	bl	8009570 <__aeabi_uldivmod>
 8002b1c:	e9cd 01be 	strd	r0, r1, [sp, #760]	@ 0x2f8
 8002b20:	4620      	mov	r0, r4
 8002b22:	2108      	movs	r1, #8
 8002b24:	f001 fe48 	bl	80047b8 <_defmt_write>
 8002b28:	4620      	mov	r0, r4
 8002b2a:	2102      	movs	r1, #2
 8002b2c:	f8ad 82f8 	strh.w	r8, [sp, #760]	@ 0x2f8
 8002b30:	f001 fe42 	bl	80047b8 <_defmt_write>
 8002b34:	4628      	mov	r0, r5
 8002b36:	4631      	mov	r1, r6
 8002b38:	f44f 727a 	mov.w	r2, #1000	@ 0x3e8
 8002b3c:	2300      	movs	r3, #0
 8002b3e:	f006 fd17 	bl	8009570 <__aeabi_uldivmod>
 8002b42:	f44f 727a 	mov.w	r2, #1000	@ 0x3e8
 8002b46:	2300      	movs	r3, #0
 8002b48:	f006 fd12 	bl	8009570 <__aeabi_uldivmod>
 8002b4c:	4620      	mov	r0, r4
 8002b4e:	2108      	movs	r1, #8
 8002b50:	e9cd 23be 	strd	r2, r3, [sp, #760]	@ 0x2f8
 8002b54:	f001 fe30 	bl	80047b8 <_defmt_write>
 8002b58:	2500      	movs	r5, #0
 8002b5a:	4620      	mov	r0, r4
 8002b5c:	2102      	movs	r1, #2
 8002b5e:	f8ad 52f8 	strh.w	r5, [sp, #760]	@ 0x2f8
 8002b62:	f001 fe29 	bl	80047b8 <_defmt_write>
 8002b66:	f001 fdaf 	bl	80046c8 <_defmt_release>
 8002b6a:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8002b6c:	e9d9 6312 	ldrd	r6, r3, [r9, #72]	@ 0x48
 8002b70:	e9d0 0104 	ldrd	r0, r1, [r0, #16]
 8002b74:	f8d9 4100 	ldr.w	r4, [r9, #256]	@ 0x100
 8002b78:	1980      	adds	r0, r0, r6
 8002b7a:	f999 2028 	ldrsb.w	r2, [r9, #40]	@ 0x28
 8002b7e:	f8c9 4108 	str.w	r4, [r9, #264]	@ 0x108
 8002b82:	f04f 0401 	mov.w	r4, #1
 8002b86:	4159      	adcs	r1, r3
 8002b88:	e9d9 ce10 	ldrd	ip, lr, [r9, #64]	@ 0x40
 8002b8c:	fa82 f254 	uqadd8	r2, r2, r4
 8002b90:	e9c9 5500 	strd	r5, r5, [r9]
 8002b94:	e9c9 0124 	strd	r0, r1, [r9, #144]	@ 0x90
 8002b98:	b2d0      	uxtb	r0, r2
 8002b9a:	2803      	cmp	r0, #3
 8002b9c:	f889 2028 	strb.w	r2, [r9, #40]	@ 0x28
 8002ba0:	e9c9 5520 	strd	r5, r5, [r9, #128]	@ 0x80
 8002ba4:	e9c9 ce22 	strd	ip, lr, [r9, #136]	@ 0x88
 8002ba8:	d30e      	bcc.n	8002bc8 <smoltcp::iface::interface::Interface::poll+0x1aac>
 8002baa:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8002bae:	2100      	movs	r1, #0
 8002bb0:	f889 1028 	strb.w	r1, [r9, #40]	@ 0x28
 8002bb4:	f242 7210 	movw	r2, #10000	@ 0x2710
 8002bb8:	0041      	lsls	r1, r0, #1
 8002bba:	4291      	cmp	r1, r2
 8002bbc:	f242 7110 	movw	r1, #10000	@ 0x2710
 8002bc0:	bf38      	it	cc
 8002bc2:	0041      	lslcc	r1, r0, #1
 8002bc4:	f8c9 1020 	str.w	r1, [r9, #32]
 8002bc8:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8002bca:	68c1      	ldr	r1, [r0, #12]
 8002bcc:	4648      	mov	r0, r9
 8002bce:	f003 fc89 	bl	80064e4 <smoltcp::socket::tcp::Socket::seq_to_transmit>
 8002bd2:	2800      	cmp	r0, #0
 8002bd4:	d03e      	beq.n	8002c54 <smoltcp::iface::interface::Interface::poll+0x1b38>
 8002bd6:	f8b9 011c 	ldrh.w	r0, [r9, #284]	@ 0x11c
 8002bda:	07c0      	lsls	r0, r0, #31
 8002bdc:	f000 83e0 	beq.w	80033a0 <smoltcp::iface::interface::Interface::poll+0x2284>
 8002be0:	f899 112a 	ldrb.w	r1, [r9, #298]	@ 0x12a
 8002be4:	f44f 7a50 	mov.w	sl, #832	@ 0x340
 8002be8:	f8d9 40c4 	ldr.w	r4, [r9, #196]	@ 0xc4
 8002bec:	2900      	cmp	r1, #0
 8002bee:	f899 012b 	ldrb.w	r0, [r9, #299]	@ 0x12b
 8002bf2:	bf18      	it	ne
 8002bf4:	f500 7a40 	addne.w	sl, r0, #768	@ 0x300
 8002bf8:	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
 8002bfc:	f340 836d 	ble.w	80032da <smoltcp::iface::interface::Interface::poll+0x21be>
 8002c00:	f8d9 011e 	ldr.w	r0, [r9, #286]	@ 0x11e
 8002c04:	903c      	str	r0, [sp, #240]	@ 0xf0
 8002c06:	f8d9 0124 	ldr.w	r0, [r9, #292]	@ 0x124
 8002c0a:	9041      	str	r0, [sp, #260]	@ 0x104
 8002c0c:	f8b9 0122 	ldrh.w	r0, [r9, #290]	@ 0x122
 8002c10:	903b      	str	r0, [sp, #236]	@ 0xec
 8002c12:	f8b9 0128 	ldrh.w	r0, [r9, #296]	@ 0x128
 8002c16:	f899 1134 	ldrb.w	r1, [r9, #308]	@ 0x134
 8002c1a:	903a      	str	r0, [sp, #232]	@ 0xe8
 8002c1c:	f8d9 00bc 	ldr.w	r0, [r9, #188]	@ 0xbc
 8002c20:	f001 011f 	and.w	r1, r1, #31
 8002c24:	e9d9 5641 	ldrd	r5, r6, [r9, #260]	@ 0x104
 8002c28:	1b00      	subs	r0, r0, r4
 8002c2a:	f8d9 2114 	ldr.w	r2, [r9, #276]	@ 0x114
 8002c2e:	fa20 f101 	lsr.w	r1, r0, r1
 8002c32:	f64f 70ff 	movw	r0, #65535	@ 0xffff
 8002c36:	4281      	cmp	r1, r0
 8002c38:	bf28      	it	cs
 8002c3a:	4601      	movcs	r1, r0
 8002c3c:	2a00      	cmp	r2, #0
 8002c3e:	9142      	str	r1, [sp, #264]	@ 0x108
 8002c40:	923d      	str	r2, [sp, #244]	@ 0xf4
 8002c42:	f000 80c1 	beq.w	8002dc8 <smoltcp::iface::interface::Interface::poll+0x1cac>
 8002c46:	f8d9 0118 	ldr.w	r0, [r9, #280]	@ 0x118
 8002c4a:	9037      	str	r0, [sp, #220]	@ 0xdc
 8002c4c:	4790      	blx	r2
 8002c4e:	9036      	str	r0, [sp, #216]	@ 0xd8
 8002c50:	2001      	movs	r0, #1
 8002c52:	e0ba      	b.n	8002dca <smoltcp::iface::interface::Interface::poll+0x1cae>
 8002c54:	f8d9 1098 	ldr.w	r1, [r9, #152]	@ 0x98
 8002c58:	2901      	cmp	r1, #1
 8002c5a:	d10f      	bne.n	8002c7c <smoltcp::iface::interface::Interface::poll+0x1b60>
 8002c5c:	f8d9 00c4 	ldr.w	r0, [r9, #196]	@ 0xc4
 8002c60:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8002c64:	f340 8339 	ble.w	80032da <smoltcp::iface::interface::Interface::poll+0x21be>
 8002c68:	f8d9 3104 	ldr.w	r3, [r9, #260]	@ 0x104
 8002c6c:	f8d9 209c 	ldr.w	r2, [r9, #156]	@ 0x9c
 8002c70:	4418      	add	r0, r3
 8002c72:	1a10      	subs	r0, r2, r0
 8002c74:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8002c78:	f340 814f 	ble.w	8002f1a <smoltcp::iface::interface::Interface::poll+0x1dfe>
 8002c7c:	f899 0133 	ldrb.w	r0, [r9, #307]	@ 0x133
 8002c80:	1e82      	subs	r2, r0, #2
 8002c82:	2a04      	cmp	r2, #4
 8002c84:	d834      	bhi.n	8002cf0 <smoltcp::iface::interface::Interface::poll+0x1bd4>
 8002c86:	f899 2134 	ldrb.w	r2, [r9, #308]	@ 0x134
 8002c8a:	f8d9 30bc 	ldr.w	r3, [r9, #188]	@ 0xbc
 8002c8e:	f8d9 50c4 	ldr.w	r5, [r9, #196]	@ 0xc4
 8002c92:	f002 021f 	and.w	r2, r2, #31
 8002c96:	1b5b      	subs	r3, r3, r5
 8002c98:	fa23 f602 	lsr.w	r6, r3, r2
 8002c9c:	f64f 73ff 	movw	r3, #65535	@ 0xffff
 8002ca0:	429e      	cmp	r6, r3
 8002ca2:	bf38      	it	cc
 8002ca4:	4633      	movcc	r3, r6
 8002ca6:	b319      	cbz	r1, 8002cf0 <smoltcp::iface::interface::Interface::poll+0x1bd4>
 8002ca8:	f1b5 3fff 	cmp.w	r5, #4294967295	@ 0xffffffff
 8002cac:	bfc2      	ittt	gt
 8002cae:	f8b9 112e 	ldrhgt.w	r1, [r9, #302]	@ 0x12e
 8002cb2:	4091      	lslgt	r1, r2
 8002cb4:	f1b1 3fff 	cmpgt.w	r1, #4294967295	@ 0xffffffff
 8002cb8:	f340 830f 	ble.w	80032da <smoltcp::iface::interface::Interface::poll+0x21be>
 8002cbc:	f8d9 409c 	ldr.w	r4, [r9, #156]	@ 0x9c
 8002cc0:	f8d9 c104 	ldr.w	ip, [r9, #260]	@ 0x104
 8002cc4:	4421      	add	r1, r4
 8002cc6:	4465      	add	r5, ip
 8002cc8:	1b49      	subs	r1, r1, r5
 8002cca:	f1b1 3fff 	cmp.w	r1, #4294967295	@ 0xffffffff
 8002cce:	f340 8373 	ble.w	80033b8 <smoltcp::iface::interface::Interface::poll+0x229c>
 8002cd2:	b16e      	cbz	r6, 8002cf0 <smoltcp::iface::interface::Interface::poll+0x1bd4>
 8002cd4:	40d1      	lsrs	r1, r2
 8002cd6:	f64f 72ff 	movw	r2, #65535	@ 0xffff
 8002cda:	4291      	cmp	r1, r2
 8002cdc:	bf28      	it	cs
 8002cde:	4611      	movcs	r1, r2
 8002ce0:	ebb1 0f53 	cmp.w	r1, r3, lsr #1
 8002ce4:	f04f 0100 	mov.w	r1, #0
 8002ce8:	bf98      	it	ls
 8002cea:	2101      	movls	r1, #1
 8002cec:	b121      	cbz	r1, 8002cf8 <smoltcp::iface::interface::Interface::poll+0x1bdc>
 8002cee:	e772      	b.n	8002bd6 <smoltcp::iface::interface::Interface::poll+0x1aba>
 8002cf0:	2100      	movs	r1, #0
 8002cf2:	2900      	cmp	r1, #0
 8002cf4:	f47f af6f 	bne.w	8002bd6 <smoltcp::iface::interface::Interface::poll+0x1aba>
 8002cf8:	2800      	cmp	r0, #0
 8002cfa:	f43f af6c 	beq.w	8002bd6 <smoltcp::iface::interface::Interface::poll+0x1aba>
 8002cfe:	4648      	mov	r0, r9
 8002d00:	994a      	ldr	r1, [sp, #296]	@ 0x128
 8002d02:	f850 5f80 	ldr.w	r5, [r0, #128]!
 8002d06:	f100 0c04 	add.w	ip, r0, #4
 8002d0a:	e9d1 2e04 	ldrd	r2, lr, [r1, #16]
 8002d0e:	e89c 1050 	ldmia.w	ip, {r4, r6, ip}
 8002d12:	ea55 0304 	orrs.w	r3, r5, r4
 8002d16:	d108      	bne.n	8002d2a <smoltcp::iface::interface::Interface::poll+0x1c0e>
 8002d18:	07f3      	lsls	r3, r6, #31
 8002d1a:	d006      	beq.n	8002d2a <smoltcp::iface::interface::Interface::poll+0x1c0e>
 8002d1c:	e9d0 3104 	ldrd	r3, r1, [r0, #16]
 8002d20:	1ad3      	subs	r3, r2, r3
 8002d22:	eb7e 0101 	sbcs.w	r1, lr, r1
 8002d26:	f6bf af56 	bge.w	8002bd6 <smoltcp::iface::interface::Interface::poll+0x1aba>
 8002d2a:	f085 0103 	eor.w	r1, r5, #3
 8002d2e:	9d48      	ldr	r5, [sp, #288]	@ 0x120
 8002d30:	4321      	orrs	r1, r4
 8002d32:	f47f acc5 	bne.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 8002d36:	1b91      	subs	r1, r2, r6
 8002d38:	eb7e 010c 	sbcs.w	r1, lr, ip
 8002d3c:	f6ff acc0 	blt.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 8002d40:	462e      	mov	r6, r5
 8002d42:	2500      	movs	r5, #0
 8002d44:	e9c0 5500 	strd	r5, r5, [r0]
 8002d48:	f44f 7196 	mov.w	r1, #300	@ 0x12c
 8002d4c:	e9c0 5502 	strd	r5, r5, [r0, #8]
 8002d50:	2064      	movs	r0, #100	@ 0x64
 8002d52:	e9c9 1008 	strd	r1, r0, [r9, #32]
 8002d56:	f109 00d0 	add.w	r0, r9, #208	@ 0xd0
 8002d5a:	2128      	movs	r1, #40	@ 0x28
 8002d5c:	e9c9 5500 	strd	r5, r5, [r9]
 8002d60:	f889 5133 	strb.w	r5, [r9, #307]	@ 0x133
 8002d64:	f8c9 5018 	str.w	r5, [r9, #24]
 8002d68:	f889 5130 	strb.w	r5, [r9, #304]	@ 0x130
 8002d6c:	e9c9 5530 	strd	r5, r5, [r9, #192]	@ 0xc0
 8002d70:	f889 5028 	strb.w	r5, [r9, #40]	@ 0x28
 8002d74:	f8d9 40bc 	ldr.w	r4, [r9, #188]	@ 0xbc
 8002d78:	f8a9 511c 	strh.w	r5, [r9, #284]	@ 0x11c
 8002d7c:	f889 50b2 	strb.w	r5, [r9, #178]	@ 0xb2
 8002d80:	f8a9 50b0 	strh.w	r5, [r9, #176]	@ 0xb0
 8002d84:	f8a9 512e 	strh.w	r5, [r9, #302]	@ 0x12e
 8002d88:	f8c9 5098 	str.w	r5, [r9, #152]	@ 0x98
 8002d8c:	f889 512c 	strb.w	r5, [r9, #300]	@ 0x12c
 8002d90:	e9c9 5540 	strd	r5, r5, [r9, #256]	@ 0x100
 8002d94:	e9c9 5542 	strd	r5, r5, [r9, #264]	@ 0x108
 8002d98:	f005 fdad 	bl	80088f6 <__aeabi_memclr8>
 8002d9c:	f44f 7006 	mov.w	r0, #536	@ 0x218
 8002da0:	e9c9 553e 	strd	r5, r5, [r9, #248]	@ 0xf8
 8002da4:	f8c9 0110 	str.w	r0, [r9, #272]	@ 0x110
 8002da8:	fab4 f084 	clz	r0, r4
 8002dac:	f1c0 0110 	rsb	r1, r0, #16
 8002db0:	2210      	movs	r2, #16
 8002db2:	e9c9 551c 	strd	r5, r5, [r9, #112]	@ 0x70
 8002db6:	4282      	cmp	r2, r0
 8002db8:	e9c9 5514 	strd	r5, r5, [r9, #80]	@ 0x50
 8002dbc:	bf38      	it	cc
 8002dbe:	4629      	movcc	r1, r5
 8002dc0:	4635      	mov	r5, r6
 8002dc2:	f889 1134 	strb.w	r1, [r9, #308]	@ 0x134
 8002dc6:	e47b      	b.n	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 8002dc8:	2000      	movs	r0, #0
 8002dca:	9039      	str	r0, [sp, #228]	@ 0xe4
 8002dcc:	1960      	adds	r0, r4, r5
 8002dce:	f899 4133 	ldrb.w	r4, [r9, #307]	@ 0x133
 8002dd2:	f04f 0800 	mov.w	r8, #0
 8002dd6:	903f      	str	r0, [sp, #252]	@ 0xfc
 8002dd8:	2001      	movs	r0, #1
 8002dda:	963e      	str	r6, [sp, #248]	@ 0xf8
 8002ddc:	2200      	movs	r2, #0
 8002dde:	9045      	str	r0, [sp, #276]	@ 0x114
 8002de0:	2001      	movs	r0, #1
 8002de2:	9040      	str	r0, [sp, #256]	@ 0x100
 8002de4:	2000      	movs	r0, #0
 8002de6:	2300      	movs	r3, #0
 8002de8:	9046      	str	r0, [sp, #280]	@ 0x118
 8002dea:	f04f 0b01 	mov.w	fp, #1
 8002dee:	f04f 0e00 	mov.w	lr, #0
 8002df2:	9d48      	ldr	r5, [sp, #288]	@ 0x120
 8002df4:	e8df f014 	tbh	[pc, r4, lsl #1]
 8002df8:	000b00a3 	.word	0x000b00a3
 8002dfc:	00510051 	.word	0x00510051
 8002e00:	000c000c 	.word	0x000c000c
 8002e04:	000c00f0 	.word	0x000c00f0
 8002e08:	000c000c 	.word	0x000c000c
 8002e0c:	00f0      	.short	0x00f0
 8002e0e:	e457      	b.n	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 8002e10:	f8d9 010c 	ldr.w	r0, [r9, #268]	@ 0x10c
 8002e14:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8002e18:	f340 825f 	ble.w	80032da <smoltcp::iface::interface::Interface::poll+0x21be>
 8002e1c:	f8d9 2100 	ldr.w	r2, [r9, #256]	@ 0x100
 8002e20:	994a      	ldr	r1, [sp, #296]	@ 0x128
 8002e22:	f8d9 3108 	ldr.w	r3, [r9, #264]	@ 0x108
 8002e26:	4410      	add	r0, r2
 8002e28:	f8d9 5110 	ldr.w	r5, [r9, #272]	@ 0x110
 8002e2c:	1ac0      	subs	r0, r0, r3
 8002e2e:	68ce      	ldr	r6, [r1, #12]
 8002e30:	ea20 71e0 	bic.w	r1, r0, r0, asr #31
 8002e34:	f1a6 0036 	sub.w	r0, r6, #54	@ 0x36
 8002e38:	428d      	cmp	r5, r1
 8002e3a:	bf38      	it	cc
 8002e3c:	4629      	movcc	r1, r5
 8002e3e:	4288      	cmp	r0, r1
 8002e40:	bf38      	it	cc
 8002e42:	4601      	movcc	r1, r0
 8002e44:	1a9b      	subs	r3, r3, r2
 8002e46:	f100 82b7 	bmi.w	80033b8 <smoltcp::iface::interface::Interface::poll+0x229c>
 8002e4a:	f8d9 20cc 	ldr.w	r2, [r9, #204]	@ 0xcc
 8002e4e:	2a00      	cmp	r2, #0
 8002e50:	d03f      	beq.n	8002ed2 <smoltcp::iface::interface::Interface::poll+0x1db6>
 8002e52:	f8d9 00d0 	ldr.w	r0, [r9, #208]	@ 0xd0
 8002e56:	4418      	add	r0, r3
 8002e58:	fbb0 f6f2 	udiv	r6, r0, r2
 8002e5c:	fb06 0012 	mls	r0, r6, r2, r0
 8002e60:	f8d9 50d4 	ldr.w	r5, [r9, #212]	@ 0xd4
 8002e64:	429d      	cmp	r5, r3
 8002e66:	d239      	bcs.n	8002edc <smoltcp::iface::interface::Interface::poll+0x1dc0>
 8002e68:	2001      	movs	r0, #1
 8002e6a:	9040      	str	r0, [sp, #256]	@ 0x100
 8002e6c:	eb08 0003 	add.w	r0, r8, r3
 8002e70:	42a8      	cmp	r0, r5
 8002e72:	d14b      	bne.n	8002f0c <smoltcp::iface::interface::Interface::poll+0x1df0>
 8002e74:	3c04      	subs	r4, #4
 8002e76:	2001      	movs	r0, #1
 8002e78:	2200      	movs	r2, #0
 8002e7a:	2c05      	cmp	r4, #5
 8002e7c:	9045      	str	r0, [sp, #276]	@ 0x114
 8002e7e:	d848      	bhi.n	8002f12 <smoltcp::iface::interface::Interface::poll+0x1df6>
 8002e80:	2100      	movs	r1, #0
 8002e82:	2300      	movs	r3, #0
 8002e84:	9146      	str	r1, [sp, #280]	@ 0x118
 8002e86:	e8df f004 	tbb	[pc, r4]
 8002e8a:	03a0      	.short	0x03a0
 8002e8c:	0303a0a7 	.word	0x0303a0a7
 8002e90:	2003      	movs	r0, #3
 8002e92:	9046      	str	r0, [sp, #280]	@ 0x118
 8002e94:	2001      	movs	r0, #1
 8002e96:	9045      	str	r0, [sp, #276]	@ 0x114
 8002e98:	e056      	b.n	8002f48 <smoltcp::iface::interface::Interface::poll+0x1e2c>
 8002e9a:	f8d9 00bc 	ldr.w	r0, [r9, #188]	@ 0xbc
 8002e9e:	f8d9 10c4 	ldr.w	r1, [r9, #196]	@ 0xc4
 8002ea2:	f8d9 2100 	ldr.w	r2, [r9, #256]	@ 0x100
 8002ea6:	1a41      	subs	r1, r0, r1
 8002ea8:	f64f 70ff 	movw	r0, #65535	@ 0xffff
 8002eac:	923e      	str	r2, [sp, #248]	@ 0xf8
 8002eae:	4281      	cmp	r1, r0
 8002eb0:	bf28      	it	cs
 8002eb2:	4601      	movcs	r1, r0
 8002eb4:	2001      	movs	r0, #1
 8002eb6:	2c02      	cmp	r4, #2
 8002eb8:	9142      	str	r1, [sp, #264]	@ 0x108
 8002eba:	d148      	bne.n	8002f4e <smoltcp::iface::interface::Interface::poll+0x1e32>
 8002ebc:	9040      	str	r0, [sp, #256]	@ 0x100
 8002ebe:	2201      	movs	r2, #1
 8002ec0:	f899 0134 	ldrb.w	r0, [r9, #308]	@ 0x134
 8002ec4:	2301      	movs	r3, #1
 8002ec6:	9032      	str	r0, [sp, #200]	@ 0xc8
 8002ec8:	2000      	movs	r0, #0
 8002eca:	9045      	str	r0, [sp, #276]	@ 0x114
 8002ecc:	2002      	movs	r0, #2
 8002ece:	9046      	str	r0, [sp, #280]	@ 0x118
 8002ed0:	e049      	b.n	8002f66 <smoltcp::iface::interface::Interface::poll+0x1e4a>
 8002ed2:	2000      	movs	r0, #0
 8002ed4:	f8d9 50d4 	ldr.w	r5, [r9, #212]	@ 0xd4
 8002ed8:	429d      	cmp	r5, r3
 8002eda:	d3c5      	bcc.n	8002e68 <smoltcp::iface::interface::Interface::poll+0x1d4c>
 8002edc:	eba5 0803 	sub.w	r8, r5, r3
 8002ee0:	4541      	cmp	r1, r8
 8002ee2:	bf38      	it	cc
 8002ee4:	4688      	movcc	r8, r1
 8002ee6:	1a11      	subs	r1, r2, r0
 8002ee8:	4588      	cmp	r8, r1
 8002eea:	bf28      	it	cs
 8002eec:	4688      	movcs	r8, r1
 8002eee:	eb18 0100 	adds.w	r1, r8, r0
 8002ef2:	f080 826c 	bcs.w	80033ce <smoltcp::iface::interface::Interface::poll+0x22b2>
 8002ef6:	4291      	cmp	r1, r2
 8002ef8:	f200 8269 	bhi.w	80033ce <smoltcp::iface::interface::Interface::poll+0x22b2>
 8002efc:	f8d9 10c8 	ldr.w	r1, [r9, #200]	@ 0xc8
 8002f00:	4408      	add	r0, r1
 8002f02:	9040      	str	r0, [sp, #256]	@ 0x100
 8002f04:	eb08 0003 	add.w	r0, r8, r3
 8002f08:	42a8      	cmp	r0, r5
 8002f0a:	d0b3      	beq.n	8002e74 <smoltcp::iface::interface::Interface::poll+0x1d58>
 8002f0c:	2001      	movs	r0, #1
 8002f0e:	2200      	movs	r2, #0
 8002f10:	9045      	str	r0, [sp, #276]	@ 0x114
 8002f12:	2300      	movs	r3, #0
 8002f14:	2000      	movs	r0, #0
 8002f16:	9046      	str	r0, [sp, #280]	@ 0x118
 8002f18:	e05e      	b.n	8002fd8 <smoltcp::iface::interface::Interface::poll+0x1ebc>
 8002f1a:	e9d9 021c 	ldrd	r0, r2, [r9, #112]	@ 0x70
 8002f1e:	f080 0001 	eor.w	r0, r0, #1
 8002f22:	4310      	orrs	r0, r2
 8002f24:	f47f ae57 	bne.w	8002bd6 <smoltcp::iface::interface::Interface::poll+0x1aba>
 8002f28:	9b4a      	ldr	r3, [sp, #296]	@ 0x128
 8002f2a:	e9d9 021e 	ldrd	r0, r2, [r9, #120]	@ 0x78
 8002f2e:	e9d3 3604 	ldrd	r3, r6, [r3, #16]
 8002f32:	1a18      	subs	r0, r3, r0
 8002f34:	eb76 0002 	sbcs.w	r0, r6, r2
 8002f38:	f6bf ae4d 	bge.w	8002bd6 <smoltcp::iface::interface::Interface::poll+0x1aba>
 8002f3c:	e69e      	b.n	8002c7c <smoltcp::iface::interface::Interface::poll+0x1b60>
 8002f3e:	2004      	movs	r0, #4
 8002f40:	9046      	str	r0, [sp, #280]	@ 0x118
 8002f42:	2001      	movs	r0, #1
 8002f44:	9045      	str	r0, [sp, #276]	@ 0x114
 8002f46:	9040      	str	r0, [sp, #256]	@ 0x100
 8002f48:	f04f 0b00 	mov.w	fp, #0
 8002f4c:	e044      	b.n	8002fd8 <smoltcp::iface::interface::Interface::poll+0x1ebc>
 8002f4e:	9045      	str	r0, [sp, #276]	@ 0x114
 8002f50:	f899 0134 	ldrb.w	r0, [r9, #308]	@ 0x134
 8002f54:	9032      	str	r0, [sp, #200]	@ 0xc8
 8002f56:	2002      	movs	r0, #2
 8002f58:	9046      	str	r0, [sp, #280]	@ 0x118
 8002f5a:	2001      	movs	r0, #1
 8002f5c:	f899 212c 	ldrb.w	r2, [r9, #300]	@ 0x12c
 8002f60:	f899 3131 	ldrb.w	r3, [r9, #305]	@ 0x131
 8002f64:	9040      	str	r0, [sp, #256]	@ 0x100
 8002f66:	f04f 0b00 	mov.w	fp, #0
 8002f6a:	f04f 0e01 	mov.w	lr, #1
 8002f6e:	e033      	b.n	8002fd8 <smoltcp::iface::interface::Interface::poll+0x1ebc>
 8002f70:	f04f 0c00 	mov.w	ip, #0
 8002f74:	9845      	ldr	r0, [sp, #276]	@ 0x114
 8002f76:	42b1      	cmp	r1, r6
 8002f78:	f200 8207 	bhi.w	800338a <smoltcp::iface::interface::Interface::poll+0x226e>
 8002f7c:	2202      	movs	r2, #2
 8002f7e:	2d00      	cmp	r5, #0
 8002f80:	f47f ad8a 	bne.w	8002a98 <smoltcp::iface::interface::Interface::poll+0x197c>
 8002f84:	2300      	movs	r3, #0
 8002f86:	9d48      	ldr	r5, [sp, #288]	@ 0x120
 8002f88:	1a61      	subs	r1, r4, r1
 8002f8a:	2a02      	cmp	r2, #2
 8002f8c:	e9c9 3112 	strd	r3, r1, [r9, #72]	@ 0x48
 8002f90:	d10a      	bne.n	8002fa8 <smoltcp::iface::interface::Interface::poll+0x1e8c>
 8002f92:	9940      	ldr	r1, [sp, #256]	@ 0x100
 8002f94:	9b41      	ldr	r3, [sp, #260]	@ 0x104
 8002f96:	3101      	adds	r1, #1
 8002f98:	fbb1 f2f3 	udiv	r2, r1, r3
 8002f9c:	fb02 1113 	mls	r1, r2, r3, r1
 8002fa0:	1e42      	subs	r2, r0, #1
 8002fa2:	e9c9 120e 	strd	r1, r2, [r9, #56]	@ 0x38
 8002fa6:	2202      	movs	r2, #2
 8002fa8:	2a02      	cmp	r2, #2
 8002faa:	f04f 0102 	mov.w	r1, #2
 8002fae:	bf28      	it	cs
 8002fb0:	460a      	movcs	r2, r1
 8002fb2:	2a02      	cmp	r2, #2
 8002fb4:	f43f ab84 	beq.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 8002fb8:	07d1      	lsls	r1, r2, #31
 8002fba:	f000 819f 	beq.w	80032fc <smoltcp::iface::interface::Interface::poll+0x21e0>
 8002fbe:	9a47      	ldr	r2, [sp, #284]	@ 0x11c
 8002fc0:	f1bc 0f00 	cmp.w	ip, #0
 8002fc4:	f040 80f3 	bne.w	80031ae <smoltcp::iface::interface::Interface::poll+0x2092>
 8002fc8:	e1cc      	b.n	8003364 <smoltcp::iface::interface::Interface::poll+0x2248>
 8002fca:	f1b8 0f00 	cmp.w	r8, #0
 8002fce:	f000 815c 	beq.w	800328a <smoltcp::iface::interface::Interface::poll+0x216e>
 8002fd2:	2001      	movs	r0, #1
 8002fd4:	e9cd 0045 	strd	r0, r0, [sp, #276]	@ 0x114
 8002fd8:	f109 0680 	add.w	r6, r9, #128	@ 0x80
 8002fdc:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8002fde:	9247      	str	r2, [sp, #284]	@ 0x11c
 8002fe0:	469c      	mov	ip, r3
 8002fe2:	ce70      	ldmia	r6, {r4, r5, r6}
 8002fe4:	e9d9 1224 	ldrd	r1, r2, [r9, #144]	@ 0x90
 8002fe8:	4325      	orrs	r5, r4
 8002fea:	e9d0 3004 	ldrd	r3, r0, [r0, #16]
 8002fee:	fab5 f585 	clz	r5, r5
 8002ff2:	1a59      	subs	r1, r3, r1
 8002ff4:	ea4f 1555 	mov.w	r5, r5, lsr #5
 8002ff8:	4190      	sbcs	r0, r2
 8002ffa:	ea06 0605 	and.w	r6, r6, r5
 8002ffe:	f04f 0000 	mov.w	r0, #0
 8003002:	fab8 f188 	clz	r1, r8
 8003006:	bfa8      	it	ge
 8003008:	2001      	movge	r0, #1
 800300a:	4030      	ands	r0, r6
 800300c:	0949      	lsrs	r1, r1, #5
 800300e:	ea00 000b 	and.w	r0, r0, fp
 8003012:	ea10 0201 	ands.w	r2, r0, r1
 8003016:	9234      	str	r2, [sp, #208]	@ 0xd0
 8003018:	d00e      	beq.n	8003038 <smoltcp::iface::interface::Interface::poll+0x1f1c>
 800301a:	f8dd b0f8 	ldr.w	fp, [sp, #248]	@ 0xf8
 800301e:	f649 4490 	movw	r4, #40080	@ 0x9c90
 8003022:	9a43      	ldr	r2, [sp, #268]	@ 0x10c
 8003024:	f04f 0801 	mov.w	r8, #1
 8003028:	9d48      	ldr	r5, [sp, #288]	@ 0x120
 800302a:	f1ab 0b01 	sub.w	fp, fp, #1
 800302e:	f6c0 0400 	movt	r4, #2048	@ 0x800
 8003032:	2000      	movs	r0, #0
 8003034:	2314      	movs	r3, #20
 8003036:	e01c      	b.n	8003072 <smoltcp::iface::interface::Interface::poll+0x1f56>
 8003038:	f8dd b0f8 	ldr.w	fp, [sp, #248]	@ 0xf8
 800303c:	f1b8 0f00 	cmp.w	r8, #0
 8003040:	d008      	beq.n	8003054 <smoltcp::iface::interface::Interface::poll+0x1f38>
 8003042:	f8d9 0100 	ldr.w	r0, [r9, #256]	@ 0x100
 8003046:	f8d9 1108 	ldr.w	r1, [r9, #264]	@ 0x108
 800304a:	1a08      	subs	r0, r1, r0
 800304c:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8003050:	f340 81b2 	ble.w	80033b8 <smoltcp::iface::interface::Interface::poll+0x229c>
 8003054:	f1be 0f00 	cmp.w	lr, #0
 8003058:	d006      	beq.n	8003068 <smoltcp::iface::interface::Interface::poll+0x1f4c>
 800305a:	984a      	ldr	r0, [sp, #296]	@ 0x128
 800305c:	2318      	movs	r3, #24
 800305e:	8980      	ldrh	r0, [r0, #12]
 8003060:	3836      	subs	r0, #54	@ 0x36
 8003062:	9033      	str	r0, [sp, #204]	@ 0xcc
 8003064:	2001      	movs	r0, #1
 8003066:	e001      	b.n	800306c <smoltcp::iface::interface::Interface::poll+0x1f50>
 8003068:	2000      	movs	r0, #0
 800306a:	2314      	movs	r3, #20
 800306c:	9a43      	ldr	r2, [sp, #268]	@ 0x10c
 800306e:	9d48      	ldr	r5, [sp, #288]	@ 0x120
 8003070:	9c40      	ldr	r4, [sp, #256]	@ 0x100
 8003072:	9946      	ldr	r1, [sp, #280]	@ 0x118
 8003074:	4666      	mov	r6, ip
 8003076:	f88d 1351 	strb.w	r1, [sp, #849]	@ 0x351
 800307a:	9942      	ldr	r1, [sp, #264]	@ 0x108
 800307c:	f8ad 134e 	strh.w	r1, [sp, #846]	@ 0x34e
 8003080:	993a      	ldr	r1, [sp, #232]	@ 0xe8
 8003082:	f8ad 134c 	strh.w	r1, [sp, #844]	@ 0x34c
 8003086:	993b      	ldr	r1, [sp, #236]	@ 0xec
 8003088:	f8ad 134a 	strh.w	r1, [sp, #842]	@ 0x34a
 800308c:	9932      	ldr	r1, [sp, #200]	@ 0xc8
 800308e:	f88d 1349 	strb.w	r1, [sp, #841]	@ 0x349
 8003092:	a9cf      	add	r1, sp, #828	@ 0x33c
 8003094:	f8ad 0338 	strh.w	r0, [sp, #824]	@ 0x338
 8003098:	9837      	ldr	r0, [sp, #220]	@ 0xdc
 800309a:	e881 0910 	stmia.w	r1, {r4, r8, fp}
 800309e:	9933      	ldr	r1, [sp, #204]	@ 0xcc
 80030a0:	90cd      	str	r0, [sp, #820]	@ 0x334
 80030a2:	9836      	ldr	r0, [sp, #216]	@ 0xd8
 80030a4:	f8ad 133a 	strh.w	r1, [sp, #826]	@ 0x33a
 80030a8:	90cc      	str	r0, [sp, #816]	@ 0x330
 80030aa:	9839      	ldr	r0, [sp, #228]	@ 0xe4
 80030ac:	993f      	ldr	r1, [sp, #252]	@ 0xfc
 80030ae:	f88d c350 	strb.w	ip, [sp, #848]	@ 0x350
 80030b2:	f8dd c11c 	ldr.w	ip, [sp, #284]	@ 0x11c
 80030b6:	90cb      	str	r0, [sp, #812]	@ 0x32c
 80030b8:	2000      	movs	r0, #0
 80030ba:	91c1      	str	r1, [sp, #772]	@ 0x304
 80030bc:	f1bc 0f00 	cmp.w	ip, #0
 80030c0:	9945      	ldr	r1, [sp, #276]	@ 0x114
 80030c2:	91c0      	str	r1, [sp, #768]	@ 0x300
 80030c4:	9c41      	ldr	r4, [sp, #260]	@ 0x104
 80030c6:	993c      	ldr	r1, [sp, #240]	@ 0xf0
 80030c8:	90c8      	str	r0, [sp, #800]	@ 0x320
 80030ca:	90c5      	str	r0, [sp, #788]	@ 0x314
 80030cc:	90c2      	str	r0, [sp, #776]	@ 0x308
 80030ce:	90bf      	str	r0, [sp, #764]	@ 0x2fc
 80030d0:	984a      	ldr	r0, [sp, #296]	@ 0x128
 80030d2:	f88d c348 	strb.w	ip, [sp, #840]	@ 0x348
 80030d6:	f8cd a360 	str.w	sl, [sp, #864]	@ 0x360
 80030da:	94d6      	str	r4, [sp, #856]	@ 0x358
 80030dc:	91d5      	str	r1, [sp, #852]	@ 0x354
 80030de:	90be      	str	r0, [sp, #760]	@ 0x2f8
 80030e0:	bf18      	it	ne
 80030e2:	f043 0303 	orrne.w	r3, r3, #3
 80030e6:	993d      	ldr	r1, [sp, #244]	@ 0xf4
 80030e8:	eb03 0046 	add.w	r0, r3, r6, lsl #1
 80030ec:	2900      	cmp	r1, #0
 80030ee:	bf18      	it	ne
 80030f0:	300a      	addne	r0, #10
 80030f2:	3003      	adds	r0, #3
 80030f4:	f020 0003 	bic.w	r0, r0, #3
 80030f8:	4440      	add	r0, r8
 80030fa:	90d7      	str	r0, [sp, #860]	@ 0x35c
 80030fc:	68b8      	ldr	r0, [r7, #8]
 80030fe:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 8003102:	4288      	cmp	r0, r1
 8003104:	f4be a869 	bcs.w	80011da <smoltcp::iface::interface::Interface::poll+0xbe>
 8003108:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 800310c:	4348      	muls	r0, r1
 800310e:	6811      	ldr	r1, [r2, #0]
 8003110:	5808      	ldr	r0, [r1, r0]
 8003112:	2800      	cmp	r0, #0
 8003114:	f100 80ec 	bmi.w	80032f0 <smoltcp::iface::interface::Interface::poll+0x21d4>
 8003118:	2000      	movs	r0, #0
 800311a:	9b38      	ldr	r3, [sp, #224]	@ 0xe0
 800311c:	90a5      	str	r0, [sp, #660]	@ 0x294
 800311e:	a9a5      	add	r1, sp, #660	@ 0x294
 8003120:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8003122:	92a7      	str	r2, [sp, #668]	@ 0x29c
 8003124:	2200      	movs	r2, #0
 8003126:	f7fd f919 	bl	800035c <smoltcp::iface::interface::InterfaceInner::dispatch_ip>
 800312a:	b2c0      	uxtb	r0, r0
 800312c:	2802      	cmp	r0, #2
 800312e:	d13d      	bne.n	80031ac <smoltcp::iface::interface::Interface::poll+0x2090>
 8003130:	e9d9 0120 	ldrd	r0, r1, [r9, #128]	@ 0x80
 8003134:	4308      	orrs	r0, r1
 8003136:	d10c      	bne.n	8003152 <smoltcp::iface::interface::Interface::poll+0x2036>
 8003138:	9e4a      	ldr	r6, [sp, #296]	@ 0x128
 800313a:	f109 0340 	add.w	r3, r9, #64	@ 0x40
 800313e:	f109 0c88 	add.w	ip, r9, #136	@ 0x88
 8003142:	e9d6 6504 	ldrd	r6, r5, [r6, #16]
 8003146:	cb0f      	ldmia	r3, {r0, r1, r2, r3}
 8003148:	1992      	adds	r2, r2, r6
 800314a:	416b      	adcs	r3, r5
 800314c:	9d48      	ldr	r5, [sp, #288]	@ 0x120
 800314e:	e88c 000f 	stmia.w	ip, {r0, r1, r2, r3}
 8003152:	2000      	movs	r0, #0
 8003154:	9447      	str	r4, [sp, #284]	@ 0x11c
 8003156:	e9c9 001c 	strd	r0, r0, [r9, #112]	@ 0x70
 800315a:	9834      	ldr	r0, [sp, #208]	@ 0xd0
 800315c:	2800      	cmp	r0, #0
 800315e:	f47f aaaf 	bne.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 8003162:	9846      	ldr	r0, [sp, #280]	@ 0x118
 8003164:	f000 0106 	and.w	r1, r0, #6
 8003168:	4640      	mov	r0, r8
 800316a:	2902      	cmp	r1, #2
 800316c:	bf08      	it	eq
 800316e:	3001      	addeq	r0, #1
 8003170:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8003174:	f340 80b1 	ble.w	80032da <smoltcp::iface::interface::Interface::poll+0x21be>
 8003178:	9a42      	ldr	r2, [sp, #264]	@ 0x108
 800317a:	4458      	add	r0, fp
 800317c:	f8a9 212e 	strh.w	r2, [r9, #302]	@ 0x12e
 8003180:	2902      	cmp	r1, #2
 8003182:	9a3f      	ldr	r2, [sp, #252]	@ 0xfc
 8003184:	f04f 0100 	mov.w	r1, #0
 8003188:	9b45      	ldr	r3, [sp, #276]	@ 0x114
 800318a:	f8c9 0108 	str.w	r0, [r9, #264]	@ 0x108
 800318e:	e9c9 3226 	strd	r3, r2, [r9, #152]	@ 0x98
 8003192:	bf08      	it	eq
 8003194:	f04f 31ff 	moveq.w	r1, #4294967295	@ 0xffffffff
 8003198:	4588      	cmp	r8, r1
 800319a:	d11c      	bne.n	80031d6 <smoltcp::iface::interface::Interface::poll+0x20ba>
 800319c:	984a      	ldr	r0, [sp, #296]	@ 0x128
 800319e:	68c1      	ldr	r1, [r0, #12]
 80031a0:	4648      	mov	r0, r9
 80031a2:	f003 f99f 	bl	80064e4 <smoltcp::socket::tcp::Socket::seq_to_transmit>
 80031a6:	f8dd 8104 	ldr.w	r8, [sp, #260]	@ 0x104
 80031aa:	e060      	b.n	800326e <smoltcp::iface::interface::Interface::poll+0x2152>
 80031ac:	4622      	mov	r2, r4
 80031ae:	984a      	ldr	r0, [sp, #296]	@ 0x128
 80031b0:	9247      	str	r2, [sp, #284]	@ 0x11c
 80031b2:	e9d0 0104 	ldrd	r0, r1, [r0, #16]
 80031b6:	f8c9 2139 	str.w	r2, [r9, #313]	@ 0x139
 80031ba:	2201      	movs	r2, #1
 80031bc:	f889 2138 	strb.w	r2, [r9, #312]	@ 0x138
 80031c0:	f244 2240 	movw	r2, #16960	@ 0x4240
 80031c4:	f2c0 020f 	movt	r2, #15
 80031c8:	1880      	adds	r0, r0, r2
 80031ca:	f141 0100 	adc.w	r1, r1, #0
 80031ce:	e9c9 0150 	strd	r0, r1, [r9, #320]	@ 0x140
 80031d2:	f7ff ba75 	b.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 80031d6:	994a      	ldr	r1, [sp, #296]	@ 0x128
 80031d8:	f8dd 8104 	ldr.w	r8, [sp, #260]	@ 0x104
 80031dc:	e9d1 2104 	ldrd	r2, r1, [r1, #16]
 80031e0:	f8d9 3018 	ldr.w	r3, [r9, #24]
 80031e4:	2b01      	cmp	r3, #1
 80031e6:	d104      	bne.n	80031f2 <smoltcp::iface::interface::Interface::poll+0x20d6>
 80031e8:	f8d9 301c 	ldr.w	r3, [r9, #28]
 80031ec:	1ac3      	subs	r3, r0, r3
 80031ee:	2b01      	cmp	r3, #1
 80031f0:	db0e      	blt.n	8003210 <smoltcp::iface::interface::Interface::poll+0x20f4>
 80031f2:	e9d9 3600 	ldrd	r3, r6, [r9]
 80031f6:	2401      	movs	r4, #1
 80031f8:	e9c9 4006 	strd	r4, r0, [r9, #24]
 80031fc:	4333      	orrs	r3, r6
 80031fe:	d107      	bne.n	8003210 <smoltcp::iface::interface::Interface::poll+0x20f4>
 8003200:	2301      	movs	r3, #1
 8003202:	e9c9 1003 	strd	r1, r0, [r9, #12]
 8003206:	f8c9 3000 	str.w	r3, [r9]
 800320a:	2300      	movs	r3, #0
 800320c:	e9c9 3201 	strd	r3, r2, [r9, #4]
 8003210:	984a      	ldr	r0, [sp, #296]	@ 0x128
 8003212:	68c1      	ldr	r1, [r0, #12]
 8003214:	4648      	mov	r0, r9
 8003216:	f003 f965 	bl	80064e4 <smoltcp::socket::tcp::Socket::seq_to_transmit>
 800321a:	bb40      	cbnz	r0, 800326e <smoltcp::iface::interface::Interface::poll+0x2152>
 800321c:	984a      	ldr	r0, [sp, #296]	@ 0x128
 800321e:	f8d9 2080 	ldr.w	r2, [r9, #128]	@ 0x80
 8003222:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 8003226:	e8df f002 	tbb	[pc, r2]
 800322a:	3502      	.short	0x3502
 800322c:	2202      	.short	0x2202
 800322e:	e9d9 2308 	ldrd	r2, r3, [r9, #32]
 8003232:	009e      	lsls	r6, r3, #2
 8003234:	2e05      	cmp	r6, #5
 8003236:	f04f 0605 	mov.w	r6, #5
 800323a:	bf88      	it	hi
 800323c:	009e      	lslhi	r6, r3, #2
 800323e:	4432      	add	r2, r6
 8003240:	2a0a      	cmp	r2, #10
 8003242:	bf98      	it	ls
 8003244:	220a      	movls	r2, #10
 8003246:	f242 7310 	movw	r3, #10000	@ 0x2710
 800324a:	429a      	cmp	r2, r3
 800324c:	bf28      	it	cs
 800324e:	461a      	movcs	r2, r3
 8003250:	f44f 737a 	mov.w	r3, #1000	@ 0x3e8
 8003254:	435a      	muls	r2, r3
 8003256:	2301      	movs	r3, #1
 8003258:	f8c9 3080 	str.w	r3, [r9, #128]	@ 0x80
 800325c:	2300      	movs	r3, #0
 800325e:	1889      	adds	r1, r1, r2
 8003260:	f140 0000 	adc.w	r0, r0, #0
 8003264:	e9c9 3121 	strd	r3, r1, [r9, #132]	@ 0x84
 8003268:	f109 018c 	add.w	r1, r9, #140	@ 0x8c
 800326c:	c10d      	stmia	r1!, {r0, r2, r3}
 800326e:	f899 0133 	ldrb.w	r0, [r9, #307]	@ 0x133
 8003272:	f8cd 811c 	str.w	r8, [sp, #284]	@ 0x11c
 8003276:	2800      	cmp	r0, #0
 8003278:	f47f aa22 	bne.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 800327c:	2000      	movs	r0, #0
 800327e:	f8a9 011c 	strh.w	r0, [r9, #284]	@ 0x11c
 8003282:	9841      	ldr	r0, [sp, #260]	@ 0x104
 8003284:	9047      	str	r0, [sp, #284]	@ 0x11c
 8003286:	f7ff ba1b 	b.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 800328a:	2001      	movs	r0, #1
 800328c:	f04f 0800 	mov.w	r8, #0
 8003290:	9045      	str	r0, [sp, #276]	@ 0x114
 8003292:	e63f      	b.n	8002f14 <smoltcp::iface::interface::Interface::poll+0x1df8>
 8003294:	e9d9 2322 	ldrd	r2, r3, [r9, #136]	@ 0x88
 8003298:	1a8a      	subs	r2, r1, r2
 800329a:	eb70 0203 	sbcs.w	r2, r0, r3
 800329e:	dbe6      	blt.n	800326e <smoltcp::iface::interface::Interface::poll+0x2152>
 80032a0:	e9d9 2324 	ldrd	r2, r3, [r9, #144]	@ 0x90
 80032a4:	005e      	lsls	r6, r3, #1
 80032a6:	1889      	adds	r1, r1, r2
 80032a8:	4158      	adcs	r0, r3
 80032aa:	2301      	movs	r3, #1
 80032ac:	f8c9 3080 	str.w	r3, [r9, #128]	@ 0x80
 80032b0:	2300      	movs	r3, #0
 80032b2:	e9c9 3121 	strd	r3, r1, [r9, #132]	@ 0x84
 80032b6:	f109 018c 	add.w	r1, r9, #140	@ 0x8c
 80032ba:	ea46 76d2 	orr.w	r6, r6, r2, lsr #31
 80032be:	0052      	lsls	r2, r2, #1
 80032c0:	c145      	stmia	r1!, {r0, r2, r6}
 80032c2:	e7d4      	b.n	800326e <smoltcp::iface::interface::Interface::poll+0x2152>
 80032c4:	f24b 000c 	movw	r0, #45068	@ 0xb00c
 80032c8:	f24b 0238 	movw	r2, #45112	@ 0xb038
 80032cc:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80032d0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80032d4:	212c      	movs	r1, #44	@ 0x2c
 80032d6:	f000 fb25 	bl	8003924 <core::panicking::panic>
 80032da:	f64a 4078 	movw	r0, #44152	@ 0xac78
 80032de:	f64a 42b0 	movw	r2, #44208	@ 0xacb0
 80032e2:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80032e6:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80032ea:	2171      	movs	r1, #113	@ 0x71
 80032ec:	f000 faf8 	bl	80038e0 <core::panicking::panic_fmt>
 80032f0:	f240 0025 	movw	r0, #37	@ 0x25
 80032f4:	f2c0 0000 	movt	r0, #0
 80032f8:	f001 f8bd 	bl	8004476 <defmt::export::acquire_header_and_release>
 80032fc:	f50d 7d5d 	add.w	sp, sp, #884	@ 0x374
 8003300:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003304:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003306:	f24b 00f8 	movw	r0, #45304	@ 0xb0f8
 800330a:	f24b 123c 	movw	r2, #45372	@ 0xb13c
 800330e:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003312:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003316:	2141      	movs	r1, #65	@ 0x41
 8003318:	f000 ff1b 	bl	8004152 <core::option::expect_failed>
 800331c:	f24b 104c 	movw	r0, #45388	@ 0xb14c
 8003320:	f24b 1290 	movw	r2, #45456	@ 0xb190
 8003324:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003328:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800332c:	2142      	movs	r1, #66	@ 0x42
 800332e:	f000 ff10 	bl	8004152 <core::option::expect_failed>
 8003332:	f24b 0368 	movw	r3, #45160	@ 0xb068
 8003336:	2000      	movs	r0, #0
 8003338:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800333c:	f240 52f2 	movw	r2, #1522	@ 0x5f2
 8003340:	f000 fa5d 	bl	80037fe <core::slice::index::slice_index_fail>
 8003344:	f64a 13bc 	movw	r3, #43452	@ 0xa9bc
 8003348:	4640      	mov	r0, r8
 800334a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800334e:	462a      	mov	r2, r5
 8003350:	f000 fa55 	bl	80037fe <core::slice::index::slice_index_fail>
 8003354:	f64a 13dc 	movw	r3, #43484	@ 0xa9dc
 8003358:	2000      	movs	r0, #0
 800335a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800335e:	4632      	mov	r2, r6
 8003360:	f000 fa4d 	bl	80037fe <core::slice::index::slice_index_fail>
 8003364:	f24a 5060 	movw	r0, #42336	@ 0xa560
 8003368:	f24a 5278 	movw	r2, #42360	@ 0xa578
 800336c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003370:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003374:	2116      	movs	r1, #22
 8003376:	f000 feec 	bl	8004152 <core::option::expect_failed>
 800337a:	f64a 13bc 	movw	r3, #43452	@ 0xa9bc
 800337e:	4628      	mov	r0, r5
 8003380:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003384:	4631      	mov	r1, r6
 8003386:	f000 fa3a 	bl	80037fe <core::slice::index::slice_index_fail>
 800338a:	f64a 1088 	movw	r0, #43400	@ 0xa988
 800338e:	f64a 12ac 	movw	r2, #43436	@ 0xa9ac
 8003392:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003396:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800339a:	2122      	movs	r1, #34	@ 0x22
 800339c:	f000 fac2 	bl	8003924 <core::panicking::panic>
 80033a0:	f64a 1058 	movw	r0, #43352	@ 0xa958
 80033a4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80033a8:	f000 fa1f 	bl	80037ea <core::option::unwrap_failed>
 80033ac:	f64a 10cc 	movw	r0, #43468	@ 0xa9cc
 80033b0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80033b4:	f000 fa19 	bl	80037ea <core::option::unwrap_failed>
 80033b8:	f64a 209c 	movw	r0, #43676	@ 0xaa9c
 80033bc:	f64a 22d0 	movw	r2, #43728	@ 0xaad0
 80033c0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80033c4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80033c8:	2167      	movs	r1, #103	@ 0x67
 80033ca:	f000 fa89 	bl	80038e0 <core::panicking::panic_fmt>
 80033ce:	f64a 1368 	movw	r3, #43368	@ 0xa968
 80033d2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80033d6:	f000 fa12 	bl	80037fe <core::slice::index::slice_index_fail>
 80033da:	f24b 0278 	movw	r2, #45176	@ 0xb078
 80033de:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80033e2:	f000 fa8b 	bl	80038fc <core::panicking::panic_bounds_check>
 80033e6:	9a2a      	ldr	r2, [sp, #168]	@ 0xa8
 80033e8:	f64a 632c 	movw	r3, #44588	@ 0xae2c
 80033ec:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80033f0:	2000      	movs	r0, #0
 80033f2:	f000 fa04 	bl	80037fe <core::slice::index::slice_index_fail>
 80033f6:	f64a 60bc 	movw	r0, #44732	@ 0xaebc
 80033fa:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80033fe:	f000 feda 	bl	80041b6 <core::panicking::panic_const::panic_const_rem_by_zero>
 8003402:	f24b 0288 	movw	r2, #45192	@ 0xb088
 8003406:	4619      	mov	r1, r3
 8003408:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800340c:	f000 fa76 	bl	80038fc <core::panicking::panic_bounds_check>
 8003410:	f64a 1278 	movw	r2, #43384	@ 0xa978
 8003414:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003418:	f000 fa70 	bl	80038fc <core::panicking::panic_bounds_check>
 800341c:	f64a 636c 	movw	r3, #44652	@ 0xae6c
 8003420:	2008      	movs	r0, #8
 8003422:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003426:	210e      	movs	r1, #14
 8003428:	f000 f9e9 	bl	80037fe <core::slice::index::slice_index_fail>
 800342c:	f64a 637c 	movw	r3, #44668	@ 0xae7c
 8003430:	200e      	movs	r0, #14
 8003432:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003436:	f000 f9e2 	bl	80037fe <core::slice::index::slice_index_fail>
 800343a:	f64a 638c 	movw	r3, #44684	@ 0xae8c
 800343e:	4608      	mov	r0, r1
 8003440:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003444:	4661      	mov	r1, ip
 8003446:	f000 f9da 	bl	80037fe <core::slice::index::slice_index_fail>
 800344a:	f64a 639c 	movw	r3, #44700	@ 0xae9c
 800344e:	2018      	movs	r0, #24
 8003450:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003454:	211c      	movs	r1, #28
 8003456:	f000 f9d2 	bl	80037fe <core::slice::index::slice_index_fail>
 800345a:	f64a 1278 	movw	r2, #43384	@ 0xa978
 800345e:	4619      	mov	r1, r3
 8003460:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003464:	f000 fa4a 	bl	80038fc <core::panicking::panic_bounds_check>
 8003468:	f64a 53ac 	movw	r3, #44460	@ 0xadac
 800346c:	4620      	mov	r0, r4
 800346e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003472:	f000 f9c4 	bl	80037fe <core::slice::index::slice_index_fail>
 8003476:	f64a 4348 	movw	r3, #44104	@ 0xac48
 800347a:	4658      	mov	r0, fp
 800347c:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003480:	4629      	mov	r1, r5
 8003482:	462a      	mov	r2, r5
 8003484:	f000 f9bb 	bl	80037fe <core::slice::index::slice_index_fail>
 8003488:	f24b 0298 	movw	r2, #45208	@ 0xb098
 800348c:	4620      	mov	r0, r4
 800348e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003492:	f000 fa33 	bl	80038fc <core::panicking::panic_bounds_check>
 8003496:	2002      	movs	r0, #2
 8003498:	908c      	str	r0, [sp, #560]	@ 0x230
 800349a:	a88c      	add	r0, sp, #560	@ 0x230
 800349c:	f002 fde5 	bl	800606a <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>
 80034a0:	f64a 200c 	movw	r0, #43532	@ 0xaa0c
 80034a4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80034a8:	f000 f99f 	bl	80037ea <core::option::unwrap_failed>
 80034ac:	f24b 0258 	movw	r2, #45144	@ 0xb058
 80034b0:	4620      	mov	r0, r4
 80034b2:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80034b6:	f000 fa21 	bl	80038fc <core::panicking::panic_bounds_check>
 80034ba:	f64a 232c 	movw	r3, #43564	@ 0xaa2c
 80034be:	2000      	movs	r0, #0
 80034c0:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80034c4:	2102      	movs	r1, #2
 80034c6:	4622      	mov	r2, r4
 80034c8:	f000 f999 	bl	80037fe <core::slice::index::slice_index_fail>
 80034cc:	f64a 4328 	movw	r3, #44072	@ 0xac28
 80034d0:	2000      	movs	r0, #0
 80034d2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80034d6:	2104      	movs	r1, #4
 80034d8:	4622      	mov	r2, r4
 80034da:	f000 f990 	bl	80037fe <core::slice::index::slice_index_fail>
 80034de:	f64a 4338 	movw	r3, #44088	@ 0xac38
 80034e2:	2004      	movs	r0, #4
 80034e4:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80034e8:	2108      	movs	r1, #8
 80034ea:	4622      	mov	r2, r4
 80034ec:	f000 f987 	bl	80037fe <core::slice::index::slice_index_fail>
 80034f0:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 80034f2:	f64a 3360 	movw	r3, #43872	@ 0xab60
 80034f6:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80034fa:	460a      	mov	r2, r1
 80034fc:	f000 f97f 	bl	80037fe <core::slice::index::slice_index_fail>

08003500 <core::fmt::write>:
 8003500:	b5f0      	push	{r4, r5, r6, r7, lr}
 8003502:	af03      	add	r7, sp, #12
 8003504:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8003508:	b085      	sub	sp, #20
 800350a:	4606      	mov	r6, r0
 800350c:	469b      	mov	fp, r3
 800350e:	468a      	mov	sl, r1
 8003510:	07d8      	lsls	r0, r3, #31
 8003512:	d164      	bne.n	80035de <core::fmt::write+0xde>
 8003514:	7815      	ldrb	r5, [r2, #0]
 8003516:	2000      	movs	r0, #0
 8003518:	2d00      	cmp	r5, #0
 800351a:	d06e      	beq.n	80035fa <core::fmt::write+0xfa>
 800351c:	f8da 900c 	ldr.w	r9, [sl, #12]
 8003520:	f04f 0800 	mov.w	r8, #0
 8003524:	e002      	b.n	800352c <core::fmt::write+0x2c>
 8003526:	7815      	ldrb	r5, [r2, #0]
 8003528:	2d00      	cmp	r5, #0
 800352a:	d06a      	beq.n	8003602 <core::fmt::write+0x102>
 800352c:	1c54      	adds	r4, r2, #1
 800352e:	b268      	sxtb	r0, r5
 8003530:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8003534:	dd07      	ble.n	8003546 <core::fmt::write+0x46>
 8003536:	4630      	mov	r0, r6
 8003538:	4621      	mov	r1, r4
 800353a:	462a      	mov	r2, r5
 800353c:	47c8      	blx	r9
 800353e:	2800      	cmp	r0, #0
 8003540:	d15a      	bne.n	80035f8 <core::fmt::write+0xf8>
 8003542:	1962      	adds	r2, r4, r5
 8003544:	e7ef      	b.n	8003526 <core::fmt::write+0x26>
 8003546:	2d80      	cmp	r5, #128	@ 0x80
 8003548:	d009      	beq.n	800355e <core::fmt::write+0x5e>
 800354a:	2dc0      	cmp	r5, #192	@ 0xc0
 800354c:	d112      	bne.n	8003574 <core::fmt::write+0x74>
 800354e:	f85b 0038 	ldr.w	r0, [fp, r8, lsl #3]
 8003552:	2100      	movs	r1, #0
 8003554:	9104      	str	r1, [sp, #16]
 8003556:	2120      	movs	r1, #32
 8003558:	f2c6 0100 	movt	r1, #24576	@ 0x6000
 800355c:	e031      	b.n	80035c2 <core::fmt::write+0xc2>
 800355e:	f8b2 4001 	ldrh.w	r4, [r2, #1]
 8003562:	1cd5      	adds	r5, r2, #3
 8003564:	4630      	mov	r0, r6
 8003566:	4629      	mov	r1, r5
 8003568:	4622      	mov	r2, r4
 800356a:	47c8      	blx	r9
 800356c:	2800      	cmp	r0, #0
 800356e:	d143      	bne.n	80035f8 <core::fmt::write+0xf8>
 8003570:	192a      	adds	r2, r5, r4
 8003572:	e7d8      	b.n	8003526 <core::fmt::write+0x26>
 8003574:	07e8      	lsls	r0, r5, #31
 8003576:	d103      	bne.n	8003580 <core::fmt::write+0x80>
 8003578:	2120      	movs	r1, #32
 800357a:	f2c6 0100 	movt	r1, #24576	@ 0x6000
 800357e:	e002      	b.n	8003586 <core::fmt::write+0x86>
 8003580:	f8d2 1001 	ldr.w	r1, [r2, #1]
 8003584:	1d54      	adds	r4, r2, #5
 8003586:	07a8      	lsls	r0, r5, #30
 8003588:	bf4c      	ite	mi
 800358a:	f834 2b02 	ldrhmi.w	r2, [r4], #2
 800358e:	2200      	movpl	r2, #0
 8003590:	0768      	lsls	r0, r5, #29
 8003592:	bf4c      	ite	mi
 8003594:	f834 3b02 	ldrhmi.w	r3, [r4], #2
 8003598:	2300      	movpl	r3, #0
 800359a:	0728      	lsls	r0, r5, #28
 800359c:	bf48      	it	mi
 800359e:	f834 8b02 	ldrhmi.w	r8, [r4], #2
 80035a2:	06e8      	lsls	r0, r5, #27
 80035a4:	bf44      	itt	mi
 80035a6:	eb0b 00c2 	addmi.w	r0, fp, r2, lsl #3
 80035aa:	8882      	ldrhmi	r2, [r0, #4]
 80035ac:	06a8      	lsls	r0, r5, #26
 80035ae:	bf44      	itt	mi
 80035b0:	eb0b 00c3 	addmi.w	r0, fp, r3, lsl #3
 80035b4:	8883      	ldrhmi	r3, [r0, #4]
 80035b6:	f85b 0038 	ldr.w	r0, [fp, r8, lsl #3]
 80035ba:	f8ad 3012 	strh.w	r3, [sp, #18]
 80035be:	f8ad 2010 	strh.w	r2, [sp, #16]
 80035c2:	9103      	str	r1, [sp, #12]
 80035c4:	eb0b 01c8 	add.w	r1, fp, r8, lsl #3
 80035c8:	f8cd a008 	str.w	sl, [sp, #8]
 80035cc:	684a      	ldr	r2, [r1, #4]
 80035ce:	a901      	add	r1, sp, #4
 80035d0:	9601      	str	r6, [sp, #4]
 80035d2:	4790      	blx	r2
 80035d4:	b980      	cbnz	r0, 80035f8 <core::fmt::write+0xf8>
 80035d6:	f108 0801 	add.w	r8, r8, #1
 80035da:	4622      	mov	r2, r4
 80035dc:	e7a3      	b.n	8003526 <core::fmt::write+0x26>
 80035de:	ea4f 035b 	mov.w	r3, fp, lsr #1
 80035e2:	4611      	mov	r1, r2
 80035e4:	f8da c00c 	ldr.w	ip, [sl, #12]
 80035e8:	4630      	mov	r0, r6
 80035ea:	461a      	mov	r2, r3
 80035ec:	b005      	add	sp, #20
 80035ee:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80035f2:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 80035f6:	4760      	bx	ip
 80035f8:	2001      	movs	r0, #1
 80035fa:	b005      	add	sp, #20
 80035fc:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003600:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003602:	2000      	movs	r0, #0
 8003604:	b005      	add	sp, #20
 8003606:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800360a:	bdf0      	pop	{r4, r5, r6, r7, pc}

0800360c <core::fmt::Formatter::pad_integral>:
 800360c:	b5f0      	push	{r4, r5, r6, r7, lr}
 800360e:	af03      	add	r7, sp, #12
 8003610:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8003614:	b085      	sub	sp, #20
 8003616:	6886      	ldr	r6, [r0, #8]
 8003618:	460c      	mov	r4, r1
 800361a:	f8b0 800c 	ldrh.w	r8, [r0, #12]
 800361e:	4692      	mov	sl, r2
 8003620:	f416 1100 	ands.w	r1, r6, #2097152	@ 0x200000
 8003624:	f04f 0b2b 	mov.w	fp, #43	@ 0x2b
 8003628:	f3c6 53c0 	ubfx	r3, r6, #23, #1
 800362c:	bf08      	it	eq
 800362e:	f44f 1b88 	moveq.w	fp, #1114112	@ 0x110000
 8003632:	eb02 5951 	add.w	r9, r2, r1, lsr #21
 8003636:	45c1      	cmp	r9, r8
 8003638:	d211      	bcs.n	800365e <core::fmt::Formatter::pad_integral+0x52>
 800363a:	01f1      	lsls	r1, r6, #7
 800363c:	d41d      	bmi.n	800367a <core::fmt::Formatter::pad_integral+0x6e>
 800363e:	f3c6 7141 	ubfx	r1, r6, #29, #2
 8003642:	9304      	str	r3, [sp, #16]
 8003644:	9403      	str	r4, [sp, #12]
 8003646:	eba8 0309 	sub.w	r3, r8, r9
 800364a:	f36f 565f 	bfc	r6, #21, #11
 800364e:	f04f 0800 	mov.w	r8, #0
 8003652:	e8df f001 	tbb	[pc, r1]
 8003656:	025b      	.short	0x025b
 8003658:	0258      	.short	0x0258
 800365a:	4698      	mov	r8, r3
 800365c:	e056      	b.n	800370c <core::fmt::Formatter::pad_integral+0x100>
 800365e:	4626      	mov	r6, r4
 8003660:	e9d0 4500 	ldrd	r4, r5, [r0]
 8003664:	4629      	mov	r1, r5
 8003666:	465a      	mov	r2, fp
 8003668:	4620      	mov	r0, r4
 800366a:	f000 f89d 	bl	80037a8 <core::fmt::Formatter::pad_integral::write_prefix>
 800366e:	b300      	cbz	r0, 80036b2 <core::fmt::Formatter::pad_integral+0xa6>
 8003670:	2001      	movs	r0, #1
 8003672:	b005      	add	sp, #20
 8003674:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003678:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800367a:	e9d0 6102 	ldrd	r6, r1, [r0, #8]
 800367e:	465a      	mov	r2, fp
 8003680:	f8cd a010 	str.w	sl, [sp, #16]
 8003684:	9102      	str	r1, [sp, #8]
 8003686:	2100      	movs	r1, #0
 8003688:	f6c9 71e0 	movt	r1, #40928	@ 0x9fe0
 800368c:	e9d0 a500 	ldrd	sl, r5, [r0]
 8003690:	4031      	ands	r1, r6
 8003692:	9003      	str	r0, [sp, #12]
 8003694:	f041 5100 	orr.w	r1, r1, #536870912	@ 0x20000000
 8003698:	f041 0130 	orr.w	r1, r1, #48	@ 0x30
 800369c:	6081      	str	r1, [r0, #8]
 800369e:	4650      	mov	r0, sl
 80036a0:	4629      	mov	r1, r5
 80036a2:	f000 f881 	bl	80037a8 <core::fmt::Formatter::pad_integral::write_prefix>
 80036a6:	b170      	cbz	r0, 80036c6 <core::fmt::Formatter::pad_integral+0xba>
 80036a8:	2001      	movs	r0, #1
 80036aa:	b005      	add	sp, #20
 80036ac:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80036b0:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80036b2:	68eb      	ldr	r3, [r5, #12]
 80036b4:	4620      	mov	r0, r4
 80036b6:	4631      	mov	r1, r6
 80036b8:	4652      	mov	r2, sl
 80036ba:	b005      	add	sp, #20
 80036bc:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80036c0:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 80036c4:	4718      	bx	r3
 80036c6:	eba8 0009 	sub.w	r0, r8, r9
 80036ca:	9601      	str	r6, [sp, #4]
 80036cc:	46a3      	mov	fp, r4
 80036ce:	2600      	movs	r6, #0
 80036d0:	b284      	uxth	r4, r0
 80036d2:	b2b0      	uxth	r0, r6
 80036d4:	42a0      	cmp	r0, r4
 80036d6:	d20b      	bcs.n	80036f0 <core::fmt::Formatter::pad_integral+0xe4>
 80036d8:	692a      	ldr	r2, [r5, #16]
 80036da:	4650      	mov	r0, sl
 80036dc:	2130      	movs	r1, #48	@ 0x30
 80036de:	4790      	blx	r2
 80036e0:	3601      	adds	r6, #1
 80036e2:	2800      	cmp	r0, #0
 80036e4:	d0f5      	beq.n	80036d2 <core::fmt::Formatter::pad_integral+0xc6>
 80036e6:	2001      	movs	r0, #1
 80036e8:	b005      	add	sp, #20
 80036ea:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80036ee:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80036f0:	9a04      	ldr	r2, [sp, #16]
 80036f2:	4650      	mov	r0, sl
 80036f4:	68eb      	ldr	r3, [r5, #12]
 80036f6:	4659      	mov	r1, fp
 80036f8:	4798      	blx	r3
 80036fa:	b398      	cbz	r0, 8003764 <core::fmt::Formatter::pad_integral+0x158>
 80036fc:	2001      	movs	r0, #1
 80036fe:	b005      	add	sp, #20
 8003700:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003704:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003706:	b299      	uxth	r1, r3
 8003708:	ea4f 0851 	mov.w	r8, r1, lsr #1
 800370c:	e9d0 9500 	ldrd	r9, r5, [r0]
 8003710:	2400      	movs	r4, #0
 8003712:	9302      	str	r3, [sp, #8]
 8003714:	fa1f f088 	uxth.w	r0, r8
 8003718:	b2a1      	uxth	r1, r4
 800371a:	4281      	cmp	r1, r0
 800371c:	d20b      	bcs.n	8003736 <core::fmt::Formatter::pad_integral+0x12a>
 800371e:	692a      	ldr	r2, [r5, #16]
 8003720:	4648      	mov	r0, r9
 8003722:	4631      	mov	r1, r6
 8003724:	4790      	blx	r2
 8003726:	3401      	adds	r4, #1
 8003728:	2800      	cmp	r0, #0
 800372a:	d0f3      	beq.n	8003714 <core::fmt::Formatter::pad_integral+0x108>
 800372c:	2001      	movs	r0, #1
 800372e:	b005      	add	sp, #20
 8003730:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003734:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003736:	9b04      	ldr	r3, [sp, #16]
 8003738:	4648      	mov	r0, r9
 800373a:	4629      	mov	r1, r5
 800373c:	465a      	mov	r2, fp
 800373e:	f000 f833 	bl	80037a8 <core::fmt::Formatter::pad_integral::write_prefix>
 8003742:	b120      	cbz	r0, 800374e <core::fmt::Formatter::pad_integral+0x142>
 8003744:	2001      	movs	r0, #1
 8003746:	b005      	add	sp, #20
 8003748:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800374c:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800374e:	9903      	ldr	r1, [sp, #12]
 8003750:	4648      	mov	r0, r9
 8003752:	68eb      	ldr	r3, [r5, #12]
 8003754:	4652      	mov	r2, sl
 8003756:	4798      	blx	r3
 8003758:	b170      	cbz	r0, 8003778 <core::fmt::Formatter::pad_integral+0x16c>
 800375a:	2001      	movs	r0, #1
 800375c:	b005      	add	sp, #20
 800375e:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003762:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003764:	9803      	ldr	r0, [sp, #12]
 8003766:	e9dd 1201 	ldrd	r1, r2, [sp, #4]
 800376a:	e9c0 1202 	strd	r1, r2, [r0, #8]
 800376e:	2000      	movs	r0, #0
 8003770:	b005      	add	sp, #20
 8003772:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003776:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003778:	9802      	ldr	r0, [sp, #8]
 800377a:	2400      	movs	r4, #0
 800377c:	eba0 0008 	sub.w	r0, r0, r8
 8003780:	fa1f f880 	uxth.w	r8, r0
 8003784:	b2a0      	uxth	r0, r4
 8003786:	4540      	cmp	r0, r8
 8003788:	d209      	bcs.n	800379e <core::fmt::Formatter::pad_integral+0x192>
 800378a:	692a      	ldr	r2, [r5, #16]
 800378c:	4648      	mov	r0, r9
 800378e:	4631      	mov	r1, r6
 8003790:	4790      	blx	r2
 8003792:	3401      	adds	r4, #1
 8003794:	4601      	mov	r1, r0
 8003796:	2001      	movs	r0, #1
 8003798:	2900      	cmp	r1, #0
 800379a:	d0f3      	beq.n	8003784 <core::fmt::Formatter::pad_integral+0x178>
 800379c:	e769      	b.n	8003672 <core::fmt::Formatter::pad_integral+0x66>
 800379e:	2000      	movs	r0, #0
 80037a0:	b005      	add	sp, #20
 80037a2:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80037a6:	bdf0      	pop	{r4, r5, r6, r7, pc}

080037a8 <core::fmt::Formatter::pad_integral::write_prefix>:
 80037a8:	b5f0      	push	{r4, r5, r6, r7, lr}
 80037aa:	af03      	add	r7, sp, #12
 80037ac:	f84d bd04 	str.w	fp, [sp, #-4]!
 80037b0:	461c      	mov	r4, r3
 80037b2:	460d      	mov	r5, r1
 80037b4:	f5b2 1f88 	cmp.w	r2, #1114112	@ 0x110000
 80037b8:	d00a      	beq.n	80037d0 <core::fmt::Formatter::pad_integral::write_prefix+0x28>
 80037ba:	692b      	ldr	r3, [r5, #16]
 80037bc:	4611      	mov	r1, r2
 80037be:	4606      	mov	r6, r0
 80037c0:	4798      	blx	r3
 80037c2:	4601      	mov	r1, r0
 80037c4:	4630      	mov	r0, r6
 80037c6:	b119      	cbz	r1, 80037d0 <core::fmt::Formatter::pad_integral::write_prefix+0x28>
 80037c8:	2001      	movs	r0, #1
 80037ca:	f85d bb04 	ldr.w	fp, [sp], #4
 80037ce:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80037d0:	b13c      	cbz	r4, 80037e2 <core::fmt::Formatter::pad_integral::write_prefix+0x3a>
 80037d2:	68eb      	ldr	r3, [r5, #12]
 80037d4:	4621      	mov	r1, r4
 80037d6:	2200      	movs	r2, #0
 80037d8:	f85d bb04 	ldr.w	fp, [sp], #4
 80037dc:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 80037e0:	4718      	bx	r3
 80037e2:	2000      	movs	r0, #0
 80037e4:	f85d bb04 	ldr.w	fp, [sp], #4
 80037e8:	bdf0      	pop	{r4, r5, r6, r7, pc}

080037ea <core::option::unwrap_failed>:
 80037ea:	b580      	push	{r7, lr}
 80037ec:	466f      	mov	r7, sp
 80037ee:	4602      	mov	r2, r0
 80037f0:	f24a 6050 	movw	r0, #42576	@ 0xa650
 80037f4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80037f8:	212b      	movs	r1, #43	@ 0x2b
 80037fa:	f000 f893 	bl	8003924 <core::panicking::panic>

080037fe <core::slice::index::slice_index_fail>:
 80037fe:	b580      	push	{r7, lr}
 8003800:	466f      	mov	r7, sp
 8003802:	4290      	cmp	r0, r2
 8003804:	d903      	bls.n	800380e <core::slice::index::slice_index_fail+0x10>
 8003806:	4611      	mov	r1, r2
 8003808:	461a      	mov	r2, r3
 800380a:	f000 f811 	bl	8003830 <core::slice::index::slice_index_fail::do_panic::runtime>
 800380e:	4291      	cmp	r1, r2
 8003810:	d904      	bls.n	800381c <core::slice::index::slice_index_fail+0x1e>
 8003812:	4608      	mov	r0, r1
 8003814:	4611      	mov	r1, r2
 8003816:	461a      	mov	r2, r3
 8003818:	f000 f820 	bl	800385c <core::slice::index::slice_index_fail::do_panic::runtime>
 800381c:	4288      	cmp	r0, r1
 800381e:	d902      	bls.n	8003826 <core::slice::index::slice_index_fail+0x28>
 8003820:	461a      	mov	r2, r3
 8003822:	f000 f847 	bl	80038b4 <core::slice::index::slice_index_fail::do_panic::runtime>
 8003826:	4608      	mov	r0, r1
 8003828:	4611      	mov	r1, r2
 800382a:	461a      	mov	r2, r3
 800382c:	f000 f82c 	bl	8003888 <core::slice::index::slice_index_fail::do_panic::runtime>

08003830 <core::slice::index::slice_index_fail::do_panic::runtime>:
 8003830:	b580      	push	{r7, lr}
 8003832:	466f      	mov	r7, sp
 8003834:	b086      	sub	sp, #24
 8003836:	e9cd 0100 	strd	r0, r1, [sp]
 800383a:	f244 0017 	movw	r0, #16407	@ 0x4017
 800383e:	a901      	add	r1, sp, #4
 8003840:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003844:	9005      	str	r0, [sp, #20]
 8003846:	e9cd 0103 	strd	r0, r1, [sp, #12]
 800384a:	4668      	mov	r0, sp
 800384c:	9002      	str	r0, [sp, #8]
 800384e:	f24a 10e3 	movw	r0, #41443	@ 0xa1e3
 8003852:	a902      	add	r1, sp, #8
 8003854:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003858:	f000 f842 	bl	80038e0 <core::panicking::panic_fmt>

0800385c <core::slice::index::slice_index_fail::do_panic::runtime>:
 800385c:	b580      	push	{r7, lr}
 800385e:	466f      	mov	r7, sp
 8003860:	b086      	sub	sp, #24
 8003862:	e9cd 0100 	strd	r0, r1, [sp]
 8003866:	f244 0017 	movw	r0, #16407	@ 0x4017
 800386a:	a901      	add	r1, sp, #4
 800386c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003870:	9005      	str	r0, [sp, #20]
 8003872:	e9cd 0103 	strd	r0, r1, [sp, #12]
 8003876:	4668      	mov	r0, sp
 8003878:	9002      	str	r0, [sp, #8]
 800387a:	f24a 103a 	movw	r0, #41274	@ 0xa13a
 800387e:	a902      	add	r1, sp, #8
 8003880:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003884:	f000 f82c 	bl	80038e0 <core::panicking::panic_fmt>

08003888 <core::slice::index::slice_index_fail::do_panic::runtime>:
 8003888:	b580      	push	{r7, lr}
 800388a:	466f      	mov	r7, sp
 800388c:	b086      	sub	sp, #24
 800388e:	e9cd 0100 	strd	r0, r1, [sp]
 8003892:	f244 0017 	movw	r0, #16407	@ 0x4017
 8003896:	a901      	add	r1, sp, #4
 8003898:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800389c:	9005      	str	r0, [sp, #20]
 800389e:	e9cd 0103 	strd	r0, r1, [sp, #12]
 80038a2:	4668      	mov	r0, sp
 80038a4:	9002      	str	r0, [sp, #8]
 80038a6:	f24a 103a 	movw	r0, #41274	@ 0xa13a
 80038aa:	a902      	add	r1, sp, #8
 80038ac:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80038b0:	f000 f816 	bl	80038e0 <core::panicking::panic_fmt>

080038b4 <core::slice::index::slice_index_fail::do_panic::runtime>:
 80038b4:	b580      	push	{r7, lr}
 80038b6:	466f      	mov	r7, sp
 80038b8:	b086      	sub	sp, #24
 80038ba:	e9cd 0100 	strd	r0, r1, [sp]
 80038be:	f244 0017 	movw	r0, #16407	@ 0x4017
 80038c2:	a901      	add	r1, sp, #4
 80038c4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80038c8:	9005      	str	r0, [sp, #20]
 80038ca:	e9cd 0103 	strd	r0, r1, [sp, #12]
 80038ce:	4668      	mov	r0, sp
 80038d0:	9002      	str	r0, [sp, #8]
 80038d2:	f649 60a0 	movw	r0, #40608	@ 0x9ea0
 80038d6:	a902      	add	r1, sp, #8
 80038d8:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80038dc:	f000 f800 	bl	80038e0 <core::panicking::panic_fmt>

080038e0 <core::panicking::panic_fmt>:
 80038e0:	b580      	push	{r7, lr}
 80038e2:	466f      	mov	r7, sp
 80038e4:	b086      	sub	sp, #24
 80038e6:	e9cd 0101 	strd	r0, r1, [sp, #4]
 80038ea:	2001      	movs	r0, #1
 80038ec:	f8ad 0014 	strh.w	r0, [sp, #20]
 80038f0:	a801      	add	r0, sp, #4
 80038f2:	9003      	str	r0, [sp, #12]
 80038f4:	a803      	add	r0, sp, #12
 80038f6:	9204      	str	r2, [sp, #16]
 80038f8:	f002 fbfa 	bl	80060f0 <__rustc::rust_begin_unwind>

080038fc <core::panicking::panic_bounds_check>:
 80038fc:	b5fe      	push	{r1, r2, r3, r4, r5, r6, r7, lr}
 80038fe:	af06      	add	r7, sp, #24
 8003900:	e9cd 0100 	strd	r0, r1, [sp]
 8003904:	4669      	mov	r1, sp
 8003906:	4805      	ldr	r0, [pc, #20]	@ (800391c <core::panicking::panic_bounds_check+0x20>)
 8003908:	9005      	str	r0, [sp, #20]
 800390a:	e9cd 0103 	strd	r0, r1, [sp, #12]
 800390e:	a801      	add	r0, sp, #4
 8003910:	9002      	str	r0, [sp, #8]
 8003912:	a902      	add	r1, sp, #8
 8003914:	4802      	ldr	r0, [pc, #8]	@ (8003920 <core::panicking::panic_bounds_check+0x24>)
 8003916:	f7ff ffe3 	bl	80038e0 <core::panicking::panic_fmt>
 800391a:	bf00      	nop
 800391c:	08004017 	.word	0x08004017
 8003920:	08009fec 	.word	0x08009fec

08003924 <core::panicking::panic>:
 8003924:	b580      	push	{r7, lr}
 8003926:	466f      	mov	r7, sp
 8003928:	0049      	lsls	r1, r1, #1
 800392a:	3101      	adds	r1, #1
 800392c:	f7ff ffd8 	bl	80038e0 <core::panicking::panic_fmt>

08003930 <<&T as core::fmt::Display>::fmt>:
 8003930:	b580      	push	{r7, lr}
 8003932:	466f      	mov	r7, sp
 8003934:	460b      	mov	r3, r1
 8003936:	e9d0 1200 	ldrd	r1, r2, [r0]
 800393a:	4618      	mov	r0, r3
 800393c:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8003940:	f000 b800 	b.w	8003944 <core::fmt::Formatter::pad>

08003944 <core::fmt::Formatter::pad>:
 8003944:	b5f0      	push	{r4, r5, r6, r7, lr}
 8003946:	af03      	add	r7, sp, #12
 8003948:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 800394c:	b08d      	sub	sp, #52	@ 0x34
 800394e:	f8d0 b008 	ldr.w	fp, [r0, #8]
 8003952:	4690      	mov	r8, r2
 8003954:	460c      	mov	r4, r1
 8003956:	f01b 5fc0 	tst.w	fp, #402653184	@ 0x18000000
 800395a:	f000 8311 	beq.w	8003f80 <core::fmt::Formatter::pad+0x63c>
 800395e:	ea5f 01cb 	movs.w	r1, fp, lsl #3
 8003962:	d43c      	bmi.n	80039de <core::fmt::Formatter::pad+0x9a>
 8003964:	f1b8 0f10 	cmp.w	r8, #16
 8003968:	d25a      	bcs.n	8003a20 <core::fmt::Formatter::pad+0xdc>
 800396a:	f1b8 0f00 	cmp.w	r8, #0
 800396e:	d079      	beq.n	8003a64 <core::fmt::Formatter::pad+0x120>
 8003970:	f008 0c03 	and.w	ip, r8, #3
 8003974:	4681      	mov	r9, r0
 8003976:	ea5f 0198 	movs.w	r1, r8, lsr #2
 800397a:	f04f 0200 	mov.w	r2, #0
 800397e:	f000 82df 	beq.w	8003f40 <core::fmt::Formatter::pad+0x5fc>
 8003982:	f001 0103 	and.w	r1, r1, #3
 8003986:	46a6      	mov	lr, r4
 8003988:	f06f 0603 	mvn.w	r6, #3
 800398c:	2300      	movs	r3, #0
 800398e:	eba2 0a81 	sub.w	sl, r2, r1, lsl #2
 8003992:	1c62      	adds	r2, r4, #1
 8003994:	1991      	adds	r1, r2, r6
 8003996:	3604      	adds	r6, #4
 8003998:	f991 4003 	ldrsb.w	r4, [r1, #3]
 800399c:	f991 5006 	ldrsb.w	r5, [r1, #6]
 80039a0:	f991 0005 	ldrsb.w	r0, [r1, #5]
 80039a4:	f114 0f41 	cmn.w	r4, #65	@ 0x41
 80039a8:	f991 1004 	ldrsb.w	r1, [r1, #4]
 80039ac:	bfc8      	it	gt
 80039ae:	3301      	addgt	r3, #1
 80039b0:	f111 0f41 	cmn.w	r1, #65	@ 0x41
 80039b4:	bfc8      	it	gt
 80039b6:	3301      	addgt	r3, #1
 80039b8:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 80039bc:	bfc8      	it	gt
 80039be:	3301      	addgt	r3, #1
 80039c0:	f115 0f41 	cmn.w	r5, #65	@ 0x41
 80039c4:	eb0a 0006 	add.w	r0, sl, r6
 80039c8:	bfc8      	it	gt
 80039ca:	3301      	addgt	r3, #1
 80039cc:	3004      	adds	r0, #4
 80039ce:	d1e1      	bne.n	8003994 <core::fmt::Formatter::pad+0x50>
 80039d0:	1d32      	adds	r2, r6, #4
 80039d2:	4674      	mov	r4, lr
 80039d4:	f1bc 0f00 	cmp.w	ip, #0
 80039d8:	f040 82b6 	bne.w	8003f48 <core::fmt::Formatter::pad+0x604>
 80039dc:	e2cc      	b.n	8003f78 <core::fmt::Formatter::pad+0x634>
 80039de:	f8b0 c00e 	ldrh.w	ip, [r0, #14]
 80039e2:	f1bc 0f00 	cmp.w	ip, #0
 80039e6:	d02c      	beq.n	8003a42 <core::fmt::Formatter::pad+0xfe>
 80039e8:	eb04 0208 	add.w	r2, r4, r8
 80039ec:	f04f 0800 	mov.w	r8, #0
 80039f0:	4626      	mov	r6, r4
 80039f2:	4661      	mov	r1, ip
 80039f4:	e007      	b.n	8003a06 <core::fmt::Formatter::pad+0xc2>
 80039f6:	2ef0      	cmp	r6, #240	@ 0xf0
 80039f8:	bf2c      	ite	cs
 80039fa:	1d1e      	addcs	r6, r3, #4
 80039fc:	1cde      	addcc	r6, r3, #3
 80039fe:	1af3      	subs	r3, r6, r3
 8003a00:	3901      	subs	r1, #1
 8003a02:	4498      	add	r8, r3
 8003a04:	d01f      	beq.n	8003a46 <core::fmt::Formatter::pad+0x102>
 8003a06:	4296      	cmp	r6, r2
 8003a08:	d01e      	beq.n	8003a48 <core::fmt::Formatter::pad+0x104>
 8003a0a:	4633      	mov	r3, r6
 8003a0c:	f916 5b01 	ldrsb.w	r5, [r6], #1
 8003a10:	f1b5 3fff 	cmp.w	r5, #4294967295	@ 0xffffffff
 8003a14:	dcf3      	bgt.n	80039fe <core::fmt::Formatter::pad+0xba>
 8003a16:	b2ee      	uxtb	r6, r5
 8003a18:	2ee0      	cmp	r6, #224	@ 0xe0
 8003a1a:	d2ec      	bcs.n	80039f6 <core::fmt::Formatter::pad+0xb2>
 8003a1c:	1c9e      	adds	r6, r3, #2
 8003a1e:	e7ee      	b.n	80039fe <core::fmt::Formatter::pad+0xba>
 8003a20:	9000      	str	r0, [sp, #0]
 8003a22:	1ce0      	adds	r0, r4, #3
 8003a24:	4622      	mov	r2, r4
 8003a26:	f020 0403 	bic.w	r4, r0, #3
 8003a2a:	ebb4 0e02 	subs.w	lr, r4, r2
 8003a2e:	f8cd 800c 	str.w	r8, [sp, #12]
 8003a32:	eba8 080e 	sub.w	r8, r8, lr
 8003a36:	9201      	str	r2, [sp, #4]
 8003a38:	f008 0c03 	and.w	ip, r8, #3
 8003a3c:	d10b      	bne.n	8003a56 <core::fmt::Formatter::pad+0x112>
 8003a3e:	2100      	movs	r1, #0
 8003a40:	e057      	b.n	8003af2 <core::fmt::Formatter::pad+0x1ae>
 8003a42:	f04f 0800 	mov.w	r8, #0
 8003a46:	2100      	movs	r1, #0
 8003a48:	ebac 0301 	sub.w	r3, ip, r1
 8003a4c:	8981      	ldrh	r1, [r0, #12]
 8003a4e:	428b      	cmp	r3, r1
 8003a50:	f0c0 8232 	bcc.w	8003eb8 <core::fmt::Formatter::pad+0x574>
 8003a54:	e294      	b.n	8003f80 <core::fmt::Formatter::pad+0x63c>
 8003a56:	1b11      	subs	r1, r2, r4
 8003a58:	f111 0f04 	cmn.w	r1, #4
 8003a5c:	d90a      	bls.n	8003a74 <core::fmt::Formatter::pad+0x130>
 8003a5e:	2200      	movs	r2, #0
 8003a60:	2100      	movs	r1, #0
 8003a62:	e02c      	b.n	8003abe <core::fmt::Formatter::pad+0x17a>
 8003a64:	2300      	movs	r3, #0
 8003a66:	f04f 0800 	mov.w	r8, #0
 8003a6a:	8981      	ldrh	r1, [r0, #12]
 8003a6c:	428b      	cmp	r3, r1
 8003a6e:	f0c0 8223 	bcc.w	8003eb8 <core::fmt::Formatter::pad+0x574>
 8003a72:	e285      	b.n	8003f80 <core::fmt::Formatter::pad+0x63c>
 8003a74:	f102 0901 	add.w	r9, r2, #1
 8003a78:	2100      	movs	r1, #0
 8003a7a:	f06f 0503 	mvn.w	r5, #3
 8003a7e:	eb09 0205 	add.w	r2, r9, r5
 8003a82:	f992 6003 	ldrsb.w	r6, [r2, #3]
 8003a86:	f992 0006 	ldrsb.w	r0, [r2, #6]
 8003a8a:	f992 3005 	ldrsb.w	r3, [r2, #5]
 8003a8e:	f116 0f41 	cmn.w	r6, #65	@ 0x41
 8003a92:	f992 2004 	ldrsb.w	r2, [r2, #4]
 8003a96:	bfc8      	it	gt
 8003a98:	3101      	addgt	r1, #1
 8003a9a:	f112 0f41 	cmn.w	r2, #65	@ 0x41
 8003a9e:	bfc8      	it	gt
 8003aa0:	3101      	addgt	r1, #1
 8003aa2:	f113 0f41 	cmn.w	r3, #65	@ 0x41
 8003aa6:	bfc8      	it	gt
 8003aa8:	3101      	addgt	r1, #1
 8003aaa:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003aae:	f105 0004 	add.w	r0, r5, #4
 8003ab2:	bfc8      	it	gt
 8003ab4:	3101      	addgt	r1, #1
 8003ab6:	f115 0208 	adds.w	r2, r5, #8
 8003aba:	4605      	mov	r5, r0
 8003abc:	d1df      	bne.n	8003a7e <core::fmt::Formatter::pad+0x13a>
 8003abe:	9801      	ldr	r0, [sp, #4]
 8003ac0:	5680      	ldrsb	r0, [r0, r2]
 8003ac2:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003ac6:	bfc8      	it	gt
 8003ac8:	3101      	addgt	r1, #1
 8003aca:	f1be 0f01 	cmp.w	lr, #1
 8003ace:	d010      	beq.n	8003af2 <core::fmt::Formatter::pad+0x1ae>
 8003ad0:	9801      	ldr	r0, [sp, #4]
 8003ad2:	4402      	add	r2, r0
 8003ad4:	f992 0001 	ldrsb.w	r0, [r2, #1]
 8003ad8:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003adc:	bfc8      	it	gt
 8003ade:	3101      	addgt	r1, #1
 8003ae0:	f1be 0f02 	cmp.w	lr, #2
 8003ae4:	d005      	beq.n	8003af2 <core::fmt::Formatter::pad+0x1ae>
 8003ae6:	f992 0002 	ldrsb.w	r0, [r2, #2]
 8003aea:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003aee:	bfc8      	it	gt
 8003af0:	3101      	addgt	r1, #1
 8003af2:	f64f 72fc 	movw	r2, #65532	@ 0xfffc
 8003af6:	ea4f 0e98 	mov.w	lr, r8, lsr #2
 8003afa:	f6c1 72ff 	movt	r2, #8191	@ 0x1fff
 8003afe:	2000      	movs	r0, #0
 8003b00:	f1bc 0f00 	cmp.w	ip, #0
 8003b04:	d01f      	beq.n	8003b46 <core::fmt::Formatter::pad+0x202>
 8003b06:	f102 42c0 	add.w	r2, r2, #1610612736	@ 0x60000000
 8003b0a:	ea02 0208 	and.w	r2, r2, r8
 8003b0e:	4422      	add	r2, r4
 8003b10:	f992 3000 	ldrsb.w	r3, [r2]
 8003b14:	f113 0f41 	cmn.w	r3, #65	@ 0x41
 8003b18:	bfc8      	it	gt
 8003b1a:	2001      	movgt	r0, #1
 8003b1c:	f8dd 800c 	ldr.w	r8, [sp, #12]
 8003b20:	f1bc 0f01 	cmp.w	ip, #1
 8003b24:	d011      	beq.n	8003b4a <core::fmt::Formatter::pad+0x206>
 8003b26:	f992 3001 	ldrsb.w	r3, [r2, #1]
 8003b2a:	f113 0f41 	cmn.w	r3, #65	@ 0x41
 8003b2e:	bfc8      	it	gt
 8003b30:	3001      	addgt	r0, #1
 8003b32:	f1bc 0f02 	cmp.w	ip, #2
 8003b36:	d008      	beq.n	8003b4a <core::fmt::Formatter::pad+0x206>
 8003b38:	f992 2002 	ldrsb.w	r2, [r2, #2]
 8003b3c:	f112 0f41 	cmn.w	r2, #65	@ 0x41
 8003b40:	bfc8      	it	gt
 8003b42:	3001      	addgt	r0, #1
 8003b44:	e001      	b.n	8003b4a <core::fmt::Formatter::pad+0x206>
 8003b46:	f8dd 800c 	ldr.w	r8, [sp, #12]
 8003b4a:	1843      	adds	r3, r0, r1
 8003b4c:	f8cd b008 	str.w	fp, [sp, #8]
 8003b50:	e014      	b.n	8003b7c <core::fmt::Formatter::pad+0x238>
 8003b52:	f04f 0900 	mov.w	r9, #0
 8003b56:	fa3f f189 	uxtb16	r1, r9
 8003b5a:	fa3f f299 	uxtb16	r2, r9, ror #8
 8003b5e:	4411      	add	r1, r2
 8003b60:	9d07      	ldr	r5, [sp, #28]
 8003b62:	9b08      	ldr	r3, [sp, #32]
 8003b64:	eb01 4101 	add.w	r1, r1, r1, lsl #16
 8003b68:	ebae 0e05 	sub.w	lr, lr, r5
 8003b6c:	eb0c 0485 	add.w	r4, ip, r5, lsl #2
 8003b70:	f015 0003 	ands.w	r0, r5, #3
 8003b74:	eb03 4311 	add.w	r3, r3, r1, lsr #16
 8003b78:	f040 816c 	bne.w	8003e54 <core::fmt::Formatter::pad+0x510>
 8003b7c:	f1be 0f00 	cmp.w	lr, #0
 8003b80:	f000 8166 	beq.w	8003e50 <core::fmt::Formatter::pad+0x50c>
 8003b84:	f1be 0fc0 	cmp.w	lr, #192	@ 0xc0
 8003b88:	4671      	mov	r1, lr
 8003b8a:	f44f 707c 	mov.w	r0, #1008	@ 0x3f0
 8003b8e:	9308      	str	r3, [sp, #32]
 8003b90:	bf28      	it	cs
 8003b92:	21c0      	movcs	r1, #192	@ 0xc0
 8003b94:	ea10 0081 	ands.w	r0, r0, r1, lsl #2
 8003b98:	46a4      	mov	ip, r4
 8003b9a:	9107      	str	r1, [sp, #28]
 8003b9c:	d0d9      	beq.n	8003b52 <core::fmt::Formatter::pad+0x20e>
 8003b9e:	eb0c 0400 	add.w	r4, ip, r0
 8003ba2:	0088      	lsls	r0, r1, #2
 8003ba4:	3810      	subs	r0, #16
 8003ba6:	2101      	movs	r1, #1
 8003ba8:	f10c 0a10 	add.w	sl, ip, #16
 8003bac:	f8cd e018 	str.w	lr, [sp, #24]
 8003bb0:	eb01 1110 	add.w	r1, r1, r0, lsr #4
 8003bb4:	2830      	cmp	r0, #48	@ 0x30
 8003bb6:	e9cd 1c04 	strd	r1, ip, [sp, #16]
 8003bba:	d203      	bcs.n	8003bc4 <core::fmt::Formatter::pad+0x280>
 8003bbc:	f04f 0900 	mov.w	r9, #0
 8003bc0:	4663      	mov	r3, ip
 8003bc2:	e0c5      	b.n	8003d50 <core::fmt::Formatter::pad+0x40c>
 8003bc4:	f64f 70fc 	movw	r0, #65532	@ 0xfffc
 8003bc8:	f04f 0900 	mov.w	r9, #0
 8003bcc:	f6c1 70ff 	movt	r0, #8191	@ 0x1fff
 8003bd0:	ea01 0e00 	and.w	lr, r1, r0
 8003bd4:	46d0      	mov	r8, sl
 8003bd6:	4666      	mov	r6, ip
 8003bd8:	9409      	str	r4, [sp, #36]	@ 0x24
 8003bda:	4645      	mov	r5, r8
 8003bdc:	45a0      	cmp	r8, r4
 8003bde:	bf18      	it	ne
 8003be0:	3510      	addne	r5, #16
 8003be2:	462a      	mov	r2, r5
 8003be4:	42a5      	cmp	r5, r4
 8003be6:	bf18      	it	ne
 8003be8:	3210      	addne	r2, #16
 8003bea:	4613      	mov	r3, r2
 8003bec:	42a2      	cmp	r2, r4
 8003bee:	bf18      	it	ne
 8003bf0:	3310      	addne	r3, #16
 8003bf2:	469a      	mov	sl, r3
 8003bf4:	42a3      	cmp	r3, r4
 8003bf6:	bf18      	it	ne
 8003bf8:	f10a 0a10 	addne.w	sl, sl, #16
 8003bfc:	e9d2 4c00 	ldrd	r4, ip, [r2]
 8003c00:	f1be 0e04 	subs.w	lr, lr, #4
 8003c04:	6890      	ldr	r0, [r2, #8]
 8003c06:	900b      	str	r0, [sp, #44]	@ 0x2c
 8003c08:	68d0      	ldr	r0, [r2, #12]
 8003c0a:	ea6f 0204 	mvn.w	r2, r4
 8003c0e:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003c12:	900c      	str	r0, [sp, #48]	@ 0x30
 8003c14:	ea42 1294 	orr.w	r2, r2, r4, lsr #6
 8003c18:	f022 30fe 	bic.w	r0, r2, #4278124286	@ 0xfefefefe
 8003c1c:	e9d6 2400 	ldrd	r2, r4, [r6]
 8003c20:	900a      	str	r0, [sp, #40]	@ 0x28
 8003c22:	e9d6 0602 	ldrd	r0, r6, [r6, #8]
 8003c26:	ea6f 0b02 	mvn.w	fp, r2
 8003c2a:	ea4f 11db 	mov.w	r1, fp, lsr #7
 8003c2e:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003c32:	ea6f 0204 	mvn.w	r2, r4
 8003c36:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003c3a:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003c3e:	ea42 1294 	orr.w	r2, r2, r4, lsr #6
 8003c42:	4449      	add	r1, r9
 8003c44:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003c48:	4411      	add	r1, r2
 8003c4a:	ea6f 0200 	mvn.w	r2, r0
 8003c4e:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003c52:	ea42 1090 	orr.w	r0, r2, r0, lsr #6
 8003c56:	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
 8003c5a:	4408      	add	r0, r1
 8003c5c:	ea6f 0106 	mvn.w	r1, r6
 8003c60:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003c64:	ea41 1196 	orr.w	r1, r1, r6, lsr #6
 8003c68:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003c6c:	eb01 0900 	add.w	r9, r1, r0
 8003c70:	e898 0056 	ldmia.w	r8, {r1, r2, r4, r6}
 8003c74:	46d0      	mov	r8, sl
 8003c76:	ea6f 0001 	mvn.w	r0, r1
 8003c7a:	ea4f 10d0 	mov.w	r0, r0, lsr #7
 8003c7e:	ea40 1091 	orr.w	r0, r0, r1, lsr #6
 8003c82:	ea6f 0102 	mvn.w	r1, r2
 8003c86:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003c8a:	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
 8003c8e:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003c92:	4448      	add	r0, r9
 8003c94:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003c98:	4408      	add	r0, r1
 8003c9a:	ea6f 0104 	mvn.w	r1, r4
 8003c9e:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003ca2:	ea41 1194 	orr.w	r1, r1, r4, lsr #6
 8003ca6:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003caa:	4408      	add	r0, r1
 8003cac:	ea6f 0106 	mvn.w	r1, r6
 8003cb0:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003cb4:	ea41 1196 	orr.w	r1, r1, r6, lsr #6
 8003cb8:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003cbc:	4408      	add	r0, r1
 8003cbe:	cd36      	ldmia	r5, {r1, r2, r4, r5}
 8003cc0:	ea6f 0601 	mvn.w	r6, r1
 8003cc4:	ea4f 16d6 	mov.w	r6, r6, lsr #7
 8003cc8:	ea46 1191 	orr.w	r1, r6, r1, lsr #6
 8003ccc:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003cd0:	461e      	mov	r6, r3
 8003cd2:	4408      	add	r0, r1
 8003cd4:	ea6f 0102 	mvn.w	r1, r2
 8003cd8:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003cdc:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003ce0:	9a0b      	ldr	r2, [sp, #44]	@ 0x2c
 8003ce2:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003ce6:	4408      	add	r0, r1
 8003ce8:	ea6f 0104 	mvn.w	r1, r4
 8003cec:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003cf0:	ea41 1194 	orr.w	r1, r1, r4, lsr #6
 8003cf4:	9c09      	ldr	r4, [sp, #36]	@ 0x24
 8003cf6:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003cfa:	4408      	add	r0, r1
 8003cfc:	ea6f 0105 	mvn.w	r1, r5
 8003d00:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003d04:	ea41 1195 	orr.w	r1, r1, r5, lsr #6
 8003d08:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003d0c:	4408      	add	r0, r1
 8003d0e:	990a      	ldr	r1, [sp, #40]	@ 0x28
 8003d10:	4408      	add	r0, r1
 8003d12:	ea6f 010c 	mvn.w	r1, ip
 8003d16:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003d1a:	ea41 119c 	orr.w	r1, r1, ip, lsr #6
 8003d1e:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003d22:	4408      	add	r0, r1
 8003d24:	ea6f 0102 	mvn.w	r1, r2
 8003d28:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003d2c:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003d30:	9a0c      	ldr	r2, [sp, #48]	@ 0x30
 8003d32:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003d36:	4408      	add	r0, r1
 8003d38:	ea6f 0102 	mvn.w	r1, r2
 8003d3c:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003d40:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003d44:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003d48:	eb01 0900 	add.w	r9, r1, r0
 8003d4c:	f47f af45 	bne.w	8003bda <core::fmt::Formatter::pad+0x296>
 8003d50:	9804      	ldr	r0, [sp, #16]
 8003d52:	e9dd b802 	ldrd	fp, r8, [sp, #8]
 8003d56:	e9dd ce05 	ldrd	ip, lr, [sp, #20]
 8003d5a:	f010 0003 	ands.w	r0, r0, #3
 8003d5e:	f43f aefa 	beq.w	8003b56 <core::fmt::Formatter::pad+0x212>
 8003d62:	e893 0046 	ldmia.w	r3, {r1, r2, r6}
 8003d66:	45a2      	cmp	sl, r4
 8003d68:	68db      	ldr	r3, [r3, #12]
 8003d6a:	ea6f 0501 	mvn.w	r5, r1
 8003d6e:	ea4f 15d5 	mov.w	r5, r5, lsr #7
 8003d72:	ea45 1191 	orr.w	r1, r5, r1, lsr #6
 8003d76:	ea6f 0502 	mvn.w	r5, r2
 8003d7a:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003d7e:	ea4f 15d5 	mov.w	r5, r5, lsr #7
 8003d82:	ea45 1292 	orr.w	r2, r5, r2, lsr #6
 8003d86:	4449      	add	r1, r9
 8003d88:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003d8c:	4411      	add	r1, r2
 8003d8e:	ea6f 0206 	mvn.w	r2, r6
 8003d92:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003d96:	ea42 1296 	orr.w	r2, r2, r6, lsr #6
 8003d9a:	4656      	mov	r6, sl
 8003d9c:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003da0:	bf18      	it	ne
 8003da2:	3610      	addne	r6, #16
 8003da4:	4411      	add	r1, r2
 8003da6:	43da      	mvns	r2, r3
 8003da8:	09d2      	lsrs	r2, r2, #7
 8003daa:	2801      	cmp	r0, #1
 8003dac:	ea42 1293 	orr.w	r2, r2, r3, lsr #6
 8003db0:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003db4:	eb02 0901 	add.w	r9, r2, r1
 8003db8:	f43f aecd 	beq.w	8003b56 <core::fmt::Formatter::pad+0x212>
 8003dbc:	e89a 002e 	ldmia.w	sl, {r1, r2, r3, r5}
 8003dc0:	2802      	cmp	r0, #2
 8003dc2:	ea6f 0401 	mvn.w	r4, r1
 8003dc6:	ea4f 14d4 	mov.w	r4, r4, lsr #7
 8003dca:	ea44 1191 	orr.w	r1, r4, r1, lsr #6
 8003dce:	ea6f 0402 	mvn.w	r4, r2
 8003dd2:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003dd6:	ea4f 14d4 	mov.w	r4, r4, lsr #7
 8003dda:	ea44 1292 	orr.w	r2, r4, r2, lsr #6
 8003dde:	4449      	add	r1, r9
 8003de0:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003de4:	4411      	add	r1, r2
 8003de6:	ea6f 0203 	mvn.w	r2, r3
 8003dea:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003dee:	ea42 1293 	orr.w	r2, r2, r3, lsr #6
 8003df2:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003df6:	4411      	add	r1, r2
 8003df8:	ea6f 0205 	mvn.w	r2, r5
 8003dfc:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003e00:	ea42 1295 	orr.w	r2, r2, r5, lsr #6
 8003e04:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003e08:	eb02 0901 	add.w	r9, r2, r1
 8003e0c:	f43f aea3 	beq.w	8003b56 <core::fmt::Formatter::pad+0x212>
 8003e10:	e896 000f 	ldmia.w	r6, {r0, r1, r2, r3}
 8003e14:	43c6      	mvns	r6, r0
 8003e16:	09f6      	lsrs	r6, r6, #7
 8003e18:	ea46 1090 	orr.w	r0, r6, r0, lsr #6
 8003e1c:	43ce      	mvns	r6, r1
 8003e1e:	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
 8003e22:	09f6      	lsrs	r6, r6, #7
 8003e24:	ea46 1191 	orr.w	r1, r6, r1, lsr #6
 8003e28:	4448      	add	r0, r9
 8003e2a:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003e2e:	4408      	add	r0, r1
 8003e30:	43d1      	mvns	r1, r2
 8003e32:	09c9      	lsrs	r1, r1, #7
 8003e34:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003e38:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003e3c:	4408      	add	r0, r1
 8003e3e:	43d9      	mvns	r1, r3
 8003e40:	09c9      	lsrs	r1, r1, #7
 8003e42:	ea41 1193 	orr.w	r1, r1, r3, lsr #6
 8003e46:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003e4a:	eb01 0900 	add.w	r9, r1, r0
 8003e4e:	e682      	b.n	8003b56 <core::fmt::Formatter::pad+0x212>
 8003e50:	9c01      	ldr	r4, [sp, #4]
 8003e52:	e02d      	b.n	8003eb0 <core::fmt::Formatter::pad+0x56c>
 8003e54:	f005 06fc 	and.w	r6, r5, #252	@ 0xfc
 8003e58:	9c01      	ldr	r4, [sp, #4]
 8003e5a:	2801      	cmp	r0, #1
 8003e5c:	f85c 1026 	ldr.w	r1, [ip, r6, lsl #2]
 8003e60:	ea6f 0201 	mvn.w	r2, r1
 8003e64:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003e68:	ea42 1191 	orr.w	r1, r2, r1, lsr #6
 8003e6c:	f021 32fe 	bic.w	r2, r1, #4278124286	@ 0xfefefefe
 8003e70:	d015      	beq.n	8003e9e <core::fmt::Formatter::pad+0x55a>
 8003e72:	eb0c 0586 	add.w	r5, ip, r6, lsl #2
 8003e76:	2802      	cmp	r0, #2
 8003e78:	6869      	ldr	r1, [r5, #4]
 8003e7a:	ea6f 0601 	mvn.w	r6, r1
 8003e7e:	ea4f 16d6 	mov.w	r6, r6, lsr #7
 8003e82:	ea46 1191 	orr.w	r1, r6, r1, lsr #6
 8003e86:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003e8a:	440a      	add	r2, r1
 8003e8c:	d007      	beq.n	8003e9e <core::fmt::Formatter::pad+0x55a>
 8003e8e:	68a8      	ldr	r0, [r5, #8]
 8003e90:	43c1      	mvns	r1, r0
 8003e92:	09c9      	lsrs	r1, r1, #7
 8003e94:	ea41 1090 	orr.w	r0, r1, r0, lsr #6
 8003e98:	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
 8003e9c:	4402      	add	r2, r0
 8003e9e:	fa3f f082 	uxtb16	r0, r2
 8003ea2:	fa3f f192 	uxtb16	r1, r2, ror #8
 8003ea6:	4408      	add	r0, r1
 8003ea8:	eb00 4000 	add.w	r0, r0, r0, lsl #16
 8003eac:	eb03 4310 	add.w	r3, r3, r0, lsr #16
 8003eb0:	9800      	ldr	r0, [sp, #0]
 8003eb2:	8981      	ldrh	r1, [r0, #12]
 8003eb4:	428b      	cmp	r3, r1
 8003eb6:	d263      	bcs.n	8003f80 <core::fmt::Formatter::pad+0x63c>
 8003eb8:	f3cb 7241 	ubfx	r2, fp, #29, #2
 8003ebc:	f8cd 800c 	str.w	r8, [sp, #12]
 8003ec0:	eba1 0803 	sub.w	r8, r1, r3
 8003ec4:	f36f 5b5f 	bfc	fp, #21, #11
 8003ec8:	f04f 0900 	mov.w	r9, #0
 8003ecc:	46a2      	mov	sl, r4
 8003ece:	e8df f002 	tbb	[pc, r2]
 8003ed2:	0208      	.short	0x0208
 8003ed4:	0804      	.short	0x0804
 8003ed6:	46c1      	mov	r9, r8
 8003ed8:	e003      	b.n	8003ee2 <core::fmt::Formatter::pad+0x59e>
 8003eda:	fa1f f188 	uxth.w	r1, r8
 8003ede:	ea4f 0951 	mov.w	r9, r1, lsr #1
 8003ee2:	e9d0 5600 	ldrd	r5, r6, [r0]
 8003ee6:	2400      	movs	r4, #0
 8003ee8:	fa1f f089 	uxth.w	r0, r9
 8003eec:	b2a1      	uxth	r1, r4
 8003eee:	4281      	cmp	r1, r0
 8003ef0:	d207      	bcs.n	8003f02 <core::fmt::Formatter::pad+0x5be>
 8003ef2:	6932      	ldr	r2, [r6, #16]
 8003ef4:	4628      	mov	r0, r5
 8003ef6:	4659      	mov	r1, fp
 8003ef8:	4790      	blx	r2
 8003efa:	3401      	adds	r4, #1
 8003efc:	2800      	cmp	r0, #0
 8003efe:	d0f3      	beq.n	8003ee8 <core::fmt::Formatter::pad+0x5a4>
 8003f00:	e014      	b.n	8003f2c <core::fmt::Formatter::pad+0x5e8>
 8003f02:	9a03      	ldr	r2, [sp, #12]
 8003f04:	4628      	mov	r0, r5
 8003f06:	68f3      	ldr	r3, [r6, #12]
 8003f08:	4651      	mov	r1, sl
 8003f0a:	4798      	blx	r3
 8003f0c:	b970      	cbnz	r0, 8003f2c <core::fmt::Formatter::pad+0x5e8>
 8003f0e:	eba8 0009 	sub.w	r0, r8, r9
 8003f12:	2400      	movs	r4, #0
 8003f14:	fa1f f880 	uxth.w	r8, r0
 8003f18:	b2a0      	uxth	r0, r4
 8003f1a:	4540      	cmp	r0, r8
 8003f1c:	d20b      	bcs.n	8003f36 <core::fmt::Formatter::pad+0x5f2>
 8003f1e:	6932      	ldr	r2, [r6, #16]
 8003f20:	4628      	mov	r0, r5
 8003f22:	4659      	mov	r1, fp
 8003f24:	4790      	blx	r2
 8003f26:	3401      	adds	r4, #1
 8003f28:	2800      	cmp	r0, #0
 8003f2a:	d0f5      	beq.n	8003f18 <core::fmt::Formatter::pad+0x5d4>
 8003f2c:	2001      	movs	r0, #1
 8003f2e:	b00d      	add	sp, #52	@ 0x34
 8003f30:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003f34:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003f36:	2000      	movs	r0, #0
 8003f38:	b00d      	add	sp, #52	@ 0x34
 8003f3a:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003f3e:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003f40:	2300      	movs	r3, #0
 8003f42:	f1bc 0f00 	cmp.w	ip, #0
 8003f46:	d017      	beq.n	8003f78 <core::fmt::Formatter::pad+0x634>
 8003f48:	56a0      	ldrsb	r0, [r4, r2]
 8003f4a:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003f4e:	bfc8      	it	gt
 8003f50:	3301      	addgt	r3, #1
 8003f52:	f1bc 0f01 	cmp.w	ip, #1
 8003f56:	d00f      	beq.n	8003f78 <core::fmt::Formatter::pad+0x634>
 8003f58:	4422      	add	r2, r4
 8003f5a:	f992 0001 	ldrsb.w	r0, [r2, #1]
 8003f5e:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003f62:	bfc8      	it	gt
 8003f64:	3301      	addgt	r3, #1
 8003f66:	f1bc 0f02 	cmp.w	ip, #2
 8003f6a:	d005      	beq.n	8003f78 <core::fmt::Formatter::pad+0x634>
 8003f6c:	f992 0002 	ldrsb.w	r0, [r2, #2]
 8003f70:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003f74:	bfc8      	it	gt
 8003f76:	3301      	addgt	r3, #1
 8003f78:	4648      	mov	r0, r9
 8003f7a:	8981      	ldrh	r1, [r0, #12]
 8003f7c:	428b      	cmp	r3, r1
 8003f7e:	d39b      	bcc.n	8003eb8 <core::fmt::Formatter::pad+0x574>
 8003f80:	e9d0 0100 	ldrd	r0, r1, [r0]
 8003f84:	4642      	mov	r2, r8
 8003f86:	68cb      	ldr	r3, [r1, #12]
 8003f88:	4621      	mov	r1, r4
 8003f8a:	b00d      	add	sp, #52	@ 0x34
 8003f8c:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003f90:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 8003f94:	4718      	bx	r3

08003f96 <core::panicking::panic_const::panic_const_div_by_zero>:
 8003f96:	b580      	push	{r7, lr}
 8003f98:	466f      	mov	r7, sp
 8003f9a:	4602      	mov	r2, r0
 8003f9c:	f24a 607b 	movw	r0, #42619	@ 0xa67b
 8003fa0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003fa4:	2133      	movs	r1, #51	@ 0x33
 8003fa6:	f7ff fc9b 	bl	80038e0 <core::panicking::panic_fmt>

08003faa <<&T as core::fmt::Debug>::fmt>:
 8003faa:	b580      	push	{r7, lr}
 8003fac:	466f      	mov	r7, sp
 8003fae:	e9d0 0200 	ldrd	r0, r2, [r0]
 8003fb2:	68d2      	ldr	r2, [r2, #12]
 8003fb4:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8003fb8:	4710      	bx	r2

08003fba <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt>:
 8003fba:	b580      	push	{r7, lr}
 8003fbc:	466f      	mov	r7, sp
 8003fbe:	b082      	sub	sp, #8
 8003fc0:	7802      	ldrb	r2, [r0, #0]
 8003fc2:	f24a 5c88 	movw	ip, #42376	@ 0xa588
 8003fc6:	460b      	mov	r3, r1
 8003fc8:	f6c0 0c00 	movt	ip, #2048	@ 0x800
 8003fcc:	2a0a      	cmp	r2, #10
 8003fce:	d30d      	bcc.n	8003fec <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x32>
 8003fd0:	2029      	movs	r0, #41	@ 0x29
 8003fd2:	4350      	muls	r0, r2
 8003fd4:	0b01      	lsrs	r1, r0, #12
 8003fd6:	2064      	movs	r0, #100	@ 0x64
 8003fd8:	fb01 2010 	mls	r0, r1, r0, r2
 8003fdc:	b2c0      	uxtb	r0, r0
 8003fde:	f83c 0010 	ldrh.w	r0, [ip, r0, lsl #1]
 8003fe2:	f827 0c02 	strh.w	r0, [r7, #-2]
 8003fe6:	2001      	movs	r0, #1
 8003fe8:	b91a      	cbnz	r2, 8003ff2 <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x38>
 8003fea:	e003      	b.n	8003ff4 <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x3a>
 8003fec:	2003      	movs	r0, #3
 8003fee:	4611      	mov	r1, r2
 8003ff0:	b102      	cbz	r2, 8003ff4 <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x3a>
 8003ff2:	b139      	cbz	r1, 8004004 <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x4a>
 8003ff4:	f001 017f 	and.w	r1, r1, #127	@ 0x7f
 8003ff8:	3801      	subs	r0, #1
 8003ffa:	1efa      	subs	r2, r7, #3
 8003ffc:	eb0c 0141 	add.w	r1, ip, r1, lsl #1
 8004000:	7849      	ldrb	r1, [r1, #1]
 8004002:	5411      	strb	r1, [r2, r0]
 8004004:	1ef9      	subs	r1, r7, #3
 8004006:	f1c0 0203 	rsb	r2, r0, #3
 800400a:	4401      	add	r1, r0
 800400c:	4618      	mov	r0, r3
 800400e:	f7ff fafd 	bl	800360c <core::fmt::Formatter::pad_integral>
 8004012:	b002      	add	sp, #8
 8004014:	bd80      	pop	{r7, pc}

08004016 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt>:
 8004016:	b5f0      	push	{r4, r5, r6, r7, lr}
 8004018:	af03      	add	r7, sp, #12
 800401a:	f84d bd04 	str.w	fp, [sp, #-4]!
 800401e:	b084      	sub	sp, #16
 8004020:	6803      	ldr	r3, [r0, #0]
 8004022:	f24a 5e88 	movw	lr, #42376	@ 0xa588
 8004026:	468c      	mov	ip, r1
 8004028:	f6c0 0e00 	movt	lr, #2048	@ 0x800
 800402c:	08d8      	lsrs	r0, r3, #3
 800402e:	287c      	cmp	r0, #124	@ 0x7c
 8004030:	d943      	bls.n	80040ba <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xa4>
 8004032:	f241 7059 	movw	r0, #5977	@ 0x1759
 8004036:	f2cd 10b7 	movt	r0, #53687	@ 0xd1b7
 800403a:	fba3 0100 	umull	r0, r1, r3, r0
 800403e:	f242 7010 	movw	r0, #10000	@ 0x2710
 8004042:	0b49      	lsrs	r1, r1, #13
 8004044:	fb01 3510 	mls	r5, r1, r0, r3
 8004048:	b2aa      	uxth	r2, r5
 800404a:	0894      	lsrs	r4, r2, #2
 800404c:	f241 427b 	movw	r2, #5243	@ 0x147b
 8004050:	4354      	muls	r4, r2
 8004052:	0c66      	lsrs	r6, r4, #17
 8004054:	2464      	movs	r4, #100	@ 0x64
 8004056:	fb06 5514 	mls	r5, r6, r4, r5
 800405a:	f83e 6016 	ldrh.w	r6, [lr, r6, lsl #1]
 800405e:	f827 6c14 	strh.w	r6, [r7, #-20]
 8004062:	b2ad      	uxth	r5, r5
 8004064:	f83e 5015 	ldrh.w	r5, [lr, r5, lsl #1]
 8004068:	f827 5c12 	strh.w	r5, [r7, #-18]
 800406c:	f249 657f 	movw	r5, #38527	@ 0x967f
 8004070:	f2c0 0598 	movt	r5, #152	@ 0x98
 8004074:	42ab      	cmp	r3, r5
 8004076:	d936      	bls.n	80040e6 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xd0>
 8004078:	f648 55b9 	movw	r5, #36281	@ 0x8db9
 800407c:	f2c0 0506 	movt	r5, #6
 8004080:	fba1 5605 	umull	r5, r6, r1, r5
 8004084:	fb06 1010 	mls	r0, r6, r0, r1
 8004088:	fb00 f102 	mul.w	r1, r0, r2
 800408c:	f643 3289 	movw	r2, #15241	@ 0x3b89
 8004090:	f2c5 52e6 	movt	r2, #21990	@ 0x55e6
 8004094:	0cc9      	lsrs	r1, r1, #19
 8004096:	fb01 0014 	mls	r0, r1, r4, r0
 800409a:	f83e 1011 	ldrh.w	r1, [lr, r1, lsl #1]
 800409e:	fba3 2402 	umull	r2, r4, r3, r2
 80040a2:	f827 1c18 	strh.w	r1, [r7, #-24]
 80040a6:	2202      	movs	r2, #2
 80040a8:	b280      	uxth	r0, r0
 80040aa:	f83e 0010 	ldrh.w	r0, [lr, r0, lsl #1]
 80040ae:	0e61      	lsrs	r1, r4, #25
 80040b0:	f827 0c16 	strh.w	r0, [r7, #-22]
 80040b4:	2909      	cmp	r1, #9
 80040b6:	d804      	bhi.n	80040c2 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xac>
 80040b8:	e018      	b.n	80040ec <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xd6>
 80040ba:	220a      	movs	r2, #10
 80040bc:	4619      	mov	r1, r3
 80040be:	2909      	cmp	r1, #9
 80040c0:	d914      	bls.n	80040ec <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xd6>
 80040c2:	b288      	uxth	r0, r1
 80040c4:	f241 447b 	movw	r4, #5243	@ 0x147b
 80040c8:	0880      	lsrs	r0, r0, #2
 80040ca:	3a02      	subs	r2, #2
 80040cc:	4360      	muls	r0, r4
 80040ce:	2464      	movs	r4, #100	@ 0x64
 80040d0:	0c40      	lsrs	r0, r0, #17
 80040d2:	fb00 1114 	mls	r1, r0, r4, r1
 80040d6:	f1a7 041a 	sub.w	r4, r7, #26
 80040da:	b289      	uxth	r1, r1
 80040dc:	f83e 1011 	ldrh.w	r1, [lr, r1, lsl #1]
 80040e0:	52a1      	strh	r1, [r4, r2]
 80040e2:	b92b      	cbnz	r3, 80040f0 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xda>
 80040e4:	e005      	b.n	80040f2 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xdc>
 80040e6:	2206      	movs	r2, #6
 80040e8:	2909      	cmp	r1, #9
 80040ea:	d8ea      	bhi.n	80040c2 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xac>
 80040ec:	4608      	mov	r0, r1
 80040ee:	b103      	cbz	r3, 80040f2 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xdc>
 80040f0:	b130      	cbz	r0, 8004100 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xea>
 80040f2:	eb0e 0040 	add.w	r0, lr, r0, lsl #1
 80040f6:	3a01      	subs	r2, #1
 80040f8:	f1a7 011a 	sub.w	r1, r7, #26
 80040fc:	7840      	ldrb	r0, [r0, #1]
 80040fe:	5488      	strb	r0, [r1, r2]
 8004100:	f1a7 001a 	sub.w	r0, r7, #26
 8004104:	1881      	adds	r1, r0, r2
 8004106:	f1c2 020a 	rsb	r2, r2, #10
 800410a:	4660      	mov	r0, ip
 800410c:	f7ff fa7e 	bl	800360c <core::fmt::Formatter::pad_integral>
 8004110:	b004      	add	sp, #16
 8004112:	f85d bb04 	ldr.w	fp, [sp], #4
 8004116:	bdf0      	pop	{r4, r5, r6, r7, pc}

08004118 <core::slice::copy_from_slice_impl::len_mismatch_fail>:
 8004118:	b580      	push	{r7, lr}
 800411a:	466f      	mov	r7, sp
 800411c:	4603      	mov	r3, r0
 800411e:	4608      	mov	r0, r1
 8004120:	4619      	mov	r1, r3
 8004122:	f000 f800 	bl	8004126 <core::slice::copy_from_slice_impl::len_mismatch_fail::do_panic::runtime>

08004126 <core::slice::copy_from_slice_impl::len_mismatch_fail::do_panic::runtime>:
 8004126:	b580      	push	{r7, lr}
 8004128:	466f      	mov	r7, sp
 800412a:	b086      	sub	sp, #24
 800412c:	e9cd 0100 	strd	r0, r1, [sp]
 8004130:	f244 0017 	movw	r0, #16407	@ 0x4017
 8004134:	a901      	add	r1, sp, #4
 8004136:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800413a:	9005      	str	r0, [sp, #20]
 800413c:	e9cd 0103 	strd	r0, r1, [sp, #12]
 8004140:	4668      	mov	r0, sp
 8004142:	9002      	str	r0, [sp, #8]
 8004144:	f649 5022 	movw	r0, #40226	@ 0x9d22
 8004148:	a902      	add	r1, sp, #8
 800414a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800414e:	f7ff fbc7 	bl	80038e0 <core::panicking::panic_fmt>

08004152 <core::option::expect_failed>:
 8004152:	b580      	push	{r7, lr}
 8004154:	466f      	mov	r7, sp
 8004156:	b084      	sub	sp, #16
 8004158:	e9cd 0100 	strd	r0, r1, [sp]
 800415c:	f643 1031 	movw	r0, #14641	@ 0x3931
 8004160:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004164:	a902      	add	r1, sp, #8
 8004166:	9003      	str	r0, [sp, #12]
 8004168:	4668      	mov	r0, sp
 800416a:	9002      	str	r0, [sp, #8]
 800416c:	f649 50d9 	movw	r0, #40409	@ 0x9dd9
 8004170:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004174:	f7ff fbb4 	bl	80038e0 <core::panicking::panic_fmt>

08004178 <core::result::unwrap_failed>:
 8004178:	b580      	push	{r7, lr}
 800417a:	466f      	mov	r7, sp
 800417c:	b088      	sub	sp, #32
 800417e:	469c      	mov	ip, r3
 8004180:	232b      	movs	r3, #43	@ 0x2b
 8004182:	e9cd 0300 	strd	r0, r3, [sp]
 8004186:	f643 70ab 	movw	r0, #16299	@ 0x3fab
 800418a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800418e:	e9cd 1202 	strd	r1, r2, [sp, #8]
 8004192:	9007      	str	r0, [sp, #28]
 8004194:	a802      	add	r0, sp, #8
 8004196:	9006      	str	r0, [sp, #24]
 8004198:	f643 1031 	movw	r0, #14641	@ 0x3931
 800419c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80041a0:	a904      	add	r1, sp, #16
 80041a2:	9005      	str	r0, [sp, #20]
 80041a4:	4668      	mov	r0, sp
 80041a6:	9004      	str	r0, [sp, #16]
 80041a8:	f649 10eb 	movw	r0, #39403	@ 0x99eb
 80041ac:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80041b0:	4662      	mov	r2, ip
 80041b2:	f7ff fb95 	bl	80038e0 <core::panicking::panic_fmt>

080041b6 <core::panicking::panic_const::panic_const_rem_by_zero>:
 80041b6:	b580      	push	{r7, lr}
 80041b8:	466f      	mov	r7, sp
 80041ba:	4602      	mov	r2, r0
 80041bc:	f24a 6094 	movw	r0, #42644	@ 0xa694
 80041c0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80041c4:	2173      	movs	r1, #115	@ 0x73
 80041c6:	f7ff fb8b 	bl	80038e0 <core::panicking::panic_fmt>

080041ca <<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt>:
 80041ca:	b5b0      	push	{r4, r5, r7, lr}
 80041cc:	af02      	add	r7, sp, #8
 80041ce:	b090      	sub	sp, #64	@ 0x40
 80041d0:	460c      	mov	r4, r1
 80041d2:	7ac9      	ldrb	r1, [r1, #11]
 80041d4:	6800      	ldr	r0, [r0, #0]
 80041d6:	f011 0f18 	tst.w	r1, #24
 80041da:	9001      	str	r0, [sp, #4]
 80041dc:	d028      	beq.n	8004230 <<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt+0x66>
 80041de:	2000      	movs	r0, #0
 80041e0:	a901      	add	r1, sp, #4
 80041e2:	9002      	str	r0, [sp, #8]
 80041e4:	f643 70bb 	movw	r0, #16315	@ 0x3fbb
 80041e8:	1cca      	adds	r2, r1, #3
 80041ea:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80041ee:	ad02      	add	r5, sp, #8
 80041f0:	e9cd 1007 	strd	r1, r0, [sp, #28]
 80041f4:	e9cd 020c 	strd	r0, r2, [sp, #48]	@ 0x30
 80041f8:	1c8a      	adds	r2, r1, #2
 80041fa:	ab07      	add	r3, sp, #28
 80041fc:	900e      	str	r0, [sp, #56]	@ 0x38
 80041fe:	e9cd 020a 	strd	r0, r2, [sp, #40]	@ 0x28
 8004202:	1c4a      	adds	r2, r1, #1
 8004204:	f24a 61d4 	movw	r1, #42708	@ 0xa6d4
 8004208:	9209      	str	r2, [sp, #36]	@ 0x24
 800420a:	f24a 1271 	movw	r2, #41329	@ 0xa171
 800420e:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8004212:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8004216:	4628      	mov	r0, r5
 8004218:	f7ff f972 	bl	8003500 <core::fmt::write>
 800421c:	bb60      	cbnz	r0, 8004278 <<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt+0xae>
 800421e:	9a02      	ldr	r2, [sp, #8]
 8004220:	2a10      	cmp	r2, #16
 8004222:	d220      	bcs.n	8004266 <<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt+0x9c>
 8004224:	1d29      	adds	r1, r5, #4
 8004226:	4620      	mov	r0, r4
 8004228:	f7ff fb8c 	bl	8003944 <core::fmt::Formatter::pad>
 800422c:	b010      	add	sp, #64	@ 0x40
 800422e:	bdb0      	pop	{r4, r5, r7, pc}
 8004230:	ab01      	add	r3, sp, #4
 8004232:	f643 72bb 	movw	r2, #16315	@ 0x3fbb
 8004236:	1cdd      	adds	r5, r3, #3
 8004238:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800423c:	e9d4 0100 	ldrd	r0, r1, [r4]
 8004240:	e9cd 250c 	strd	r2, r5, [sp, #48]	@ 0x30
 8004244:	1c9d      	adds	r5, r3, #2
 8004246:	920e      	str	r2, [sp, #56]	@ 0x38
 8004248:	e9cd 250a 	strd	r2, r5, [sp, #40]	@ 0x28
 800424c:	1c5d      	adds	r5, r3, #1
 800424e:	e9cd 3207 	strd	r3, r2, [sp, #28]
 8004252:	f24a 1271 	movw	r2, #41329	@ 0xa171
 8004256:	ab07      	add	r3, sp, #28
 8004258:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800425c:	9509      	str	r5, [sp, #36]	@ 0x24
 800425e:	f7ff f94f 	bl	8003500 <core::fmt::write>
 8004262:	b010      	add	sp, #64	@ 0x40
 8004264:	bdb0      	pop	{r4, r5, r7, pc}
 8004266:	f24a 63ec 	movw	r3, #42732	@ 0xa6ec
 800426a:	4611      	mov	r1, r2
 800426c:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8004270:	2000      	movs	r0, #0
 8004272:	220f      	movs	r2, #15
 8004274:	f7ff fac3 	bl	80037fe <core::slice::index::slice_index_fail>
 8004278:	f64a 1010 	movw	r0, #43280	@ 0xa910
 800427c:	f24a 62fc 	movw	r2, #42748	@ 0xa6fc
 8004280:	f24a 730c 	movw	r3, #42764	@ 0xa70c
 8004284:	f1a7 0109 	sub.w	r1, r7, #9
 8004288:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800428c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8004290:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8004294:	f7ff ff70 	bl	8004178 <core::result::unwrap_failed>

08004298 <<core::fmt::Error as core::fmt::Debug>::fmt>:
 8004298:	b580      	push	{r7, lr}
 800429a:	466f      	mov	r7, sp
 800429c:	e9d1 0200 	ldrd	r0, r2, [r1]
 80042a0:	f24a 61cd 	movw	r1, #42701	@ 0xa6cd
 80042a4:	68d3      	ldr	r3, [r2, #12]
 80042a6:	f6c0 0100 	movt	r1, #2048	@ 0x800
 80042aa:	2205      	movs	r2, #5
 80042ac:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 80042b0:	4718      	bx	r3

080042b2 <<core::net::display_buffer::DisplayBuffer<_> as core::fmt::Write>::write_str>:
 80042b2:	b5f0      	push	{r4, r5, r6, r7, lr}
 80042b4:	af03      	add	r7, sp, #12
 80042b6:	f84d bd04 	str.w	fp, [sp, #-4]!
 80042ba:	6803      	ldr	r3, [r0, #0]
 80042bc:	2400      	movs	r4, #0
 80042be:	189e      	adds	r6, r3, r2
 80042c0:	f144 0500 	adc.w	r5, r4, #0
 80042c4:	2e0f      	cmp	r6, #15
 80042c6:	bf88      	it	hi
 80042c8:	2401      	movhi	r4, #1
 80042ca:	432c      	orrs	r4, r5
 80042cc:	d106      	bne.n	80042dc <<core::net::display_buffer::DisplayBuffer<_> as core::fmt::Write>::write_str+0x2a>
 80042ce:	4403      	add	r3, r0
 80042d0:	4605      	mov	r5, r0
 80042d2:	3304      	adds	r3, #4
 80042d4:	4618      	mov	r0, r3
 80042d6:	f005 f88f 	bl	80093f8 <__aeabi_memcpy>
 80042da:	602e      	str	r6, [r5, #0]
 80042dc:	4620      	mov	r0, r4
 80042de:	f85d bb04 	ldr.w	fp, [sp], #4
 80042e2:	bdf0      	pop	{r4, r5, r6, r7, pc}

080042e4 <core::fmt::Write::write_char>:
 80042e4:	b5f0      	push	{r4, r5, r6, r7, lr}
 80042e6:	af03      	add	r7, sp, #12
 80042e8:	f84d bd04 	str.w	fp, [sp, #-4]!
 80042ec:	b082      	sub	sp, #8
 80042ee:	2200      	movs	r2, #0
 80042f0:	2980      	cmp	r1, #128	@ 0x80
 80042f2:	9201      	str	r2, [sp, #4]
 80042f4:	d203      	bcs.n	80042fe <core::fmt::Write::write_char+0x1a>
 80042f6:	f88d 1004 	strb.w	r1, [sp, #4]
 80042fa:	2201      	movs	r2, #1
 80042fc:	e02f      	b.n	800435e <core::fmt::Write::write_char+0x7a>
 80042fe:	f64f 7cfe 	movw	ip, #65534	@ 0xfffe
 8004302:	460b      	mov	r3, r1
 8004304:	f2c0 3cff 	movt	ip, #1023	@ 0x3ff
 8004308:	098a      	lsrs	r2, r1, #6
 800430a:	f36c 139f 	bfi	r3, ip, #6, #26
 800430e:	f5b1 6f00 	cmp.w	r1, #2048	@ 0x800
 8004312:	d207      	bcs.n	8004324 <core::fmt::Write::write_char+0x40>
 8004314:	f042 01c0 	orr.w	r1, r2, #192	@ 0xc0
 8004318:	f88d 3005 	strb.w	r3, [sp, #5]
 800431c:	f88d 1004 	strb.w	r1, [sp, #4]
 8004320:	2202      	movs	r2, #2
 8004322:	e01c      	b.n	800435e <core::fmt::Write::write_char+0x7a>
 8004324:	f36c 129f 	bfi	r2, ip, #6, #26
 8004328:	0b0c      	lsrs	r4, r1, #12
 800432a:	f5b1 3f80 	cmp.w	r1, #65536	@ 0x10000
 800432e:	d207      	bcs.n	8004340 <core::fmt::Write::write_char+0x5c>
 8004330:	f88d 2005 	strb.w	r2, [sp, #5]
 8004334:	2203      	movs	r2, #3
 8004336:	f044 01e0 	orr.w	r1, r4, #224	@ 0xe0
 800433a:	f88d 3006 	strb.w	r3, [sp, #6]
 800433e:	e00c      	b.n	800435a <core::fmt::Write::write_char+0x76>
 8004340:	f06f 050f 	mvn.w	r5, #15
 8004344:	ea45 4191 	orr.w	r1, r5, r1, lsr #18
 8004348:	f36c 149f 	bfi	r4, ip, #6, #26
 800434c:	f88d 2006 	strb.w	r2, [sp, #6]
 8004350:	f88d 3007 	strb.w	r3, [sp, #7]
 8004354:	2204      	movs	r2, #4
 8004356:	f88d 4005 	strb.w	r4, [sp, #5]
 800435a:	f88d 1004 	strb.w	r1, [sp, #4]
 800435e:	6801      	ldr	r1, [r0, #0]
 8004360:	2300      	movs	r3, #0
 8004362:	188d      	adds	r5, r1, r2
 8004364:	f143 0400 	adc.w	r4, r3, #0
 8004368:	2d0f      	cmp	r5, #15
 800436a:	bf88      	it	hi
 800436c:	2301      	movhi	r3, #1
 800436e:	431c      	orrs	r4, r3
 8004370:	d107      	bne.n	8004382 <core::fmt::Write::write_char+0x9e>
 8004372:	4401      	add	r1, r0
 8004374:	4606      	mov	r6, r0
 8004376:	1d0b      	adds	r3, r1, #4
 8004378:	a901      	add	r1, sp, #4
 800437a:	4618      	mov	r0, r3
 800437c:	f005 f83c 	bl	80093f8 <__aeabi_memcpy>
 8004380:	6035      	str	r5, [r6, #0]
 8004382:	4620      	mov	r0, r4
 8004384:	b002      	add	sp, #8
 8004386:	f85d bb04 	ldr.w	fp, [sp], #4
 800438a:	bdf0      	pop	{r4, r5, r6, r7, pc}

0800438c <core::fmt::Write::write_fmt>:
 800438c:	b580      	push	{r7, lr}
 800438e:	466f      	mov	r7, sp
 8004390:	4613      	mov	r3, r2
 8004392:	460a      	mov	r2, r1
 8004394:	f24a 61d4 	movw	r1, #42708	@ 0xa6d4
 8004398:	f6c0 0100 	movt	r1, #2048	@ 0x800
 800439c:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 80043a0:	f7ff b8ae 	b.w	8003500 <core::fmt::write>

080043a4 <DefaultHandler_>:
 80043a4:	b580      	push	{r7, lr}
 80043a6:	466f      	mov	r7, sp
 80043a8:	e7fe      	b.n	80043a8 <DefaultHandler_+0x4>

080043aa <DefaultPreInit>:
 80043aa:	b580      	push	{r7, lr}
 80043ac:	466f      	mov	r7, sp
 80043ae:	bd80      	pop	{r7, pc}

080043b0 <defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>:
 80043b0:	b5f0      	push	{r4, r5, r6, r7, lr}
 80043b2:	af03      	add	r7, sp, #12
 80043b4:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 80043b8:	b086      	sub	sp, #24
 80043ba:	7805      	ldrb	r5, [r0, #0]
 80043bc:	2102      	movs	r1, #2
 80043be:	7844      	ldrb	r4, [r0, #1]
 80043c0:	f890 9002 	ldrb.w	r9, [r0, #2]
 80043c4:	f890 8003 	ldrb.w	r8, [r0, #3]
 80043c8:	f240 0032 	movw	r0, #50	@ 0x32
 80043cc:	f2c0 0000 	movt	r0, #0
 80043d0:	f827 0c2a 	strh.w	r0, [r7, #-42]
 80043d4:	f1a7 002a 	sub.w	r0, r7, #42	@ 0x2a
 80043d8:	f000 f9ee 	bl	80047b8 <_defmt_write>
 80043dc:	f240 0602 	movw	r6, #2
 80043e0:	a802      	add	r0, sp, #8
 80043e2:	f2c0 0600 	movt	r6, #0
 80043e6:	2102      	movs	r1, #2
 80043e8:	f8ad 6008 	strh.w	r6, [sp, #8]
 80043ec:	f000 f9e4 	bl	80047b8 <_defmt_write>
 80043f0:	f1a7 0025 	sub.w	r0, r7, #37	@ 0x25
 80043f4:	2101      	movs	r1, #1
 80043f6:	f807 5c25 	strb.w	r5, [r7, #-37]
 80043fa:	f000 f9dd 	bl	80047b8 <_defmt_write>
 80043fe:	a803      	add	r0, sp, #12
 8004400:	2102      	movs	r1, #2
 8004402:	f8ad 600c 	strh.w	r6, [sp, #12]
 8004406:	f000 f9d7 	bl	80047b8 <_defmt_write>
 800440a:	f1a7 0021 	sub.w	r0, r7, #33	@ 0x21
 800440e:	2101      	movs	r1, #1
 8004410:	f807 4c21 	strb.w	r4, [r7, #-33]
 8004414:	f000 f9d0 	bl	80047b8 <_defmt_write>
 8004418:	a804      	add	r0, sp, #16
 800441a:	2102      	movs	r1, #2
 800441c:	f8ad 6010 	strh.w	r6, [sp, #16]
 8004420:	f000 f9ca 	bl	80047b8 <_defmt_write>
 8004424:	f1a7 001d 	sub.w	r0, r7, #29
 8004428:	2101      	movs	r1, #1
 800442a:	f807 9c1d 	strb.w	r9, [r7, #-29]
 800442e:	f000 f9c3 	bl	80047b8 <_defmt_write>
 8004432:	a805      	add	r0, sp, #20
 8004434:	2102      	movs	r1, #2
 8004436:	f8ad 6014 	strh.w	r6, [sp, #20]
 800443a:	f000 f9bd 	bl	80047b8 <_defmt_write>
 800443e:	f1a7 0019 	sub.w	r0, r7, #25
 8004442:	2101      	movs	r1, #1
 8004444:	f807 8c19 	strb.w	r8, [r7, #-25]
 8004448:	f000 f9b6 	bl	80047b8 <_defmt_write>
 800444c:	b006      	add	sp, #24
 800444e:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8004452:	bdf0      	pop	{r4, r5, r6, r7, pc}

08004454 <defmt::export::acquire_and_header>:
 8004454:	b5d0      	push	{r4, r6, r7, lr}
 8004456:	af02      	add	r7, sp, #8
 8004458:	b082      	sub	sp, #8
 800445a:	4604      	mov	r4, r0
 800445c:	f000 f8e5 	bl	800462a <_defmt_acquire>
 8004460:	f1a7 000a 	sub.w	r0, r7, #10
 8004464:	2102      	movs	r1, #2
 8004466:	f827 4c0a 	strh.w	r4, [r7, #-10]
 800446a:	f000 f9a5 	bl	80047b8 <_defmt_write>
 800446e:	f7ff ff9c 	bl	80043aa <DefaultPreInit>
 8004472:	b002      	add	sp, #8
 8004474:	bdd0      	pop	{r4, r6, r7, pc}

08004476 <defmt::export::acquire_header_and_release>:
 8004476:	b5d0      	push	{r4, r6, r7, lr}
 8004478:	af02      	add	r7, sp, #8
 800447a:	b082      	sub	sp, #8
 800447c:	4604      	mov	r4, r0
 800447e:	f000 f8d4 	bl	800462a <_defmt_acquire>
 8004482:	f1a7 000a 	sub.w	r0, r7, #10
 8004486:	2102      	movs	r1, #2
 8004488:	f827 4c0a 	strh.w	r4, [r7, #-10]
 800448c:	f000 f994 	bl	80047b8 <_defmt_write>
 8004490:	f7ff ff8b 	bl	80043aa <DefaultPreInit>
 8004494:	f000 f918 	bl	80046c8 <_defmt_release>
 8004498:	b002      	add	sp, #8
 800449a:	bdd0      	pop	{r4, r6, r7, pc}

0800449c <<defmt::export::FmtWrite as core::fmt::Write>::write_str>:
 800449c:	b580      	push	{r7, lr}
 800449e:	466f      	mov	r7, sp
 80044a0:	4608      	mov	r0, r1
 80044a2:	4611      	mov	r1, r2
 80044a4:	f000 f988 	bl	80047b8 <_defmt_write>
 80044a8:	2000      	movs	r0, #0
 80044aa:	bd80      	pop	{r7, pc}

080044ac <core::fmt::Write::write_char>:
 80044ac:	b580      	push	{r7, lr}
 80044ae:	466f      	mov	r7, sp
 80044b0:	b082      	sub	sp, #8
 80044b2:	2000      	movs	r0, #0
 80044b4:	2980      	cmp	r1, #128	@ 0x80
 80044b6:	9001      	str	r0, [sp, #4]
 80044b8:	d203      	bcs.n	80044c2 <core::fmt::Write::write_char+0x16>
 80044ba:	f88d 1004 	strb.w	r1, [sp, #4]
 80044be:	2101      	movs	r1, #1
 80044c0:	e032      	b.n	8004528 <core::fmt::Write::write_char+0x7c>
 80044c2:	f64f 7cfe 	movw	ip, #65534	@ 0xfffe
 80044c6:	460a      	mov	r2, r1
 80044c8:	f2c0 3cff 	movt	ip, #1023	@ 0x3ff
 80044cc:	0988      	lsrs	r0, r1, #6
 80044ce:	f36c 129f 	bfi	r2, ip, #6, #26
 80044d2:	f5b1 6f00 	cmp.w	r1, #2048	@ 0x800
 80044d6:	d207      	bcs.n	80044e8 <core::fmt::Write::write_char+0x3c>
 80044d8:	f040 00c0 	orr.w	r0, r0, #192	@ 0xc0
 80044dc:	f88d 2005 	strb.w	r2, [sp, #5]
 80044e0:	f88d 0004 	strb.w	r0, [sp, #4]
 80044e4:	2102      	movs	r1, #2
 80044e6:	e01f      	b.n	8004528 <core::fmt::Write::write_char+0x7c>
 80044e8:	f36c 109f 	bfi	r0, ip, #6, #26
 80044ec:	ea4f 3e11 	mov.w	lr, r1, lsr #12
 80044f0:	f5b1 3f80 	cmp.w	r1, #65536	@ 0x10000
 80044f4:	d209      	bcs.n	800450a <core::fmt::Write::write_char+0x5e>
 80044f6:	2103      	movs	r1, #3
 80044f8:	f88d 0005 	strb.w	r0, [sp, #5]
 80044fc:	f04e 00e0 	orr.w	r0, lr, #224	@ 0xe0
 8004500:	f88d 2006 	strb.w	r2, [sp, #6]
 8004504:	f88d 0004 	strb.w	r0, [sp, #4]
 8004508:	e00e      	b.n	8004528 <core::fmt::Write::write_char+0x7c>
 800450a:	f06f 030f 	mvn.w	r3, #15
 800450e:	ea43 4191 	orr.w	r1, r3, r1, lsr #18
 8004512:	f36c 1e9f 	bfi	lr, ip, #6, #26
 8004516:	f88d 1004 	strb.w	r1, [sp, #4]
 800451a:	f88d 2007 	strb.w	r2, [sp, #7]
 800451e:	2104      	movs	r1, #4
 8004520:	f88d 0006 	strb.w	r0, [sp, #6]
 8004524:	f88d e005 	strb.w	lr, [sp, #5]
 8004528:	a801      	add	r0, sp, #4
 800452a:	f000 f945 	bl	80047b8 <_defmt_write>
 800452e:	2000      	movs	r0, #0
 8004530:	b002      	add	sp, #8
 8004532:	bd80      	pop	{r7, pc}

08004534 <core::fmt::Write::write_fmt>:
 8004534:	b580      	push	{r7, lr}
 8004536:	466f      	mov	r7, sp
 8004538:	4613      	mov	r3, r2
 800453a:	460a      	mov	r2, r1
 800453c:	f24a 712c 	movw	r1, #42796	@ 0xa72c
 8004540:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8004544:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8004548:	f7fe bfda 	b.w	8003500 <core::fmt::write>

0800454c <defmt_rtt::channel::Channel::blocking_write>:
 800454c:	b5f0      	push	{r4, r5, r6, r7, lr}
 800454e:	af03      	add	r7, sp, #12
 8004550:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 8004554:	b152      	cbz	r2, 800456c <defmt_rtt::channel::Channel::blocking_write+0x20>
 8004556:	4605      	mov	r5, r0
 8004558:	4614      	mov	r4, r2
 800455a:	6900      	ldr	r0, [r0, #16]
 800455c:	68ea      	ldr	r2, [r5, #12]
 800455e:	f3bf 8f5f 	dmb	sy
 8004562:	4290      	cmp	r0, r2
 8004564:	d907      	bls.n	8004576 <defmt_rtt::channel::Channel::blocking_write+0x2a>
 8004566:	43d3      	mvns	r3, r2
 8004568:	4418      	add	r0, r3
 800456a:	b968      	cbnz	r0, 8004588 <defmt_rtt::channel::Channel::blocking_write+0x3c>
 800456c:	2400      	movs	r4, #0
 800456e:	4620      	mov	r0, r4
 8004570:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8004574:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8004576:	2800      	cmp	r0, #0
 8004578:	bf06      	itte	eq
 800457a:	f240 30ff 	movweq	r0, #1023	@ 0x3ff
 800457e:	1a80      	subeq	r0, r0, r2
 8004580:	f5c2 6080 	rsbne	r0, r2, #1024	@ 0x400
 8004584:	2800      	cmp	r0, #0
 8004586:	d0f1      	beq.n	800456c <defmt_rtt::channel::Channel::blocking_write+0x20>
 8004588:	42a0      	cmp	r0, r4
 800458a:	bf38      	it	cc
 800458c:	4604      	movcc	r4, r0
 800458e:	18a6      	adds	r6, r4, r2
 8004590:	f5b6 6f80 	cmp.w	r6, #1024	@ 0x400
 8004594:	d90d      	bls.n	80045b2 <defmt_rtt::channel::Channel::blocking_write+0x66>
 8004596:	6868      	ldr	r0, [r5, #4]
 8004598:	f5c2 6980 	rsb	r9, r2, #1024	@ 0x400
 800459c:	4688      	mov	r8, r1
 800459e:	4410      	add	r0, r2
 80045a0:	464a      	mov	r2, r9
 80045a2:	f004 ff29 	bl	80093f8 <__aeabi_memcpy>
 80045a6:	6868      	ldr	r0, [r5, #4]
 80045a8:	eb08 0109 	add.w	r1, r8, r9
 80045ac:	eba4 0209 	sub.w	r2, r4, r9
 80045b0:	e002      	b.n	80045b8 <defmt_rtt::channel::Channel::blocking_write+0x6c>
 80045b2:	6868      	ldr	r0, [r5, #4]
 80045b4:	4410      	add	r0, r2
 80045b6:	4622      	mov	r2, r4
 80045b8:	f004 ff1e 	bl	80093f8 <__aeabi_memcpy>
 80045bc:	f36f 269f 	bfc	r6, #10, #22
 80045c0:	f3bf 8f5f 	dmb	sy
 80045c4:	60ee      	str	r6, [r5, #12]
 80045c6:	4620      	mov	r0, r4
 80045c8:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80045cc:	bdf0      	pop	{r4, r5, r6, r7, pc}

080045ce <defmt_rtt::channel::Channel::nonblocking_write>:
 80045ce:	b5f0      	push	{r4, r5, r6, r7, lr}
 80045d0:	af03      	add	r7, sp, #12
 80045d2:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 80045d6:	4614      	mov	r4, r2
 80045d8:	68c2      	ldr	r2, [r0, #12]
 80045da:	f5b4 6f80 	cmp.w	r4, #1024	@ 0x400
 80045de:	bf28      	it	cs
 80045e0:	f44f 6480 	movcs.w	r4, #1024	@ 0x400
 80045e4:	1916      	adds	r6, r2, r4
 80045e6:	4605      	mov	r5, r0
 80045e8:	f5b6 6f80 	cmp.w	r6, #1024	@ 0x400
 80045ec:	f3bf 8f5f 	dmb	sy
 80045f0:	d90d      	bls.n	800460e <defmt_rtt::channel::Channel::nonblocking_write+0x40>
 80045f2:	6868      	ldr	r0, [r5, #4]
 80045f4:	f5c2 6980 	rsb	r9, r2, #1024	@ 0x400
 80045f8:	4688      	mov	r8, r1
 80045fa:	4410      	add	r0, r2
 80045fc:	464a      	mov	r2, r9
 80045fe:	f004 fefb 	bl	80093f8 <__aeabi_memcpy>
 8004602:	6868      	ldr	r0, [r5, #4]
 8004604:	eb08 0109 	add.w	r1, r8, r9
 8004608:	eba4 0209 	sub.w	r2, r4, r9
 800460c:	e002      	b.n	8004614 <defmt_rtt::channel::Channel::nonblocking_write+0x46>
 800460e:	6868      	ldr	r0, [r5, #4]
 8004610:	4410      	add	r0, r2
 8004612:	4622      	mov	r2, r4
 8004614:	f004 fef0 	bl	80093f8 <__aeabi_memcpy>
 8004618:	f3bf 8f5f 	dmb	sy
 800461c:	f36f 269f 	bfc	r6, #10, #22
 8004620:	60ee      	str	r6, [r5, #12]
 8004622:	4620      	mov	r0, r4
 8004624:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8004628:	bdf0      	pop	{r4, r5, r6, r7, pc}

0800462a <_defmt_acquire>:
 800462a:	b5f0      	push	{r4, r5, r6, r7, lr}
 800462c:	af03      	add	r7, sp, #12
 800462e:	f84d bd04 	str.w	fp, [sp, #-4]!
 8004632:	b082      	sub	sp, #8
 8004634:	f004 f95a 	bl	80088ec <__primask_r>
 8004638:	4604      	mov	r4, r0
 800463a:	f004 f94a 	bl	80088d2 <__cpsid>
 800463e:	f240 0040 	movw	r0, #64	@ 0x40
 8004642:	f2c2 0000 	movt	r0, #8192	@ 0x2000
 8004646:	7841      	ldrb	r1, [r0, #1]
 8004648:	bb59      	cbnz	r1, 80046a2 <_defmt_acquire+0x78>
 800464a:	2101      	movs	r1, #1
 800464c:	ea21 0304 	bic.w	r3, r1, r4
 8004650:	7041      	strb	r1, [r0, #1]
 8004652:	7882      	ldrb	r2, [r0, #2]
 8004654:	7003      	strb	r3, [r0, #0]
 8004656:	bb02      	cbnz	r2, 800469a <_defmt_acquire+0x70>
 8004658:	7081      	strb	r1, [r0, #2]
 800465a:	f240 0508 	movw	r5, #8
 800465e:	2000      	movs	r0, #0
 8004660:	f2c2 0500 	movt	r5, #8192	@ 0x2000
 8004664:	f807 0c11 	strb.w	r0, [r7, #-17]
 8004668:	f244 56cf 	movw	r6, #17871	@ 0x45cf
 800466c:	6ae8      	ldr	r0, [r5, #44]	@ 0x2c
 800466e:	f1a7 0411 	sub.w	r4, r7, #17
 8004672:	f244 514d 	movw	r1, #17741	@ 0x454d
 8004676:	f6c0 0600 	movt	r6, #2048	@ 0x800
 800467a:	f000 0003 	and.w	r0, r0, #3
 800467e:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8004682:	2802      	cmp	r0, #2
 8004684:	bf08      	it	eq
 8004686:	460e      	moveq	r6, r1
 8004688:	f105 0018 	add.w	r0, r5, #24
 800468c:	4621      	mov	r1, r4
 800468e:	2201      	movs	r2, #1
 8004690:	47b0      	blx	r6
 8004692:	2800      	cmp	r0, #0
 8004694:	d0f8      	beq.n	8004688 <_defmt_acquire+0x5e>
 8004696:	2801      	cmp	r0, #1
 8004698:	d10e      	bne.n	80046b8 <_defmt_acquire+0x8e>
 800469a:	b002      	add	sp, #8
 800469c:	f85d bb04 	ldr.w	fp, [sp], #4
 80046a0:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80046a2:	f24a 7045 	movw	r0, #42821	@ 0xa745
 80046a6:	f24a 7264 	movw	r2, #42852	@ 0xa764
 80046aa:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80046ae:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80046b2:	213d      	movs	r1, #61	@ 0x3d
 80046b4:	f7ff f914 	bl	80038e0 <core::panicking::panic_fmt>
 80046b8:	f24a 7374 	movw	r3, #42868	@ 0xa774
 80046bc:	2101      	movs	r1, #1
 80046be:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80046c2:	2201      	movs	r2, #1
 80046c4:	f7ff f89b 	bl	80037fe <core::slice::index::slice_index_fail>

080046c8 <_defmt_release>:
 80046c8:	b5f0      	push	{r4, r5, r6, r7, lr}
 80046ca:	af03      	add	r7, sp, #12
 80046cc:	e92d 0700 	stmdb	sp!, {r8, r9, sl}
 80046d0:	b082      	sub	sp, #8
 80046d2:	f240 0840 	movw	r8, #64	@ 0x40
 80046d6:	f240 0608 	movw	r6, #8
 80046da:	f2c2 0800 	movt	r8, #8192	@ 0x2000
 80046de:	f244 5a4d 	movw	sl, #17741	@ 0x454d
 80046e2:	f898 0003 	ldrb.w	r0, [r8, #3]
 80046e6:	f244 55cf 	movw	r5, #17871	@ 0x45cf
 80046ea:	f2c2 0600 	movt	r6, #8192	@ 0x2000
 80046ee:	f6c0 0a00 	movt	sl, #2048	@ 0x800
 80046f2:	f6c0 0500 	movt	r5, #2048	@ 0x800
 80046f6:	b3a0      	cbz	r0, 8004762 <_defmt_release+0x9a>
 80046f8:	2807      	cmp	r0, #7
 80046fa:	d21b      	bcs.n	8004734 <_defmt_release+0x6c>
 80046fc:	f04f 32ff 	mov.w	r2, #4294967295	@ 0xffffffff
 8004700:	f898 1004 	ldrb.w	r1, [r8, #4]
 8004704:	fa02 f000 	lsl.w	r0, r2, r0
 8004708:	f1a7 091a 	sub.w	r9, r7, #26
 800470c:	4308      	orrs	r0, r1
 800470e:	462c      	mov	r4, r5
 8004710:	f000 007f 	and.w	r0, r0, #127	@ 0x7f
 8004714:	f807 0c1a 	strb.w	r0, [r7, #-26]
 8004718:	6af0      	ldr	r0, [r6, #44]	@ 0x2c
 800471a:	f000 0003 	and.w	r0, r0, #3
 800471e:	2802      	cmp	r0, #2
 8004720:	bf08      	it	eq
 8004722:	4654      	moveq	r4, sl
 8004724:	f106 0018 	add.w	r0, r6, #24
 8004728:	4649      	mov	r1, r9
 800472a:	2201      	movs	r2, #1
 800472c:	47a0      	blx	r4
 800472e:	2800      	cmp	r0, #0
 8004730:	d0f8      	beq.n	8004724 <_defmt_release+0x5c>
 8004732:	e014      	b.n	800475e <_defmt_release+0x96>
 8004734:	3079      	adds	r0, #121	@ 0x79
 8004736:	f1a7 091b 	sub.w	r9, r7, #27
 800473a:	f040 0080 	orr.w	r0, r0, #128	@ 0x80
 800473e:	f807 0c1b 	strb.w	r0, [r7, #-27]
 8004742:	6af0      	ldr	r0, [r6, #44]	@ 0x2c
 8004744:	462c      	mov	r4, r5
 8004746:	f000 0003 	and.w	r0, r0, #3
 800474a:	2802      	cmp	r0, #2
 800474c:	bf08      	it	eq
 800474e:	4654      	moveq	r4, sl
 8004750:	f106 0018 	add.w	r0, r6, #24
 8004754:	4649      	mov	r1, r9
 8004756:	2201      	movs	r2, #1
 8004758:	47a0      	blx	r4
 800475a:	2800      	cmp	r0, #0
 800475c:	d0f8      	beq.n	8004750 <_defmt_release+0x88>
 800475e:	2801      	cmp	r0, #1
 8004760:	d122      	bne.n	80047a8 <_defmt_release+0xe0>
 8004762:	2000      	movs	r0, #0
 8004764:	f1a7 0419 	sub.w	r4, r7, #25
 8004768:	f807 0c19 	strb.w	r0, [r7, #-25]
 800476c:	6af0      	ldr	r0, [r6, #44]	@ 0x2c
 800476e:	f000 0003 	and.w	r0, r0, #3
 8004772:	2802      	cmp	r0, #2
 8004774:	bf08      	it	eq
 8004776:	4655      	moveq	r5, sl
 8004778:	f106 0018 	add.w	r0, r6, #24
 800477c:	4621      	mov	r1, r4
 800477e:	2201      	movs	r2, #1
 8004780:	47a8      	blx	r5
 8004782:	2800      	cmp	r0, #0
 8004784:	d0f8      	beq.n	8004778 <_defmt_release+0xb0>
 8004786:	2801      	cmp	r0, #1
 8004788:	d10e      	bne.n	80047a8 <_defmt_release+0xe0>
 800478a:	2000      	movs	r0, #0
 800478c:	f8a8 0003 	strh.w	r0, [r8, #3]
 8004790:	f888 0001 	strb.w	r0, [r8, #1]
 8004794:	f898 0000 	ldrb.w	r0, [r8]
 8004798:	07c0      	lsls	r0, r0, #31
 800479a:	bf18      	it	ne
 800479c:	f004 f89b 	blne	80088d6 <__cpsie>
 80047a0:	b002      	add	sp, #8
 80047a2:	e8bd 0700 	ldmia.w	sp!, {r8, r9, sl}
 80047a6:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80047a8:	f24a 7374 	movw	r3, #42868	@ 0xa774
 80047ac:	2101      	movs	r1, #1
 80047ae:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80047b2:	2201      	movs	r2, #1
 80047b4:	f7ff f823 	bl	80037fe <core::slice::index::slice_index_fail>

080047b8 <_defmt_write>:
 80047b8:	b5f0      	push	{r4, r5, r6, r7, lr}
 80047ba:	af03      	add	r7, sp, #12
 80047bc:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 80047c0:	b083      	sub	sp, #12
 80047c2:	2900      	cmp	r1, #0
 80047c4:	f000 80ba 	beq.w	800493c <_defmt_write+0x184>
 80047c8:	f240 0840 	movw	r8, #64	@ 0x40
 80047cc:	4604      	mov	r4, r0
 80047ce:	f2c2 0800 	movt	r8, #8192	@ 0x2000
 80047d2:	eb00 0a01 	add.w	sl, r0, r1
 80047d6:	f898 0003 	ldrb.w	r0, [r8, #3]
 80047da:	f240 0908 	movw	r9, #8
 80047de:	f244 5bcf 	movw	fp, #17871	@ 0x45cf
 80047e2:	f2c2 0900 	movt	r9, #8192	@ 0x2000
 80047e6:	f6c0 0b00 	movt	fp, #2048	@ 0x800
 80047ea:	e008      	b.n	80047fe <_defmt_write+0x46>
 80047ec:	2801      	cmp	r0, #1
 80047ee:	f040 80a9 	bne.w	8004944 <_defmt_write+0x18c>
 80047f2:	2000      	movs	r0, #0
 80047f4:	f8a8 0003 	strh.w	r0, [r8, #3]
 80047f8:	4554      	cmp	r4, sl
 80047fa:	f000 809f 	beq.w	800493c <_defmt_write+0x184>
 80047fe:	f814 1b01 	ldrb.w	r1, [r4], #1
 8004802:	b2c2      	uxtb	r2, r0
 8004804:	2a07      	cmp	r2, #7
 8004806:	d21f      	bcs.n	8004848 <_defmt_write+0x90>
 8004808:	2900      	cmp	r1, #0
 800480a:	d057      	beq.n	80048bc <_defmt_write+0x104>
 800480c:	f807 1c21 	strb.w	r1, [r7, #-33]
 8004810:	465d      	mov	r5, fp
 8004812:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 8004816:	f000 0003 	and.w	r0, r0, #3
 800481a:	2802      	cmp	r0, #2
 800481c:	f244 504d 	movw	r0, #17741	@ 0x454d
 8004820:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004824:	bf08      	it	eq
 8004826:	4605      	moveq	r5, r0
 8004828:	f109 0018 	add.w	r0, r9, #24
 800482c:	f1a7 0121 	sub.w	r1, r7, #33	@ 0x21
 8004830:	2201      	movs	r2, #1
 8004832:	47a8      	blx	r5
 8004834:	2800      	cmp	r0, #0
 8004836:	d0f7      	beq.n	8004828 <_defmt_write+0x70>
 8004838:	2801      	cmp	r0, #1
 800483a:	f040 8083 	bne.w	8004944 <_defmt_write+0x18c>
 800483e:	f898 0003 	ldrb.w	r0, [r8, #3]
 8004842:	f898 1004 	ldrb.w	r1, [r8, #4]
 8004846:	e041      	b.n	80048cc <_defmt_write+0x114>
 8004848:	2900      	cmp	r1, #0
 800484a:	d05d      	beq.n	8004908 <_defmt_write+0x150>
 800484c:	f807 1c1e 	strb.w	r1, [r7, #-30]
 8004850:	465d      	mov	r5, fp
 8004852:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 8004856:	f000 0003 	and.w	r0, r0, #3
 800485a:	2802      	cmp	r0, #2
 800485c:	f244 504d 	movw	r0, #17741	@ 0x454d
 8004860:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004864:	bf08      	it	eq
 8004866:	4605      	moveq	r5, r0
 8004868:	f109 0618 	add.w	r6, r9, #24
 800486c:	f1a7 011e 	sub.w	r1, r7, #30
 8004870:	2201      	movs	r2, #1
 8004872:	4630      	mov	r0, r6
 8004874:	47a8      	blx	r5
 8004876:	2800      	cmp	r0, #0
 8004878:	d0f6      	beq.n	8004868 <_defmt_write+0xb0>
 800487a:	2801      	cmp	r0, #1
 800487c:	d162      	bne.n	8004944 <_defmt_write+0x18c>
 800487e:	f898 0003 	ldrb.w	r0, [r8, #3]
 8004882:	3001      	adds	r0, #1
 8004884:	f888 0003 	strb.w	r0, [r8, #3]
 8004888:	b2c1      	uxtb	r1, r0
 800488a:	2986      	cmp	r1, #134	@ 0x86
 800488c:	d1b4      	bne.n	80047f8 <_defmt_write+0x40>
 800488e:	20ff      	movs	r0, #255	@ 0xff
 8004890:	465d      	mov	r5, fp
 8004892:	f807 0c1d 	strb.w	r0, [r7, #-29]
 8004896:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 800489a:	f000 0003 	and.w	r0, r0, #3
 800489e:	2802      	cmp	r0, #2
 80048a0:	f244 504d 	movw	r0, #17741	@ 0x454d
 80048a4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80048a8:	bf08      	it	eq
 80048aa:	4605      	moveq	r5, r0
 80048ac:	f1a7 011d 	sub.w	r1, r7, #29
 80048b0:	4630      	mov	r0, r6
 80048b2:	2201      	movs	r2, #1
 80048b4:	47a8      	blx	r5
 80048b6:	2800      	cmp	r0, #0
 80048b8:	d0f8      	beq.n	80048ac <_defmt_write+0xf4>
 80048ba:	e797      	b.n	80047ec <_defmt_write+0x34>
 80048bc:	f898 1004 	ldrb.w	r1, [r8, #4]
 80048c0:	2301      	movs	r3, #1
 80048c2:	fa03 f202 	lsl.w	r2, r3, r2
 80048c6:	4311      	orrs	r1, r2
 80048c8:	f888 1004 	strb.w	r1, [r8, #4]
 80048cc:	3001      	adds	r0, #1
 80048ce:	f888 0003 	strb.w	r0, [r8, #3]
 80048d2:	b2c2      	uxtb	r2, r0
 80048d4:	2a07      	cmp	r2, #7
 80048d6:	d18f      	bne.n	80047f8 <_defmt_write+0x40>
 80048d8:	060a      	lsls	r2, r1, #24
 80048da:	d08d      	beq.n	80047f8 <_defmt_write+0x40>
 80048dc:	f88d 1008 	strb.w	r1, [sp, #8]
 80048e0:	465d      	mov	r5, fp
 80048e2:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 80048e6:	f000 0003 	and.w	r0, r0, #3
 80048ea:	2802      	cmp	r0, #2
 80048ec:	f244 504d 	movw	r0, #17741	@ 0x454d
 80048f0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80048f4:	bf08      	it	eq
 80048f6:	4605      	moveq	r5, r0
 80048f8:	f109 0018 	add.w	r0, r9, #24
 80048fc:	a902      	add	r1, sp, #8
 80048fe:	2201      	movs	r2, #1
 8004900:	47a8      	blx	r5
 8004902:	2800      	cmp	r0, #0
 8004904:	d0f8      	beq.n	80048f8 <_defmt_write+0x140>
 8004906:	e771      	b.n	80047ec <_defmt_write+0x34>
 8004908:	3079      	adds	r0, #121	@ 0x79
 800490a:	465d      	mov	r5, fp
 800490c:	f040 0080 	orr.w	r0, r0, #128	@ 0x80
 8004910:	f807 0c1f 	strb.w	r0, [r7, #-31]
 8004914:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 8004918:	f000 0003 	and.w	r0, r0, #3
 800491c:	2802      	cmp	r0, #2
 800491e:	f244 504d 	movw	r0, #17741	@ 0x454d
 8004922:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004926:	bf08      	it	eq
 8004928:	4605      	moveq	r5, r0
 800492a:	f109 0018 	add.w	r0, r9, #24
 800492e:	f1a7 011f 	sub.w	r1, r7, #31
 8004932:	2201      	movs	r2, #1
 8004934:	47a8      	blx	r5
 8004936:	2800      	cmp	r0, #0
 8004938:	d0f7      	beq.n	800492a <_defmt_write+0x172>
 800493a:	e757      	b.n	80047ec <_defmt_write+0x34>
 800493c:	b003      	add	sp, #12
 800493e:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8004942:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8004944:	f24a 7374 	movw	r3, #42868	@ 0xa774
 8004948:	2101      	movs	r1, #1
 800494a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800494e:	2201      	movs	r2, #1
 8004950:	f7fe ff55 	bl	80037fe <core::slice::index::slice_index_fail>

08004954 <ETH>:
 8004954:	b5b0      	push	{r4, r5, r7, lr}
 8004956:	af02      	add	r7, sp, #8
 8004958:	f003 ffc8 	bl	80088ec <__primask_r>
 800495c:	4604      	mov	r4, r0
 800495e:	f003 ffb8 	bl	80088d2 <__cpsid>
 8004962:	07e0      	lsls	r0, r4, #31
 8004964:	bf08      	it	eq
 8004966:	f003 ffb6 	bleq	80088d6 <__cpsie>
 800496a:	f249 0014 	movw	r0, #36884	@ 0x9014
 800496e:	2141      	movs	r1, #65	@ 0x41
 8004970:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 8004974:	f240 0440 	movw	r4, #64	@ 0x40
 8004978:	6805      	ldr	r5, [r0, #0]
 800497a:	f2c0 0101 	movt	r1, #1
 800497e:	f2c2 0400 	movt	r4, #8192	@ 0x2000
 8004982:	6001      	str	r1, [r0, #0]
 8004984:	07e8      	lsls	r0, r5, #31
 8004986:	d01d      	beq.n	80049c4 <ETH+0x70>
 8004988:	f3bf 8f5f 	dmb	sy
 800498c:	e854 0f09 	ldrex	r0, [r4, #36]	@ 0x24
 8004990:	f040 0102 	orr.w	r1, r0, #2
 8004994:	e844 1209 	strex	r2, r1, [r4, #36]	@ 0x24
 8004998:	2a00      	cmp	r2, #0
 800499a:	d1f7      	bne.n	800498c <ETH+0x38>
 800499c:	f3bf 8f5f 	dmb	sy
 80049a0:	b980      	cbnz	r0, 80049c4 <ETH+0x70>
 80049a2:	e9d4 1007 	ldrd	r1, r0, [r4, #28]
 80049a6:	2200      	movs	r2, #0
 80049a8:	61e2      	str	r2, [r4, #28]
 80049aa:	f3bf 8f5f 	dmb	sy
 80049ae:	e854 2f09 	ldrex	r2, [r4, #36]	@ 0x24
 80049b2:	f022 0202 	bic.w	r2, r2, #2
 80049b6:	e844 2309 	strex	r3, r2, [r4, #36]	@ 0x24
 80049ba:	2b00      	cmp	r3, #0
 80049bc:	d1f7      	bne.n	80049ae <ETH+0x5a>
 80049be:	b109      	cbz	r1, 80049c4 <ETH+0x70>
 80049c0:	6849      	ldr	r1, [r1, #4]
 80049c2:	4788      	blx	r1
 80049c4:	0668      	lsls	r0, r5, #25
 80049c6:	d51e      	bpl.n	8004a06 <ETH+0xb2>
 80049c8:	f3bf 8f5f 	dmb	sy
 80049cc:	e854 0f06 	ldrex	r0, [r4, #24]
 80049d0:	f040 0102 	orr.w	r1, r0, #2
 80049d4:	e844 1206 	strex	r2, r1, [r4, #24]
 80049d8:	2a00      	cmp	r2, #0
 80049da:	d1f7      	bne.n	80049cc <ETH+0x78>
 80049dc:	f3bf 8f5f 	dmb	sy
 80049e0:	b988      	cbnz	r0, 8004a06 <ETH+0xb2>
 80049e2:	e9d4 1004 	ldrd	r1, r0, [r4, #16]
 80049e6:	2200      	movs	r2, #0
 80049e8:	6122      	str	r2, [r4, #16]
 80049ea:	f3bf 8f5f 	dmb	sy
 80049ee:	e854 2f06 	ldrex	r2, [r4, #24]
 80049f2:	f022 0202 	bic.w	r2, r2, #2
 80049f6:	e844 2306 	strex	r3, r2, [r4, #24]
 80049fa:	2b00      	cmp	r3, #0
 80049fc:	d1f7      	bne.n	80049ee <ETH+0x9a>
 80049fe:	2900      	cmp	r1, #0
 8004a00:	bf1c      	itt	ne
 8004a02:	6849      	ldrne	r1, [r1, #4]
 8004a04:	4788      	blxne	r1
 8004a06:	f248 0138 	movw	r1, #32824	@ 0x8038
 8004a0a:	f2c4 0102 	movt	r1, #16386	@ 0x4002
 8004a0e:	6808      	ldr	r0, [r1, #0]
 8004a10:	0580      	lsls	r0, r0, #22
 8004a12:	bf44      	itt	mi
 8004a14:	f44f 7000 	movmi.w	r0, #512	@ 0x200
 8004a18:	6048      	strmi	r0, [r1, #4]
 8004a1a:	f3bf 8f5f 	dmb	sy
 8004a1e:	e854 0f0c 	ldrex	r0, [r4, #48]	@ 0x30
 8004a22:	f040 0202 	orr.w	r2, r0, #2
 8004a26:	e844 230c 	strex	r3, r2, [r4, #48]	@ 0x30
 8004a2a:	2b00      	cmp	r3, #0
 8004a2c:	d1f7      	bne.n	8004a1e <ETH+0xca>
 8004a2e:	f3bf 8f5f 	dmb	sy
 8004a32:	b110      	cbz	r0, 8004a3a <ETH+0xe6>
 8004a34:	f8d1 06f0 	ldr.w	r0, [r1, #1776]	@ 0x6f0
 8004a38:	bdb0      	pop	{r4, r5, r7, pc}
 8004a3a:	e9d4 200a 	ldrd	r2, r0, [r4, #40]	@ 0x28
 8004a3e:	2300      	movs	r3, #0
 8004a40:	62a3      	str	r3, [r4, #40]	@ 0x28
 8004a42:	f3bf 8f5f 	dmb	sy
 8004a46:	e854 3f0c 	ldrex	r3, [r4, #48]	@ 0x30
 8004a4a:	f023 0302 	bic.w	r3, r3, #2
 8004a4e:	e844 350c 	strex	r5, r3, [r4, #48]	@ 0x30
 8004a52:	2d00      	cmp	r5, #0
 8004a54:	d1f7      	bne.n	8004a46 <ETH+0xf2>
 8004a56:	2a00      	cmp	r2, #0
 8004a58:	d0ec      	beq.n	8004a34 <ETH+0xe0>
 8004a5a:	6851      	ldr	r1, [r2, #4]
 8004a5c:	e8bd 40b0 	ldmia.w	sp!, {r4, r5, r7, lr}
 8004a60:	4708      	bx	r1

08004a62 <SysTick>:
 8004a62:	b5d0      	push	{r4, r6, r7, lr}
 8004a64:	af02      	add	r7, sp, #8
 8004a66:	f003 ff41 	bl	80088ec <__primask_r>
 8004a6a:	4604      	mov	r4, r0
 8004a6c:	f003 ff31 	bl	80088d2 <__cpsid>
 8004a70:	f240 0040 	movw	r0, #64	@ 0x40
 8004a74:	f2c2 0000 	movt	r0, #8192	@ 0x2000
 8004a78:	e9d0 1202 	ldrd	r1, r2, [r0, #8]
 8004a7c:	3101      	adds	r1, #1
 8004a7e:	f142 0200 	adc.w	r2, r2, #0
 8004a82:	e9c0 1202 	strd	r1, r2, [r0, #8]
 8004a86:	07e0      	lsls	r0, r4, #31
 8004a88:	bf18      	it	ne
 8004a8a:	bdd0      	popne	{r4, r6, r7, pc}
 8004a8c:	e8bd 40d0 	ldmia.w	sp!, {r4, r6, r7, lr}
 8004a90:	f003 bf21 	b.w	80088d6 <__cpsie>

08004a94 <main>:
 8004a94:	b580      	push	{r7, lr}
 8004a96:	466f      	mov	r7, sp
 8004a98:	f000 f800 	bl	8004a9c <ip::__cortex_m_rt_main>

08004a9c <ip::__cortex_m_rt_main>:
 8004a9c:	b5b0      	push	{r4, r5, r7, lr}
 8004a9e:	af02      	add	r7, sp, #8
 8004aa0:	f5ad 5d01 	sub.w	sp, sp, #8256	@ 0x2040
 8004aa4:	f003 ff22 	bl	80088ec <__primask_r>
 8004aa8:	4604      	mov	r4, r0
 8004aaa:	f003 ff12 	bl	80088d2 <__cpsid>
 8004aae:	f240 0540 	movw	r5, #64	@ 0x40
 8004ab2:	07e1      	lsls	r1, r4, #31
 8004ab4:	f2c2 0500 	movt	r5, #8192	@ 0x2000
 8004ab8:	79a8      	ldrb	r0, [r5, #6]
 8004aba:	d107      	bne.n	8004acc <ip::__cortex_m_rt_main+0x30>
 8004abc:	07c0      	lsls	r0, r0, #31
 8004abe:	f041 82b0 	bne.w	8006022 <ip::__cortex_m_rt_main+0x1586>
 8004ac2:	2001      	movs	r0, #1
 8004ac4:	71a8      	strb	r0, [r5, #6]
 8004ac6:	f003 ff06 	bl	80088d6 <__cpsie>
 8004aca:	e004      	b.n	8004ad6 <ip::__cortex_m_rt_main+0x3a>
 8004acc:	07c0      	lsls	r0, r0, #31
 8004ace:	f041 82aa 	bne.w	8006026 <ip::__cortex_m_rt_main+0x158a>
 8004ad2:	2001      	movs	r0, #1
 8004ad4:	71a8      	strb	r0, [r5, #6]
 8004ad6:	f003 ff09 	bl	80088ec <__primask_r>
 8004ada:	4604      	mov	r4, r0
 8004adc:	f003 fef9 	bl	80088d2 <__cpsid>
 8004ae0:	7968      	ldrb	r0, [r5, #5]
 8004ae2:	07e1      	lsls	r1, r4, #31
 8004ae4:	d107      	bne.n	8004af6 <ip::__cortex_m_rt_main+0x5a>
 8004ae6:	2800      	cmp	r0, #0
 8004ae8:	f041 82a3 	bne.w	8006032 <ip::__cortex_m_rt_main+0x1596>
 8004aec:	2001      	movs	r0, #1
 8004aee:	7168      	strb	r0, [r5, #5]
 8004af0:	f003 fef1 	bl	80088d6 <__cpsie>
 8004af4:	e004      	b.n	8004b00 <ip::__cortex_m_rt_main+0x64>
 8004af6:	2800      	cmp	r0, #0
 8004af8:	f041 829d 	bne.w	8006036 <ip::__cortex_m_rt_main+0x159a>
 8004afc:	2001      	movs	r0, #1
 8004afe:	7168      	strb	r0, [r5, #5]
 8004b00:	f10d 0e18 	add.w	lr, sp, #24
 8004b04:	f64d 0c00 	movw	ip, #55296	@ 0xd800
 8004b08:	f24f 0480 	movw	r4, #61568	@ 0xf080
 8004b0c:	f64c 4300 	movw	r3, #52224	@ 0xcc00
 8004b10:	f245 0903 	movw	r9, #20483	@ 0x5003
 8004b14:	f24a 0b07 	movw	fp, #40967	@ 0xa007
 8004b18:	f50e 5097 	add.w	r0, lr, #4832	@ 0x12e0
 8004b1c:	f24a 1e20 	movw	lr, #41248	@ 0xa120
 8004b20:	f2c0 0e07 	movt	lr, #7
 8004b24:	f2c0 5cb8 	movt	ip, #1464	@ 0x5b8
 8004b28:	f04f 0a02 	mov.w	sl, #2
 8004b2c:	f04f 4800 	mov.w	r8, #2147483648	@ 0x80000000
 8004b30:	f2c0 24fa 	movt	r4, #762	@ 0x2fa
 8004b34:	f6c1 13bf 	movt	r3, #6591	@ 0x19bf
 8004b38:	f2cf 498e 	movt	r9, #62606	@ 0xf48e
 8004b3c:	f6ce 1b1c 	movt	fp, #59676	@ 0xe91c
 8004b40:	f8c0 eaf0 	str.w	lr, [r0, #2800]	@ 0xaf0
 8004b44:	f8cd cca0 	str.w	ip, [sp, #3232]	@ 0xca0
 8004b48:	e001      	b.n	8004b4e <ip::__cortex_m_rt_main+0xb2>
 8004b4a:	f10a 0a01 	add.w	sl, sl, #1
 8004b4e:	f1ba 0f09 	cmp.w	sl, #9
 8004b52:	f001 8246 	beq.w	8005fe2 <ip::__cortex_m_rt_main+0x1546>
 8004b56:	f1ba 0f04 	cmp.w	sl, #4
 8004b5a:	d3f6      	bcc.n	8004b4a <ip::__cortex_m_rt_main+0xae>
 8004b5c:	fbb8 f0fa 	udiv	r0, r8, sl
 8004b60:	f44f 72d8 	mov.w	r2, #432	@ 0x1b0
 8004b64:	3001      	adds	r0, #1
 8004b66:	0840      	lsrs	r0, r0, #1
 8004b68:	fba0 010e 	umull	r0, r1, r0, lr
 8004b6c:	e003      	b.n	8004b76 <ip::__cortex_m_rt_main+0xda>
 8004b6e:	1e55      	subs	r5, r2, #1
 8004b70:	2a33      	cmp	r2, #51	@ 0x33
 8004b72:	462a      	mov	r2, r5
 8004b74:	d3e9      	bcc.n	8004b4a <ip::__cortex_m_rt_main+0xae>
 8004b76:	fba0 5602 	umull	r5, r6, r0, r2
 8004b7a:	fb01 6602 	mla	r6, r1, r2, r6
 8004b7e:	0ead      	lsrs	r5, r5, #26
 8004b80:	ea45 1586 	orr.w	r5, r5, r6, lsl #6
 8004b84:	42a5      	cmp	r5, r4
 8004b86:	d3e0      	bcc.n	8004b4a <ip::__cortex_m_rt_main+0xae>
 8004b88:	f025 050f 	bic.w	r5, r5, #15
 8004b8c:	429d      	cmp	r5, r3
 8004b8e:	d8ee      	bhi.n	8004b6e <ip::__cortex_m_rt_main+0xd2>
 8004b90:	eb05 060b 	add.w	r6, r5, fp
 8004b94:	2e0f      	cmp	r6, #15
 8004b96:	bf24      	itt	cs
 8004b98:	444d      	addcs	r5, r9
 8004b9a:	2d07      	cmpcs	r5, #7
 8004b9c:	d2e7      	bcs.n	8004b6e <ip::__cortex_m_rt_main+0xd2>
 8004b9e:	ea5f 600a 	movs.w	r0, sl, lsl #24
 8004ba2:	d105      	bne.n	8004bb0 <ip::__cortex_m_rt_main+0x114>
 8004ba4:	f24b 10d8 	movw	r0, #45528	@ 0xb1d8
 8004ba8:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004bac:	f7ff f9f3 	bl	8003f96 <core::panicking::panic_const::panic_const_div_by_zero>
 8004bb0:	fa5f f08a 	uxtb.w	r0, sl
 8004bb4:	f04f 4100 	mov.w	r1, #2147483648	@ 0x80000000
 8004bb8:	fbb1 f0f0 	udiv	r0, r1, r0
 8004bbc:	b293      	uxth	r3, r2
 8004bbe:	3001      	adds	r0, #1
 8004bc0:	2e0f      	cmp	r6, #15
 8004bc2:	ea4f 0050 	mov.w	r0, r0, lsr #1
 8004bc6:	fba0 010e 	umull	r0, r1, r0, lr
 8004bca:	fba0 0503 	umull	r0, r5, r0, r3
 8004bce:	fb01 5103 	mla	r1, r1, r3, r5
 8004bd2:	f04f 051d 	mov.w	r5, #29
 8004bd6:	bf38      	it	cc
 8004bd8:	251c      	movcc	r5, #28
 8004bda:	f085 041f 	eor.w	r4, r5, #31
 8004bde:	0f80      	lsrs	r0, r0, #30
 8004be0:	ea40 0081 	orr.w	r0, r0, r1, lsl #2
 8004be4:	0f8b      	lsrs	r3, r1, #30
 8004be6:	40ab      	lsls	r3, r5
 8004be8:	0841      	lsrs	r1, r0, #1
 8004bea:	40a8      	lsls	r0, r5
 8004bec:	40e1      	lsrs	r1, r4
 8004bee:	0e80      	lsrs	r0, r0, #26
 8004bf0:	ea40 1081 	orr.w	r0, r0, r1, lsl #6
 8004bf4:	430b      	orrs	r3, r1
 8004bf6:	f020 010c 	bic.w	r1, r0, #12
 8004bfa:	f24e 6001 	movw	r0, #58881	@ 0xe601
 8004bfe:	f6c0 40df 	movt	r0, #3295	@ 0xcdf
 8004c02:	0e9b      	lsrs	r3, r3, #26
 8004c04:	1a08      	subs	r0, r1, r0
 8004c06:	f173 0000 	sbcs.w	r0, r3, #0
 8004c0a:	f081 81b7 	bcs.w	8005f7c <ip::__cortex_m_rt_main+0x14e0>
 8004c0e:	ee00 1a10 	vmov	s0, r1
 8004c12:	4561      	cmp	r1, ip
 8004c14:	bf38      	it	cc
 8004c16:	468c      	movcc	ip, r1
 8004c18:	eeb8 1a40 	vcvt.f32.u32	s2, s0
 8004c1c:	ee00 ca10 	vmov	s0, ip
 8004c20:	eeb8 0a40 	vcvt.f32.u32	s0, s0
 8004c24:	eeb1 2a41 	vneg.f32	s4, s2
 8004c28:	ee82 2a00 	vdiv.f32	s4, s4, s0
 8004c2c:	eebd 3ac2 	vcvt.s32.f32	s6, s4
 8004c30:	eebf 0a00 	vmov.f32	s0, #240	@ 0xbf800000 -1.0
 8004c34:	eeb8 3ac3 	vcvt.f32.s32	s6, s6
 8004c38:	eeb4 2a43 	vcmp.f32	s4, s6
 8004c3c:	ee33 4a00 	vadd.f32	s8, s6, s0
 8004c40:	eef1 fa10 	vmrs	APSR_nzcv, fpscr
 8004c44:	bf48      	it	mi
 8004c46:	eeb0 3a44 	vmovmi.f32	s6, s8
 8004c4a:	eeb1 2a43 	vneg.f32	s4, s6
 8004c4e:	eebc 2ac2 	vcvt.u32.f32	s4, s4
 8004c52:	ee12 0a10 	vmov	r0, s4
 8004c56:	2801      	cmp	r0, #1
 8004c58:	d011      	beq.n	8004c7e <ip::__cortex_m_rt_main+0x1e2>
 8004c5a:	2802      	cmp	r0, #2
 8004c5c:	d00a      	beq.n	8004c74 <ip::__cortex_m_rt_main+0x1d8>
 8004c5e:	2800      	cmp	r0, #0
 8004c60:	f001 8197 	beq.w	8005f92 <ip::__cortex_m_rt_main+0x14f6>
 8004c64:	1ec3      	subs	r3, r0, #3
 8004c66:	2b03      	cmp	r3, #3
 8004c68:	d26f      	bcs.n	8004d4a <ip::__cortex_m_rt_main+0x2ae>
 8004c6a:	eeb1 2a00 	vmov.f32	s4, #16	@ 0x40800000  4.0
 8004c6e:	f04f 0e90 	mov.w	lr, #144	@ 0x90
 8004c72:	e008      	b.n	8004c86 <ip::__cortex_m_rt_main+0x1ea>
 8004c74:	eeb0 2a00 	vmov.f32	s4, #0	@ 0x40000000  2.0
 8004c78:	f04f 0e80 	mov.w	lr, #128	@ 0x80
 8004c7c:	e003      	b.n	8004c86 <ip::__cortex_m_rt_main+0x1ea>
 8004c7e:	eeb7 2a00 	vmov.f32	s4, #112	@ 0x3f800000  1.0
 8004c82:	f04f 0e00 	mov.w	lr, #0
 8004c86:	ee81 1a02 	vdiv.f32	s2, s2, s4
 8004c8a:	f249 5c00 	movw	ip, #38144	@ 0x9500
 8004c8e:	f64f 1080 	movw	r0, #63872	@ 0xf980
 8004c92:	f24f 3300 	movw	r3, #62208	@ 0xf300
 8004c96:	f6c0 2cba 	movt	ip, #2746	@ 0xaba
 8004c9a:	f10c 0901 	add.w	r9, ip, #1
 8004c9e:	f2c0 3037 	movt	r0, #823	@ 0x337
 8004ca2:	f2c0 636f 	movt	r3, #1647	@ 0x66f
 8004ca6:	eebd 2ac1 	vcvt.s32.f32	s4, s2
 8004caa:	eeb8 2ac2 	vcvt.f32.s32	s4, s4
 8004cae:	eeb4 1a42 	vcmp.f32	s2, s4
 8004cb2:	ee32 3a00 	vadd.f32	s6, s4, s0
 8004cb6:	eef1 fa10 	vmrs	APSR_nzcv, fpscr
 8004cba:	bf48      	it	mi
 8004cbc:	eeb0 2a43 	vmovmi.f32	s4, s6
 8004cc0:	eebc 1ac2 	vcvt.u32.f32	s2, s4
 8004cc4:	4549      	cmp	r1, r9
 8004cc6:	bf3c      	itt	cc
 8004cc8:	f24a 5040 	movwcc	r0, #42304	@ 0xa540
 8004ccc:	f2c0 20ae 	movtcc	r0, #686	@ 0x2ae
 8004cd0:	bf3c      	itt	cc
 8004cd2:	f644 2380 	movwcc	r3, #19072	@ 0x4a80
 8004cd6:	f2c0 535d 	movtcc	r3, #1373	@ 0x55d
 8004cda:	ee11 5a10 	vmov	r5, s2
 8004cde:	eeb8 1a41 	vcvt.f32.u32	s2, s2
 8004ce2:	eeb1 1a41 	vneg.f32	s2, s2
 8004ce6:	4285      	cmp	r5, r0
 8004ce8:	bf38      	it	cc
 8004cea:	4628      	movcc	r0, r5
 8004cec:	ee02 0a10 	vmov	s4, r0
 8004cf0:	eeb8 2a42 	vcvt.f32.u32	s4, s4
 8004cf4:	ee81 2a02 	vdiv.f32	s4, s2, s4
 8004cf8:	eebd 3ac2 	vcvt.s32.f32	s6, s4
 8004cfc:	eeb8 3ac3 	vcvt.f32.s32	s6, s6
 8004d00:	eeb4 2a43 	vcmp.f32	s4, s6
 8004d04:	ee33 4a00 	vadd.f32	s8, s6, s0
 8004d08:	eef1 fa10 	vmrs	APSR_nzcv, fpscr
 8004d0c:	bf48      	it	mi
 8004d0e:	eeb0 3a44 	vmovmi.f32	s6, s8
 8004d12:	eeb1 2a43 	vneg.f32	s4, s6
 8004d16:	429d      	cmp	r5, r3
 8004d18:	bf38      	it	cc
 8004d1a:	462b      	movcc	r3, r5
 8004d1c:	eebc 2ac2 	vcvt.u32.f32	s4, s4
 8004d20:	ee12 0a10 	vmov	r0, s4
 8004d24:	2801      	cmp	r0, #1
 8004d26:	d00d      	beq.n	8004d44 <ip::__cortex_m_rt_main+0x2a8>
 8004d28:	2802      	cmp	r0, #2
 8004d2a:	d008      	beq.n	8004d3e <ip::__cortex_m_rt_main+0x2a2>
 8004d2c:	2800      	cmp	r0, #0
 8004d2e:	f001 813b 	beq.w	8005fa8 <ip::__cortex_m_rt_main+0x150c>
 8004d32:	1ec4      	subs	r4, r0, #3
 8004d34:	2c04      	cmp	r4, #4
 8004d36:	d210      	bcs.n	8004d5a <ip::__cortex_m_rt_main+0x2be>
 8004d38:	f44f 58a0 	mov.w	r8, #5120	@ 0x1400
 8004d3c:	e014      	b.n	8004d68 <ip::__cortex_m_rt_main+0x2cc>
 8004d3e:	f44f 5880 	mov.w	r8, #4096	@ 0x1000
 8004d42:	e011      	b.n	8004d68 <ip::__cortex_m_rt_main+0x2cc>
 8004d44:	f04f 0800 	mov.w	r8, #0
 8004d48:	e00e      	b.n	8004d68 <ip::__cortex_m_rt_main+0x2cc>
 8004d4a:	1f83      	subs	r3, r0, #6
 8004d4c:	2b06      	cmp	r3, #6
 8004d4e:	d272      	bcs.n	8004e36 <ip::__cortex_m_rt_main+0x39a>
 8004d50:	eeb2 2a00 	vmov.f32	s4, #32	@ 0x41000000  8.0
 8004d54:	f04f 0ea0 	mov.w	lr, #160	@ 0xa0
 8004d58:	e795      	b.n	8004c86 <ip::__cortex_m_rt_main+0x1ea>
 8004d5a:	3807      	subs	r0, #7
 8004d5c:	f44f 58e0 	mov.w	r8, #7168	@ 0x1c00
 8004d60:	2806      	cmp	r0, #6
 8004d62:	bf38      	it	cc
 8004d64:	f44f 58c0 	movcc.w	r8, #6144	@ 0x1800
 8004d68:	ee02 3a10 	vmov	s4, r3
 8004d6c:	eeb8 2a42 	vcvt.f32.u32	s4, s4
 8004d70:	ee81 1a02 	vdiv.f32	s2, s2, s4
 8004d74:	eebd 2ac1 	vcvt.s32.f32	s4, s2
 8004d78:	eeb8 2ac2 	vcvt.f32.s32	s4, s4
 8004d7c:	eeb4 1a42 	vcmp.f32	s2, s4
 8004d80:	ee32 0a00 	vadd.f32	s0, s4, s0
 8004d84:	eef1 fa10 	vmrs	APSR_nzcv, fpscr
 8004d88:	bf48      	it	mi
 8004d8a:	eeb0 2a40 	vmovmi.f32	s4, s0
 8004d8e:	eeb1 0a42 	vneg.f32	s0, s4
 8004d92:	9507      	str	r5, [sp, #28]
 8004d94:	eebc 0ac0 	vcvt.u32.f32	s0, s0
 8004d98:	ee10 0a10 	vmov	r0, s0
 8004d9c:	2801      	cmp	r0, #1
 8004d9e:	d019      	beq.n	8004dd4 <ip::__cortex_m_rt_main+0x338>
 8004da0:	2802      	cmp	r0, #2
 8004da2:	d00e      	beq.n	8004dc2 <ip::__cortex_m_rt_main+0x326>
 8004da4:	2800      	cmp	r0, #0
 8004da6:	f001 810a 	beq.w	8005fbe <ip::__cortex_m_rt_main+0x1522>
 8004daa:	1ec3      	subs	r3, r0, #3
 8004dac:	2b04      	cmp	r3, #4
 8004dae:	d21b      	bcs.n	8004de8 <ip::__cortex_m_rt_main+0x34c>
 8004db0:	f44f 4b20 	mov.w	fp, #40960	@ 0xa000
 8004db4:	f24c 3081 	movw	r0, #50049	@ 0xc381
 8004db8:	f2c0 10c9 	movt	r0, #457	@ 0x1c9
 8004dbc:	4281      	cmp	r1, r0
 8004dbe:	d311      	bcc.n	8004de4 <ip::__cortex_m_rt_main+0x348>
 8004dc0:	e01f      	b.n	8004e02 <ip::__cortex_m_rt_main+0x366>
 8004dc2:	f44f 4b00 	mov.w	fp, #32768	@ 0x8000
 8004dc6:	f24c 3081 	movw	r0, #50049	@ 0xc381
 8004dca:	f2c0 10c9 	movt	r0, #457	@ 0x1c9
 8004dce:	4281      	cmp	r1, r0
 8004dd0:	d308      	bcc.n	8004de4 <ip::__cortex_m_rt_main+0x348>
 8004dd2:	e016      	b.n	8004e02 <ip::__cortex_m_rt_main+0x366>
 8004dd4:	f04f 0b00 	mov.w	fp, #0
 8004dd8:	f24c 3081 	movw	r0, #50049	@ 0xc381
 8004ddc:	f2c0 10c9 	movt	r0, #457	@ 0x1c9
 8004de0:	4281      	cmp	r1, r0
 8004de2:	d20e      	bcs.n	8004e02 <ip::__cortex_m_rt_main+0x366>
 8004de4:	2300      	movs	r3, #0
 8004de6:	e043      	b.n	8004e70 <ip::__cortex_m_rt_main+0x3d4>
 8004de8:	3807      	subs	r0, #7
 8004dea:	f44f 4b60 	mov.w	fp, #57344	@ 0xe000
 8004dee:	2806      	cmp	r0, #6
 8004df0:	bf38      	it	cc
 8004df2:	f44f 4b40 	movcc.w	fp, #49152	@ 0xc000
 8004df6:	f24c 3081 	movw	r0, #50049	@ 0xc381
 8004dfa:	f2c0 10c9 	movt	r0, #457	@ 0x1c9
 8004dfe:	4281      	cmp	r1, r0
 8004e00:	d3f0      	bcc.n	8004de4 <ip::__cortex_m_rt_main+0x348>
 8004e02:	f248 7000 	movw	r0, #34560	@ 0x8700
 8004e06:	f2c0 3093 	movt	r0, #915	@ 0x393
 8004e0a:	3001      	adds	r0, #1
 8004e0c:	4281      	cmp	r1, r0
 8004e0e:	d201      	bcs.n	8004e14 <ip::__cortex_m_rt_main+0x378>
 8004e10:	2301      	movs	r3, #1
 8004e12:	e02d      	b.n	8004e70 <ip::__cortex_m_rt_main+0x3d4>
 8004e14:	f644 2080 	movw	r0, #19072	@ 0x4a80
 8004e18:	f2c0 505d 	movt	r0, #1373	@ 0x55d
 8004e1c:	3001      	adds	r0, #1
 8004e1e:	4281      	cmp	r1, r0
 8004e20:	d201      	bcs.n	8004e26 <ip::__cortex_m_rt_main+0x38a>
 8004e22:	2302      	movs	r3, #2
 8004e24:	e024      	b.n	8004e70 <ip::__cortex_m_rt_main+0x3d4>
 8004e26:	f640 6001 	movw	r0, #3585	@ 0xe01
 8004e2a:	f2c0 7027 	movt	r0, #1831	@ 0x727
 8004e2e:	4281      	cmp	r1, r0
 8004e30:	d20b      	bcs.n	8004e4a <ip::__cortex_m_rt_main+0x3ae>
 8004e32:	2303      	movs	r3, #3
 8004e34:	e01c      	b.n	8004e70 <ip::__cortex_m_rt_main+0x3d4>
 8004e36:	f1a0 030c 	sub.w	r3, r0, #12
 8004e3a:	2b1c      	cmp	r3, #28
 8004e3c:	f080 850d 	bcs.w	800585a <ip::__cortex_m_rt_main+0xdbe>
 8004e40:	eeb3 2a00 	vmov.f32	s4, #48	@ 0x41800000  16.0
 8004e44:	f04f 0eb0 	mov.w	lr, #176	@ 0xb0
 8004e48:	e71d      	b.n	8004c86 <ip::__cortex_m_rt_main+0x1ea>
 8004e4a:	f24d 1080 	movw	r0, #53632	@ 0xd180
 8004e4e:	2305      	movs	r3, #5
 8004e50:	f6c0 00f0 	movt	r0, #2288	@ 0x8f0
 8004e54:	3001      	adds	r0, #1
 8004e56:	4281      	cmp	r1, r0
 8004e58:	bf38      	it	cc
 8004e5a:	2304      	movcc	r3, #4
 8004e5c:	4549      	cmp	r1, r9
 8004e5e:	d307      	bcc.n	8004e70 <ip::__cortex_m_rt_main+0x3d4>
 8004e60:	f645 0081 	movw	r0, #22657	@ 0x5881
 8004e64:	2307      	movs	r3, #7
 8004e66:	f6c0 4084 	movt	r0, #3204	@ 0xc84
 8004e6a:	4281      	cmp	r1, r0
 8004e6c:	bf38      	it	cc
 8004e6e:	2306      	movcc	r3, #6
 8004e70:	f643 0910 	movw	r9, #14352	@ 0x3810
 8004e74:	f2c4 0902 	movt	r9, #16386	@ 0x4002
 8004e78:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004e7c:	f040 0001 	orr.w	r0, r0, #1
 8004e80:	f849 0c10 	str.w	r0, [r9, #-16]
 8004e84:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004e88:	0780      	lsls	r0, r0, #30
 8004e8a:	d40c      	bmi.n	8004ea6 <ip::__cortex_m_rt_main+0x40a>
 8004e8c:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004e90:	0780      	lsls	r0, r0, #30
 8004e92:	bf5c      	itt	pl
 8004e94:	f859 0c10 	ldrpl.w	r0, [r9, #-16]
 8004e98:	ea5f 7080 	movspl.w	r0, r0, lsl #30
 8004e9c:	d403      	bmi.n	8004ea6 <ip::__cortex_m_rt_main+0x40a>
 8004e9e:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004ea2:	0780      	lsls	r0, r0, #30
 8004ea4:	d5ee      	bpl.n	8004e84 <ip::__cortex_m_rt_main+0x3e8>
 8004ea6:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8004eaa:	2500      	movs	r5, #0
 8004eac:	f020 0003 	bic.w	r0, r0, #3
 8004eb0:	f849 0c08 	str.w	r0, [r9, #-8]
 8004eb4:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004eb8:	f440 2080 	orr.w	r0, r0, #262144	@ 0x40000
 8004ebc:	f849 0c10 	str.w	r0, [r9, #-16]
 8004ec0:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004ec4:	f440 3080 	orr.w	r0, r0, #65536	@ 0x10000
 8004ec8:	f849 0c10 	str.w	r0, [r9, #-16]
 8004ecc:	f647 2001 	movw	r0, #31233	@ 0x7a01
 8004ed0:	f6c0 2003 	movt	r0, #2563	@ 0xa03
 8004ed4:	4281      	cmp	r1, r0
 8004ed6:	f244 4001 	movw	r0, #17409	@ 0x4401
 8004eda:	bf38      	it	cc
 8004edc:	2501      	movcc	r5, #1
 8004ede:	f6c0 0095 	movt	r0, #2197	@ 0x895
 8004ee2:	4281      	cmp	r1, r0
 8004ee4:	bf38      	it	cc
 8004ee6:	2502      	movcc	r5, #2
 8004ee8:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004eec:	0380      	lsls	r0, r0, #14
 8004eee:	d40c      	bmi.n	8004f0a <ip::__cortex_m_rt_main+0x46e>
 8004ef0:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004ef4:	0380      	lsls	r0, r0, #14
 8004ef6:	bf5c      	itt	pl
 8004ef8:	f859 0c10 	ldrpl.w	r0, [r9, #-16]
 8004efc:	ea5f 3080 	movspl.w	r0, r0, lsl #14
 8004f00:	d403      	bmi.n	8004f0a <ip::__cortex_m_rt_main+0x46e>
 8004f02:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004f06:	0380      	lsls	r0, r0, #14
 8004f08:	d5ee      	bpl.n	8004ee8 <ip::__cortex_m_rt_main+0x44c>
 8004f0a:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004f0e:	2e0f      	cmp	r6, #15
 8004f10:	f020 7080 	bic.w	r0, r0, #16777216	@ 0x1000000
 8004f14:	f849 0c10 	str.w	r0, [r9, #-16]
 8004f18:	f647 70c0 	movw	r0, #32704	@ 0x7fc0
 8004f1c:	f859 4c0c 	ldr.w	r4, [r9, #-12]
 8004f20:	ea00 1082 	and.w	r0, r0, r2, lsl #6
 8004f24:	f248 0200 	movw	r2, #32768	@ 0x8000
 8004f28:	f2cf 02bc 	movt	r2, #61628	@ 0xf0bc
 8004f2c:	ea02 0204 	and.w	r2, r2, r4
 8004f30:	f00a 043f 	and.w	r4, sl, #63	@ 0x3f
 8004f34:	4422      	add	r2, r4
 8004f36:	4410      	add	r0, r2
 8004f38:	bf38      	it	cc
 8004f3a:	f500 3080 	addcc.w	r0, r0, #65536	@ 0x10000
 8004f3e:	2d02      	cmp	r5, #2
 8004f40:	f100 7010 	add.w	r0, r0, #37748736	@ 0x2400000
 8004f44:	f849 0c0c 	str.w	r0, [r9, #-12]
 8004f48:	f8d9 0030 	ldr.w	r0, [r9, #48]	@ 0x30
 8004f4c:	f040 5080 	orr.w	r0, r0, #268435456	@ 0x10000000
 8004f50:	f8c9 0030 	str.w	r0, [r9, #48]	@ 0x30
 8004f54:	f247 0004 	movw	r0, #28676	@ 0x7004
 8004f58:	f2c4 0000 	movt	r0, #16384	@ 0x4000
 8004f5c:	f850 2c04 	ldr.w	r2, [r0, #-4]
 8004f60:	d003      	beq.n	8004f6a <ip::__cortex_m_rt_main+0x4ce>
 8004f62:	2d01      	cmp	r5, #1
 8004f64:	d105      	bne.n	8004f72 <ip::__cortex_m_rt_main+0x4d6>
 8004f66:	2402      	movs	r4, #2
 8004f68:	e000      	b.n	8004f6c <ip::__cortex_m_rt_main+0x4d0>
 8004f6a:	2401      	movs	r4, #1
 8004f6c:	f364 328f 	bfi	r2, r4, #14, #2
 8004f70:	e001      	b.n	8004f76 <ip::__cortex_m_rt_main+0x4da>
 8004f72:	f442 4240 	orr.w	r2, r2, #49152	@ 0xc000
 8004f76:	f840 2c04 	str.w	r2, [r0, #-4]
 8004f7a:	f859 2c10 	ldr.w	r2, [r9, #-16]
 8004f7e:	f042 7280 	orr.w	r2, r2, #16777216	@ 0x1000000
 8004f82:	f849 2c10 	str.w	r2, [r9, #-16]
 8004f86:	f859 2c10 	ldr.w	r2, [r9, #-16]
 8004f8a:	0192      	lsls	r2, r2, #6
 8004f8c:	d40c      	bmi.n	8004fa8 <ip::__cortex_m_rt_main+0x50c>
 8004f8e:	f859 2c10 	ldr.w	r2, [r9, #-16]
 8004f92:	0192      	lsls	r2, r2, #6
 8004f94:	bf5c      	itt	pl
 8004f96:	f859 2c10 	ldrpl.w	r2, [r9, #-16]
 8004f9a:	ea5f 1282 	movspl.w	r2, r2, lsl #6
 8004f9e:	d403      	bmi.n	8004fa8 <ip::__cortex_m_rt_main+0x50c>
 8004fa0:	f859 2c10 	ldr.w	r2, [r9, #-16]
 8004fa4:	0192      	lsls	r2, r2, #6
 8004fa6:	d5ee      	bpl.n	8004f86 <ip::__cortex_m_rt_main+0x4ea>
 8004fa8:	4561      	cmp	r1, ip
 8004faa:	d925      	bls.n	8004ff8 <ip::__cortex_m_rt_main+0x55c>
 8004fac:	f850 1c04 	ldr.w	r1, [r0, #-4]
 8004fb0:	f441 3180 	orr.w	r1, r1, #65536	@ 0x10000
 8004fb4:	f840 1c04 	str.w	r1, [r0, #-4]
 8004fb8:	6801      	ldr	r1, [r0, #0]
 8004fba:	03c9      	lsls	r1, r1, #15
 8004fbc:	d409      	bmi.n	8004fd2 <ip::__cortex_m_rt_main+0x536>
 8004fbe:	6801      	ldr	r1, [r0, #0]
 8004fc0:	03c9      	lsls	r1, r1, #15
 8004fc2:	bf5c      	itt	pl
 8004fc4:	6801      	ldrpl	r1, [r0, #0]
 8004fc6:	ea5f 31c1 	movspl.w	r1, r1, lsl #15
 8004fca:	d402      	bmi.n	8004fd2 <ip::__cortex_m_rt_main+0x536>
 8004fcc:	6801      	ldr	r1, [r0, #0]
 8004fce:	03c9      	lsls	r1, r1, #15
 8004fd0:	d5f2      	bpl.n	8004fb8 <ip::__cortex_m_rt_main+0x51c>
 8004fd2:	f850 1c04 	ldr.w	r1, [r0, #-4]
 8004fd6:	f441 3100 	orr.w	r1, r1, #131072	@ 0x20000
 8004fda:	f840 1c04 	str.w	r1, [r0, #-4]
 8004fde:	6801      	ldr	r1, [r0, #0]
 8004fe0:	0389      	lsls	r1, r1, #14
 8004fe2:	d409      	bmi.n	8004ff8 <ip::__cortex_m_rt_main+0x55c>
 8004fe4:	6801      	ldr	r1, [r0, #0]
 8004fe6:	0389      	lsls	r1, r1, #14
 8004fe8:	bf5c      	itt	pl
 8004fea:	6801      	ldrpl	r1, [r0, #0]
 8004fec:	ea5f 3181 	movspl.w	r1, r1, lsl #14
 8004ff0:	d402      	bmi.n	8004ff8 <ip::__cortex_m_rt_main+0x55c>
 8004ff2:	6801      	ldr	r1, [r0, #0]
 8004ff4:	0389      	lsls	r1, r1, #14
 8004ff6:	d5f2      	bpl.n	8004fde <ip::__cortex_m_rt_main+0x542>
 8004ff8:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8004ffc:	f64f 71ff 	movw	r1, #65535	@ 0xffff
 8005000:	f2c0 019f 	movt	r1, #159	@ 0x9f
 8005004:	f64f 42f0 	movw	r2, #64752	@ 0xfcf0
 8005008:	4008      	ands	r0, r1
 800500a:	f849 0c08 	str.w	r0, [r9, #-8]
 800500e:	f8c9 33f0 	str.w	r3, [r9, #1008]	@ 0x3f0
 8005012:	ea48 000e 	orr.w	r0, r8, lr
 8005016:	f859 1c08 	ldr.w	r1, [r9, #-8]
 800501a:	ea40 000b 	orr.w	r0, r0, fp
 800501e:	4391      	bics	r1, r2
 8005020:	4308      	orrs	r0, r1
 8005022:	f849 0c08 	str.w	r0, [r9, #-8]
 8005026:	f859 0c08 	ldr.w	r0, [r9, #-8]
 800502a:	2102      	movs	r1, #2
 800502c:	f361 0001 	bfi	r0, r1, #0, #2
 8005030:	f849 0c08 	str.w	r0, [r9, #-8]
 8005034:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8005038:	f000 000c 	and.w	r0, r0, #12
 800503c:	2808      	cmp	r0, #8
 800503e:	d011      	beq.n	8005064 <ip::__cortex_m_rt_main+0x5c8>
 8005040:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8005044:	f000 000c 	and.w	r0, r0, #12
 8005048:	2808      	cmp	r0, #8
 800504a:	bf1e      	ittt	ne
 800504c:	f859 0c08 	ldrne.w	r0, [r9, #-8]
 8005050:	f000 000c 	andne.w	r0, r0, #12
 8005054:	2808      	cmpne	r0, #8
 8005056:	d005      	beq.n	8005064 <ip::__cortex_m_rt_main+0x5c8>
 8005058:	f859 0c08 	ldr.w	r0, [r9, #-8]
 800505c:	f000 000c 	and.w	r0, r0, #12
 8005060:	2808      	cmp	r0, #8
 8005062:	d1e7      	bne.n	8005034 <ip::__cortex_m_rt_main+0x598>
 8005064:	2010      	movs	r0, #16
 8005066:	f003 fc38 	bl	80088da <__delay>
 800506a:	f8d9 0020 	ldr.w	r0, [r9, #32]
 800506e:	f040 0001 	orr.w	r0, r0, #1
 8005072:	f8c9 0020 	str.w	r0, [r9, #32]
 8005076:	f003 fc36 	bl	80088e6 <__dsb>
 800507a:	f8d9 0000 	ldr.w	r0, [r9]
 800507e:	f040 0001 	orr.w	r0, r0, #1
 8005082:	f8c9 0000 	str.w	r0, [r9]
 8005086:	f8d9 0000 	ldr.w	r0, [r9]
 800508a:	f020 0001 	bic.w	r0, r0, #1
 800508e:	f8c9 0000 	str.w	r0, [r9]
 8005092:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8005096:	f040 0002 	orr.w	r0, r0, #2
 800509a:	f8c9 0020 	str.w	r0, [r9, #32]
 800509e:	f003 fc22 	bl	80088e6 <__dsb>
 80050a2:	f8d9 0000 	ldr.w	r0, [r9]
 80050a6:	f040 0002 	orr.w	r0, r0, #2
 80050aa:	f8c9 0000 	str.w	r0, [r9]
 80050ae:	f8d9 0000 	ldr.w	r0, [r9]
 80050b2:	f020 0002 	bic.w	r0, r0, #2
 80050b6:	f8c9 0000 	str.w	r0, [r9]
 80050ba:	f8d9 0020 	ldr.w	r0, [r9, #32]
 80050be:	f040 0004 	orr.w	r0, r0, #4
 80050c2:	f8c9 0020 	str.w	r0, [r9, #32]
 80050c6:	f003 fc0e 	bl	80088e6 <__dsb>
 80050ca:	f8d9 0000 	ldr.w	r0, [r9]
 80050ce:	f040 0004 	orr.w	r0, r0, #4
 80050d2:	f8c9 0000 	str.w	r0, [r9]
 80050d6:	f8d9 0000 	ldr.w	r0, [r9]
 80050da:	f020 0004 	bic.w	r0, r0, #4
 80050de:	f8c9 0000 	str.w	r0, [r9]
 80050e2:	f8d9 0020 	ldr.w	r0, [r9, #32]
 80050e6:	f040 0040 	orr.w	r0, r0, #64	@ 0x40
 80050ea:	f8c9 0020 	str.w	r0, [r9, #32]
 80050ee:	f003 fbfa 	bl	80088e6 <__dsb>
 80050f2:	f8d9 0000 	ldr.w	r0, [r9]
 80050f6:	f24e 0210 	movw	r2, #57360	@ 0xe010
 80050fa:	f2ce 0200 	movt	r2, #57344	@ 0xe000
 80050fe:	f64f 71fe 	movw	r1, #65534	@ 0xfffe
 8005102:	f040 0040 	orr.w	r0, r0, #64	@ 0x40
 8005106:	f8c9 0000 	str.w	r0, [r9]
 800510a:	f8d9 0000 	ldr.w	r0, [r9]
 800510e:	f2c0 01ff 	movt	r1, #255	@ 0xff
 8005112:	f020 0040 	bic.w	r0, r0, #64	@ 0x40
 8005116:	f8c9 0000 	str.w	r0, [r9]
 800511a:	68d0      	ldr	r0, [r2, #12]
 800511c:	4008      	ands	r0, r1
 800511e:	f649 119a 	movw	r1, #39322	@ 0x999a
 8005122:	f6c1 1199 	movt	r1, #6553	@ 0x1999
 8005126:	fba0 0101 	umull	r0, r1, r0, r1
 800512a:	6051      	str	r1, [r2, #4]
 800512c:	6810      	ldr	r0, [r2, #0]
 800512e:	f040 0001 	orr.w	r0, r0, #1
 8005132:	6010      	str	r0, [r2, #0]
 8005134:	6810      	ldr	r0, [r2, #0]
 8005136:	f040 0002 	orr.w	r0, r0, #2
 800513a:	6010      	str	r0, [r2, #0]
 800513c:	f240 002b 	movw	r0, #43	@ 0x2b
 8005140:	f2c0 0000 	movt	r0, #0
 8005144:	f7ff f997 	bl	8004476 <defmt::export::acquire_header_and_release>
 8005148:	2500      	movs	r5, #0
 800514a:	f640 0400 	movw	r4, #2048	@ 0x800
 800514e:	f2c4 0502 	movt	r5, #16386	@ 0x4002
 8005152:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 8005156:	68e8      	ldr	r0, [r5, #12]
 8005158:	f640 0208 	movw	r2, #2056	@ 0x808
 800515c:	f2c4 0202 	movt	r2, #16386	@ 0x4002
 8005160:	f641 0a00 	movw	sl, #6144	@ 0x1800
 8005164:	f020 000c 	bic.w	r0, r0, #12
 8005168:	60e8      	str	r0, [r5, #12]
 800516a:	6828      	ldr	r0, [r5, #0]
 800516c:	f2c4 0a02 	movt	sl, #16386	@ 0x4002
 8005170:	230b      	movs	r3, #11
 8005172:	2602      	movs	r6, #2
 8005174:	f020 000c 	bic.w	r0, r0, #12
 8005178:	6028      	str	r0, [r5, #0]
 800517a:	68e8      	ldr	r0, [r5, #12]
 800517c:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005180:	f04f 0800 	mov.w	r8, #0
 8005184:	f50d 6b4a 	add.w	fp, sp, #3232	@ 0xca0
 8005188:	f420 4040 	bic.w	r0, r0, #49152	@ 0xc000
 800518c:	60e8      	str	r0, [r5, #12]
 800518e:	6828      	ldr	r0, [r5, #0]
 8005190:	f420 4040 	bic.w	r0, r0, #49152	@ 0xc000
 8005194:	6028      	str	r0, [r5, #0]
 8005196:	f64f 400c 	movw	r0, #64524	@ 0xfc0c
 800519a:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 800519e:	5821      	ldr	r1, [r4, r0]
 80051a0:	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
 80051a4:	5021      	str	r1, [r4, r0]
 80051a6:	f8d5 1400 	ldr.w	r1, [r5, #1024]	@ 0x400
 80051aa:	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
 80051ae:	f8c5 1400 	str.w	r1, [r5, #1024]	@ 0x400
 80051b2:	6851      	ldr	r1, [r2, #4]
 80051b4:	f421 7140 	bic.w	r1, r1, #768	@ 0x300
 80051b8:	6051      	str	r1, [r2, #4]
 80051ba:	6821      	ldr	r1, [r4, #0]
 80051bc:	f421 7140 	bic.w	r1, r1, #768	@ 0x300
 80051c0:	6021      	str	r1, [r4, #0]
 80051c2:	6851      	ldr	r1, [r2, #4]
 80051c4:	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
 80051c8:	6051      	str	r1, [r2, #4]
 80051ca:	6821      	ldr	r1, [r4, #0]
 80051cc:	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
 80051d0:	6021      	str	r1, [r4, #0]
 80051d2:	f8da 100c 	ldr.w	r1, [sl, #12]
 80051d6:	f421 0140 	bic.w	r1, r1, #12582912	@ 0xc00000
 80051da:	f8ca 100c 	str.w	r1, [sl, #12]
 80051de:	f8da 1000 	ldr.w	r1, [sl]
 80051e2:	f421 0140 	bic.w	r1, r1, #12582912	@ 0xc00000
 80051e6:	f8ca 1000 	str.w	r1, [sl]
 80051ea:	f8da 100c 	ldr.w	r1, [sl, #12]
 80051ee:	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
 80051f2:	f8ca 100c 	str.w	r1, [sl, #12]
 80051f6:	f8da 1000 	ldr.w	r1, [sl]
 80051fa:	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
 80051fe:	f8ca 1000 	str.w	r1, [sl]
 8005202:	6a29      	ldr	r1, [r5, #32]
 8005204:	f363 210b 	bfi	r1, r3, #8, #4
 8005208:	6229      	str	r1, [r5, #32]
 800520a:	6829      	ldr	r1, [r5, #0]
 800520c:	f366 1105 	bfi	r1, r6, #4, #2
 8005210:	6029      	str	r1, [r5, #0]
 8005212:	68a9      	ldr	r1, [r5, #8]
 8005214:	f041 0130 	orr.w	r1, r1, #48	@ 0x30
 8005218:	60a9      	str	r1, [r5, #8]
 800521a:	6991      	ldr	r1, [r2, #24]
 800521c:	f363 1107 	bfi	r1, r3, #4, #4
 8005220:	6191      	str	r1, [r2, #24]
 8005222:	6821      	ldr	r1, [r4, #0]
 8005224:	f366 0183 	bfi	r1, r6, #2, #2
 8005228:	6021      	str	r1, [r4, #0]
 800522a:	6811      	ldr	r1, [r2, #0]
 800522c:	f041 010c 	orr.w	r1, r1, #12
 8005230:	6011      	str	r1, [r2, #0]
 8005232:	f64f 4118 	movw	r1, #64536	@ 0xfc18
 8005236:	f44f 1200 	mov.w	r2, #2097152	@ 0x200000
 800523a:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 800523e:	5062      	str	r2, [r4, r1]
 8005240:	5821      	ldr	r1, [r4, r0]
 8005242:	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
 8005246:	5021      	str	r1, [r4, r0]
 8005248:	f64f 4004 	movw	r0, #64516	@ 0xfc04
 800524c:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 8005250:	5821      	ldr	r1, [r4, r0]
 8005252:	f021 0120 	bic.w	r1, r1, #32
 8005256:	5021      	str	r1, [r4, r0]
 8005258:	2101      	movs	r1, #1
 800525a:	f8d5 0400 	ldr.w	r0, [r5, #1024]	@ 0x400
 800525e:	f361 208b 	bfi	r0, r1, #10, #2
 8005262:	f8c5 0400 	str.w	r0, [r5, #1024]	@ 0x400
 8005266:	f8ce 82ec 	str.w	r8, [lr, #748]	@ 0x2ec
 800526a:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800526e:	f50b 60cd 	add.w	r0, fp, #1640	@ 0x668
 8005272:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 8005276:	f8ce 82e8 	str.w	r8, [lr, #744]	@ 0x2e8
 800527a:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800527e:	f8cd 8660 	str.w	r8, [sp, #1632]	@ 0x660
 8005282:	f8ce 82e4 	str.w	r8, [lr, #740]	@ 0x2e4
 8005286:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800528a:	f8cd 8664 	str.w	r8, [sp, #1636]	@ 0x664
 800528e:	f8ce 82e0 	str.w	r8, [lr, #736]	@ 0x2e0
 8005292:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005296:	f8cd 8668 	str.w	r8, [sp, #1640]	@ 0x668
 800529a:	f8ce 82dc 	str.w	r8, [lr, #732]	@ 0x2dc
 800529e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 80052a2:	f8cd 866c 	str.w	r8, [sp, #1644]	@ 0x66c
 80052a6:	f8ce 82d8 	str.w	r8, [lr, #728]	@ 0x2d8
 80052aa:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 80052ae:	f8cd 8670 	str.w	r8, [sp, #1648]	@ 0x670
 80052b2:	f8ce 82d4 	str.w	r8, [lr, #724]	@ 0x2d4
 80052b6:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 80052ba:	f8cd 8674 	str.w	r8, [sp, #1652]	@ 0x674
 80052be:	f8cd 8678 	str.w	r8, [sp, #1656]	@ 0x678
 80052c2:	f8cd 867c 	str.w	r8, [sp, #1660]	@ 0x67c
 80052c6:	f8ce 82d0 	str.w	r8, [lr, #720]	@ 0x2d0
 80052ca:	9006      	str	r0, [sp, #24]
 80052cc:	f003 fb13 	bl	80088f6 <__aeabi_memclr8>
 80052d0:	ae08      	add	r6, sp, #32
 80052d2:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 80052d6:	f506 60d1 	add.w	r0, r6, #1672	@ 0x688
 80052da:	9005      	str	r0, [sp, #20]
 80052dc:	f003 fb0b 	bl	80088f6 <__aeabi_memclr8>
 80052e0:	4630      	mov	r0, r6
 80052e2:	2124      	movs	r1, #36	@ 0x24
 80052e4:	e9cd 8816 	strd	r8, r8, [sp, #88]	@ 0x58
 80052e8:	f8cd 8050 	str.w	r8, [sp, #80]	@ 0x50
 80052ec:	f8cd 8048 	str.w	r8, [sp, #72]	@ 0x48
 80052f0:	f003 fb01 	bl	80088f6 <__aeabi_memclr8>
 80052f4:	f106 0048 	add.w	r0, r6, #72	@ 0x48
 80052f8:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 80052fc:	9003      	str	r0, [sp, #12]
 80052fe:	f003 fafa 	bl	80088f6 <__aeabi_memclr8>
 8005302:	4658      	mov	r0, fp
 8005304:	2124      	movs	r1, #36	@ 0x24
 8005306:	f8cd 869c 	str.w	r8, [sp, #1692]	@ 0x69c
 800530a:	f8cd 8698 	str.w	r8, [sp, #1688]	@ 0x698
 800530e:	f8cd 8690 	str.w	r8, [sp, #1680]	@ 0x690
 8005312:	f8cd 8688 	str.w	r8, [sp, #1672]	@ 0x688
 8005316:	f8cd 8680 	str.w	r8, [sp, #1664]	@ 0x680
 800531a:	f88d 8cd0 	strb.w	r8, [sp, #3280]	@ 0xcd0
 800531e:	f8cd 8ccc 	str.w	r8, [sp, #3276]	@ 0xccc
 8005322:	f8cd 8cc8 	str.w	r8, [sp, #3272]	@ 0xcc8
 8005326:	f003 fae6 	bl	80088f6 <__aeabi_memclr8>
 800532a:	f10b 0038 	add.w	r0, fp, #56	@ 0x38
 800532e:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 8005332:	9004      	str	r0, [sp, #16]
 8005334:	f003 fadf 	bl	80088f6 <__aeabi_memclr8>
 8005338:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800533c:	210b      	movs	r1, #11
 800533e:	2302      	movs	r3, #2
 8005340:	220b      	movs	r2, #11
 8005342:	f88e 8300 	strb.w	r8, [lr, #768]	@ 0x300
 8005346:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800534a:	f8ce 82fc 	str.w	r8, [lr, #764]	@ 0x2fc
 800534e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005352:	f8ce 82f8 	str.w	r8, [lr, #760]	@ 0x2f8
 8005356:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800535a:	f8ce 82f0 	str.w	r8, [lr, #752]	@ 0x2f0
 800535e:	6a28      	ldr	r0, [r5, #32]
 8005360:	f361 1007 	bfi	r0, r1, #4, #4
 8005364:	6228      	str	r0, [r5, #32]
 8005366:	6828      	ldr	r0, [r5, #0]
 8005368:	f363 0083 	bfi	r0, r3, #2, #2
 800536c:	6028      	str	r0, [r5, #0]
 800536e:	68a8      	ldr	r0, [r5, #8]
 8005370:	f040 000c 	orr.w	r0, r0, #12
 8005374:	60a8      	str	r0, [r5, #8]
 8005376:	6a28      	ldr	r0, [r5, #32]
 8005378:	f361 701f 	bfi	r0, r1, #28, #4
 800537c:	6228      	str	r0, [r5, #32]
 800537e:	6828      	ldr	r0, [r5, #0]
 8005380:	f363 308f 	bfi	r0, r3, #14, #2
 8005384:	6028      	str	r0, [r5, #0]
 8005386:	68a8      	ldr	r0, [r5, #8]
 8005388:	f440 4040 	orr.w	r0, r0, #49152	@ 0xc000
 800538c:	60a8      	str	r0, [r5, #8]
 800538e:	f8da 0024 	ldr.w	r0, [sl, #36]	@ 0x24
 8005392:	f362 300f 	bfi	r0, r2, #12, #4
 8005396:	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
 800539a:	f8da 0000 	ldr.w	r0, [sl]
 800539e:	f363 5097 	bfi	r0, r3, #22, #2
 80053a2:	f8ca 0000 	str.w	r0, [sl]
 80053a6:	f8da 0008 	ldr.w	r0, [sl, #8]
 80053aa:	f440 0040 	orr.w	r0, r0, #12582912	@ 0xc00000
 80053ae:	f8ca 0008 	str.w	r0, [sl, #8]
 80053b2:	f8da 0024 	ldr.w	r0, [sl, #36]	@ 0x24
 80053b6:	f362 5017 	bfi	r0, r2, #20, #4
 80053ba:	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
 80053be:	f8da 0000 	ldr.w	r0, [sl]
 80053c2:	f363 609b 	bfi	r0, r3, #26, #2
 80053c6:	f8ca 0000 	str.w	r0, [sl]
 80053ca:	f8da 0008 	ldr.w	r0, [sl, #8]
 80053ce:	f040 6040 	orr.w	r0, r0, #201326592	@ 0xc000000
 80053d2:	f8ca 0008 	str.w	r0, [sl, #8]
 80053d6:	f64f 4024 	movw	r0, #64548	@ 0xfc24
 80053da:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 80053de:	5821      	ldr	r1, [r4, r0]
 80053e0:	f362 5117 	bfi	r1, r2, #20, #4
 80053e4:	5021      	str	r1, [r4, r0]
 80053e6:	f8d5 0400 	ldr.w	r0, [r5, #1024]	@ 0x400
 80053ea:	f363 609b 	bfi	r0, r3, #26, #2
 80053ee:	f8c5 0400 	str.w	r0, [r5, #1024]	@ 0x400
 80053f2:	f64f 4008 	movw	r0, #64520	@ 0xfc08
 80053f6:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 80053fa:	5821      	ldr	r1, [r4, r0]
 80053fc:	f041 6140 	orr.w	r1, r1, #201326592	@ 0xc000000
 8005400:	5021      	str	r1, [r4, r0]
 8005402:	f640 0108 	movw	r1, #2056	@ 0x808
 8005406:	f2c4 0102 	movt	r1, #16386	@ 0x4002
 800540a:	6988      	ldr	r0, [r1, #24]
 800540c:	f362 4013 	bfi	r0, r2, #16, #4
 8005410:	6188      	str	r0, [r1, #24]
 8005412:	6820      	ldr	r0, [r4, #0]
 8005414:	f363 2009 	bfi	r0, r3, #8, #2
 8005418:	6020      	str	r0, [r4, #0]
 800541a:	6808      	ldr	r0, [r1, #0]
 800541c:	f440 7040 	orr.w	r0, r0, #768	@ 0x300
 8005420:	6008      	str	r0, [r1, #0]
 8005422:	6988      	ldr	r0, [r1, #24]
 8005424:	f362 5017 	bfi	r0, r2, #20, #4
 8005428:	6188      	str	r0, [r1, #24]
 800542a:	6820      	ldr	r0, [r4, #0]
 800542c:	f363 208b 	bfi	r0, r3, #10, #2
 8005430:	6020      	str	r0, [r4, #0]
 8005432:	6808      	ldr	r0, [r1, #0]
 8005434:	f440 6040 	orr.w	r0, r0, #3072	@ 0xc00
 8005438:	6008      	str	r0, [r1, #0]
 800543a:	f003 fa57 	bl	80088ec <__primask_r>
 800543e:	4680      	mov	r8, r0
 8005440:	f003 fa47 	bl	80088d2 <__cpsid>
 8005444:	f8d9 0034 	ldr.w	r0, [r9, #52]	@ 0x34
 8005448:	ea5f 71c8 	movs.w	r1, r8, lsl #31
 800544c:	f440 4080 	orr.w	r0, r0, #16384	@ 0x4000
 8005450:	f8c9 0034 	str.w	r0, [r9, #52]	@ 0x34
 8005454:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8005458:	d126      	bne.n	80054a8 <ip::__cortex_m_rt_main+0xa0c>
 800545a:	0180      	lsls	r0, r0, #6
 800545c:	f8dd 801c 	ldr.w	r8, [sp, #28]
 8005460:	bf42      	ittt	mi
 8005462:	f8d9 0020 	ldrmi.w	r0, [r9, #32]
 8005466:	f020 7000 	bicmi.w	r0, r0, #33554432	@ 0x2000000
 800546a:	f8c9 0020 	strmi.w	r0, [r9, #32]
 800546e:	f643 0004 	movw	r0, #14340	@ 0x3804
 8005472:	f2c4 0001 	movt	r0, #16385	@ 0x4001
 8005476:	6801      	ldr	r1, [r0, #0]
 8005478:	f441 0100 	orr.w	r1, r1, #8388608	@ 0x800000
 800547c:	6001      	str	r1, [r0, #0]
 800547e:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8005482:	f040 6060 	orr.w	r0, r0, #234881024	@ 0xe000000
 8005486:	f8c9 0020 	str.w	r0, [r9, #32]
 800548a:	f8d9 0000 	ldr.w	r0, [r9]
 800548e:	f040 7000 	orr.w	r0, r0, #33554432	@ 0x2000000
 8005492:	f8c9 0000 	str.w	r0, [r9]
 8005496:	f8d9 0000 	ldr.w	r0, [r9]
 800549a:	f020 7000 	bic.w	r0, r0, #33554432	@ 0x2000000
 800549e:	f8c9 0000 	str.w	r0, [r9]
 80054a2:	f003 fa18 	bl	80088d6 <__cpsie>
 80054a6:	e023      	b.n	80054f0 <ip::__cortex_m_rt_main+0xa54>
 80054a8:	0180      	lsls	r0, r0, #6
 80054aa:	f8dd 801c 	ldr.w	r8, [sp, #28]
 80054ae:	bf42      	ittt	mi
 80054b0:	f8d9 0020 	ldrmi.w	r0, [r9, #32]
 80054b4:	f020 7000 	bicmi.w	r0, r0, #33554432	@ 0x2000000
 80054b8:	f8c9 0020 	strmi.w	r0, [r9, #32]
 80054bc:	f643 0004 	movw	r0, #14340	@ 0x3804
 80054c0:	f2c4 0001 	movt	r0, #16385	@ 0x4001
 80054c4:	6801      	ldr	r1, [r0, #0]
 80054c6:	f441 0100 	orr.w	r1, r1, #8388608	@ 0x800000
 80054ca:	6001      	str	r1, [r0, #0]
 80054cc:	f8d9 0020 	ldr.w	r0, [r9, #32]
 80054d0:	f040 6060 	orr.w	r0, r0, #234881024	@ 0xe000000
 80054d4:	f8c9 0020 	str.w	r0, [r9, #32]
 80054d8:	f8d9 0000 	ldr.w	r0, [r9]
 80054dc:	f040 7000 	orr.w	r0, r0, #33554432	@ 0x2000000
 80054e0:	f8c9 0000 	str.w	r0, [r9]
 80054e4:	f8d9 0000 	ldr.w	r0, [r9]
 80054e8:	f020 7000 	bic.w	r0, r0, #33554432	@ 0x2000000
 80054ec:	f8c9 0000 	str.w	r0, [r9]
 80054f0:	f249 0418 	movw	r4, #36888	@ 0x9018
 80054f4:	e9dd 5c05 	ldrd	r5, ip, [sp, #20]
 80054f8:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 80054fc:	e9dd 6e03 	ldrd	r6, lr, [sp, #12]
 8005500:	f854 0c18 	ldr.w	r0, [r4, #-24]
 8005504:	f50d 6a4a 	add.w	sl, sp, #3232	@ 0xca0
 8005508:	f10d 0b20 	add.w	fp, sp, #32
 800550c:	f040 0001 	orr.w	r0, r0, #1
 8005510:	f844 0c18 	str.w	r0, [r4, #-24]
 8005514:	a806      	add	r0, sp, #24
 8005516:	f500 5997 	add.w	r9, r0, #4832	@ 0x12e0
 800551a:	f854 0c18 	ldr.w	r0, [r4, #-24]
 800551e:	07c0      	lsls	r0, r0, #31
 8005520:	d00c      	beq.n	800553c <ip::__cortex_m_rt_main+0xaa0>
 8005522:	f854 0c18 	ldr.w	r0, [r4, #-24]
 8005526:	07c0      	lsls	r0, r0, #31
 8005528:	bf1c      	itt	ne
 800552a:	f854 0c18 	ldrne.w	r0, [r4, #-24]
 800552e:	ea5f 70c0 	movsne.w	r0, r0, lsl #31
 8005532:	d003      	beq.n	800553c <ip::__cortex_m_rt_main+0xaa0>
 8005534:	f854 0c18 	ldr.w	r0, [r4, #-24]
 8005538:	07c0      	lsls	r0, r0, #31
 800553a:	d1ee      	bne.n	800551a <ip::__cortex_m_rt_main+0xa7e>
 800553c:	6820      	ldr	r0, [r4, #0]
 800553e:	217f      	movs	r1, #127	@ 0x7f
 8005540:	f6cf 5100 	movt	r1, #64768	@ 0xfd00
 8005544:	f240 52f2 	movw	r2, #1522	@ 0x5f2
 8005548:	f040 60e4 	orr.w	r0, r0, #119537664	@ 0x7200000
 800554c:	f50b 63c8 	add.w	r3, fp, #1600	@ 0x640
 8005550:	f040 0084 	orr.w	r0, r0, #132	@ 0x84
 8005554:	6020      	str	r0, [r4, #0]
 8005556:	f854 0c18 	ldr.w	r0, [r4, #-24]
 800555a:	4008      	ands	r0, r1
 800555c:	f246 0180 	movw	r1, #24704	@ 0x6080
 8005560:	f2c0 21c1 	movt	r1, #705	@ 0x2c1
 8005564:	4308      	orrs	r0, r1
 8005566:	f44f 4180 	mov.w	r1, #16384	@ 0x4000
 800556a:	f844 0c18 	str.w	r0, [r4, #-24]
 800556e:	9109      	str	r1, [sp, #36]	@ 0x24
 8005570:	960a      	str	r6, [sp, #40]	@ 0x28
 8005572:	9809      	ldr	r0, [sp, #36]	@ 0x24
 8005574:	9611      	str	r6, [sp, #68]	@ 0x44
 8005576:	f362 000b 	bfi	r0, r2, #0, #12
 800557a:	9009      	str	r0, [sp, #36]	@ 0x24
 800557c:	2001      	movs	r0, #1
 800557e:	930b      	str	r3, [sp, #44]	@ 0x2c
 8005580:	960a      	str	r6, [sp, #40]	@ 0x28
 8005582:	9010      	str	r0, [sp, #64]	@ 0x40
 8005584:	e9cd 0312 	strd	r0, r3, [sp, #72]	@ 0x48
 8005588:	930b      	str	r3, [sp, #44]	@ 0x2c
 800558a:	f04f 4300 	mov.w	r3, #2147483648	@ 0x80000000
 800558e:	f3bf 8f5f 	dmb	sy
 8005592:	9308      	str	r3, [sp, #32]
 8005594:	f3bf 8f5f 	dmb	sy
 8005598:	f8cd 1664 	str.w	r1, [sp, #1636]	@ 0x664
 800559c:	f8cd 5668 	str.w	r5, [sp, #1640]	@ 0x668
 80055a0:	f8dd 1664 	ldr.w	r1, [sp, #1636]	@ 0x664
 80055a4:	f362 010b 	bfi	r1, r2, #0, #12
 80055a8:	f8cd 1664 	str.w	r1, [sp, #1636]	@ 0x664
 80055ac:	2100      	movs	r1, #0
 80055ae:	f8cd 166c 	str.w	r1, [sp, #1644]	@ 0x66c
 80055b2:	f8dd 2664 	ldr.w	r2, [sp, #1636]	@ 0x664
 80055b6:	f442 4200 	orr.w	r2, r2, #32768	@ 0x8000
 80055ba:	f8cd 2664 	str.w	r2, [sp, #1636]	@ 0x664
 80055be:	f8cd 5668 	str.w	r5, [sp, #1640]	@ 0x668
 80055c2:	f8cd 5684 	str.w	r5, [sp, #1668]	@ 0x684
 80055c6:	f248 7500 	movw	r5, #34560	@ 0x8700
 80055ca:	f8cd 0680 	str.w	r0, [sp, #1664]	@ 0x680
 80055ce:	f2c4 0502 	movt	r5, #16386	@ 0x4002
 80055d2:	f8cd 168c 	str.w	r1, [sp, #1676]	@ 0x68c
 80055d6:	f8cd 0688 	str.w	r0, [sp, #1672]	@ 0x688
 80055da:	f8cd 166c 	str.w	r1, [sp, #1644]	@ 0x66c
 80055de:	f3bf 8f5f 	dmb	sy
 80055e2:	f8cd 3660 	str.w	r3, [sp, #1632]	@ 0x660
 80055e6:	f3bf 8f5f 	dmb	sy
 80055ea:	f844 bc0c 	str.w	fp, [r4, #-12]
 80055ee:	6822      	ldr	r2, [r4, #0]
 80055f0:	f042 0202 	orr.w	r2, r2, #2
 80055f4:	6022      	str	r2, [r4, #0]
 80055f6:	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
 80055fa:	f844 0c10 	str.w	r0, [r4, #-16]
 80055fe:	f8cd 1ca0 	str.w	r1, [sp, #3232]	@ 0xca0
 8005602:	f8cd 1ca4 	str.w	r1, [sp, #3236]	@ 0xca4
 8005606:	f8cd 1ca8 	str.w	r1, [sp, #3240]	@ 0xca8
 800560a:	f8cd 1cac 	str.w	r1, [sp, #3244]	@ 0xcac
 800560e:	f8cd 1cb0 	str.w	r1, [sp, #3248]	@ 0xcb0
 8005612:	f8cd 1cb4 	str.w	r1, [sp, #3252]	@ 0xcb4
 8005616:	f8cd 1cb8 	str.w	r1, [sp, #3256]	@ 0xcb8
 800561a:	f8cd 1cbc 	str.w	r1, [sp, #3260]	@ 0xcbc
 800561e:	f8c2 12d0 	str.w	r1, [r2, #720]	@ 0x2d0
 8005622:	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
 8005626:	f8c2 12d4 	str.w	r1, [r2, #724]	@ 0x2d4
 800562a:	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
 800562e:	f8c2 12d8 	str.w	r1, [r2, #728]	@ 0x2d8
 8005632:	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
 8005636:	f8c2 12dc 	str.w	r1, [r2, #732]	@ 0x2dc
 800563a:	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
 800563e:	f8c2 12e0 	str.w	r1, [r2, #736]	@ 0x2e0
 8005642:	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
 8005646:	f8c2 12e4 	str.w	r1, [r2, #740]	@ 0x2e4
 800564a:	f50a 62c6 	add.w	r2, sl, #1584	@ 0x630
 800564e:	f8cd 2ccc 	str.w	r2, [sp, #3276]	@ 0xccc
 8005652:	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
 8005656:	f8c2 12e8 	str.w	r1, [r2, #744]	@ 0x2e8
 800565a:	f8cd ecc8 	str.w	lr, [sp, #3272]	@ 0xcc8
 800565e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005662:	f88e 0300 	strb.w	r0, [lr, #768]	@ 0x300
 8005666:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800566a:	f8ce 12ec 	str.w	r1, [lr, #748]	@ 0x2ec
 800566e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005672:	f8c9 c000 	str.w	ip, [r9]
 8005676:	f8ce 12fc 	str.w	r1, [lr, #764]	@ 0x2fc
 800567a:	f44f 7100 	mov.w	r1, #512	@ 0x200
 800567e:	f844 ac08 	str.w	sl, [r4, #-8]
 8005682:	f3bf 8f5f 	dmb	sy
 8005686:	6820      	ldr	r0, [r4, #0]
 8005688:	f440 5000 	orr.w	r0, r0, #8192	@ 0x2000
 800568c:	6020      	str	r0, [r4, #0]
 800568e:	f64f 103c 	movw	r0, #63804	@ 0xf93c
 8005692:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 8005696:	5029      	str	r1, [r5, r0]
 8005698:	ea5f 0058 	movs.w	r0, r8, lsr #1
 800569c:	d105      	bne.n	80056aa <ip::__cortex_m_rt_main+0xc0e>
 800569e:	f24b 00e8 	movw	r0, #45288	@ 0xb0e8
 80056a2:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80056a6:	f7fe fc76 	bl	8003f96 <core::panicking::panic_const::panic_const_div_by_zero>
 80056aa:	f06f 4100 	mvn.w	r1, #2147483648	@ 0x80000000
 80056ae:	eb01 0298 	add.w	r2, r1, r8, lsr #2
 80056b2:	fbb2 f2f0 	udiv	r2, r2, r0
 80056b6:	f248 7604 	movw	r6, #34564	@ 0x8704
 80056ba:	fbb1 f0f2 	udiv	r0, r1, r2
 80056be:	f04f 31ff 	mov.w	r1, #4294967295	@ 0xffffffff
 80056c2:	fba0 0101 	umull	r0, r1, r0, r1
 80056c6:	f242 1303 	movw	r3, #8451	@ 0x2103
 80056ca:	b2d2      	uxtb	r2, r2
 80056cc:	f2c4 0602 	movt	r6, #16386	@ 0x4002
 80056d0:	602b      	str	r3, [r5, #0]
 80056d2:	2300      	movs	r3, #0
 80056d4:	6032      	str	r2, [r6, #0]
 80056d6:	4642      	mov	r2, r8
 80056d8:	f003 ff4a 	bl	8009570 <__aeabi_uldivmod>
 80056dc:	6170      	str	r0, [r6, #20]
 80056de:	6828      	ldr	r0, [r5, #0]
 80056e0:	0680      	lsls	r0, r0, #26
 80056e2:	d509      	bpl.n	80056f8 <ip::__cortex_m_rt_main+0xc5c>
 80056e4:	6828      	ldr	r0, [r5, #0]
 80056e6:	0680      	lsls	r0, r0, #26
 80056e8:	bf44      	itt	mi
 80056ea:	6828      	ldrmi	r0, [r5, #0]
 80056ec:	ea5f 6080 	movsmi.w	r0, r0, lsl #26
 80056f0:	d502      	bpl.n	80056f8 <ip::__cortex_m_rt_main+0xc5c>
 80056f2:	6828      	ldr	r0, [r5, #0]
 80056f4:	0680      	lsls	r0, r0, #26
 80056f6:	d4f2      	bmi.n	80056de <ip::__cortex_m_rt_main+0xc42>
 80056f8:	6828      	ldr	r0, [r5, #0]
 80056fa:	f040 0020 	orr.w	r0, r0, #32
 80056fe:	6028      	str	r0, [r5, #0]
 8005700:	6828      	ldr	r0, [r5, #0]
 8005702:	0680      	lsls	r0, r0, #26
 8005704:	d509      	bpl.n	800571a <ip::__cortex_m_rt_main+0xc7e>
 8005706:	6828      	ldr	r0, [r5, #0]
 8005708:	0680      	lsls	r0, r0, #26
 800570a:	bf44      	itt	mi
 800570c:	6828      	ldrmi	r0, [r5, #0]
 800570e:	ea5f 6080 	movsmi.w	r0, r0, lsl #26
 8005712:	d502      	bpl.n	800571a <ip::__cortex_m_rt_main+0xc7e>
 8005714:	6828      	ldr	r0, [r5, #0]
 8005716:	0680      	lsls	r0, r0, #26
 8005718:	d4f2      	bmi.n	8005700 <ip::__cortex_m_rt_main+0xc64>
 800571a:	2000      	movs	r0, #0
 800571c:	60f0      	str	r0, [r6, #12]
 800571e:	6130      	str	r0, [r6, #16]
 8005720:	6828      	ldr	r0, [r5, #0]
 8005722:	0740      	lsls	r0, r0, #29
 8005724:	d509      	bpl.n	800573a <ip::__cortex_m_rt_main+0xc9e>
 8005726:	6828      	ldr	r0, [r5, #0]
 8005728:	0740      	lsls	r0, r0, #29
 800572a:	bf44      	itt	mi
 800572c:	6828      	ldrmi	r0, [r5, #0]
 800572e:	ea5f 7040 	movsmi.w	r0, r0, lsl #29
 8005732:	d502      	bpl.n	800573a <ip::__cortex_m_rt_main+0xc9e>
 8005734:	6828      	ldr	r0, [r5, #0]
 8005736:	0740      	lsls	r0, r0, #29
 8005738:	d4f2      	bmi.n	8005720 <ip::__cortex_m_rt_main+0xc84>
 800573a:	6828      	ldr	r0, [r5, #0]
 800573c:	f040 0004 	orr.w	r0, r0, #4
 8005740:	6028      	str	r0, [r5, #0]
 8005742:	6828      	ldr	r0, [r5, #0]
 8005744:	0740      	lsls	r0, r0, #29
 8005746:	d509      	bpl.n	800575c <ip::__cortex_m_rt_main+0xcc0>
 8005748:	6828      	ldr	r0, [r5, #0]
 800574a:	0740      	lsls	r0, r0, #29
 800574c:	bf44      	itt	mi
 800574e:	6828      	ldrmi	r0, [r5, #0]
 8005750:	ea5f 7040 	movsmi.w	r0, r0, lsl #29
 8005754:	d502      	bpl.n	800575c <ip::__cortex_m_rt_main+0xcc0>
 8005756:	6828      	ldr	r0, [r5, #0]
 8005758:	0740      	lsls	r0, r0, #29
 800575a:	d4f2      	bmi.n	8005742 <ip::__cortex_m_rt_main+0xca6>
 800575c:	f647 0040 	movw	r0, #30784	@ 0x7840
 8005760:	f2c0 107d 	movt	r0, #381	@ 0x17d
 8005764:	4580      	cmp	r8, r0
 8005766:	d260      	bcs.n	800582a <ip::__cortex_m_rt_main+0xd8e>
 8005768:	6820      	ldr	r0, [r4, #0]
 800576a:	f420 5000 	bic.w	r0, r0, #8192	@ 0x2000
 800576e:	6020      	str	r0, [r4, #0]
 8005770:	f854 0c04 	ldr.w	r0, [r4, #-4]
 8005774:	f3c0 5002 	ubfx	r0, r0, #20, #3
 8005778:	3801      	subs	r0, #1
 800577a:	2802      	cmp	r0, #2
 800577c:	d814      	bhi.n	80057a8 <ip::__cortex_m_rt_main+0xd0c>
 800577e:	f854 0c04 	ldr.w	r0, [r4, #-4]
 8005782:	f3c0 5002 	ubfx	r0, r0, #20, #3
 8005786:	3801      	subs	r0, #1
 8005788:	2802      	cmp	r0, #2
 800578a:	bf9f      	itttt	ls
 800578c:	f854 0c04 	ldrls.w	r0, [r4, #-4]
 8005790:	f3c0 5002 	ubfxls	r0, r0, #20, #3
 8005794:	3801      	subls	r0, #1
 8005796:	2802      	cmpls	r0, #2
 8005798:	d806      	bhi.n	80057a8 <ip::__cortex_m_rt_main+0xd0c>
 800579a:	f854 0c04 	ldr.w	r0, [r4, #-4]
 800579e:	f3c0 5002 	ubfx	r0, r0, #20, #3
 80057a2:	3801      	subs	r0, #1
 80057a4:	2803      	cmp	r0, #3
 80057a6:	d3e3      	bcc.n	8005770 <ip::__cortex_m_rt_main+0xcd4>
 80057a8:	6820      	ldr	r0, [r4, #0]
 80057aa:	f020 0002 	bic.w	r0, r0, #2
 80057ae:	6020      	str	r0, [r4, #0]
 80057b0:	2001      	movs	r0, #1
 80057b2:	f854 1c04 	ldr.w	r1, [r4, #-4]
 80057b6:	f3c1 4142 	ubfx	r1, r1, #17, #3
 80057ba:	2907      	cmp	r1, #7
 80057bc:	d825      	bhi.n	800580a <ip::__cortex_m_rt_main+0xd6e>
 80057be:	fa00 f101 	lsl.w	r1, r0, r1
 80057c2:	f011 0faa 	tst.w	r1, #170	@ 0xaa
 80057c6:	d020      	beq.n	800580a <ip::__cortex_m_rt_main+0xd6e>
 80057c8:	f854 1c04 	ldr.w	r1, [r4, #-4]
 80057cc:	f3c1 4142 	ubfx	r1, r1, #17, #3
 80057d0:	2907      	cmp	r1, #7
 80057d2:	d81a      	bhi.n	800580a <ip::__cortex_m_rt_main+0xd6e>
 80057d4:	fa00 f101 	lsl.w	r1, r0, r1
 80057d8:	f011 0faa 	tst.w	r1, #170	@ 0xaa
 80057dc:	d015      	beq.n	800580a <ip::__cortex_m_rt_main+0xd6e>
 80057de:	f854 1c04 	ldr.w	r1, [r4, #-4]
 80057e2:	f3c1 4142 	ubfx	r1, r1, #17, #3
 80057e6:	2907      	cmp	r1, #7
 80057e8:	d80f      	bhi.n	800580a <ip::__cortex_m_rt_main+0xd6e>
 80057ea:	fa00 f101 	lsl.w	r1, r0, r1
 80057ee:	f011 0faa 	tst.w	r1, #170	@ 0xaa
 80057f2:	d00a      	beq.n	800580a <ip::__cortex_m_rt_main+0xd6e>
 80057f4:	f854 1c04 	ldr.w	r1, [r4, #-4]
 80057f8:	f3c1 4142 	ubfx	r1, r1, #17, #3
 80057fc:	2907      	cmp	r1, #7
 80057fe:	d804      	bhi.n	800580a <ip::__cortex_m_rt_main+0xd6e>
 8005800:	fa00 f101 	lsl.w	r1, r0, r1
 8005804:	f011 0faa 	tst.w	r1, #170	@ 0xaa
 8005808:	d1d3      	bne.n	80057b2 <ip::__cortex_m_rt_main+0xd16>
 800580a:	f64a 1010 	movw	r0, #43280	@ 0xa910
 800580e:	f64a 1200 	movw	r2, #43264	@ 0xa900
 8005812:	f64a 133c 	movw	r3, #43324	@ 0xa93c
 8005816:	f1a7 0111 	sub.w	r1, r7, #17
 800581a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800581e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005822:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8005826:	f7fe fca7 	bl	8004178 <core::result::unwrap_failed>
 800582a:	f640 60c0 	movw	r0, #3776	@ 0xec0
 800582e:	f2c0 2016 	movt	r0, #534	@ 0x216
 8005832:	4580      	cmp	r8, r0
 8005834:	d201      	bcs.n	800583a <ip::__cortex_m_rt_main+0xd9e>
 8005836:	2008      	movs	r0, #8
 8005838:	e022      	b.n	8005880 <ip::__cortex_m_rt_main+0xde4>
 800583a:	f248 7000 	movw	r0, #34560	@ 0x8700
 800583e:	f2c0 3093 	movt	r0, #915	@ 0x393
 8005842:	4580      	cmp	r8, r0
 8005844:	d201      	bcs.n	800584a <ip::__cortex_m_rt_main+0xdae>
 8005846:	200c      	movs	r0, #12
 8005848:	e01a      	b.n	8005880 <ip::__cortex_m_rt_main+0xde4>
 800584a:	f24e 1000 	movw	r0, #57600	@ 0xe100
 800584e:	f2c0 50f5 	movt	r0, #1525	@ 0x5f5
 8005852:	4580      	cmp	r8, r0
 8005854:	d20c      	bcs.n	8005870 <ip::__cortex_m_rt_main+0xdd4>
 8005856:	2000      	movs	r0, #0
 8005858:	e012      	b.n	8005880 <ip::__cortex_m_rt_main+0xde4>
 800585a:	f1a0 0328 	sub.w	r3, r0, #40	@ 0x28
 800585e:	2b38      	cmp	r3, #56	@ 0x38
 8005860:	f080 8341 	bcs.w	8005ee6 <ip::__cortex_m_rt_main+0x144a>
 8005864:	ed9f 2acf 	vldr	s4, [pc, #828]	@ 8005ba4 <ip::__cortex_m_rt_main+0x1108>
 8005868:	f04f 0ec0 	mov.w	lr, #192	@ 0xc0
 800586c:	f7ff ba0b 	b.w	8004c86 <ip::__cortex_m_rt_main+0x1ea>
 8005870:	f24d 1180 	movw	r1, #53632	@ 0xd180
 8005874:	2010      	movs	r0, #16
 8005876:	f6c0 01f0 	movt	r1, #2288	@ 0x8f0
 800587a:	4588      	cmp	r8, r1
 800587c:	bf38      	it	cc
 800587e:	2004      	movcc	r0, #4
 8005880:	f64f 1110 	movw	r1, #63760	@ 0xf910
 8005884:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005888:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 800588c:	f50d 56c9 	add.w	r6, sp, #6432	@ 0x1920
 8005890:	586a      	ldr	r2, [r5, r1]
 8005892:	f022 021c 	bic.w	r2, r2, #28
 8005896:	4310      	orrs	r0, r2
 8005898:	5068      	str	r0, [r5, r1]
 800589a:	f64f 1000 	movw	r0, #63744	@ 0xf900
 800589e:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 80058a2:	f644 628c 	movw	r2, #20108	@ 0x4e8c
 80058a6:	5829      	ldr	r1, [r5, r0]
 80058a8:	f2c0 2200 	movt	r2, #512	@ 0x200
 80058ac:	4311      	orrs	r1, r2
 80058ae:	5029      	str	r1, [r5, r0]
 80058b0:	f64f 1104 	movw	r1, #63748	@ 0xf904
 80058b4:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 80058b8:	586a      	ldr	r2, [r5, r1]
 80058ba:	f042 4200 	orr.w	r2, r2, #2147483648	@ 0x80000000
 80058be:	f042 0201 	orr.w	r2, r2, #1
 80058c2:	506a      	str	r2, [r5, r1]
 80058c4:	f64f 1118 	movw	r1, #63768	@ 0xf918
 80058c8:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 80058cc:	586a      	ldr	r2, [r5, r1]
 80058ce:	f2c0 1200 	movt	r2, #256	@ 0x100
 80058d2:	506a      	str	r2, [r5, r1]
 80058d4:	f64f 210c 	movw	r1, #64012	@ 0xfa0c
 80058d8:	2260      	movs	r2, #96	@ 0x60
 80058da:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 80058de:	f2c0 0202 	movt	r2, #2
 80058e2:	506a      	str	r2, [r5, r1]
 80058e4:	f64f 2110 	movw	r1, #64016	@ 0xfa10
 80058e8:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 80058ec:	f44f 1203 	mov.w	r2, #2146304	@ 0x20c000
 80058f0:	506a      	str	r2, [r5, r1]
 80058f2:	586a      	ldr	r2, [r5, r1]
 80058f4:	f442 1200 	orr.w	r2, r2, #2097152	@ 0x200000
 80058f8:	506a      	str	r2, [r5, r1]
 80058fa:	f04f 3203 	mov.w	r2, #50529027	@ 0x3030303
 80058fe:	5829      	ldr	r1, [r5, r0]
 8005900:	f441 4190 	orr.w	r1, r1, #18432	@ 0x4800
 8005904:	5029      	str	r1, [r5, r0]
 8005906:	2500      	movs	r5, #0
 8005908:	2002      	movs	r0, #2
 800590a:	f509 61c2 	add.w	r1, r9, #1552	@ 0x610
 800590e:	e881 0421 	stmia.w	r1, {r0, r5, sl}
 8005912:	f24e 0110 	movw	r1, #57360	@ 0xe010
 8005916:	f2ce 0100 	movt	r1, #57344	@ 0xe000
 800591a:	f8c9 5624 	str.w	r5, [r9, #1572]	@ 0x624
 800591e:	f8c9 5620 	str.w	r5, [r9, #1568]	@ 0x620
 8005922:	f8c9 061c 	str.w	r0, [r9, #1564]	@ 0x61c
 8005926:	f8c9 b60c 	str.w	fp, [r9, #1548]	@ 0x60c
 800592a:	6860      	ldr	r0, [r4, #4]
 800592c:	f440 3080 	orr.w	r0, r0, #65536	@ 0x10000
 8005930:	f040 0041 	orr.w	r0, r0, #65	@ 0x41
 8005934:	6060      	str	r0, [r4, #4]
 8005936:	f04f 5000 	mov.w	r0, #536870912	@ 0x20000000
 800593a:	2401      	movs	r4, #1
 800593c:	f8c1 00f4 	str.w	r0, [r1, #244]	@ 0xf4
 8005940:	f64e 71be 	movw	r1, #61374	@ 0xefbe
 8005944:	f64e 7039 	movw	r0, #61241	@ 0xef39
 8005948:	f8ae 1a8c 	strh.w	r1, [lr, #2700]	@ 0xa8c
 800594c:	2100      	movs	r1, #0
 800594e:	f2c7 5090 	movt	r0, #30096	@ 0x7590
 8005952:	f6ca 51de 	movt	r1, #44510	@ 0xadde
 8005956:	f8c9 0640 	str.w	r0, [r9, #1600]	@ 0x640
 800595a:	f8c9 1790 	str.w	r1, [r9, #1936]	@ 0x790
 800595e:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 8005962:	f106 0020 	add.w	r0, r6, #32
 8005966:	f8c9 1634 	str.w	r1, [r9, #1588]	@ 0x634
 800596a:	21c0      	movs	r1, #192	@ 0xc0
 800596c:	f8c9 5710 	str.w	r5, [r9, #1808]	@ 0x710
 8005970:	f8c9 5714 	str.w	r5, [r9, #1812]	@ 0x714
 8005974:	f8c9 5718 	str.w	r5, [r9, #1816]	@ 0x718
 8005978:	f8c9 462c 	str.w	r4, [r9, #1580]	@ 0x62c
 800597c:	f8c9 2630 	str.w	r2, [r9, #1584]	@ 0x630
 8005980:	f8c9 5638 	str.w	r5, [r9, #1592]	@ 0x638
 8005984:	f8c9 563c 	str.w	r5, [r9, #1596]	@ 0x63c
 8005988:	f8c9 5644 	str.w	r5, [r9, #1604]	@ 0x644
 800598c:	f8c9 4628 	str.w	r4, [r9, #1576]	@ 0x628
 8005990:	f002 ffb1 	bl	80088f6 <__aeabi_memclr8>
 8005994:	f506 7080 	add.w	r0, r6, #256	@ 0x100
 8005998:	2164      	movs	r1, #100	@ 0x64
 800599a:	f002 ffac 	bl	80088f6 <__aeabi_memclr8>
 800599e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 80059a2:	2018      	movs	r0, #24
 80059a4:	f8c9 4718 	str.w	r4, [r9, #1816]	@ 0x718
 80059a8:	f88e 0a18 	strb.w	r0, [lr, #2584]	@ 0xa18
 80059ac:	f64f 70fe 	movw	r0, #65534	@ 0xfffe
 80059b0:	f2c0 00ff 	movt	r0, #255	@ 0xff
 80059b4:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 80059b8:	300c      	adds	r0, #12
 80059ba:	f8c9 071c 	str.w	r0, [r9, #1820]	@ 0x71c
 80059be:	f000 01f0 	and.w	r1, r0, #240	@ 0xf0
 80059c2:	f88e 5a8e 	strb.w	r5, [lr, #2702]	@ 0xa8e
 80059c6:	29e0      	cmp	r1, #224	@ 0xe0
 80059c8:	f8c9 5708 	str.w	r5, [r9, #1800]	@ 0x708
 80059cc:	bf18      	it	ne
 80059ce:	f110 0101 	addsne.w	r1, r0, #1
 80059d2:	d11b      	bne.n	8005a0c <ip::__cortex_m_rt_main+0xf70>
 80059d4:	f8c9 08f0 	str.w	r0, [r9, #2288]	@ 0x8f0
 80059d8:	f647 4065 	movw	r0, #31845	@ 0x7c65
 80059dc:	f10d 0e08 	add.w	lr, sp, #8
 80059e0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80059e4:	f50e 51df 	add.w	r1, lr, #7136	@ 0x1be0
 80059e8:	f10d 0e08 	add.w	lr, sp, #8
 80059ec:	f64a 52bc 	movw	r2, #44476	@ 0xadbc
 80059f0:	f8c9 1af0 	str.w	r1, [r9, #2800]	@ 0xaf0
 80059f4:	f8c9 0af4 	str.w	r0, [r9, #2804]	@ 0xaf4
 80059f8:	f649 205b 	movw	r0, #39515	@ 0x9a5b
 80059fc:	f50e 51ef 	add.w	r1, lr, #7648	@ 0x1de0
 8005a00:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005a04:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005a08:	f7fd ff6a 	bl	80038e0 <core::panicking::panic_fmt>
 8005a0c:	f10d 0e10 	add.w	lr, sp, #16
 8005a10:	f24a 71a8 	movw	r1, #42920	@ 0xa7a8
 8005a14:	f50e 58d4 	add.w	r8, lr, #6784	@ 0x1a80
 8005a18:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8005a1c:	f44f 72a8 	mov.w	r2, #336	@ 0x150
 8005a20:	4640      	mov	r0, r8
 8005a22:	f003 fcef 	bl	8009404 <__aeabi_memcpy8>
 8005a26:	f10d 0e08 	add.w	lr, sp, #8
 8005a2a:	f44f 7100 	mov.w	r1, #512	@ 0x200
 8005a2e:	f50e 56df 	add.w	r6, lr, #7136	@ 0x1be0
 8005a32:	f8c9 88e8 	str.w	r8, [r9, #2280]	@ 0x8e8
 8005a36:	f8c9 48ec 	str.w	r4, [r9, #2284]	@ 0x8ec
 8005a3a:	f44f 7b00 	mov.w	fp, #512	@ 0x200
 8005a3e:	4630      	mov	r0, r6
 8005a40:	f003 fc83 	bl	800934a <__aeabi_memclr4>
 8005a44:	f10d 0e08 	add.w	lr, sp, #8
 8005a48:	f44f 7100 	mov.w	r1, #512	@ 0x200
 8005a4c:	f50e 50ef 	add.w	r0, lr, #7648	@ 0x1de0
 8005a50:	f003 fc7b 	bl	800934a <__aeabi_memclr4>
 8005a54:	f8d9 0798 	ldr.w	r0, [r9, #1944]	@ 0x798
 8005a58:	f04f 0a00 	mov.w	sl, #0
 8005a5c:	f8d9 179c 	ldr.w	r1, [r9, #1948]	@ 0x79c
 8005a60:	f080 0003 	eor.w	r0, r0, #3
 8005a64:	e947 aa0e 	strd	sl, sl, [r7, #-56]	@ 0x38
 8005a68:	4308      	orrs	r0, r1
 8005a6a:	e947 aa10 	strd	sl, sl, [r7, #-64]	@ 0x40
 8005a6e:	e947 aa12 	strd	sl, sl, [r7, #-72]	@ 0x48
 8005a72:	e947 aa14 	strd	sl, sl, [r7, #-80]	@ 0x50
 8005a76:	e947 aa0a 	strd	sl, sl, [r7, #-40]	@ 0x28
 8005a7a:	e947 aa08 	strd	sl, sl, [r7, #-32]
 8005a7e:	e947 aa18 	strd	sl, sl, [r7, #-96]	@ 0x60
 8005a82:	e947 aa16 	strd	sl, sl, [r7, #-88]	@ 0x58
 8005a86:	f040 8223 	bne.w	8005ed0 <ip::__cortex_m_rt_main+0x1434>
 8005a8a:	f242 7010 	movw	r0, #10000	@ 0x2710
 8005a8e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005a92:	f44f 7196 	mov.w	r1, #300	@ 0x12c
 8005a96:	f8c9 0800 	str.w	r0, [r9, #2048]	@ 0x800
 8005a9a:	2064      	movs	r0, #100	@ 0x64
 8005a9c:	f8c9 17b8 	str.w	r1, [r9, #1976]	@ 0x7b8
 8005aa0:	f1a7 0130 	sub.w	r1, r7, #48	@ 0x30
 8005aa4:	f88e aab8 	strb.w	sl, [lr, #2744]	@ 0xab8
 8005aa8:	f8c9 47f8 	str.w	r4, [r9, #2040]	@ 0x7f8
 8005aac:	46de      	mov	lr, fp
 8005aae:	f8c9 a7fc 	str.w	sl, [r9, #2044]	@ 0x7fc
 8005ab2:	46b3      	mov	fp, r6
 8005ab4:	f8c9 a804 	str.w	sl, [r9, #2052]	@ 0x804
 8005ab8:	f108 0c78 	add.w	ip, r8, #120	@ 0x78
 8005abc:	f8c9 a808 	str.w	sl, [r9, #2056]	@ 0x808
 8005ac0:	f8c9 a80c 	str.w	sl, [r9, #2060]	@ 0x80c
 8005ac4:	f8c9 a7e8 	str.w	sl, [r9, #2024]	@ 0x7e8
 8005ac8:	f8c9 a7ec 	str.w	sl, [r9, #2028]	@ 0x7ec
 8005acc:	f8c9 a7d8 	str.w	sl, [r9, #2008]	@ 0x7d8
 8005ad0:	f8c9 a7dc 	str.w	sl, [r9, #2012]	@ 0x7dc
 8005ad4:	f8c9 a7c8 	str.w	sl, [r9, #1992]	@ 0x7c8
 8005ad8:	f8c9 a7cc 	str.w	sl, [r9, #1996]	@ 0x7cc
 8005adc:	f8c9 a798 	str.w	sl, [r9, #1944]	@ 0x798
 8005ae0:	f8c9 a79c 	str.w	sl, [r9, #1948]	@ 0x79c
 8005ae4:	f8c9 07bc 	str.w	r0, [r9, #1980]	@ 0x7bc
 8005ae8:	f8c9 a7b0 	str.w	sl, [r9, #1968]	@ 0x7b0
 8005aec:	e891 007d 	ldmia.w	r1, {r0, r2, r3, r4, r5, r6}
 8005af0:	f1a7 0150 	sub.w	r1, r7, #80	@ 0x50
 8005af4:	e88c 007d 	stmia.w	ip, {r0, r2, r3, r4, r5, r6}
 8005af8:	a802      	add	r0, sp, #8
 8005afa:	f500 50ef 	add.w	r0, r0, #7648	@ 0x1de0
 8005afe:	f8c9 e854 	str.w	lr, [r9, #2132]	@ 0x854
 8005b02:	f8c9 e864 	str.w	lr, [r9, #2148]	@ 0x864
 8005b06:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005b0a:	f8c9 0860 	str.w	r0, [r9, #2144]	@ 0x860
 8005b0e:	f108 00d8 	add.w	r0, r8, #216	@ 0xd8
 8005b12:	f88e ab42 	strb.w	sl, [lr, #2882]	@ 0xb42
 8005b16:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005b1a:	f8c9 b850 	str.w	fp, [r9, #2128]	@ 0x850
 8005b1e:	f240 0840 	movw	r8, #64	@ 0x40
 8005b22:	f8ae ab40 	strh.w	sl, [lr, #2880]	@ 0xb40
 8005b26:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005b2a:	f8c9 a858 	str.w	sl, [r9, #2136]	@ 0x858
 8005b2e:	f2c2 0800 	movt	r8, #8192	@ 0x2000
 8005b32:	f8c9 a85c 	str.w	sl, [r9, #2140]	@ 0x85c
 8005b36:	f8c9 a868 	str.w	sl, [r9, #2152]	@ 0x868
 8005b3a:	f8c9 a86c 	str.w	sl, [r9, #2156]	@ 0x86c
 8005b3e:	f8c9 a840 	str.w	sl, [r9, #2112]	@ 0x840
 8005b42:	f8c9 a838 	str.w	sl, [r9, #2104]	@ 0x838
 8005b46:	f8c9 a830 	str.w	sl, [r9, #2096]	@ 0x830
 8005b4a:	c96c      	ldmia	r1!, {r2, r3, r5, r6}
 8005b4c:	c06c      	stmia	r0!, {r2, r3, r5, r6}
 8005b4e:	e891 006c 	ldmia.w	r1, {r2, r3, r5, r6}
 8005b52:	c06c      	stmia	r0!, {r2, r3, r5, r6}
 8005b54:	f1a7 0360 	sub.w	r3, r7, #96	@ 0x60
 8005b58:	2601      	movs	r6, #1
 8005b5a:	f88e abc8 	strb.w	sl, [lr, #3016]	@ 0xbc8
 8005b5e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005b62:	f8c9 68ca 	str.w	r6, [r9, #2250]	@ 0x8ca
 8005b66:	f44f 7606 	mov.w	r6, #536	@ 0x218
 8005b6a:	cb0f      	ldmia	r3, {r0, r1, r2, r3}
 8005b6c:	f8c9 68a8 	str.w	r6, [r9, #2216]	@ 0x8a8
 8005b70:	464e      	mov	r6, r9
 8005b72:	f8c9 a8c6 	str.w	sl, [r9, #2246]	@ 0x8c6
 8005b76:	f8c9 a8c2 	str.w	sl, [r9, #2242]	@ 0x8c2
 8005b7a:	f8c9 a8e0 	str.w	sl, [r9, #2272]	@ 0x8e0
 8005b7e:	f8ae abac 	strh.w	sl, [lr, #2988]	@ 0xbac
 8005b82:	f8c9 a8b0 	str.w	sl, [r9, #2224]	@ 0x8b0
 8005b86:	f8c9 a890 	str.w	sl, [r9, #2192]	@ 0x890
 8005b8a:	f8c9 a894 	str.w	sl, [r9, #2196]	@ 0x894
 8005b8e:	f8c9 0898 	str.w	r0, [r9, #2200]	@ 0x898
 8005b92:	f8c9 189c 	str.w	r1, [r9, #2204]	@ 0x89c
 8005b96:	f8c9 28a0 	str.w	r2, [r9, #2208]	@ 0x8a0
 8005b9a:	f8c9 38a4 	str.w	r3, [r9, #2212]	@ 0x8a4
 8005b9e:	f8c9 a8ac 	str.w	sl, [r9, #2220]	@ 0x8ac
 8005ba2:	e09b      	b.n	8005cdc <ip::__cortex_m_rt_main+0x1240>
 8005ba4:	42800000 	.word	0x42800000
 8005ba8:	2064      	movs	r0, #100	@ 0x64
 8005baa:	f44f 7196 	mov.w	r1, #300	@ 0x12c
 8005bae:	e9c5 1008 	strd	r1, r0, [r5, #32]
 8005bb2:	f105 00d0 	add.w	r0, r5, #208	@ 0xd0
 8005bb6:	2128      	movs	r1, #40	@ 0x28
 8005bb8:	e9c5 aa00 	strd	sl, sl, [r5]
 8005bbc:	f8c5 a018 	str.w	sl, [r5, #24]
 8005bc0:	e9c5 aa20 	strd	sl, sl, [r5, #128]	@ 0x80
 8005bc4:	e9c5 aa22 	strd	sl, sl, [r5, #136]	@ 0x88
 8005bc8:	f885 a130 	strb.w	sl, [r5, #304]	@ 0x130
 8005bcc:	e9c5 aa30 	strd	sl, sl, [r5, #192]	@ 0xc0
 8005bd0:	f885 a028 	strb.w	sl, [r5, #40]	@ 0x28
 8005bd4:	f8d5 40bc 	ldr.w	r4, [r5, #188]	@ 0xbc
 8005bd8:	f8a5 a12e 	strh.w	sl, [r5, #302]	@ 0x12e
 8005bdc:	f8c5 a098 	str.w	sl, [r5, #152]	@ 0x98
 8005be0:	f885 a12c 	strb.w	sl, [r5, #300]	@ 0x12c
 8005be4:	e9c5 aa40 	strd	sl, sl, [r5, #256]	@ 0x100
 8005be8:	e9c5 aa42 	strd	sl, sl, [r5, #264]	@ 0x108
 8005bec:	f002 fe83 	bl	80088f6 <__aeabi_memclr8>
 8005bf0:	2050      	movs	r0, #80	@ 0x50
 8005bf2:	2210      	movs	r2, #16
 8005bf4:	e9c5 0a2c 	strd	r0, sl, [r5, #176]	@ 0xb0
 8005bf8:	f44f 7006 	mov.w	r0, #536	@ 0x218
 8005bfc:	f8c5 0110 	str.w	r0, [r5, #272]	@ 0x110
 8005c00:	fab4 f084 	clz	r0, r4
 8005c04:	f1c0 0110 	rsb	r1, r0, #16
 8005c08:	4282      	cmp	r2, r0
 8005c0a:	f240 002c 	movw	r0, #44	@ 0x2c
 8005c0e:	f04f 0901 	mov.w	r9, #1
 8005c12:	f2c0 0000 	movt	r0, #0
 8005c16:	e9c5 aa3e 	strd	sl, sl, [r5, #248]	@ 0xf8
 8005c1a:	e9c5 aa1c 	strd	sl, sl, [r5, #112]	@ 0x70
 8005c1e:	e9c5 aa14 	strd	sl, sl, [r5, #80]	@ 0x50
 8005c22:	f885 9133 	strb.w	r9, [r5, #307]	@ 0x133
 8005c26:	f8a5 a11c 	strh.w	sl, [r5, #284]	@ 0x11c
 8005c2a:	bf38      	it	cc
 8005c2c:	4651      	movcc	r1, sl
 8005c2e:	f885 1134 	strb.w	r1, [r5, #308]	@ 0x134
 8005c32:	f7fe fc0f 	bl	8004454 <defmt::export::acquire_and_header>
 8005c36:	f240 0007 	movw	r0, #7
 8005c3a:	f1a7 0414 	sub.w	r4, r7, #20
 8005c3e:	f2c0 0000 	movt	r0, #0
 8005c42:	2102      	movs	r1, #2
 8005c44:	f827 0c14 	strh.w	r0, [r7, #-20]
 8005c48:	4620      	mov	r0, r4
 8005c4a:	f7fe fdb5 	bl	80047b8 <_defmt_write>
 8005c4e:	f240 0032 	movw	r0, #50	@ 0x32
 8005c52:	2102      	movs	r1, #2
 8005c54:	f2c0 0000 	movt	r0, #0
 8005c58:	f827 0c14 	strh.w	r0, [r7, #-20]
 8005c5c:	4620      	mov	r0, r4
 8005c5e:	f7fe fdab 	bl	80047b8 <_defmt_write>
 8005c62:	f240 0502 	movw	r5, #2
 8005c66:	4620      	mov	r0, r4
 8005c68:	f2c0 0500 	movt	r5, #0
 8005c6c:	2102      	movs	r1, #2
 8005c6e:	f827 5c14 	strh.w	r5, [r7, #-20]
 8005c72:	f7fe fda1 	bl	80047b8 <_defmt_write>
 8005c76:	200a      	movs	r0, #10
 8005c78:	2101      	movs	r1, #1
 8005c7a:	f807 0c14 	strb.w	r0, [r7, #-20]
 8005c7e:	4620      	mov	r0, r4
 8005c80:	f7fe fd9a 	bl	80047b8 <_defmt_write>
 8005c84:	4620      	mov	r0, r4
 8005c86:	2102      	movs	r1, #2
 8005c88:	f827 5c14 	strh.w	r5, [r7, #-20]
 8005c8c:	f7fe fd94 	bl	80047b8 <_defmt_write>
 8005c90:	4620      	mov	r0, r4
 8005c92:	2101      	movs	r1, #1
 8005c94:	f807 ac14 	strb.w	sl, [r7, #-20]
 8005c98:	f7fe fd8e 	bl	80047b8 <_defmt_write>
 8005c9c:	4620      	mov	r0, r4
 8005c9e:	2102      	movs	r1, #2
 8005ca0:	f827 5c14 	strh.w	r5, [r7, #-20]
 8005ca4:	f7fe fd88 	bl	80047b8 <_defmt_write>
 8005ca8:	4620      	mov	r0, r4
 8005caa:	2101      	movs	r1, #1
 8005cac:	f807 ac14 	strb.w	sl, [r7, #-20]
 8005cb0:	f7fe fd82 	bl	80047b8 <_defmt_write>
 8005cb4:	4620      	mov	r0, r4
 8005cb6:	2102      	movs	r1, #2
 8005cb8:	f827 5c14 	strh.w	r5, [r7, #-20]
 8005cbc:	f7fe fd7c 	bl	80047b8 <_defmt_write>
 8005cc0:	4620      	mov	r0, r4
 8005cc2:	2101      	movs	r1, #1
 8005cc4:	f807 9c14 	strb.w	r9, [r7, #-20]
 8005cc8:	f7fe fd76 	bl	80047b8 <_defmt_write>
 8005ccc:	4620      	mov	r0, r4
 8005cce:	2102      	movs	r1, #2
 8005cd0:	f827 ac14 	strh.w	sl, [r7, #-20]
 8005cd4:	f7fe fd70 	bl	80047b8 <_defmt_write>
 8005cd8:	f7fe fcf6 	bl	80046c8 <_defmt_release>
 8005cdc:	f002 fe06 	bl	80088ec <__primask_r>
 8005ce0:	4604      	mov	r4, r0
 8005ce2:	f002 fdf6 	bl	80088d2 <__cpsid>
 8005ce6:	07e0      	lsls	r0, r4, #31
 8005ce8:	e9d8 5902 	ldrd	r5, r9, [r8, #8]
 8005cec:	bf08      	it	eq
 8005cee:	f002 fdf2 	bleq	80088d6 <__cpsie>
 8005cf2:	f002 fdfb 	bl	80088ec <__primask_r>
 8005cf6:	4604      	mov	r4, r0
 8005cf8:	f002 fdeb 	bl	80088d2 <__cpsid>
 8005cfc:	07e0      	lsls	r0, r4, #31
 8005cfe:	bf08      	it	eq
 8005d00:	f002 fde9 	bleq	80088d6 <__cpsie>
 8005d04:	f44f 717a 	mov.w	r1, #1000	@ 0x3e8
 8005d08:	f10d 0e04 	add.w	lr, sp, #4
 8005d0c:	fba5 b001 	umull	fp, r0, r5, r1
 8005d10:	fb09 0401 	mla	r4, r9, r1, r0
 8005d14:	f50d 50df 	add.w	r0, sp, #7136	@ 0x1be0
 8005d18:	f50e 51c8 	add.w	r1, lr, #6400	@ 0x1900
 8005d1c:	e9cd 1000 	strd	r1, r0, [sp]
 8005d20:	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
 8005d24:	465a      	mov	r2, fp
 8005d26:	4623      	mov	r3, r4
 8005d28:	f7fb f9f8 	bl	800111c <smoltcp::iface::interface::Interface::poll>
 8005d2c:	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
 8005d30:	2900      	cmp	r1, #0
 8005d32:	f000 816f 	beq.w	8006014 <ip::__cortex_m_rt_main+0x1578>
 8005d36:	f8d6 58e8 	ldr.w	r5, [r6, #2280]	@ 0x8e8
 8005d3a:	6828      	ldr	r0, [r5, #0]
 8005d3c:	43c1      	mvns	r1, r0
 8005d3e:	0789      	lsls	r1, r1, #30
 8005d40:	f000 80ff 	beq.w	8005f42 <ip::__cortex_m_rt_main+0x14a6>
 8005d44:	2802      	cmp	r0, #2
 8005d46:	f000 8107 	beq.w	8005f58 <ip::__cortex_m_rt_main+0x14bc>
 8005d4a:	f895 0133 	ldrb.w	r0, [r5, #307]	@ 0x133
 8005d4e:	2800      	cmp	r0, #0
 8005d50:	bf18      	it	ne
 8005d52:	280a      	cmpne	r0, #10
 8005d54:	f43f af28 	beq.w	8005ba8 <ip::__cortex_m_rt_main+0x110c>
 8005d58:	2807      	cmp	r0, #7
 8005d5a:	bf18      	it	ne
 8005d5c:	2804      	cmpne	r0, #4
 8005d5e:	d1bd      	bne.n	8005cdc <ip::__cortex_m_rt_main+0x1240>
 8005d60:	f8d5 60d4 	ldr.w	r6, [r5, #212]	@ 0xd4
 8005d64:	2e00      	cmp	r6, #0
 8005d66:	bf08      	it	eq
 8005d68:	f8c5 a0d0 	streq.w	sl, [r5, #208]	@ 0xd0
 8005d6c:	f8d5 90cc 	ldr.w	r9, [r5, #204]	@ 0xcc
 8005d70:	f1b9 0f00 	cmp.w	r9, #0
 8005d74:	d007      	beq.n	8005d86 <ip::__cortex_m_rt_main+0x12ea>
 8005d76:	f8d5 00d0 	ldr.w	r0, [r5, #208]	@ 0xd0
 8005d7a:	4430      	add	r0, r6
 8005d7c:	fbb0 f1f9 	udiv	r1, r0, r9
 8005d80:	fb01 0019 	mls	r0, r1, r9, r0
 8005d84:	e000      	b.n	8005d88 <ip::__cortex_m_rt_main+0x12ec>
 8005d86:	2000      	movs	r0, #0
 8005d88:	eba9 0806 	sub.w	r8, r9, r6
 8005d8c:	eba9 0100 	sub.w	r1, r9, r0
 8005d90:	4541      	cmp	r1, r8
 8005d92:	bf38      	it	cc
 8005d94:	4688      	movcc	r8, r1
 8005d96:	eb18 0100 	adds.w	r1, r8, r0
 8005d9a:	f080 80e8 	bcs.w	8005f6e <ip::__cortex_m_rt_main+0x14d2>
 8005d9e:	4549      	cmp	r1, r9
 8005da0:	f200 80e5 	bhi.w	8005f6e <ip::__cortex_m_rt_main+0x14d2>
 8005da4:	f8d5 10c8 	ldr.w	r1, [r5, #200]	@ 0xc8
 8005da8:	f1b8 0f06 	cmp.w	r8, #6
 8005dac:	bf28      	it	cs
 8005dae:	f04f 0806 	movcs.w	r8, #6
 8005db2:	9106      	str	r1, [sp, #24]
 8005db4:	4408      	add	r0, r1
 8005db6:	f64a 01f8 	movw	r1, #43256	@ 0xa8f8
 8005dba:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8005dbe:	4642      	mov	r2, r8
 8005dc0:	f003 fb1a 	bl	80093f8 <__aeabi_memcpy>
 8005dc4:	eb18 0206 	adds.w	r2, r8, r6
 8005dc8:	bf08      	it	eq
 8005dca:	f8c5 a0d0 	streq.w	sl, [r5, #208]	@ 0xd0
 8005dce:	f1b9 0f00 	cmp.w	r9, #0
 8005dd2:	9407      	str	r4, [sp, #28]
 8005dd4:	d007      	beq.n	8005de6 <ip::__cortex_m_rt_main+0x134a>
 8005dd6:	f8d5 00d0 	ldr.w	r0, [r5, #208]	@ 0xd0
 8005dda:	4410      	add	r0, r2
 8005ddc:	fbb0 f1f9 	udiv	r1, r0, r9
 8005de0:	fb01 0019 	mls	r0, r1, r9, r0
 8005de4:	e000      	b.n	8005de8 <ip::__cortex_m_rt_main+0x134c>
 8005de6:	2000      	movs	r0, #0
 8005de8:	eba9 0402 	sub.w	r4, r9, r2
 8005dec:	eba9 0100 	sub.w	r1, r9, r0
 8005df0:	42a1      	cmp	r1, r4
 8005df2:	bf38      	it	cc
 8005df4:	460c      	movcc	r4, r1
 8005df6:	1821      	adds	r1, r4, r0
 8005df8:	f080 80b9 	bcs.w	8005f6e <ip::__cortex_m_rt_main+0x14d2>
 8005dfc:	4549      	cmp	r1, r9
 8005dfe:	f200 80b6 	bhi.w	8005f6e <ip::__cortex_m_rt_main+0x14d2>
 8005e02:	f1c8 0106 	rsb	r1, r8, #6
 8005e06:	46b1      	mov	r9, r6
 8005e08:	42a1      	cmp	r1, r4
 8005e0a:	bf38      	it	cc
 8005e0c:	460c      	movcc	r4, r1
 8005e0e:	9906      	ldr	r1, [sp, #24]
 8005e10:	4616      	mov	r6, r2
 8005e12:	4622      	mov	r2, r4
 8005e14:	4408      	add	r0, r1
 8005e16:	f64a 01f8 	movw	r1, #43256	@ 0xa8f8
 8005e1a:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8005e1e:	4441      	add	r1, r8
 8005e20:	f003 faea 	bl	80093f8 <__aeabi_memcpy>
 8005e24:	19a0      	adds	r0, r4, r6
 8005e26:	f1b9 0f00 	cmp.w	r9, #0
 8005e2a:	f8c5 00d4 	str.w	r0, [r5, #212]	@ 0xd4
 8005e2e:	d104      	bne.n	8005e3a <ip::__cortex_m_rt_main+0x139e>
 8005e30:	ea54 0008 	orrs.w	r0, r4, r8
 8005e34:	bf18      	it	ne
 8005e36:	e9c5 aa14 	strdne	sl, sl, [r5, #80]	@ 0x50
 8005e3a:	9807      	ldr	r0, [sp, #28]
 8005e3c:	f51b 747a 	adds.w	r4, fp, #1000	@ 0x3e8
 8005e40:	f10d 0e18 	add.w	lr, sp, #24
 8005e44:	f240 0840 	movw	r8, #64	@ 0x40
 8005e48:	f140 0500 	adc.w	r5, r0, #0
 8005e4c:	f50e 5697 	add.w	r6, lr, #4832	@ 0x12e0
 8005e50:	f50d 59df 	add.w	r9, sp, #7136	@ 0x1be0
 8005e54:	f2c2 0800 	movt	r8, #8192	@ 0x2000
 8005e58:	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
 8005e5c:	2900      	cmp	r1, #0
 8005e5e:	f000 80b9 	beq.w	8005fd4 <ip::__cortex_m_rt_main+0x1538>
 8005e62:	f8d6 08e8 	ldr.w	r0, [r6, #2280]	@ 0x8e8
 8005e66:	6801      	ldr	r1, [r0, #0]
 8005e68:	43ca      	mvns	r2, r1
 8005e6a:	0792      	lsls	r2, r2, #30
 8005e6c:	d045      	beq.n	8005efa <ip::__cortex_m_rt_main+0x145e>
 8005e6e:	2902      	cmp	r1, #2
 8005e70:	d04e      	beq.n	8005f10 <ip::__cortex_m_rt_main+0x1474>
 8005e72:	f8d0 20d4 	ldr.w	r2, [r0, #212]	@ 0xd4
 8005e76:	b182      	cbz	r2, 8005e9a <ip::__cortex_m_rt_main+0x13fe>
 8005e78:	f10d 0e04 	add.w	lr, sp, #4
 8005e7c:	4622      	mov	r2, r4
 8005e7e:	f50e 50c8 	add.w	r0, lr, #6400	@ 0x1900
 8005e82:	e9cd 0900 	strd	r0, r9, [sp]
 8005e86:	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
 8005e8a:	462b      	mov	r3, r5
 8005e8c:	f7fb f946 	bl	800111c <smoltcp::iface::interface::Interface::poll>
 8005e90:	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
 8005e94:	2900      	cmp	r1, #0
 8005e96:	d1e4      	bne.n	8005e62 <ip::__cortex_m_rt_main+0x13c6>
 8005e98:	e09c      	b.n	8005fd4 <ip::__cortex_m_rt_main+0x1538>
 8005e9a:	f001 0103 	and.w	r1, r1, #3
 8005e9e:	2903      	cmp	r1, #3
 8005ea0:	d04f      	beq.n	8005f42 <ip::__cortex_m_rt_main+0x14a6>
 8005ea2:	2902      	cmp	r1, #2
 8005ea4:	d058      	beq.n	8005f58 <ip::__cortex_m_rt_main+0x14bc>
 8005ea6:	f880 a133 	strb.w	sl, [r0, #307]	@ 0x133
 8005eaa:	f240 002d 	movw	r0, #45	@ 0x2d
 8005eae:	f2c0 0000 	movt	r0, #0
 8005eb2:	f7fe fae0 	bl	8004476 <defmt::export::acquire_header_and_release>
 8005eb6:	f10d 0e04 	add.w	lr, sp, #4
 8005eba:	9b07      	ldr	r3, [sp, #28]
 8005ebc:	f50e 50c8 	add.w	r0, lr, #6400	@ 0x1900
 8005ec0:	e9cd 0900 	strd	r0, r9, [sp]
 8005ec4:	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
 8005ec8:	465a      	mov	r2, fp
 8005eca:	f7fb f927 	bl	800111c <smoltcp::iface::interface::Interface::poll>
 8005ece:	e705      	b.n	8005cdc <ip::__cortex_m_rt_main+0x1240>
 8005ed0:	f649 10c8 	movw	r0, #39368	@ 0x99c8
 8005ed4:	f24a 4254 	movw	r2, #42068	@ 0xa454
 8005ed8:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005edc:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005ee0:	2147      	movs	r1, #71	@ 0x47
 8005ee2:	f7fd fcfd 	bl	80038e0 <core::panicking::panic_fmt>
 8005ee6:	f1a0 0360 	sub.w	r3, r0, #96	@ 0x60
 8005eea:	2b60      	cmp	r3, #96	@ 0x60
 8005eec:	d21b      	bcs.n	8005f26 <ip::__cortex_m_rt_main+0x148a>
 8005eee:	ed9f 2a55 	vldr	s4, [pc, #340]	@ 8006044 <ip::__cortex_m_rt_main+0x15a8>
 8005ef2:	f04f 0ed0 	mov.w	lr, #208	@ 0xd0
 8005ef6:	f7fe bec6 	b.w	8004c86 <ip::__cortex_m_rt_main+0x1ea>
 8005efa:	f24a 40a0 	movw	r0, #42144	@ 0xa4a0
 8005efe:	f24a 42c8 	movw	r2, #42184	@ 0xa4c8
 8005f02:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005f06:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005f0a:	214f      	movs	r1, #79	@ 0x4f
 8005f0c:	f7fd fce8 	bl	80038e0 <core::panicking::panic_fmt>
 8005f10:	f24a 4064 	movw	r0, #42084	@ 0xa464
 8005f14:	f24a 42d8 	movw	r2, #42200	@ 0xa4d8
 8005f18:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005f1c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005f20:	2129      	movs	r1, #41	@ 0x29
 8005f22:	f7fe f916 	bl	8004152 <core::option::expect_failed>
 8005f26:	38c0      	subs	r0, #192	@ 0xc0
 8005f28:	a347      	add	r3, pc, #284	@ (adr r3, 8006048 <ip::__cortex_m_rt_main+0x15ac>)
 8005f2a:	f04f 0ef0 	mov.w	lr, #240	@ 0xf0
 8005f2e:	28c0      	cmp	r0, #192	@ 0xc0
 8005f30:	bf38      	it	cc
 8005f32:	3304      	addcc	r3, #4
 8005f34:	ed93 2a00 	vldr	s4, [r3]
 8005f38:	bf38      	it	cc
 8005f3a:	f04f 0ee0 	movcc.w	lr, #224	@ 0xe0
 8005f3e:	f7fe bea2 	b.w	8004c86 <ip::__cortex_m_rt_main+0x1ea>
 8005f42:	f24a 40a0 	movw	r0, #42144	@ 0xa4a0
 8005f46:	f24a 42f8 	movw	r2, #42232	@ 0xa4f8
 8005f4a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005f4e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005f52:	214f      	movs	r1, #79	@ 0x4f
 8005f54:	f7fd fcc4 	bl	80038e0 <core::panicking::panic_fmt>
 8005f58:	f24a 4064 	movw	r0, #42084	@ 0xa464
 8005f5c:	f24a 5208 	movw	r2, #42248	@ 0xa508
 8005f60:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005f64:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005f68:	2129      	movs	r1, #41	@ 0x29
 8005f6a:	f7fe f8f2 	bl	8004152 <core::option::expect_failed>
 8005f6e:	f64a 7304 	movw	r3, #44804	@ 0xaf04
 8005f72:	464a      	mov	r2, r9
 8005f74:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8005f78:	f7fd fc41 	bl	80037fe <core::slice::index::slice_index_fail>
 8005f7c:	f24b 10e8 	movw	r0, #45544	@ 0xb1e8
 8005f80:	f24b 2210 	movw	r2, #45584	@ 0xb210
 8005f84:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005f88:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005f8c:	2127      	movs	r1, #39	@ 0x27
 8005f8e:	f7fd fcc9 	bl	8003924 <core::panicking::panic>
 8005f92:	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
 8005f96:	f24b 2220 	movw	r2, #45600	@ 0xb220
 8005f9a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005f9e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005fa2:	2128      	movs	r1, #40	@ 0x28
 8005fa4:	f7fd fcbe 	bl	8003924 <core::panicking::panic>
 8005fa8:	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
 8005fac:	f24b 2230 	movw	r2, #45616	@ 0xb230
 8005fb0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005fb4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005fb8:	2128      	movs	r1, #40	@ 0x28
 8005fba:	f7fd fcb3 	bl	8003924 <core::panicking::panic>
 8005fbe:	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
 8005fc2:	f24b 2240 	movw	r2, #45632	@ 0xb240
 8005fc6:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005fca:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005fce:	2128      	movs	r1, #40	@ 0x28
 8005fd0:	f7fd fca8 	bl	8003924 <core::panicking::panic>
 8005fd4:	f24a 4290 	movw	r2, #42128	@ 0xa490
 8005fd8:	2000      	movs	r0, #0
 8005fda:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005fde:	f7fd fc8d 	bl	80038fc <core::panicking::panic_bounds_check>
 8005fe2:	f10d 0e08 	add.w	lr, sp, #8
 8005fe6:	f244 0017 	movw	r0, #16407	@ 0x4017
 8005fea:	f50e 51ef 	add.w	r1, lr, #7648	@ 0x1de0
 8005fee:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005ff2:	900b      	str	r0, [sp, #44]	@ 0x2c
 8005ff4:	f24b 12c8 	movw	r2, #45512	@ 0xb1c8
 8005ff8:	e9cd 0109 	strd	r0, r1, [sp, #36]	@ 0x24
 8005ffc:	f50d 604a 	add.w	r0, sp, #3232	@ 0xca0
 8006000:	9008      	str	r0, [sp, #32]
 8006002:	f24a 4009 	movw	r0, #41993	@ 0xa409
 8006006:	a908      	add	r1, sp, #32
 8006008:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800600c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8006010:	f7fd fc66 	bl	80038e0 <core::panicking::panic_fmt>
 8006014:	f24a 42e8 	movw	r2, #42216	@ 0xa4e8
 8006018:	2000      	movs	r0, #0
 800601a:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800601e:	f7fd fc6d 	bl	80038fc <core::panicking::panic_bounds_check>
 8006022:	f002 fc58 	bl	80088d6 <__cpsie>
 8006026:	f24a 7084 	movw	r0, #42884	@ 0xa784
 800602a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800602e:	f7fd fbdc 	bl	80037ea <core::option::unwrap_failed>
 8006032:	f002 fc50 	bl	80088d6 <__cpsie>
 8006036:	f24a 7094 	movw	r0, #42900	@ 0xa794
 800603a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800603e:	f7fd fbd4 	bl	80037ea <core::option::unwrap_failed>
 8006042:	bf00      	nop
 8006044:	43000000 	.word	0x43000000
 8006048:	44000000 	.word	0x44000000
 800604c:	43800000 	.word	0x43800000

08006050 <<stm32_eth::mac::WrongClock as core::fmt::Debug>::fmt>:
 8006050:	b580      	push	{r7, lr}
 8006052:	466f      	mov	r7, sp
 8006054:	e9d1 0200 	ldrd	r0, r2, [r1]
 8006058:	f64a 114c 	movw	r1, #43340	@ 0xa94c
 800605c:	68d3      	ldr	r3, [r2, #12]
 800605e:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8006062:	220a      	movs	r2, #10
 8006064:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8006068:	4718      	bx	r3

0800606a <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>:
 800606a:	b5b0      	push	{r4, r5, r7, lr}
 800606c:	af02      	add	r7, sp, #8
 800606e:	6802      	ldr	r2, [r0, #0]
 8006070:	2a02      	cmp	r2, #2
 8006072:	bf08      	it	eq
 8006074:	bdb0      	popeq	{r4, r5, r7, pc}
 8006076:	e9d0 3c02 	ldrd	r3, ip, [r0, #8]
 800607a:	6859      	ldr	r1, [r3, #4]
 800607c:	458c      	cmp	ip, r1
 800607e:	d230      	bcs.n	80060e2 <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>+0x78>
 8006080:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 8006084:	681b      	ldr	r3, [r3, #0]
 8006086:	fb0c 3101 	mla	r1, ip, r1, r3
 800608a:	f8d0 e010 	ldr.w	lr, [r0, #16]
 800608e:	2500      	movs	r5, #0
 8006090:	f2cf 05d0 	movt	r5, #61648	@ 0xf0d0
 8006094:	07d2      	lsls	r2, r2, #31
 8006096:	d007      	beq.n	80060a8 <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>+0x3e>
 8006098:	6840      	ldr	r0, [r0, #4]
 800609a:	f04f 0c01 	mov.w	ip, #1
 800609e:	f105 7500 	add.w	r5, r5, #33554432	@ 0x2000000
 80060a2:	e9c1 c008 	strd	ip, r0, [r1, #32]
 80060a6:	e001      	b.n	80060ac <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>+0x42>
 80060a8:	2000      	movs	r0, #0
 80060aa:	6208      	str	r0, [r1, #32]
 80060ac:	6848      	ldr	r0, [r1, #4]
 80060ae:	f640 74ff 	movw	r4, #4095	@ 0xfff
 80060b2:	e9d1 320a 	ldrd	r3, r2, [r1, #40]	@ 0x28
 80060b6:	43a0      	bics	r0, r4
 80060b8:	ea40 000e 	orr.w	r0, r0, lr
 80060bc:	6048      	str	r0, [r1, #4]
 80060be:	608b      	str	r3, [r1, #8]
 80060c0:	60ca      	str	r2, [r1, #12]
 80060c2:	f3bf 8f5f 	dmb	sy
 80060c6:	f891 0030 	ldrb.w	r0, [r1, #48]	@ 0x30
 80060ca:	ea45 5040 	orr.w	r0, r5, r0, lsl #21
 80060ce:	6008      	str	r0, [r1, #0]
 80060d0:	f249 0004 	movw	r0, #36868	@ 0x9004
 80060d4:	2100      	movs	r1, #0
 80060d6:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 80060da:	f3bf 8f5f 	dmb	sy
 80060de:	6001      	str	r1, [r0, #0]
 80060e0:	bdb0      	pop	{r4, r5, r7, pc}
 80060e2:	f24b 0258 	movw	r2, #45144	@ 0xb058
 80060e6:	4660      	mov	r0, ip
 80060e8:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80060ec:	f7fd fc06 	bl	80038fc <core::panicking::panic_bounds_check>

080060f0 <__rustc::rust_begin_unwind>:
 80060f0:	b580      	push	{r7, lr}
 80060f2:	466f      	mov	r7, sp
 80060f4:	b086      	sub	sp, #24
 80060f6:	4604      	mov	r4, r0
 80060f8:	f002 fbeb 	bl	80088d2 <__cpsid>
 80060fc:	f240 0038 	movw	r0, #56	@ 0x38
 8006100:	f2c2 0000 	movt	r0, #8192	@ 0x2000
 8006104:	7801      	ldrb	r1, [r0, #0]
 8006106:	bba9      	cbnz	r1, 8006174 <__rustc::rust_begin_unwind+0x84>
 8006108:	2101      	movs	r1, #1
 800610a:	7001      	strb	r1, [r0, #0]
 800610c:	f240 002e 	movw	r0, #46	@ 0x2e
 8006110:	f2c0 0000 	movt	r0, #0
 8006114:	9400      	str	r4, [sp, #0]
 8006116:	f7fe f99d 	bl	8004454 <defmt::export::acquire_and_header>
 800611a:	f240 0001 	movw	r0, #1
 800611e:	2102      	movs	r1, #2
 8006120:	f2c0 0000 	movt	r0, #0
 8006124:	f8ad 000c 	strh.w	r0, [sp, #12]
 8006128:	a803      	add	r0, sp, #12
 800612a:	f7fe fb45 	bl	80047b8 <_defmt_write>
 800612e:	f64a 201c 	movw	r0, #43548	@ 0xaa1c
 8006132:	f24a 712c 	movw	r1, #42796	@ 0xa72c
 8006136:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800613a:	f649 52d9 	movw	r2, #40409	@ 0x9dd9
 800613e:	9002      	str	r0, [sp, #8]
 8006140:	4668      	mov	r0, sp
 8006142:	9001      	str	r0, [sp, #4]
 8006144:	f643 70ab 	movw	r0, #16299	@ 0x3fab
 8006148:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800614c:	ab03      	add	r3, sp, #12
 800614e:	9004      	str	r0, [sp, #16]
 8006150:	a801      	add	r0, sp, #4
 8006152:	9003      	str	r0, [sp, #12]
 8006154:	1e78      	subs	r0, r7, #1
 8006156:	f6c0 0100 	movt	r1, #2048	@ 0x800
 800615a:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800615e:	f7fd f9cf 	bl	8003500 <core::fmt::write>
 8006162:	f24a 7044 	movw	r0, #42820	@ 0xa744
 8006166:	2101      	movs	r1, #1
 8006168:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800616c:	f7fe fb24 	bl	80047b8 <_defmt_write>
 8006170:	f7fe faaa 	bl	80046c8 <_defmt_release>
 8006174:	f000 f800 	bl	8006178 <panic_probe::hard_fault>

08006178 <panic_probe::hard_fault>:
 8006178:	b580      	push	{r7, lr}
 800617a:	466f      	mov	r7, sp
 800617c:	f64e 5024 	movw	r0, #60708	@ 0xed24
 8006180:	f2ce 0000 	movt	r0, #57344	@ 0xe000
 8006184:	6801      	ldr	r1, [r0, #0]
 8006186:	f421 2180 	bic.w	r1, r1, #262144	@ 0x40000
 800618a:	6001      	str	r1, [r0, #0]
 800618c:	f002 fbb1 	bl	80088f2 <__udf>

08006190 <<&T as core::fmt::Display>::fmt>:
 8006190:	b5f0      	push	{r4, r5, r6, r7, lr}
 8006192:	af03      	add	r7, sp, #12
 8006194:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 8006198:	b088      	sub	sp, #32
 800619a:	e9d1 5400 	ldrd	r5, r4, [r1]
 800619e:	f24a 711c 	movw	r1, #42780	@ 0xa71c
 80061a2:	6800      	ldr	r0, [r0, #0]
 80061a4:	f6c0 0100 	movt	r1, #2048	@ 0x800
 80061a8:	f8d4 900c 	ldr.w	r9, [r4, #12]
 80061ac:	220c      	movs	r2, #12
 80061ae:	e9d0 8600 	ldrd	r8, r6, [r0]
 80061b2:	4628      	mov	r0, r5
 80061b4:	47c8      	blx	r9
 80061b6:	b120      	cbz	r0, 80061c2 <<&T as core::fmt::Display>::fmt+0x32>
 80061b8:	2001      	movs	r0, #1
 80061ba:	b008      	add	sp, #32
 80061bc:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80061c0:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80061c2:	e9d6 0100 	ldrd	r0, r1, [r6]
 80061c6:	f24a 4227 	movw	r2, #42023	@ 0xa427
 80061ca:	e9cd 0100 	strd	r0, r1, [sp]
 80061ce:	f244 0017 	movw	r0, #16407	@ 0x4017
 80061d2:	f106 010c 	add.w	r1, r6, #12
 80061d6:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80061da:	9007      	str	r0, [sp, #28]
 80061dc:	ab02      	add	r3, sp, #8
 80061de:	e9cd 0105 	strd	r0, r1, [sp, #20]
 80061e2:	f106 0008 	add.w	r0, r6, #8
 80061e6:	9004      	str	r0, [sp, #16]
 80061e8:	f643 1031 	movw	r0, #14641	@ 0x3931
 80061ec:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80061f0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80061f4:	9003      	str	r0, [sp, #12]
 80061f6:	4668      	mov	r0, sp
 80061f8:	9002      	str	r0, [sp, #8]
 80061fa:	4628      	mov	r0, r5
 80061fc:	4621      	mov	r1, r4
 80061fe:	f7fd f97f 	bl	8003500 <core::fmt::write>
 8006202:	b120      	cbz	r0, 800620e <<&T as core::fmt::Display>::fmt+0x7e>
 8006204:	2001      	movs	r0, #1
 8006206:	b008      	add	sp, #32
 8006208:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 800620c:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800620e:	f24a 7128 	movw	r1, #42792	@ 0xa728
 8006212:	4628      	mov	r0, r5
 8006214:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8006218:	2202      	movs	r2, #2
 800621a:	47c8      	blx	r9
 800621c:	b120      	cbz	r0, 8006228 <<&T as core::fmt::Display>::fmt+0x98>
 800621e:	2001      	movs	r0, #1
 8006220:	b008      	add	sp, #32
 8006222:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8006226:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8006228:	e9d8 2300 	ldrd	r2, r3, [r8]
 800622c:	4628      	mov	r0, r5
 800622e:	4621      	mov	r1, r4
 8006230:	f7fd f966 	bl	8003500 <core::fmt::write>
 8006234:	b008      	add	sp, #32
 8006236:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 800623a:	bdf0      	pop	{r4, r5, r6, r7, pc}

0800623c <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold>:
 800623c:	b5b0      	push	{r4, r5, r7, lr}
 800623e:	af02      	add	r7, sp, #8
 8006240:	4288      	cmp	r0, r1
 8006242:	d02c      	beq.n	800629e <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x62>
 8006244:	f64a 23ab 	movw	r3, #43691	@ 0xaaab
 8006248:	1a09      	subs	r1, r1, r0
 800624a:	f6ca 23aa 	movt	r3, #43690	@ 0xaaaa
 800624e:	6815      	ldr	r5, [r2, #0]
 8006250:	fba1 1303 	umull	r1, r3, r1, r3
 8006254:	4611      	mov	r1, r2
 8006256:	688c      	ldr	r4, [r1, #8]
 8006258:	6852      	ldr	r2, [r2, #4]
 800625a:	f105 0e04 	add.w	lr, r5, #4
 800625e:	08d9      	lsrs	r1, r3, #3
 8006260:	1d03      	adds	r3, r0, #4
 8006262:	00e0      	lsls	r0, r4, #3
 8006264:	e007      	b.n	8006276 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x3a>
 8006266:	2400      	movs	r4, #0
 8006268:	f843 4c04 	str.w	r4, [r3, #-4]
 800626c:	3008      	adds	r0, #8
 800626e:	e8e3 5c03 	strd	r5, ip, [r3], #12
 8006272:	3901      	subs	r1, #1
 8006274:	d013      	beq.n	800629e <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x62>
 8006276:	4290      	cmp	r0, r2
 8006278:	d2f5      	bcs.n	8006266 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x2a>
 800627a:	f100 0c04 	add.w	ip, r0, #4
 800627e:	4594      	cmp	ip, r2
 8006280:	d817      	bhi.n	80062b2 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x76>
 8006282:	1dc4      	adds	r4, r0, #7
 8006284:	4294      	cmp	r4, r2
 8006286:	d20b      	bcs.n	80062a0 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x64>
 8006288:	eb0e 0400 	add.w	r4, lr, r0
 800628c:	f85e 5000 	ldr.w	r5, [lr, r0]
 8006290:	f854 4c04 	ldr.w	r4, [r4, #-4]
 8006294:	fa95 fc85 	rev.w	ip, r5
 8006298:	ba25      	rev	r5, r4
 800629a:	2401      	movs	r4, #1
 800629c:	e7e4      	b.n	8006268 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x2c>
 800629e:	bdb0      	pop	{r4, r5, r7, pc}
 80062a0:	f64a 4358 	movw	r3, #44120	@ 0xac58
 80062a4:	f100 0108 	add.w	r1, r0, #8
 80062a8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80062ac:	4660      	mov	r0, ip
 80062ae:	f7fd faa6 	bl	80037fe <core::slice::index::slice_index_fail>
 80062b2:	f64a 4368 	movw	r3, #44136	@ 0xac68
 80062b6:	4661      	mov	r1, ip
 80062b8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80062bc:	f7fd fa9f 	bl	80037fe <core::slice::index::slice_index_fail>

080062c0 <smoltcp::wire::tcp::TcpOption::emit>:
 80062c0:	b5f0      	push	{r4, r5, r6, r7, lr}
 80062c2:	af03      	add	r7, sp, #12
 80062c4:	f84d bd04 	str.w	fp, [sp, #-4]!
 80062c8:	6803      	ldr	r3, [r0, #0]
 80062ca:	2405      	movs	r4, #5
 80062cc:	2504      	movs	r5, #4
 80062ce:	2b01      	cmp	r3, #1
 80062d0:	bf88      	it	hi
 80062d2:	1e9c      	subhi	r4, r3, #2
 80062d4:	e8df f004 	tbb	[pc, r4]
 80062d8:	13251904 	.word	0x13251904
 80062dc:	17391f0f 	.word	0x17391f0f
 80062e0:	b13a      	cbz	r2, 80062f2 <smoltcp::wire::tcp::TcpOption::emit+0x32>
 80062e2:	4608      	mov	r0, r1
 80062e4:	460c      	mov	r4, r1
 80062e6:	4611      	mov	r1, r2
 80062e8:	4615      	mov	r5, r2
 80062ea:	f003 f8d9 	bl	80094a0 <__aeabi_memclr>
 80062ee:	4621      	mov	r1, r4
 80062f0:	462a      	mov	r2, r5
 80062f2:	2501      	movs	r5, #1
 80062f4:	e07d      	b.n	80063f2 <smoltcp::wire::tcp::TcpOption::emit+0x132>
 80062f6:	2502      	movs	r5, #2
 80062f8:	2a01      	cmp	r2, #1
 80062fa:	d814      	bhi.n	8006326 <smoltcp::wire::tcp::TcpOption::emit+0x66>
 80062fc:	e028      	b.n	8006350 <smoltcp::wire::tcp::TcpOption::emit+0x90>
 80062fe:	2503      	movs	r5, #3
 8006300:	2a01      	cmp	r2, #1
 8006302:	d810      	bhi.n	8006326 <smoltcp::wire::tcp::TcpOption::emit+0x66>
 8006304:	e024      	b.n	8006350 <smoltcp::wire::tcp::TcpOption::emit+0x90>
 8006306:	6886      	ldr	r6, [r0, #8]
 8006308:	e00a      	b.n	8006320 <smoltcp::wire::tcp::TcpOption::emit+0x60>
 800630a:	2a00      	cmp	r2, #0
 800630c:	f000 80da 	beq.w	80064c4 <smoltcp::wire::tcp::TcpOption::emit+0x204>
 8006310:	2501      	movs	r5, #1
 8006312:	700d      	strb	r5, [r1, #0]
 8006314:	e06d      	b.n	80063f2 <smoltcp::wire::tcp::TcpOption::emit+0x132>
 8006316:	68c6      	ldr	r6, [r0, #12]
 8006318:	6985      	ldr	r5, [r0, #24]
 800631a:	441e      	add	r6, r3
 800631c:	442e      	add	r6, r5
 800631e:	00f6      	lsls	r6, r6, #3
 8006320:	1cb5      	adds	r5, r6, #2
 8006322:	2a01      	cmp	r2, #1
 8006324:	d914      	bls.n	8006350 <smoltcp::wire::tcp::TcpOption::emit+0x90>
 8006326:	3c02      	subs	r4, #2
 8006328:	704d      	strb	r5, [r1, #1]
 800632a:	e8df f004 	tbb	[pc, r4]
 800632e:	4a03      	.short	0x4a03
 8006330:	521a312e 	.word	0x521a312e
 8006334:	2302      	movs	r3, #2
 8006336:	700b      	strb	r3, [r1, #0]
 8006338:	f002 033e 	and.w	r3, r2, #62	@ 0x3e
 800633c:	2b02      	cmp	r3, #2
 800633e:	f000 80a7 	beq.w	8006490 <smoltcp::wire::tcp::TcpOption::emit+0x1d0>
 8006342:	8880      	ldrh	r0, [r0, #4]
 8006344:	ba40      	rev16	r0, r0
 8006346:	8048      	strh	r0, [r1, #2]
 8006348:	e053      	b.n	80063f2 <smoltcp::wire::tcp::TcpOption::emit+0x132>
 800634a:	250a      	movs	r5, #10
 800634c:	2a01      	cmp	r2, #1
 800634e:	d8ea      	bhi.n	8006326 <smoltcp::wire::tcp::TcpOption::emit+0x66>
 8006350:	f64a 33d8 	movw	r3, #43992	@ 0xabd8
 8006354:	2001      	movs	r0, #1
 8006356:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800635a:	4611      	mov	r1, r2
 800635c:	461a      	mov	r2, r3
 800635e:	f7fd facd 	bl	80038fc <core::panicking::panic_bounds_check>
 8006362:	2608      	movs	r6, #8
 8006364:	e9d0 3001 	ldrd	r3, r0, [r0, #4]
 8006368:	700e      	strb	r6, [r1, #0]
 800636a:	1f96      	subs	r6, r2, #6
 800636c:	f116 0f04 	cmn.w	r6, #4
 8006370:	bf3f      	itttt	cc
 8006372:	ba1b      	revcc	r3, r3
 8006374:	f8c1 3002 	strcc.w	r3, [r1, #2]
 8006378:	f1a2 030a 	subcc.w	r3, r2, #10
 800637c:	f113 0f04 	cmncc.w	r3, #4
 8006380:	d278      	bcs.n	8006474 <smoltcp::wire::tcp::TcpOption::emit+0x1b4>
 8006382:	ba00      	rev	r0, r0
 8006384:	f8c1 0006 	str.w	r0, [r1, #6]
 8006388:	e033      	b.n	80063f2 <smoltcp::wire::tcp::TcpOption::emit+0x132>
 800638a:	2004      	movs	r0, #4
 800638c:	7008      	strb	r0, [r1, #0]
 800638e:	e030      	b.n	80063f2 <smoltcp::wire::tcp::TcpOption::emit+0x132>
 8006390:	2605      	movs	r6, #5
 8006392:	700e      	strb	r6, [r1, #0]
 8006394:	b3e3      	cbz	r3, 8006410 <smoltcp::wire::tcp::TcpOption::emit+0x150>
 8006396:	1f93      	subs	r3, r2, #6
 8006398:	f113 0f04 	cmn.w	r3, #4
 800639c:	bf3f      	itttt	cc
 800639e:	e9d0 6301 	ldrdcc	r6, r3, [r0, #4]
 80063a2:	ba36      	revcc	r6, r6
 80063a4:	f8c1 6002 	strcc.w	r6, [r1, #2]
 80063a8:	f1a2 060a 	subcc.w	r6, r2, #10
 80063ac:	bf38      	it	cc
 80063ae:	f116 0f04 	cmncc.w	r6, #4
 80063b2:	d25f      	bcs.n	8006474 <smoltcp::wire::tcp::TcpOption::emit+0x1b4>
 80063b4:	ba1b      	rev	r3, r3
 80063b6:	f8c1 3006 	str.w	r3, [r1, #6]
 80063ba:	2301      	movs	r3, #1
 80063bc:	68c6      	ldr	r6, [r0, #12]
 80063be:	bb56      	cbnz	r6, 8006416 <smoltcp::wire::tcp::TcpOption::emit+0x156>
 80063c0:	e041      	b.n	8006446 <smoltcp::wire::tcp::TcpOption::emit+0x186>
 80063c2:	7900      	ldrb	r0, [r0, #4]
 80063c4:	2303      	movs	r3, #3
 80063c6:	2a02      	cmp	r2, #2
 80063c8:	700b      	strb	r3, [r1, #0]
 80063ca:	f000 8083 	beq.w	80064d4 <smoltcp::wire::tcp::TcpOption::emit+0x214>
 80063ce:	7088      	strb	r0, [r1, #2]
 80063d0:	e00f      	b.n	80063f2 <smoltcp::wire::tcp::TcpOption::emit+0x132>
 80063d2:	e9d0 c301 	ldrd	ip, r3, [r0, #4]
 80063d6:	7b00      	ldrb	r0, [r0, #12]
 80063d8:	7008      	strb	r0, [r1, #0]
 80063da:	1e90      	subs	r0, r2, #2
 80063dc:	4298      	cmp	r0, r3
 80063de:	d162      	bne.n	80064a6 <smoltcp::wire::tcp::TcpOption::emit+0x1e6>
 80063e0:	1c88      	adds	r0, r1, #2
 80063e2:	4614      	mov	r4, r2
 80063e4:	460e      	mov	r6, r1
 80063e6:	4661      	mov	r1, ip
 80063e8:	461a      	mov	r2, r3
 80063ea:	f003 f805 	bl	80093f8 <__aeabi_memcpy>
 80063ee:	4631      	mov	r1, r6
 80063f0:	4622      	mov	r2, r4
 80063f2:	42aa      	cmp	r2, r5
 80063f4:	bf21      	itttt	cs
 80063f6:	1948      	addcs	r0, r1, r5
 80063f8:	1b51      	subcs	r1, r2, r5
 80063fa:	f85d bb04 	ldrcs.w	fp, [sp], #4
 80063fe:	bdf0      	popcs	{r4, r5, r6, r7, pc}
 8006400:	f64a 4308 	movw	r3, #44040	@ 0xac08
 8006404:	4628      	mov	r0, r5
 8006406:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800640a:	4611      	mov	r1, r2
 800640c:	f7fd f9f7 	bl	80037fe <core::slice::index::slice_index_fail>
 8006410:	2300      	movs	r3, #0
 8006412:	68c6      	ldr	r6, [r0, #12]
 8006414:	b1be      	cbz	r6, 8006446 <smoltcp::wire::tcp::TcpOption::emit+0x186>
 8006416:	00dc      	lsls	r4, r3, #3
 8006418:	f104 0c02 	add.w	ip, r4, #2
 800641c:	4562      	cmp	r2, ip
 800641e:	d349      	bcc.n	80064b4 <smoltcp::wire::tcp::TcpOption::emit+0x1f4>
 8006420:	eba2 060c 	sub.w	r6, r2, ip
 8006424:	2e04      	cmp	r6, #4
 8006426:	bf21      	itttt	cs
 8006428:	e9d0 6e04 	ldrdcs	r6, lr, [r0, #16]
 800642c:	ba36      	revcs	r6, r6
 800642e:	f841 600c 	strcs.w	r6, [r1, ip]
 8006432:	f044 0406 	orrcs.w	r4, r4, #6
 8006436:	bf24      	itt	cs
 8006438:	1b16      	subcs	r6, r2, r4
 800643a:	2e04      	cmpcs	r6, #4
 800643c:	d31a      	bcc.n	8006474 <smoltcp::wire::tcp::TcpOption::emit+0x1b4>
 800643e:	3301      	adds	r3, #1
 8006440:	fa9e f68e 	rev.w	r6, lr
 8006444:	510e      	str	r6, [r1, r4]
 8006446:	6986      	ldr	r6, [r0, #24]
 8006448:	2e00      	cmp	r6, #0
 800644a:	d0d2      	beq.n	80063f2 <smoltcp::wire::tcp::TcpOption::emit+0x132>
 800644c:	00db      	lsls	r3, r3, #3
 800644e:	f103 0c02 	add.w	ip, r3, #2
 8006452:	4562      	cmp	r2, ip
 8006454:	d32e      	bcc.n	80064b4 <smoltcp::wire::tcp::TcpOption::emit+0x1f4>
 8006456:	eba2 060c 	sub.w	r6, r2, ip
 800645a:	2e04      	cmp	r6, #4
 800645c:	bf21      	itttt	cs
 800645e:	e9d0 6007 	ldrdcs	r6, r0, [r0, #28]
 8006462:	ba36      	revcs	r6, r6
 8006464:	f841 600c 	strcs.w	r6, [r1, ip]
 8006468:	f043 0306 	orrcs.w	r3, r3, #6
 800646c:	bf24      	itt	cs
 800646e:	1ad6      	subcs	r6, r2, r3
 8006470:	2e04      	cmpcs	r6, #4
 8006472:	d20a      	bcs.n	800648a <smoltcp::wire::tcp::TcpOption::emit+0x1ca>
 8006474:	f64a 203c 	movw	r0, #43580	@ 0xaa3c
 8006478:	f64a 228c 	movw	r2, #43660	@ 0xaa8c
 800647c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8006480:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8006484:	2120      	movs	r1, #32
 8006486:	f7fd fa4d 	bl	8003924 <core::panicking::panic>
 800648a:	ba00      	rev	r0, r0
 800648c:	50c8      	str	r0, [r1, r3]
 800648e:	e7b0      	b.n	80063f2 <smoltcp::wire::tcp::TcpOption::emit+0x132>
 8006490:	f64a 205c 	movw	r0, #43612	@ 0xaa5c
 8006494:	f64a 227c 	movw	r2, #43644	@ 0xaa7c
 8006498:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800649c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80064a0:	2120      	movs	r1, #32
 80064a2:	f7fd fa3f 	bl	8003924 <core::panicking::panic>
 80064a6:	f64a 32f8 	movw	r2, #44024	@ 0xabf8
 80064aa:	4619      	mov	r1, r3
 80064ac:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80064b0:	f7fd fe32 	bl	8004118 <core::slice::copy_from_slice_impl::len_mismatch_fail>
 80064b4:	f64a 4318 	movw	r3, #44056	@ 0xac18
 80064b8:	4660      	mov	r0, ip
 80064ba:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80064be:	4611      	mov	r1, r2
 80064c0:	f7fd f99d 	bl	80037fe <core::slice::index::slice_index_fail>
 80064c4:	f64a 32c8 	movw	r2, #43976	@ 0xabc8
 80064c8:	2000      	movs	r0, #0
 80064ca:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80064ce:	2100      	movs	r1, #0
 80064d0:	f7fd fa14 	bl	80038fc <core::panicking::panic_bounds_check>
 80064d4:	f64a 32e8 	movw	r2, #44008	@ 0xabe8
 80064d8:	2002      	movs	r0, #2
 80064da:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80064de:	2102      	movs	r1, #2
 80064e0:	f7fd fa0c 	bl	80038fc <core::panicking::panic_bounds_check>

080064e4 <smoltcp::socket::tcp::Socket::seq_to_transmit>:
 80064e4:	b5b0      	push	{r4, r5, r7, lr}
 80064e6:	af02      	add	r7, sp, #8
 80064e8:	f8b0 211c 	ldrh.w	r2, [r0, #284]	@ 0x11c
 80064ec:	07d2      	lsls	r2, r2, #31
 80064ee:	d053      	beq.n	8006598 <smoltcp::socket::tcp::Socket::seq_to_transmit+0xb4>
 80064f0:	f8d0 3110 	ldr.w	r3, [r0, #272]	@ 0x110
 80064f4:	3936      	subs	r1, #54	@ 0x36
 80064f6:	f8d0 5100 	ldr.w	r5, [r0, #256]	@ 0x100
 80064fa:	f8d0 e108 	ldr.w	lr, [r0, #264]	@ 0x108
 80064fe:	428b      	cmp	r3, r1
 8006500:	f890 2133 	ldrb.w	r2, [r0, #307]	@ 0x133
 8006504:	bf38      	it	cc
 8006506:	4619      	movcc	r1, r3
 8006508:	45ae      	cmp	lr, r5
 800650a:	bf04      	itt	eq
 800650c:	f002 030e 	andeq.w	r3, r2, #14
 8006510:	2b02      	cmpeq	r3, #2
 8006512:	d024      	beq.n	800655e <smoltcp::socket::tcp::Socket::seq_to_transmit+0x7a>
 8006514:	f8d0 40d4 	ldr.w	r4, [r0, #212]	@ 0xd4
 8006518:	f8d0 310c 	ldr.w	r3, [r0, #268]	@ 0x10c
 800651c:	429c      	cmp	r4, r3
 800651e:	bf38      	it	cc
 8006520:	4623      	movcc	r3, r4
 8006522:	f1b3 3fff 	cmp.w	r3, #4294967295	@ 0xffffffff
 8006526:	dd2c      	ble.n	8006582 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x9e>
 8006528:	442b      	add	r3, r5
 800652a:	2a09      	cmp	r2, #9
 800652c:	eba3 0c0e 	sub.w	ip, r3, lr
 8006530:	d817      	bhi.n	8006562 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x7e>
 8006532:	2301      	movs	r3, #1
 8006534:	fa03 f202 	lsl.w	r2, r3, r2
 8006538:	f412 7f48 	tst.w	r2, #800	@ 0x320
 800653c:	d011      	beq.n	8006562 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x7e>
 800653e:	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
 8006542:	dd1e      	ble.n	8006582 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x9e>
 8006544:	1960      	adds	r0, r4, r5
 8006546:	ebae 0000 	sub.w	r0, lr, r0
 800654a:	fab0 f080 	clz	r0, r0
 800654e:	0940      	lsrs	r0, r0, #5
 8006550:	2100      	movs	r1, #0
 8006552:	f1bc 0f00 	cmp.w	ip, #0
 8006556:	bfc8      	it	gt
 8006558:	2101      	movgt	r1, #1
 800655a:	4308      	orrs	r0, r1
 800655c:	bdb0      	pop	{r4, r5, r7, pc}
 800655e:	2001      	movs	r0, #1
 8006560:	bdb0      	pop	{r4, r5, r7, pc}
 8006562:	2200      	movs	r2, #0
 8006564:	45ae      	cmp	lr, r5
 8006566:	d00a      	beq.n	800657e <smoltcp::socket::tcp::Socket::seq_to_transmit+0x9a>
 8006568:	ea2c 73ec 	bic.w	r3, ip, ip, asr #31
 800656c:	428b      	cmp	r3, r1
 800656e:	d206      	bcs.n	800657e <smoltcp::socket::tcp::Socket::seq_to_transmit+0x9a>
 8006570:	f890 0132 	ldrb.w	r0, [r0, #306]	@ 0x132
 8006574:	07c0      	lsls	r0, r0, #31
 8006576:	4610      	mov	r0, r2
 8006578:	bf18      	it	ne
 800657a:	bdb0      	popne	{r4, r5, r7, pc}
 800657c:	e7e8      	b.n	8006550 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x6c>
 800657e:	4610      	mov	r0, r2
 8006580:	e7e6      	b.n	8006550 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x6c>
 8006582:	f64a 4078 	movw	r0, #44152	@ 0xac78
 8006586:	f64a 42b0 	movw	r2, #44208	@ 0xacb0
 800658a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800658e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8006592:	2171      	movs	r1, #113	@ 0x71
 8006594:	f7fd f9a4 	bl	80038e0 <core::panicking::panic_fmt>
 8006598:	f64a 5040 	movw	r0, #44352	@ 0xad40
 800659c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80065a0:	f7fd f923 	bl	80037ea <core::option::unwrap_failed>

080065a4 <smoltcp::socket::tcp::Socket::process>:
 80065a4:	b5f0      	push	{r4, r5, r6, r7, lr}
 80065a6:	af03      	add	r7, sp, #12
 80065a8:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 80065ac:	b091      	sub	sp, #68	@ 0x44
 80065ae:	f891 9133 	ldrb.w	r9, [r1, #307]	@ 0x133
 80065b2:	4682      	mov	sl, r0
 80065b4:	200c      	movs	r0, #12
 80065b6:	68fd      	ldr	r5, [r7, #12]
 80065b8:	469b      	mov	fp, r3
 80065ba:	f8d7 8008 	ldr.w	r8, [r7, #8]
 80065be:	fa20 f009 	lsr.w	r0, r0, r9
 80065c2:	f000 0301 	and.w	r3, r0, #1
 80065c6:	f44f 7048 	mov.w	r0, #800	@ 0x320
 80065ca:	f895 4051 	ldrb.w	r4, [r5, #81]	@ 0x51
 80065ce:	fa20 f609 	lsr.w	r6, r0, r9
 80065d2:	e9d5 0c00 	ldrd	r0, ip, [r5]
 80065d6:	f1b9 0f02 	cmp.w	r9, #2
 80065da:	d113      	bne.n	8006604 <smoltcp::socket::tcp::Socket::process+0x60>
 80065dc:	b35c      	cbz	r4, 8006636 <smoltcp::socket::tcp::Socket::process+0x92>
 80065de:	2c02      	cmp	r4, #2
 80065e0:	d01c      	beq.n	800661c <smoltcp::socket::tcp::Socket::process+0x78>
 80065e2:	2c04      	cmp	r4, #4
 80065e4:	d133      	bne.n	800664e <smoltcp::socket::tcp::Socket::process+0xaa>
 80065e6:	2800      	cmp	r0, #0
 80065e8:	f000 80d2 	beq.w	8006790 <smoltcp::socket::tcp::Socket::process+0x1ec>
 80065ec:	4686      	mov	lr, r0
 80065ee:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 80065f2:	3001      	adds	r0, #1
 80065f4:	4584      	cmp	ip, r0
 80065f6:	4670      	mov	r0, lr
 80065f8:	d031      	beq.n	800665e <smoltcp::socket::tcp::Socket::process+0xba>
 80065fa:	f240 001d 	movw	r0, #29
 80065fe:	f2c0 0000 	movt	r0, #0
 8006602:	e101      	b.n	8006808 <smoltcp::socket::tcp::Socket::process+0x264>
 8006604:	2c04      	cmp	r4, #4
 8006606:	d02a      	beq.n	800665e <smoltcp::socket::tcp::Socket::process+0xba>
 8006608:	f1b9 0f01 	cmp.w	r9, #1
 800660c:	d024      	beq.n	8006658 <smoltcp::socket::tcp::Socket::process+0xb4>
 800660e:	f1b9 0f02 	cmp.w	r9, #2
 8006612:	f040 809c 	bne.w	800674e <smoltcp::socket::tcp::Socket::process+0x1aa>
 8006616:	b174      	cbz	r4, 8006636 <smoltcp::socket::tcp::Socket::process+0x92>
 8006618:	2c02      	cmp	r4, #2
 800661a:	d118      	bne.n	800664e <smoltcp::socket::tcp::Socket::process+0xaa>
 800661c:	b1f8      	cbz	r0, 800665e <smoltcp::socket::tcp::Socket::process+0xba>
 800661e:	4686      	mov	lr, r0
 8006620:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 8006624:	3001      	adds	r0, #1
 8006626:	4584      	cmp	ip, r0
 8006628:	4670      	mov	r0, lr
 800662a:	d018      	beq.n	800665e <smoltcp::socket::tcp::Socket::process+0xba>
 800662c:	f240 001e 	movw	r0, #30
 8006630:	f2c0 0000 	movt	r0, #0
 8006634:	e12c      	b.n	8006890 <smoltcp::socket::tcp::Socket::process+0x2ec>
 8006636:	b150      	cbz	r0, 800664e <smoltcp::socket::tcp::Socket::process+0xaa>
 8006638:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 800663c:	3001      	adds	r0, #1
 800663e:	4584      	cmp	ip, r0
 8006640:	f040 8122 	bne.w	8006888 <smoltcp::socket::tcp::Socket::process+0x2e4>
 8006644:	f240 000e 	movw	r0, #14
 8006648:	f2c0 0000 	movt	r0, #0
 800664c:	e0dc      	b.n	8006808 <smoltcp::socket::tcp::Socket::process+0x264>
 800664e:	f240 000d 	movw	r0, #13
 8006652:	f2c0 0000 	movt	r0, #0
 8006656:	e0d7      	b.n	8006808 <smoltcp::socket::tcp::Socket::process+0x264>
 8006658:	2800      	cmp	r0, #0
 800665a:	f041 8070 	bne.w	800773e <smoltcp::socket::tcp::Socket::process+0x119a>
 800665e:	900d      	str	r0, [sp, #52]	@ 0x34
 8006660:	f8d1 00c4 	ldr.w	r0, [r1, #196]	@ 0xc4
 8006664:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8006668:	dd0e      	ble.n	8006688 <smoltcp::socket::tcp::Socket::process+0xe4>
 800666a:	e9cd 6305 	strd	r6, r3, [sp, #20]
 800666e:	f8cd 801c 	str.w	r8, [sp, #28]
 8006672:	f8d1 80bc 	ldr.w	r8, [r1, #188]	@ 0xbc
 8006676:	f1b8 3fff 	cmp.w	r8, #4294967295	@ 0xffffffff
 800667a:	bfc1      	itttt	gt
 800667c:	920a      	strgt	r2, [sp, #40]	@ 0x28
 800667e:	6c2b      	ldrgt	r3, [r5, #64]	@ 0x40
 8006680:	930b      	strgt	r3, [sp, #44]	@ 0x2c
 8006682:	f1b3 3fff 	cmpgt.w	r3, #4294967295	@ 0xffffffff
 8006686:	dc0a      	bgt.n	800669e <smoltcp::socket::tcp::Socket::process+0xfa>
 8006688:	f64a 4078 	movw	r0, #44152	@ 0xac78
 800668c:	f64a 42b0 	movw	r2, #44208	@ 0xacb0
 8006690:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8006694:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8006698:	2171      	movs	r1, #113	@ 0x71
 800669a:	f7fd f921 	bl	80038e0 <core::panicking::panic_fmt>
 800669e:	f8d1 6104 	ldr.w	r6, [r1, #260]	@ 0x104
 80066a2:	46a6      	mov	lr, r4
 80066a4:	6c6c      	ldr	r4, [r5, #68]	@ 0x44
 80066a6:	f1a9 0301 	sub.w	r3, r9, #1
 80066aa:	f8cd c03c 	str.w	ip, [sp, #60]	@ 0x3c
 80066ae:	eb00 0c06 	add.w	ip, r0, r6
 80066b2:	2b02      	cmp	r3, #2
 80066b4:	f8cd a030 	str.w	sl, [sp, #48]	@ 0x30
 80066b8:	950e      	str	r5, [sp, #56]	@ 0x38
 80066ba:	f8cd b020 	str.w	fp, [sp, #32]
 80066be:	d239      	bcs.n	8006734 <smoltcp::socket::tcp::Socket::process+0x190>
 80066c0:	9603      	str	r6, [sp, #12]
 80066c2:	46e2      	mov	sl, ip
 80066c4:	2501      	movs	r5, #1
 80066c6:	2300      	movs	r3, #0
 80066c8:	f04f 0c00 	mov.w	ip, #0
 80066cc:	4676      	mov	r6, lr
 80066ce:	9409      	str	r4, [sp, #36]	@ 0x24
 80066d0:	980d      	ldr	r0, [sp, #52]	@ 0x34
 80066d2:	f04f 0b00 	mov.w	fp, #0
 80066d6:	2e04      	cmp	r6, #4
 80066d8:	f000 81f0 	beq.w	8006abc <smoltcp::socket::tcp::Socket::process+0x518>
 80066dc:	2400      	movs	r4, #0
 80066de:	f04f 0800 	mov.w	r8, #0
 80066e2:	2800      	cmp	r0, #0
 80066e4:	f000 81ed 	beq.w	8006ac2 <smoltcp::socket::tcp::Socket::process+0x51e>
 80066e8:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 80066ec:	9a06      	ldr	r2, [sp, #24]
 80066ee:	4410      	add	r0, r2
 80066f0:	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
 80066f2:	1a14      	subs	r4, r2, r0
 80066f4:	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
 80066f8:	f340 80fc 	ble.w	80068f4 <smoltcp::socket::tcp::Socket::process+0x350>
 80066fc:	9805      	ldr	r0, [sp, #20]
 80066fe:	07c0      	lsls	r0, r0, #31
 8006700:	d00b      	beq.n	800671a <smoltcp::socket::tcp::Socket::process+0x176>
 8006702:	f8d1 00d4 	ldr.w	r0, [r1, #212]	@ 0xd4
 8006706:	3001      	adds	r0, #1
 8006708:	42a0      	cmp	r0, r4
 800670a:	eba0 0004 	sub.w	r0, r0, r4
 800670e:	fab0 f080 	clz	r0, r0
 8006712:	bf08      	it	eq
 8006714:	3c01      	subeq	r4, #1
 8006716:	ea4f 1850 	mov.w	r8, r0, lsr #5
 800671a:	f8d1 0108 	ldr.w	r0, [r1, #264]	@ 0x108
 800671e:	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
 8006720:	1a80      	subs	r0, r0, r2
 8006722:	fab0 f080 	clz	r0, r0
 8006726:	ea4f 1b50 	mov.w	fp, r0, lsr #5
 800672a:	6808      	ldr	r0, [r1, #0]
 800672c:	07c0      	lsls	r0, r0, #31
 800672e:	f040 80e6 	bne.w	80068fe <smoltcp::socket::tcp::Socket::process+0x35a>
 8006732:	e1c6      	b.n	8006ac2 <smoltcp::socket::tcp::Socket::process+0x51e>
 8006734:	9b0b      	ldr	r3, [sp, #44]	@ 0x2c
 8006736:	eb08 0b06 	add.w	fp, r8, r6
 800673a:	191d      	adds	r5, r3, r4
 800673c:	b36b      	cbz	r3, 800679a <smoltcp::socket::tcp::Socket::process+0x1f6>
 800673e:	4540      	cmp	r0, r8
 8006740:	d178      	bne.n	8006834 <smoltcp::socket::tcp::Socket::process+0x290>
 8006742:	f240 0011 	movw	r0, #17
 8006746:	460e      	mov	r6, r1
 8006748:	f2c0 0000 	movt	r0, #0
 800674c:	e02f      	b.n	80067ae <smoltcp::socket::tcp::Socket::process+0x20a>
 800674e:	2800      	cmp	r0, #0
 8006750:	d056      	beq.n	8006800 <smoltcp::socket::tcp::Socket::process+0x25c>
 8006752:	469e      	mov	lr, r3
 8006754:	f1b9 0f03 	cmp.w	r9, #3
 8006758:	900d      	str	r0, [sp, #52]	@ 0x34
 800675a:	f040 811d 	bne.w	8006998 <smoltcp::socket::tcp::Socket::process+0x3f4>
 800675e:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 8006762:	4673      	mov	r3, lr
 8006764:	3001      	adds	r0, #1
 8006766:	4584      	cmp	ip, r0
 8006768:	980d      	ldr	r0, [sp, #52]	@ 0x34
 800676a:	f43f af78 	beq.w	800665e <smoltcp::socket::tcp::Socket::process+0xba>
 800676e:	f240 001b 	movw	r0, #27
 8006772:	f2c0 0000 	movt	r0, #0
 8006776:	f7fd fe7e 	bl	8004476 <defmt::export::acquire_header_and_release>
 800677a:	4650      	mov	r0, sl
 800677c:	4659      	mov	r1, fp
 800677e:	4642      	mov	r2, r8
 8006780:	462b      	mov	r3, r5
 8006782:	b011      	add	sp, #68	@ 0x44
 8006784:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8006788:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 800678c:	f001 b819 	b.w	80077c2 <smoltcp::socket::tcp::Socket::rst_reply>
 8006790:	f240 001c 	movw	r0, #28
 8006794:	f2c0 0000 	movt	r0, #0
 8006798:	e036      	b.n	8006808 <smoltcp::socket::tcp::Socket::process+0x264>
 800679a:	f1ac 0301 	sub.w	r3, ip, #1
 800679e:	429c      	cmp	r4, r3
 80067a0:	f040 80ef 	bne.w	8006982 <smoltcp::socket::tcp::Socket::process+0x3de>
 80067a4:	f240 0012 	movw	r0, #18
 80067a8:	460e      	mov	r6, r1
 80067aa:	f2c0 0000 	movt	r0, #0
 80067ae:	f7fd fe62 	bl	8004476 <defmt::export::acquire_header_and_release>
 80067b2:	980a      	ldr	r0, [sp, #40]	@ 0x28
 80067b4:	4631      	mov	r1, r6
 80067b6:	f8dd e038 	ldr.w	lr, [sp, #56]	@ 0x38
 80067ba:	e9d0 4504 	ldrd	r4, r5, [r0, #16]
 80067be:	980c      	ldr	r0, [sp, #48]	@ 0x30
 80067c0:	9a08      	ldr	r2, [sp, #32]
 80067c2:	f896 3133 	ldrb.w	r3, [r6, #307]	@ 0x133
 80067c6:	2b0a      	cmp	r3, #10
 80067c8:	d10e      	bne.n	80067e8 <smoltcp::socket::tcp::Socket::process+0x244>
 80067ca:	f249 6380 	movw	r3, #38528	@ 0x9680
 80067ce:	f04f 0c00 	mov.w	ip, #0
 80067d2:	f2c0 0398 	movt	r3, #152	@ 0x98
 80067d6:	f04f 0803 	mov.w	r8, #3
 80067da:	191b      	adds	r3, r3, r4
 80067dc:	e9c1 8c20 	strd	r8, ip, [r1, #128]	@ 0x80
 80067e0:	f145 0600 	adc.w	r6, r5, #0
 80067e4:	e9c1 3622 	strd	r3, r6, [r1, #136]	@ 0x88
 80067e8:	e9d1 363e 	ldrd	r3, r6, [r1, #248]	@ 0xf8
 80067ec:	1ae3      	subs	r3, r4, r3
 80067ee:	eb75 0306 	sbcs.w	r3, r5, r6
 80067f2:	da12      	bge.n	800681a <smoltcp::socket::tcp::Socket::process+0x276>
 80067f4:	2102      	movs	r1, #2
 80067f6:	6101      	str	r1, [r0, #16]
 80067f8:	b011      	add	sp, #68	@ 0x44
 80067fa:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80067fe:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8006800:	f240 0010 	movw	r0, #16
 8006804:	f2c0 0000 	movt	r0, #0
 8006808:	f7fd fe35 	bl	8004476 <defmt::export::acquire_header_and_release>
 800680c:	2002      	movs	r0, #2
 800680e:	f8ca 0010 	str.w	r0, [sl, #16]
 8006812:	b011      	add	sp, #68	@ 0x44
 8006814:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8006818:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800681a:	f244 2340 	movw	r3, #16960	@ 0x4240
 800681e:	f2c0 030f 	movt	r3, #15
 8006822:	18e6      	adds	r6, r4, r3
 8006824:	f145 0300 	adc.w	r3, r5, #0
 8006828:	e9c1 633e 	strd	r6, r3, [r1, #248]	@ 0xf8
 800682c:	f8c7 e008 	str.w	lr, [r7, #8]
 8006830:	f000 bf6b 	b.w	800770a <smoltcp::socket::tcp::Socket::process+0x1166>
 8006834:	ebac 0004 	sub.w	r0, ip, r4
 8006838:	2800      	cmp	r0, #0
 800683a:	dc04      	bgt.n	8006846 <smoltcp::socket::tcp::Socket::process+0x2a2>
 800683c:	4623      	mov	r3, r4
 800683e:	ebb4 020b 	subs.w	r2, r4, fp
 8006842:	f100 8109 	bmi.w	8006a58 <smoltcp::socket::tcp::Socket::process+0x4b4>
 8006846:	ebac 0205 	sub.w	r2, ip, r5
 800684a:	f1b2 3fff 	cmp.w	r2, #4294967295	@ 0xffffffff
 800684e:	bfdc      	itt	le
 8006850:	eba5 020b 	suble.w	r2, r5, fp
 8006854:	2a01      	cmple	r2, #1
 8006856:	f2c0 80fb 	blt.w	8006a50 <smoltcp::socket::tcp::Socket::process+0x4ac>
 800685a:	f240 0016 	movw	r0, #22
 800685e:	460e      	mov	r6, r1
 8006860:	f2c0 0000 	movt	r0, #0
 8006864:	46e0      	mov	r8, ip
 8006866:	f7fd fdf5 	bl	8004454 <defmt::export::acquire_and_header>
 800686a:	4620      	mov	r0, r4
 800686c:	f001 fb03 	bl	8007e76 <defmt::export::fmt>
 8006870:	4628      	mov	r0, r5
 8006872:	f001 fb00 	bl	8007e76 <defmt::export::fmt>
 8006876:	4640      	mov	r0, r8
 8006878:	f001 fafd 	bl	8007e76 <defmt::export::fmt>
 800687c:	4658      	mov	r0, fp
 800687e:	f001 fafa 	bl	8007e76 <defmt::export::fmt>
 8006882:	f7fd ff21 	bl	80046c8 <_defmt_release>
 8006886:	e794      	b.n	80067b2 <smoltcp::socket::tcp::Socket::process+0x20e>
 8006888:	f240 000f 	movw	r0, #15
 800688c:	f2c0 0000 	movt	r0, #0
 8006890:	4666      	mov	r6, ip
 8006892:	f7fd fdf0 	bl	8004476 <defmt::export::acquire_header_and_release>
 8006896:	2000      	movs	r0, #0
 8006898:	2101      	movs	r1, #1
 800689a:	f88a 0058 	strb.w	r0, [sl, #88]	@ 0x58
 800689e:	f04f 6280 	mov.w	r2, #67108864	@ 0x4000000
 80068a2:	f8aa 0048 	strh.w	r0, [sl, #72]	@ 0x48
 80068a6:	f8ca 003c 	str.w	r0, [sl, #60]	@ 0x3c
 80068aa:	f8ca 0030 	str.w	r0, [sl, #48]	@ 0x30
 80068ae:	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
 80068b2:	f8ca 0018 	str.w	r0, [sl, #24]
 80068b6:	f8ca 0050 	str.w	r0, [sl, #80]	@ 0x50
 80068ba:	f8ca 0010 	str.w	r0, [sl, #16]
 80068be:	f44f 7050 	mov.w	r0, #832	@ 0x340
 80068c2:	f8aa 000c 	strh.w	r0, [sl, #12]
 80068c6:	f8d5 004a 	ldr.w	r0, [r5, #74]	@ 0x4a
 80068ca:	f8ca 104c 	str.w	r1, [sl, #76]	@ 0x4c
 80068ce:	2114      	movs	r1, #20
 80068d0:	f8ca 205e 	str.w	r2, [sl, #94]	@ 0x5e
 80068d4:	ea4f 4030 	mov.w	r0, r0, ror #16
 80068d8:	f8ca 6054 	str.w	r6, [sl, #84]	@ 0x54
 80068dc:	f8ca 8000 	str.w	r8, [sl]
 80068e0:	f8ca b004 	str.w	fp, [sl, #4]
 80068e4:	f8ca 1008 	str.w	r1, [sl, #8]
 80068e8:	f8ca 005a 	str.w	r0, [sl, #90]	@ 0x5a
 80068ec:	b011      	add	sp, #68	@ 0x44
 80068ee:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80068f2:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80068f4:	2400      	movs	r4, #0
 80068f6:	6808      	ldr	r0, [r1, #0]
 80068f8:	07c0      	lsls	r0, r0, #31
 80068fa:	f000 80e2 	beq.w	8006ac2 <smoltcp::socket::tcp::Socket::process+0x51e>
 80068fe:	6908      	ldr	r0, [r1, #16]
 8006900:	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
 8006902:	1a10      	subs	r0, r2, r0
 8006904:	f100 80dd 	bmi.w	8006ac2 <smoltcp::socket::tcp::Socket::process+0x51e>
 8006908:	e9cd 6500 	strd	r6, r5, [sp]
 800690c:	f8cd c010 	str.w	ip, [sp, #16]
 8006910:	9302      	str	r3, [sp, #8]
 8006912:	980a      	ldr	r0, [sp, #40]	@ 0x28
 8006914:	e9d1 3502 	ldrd	r3, r5, [r1, #8]
 8006918:	e9d0 0c04 	ldrd	r0, ip, [r0, #16]
 800691c:	e9d1 6208 	ldrd	r6, r2, [r1, #32]
 8006920:	e9cd 6205 	strd	r6, r2, [sp, #20]
 8006924:	2200      	movs	r2, #0
 8006926:	1ac0      	subs	r0, r0, r3
 8006928:	e9c1 2200 	strd	r2, r2, [r1]
 800692c:	f881 2028 	strb.w	r2, [r1, #40]	@ 0x28
 8006930:	eb6c 0205 	sbc.w	r2, ip, r5
 8006934:	460d      	mov	r5, r1
 8006936:	ea80 70e2 	eor.w	r0, r0, r2, asr #31
 800693a:	ea82 73e2 	eor.w	r3, r2, r2, asr #31
 800693e:	ebb0 70e2 	subs.w	r0, r0, r2, asr #31
 8006942:	eb63 72e2 	sbc.w	r2, r3, r2, asr #31
 8006946:	2300      	movs	r3, #0
 8006948:	4611      	mov	r1, r2
 800694a:	f44f 727a 	mov.w	r2, #1000	@ 0x3e8
 800694e:	f002 fe0f 	bl	8009570 <__aeabi_uldivmod>
 8006952:	9a05      	ldr	r2, [sp, #20]
 8006954:	4629      	mov	r1, r5
 8006956:	ebc2 02c2 	rsb	r2, r2, r2, lsl #3
 800695a:	4402      	add	r2, r0
 800695c:	3207      	adds	r2, #7
 800695e:	08d2      	lsrs	r2, r2, #3
 8006960:	1a10      	subs	r0, r2, r0
 8006962:	622a      	str	r2, [r5, #32]
 8006964:	bf48      	it	mi
 8006966:	4240      	negmi	r0, r0
 8006968:	9a06      	ldr	r2, [sp, #24]
 800696a:	9b02      	ldr	r3, [sp, #8]
 800696c:	e9dd 6500 	ldrd	r6, r5, [sp]
 8006970:	eb02 0242 	add.w	r2, r2, r2, lsl #1
 8006974:	f8dd c010 	ldr.w	ip, [sp, #16]
 8006978:	4410      	add	r0, r2
 800697a:	3003      	adds	r0, #3
 800697c:	0880      	lsrs	r0, r0, #2
 800697e:	6248      	str	r0, [r1, #36]	@ 0x24
 8006980:	e09f      	b.n	8006ac2 <smoltcp::socket::tcp::Socket::process+0x51e>
 8006982:	4540      	cmp	r0, r8
 8006984:	d156      	bne.n	8006a34 <smoltcp::socket::tcp::Socket::process+0x490>
 8006986:	4663      	mov	r3, ip
 8006988:	45a4      	cmp	ip, r4
 800698a:	d065      	beq.n	8006a58 <smoltcp::socket::tcp::Socket::process+0x4b4>
 800698c:	f240 0021 	movw	r0, #33	@ 0x21
 8006990:	460e      	mov	r6, r1
 8006992:	f2c0 0000 	movt	r0, #0
 8006996:	e70a      	b.n	80067ae <smoltcp::socket::tcp::Socket::process+0x20a>
 8006998:	f006 0001 	and.w	r0, r6, #1
 800699c:	f8d1 30d4 	ldr.w	r3, [r1, #212]	@ 0xd4
 80069a0:	4470      	add	r0, lr
 80069a2:	f8cd 801c 	str.w	r8, [sp, #28]
 80069a6:	4418      	add	r0, r3
 80069a8:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 80069ac:	f77f ae6c 	ble.w	8006688 <smoltcp::socket::tcp::Socket::process+0xe4>
 80069b0:	f8d1 3100 	ldr.w	r3, [r1, #256]	@ 0x100
 80069b4:	eb03 080e 	add.w	r8, r3, lr
 80069b8:	4418      	add	r0, r3
 80069ba:	900f      	str	r0, [sp, #60]	@ 0x3c
 80069bc:	ebbc 0008 	subs.w	r0, ip, r8
 80069c0:	f100 830f 	bmi.w	8006fe2 <smoltcp::socket::tcp::Socket::process+0xa3e>
 80069c4:	f8cd 8038 	str.w	r8, [sp, #56]	@ 0x38
 80069c8:	4673      	mov	r3, lr
 80069ca:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 80069cc:	f8dd 801c 	ldr.w	r8, [sp, #28]
 80069d0:	ebac 0000 	sub.w	r0, ip, r0
 80069d4:	2800      	cmp	r0, #0
 80069d6:	980d      	ldr	r0, [sp, #52]	@ 0x34
 80069d8:	f77f ae41 	ble.w	800665e <smoltcp::socket::tcp::Socket::process+0xba>
 80069dc:	f240 001a 	movw	r0, #26
 80069e0:	4688      	mov	r8, r1
 80069e2:	f2c0 0000 	movt	r0, #0
 80069e6:	4664      	mov	r4, ip
 80069e8:	4691      	mov	r9, r2
 80069ea:	f7fd fd33 	bl	8004454 <defmt::export::acquire_and_header>
 80069ee:	4620      	mov	r0, r4
 80069f0:	f001 fa41 	bl	8007e76 <defmt::export::fmt>
 80069f4:	980e      	ldr	r0, [sp, #56]	@ 0x38
 80069f6:	f001 fa3e 	bl	8007e76 <defmt::export::fmt>
 80069fa:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 80069fc:	f001 fa3b 	bl	8007e76 <defmt::export::fmt>
 8006a00:	f7fd fe62 	bl	80046c8 <_defmt_release>
 8006a04:	e9d9 2004 	ldrd	r2, r0, [r9, #16]
 8006a08:	e9d8 363e 	ldrd	r3, r6, [r8, #248]	@ 0xf8
 8006a0c:	1ad3      	subs	r3, r2, r3
 8006a0e:	eb70 0306 	sbcs.w	r3, r0, r6
 8006a12:	f6ff aefb 	blt.w	800680c <smoltcp::socket::tcp::Socket::process+0x268>
 8006a16:	f244 2340 	movw	r3, #16960	@ 0x4240
 8006a1a:	4641      	mov	r1, r8
 8006a1c:	f2c0 030f 	movt	r3, #15
 8006a20:	18d2      	adds	r2, r2, r3
 8006a22:	f140 0000 	adc.w	r0, r0, #0
 8006a26:	e9c1 203e 	strd	r2, r0, [r1, #248]	@ 0xf8
 8006a2a:	4650      	mov	r0, sl
 8006a2c:	60bd      	str	r5, [r7, #8]
 8006a2e:	465a      	mov	r2, fp
 8006a30:	f000 be6b 	b.w	800770a <smoltcp::socket::tcp::Socket::process+0x1166>
 8006a34:	ebac 0004 	sub.w	r0, ip, r4
 8006a38:	2800      	cmp	r0, #0
 8006a3a:	dc03      	bgt.n	8006a44 <smoltcp::socket::tcp::Socket::process+0x4a0>
 8006a3c:	4623      	mov	r3, r4
 8006a3e:	ebb4 000b 	subs.w	r0, r4, fp
 8006a42:	d409      	bmi.n	8006a58 <smoltcp::socket::tcp::Socket::process+0x4b4>
 8006a44:	f240 0020 	movw	r0, #32
 8006a48:	460e      	mov	r6, r1
 8006a4a:	f2c0 0000 	movt	r0, #0
 8006a4e:	e6ae      	b.n	80067ae <smoltcp::socket::tcp::Socket::process+0x20a>
 8006a50:	4623      	mov	r3, r4
 8006a52:	2800      	cmp	r0, #0
 8006a54:	bfc8      	it	gt
 8006a56:	4663      	movgt	r3, ip
 8006a58:	2001      	movs	r0, #1
 8006a5a:	e9c1 0428 	strd	r0, r4, [r1, #160]	@ 0xa0
 8006a5e:	1b18      	subs	r0, r3, r4
 8006a60:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8006a64:	f340 8660 	ble.w	8007728 <smoltcp::socket::tcp::Socket::process+0x1184>
 8006a68:	ebbb 0205 	subs.w	r2, fp, r5
 8006a6c:	9603      	str	r6, [sp, #12]
 8006a6e:	bf48      	it	mi
 8006a70:	465d      	movmi	r5, fp
 8006a72:	1b2a      	subs	r2, r5, r4
 8006a74:	f1b2 3fff 	cmp.w	r2, #4294967295	@ 0xffffffff
 8006a78:	f340 8656 	ble.w	8007728 <smoltcp::socket::tcp::Socket::process+0x1184>
 8006a7c:	46e2      	mov	sl, ip
 8006a7e:	f8dd c038 	ldr.w	ip, [sp, #56]	@ 0x38
 8006a82:	4282      	cmp	r2, r0
 8006a84:	f0c0 8666 	bcc.w	8007754 <smoltcp::socket::tcp::Socket::process+0x11b0>
 8006a88:	9d0b      	ldr	r5, [sp, #44]	@ 0x2c
 8006a8a:	42aa      	cmp	r2, r5
 8006a8c:	f200 8662 	bhi.w	8007754 <smoltcp::socket::tcp::Socket::process+0x11b0>
 8006a90:	eba3 030a 	sub.w	r3, r3, sl
 8006a94:	9409      	str	r4, [sp, #36]	@ 0x24
 8006a96:	f1b3 3fff 	cmp.w	r3, #4294967295	@ 0xffffffff
 8006a9a:	f340 8645 	ble.w	8007728 <smoltcp::socket::tcp::Socket::process+0x1184>
 8006a9e:	461d      	mov	r5, r3
 8006aa0:	f8dc 303c 	ldr.w	r3, [ip, #60]	@ 0x3c
 8006aa4:	eba2 0c00 	sub.w	ip, r2, r0
 8006aa8:	4676      	mov	r6, lr
 8006aaa:	4418      	add	r0, r3
 8006aac:	462b      	mov	r3, r5
 8006aae:	4605      	mov	r5, r0
 8006ab0:	980d      	ldr	r0, [sp, #52]	@ 0x34
 8006ab2:	f04f 0b00 	mov.w	fp, #0
 8006ab6:	2e04      	cmp	r6, #4
 8006ab8:	f47f ae10 	bne.w	80066dc <smoltcp::socket::tcp::Socket::process+0x138>
 8006abc:	2400      	movs	r4, #0
 8006abe:	f04f 0800 	mov.w	r8, #0
 8006ac2:	1e72      	subs	r2, r6, #1
 8006ac4:	bf18      	it	ne
 8006ac6:	4632      	movne	r2, r6
 8006ac8:	9809      	ldr	r0, [sp, #36]	@ 0x24
 8006aca:	ebba 0000 	subs.w	r0, sl, r0
 8006ace:	4610      	mov	r0, r2
 8006ad0:	bf48      	it	mi
 8006ad2:	2000      	movmi	r0, #0
 8006ad4:	2a03      	cmp	r2, #3
 8006ad6:	bf18      	it	ne
 8006ad8:	4610      	movne	r0, r2
 8006ada:	f1b9 0f01 	cmp.w	r9, #1
 8006ade:	d039      	beq.n	8006b54 <smoltcp::socket::tcp::Socket::process+0x5b0>
 8006ae0:	f1b9 0f03 	cmp.w	r9, #3
 8006ae4:	b2c0      	uxtb	r0, r0
 8006ae6:	bf08      	it	eq
 8006ae8:	2804      	cmpeq	r0, #4
 8006aea:	f000 8082 	beq.w	8006bf2 <smoltcp::socket::tcp::Socket::process+0x64e>
 8006aee:	e8df f010 	tbh	[pc, r0, lsl #1]
 8006af2:	0005      	.short	0x0005
 8006af4:	00980039 	.word	0x00980039
 8006af8:	008c00eb 	.word	0x008c00eb
 8006afc:	f1a9 0003 	sub.w	r0, r9, #3
 8006b00:	2806      	cmp	r0, #6
 8006b02:	d82f      	bhi.n	8006b64 <smoltcp::socket::tcp::Socket::process+0x5c0>
 8006b04:	e9cd 5301 	strd	r5, r3, [sp, #4]
 8006b08:	f8cd c010 	str.w	ip, [sp, #16]
 8006b0c:	9406      	str	r4, [sp, #24]
 8006b0e:	e8df f010 	tbh	[pc, r0, lsl #1]
 8006b12:	02c9      	.short	0x02c9
 8006b14:	02a902e7 	.word	0x02a902e7
 8006b18:	00070007 	.word	0x00070007
 8006b1c:	0435030a 	.word	0x0435030a
 8006b20:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006b24:	4634      	mov	r4, r6
 8006b26:	6c8b      	ldr	r3, [r1, #72]	@ 0x48
 8006b28:	f04f 0e00 	mov.w	lr, #0
 8006b2c:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8006b30:	6cce      	ldr	r6, [r1, #76]	@ 0x4c
 8006b32:	195b      	adds	r3, r3, r5
 8006b34:	f8d1 c040 	ldr.w	ip, [r1, #64]	@ 0x40
 8006b38:	4170      	adcs	r0, r6
 8006b3a:	6c4a      	ldr	r2, [r1, #68]	@ 0x44
 8006b3c:	4626      	mov	r6, r4
 8006b3e:	f8c1 e080 	str.w	lr, [r1, #128]	@ 0x80
 8006b42:	f8c1 c088 	str.w	ip, [r1, #136]	@ 0x88
 8006b46:	f8c1 e084 	str.w	lr, [r1, #132]	@ 0x84
 8006b4a:	f8c1 208c 	str.w	r2, [r1, #140]	@ 0x8c
 8006b4e:	f8c1 3090 	str.w	r3, [r1, #144]	@ 0x90
 8006b52:	e2c2      	b.n	80070da <smoltcp::socket::tcp::Socket::process+0xb36>
 8006b54:	b2c0      	uxtb	r0, r0
 8006b56:	e8df f010 	tbh	[pc, r0, lsl #1]
 8006b5a:	0005      	.short	0x0005
 8006b5c:	00ef0005 	.word	0x00ef0005
 8006b60:	005d0005 	.word	0x005d0005
 8006b64:	f240 001f 	movw	r0, #31
 8006b68:	f2c0 0000 	movt	r0, #0
 8006b6c:	f7fd fc72 	bl	8004454 <defmt::export::acquire_and_header>
 8006b70:	f240 0907 	movw	r9, #7
 8006b74:	a810      	add	r0, sp, #64	@ 0x40
 8006b76:	f2c0 0900 	movt	r9, #0
 8006b7a:	2102      	movs	r1, #2
 8006b7c:	f8ad 9040 	strh.w	r9, [sp, #64]	@ 0x40
 8006b80:	f7fd fe1a 	bl	80047b8 <_defmt_write>
 8006b84:	f240 003c 	movw	r0, #60	@ 0x3c
 8006b88:	2102      	movs	r1, #2
 8006b8a:	f2c0 0000 	movt	r0, #0
 8006b8e:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006b92:	a810      	add	r0, sp, #64	@ 0x40
 8006b94:	f7fd fe10 	bl	80047b8 <_defmt_write>
 8006b98:	9c0e      	ldr	r4, [sp, #56]	@ 0x38
 8006b9a:	f240 0a04 	movw	sl, #4
 8006b9e:	a810      	add	r0, sp, #64	@ 0x40
 8006ba0:	f2c0 0a00 	movt	sl, #0
 8006ba4:	2102      	movs	r1, #2
 8006ba6:	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
 8006baa:	f8b4 504a 	ldrh.w	r5, [r4, #74]	@ 0x4a
 8006bae:	f7fd fe03 	bl	80047b8 <_defmt_write>
 8006bb2:	a810      	add	r0, sp, #64	@ 0x40
 8006bb4:	2102      	movs	r1, #2
 8006bb6:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006bba:	f7fd fdfd 	bl	80047b8 <_defmt_write>
 8006bbe:	a810      	add	r0, sp, #64	@ 0x40
 8006bc0:	2102      	movs	r1, #2
 8006bc2:	f8b4 504c 	ldrh.w	r5, [r4, #76]	@ 0x4c
 8006bc6:	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
 8006bca:	f7fd fdf5 	bl	80047b8 <_defmt_write>
 8006bce:	a810      	add	r0, sp, #64	@ 0x40
 8006bd0:	2102      	movs	r1, #2
 8006bd2:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006bd6:	f7fd fdef 	bl	80047b8 <_defmt_write>
 8006bda:	e8df f016 	tbh	[pc, r6, lsl #1]
 8006bde:	0149      	.short	0x0149
 8006be0:	013a0005 	.word	0x013a0005
 8006be4:	0135013f 	.word	0x0135013f
 8006be8:	f240 0037 	movw	r0, #55	@ 0x37
 8006bec:	f2c0 0000 	movt	r0, #0
 8006bf0:	e138      	b.n	8006e64 <smoltcp::socket::tcp::Socket::process+0x8c0>
 8006bf2:	f8b1 20b0 	ldrh.w	r2, [r1, #176]	@ 0xb0
 8006bf6:	2a00      	cmp	r2, #0
 8006bf8:	f43f af79 	beq.w	8006aee <smoltcp::socket::tcp::Socket::process+0x54a>
 8006bfc:	2001      	movs	r0, #1
 8006bfe:	f881 0133 	strb.w	r0, [r1, #307]	@ 0x133
 8006c02:	2000      	movs	r0, #0
 8006c04:	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
 8006c08:	e004      	b.n	8006c14 <smoltcp::socket::tcp::Socket::process+0x670>
 8006c0a:	2000      	movs	r0, #0
 8006c0c:	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
 8006c10:	f881 0133 	strb.w	r0, [r1, #307]	@ 0x133
 8006c14:	990c      	ldr	r1, [sp, #48]	@ 0x30
 8006c16:	2002      	movs	r0, #2
 8006c18:	6108      	str	r0, [r1, #16]
 8006c1a:	b011      	add	sp, #68	@ 0x44
 8006c1c:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8006c20:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8006c22:	f1b9 0f01 	cmp.w	r9, #1
 8006c26:	f000 8087 	beq.w	8006d38 <smoltcp::socket::tcp::Socket::process+0x794>
 8006c2a:	f1b9 0f02 	cmp.w	r9, #2
 8006c2e:	d199      	bne.n	8006b64 <smoltcp::socket::tcp::Socket::process+0x5c0>
 8006c30:	f8cd c010 	str.w	ip, [sp, #16]
 8006c34:	9501      	str	r5, [sp, #4]
 8006c36:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8006c3a:	f8dd a024 	ldr.w	sl, [sp, #36]	@ 0x24
 8006c3e:	f8bb 0038 	ldrh.w	r0, [fp, #56]	@ 0x38
 8006c42:	b128      	cbz	r0, 8006c50 <smoltcp::socket::tcp::Socket::process+0x6ac>
 8006c44:	f8bb 003a 	ldrh.w	r0, [fp, #58]	@ 0x3a
 8006c48:	2800      	cmp	r0, #0
 8006c4a:	d0e3      	beq.n	8006c14 <smoltcp::socket::tcp::Socket::process+0x670>
 8006c4c:	f8c1 0110 	str.w	r0, [r1, #272]	@ 0x110
 8006c50:	46b0      	mov	r8, r6
 8006c52:	f10a 0601 	add.w	r6, sl, #1
 8006c56:	f89b 0048 	ldrb.w	r0, [fp, #72]	@ 0x48
 8006c5a:	2501      	movs	r5, #1
 8006c5c:	f8c1 6104 	str.w	r6, [r1, #260]	@ 0x104
 8006c60:	f04f 0e00 	mov.w	lr, #0
 8006c64:	f8d1 6100 	ldr.w	r6, [r1, #256]	@ 0x100
 8006c68:	2800      	cmp	r0, #0
 8006c6a:	9406      	str	r4, [sp, #24]
 8006c6c:	9302      	str	r3, [sp, #8]
 8006c6e:	f89b 2049 	ldrb.w	r2, [fp, #73]	@ 0x49
 8006c72:	f89b 3050 	ldrb.w	r3, [fp, #80]	@ 0x50
 8006c76:	f881 212d 	strb.w	r2, [r1, #301]	@ 0x12d
 8006c7a:	f106 0201 	add.w	r2, r6, #1
 8006c7e:	f8c1 2108 	str.w	r2, [r1, #264]	@ 0x108
 8006c82:	f04f 0204 	mov.w	r2, #4
 8006c86:	e9c1 5a26 	strd	r5, sl, [r1, #152]	@ 0x98
 8006c8a:	f881 3131 	strb.w	r3, [r1, #305]	@ 0x131
 8006c8e:	f881 012c 	strb.w	r0, [r1, #300]	@ 0x12c
 8006c92:	bf04      	itt	eq
 8006c94:	2000      	moveq	r0, #0
 8006c96:	f881 0134 	strbeq.w	r0, [r1, #308]	@ 0x134
 8006c9a:	f8db 002c 	ldr.w	r0, [fp, #44]	@ 0x2c
 8006c9e:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006ca2:	2800      	cmp	r0, #0
 8006ca4:	bf04      	itt	eq
 8006ca6:	2000      	moveq	r0, #0
 8006ca8:	f8c1 0114 	streq.w	r0, [r1, #276]	@ 0x114
 8006cac:	9b0d      	ldr	r3, [sp, #52]	@ 0x34
 8006cae:	e9d1 4612 	ldrd	r4, r6, [r1, #72]	@ 0x48
 8006cb2:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8006cb6:	2b00      	cmp	r3, #0
 8006cb8:	e9d1 c910 	ldrd	ip, r9, [r1, #64]	@ 0x40
 8006cbc:	bf08      	it	eq
 8006cbe:	2203      	moveq	r2, #3
 8006cc0:	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
 8006cc4:	192a      	adds	r2, r5, r4
 8006cc6:	e0b0      	b.n	8006e2a <smoltcp::socket::tcp::Socket::process+0x886>
 8006cc8:	f1a9 0003 	sub.w	r0, r9, #3
 8006ccc:	2803      	cmp	r0, #3
 8006cce:	f63f af49 	bhi.w	8006b64 <smoltcp::socket::tcp::Socket::process+0x5c0>
 8006cd2:	e9cd 5301 	strd	r5, r3, [sp, #4]
 8006cd6:	f8cd c010 	str.w	ip, [sp, #16]
 8006cda:	9406      	str	r4, [sp, #24]
 8006cdc:	e8df f010 	tbh	[pc, r0, lsl #1]
 8006ce0:	00040004 	.word	0x00040004
 8006ce4:	01b10195 	.word	0x01b10195
 8006ce8:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006cec:	4634      	mov	r4, r6
 8006cee:	2607      	movs	r6, #7
 8006cf0:	9803      	ldr	r0, [sp, #12]
 8006cf2:	e9da 8e04 	ldrd	r8, lr, [sl, #16]
 8006cf6:	f04f 0c00 	mov.w	ip, #0
 8006cfa:	6c8b      	ldr	r3, [r1, #72]	@ 0x48
 8006cfc:	f881 6133 	strb.w	r6, [r1, #307]	@ 0x133
 8006d00:	2601      	movs	r6, #1
 8006d02:	6cca      	ldr	r2, [r1, #76]	@ 0x4c
 8006d04:	eb13 0308 	adds.w	r3, r3, r8
 8006d08:	f881 6130 	strb.w	r6, [r1, #304]	@ 0x130
 8006d0c:	f100 0601 	add.w	r6, r0, #1
 8006d10:	6c0d      	ldr	r5, [r1, #64]	@ 0x40
 8006d12:	eb42 020e 	adc.w	r2, r2, lr
 8006d16:	6c48      	ldr	r0, [r1, #68]	@ 0x44
 8006d18:	f8c1 6104 	str.w	r6, [r1, #260]	@ 0x104
 8006d1c:	4626      	mov	r6, r4
 8006d1e:	f8c1 c080 	str.w	ip, [r1, #128]	@ 0x80
 8006d22:	f8c1 5088 	str.w	r5, [r1, #136]	@ 0x88
 8006d26:	f8c1 3090 	str.w	r3, [r1, #144]	@ 0x90
 8006d2a:	f8c1 c084 	str.w	ip, [r1, #132]	@ 0x84
 8006d2e:	f8c1 008c 	str.w	r0, [r1, #140]	@ 0x8c
 8006d32:	f8c1 2094 	str.w	r2, [r1, #148]	@ 0x94
 8006d36:	e212      	b.n	800715e <smoltcp::socket::tcp::Socket::process+0xbba>
 8006d38:	f8cd c010 	str.w	ip, [sp, #16]
 8006d3c:	e9cd 5301 	strd	r5, r3, [sp, #4]
 8006d40:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8006d44:	e9dd 3a08 	ldrd	r3, sl, [sp, #32]
 8006d48:	f8bb 0038 	ldrh.w	r0, [fp, #56]	@ 0x38
 8006d4c:	b130      	cbz	r0, 8006d5c <smoltcp::socket::tcp::Socket::process+0x7b8>
 8006d4e:	f8bb 003a 	ldrh.w	r0, [fp, #58]	@ 0x3a
 8006d52:	2800      	cmp	r0, #0
 8006d54:	f43f af5e 	beq.w	8006c14 <smoltcp::socket::tcp::Socket::process+0x670>
 8006d58:	f8c1 0110 	str.w	r0, [r1, #272]	@ 0x110
 8006d5c:	f8dd e028 	ldr.w	lr, [sp, #40]	@ 0x28
 8006d60:	2001      	movs	r0, #1
 8006d62:	f8c1 3124 	str.w	r3, [r1, #292]	@ 0x124
 8006d66:	46b0      	mov	r8, r6
 8006d68:	9b07      	ldr	r3, [sp, #28]
 8006d6a:	f10a 0601 	add.w	r6, sl, #1
 8006d6e:	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
 8006d72:	f8c1 311e 	str.w	r3, [r1, #286]	@ 0x11e
 8006d76:	f8bb 304a 	ldrh.w	r3, [fp, #74]	@ 0x4a
 8006d7a:	e9de 0206 	ldrd	r0, r2, [lr, #24]
 8006d7e:	f8a1 3128 	strh.w	r3, [r1, #296]	@ 0x128
 8006d82:	f246 131d 	movw	r3, #24861	@ 0x611d
 8006d86:	f8bb 504c 	ldrh.w	r5, [fp, #76]	@ 0x4c
 8006d8a:	f6c3 4339 	movt	r3, #15417	@ 0x3c39
 8006d8e:	f8c1 6104 	str.w	r6, [r1, #260]	@ 0x104
 8006d92:	fb02 fc03 	mul.w	ip, r2, r3
 8006d96:	f8a1 5122 	strh.w	r5, [r1, #290]	@ 0x122
 8006d9a:	fba0 6503 	umull	r6, r5, r0, r3
 8006d9e:	f64f 43ec 	movw	r3, #64748	@ 0xfcec
 8006da2:	f6cb 332e 	movt	r3, #47918	@ 0xbb2e
 8006da6:	f89b 2050 	ldrb.w	r2, [fp, #80]	@ 0x50
 8006daa:	fb00 5003 	mla	r0, r0, r3, r5
 8006dae:	f881 2131 	strb.w	r2, [r1, #305]	@ 0x131
 8006db2:	f64e 7239 	movw	r2, #61241	@ 0xef39
 8006db6:	f2c7 5290 	movt	r2, #30096	@ 0x7590
 8006dba:	f89b 5049 	ldrb.w	r5, [fp, #73]	@ 0x49
 8006dbe:	1992      	adds	r2, r2, r6
 8006dc0:	f04f 061d 	mov.w	r6, #29
 8006dc4:	f881 512d 	strb.w	r5, [r1, #301]	@ 0x12d
 8006dc8:	eb40 000c 	adc.w	r0, r0, ip
 8006dcc:	f89b 3048 	ldrb.w	r3, [fp, #72]	@ 0x48
 8006dd0:	f881 312c 	strb.w	r3, [r1, #300]	@ 0x12c
 8006dd4:	eba6 7650 	sub.w	r6, r6, r0, lsr #29
 8006dd8:	e9ce 2006 	strd	r2, r0, [lr, #24]
 8006ddc:	f086 051f 	eor.w	r5, r6, #31
 8006de0:	0040      	lsls	r0, r0, #1
 8006de2:	40f2      	lsrs	r2, r6
 8006de4:	2b00      	cmp	r3, #0
 8006de6:	fa00 f005 	lsl.w	r0, r0, r5
 8006dea:	f04f 0e00 	mov.w	lr, #0
 8006dee:	ea40 0002 	orr.w	r0, r0, r2
 8006df2:	f8c1 0108 	str.w	r0, [r1, #264]	@ 0x108
 8006df6:	f8c1 0100 	str.w	r0, [r1, #256]	@ 0x100
 8006dfa:	bf04      	itt	eq
 8006dfc:	2000      	moveq	r0, #0
 8006dfe:	f881 0134 	strbeq.w	r0, [r1, #308]	@ 0x134
 8006e02:	f8db 002c 	ldr.w	r0, [fp, #44]	@ 0x2c
 8006e06:	2203      	movs	r2, #3
 8006e08:	9406      	str	r4, [sp, #24]
 8006e0a:	2800      	cmp	r0, #0
 8006e0c:	bf04      	itt	eq
 8006e0e:	2000      	moveq	r0, #0
 8006e10:	f8c1 0114 	streq.w	r0, [r1, #276]	@ 0x114
 8006e14:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006e18:	e9d1 3612 	ldrd	r3, r6, [r1, #72]	@ 0x48
 8006e1c:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8006e20:	e9d1 c910 	ldrd	ip, r9, [r1, #64]	@ 0x40
 8006e24:	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
 8006e28:	18ea      	adds	r2, r5, r3
 8006e2a:	4170      	adcs	r0, r6
 8006e2c:	f8c1 e080 	str.w	lr, [r1, #128]	@ 0x80
 8006e30:	f8c1 c088 	str.w	ip, [r1, #136]	@ 0x88
 8006e34:	4646      	mov	r6, r8
 8006e36:	f8c1 2090 	str.w	r2, [r1, #144]	@ 0x90
 8006e3a:	f8c1 e084 	str.w	lr, [r1, #132]	@ 0x84
 8006e3e:	f8c1 908c 	str.w	r9, [r1, #140]	@ 0x8c
 8006e42:	f8c1 0094 	str.w	r0, [r1, #148]	@ 0x94
 8006e46:	e18c      	b.n	8007162 <smoltcp::socket::tcp::Socket::process+0xbbe>
 8006e48:	f240 0038 	movw	r0, #56	@ 0x38
 8006e4c:	f2c0 0000 	movt	r0, #0
 8006e50:	e008      	b.n	8006e64 <smoltcp::socket::tcp::Socket::process+0x8c0>
 8006e52:	f240 003a 	movw	r0, #58	@ 0x3a
 8006e56:	f2c0 0000 	movt	r0, #0
 8006e5a:	e003      	b.n	8006e64 <smoltcp::socket::tcp::Socket::process+0x8c0>
 8006e5c:	f240 0034 	movw	r0, #52	@ 0x34
 8006e60:	f2c0 0000 	movt	r0, #0
 8006e64:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006e68:	a810      	add	r0, sp, #64	@ 0x40
 8006e6a:	2102      	movs	r1, #2
 8006e6c:	f7fd fca4 	bl	80047b8 <_defmt_write>
 8006e70:	f240 0039 	movw	r0, #57	@ 0x39
 8006e74:	2102      	movs	r1, #2
 8006e76:	f2c0 0000 	movt	r0, #0
 8006e7a:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006e7e:	a810      	add	r0, sp, #64	@ 0x40
 8006e80:	f7fd fc9a 	bl	80047b8 <_defmt_write>
 8006e84:	a810      	add	r0, sp, #64	@ 0x40
 8006e86:	2102      	movs	r1, #2
 8006e88:	f8ad 9040 	strh.w	r9, [sp, #64]	@ 0x40
 8006e8c:	f7fd fc94 	bl	80047b8 <_defmt_write>
 8006e90:	f240 0b3d 	movw	fp, #61	@ 0x3d
 8006e94:	a810      	add	r0, sp, #64	@ 0x40
 8006e96:	f2c0 0b00 	movt	fp, #0
 8006e9a:	2102      	movs	r1, #2
 8006e9c:	f8ad b040 	strh.w	fp, [sp, #64]	@ 0x40
 8006ea0:	f7fd fc8a 	bl	80047b8 <_defmt_write>
 8006ea4:	f240 0508 	movw	r5, #8
 8006ea8:	a810      	add	r0, sp, #64	@ 0x40
 8006eaa:	f2c0 0500 	movt	r5, #0
 8006eae:	2102      	movs	r1, #2
 8006eb0:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006eb4:	f7fd fc80 	bl	80047b8 <_defmt_write>
 8006eb8:	9809      	ldr	r0, [sp, #36]	@ 0x24
 8006eba:	2104      	movs	r1, #4
 8006ebc:	9010      	str	r0, [sp, #64]	@ 0x40
 8006ebe:	a810      	add	r0, sp, #64	@ 0x40
 8006ec0:	f7fd fc7a 	bl	80047b8 <_defmt_write>
 8006ec4:	a810      	add	r0, sp, #64	@ 0x40
 8006ec6:	2600      	movs	r6, #0
 8006ec8:	2102      	movs	r1, #2
 8006eca:	f8ad 6040 	strh.w	r6, [sp, #64]	@ 0x40
 8006ece:	f7fd fc73 	bl	80047b8 <_defmt_write>
 8006ed2:	f8dd 8030 	ldr.w	r8, [sp, #48]	@ 0x30
 8006ed6:	980d      	ldr	r0, [sp, #52]	@ 0x34
 8006ed8:	b338      	cbz	r0, 8006f2a <smoltcp::socket::tcp::Socket::process+0x986>
 8006eda:	f240 0033 	movw	r0, #51	@ 0x33
 8006ede:	2102      	movs	r1, #2
 8006ee0:	f2c0 0000 	movt	r0, #0
 8006ee4:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006ee8:	a810      	add	r0, sp, #64	@ 0x40
 8006eea:	f7fd fc65 	bl	80047b8 <_defmt_write>
 8006eee:	a810      	add	r0, sp, #64	@ 0x40
 8006ef0:	2102      	movs	r1, #2
 8006ef2:	f8ad 9040 	strh.w	r9, [sp, #64]	@ 0x40
 8006ef6:	f7fd fc5f 	bl	80047b8 <_defmt_write>
 8006efa:	a810      	add	r0, sp, #64	@ 0x40
 8006efc:	2102      	movs	r1, #2
 8006efe:	f8ad b040 	strh.w	fp, [sp, #64]	@ 0x40
 8006f02:	f7fd fc59 	bl	80047b8 <_defmt_write>
 8006f06:	a810      	add	r0, sp, #64	@ 0x40
 8006f08:	2102      	movs	r1, #2
 8006f0a:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006f0e:	f7fd fc53 	bl	80047b8 <_defmt_write>
 8006f12:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 8006f14:	2104      	movs	r1, #4
 8006f16:	9010      	str	r0, [sp, #64]	@ 0x40
 8006f18:	a810      	add	r0, sp, #64	@ 0x40
 8006f1a:	f7fd fc4d 	bl	80047b8 <_defmt_write>
 8006f1e:	a810      	add	r0, sp, #64	@ 0x40
 8006f20:	2102      	movs	r1, #2
 8006f22:	f8ad 6040 	strh.w	r6, [sp, #64]	@ 0x40
 8006f26:	f7fd fc47 	bl	80047b8 <_defmt_write>
 8006f2a:	f240 003b 	movw	r0, #59	@ 0x3b
 8006f2e:	2102      	movs	r1, #2
 8006f30:	f2c0 0000 	movt	r0, #0
 8006f34:	2602      	movs	r6, #2
 8006f36:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006f3a:	a810      	add	r0, sp, #64	@ 0x40
 8006f3c:	f7fd fc3c 	bl	80047b8 <_defmt_write>
 8006f40:	a810      	add	r0, sp, #64	@ 0x40
 8006f42:	2102      	movs	r1, #2
 8006f44:	f8b4 504e 	ldrh.w	r5, [r4, #78]	@ 0x4e
 8006f48:	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
 8006f4c:	f7fd fc34 	bl	80047b8 <_defmt_write>
 8006f50:	a810      	add	r0, sp, #64	@ 0x40
 8006f52:	2102      	movs	r1, #2
 8006f54:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006f58:	f7fd fc2e 	bl	80047b8 <_defmt_write>
 8006f5c:	f240 0035 	movw	r0, #53	@ 0x35
 8006f60:	2102      	movs	r1, #2
 8006f62:	f2c0 0000 	movt	r0, #0
 8006f66:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006f6a:	a810      	add	r0, sp, #64	@ 0x40
 8006f6c:	f7fd fc24 	bl	80047b8 <_defmt_write>
 8006f70:	f240 0006 	movw	r0, #6
 8006f74:	2102      	movs	r1, #2
 8006f76:	f2c0 0000 	movt	r0, #0
 8006f7a:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006f7e:	a810      	add	r0, sp, #64	@ 0x40
 8006f80:	f7fd fc1a 	bl	80047b8 <_defmt_write>
 8006f84:	980b      	ldr	r0, [sp, #44]	@ 0x2c
 8006f86:	2104      	movs	r1, #4
 8006f88:	9010      	str	r0, [sp, #64]	@ 0x40
 8006f8a:	a810      	add	r0, sp, #64	@ 0x40
 8006f8c:	f7fd fc14 	bl	80047b8 <_defmt_write>
 8006f90:	8f20      	ldrh	r0, [r4, #56]	@ 0x38
 8006f92:	2801      	cmp	r0, #1
 8006f94:	d116      	bne.n	8006fc4 <smoltcp::socket::tcp::Socket::process+0xa20>
 8006f96:	f240 0036 	movw	r0, #54	@ 0x36
 8006f9a:	2102      	movs	r1, #2
 8006f9c:	f2c0 0000 	movt	r0, #0
 8006fa0:	8f65      	ldrh	r5, [r4, #58]	@ 0x3a
 8006fa2:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006fa6:	a810      	add	r0, sp, #64	@ 0x40
 8006fa8:	f7fd fc06 	bl	80047b8 <_defmt_write>
 8006fac:	a810      	add	r0, sp, #64	@ 0x40
 8006fae:	2102      	movs	r1, #2
 8006fb0:	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
 8006fb4:	f7fd fc00 	bl	80047b8 <_defmt_write>
 8006fb8:	a810      	add	r0, sp, #64	@ 0x40
 8006fba:	2102      	movs	r1, #2
 8006fbc:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006fc0:	f7fd fbfa 	bl	80047b8 <_defmt_write>
 8006fc4:	2000      	movs	r0, #0
 8006fc6:	2102      	movs	r1, #2
 8006fc8:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006fcc:	a810      	add	r0, sp, #64	@ 0x40
 8006fce:	f7fd fbf3 	bl	80047b8 <_defmt_write>
 8006fd2:	f7fd fb79 	bl	80046c8 <_defmt_release>
 8006fd6:	f8c8 6010 	str.w	r6, [r8, #16]
 8006fda:	b011      	add	sp, #68	@ 0x44
 8006fdc:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8006fe0:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8006fe2:	f240 000c 	movw	r0, #12
 8006fe6:	4665      	mov	r5, ip
 8006fe8:	f2c0 0000 	movt	r0, #0
 8006fec:	f7fd fa32 	bl	8004454 <defmt::export::acquire_and_header>
 8006ff0:	4628      	mov	r0, r5
 8006ff2:	f000 ff40 	bl	8007e76 <defmt::export::fmt>
 8006ff6:	4640      	mov	r0, r8
 8006ff8:	f000 ff3d 	bl	8007e76 <defmt::export::fmt>
 8006ffc:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 8006ffe:	f000 ff3a 	bl	8007e76 <defmt::export::fmt>
 8007002:	f7fd fb61 	bl	80046c8 <_defmt_release>
 8007006:	f7ff bc01 	b.w	800680c <smoltcp::socket::tcp::Socket::process+0x268>
 800700a:	2001      	movs	r0, #1
 800700c:	f1b8 0f00 	cmp.w	r8, #0
 8007010:	f881 0130 	strb.w	r0, [r1, #304]	@ 0x130
 8007014:	9803      	ldr	r0, [sp, #12]
 8007016:	f100 0001 	add.w	r0, r0, #1
 800701a:	f8c1 0104 	str.w	r0, [r1, #260]	@ 0x104
 800701e:	f040 8086 	bne.w	800712e <smoltcp::socket::tcp::Socket::process+0xb8a>
 8007022:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8007026:	2208      	movs	r2, #8
 8007028:	e9d1 3412 	ldrd	r3, r4, [r1, #72]	@ 0x48
 800702c:	f04f 0e00 	mov.w	lr, #0
 8007030:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8007034:	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
 8007038:	18ea      	adds	r2, r5, r3
 800703a:	e9d1 c810 	ldrd	ip, r8, [r1, #64]	@ 0x40
 800703e:	4160      	adcs	r0, r4
 8007040:	e041      	b.n	80070c6 <smoltcp::socket::tcp::Socket::process+0xb22>
 8007042:	250a      	movs	r5, #10
 8007044:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8007048:	f881 5133 	strb.w	r5, [r1, #307]	@ 0x133
 800704c:	2501      	movs	r5, #1
 800704e:	f881 5130 	strb.w	r5, [r1, #304]	@ 0x130
 8007052:	2000      	movs	r0, #0
 8007054:	9d03      	ldr	r5, [sp, #12]
 8007056:	2403      	movs	r4, #3
 8007058:	e9da 2304 	ldrd	r2, r3, [sl, #16]
 800705c:	3501      	adds	r5, #1
 800705e:	f8c1 5104 	str.w	r5, [r1, #260]	@ 0x104
 8007062:	e06d      	b.n	8007140 <smoltcp::socket::tcp::Socket::process+0xb9c>
 8007064:	f1b8 0f00 	cmp.w	r8, #0
 8007068:	bf1c      	itt	ne
 800706a:	2006      	movne	r0, #6
 800706c:	f881 0133 	strbne.w	r0, [r1, #307]	@ 0x133
 8007070:	f1bb 0f00 	cmp.w	fp, #0
 8007074:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8007078:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 800707c:	d071      	beq.n	8007162 <smoltcp::socket::tcp::Socket::process+0xbbe>
 800707e:	6c8b      	ldr	r3, [r1, #72]	@ 0x48
 8007080:	4634      	mov	r4, r6
 8007082:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8007086:	f04f 0e00 	mov.w	lr, #0
 800708a:	6cce      	ldr	r6, [r1, #76]	@ 0x4c
 800708c:	195b      	adds	r3, r3, r5
 800708e:	e9d1 c210 	ldrd	ip, r2, [r1, #64]	@ 0x40
 8007092:	4170      	adcs	r0, r6
 8007094:	4626      	mov	r6, r4
 8007096:	e9c1 ee20 	strd	lr, lr, [r1, #128]	@ 0x80
 800709a:	e9c1 c222 	strd	ip, r2, [r1, #136]	@ 0x88
 800709e:	e9c1 3024 	strd	r3, r0, [r1, #144]	@ 0x90
 80070a2:	e05e      	b.n	8007162 <smoltcp::socket::tcp::Socket::process+0xbbe>
 80070a4:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 80070a8:	4634      	mov	r4, r6
 80070aa:	6c8b      	ldr	r3, [r1, #72]	@ 0x48
 80070ac:	2204      	movs	r2, #4
 80070ae:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 80070b2:	f04f 0e00 	mov.w	lr, #0
 80070b6:	6cce      	ldr	r6, [r1, #76]	@ 0x4c
 80070b8:	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
 80070bc:	18ea      	adds	r2, r5, r3
 80070be:	4170      	adcs	r0, r6
 80070c0:	e9d1 c810 	ldrd	ip, r8, [r1, #64]	@ 0x40
 80070c4:	4626      	mov	r6, r4
 80070c6:	f8c1 e080 	str.w	lr, [r1, #128]	@ 0x80
 80070ca:	f8c1 c088 	str.w	ip, [r1, #136]	@ 0x88
 80070ce:	f8c1 e084 	str.w	lr, [r1, #132]	@ 0x84
 80070d2:	f8c1 808c 	str.w	r8, [r1, #140]	@ 0x8c
 80070d6:	f8c1 2090 	str.w	r2, [r1, #144]	@ 0x90
 80070da:	f8c1 0094 	str.w	r0, [r1, #148]	@ 0x94
 80070de:	e03e      	b.n	800715e <smoltcp::socket::tcp::Socket::process+0xbba>
 80070e0:	f8d1 0080 	ldr.w	r0, [r1, #128]	@ 0x80
 80070e4:	f04f 0c00 	mov.w	ip, #0
 80070e8:	1ec2      	subs	r2, r0, #3
 80070ea:	f112 0f02 	cmn.w	r2, #2
 80070ee:	f04f 0200 	mov.w	r2, #0
 80070f2:	bf38      	it	cc
 80070f4:	2201      	movcc	r2, #1
 80070f6:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 80070fa:	ea52 020b 	orrs.w	r2, r2, fp
 80070fe:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8007102:	d02e      	beq.n	8007162 <smoltcp::socket::tcp::Socket::process+0xbbe>
 8007104:	4634      	mov	r4, r6
 8007106:	e9d1 6512 	ldrd	r6, r5, [r1, #72]	@ 0x48
 800710a:	e9da 0204 	ldrd	r0, r2, [sl, #16]
 800710e:	e9d1 e310 	ldrd	lr, r3, [r1, #64]	@ 0x40
 8007112:	1980      	adds	r0, r0, r6
 8007114:	4626      	mov	r6, r4
 8007116:	416a      	adcs	r2, r5
 8007118:	e9c1 cc20 	strd	ip, ip, [r1, #128]	@ 0x80
 800711c:	e9c1 e322 	strd	lr, r3, [r1, #136]	@ 0x88
 8007120:	e9c1 0224 	strd	r0, r2, [r1, #144]	@ 0x90
 8007124:	e01d      	b.n	8007162 <smoltcp::socket::tcp::Socket::process+0xbbe>
 8007126:	f1b8 0f00 	cmp.w	r8, #0
 800712a:	f000 813d 	beq.w	80073a8 <smoltcp::socket::tcp::Socket::process+0xe04>
 800712e:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8007132:	250a      	movs	r5, #10
 8007134:	2000      	movs	r0, #0
 8007136:	2403      	movs	r4, #3
 8007138:	e9da 2304 	ldrd	r2, r3, [sl, #16]
 800713c:	f881 5133 	strb.w	r5, [r1, #307]	@ 0x133
 8007140:	f249 6580 	movw	r5, #38528	@ 0x9680
 8007144:	f8c1 4080 	str.w	r4, [r1, #128]	@ 0x80
 8007148:	f2c0 0598 	movt	r5, #152	@ 0x98
 800714c:	1952      	adds	r2, r2, r5
 800714e:	f143 0300 	adc.w	r3, r3, #0
 8007152:	f8c1 2088 	str.w	r2, [r1, #136]	@ 0x88
 8007156:	f8c1 0084 	str.w	r0, [r1, #132]	@ 0x84
 800715a:	f8c1 308c 	str.w	r3, [r1, #140]	@ 0x8c
 800715e:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8007162:	e9da 3904 	ldrd	r3, r9, [sl, #16]
 8007166:	2200      	movs	r2, #0
 8007168:	2001      	movs	r0, #1
 800716a:	2e02      	cmp	r6, #2
 800716c:	e9c1 0214 	strd	r0, r2, [r1, #80]	@ 0x50
 8007170:	930a      	str	r3, [sp, #40]	@ 0x28
 8007172:	e9c1 3916 	strd	r3, r9, [r1, #88]	@ 0x58
 8007176:	d007      	beq.n	8007188 <smoltcp::socket::tcp::Socket::process+0xbe4>
 8007178:	f891 212c 	ldrb.w	r2, [r1, #300]	@ 0x12c
 800717c:	f891 012d 	ldrb.w	r0, [r1, #301]	@ 0x12d
 8007180:	2a00      	cmp	r2, #0
 8007182:	bf18      	it	ne
 8007184:	f000 021f 	andne.w	r2, r0, #31
 8007188:	f8bb 304e 	ldrh.w	r3, [fp, #78]	@ 0x4e
 800718c:	9d06      	ldr	r5, [sp, #24]
 800718e:	f8d1 010c 	ldr.w	r0, [r1, #268]	@ 0x10c
 8007192:	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
 8007196:	fa03 f202 	lsl.w	r2, r3, r2
 800719a:	9c04      	ldr	r4, [sp, #16]
 800719c:	f8c1 210c 	str.w	r2, [r1, #268]	@ 0x10c
 80071a0:	b1ad      	cbz	r5, 80071ce <smoltcp::socket::tcp::Socket::process+0xc2a>
 80071a2:	f8d1 30d4 	ldr.w	r3, [r1, #212]	@ 0xd4
 80071a6:	42ab      	cmp	r3, r5
 80071a8:	f0c0 82dc 	bcc.w	8007764 <smoltcp::socket::tcp::Socket::process+0x11c0>
 80071ac:	1b5e      	subs	r6, r3, r5
 80071ae:	f8d1 30cc 	ldr.w	r3, [r1, #204]	@ 0xcc
 80071b2:	f8c1 60d4 	str.w	r6, [r1, #212]	@ 0xd4
 80071b6:	b13b      	cbz	r3, 80071c8 <smoltcp::socket::tcp::Socket::process+0xc24>
 80071b8:	f8d1 60d0 	ldr.w	r6, [r1, #208]	@ 0xd0
 80071bc:	442e      	add	r6, r5
 80071be:	fbb6 f5f3 	udiv	r5, r6, r3
 80071c2:	fb05 6313 	mls	r3, r5, r3, r6
 80071c6:	e000      	b.n	80071ca <smoltcp::socket::tcp::Socket::process+0xc26>
 80071c8:	2300      	movs	r3, #0
 80071ca:	f8c1 30d0 	str.w	r3, [r1, #208]	@ 0xd0
 80071ce:	9b0d      	ldr	r3, [sp, #52]	@ 0x34
 80071d0:	2b00      	cmp	r3, #0
 80071d2:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 80071d4:	d06e      	beq.n	80072b4 <smoltcp::socket::tcp::Socket::process+0xd10>
 80071d6:	9e0b      	ldr	r6, [sp, #44]	@ 0x2c
 80071d8:	2e00      	cmp	r6, #0
 80071da:	d14f      	bne.n	800727c <smoltcp::socket::tcp::Socket::process+0xcd8>
 80071dc:	f8d1 30a8 	ldr.w	r3, [r1, #168]	@ 0xa8
 80071e0:	2b00      	cmp	r3, #0
 80071e2:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 80071e4:	d04a      	beq.n	800727c <smoltcp::socket::tcp::Socket::process+0xcd8>
 80071e6:	f8d1 30ac 	ldr.w	r3, [r1, #172]	@ 0xac
 80071ea:	9e0f      	ldr	r6, [sp, #60]	@ 0x3c
 80071ec:	42b3      	cmp	r3, r6
 80071ee:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 80071f0:	bf08      	it	eq
 80071f2:	4282      	cmpeq	r2, r0
 80071f4:	d142      	bne.n	800727c <smoltcp::socket::tcp::Socket::process+0xcd8>
 80071f6:	f8d1 0108 	ldr.w	r0, [r1, #264]	@ 0x108
 80071fa:	1a18      	subs	r0, r3, r0
 80071fc:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8007200:	dc3c      	bgt.n	800727c <smoltcp::socket::tcp::Socket::process+0xcd8>
 8007202:	f991 0135 	ldrsb.w	r0, [r1, #309]	@ 0x135
 8007206:	f04f 0801 	mov.w	r8, #1
 800720a:	4625      	mov	r5, r4
 800720c:	460c      	mov	r4, r1
 800720e:	fa80 f658 	uqadd8	r6, r0, r8
 8007212:	f240 0013 	movw	r0, #19
 8007216:	f2c0 0000 	movt	r0, #0
 800721a:	f881 6135 	strb.w	r6, [r1, #309]	@ 0x135
 800721e:	f7fd f919 	bl	8004454 <defmt::export::acquire_and_header>
 8007222:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 8007224:	f000 fe27 	bl	8007e76 <defmt::export::fmt>
 8007228:	f894 0135 	ldrb.w	r0, [r4, #309]	@ 0x135
 800722c:	f000 fe0c 	bl	8007e48 <defmt::export::fmt>
 8007230:	b2f1      	uxtb	r1, r6
 8007232:	29ff      	cmp	r1, #255	@ 0xff
 8007234:	f06f 01fe 	mvn.w	r1, #254	@ 0xfe
 8007238:	f64a 5060 	movw	r0, #44384	@ 0xad60
 800723c:	fa51 f186 	uxtab	r1, r1, r6
 8007240:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8007244:	fab1 f181 	clz	r1, r1
 8007248:	bf18      	it	ne
 800724a:	4640      	movne	r0, r8
 800724c:	0949      	lsrs	r1, r1, #5
 800724e:	f000 fde0 	bl	8007e12 <defmt::export::fmt>
 8007252:	f7fd fa39 	bl	80046c8 <_defmt_release>
 8007256:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 8007258:	4621      	mov	r1, r4
 800725a:	f894 0135 	ldrb.w	r0, [r4, #309]	@ 0x135
 800725e:	2803      	cmp	r0, #3
 8007260:	d11f      	bne.n	80072a2 <smoltcp::socket::tcp::Socket::process+0xcfe>
 8007262:	2000      	movs	r0, #0
 8007264:	2202      	movs	r2, #2
 8007266:	e9c1 2020 	strd	r2, r0, [r1, #128]	@ 0x80
 800726a:	f240 0018 	movw	r0, #24
 800726e:	f2c0 0000 	movt	r0, #0
 8007272:	f7fd f900 	bl	8004476 <defmt::export::acquire_header_and_release>
 8007276:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 8007278:	4621      	mov	r1, r4
 800727a:	e012      	b.n	80072a2 <smoltcp::socket::tcp::Socket::process+0xcfe>
 800727c:	f891 0135 	ldrb.w	r0, [r1, #309]	@ 0x135
 8007280:	4625      	mov	r5, r4
 8007282:	b158      	cbz	r0, 800729c <smoltcp::socket::tcp::Socket::process+0xcf8>
 8007284:	2000      	movs	r0, #0
 8007286:	460c      	mov	r4, r1
 8007288:	f881 0135 	strb.w	r0, [r1, #309]	@ 0x135
 800728c:	f240 0014 	movw	r0, #20
 8007290:	f2c0 0000 	movt	r0, #0
 8007294:	f7fd f8ef 	bl	8004476 <defmt::export::acquire_header_and_release>
 8007298:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 800729a:	4621      	mov	r1, r4
 800729c:	2001      	movs	r0, #1
 800729e:	e9c1 032a 	strd	r0, r3, [r1, #168]	@ 0xa8
 80072a2:	f8d1 0108 	ldr.w	r0, [r1, #264]	@ 0x108
 80072a6:	462c      	mov	r4, r5
 80072a8:	f8c1 3100 	str.w	r3, [r1, #256]	@ 0x100
 80072ac:	1ac0      	subs	r0, r0, r3
 80072ae:	bf48      	it	mi
 80072b0:	f8c1 3108 	strmi.w	r3, [r1, #264]	@ 0x108
 80072b4:	980e      	ldr	r0, [sp, #56]	@ 0x38
 80072b6:	9b02      	ldr	r3, [sp, #8]
 80072b8:	6ac0      	ldr	r0, [r0, #44]	@ 0x2c
 80072ba:	2800      	cmp	r0, #0
 80072bc:	bf1e      	ittt	ne
 80072be:	980e      	ldrne	r0, [sp, #56]	@ 0x38
 80072c0:	6b00      	ldrne	r0, [r0, #48]	@ 0x30
 80072c2:	f8c1 0118 	strne.w	r0, [r1, #280]	@ 0x118
 80072c6:	2c00      	cmp	r4, #0
 80072c8:	f43f aaa0 	beq.w	800680c <smoltcp::socket::tcp::Socket::process+0x268>
 80072cc:	4688      	mov	r8, r1
 80072ce:	9404      	str	r4, [sp, #16]
 80072d0:	f858 6fd8 	ldr.w	r6, [r8, #216]!
 80072d4:	f8cd 9024 	str.w	r9, [sp, #36]	@ 0x24
 80072d8:	f8d8 0004 	ldr.w	r0, [r8, #4]
 80072dc:	900f      	str	r0, [sp, #60]	@ 0x3c
 80072de:	b933      	cbnz	r3, 80072ee <smoltcp::socket::tcp::Socket::process+0xd4a>
 80072e0:	42a6      	cmp	r6, r4
 80072e2:	d904      	bls.n	80072ee <smoltcp::socket::tcp::Socket::process+0xd4a>
 80072e4:	46a4      	mov	ip, r4
 80072e6:	1b30      	subs	r0, r6, r4
 80072e8:	f8c8 0000 	str.w	r0, [r8]
 80072ec:	e15a      	b.n	80075a4 <smoltcp::socket::tcp::Socket::process+0x1000>
 80072ee:	b398      	cbz	r0, 8007358 <smoltcp::socket::tcp::Socket::process+0xdb4>
 80072f0:	eb06 0e00 	add.w	lr, r6, r0
 80072f4:	4573      	cmp	r3, lr
 80072f6:	d934      	bls.n	8007362 <smoltcp::socket::tcp::Socket::process+0xdbe>
 80072f8:	f8d1 50e4 	ldr.w	r5, [r1, #228]	@ 0xe4
 80072fc:	eba3 090e 	sub.w	r9, r3, lr
 8007300:	f101 0be0 	add.w	fp, r1, #224	@ 0xe0
 8007304:	b355      	cbz	r5, 800735c <smoltcp::socket::tcp::Socket::process+0xdb8>
 8007306:	f8db 3000 	ldr.w	r3, [fp]
 800730a:	eb03 0c05 	add.w	ip, r3, r5
 800730e:	45e1      	cmp	r9, ip
 8007310:	d942      	bls.n	8007398 <smoltcp::socket::tcp::Socket::process+0xdf4>
 8007312:	f8d1 50ec 	ldr.w	r5, [r1, #236]	@ 0xec
 8007316:	eba9 090c 	sub.w	r9, r9, ip
 800731a:	f101 0be8 	add.w	fp, r1, #232	@ 0xe8
 800731e:	2d00      	cmp	r5, #0
 8007320:	d050      	beq.n	80073c4 <smoltcp::socket::tcp::Socket::process+0xe20>
 8007322:	f8db 3000 	ldr.w	r3, [fp]
 8007326:	eb03 0c05 	add.w	ip, r3, r5
 800732a:	45e1      	cmp	r9, ip
 800732c:	d94e      	bls.n	80073cc <smoltcp::socket::tcp::Socket::process+0xe28>
 800732e:	f8d1 50f4 	ldr.w	r5, [r1, #244]	@ 0xf4
 8007332:	eba9 090c 	sub.w	r9, r9, ip
 8007336:	f101 0bf0 	add.w	fp, r1, #240	@ 0xf0
 800733a:	2d00      	cmp	r5, #0
 800733c:	d042      	beq.n	80073c4 <smoltcp::socket::tcp::Socket::process+0xe20>
 800733e:	f8db 3000 	ldr.w	r3, [fp]
 8007342:	eb03 0c05 	add.w	ip, r3, r5
 8007346:	45e1      	cmp	r9, ip
 8007348:	d853      	bhi.n	80073f2 <smoltcp::socket::tcp::Socket::process+0xe4e>
 800734a:	2203      	movs	r2, #3
 800734c:	920d      	str	r2, [sp, #52]	@ 0x34
 800734e:	2201      	movs	r2, #1
 8007350:	9206      	str	r2, [sp, #24]
 8007352:	2200      	movs	r2, #0
 8007354:	920b      	str	r2, [sp, #44]	@ 0x2c
 8007356:	e040      	b.n	80073da <smoltcp::socket::tcp::Socket::process+0xe36>
 8007358:	4699      	mov	r9, r3
 800735a:	46c3      	mov	fp, r8
 800735c:	e9cb 9400 	strd	r9, r4, [fp]
 8007360:	e0fb      	b.n	800755a <smoltcp::socket::tcp::Socket::process+0xfb6>
 8007362:	2200      	movs	r2, #0
 8007364:	4699      	mov	r9, r3
 8007366:	9206      	str	r2, [sp, #24]
 8007368:	2201      	movs	r2, #1
 800736a:	920b      	str	r2, [sp, #44]	@ 0x2c
 800736c:	4605      	mov	r5, r0
 800736e:	9205      	str	r2, [sp, #20]
 8007370:	2200      	movs	r2, #0
 8007372:	4633      	mov	r3, r6
 8007374:	46f4      	mov	ip, lr
 8007376:	920d      	str	r2, [sp, #52]	@ 0x34
 8007378:	46c3      	mov	fp, r8
 800737a:	e02f      	b.n	80073dc <smoltcp::socket::tcp::Socket::process+0xe38>
 800737c:	f1b8 0f00 	cmp.w	r8, #0
 8007380:	f43f abce 	beq.w	8006b20 <smoltcp::socket::tcp::Socket::process+0x57c>
 8007384:	2000      	movs	r0, #0
 8007386:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 800738a:	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
 800738e:	f881 0133 	strb.w	r0, [r1, #307]	@ 0x133
 8007392:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8007396:	e6e4      	b.n	8007162 <smoltcp::socket::tcp::Socket::process+0xbbe>
 8007398:	2201      	movs	r2, #1
 800739a:	920b      	str	r2, [sp, #44]	@ 0x2c
 800739c:	2200      	movs	r2, #0
 800739e:	9206      	str	r2, [sp, #24]
 80073a0:	2201      	movs	r2, #1
 80073a2:	9205      	str	r2, [sp, #20]
 80073a4:	920d      	str	r2, [sp, #52]	@ 0x34
 80073a6:	e019      	b.n	80073dc <smoltcp::socket::tcp::Socket::process+0xe38>
 80073a8:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 80073ac:	f04f 0e00 	mov.w	lr, #0
 80073b0:	e9d1 3412 	ldrd	r3, r4, [r1, #72]	@ 0x48
 80073b4:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 80073b8:	e9d1 c210 	ldrd	ip, r2, [r1, #64]	@ 0x40
 80073bc:	195b      	adds	r3, r3, r5
 80073be:	4160      	adcs	r0, r4
 80073c0:	f7ff bbbd 	b.w	8006b3e <smoltcp::socket::tcp::Socket::process+0x59a>
 80073c4:	9b02      	ldr	r3, [sp, #8]
 80073c6:	e9cb 9400 	strd	r9, r4, [fp]
 80073ca:	e0c6      	b.n	800755a <smoltcp::socket::tcp::Socket::process+0xfb6>
 80073cc:	2202      	movs	r2, #2
 80073ce:	920d      	str	r2, [sp, #52]	@ 0x34
 80073d0:	2200      	movs	r2, #0
 80073d2:	9206      	str	r2, [sp, #24]
 80073d4:	2201      	movs	r2, #1
 80073d6:	920b      	str	r2, [sp, #44]	@ 0x2c
 80073d8:	2200      	movs	r2, #0
 80073da:	9205      	str	r2, [sp, #20]
 80073dc:	eb09 0a04 	add.w	sl, r9, r4
 80073e0:	4599      	cmp	r9, r3
 80073e2:	d231      	bcs.n	8007448 <smoltcp::socket::tcp::Socket::process+0xea4>
 80073e4:	459a      	cmp	sl, r3
 80073e6:	d231      	bcs.n	800744c <smoltcp::socket::tcp::Socket::process+0xea8>
 80073e8:	f8d1 00f4 	ldr.w	r0, [r1, #244]	@ 0xf4
 80073ec:	2800      	cmp	r0, #0
 80073ee:	f000 8096 	beq.w	800751e <smoltcp::socket::tcp::Socket::process+0xf7a>
 80073f2:	f240 000b 	movw	r0, #11
 80073f6:	f2c0 0000 	movt	r0, #0
 80073fa:	f7fd f82b 	bl	8004454 <defmt::export::acquire_and_header>
 80073fe:	f240 0506 	movw	r5, #6
 8007402:	a810      	add	r0, sp, #64	@ 0x40
 8007404:	f2c0 0500 	movt	r5, #0
 8007408:	2102      	movs	r1, #2
 800740a:	4626      	mov	r6, r4
 800740c:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8007410:	2402      	movs	r4, #2
 8007412:	f7fd f9d1 	bl	80047b8 <_defmt_write>
 8007416:	a810      	add	r0, sp, #64	@ 0x40
 8007418:	2104      	movs	r1, #4
 800741a:	9610      	str	r6, [sp, #64]	@ 0x40
 800741c:	f7fd f9cc 	bl	80047b8 <_defmt_write>
 8007420:	a810      	add	r0, sp, #64	@ 0x40
 8007422:	2102      	movs	r1, #2
 8007424:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8007428:	f7fd f9c6 	bl	80047b8 <_defmt_write>
 800742c:	9802      	ldr	r0, [sp, #8]
 800742e:	2104      	movs	r1, #4
 8007430:	9010      	str	r0, [sp, #64]	@ 0x40
 8007432:	a810      	add	r0, sp, #64	@ 0x40
 8007434:	f7fd f9c0 	bl	80047b8 <_defmt_write>
 8007438:	f7fd f946 	bl	80046c8 <_defmt_release>
 800743c:	980c      	ldr	r0, [sp, #48]	@ 0x30
 800743e:	6104      	str	r4, [r0, #16]
 8007440:	b011      	add	sp, #68	@ 0x44
 8007442:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007446:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007448:	4699      	mov	r9, r3
 800744a:	e003      	b.n	8007454 <smoltcp::socket::tcp::Socket::process+0xeb0>
 800744c:	ebac 0509 	sub.w	r5, ip, r9
 8007450:	e9cb 9500 	strd	r9, r5, [fp]
 8007454:	f8dd e034 	ldr.w	lr, [sp, #52]	@ 0x34
 8007458:	980b      	ldr	r0, [sp, #44]	@ 0x2c
 800745a:	f10e 0c01 	add.w	ip, lr, #1
 800745e:	b1b0      	cbz	r0, 800748e <smoltcp::socket::tcp::Socket::process+0xeea>
 8007460:	4672      	mov	r2, lr
 8007462:	4666      	mov	r6, ip
 8007464:	eb01 03c2 	add.w	r3, r1, r2, lsl #3
 8007468:	f8d3 40e4 	ldr.w	r4, [r3, #228]	@ 0xe4
 800746c:	b18c      	cbz	r4, 8007492 <smoltcp::socket::tcp::Socket::process+0xeee>
 800746e:	f8d3 30e0 	ldr.w	r3, [r3, #224]	@ 0xe0
 8007472:	442b      	add	r3, r5
 8007474:	eb03 0009 	add.w	r0, r3, r9
 8007478:	4582      	cmp	sl, r0
 800747a:	d30b      	bcc.n	8007494 <smoltcp::socket::tcp::Socket::process+0xef0>
 800747c:	191d      	adds	r5, r3, r4
 800747e:	3201      	adds	r2, #1
 8007480:	3601      	adds	r6, #1
 8007482:	2a03      	cmp	r2, #3
 8007484:	f8cb 5004 	str.w	r5, [fp, #4]
 8007488:	d1ec      	bne.n	8007464 <smoltcp::socket::tcp::Socket::process+0xec0>
 800748a:	2604      	movs	r6, #4
 800748c:	e002      	b.n	8007494 <smoltcp::socket::tcp::Socket::process+0xef0>
 800748e:	9b02      	ldr	r3, [sp, #8]
 8007490:	e02e      	b.n	80074f0 <smoltcp::socket::tcp::Socket::process+0xf4c>
 8007492:	1c56      	adds	r6, r2, #1
 8007494:	9b02      	ldr	r3, [sp, #8]
 8007496:	ea6f 020e 	mvn.w	r2, lr
 800749a:	1992      	adds	r2, r2, r6
 800749c:	d028      	beq.n	80074f0 <smoltcp::socket::tcp::Socket::process+0xf4c>
 800749e:	f8cd c02c 	str.w	ip, [sp, #44]	@ 0x2c
 80074a2:	eb01 09ce 	add.w	r9, r1, lr, lsl #3
 80074a6:	eb01 0cc6 	add.w	ip, r1, r6, lsl #3
 80074aa:	f08e 0203 	eor.w	r2, lr, #3
 80074ae:	2500      	movs	r5, #0
 80074b0:	9302      	str	r3, [sp, #8]
 80074b2:	e008      	b.n	80074c6 <smoltcp::socket::tcp::Socket::process+0xf22>
 80074b4:	eb0c 00c5 	add.w	r0, ip, r5, lsl #3
 80074b8:	e9d0 3e36 	ldrd	r3, lr, [r0, #216]	@ 0xd8
 80074bc:	3501      	adds	r5, #1
 80074be:	e9c4 3e38 	strd	r3, lr, [r4, #224]	@ 0xe0
 80074c2:	42aa      	cmp	r2, r5
 80074c4:	d00f      	beq.n	80074e6 <smoltcp::socket::tcp::Socket::process+0xf42>
 80074c6:	eb09 04c5 	add.w	r4, r9, r5, lsl #3
 80074ca:	f8d4 00e4 	ldr.w	r0, [r4, #228]	@ 0xe4
 80074ce:	b150      	cbz	r0, 80074e6 <smoltcp::socket::tcp::Socket::process+0xf42>
 80074d0:	1970      	adds	r0, r6, r5
 80074d2:	2803      	cmp	r0, #3
 80074d4:	d9ee      	bls.n	80074b4 <smoltcp::socket::tcp::Socket::process+0xf10>
 80074d6:	2300      	movs	r3, #0
 80074d8:	f04f 0e00 	mov.w	lr, #0
 80074dc:	3501      	adds	r5, #1
 80074de:	e9c4 3e38 	strd	r3, lr, [r4, #224]	@ 0xe0
 80074e2:	42aa      	cmp	r2, r5
 80074e4:	d1ef      	bne.n	80074c6 <smoltcp::socket::tcp::Socket::process+0xf22>
 80074e6:	e9db 9500 	ldrd	r9, r5, [fp]
 80074ea:	9b02      	ldr	r3, [sp, #8]
 80074ec:	f8dd c02c 	ldr.w	ip, [sp, #44]	@ 0x2c
 80074f0:	eb05 0209 	add.w	r2, r5, r9
 80074f4:	4592      	cmp	sl, r2
 80074f6:	d930      	bls.n	800755a <smoltcp::socket::tcp::Socket::process+0xfb6>
 80074f8:	ebaa 0202 	sub.w	r2, sl, r2
 80074fc:	1950      	adds	r0, r2, r5
 80074fe:	f8cb 0004 	str.w	r0, [fp, #4]
 8007502:	9806      	ldr	r0, [sp, #24]
 8007504:	bb48      	cbnz	r0, 800755a <smoltcp::socket::tcp::Socket::process+0xfb6>
 8007506:	eb08 00cc 	add.w	r0, r8, ip, lsl #3
 800750a:	461e      	mov	r6, r3
 800750c:	6843      	ldr	r3, [r0, #4]
 800750e:	2b00      	cmp	r3, #0
 8007510:	4633      	mov	r3, r6
 8007512:	bf1f      	itttt	ne
 8007514:	6803      	ldrne	r3, [r0, #0]
 8007516:	1a9a      	subne	r2, r3, r2
 8007518:	4633      	movne	r3, r6
 800751a:	6002      	strne	r2, [r0, #0]
 800751c:	e01d      	b.n	800755a <smoltcp::socket::tcp::Socket::process+0xfb6>
 800751e:	9806      	ldr	r0, [sp, #24]
 8007520:	2800      	cmp	r0, #0
 8007522:	f040 8144 	bne.w	80077ae <smoltcp::socket::tcp::Socket::process+0x120a>
 8007526:	e9d1 033a 	ldrd	r0, r3, [r1, #232]	@ 0xe8
 800752a:	e9c1 033c 	strd	r0, r3, [r1, #240]	@ 0xf0
 800752e:	9b02      	ldr	r3, [sp, #8]
 8007530:	9805      	ldr	r0, [sp, #20]
 8007532:	b140      	cbz	r0, 8007546 <smoltcp::socket::tcp::Socket::process+0xfa2>
 8007534:	e9d1 0238 	ldrd	r0, r2, [r1, #224]	@ 0xe0
 8007538:	4573      	cmp	r3, lr
 800753a:	e9c1 023a 	strd	r0, r2, [r1, #232]	@ 0xe8
 800753e:	bf9c      	itt	ls
 8007540:	980f      	ldrls	r0, [sp, #60]	@ 0x3c
 8007542:	e9c1 6038 	strdls	r6, r0, [r1, #224]	@ 0xe0
 8007546:	f8db 0008 	ldr.w	r0, [fp, #8]
 800754a:	f8cb 4004 	str.w	r4, [fp, #4]
 800754e:	eba0 000a 	sub.w	r0, r0, sl
 8007552:	f8cb 9000 	str.w	r9, [fp]
 8007556:	f8cb 0008 	str.w	r0, [fp, #8]
 800755a:	f8d1 00d8 	ldr.w	r0, [r1, #216]	@ 0xd8
 800755e:	f04f 0c00 	mov.w	ip, #0
 8007562:	b9e8      	cbnz	r0, 80075a0 <smoltcp::socket::tcp::Socket::process+0xffc>
 8007564:	f8d1 00dc 	ldr.w	r0, [r1, #220]	@ 0xdc
 8007568:	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
 800756c:	b1d0      	cbz	r0, 80075a4 <smoltcp::socket::tcp::Socket::process+0x1000>
 800756e:	461d      	mov	r5, r3
 8007570:	e9d1 2338 	ldrd	r2, r3, [r1, #224]	@ 0xe0
 8007574:	e9c1 2336 	strd	r2, r3, [r1, #216]	@ 0xd8
 8007578:	2b00      	cmp	r3, #0
 800757a:	bf1e      	ittt	ne
 800757c:	e9d1 233a 	ldrdne	r2, r3, [r1, #232]	@ 0xe8
 8007580:	e9c1 2338 	strdne	r2, r3, [r1, #224]	@ 0xe0
 8007584:	2b00      	cmpne	r3, #0
 8007586:	d008      	beq.n	800759a <smoltcp::socket::tcp::Socket::process+0xff6>
 8007588:	e9d1 233c 	ldrd	r2, r3, [r1, #240]	@ 0xf0
 800758c:	2600      	movs	r6, #0
 800758e:	f101 0ce8 	add.w	ip, r1, #232	@ 0xe8
 8007592:	f8c1 60f4 	str.w	r6, [r1, #244]	@ 0xf4
 8007596:	e88c 004c 	stmia.w	ip, {r2, r3, r6}
 800759a:	4684      	mov	ip, r0
 800759c:	462b      	mov	r3, r5
 800759e:	e001      	b.n	80075a4 <smoltcp::socket::tcp::Socket::process+0x1000>
 80075a0:	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
 80075a4:	f8d1 40bc 	ldr.w	r4, [r1, #188]	@ 0xbc
 80075a8:	f8d1 50c4 	ldr.w	r5, [r1, #196]	@ 0xc4
 80075ac:	b144      	cbz	r4, 80075c0 <smoltcp::socket::tcp::Socket::process+0x101c>
 80075ae:	f8d1 00c0 	ldr.w	r0, [r1, #192]	@ 0xc0
 80075b2:	18ea      	adds	r2, r5, r3
 80075b4:	4410      	add	r0, r2
 80075b6:	fbb0 f2f4 	udiv	r2, r0, r4
 80075ba:	fb02 0014 	mls	r0, r2, r4, r0
 80075be:	e000      	b.n	80075c2 <smoltcp::socket::tcp::Socket::process+0x101e>
 80075c0:	2000      	movs	r0, #0
 80075c2:	eba4 0805 	sub.w	r8, r4, r5
 80075c6:	f8cd c034 	str.w	ip, [sp, #52]	@ 0x34
 80075ca:	4598      	cmp	r8, r3
 80075cc:	d205      	bcs.n	80075da <smoltcp::socket::tcp::Socket::process+0x1036>
 80075ce:	46d1      	mov	r9, sl
 80075d0:	469b      	mov	fp, r3
 80075d2:	468a      	mov	sl, r1
 80075d4:	2600      	movs	r6, #0
 80075d6:	2001      	movs	r0, #1
 80075d8:	e015      	b.n	8007606 <smoltcp::socket::tcp::Socket::process+0x1062>
 80075da:	9a04      	ldr	r2, [sp, #16]
 80075dc:	eba8 0603 	sub.w	r6, r8, r3
 80075e0:	42b2      	cmp	r2, r6
 80075e2:	bf38      	it	cc
 80075e4:	4616      	movcc	r6, r2
 80075e6:	1a22      	subs	r2, r4, r0
 80075e8:	4296      	cmp	r6, r2
 80075ea:	bf28      	it	cs
 80075ec:	4616      	movcs	r6, r2
 80075ee:	1832      	adds	r2, r6, r0
 80075f0:	f080 80c3 	bcs.w	800777a <smoltcp::socket::tcp::Socket::process+0x11d6>
 80075f4:	42a2      	cmp	r2, r4
 80075f6:	f200 80c0 	bhi.w	800777a <smoltcp::socket::tcp::Socket::process+0x11d6>
 80075fa:	46d1      	mov	r9, sl
 80075fc:	468a      	mov	sl, r1
 80075fe:	f8d1 10b8 	ldr.w	r1, [r1, #184]	@ 0xb8
 8007602:	469b      	mov	fp, r3
 8007604:	4408      	add	r0, r1
 8007606:	9901      	ldr	r1, [sp, #4]
 8007608:	4632      	mov	r2, r6
 800760a:	f001 fef5 	bl	80093f8 <__aeabi_memcpy>
 800760e:	eb06 010b 	add.w	r1, r6, fp
 8007612:	b14c      	cbz	r4, 8007628 <smoltcp::socket::tcp::Socket::process+0x1084>
 8007614:	f8da 00c0 	ldr.w	r0, [sl, #192]	@ 0xc0
 8007618:	194a      	adds	r2, r1, r5
 800761a:	4653      	mov	r3, sl
 800761c:	4410      	add	r0, r2
 800761e:	fbb0 f2f4 	udiv	r2, r0, r4
 8007622:	fb02 0014 	mls	r0, r2, r4, r0
 8007626:	e001      	b.n	800762c <smoltcp::socket::tcp::Socket::process+0x1088>
 8007628:	2000      	movs	r0, #0
 800762a:	4653      	mov	r3, sl
 800762c:	4588      	cmp	r8, r1
 800762e:	d203      	bcs.n	8007638 <smoltcp::socket::tcp::Socket::process+0x1094>
 8007630:	461c      	mov	r4, r3
 8007632:	2200      	movs	r2, #0
 8007634:	2001      	movs	r0, #1
 8007636:	e015      	b.n	8007664 <smoltcp::socket::tcp::Socket::process+0x10c0>
 8007638:	9a04      	ldr	r2, [sp, #16]
 800763a:	1b95      	subs	r5, r2, r6
 800763c:	eba8 0201 	sub.w	r2, r8, r1
 8007640:	4295      	cmp	r5, r2
 8007642:	eba4 0100 	sub.w	r1, r4, r0
 8007646:	bf38      	it	cc
 8007648:	462a      	movcc	r2, r5
 800764a:	428a      	cmp	r2, r1
 800764c:	bf28      	it	cs
 800764e:	460a      	movcs	r2, r1
 8007650:	1811      	adds	r1, r2, r0
 8007652:	f080 809a 	bcs.w	800778a <smoltcp::socket::tcp::Socket::process+0x11e6>
 8007656:	42a1      	cmp	r1, r4
 8007658:	f200 8097 	bhi.w	800778a <smoltcp::socket::tcp::Socket::process+0x11e6>
 800765c:	f8d3 10b8 	ldr.w	r1, [r3, #184]	@ 0xb8
 8007660:	461c      	mov	r4, r3
 8007662:	4408      	add	r0, r1
 8007664:	9901      	ldr	r1, [sp, #4]
 8007666:	4431      	add	r1, r6
 8007668:	f001 fec6 	bl	80093f8 <__aeabi_memcpy>
 800766c:	9b0d      	ldr	r3, [sp, #52]	@ 0x34
 800766e:	4621      	mov	r1, r4
 8007670:	b153      	cbz	r3, 8007688 <smoltcp::socket::tcp::Socket::process+0x10e4>
 8007672:	f8d1 20bc 	ldr.w	r2, [r1, #188]	@ 0xbc
 8007676:	f8d1 00c4 	ldr.w	r0, [r1, #196]	@ 0xc4
 800767a:	1a12      	subs	r2, r2, r0
 800767c:	4293      	cmp	r3, r2
 800767e:	f200 808b 	bhi.w	8007798 <smoltcp::socket::tcp::Socket::process+0x11f4>
 8007682:	4418      	add	r0, r3
 8007684:	f8c1 00c4 	str.w	r0, [r1, #196]	@ 0xc4
 8007688:	6e08      	ldr	r0, [r1, #96]	@ 0x60
 800768a:	07c0      	lsls	r0, r0, #31
 800768c:	d034      	beq.n	80076f8 <smoltcp::socket::tcp::Socket::process+0x1154>
 800768e:	f8d1 0098 	ldr.w	r0, [r1, #152]	@ 0x98
 8007692:	2801      	cmp	r0, #1
 8007694:	d130      	bne.n	80076f8 <smoltcp::socket::tcp::Socket::process+0x1154>
 8007696:	f8d1 00c4 	ldr.w	r0, [r1, #196]	@ 0xc4
 800769a:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 800769e:	f77e aff3 	ble.w	8006688 <smoltcp::socket::tcp::Socket::process+0xe4>
 80076a2:	f8d1 3104 	ldr.w	r3, [r1, #260]	@ 0x104
 80076a6:	f8d1 209c 	ldr.w	r2, [r1, #156]	@ 0x9c
 80076aa:	4418      	add	r0, r3
 80076ac:	1a10      	subs	r0, r2, r0
 80076ae:	d523      	bpl.n	80076f8 <smoltcp::socket::tcp::Socket::process+0x1154>
 80076b0:	6f08      	ldr	r0, [r1, #112]	@ 0x70
 80076b2:	2802      	cmp	r0, #2
 80076b4:	d00a      	beq.n	80076cc <smoltcp::socket::tcp::Socket::process+0x1128>
 80076b6:	2801      	cmp	r0, #1
 80076b8:	d10b      	bne.n	80076d2 <smoltcp::socket::tcp::Socket::process+0x112e>
 80076ba:	4608      	mov	r0, r1
 80076bc:	460c      	mov	r4, r1
 80076be:	f000 f9cf 	bl	8007a60 <smoltcp::socket::tcp::Socket::immediate_ack_to_transmit>
 80076c2:	b180      	cbz	r0, 80076e6 <smoltcp::socket::tcp::Socket::process+0x1142>
 80076c4:	2000      	movs	r0, #0
 80076c6:	2202      	movs	r2, #2
 80076c8:	4621      	mov	r1, r4
 80076ca:	e011      	b.n	80076f0 <smoltcp::socket::tcp::Socket::process+0x114c>
 80076cc:	2000      	movs	r0, #0
 80076ce:	2202      	movs	r2, #2
 80076d0:	e00e      	b.n	80076f0 <smoltcp::socket::tcp::Socket::process+0x114c>
 80076d2:	e9d1 201a 	ldrd	r2, r0, [r1, #104]	@ 0x68
 80076d6:	9b0a      	ldr	r3, [sp, #40]	@ 0x28
 80076d8:	189b      	adds	r3, r3, r2
 80076da:	9a09      	ldr	r2, [sp, #36]	@ 0x24
 80076dc:	eb40 0602 	adc.w	r6, r0, r2
 80076e0:	2000      	movs	r0, #0
 80076e2:	2201      	movs	r2, #1
 80076e4:	e004      	b.n	80076f0 <smoltcp::socket::tcp::Socket::process+0x114c>
 80076e6:	4621      	mov	r1, r4
 80076e8:	e9d4 201c 	ldrd	r2, r0, [r4, #112]	@ 0x70
 80076ec:	e9d4 361e 	ldrd	r3, r6, [r4, #120]	@ 0x78
 80076f0:	e9c1 201c 	strd	r2, r0, [r1, #112]	@ 0x70
 80076f4:	e9c1 361e 	strd	r3, r6, [r1, #120]	@ 0x78
 80076f8:	f8d1 00dc 	ldr.w	r0, [r1, #220]	@ 0xdc
 80076fc:	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
 80076fe:	4310      	orrs	r0, r2
 8007700:	d00b      	beq.n	800771a <smoltcp::socket::tcp::Socket::process+0x1176>
 8007702:	980e      	ldr	r0, [sp, #56]	@ 0x38
 8007704:	60b8      	str	r0, [r7, #8]
 8007706:	4648      	mov	r0, r9
 8007708:	9a08      	ldr	r2, [sp, #32]
 800770a:	9b07      	ldr	r3, [sp, #28]
 800770c:	b011      	add	sp, #68	@ 0x44
 800770e:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007712:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 8007716:	f000 b89f 	b.w	8007858 <smoltcp::socket::tcp::Socket::ack_reply>
 800771a:	2002      	movs	r0, #2
 800771c:	f8c9 0010 	str.w	r0, [r9, #16]
 8007720:	b011      	add	sp, #68	@ 0x44
 8007722:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007726:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007728:	f64a 209c 	movw	r0, #43676	@ 0xaa9c
 800772c:	f64a 22d0 	movw	r2, #43728	@ 0xaad0
 8007730:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8007734:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007738:	2167      	movs	r1, #103	@ 0x67
 800773a:	f7fc f8d1 	bl	80038e0 <core::panicking::panic_fmt>
 800773e:	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
 8007742:	f64a 5250 	movw	r2, #44368	@ 0xad50
 8007746:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800774a:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800774e:	2128      	movs	r1, #40	@ 0x28
 8007750:	f7fc f8e8 	bl	8003924 <core::panicking::panic>
 8007754:	4611      	mov	r1, r2
 8007756:	9a0b      	ldr	r2, [sp, #44]	@ 0x2c
 8007758:	f64a 539c 	movw	r3, #44444	@ 0xad9c
 800775c:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007760:	f7fc f84d 	bl	80037fe <core::slice::index::slice_index_fail>
 8007764:	f64a 60cc 	movw	r0, #44748	@ 0xaecc
 8007768:	f64a 62f4 	movw	r2, #44788	@ 0xaef4
 800776c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8007770:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007774:	2125      	movs	r1, #37	@ 0x25
 8007776:	f7fc f8d5 	bl	8003924 <core::panicking::panic>
 800777a:	f64a 63ac 	movw	r3, #44716	@ 0xaeac
 800777e:	4611      	mov	r1, r2
 8007780:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007784:	4622      	mov	r2, r4
 8007786:	f7fc f83a 	bl	80037fe <core::slice::index::slice_index_fail>
 800778a:	f64a 63ac 	movw	r3, #44716	@ 0xaeac
 800778e:	4622      	mov	r2, r4
 8007790:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007794:	f7fc f833 	bl	80037fe <core::slice::index::slice_index_fail>
 8007798:	f64a 5061 	movw	r0, #44385	@ 0xad61
 800779c:	f64a 528c 	movw	r2, #44428	@ 0xad8c
 80077a0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80077a4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80077a8:	2128      	movs	r1, #40	@ 0x28
 80077aa:	f7fc f8bb 	bl	8003924 <core::panicking::panic>
 80077ae:	f24b 0248 	movw	r2, #45128	@ 0xb048
 80077b2:	2004      	movs	r0, #4
 80077b4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80077b8:	2104      	movs	r1, #4
 80077ba:	e9cb 9400 	strd	r9, r4, [fp]
 80077be:	f7fc f89d 	bl	80038fc <core::panicking::panic_bounds_check>

080077c2 <smoltcp::socket::tcp::Socket::rst_reply>:
 80077c2:	b5f0      	push	{r4, r5, r6, r7, lr}
 80077c4:	af03      	add	r7, sp, #12
 80077c6:	f84d 8d04 	str.w	r8, [sp, #-4]!
 80077ca:	f8b3 c04c 	ldrh.w	ip, [r3, #76]	@ 0x4c
 80077ce:	f8b3 e04a 	ldrh.w	lr, [r3, #74]	@ 0x4a
 80077d2:	681c      	ldr	r4, [r3, #0]
 80077d4:	2c01      	cmp	r4, #1
 80077d6:	d102      	bne.n	80077de <smoltcp::socket::tcp::Socket::rst_reply+0x1c>
 80077d8:	685d      	ldr	r5, [r3, #4]
 80077da:	2400      	movs	r4, #0
 80077dc:	e010      	b.n	8007800 <smoltcp::socket::tcp::Socket::rst_reply+0x3e>
 80077de:	f893 4051 	ldrb.w	r4, [r3, #81]	@ 0x51
 80077e2:	2c02      	cmp	r4, #2
 80077e4:	d10a      	bne.n	80077fc <smoltcp::socket::tcp::Socket::rst_reply+0x3a>
 80077e6:	6c1c      	ldr	r4, [r3, #64]	@ 0x40
 80077e8:	3401      	adds	r4, #1
 80077ea:	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
 80077ee:	dd28      	ble.n	8007842 <smoltcp::socket::tcp::Socket::rst_reply+0x80>
 80077f0:	6c5b      	ldr	r3, [r3, #68]	@ 0x44
 80077f2:	2500      	movs	r5, #0
 80077f4:	eb04 0803 	add.w	r8, r4, r3
 80077f8:	2401      	movs	r4, #1
 80077fa:	e001      	b.n	8007800 <smoltcp::socket::tcp::Socket::rst_reply+0x3e>
 80077fc:	2500      	movs	r5, #0
 80077fe:	2400      	movs	r4, #0
 8007800:	f04f 6680 	mov.w	r6, #67108864	@ 0x4000000
 8007804:	2301      	movs	r3, #1
 8007806:	f8c0 605e 	str.w	r6, [r0, #94]	@ 0x5e
 800780a:	2600      	movs	r6, #0
 800780c:	e9c0 3613 	strd	r3, r6, [r0, #76]	@ 0x4c
 8007810:	f44f 7350 	mov.w	r3, #832	@ 0x340
 8007814:	8183      	strh	r3, [r0, #12]
 8007816:	2314      	movs	r3, #20
 8007818:	f8a0 e05c 	strh.w	lr, [r0, #92]	@ 0x5c
 800781c:	f8a0 c05a 	strh.w	ip, [r0, #90]	@ 0x5a
 8007820:	f880 6058 	strb.w	r6, [r0, #88]	@ 0x58
 8007824:	6545      	str	r5, [r0, #84]	@ 0x54
 8007826:	f8a0 6048 	strh.w	r6, [r0, #72]	@ 0x48
 800782a:	63c6      	str	r6, [r0, #60]	@ 0x3c
 800782c:	6306      	str	r6, [r0, #48]	@ 0x30
 800782e:	6246      	str	r6, [r0, #36]	@ 0x24
 8007830:	e9c0 4804 	strd	r4, r8, [r0, #16]
 8007834:	6186      	str	r6, [r0, #24]
 8007836:	e9c0 2100 	strd	r2, r1, [r0]
 800783a:	6083      	str	r3, [r0, #8]
 800783c:	f85d 8b04 	ldr.w	r8, [sp], #4
 8007840:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007842:	f64a 4078 	movw	r0, #44152	@ 0xac78
 8007846:	f64a 42b0 	movw	r2, #44208	@ 0xacb0
 800784a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800784e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007852:	2171      	movs	r1, #113	@ 0x71
 8007854:	f7fc f844 	bl	80038e0 <core::panicking::panic_fmt>

08007858 <smoltcp::socket::tcp::Socket::ack_reply>:
 8007858:	b5f0      	push	{r4, r5, r6, r7, lr}
 800785a:	af03      	add	r7, sp, #12
 800785c:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8007860:	b089      	sub	sp, #36	@ 0x24
 8007862:	4604      	mov	r4, r0
 8007864:	68b8      	ldr	r0, [r7, #8]
 8007866:	4689      	mov	r9, r1
 8007868:	6ac1      	ldr	r1, [r0, #44]	@ 0x2c
 800786a:	f8b0 b04c 	ldrh.w	fp, [r0, #76]	@ 0x4c
 800786e:	f8b0 504a 	ldrh.w	r5, [r0, #74]	@ 0x4a
 8007872:	07c9      	lsls	r1, r1, #31
 8007874:	bf1c      	itt	ne
 8007876:	f8d9 1114 	ldrne.w	r1, [r9, #276]	@ 0x114
 800787a:	2900      	cmpne	r1, #0
 800787c:	f040 8085 	bne.w	800798a <smoltcp::socket::tcp::Socket::ack_reply+0x132>
 8007880:	2600      	movs	r6, #0
 8007882:	2117      	movs	r1, #23
 8007884:	f8d9 00c4 	ldr.w	r0, [r9, #196]	@ 0xc4
 8007888:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 800788c:	f340 808d 	ble.w	80079aa <smoltcp::socket::tcp::Socket::ack_reply+0x152>
 8007890:	e9cd 6106 	strd	r6, r1, [sp, #24]
 8007894:	f04f 0c01 	mov.w	ip, #1
 8007898:	e9d9 1641 	ldrd	r1, r6, [r9, #260]	@ 0x104
 800789c:	f64f 78ff 	movw	r8, #65535	@ 0xffff
 80078a0:	9508      	str	r5, [sp, #32]
 80078a2:	9605      	str	r6, [sp, #20]
 80078a4:	eb00 0a01 	add.w	sl, r0, r1
 80078a8:	f8d9 50bc 	ldr.w	r5, [r9, #188]	@ 0xbc
 80078ac:	f899 6134 	ldrb.w	r6, [r9, #308]	@ 0x134
 80078b0:	1a28      	subs	r0, r5, r0
 80078b2:	f899 e131 	ldrb.w	lr, [r9, #305]	@ 0x131
 80078b6:	f006 011f 	and.w	r1, r6, #31
 80078ba:	e9c9 ca26 	strd	ip, sl, [r9, #152]	@ 0x98
 80078be:	40c8      	lsrs	r0, r1
 80078c0:	4540      	cmp	r0, r8
 80078c2:	bf38      	it	cc
 80078c4:	4680      	movcc	r8, r0
 80078c6:	f1be 0f00 	cmp.w	lr, #0
 80078ca:	f8a9 812e 	strh.w	r8, [r9, #302]	@ 0x12e
 80078ce:	f000 8093 	beq.w	80079f8 <smoltcp::socket::tcp::Socket::ack_reply+0x1a0>
 80078d2:	f240 0017 	movw	r0, #23
 80078d6:	e9cd 3201 	strd	r3, r2, [sp, #4]
 80078da:	f2c0 0000 	movt	r0, #0
 80078de:	f7fc fdca 	bl	8004476 <defmt::export::acquire_header_and_release>
 80078e2:	f8d9 00a0 	ldr.w	r0, [r9, #160]	@ 0xa0
 80078e6:	2801      	cmp	r0, #1
 80078e8:	d147      	bne.n	800797a <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 80078ea:	f8d9 c0a4 	ldr.w	ip, [r9, #164]	@ 0xa4
 80078ee:	f109 00d8 	add.w	r0, r9, #216	@ 0xd8
 80078f2:	2600      	movs	r6, #0
 80078f4:	2200      	movs	r2, #0
 80078f6:	e000      	b.n	80078fa <smoltcp::socket::tcp::Socket::ack_reply+0xa2>
 80078f8:	3201      	adds	r2, #1
 80078fa:	2a03      	cmp	r2, #3
 80078fc:	d83d      	bhi.n	800797a <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 80078fe:	eb00 03c2 	add.w	r3, r0, r2, lsl #3
 8007902:	f850 1032 	ldr.w	r1, [r0, r2, lsl #3]
 8007906:	685d      	ldr	r5, [r3, #4]
 8007908:	1873      	adds	r3, r6, r1
 800790a:	195e      	adds	r6, r3, r5
 800790c:	42b3      	cmp	r3, r6
 800790e:	d325      	bcc.n	800795c <smoltcp::socket::tcp::Socket::ack_reply+0x104>
 8007910:	1c51      	adds	r1, r2, #1
 8007912:	2904      	cmp	r1, #4
 8007914:	d031      	beq.n	800797a <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 8007916:	eb00 05c1 	add.w	r5, r0, r1, lsl #3
 800791a:	f850 6031 	ldr.w	r6, [r0, r1, lsl #3]
 800791e:	686d      	ldr	r5, [r5, #4]
 8007920:	4433      	add	r3, r6
 8007922:	195e      	adds	r6, r3, r5
 8007924:	42b3      	cmp	r3, r6
 8007926:	d30b      	bcc.n	8007940 <smoltcp::socket::tcp::Socket::ack_reply+0xe8>
 8007928:	1c91      	adds	r1, r2, #2
 800792a:	2904      	cmp	r1, #4
 800792c:	d025      	beq.n	800797a <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 800792e:	eb00 05c1 	add.w	r5, r0, r1, lsl #3
 8007932:	f850 6031 	ldr.w	r6, [r0, r1, lsl #3]
 8007936:	686d      	ldr	r5, [r5, #4]
 8007938:	4433      	add	r3, r6
 800793a:	195e      	adds	r6, r3, r5
 800793c:	42b3      	cmp	r3, r6
 800793e:	d201      	bcs.n	8007944 <smoltcp::socket::tcp::Socket::ack_reply+0xec>
 8007940:	460a      	mov	r2, r1
 8007942:	e00b      	b.n	800795c <smoltcp::socket::tcp::Socket::ack_reply+0x104>
 8007944:	3203      	adds	r2, #3
 8007946:	2a04      	cmp	r2, #4
 8007948:	d017      	beq.n	800797a <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 800794a:	eb00 06c2 	add.w	r6, r0, r2, lsl #3
 800794e:	f850 1032 	ldr.w	r1, [r0, r2, lsl #3]
 8007952:	6876      	ldr	r6, [r6, #4]
 8007954:	440b      	add	r3, r1
 8007956:	441e      	add	r6, r3
 8007958:	42b3      	cmp	r3, r6
 800795a:	d20e      	bcs.n	800797a <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 800795c:	eb0a 0503 	add.w	r5, sl, r3
 8007960:	4565      	cmp	r5, ip
 8007962:	d8c9      	bhi.n	80078f8 <smoltcp::socket::tcp::Socket::ack_reply+0xa0>
 8007964:	eb0a 0106 	add.w	r1, sl, r6
 8007968:	4561      	cmp	r1, ip
 800796a:	d3c5      	bcc.n	80078f8 <smoltcp::socket::tcp::Socket::ack_reply+0xa0>
 800796c:	f04f 090a 	mov.w	r9, #10
 8007970:	f04f 0c01 	mov.w	ip, #1
 8007974:	e9dd 3201 	ldrd	r3, r2, [sp, #4]
 8007978:	e042      	b.n	8007a00 <smoltcp::socket::tcp::Socket::ack_reply+0x1a8>
 800797a:	e9d9 0136 	ldrd	r0, r1, [r9, #216]	@ 0xd8
 800797e:	4401      	add	r1, r0
 8007980:	4288      	cmp	r0, r1
 8007982:	d21d      	bcs.n	80079c0 <smoltcp::socket::tcp::Socket::ack_reply+0x168>
 8007984:	e9dd 3201 	ldrd	r3, r2, [sp, #4]
 8007988:	e02e      	b.n	80079e8 <smoltcp::socket::tcp::Socket::ack_reply+0x190>
 800798a:	6b00      	ldr	r0, [r0, #48]	@ 0x30
 800798c:	4690      	mov	r8, r2
 800798e:	9004      	str	r0, [sp, #16]
 8007990:	461e      	mov	r6, r3
 8007992:	4788      	blx	r1
 8007994:	4633      	mov	r3, r6
 8007996:	4642      	mov	r2, r8
 8007998:	2601      	movs	r6, #1
 800799a:	2121      	movs	r1, #33	@ 0x21
 800799c:	9003      	str	r0, [sp, #12]
 800799e:	f8d9 00c4 	ldr.w	r0, [r9, #196]	@ 0xc4
 80079a2:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 80079a6:	f73f af73 	bgt.w	8007890 <smoltcp::socket::tcp::Socket::ack_reply+0x38>
 80079aa:	f64a 4078 	movw	r0, #44152	@ 0xac78
 80079ae:	f64a 42b0 	movw	r2, #44208	@ 0xacb0
 80079b2:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80079b6:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80079ba:	2171      	movs	r1, #113	@ 0x71
 80079bc:	f7fb ff90 	bl	80038e0 <core::panicking::panic_fmt>
 80079c0:	e9d9 1238 	ldrd	r1, r2, [r9, #224]	@ 0xe0
 80079c4:	4408      	add	r0, r1
 80079c6:	1881      	adds	r1, r0, r2
 80079c8:	e9dd 3201 	ldrd	r3, r2, [sp, #4]
 80079cc:	4288      	cmp	r0, r1
 80079ce:	d30b      	bcc.n	80079e8 <smoltcp::socket::tcp::Socket::ack_reply+0x190>
 80079d0:	e9d9 163a 	ldrd	r1, r6, [r9, #232]	@ 0xe8
 80079d4:	4408      	add	r0, r1
 80079d6:	1981      	adds	r1, r0, r6
 80079d8:	4288      	cmp	r0, r1
 80079da:	d305      	bcc.n	80079e8 <smoltcp::socket::tcp::Socket::ack_reply+0x190>
 80079dc:	e9d9 163c 	ldrd	r1, r6, [r9, #240]	@ 0xf0
 80079e0:	4408      	add	r0, r1
 80079e2:	1981      	adds	r1, r0, r6
 80079e4:	4288      	cmp	r0, r1
 80079e6:	d207      	bcs.n	80079f8 <smoltcp::socket::tcp::Socket::ack_reply+0x1a0>
 80079e8:	4451      	add	r1, sl
 80079ea:	eb0a 0500 	add.w	r5, sl, r0
 80079ee:	f04f 0c01 	mov.w	ip, #1
 80079f2:	f04f 090a 	mov.w	r9, #10
 80079f6:	e003      	b.n	8007a00 <smoltcp::socket::tcp::Socket::ack_reply+0x1a8>
 80079f8:	f04f 0c00 	mov.w	ip, #0
 80079fc:	f04f 0900 	mov.w	r9, #0
 8007a00:	9808      	ldr	r0, [sp, #32]
 8007a02:	2600      	movs	r6, #0
 8007a04:	f8a4 005c 	strh.w	r0, [r4, #92]	@ 0x5c
 8007a08:	f04f 0e01 	mov.w	lr, #1
 8007a0c:	9805      	ldr	r0, [sp, #20]
 8007a0e:	6560      	str	r0, [r4, #84]	@ 0x54
 8007a10:	9806      	ldr	r0, [sp, #24]
 8007a12:	63e0      	str	r0, [r4, #60]	@ 0x3c
 8007a14:	9803      	ldr	r0, [sp, #12]
 8007a16:	6420      	str	r0, [r4, #64]	@ 0x40
 8007a18:	9804      	ldr	r0, [sp, #16]
 8007a1a:	6460      	str	r0, [r4, #68]	@ 0x44
 8007a1c:	9807      	ldr	r0, [sp, #28]
 8007a1e:	e9c4 3200 	strd	r3, r2, [r4]
 8007a22:	f104 0208 	add.w	r2, r4, #8
 8007a26:	4448      	add	r0, r9
 8007a28:	e9c4 1608 	strd	r1, r6, [r4, #32]
 8007a2c:	f000 007c 	and.w	r0, r0, #124	@ 0x7c
 8007a30:	f44f 7150 	mov.w	r1, #832	@ 0x340
 8007a34:	f8a4 6060 	strh.w	r6, [r4, #96]	@ 0x60
 8007a38:	f8a4 805e 	strh.w	r8, [r4, #94]	@ 0x5e
 8007a3c:	f8a4 b05a 	strh.w	fp, [r4, #90]	@ 0x5a
 8007a40:	f884 6058 	strb.w	r6, [r4, #88]	@ 0x58
 8007a44:	e9c4 e613 	strd	lr, r6, [r4, #76]	@ 0x4c
 8007a48:	f8a4 6048 	strh.w	r6, [r4, #72]	@ 0x48
 8007a4c:	6326      	str	r6, [r4, #48]	@ 0x30
 8007a4e:	e882 4003 	stmia.w	r2, {r0, r1, lr}
 8007a52:	e9c4 ac05 	strd	sl, ip, [r4, #20]
 8007a56:	61e5      	str	r5, [r4, #28]
 8007a58:	b009      	add	sp, #36	@ 0x24
 8007a5a:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007a5e:	bdf0      	pop	{r4, r5, r6, r7, pc}

08007a60 <smoltcp::socket::tcp::Socket::immediate_ack_to_transmit>:
 8007a60:	b580      	push	{r7, lr}
 8007a62:	466f      	mov	r7, sp
 8007a64:	f8d0 1098 	ldr.w	r1, [r0, #152]	@ 0x98
 8007a68:	2901      	cmp	r1, #1
 8007a6a:	bf1c      	itt	ne
 8007a6c:	2000      	movne	r0, #0
 8007a6e:	bd80      	popne	{r7, pc}
 8007a70:	f8d0 1110 	ldr.w	r1, [r0, #272]	@ 0x110
 8007a74:	f1b1 3fff 	cmp.w	r1, #4294967295	@ 0xffffffff
 8007a78:	bfc1      	itttt	gt
 8007a7a:	f8d0 20c4 	ldrgt.w	r2, [r0, #196]	@ 0xc4
 8007a7e:	f1b2 3fff 	cmpgt.w	r2, #4294967295	@ 0xffffffff
 8007a82:	f8d0 309c 	ldrgt.w	r3, [r0, #156]	@ 0x9c
 8007a86:	f8d0 0104 	ldrgt.w	r0, [r0, #260]	@ 0x104
 8007a8a:	bfc1      	itttt	gt
 8007a8c:	4410      	addgt	r0, r2
 8007a8e:	4419      	addgt	r1, r3
 8007a90:	1a08      	subgt	r0, r1, r0
 8007a92:	0fc0      	lsrgt	r0, r0, #31
 8007a94:	bfc8      	it	gt
 8007a96:	bd80      	popgt	{r7, pc}
 8007a98:	f64a 4078 	movw	r0, #44152	@ 0xac78
 8007a9c:	f64a 42b0 	movw	r2, #44208	@ 0xacb0
 8007aa0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8007aa4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007aa8:	2171      	movs	r1, #113	@ 0x71
 8007aaa:	f7fb ff19 	bl	80038e0 <core::panicking::panic_fmt>

08007aae <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>:
 8007aae:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007ab0:	af03      	add	r7, sp, #12
 8007ab2:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8007ab6:	f002 06f0 	and.w	r6, r2, #240	@ 0xf0
 8007aba:	2ee0      	cmp	r6, #224	@ 0xe0
 8007abc:	d002      	beq.n	8007ac4 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007abe:	1c56      	adds	r6, r2, #1
 8007ac0:	2e02      	cmp	r6, #2
 8007ac2:	d204      	bcs.n	8007ace <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x20>
 8007ac4:	2104      	movs	r1, #4
 8007ac6:	6001      	str	r1, [r0, #0]
 8007ac8:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007acc:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007ace:	f8d1 c0f0 	ldr.w	ip, [r1, #240]	@ 0xf0
 8007ad2:	f1bc 0f00 	cmp.w	ip, #0
 8007ad6:	eb0c 068c 	add.w	r6, ip, ip, lsl #2
 8007ada:	440e      	add	r6, r1
 8007adc:	f106 0ef4 	add.w	lr, r6, #244	@ 0xf4
 8007ae0:	d023      	beq.n	8007b2a <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x7c>
 8007ae2:	f101 04f4 	add.w	r4, r1, #244	@ 0xf4
 8007ae6:	f04f 38ff 	mov.w	r8, #4294967295	@ 0xffffffff
 8007aea:	e00d      	b.n	8007b08 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x5a>
 8007aec:	2500      	movs	r5, #0
 8007aee:	f006 061f 	and.w	r6, r6, #31
 8007af2:	ea05 0509 	and.w	r5, r5, r9
 8007af6:	fa28 f606 	lsr.w	r6, r8, r6
 8007afa:	ba36      	rev	r6, r6
 8007afc:	4335      	orrs	r5, r6
 8007afe:	42aa      	cmp	r2, r5
 8007b00:	d0e0      	beq.n	8007ac4 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007b02:	3405      	adds	r4, #5
 8007b04:	4574      	cmp	r4, lr
 8007b06:	d010      	beq.n	8007b2a <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x7c>
 8007b08:	7926      	ldrb	r6, [r4, #4]
 8007b0a:	f8d4 9000 	ldr.w	r9, [r4]
 8007b0e:	2e00      	cmp	r6, #0
 8007b10:	d0ec      	beq.n	8007aec <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x3e>
 8007b12:	f1a6 051f 	sub.w	r5, r6, #31
 8007b16:	b2ed      	uxtb	r5, r5
 8007b18:	2d02      	cmp	r5, #2
 8007b1a:	d3f2      	bcc.n	8007b02 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x54>
 8007b1c:	4275      	negs	r5, r6
 8007b1e:	f005 051f 	and.w	r5, r5, #31
 8007b22:	fa08 f505 	lsl.w	r5, r8, r5
 8007b26:	ba2d      	rev	r5, r5
 8007b28:	e7e1      	b.n	8007aee <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x40>
 8007b2a:	f8d7 9008 	ldr.w	r9, [r7, #8]
 8007b2e:	f003 04f0 	and.w	r4, r3, #240	@ 0xf0
 8007b32:	2ce0      	cmp	r4, #224	@ 0xe0
 8007b34:	d003      	beq.n	8007b3e <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x90>
 8007b36:	f113 0801 	adds.w	r8, r3, #1
 8007b3a:	d070      	beq.n	8007c1e <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x170>
 8007b3c:	bb33      	cbnz	r3, 8007b8c <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xde>
 8007b3e:	f1bc 0f00 	cmp.w	ip, #0
 8007b42:	d0bf      	beq.n	8007ac4 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007b44:	f101 05f4 	add.w	r5, r1, #244	@ 0xf4
 8007b48:	f04f 38ff 	mov.w	r8, #4294967295	@ 0xffffffff
 8007b4c:	e00d      	b.n	8007b6a <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xbc>
 8007b4e:	2600      	movs	r6, #0
 8007b50:	f004 041f 	and.w	r4, r4, #31
 8007b54:	ea06 060a 	and.w	r6, r6, sl
 8007b58:	fa28 f404 	lsr.w	r4, r8, r4
 8007b5c:	ba24      	rev	r4, r4
 8007b5e:	4334      	orrs	r4, r6
 8007b60:	42a3      	cmp	r3, r4
 8007b62:	d05c      	beq.n	8007c1e <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x170>
 8007b64:	3505      	adds	r5, #5
 8007b66:	4575      	cmp	r5, lr
 8007b68:	d0ac      	beq.n	8007ac4 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007b6a:	792c      	ldrb	r4, [r5, #4]
 8007b6c:	f8d5 a000 	ldr.w	sl, [r5]
 8007b70:	2c00      	cmp	r4, #0
 8007b72:	d0ec      	beq.n	8007b4e <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xa0>
 8007b74:	f1a4 061f 	sub.w	r6, r4, #31
 8007b78:	b2f6      	uxtb	r6, r6
 8007b7a:	2e02      	cmp	r6, #2
 8007b7c:	d3f2      	bcc.n	8007b64 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xb6>
 8007b7e:	4266      	negs	r6, r4
 8007b80:	f006 061f 	and.w	r6, r6, #31
 8007b84:	fa08 f606 	lsl.w	r6, r8, r6
 8007b88:	ba36      	rev	r6, r6
 8007b8a:	e7e1      	b.n	8007b50 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xa2>
 8007b8c:	f1bc 0f00 	cmp.w	ip, #0
 8007b90:	d023      	beq.n	8007bda <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x12c>
 8007b92:	f101 06f4 	add.w	r6, r1, #244	@ 0xf4
 8007b96:	f04f 3aff 	mov.w	sl, #4294967295	@ 0xffffffff
 8007b9a:	e00d      	b.n	8007bb8 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x10a>
 8007b9c:	2500      	movs	r5, #0
 8007b9e:	f004 041f 	and.w	r4, r4, #31
 8007ba2:	ea05 050b 	and.w	r5, r5, fp
 8007ba6:	fa2a f404 	lsr.w	r4, sl, r4
 8007baa:	ba24      	rev	r4, r4
 8007bac:	432c      	orrs	r4, r5
 8007bae:	42a3      	cmp	r3, r4
 8007bb0:	d032      	beq.n	8007c18 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16a>
 8007bb2:	3605      	adds	r6, #5
 8007bb4:	4576      	cmp	r6, lr
 8007bb6:	d010      	beq.n	8007bda <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x12c>
 8007bb8:	7934      	ldrb	r4, [r6, #4]
 8007bba:	f8d6 b000 	ldr.w	fp, [r6]
 8007bbe:	2c00      	cmp	r4, #0
 8007bc0:	d0ec      	beq.n	8007b9c <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xee>
 8007bc2:	f1a4 051f 	sub.w	r5, r4, #31
 8007bc6:	b2ed      	uxtb	r5, r5
 8007bc8:	2d02      	cmp	r5, #2
 8007bca:	d3f2      	bcc.n	8007bb2 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x104>
 8007bcc:	4265      	negs	r5, r4
 8007bce:	f005 051f 	and.w	r5, r5, #31
 8007bd2:	fa0a f505 	lsl.w	r5, sl, r5
 8007bd6:	ba2d      	rev	r5, r5
 8007bd8:	e7e1      	b.n	8007b9e <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xf0>
 8007bda:	2102      	movs	r1, #2
 8007bdc:	f899 8000 	ldrb.w	r8, [r9]
 8007be0:	6001      	str	r1, [r0, #0]
 8007be2:	1d04      	adds	r4, r0, #4
 8007be4:	e9d9 ec02 	ldrd	lr, ip, [r9, #8]
 8007be8:	f1b8 0f02 	cmp.w	r8, #2
 8007bec:	e8b9 0062 	ldmia.w	r9!, {r1, r5, r6}
 8007bf0:	c462      	stmia	r4!, {r1, r5, r6}
 8007bf2:	e899 0462 	ldmia.w	r9, {r1, r5, r6, sl}
 8007bf6:	e884 0462 	stmia.w	r4, {r1, r5, r6, sl}
 8007bfa:	f44f 71a0 	mov.w	r1, #320	@ 0x140
 8007bfe:	f8a0 1060 	strh.w	r1, [r0, #96]	@ 0x60
 8007c02:	f10e 011c 	add.w	r1, lr, #28
 8007c06:	bf38      	it	cc
 8007c08:	f10c 0108 	addcc.w	r1, ip, #8
 8007c0c:	6543      	str	r3, [r0, #84]	@ 0x54
 8007c0e:	6582      	str	r2, [r0, #88]	@ 0x58
 8007c10:	65c1      	str	r1, [r0, #92]	@ 0x5c
 8007c12:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007c16:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007c18:	f1b8 0f00 	cmp.w	r8, #0
 8007c1c:	d18f      	bne.n	8007b3e <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x90>
 8007c1e:	f899 3000 	ldrb.w	r3, [r9]
 8007c22:	2b01      	cmp	r3, #1
 8007c24:	f47f af4e 	bne.w	8007ac4 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007c28:	f1bc 0f00 	cmp.w	ip, #0
 8007c2c:	f43f af4a 	beq.w	8007ac4 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007c30:	2302      	movs	r3, #2
 8007c32:	f8d1 c0f4 	ldr.w	ip, [r1, #244]	@ 0xf4
 8007c36:	6003      	str	r3, [r0, #0]
 8007c38:	1d06      	adds	r6, r0, #4
 8007c3a:	f8d9 e00c 	ldr.w	lr, [r9, #12]
 8007c3e:	e8b9 0032 	ldmia.w	r9!, {r1, r4, r5}
 8007c42:	c632      	stmia	r6!, {r1, r4, r5}
 8007c44:	e899 003a 	ldmia.w	r9, {r1, r3, r4, r5}
 8007c48:	c63a      	stmia	r6!, {r1, r3, r4, r5}
 8007c4a:	f44f 71a0 	mov.w	r1, #320	@ 0x140
 8007c4e:	f8a0 1060 	strh.w	r1, [r0, #96]	@ 0x60
 8007c52:	f10e 0108 	add.w	r1, lr, #8
 8007c56:	f8c0 c054 	str.w	ip, [r0, #84]	@ 0x54
 8007c5a:	6582      	str	r2, [r0, #88]	@ 0x58
 8007c5c:	65c1      	str	r1, [r0, #92]	@ 0x5c
 8007c5e:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007c62:	bdf0      	pop	{r4, r5, r6, r7, pc}

08007c64 <<smoltcp::wire::ip::Address as core::fmt::Display>::fmt>:
 8007c64:	b580      	push	{r7, lr}
 8007c66:	466f      	mov	r7, sp
 8007c68:	b084      	sub	sp, #16
 8007c6a:	f244 12cb 	movw	r2, #16843	@ 0x41cb
 8007c6e:	6800      	ldr	r0, [r0, #0]
 8007c70:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007c74:	9001      	str	r0, [sp, #4]
 8007c76:	9203      	str	r2, [sp, #12]
 8007c78:	aa01      	add	r2, sp, #4
 8007c7a:	e9d1 0100 	ldrd	r0, r1, [r1]
 8007c7e:	ab02      	add	r3, sp, #8
 8007c80:	9202      	str	r2, [sp, #8]
 8007c82:	f649 52d9 	movw	r2, #40409	@ 0x9dd9
 8007c86:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007c8a:	f7fb fc39 	bl	8003500 <core::fmt::write>
 8007c8e:	b004      	add	sp, #16
 8007c90:	bd80      	pop	{r7, pc}
 8007c92:	d4d4      	bmi.n	8007c3e <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x190>

08007c94 <smoltcp::wire::ipv4::Repr::emit>:
 8007c94:	b5d0      	push	{r4, r6, r7, lr}
 8007c96:	af02      	add	r7, sp, #8
 8007c98:	2a00      	cmp	r2, #0
 8007c9a:	f000 809a 	beq.w	8007dd2 <smoltcp::wire::ipv4::Repr::emit+0x13e>
 8007c9e:	f04f 0c45 	mov.w	ip, #69	@ 0x45
 8007ca2:	2a01      	cmp	r2, #1
 8007ca4:	f881 c000 	strb.w	ip, [r1]
 8007ca8:	f000 809b 	beq.w	8007de2 <smoltcp::wire::ipv4::Repr::emit+0x14e>
 8007cac:	f04f 0c00 	mov.w	ip, #0
 8007cb0:	2a03      	cmp	r2, #3
 8007cb2:	f881 c001 	strb.w	ip, [r1, #1]
 8007cb6:	d964      	bls.n	8007d82 <smoltcp::wire::ipv4::Repr::emit+0xee>
 8007cb8:	f8b0 c008 	ldrh.w	ip, [r0, #8]
 8007cbc:	2a05      	cmp	r2, #5
 8007cbe:	f10c 0c14 	add.w	ip, ip, #20
 8007cc2:	fa9c fc8c 	rev.w	ip, ip
 8007cc6:	ea4f 4c1c 	mov.w	ip, ip, lsr #16
 8007cca:	f8a1 c002 	strh.w	ip, [r1, #2]
 8007cce:	d960      	bls.n	8007d92 <smoltcp::wire::ipv4::Repr::emit+0xfe>
 8007cd0:	f04f 0c00 	mov.w	ip, #0
 8007cd4:	2a07      	cmp	r2, #7
 8007cd6:	f8a1 c004 	strh.w	ip, [r1, #4]
 8007cda:	d962      	bls.n	8007da2 <smoltcp::wire::ipv4::Repr::emit+0x10e>
 8007cdc:	f04f 0c40 	mov.w	ip, #64	@ 0x40
 8007ce0:	2a08      	cmp	r2, #8
 8007ce2:	f8a1 c006 	strh.w	ip, [r1, #6]
 8007ce6:	f000 8084 	beq.w	8007df2 <smoltcp::wire::ipv4::Repr::emit+0x15e>
 8007cea:	f890 c00d 	ldrb.w	ip, [r0, #13]
 8007cee:	f890 e00c 	ldrb.w	lr, [r0, #12]
 8007cf2:	f881 e008 	strb.w	lr, [r1, #8]
 8007cf6:	e8df f00c 	tbb	[pc, ip]
 8007cfa:	2424      	.short	0x2424
 8007cfc:	190d0724 	.word	0x190d0724
 8007d00:	1322161c 	.word	0x1322161c
 8007d04:	00100a1f 	.word	0x00100a1f
 8007d08:	f04f 0c06 	mov.w	ip, #6
 8007d0c:	e019      	b.n	8007d42 <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007d0e:	f04f 0c3c 	mov.w	ip, #60	@ 0x3c
 8007d12:	e016      	b.n	8007d42 <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007d14:	f04f 0c11 	mov.w	ip, #17
 8007d18:	e013      	b.n	8007d42 <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007d1a:	f890 c00e 	ldrb.w	ip, [r0, #14]
 8007d1e:	e010      	b.n	8007d42 <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007d20:	f04f 0c3a 	mov.w	ip, #58	@ 0x3a
 8007d24:	e00d      	b.n	8007d42 <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007d26:	f04f 0c32 	mov.w	ip, #50	@ 0x32
 8007d2a:	e00a      	b.n	8007d42 <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007d2c:	f04f 0c2b 	mov.w	ip, #43	@ 0x2b
 8007d30:	e007      	b.n	8007d42 <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007d32:	f04f 0c2c 	mov.w	ip, #44	@ 0x2c
 8007d36:	e004      	b.n	8007d42 <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007d38:	f04f 0c3b 	mov.w	ip, #59	@ 0x3b
 8007d3c:	e001      	b.n	8007d42 <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007d3e:	f04f 0c33 	mov.w	ip, #51	@ 0x33
 8007d42:	2a09      	cmp	r2, #9
 8007d44:	d95d      	bls.n	8007e02 <smoltcp::wire::ipv4::Repr::emit+0x16e>
 8007d46:	2a0f      	cmp	r2, #15
 8007d48:	f881 c009 	strb.w	ip, [r1, #9]
 8007d4c:	d931      	bls.n	8007db2 <smoltcp::wire::ipv4::Repr::emit+0x11e>
 8007d4e:	f8d0 c000 	ldr.w	ip, [r0]
 8007d52:	2a13      	cmp	r2, #19
 8007d54:	f8c1 c00c 	str.w	ip, [r1, #12]
 8007d58:	d933      	bls.n	8007dc2 <smoltcp::wire::ipv4::Repr::emit+0x12e>
 8007d5a:	6840      	ldr	r0, [r0, #4]
 8007d5c:	f013 0ffd 	tst.w	r3, #253	@ 0xfd
 8007d60:	6108      	str	r0, [r1, #16]
 8007d62:	f04f 0000 	mov.w	r0, #0
 8007d66:	d10a      	bne.n	8007d7e <smoltcp::wire::ipv4::Repr::emit+0xea>
 8007d68:	8148      	strh	r0, [r1, #10]
 8007d6a:	4608      	mov	r0, r1
 8007d6c:	460c      	mov	r4, r1
 8007d6e:	2114      	movs	r1, #20
 8007d70:	f000 f9f1 	bl	8008156 <smoltcp::wire::ip::checksum::data>
 8007d74:	43c0      	mvns	r0, r0
 8007d76:	ba00      	rev	r0, r0
 8007d78:	0c00      	lsrs	r0, r0, #16
 8007d7a:	8160      	strh	r0, [r4, #10]
 8007d7c:	bdd0      	pop	{r4, r6, r7, pc}
 8007d7e:	8148      	strh	r0, [r1, #10]
 8007d80:	bdd0      	pop	{r4, r6, r7, pc}
 8007d82:	f64a 630c 	movw	r3, #44556	@ 0xae0c
 8007d86:	2002      	movs	r0, #2
 8007d88:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007d8c:	2104      	movs	r1, #4
 8007d8e:	f7fb fd36 	bl	80037fe <core::slice::index::slice_index_fail>
 8007d92:	f64a 634c 	movw	r3, #44620	@ 0xae4c
 8007d96:	2004      	movs	r0, #4
 8007d98:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007d9c:	2106      	movs	r1, #6
 8007d9e:	f7fb fd2e 	bl	80037fe <core::slice::index::slice_index_fail>
 8007da2:	f64a 53cc 	movw	r3, #44492	@ 0xadcc
 8007da6:	2006      	movs	r0, #6
 8007da8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007dac:	2108      	movs	r1, #8
 8007dae:	f7fb fd26 	bl	80037fe <core::slice::index::slice_index_fail>
 8007db2:	f64a 53fc 	movw	r3, #44540	@ 0xadfc
 8007db6:	200c      	movs	r0, #12
 8007db8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007dbc:	2110      	movs	r1, #16
 8007dbe:	f7fb fd1e 	bl	80037fe <core::slice::index::slice_index_fail>
 8007dc2:	f64a 53ec 	movw	r3, #44524	@ 0xadec
 8007dc6:	2010      	movs	r0, #16
 8007dc8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007dcc:	2114      	movs	r1, #20
 8007dce:	f7fb fd16 	bl	80037fe <core::slice::index::slice_index_fail>
 8007dd2:	f64a 52dc 	movw	r2, #44508	@ 0xaddc
 8007dd6:	2000      	movs	r0, #0
 8007dd8:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007ddc:	2100      	movs	r1, #0
 8007dde:	f7fb fd8d 	bl	80038fc <core::panicking::panic_bounds_check>
 8007de2:	f64a 623c 	movw	r2, #44604	@ 0xae3c
 8007de6:	2001      	movs	r0, #1
 8007de8:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007dec:	2101      	movs	r1, #1
 8007dee:	f7fb fd85 	bl	80038fc <core::panicking::panic_bounds_check>
 8007df2:	f64a 625c 	movw	r2, #44636	@ 0xae5c
 8007df6:	2008      	movs	r0, #8
 8007df8:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007dfc:	2108      	movs	r1, #8
 8007dfe:	f7fb fd7d 	bl	80038fc <core::panicking::panic_bounds_check>
 8007e02:	f64a 621c 	movw	r2, #44572	@ 0xae1c
 8007e06:	2009      	movs	r0, #9
 8007e08:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007e0c:	2109      	movs	r1, #9
 8007e0e:	f7fb fd75 	bl	80038fc <core::panicking::panic_bounds_check>

08007e12 <defmt::export::fmt>:
 8007e12:	b5b0      	push	{r4, r5, r7, lr}
 8007e14:	af02      	add	r7, sp, #8
 8007e16:	b082      	sub	sp, #8
 8007e18:	4605      	mov	r5, r0
 8007e1a:	f240 0003 	movw	r0, #3
 8007e1e:	f2c0 0000 	movt	r0, #0
 8007e22:	460c      	mov	r4, r1
 8007e24:	f827 0c0e 	strh.w	r0, [r7, #-14]
 8007e28:	f1a7 000e 	sub.w	r0, r7, #14
 8007e2c:	2102      	movs	r1, #2
 8007e2e:	f7fc fcc3 	bl	80047b8 <_defmt_write>
 8007e32:	a801      	add	r0, sp, #4
 8007e34:	2104      	movs	r1, #4
 8007e36:	9401      	str	r4, [sp, #4]
 8007e38:	f7fc fcbe 	bl	80047b8 <_defmt_write>
 8007e3c:	4628      	mov	r0, r5
 8007e3e:	4621      	mov	r1, r4
 8007e40:	f7fc fcba 	bl	80047b8 <_defmt_write>
 8007e44:	b002      	add	sp, #8
 8007e46:	bdb0      	pop	{r4, r5, r7, pc}

08007e48 <defmt::export::fmt>:
 8007e48:	b5d0      	push	{r4, r6, r7, lr}
 8007e4a:	af02      	add	r7, sp, #8
 8007e4c:	b082      	sub	sp, #8
 8007e4e:	4604      	mov	r4, r0
 8007e50:	f240 0002 	movw	r0, #2
 8007e54:	f2c0 0000 	movt	r0, #0
 8007e58:	2102      	movs	r1, #2
 8007e5a:	f8ad 0004 	strh.w	r0, [sp, #4]
 8007e5e:	a801      	add	r0, sp, #4
 8007e60:	f7fc fcaa 	bl	80047b8 <_defmt_write>
 8007e64:	f1a7 0009 	sub.w	r0, r7, #9
 8007e68:	2101      	movs	r1, #1
 8007e6a:	f807 4c09 	strb.w	r4, [r7, #-9]
 8007e6e:	f7fc fca3 	bl	80047b8 <_defmt_write>
 8007e72:	b002      	add	sp, #8
 8007e74:	bdd0      	pop	{r4, r6, r7, pc}

08007e76 <defmt::export::fmt>:
 8007e76:	b5d0      	push	{r4, r6, r7, lr}
 8007e78:	af02      	add	r7, sp, #8
 8007e7a:	b084      	sub	sp, #16
 8007e7c:	4604      	mov	r4, r0
 8007e7e:	f240 0007 	movw	r0, #7
 8007e82:	f2c0 0000 	movt	r0, #0
 8007e86:	2102      	movs	r1, #2
 8007e88:	f827 0c16 	strh.w	r0, [r7, #-22]
 8007e8c:	f1a7 0016 	sub.w	r0, r7, #22
 8007e90:	f7fc fc92 	bl	80047b8 <_defmt_write>
 8007e94:	f240 003d 	movw	r0, #61	@ 0x3d
 8007e98:	2102      	movs	r1, #2
 8007e9a:	f2c0 0000 	movt	r0, #0
 8007e9e:	f8ad 0004 	strh.w	r0, [sp, #4]
 8007ea2:	a801      	add	r0, sp, #4
 8007ea4:	f7fc fc88 	bl	80047b8 <_defmt_write>
 8007ea8:	f240 0008 	movw	r0, #8
 8007eac:	2102      	movs	r1, #2
 8007eae:	f2c0 0000 	movt	r0, #0
 8007eb2:	f827 0c12 	strh.w	r0, [r7, #-18]
 8007eb6:	f1a7 0012 	sub.w	r0, r7, #18
 8007eba:	f7fc fc7d 	bl	80047b8 <_defmt_write>
 8007ebe:	a802      	add	r0, sp, #8
 8007ec0:	2104      	movs	r1, #4
 8007ec2:	9402      	str	r4, [sp, #8]
 8007ec4:	f7fc fc78 	bl	80047b8 <_defmt_write>
 8007ec8:	2000      	movs	r0, #0
 8007eca:	2102      	movs	r1, #2
 8007ecc:	f827 0c0a 	strh.w	r0, [r7, #-10]
 8007ed0:	f1a7 000a 	sub.w	r0, r7, #10
 8007ed4:	f7fc fc70 	bl	80047b8 <_defmt_write>
 8007ed8:	b004      	add	sp, #16
 8007eda:	bdd0      	pop	{r4, r6, r7, pc}

08007edc <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with>:
 8007edc:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007ede:	af03      	add	r7, sp, #12
 8007ee0:	f84d bd04 	str.w	fp, [sp, #-4]!
 8007ee4:	68ce      	ldr	r6, [r1, #12]
 8007ee6:	2e00      	cmp	r6, #0
 8007ee8:	bf04      	itt	eq
 8007eea:	2500      	moveq	r5, #0
 8007eec:	608d      	streq	r5, [r1, #8]
 8007eee:	684c      	ldr	r4, [r1, #4]
 8007ef0:	b134      	cbz	r4, 8007f00 <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x24>
 8007ef2:	688d      	ldr	r5, [r1, #8]
 8007ef4:	4435      	add	r5, r6
 8007ef6:	fbb5 f3f4 	udiv	r3, r5, r4
 8007efa:	fb03 5c14 	mls	ip, r3, r4, r5
 8007efe:	e001      	b.n	8007f04 <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x28>
 8007f00:	f04f 0c00 	mov.w	ip, #0
 8007f04:	1ba5      	subs	r5, r4, r6
 8007f06:	eba4 030c 	sub.w	r3, r4, ip
 8007f0a:	42ab      	cmp	r3, r5
 8007f0c:	bf38      	it	cc
 8007f0e:	461d      	movcc	r5, r3
 8007f10:	eb15 0e0c 	adds.w	lr, r5, ip
 8007f14:	d20e      	bcs.n	8007f34 <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x58>
 8007f16:	45a6      	cmp	lr, r4
 8007f18:	d80c      	bhi.n	8007f34 <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x58>
 8007f1a:	680b      	ldr	r3, [r1, #0]
 8007f1c:	4295      	cmp	r5, r2
 8007f1e:	bf38      	it	cc
 8007f20:	462a      	movcc	r2, r5
 8007f22:	6082      	str	r2, [r0, #8]
 8007f24:	4463      	add	r3, ip
 8007f26:	e9c0 2300 	strd	r2, r3, [r0]
 8007f2a:	1990      	adds	r0, r2, r6
 8007f2c:	60c8      	str	r0, [r1, #12]
 8007f2e:	f85d bb04 	ldr.w	fp, [sp], #4
 8007f32:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007f34:	f64a 7304 	movw	r3, #44804	@ 0xaf04
 8007f38:	4660      	mov	r0, ip
 8007f3a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007f3e:	4671      	mov	r1, lr
 8007f40:	4622      	mov	r2, r4
 8007f42:	f7fb fc5c 	bl	80037fe <core::slice::index::slice_index_fail>

08007f46 <smoltcp::socket::udp::Socket::accepts>:
 8007f46:	4684      	mov	ip, r0
 8007f48:	8800      	ldrh	r0, [r0, #0]
 8007f4a:	b29b      	uxth	r3, r3
 8007f4c:	4298      	cmp	r0, r3
 8007f4e:	d10d      	bne.n	8007f6c <smoltcp::socket::udp::Socket::accepts+0x26>
 8007f50:	f89c 3002 	ldrb.w	r3, [ip, #2]
 8007f54:	2001      	movs	r0, #1
 8007f56:	2b01      	cmp	r3, #1
 8007f58:	bf18      	it	ne
 8007f5a:	4770      	bxne	lr
 8007f5c:	f8dc 3003 	ldr.w	r3, [ip, #3]
 8007f60:	4293      	cmp	r3, r2
 8007f62:	bf18      	it	ne
 8007f64:	f112 0301 	addsne.w	r3, r2, #1
 8007f68:	d102      	bne.n	8007f70 <smoltcp::socket::udp::Socket::accepts+0x2a>
 8007f6a:	4770      	bx	lr
 8007f6c:	2000      	movs	r0, #0
 8007f6e:	4770      	bx	lr
 8007f70:	b5d0      	push	{r4, r6, r7, lr}
 8007f72:	af02      	add	r7, sp, #8
 8007f74:	f8d1 00f0 	ldr.w	r0, [r1, #240]	@ 0xf0
 8007f78:	b328      	cbz	r0, 8007fc6 <smoltcp::socket::udp::Socket::accepts+0x80>
 8007f7a:	eb00 0080 	add.w	r0, r0, r0, lsl #2
 8007f7e:	f04f 3cff 	mov.w	ip, #4294967295	@ 0xffffffff
 8007f82:	4408      	add	r0, r1
 8007f84:	31f4      	adds	r1, #244	@ 0xf4
 8007f86:	f100 0ef4 	add.w	lr, r0, #244	@ 0xf4
 8007f8a:	e00c      	b.n	8007fa6 <smoltcp::socket::udp::Socket::accepts+0x60>
 8007f8c:	2400      	movs	r4, #0
 8007f8e:	f003 031f 	and.w	r3, r3, #31
 8007f92:	4020      	ands	r0, r4
 8007f94:	fa2c f303 	lsr.w	r3, ip, r3
 8007f98:	ba1b      	rev	r3, r3
 8007f9a:	4318      	orrs	r0, r3
 8007f9c:	4282      	cmp	r2, r0
 8007f9e:	d019      	beq.n	8007fd4 <smoltcp::socket::udp::Socket::accepts+0x8e>
 8007fa0:	3105      	adds	r1, #5
 8007fa2:	4571      	cmp	r1, lr
 8007fa4:	d00f      	beq.n	8007fc6 <smoltcp::socket::udp::Socket::accepts+0x80>
 8007fa6:	790b      	ldrb	r3, [r1, #4]
 8007fa8:	6808      	ldr	r0, [r1, #0]
 8007faa:	2b00      	cmp	r3, #0
 8007fac:	d0ee      	beq.n	8007f8c <smoltcp::socket::udp::Socket::accepts+0x46>
 8007fae:	f1a3 041f 	sub.w	r4, r3, #31
 8007fb2:	b2e4      	uxtb	r4, r4
 8007fb4:	2c02      	cmp	r4, #2
 8007fb6:	d3f3      	bcc.n	8007fa0 <smoltcp::socket::udp::Socket::accepts+0x5a>
 8007fb8:	425c      	negs	r4, r3
 8007fba:	f004 041f 	and.w	r4, r4, #31
 8007fbe:	fa0c f404 	lsl.w	r4, ip, r4
 8007fc2:	ba24      	rev	r4, r4
 8007fc4:	e7e3      	b.n	8007f8e <smoltcp::socket::udp::Socket::accepts+0x48>
 8007fc6:	f002 00f0 	and.w	r0, r2, #240	@ 0xf0
 8007fca:	38e0      	subs	r0, #224	@ 0xe0
 8007fcc:	fab0 f080 	clz	r0, r0
 8007fd0:	0940      	lsrs	r0, r0, #5
 8007fd2:	bdd0      	pop	{r4, r6, r7, pc}
 8007fd4:	2001      	movs	r0, #1
 8007fd6:	bdd0      	pop	{r4, r6, r7, pc}

08007fd8 <smoltcp::socket::udp::Socket::process>:
 8007fd8:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007fda:	af03      	add	r7, sp, #12
 8007fdc:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8007fe0:	b085      	sub	sp, #20
 8007fe2:	f8d7 a010 	ldr.w	sl, [r7, #16]
 8007fe6:	69c5      	ldr	r5, [r0, #28]
 8007fe8:	4555      	cmp	r5, sl
 8007fea:	d34f      	bcc.n	800808c <smoltcp::socket::udp::Socket::process+0xb4>
 8007fec:	f8d0 b00c 	ldr.w	fp, [r0, #12]
 8007ff0:	f8d0 c014 	ldr.w	ip, [r0, #20]
 8007ff4:	45e3      	cmp	fp, ip
 8007ff6:	d049      	beq.n	800808c <smoltcp::socket::udp::Socket::process+0xb4>
 8007ff8:	6a44      	ldr	r4, [r0, #36]	@ 0x24
 8007ffa:	e9d7 9802 	ldrd	r9, r8, [r7, #8]
 8007ffe:	eba5 0e04 	sub.w	lr, r5, r4
 8008002:	2c00      	cmp	r4, #0
 8008004:	bf04      	itt	eq
 8008006:	2600      	moveq	r6, #0
 8008008:	6206      	streq	r6, [r0, #32]
 800800a:	b135      	cbz	r5, 800801a <smoltcp::socket::udp::Socket::process+0x42>
 800800c:	6a06      	ldr	r6, [r0, #32]
 800800e:	4434      	add	r4, r6
 8008010:	fbb4 f6f5 	udiv	r6, r4, r5
 8008014:	fb06 4415 	mls	r4, r6, r5, r4
 8008018:	e000      	b.n	800801c <smoltcp::socket::udp::Socket::process+0x44>
 800801a:	2400      	movs	r4, #0
 800801c:	1b2c      	subs	r4, r5, r4
 800801e:	4675      	mov	r5, lr
 8008020:	4574      	cmp	r4, lr
 8008022:	bf38      	it	cc
 8008024:	4625      	movcc	r5, r4
 8008026:	45d6      	cmp	lr, sl
 8008028:	d330      	bcc.n	800808c <smoltcp::socket::udp::Socket::process+0xb4>
 800802a:	4554      	cmp	r4, sl
 800802c:	d22c      	bcs.n	8008088 <smoltcp::socket::udp::Socket::process+0xb0>
 800802e:	ebae 0605 	sub.w	r6, lr, r5
 8008032:	4556      	cmp	r6, sl
 8008034:	d32a      	bcc.n	800808c <smoltcp::socket::udp::Socket::process+0xb4>
 8008036:	f1bb 0f00 	cmp.w	fp, #0
 800803a:	e9cd 1300 	strd	r1, r3, [sp]
 800803e:	f000 8084 	beq.w	800814a <smoltcp::socket::udp::Socket::process+0x172>
 8008042:	6903      	ldr	r3, [r0, #16]
 8008044:	4614      	mov	r4, r2
 8008046:	6882      	ldr	r2, [r0, #8]
 8008048:	f100 0118 	add.w	r1, r0, #24
 800804c:	4463      	add	r3, ip
 800804e:	fbb3 f6fb 	udiv	r6, r3, fp
 8008052:	fb06 331b 	mls	r3, r6, fp, r3
 8008056:	f10c 0601 	add.w	r6, ip, #1
 800805a:	6146      	str	r6, [r0, #20]
 800805c:	4606      	mov	r6, r0
 800805e:	eb03 0383 	add.w	r3, r3, r3, lsl #2
 8008062:	f842 5023 	str.w	r5, [r2, r3, lsl #2]
 8008066:	eb02 0283 	add.w	r2, r2, r3, lsl #2
 800806a:	2302      	movs	r3, #2
 800806c:	7393      	strb	r3, [r2, #14]
 800806e:	aa02      	add	r2, sp, #8
 8008070:	4610      	mov	r0, r2
 8008072:	462a      	mov	r2, r5
 8008074:	f7ff ff32 	bl	8007edc <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with>
 8008078:	4630      	mov	r0, r6
 800807a:	f8d6 b00c 	ldr.w	fp, [r6, #12]
 800807e:	f8d6 c014 	ldr.w	ip, [r6, #20]
 8008082:	4622      	mov	r2, r4
 8008084:	e9dd 1300 	ldrd	r1, r3, [sp]
 8008088:	45e3      	cmp	fp, ip
 800808a:	d103      	bne.n	8008094 <smoltcp::socket::udp::Socket::process+0xbc>
 800808c:	b005      	add	sp, #20
 800808e:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8008092:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8008094:	f1bb 0f00 	cmp.w	fp, #0
 8008098:	d057      	beq.n	800814a <smoltcp::socket::udp::Socket::process+0x172>
 800809a:	6905      	ldr	r5, [r0, #16]
 800809c:	6884      	ldr	r4, [r0, #8]
 800809e:	4465      	add	r5, ip
 80080a0:	f8d0 e024 	ldr.w	lr, [r0, #36]	@ 0x24
 80080a4:	fbb5 f6fb 	udiv	r6, r5, fp
 80080a8:	fb06 551b 	mls	r5, r6, fp, r5
 80080ac:	f10c 0601 	add.w	r6, ip, #1
 80080b0:	6146      	str	r6, [r0, #20]
 80080b2:	f1be 0f00 	cmp.w	lr, #0
 80080b6:	eb05 0585 	add.w	r5, r5, r5, lsl #2
 80080ba:	eb04 0685 	add.w	r6, r4, r5, lsl #2
 80080be:	f844 a025 	str.w	sl, [r4, r5, lsl #2]
 80080c2:	f8c6 300f 	str.w	r3, [r6, #15]
 80080c6:	f04f 0301 	mov.w	r3, #1
 80080ca:	73b3      	strb	r3, [r6, #14]
 80080cc:	f8a6 900c 	strh.w	r9, [r6, #12]
 80080d0:	e9c6 1201 	strd	r1, r2, [r6, #4]
 80080d4:	bf04      	itt	eq
 80080d6:	2100      	moveq	r1, #0
 80080d8:	6201      	streq	r1, [r0, #32]
 80080da:	69c2      	ldr	r2, [r0, #28]
 80080dc:	b132      	cbz	r2, 80080ec <smoltcp::socket::udp::Socket::process+0x114>
 80080de:	6a01      	ldr	r1, [r0, #32]
 80080e0:	4471      	add	r1, lr
 80080e2:	fbb1 f3f2 	udiv	r3, r1, r2
 80080e6:	fb03 1512 	mls	r5, r3, r2, r1
 80080ea:	e000      	b.n	80080ee <smoltcp::socket::udp::Socket::process+0x116>
 80080ec:	2500      	movs	r5, #0
 80080ee:	eba2 030e 	sub.w	r3, r2, lr
 80080f2:	1b51      	subs	r1, r2, r5
 80080f4:	4299      	cmp	r1, r3
 80080f6:	bf38      	it	cc
 80080f8:	460b      	movcc	r3, r1
 80080fa:	1959      	adds	r1, r3, r5
 80080fc:	d21e      	bcs.n	800813c <smoltcp::socket::udp::Socket::process+0x164>
 80080fe:	4291      	cmp	r1, r2
 8008100:	d81c      	bhi.n	800813c <smoltcp::socket::udp::Socket::process+0x164>
 8008102:	6982      	ldr	r2, [r0, #24]
 8008104:	4553      	cmp	r3, sl
 8008106:	4651      	mov	r1, sl
 8008108:	bf38      	it	cc
 800810a:	4619      	movcc	r1, r3
 800810c:	459a      	cmp	sl, r3
 800810e:	eb01 060e 	add.w	r6, r1, lr
 8008112:	6246      	str	r6, [r0, #36]	@ 0x24
 8008114:	bf9f      	itttt	ls
 8008116:	1950      	addls	r0, r2, r5
 8008118:	4641      	movls	r1, r8
 800811a:	4652      	movls	r2, sl
 800811c:	b005      	addls	sp, #20
 800811e:	bf9e      	ittt	ls
 8008120:	e8bd 0f00 	ldmials.w	sp!, {r8, r9, sl, fp}
 8008124:	e8bd 40f0 	ldmials.w	sp!, {r4, r5, r6, r7, lr}
 8008128:	f001 b966 	bls.w	80093f8 <__aeabi_memcpy>
 800812c:	f64a 7248 	movw	r2, #44872	@ 0xaf48
 8008130:	4608      	mov	r0, r1
 8008132:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8008136:	4651      	mov	r1, sl
 8008138:	f7fb ffee 	bl	8004118 <core::slice::copy_from_slice_impl::len_mismatch_fail>
 800813c:	f64a 7304 	movw	r3, #44804	@ 0xaf04
 8008140:	4628      	mov	r0, r5
 8008142:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8008146:	f7fb fb5a 	bl	80037fe <core::slice::index::slice_index_fail>
 800814a:	f64a 60bc 	movw	r0, #44732	@ 0xaebc
 800814e:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8008152:	f7fc f830 	bl	80041b6 <core::panicking::panic_const::panic_const_rem_by_zero>

08008156 <smoltcp::wire::ip::checksum::data>:
 8008156:	b5f0      	push	{r4, r5, r6, r7, lr}
 8008158:	af03      	add	r7, sp, #12
 800815a:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 800815e:	2920      	cmp	r1, #32
 8008160:	f0c0 80b5 	bcc.w	80082ce <smoltcp::wire::ip::checksum::data+0x178>
 8008164:	f1a1 0c20 	sub.w	ip, r1, #32
 8008168:	ea5f 125c 	movs.w	r2, ip, lsr #5
 800816c:	f000 80b1 	beq.w	80082d2 <smoltcp::wire::ip::checksum::data+0x17c>
 8008170:	3201      	adds	r2, #1
 8008172:	f022 0301 	bic.w	r3, r2, #1
 8008176:	2200      	movs	r2, #0
 8008178:	f830 6b40 	ldrh.w	r6, [r0], #64
 800817c:	3940      	subs	r1, #64	@ 0x40
 800817e:	3b02      	subs	r3, #2
 8008180:	f830 4c3e 	ldrh.w	r4, [r0, #-62]
 8008184:	ba36      	rev	r6, r6
 8008186:	f830 5c3c 	ldrh.w	r5, [r0, #-60]
 800818a:	ba24      	rev	r4, r4
 800818c:	eb02 4216 	add.w	r2, r2, r6, lsr #16
 8008190:	f830 6c3a 	ldrh.w	r6, [r0, #-58]
 8008194:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008198:	ba2c      	rev	r4, r5
 800819a:	f830 5c38 	ldrh.w	r5, [r0, #-56]
 800819e:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081a2:	ba34      	rev	r4, r6
 80081a4:	f830 6c36 	ldrh.w	r6, [r0, #-54]
 80081a8:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081ac:	ba2c      	rev	r4, r5
 80081ae:	f830 5c34 	ldrh.w	r5, [r0, #-52]
 80081b2:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081b6:	ba34      	rev	r4, r6
 80081b8:	f830 6c32 	ldrh.w	r6, [r0, #-50]
 80081bc:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081c0:	ba2c      	rev	r4, r5
 80081c2:	f830 5c30 	ldrh.w	r5, [r0, #-48]
 80081c6:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081ca:	ba34      	rev	r4, r6
 80081cc:	f830 6c2e 	ldrh.w	r6, [r0, #-46]
 80081d0:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081d4:	ba2c      	rev	r4, r5
 80081d6:	f830 5c2c 	ldrh.w	r5, [r0, #-44]
 80081da:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081de:	ba34      	rev	r4, r6
 80081e0:	f830 6c2a 	ldrh.w	r6, [r0, #-42]
 80081e4:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081e8:	ba2c      	rev	r4, r5
 80081ea:	f830 5c28 	ldrh.w	r5, [r0, #-40]
 80081ee:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081f2:	ba34      	rev	r4, r6
 80081f4:	f830 6c26 	ldrh.w	r6, [r0, #-38]
 80081f8:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081fc:	ba2c      	rev	r4, r5
 80081fe:	f830 5c24 	ldrh.w	r5, [r0, #-36]
 8008202:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008206:	ba34      	rev	r4, r6
 8008208:	f830 6c22 	ldrh.w	r6, [r0, #-34]
 800820c:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008210:	ba2c      	rev	r4, r5
 8008212:	f830 5c20 	ldrh.w	r5, [r0, #-32]
 8008216:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800821a:	ba34      	rev	r4, r6
 800821c:	f830 6c1e 	ldrh.w	r6, [r0, #-30]
 8008220:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008224:	ba2c      	rev	r4, r5
 8008226:	f830 5c1c 	ldrh.w	r5, [r0, #-28]
 800822a:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800822e:	ba34      	rev	r4, r6
 8008230:	f830 6c1a 	ldrh.w	r6, [r0, #-26]
 8008234:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008238:	ba2c      	rev	r4, r5
 800823a:	f830 5c18 	ldrh.w	r5, [r0, #-24]
 800823e:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008242:	ba34      	rev	r4, r6
 8008244:	f830 6c16 	ldrh.w	r6, [r0, #-22]
 8008248:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800824c:	ba2c      	rev	r4, r5
 800824e:	f830 5c14 	ldrh.w	r5, [r0, #-20]
 8008252:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008256:	ba34      	rev	r4, r6
 8008258:	f830 6c12 	ldrh.w	r6, [r0, #-18]
 800825c:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008260:	ba2c      	rev	r4, r5
 8008262:	f830 5c10 	ldrh.w	r5, [r0, #-16]
 8008266:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800826a:	ba34      	rev	r4, r6
 800826c:	f830 6c0e 	ldrh.w	r6, [r0, #-14]
 8008270:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008274:	ba2c      	rev	r4, r5
 8008276:	f830 5c0c 	ldrh.w	r5, [r0, #-12]
 800827a:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800827e:	ba34      	rev	r4, r6
 8008280:	f830 6c0a 	ldrh.w	r6, [r0, #-10]
 8008284:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008288:	ba2c      	rev	r4, r5
 800828a:	f830 5c08 	ldrh.w	r5, [r0, #-8]
 800828e:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008292:	ba34      	rev	r4, r6
 8008294:	f830 9c06 	ldrh.w	r9, [r0, #-6]
 8008298:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800829c:	ba2c      	rev	r4, r5
 800829e:	f830 8c04 	ldrh.w	r8, [r0, #-4]
 80082a2:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80082a6:	fa99 f489 	rev.w	r4, r9
 80082aa:	f830 ec02 	ldrh.w	lr, [r0, #-2]
 80082ae:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80082b2:	fa98 f488 	rev.w	r4, r8
 80082b6:	fa9e f68e 	rev.w	r6, lr
 80082ba:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80082be:	eb02 4216 	add.w	r2, r2, r6, lsr #16
 80082c2:	f47f af59 	bne.w	8008178 <smoltcp::wire::ip::checksum::data+0x22>
 80082c6:	ea5f 638c 	movs.w	r3, ip, lsl #26
 80082ca:	d506      	bpl.n	80082da <smoltcp::wire::ip::checksum::data+0x184>
 80082cc:	e058      	b.n	8008380 <smoltcp::wire::ip::checksum::data+0x22a>
 80082ce:	2200      	movs	r2, #0
 80082d0:	e056      	b.n	8008380 <smoltcp::wire::ip::checksum::data+0x22a>
 80082d2:	2200      	movs	r2, #0
 80082d4:	ea5f 638c 	movs.w	r3, ip, lsl #26
 80082d8:	d452      	bmi.n	8008380 <smoltcp::wire::ip::checksum::data+0x22a>
 80082da:	f830 6b20 	ldrh.w	r6, [r0], #32
 80082de:	3920      	subs	r1, #32
 80082e0:	f830 3c1e 	ldrh.w	r3, [r0, #-30]
 80082e4:	ba36      	rev	r6, r6
 80082e6:	f830 5c1c 	ldrh.w	r5, [r0, #-28]
 80082ea:	ba1b      	rev	r3, r3
 80082ec:	eb02 4216 	add.w	r2, r2, r6, lsr #16
 80082f0:	f830 6c1a 	ldrh.w	r6, [r0, #-26]
 80082f4:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 80082f8:	ba2b      	rev	r3, r5
 80082fa:	f830 5c18 	ldrh.w	r5, [r0, #-24]
 80082fe:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008302:	ba33      	rev	r3, r6
 8008304:	f830 6c16 	ldrh.w	r6, [r0, #-22]
 8008308:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 800830c:	ba2b      	rev	r3, r5
 800830e:	f830 5c14 	ldrh.w	r5, [r0, #-20]
 8008312:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008316:	ba33      	rev	r3, r6
 8008318:	f830 6c12 	ldrh.w	r6, [r0, #-18]
 800831c:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008320:	ba2b      	rev	r3, r5
 8008322:	f830 5c10 	ldrh.w	r5, [r0, #-16]
 8008326:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 800832a:	ba33      	rev	r3, r6
 800832c:	f830 6c0e 	ldrh.w	r6, [r0, #-14]
 8008330:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008334:	ba2b      	rev	r3, r5
 8008336:	f830 5c0c 	ldrh.w	r5, [r0, #-12]
 800833a:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 800833e:	ba33      	rev	r3, r6
 8008340:	f830 6c0a 	ldrh.w	r6, [r0, #-10]
 8008344:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008348:	ba2b      	rev	r3, r5
 800834a:	f830 5c08 	ldrh.w	r5, [r0, #-8]
 800834e:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008352:	ba33      	rev	r3, r6
 8008354:	f830 4c06 	ldrh.w	r4, [r0, #-6]
 8008358:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 800835c:	ba2b      	rev	r3, r5
 800835e:	f830 ec04 	ldrh.w	lr, [r0, #-4]
 8008362:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008366:	ba23      	rev	r3, r4
 8008368:	f830 cc02 	ldrh.w	ip, [r0, #-2]
 800836c:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008370:	fa9e f38e 	rev.w	r3, lr
 8008374:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008378:	fa9c f38c 	rev.w	r3, ip
 800837c:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008380:	2902      	cmp	r1, #2
 8008382:	d306      	bcc.n	8008392 <smoltcp::wire::ip::checksum::data+0x23c>
 8008384:	1e8d      	subs	r5, r1, #2
 8008386:	43eb      	mvns	r3, r5
 8008388:	f013 0f06 	tst.w	r3, #6
 800838c:	d103      	bne.n	8008396 <smoltcp::wire::ip::checksum::data+0x240>
 800838e:	4603      	mov	r3, r0
 8008390:	e01a      	b.n	80083c8 <smoltcp::wire::ip::checksum::data+0x272>
 8008392:	4603      	mov	r3, r0
 8008394:	e02f      	b.n	80083f6 <smoltcp::wire::ip::checksum::data+0x2a0>
 8008396:	4603      	mov	r3, r0
 8008398:	f015 0f06 	tst.w	r5, #6
 800839c:	f833 6b02 	ldrh.w	r6, [r3], #2
 80083a0:	ba36      	rev	r6, r6
 80083a2:	eb02 4216 	add.w	r2, r2, r6, lsr #16
 80083a6:	d00c      	beq.n	80083c2 <smoltcp::wire::ip::checksum::data+0x26c>
 80083a8:	8843      	ldrh	r3, [r0, #2]
 80083aa:	ba1b      	rev	r3, r3
 80083ac:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 80083b0:	f005 0306 	and.w	r3, r5, #6
 80083b4:	2b02      	cmp	r3, #2
 80083b6:	d12b      	bne.n	8008410 <smoltcp::wire::ip::checksum::data+0x2ba>
 80083b8:	1d03      	adds	r3, r0, #4
 80083ba:	3904      	subs	r1, #4
 80083bc:	2d06      	cmp	r5, #6
 80083be:	d203      	bcs.n	80083c8 <smoltcp::wire::ip::checksum::data+0x272>
 80083c0:	e019      	b.n	80083f6 <smoltcp::wire::ip::checksum::data+0x2a0>
 80083c2:	4629      	mov	r1, r5
 80083c4:	2d06      	cmp	r5, #6
 80083c6:	d316      	bcc.n	80083f6 <smoltcp::wire::ip::checksum::data+0x2a0>
 80083c8:	f833 0b08 	ldrh.w	r0, [r3], #8
 80083cc:	3908      	subs	r1, #8
 80083ce:	2901      	cmp	r1, #1
 80083d0:	ba00      	rev	r0, r0
 80083d2:	f833 4c06 	ldrh.w	r4, [r3, #-6]
 80083d6:	f833 5c04 	ldrh.w	r5, [r3, #-4]
 80083da:	eb02 4010 	add.w	r0, r2, r0, lsr #16
 80083de:	ba22      	rev	r2, r4
 80083e0:	f833 6c02 	ldrh.w	r6, [r3, #-2]
 80083e4:	eb00 4012 	add.w	r0, r0, r2, lsr #16
 80083e8:	ba2a      	rev	r2, r5
 80083ea:	eb00 4012 	add.w	r0, r0, r2, lsr #16
 80083ee:	ba32      	rev	r2, r6
 80083f0:	eb00 4212 	add.w	r2, r0, r2, lsr #16
 80083f4:	d8e8      	bhi.n	80083c8 <smoltcp::wire::ip::checksum::data+0x272>
 80083f6:	2900      	cmp	r1, #0
 80083f8:	bf1c      	itt	ne
 80083fa:	7818      	ldrbne	r0, [r3, #0]
 80083fc:	eb02 2200 	addne.w	r2, r2, r0, lsl #8
 8008400:	0c10      	lsrs	r0, r2, #16
 8008402:	fa10 f082 	uxtah	r0, r0, r2
 8008406:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 800840a:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 800840e:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8008410:	8883      	ldrh	r3, [r0, #4]
 8008412:	3906      	subs	r1, #6
 8008414:	ba1b      	rev	r3, r3
 8008416:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 800841a:	1d83      	adds	r3, r0, #6
 800841c:	2d06      	cmp	r5, #6
 800841e:	d2d3      	bcs.n	80083c8 <smoltcp::wire::ip::checksum::data+0x272>
 8008420:	e7e9      	b.n	80083f6 <smoltcp::wire::ip::checksum::data+0x2a0>

08008422 <smoltcp::iface::route::Routes::lookup>:
 8008422:	b5f0      	push	{r4, r5, r6, r7, lr}
 8008424:	af03      	add	r7, sp, #12
 8008426:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 800842a:	b081      	sub	sp, #4
 800842c:	1c53      	adds	r3, r2, #1
 800842e:	d007      	beq.n	8008440 <smoltcp::iface::route::Routes::lookup+0x1e>
 8008430:	f002 03f0 	and.w	r3, r2, #240	@ 0xf0
 8008434:	2be0      	cmp	r3, #224	@ 0xe0
 8008436:	bf1c      	itt	ne
 8008438:	b2d3      	uxtbne	r3, r2
 800843a:	ea53 2312 	orrsne.w	r3, r3, r2, lsr #8
 800843e:	d10a      	bne.n	8008456 <smoltcp::iface::route::Routes::lookup+0x34>
 8008440:	f64a 70b8 	movw	r0, #44984	@ 0xafb8
 8008444:	f64a 72dc 	movw	r2, #45020	@ 0xafdc
 8008448:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800844c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8008450:	2123      	movs	r1, #35	@ 0x23
 8008452:	f7fb fa67 	bl	8003924 <core::panicking::panic>
 8008456:	6e0b      	ldr	r3, [r1, #96]	@ 0x60
 8008458:	2b00      	cmp	r3, #0
 800845a:	d072      	beq.n	8008542 <smoltcp::iface::route::Routes::lookup+0x120>
 800845c:	eb03 0343 	add.w	r3, r3, r3, lsl #1
 8008460:	e9d7 ec02 	ldrd	lr, ip, [r7, #8]
 8008464:	2600      	movs	r6, #0
 8008466:	f04f 3aff 	mov.w	sl, #4294967295	@ 0xffffffff
 800846a:	eb01 1903 	add.w	r9, r1, r3, lsl #4
 800846e:	ea4f 1803 	mov.w	r8, r3, lsl #4
 8008472:	f1a8 0330 	sub.w	r3, r8, #48	@ 0x30
 8008476:	9300      	str	r3, [sp, #0]
 8008478:	e007      	b.n	800848a <smoltcp::iface::route::Routes::lookup+0x68>
 800847a:	2400      	movs	r4, #0
 800847c:	6a5d      	ldr	r5, [r3, #36]	@ 0x24
 800847e:	4055      	eors	r5, r2
 8008480:	422c      	tst	r4, r5
 8008482:	d01d      	beq.n	80084c0 <smoltcp::iface::route::Routes::lookup+0x9e>
 8008484:	3630      	adds	r6, #48	@ 0x30
 8008486:	45b0      	cmp	r8, r6
 8008488:	d05b      	beq.n	8008542 <smoltcp::iface::route::Routes::lookup+0x120>
 800848a:	198b      	adds	r3, r1, r6
 800848c:	691c      	ldr	r4, [r3, #16]
 800848e:	07e4      	lsls	r4, r4, #31
 8008490:	d006      	beq.n	80084a0 <smoltcp::iface::route::Routes::lookup+0x7e>
 8008492:	e9d3 4506 	ldrd	r4, r5, [r3, #24]
 8008496:	ebb4 040e 	subs.w	r4, r4, lr
 800849a:	eb75 040c 	sbcs.w	r4, r5, ip
 800849e:	dbf1      	blt.n	8008484 <smoltcp::iface::route::Routes::lookup+0x62>
 80084a0:	f893 b028 	ldrb.w	fp, [r3, #40]	@ 0x28
 80084a4:	f1bb 0f00 	cmp.w	fp, #0
 80084a8:	d0e7      	beq.n	800847a <smoltcp::iface::route::Routes::lookup+0x58>
 80084aa:	f1cb 0400 	rsb	r4, fp, #0
 80084ae:	f004 041f 	and.w	r4, r4, #31
 80084b2:	fa0a f404 	lsl.w	r4, sl, r4
 80084b6:	ba24      	rev	r4, r4
 80084b8:	6a5d      	ldr	r5, [r3, #36]	@ 0x24
 80084ba:	4055      	eors	r5, r2
 80084bc:	422c      	tst	r4, r5
 80084be:	d1e1      	bne.n	8008484 <smoltcp::iface::route::Routes::lookup+0x62>
 80084c0:	f103 0130 	add.w	r1, r3, #48	@ 0x30
 80084c4:	4549      	cmp	r1, r9
 80084c6:	d042      	beq.n	800854e <smoltcp::iface::route::Routes::lookup+0x12c>
 80084c8:	9900      	ldr	r1, [sp, #0]
 80084ca:	f64a 25ab 	movw	r5, #43691	@ 0xaaab
 80084ce:	f6ca 25aa 	movt	r5, #43690	@ 0xaaaa
 80084d2:	f04f 38ff 	mov.w	r8, #4294967295	@ 0xffffffff
 80084d6:	1b8c      	subs	r4, r1, r6
 80084d8:	f103 0158 	add.w	r1, r3, #88	@ 0x58
 80084dc:	fba4 4505 	umull	r4, r5, r4, r5
 80084e0:	096d      	lsrs	r5, r5, #5
 80084e2:	e00a      	b.n	80084fa <smoltcp::iface::route::Routes::lookup+0xd8>
 80084e4:	fa5f f68b 	uxtb.w	r6, fp
 80084e8:	42a6      	cmp	r6, r4
 80084ea:	bf8c      	ite	hi
 80084ec:	4634      	movhi	r4, r6
 80084ee:	f1a1 0328 	subls.w	r3, r1, #40	@ 0x28
 80084f2:	46a3      	mov	fp, r4
 80084f4:	3130      	adds	r1, #48	@ 0x30
 80084f6:	3d01      	subs	r5, #1
 80084f8:	d029      	beq.n	800854e <smoltcp::iface::route::Routes::lookup+0x12c>
 80084fa:	f851 4c18 	ldr.w	r4, [r1, #-24]
 80084fe:	07e4      	lsls	r4, r4, #31
 8008500:	d006      	beq.n	8008510 <smoltcp::iface::route::Routes::lookup+0xee>
 8008502:	e951 4604 	ldrd	r4, r6, [r1, #-16]
 8008506:	ebb4 040e 	subs.w	r4, r4, lr
 800850a:	eb76 040c 	sbcs.w	r4, r6, ip
 800850e:	dbf1      	blt.n	80084f4 <smoltcp::iface::route::Routes::lookup+0xd2>
 8008510:	780c      	ldrb	r4, [r1, #0]
 8008512:	b16c      	cbz	r4, 8008530 <smoltcp::iface::route::Routes::lookup+0x10e>
 8008514:	4266      	negs	r6, r4
 8008516:	f006 061f 	and.w	r6, r6, #31
 800851a:	fa08 f606 	lsl.w	r6, r8, r6
 800851e:	fa96 f986 	rev.w	r9, r6
 8008522:	f851 6c04 	ldr.w	r6, [r1, #-4]
 8008526:	4056      	eors	r6, r2
 8008528:	ea19 0f06 	tst.w	r9, r6
 800852c:	d1e2      	bne.n	80084f4 <smoltcp::iface::route::Routes::lookup+0xd2>
 800852e:	e7d9      	b.n	80084e4 <smoltcp::iface::route::Routes::lookup+0xc2>
 8008530:	f04f 0900 	mov.w	r9, #0
 8008534:	f851 6c04 	ldr.w	r6, [r1, #-4]
 8008538:	4056      	eors	r6, r2
 800853a:	ea19 0f06 	tst.w	r9, r6
 800853e:	d1d9      	bne.n	80084f4 <smoltcp::iface::route::Routes::lookup+0xd2>
 8008540:	e7d0      	b.n	80084e4 <smoltcp::iface::route::Routes::lookup+0xc2>
 8008542:	2100      	movs	r1, #0
 8008544:	7001      	strb	r1, [r0, #0]
 8008546:	b001      	add	sp, #4
 8008548:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800854c:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800854e:	6a19      	ldr	r1, [r3, #32]
 8008550:	f8c0 1001 	str.w	r1, [r0, #1]
 8008554:	2101      	movs	r1, #1
 8008556:	7001      	strb	r1, [r0, #0]
 8008558:	b001      	add	sp, #4
 800855a:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800855e:	bdf0      	pop	{r4, r5, r6, r7, pc}

08008560 <smoltcp::iface::neighbor::Cache::fill>:
 8008560:	b5f0      	push	{r4, r5, r6, r7, lr}
 8008562:	af03      	add	r7, sp, #12
 8008564:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8008568:	b089      	sub	sp, #36	@ 0x24
 800856a:	f8d0 60c0 	ldr.w	r6, [r0, #192]	@ 0xc0
 800856e:	f248 7400 	movw	r4, #34560	@ 0x8700
 8008572:	f8d7 c008 	ldr.w	ip, [r7, #8]
 8008576:	f2c0 3493 	movt	r4, #915	@ 0x393
 800857a:	68fd      	ldr	r5, [r7, #12]
 800857c:	fa1f fe83 	uxth.w	lr, r3
 8008580:	eb06 0846 	add.w	r8, r6, r6, lsl #1
 8008584:	eb1c 0a04 	adds.w	sl, ip, r4
 8008588:	eba6 0386 	sub.w	r3, r6, r6, lsl #2
 800858c:	f145 0c00 	adc.w	ip, r5, #0
 8008590:	eb00 0bc8 	add.w	fp, r0, r8, lsl #3
 8008594:	9607      	str	r6, [sp, #28]
 8008596:	ea4f 09c3 	mov.w	r9, r3, lsl #3
 800859a:	2300      	movs	r3, #0
 800859c:	eb19 0403 	adds.w	r4, r9, r3
 80085a0:	d01a      	beq.n	80085d8 <smoltcp::iface::neighbor::Cache::fill+0x78>
 80085a2:	58c6      	ldr	r6, [r0, r3]
 80085a4:	18c5      	adds	r5, r0, r3
 80085a6:	428e      	cmp	r6, r1
 80085a8:	f000 8082 	beq.w	80086b0 <smoltcp::iface::neighbor::Cache::fill+0x150>
 80085ac:	f114 0618 	adds.w	r6, r4, #24
 80085b0:	d012      	beq.n	80085d8 <smoltcp::iface::neighbor::Cache::fill+0x78>
 80085b2:	69ae      	ldr	r6, [r5, #24]
 80085b4:	428e      	cmp	r6, r1
 80085b6:	d078      	beq.n	80086aa <smoltcp::iface::neighbor::Cache::fill+0x14a>
 80085b8:	f114 0630 	adds.w	r6, r4, #48	@ 0x30
 80085bc:	d00c      	beq.n	80085d8 <smoltcp::iface::neighbor::Cache::fill+0x78>
 80085be:	6b2e      	ldr	r6, [r5, #48]	@ 0x30
 80085c0:	428e      	cmp	r6, r1
 80085c2:	d074      	beq.n	80086ae <smoltcp::iface::neighbor::Cache::fill+0x14e>
 80085c4:	3448      	adds	r4, #72	@ 0x48
 80085c6:	d007      	beq.n	80085d8 <smoltcp::iface::neighbor::Cache::fill+0x78>
 80085c8:	6cac      	ldr	r4, [r5, #72]	@ 0x48
 80085ca:	3360      	adds	r3, #96	@ 0x60
 80085cc:	428c      	cmp	r4, r1
 80085ce:	d1e5      	bne.n	800859c <smoltcp::iface::neighbor::Cache::fill+0x3c>
 80085d0:	4418      	add	r0, r3
 80085d2:	f1a0 0518 	sub.w	r5, r0, #24
 80085d6:	e06b      	b.n	80086b0 <smoltcp::iface::neighbor::Cache::fill+0x150>
 80085d8:	9b07      	ldr	r3, [sp, #28]
 80085da:	2b07      	cmp	r3, #7
 80085dc:	d80a      	bhi.n	80085f4 <smoltcp::iface::neighbor::Cache::fill+0x94>
 80085de:	f8cb a008 	str.w	sl, [fp, #8]
 80085e2:	f8cb 1000 	str.w	r1, [fp]
 80085e6:	e9cb 2e04 	strd	r2, lr, [fp, #16]
 80085ea:	f8d0 10c0 	ldr.w	r1, [r0, #192]	@ 0xc0
 80085ee:	f8cb c00c 	str.w	ip, [fp, #12]
 80085f2:	e146      	b.n	8008882 <smoltcp::iface::neighbor::Cache::fill+0x322>
 80085f4:	f06f 032f 	mvn.w	r3, #47	@ 0x2f
 80085f8:	f64a 24ab 	movw	r4, #43691	@ 0xaaab
 80085fc:	eb03 03c8 	add.w	r3, r3, r8, lsl #3
 8008600:	f6ca 24aa 	movt	r4, #43690	@ 0xaaaa
 8008604:	f8cd b020 	str.w	fp, [sp, #32]
 8008608:	fba3 4304 	umull	r4, r3, r3, r4
 800860c:	9104      	str	r1, [sp, #16]
 800860e:	e9d0 4502 	ldrd	r4, r5, [r0, #8]
 8008612:	f8cd a008 	str.w	sl, [sp, #8]
 8008616:	f8cd c00c 	str.w	ip, [sp, #12]
 800861a:	f8cd e004 	str.w	lr, [sp, #4]
 800861e:	e9cd 0205 	strd	r0, r2, [sp, #20]
 8008622:	ea6f 1613 	mvn.w	r6, r3, lsr #4
 8008626:	07b6      	lsls	r6, r6, #30
 8008628:	f100 0618 	add.w	r6, r0, #24
 800862c:	d105      	bne.n	800863a <smoltcp::iface::neighbor::Cache::fill+0xda>
 800862e:	46b3      	mov	fp, r6
 8008630:	4603      	mov	r3, r0
 8008632:	4606      	mov	r6, r0
 8008634:	46a2      	mov	sl, r4
 8008636:	46a9      	mov	r9, r5
 8008638:	e058      	b.n	80086ec <smoltcp::iface::neighbor::Cache::fill+0x18c>
 800863a:	e9d0 b808 	ldrd	fp, r8, [r0, #32]
 800863e:	ea4f 1a13 	mov.w	sl, r3, lsr #4
 8008642:	4601      	mov	r1, r0
 8008644:	4650      	mov	r0, sl
 8008646:	ebb4 030b 	subs.w	r3, r4, fp
 800864a:	46c1      	mov	r9, r8
 800864c:	eb75 0308 	sbcs.w	r3, r5, r8
 8008650:	46da      	mov	sl, fp
 8008652:	bfb8      	it	lt
 8008654:	46a9      	movlt	r9, r5
 8008656:	bfb8      	it	lt
 8008658:	46a2      	movlt	sl, r4
 800865a:	ebbb 0304 	subs.w	r3, fp, r4
 800865e:	f101 0b30 	add.w	fp, r1, #48	@ 0x30
 8008662:	eb78 0305 	sbcs.w	r3, r8, r5
 8008666:	4633      	mov	r3, r6
 8008668:	bfa8      	it	ge
 800866a:	460b      	movge	r3, r1
 800866c:	0784      	lsls	r4, r0, #30
 800866e:	d03d      	beq.n	80086ec <smoltcp::iface::neighbor::Cache::fill+0x18c>
 8008670:	e9d1 460e 	ldrd	r4, r6, [r1, #56]	@ 0x38
 8008674:	f000 0003 	and.w	r0, r0, #3
 8008678:	ebba 0504 	subs.w	r5, sl, r4
 800867c:	eb79 0506 	sbcs.w	r5, r9, r6
 8008680:	46a0      	mov	r8, r4
 8008682:	4635      	mov	r5, r6
 8008684:	bfbc      	itt	lt
 8008686:	464d      	movlt	r5, r9
 8008688:	46d0      	movlt	r8, sl
 800868a:	ebb4 040a 	subs.w	r4, r4, sl
 800868e:	eb76 0409 	sbcs.w	r4, r6, r9
 8008692:	f101 0648 	add.w	r6, r1, #72	@ 0x48
 8008696:	bfb8      	it	lt
 8008698:	465b      	movlt	r3, fp
 800869a:	2801      	cmp	r0, #1
 800869c:	d113      	bne.n	80086c6 <smoltcp::iface::neighbor::Cache::fill+0x166>
 800869e:	4658      	mov	r0, fp
 80086a0:	46b3      	mov	fp, r6
 80086a2:	46c2      	mov	sl, r8
 80086a4:	46a9      	mov	r9, r5
 80086a6:	4606      	mov	r6, r0
 80086a8:	e020      	b.n	80086ec <smoltcp::iface::neighbor::Cache::fill+0x18c>
 80086aa:	3518      	adds	r5, #24
 80086ac:	e000      	b.n	80086b0 <smoltcp::iface::neighbor::Cache::fill+0x150>
 80086ae:	3530      	adds	r5, #48	@ 0x30
 80086b0:	f8c5 e014 	str.w	lr, [r5, #20]
 80086b4:	612a      	str	r2, [r5, #16]
 80086b6:	f8c5 c00c 	str.w	ip, [r5, #12]
 80086ba:	f8c5 a008 	str.w	sl, [r5, #8]
 80086be:	b009      	add	sp, #36	@ 0x24
 80086c0:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80086c4:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80086c6:	e9d1 2b14 	ldrd	r2, fp, [r1, #80]	@ 0x50
 80086ca:	46d9      	mov	r9, fp
 80086cc:	ebb8 0402 	subs.w	r4, r8, r2
 80086d0:	eb75 040b 	sbcs.w	r4, r5, fp
 80086d4:	4692      	mov	sl, r2
 80086d6:	bfbc      	itt	lt
 80086d8:	46a9      	movlt	r9, r5
 80086da:	46c2      	movlt	sl, r8
 80086dc:	ebb2 0208 	subs.w	r2, r2, r8
 80086e0:	eb7b 0205 	sbcs.w	r2, fp, r5
 80086e4:	f101 0b60 	add.w	fp, r1, #96	@ 0x60
 80086e8:	bfb8      	it	lt
 80086ea:	4633      	movlt	r3, r6
 80086ec:	f04f 0800 	mov.w	r8, #0
 80086f0:	e9d6 4508 	ldrd	r4, r5, [r6, #32]
 80086f4:	eb0b 0608 	add.w	r6, fp, r8
 80086f8:	f108 0860 	add.w	r8, r8, #96	@ 0x60
 80086fc:	ebb4 020a 	subs.w	r2, r4, sl
 8008700:	eb75 0209 	sbcs.w	r2, r5, r9
 8008704:	bfb8      	it	lt
 8008706:	4633      	movlt	r3, r6
 8008708:	ebba 0204 	subs.w	r2, sl, r4
 800870c:	eb79 0205 	sbcs.w	r2, r9, r5
 8008710:	bfbc      	itt	lt
 8008712:	464d      	movlt	r5, r9
 8008714:	4654      	movlt	r4, sl
 8008716:	e9d6 9208 	ldrd	r9, r2, [r6, #32]
 800871a:	e9d6 ac0e 	ldrd	sl, ip, [r6, #56]	@ 0x38
 800871e:	ebb9 0e04 	subs.w	lr, r9, r4
 8008722:	eb72 0005 	sbcs.w	r0, r2, r5
 8008726:	bfb8      	it	lt
 8008728:	f106 0318 	addlt.w	r3, r6, #24
 800872c:	ebb4 0009 	subs.w	r0, r4, r9
 8008730:	eb75 0002 	sbcs.w	r0, r5, r2
 8008734:	bfa4      	itt	ge
 8008736:	4615      	movge	r5, r2
 8008738:	464c      	movge	r4, r9
 800873a:	ebba 0004 	subs.w	r0, sl, r4
 800873e:	eb7c 0005 	sbcs.w	r0, ip, r5
 8008742:	bfb8      	it	lt
 8008744:	f106 0330 	addlt.w	r3, r6, #48	@ 0x30
 8008748:	ebb4 000a 	subs.w	r0, r4, sl
 800874c:	eb75 000c 	sbcs.w	r0, r5, ip
 8008750:	bfa8      	it	ge
 8008752:	4665      	movge	r5, ip
 8008754:	e9d6 2014 	ldrd	r2, r0, [r6, #80]	@ 0x50
 8008758:	bfa8      	it	ge
 800875a:	4654      	movge	r4, sl
 800875c:	3648      	adds	r6, #72	@ 0x48
 800875e:	1b11      	subs	r1, r2, r4
 8008760:	eb70 0105 	sbcs.w	r1, r0, r5
 8008764:	bfb8      	it	lt
 8008766:	4633      	movlt	r3, r6
 8008768:	1aa1      	subs	r1, r4, r2
 800876a:	eb75 0100 	sbcs.w	r1, r5, r0
 800876e:	bfa4      	itt	ge
 8008770:	4605      	movge	r5, r0
 8008772:	4614      	movge	r4, r2
 8008774:	9908      	ldr	r1, [sp, #32]
 8008776:	eb0b 0008 	add.w	r0, fp, r8
 800877a:	46a2      	mov	sl, r4
 800877c:	46a9      	mov	r9, r5
 800877e:	4288      	cmp	r0, r1
 8008780:	d1b6      	bne.n	80086f0 <smoltcp::iface::neighbor::Cache::fill+0x190>
 8008782:	9905      	ldr	r1, [sp, #20]
 8008784:	681c      	ldr	r4, [r3, #0]
 8008786:	2300      	movs	r3, #0
 8008788:	f8dd 8018 	ldr.w	r8, [sp, #24]
 800878c:	460d      	mov	r5, r1
 800878e:	9a04      	ldr	r2, [sp, #16]
 8008790:	9e08      	ldr	r6, [sp, #32]
 8008792:	6828      	ldr	r0, [r5, #0]
 8008794:	42a0      	cmp	r0, r4
 8008796:	d023      	beq.n	80087e0 <smoltcp::iface::neighbor::Cache::fill+0x280>
 8008798:	3518      	adds	r5, #24
 800879a:	42b5      	cmp	r5, r6
 800879c:	d012      	beq.n	80087c4 <smoltcp::iface::neighbor::Cache::fill+0x264>
 800879e:	6828      	ldr	r0, [r5, #0]
 80087a0:	42a0      	cmp	r0, r4
 80087a2:	d015      	beq.n	80087d0 <smoltcp::iface::neighbor::Cache::fill+0x270>
 80087a4:	3518      	adds	r5, #24
 80087a6:	42b5      	cmp	r5, r6
 80087a8:	d00c      	beq.n	80087c4 <smoltcp::iface::neighbor::Cache::fill+0x264>
 80087aa:	6828      	ldr	r0, [r5, #0]
 80087ac:	42a0      	cmp	r0, r4
 80087ae:	d012      	beq.n	80087d6 <smoltcp::iface::neighbor::Cache::fill+0x276>
 80087b0:	3518      	adds	r5, #24
 80087b2:	42b5      	cmp	r5, r6
 80087b4:	d006      	beq.n	80087c4 <smoltcp::iface::neighbor::Cache::fill+0x264>
 80087b6:	6828      	ldr	r0, [r5, #0]
 80087b8:	42a0      	cmp	r0, r4
 80087ba:	d00f      	beq.n	80087dc <smoltcp::iface::neighbor::Cache::fill+0x27c>
 80087bc:	3518      	adds	r5, #24
 80087be:	3304      	adds	r3, #4
 80087c0:	42b5      	cmp	r5, r6
 80087c2:	d1e6      	bne.n	8008792 <smoltcp::iface::neighbor::Cache::fill+0x232>
 80087c4:	f64a 70ec 	movw	r0, #45036	@ 0xafec
 80087c8:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80087cc:	f7fb f80d 	bl	80037ea <core::option::unwrap_failed>
 80087d0:	f043 0301 	orr.w	r3, r3, #1
 80087d4:	e004      	b.n	80087e0 <smoltcp::iface::neighbor::Cache::fill+0x280>
 80087d6:	f043 0302 	orr.w	r3, r3, #2
 80087da:	e001      	b.n	80087e0 <smoltcp::iface::neighbor::Cache::fill+0x280>
 80087dc:	f043 0303 	orr.w	r3, r3, #3
 80087e0:	9807      	ldr	r0, [sp, #28]
 80087e2:	4283      	cmp	r3, r0
 80087e4:	d254      	bcs.n	8008890 <smoltcp::iface::neighbor::Cache::fill+0x330>
 80087e6:	eb03 0043 	add.w	r0, r3, r3, lsl #1
 80087ea:	f1a6 0318 	sub.w	r3, r6, #24
 80087ee:	4615      	mov	r5, r2
 80087f0:	460c      	mov	r4, r1
 80087f2:	eb01 00c0 	add.w	r0, r1, r0, lsl #3
 80087f6:	4619      	mov	r1, r3
 80087f8:	2218      	movs	r2, #24
 80087fa:	f000 fda0 	bl	800933e <__aeabi_memmove8>
 80087fe:	f8d4 20c0 	ldr.w	r2, [r4, #192]	@ 0xc0
 8008802:	4629      	mov	r1, r5
 8008804:	4620      	mov	r0, r4
 8008806:	46c4      	mov	ip, r8
 8008808:	f1a2 0e01 	sub.w	lr, r2, #1
 800880c:	f8c4 e0c0 	str.w	lr, [r4, #192]	@ 0xc0
 8008810:	eba2 0282 	sub.w	r2, r2, r2, lsl #2
 8008814:	eb0e 034e 	add.w	r3, lr, lr, lsl #1
 8008818:	00d2      	lsls	r2, r2, #3
 800881a:	eb04 09c3 	add.w	r9, r4, r3, lsl #3
 800881e:	2400      	movs	r4, #0
 8008820:	1916      	adds	r6, r2, r4
 8008822:	f116 0518 	adds.w	r5, r6, #24
 8008826:	d01a      	beq.n	800885e <smoltcp::iface::neighbor::Cache::fill+0x2fe>
 8008828:	5903      	ldr	r3, [r0, r4]
 800882a:	1905      	adds	r5, r0, r4
 800882c:	428b      	cmp	r3, r1
 800882e:	d03d      	beq.n	80088ac <smoltcp::iface::neighbor::Cache::fill+0x34c>
 8008830:	f116 0330 	adds.w	r3, r6, #48	@ 0x30
 8008834:	d013      	beq.n	800885e <smoltcp::iface::neighbor::Cache::fill+0x2fe>
 8008836:	69ab      	ldr	r3, [r5, #24]
 8008838:	428b      	cmp	r3, r1
 800883a:	d034      	beq.n	80088a6 <smoltcp::iface::neighbor::Cache::fill+0x346>
 800883c:	f116 0348 	adds.w	r3, r6, #72	@ 0x48
 8008840:	d00d      	beq.n	800885e <smoltcp::iface::neighbor::Cache::fill+0x2fe>
 8008842:	6b2b      	ldr	r3, [r5, #48]	@ 0x30
 8008844:	428b      	cmp	r3, r1
 8008846:	d030      	beq.n	80088aa <smoltcp::iface::neighbor::Cache::fill+0x34a>
 8008848:	f116 0360 	adds.w	r3, r6, #96	@ 0x60
 800884c:	d007      	beq.n	800885e <smoltcp::iface::neighbor::Cache::fill+0x2fe>
 800884e:	6cab      	ldr	r3, [r5, #72]	@ 0x48
 8008850:	3460      	adds	r4, #96	@ 0x60
 8008852:	428b      	cmp	r3, r1
 8008854:	d1e4      	bne.n	8008820 <smoltcp::iface::neighbor::Cache::fill+0x2c0>
 8008856:	4420      	add	r0, r4
 8008858:	f1a0 0518 	sub.w	r5, r0, #24
 800885c:	e026      	b.n	80088ac <smoltcp::iface::neighbor::Cache::fill+0x34c>
 800885e:	f1be 0f07 	cmp.w	lr, #7
 8008862:	d82b      	bhi.n	80088bc <smoltcp::iface::neighbor::Cache::fill+0x35c>
 8008864:	9a02      	ldr	r2, [sp, #8]
 8008866:	f8c9 2008 	str.w	r2, [r9, #8]
 800886a:	f8c9 1000 	str.w	r1, [r9]
 800886e:	9901      	ldr	r1, [sp, #4]
 8008870:	f8c9 c010 	str.w	ip, [r9, #16]
 8008874:	9a03      	ldr	r2, [sp, #12]
 8008876:	f8c9 1014 	str.w	r1, [r9, #20]
 800887a:	f8d0 10c0 	ldr.w	r1, [r0, #192]	@ 0xc0
 800887e:	f8c9 200c 	str.w	r2, [r9, #12]
 8008882:	3101      	adds	r1, #1
 8008884:	f8c0 10c0 	str.w	r1, [r0, #192]	@ 0xc0
 8008888:	b009      	add	sp, #36	@ 0x24
 800888a:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800888e:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8008890:	f64a 7014 	movw	r0, #44820	@ 0xaf14
 8008894:	f64a 7238 	movw	r2, #44856	@ 0xaf38
 8008898:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800889c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80088a0:	2122      	movs	r1, #34	@ 0x22
 80088a2:	f7fb f83f 	bl	8003924 <core::panicking::panic>
 80088a6:	3518      	adds	r5, #24
 80088a8:	e000      	b.n	80088ac <smoltcp::iface::neighbor::Cache::fill+0x34c>
 80088aa:	3530      	adds	r5, #48	@ 0x30
 80088ac:	9801      	ldr	r0, [sp, #4]
 80088ae:	6168      	str	r0, [r5, #20]
 80088b0:	9803      	ldr	r0, [sp, #12]
 80088b2:	60e8      	str	r0, [r5, #12]
 80088b4:	9802      	ldr	r0, [sp, #8]
 80088b6:	f8c5 8010 	str.w	r8, [r5, #16]
 80088ba:	60a8      	str	r0, [r5, #8]
 80088bc:	f24b 10a0 	movw	r0, #45472	@ 0xb1a0
 80088c0:	f64a 72fc 	movw	r2, #45052	@ 0xaffc
 80088c4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80088c8:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80088cc:	2128      	movs	r1, #40	@ 0x28
 80088ce:	f7fb f829 	bl	8003924 <core::panicking::panic>

080088d2 <__cpsid>:
 80088d2:	b672      	cpsid	i
 80088d4:	4770      	bx	lr

080088d6 <__cpsie>:
 80088d6:	b662      	cpsie	i
 80088d8:	4770      	bx	lr

080088da <__delay>:
 80088da:	2101      	movs	r1, #1
 80088dc:	eb01 0050 	add.w	r0, r1, r0, lsr #1
 80088e0:	3801      	subs	r0, #1
 80088e2:	d1fd      	bne.n	80088e0 <__delay+0x6>
 80088e4:	4770      	bx	lr

080088e6 <__dsb>:
 80088e6:	f3bf 8f4f 	dsb	sy
 80088ea:	4770      	bx	lr

080088ec <__primask_r>:
 80088ec:	f3ef 8010 	mrs	r0, PRIMASK
 80088f0:	4770      	bx	lr

080088f2 <__udf>:
 80088f2:	de00      	udf	#0
 80088f4:	defe      	udf	#254	@ 0xfe

080088f6 <__aeabi_memclr8>:
 80088f6:	b580      	push	{r7, lr}
 80088f8:	466f      	mov	r7, sp
 80088fa:	2904      	cmp	r1, #4
 80088fc:	d305      	bcc.n	800890a <__aeabi_memclr8+0x14>
 80088fe:	1f0b      	subs	r3, r1, #4
 8008900:	43da      	mvns	r2, r3
 8008902:	f012 0f0c 	tst.w	r2, #12
 8008906:	d102      	bne.n	800890e <__aeabi_memclr8+0x18>
 8008908:	e01a      	b.n	8008940 <__aeabi_memclr8+0x4a>
 800890a:	4602      	mov	r2, r0
 800890c:	e024      	b.n	8008958 <__aeabi_memclr8+0x62>
 800890e:	f04f 0c00 	mov.w	ip, #0
 8008912:	4602      	mov	r2, r0
 8008914:	f842 cb04 	str.w	ip, [r2], #4
 8008918:	f013 0f0c 	tst.w	r3, #12
 800891c:	d008      	beq.n	8008930 <__aeabi_memclr8+0x3a>
 800891e:	f003 020c 	and.w	r2, r3, #12
 8008922:	f8c0 c004 	str.w	ip, [r0, #4]
 8008926:	2a04      	cmp	r2, #4
 8008928:	d105      	bne.n	8008936 <__aeabi_memclr8+0x40>
 800892a:	3908      	subs	r1, #8
 800892c:	3008      	adds	r0, #8
 800892e:	e006      	b.n	800893e <__aeabi_memclr8+0x48>
 8008930:	4619      	mov	r1, r3
 8008932:	4610      	mov	r0, r2
 8008934:	e004      	b.n	8008940 <__aeabi_memclr8+0x4a>
 8008936:	2200      	movs	r2, #0
 8008938:	390c      	subs	r1, #12
 800893a:	6082      	str	r2, [r0, #8]
 800893c:	300c      	adds	r0, #12
 800893e:	4602      	mov	r2, r0
 8008940:	2b0c      	cmp	r3, #12
 8008942:	d309      	bcc.n	8008958 <__aeabi_memclr8+0x62>
 8008944:	2300      	movs	r3, #0
 8008946:	4602      	mov	r2, r0
 8008948:	3910      	subs	r1, #16
 800894a:	e9c2 3300 	strd	r3, r3, [r2]
 800894e:	e9c2 3302 	strd	r3, r3, [r2, #8]
 8008952:	3210      	adds	r2, #16
 8008954:	2903      	cmp	r1, #3
 8008956:	d8f7      	bhi.n	8008948 <__aeabi_memclr8+0x52>
 8008958:	1850      	adds	r0, r2, r1
 800895a:	4282      	cmp	r2, r0
 800895c:	d221      	bcs.n	80089a2 <__aeabi_memclr8+0xac>
 800895e:	f1a1 0c01 	sub.w	ip, r1, #1
 8008962:	f011 0303 	ands.w	r3, r1, #3
 8008966:	d00c      	beq.n	8008982 <__aeabi_memclr8+0x8c>
 8008968:	f04f 0e00 	mov.w	lr, #0
 800896c:	4611      	mov	r1, r2
 800896e:	f801 eb01 	strb.w	lr, [r1], #1
 8008972:	2b01      	cmp	r3, #1
 8008974:	d00a      	beq.n	800898c <__aeabi_memclr8+0x96>
 8008976:	2b02      	cmp	r3, #2
 8008978:	f882 e001 	strb.w	lr, [r2, #1]
 800897c:	d103      	bne.n	8008986 <__aeabi_memclr8+0x90>
 800897e:	1c91      	adds	r1, r2, #2
 8008980:	e004      	b.n	800898c <__aeabi_memclr8+0x96>
 8008982:	4611      	mov	r1, r2
 8008984:	e002      	b.n	800898c <__aeabi_memclr8+0x96>
 8008986:	2100      	movs	r1, #0
 8008988:	7091      	strb	r1, [r2, #2]
 800898a:	1cd1      	adds	r1, r2, #3
 800898c:	f1bc 0f03 	cmp.w	ip, #3
 8008990:	bf38      	it	cc
 8008992:	bd80      	popcc	{r7, pc}
 8008994:	3904      	subs	r1, #4
 8008996:	2200      	movs	r2, #0
 8008998:	f841 2f04 	str.w	r2, [r1, #4]!
 800899c:	1d0b      	adds	r3, r1, #4
 800899e:	4283      	cmp	r3, r0
 80089a0:	d1fa      	bne.n	8008998 <__aeabi_memclr8+0xa2>
 80089a2:	bd80      	pop	{r7, pc}

080089a4 <__aeabi_memcpy4>:
 80089a4:	2a04      	cmp	r2, #4
 80089a6:	d309      	bcc.n	80089bc <__aeabi_memcpy4+0x18>
 80089a8:	b5d0      	push	{r4, r6, r7, lr}
 80089aa:	af02      	add	r7, sp, #8
 80089ac:	f1a2 0e04 	sub.w	lr, r2, #4
 80089b0:	ea6f 030e 	mvn.w	r3, lr
 80089b4:	f013 0f0c 	tst.w	r3, #12
 80089b8:	d106      	bne.n	80089c8 <__aeabi_memcpy4+0x24>
 80089ba:	e023      	b.n	8008a04 <__aeabi_memcpy4+0x60>
 80089bc:	460b      	mov	r3, r1
 80089be:	4684      	mov	ip, r0
 80089c0:	4660      	mov	r0, ip
 80089c2:	4619      	mov	r1, r3
 80089c4:	f000 b83c 	b.w	8008a40 <compiler_builtins::mem::memcpy>
 80089c8:	460b      	mov	r3, r1
 80089ca:	4684      	mov	ip, r0
 80089cc:	f853 4b04 	ldr.w	r4, [r3], #4
 80089d0:	f01e 0f0c 	tst.w	lr, #12
 80089d4:	f84c 4b04 	str.w	r4, [ip], #4
 80089d8:	d009      	beq.n	80089ee <__aeabi_memcpy4+0x4a>
 80089da:	684b      	ldr	r3, [r1, #4]
 80089dc:	6043      	str	r3, [r0, #4]
 80089de:	f00e 030c 	and.w	r3, lr, #12
 80089e2:	2b04      	cmp	r3, #4
 80089e4:	d107      	bne.n	80089f6 <__aeabi_memcpy4+0x52>
 80089e6:	3a08      	subs	r2, #8
 80089e8:	3108      	adds	r1, #8
 80089ea:	3008      	adds	r0, #8
 80089ec:	e008      	b.n	8008a00 <__aeabi_memcpy4+0x5c>
 80089ee:	4672      	mov	r2, lr
 80089f0:	4660      	mov	r0, ip
 80089f2:	4619      	mov	r1, r3
 80089f4:	e006      	b.n	8008a04 <__aeabi_memcpy4+0x60>
 80089f6:	688b      	ldr	r3, [r1, #8]
 80089f8:	3a0c      	subs	r2, #12
 80089fa:	6083      	str	r3, [r0, #8]
 80089fc:	310c      	adds	r1, #12
 80089fe:	300c      	adds	r0, #12
 8008a00:	4684      	mov	ip, r0
 8008a02:	460b      	mov	r3, r1
 8008a04:	f1be 0f0c 	cmp.w	lr, #12
 8008a08:	d314      	bcc.n	8008a34 <__aeabi_memcpy4+0x90>
 8008a0a:	4684      	mov	ip, r0
 8008a0c:	460b      	mov	r3, r1
 8008a0e:	6818      	ldr	r0, [r3, #0]
 8008a10:	3a10      	subs	r2, #16
 8008a12:	f8cc 0000 	str.w	r0, [ip]
 8008a16:	2a03      	cmp	r2, #3
 8008a18:	6858      	ldr	r0, [r3, #4]
 8008a1a:	f8cc 0004 	str.w	r0, [ip, #4]
 8008a1e:	6898      	ldr	r0, [r3, #8]
 8008a20:	f8cc 0008 	str.w	r0, [ip, #8]
 8008a24:	68d8      	ldr	r0, [r3, #12]
 8008a26:	f103 0310 	add.w	r3, r3, #16
 8008a2a:	f8cc 000c 	str.w	r0, [ip, #12]
 8008a2e:	f10c 0c10 	add.w	ip, ip, #16
 8008a32:	d8ec      	bhi.n	8008a0e <__aeabi_memcpy4+0x6a>
 8008a34:	e8bd 40d0 	ldmia.w	sp!, {r4, r6, r7, lr}
 8008a38:	4660      	mov	r0, ip
 8008a3a:	4619      	mov	r1, r3
 8008a3c:	f000 b800 	b.w	8008a40 <compiler_builtins::mem::memcpy>

08008a40 <compiler_builtins::mem::memcpy>:
 8008a40:	b5f0      	push	{r4, r5, r6, r7, lr}
 8008a42:	af03      	add	r7, sp, #12
 8008a44:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8008a48:	b08c      	sub	sp, #48	@ 0x30
 8008a4a:	2a10      	cmp	r2, #16
 8008a4c:	d31e      	bcc.n	8008a8c <compiler_builtins::mem::memcpy+0x4c>
 8008a4e:	4243      	negs	r3, r0
 8008a50:	f003 0803 	and.w	r8, r3, #3
 8008a54:	eb00 0308 	add.w	r3, r0, r8
 8008a58:	4298      	cmp	r0, r3
 8008a5a:	d233      	bcs.n	8008ac4 <compiler_builtins::mem::memcpy+0x84>
 8008a5c:	f1a8 0c01 	sub.w	ip, r8, #1
 8008a60:	4606      	mov	r6, r0
 8008a62:	460c      	mov	r4, r1
 8008a64:	f1b8 0f00 	cmp.w	r8, #0
 8008a68:	d01a      	beq.n	8008aa0 <compiler_builtins::mem::memcpy+0x60>
 8008a6a:	460c      	mov	r4, r1
 8008a6c:	4606      	mov	r6, r0
 8008a6e:	f814 eb01 	ldrb.w	lr, [r4], #1
 8008a72:	f1b8 0f01 	cmp.w	r8, #1
 8008a76:	f806 eb01 	strb.w	lr, [r6], #1
 8008a7a:	d011      	beq.n	8008aa0 <compiler_builtins::mem::memcpy+0x60>
 8008a7c:	784e      	ldrb	r6, [r1, #1]
 8008a7e:	f1b8 0f02 	cmp.w	r8, #2
 8008a82:	7046      	strb	r6, [r0, #1]
 8008a84:	d108      	bne.n	8008a98 <compiler_builtins::mem::memcpy+0x58>
 8008a86:	1c8c      	adds	r4, r1, #2
 8008a88:	1c86      	adds	r6, r0, #2
 8008a8a:	e009      	b.n	8008aa0 <compiler_builtins::mem::memcpy+0x60>
 8008a8c:	4684      	mov	ip, r0
 8008a8e:	eb0c 0302 	add.w	r3, ip, r2
 8008a92:	459c      	cmp	ip, r3
 8008a94:	d340      	bcc.n	8008b18 <compiler_builtins::mem::memcpy+0xd8>
 8008a96:	e06c      	b.n	8008b72 <compiler_builtins::mem::memcpy+0x132>
 8008a98:	788e      	ldrb	r6, [r1, #2]
 8008a9a:	1ccc      	adds	r4, r1, #3
 8008a9c:	7086      	strb	r6, [r0, #2]
 8008a9e:	1cc6      	adds	r6, r0, #3
 8008aa0:	f1bc 0f03 	cmp.w	ip, #3
 8008aa4:	d30e      	bcc.n	8008ac4 <compiler_builtins::mem::memcpy+0x84>
 8008aa6:	3c04      	subs	r4, #4
 8008aa8:	3e04      	subs	r6, #4
 8008aaa:	f814 5f04 	ldrb.w	r5, [r4, #4]!
 8008aae:	f806 5f04 	strb.w	r5, [r6, #4]!
 8008ab2:	7865      	ldrb	r5, [r4, #1]
 8008ab4:	7075      	strb	r5, [r6, #1]
 8008ab6:	78a5      	ldrb	r5, [r4, #2]
 8008ab8:	70b5      	strb	r5, [r6, #2]
 8008aba:	78e5      	ldrb	r5, [r4, #3]
 8008abc:	70f5      	strb	r5, [r6, #3]
 8008abe:	1d35      	adds	r5, r6, #4
 8008ac0:	429d      	cmp	r5, r3
 8008ac2:	d1f2      	bne.n	8008aaa <compiler_builtins::mem::memcpy+0x6a>
 8008ac4:	eba2 0e08 	sub.w	lr, r2, r8
 8008ac8:	eb01 0408 	add.w	r4, r1, r8
 8008acc:	f02e 0203 	bic.w	r2, lr, #3
 8008ad0:	f014 0603 	ands.w	r6, r4, #3
 8008ad4:	eb03 0c02 	add.w	ip, r3, r2
 8008ad8:	d159      	bne.n	8008b8e <compiler_builtins::mem::memcpy+0x14e>
 8008ada:	4563      	cmp	r3, ip
 8008adc:	d215      	bcs.n	8008b0a <compiler_builtins::mem::memcpy+0xca>
 8008ade:	4621      	mov	r1, r4
 8008ae0:	680d      	ldr	r5, [r1, #0]
 8008ae2:	f843 5b04 	str.w	r5, [r3], #4
 8008ae6:	4563      	cmp	r3, ip
 8008ae8:	d20f      	bcs.n	8008b0a <compiler_builtins::mem::memcpy+0xca>
 8008aea:	684d      	ldr	r5, [r1, #4]
 8008aec:	f843 5b04 	str.w	r5, [r3], #4
 8008af0:	4563      	cmp	r3, ip
 8008af2:	bf3e      	ittt	cc
 8008af4:	688d      	ldrcc	r5, [r1, #8]
 8008af6:	f843 5b04 	strcc.w	r5, [r3], #4
 8008afa:	4563      	cmpcc	r3, ip
 8008afc:	d205      	bcs.n	8008b0a <compiler_builtins::mem::memcpy+0xca>
 8008afe:	68cd      	ldr	r5, [r1, #12]
 8008b00:	3110      	adds	r1, #16
 8008b02:	f843 5b04 	str.w	r5, [r3], #4
 8008b06:	4563      	cmp	r3, ip
 8008b08:	d3ea      	bcc.n	8008ae0 <compiler_builtins::mem::memcpy+0xa0>
 8008b0a:	18a1      	adds	r1, r4, r2
 8008b0c:	f00e 0203 	and.w	r2, lr, #3
 8008b10:	eb0c 0302 	add.w	r3, ip, r2
 8008b14:	459c      	cmp	ip, r3
 8008b16:	d22c      	bcs.n	8008b72 <compiler_builtins::mem::memcpy+0x132>
 8008b18:	f1a2 0e01 	sub.w	lr, r2, #1
 8008b1c:	f012 0403 	ands.w	r4, r2, #3
 8008b20:	d013      	beq.n	8008b4a <compiler_builtins::mem::memcpy+0x10a>
 8008b22:	460a      	mov	r2, r1
 8008b24:	4665      	mov	r5, ip
 8008b26:	f812 6b01 	ldrb.w	r6, [r2], #1
 8008b2a:	2c01      	cmp	r4, #1
 8008b2c:	f805 6b01 	strb.w	r6, [r5], #1
 8008b30:	d00d      	beq.n	8008b4e <compiler_builtins::mem::memcpy+0x10e>
 8008b32:	784a      	ldrb	r2, [r1, #1]
 8008b34:	2c02      	cmp	r4, #2
 8008b36:	f88c 2001 	strb.w	r2, [ip, #1]
 8008b3a:	d11e      	bne.n	8008b7a <compiler_builtins::mem::memcpy+0x13a>
 8008b3c:	1c8a      	adds	r2, r1, #2
 8008b3e:	f10c 0502 	add.w	r5, ip, #2
 8008b42:	f1be 0f03 	cmp.w	lr, #3
 8008b46:	d205      	bcs.n	8008b54 <compiler_builtins::mem::memcpy+0x114>
 8008b48:	e013      	b.n	8008b72 <compiler_builtins::mem::memcpy+0x132>
 8008b4a:	4665      	mov	r5, ip
 8008b4c:	460a      	mov	r2, r1
 8008b4e:	f1be 0f03 	cmp.w	lr, #3
 8008b52:	d30e      	bcc.n	8008b72 <compiler_builtins::mem::memcpy+0x132>
 8008b54:	1f11      	subs	r1, r2, #4
 8008b56:	1f2a      	subs	r2, r5, #4
 8008b58:	f811 6f04 	ldrb.w	r6, [r1, #4]!
 8008b5c:	f802 6f04 	strb.w	r6, [r2, #4]!
 8008b60:	784e      	ldrb	r6, [r1, #1]
 8008b62:	7056      	strb	r6, [r2, #1]
 8008b64:	788e      	ldrb	r6, [r1, #2]
 8008b66:	7096      	strb	r6, [r2, #2]
 8008b68:	78ce      	ldrb	r6, [r1, #3]
 8008b6a:	70d6      	strb	r6, [r2, #3]
 8008b6c:	1d16      	adds	r6, r2, #4
 8008b6e:	429e      	cmp	r6, r3
 8008b70:	d1f2      	bne.n	8008b58 <compiler_builtins::mem::memcpy+0x118>
 8008b72:	b00c      	add	sp, #48	@ 0x30
 8008b74:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8008b78:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8008b7a:	788a      	ldrb	r2, [r1, #2]
 8008b7c:	f10c 0503 	add.w	r5, ip, #3
 8008b80:	f88c 2002 	strb.w	r2, [ip, #2]
 8008b84:	1cca      	adds	r2, r1, #3
 8008b86:	f1be 0f03 	cmp.w	lr, #3
 8008b8a:	d2e3      	bcs.n	8008b54 <compiler_builtins::mem::memcpy+0x114>
 8008b8c:	e7f1      	b.n	8008b72 <compiler_builtins::mem::memcpy+0x132>
 8008b8e:	f04f 0900 	mov.w	r9, #0
 8008b92:	f1c6 0a04 	rsb	sl, r6, #4
 8008b96:	ad0b      	add	r5, sp, #44	@ 0x2c
 8008b98:	f8cd 902c 	str.w	r9, [sp, #44]	@ 0x2c
 8008b9c:	eb05 0b06 	add.w	fp, r5, r6
 8008ba0:	ea5f 75ca 	movs.w	r5, sl, lsl #31
 8008ba4:	bf1e      	ittt	ne
 8008ba6:	7825      	ldrbne	r5, [r4, #0]
 8008ba8:	f88b 5000 	strbne.w	r5, [fp]
 8008bac:	f04f 0901 	movne.w	r9, #1
 8008bb0:	1ba5      	subs	r5, r4, r6
 8008bb2:	9507      	str	r5, [sp, #28]
 8008bb4:	00f5      	lsls	r5, r6, #3
 8008bb6:	9501      	str	r5, [sp, #4]
 8008bb8:	ea5f 758a 	movs.w	r5, sl, lsl #30
 8008bbc:	bf44      	itt	mi
 8008bbe:	f834 5009 	ldrhmi.w	r5, [r4, r9]
 8008bc2:	f82b 5009 	strhmi.w	r5, [fp, r9]
 8008bc6:	1d1d      	adds	r5, r3, #4
 8008bc8:	f8dd b02c 	ldr.w	fp, [sp, #44]	@ 0x2c
 8008bcc:	4565      	cmp	r5, ip
 8008bce:	9d01      	ldr	r5, [sp, #4]
 8008bd0:	46aa      	mov	sl, r5
 8008bd2:	f1c5 0500 	rsb	r5, r5, #0
 8008bd6:	d25c      	bcs.n	8008c92 <compiler_builtins::mem::memcpy+0x252>
 8008bd8:	4273      	negs	r3, r6
 8008bda:	9500      	str	r5, [sp, #0]
 8008bdc:	eb01 0903 	add.w	r9, r1, r3
 8008be0:	f005 0118 	and.w	r1, r5, #24
 8008be4:	4605      	mov	r5, r0
 8008be6:	9107      	str	r1, [sp, #28]
 8008be8:	f8cd 9010 	str.w	r9, [sp, #16]
 8008bec:	44c1      	add	r9, r8
 8008bee:	9b07      	ldr	r3, [sp, #28]
 8008bf0:	fa2b fb0a 	lsr.w	fp, fp, sl
 8008bf4:	f8d9 1004 	ldr.w	r1, [r9, #4]
 8008bf8:	9108      	str	r1, [sp, #32]
 8008bfa:	fa01 f303 	lsl.w	r3, r1, r3
 8008bfe:	eb05 0108 	add.w	r1, r5, r8
 8008c02:	ea43 030b 	orr.w	r3, r3, fp
 8008c06:	468b      	mov	fp, r1
 8008c08:	f84b 3b08 	str.w	r3, [fp], #8
 8008c0c:	45e3      	cmp	fp, ip
 8008c0e:	d242      	bcs.n	8008c96 <compiler_builtins::mem::memcpy+0x256>
 8008c10:	9b08      	ldr	r3, [sp, #32]
 8008c12:	9503      	str	r5, [sp, #12]
 8008c14:	f8cd 9018 	str.w	r9, [sp, #24]
 8008c18:	fa23 f50a 	lsr.w	r5, r3, sl
 8008c1c:	f8d9 9008 	ldr.w	r9, [r9, #8]
 8008c20:	9b07      	ldr	r3, [sp, #28]
 8008c22:	9105      	str	r1, [sp, #20]
 8008c24:	fa09 f303 	lsl.w	r3, r9, r3
 8008c28:	432b      	orrs	r3, r5
 8008c2a:	604b      	str	r3, [r1, #4]
 8008c2c:	f101 030c 	add.w	r3, r1, #12
 8008c30:	4563      	cmp	r3, ip
 8008c32:	d236      	bcs.n	8008ca2 <compiler_builtins::mem::memcpy+0x262>
 8008c34:	9d06      	ldr	r5, [sp, #24]
 8008c36:	68ed      	ldr	r5, [r5, #12]
 8008c38:	9508      	str	r5, [sp, #32]
 8008c3a:	fa29 f50a 	lsr.w	r5, r9, sl
 8008c3e:	9502      	str	r5, [sp, #8]
 8008c40:	e9dd 5107 	ldrd	r5, r1, [sp, #28]
 8008c44:	fa01 f905 	lsl.w	r9, r1, r5
 8008c48:	9905      	ldr	r1, [sp, #20]
 8008c4a:	9d02      	ldr	r5, [sp, #8]
 8008c4c:	3110      	adds	r1, #16
 8008c4e:	ea45 0509 	orr.w	r5, r5, r9
 8008c52:	4561      	cmp	r1, ip
 8008c54:	f8cb 5000 	str.w	r5, [fp]
 8008c58:	d22d      	bcs.n	8008cb6 <compiler_builtins::mem::memcpy+0x276>
 8008c5a:	9906      	ldr	r1, [sp, #24]
 8008c5c:	9d07      	ldr	r5, [sp, #28]
 8008c5e:	f8dd 9010 	ldr.w	r9, [sp, #16]
 8008c62:	f8d1 b010 	ldr.w	fp, [r1, #16]
 8008c66:	9908      	ldr	r1, [sp, #32]
 8008c68:	f109 0910 	add.w	r9, r9, #16
 8008c6c:	fa0b f505 	lsl.w	r5, fp, r5
 8008c70:	fa21 f10a 	lsr.w	r1, r1, sl
 8008c74:	4329      	orrs	r1, r5
 8008c76:	9d03      	ldr	r5, [sp, #12]
 8008c78:	6019      	str	r1, [r3, #0]
 8008c7a:	3510      	adds	r5, #16
 8008c7c:	eb05 0308 	add.w	r3, r5, r8
 8008c80:	1d19      	adds	r1, r3, #4
 8008c82:	4561      	cmp	r1, ip
 8008c84:	d3b0      	bcc.n	8008be8 <compiler_builtins::mem::memcpy+0x1a8>
 8008c86:	eb09 0108 	add.w	r1, r9, r8
 8008c8a:	9107      	str	r1, [sp, #28]
 8008c8c:	f8dd a000 	ldr.w	sl, [sp]
 8008c90:	e018      	b.n	8008cc4 <compiler_builtins::mem::memcpy+0x284>
 8008c92:	46aa      	mov	sl, r5
 8008c94:	e016      	b.n	8008cc4 <compiler_builtins::mem::memcpy+0x284>
 8008c96:	460b      	mov	r3, r1
 8008c98:	f109 0104 	add.w	r1, r9, #4
 8008c9c:	9107      	str	r1, [sp, #28]
 8008c9e:	3304      	adds	r3, #4
 8008ca0:	e00c      	b.n	8008cbc <compiler_builtins::mem::memcpy+0x27c>
 8008ca2:	9906      	ldr	r1, [sp, #24]
 8008ca4:	46cb      	mov	fp, r9
 8008ca6:	f8dd a000 	ldr.w	sl, [sp]
 8008caa:	3108      	adds	r1, #8
 8008cac:	9107      	str	r1, [sp, #28]
 8008cae:	9905      	ldr	r1, [sp, #20]
 8008cb0:	f101 0308 	add.w	r3, r1, #8
 8008cb4:	e006      	b.n	8008cc4 <compiler_builtins::mem::memcpy+0x284>
 8008cb6:	9906      	ldr	r1, [sp, #24]
 8008cb8:	310c      	adds	r1, #12
 8008cba:	9107      	str	r1, [sp, #28]
 8008cbc:	f8dd a000 	ldr.w	sl, [sp]
 8008cc0:	f8dd b020 	ldr.w	fp, [sp, #32]
 8008cc4:	2500      	movs	r5, #0
 8008cc6:	2e01      	cmp	r6, #1
 8008cc8:	f88d 5028 	strb.w	r5, [sp, #40]	@ 0x28
 8008ccc:	f807 5c26 	strb.w	r5, [r7, #-38]
 8008cd0:	d105      	bne.n	8008cde <compiler_builtins::mem::memcpy+0x29e>
 8008cd2:	f10d 0828 	add.w	r8, sp, #40	@ 0x28
 8008cd6:	f04f 0900 	mov.w	r9, #0
 8008cda:	2100      	movs	r1, #0
 8008cdc:	e009      	b.n	8008cf2 <compiler_builtins::mem::memcpy+0x2b2>
 8008cde:	9907      	ldr	r1, [sp, #28]
 8008ce0:	f1a7 0826 	sub.w	r8, r7, #38	@ 0x26
 8008ce4:	790d      	ldrb	r5, [r1, #4]
 8008ce6:	7949      	ldrb	r1, [r1, #5]
 8008ce8:	f88d 5028 	strb.w	r5, [sp, #40]	@ 0x28
 8008cec:	ea4f 2901 	mov.w	r9, r1, lsl #8
 8008cf0:	2102      	movs	r1, #2
 8008cf2:	07e6      	lsls	r6, r4, #31
 8008cf4:	d101      	bne.n	8008cfa <compiler_builtins::mem::memcpy+0x2ba>
 8008cf6:	2100      	movs	r1, #0
 8008cf8:	e009      	b.n	8008d0e <compiler_builtins::mem::memcpy+0x2ce>
 8008cfa:	9d07      	ldr	r5, [sp, #28]
 8008cfc:	3504      	adds	r5, #4
 8008cfe:	5c69      	ldrb	r1, [r5, r1]
 8008d00:	f888 1000 	strb.w	r1, [r8]
 8008d04:	f817 1c26 	ldrb.w	r1, [r7, #-38]
 8008d08:	f89d 5028 	ldrb.w	r5, [sp, #40]	@ 0x28
 8008d0c:	0409      	lsls	r1, r1, #16
 8008d0e:	ea41 0109 	orr.w	r1, r1, r9
 8008d12:	9e01      	ldr	r6, [sp, #4]
 8008d14:	4329      	orrs	r1, r5
 8008d16:	f00a 0518 	and.w	r5, sl, #24
 8008d1a:	40a9      	lsls	r1, r5
 8008d1c:	fa2b f606 	lsr.w	r6, fp, r6
 8008d20:	4331      	orrs	r1, r6
 8008d22:	6019      	str	r1, [r3, #0]
 8008d24:	e6f1      	b.n	8008b0a <compiler_builtins::mem::memcpy+0xca>

08008d26 <compiler_builtins::mem::memmove>:
 8008d26:	b5f0      	push	{r4, r5, r6, r7, lr}
 8008d28:	af03      	add	r7, sp, #12
 8008d2a:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8008d2e:	b08e      	sub	sp, #56	@ 0x38
 8008d30:	1a43      	subs	r3, r0, r1
 8008d32:	4293      	cmp	r3, r2
 8008d34:	d22b      	bcs.n	8008d8e <compiler_builtins::mem::memmove+0x68>
 8008d36:	eb01 0902 	add.w	r9, r1, r2
 8008d3a:	eb00 0c02 	add.w	ip, r0, r2
 8008d3e:	2a10      	cmp	r2, #16
 8008d40:	d34a      	bcc.n	8008dd8 <compiler_builtins::mem::memmove+0xb2>
 8008d42:	f00c 0a03 	and.w	sl, ip, #3
 8008d46:	f02c 0503 	bic.w	r5, ip, #3
 8008d4a:	f1ca 0b00 	rsb	fp, sl, #0
 8008d4e:	4565      	cmp	r5, ip
 8008d50:	d264      	bcs.n	8008e1c <compiler_builtins::mem::memmove+0xf6>
 8008d52:	f1aa 0e01 	sub.w	lr, sl, #1
 8008d56:	f1ba 0f00 	cmp.w	sl, #0
 8008d5a:	d049      	beq.n	8008df0 <compiler_builtins::mem::memmove+0xca>
 8008d5c:	46c8      	mov	r8, r9
 8008d5e:	4663      	mov	r3, ip
 8008d60:	f818 4d01 	ldrb.w	r4, [r8, #-1]!
 8008d64:	f1ba 0f01 	cmp.w	sl, #1
 8008d68:	f803 4d01 	strb.w	r4, [r3, #-1]!
 8008d6c:	d042      	beq.n	8008df4 <compiler_builtins::mem::memmove+0xce>
 8008d6e:	46c8      	mov	r8, r9
 8008d70:	4663      	mov	r3, ip
 8008d72:	f818 4d02 	ldrb.w	r4, [r8, #-2]!
 8008d76:	f1ba 0f02 	cmp.w	sl, #2
 8008d7a:	f803 4d02 	strb.w	r4, [r3, #-2]!
 8008d7e:	bf1f      	itttt	ne
 8008d80:	46c8      	movne	r8, r9
 8008d82:	f818 4d03 	ldrbne.w	r4, [r8, #-3]!
 8008d86:	4663      	movne	r3, ip
 8008d88:	f803 4d03 	strbne.w	r4, [r3, #-3]!
 8008d8c:	e032      	b.n	8008df4 <compiler_builtins::mem::memmove+0xce>
 8008d8e:	2a10      	cmp	r2, #16
 8008d90:	d327      	bcc.n	8008de2 <compiler_builtins::mem::memmove+0xbc>
 8008d92:	4243      	negs	r3, r0
 8008d94:	f003 0803 	and.w	r8, r3, #3
 8008d98:	eb00 0308 	add.w	r3, r0, r8
 8008d9c:	4298      	cmp	r0, r3
 8008d9e:	f080 80c3 	bcs.w	8008f28 <compiler_builtins::mem::memmove+0x202>
 8008da2:	f1a8 0c01 	sub.w	ip, r8, #1
 8008da6:	4606      	mov	r6, r0
 8008da8:	460d      	mov	r5, r1
 8008daa:	f1b8 0f00 	cmp.w	r8, #0
 8008dae:	f000 80a9 	beq.w	8008f04 <compiler_builtins::mem::memmove+0x1de>
 8008db2:	460d      	mov	r5, r1
 8008db4:	4606      	mov	r6, r0
 8008db6:	f815 eb01 	ldrb.w	lr, [r5], #1
 8008dba:	f1b8 0f01 	cmp.w	r8, #1
 8008dbe:	f806 eb01 	strb.w	lr, [r6], #1
 8008dc2:	f000 809f 	beq.w	8008f04 <compiler_builtins::mem::memmove+0x1de>
 8008dc6:	784e      	ldrb	r6, [r1, #1]
 8008dc8:	f1b8 0f02 	cmp.w	r8, #2
 8008dcc:	7046      	strb	r6, [r0, #1]
 8008dce:	f040 8095 	bne.w	8008efc <compiler_builtins::mem::memmove+0x1d6>
 8008dd2:	1c8d      	adds	r5, r1, #2
 8008dd4:	1c86      	adds	r6, r0, #2
 8008dd6:	e095      	b.n	8008f04 <compiler_builtins::mem::memmove+0x1de>
 8008dd8:	4663      	mov	r3, ip
 8008dda:	1a99      	subs	r1, r3, r2
 8008ddc:	4299      	cmp	r1, r3
 8008dde:	d35b      	bcc.n	8008e98 <compiler_builtins::mem::memmove+0x172>
 8008de0:	e0f9      	b.n	8008fd6 <compiler_builtins::mem::memmove+0x2b0>
 8008de2:	4684      	mov	ip, r0
 8008de4:	eb0c 0302 	add.w	r3, ip, r2
 8008de8:	459c      	cmp	ip, r3
 8008dea:	f0c0 80c7 	bcc.w	8008f7c <compiler_builtins::mem::memmove+0x256>
 8008dee:	e0f2      	b.n	8008fd6 <compiler_builtins::mem::memmove+0x2b0>
 8008df0:	4663      	mov	r3, ip
 8008df2:	46c8      	mov	r8, r9
 8008df4:	f1be 0f03 	cmp.w	lr, #3
 8008df8:	d310      	bcc.n	8008e1c <compiler_builtins::mem::memmove+0xf6>
 8008dfa:	f1a8 0604 	sub.w	r6, r8, #4
 8008dfe:	78f4      	ldrb	r4, [r6, #3]
 8008e00:	f803 4c01 	strb.w	r4, [r3, #-1]
 8008e04:	78b4      	ldrb	r4, [r6, #2]
 8008e06:	f803 4c02 	strb.w	r4, [r3, #-2]
 8008e0a:	7874      	ldrb	r4, [r6, #1]
 8008e0c:	f803 4c03 	strb.w	r4, [r3, #-3]
 8008e10:	f816 4904 	ldrb.w	r4, [r6], #-4
 8008e14:	f803 4d04 	strb.w	r4, [r3, #-4]!
 8008e18:	429d      	cmp	r5, r3
 8008e1a:	d3f0      	bcc.n	8008dfe <compiler_builtins::mem::memmove+0xd8>
 8008e1c:	eba2 0e0a 	sub.w	lr, r2, sl
 8008e20:	eb09 0a0b 	add.w	sl, r9, fp
 8008e24:	f02e 0403 	bic.w	r4, lr, #3
 8008e28:	1b2b      	subs	r3, r5, r4
 8008e2a:	f1c4 0800 	rsb	r8, r4, #0
 8008e2e:	f01a 0403 	ands.w	r4, sl, #3
 8008e32:	f040 80de 	bne.w	8008ff2 <compiler_builtins::mem::memmove+0x2cc>
 8008e36:	42ab      	cmp	r3, r5
 8008e38:	d226      	bcs.n	8008e88 <compiler_builtins::mem::memmove+0x162>
 8008e3a:	f1a9 0110 	sub.w	r1, r9, #16
 8008e3e:	f1ab 0904 	sub.w	r9, fp, #4
 8008e42:	eb01 060b 	add.w	r6, r1, fp
 8008e46:	eb0c 050b 	add.w	r5, ip, fp
 8008e4a:	68f4      	ldr	r4, [r6, #12]
 8008e4c:	f845 4c04 	str.w	r4, [r5, #-4]
 8008e50:	eb0c 0409 	add.w	r4, ip, r9
 8008e54:	42a3      	cmp	r3, r4
 8008e56:	d217      	bcs.n	8008e88 <compiler_builtins::mem::memmove+0x162>
 8008e58:	68b2      	ldr	r2, [r6, #8]
 8008e5a:	f845 2c08 	str.w	r2, [r5, #-8]
 8008e5e:	1f22      	subs	r2, r4, #4
 8008e60:	4293      	cmp	r3, r2
 8008e62:	bf3f      	itttt	cc
 8008e64:	6872      	ldrcc	r2, [r6, #4]
 8008e66:	f845 2c0c 	strcc.w	r2, [r5, #-12]
 8008e6a:	f1a4 0208 	subcc.w	r2, r4, #8
 8008e6e:	4293      	cmpcc	r3, r2
 8008e70:	d20a      	bcs.n	8008e88 <compiler_builtins::mem::memmove+0x162>
 8008e72:	f851 200b 	ldr.w	r2, [r1, fp]
 8008e76:	f1ac 0c10 	sub.w	ip, ip, #16
 8008e7a:	3910      	subs	r1, #16
 8008e7c:	f845 2c10 	str.w	r2, [r5, #-16]
 8008e80:	eb0c 020b 	add.w	r2, ip, fp
 8008e84:	4293      	cmp	r3, r2
 8008e86:	d3dc      	bcc.n	8008e42 <compiler_builtins::mem::memmove+0x11c>
 8008e88:	eb0a 0908 	add.w	r9, sl, r8
 8008e8c:	f00e 0203 	and.w	r2, lr, #3
 8008e90:	1a99      	subs	r1, r3, r2
 8008e92:	4299      	cmp	r1, r3
 8008e94:	f080 809f 	bcs.w	8008fd6 <compiler_builtins::mem::memmove+0x2b0>
 8008e98:	f1a2 0c01 	sub.w	ip, r2, #1
 8008e9c:	f012 0403 	ands.w	r4, r2, #3
 8008ea0:	d013      	beq.n	8008eca <compiler_builtins::mem::memmove+0x1a4>
 8008ea2:	464d      	mov	r5, r9
 8008ea4:	461a      	mov	r2, r3
 8008ea6:	f815 6d01 	ldrb.w	r6, [r5, #-1]!
 8008eaa:	2c01      	cmp	r4, #1
 8008eac:	f802 6d01 	strb.w	r6, [r2, #-1]!
 8008eb0:	d00d      	beq.n	8008ece <compiler_builtins::mem::memmove+0x1a8>
 8008eb2:	464d      	mov	r5, r9
 8008eb4:	461a      	mov	r2, r3
 8008eb6:	f815 6d02 	ldrb.w	r6, [r5, #-2]!
 8008eba:	2c02      	cmp	r4, #2
 8008ebc:	f802 6d02 	strb.w	r6, [r2, #-2]!
 8008ec0:	d005      	beq.n	8008ece <compiler_builtins::mem::memmove+0x1a8>
 8008ec2:	f819 2d03 	ldrb.w	r2, [r9, #-3]!
 8008ec6:	f803 2d03 	strb.w	r2, [r3, #-3]!
 8008eca:	464d      	mov	r5, r9
 8008ecc:	461a      	mov	r2, r3
 8008ece:	f1bc 0f03 	cmp.w	ip, #3
 8008ed2:	f0c0 8080 	bcc.w	8008fd6 <compiler_builtins::mem::memmove+0x2b0>
 8008ed6:	1eab      	subs	r3, r5, #2
 8008ed8:	785e      	ldrb	r6, [r3, #1]
 8008eda:	f802 6c01 	strb.w	r6, [r2, #-1]
 8008ede:	781e      	ldrb	r6, [r3, #0]
 8008ee0:	f802 6c02 	strb.w	r6, [r2, #-2]
 8008ee4:	f813 6c01 	ldrb.w	r6, [r3, #-1]
 8008ee8:	f802 6c03 	strb.w	r6, [r2, #-3]
 8008eec:	f813 6c02 	ldrb.w	r6, [r3, #-2]
 8008ef0:	3b04      	subs	r3, #4
 8008ef2:	f802 6d04 	strb.w	r6, [r2, #-4]!
 8008ef6:	4291      	cmp	r1, r2
 8008ef8:	d3ee      	bcc.n	8008ed8 <compiler_builtins::mem::memmove+0x1b2>
 8008efa:	e06c      	b.n	8008fd6 <compiler_builtins::mem::memmove+0x2b0>
 8008efc:	788e      	ldrb	r6, [r1, #2]
 8008efe:	1ccd      	adds	r5, r1, #3
 8008f00:	7086      	strb	r6, [r0, #2]
 8008f02:	1cc6      	adds	r6, r0, #3
 8008f04:	f1bc 0f03 	cmp.w	ip, #3
 8008f08:	d30e      	bcc.n	8008f28 <compiler_builtins::mem::memmove+0x202>
 8008f0a:	3d04      	subs	r5, #4
 8008f0c:	3e04      	subs	r6, #4
 8008f0e:	f815 4f04 	ldrb.w	r4, [r5, #4]!
 8008f12:	f806 4f04 	strb.w	r4, [r6, #4]!
 8008f16:	786c      	ldrb	r4, [r5, #1]
 8008f18:	7074      	strb	r4, [r6, #1]
 8008f1a:	78ac      	ldrb	r4, [r5, #2]
 8008f1c:	70b4      	strb	r4, [r6, #2]
 8008f1e:	78ec      	ldrb	r4, [r5, #3]
 8008f20:	70f4      	strb	r4, [r6, #3]
 8008f22:	1d34      	adds	r4, r6, #4
 8008f24:	429c      	cmp	r4, r3
 8008f26:	d1f2      	bne.n	8008f0e <compiler_builtins::mem::memmove+0x1e8>
 8008f28:	eba2 0208 	sub.w	r2, r2, r8
 8008f2c:	eb01 0508 	add.w	r5, r1, r8
 8008f30:	f022 0603 	bic.w	r6, r2, #3
 8008f34:	f015 0903 	ands.w	r9, r5, #3
 8008f38:	eb03 0c06 	add.w	ip, r3, r6
 8008f3c:	d16d      	bne.n	800901a <compiler_builtins::mem::memmove+0x2f4>
 8008f3e:	4563      	cmp	r3, ip
 8008f40:	d215      	bcs.n	8008f6e <compiler_builtins::mem::memmove+0x248>
 8008f42:	4629      	mov	r1, r5
 8008f44:	680c      	ldr	r4, [r1, #0]
 8008f46:	f843 4b04 	str.w	r4, [r3], #4
 8008f4a:	4563      	cmp	r3, ip
 8008f4c:	d20f      	bcs.n	8008f6e <compiler_builtins::mem::memmove+0x248>
 8008f4e:	684c      	ldr	r4, [r1, #4]
 8008f50:	f843 4b04 	str.w	r4, [r3], #4
 8008f54:	4563      	cmp	r3, ip
 8008f56:	bf3e      	ittt	cc
 8008f58:	688c      	ldrcc	r4, [r1, #8]
 8008f5a:	f843 4b04 	strcc.w	r4, [r3], #4
 8008f5e:	4563      	cmpcc	r3, ip
 8008f60:	d205      	bcs.n	8008f6e <compiler_builtins::mem::memmove+0x248>
 8008f62:	68cc      	ldr	r4, [r1, #12]
 8008f64:	3110      	adds	r1, #16
 8008f66:	f843 4b04 	str.w	r4, [r3], #4
 8008f6a:	4563      	cmp	r3, ip
 8008f6c:	d3ea      	bcc.n	8008f44 <compiler_builtins::mem::memmove+0x21e>
 8008f6e:	19a9      	adds	r1, r5, r6
 8008f70:	f002 0203 	and.w	r2, r2, #3
 8008f74:	eb0c 0302 	add.w	r3, ip, r2
 8008f78:	459c      	cmp	ip, r3
 8008f7a:	d22c      	bcs.n	8008fd6 <compiler_builtins::mem::memmove+0x2b0>
 8008f7c:	f1a2 0e01 	sub.w	lr, r2, #1
 8008f80:	f012 0403 	ands.w	r4, r2, #3
 8008f84:	d013      	beq.n	8008fae <compiler_builtins::mem::memmove+0x288>
 8008f86:	460a      	mov	r2, r1
 8008f88:	4665      	mov	r5, ip
 8008f8a:	f812 6b01 	ldrb.w	r6, [r2], #1
 8008f8e:	2c01      	cmp	r4, #1
 8008f90:	f805 6b01 	strb.w	r6, [r5], #1
 8008f94:	d00d      	beq.n	8008fb2 <compiler_builtins::mem::memmove+0x28c>
 8008f96:	784a      	ldrb	r2, [r1, #1]
 8008f98:	2c02      	cmp	r4, #2
 8008f9a:	f88c 2001 	strb.w	r2, [ip, #1]
 8008f9e:	d11e      	bne.n	8008fde <compiler_builtins::mem::memmove+0x2b8>
 8008fa0:	1c8a      	adds	r2, r1, #2
 8008fa2:	f10c 0502 	add.w	r5, ip, #2
 8008fa6:	f1be 0f03 	cmp.w	lr, #3
 8008faa:	d205      	bcs.n	8008fb8 <compiler_builtins::mem::memmove+0x292>
 8008fac:	e013      	b.n	8008fd6 <compiler_builtins::mem::memmove+0x2b0>
 8008fae:	4665      	mov	r5, ip
 8008fb0:	460a      	mov	r2, r1
 8008fb2:	f1be 0f03 	cmp.w	lr, #3
 8008fb6:	d30e      	bcc.n	8008fd6 <compiler_builtins::mem::memmove+0x2b0>
 8008fb8:	1f11      	subs	r1, r2, #4
 8008fba:	1f2a      	subs	r2, r5, #4
 8008fbc:	f811 6f04 	ldrb.w	r6, [r1, #4]!
 8008fc0:	f802 6f04 	strb.w	r6, [r2, #4]!
 8008fc4:	784e      	ldrb	r6, [r1, #1]
 8008fc6:	7056      	strb	r6, [r2, #1]
 8008fc8:	788e      	ldrb	r6, [r1, #2]
 8008fca:	7096      	strb	r6, [r2, #2]
 8008fcc:	78ce      	ldrb	r6, [r1, #3]
 8008fce:	70d6      	strb	r6, [r2, #3]
 8008fd0:	1d16      	adds	r6, r2, #4
 8008fd2:	429e      	cmp	r6, r3
 8008fd4:	d1f2      	bne.n	8008fbc <compiler_builtins::mem::memmove+0x296>
 8008fd6:	b00e      	add	sp, #56	@ 0x38
 8008fd8:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8008fdc:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8008fde:	788a      	ldrb	r2, [r1, #2]
 8008fe0:	f10c 0503 	add.w	r5, ip, #3
 8008fe4:	f88c 2002 	strb.w	r2, [ip, #2]
 8008fe8:	1cca      	adds	r2, r1, #3
 8008fea:	f1be 0f03 	cmp.w	lr, #3
 8008fee:	d2e3      	bcs.n	8008fb8 <compiler_builtins::mem::memmove+0x292>
 8008ff0:	e7f1      	b.n	8008fd6 <compiler_builtins::mem::memmove+0x2b0>
 8008ff2:	ebaa 0604 	sub.w	r6, sl, r4
 8008ff6:	f04f 0900 	mov.w	r9, #0
 8008ffa:	9607      	str	r6, [sp, #28]
 8008ffc:	00e6      	lsls	r6, r4, #3
 8008ffe:	2c01      	cmp	r4, #1
 8009000:	f88d 9028 	strb.w	r9, [sp, #40]	@ 0x28
 8009004:	f807 9c2e 	strb.w	r9, [r7, #-46]
 8009008:	9606      	str	r6, [sp, #24]
 800900a:	9402      	str	r4, [sp, #8]
 800900c:	f040 808e 	bne.w	800912c <compiler_builtins::mem::memmove+0x406>
 8009010:	ae0a      	add	r6, sp, #40	@ 0x28
 8009012:	9605      	str	r6, [sp, #20]
 8009014:	2600      	movs	r6, #0
 8009016:	9608      	str	r6, [sp, #32]
 8009018:	e099      	b.n	800914e <compiler_builtins::mem::memmove+0x428>
 800901a:	f04f 0b00 	mov.w	fp, #0
 800901e:	f1c9 0a04 	rsb	sl, r9, #4
 8009022:	ac0d      	add	r4, sp, #52	@ 0x34
 8009024:	f8cd b034 	str.w	fp, [sp, #52]	@ 0x34
 8009028:	eb04 0e09 	add.w	lr, r4, r9
 800902c:	ea5f 74ca 	movs.w	r4, sl, lsl #31
 8009030:	bf1e      	ittt	ne
 8009032:	782c      	ldrbne	r4, [r5, #0]
 8009034:	f88e 4000 	strbne.w	r4, [lr]
 8009038:	f04f 0b01 	movne.w	fp, #1
 800903c:	eba5 0409 	sub.w	r4, r5, r9
 8009040:	9407      	str	r4, [sp, #28]
 8009042:	ea4f 04c9 	mov.w	r4, r9, lsl #3
 8009046:	9401      	str	r4, [sp, #4]
 8009048:	ea5f 748a 	movs.w	r4, sl, lsl #30
 800904c:	bf44      	itt	mi
 800904e:	f835 400b 	ldrhmi.w	r4, [r5, fp]
 8009052:	f82e 400b 	strhmi.w	r4, [lr, fp]
 8009056:	9c0d      	ldr	r4, [sp, #52]	@ 0x34
 8009058:	f8dd a004 	ldr.w	sl, [sp, #4]
 800905c:	9408      	str	r4, [sp, #32]
 800905e:	1d1c      	adds	r4, r3, #4
 8009060:	4564      	cmp	r4, ip
 8009062:	f1ca 0400 	rsb	r4, sl, #0
 8009066:	9400      	str	r4, [sp, #0]
 8009068:	f080 8105 	bcs.w	8009276 <compiler_builtins::mem::memmove+0x550>
 800906c:	f1c9 0300 	rsb	r3, r9, #0
 8009070:	f8dd b020 	ldr.w	fp, [sp, #32]
 8009074:	eb01 0e03 	add.w	lr, r1, r3
 8009078:	f004 0118 	and.w	r1, r4, #24
 800907c:	4604      	mov	r4, r0
 800907e:	9107      	str	r1, [sp, #28]
 8009080:	f8cd e010 	str.w	lr, [sp, #16]
 8009084:	44c6      	add	lr, r8
 8009086:	9b07      	ldr	r3, [sp, #28]
 8009088:	fa2b fb0a 	lsr.w	fp, fp, sl
 800908c:	f8de 1004 	ldr.w	r1, [lr, #4]
 8009090:	9108      	str	r1, [sp, #32]
 8009092:	9403      	str	r4, [sp, #12]
 8009094:	fa01 f303 	lsl.w	r3, r1, r3
 8009098:	eb04 0108 	add.w	r1, r4, r8
 800909c:	ea43 030b 	orr.w	r3, r3, fp
 80090a0:	468b      	mov	fp, r1
 80090a2:	f84b 3b08 	str.w	r3, [fp], #8
 80090a6:	45e3      	cmp	fp, ip
 80090a8:	f080 80c6 	bcs.w	8009238 <compiler_builtins::mem::memmove+0x512>
 80090ac:	9b08      	ldr	r3, [sp, #32]
 80090ae:	f8cd e018 	str.w	lr, [sp, #24]
 80090b2:	f8de e008 	ldr.w	lr, [lr, #8]
 80090b6:	fa23 f40a 	lsr.w	r4, r3, sl
 80090ba:	9b07      	ldr	r3, [sp, #28]
 80090bc:	9105      	str	r1, [sp, #20]
 80090be:	fa0e f303 	lsl.w	r3, lr, r3
 80090c2:	4323      	orrs	r3, r4
 80090c4:	604b      	str	r3, [r1, #4]
 80090c6:	f101 030c 	add.w	r3, r1, #12
 80090ca:	4563      	cmp	r3, ip
 80090cc:	f080 80be 	bcs.w	800924c <compiler_builtins::mem::memmove+0x526>
 80090d0:	9c06      	ldr	r4, [sp, #24]
 80090d2:	68e4      	ldr	r4, [r4, #12]
 80090d4:	9408      	str	r4, [sp, #32]
 80090d6:	fa2e f40a 	lsr.w	r4, lr, sl
 80090da:	9402      	str	r4, [sp, #8]
 80090dc:	e9dd 4107 	ldrd	r4, r1, [sp, #28]
 80090e0:	fa01 fe04 	lsl.w	lr, r1, r4
 80090e4:	9905      	ldr	r1, [sp, #20]
 80090e6:	9c02      	ldr	r4, [sp, #8]
 80090e8:	3110      	adds	r1, #16
 80090ea:	ea44 040e 	orr.w	r4, r4, lr
 80090ee:	4561      	cmp	r1, ip
 80090f0:	f8cb 4000 	str.w	r4, [fp]
 80090f4:	f080 80bc 	bcs.w	8009270 <compiler_builtins::mem::memmove+0x54a>
 80090f8:	9906      	ldr	r1, [sp, #24]
 80090fa:	9c07      	ldr	r4, [sp, #28]
 80090fc:	f8dd e010 	ldr.w	lr, [sp, #16]
 8009100:	f8d1 b010 	ldr.w	fp, [r1, #16]
 8009104:	9908      	ldr	r1, [sp, #32]
 8009106:	f10e 0e10 	add.w	lr, lr, #16
 800910a:	fa0b f404 	lsl.w	r4, fp, r4
 800910e:	fa21 f10a 	lsr.w	r1, r1, sl
 8009112:	4321      	orrs	r1, r4
 8009114:	9c03      	ldr	r4, [sp, #12]
 8009116:	6019      	str	r1, [r3, #0]
 8009118:	3410      	adds	r4, #16
 800911a:	eb04 0308 	add.w	r3, r4, r8
 800911e:	1d19      	adds	r1, r3, #4
 8009120:	4561      	cmp	r1, ip
 8009122:	d3ad      	bcc.n	8009080 <compiler_builtins::mem::memmove+0x35a>
 8009124:	eb0e 0108 	add.w	r1, lr, r8
 8009128:	9107      	str	r1, [sp, #28]
 800912a:	e0a6      	b.n	800927a <compiler_builtins::mem::memmove+0x554>
 800912c:	9e07      	ldr	r6, [sp, #28]
 800912e:	f896 9000 	ldrb.w	r9, [r6]
 8009132:	7876      	ldrb	r6, [r6, #1]
 8009134:	9608      	str	r6, [sp, #32]
 8009136:	ea5f 76ca 	movs.w	r6, sl, lsl #31
 800913a:	f88d 9028 	strb.w	r9, [sp, #40]	@ 0x28
 800913e:	d101      	bne.n	8009144 <compiler_builtins::mem::memmove+0x41e>
 8009140:	2400      	movs	r4, #0
 8009142:	e00e      	b.n	8009162 <compiler_builtins::mem::memmove+0x43c>
 8009144:	f04f 0902 	mov.w	r9, #2
 8009148:	f1a7 062e 	sub.w	r6, r7, #46	@ 0x2e
 800914c:	9605      	str	r6, [sp, #20]
 800914e:	9e07      	ldr	r6, [sp, #28]
 8009150:	9c05      	ldr	r4, [sp, #20]
 8009152:	f816 6009 	ldrb.w	r6, [r6, r9]
 8009156:	7026      	strb	r6, [r4, #0]
 8009158:	f817 6c2e 	ldrb.w	r6, [r7, #-46]
 800915c:	f89d 9028 	ldrb.w	r9, [sp, #40]	@ 0x28
 8009160:	0434      	lsls	r4, r6, #16
 8009162:	9e08      	ldr	r6, [sp, #32]
 8009164:	ea44 2606 	orr.w	r6, r4, r6, lsl #8
 8009168:	9c02      	ldr	r4, [sp, #8]
 800916a:	ea46 0609 	orr.w	r6, r6, r9
 800916e:	9605      	str	r6, [sp, #20]
 8009170:	1d1e      	adds	r6, r3, #4
 8009172:	9608      	str	r6, [sp, #32]
 8009174:	42ae      	cmp	r6, r5
 8009176:	9e06      	ldr	r6, [sp, #24]
 8009178:	f1c6 0900 	rsb	r9, r6, #0
 800917c:	d259      	bcs.n	8009232 <compiler_builtins::mem::memmove+0x50c>
 800917e:	1b12      	subs	r2, r2, r4
 8009180:	f8cd 9004 	str.w	r9, [sp, #4]
 8009184:	4411      	add	r1, r2
 8009186:	f1ab 0204 	sub.w	r2, fp, #4
 800918a:	9203      	str	r2, [sp, #12]
 800918c:	f009 0218 	and.w	r2, r9, #24
 8009190:	9207      	str	r2, [sp, #28]
 8009192:	9a05      	ldr	r2, [sp, #20]
 8009194:	eb01 090b 	add.w	r9, r1, fp
 8009198:	9104      	str	r1, [sp, #16]
 800919a:	9907      	ldr	r1, [sp, #28]
 800919c:	eb0c 060b 	add.w	r6, ip, fp
 80091a0:	f859 4c04 	ldr.w	r4, [r9, #-4]
 80091a4:	9d06      	ldr	r5, [sp, #24]
 80091a6:	fa02 f101 	lsl.w	r1, r2, r1
 80091aa:	fa24 f205 	lsr.w	r2, r4, r5
 80091ae:	4311      	orrs	r1, r2
 80091b0:	f846 1c04 	str.w	r1, [r6, #-4]
 80091b4:	9903      	ldr	r1, [sp, #12]
 80091b6:	eb0c 0201 	add.w	r2, ip, r1
 80091ba:	9908      	ldr	r1, [sp, #32]
 80091bc:	4291      	cmp	r1, r2
 80091be:	d240      	bcs.n	8009242 <compiler_builtins::mem::memmove+0x51c>
 80091c0:	f859 1c08 	ldr.w	r1, [r9, #-8]
 80091c4:	9105      	str	r1, [sp, #20]
 80091c6:	9907      	ldr	r1, [sp, #28]
 80091c8:	408c      	lsls	r4, r1
 80091ca:	9905      	ldr	r1, [sp, #20]
 80091cc:	fa21 f505 	lsr.w	r5, r1, r5
 80091d0:	9908      	ldr	r1, [sp, #32]
 80091d2:	4325      	orrs	r5, r4
 80091d4:	f846 5c08 	str.w	r5, [r6, #-8]
 80091d8:	1f15      	subs	r5, r2, #4
 80091da:	42a9      	cmp	r1, r5
 80091dc:	d23e      	bcs.n	800925c <compiler_builtins::mem::memmove+0x536>
 80091de:	9907      	ldr	r1, [sp, #28]
 80091e0:	9d05      	ldr	r5, [sp, #20]
 80091e2:	f859 4c0c 	ldr.w	r4, [r9, #-12]
 80091e6:	fa05 f101 	lsl.w	r1, r5, r1
 80091ea:	9d06      	ldr	r5, [sp, #24]
 80091ec:	fa24 f505 	lsr.w	r5, r4, r5
 80091f0:	4329      	orrs	r1, r5
 80091f2:	f846 1c0c 	str.w	r1, [r6, #-12]
 80091f6:	f1a2 0508 	sub.w	r5, r2, #8
 80091fa:	9908      	ldr	r1, [sp, #32]
 80091fc:	42a9      	cmp	r1, r5
 80091fe:	d275      	bcs.n	80092ec <compiler_builtins::mem::memmove+0x5c6>
 8009200:	9907      	ldr	r1, [sp, #28]
 8009202:	f1ac 0c10 	sub.w	ip, ip, #16
 8009206:	f859 9c10 	ldr.w	r9, [r9, #-16]
 800920a:	eb0c 050b 	add.w	r5, ip, fp
 800920e:	9a06      	ldr	r2, [sp, #24]
 8009210:	fa04 f101 	lsl.w	r1, r4, r1
 8009214:	fa29 f202 	lsr.w	r2, r9, r2
 8009218:	4311      	orrs	r1, r2
 800921a:	f846 1c10 	str.w	r1, [r6, #-16]
 800921e:	9904      	ldr	r1, [sp, #16]
 8009220:	9a08      	ldr	r2, [sp, #32]
 8009222:	3910      	subs	r1, #16
 8009224:	42aa      	cmp	r2, r5
 8009226:	464a      	mov	r2, r9
 8009228:	d3b4      	bcc.n	8009194 <compiler_builtins::mem::memmove+0x46e>
 800922a:	4459      	add	r1, fp
 800922c:	4693      	mov	fp, r2
 800922e:	9107      	str	r1, [sp, #28]
 8009230:	e01b      	b.n	800926a <compiler_builtins::mem::memmove+0x544>
 8009232:	f8dd b014 	ldr.w	fp, [sp, #20]
 8009236:	e062      	b.n	80092fe <compiler_builtins::mem::memmove+0x5d8>
 8009238:	460b      	mov	r3, r1
 800923a:	f10e 0104 	add.w	r1, lr, #4
 800923e:	3304      	adds	r3, #4
 8009240:	e018      	b.n	8009274 <compiler_builtins::mem::memmove+0x54e>
 8009242:	e9dd 6103 	ldrd	r6, r1, [sp, #12]
 8009246:	4615      	mov	r5, r2
 8009248:	4431      	add	r1, r6
 800924a:	e053      	b.n	80092f4 <compiler_builtins::mem::memmove+0x5ce>
 800924c:	9906      	ldr	r1, [sp, #24]
 800924e:	46f3      	mov	fp, lr
 8009250:	3108      	adds	r1, #8
 8009252:	9107      	str	r1, [sp, #28]
 8009254:	9905      	ldr	r1, [sp, #20]
 8009256:	f101 0308 	add.w	r3, r1, #8
 800925a:	e00e      	b.n	800927a <compiler_builtins::mem::memmove+0x554>
 800925c:	e9dd 6203 	ldrd	r6, r2, [sp, #12]
 8009260:	f8dd b014 	ldr.w	fp, [sp, #20]
 8009264:	4432      	add	r2, r6
 8009266:	3a04      	subs	r2, #4
 8009268:	9207      	str	r2, [sp, #28]
 800926a:	e9dd 9401 	ldrd	r9, r4, [sp, #4]
 800926e:	e046      	b.n	80092fe <compiler_builtins::mem::memmove+0x5d8>
 8009270:	9906      	ldr	r1, [sp, #24]
 8009272:	310c      	adds	r1, #12
 8009274:	9107      	str	r1, [sp, #28]
 8009276:	f8dd b020 	ldr.w	fp, [sp, #32]
 800927a:	2400      	movs	r4, #0
 800927c:	f1b9 0f01 	cmp.w	r9, #1
 8009280:	f88d 402c 	strb.w	r4, [sp, #44]	@ 0x2c
 8009284:	f807 4c2a 	strb.w	r4, [r7, #-42]
 8009288:	d106      	bne.n	8009298 <compiler_builtins::mem::memmove+0x572>
 800928a:	f10d 092c 	add.w	r9, sp, #44	@ 0x2c
 800928e:	f04f 0800 	mov.w	r8, #0
 8009292:	f04f 0e00 	mov.w	lr, #0
 8009296:	e00a      	b.n	80092ae <compiler_builtins::mem::memmove+0x588>
 8009298:	9907      	ldr	r1, [sp, #28]
 800929a:	f1a7 092a 	sub.w	r9, r7, #42	@ 0x2a
 800929e:	f04f 0e02 	mov.w	lr, #2
 80092a2:	790c      	ldrb	r4, [r1, #4]
 80092a4:	7949      	ldrb	r1, [r1, #5]
 80092a6:	f88d 402c 	strb.w	r4, [sp, #44]	@ 0x2c
 80092aa:	ea4f 2801 	mov.w	r8, r1, lsl #8
 80092ae:	07e9      	lsls	r1, r5, #31
 80092b0:	d102      	bne.n	80092b8 <compiler_builtins::mem::memmove+0x592>
 80092b2:	f04f 0e00 	mov.w	lr, #0
 80092b6:	e00b      	b.n	80092d0 <compiler_builtins::mem::memmove+0x5aa>
 80092b8:	9907      	ldr	r1, [sp, #28]
 80092ba:	3104      	adds	r1, #4
 80092bc:	f811 100e 	ldrb.w	r1, [r1, lr]
 80092c0:	f889 1000 	strb.w	r1, [r9]
 80092c4:	f817 1c2a 	ldrb.w	r1, [r7, #-42]
 80092c8:	f89d 402c 	ldrb.w	r4, [sp, #44]	@ 0x2c
 80092cc:	ea4f 4e01 	mov.w	lr, r1, lsl #16
 80092d0:	9901      	ldr	r1, [sp, #4]
 80092d2:	fa2b f901 	lsr.w	r9, fp, r1
 80092d6:	ea48 010e 	orr.w	r1, r8, lr
 80092da:	4321      	orrs	r1, r4
 80092dc:	9c00      	ldr	r4, [sp, #0]
 80092de:	f004 0418 	and.w	r4, r4, #24
 80092e2:	40a1      	lsls	r1, r4
 80092e4:	ea41 0109 	orr.w	r1, r1, r9
 80092e8:	6019      	str	r1, [r3, #0]
 80092ea:	e640      	b.n	8008f6e <compiler_builtins::mem::memmove+0x248>
 80092ec:	e9dd 2103 	ldrd	r2, r1, [sp, #12]
 80092f0:	4411      	add	r1, r2
 80092f2:	3908      	subs	r1, #8
 80092f4:	9107      	str	r1, [sp, #28]
 80092f6:	46a3      	mov	fp, r4
 80092f8:	f8dd 9004 	ldr.w	r9, [sp, #4]
 80092fc:	9c02      	ldr	r4, [sp, #8]
 80092fe:	a90c      	add	r1, sp, #48	@ 0x30
 8009300:	2600      	movs	r6, #0
 8009302:	eb01 0c04 	add.w	ip, r1, r4
 8009306:	9907      	ldr	r1, [sp, #28]
 8009308:	960c      	str	r6, [sp, #48]	@ 0x30
 800930a:	4421      	add	r1, r4
 800930c:	1f0a      	subs	r2, r1, #4
 800930e:	f1c4 0104 	rsb	r1, r4, #4
 8009312:	07cc      	lsls	r4, r1, #31
 8009314:	bf1e      	ittt	ne
 8009316:	7814      	ldrbne	r4, [r2, #0]
 8009318:	f88c 4000 	strbne.w	r4, [ip]
 800931c:	2601      	movne	r6, #1
 800931e:	0789      	lsls	r1, r1, #30
 8009320:	bf44      	itt	mi
 8009322:	5b91      	ldrhmi	r1, [r2, r6]
 8009324:	f82c 1006 	strhmi.w	r1, [ip, r6]
 8009328:	990c      	ldr	r1, [sp, #48]	@ 0x30
 800932a:	9a06      	ldr	r2, [sp, #24]
 800932c:	40d1      	lsrs	r1, r2
 800932e:	f009 0218 	and.w	r2, r9, #24
 8009332:	fa0b f202 	lsl.w	r2, fp, r2
 8009336:	4311      	orrs	r1, r2
 8009338:	f845 1c04 	str.w	r1, [r5, #-4]
 800933c:	e5a4      	b.n	8008e88 <compiler_builtins::mem::memmove+0x162>

0800933e <__aeabi_memmove8>:
 800933e:	b580      	push	{r7, lr}
 8009340:	466f      	mov	r7, sp
 8009342:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8009346:	f7ff bcee 	b.w	8008d26 <compiler_builtins::mem::memmove>

0800934a <__aeabi_memclr4>:
 800934a:	b580      	push	{r7, lr}
 800934c:	466f      	mov	r7, sp
 800934e:	2904      	cmp	r1, #4
 8009350:	d305      	bcc.n	800935e <__aeabi_memclr4+0x14>
 8009352:	1f0b      	subs	r3, r1, #4
 8009354:	43da      	mvns	r2, r3
 8009356:	f012 0f0c 	tst.w	r2, #12
 800935a:	d102      	bne.n	8009362 <__aeabi_memclr4+0x18>
 800935c:	e01a      	b.n	8009394 <__aeabi_memclr4+0x4a>
 800935e:	4602      	mov	r2, r0
 8009360:	e024      	b.n	80093ac <__aeabi_memclr4+0x62>
 8009362:	f04f 0c00 	mov.w	ip, #0
 8009366:	4602      	mov	r2, r0
 8009368:	f842 cb04 	str.w	ip, [r2], #4
 800936c:	f013 0f0c 	tst.w	r3, #12
 8009370:	d008      	beq.n	8009384 <__aeabi_memclr4+0x3a>
 8009372:	f003 020c 	and.w	r2, r3, #12
 8009376:	f8c0 c004 	str.w	ip, [r0, #4]
 800937a:	2a04      	cmp	r2, #4
 800937c:	d105      	bne.n	800938a <__aeabi_memclr4+0x40>
 800937e:	3908      	subs	r1, #8
 8009380:	3008      	adds	r0, #8
 8009382:	e006      	b.n	8009392 <__aeabi_memclr4+0x48>
 8009384:	4619      	mov	r1, r3
 8009386:	4610      	mov	r0, r2
 8009388:	e004      	b.n	8009394 <__aeabi_memclr4+0x4a>
 800938a:	2200      	movs	r2, #0
 800938c:	390c      	subs	r1, #12
 800938e:	6082      	str	r2, [r0, #8]
 8009390:	300c      	adds	r0, #12
 8009392:	4602      	mov	r2, r0
 8009394:	2b0c      	cmp	r3, #12
 8009396:	d309      	bcc.n	80093ac <__aeabi_memclr4+0x62>
 8009398:	2300      	movs	r3, #0
 800939a:	4602      	mov	r2, r0
 800939c:	3910      	subs	r1, #16
 800939e:	e9c2 3300 	strd	r3, r3, [r2]
 80093a2:	e9c2 3302 	strd	r3, r3, [r2, #8]
 80093a6:	3210      	adds	r2, #16
 80093a8:	2903      	cmp	r1, #3
 80093aa:	d8f7      	bhi.n	800939c <__aeabi_memclr4+0x52>
 80093ac:	1850      	adds	r0, r2, r1
 80093ae:	4282      	cmp	r2, r0
 80093b0:	d221      	bcs.n	80093f6 <__aeabi_memclr4+0xac>
 80093b2:	f1a1 0c01 	sub.w	ip, r1, #1
 80093b6:	f011 0303 	ands.w	r3, r1, #3
 80093ba:	d00c      	beq.n	80093d6 <__aeabi_memclr4+0x8c>
 80093bc:	f04f 0e00 	mov.w	lr, #0
 80093c0:	4611      	mov	r1, r2
 80093c2:	f801 eb01 	strb.w	lr, [r1], #1
 80093c6:	2b01      	cmp	r3, #1
 80093c8:	d00a      	beq.n	80093e0 <__aeabi_memclr4+0x96>
 80093ca:	2b02      	cmp	r3, #2
 80093cc:	f882 e001 	strb.w	lr, [r2, #1]
 80093d0:	d103      	bne.n	80093da <__aeabi_memclr4+0x90>
 80093d2:	1c91      	adds	r1, r2, #2
 80093d4:	e004      	b.n	80093e0 <__aeabi_memclr4+0x96>
 80093d6:	4611      	mov	r1, r2
 80093d8:	e002      	b.n	80093e0 <__aeabi_memclr4+0x96>
 80093da:	2100      	movs	r1, #0
 80093dc:	7091      	strb	r1, [r2, #2]
 80093de:	1cd1      	adds	r1, r2, #3
 80093e0:	f1bc 0f03 	cmp.w	ip, #3
 80093e4:	bf38      	it	cc
 80093e6:	bd80      	popcc	{r7, pc}
 80093e8:	3904      	subs	r1, #4
 80093ea:	2200      	movs	r2, #0
 80093ec:	f841 2f04 	str.w	r2, [r1, #4]!
 80093f0:	1d0b      	adds	r3, r1, #4
 80093f2:	4283      	cmp	r3, r0
 80093f4:	d1fa      	bne.n	80093ec <__aeabi_memclr4+0xa2>
 80093f6:	bd80      	pop	{r7, pc}

080093f8 <__aeabi_memcpy>:
 80093f8:	b580      	push	{r7, lr}
 80093fa:	466f      	mov	r7, sp
 80093fc:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8009400:	f7ff bb1e 	b.w	8008a40 <compiler_builtins::mem::memcpy>

08009404 <__aeabi_memcpy8>:
 8009404:	2a04      	cmp	r2, #4
 8009406:	d309      	bcc.n	800941c <__aeabi_memcpy8+0x18>
 8009408:	b5d0      	push	{r4, r6, r7, lr}
 800940a:	af02      	add	r7, sp, #8
 800940c:	f1a2 0e04 	sub.w	lr, r2, #4
 8009410:	ea6f 030e 	mvn.w	r3, lr
 8009414:	f013 0f0c 	tst.w	r3, #12
 8009418:	d106      	bne.n	8009428 <__aeabi_memcpy8+0x24>
 800941a:	e023      	b.n	8009464 <__aeabi_memcpy8+0x60>
 800941c:	460b      	mov	r3, r1
 800941e:	4684      	mov	ip, r0
 8009420:	4660      	mov	r0, ip
 8009422:	4619      	mov	r1, r3
 8009424:	f7ff bb0c 	b.w	8008a40 <compiler_builtins::mem::memcpy>
 8009428:	460b      	mov	r3, r1
 800942a:	4684      	mov	ip, r0
 800942c:	f853 4b04 	ldr.w	r4, [r3], #4
 8009430:	f01e 0f0c 	tst.w	lr, #12
 8009434:	f84c 4b04 	str.w	r4, [ip], #4
 8009438:	d009      	beq.n	800944e <__aeabi_memcpy8+0x4a>
 800943a:	684b      	ldr	r3, [r1, #4]
 800943c:	6043      	str	r3, [r0, #4]
 800943e:	f00e 030c 	and.w	r3, lr, #12
 8009442:	2b04      	cmp	r3, #4
 8009444:	d107      	bne.n	8009456 <__aeabi_memcpy8+0x52>
 8009446:	3a08      	subs	r2, #8
 8009448:	3108      	adds	r1, #8
 800944a:	3008      	adds	r0, #8
 800944c:	e008      	b.n	8009460 <__aeabi_memcpy8+0x5c>
 800944e:	4672      	mov	r2, lr
 8009450:	4660      	mov	r0, ip
 8009452:	4619      	mov	r1, r3
 8009454:	e006      	b.n	8009464 <__aeabi_memcpy8+0x60>
 8009456:	688b      	ldr	r3, [r1, #8]
 8009458:	3a0c      	subs	r2, #12
 800945a:	6083      	str	r3, [r0, #8]
 800945c:	310c      	adds	r1, #12
 800945e:	300c      	adds	r0, #12
 8009460:	4684      	mov	ip, r0
 8009462:	460b      	mov	r3, r1
 8009464:	f1be 0f0c 	cmp.w	lr, #12
 8009468:	d314      	bcc.n	8009494 <__aeabi_memcpy8+0x90>
 800946a:	4684      	mov	ip, r0
 800946c:	460b      	mov	r3, r1
 800946e:	6818      	ldr	r0, [r3, #0]
 8009470:	3a10      	subs	r2, #16
 8009472:	f8cc 0000 	str.w	r0, [ip]
 8009476:	2a03      	cmp	r2, #3
 8009478:	6858      	ldr	r0, [r3, #4]
 800947a:	f8cc 0004 	str.w	r0, [ip, #4]
 800947e:	6898      	ldr	r0, [r3, #8]
 8009480:	f8cc 0008 	str.w	r0, [ip, #8]
 8009484:	68d8      	ldr	r0, [r3, #12]
 8009486:	f103 0310 	add.w	r3, r3, #16
 800948a:	f8cc 000c 	str.w	r0, [ip, #12]
 800948e:	f10c 0c10 	add.w	ip, ip, #16
 8009492:	d8ec      	bhi.n	800946e <__aeabi_memcpy8+0x6a>
 8009494:	e8bd 40d0 	ldmia.w	sp!, {r4, r6, r7, lr}
 8009498:	4660      	mov	r0, ip
 800949a:	4619      	mov	r1, r3
 800949c:	f7ff bad0 	b.w	8008a40 <compiler_builtins::mem::memcpy>

080094a0 <__aeabi_memclr>:
 80094a0:	b5b0      	push	{r4, r5, r7, lr}
 80094a2:	af02      	add	r7, sp, #8
 80094a4:	2910      	cmp	r1, #16
 80094a6:	d33f      	bcc.n	8009528 <__aeabi_memclr+0x88>
 80094a8:	4242      	negs	r2, r0
 80094aa:	f002 0503 	and.w	r5, r2, #3
 80094ae:	1942      	adds	r2, r0, r5
 80094b0:	4290      	cmp	r0, r2
 80094b2:	d220      	bcs.n	80094f6 <__aeabi_memclr+0x56>
 80094b4:	f1a5 0c01 	sub.w	ip, r5, #1
 80094b8:	b18d      	cbz	r5, 80094de <__aeabi_memclr+0x3e>
 80094ba:	2300      	movs	r3, #0
 80094bc:	4686      	mov	lr, r0
 80094be:	f80e 3b01 	strb.w	r3, [lr], #1
 80094c2:	2d01      	cmp	r5, #1
 80094c4:	d007      	beq.n	80094d6 <__aeabi_memclr+0x36>
 80094c6:	7043      	strb	r3, [r0, #1]
 80094c8:	2d02      	cmp	r5, #2
 80094ca:	bf1a      	itte	ne
 80094cc:	7083      	strbne	r3, [r0, #2]
 80094ce:	f100 0e03 	addne.w	lr, r0, #3
 80094d2:	f100 0e02 	addeq.w	lr, r0, #2
 80094d6:	f1bc 0f03 	cmp.w	ip, #3
 80094da:	d204      	bcs.n	80094e6 <__aeabi_memclr+0x46>
 80094dc:	e00b      	b.n	80094f6 <__aeabi_memclr+0x56>
 80094de:	4686      	mov	lr, r0
 80094e0:	f1bc 0f03 	cmp.w	ip, #3
 80094e4:	d307      	bcc.n	80094f6 <__aeabi_memclr+0x56>
 80094e6:	f1ae 0004 	sub.w	r0, lr, #4
 80094ea:	2300      	movs	r3, #0
 80094ec:	f840 3f04 	str.w	r3, [r0, #4]!
 80094f0:	1d04      	adds	r4, r0, #4
 80094f2:	4294      	cmp	r4, r2
 80094f4:	d1fa      	bne.n	80094ec <__aeabi_memclr+0x4c>
 80094f6:	1b49      	subs	r1, r1, r5
 80094f8:	f021 0003 	bic.w	r0, r1, #3
 80094fc:	4410      	add	r0, r2
 80094fe:	4282      	cmp	r2, r0
 8009500:	d210      	bcs.n	8009524 <__aeabi_memclr+0x84>
 8009502:	2300      	movs	r3, #0
 8009504:	f842 3b04 	str.w	r3, [r2], #4
 8009508:	4282      	cmp	r2, r0
 800950a:	d20b      	bcs.n	8009524 <__aeabi_memclr+0x84>
 800950c:	f842 3b04 	str.w	r3, [r2], #4
 8009510:	4282      	cmp	r2, r0
 8009512:	bf3c      	itt	cc
 8009514:	f842 3b04 	strcc.w	r3, [r2], #4
 8009518:	4282      	cmpcc	r2, r0
 800951a:	d203      	bcs.n	8009524 <__aeabi_memclr+0x84>
 800951c:	f842 3b04 	str.w	r3, [r2], #4
 8009520:	4282      	cmp	r2, r0
 8009522:	d3ef      	bcc.n	8009504 <__aeabi_memclr+0x64>
 8009524:	f001 0103 	and.w	r1, r1, #3
 8009528:	1842      	adds	r2, r0, r1
 800952a:	4290      	cmp	r0, r2
 800952c:	d21f      	bcs.n	800956e <__aeabi_memclr+0xce>
 800952e:	1e4b      	subs	r3, r1, #1
 8009530:	f011 0403 	ands.w	r4, r1, #3
 8009534:	d00c      	beq.n	8009550 <__aeabi_memclr+0xb0>
 8009536:	f04f 0c00 	mov.w	ip, #0
 800953a:	4601      	mov	r1, r0
 800953c:	f801 cb01 	strb.w	ip, [r1], #1
 8009540:	2c01      	cmp	r4, #1
 8009542:	d00a      	beq.n	800955a <__aeabi_memclr+0xba>
 8009544:	2c02      	cmp	r4, #2
 8009546:	f880 c001 	strb.w	ip, [r0, #1]
 800954a:	d103      	bne.n	8009554 <__aeabi_memclr+0xb4>
 800954c:	1c81      	adds	r1, r0, #2
 800954e:	e004      	b.n	800955a <__aeabi_memclr+0xba>
 8009550:	4601      	mov	r1, r0
 8009552:	e002      	b.n	800955a <__aeabi_memclr+0xba>
 8009554:	2100      	movs	r1, #0
 8009556:	7081      	strb	r1, [r0, #2]
 8009558:	1cc1      	adds	r1, r0, #3
 800955a:	2b03      	cmp	r3, #3
 800955c:	bf38      	it	cc
 800955e:	bdb0      	popcc	{r4, r5, r7, pc}
 8009560:	1f08      	subs	r0, r1, #4
 8009562:	2100      	movs	r1, #0
 8009564:	f840 1f04 	str.w	r1, [r0, #4]!
 8009568:	1d03      	adds	r3, r0, #4
 800956a:	4293      	cmp	r3, r2
 800956c:	d1fa      	bne.n	8009564 <__aeabi_memclr+0xc4>
 800956e:	bdb0      	pop	{r4, r5, r7, pc}

08009570 <__aeabi_uldivmod>:
 8009570:	b510      	push	{r4, lr}
 8009572:	b084      	sub	sp, #16
 8009574:	ac02      	add	r4, sp, #8
 8009576:	9400      	str	r4, [sp, #0]
 8009578:	f000 f804 	bl	8009584 <__udivmoddi4>
 800957c:	9a02      	ldr	r2, [sp, #8]
 800957e:	9b03      	ldr	r3, [sp, #12]
 8009580:	b004      	add	sp, #16
 8009582:	bd10      	pop	{r4, pc}

08009584 <__udivmoddi4>:
 8009584:	b580      	push	{r7, lr}
 8009586:	466f      	mov	r7, sp
 8009588:	b086      	sub	sp, #24
 800958a:	4684      	mov	ip, r0
 800958c:	a802      	add	r0, sp, #8
 800958e:	e9cd 2300 	strd	r2, r3, [sp]
 8009592:	4662      	mov	r2, ip
 8009594:	460b      	mov	r3, r1
 8009596:	f000 f80d 	bl	80095b4 <compiler_builtins::int::specialized_div_rem::u64_div_rem>
 800959a:	f8d7 c008 	ldr.w	ip, [r7, #8]
 800959e:	e9dd 0102 	ldrd	r0, r1, [sp, #8]
 80095a2:	f1bc 0f00 	cmp.w	ip, #0
 80095a6:	bf1c      	itt	ne
 80095a8:	e9dd 3204 	ldrdne	r3, r2, [sp, #16]
 80095ac:	e9cc 3200 	strdne	r3, r2, [ip]
 80095b0:	b006      	add	sp, #24
 80095b2:	bd80      	pop	{r7, pc}

080095b4 <compiler_builtins::int::specialized_div_rem::u64_div_rem>:
 80095b4:	b5f0      	push	{r4, r5, r6, r7, lr}
 80095b6:	af03      	add	r7, sp, #12
 80095b8:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 80095bc:	e9d7 e802 	ldrd	lr, r8, [r7, #8]
 80095c0:	f1be 0f00 	cmp.w	lr, #0
 80095c4:	d072      	beq.n	80096ac <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xf8>
 80095c6:	f1b8 0f00 	cmp.w	r8, #0
 80095ca:	d16f      	bne.n	80096ac <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xf8>
 80095cc:	2b00      	cmp	r3, #0
 80095ce:	f000 80fc 	beq.w	80097ca <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x216>
 80095d2:	4573      	cmp	r3, lr
 80095d4:	f080 8107 	bcs.w	80097e6 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x232>
 80095d8:	fab3 f183 	clz	r1, r3
 80095dc:	fabe f68e 	clz	r6, lr
 80095e0:	1a71      	subs	r1, r6, r1
 80095e2:	f101 0620 	add.w	r6, r1, #32
 80095e6:	bf08      	it	eq
 80095e8:	261f      	moveq	r6, #31
 80095ea:	f1c6 0520 	rsb	r5, r6, #32
 80095ee:	fa08 f106 	lsl.w	r1, r8, r6
 80095f2:	fa0e f806 	lsl.w	r8, lr, r6
 80095f6:	fa2e f505 	lsr.w	r5, lr, r5
 80095fa:	4329      	orrs	r1, r5
 80095fc:	f1b6 0520 	subs.w	r5, r6, #32
 8009600:	f006 061f 	and.w	r6, r6, #31
 8009604:	bf58      	it	pl
 8009606:	fa0e f105 	lslpl.w	r1, lr, r5
 800960a:	f04f 0501 	mov.w	r5, #1
 800960e:	fa05 fc06 	lsl.w	ip, r5, r6
 8009612:	f04f 0500 	mov.w	r5, #0
 8009616:	bf58      	it	pl
 8009618:	f04f 0800 	movpl.w	r8, #0
 800961c:	e008      	b.n	8009630 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x7c>
 800961e:	4622      	mov	r2, r4
 8009620:	4633      	mov	r3, r6
 8009622:	ea4f 1418 	mov.w	r4, r8, lsr #4
 8009626:	ea44 7801 	orr.w	r8, r4, r1, lsl #28
 800962a:	ea4f 1c1c 	mov.w	ip, ip, lsr #4
 800962e:	0909      	lsrs	r1, r1, #4
 8009630:	ebb2 0408 	subs.w	r4, r2, r8
 8009634:	eb63 0601 	sbc.w	r6, r3, r1
 8009638:	2e00      	cmp	r6, #0
 800963a:	d403      	bmi.n	8009644 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x90>
 800963c:	ea45 050c 	orr.w	r5, r5, ip
 8009640:	d102      	bne.n	8009648 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x94>
 8009642:	e02d      	b.n	80096a0 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xec>
 8009644:	4614      	mov	r4, r2
 8009646:	461e      	mov	r6, r3
 8009648:	ea5f 0351 	movs.w	r3, r1, lsr #1
 800964c:	ea4f 0238 	mov.w	r2, r8, rrx
 8009650:	1aa2      	subs	r2, r4, r2
 8009652:	eb66 0303 	sbc.w	r3, r6, r3
 8009656:	2b00      	cmp	r3, #0
 8009658:	d404      	bmi.n	8009664 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xb0>
 800965a:	ea45 055c 	orr.w	r5, r5, ip, lsr #1
 800965e:	4614      	mov	r4, r2
 8009660:	d102      	bne.n	8009668 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xb4>
 8009662:	e01d      	b.n	80096a0 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xec>
 8009664:	4622      	mov	r2, r4
 8009666:	4633      	mov	r3, r6
 8009668:	ea4f 0498 	mov.w	r4, r8, lsr #2
 800966c:	ea44 7481 	orr.w	r4, r4, r1, lsl #30
 8009670:	1b14      	subs	r4, r2, r4
 8009672:	eb63 0691 	sbc.w	r6, r3, r1, lsr #2
 8009676:	2e00      	cmp	r6, #0
 8009678:	d403      	bmi.n	8009682 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xce>
 800967a:	ea45 059c 	orr.w	r5, r5, ip, lsr #2
 800967e:	d102      	bne.n	8009686 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xd2>
 8009680:	e00e      	b.n	80096a0 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xec>
 8009682:	4614      	mov	r4, r2
 8009684:	461e      	mov	r6, r3
 8009686:	ea4f 02d8 	mov.w	r2, r8, lsr #3
 800968a:	ea42 7241 	orr.w	r2, r2, r1, lsl #29
 800968e:	1aa2      	subs	r2, r4, r2
 8009690:	eb66 03d1 	sbc.w	r3, r6, r1, lsr #3
 8009694:	2b00      	cmp	r3, #0
 8009696:	d4c2      	bmi.n	800961e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x6a>
 8009698:	ea45 05dc 	orr.w	r5, r5, ip, lsr #3
 800969c:	4614      	mov	r4, r2
 800969e:	d1c0      	bne.n	8009622 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x6e>
 80096a0:	fbb4 f1fe 	udiv	r1, r4, lr
 80096a4:	fb01 461e 	mls	r6, r1, lr, r4
 80096a8:	4329      	orrs	r1, r5
 80096aa:	e092      	b.n	80097d2 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x21e>
 80096ac:	ebb2 060e 	subs.w	r6, r2, lr
 80096b0:	f04f 0100 	mov.w	r1, #0
 80096b4:	eb73 0608 	sbcs.w	r6, r3, r8
 80096b8:	d37c      	bcc.n	80097b4 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x200>
 80096ba:	2b00      	cmp	r3, #0
 80096bc:	d07a      	beq.n	80097b4 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x200>
 80096be:	fab3 f183 	clz	r1, r3
 80096c2:	fab8 f688 	clz	r6, r8
 80096c6:	1a71      	subs	r1, r6, r1
 80096c8:	f001 063f 	and.w	r6, r1, #63	@ 0x3f
 80096cc:	f001 011f 	and.w	r1, r1, #31
 80096d0:	f1c6 0420 	rsb	r4, r6, #32
 80096d4:	fa08 f506 	lsl.w	r5, r8, r6
 80096d8:	fa0e fc06 	lsl.w	ip, lr, r6
 80096dc:	fa2e f404 	lsr.w	r4, lr, r4
 80096e0:	432c      	orrs	r4, r5
 80096e2:	f1b6 0520 	subs.w	r5, r6, #32
 80096e6:	bf58      	it	pl
 80096e8:	fa0e f405 	lslpl.w	r4, lr, r5
 80096ec:	f04f 0501 	mov.w	r5, #1
 80096f0:	fa05 f901 	lsl.w	r9, r5, r1
 80096f4:	f04f 0100 	mov.w	r1, #0
 80096f8:	bf58      	it	pl
 80096fa:	f04f 0c00 	movpl.w	ip, #0
 80096fe:	e008      	b.n	8009712 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x15e>
 8009700:	4632      	mov	r2, r6
 8009702:	462b      	mov	r3, r5
 8009704:	ea4f 161c 	mov.w	r6, ip, lsr #4
 8009708:	ea46 7c04 	orr.w	ip, r6, r4, lsl #28
 800970c:	ea4f 1919 	mov.w	r9, r9, lsr #4
 8009710:	0924      	lsrs	r4, r4, #4
 8009712:	ebb2 060c 	subs.w	r6, r2, ip
 8009716:	eb73 0504 	sbcs.w	r5, r3, r4
 800971a:	d407      	bmi.n	800972c <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x178>
 800971c:	ea41 0109 	orr.w	r1, r1, r9
 8009720:	ebb6 020e 	subs.w	r2, r6, lr
 8009724:	eb75 0208 	sbcs.w	r2, r5, r8
 8009728:	d202      	bcs.n	8009730 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x17c>
 800972a:	e03a      	b.n	80097a2 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ee>
 800972c:	4616      	mov	r6, r2
 800972e:	461d      	mov	r5, r3
 8009730:	ea5f 0354 	movs.w	r3, r4, lsr #1
 8009734:	ea4f 023c 	mov.w	r2, ip, rrx
 8009738:	1ab2      	subs	r2, r6, r2
 800973a:	eb75 0303 	sbcs.w	r3, r5, r3
 800973e:	d409      	bmi.n	8009754 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1a0>
 8009740:	ebb2 050e 	subs.w	r5, r2, lr
 8009744:	ea41 0159 	orr.w	r1, r1, r9, lsr #1
 8009748:	eb73 0508 	sbcs.w	r5, r3, r8
 800974c:	4616      	mov	r6, r2
 800974e:	461d      	mov	r5, r3
 8009750:	d202      	bcs.n	8009758 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1a4>
 8009752:	e026      	b.n	80097a2 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ee>
 8009754:	4632      	mov	r2, r6
 8009756:	462b      	mov	r3, r5
 8009758:	ea4f 059c 	mov.w	r5, ip, lsr #2
 800975c:	ea45 7584 	orr.w	r5, r5, r4, lsl #30
 8009760:	1b56      	subs	r6, r2, r5
 8009762:	eb63 0594 	sbc.w	r5, r3, r4, lsr #2
 8009766:	2d00      	cmp	r5, #0
 8009768:	d407      	bmi.n	800977a <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1c6>
 800976a:	ea41 0199 	orr.w	r1, r1, r9, lsr #2
 800976e:	ebb6 020e 	subs.w	r2, r6, lr
 8009772:	eb75 0208 	sbcs.w	r2, r5, r8
 8009776:	d202      	bcs.n	800977e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ca>
 8009778:	e013      	b.n	80097a2 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ee>
 800977a:	4616      	mov	r6, r2
 800977c:	461d      	mov	r5, r3
 800977e:	ea4f 02dc 	mov.w	r2, ip, lsr #3
 8009782:	ea42 7244 	orr.w	r2, r2, r4, lsl #29
 8009786:	1ab2      	subs	r2, r6, r2
 8009788:	eb65 03d4 	sbc.w	r3, r5, r4, lsr #3
 800978c:	2b00      	cmp	r3, #0
 800978e:	d4b7      	bmi.n	8009700 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x14c>
 8009790:	ebb2 050e 	subs.w	r5, r2, lr
 8009794:	ea41 01d9 	orr.w	r1, r1, r9, lsr #3
 8009798:	eb73 0508 	sbcs.w	r5, r3, r8
 800979c:	4616      	mov	r6, r2
 800979e:	461d      	mov	r5, r3
 80097a0:	d2b0      	bcs.n	8009704 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x150>
 80097a2:	f04f 0c00 	mov.w	ip, #0
 80097a6:	e9c0 1c00 	strd	r1, ip, [r0]
 80097aa:	e9c0 6502 	strd	r6, r5, [r0, #8]
 80097ae:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80097b2:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80097b4:	f04f 0c00 	mov.w	ip, #0
 80097b8:	4616      	mov	r6, r2
 80097ba:	461d      	mov	r5, r3
 80097bc:	e9c0 1c00 	strd	r1, ip, [r0]
 80097c0:	e9c0 6502 	strd	r6, r5, [r0, #8]
 80097c4:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80097c8:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80097ca:	fbb2 f1fe 	udiv	r1, r2, lr
 80097ce:	fb01 261e 	mls	r6, r1, lr, r2
 80097d2:	f04f 0c00 	mov.w	ip, #0
 80097d6:	2500      	movs	r5, #0
 80097d8:	e9c0 1c00 	strd	r1, ip, [r0]
 80097dc:	e9c0 6502 	strd	r6, r5, [r0, #8]
 80097e0:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80097e4:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80097e6:	d10d      	bne.n	8009804 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x250>
 80097e8:	fbb2 f1f3 	udiv	r1, r2, r3
 80097ec:	2500      	movs	r5, #0
 80097ee:	fb01 2613 	mls	r6, r1, r3, r2
 80097f2:	f04f 0c01 	mov.w	ip, #1
 80097f6:	e9c0 1c00 	strd	r1, ip, [r0]
 80097fa:	e9c0 6502 	strd	r6, r5, [r0, #8]
 80097fe:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8009802:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8009804:	fbb3 fcfe 	udiv	ip, r3, lr
 8009808:	f5be 3f80 	cmp.w	lr, #65536	@ 0x10000
 800980c:	fb0c 351e 	mls	r5, ip, lr, r3
 8009810:	d21a      	bcs.n	8009848 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x294>
 8009812:	0429      	lsls	r1, r5, #16
 8009814:	2500      	movs	r5, #0
 8009816:	ea41 4112 	orr.w	r1, r1, r2, lsr #16
 800981a:	fbb1 f3fe 	udiv	r3, r1, lr
 800981e:	fb03 f10e 	mul.w	r1, r3, lr
 8009822:	ea4c 4c13 	orr.w	ip, ip, r3, lsr #16
 8009826:	ebc1 4112 	rsb	r1, r1, r2, lsr #16
 800982a:	eac2 4101 	pkhbt	r1, r2, r1, lsl #16
 800982e:	fbb1 f2fe 	udiv	r2, r1, lr
 8009832:	fb02 161e 	mls	r6, r2, lr, r1
 8009836:	ea42 4103 	orr.w	r1, r2, r3, lsl #16
 800983a:	e9c0 1c00 	strd	r1, ip, [r0]
 800983e:	e9c0 6502 	strd	r6, r5, [r0, #8]
 8009842:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8009846:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8009848:	ebb2 010e 	subs.w	r1, r2, lr
 800984c:	eb75 0108 	sbcs.w	r1, r5, r8
 8009850:	d208      	bcs.n	8009864 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2b0>
 8009852:	2100      	movs	r1, #0
 8009854:	4616      	mov	r6, r2
 8009856:	e9c0 1c00 	strd	r1, ip, [r0]
 800985a:	e9c0 6502 	strd	r6, r5, [r0, #8]
 800985e:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8009862:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8009864:	ea4f 71c8 	mov.w	r1, r8, lsl #31
 8009868:	ea41 035e 	orr.w	r3, r1, lr, lsr #1
 800986c:	ea4f 79ce 	mov.w	r9, lr, lsl #31
 8009870:	f04f 4800 	mov.w	r8, #2147483648	@ 0x80000000
 8009874:	2100      	movs	r1, #0
 8009876:	e008      	b.n	800988a <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2d6>
 8009878:	4622      	mov	r2, r4
 800987a:	4635      	mov	r5, r6
 800987c:	ea4f 1419 	mov.w	r4, r9, lsr #4
 8009880:	ea44 7903 	orr.w	r9, r4, r3, lsl #28
 8009884:	ea4f 1818 	mov.w	r8, r8, lsr #4
 8009888:	091b      	lsrs	r3, r3, #4
 800988a:	ebb2 0409 	subs.w	r4, r2, r9
 800988e:	eb65 0603 	sbc.w	r6, r5, r3
 8009892:	2e00      	cmp	r6, #0
 8009894:	d403      	bmi.n	800989e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2ea>
 8009896:	ea41 0108 	orr.w	r1, r1, r8
 800989a:	d102      	bne.n	80098a2 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2ee>
 800989c:	e02d      	b.n	80098fa <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x346>
 800989e:	4614      	mov	r4, r2
 80098a0:	462e      	mov	r6, r5
 80098a2:	ea5f 0553 	movs.w	r5, r3, lsr #1
 80098a6:	ea4f 0239 	mov.w	r2, r9, rrx
 80098aa:	1aa2      	subs	r2, r4, r2
 80098ac:	eb66 0505 	sbc.w	r5, r6, r5
 80098b0:	2d00      	cmp	r5, #0
 80098b2:	d404      	bmi.n	80098be <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x30a>
 80098b4:	ea41 0158 	orr.w	r1, r1, r8, lsr #1
 80098b8:	4614      	mov	r4, r2
 80098ba:	d102      	bne.n	80098c2 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x30e>
 80098bc:	e01d      	b.n	80098fa <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x346>
 80098be:	4622      	mov	r2, r4
 80098c0:	4635      	mov	r5, r6
 80098c2:	ea4f 0499 	mov.w	r4, r9, lsr #2
 80098c6:	ea44 7483 	orr.w	r4, r4, r3, lsl #30
 80098ca:	1b14      	subs	r4, r2, r4
 80098cc:	eb65 0693 	sbc.w	r6, r5, r3, lsr #2
 80098d0:	2e00      	cmp	r6, #0
 80098d2:	d403      	bmi.n	80098dc <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x328>
 80098d4:	ea41 0198 	orr.w	r1, r1, r8, lsr #2
 80098d8:	d102      	bne.n	80098e0 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x32c>
 80098da:	e00e      	b.n	80098fa <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x346>
 80098dc:	4614      	mov	r4, r2
 80098de:	462e      	mov	r6, r5
 80098e0:	ea4f 02d9 	mov.w	r2, r9, lsr #3
 80098e4:	ea42 7243 	orr.w	r2, r2, r3, lsl #29
 80098e8:	1aa2      	subs	r2, r4, r2
 80098ea:	eb66 05d3 	sbc.w	r5, r6, r3, lsr #3
 80098ee:	2d00      	cmp	r5, #0
 80098f0:	d4c2      	bmi.n	8009878 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2c4>
 80098f2:	ea41 01d8 	orr.w	r1, r1, r8, lsr #3
 80098f6:	4614      	mov	r4, r2
 80098f8:	d1c0      	bne.n	800987c <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2c8>
 80098fa:	fbb4 f2fe 	udiv	r2, r4, lr
 80098fe:	2500      	movs	r5, #0
 8009900:	fb02 461e 	mls	r6, r2, lr, r4
 8009904:	4311      	orrs	r1, r2
 8009906:	e9c0 1c00 	strd	r1, ip, [r0]
 800990a:	e9c0 6502 	strd	r6, r5, [r0, #8]
 800990e:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8009912:	bdf0      	pop	{r4, r5, r6, r7, pc}

08009914 <HardFaultTrampoline>:
 8009914:	4670      	mov	r0, lr
 8009916:	2104      	movs	r1, #4
 8009918:	4208      	tst	r0, r1
 800991a:	d103      	bne.n	8009924 <HardFaultTrampoline+0x10>
 800991c:	f3ef 8008 	mrs	r0, MSP
 8009920:	f000 b804 	b.w	800992c <HardFault_>
 8009924:	f3ef 8009 	mrs	r0, PSP
 8009928:	f000 b800 	b.w	800992c <HardFault_>

0800992c <HardFault_>:
 800992c:	b580      	push	{r7, lr}
 800992e:	466f      	mov	r7, sp
 8009930:	e7fe      	b.n	8009930 <HardFault_+0x4>
 8009932:	d4d4      	bmi.n	80098de <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x32a>
