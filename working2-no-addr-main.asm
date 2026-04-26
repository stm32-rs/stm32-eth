
./working2:     file format elf32-littlearm
./working2
architecture: armv7e-m, flags 0x00000112:
EXEC_P, HAS_SYMS, D_PAGED
start address 0x080001f9

Program Header:
    LOAD off    0x00010000 vaddr 0x08000000 paddr 0x08000000 align 2**16
         filesz 0x000001f8 memsz 0x000001f8 flags r--
    LOAD off    0x000101f8 vaddr 0x080001f8 paddr 0x080001f8 align 2**16
         filesz 0x00009734 memsz 0x00009734 flags r-x
    LOAD off    0x00019930 vaddr 0x08009930 paddr 0x08009930 align 2**16
         filesz 0x00001918 memsz 0x00001918 flags r--
    LOAD off    0x00020000 vaddr 0x20000000 paddr 0x0800b248 align 2**16
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
  1 .text         00009734  080001f8  080001f8  000101f8  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
  2 .rodata       00001918  08009930  08009930  00019930  2**3
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
  3 .data         00000038  20000000  0800b248  00020000  2**2
                  CONTENTS, ALLOC, LOAD, DATA
  4 .gnu.sgstubs  00000000  0800b280  0800b280  00020040  2**5
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
 10 .debug_info   0006fb6f  00000000  00000000  00056336  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 11 .debug_aranges 00001600  00000000  00000000  000c5ea5  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 12 .debug_ranges 0000ab30  00000000  00000000  000c74a5  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 13 .debug_str    0006ae81  00000000  00000000  000d1fd5  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 14 .comment      0000008b  00000000  00000000  0013ce56  2**0
                  CONTENTS, READONLY
 15 .ARM.attributes 00000038  00000000  00000000  0013cee1  2**0
                  CONTENTS, READONLY
 16 .debug_frame  00000eac  00000000  00000000  0013cf1c  2**2
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 17 .debug_line   00015a4f  00000000  00000000  0013ddc8  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 18 .debug_pubnames 000002be  00000000  00000000  00153817  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 19 .debug_pubtypes 00000047  00000000  00000000  00153ad5  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
SYMBOL TABLE:
00000000 l    df *ABS*	00000000 ip.db5b21dddbb36540-cgu.3
0800990c l     F .text	00000018 HardFaultTrampoline
08000250 l     F .text	0000010c smoltcp::wire::arp::Repr::emit
08009990 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.8
080037f6 l     F .text	00000032 core::slice::index::slice_index_fail
08009960 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.5
08009940 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.3
08009950 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.4
08009930 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.2
08009970 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.6
08009980 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.7
080099a0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.9
080038f4 l     F .text	00000028 core::panicking::panic_bounds_check
080099b0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.10
0800035c l     F .text	00000dc0 smoltcp::iface::interface::InterfaceInner::dispatch_ip
0800446e l     F .text	00000026 defmt::export::acquire_header_and_release
0800841a l     F .text	0000013e smoltcp::iface::route::Routes::lookup
0800b004 l     O .rodata	0000002c .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.7
0800b030 l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.8
0800391c l     F .text	0000000c core::panicking::panic
0800814e l     F .text	000002cc smoltcp::wire::ip::checksum::data
08007c8c l     F .text	0000017e smoltcp::wire::ipv4::Repr::emit
0800aad8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.41
080062b8 l     F .text	00000224 smoltcp::wire::tcp::TcpOption::emit
0800ab68 l     O .rodata	00000024 .Lanon.ee555cc9a30366a9abf098ddf788bd84.55
0800ab8c l     O .rodata	00000024 .Lanon.ee555cc9a30366a9abf098ddf788bd84.56
0800444c l     F .text	00000022 defmt::export::acquire_and_header
080043a8 l     F .text	000000a4 defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format
0800a510 l     O .rodata	00000036 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.22
0800a548 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.24
0800b0b0 l     O .rodata	0000001d .Lanon.cfd37be823b102280342b4283862930c.21
0800b0d0 l     O .rodata	00000010 .Lanon.cfd37be823b102280342b4283862930c.22
0800a9e4 l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.15
0800a9f4 l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.17
0800ae04 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.15
0800ae44 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.21
0800adc4 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.5
0800adf4 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.11
0800ade4 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.9
0800af50 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.12
0800af80 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.15
0800ab08 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.44
0800af70 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.14
0800aaf8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.43
0800af90 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.19
0800ab38 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.47
0800ab28 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.46
0800ab48 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.48
0800afa0 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.29
08004110 l     F .text	0000000e core::slice::copy_from_slice_impl::len_mismatch_fail
0800af60 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.13
0800abb0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.58
0800acd8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.91
0800acb8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.87
0800acc8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.88
0800ab18 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.45
0800aae8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.42
0800ad08 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.96
0800ad28 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.98
0800acf8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.95
0800ad18 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.97
0800b090 l     O .rodata	00000010 .Lanon.cfd37be823b102280342b4283862930c.17
08006062 l     F .text	00000086 core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>
0800aa04 l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.19
080037e2 l     F .text	00000014 core::option::unwrap_failed
0800add4 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.8
0800ae34 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.19
0800ae54 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.26
0800ae14 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.16
0800b050 l     O .rodata	00000010 .Lanon.cfd37be823b102280342b4283862930c.5
0800ace8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.92
0800111c l     F .text	000023dc smoltcp::iface::interface::Interface::poll
0800b0a0 l     O .rodata	00000010 .Lanon.cfd37be823b102280342b4283862930c.19
08008558 l     F .text	00000372 smoltcp::iface::neighbor::Cache::fill
08007aa6 l     F .text	000001b6 smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply
08007f3e l     F .text	00000092 smoltcp::socket::udp::Socket::accepts
08007fd0 l     F .text	0000017e smoltcp::socket::udp::Socket::process
08006234 l     F .text	00000084 <core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold
0800659c l     F .text	0000121e smoltcp::socket::tcp::Socket::process
080077ba l     F .text	00000096 smoltcp::socket::tcp::Socket::rst_reply
080064dc l     F .text	000000c0 smoltcp::socket::tcp::Socket::seq_to_transmit
0800ac70 l     O .rodata	00000038 .Lanon.ee555cc9a30366a9abf098ddf788bd84.70
0800aca8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.71
080038d8 l     F .text	0000001c core::panicking::panic_fmt
0800b0f0 l     O .rodata	00000041 .Lanon.4ff6eb6a4372ef11fb89d3866c9ca712.2
0800b134 l     O .rodata	00000010 .Lanon.4ff6eb6a4372ef11fb89d3866c9ca712.4
0800414a l     F .text	00000026 core::option::expect_failed
0800b144 l     O .rodata	00000042 .Lanon.4ff6eb6a4372ef11fb89d3866c9ca712.5
0800b188 l     O .rodata	00000010 .Lanon.4ff6eb6a4372ef11fb89d3866c9ca712.6
0800b060 l     O .rodata	00000010 .Lanon.cfd37be823b102280342b4283862930c.7
0800a9b4 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.20
0800a9d4 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.23
0800a558 l     O .rodata	00000016 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.25
0800a570 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.26
0800a980 l     O .rodata	00000022 .Lanon.02e056d4c889a085f771c01290e74e2f.18
0800a9a4 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.19
0800a950 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.13
0800a9c4 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.22
0800aa94 l     O .rodata	00000033 .Lanon.ee555cc9a30366a9abf098ddf788bd84.23
0800aac8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.25
0800a960 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.15
0800b070 l     O .rodata	00000010 .Lanon.cfd37be823b102280342b4283862930c.14
0800ae24 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.17
0800aeb4 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.12
080041ae l     F .text	00000014 core::panicking::panic_const::panic_const_rem_by_zero
0800b080 l     O .rodata	00000010 .Lanon.cfd37be823b102280342b4283862930c.16
0800a970 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.17
0800ae64 l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.21
0800ae74 l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.22
0800ae84 l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.23
0800ae94 l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.24
0800ada4 l     O .rodata	00000010 .Lanon.ee56ee3d56b2a23f692cb60d33ec53f4.8
0800ac40 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.67
0800aa24 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.2
0800ac20 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.65
0800ac30 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.66
0800ab58 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.54
080034f8 l     F .text	0000010c core::fmt::write
08003604 l     F .text	0000019c core::fmt::Formatter::pad_integral
080037a0 l     F .text	00000042 core::fmt::Formatter::pad_integral::write_prefix
0800a648 l     O .rodata	0000002b .Lanon.dc388a968c23fa524085dd94a83f5751.251
08003828 l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
08003854 l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
080038ac l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
08003880 l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
0800400e l     F .text	00000102 core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt
080060e8 l     F .text	00000088 __rustc::rust_begin_unwind
08003928 l     F .text	00000014 <&T as core::fmt::Display>::fmt
0800393c l     F .text	00000652 core::fmt::Formatter::pad
08003f8e l     F .text	00000014 core::panicking::panic_const::panic_const_div_by_zero
0800a673 l     O .rodata	00000019 .Lanon.dc388a968c23fa524085dd94a83f5751.271
08003fa2 l     F .text	00000010 <&T as core::fmt::Debug>::fmt
08003fb2 l     F .text	0000005c core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt
0800a580 l     O .rodata	000000c8 .Lanon.dc388a968c23fa524085dd94a83f5751.14
0800411e l     F .text	0000002c core::slice::copy_from_slice_impl::len_mismatch_fail::do_panic::runtime
08004170 l     F .text	0000003e core::result::unwrap_failed
0800a68c l     O .rodata	00000039 .Lanon.dc388a968c23fa524085dd94a83f5751.273
080041c2 l     F .text	000000ce <core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt
0800a6cc l     O .rodata	00000018 .Lanon.dc388a968c23fa524085dd94a83f5751.363
0800a6e4 l     O .rodata	00000010 .Lanon.dc388a968c23fa524085dd94a83f5751.365
0800a908 l     O .rodata	0000002b .Lanon.02e056d4c889a085f771c01290e74e2f.8
0800a6f4 l     O .rodata	00000010 .Lanon.dc388a968c23fa524085dd94a83f5751.366
0800a704 l     O .rodata	00000010 .Lanon.dc388a968c23fa524085dd94a83f5751.369
08004290 l     F .text	0000001a <core::fmt::Error as core::fmt::Debug>::fmt
0800a6c5 l     O .rodata	00000005 .Lanon.dc388a968c23fa524085dd94a83f5751.303
080042aa l     F .text	00000032 <core::net::display_buffer::DisplayBuffer<_> as core::fmt::Write>::write_str
080042dc l     F .text	000000a8 core::fmt::Write::write_char
08004384 l     F .text	00000018 core::fmt::Write::write_fmt
08004494 l     F .text	00000010 <defmt::export::FmtWrite as core::fmt::Write>::write_str
080044a4 l     F .text	00000088 core::fmt::Write::write_char
0800452c l     F .text	00000018 core::fmt::Write::write_fmt
0800a724 l     O .rodata	00000018 .Lanon.ea5d5760b73fa5b90415fc99172c09b2.1
08004544 l     F .text	00000082 defmt_rtt::channel::Channel::blocking_write
080045c6 l     F .text	0000005c defmt_rtt::channel::Channel::nonblocking_write
20000040 l     O .bss	00000034 .L_MergedGlobals
0800a73d l     O .rodata	0000001e .Lanon.1d905ed0c947f0695d8369569f054732.0
0800a75c l     O .rodata	00000010 .Lanon.1d905ed0c947f0695d8369569f054732.2
0800a76c l     O .rodata	00000010 .Lanon.1d905ed0c947f0695d8369569f054732.4
08004a94 l     F .text	000015b4 ip::__cortex_m_rt_main
0800b1d0 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.6
0800b0e0 l     O .rodata	00000010 .Lanon.cfd37be823b102280342b4283862930c.24
0800a8f8 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.7
0800a934 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.9
08007c5c l     F .text	0000002e <smoltcp::wire::ip::Address as core::fmt::Display>::fmt
0800adb4 l     O .rodata	00000010 .Lanon.ee56ee3d56b2a23f692cb60d33ec53f4.14
0800a7a0 l     O .rodata	00000150 .Lanon.02e056d4c889a085f771c01290e74e2f.5
0800a8f0 l     O .rodata	00000006 .Lanon.02e056d4c889a085f771c01290e74e2f.6
080099c0 l     O .rodata	00000023 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.11
0800a44c l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.13
0800a498 l     O .rodata	00000027 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.16
0800a4c0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.17
0800a45c l     O .rodata	00000029 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.14
0800a4d0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.18
0800a4f0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.20
0800a500 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.21
0800aefc l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.17
0800b1e0 l     O .rodata	00000027 .Lanon.4bc4785c9ada05daa616a70a22937c0f.10
0800b208 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.11
0800b198 l     O .rodata	00000028 .Lanon.4bc4785c9ada05daa616a70a22937c0f.3
0800b218 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.12
0800b228 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.13
0800b238 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.14
0800a488 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.15
0800b1c0 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.5
0800a4e0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.19
0800a77c l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.2
0800a78c l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.3
08006048 l     F .text	0000001a <stm32_eth::mac::WrongClock as core::fmt::Debug>::fmt
0800a944 l     O .rodata	0000000a .Lanon.02e056d4c889a085f771c01290e74e2f.10
20000038 l     O .bss	00000001 panic_probe::imp::panic::PANICKED
0800aa14 l     O .rodata	00000010 .Lanon.6fd57bafcc6bbd1a0f0c6a4a6d894bad.0
0800a73c l     O .rodata	00000001 .Lanon.ea5d5760b73fa5b90415fc99172c09b2.2
08006170 l     F .text	00000018 panic_probe::hard_fault
08006188 l     F .text	000000ac <&T as core::fmt::Display>::fmt
0800a714 l     O .rodata	0000000c .Lanon.dc388a968c23fa524085dd94a83f5751.408
0800a720 l     O .rodata	00000002 .Lanon.dc388a968c23fa524085dd94a83f5751.409
0800ac50 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.68
0800ac60 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.69
0800abd0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.60
0800ac00 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.63
0800aa84 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.6
0800aa74 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.4
0800abf0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.62
0800ac10 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.64
0800abc0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.59
0800abe0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.61
0800ad38 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.29
08007e6e l     F .text	00000066 defmt::export::fmt
08007e40 l     F .text	0000002e defmt::export::fmt
0800ad58 l     O .rodata	00000001 .Lanon.c1140054cd359bfdcd454e5b7dad9548.32
08007e0a l     F .text	00000036 defmt::export::fmt
08007a58 l     F .text	0000004e smoltcp::socket::tcp::Socket::immediate_ack_to_transmit
08007850 l     F .text	00000208 smoltcp::socket::tcp::Socket::ack_reply
0800ad48 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.31
0800ad94 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.36
0800aec4 l     O .rodata	00000025 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.14
0800aeec l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.15
0800aea4 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.11
0800ad59 l     O .rodata	00000028 .Lanon.c1140054cd359bfdcd454e5b7dad9548.33
0800ad84 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.35
0800b040 l     O .rodata	00000010 .Lanon.d4b94c958ea1f8000f4d53dc8b85c559.8
08007ed4 l     F .text	0000006a smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with
0800af40 l     O .rodata	00000010 .Lanon.c1510acb27196742b5d924b6a5d99da6.17
0800afb0 l     O .rodata	00000023 .Lanon.f0ac2cabe1edd51d584ebf3fae05aeec.16
0800afd4 l     O .rodata	00000010 .Lanon.f0ac2cabe1edd51d584ebf3fae05aeec.18
0800afe4 l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.4
0800af0c l     O .rodata	00000022 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.20
0800af30 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.22
0800aff4 l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.6
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
080088ee l     F .text	000000ae .hidden __aeabi_memclr8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.250
0800899c l     F .text	0000009c .hidden __aeabi_memcpy4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.012
08008a38 l     F .text	000002e6 .hidden compiler_builtins::mem::memcpy
08008d1e l     F .text	00000618 .hidden compiler_builtins::mem::memmove
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.275
08009336 l     F .text	0000000c .hidden __aeabi_memmove8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.279
08009342 l     F .text	000000ae .hidden __aeabi_memclr4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.288
080093f0 l     F .text	0000000c .hidden __aeabi_memcpy
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.320
080093fc l     F .text	0000009c .hidden __aeabi_memcpy8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.373
08009498 l     F .text	000000d0 .hidden __aeabi_memclr
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.413
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.362
0800957c l     F .text	00000030 .hidden __udivmoddi4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.017
080095ac l     F .text	00000360 .hidden compiler_builtins::int::specialized_div_rem::u64_div_rem
08000004 g     O .vector_table	00000004 __RESET_VECTOR
080001f8 g     F .text	0000003e Reset
08000008 g     O .vector_table	00000038 __EXCEPTIONS
0800439c g     F .text	00000000 DefaultHandler
08000040 g     O .vector_table	000001b8 __INTERRUPTS
08004622 g     F .text	0000009e _defmt_acquire
080046c0 g     F .text	000000f0 _defmt_release
080043a2 g     F .text	00000006 __defmt_default_timestamp
08009924 g     F .text	00000000 HardFault
080043a2 g     F .text	00000000 __pre_init
08004a8c g     F .text	00000008 main
00000022 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"Enable the `proto-ipv4-fragmentation` feature for fragmentation support.","disambiguator":"5448626954131158173","crate_name":"smoltcp"}
00000024 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"address {} not in neighbor cache, sending ARP request","disambiguator":"12066069798907769471","crate_name":"smoltcp"}
00000007 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=__internal_FormatSequence}","disambiguator":"8408127155438382822","crate_name":"defmt"}
080047b0 g     F .text	0000019c _defmt_write
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
08009568  w    F .text	00000014 __aeabi_uldivmod
080088e4 g     F .text	00000006 __primask_r
080088ca g     F .text	00000004 __cpsid
080088ce g     F .text	00000004 __cpsie
0800439c g     F .text	00000006 DefaultHandler_
080043a2 g     F .text	00000006 DefaultPreInit
08009924 g     F .text	00000006 HardFault_
00000032 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_write","data":"{}.{}.{}.{}","disambiguator":"14742771906021295210","crate_name":"defmt"}
00000002 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=u8}","disambiguator":"16912398427198947700","crate_name":"defmt"}
080043a2 g     F .text	00000000 _defmt_timestamp
20000008 g     O .data	00000030 _SEGGER_RTT
0800494c g     F .text	0000010e ETH
08004a5a g     F .text	00000032 SysTick
080088d2 g     F .text	0000000c __delay
080088de g     F .text	00000006 __dsb
0000002b g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Enabling ethernet...","disambiguator":"10404236363425019797","crate_name":"ip"}
0000002c g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Listening at {}:80...","disambiguator":"3686426055769507929","crate_name":"ip"}
0000002d g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Transmitted hello! Closing socket...","disambiguator":"11635372039424199218","crate_name":"ip"}
0000002e g     O .defmt	00000001 {"package":"panic-probe","tag":"defmt_error","data":"{}","disambiguator":"3281125884690968605","crate_name":"panic_probe"}
00000001 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=__internal_Display}","disambiguator":"14158116662542268607","crate_name":"defmt"}
080088ea g     F .text	00000004 __udf
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
0800439c g     F .text	00000000 NonMaskableInt
0800439c g     F .text	00000000 MemoryManagement
0800439c g     F .text	00000000 BusFault
0800439c g     F .text	00000000 UsageFault
0800439c g     F .text	00000000 SVCall
0800439c g     F .text	00000000 DebugMonitor
0800439c g     F .text	00000000 PendSV
00000030 g     O .defmt	00000001 _defmt_encoding_ = rzcobs
00000031 g     O .defmt	00000001 _defmt_version_ = 4
0800439c g     F .text	00000000 WWDG
0800439c g     F .text	00000000 PVD
0800439c g     F .text	00000000 TAMP_STAMP
0800439c g     F .text	00000000 RTC_WKUP
0800439c g     F .text	00000000 FLASH
0800439c g     F .text	00000000 RCC
0800439c g     F .text	00000000 EXTI0
0800439c g     F .text	00000000 EXTI1
0800439c g     F .text	00000000 EXTI2
0800439c g     F .text	00000000 EXTI3
0800439c g     F .text	00000000 EXTI4
0800439c g     F .text	00000000 DMA1_STREAM0
0800439c g     F .text	00000000 DMA1_STREAM1
0800439c g     F .text	00000000 DMA1_STREAM2
0800439c g     F .text	00000000 DMA1_STREAM3
0800439c g     F .text	00000000 DMA1_STREAM4
0800439c g     F .text	00000000 DMA1_STREAM5
0800439c g     F .text	00000000 DMA1_STREAM6
0800439c g     F .text	00000000 ADC
0800439c g     F .text	00000000 CAN1_TX
0800439c g     F .text	00000000 CAN1_RX0
0800439c g     F .text	00000000 CAN1_RX1
0800439c g     F .text	00000000 CAN1_SCE
0800439c g     F .text	00000000 EXTI9_5
0800439c g     F .text	00000000 TIM1_BRK_TIM9
0800439c g     F .text	00000000 TIM1_UP_TIM10
0800439c g     F .text	00000000 TIM1_TRG_COM_TIM11
0800439c g     F .text	00000000 TIM1_CC
0800439c g     F .text	00000000 TIM2
0800439c g     F .text	00000000 TIM3
0800439c g     F .text	00000000 TIM4
0800439c g     F .text	00000000 I2C1_EV
0800439c g     F .text	00000000 I2C1_ER
0800439c g     F .text	00000000 I2C2_EV
0800439c g     F .text	00000000 I2C2_ER
0800439c g     F .text	00000000 SPI1
0800439c g     F .text	00000000 SPI2
0800439c g     F .text	00000000 USART1
0800439c g     F .text	00000000 USART2
0800439c g     F .text	00000000 USART3
0800439c g     F .text	00000000 EXTI15_10
0800439c g     F .text	00000000 RTC_ALARM
0800439c g     F .text	00000000 OTG_FS_WKUP
0800439c g     F .text	00000000 TIM8_BRK_TIM12
0800439c g     F .text	00000000 TIM8_UP_TIM13
0800439c g     F .text	00000000 TIM8_TRG_COM_TIM14
0800439c g     F .text	00000000 TIM8_CC
0800439c g     F .text	00000000 DMA1_STREAM7
0800439c g     F .text	00000000 FMC
0800439c g     F .text	00000000 SDMMC1
0800439c g     F .text	00000000 TIM5
0800439c g     F .text	00000000 SPI3
0800439c g     F .text	00000000 UART4
0800439c g     F .text	00000000 UART5
0800439c g     F .text	00000000 TIM6_DAC
0800439c g     F .text	00000000 TIM7
0800439c g     F .text	00000000 DMA2_STREAM0
0800439c g     F .text	00000000 DMA2_STREAM1
0800439c g     F .text	00000000 DMA2_STREAM2
0800439c g     F .text	00000000 DMA2_STREAM3
0800439c g     F .text	00000000 DMA2_STREAM4
0800439c g     F .text	00000000 ETH_WKUP
0800439c g     F .text	00000000 CAN2_TX
0800439c g     F .text	00000000 CAN2_RX0
0800439c g     F .text	00000000 CAN2_RX1
0800439c g     F .text	00000000 CAN2_SCE
0800439c g     F .text	00000000 OTG_FS
0800439c g     F .text	00000000 DMA2_STREAM5
0800439c g     F .text	00000000 DMA2_STREAM6
0800439c g     F .text	00000000 DMA2_STREAM7
0800439c g     F .text	00000000 USART6
0800439c g     F .text	00000000 I2C3_EV
0800439c g     F .text	00000000 I2C3_ER
0800439c g     F .text	00000000 OTG_HS_EP1_OUT
0800439c g     F .text	00000000 OTG_HS_EP1_IN
0800439c g     F .text	00000000 OTG_HS_WKUP
0800439c g     F .text	00000000 OTG_HS
0800439c g     F .text	00000000 DCMI
0800439c g     F .text	00000000 CRYP
0800439c g     F .text	00000000 HASH_RNG
0800439c g     F .text	00000000 FPU
0800439c g     F .text	00000000 UART7
0800439c g     F .text	00000000 UART8
0800439c g     F .text	00000000 SPI4
0800439c g     F .text	00000000 SPI5
0800439c g     F .text	00000000 SPI6
0800439c g     F .text	00000000 SAI1
0800439c g     F .text	00000000 LTDC
0800439c g     F .text	00000000 LTDC_ER
0800439c g     F .text	00000000 DMA2D
0800439c g     F .text	00000000 SAI2
0800439c g     F .text	00000000 QUADSPI
0800439c g     F .text	00000000 LP_TIMER1
0800439c g     F .text	00000000 HDMI_CEC
0800439c g     F .text	00000000 I2C4_EV
0800439c g     F .text	00000000 I2C4_ER
0800439c g     F .text	00000000 SPDIFRX
0800439c g     F .text	00000000 DSIHOST
0800439c g     F .text	00000000 DFSDM1_FLT0
0800439c g     F .text	00000000 DFSDM1_FLT1
0800439c g     F .text	00000000 DFSDM1_FLT2
0800439c g     F .text	00000000 DFSDM1_FLT3
0800439c g     F .text	00000000 SDMMC2
0800439c g     F .text	00000000 CAN3_TX
0800439c g     F .text	00000000 CAN3_RX0
0800439c g     F .text	00000000 CAN3_RX1
0800439c g     F .text	00000000 CAN3_SCE
0800439c g     F .text	00000000 JPEG
0800439c g     F .text	00000000 MDIOS
20000038 g       .bss	00000000 __sbss
20000074 g       .bss	00000000 __ebss
20000000 g       .data	00000000 __sdata
20000038 g       .data	00000000 __edata
0800b248 g       *ABS*	00000000 __sidata
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
0800992c g       .text	00000000 __etext
08009930 g       .rodata	00000000 __srodata
0800b248 g       .rodata	00000000 __erodata
0800b280 g       .gnu.sgstubs	00000000 __veneer_base
0800b280 g       .gnu.sgstubs	00000000 __veneer_limit
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

<ip::__cortex_m_rt_main>:
	b5b0      	push	{r4, r5, r7, lr}

static TIME: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));
static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
	af02      	add	r7, sp, #8
	f5ad 5d01 	sub.w	sp, sp, #8256	@ 0x2040
                () => {
                    extern "C" {
                        fn $func($($args: $tys),*) $(-> $ret)?;
                    }

                    $func($($args),*)
	f003 ff22 	bl	<__primask_r>
	4604      	mov	r4, r0
	f003 ff12 	bl	<__cpsid>
impl Peripherals {
    #[doc = r"Returns all the peripherals *once*"]
    #[inline]
    pub fn take() -> Option<Self> {
        cortex_m::interrupt::free(|_| {
            if unsafe { DEVICE_PERIPHERALS } {
	f240 0540 	movw	r5, #64	@ 0x40

/// Reads the CPU register
#[inline]
pub fn read() -> Primask {
    let r: u32 = call_asm!(__primask_r() -> u32);
    if r & (1 << 0) == (1 << 0) {
	07e1      	lsls	r1, r4, #31
	f2c2 0500 	movt	r5, #8192	@ 0x2000
	79a8      	ldrb	r0, [r5, #6]
	d107      	bne.n	<ip::__cortex_m_rt_main+0x30>
	07c0      	lsls	r0, r0, #31
	f041 82b0 	bne.w	<ip::__cortex_m_rt_main+0x1586>
	2001      	movs	r0, #1
        })
    }
    #[doc = r"Unchecked version of `Peripherals::take`"]
    #[inline]
    pub unsafe fn steal() -> Self {
        DEVICE_PERIPHERALS = true;
	71a8      	strb	r0, [r5, #6]
	f003 ff06 	bl	<__cpsie>
    #[stable(feature = "rust1", since = "1.0.0")]
    #[rustc_diagnostic_item = "option_unwrap"]
    #[rustc_allow_const_fn_unstable(const_precise_live_drops)]
    #[rustc_const_stable(feature = "const_option", since = "1.83.0")]
    pub const fn unwrap(self) -> T {
        match self {
	e004      	b.n	<ip::__cortex_m_rt_main+0x3a>
            if unsafe { DEVICE_PERIPHERALS } {
	07c0      	lsls	r0, r0, #31
	f041 82aa 	bne.w	<ip::__cortex_m_rt_main+0x158a>
	2001      	movs	r0, #1
        DEVICE_PERIPHERALS = true;
	71a8      	strb	r0, [r5, #6]
	f003 ff09 	bl	<__primask_r>
	4604      	mov	r4, r0
	f003 fef9 	bl	<__cpsid>
impl Peripherals {
    /// Returns all the core peripherals *once*
    #[inline]
    pub fn take() -> Option<Self> {
        interrupt::free(|_| {
            if unsafe { TAKEN } {
	7968      	ldrb	r0, [r5, #5]
	07e1      	lsls	r1, r4, #31
	d107      	bne.n	<ip::__cortex_m_rt_main+0x5a>
	2800      	cmp	r0, #0
	f041 82a3 	bne.w	<ip::__cortex_m_rt_main+0x1596>
	2001      	movs	r0, #1
    }

    /// Unchecked version of `Peripherals::take`
    #[inline]
    pub unsafe fn steal() -> Self {
        TAKEN = true;
	7168      	strb	r0, [r5, #5]
	f003 fef1 	bl	<__cpsie>
	e004      	b.n	<ip::__cortex_m_rt_main+0x64>
            if unsafe { TAKEN } {
	2800      	cmp	r0, #0
	f041 829d 	bne.w	<ip::__cortex_m_rt_main+0x159a>
	2001      	movs	r0, #1
        TAKEN = true;
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

        Some((m, n, p, q))
    }

    fn pll_configure(&mut self) {
        let base_clk = match self.hse.as_ref() {
	f8c0 eaf0 	str.w	lr, [r0, #2800]	@ 0xaf0
	f8cd cca0 	str.w	ip, [sp, #3232]	@ 0xca0
	e001      	b.n	<ip::__cortex_m_rt_main+0xb2>
	f10a 0a01 	add.w	sl, sl, #1
            if f_vco_input < 1_000_000 {
	f1ba 0f09 	cmp.w	sl, #9
	f001 8246 	beq.w	<ip::__cortex_m_rt_main+0x1546>
	f1ba 0f04 	cmp.w	sl, #4
	d3f6      	bcc.n	<ip::__cortex_m_rt_main+0xae>
	fbb8 f0fa 	udiv	r0, r8, sl
	f44f 72d8 	mov.w	r2, #432	@ 0x1b0
	3001      	adds	r0, #1
	0840      	lsrs	r0, r0, #1
            if f_vco_input > 2_000_000 || n < 50 {
	fba0 010e 	umull	r0, r1, r0, lr
	e003      	b.n	<ip::__cortex_m_rt_main+0xda>
	1e55      	subs	r5, r2, #1
	2a33      	cmp	r2, #51	@ 0x33
	462a      	mov	r2, r5
	d3e9      	bcc.n	<ip::__cortex_m_rt_main+0xae>
	fba0 5602 	umull	r5, r6, r0, r2
	fb01 6602 	mla	r6, r1, r2, r6
            let f_vco_clock = (((f_pll_clock_input as u64 >> Self::BASE_CLK_SHIFT)
	0ead      	lsrs	r5, r5, #26
	ea45 1586 	orr.w	r5, r5, r6, lsl #6
            if f_vco_clock < 50_000_000 {
	42a5      	cmp	r5, r4
	d3e0      	bcc.n	<ip::__cortex_m_rt_main+0xae>
	f025 050f 	bic.w	r5, r5, #15
            if f_vco_clock > 432_000_000 {
	429d      	cmp	r5, r3
	d8ee      	bhi.n	<ip::__cortex_m_rt_main+0xd2>
                    if f_pll_clock_output >= p_freq_min && f_pll_clock_output <= p_freq_max {
	eb05 060b 	add.w	r6, r5, fp
                if div.is_some() {
	2e0f      	cmp	r6, #15
	bf24      	itt	cs
	444d      	addcs	r5, r9
	2d07      	cmpcs	r5, #7
	d2e7      	bcs.n	<ip::__cortex_m_rt_main+0xd2>
            let one_over_m = ((1 << Self::FIXED_POINT_LSHIFT) / (self.pllm as u32) + 1) >> 1;
	ea5f 600a 	movs.w	r0, sl, lsl #24
	d105      	bne.n	<ip::__cortex_m_rt_main+0x114>
	f24b 10d0 	movw	r0, #45520	@ 0xb1d0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7ff f9f3 	bl	<core::panicking::panic_const::panic_const_div_by_zero>
	fa5f f08a 	uxtb.w	r0, sl
	f04f 4100 	mov.w	r1, #2147483648	@ 0x80000000
	fbb1 f0f0 	udiv	r0, r1, r0
            sysclk = (((base_clk as u64 * self.plln as u64 * one_over_m as u64)
	b293      	uxth	r3, r2
            let one_over_m = ((1 << Self::FIXED_POINT_LSHIFT) / (self.pllm as u32) + 1) >> 1;
	3001      	adds	r0, #1
            sysclk = (((base_clk as u64 * self.plln as u64 * one_over_m as u64)
	2e0f      	cmp	r6, #15
            let one_over_m = ((1 << Self::FIXED_POINT_LSHIFT) / (self.pllm as u32) + 1) >> 1;
	ea4f 0050 	mov.w	r0, r0, lsr #1
            sysclk = (((base_clk as u64 * self.plln as u64 * one_over_m as u64)
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
        assert!(sysclk <= 216_000_000);
	1a08      	subs	r0, r1, r0
	f173 0000 	sbcs.w	r0, r3, #0
	f081 81b7 	bcs.w	<ip::__cortex_m_rt_main+0x14e0>
        let hpre_val: f32 = (sysclk as f32 / hclk as f32).ceil();
	ee00 1a10 	vmov	s0, r1
    #[rustc_diagnostic_item = "cmp_ord_min"]
    fn min(self, other: Self) -> Self
    where
        Self: Sized + [const] Destruct,
    {
        if other < self { other } else { self }
	4561      	cmp	r1, ip
	bf38      	it	cc
	468c      	movcc	ip, r1
	eeb8 1a40 	vcvt.f32.u32	s2, s0
	ee00 ca10 	vmov	s0, ip
	eeb8 0a40 	vcvt.f32.u32	s0, s0
impl Neg for F32 {
    type Output = F32;

    #[inline]
    fn neg(self) -> F32 {
        F32(-self.0)
	eeb1 2a41 	vneg.f32	s4, s2
	ee82 2a00 	vdiv.f32	s4, s4, s0
use super::F32;

impl F32 {
    /// Returns the largest integer less than or equal to a number.
    pub fn floor(self) -> Self {
        let mut res = (self.0 as i32) as f32;
	eebd 3ac2 	vcvt.s32.f32	s6, s4
	eebf 0a00 	vmov.f32	s0, #240	@ 0xbf800000 -1.0
	eeb8 3ac3 	vcvt.f32.s32	s6, s6

        if self.0 < res {
	eeb4 2a43 	vcmp.f32	s4, s6
	ee33 4a00 	vadd.f32	s8, s6, s0
	eef1 fa10 	vmrs	APSR_nzcv, fpscr
	bf48      	it	mi
	eeb0 3a44 	vmovmi.f32	s6, s8
	eeb1 2a43 	vneg.f32	s4, s6
        let (hpre_val, hpre): (f32, u8) = match hpre_val as u32 {
	eebc 2ac2 	vcvt.u32.f32	s4, s4
	ee12 0a10 	vmov	r0, s4
	2801      	cmp	r0, #1
	d011      	beq.n	<ip::__cortex_m_rt_main+0x1e2>
	2802      	cmp	r0, #2
	d00a      	beq.n	<ip::__cortex_m_rt_main+0x1d8>
	2800      	cmp	r0, #0
	f001 8197 	beq.w	<ip::__cortex_m_rt_main+0x14f6>
            3..=5 => (4.0, 0b1001),
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
        hclk = (sysclk as f32 / hpre_val).floor() as u32;
	ee81 1a02 	vdiv.f32	s2, s2, s4
	f249 5c00 	movw	ip, #38144	@ 0x9500
	f64f 1080 	movw	r0, #63872	@ 0xf980
	f24f 3300 	movw	r3, #62208	@ 0xf300
	f6c0 2cba 	movt	ip, #2746	@ 0xaba
        let max_pclk1 = if sysclk <= 180_000_000 {
	f10c 0901 	add.w	r9, ip, #1
	f2c0 3037 	movt	r0, #823	@ 0x337
	f2c0 636f 	movt	r3, #1647	@ 0x66f
        let mut res = (self.0 as i32) as f32;
	eebd 2ac1 	vcvt.s32.f32	s4, s2
	eeb8 2ac2 	vcvt.f32.s32	s4, s4
        if self.0 < res {
	eeb4 1a42 	vcmp.f32	s2, s4
	ee32 3a00 	vadd.f32	s6, s4, s0
	eef1 fa10 	vmrs	APSR_nzcv, fpscr
	bf48      	it	mi
	eeb0 2a43 	vmovmi.f32	s4, s6
        hclk = (sysclk as f32 / hpre_val).floor() as u32;
	eebc 1ac2 	vcvt.u32.f32	s2, s4
        let max_pclk1 = if sysclk <= 180_000_000 {
	4549      	cmp	r1, r9
	bf3c      	itt	cc
	f24a 5040 	movwcc	r0, #42304	@ 0xa540
	f2c0 20ae 	movtcc	r0, #686	@ 0x2ae
        let max_pclk2 = if sysclk <= 180_000_000 {
	bf3c      	itt	cc
	f644 2380 	movwcc	r3, #19072	@ 0x4a80
	f2c0 535d 	movtcc	r3, #1373	@ 0x55d
        hclk = (sysclk as f32 / hpre_val).floor() as u32;
	ee11 5a10 	vmov	r5, s2
        let mut ppre1_val: u32 = (hclk as f32 / pclk1 as f32).ceil() as u32;
	eeb8 1a41 	vcvt.f32.u32	s2, s2
	eeb1 1a41 	vneg.f32	s2, s2
	4285      	cmp	r5, r0
	bf38      	it	cc
	4628      	movcc	r0, r5
	ee02 0a10 	vmov	s4, r0
	eeb8 2a42 	vcvt.f32.u32	s4, s4
	ee81 2a02 	vdiv.f32	s4, s2, s4
        let mut res = (self.0 as i32) as f32;
	eebd 3ac2 	vcvt.s32.f32	s6, s4
	eeb8 3ac3 	vcvt.f32.s32	s6, s6
        if self.0 < res {
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
        config.ppre1 = match ppre1_val {
	2801      	cmp	r0, #1
	d00d      	beq.n	<ip::__cortex_m_rt_main+0x2a8>
	2802      	cmp	r0, #2
	d008      	beq.n	<ip::__cortex_m_rt_main+0x2a2>
	2800      	cmp	r0, #0
	f001 813b 	beq.w	<ip::__cortex_m_rt_main+0x150c>
            3..=6 => {
	1ec4      	subs	r4, r0, #3
	2c04      	cmp	r4, #4
	d210      	bcs.n	<ip::__cortex_m_rt_main+0x2be>
	f44f 58a0 	mov.w	r8, #5120	@ 0x1400
	e014      	b.n	<ip::__cortex_m_rt_main+0x2cc>
	f44f 5880 	mov.w	r8, #4096	@ 0x1000
                0b100
	e011      	b.n	<ip::__cortex_m_rt_main+0x2cc>
	f04f 0800 	mov.w	r8, #0
        config.ppre1 = match ppre1_val {
	e00e      	b.n	<ip::__cortex_m_rt_main+0x2cc>
            6..=11 => (8.0, 0b1010),
	1f83      	subs	r3, r0, #6
	2b06      	cmp	r3, #6
	d272      	bcs.n	<ip::__cortex_m_rt_main+0x39a>
	eeb2 2a00 	vmov.f32	s4, #32	@ 0x41000000  8.0
	f04f 0ea0 	mov.w	lr, #160	@ 0xa0
	e795      	b.n	<ip::__cortex_m_rt_main+0x1ea>
            7..=12 => {
	3807      	subs	r0, #7
	f44f 58e0 	mov.w	r8, #7168	@ 0x1c00
	2806      	cmp	r0, #6
	bf38      	it	cc
	f44f 58c0 	movcc.w	r8, #6144	@ 0x1800
        let mut ppre2_val: u32 = (hclk as f32 / pclk2 as f32).ceil() as u32;
	ee02 3a10 	vmov	s4, r3
	eeb8 2a42 	vcvt.f32.u32	s4, s4
	ee81 1a02 	vdiv.f32	s2, s2, s4
        let mut res = (self.0 as i32) as f32;
	eebd 2ac1 	vcvt.s32.f32	s4, s2
	eeb8 2ac2 	vcvt.f32.s32	s4, s4
        if self.0 < res {
	eeb4 1a42 	vcmp.f32	s2, s4
	ee32 0a00 	vadd.f32	s0, s4, s0
	eef1 fa10 	vmrs	APSR_nzcv, fpscr
	bf48      	it	mi
	eeb0 2a40 	vmovmi.f32	s4, s0
	eeb1 0a42 	vneg.f32	s0, s4
        config.ppre2 = match ppre2_val {
	9507      	str	r5, [sp, #28]
        let mut ppre2_val: u32 = (hclk as f32 / pclk2 as f32).ceil() as u32;
	eebc 0ac0 	vcvt.u32.f32	s0, s0
	ee10 0a10 	vmov	r0, s0
        config.ppre2 = match ppre2_val {
	2801      	cmp	r0, #1
	d019      	beq.n	<ip::__cortex_m_rt_main+0x338>
	2802      	cmp	r0, #2
	d00e      	beq.n	<ip::__cortex_m_rt_main+0x326>
	2800      	cmp	r0, #0
	f001 810a 	beq.w	<ip::__cortex_m_rt_main+0x1522>
            3..=6 => {
	1ec3      	subs	r3, r0, #3
	2b04      	cmp	r3, #4
	d21b      	bcs.n	<ip::__cortex_m_rt_main+0x34c>
	f44f 4b20 	mov.w	fp, #40960	@ 0xa000
	f24c 3081 	movw	r0, #50049	@ 0xc381
	f2c0 10c9 	movt	r0, #457	@ 0x1c9
        config.flash_waitstates = if sysclk <= 30_000_000 {
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
            7..=12 => {
	3807      	subs	r0, #7
	f44f 4b60 	mov.w	fp, #57344	@ 0xe000
	2806      	cmp	r0, #6
	bf38      	it	cc
	f44f 4b40 	movcc.w	fp, #49152	@ 0xc000
	f24c 3081 	movw	r0, #50049	@ 0xc381
	f2c0 10c9 	movt	r0, #457	@ 0x1c9
        config.flash_waitstates = if sysclk <= 30_000_000 {
	4281      	cmp	r1, r0
	d3f0      	bcc.n	<ip::__cortex_m_rt_main+0x348>
	f248 7000 	movw	r0, #34560	@ 0x8700
	f2c0 3093 	movt	r0, #915	@ 0x393
        } else if sysclk <= 60_000_000 {
	3001      	adds	r0, #1
	4281      	cmp	r1, r0
	d201      	bcs.n	<ip::__cortex_m_rt_main+0x378>
	2301      	movs	r3, #1
	e02d      	b.n	<ip::__cortex_m_rt_main+0x3d4>
	f644 2080 	movw	r0, #19072	@ 0x4a80
	f2c0 505d 	movt	r0, #1373	@ 0x55d
        } else if sysclk <= 90_000_000 {
	3001      	adds	r0, #1
	4281      	cmp	r1, r0
	d201      	bcs.n	<ip::__cortex_m_rt_main+0x38a>
	2302      	movs	r3, #2
	e024      	b.n	<ip::__cortex_m_rt_main+0x3d4>
	f640 6001 	movw	r0, #3585	@ 0xe01
	f2c0 7027 	movt	r0, #1831	@ 0x727
        } else if sysclk <= 120_000_000 {
	4281      	cmp	r1, r0
	d20b      	bcs.n	<ip::__cortex_m_rt_main+0x3ae>
	2303      	movs	r3, #3
	e01c      	b.n	<ip::__cortex_m_rt_main+0x3d4>
            12..=39 => (16.0, 0b1011),
	f1a0 030c 	sub.w	r3, r0, #12
	2b1c      	cmp	r3, #28
	f080 850d 	bcs.w	<ip::__cortex_m_rt_main+0xdbe>
	eeb3 2a00 	vmov.f32	s4, #48	@ 0x41800000  16.0
	f04f 0eb0 	mov.w	lr, #176	@ 0xb0
	e71d      	b.n	<ip::__cortex_m_rt_main+0x1ea>
	f24d 1080 	movw	r0, #53632	@ 0xd180
	2305      	movs	r3, #5
	f6c0 00f0 	movt	r0, #2288	@ 0x8f0
        } else if sysclk <= 150_000_000 {
	3001      	adds	r0, #1
	4281      	cmp	r1, r0
	bf38      	it	cc
	2304      	movcc	r3, #4
	4549      	cmp	r1, r9
	d307      	bcc.n	<ip::__cortex_m_rt_main+0x3d4>
	f645 0081 	movw	r0, #22657	@ 0x5881
	2307      	movs	r3, #7
	f6c0 4084 	movt	r0, #3204	@ 0xc84
        } else if sysclk <= 210_000_000 {
	4281      	cmp	r1, r0
	bf38      	it	cc
	2306      	movcc	r3, #6
	f643 0910 	movw	r9, #14352	@ 0x3810
	f2c4 0902 	movt	r9, #16386	@ 0x4002
            (
                addr: *const () = src as *const (),
                align: usize = align_of::<T>(),
            ) => ub_checks::maybe_is_aligned(addr, align)
        );
        intrinsics::volatile_load(src)
	f859 0c10 	ldr.w	r0, [r9, #-16]
            FI: Into<bool>,
        {
            /// Writes bit to the field
            #[inline(always)]
            pub fn bit(self, value: bool) -> &'a mut REG::Writer {
                self.w.bits = (self.w.bits & !(1 << { OF })) | ((<$U>::from(value) & 1) << { OF });
	f040 0001 	orr.w	r0, r0, #1
            (
                addr: *mut () = dst as *mut (),
                align: usize = align_of::<T>(),
            ) => ub_checks::maybe_is_aligned(addr, align)
        );
        intrinsics::volatile_store(dst, src);
	f849 0c10 	str.w	r0, [r9, #-16]
        intrinsics::volatile_load(src)
	f859 0c10 	ldr.w	r0, [r9, #-16]

        // Switch to fail-safe clock settings.
        // This is useful when booting from a bootloader that alters clock tree configuration.
        // Turn on HSI
        rcc.cr.modify(|_, w| w.hsion().set_bit());
        while rcc.cr.read().hsirdy().bit_is_clear() {}
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
            ///
            /// Passing incorrect value can cause undefined behaviour. See reference manual
            #[inline(always)]
            pub unsafe fn bits(self, value: N) -> &'a mut REG::Writer {
                self.w.bits =
                    (self.w.bits & !(Self::MASK << { OF })) | ((value.into() & Self::MASK) << { OF });
	f020 0003 	bic.w	r0, r0, #3
        intrinsics::volatile_store(dst, src);
	f849 0c08 	str.w	r0, [r9, #-8]
	f859 0c10 	ldr.w	r0, [r9, #-16]
        rcc.cfgr.modify(|_, w| w.sw().hsi());

        // Configure HSE if provided
        if self.hse.is_some() {
            // Configure the HSE mode
            match self.hse.as_ref().unwrap().mode {
	f440 2080 	orr.w	r0, r0, #262144	@ 0x40000
	f849 0c10 	str.w	r0, [r9, #-16]
        intrinsics::volatile_load(src)
	f859 0c10 	ldr.w	r0, [r9, #-16]
                self.w.bits = (self.w.bits & !(1 << { OF })) | ((<$U>::from(value) & 1) << { OF });
	f440 3080 	orr.w	r0, r0, #65536	@ 0x10000
        intrinsics::volatile_store(dst, src);
	f849 0c10 	str.w	r0, [r9, #-16]
	f647 2001 	movw	r0, #31233	@ 0x7a01
	f6c0 2003 	movt	r0, #2563	@ 0xa03
        config.vos_scale = if sysclk <= 144_000_000 {
	4281      	cmp	r1, r0
	f244 4001 	movw	r0, #17409	@ 0x4401
	bf38      	it	cc
	2501      	movcc	r5, #1
	f6c0 0095 	movt	r0, #2197	@ 0x895
	4281      	cmp	r1, r0
	bf38      	it	cc
	2502      	movcc	r5, #2
        intrinsics::volatile_load(src)
	f859 0c10 	ldr.w	r0, [r9, #-16]
                HSEClockMode::Bypass => rcc.cr.modify(|_, w| w.hsebyp().bypassed()),
                HSEClockMode::Oscillator => rcc.cr.modify(|_, w| w.hsebyp().not_bypassed()),
            }
            // Start HSE
            rcc.cr.modify(|_, w| w.hseon().on());
            while rcc.cr.read().hserdy().is_not_ready() {}
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
        {
            const MASK: $U = <$U>::MAX >> (<$U>::MAX.leading_ones() as u8 - { WI });
            /// Writes raw bits to the field
            #[inline(always)]
            pub fn bits(self, value: N) -> &'a mut REG::Writer {
                self.w.bits =
	2e0f      	cmp	r6, #15
                self.w.bits = (self.w.bits & !(1 << { OF })) | ((<$U>::from(value) & 1) << { OF });
	f020 7080 	bic.w	r0, r0, #16777216	@ 0x1000000
        intrinsics::volatile_store(dst, src);
	f849 0c10 	str.w	r0, [r9, #-16]
	f647 70c0 	movw	r0, #32704	@ 0x7fc0
        intrinsics::volatile_load(src)
	f859 4c0c 	ldr.w	r4, [r9, #-12]
                    (self.w.bits & !(Self::MASK << { OF })) | ((value.into() & Self::MASK) << { OF });
	ea00 1082 	and.w	r0, r0, r2, lsl #6
	f248 0200 	movw	r2, #32768	@ 0x8000
	f2cf 02bc 	movt	r2, #61628	@ 0xf0bc
	ea02 0204 	and.w	r2, r2, r4
	f00a 043f 	and.w	r4, sl, #63	@ 0x3f
	4422      	add	r2, r4
                self.w.bits =
	4410      	add	r0, r2
                self.w.bits =
	bf38      	it	cc
	f500 3080 	addcc.w	r0, r0, #65536	@ 0x10000
            });

            // Enable PWR domain and setup VOSscale and Overdrive options
            rcc.apb1enr.modify(|_, w| w.pwren().set_bit());

            pwr.cr1.modify(|_, w| match config.vos_scale {
	2d02      	cmp	r5, #2
                self.w.bits = (self.w.bits & !(1 << { OF })) | ((<$U>::from(value) & 1) << { OF });
	f100 7010 	add.w	r0, r0, #37748736	@ 0x2400000
        intrinsics::volatile_store(dst, src);
	f849 0c0c 	str.w	r0, [r9, #-12]
        intrinsics::volatile_load(src)
	f8d9 0030 	ldr.w	r0, [r9, #48]	@ 0x30
	f040 5080 	orr.w	r0, r0, #268435456	@ 0x10000000
        intrinsics::volatile_store(dst, src);
	f8c9 0030 	str.w	r0, [r9, #48]	@ 0x30
	f247 0004 	movw	r0, #28676	@ 0x7004
	f2c4 0000 	movt	r0, #16384	@ 0x4000
        intrinsics::volatile_load(src)
	f850 2c04 	ldr.w	r2, [r0, #-4]
	d003      	beq.n	<ip::__cortex_m_rt_main+0x4ce>
	2d01      	cmp	r5, #1
	d105      	bne.n	<ip::__cortex_m_rt_main+0x4d6>
	2402      	movs	r4, #2
                self.w.bits =
	e000      	b.n	<ip::__cortex_m_rt_main+0x4d0>
	2401      	movs	r4, #1
	f364 328f 	bfi	r2, r4, #14, #2
	e001      	b.n	<ip::__cortex_m_rt_main+0x4da>
	f442 4240 	orr.w	r2, r2, #49152	@ 0xc000
        intrinsics::volatile_store(dst, src);
	f840 2c04 	str.w	r2, [r0, #-4]
        intrinsics::volatile_load(src)
	f859 2c10 	ldr.w	r2, [r9, #-16]
                self.w.bits = (self.w.bits & !(1 << { OF })) | ((<$U>::from(value) & 1) << { OF });
	f042 7280 	orr.w	r2, r2, #16777216	@ 0x1000000
        intrinsics::volatile_store(dst, src);
	f849 2c10 	str.w	r2, [r9, #-16]
        intrinsics::volatile_load(src)
	f859 2c10 	ldr.w	r2, [r9, #-16]

            // Enable PLL
            rcc.cr.modify(|_, w| w.pllon().on());

            // Wait for PLL to stabilise
            while rcc.cr.read().pllrdy().is_not_ready() {}
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

            //Over-drive
            if config.overdrive {
	4561      	cmp	r1, ip
	d925      	bls.n	<ip::__cortex_m_rt_main+0x55c>
	f850 1c04 	ldr.w	r1, [r0, #-4]
	f441 3180 	orr.w	r1, r1, #65536	@ 0x10000
        intrinsics::volatile_store(dst, src);
	f840 1c04 	str.w	r1, [r0, #-4]
        intrinsics::volatile_load(src)
	6801      	ldr	r1, [r0, #0]
                // Entering Over-drive mode
                //enable the Over-drive mode
                pwr.cr1.modify(|_, w| w.oden().set_bit());

                //wait for the ODRDY flag to be set
                while !pwr.csr1.read().odrdy().bit_is_set() {}
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
        intrinsics::volatile_store(dst, src);
	f840 1c04 	str.w	r1, [r0, #-4]
        intrinsics::volatile_load(src)
	6801      	ldr	r1, [r0, #0]

                //switch the voltage regulator from Normal mode to Over-drive mode
                pwr.cr1.modify(|_, w| w.odswen().set_bit());

                //Wait for the ODSWRDY flag in the PWR_CSR1 to be set.
                while !pwr.csr1.read().odswrdy().bit_is_set() {}
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
                    (self.w.bits & !(Self::MASK << { OF })) | ((value.into() & Self::MASK) << { OF });
	4008      	ands	r0, r1
        intrinsics::volatile_store(dst, src);
	f849 0c08 	str.w	r0, [r9, #-8]
	f8c9 33f0 	str.w	r3, [r9, #1008]	@ 0x3f0
                self.w.bits =
	ea48 000e 	orr.w	r0, r8, lr
        intrinsics::volatile_load(src)
	f859 1c08 	ldr.w	r1, [r9, #-8]
	ea40 000b 	orr.w	r0, r0, fp
                    (self.w.bits & !(Self::MASK << { OF })) | ((value.into() & Self::MASK) << { OF });
	4391      	bics	r1, r2
                self.w.bits =
	4308      	orrs	r0, r1
        intrinsics::volatile_store(dst, src);
	f849 0c08 	str.w	r0, [r9, #-8]
        intrinsics::volatile_load(src)
	f859 0c08 	ldr.w	r0, [r9, #-8]
	2102      	movs	r1, #2
	f361 0001 	bfi	r0, r1, #0, #2
        intrinsics::volatile_store(dst, src);
	f849 0c08 	str.w	r0, [r9, #-8]
        intrinsics::volatile_load(src)
	f859 0c08 	ldr.w	r0, [r9, #-8]
        ($($t:ty)*) => ($(
            #[stable(feature = "rust1", since = "1.0.0")]
            #[rustc_const_unstable(feature = "const_cmp", issue = "143800")]
            impl const PartialEq for $t {
                #[inline]
                fn eq(&self, other: &Self) -> bool { *self == *other }
	f000 000c 	and.w	r0, r0, #12
        });

        // Select SYSCLK source
        if self.use_pll {
            rcc.cfgr.modify(|_, w| w.sw().pll());
            while !rcc.cfgr.read().sws().is_pll() {}
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
	f003 fc38 	bl	<__delay>
	f8d9 0020 	ldr.w	r0, [r9, #32]
                self.w.bits = (self.w.bits & !(1 << { OF })) | ((<$U>::from(value) & 1) << { OF });
	f040 0001 	orr.w	r0, r0, #1
        intrinsics::volatile_store(dst, src);
	f8c9 0020 	str.w	r0, [r9, #32]
	f003 fc36 	bl	<__dsb>
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f040 0001 	orr.w	r0, r0, #1
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f020 0001 	bic.w	r0, r0, #1
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 0002 	orr.w	r0, r0, #2
        intrinsics::volatile_store(dst, src);
	f8c9 0020 	str.w	r0, [r9, #32]
	f003 fc22 	bl	<__dsb>
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f040 0002 	orr.w	r0, r0, #2
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f020 0002 	bic.w	r0, r0, #2
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 0004 	orr.w	r0, r0, #4
        intrinsics::volatile_store(dst, src);
	f8c9 0020 	str.w	r0, [r9, #32]
	f003 fc0e 	bl	<__dsb>
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f040 0004 	orr.w	r0, r0, #4
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f020 0004 	bic.w	r0, r0, #4
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 0040 	orr.w	r0, r0, #64	@ 0x40
        intrinsics::volatile_store(dst, src);
	f8c9 0020 	str.w	r0, [r9, #32]
	f003 fbfa 	bl	<__dsb>
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f24e 0210 	movw	r2, #57360	@ 0xe010
	f2ce 0200 	movt	r2, #57344	@ 0xe000
	f64f 71fe 	movw	r1, #65534	@ 0xfffe
	f040 0040 	orr.w	r0, r0, #64	@ 0x40
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f2c0 01ff 	movt	r1, #255	@ 0xff
	f020 0040 	bic.w	r0, r0, #64	@ 0x40
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	68d0      	ldr	r0, [r2, #12]
    /// Returns `0` if the value is not known (e.g. because the clock can
    /// change dynamically).
    #[inline]
    pub fn get_ticks_per_10ms() -> u32 {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Self::PTR).calib.read() & SYST_COUNTER_MASK }
	4008      	ands	r0, r1
	f649 119a 	movw	r1, #39322	@ 0x999a
	f6c1 1199 	movt	r1, #6553	@ 0x1999
        }
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
	fba0 0101 	umull	r0, r1, r0, r1
        intrinsics::volatile_store(dst, src);
	6051      	str	r1, [r2, #4]
        intrinsics::volatile_load(src)
	6810      	ldr	r0, [r2, #0]
        unsafe { self.csr.modify(|v| v | SYST_CSR_ENABLE) }
	f040 0001 	orr.w	r0, r0, #1
        intrinsics::volatile_store(dst, src);
	6010      	str	r0, [r2, #0]
        intrinsics::volatile_load(src)
	6810      	ldr	r0, [r2, #0]
        unsafe { self.csr.modify(|v| v | SYST_CSR_TICKINT) }
	f040 0002 	orr.w	r0, r0, #2
        intrinsics::volatile_store(dst, src);
	6010      	str	r0, [r2, #0]
    defmt::info!("Enabling ethernet...");
	f240 002b 	movw	r0, #43	@ 0x2b
	f2c0 0000 	movt	r0, #0
	f7ff f997 	bl	<defmt::export::acquire_header_and_release>
	2500      	movs	r5, #0
	f640 0400 	movw	r4, #2048	@ 0x800
	f2c4 0502 	movt	r5, #16386	@ 0x4002
	f2c4 0402 	movt	r4, #16386	@ 0x4002
        intrinsics::volatile_load(src)
	68e8      	ldr	r0, [r5, #12]
	f640 0208 	movw	r2, #2056	@ 0x808
	f2c4 0202 	movt	r2, #16386	@ 0x4002
	f641 0a00 	movw	sl, #6144	@ 0x1800
    pub(super) fn mode<M: PinMode>(&mut self) {
        let offset = 2 * N;
        unsafe {
            (*Gpio::<P>::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::PUPDR << offset)));
	f020 000c 	bic.w	r0, r0, #12
        intrinsics::volatile_store(dst, src);
	60e8      	str	r0, [r5, #12]
        intrinsics::volatile_load(src)
	6828      	ldr	r0, [r5, #0]
	f2c4 0a02 	movt	sl, #16386	@ 0x4002
	230b      	movs	r3, #11
	2602      	movs	r6, #2
                    .modify(|r, w| w.bits(r.bits() & !(0b1 << N) | (otyper << N)));
            }

            (*Gpio::<P>::ptr())
                .moder
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::MODER << offset)));
	f020 000c 	bic.w	r0, r0, #12
        intrinsics::volatile_store(dst, src);
	6028      	str	r0, [r5, #0]
        intrinsics::volatile_load(src)
	68e8      	ldr	r0, [r5, #12]
}

impl Descriptor {
    pub const fn new() -> Self {
        Self {
            desc: Aligned([0; DESC_SIZE]),
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f04f 0800 	mov.w	r8, #0
	f50d 6b4a 	add.w	fp, sp, #3232	@ 0xca0
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::PUPDR << offset)));
	f420 4040 	bic.w	r0, r0, #49152	@ 0xc000
        intrinsics::volatile_store(dst, src);
	60e8      	str	r0, [r5, #12]
        intrinsics::volatile_load(src)
	6828      	ldr	r0, [r5, #0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::MODER << offset)));
	f420 4040 	bic.w	r0, r0, #49152	@ 0xc000
        intrinsics::volatile_store(dst, src);
	6028      	str	r0, [r5, #0]
	f64f 400c 	movw	r0, #64524	@ 0xfc0c
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
        intrinsics::volatile_load(src)
	5821      	ldr	r1, [r4, r0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::PUPDR << offset)));
	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
        intrinsics::volatile_store(dst, src);
	5021      	str	r1, [r4, r0]
        intrinsics::volatile_load(src)
	f8d5 1400 	ldr.w	r1, [r5, #1024]	@ 0x400
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::MODER << offset)));
	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
        intrinsics::volatile_store(dst, src);
	f8c5 1400 	str.w	r1, [r5, #1024]	@ 0x400
        intrinsics::volatile_load(src)
	6851      	ldr	r1, [r2, #4]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::PUPDR << offset)));
	f421 7140 	bic.w	r1, r1, #768	@ 0x300
        intrinsics::volatile_store(dst, src);
	6051      	str	r1, [r2, #4]
        intrinsics::volatile_load(src)
	6821      	ldr	r1, [r4, #0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::MODER << offset)));
	f421 7140 	bic.w	r1, r1, #768	@ 0x300
        intrinsics::volatile_store(dst, src);
	6021      	str	r1, [r4, #0]
        intrinsics::volatile_load(src)
	6851      	ldr	r1, [r2, #4]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::PUPDR << offset)));
	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
        intrinsics::volatile_store(dst, src);
	6051      	str	r1, [r2, #4]
        intrinsics::volatile_load(src)
	6821      	ldr	r1, [r4, #0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::MODER << offset)));
	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
        intrinsics::volatile_store(dst, src);
	6021      	str	r1, [r4, #0]
        intrinsics::volatile_load(src)
	f8da 100c 	ldr.w	r1, [sl, #12]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::PUPDR << offset)));
	f421 0140 	bic.w	r1, r1, #12582912	@ 0xc00000
        intrinsics::volatile_store(dst, src);
	f8ca 100c 	str.w	r1, [sl, #12]
        intrinsics::volatile_load(src)
	f8da 1000 	ldr.w	r1, [sl]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::MODER << offset)));
	f421 0140 	bic.w	r1, r1, #12582912	@ 0xc00000
        intrinsics::volatile_store(dst, src);
	f8ca 1000 	str.w	r1, [sl]
        intrinsics::volatile_load(src)
	f8da 100c 	ldr.w	r1, [sl, #12]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::PUPDR << offset)));
	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
        intrinsics::volatile_store(dst, src);
	f8ca 100c 	str.w	r1, [sl, #12]
        intrinsics::volatile_load(src)
	f8da 1000 	ldr.w	r1, [sl]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::MODER << offset)));
	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
        intrinsics::volatile_store(dst, src);
	f8ca 1000 	str.w	r1, [sl]
        intrinsics::volatile_load(src)
	6a29      	ldr	r1, [r5, #32]
                    w.bits((r.bits() & !(0b1111 << offset2)) | ((A as u32) << offset2))
	f363 210b 	bfi	r1, r3, #8, #4
        intrinsics::volatile_store(dst, src);
	6229      	str	r1, [r5, #32]
        intrinsics::volatile_load(src)
	6829      	ldr	r1, [r5, #0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
	f366 1105 	bfi	r1, r6, #4, #2
        intrinsics::volatile_store(dst, src);
	6029      	str	r1, [r5, #0]
        intrinsics::volatile_load(src)
	68a9      	ldr	r1, [r5, #8]
        let offset = 2 * { N };

        unsafe {
            (*Gpio::<P>::ptr())
                .ospeedr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset)))
	f041 0130 	orr.w	r1, r1, #48	@ 0x30
        intrinsics::volatile_store(dst, src);
	60a9      	str	r1, [r5, #8]
        intrinsics::volatile_load(src)
	6991      	ldr	r1, [r2, #24]
                    w.bits((r.bits() & !(0b1111 << offset2)) | ((A as u32) << offset2))
	f363 1107 	bfi	r1, r3, #4, #4
        intrinsics::volatile_store(dst, src);
	6191      	str	r1, [r2, #24]
        intrinsics::volatile_load(src)
	6821      	ldr	r1, [r4, #0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
	f366 0183 	bfi	r1, r6, #2, #2
        intrinsics::volatile_store(dst, src);
	6021      	str	r1, [r4, #0]
        intrinsics::volatile_load(src)
	6811      	ldr	r1, [r2, #0]
	f041 010c 	orr.w	r1, r1, #12
        intrinsics::volatile_store(dst, src);
	6011      	str	r1, [r2, #0]
	f64f 4118 	movw	r1, #64536	@ 0xfc18
	f44f 1200 	mov.w	r2, #2097152	@ 0x200000
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
	5062      	str	r2, [r4, r1]
        intrinsics::volatile_load(src)
	5821      	ldr	r1, [r4, r0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::PUPDR << offset)));
	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
        intrinsics::volatile_store(dst, src);
	5021      	str	r1, [r4, r0]
	f64f 4004 	movw	r0, #64516	@ 0xfc04
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
        intrinsics::volatile_load(src)
	5821      	ldr	r1, [r4, r0]
                    .modify(|r, w| w.bits(r.bits() & !(0b1 << N) | (otyper << N)));
	f021 0120 	bic.w	r1, r1, #32
        intrinsics::volatile_store(dst, src);
	5021      	str	r1, [r4, r0]
	2101      	movs	r1, #1
        intrinsics::volatile_load(src)
	f8d5 0400 	ldr.w	r0, [r5, #1024]	@ 0x400
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::MODER << offset)));
	f361 208b 	bfi	r0, r1, #10, #2
        intrinsics::volatile_store(dst, src);
	f8c5 0400 	str.w	r0, [r5, #1024]	@ 0x400
	f8ce 82ec 	str.w	r8, [lr, #748]	@ 0x2ec
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
macro_rules! array_impl_default {
    {$n:expr, $t:ident $($ts:ident)*} => {
        #[stable(since = "1.4.0", feature = "array_default")]
        impl<T> Default for [T; $n] where T: Default {
            fn default() -> [T; $n] {
                [$t::default(), $($ts::default()),*]
	f50b 60cd 	add.w	r0, fp, #1640	@ 0x668
    buffer: [u8; MTU],
}

impl Buffer {
    pub const fn new() -> Self {
        Self { buffer: [0; MTU] }
	f240 51f2 	movw	r1, #1522	@ 0x5f2
	f8ce 82e8 	str.w	r8, [lr, #744]	@ 0x2e8
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
}

impl RxDescriptor {
    /// Creates an zeroed RxDescriptor.
    pub const fn new() -> Self {
        Self {
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
	f003 fb13 	bl	<__aeabi_memclr8>
	ae08      	add	r6, sp, #32
    buffer: Buffer,
}

impl<T: RingDescriptor + Default> Default for RingEntry<T> {
    fn default() -> Self {
        RingEntry {
	f240 51f2 	movw	r1, #1522	@ 0x5f2
	f506 60d1 	add.w	r0, r6, #1672	@ 0x688
	9005      	str	r0, [sp, #20]
	f003 fb0b 	bl	<__aeabi_memclr8>
	4630      	mov	r0, r6
	2124      	movs	r1, #36	@ 0x24
	e9cd 8816 	strd	r8, r8, [sp, #88]	@ 0x58
	f8cd 8050 	str.w	r8, [sp, #80]	@ 0x50
	f8cd 8048 	str.w	r8, [sp, #72]	@ 0x48
	f003 fb01 	bl	<__aeabi_memclr8>
	f106 0048 	add.w	r0, r6, #72	@ 0x48
	f240 51f2 	movw	r1, #1522	@ 0x5f2
	9003      	str	r0, [sp, #12]
	f003 fafa 	bl	<__aeabi_memclr8>
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
	f003 fae6 	bl	<__aeabi_memclr8>
	f10b 0038 	add.w	r0, fp, #56	@ 0x38
	f240 51f2 	movw	r1, #1522	@ 0x5f2
	9004      	str	r0, [sp, #16]
	f003 fadf 	bl	<__aeabi_memclr8>
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
        intrinsics::volatile_load(src)
	6a28      	ldr	r0, [r5, #32]
                    w.bits((r.bits() & !(0b1111 << offset2)) | ((A as u32) << offset2))
	f361 1007 	bfi	r0, r1, #4, #4
        intrinsics::volatile_store(dst, src);
	6228      	str	r0, [r5, #32]
        intrinsics::volatile_load(src)
	6828      	ldr	r0, [r5, #0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
	f363 0083 	bfi	r0, r3, #2, #2
        intrinsics::volatile_store(dst, src);
	6028      	str	r0, [r5, #0]
        intrinsics::volatile_load(src)
	68a8      	ldr	r0, [r5, #8]
	f040 000c 	orr.w	r0, r0, #12
        intrinsics::volatile_store(dst, src);
	60a8      	str	r0, [r5, #8]
        intrinsics::volatile_load(src)
	6a28      	ldr	r0, [r5, #32]
                    w.bits((r.bits() & !(0b1111 << offset2)) | ((A as u32) << offset2))
	f361 701f 	bfi	r0, r1, #28, #4
        intrinsics::volatile_store(dst, src);
	6228      	str	r0, [r5, #32]
        intrinsics::volatile_load(src)
	6828      	ldr	r0, [r5, #0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
	f363 308f 	bfi	r0, r3, #14, #2
        intrinsics::volatile_store(dst, src);
	6028      	str	r0, [r5, #0]
        intrinsics::volatile_load(src)
	68a8      	ldr	r0, [r5, #8]
	f440 4040 	orr.w	r0, r0, #49152	@ 0xc000
        intrinsics::volatile_store(dst, src);
	60a8      	str	r0, [r5, #8]
        intrinsics::volatile_load(src)
	f8da 0024 	ldr.w	r0, [sl, #36]	@ 0x24
                    w.bits((r.bits() & !(0b1111 << offset2)) | ((A as u32) << offset2))
	f362 300f 	bfi	r0, r2, #12, #4
        intrinsics::volatile_store(dst, src);
	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
        intrinsics::volatile_load(src)
	f8da 0000 	ldr.w	r0, [sl]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
	f363 5097 	bfi	r0, r3, #22, #2
        intrinsics::volatile_store(dst, src);
	f8ca 0000 	str.w	r0, [sl]
        intrinsics::volatile_load(src)
	f8da 0008 	ldr.w	r0, [sl, #8]
	f440 0040 	orr.w	r0, r0, #12582912	@ 0xc00000
        intrinsics::volatile_store(dst, src);
	f8ca 0008 	str.w	r0, [sl, #8]
        intrinsics::volatile_load(src)
	f8da 0024 	ldr.w	r0, [sl, #36]	@ 0x24
                    w.bits((r.bits() & !(0b1111 << offset2)) | ((A as u32) << offset2))
	f362 5017 	bfi	r0, r2, #20, #4
        intrinsics::volatile_store(dst, src);
	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
        intrinsics::volatile_load(src)
	f8da 0000 	ldr.w	r0, [sl]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
	f363 609b 	bfi	r0, r3, #26, #2
        intrinsics::volatile_store(dst, src);
	f8ca 0000 	str.w	r0, [sl]
        intrinsics::volatile_load(src)
	f8da 0008 	ldr.w	r0, [sl, #8]
	f040 6040 	orr.w	r0, r0, #201326592	@ 0xc000000
        intrinsics::volatile_store(dst, src);
	f8ca 0008 	str.w	r0, [sl, #8]
	f64f 4024 	movw	r0, #64548	@ 0xfc24
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
        intrinsics::volatile_load(src)
	5821      	ldr	r1, [r4, r0]
                    w.bits((r.bits() & !(0b1111 << offset2)) | ((A as u32) << offset2))
	f362 5117 	bfi	r1, r2, #20, #4
        intrinsics::volatile_store(dst, src);
	5021      	str	r1, [r4, r0]
        intrinsics::volatile_load(src)
	f8d5 0400 	ldr.w	r0, [r5, #1024]	@ 0x400
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
	f363 609b 	bfi	r0, r3, #26, #2
        intrinsics::volatile_store(dst, src);
	f8c5 0400 	str.w	r0, [r5, #1024]	@ 0x400
	f64f 4008 	movw	r0, #64520	@ 0xfc08
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
        intrinsics::volatile_load(src)
	5821      	ldr	r1, [r4, r0]
	f041 6140 	orr.w	r1, r1, #201326592	@ 0xc000000
        intrinsics::volatile_store(dst, src);
	5021      	str	r1, [r4, r0]
	f640 0108 	movw	r1, #2056	@ 0x808
	f2c4 0102 	movt	r1, #16386	@ 0x4002
        intrinsics::volatile_load(src)
	6988      	ldr	r0, [r1, #24]
                    w.bits((r.bits() & !(0b1111 << offset2)) | ((A as u32) << offset2))
	f362 4013 	bfi	r0, r2, #16, #4
        intrinsics::volatile_store(dst, src);
	6188      	str	r0, [r1, #24]
        intrinsics::volatile_load(src)
	6820      	ldr	r0, [r4, #0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
	f363 2009 	bfi	r0, r3, #8, #2
        intrinsics::volatile_store(dst, src);
	6020      	str	r0, [r4, #0]
        intrinsics::volatile_load(src)
	6808      	ldr	r0, [r1, #0]
	f440 7040 	orr.w	r0, r0, #768	@ 0x300
        intrinsics::volatile_store(dst, src);
	6008      	str	r0, [r1, #0]
        intrinsics::volatile_load(src)
	6988      	ldr	r0, [r1, #24]
                    w.bits((r.bits() & !(0b1111 << offset2)) | ((A as u32) << offset2))
	f362 5017 	bfi	r0, r2, #20, #4
        intrinsics::volatile_store(dst, src);
	6188      	str	r0, [r1, #24]
        intrinsics::volatile_load(src)
	6820      	ldr	r0, [r4, #0]
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
	f363 208b 	bfi	r0, r3, #10, #2
        intrinsics::volatile_store(dst, src);
	6020      	str	r0, [r4, #0]
        intrinsics::volatile_load(src)
	6808      	ldr	r0, [r1, #0]
	f440 6040 	orr.w	r0, r0, #3072	@ 0xc00
        intrinsics::volatile_store(dst, src);
	6008      	str	r0, [r1, #0]
	f003 fa57 	bl	<__primask_r>
	4680      	mov	r8, r0
	f003 fa47 	bl	<__cpsid>
        intrinsics::volatile_load(src)
	f8d9 0034 	ldr.w	r0, [r9, #52]	@ 0x34
	ea5f 71c8 	movs.w	r1, r8, lsl #31
	f440 4080 	orr.w	r0, r0, #16384	@ 0x4000
        intrinsics::volatile_store(dst, src);
	f8c9 0034 	str.w	r0, [r9, #52]	@ 0x34
        intrinsics::volatile_load(src)
	f8d9 0020 	ldr.w	r0, [r9, #32]
	d126      	bne.n	<ip::__cortex_m_rt_main+0xa0c>
        let rcc = &*RCC::ptr();
        let syscfg = &*SYSCFG::ptr();
        // enable syscfg clock
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

        if rcc.ahb1enr.read().ethmacen().bit_is_set() {
	0180      	lsls	r0, r0, #6
	f8dd 801c 	ldr.w	r8, [sp, #28]
	bf42      	ittt	mi
	f8d9 0020 	ldrmi.w	r0, [r9, #32]
	f020 7000 	bicmi.w	r0, r0, #33554432	@ 0x2000000
        intrinsics::volatile_store(dst, src);
	f8c9 0020 	strmi.w	r0, [r9, #32]
	f643 0004 	movw	r0, #14340	@ 0x3804
	f2c4 0001 	movt	r0, #16385	@ 0x4001
        intrinsics::volatile_load(src)
	6801      	ldr	r1, [r0, #0]
	f441 0100 	orr.w	r1, r1, #8388608	@ 0x800000
        intrinsics::volatile_store(dst, src);
	6001      	str	r1, [r0, #0]
        intrinsics::volatile_load(src)
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 6060 	orr.w	r0, r0, #234881024	@ 0xe000000
        intrinsics::volatile_store(dst, src);
	f8c9 0020 	str.w	r0, [r9, #32]
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f040 7000 	orr.w	r0, r0, #33554432	@ 0x2000000
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f020 7000 	bic.w	r0, r0, #33554432	@ 0x2000000
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
	f003 fa18 	bl	<__cpsie>
	e023      	b.n	<ip::__cortex_m_rt_main+0xa54>
	0180      	lsls	r0, r0, #6
	f8dd 801c 	ldr.w	r8, [sp, #28]
        intrinsics::volatile_load(src)
	bf42      	ittt	mi
	f8d9 0020 	ldrmi.w	r0, [r9, #32]
	f020 7000 	bicmi.w	r0, r0, #33554432	@ 0x2000000
        intrinsics::volatile_store(dst, src);
	f8c9 0020 	strmi.w	r0, [r9, #32]
	f643 0004 	movw	r0, #14340	@ 0x3804
	f2c4 0001 	movt	r0, #16385	@ 0x4001
        intrinsics::volatile_load(src)
	6801      	ldr	r1, [r0, #0]
	f441 0100 	orr.w	r1, r1, #8388608	@ 0x800000
        intrinsics::volatile_store(dst, src);
	6001      	str	r1, [r0, #0]
        intrinsics::volatile_load(src)
	f8d9 0020 	ldr.w	r0, [r9, #32]
	f040 6060 	orr.w	r0, r0, #234881024	@ 0xe000000
        intrinsics::volatile_store(dst, src);
	f8c9 0020 	str.w	r0, [r9, #32]
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f040 7000 	orr.w	r0, r0, #33554432	@ 0x2000000
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
        intrinsics::volatile_load(src)
	f8d9 0000 	ldr.w	r0, [r9]
	f020 7000 	bic.w	r0, r0, #33554432	@ 0x2000000
        intrinsics::volatile_store(dst, src);
	f8c9 0000 	str.w	r0, [r9]
	f249 0418 	movw	r4, #36888	@ 0x9018
	e9dd 5c05 	ldrd	r5, ip, [sp, #20]
	f2c4 0402 	movt	r4, #16386	@ 0x4002
	e9dd 6e03 	ldrd	r6, lr, [sp, #12]
        intrinsics::volatile_load(src)
	f854 0c18 	ldr.w	r0, [r4, #-24]
	f50d 6a4a 	add.w	sl, sp, #3232	@ 0xca0
	f10d 0b20 	add.w	fp, sp, #32
	f040 0001 	orr.w	r0, r0, #1
        intrinsics::volatile_store(dst, src);
	f844 0c18 	str.w	r0, [r4, #-24]
	a806      	add	r0, sp, #24
	f500 5997 	add.w	r9, r0, #4832	@ 0x12e0
        intrinsics::volatile_load(src)
	f854 0c18 	ldr.w	r0, [r4, #-24]
    ) -> Self {
        // reset DMA bus mode register
        eth_dma.dmabmr.modify(|_, w| w.sr().set_bit());

        // Wait until done
        while eth_dma.dmabmr.read().sr().bit_is_set() {}
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
    {
        // SAFETY: the caller must uphold the safety contract for `offset`.
        // Additionally safety contract of `offset` guarantees that the resulting pointer is
        // pointing to an allocation, there can't be an allocation at null, thus it's safe to
        // construct `NonNull`.
        unsafe { NonNull { pointer: intrinsics::offset(self.as_ptr(), count) } }
	f50b 63c8 	add.w	r3, fp, #1600	@ 0x640
	f040 0084 	orr.w	r0, r0, #132	@ 0x84
        intrinsics::volatile_store(dst, src);
	6020      	str	r0, [r4, #0]
        intrinsics::volatile_load(src)
	f854 0c18 	ldr.w	r0, [r4, #-24]
                    (self.w.bits & !(Self::MASK << { OF })) | ((value.into() & Self::MASK) << { OF });
	4008      	ands	r0, r1
	f246 0180 	movw	r1, #24704	@ 0x6080
	f2c0 21c1 	movt	r1, #705	@ 0x2c1
                self.w.bits = (self.w.bits & !(1 << { OF })) | ((<$U>::from(value) & 1) << { OF });
	4308      	orrs	r0, r1
	f44f 4180 	mov.w	r1, #16384	@ 0x4000
        intrinsics::volatile_store(dst, src);
	f844 0c18 	str.w	r0, [r4, #-24]
	9109      	str	r1, [sp, #36]	@ 0x24
	960a      	str	r6, [sp, #40]	@ 0x28
        intrinsics::volatile_load(src)
	9809      	ldr	r0, [sp, #36]	@ 0x24
            self.desc.write(2, buffer_addr);
        }
    }

    fn set_buffer1(&mut self, buffer: *const u8, len: usize) {
        self.buffer1 = Some(buffer as u32);
	9611      	str	r6, [sp, #68]	@ 0x44
        self.write_buffer1();
        unsafe {
            self.desc.modify(1, |w| {
                (w & !RXDESC_1_RBS_MASK) | ((len as u32) << RXDESC_1_RBS_SHIFT)
	f362 000b 	bfi	r0, r2, #0, #12
        intrinsics::volatile_store(dst, src);
	9009      	str	r0, [sp, #36]	@ 0x24
	2001      	movs	r0, #1
	930b      	str	r3, [sp, #44]	@ 0x2c
	960a      	str	r6, [sp, #40]	@ 0x28
        self.buffer1 = Some(buffer as u32);
	9010      	str	r0, [sp, #64]	@ 0x40
        }
    }

    // points to next descriptor (RCH)
    fn set_buffer2(&mut self, buffer: *const u8) {
        self.next_descriptor = Some(buffer as u32);
	e9cd 0312 	strd	r0, r3, [sp, #72]	@ 0x48
	930b      	str	r3, [sp, #44]	@ 0x2c
	f04f 4300 	mov.w	r3, #2147483648	@ 0x80000000
pub fn fence(order: Ordering) {
    // SAFETY: using an atomic fence is safe.
    unsafe {
        match order {
            Acquire => intrinsics::atomic_fence::<{ AO::Acquire }>(),
            Release => intrinsics::atomic_fence::<{ AO::Release }>(),
	f3bf 8f5f 	dmb	sy
	9308      	str	r3, [sp, #32]
            AcqRel => intrinsics::atomic_fence::<{ AO::AcqRel }>(),
            SeqCst => intrinsics::atomic_fence::<{ AO::SeqCst }>(),
	f3bf 8f5f 	dmb	sy
	f8cd 1664 	str.w	r1, [sp, #1636]	@ 0x664
	f8cd 5668 	str.w	r5, [sp, #1640]	@ 0x668
        intrinsics::volatile_load(src)
	f8dd 1664 	ldr.w	r1, [sp, #1636]	@ 0x664
                (w & !RXDESC_1_RBS_MASK) | ((len as u32) << RXDESC_1_RBS_SHIFT)
	f362 010b 	bfi	r1, r2, #0, #12
        intrinsics::volatile_store(dst, src);
	f8cd 1664 	str.w	r1, [sp, #1636]	@ 0x664
	2100      	movs	r1, #0
	f8cd 166c 	str.w	r1, [sp, #1644]	@ 0x66c
        intrinsics::volatile_load(src)
	f8dd 2664 	ldr.w	r2, [sp, #1636]	@ 0x664
        self.write_buffer2();
    }

    fn set_end_of_ring(&mut self) {
        unsafe {
            self.desc.modify(1, |w| w | RXDESC_1_RER);
	f442 4200 	orr.w	r2, r2, #32768	@ 0x8000
        intrinsics::volatile_store(dst, src);
	f8cd 2664 	str.w	r2, [sp, #1636]	@ 0x664
	f8cd 5668 	str.w	r5, [sp, #1640]	@ 0x668
        self.buffer1 = Some(buffer as u32);
	f8cd 5684 	str.w	r5, [sp, #1668]	@ 0x684
	f248 7500 	movw	r5, #34560	@ 0x8700
	f8cd 0680 	str.w	r0, [sp, #1664]	@ 0x680
	f2c4 0502 	movt	r5, #16386	@ 0x4002
        self.next_descriptor = Some(buffer as u32);
	f8cd 168c 	str.w	r1, [sp, #1676]	@ 0x68c
	f8cd 0688 	str.w	r0, [sp, #1672]	@ 0x688
	f8cd 166c 	str.w	r1, [sp, #1644]	@ 0x66c
            Release => intrinsics::atomic_fence::<{ AO::Release }>(),
	f3bf 8f5f 	dmb	sy
	f8cd 3660 	str.w	r3, [sp, #1632]	@ 0x660
            SeqCst => intrinsics::atomic_fence::<{ AO::SeqCst }>(),
	f3bf 8f5f 	dmb	sy
	f844 bc0c 	str.w	fp, [r4, #-12]
        intrinsics::volatile_load(src)
	6822      	ldr	r2, [r4, #0]
	f042 0202 	orr.w	r2, r2, #2
        intrinsics::volatile_store(dst, src);
	6022      	str	r2, [r4, #0]
	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
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
            self.is_last = true;
            0
        };

        self.buffer1 = buffer as u32;
        self.next_descriptor = next_desc_addr;
	f8cd 2ccc 	str.w	r2, [sp, #3276]	@ 0xccc
	f50d 5280 	add.w	r2, sp, #4096	@ 0x1000
	f8c2 12e8 	str.w	r1, [r2, #744]	@ 0x2e8
        self.buffer1 = buffer as u32;
	f8cd ecc8 	str.w	lr, [sp, #3272]	@ 0xcc8
            self.is_last = true;
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f88e 0300 	strb.w	r0, [lr, #768]	@ 0x300
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f8ce 12ec 	str.w	r1, [lr, #748]	@ 0x2ec
        self.next_descriptor = next_desc_addr;
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
        self.buffer1 = buffer as u32;
	f8c9 c000 	str.w	ip, [r9]
        self.next_descriptor = next_desc_addr;
	f8ce 12fc 	str.w	r1, [lr, #764]	@ 0x2fc
	f44f 7100 	mov.w	r1, #512	@ 0x200
	f844 ac08 	str.w	sl, [r4, #-8]
            Release => intrinsics::atomic_fence::<{ AO::Release }>(),
	f3bf 8f5f 	dmb	sy
        intrinsics::volatile_load(src)
	6820      	ldr	r0, [r4, #0]
	f440 5000 	orr.w	r0, r0, #8192	@ 0x2000
        intrinsics::volatile_store(dst, src);
	6020      	str	r0, [r4, #0]
	f64f 103c 	movw	r0, #63804	@ 0xf93c
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
	5029      	str	r1, [r5, r0]

impl EthernetPTP {
    // Calculate the `addend` required for running `global_time` at
    // the correct rate
    const fn calculate_regs(hclk: u32) -> (Subseconds, u32) {
        let half_hclk = hclk / 2;
	ea5f 0058 	movs.w	r0, r8, lsr #1
    pub(crate) const fn hertz(&self) -> u32 {
        SUBSECONDS_PER_SECOND as u32 / self.0
    }

    pub(crate) const fn nearest_increment(input_clk_hz: u32) -> Subseconds {
        let hclk_half_subs = (SUBSECONDS_PER_SECOND as u32 + (input_clk_hz / 2)) / input_clk_hz;
	d105      	bne.n	<ip::__cortex_m_rt_main+0xc0e>
	f24b 00e0 	movw	r0, #45280	@ 0xb0e0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fe fc76 	bl	<core::panicking::panic_const::panic_const_div_by_zero>
	f06f 4100 	mvn.w	r1, #2147483648	@ 0x80000000
	eb01 0298 	add.w	r2, r1, r8, lsr #2
	fbb2 f2f0 	udiv	r2, r2, r0
	f248 7604 	movw	r6, #34564	@ 0x8704
        SUBSECONDS_PER_SECOND as u32 / self.0
	fbb1 f0f2 	udiv	r0, r1, r2
	f04f 31ff 	mov.w	r1, #4294967295	@ 0xffffffff
        let half_rate_subsec_increment_hz = stssi.hertz();

        // Calculate the `addend` required for running `global_time` at
        // the correct rate, given that we increment `global_time` by `stssi` every
        // time `accumulator` overflows.
        let tsa = ((half_rate_subsec_increment_hz as u64 * u32::MAX as u64) / hclk as u64) as u32;
	fba0 0101 	umull	r0, r1, r0, r1
	f242 1303 	movw	r3, #8451	@ 0x2103
            // Rustdocs on the impl block show a "[+] show undocumented items" toggle.
            // Rustdocs on functions do not.
            #[doc = $doc]
            #[inline(always)]
            fn from(small: $Small) -> Self {
                small as Self
	b2d2      	uxtb	r2, r2
	f2c4 0602 	movt	r6, #16386	@ 0x4002
	602b      	str	r3, [r5, #0]
	2300      	movs	r3, #0
	6032      	str	r2, [r6, #0]
	4642      	mov	r2, r8
	f003 ff4a 	bl	<__aeabi_uldivmod>
	6170      	str	r0, [r6, #20]
        intrinsics::volatile_load(src)
	6828      	ldr	r0, [r5, #0]
            while ptp.ptptscr.read().tsaru().bit_is_set() {}
        }

        #[cfg(not(feature = "stm32f1xx-hal"))]
        {
            while ptp.ptptscr.read().ttsaru().bit_is_set() {}
	0680      	lsls	r0, r0, #26
	d509      	bpl.n	<ip::__cortex_m_rt_main+0xc5c>
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	bf44      	itt	mi
	6828      	ldrmi	r0, [r5, #0]
	ea5f 6080 	movsmi.w	r0, r0, lsl #26
	d502      	bpl.n	<ip::__cortex_m_rt_main+0xc5c>
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	d4f2      	bmi.n	<ip::__cortex_m_rt_main+0xc42>
	6828      	ldr	r0, [r5, #0]
	f040 0020 	orr.w	r0, r0, #32
        intrinsics::volatile_store(dst, src);
	6028      	str	r0, [r5, #0]
        intrinsics::volatile_load(src)
	6828      	ldr	r0, [r5, #0]
            ptp.ptptscr.modify(|_, w| w.ttsaru().set_bit());
            while ptp.ptptscr.read().ttsaru().bit_is_set() {}
	0680      	lsls	r0, r0, #26
	d509      	bpl.n	<ip::__cortex_m_rt_main+0xc7e>
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	bf44      	itt	mi
	6828      	ldrmi	r0, [r5, #0]
	ea5f 6080 	movsmi.w	r0, r0, lsl #26
	d502      	bpl.n	<ip::__cortex_m_rt_main+0xc7e>
	6828      	ldr	r0, [r5, #0]
	0680      	lsls	r0, r0, #26
	d4f2      	bmi.n	<ip::__cortex_m_rt_main+0xc64>
	2000      	movs	r0, #0
        intrinsics::volatile_store(dst, src);
	60f0      	str	r0, [r6, #12]
	6130      	str	r0, [r6, #16]
        intrinsics::volatile_load(src)
	6828      	ldr	r0, [r5, #0]

        ptp.ptptshur.write(|w| unsafe { w.bits(seconds) });
        ptp.ptptslur.write(|w| unsafe { w.bits(subseconds) });

        // Initialise timestamp
        while ptp.ptptscr.read().tssti().bit_is_set() {}
	0740      	lsls	r0, r0, #29
	d509      	bpl.n	<ip::__cortex_m_rt_main+0xc9e>
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	bf44      	itt	mi
	6828      	ldrmi	r0, [r5, #0]
	ea5f 7040 	movsmi.w	r0, r0, lsl #29
	d502      	bpl.n	<ip::__cortex_m_rt_main+0xc9e>
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	d4f2      	bmi.n	<ip::__cortex_m_rt_main+0xc84>
	6828      	ldr	r0, [r5, #0]
	f040 0004 	orr.w	r0, r0, #4
        intrinsics::volatile_store(dst, src);
	6028      	str	r0, [r5, #0]
        intrinsics::volatile_load(src)
	6828      	ldr	r0, [r5, #0]
        ptp.ptptscr.modify(|_, w| w.tssti().set_bit());
        while ptp.ptptscr.read().tssti().bit_is_set() {}
	0740      	lsls	r0, r0, #29
	d509      	bpl.n	<ip::__cortex_m_rt_main+0xcc0>
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	bf44      	itt	mi
	6828      	ldrmi	r0, [r5, #0]
	ea5f 7040 	movsmi.w	r0, r0, lsl #29
	d502      	bpl.n	<ip::__cortex_m_rt_main+0xcc0>
	6828      	ldr	r0, [r5, #0]
	0740      	lsls	r0, r0, #29
	d4f2      	bmi.n	<ip::__cortex_m_rt_main+0xca6>
	f647 0040 	movw	r0, #30784	@ 0x7840
	f2c0 107d 	movt	r0, #381	@ 0x17d
        _dma: &EthernetDMA,
    ) -> Result<Self, WrongClock> {
        let clock_frequency = clocks.hclk().to_Hz();

        let clock_range = match clock_frequency {
            0..=24_999_999 => return Err(WrongClock),
	4580      	cmp	r8, r0
	d260      	bcs.n	<ip::__cortex_m_rt_main+0xd8e>
	6820      	ldr	r0, [r4, #0]
	f420 5000 	bic.w	r0, r0, #8192	@ 0x2000
        intrinsics::volatile_store(dst, src);
	6020      	str	r0, [r4, #0]
        intrinsics::volatile_load(src)
	f854 0c04 	ldr.w	r0, [r4, #-4]
        RPS_R::new(((self.bits >> 17) & 7) as u8)
    }
    #[doc = "Bits 20:22 - Transmit process state"]
    #[inline(always)]
    pub fn tps(&self) -> TPS_R {
        TPS_R::new(((self.bits >> 20) & 7) as u8)
	f3c0 5002 	ubfx	r0, r0, #20, #3

    pub(crate) fn running_state(&self) -> RunningState {
        // SAFETY: we only perform an atomic read of `dmasr`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        match eth_dma.dmasr.read().tps().bits() {
	3801      	subs	r0, #1
	2802      	cmp	r0, #2
	d814      	bhi.n	<ip::__cortex_m_rt_main+0xd0c>
	f854 0c04 	ldr.w	r0, [r4, #-4]
	f3c0 5002 	ubfx	r0, r0, #20, #3
	3801      	subs	r0, #1
	2802      	cmp	r0, #2
	bf9f      	itttt	ls
	f854 0c04 	ldrls.w	r0, [r4, #-4]
	f3c0 5002 	ubfxls	r0, r0, #20, #3
	3801      	subls	r0, #1
	2802      	cmpls	r0, #2
	d806      	bhi.n	<ip::__cortex_m_rt_main+0xd0c>
	f854 0c04 	ldr.w	r0, [r4, #-4]
	f3c0 5002 	ubfx	r0, r0, #20, #3
	3801      	subs	r0, #1
	2803      	cmp	r0, #3
	d3e3      	bcc.n	<ip::__cortex_m_rt_main+0xcd4>
	6820      	ldr	r0, [r4, #0]
	f020 0002 	bic.w	r0, r0, #2
        intrinsics::volatile_store(dst, src);
	6020      	str	r0, [r4, #0]
	2001      	movs	r0, #1
        intrinsics::volatile_load(src)
	f854 1c04 	ldr.w	r1, [r4, #-4]
        RPS_R::new(((self.bits >> 17) & 7) as u8)
	f3c1 4142 	ubfx	r1, r1, #17, #3
    pub fn running_state(&self) -> RunningState {
        // SAFETY: we only perform an atomic read of `dmasr`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };
        match eth_dma.dmasr.read().rps().bits() {
            //  Reset or Stop Receive Command issued
            0b000 => RunningState::Stopped,
	2907      	cmp	r1, #7
	d825      	bhi.n	<ip::__cortex_m_rt_main+0xd6e>
	fa00 f101 	lsl.w	r1, r0, r1
	f011 0faa 	tst.w	r1, #170	@ 0xaa
	d020      	beq.n	<ip::__cortex_m_rt_main+0xd6e>
	f854 1c04 	ldr.w	r1, [r4, #-4]
	f3c1 4142 	ubfx	r1, r1, #17, #3
	2907      	cmp	r1, #7
	d81a      	bhi.n	<ip::__cortex_m_rt_main+0xd6e>
	fa00 f101 	lsl.w	r1, r0, r1
	f011 0faa 	tst.w	r1, #170	@ 0xaa
	d015      	beq.n	<ip::__cortex_m_rt_main+0xd6e>
	f854 1c04 	ldr.w	r1, [r4, #-4]
	f3c1 4142 	ubfx	r1, r1, #17, #3
	2907      	cmp	r1, #7
	d80f      	bhi.n	<ip::__cortex_m_rt_main+0xd6e>
	fa00 f101 	lsl.w	r1, r0, r1
	f011 0faa 	tst.w	r1, #170	@ 0xaa
	d00a      	beq.n	<ip::__cortex_m_rt_main+0xd6e>
	f854 1c04 	ldr.w	r1, [r4, #-4]
	f3c1 4142 	ubfx	r1, r1, #17, #3
	2907      	cmp	r1, #7
	d804      	bhi.n	<ip::__cortex_m_rt_main+0xd6e>
	fa00 f101 	lsl.w	r1, r0, r1
	f011 0faa 	tst.w	r1, #170	@ 0xaa
	d1d3      	bne.n	<ip::__cortex_m_rt_main+0xd16>
    where
        E: fmt::Debug,
    {
        match self {
            Ok(t) => t,
            Err(e) => unwrap_failed("called `Result::unwrap()` on an `Err` value", &e),
	f64a 1008 	movw	r0, #43272	@ 0xa908
	f64a 02f8 	movw	r2, #43256	@ 0xa8f8
	f64a 1334 	movw	r3, #43316	@ 0xa934
	f1a7 0111 	sub.w	r1, r7, #17
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f7fe fca7 	bl	<core::result::unwrap_failed>
	f640 60c0 	movw	r0, #3776	@ 0xec0
	f2c0 2016 	movt	r0, #534	@ 0x216
            25_000_000..=34_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_16,
	4580      	cmp	r8, r0
	d201      	bcs.n	<ip::__cortex_m_rt_main+0xd9e>
	2008      	movs	r0, #8
	e022      	b.n	<ip::__cortex_m_rt_main+0xde4>
	f248 7000 	movw	r0, #34560	@ 0x8700
	f2c0 3093 	movt	r0, #915	@ 0x393
            35_000_000..=59_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_26,
	4580      	cmp	r8, r0
	d201      	bcs.n	<ip::__cortex_m_rt_main+0xdae>
	200c      	movs	r0, #12
	e01a      	b.n	<ip::__cortex_m_rt_main+0xde4>
	f24e 1000 	movw	r0, #57600	@ 0xe100
	f2c0 50f5 	movt	r0, #1525	@ 0x5f5
            60_000_000..=99_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_42,
	4580      	cmp	r8, r0
	d20c      	bcs.n	<ip::__cortex_m_rt_main+0xdd4>
	2000      	movs	r0, #0
	e012      	b.n	<ip::__cortex_m_rt_main+0xde4>
            40..=95 => (64.0, 0b1100),
	f1a0 0328 	sub.w	r3, r0, #40	@ 0x28
	2b38      	cmp	r3, #56	@ 0x38
	f080 8341 	bcs.w	<ip::__cortex_m_rt_main+0x144a>
	ed9f 2acf 	vldr	s4, [pc, #828]	@ <ip::__cortex_m_rt_main+0x1108>
	f04f 0ec0 	mov.w	lr, #192	@ 0xc0
	f7ff ba0b 	b.w	<ip::__cortex_m_rt_main+0x1ea>
	f24d 1180 	movw	r1, #53632	@ 0xd180
	2010      	movs	r0, #16
	f6c0 01f0 	movt	r1, #2288	@ 0x8f0
            100_000_000..=149_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_62,
	4588      	cmp	r8, r1
	bf38      	it	cc
	2004      	movcc	r0, #4
	f64f 1110 	movw	r1, #63760	@ 0xf910
            },
            fragmenter: Fragmenter::new(),
            inner: InterfaceInner {
                now,
                caps,
                hardware_addr: config.hardware_addr,
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
	f50d 56c9 	add.w	r6, sp, #6432	@ 0x1920
	586a      	ldr	r2, [r5, r1]
                    (self.w.bits & !(Self::MASK << { OF })) | ((value.into() & Self::MASK) << { OF });
	f022 021c 	bic.w	r2, r2, #28
                self.w.bits =
	4310      	orrs	r0, r2
        intrinsics::volatile_store(dst, src);
	5068      	str	r0, [r5, r1]
	f64f 1000 	movw	r0, #63744	@ 0xf900
	f6cf 70ff 	movt	r0, #65535	@ 0xffff
	f644 628c 	movw	r2, #20108	@ 0x4e8c
        intrinsics::volatile_load(src)
	5829      	ldr	r1, [r5, r0]
	f2c0 2200 	movt	r2, #512	@ 0x200
                self.w.bits = (self.w.bits & !(1 << { OF })) | ((<$U>::from(value) & 1) << { OF });
	4311      	orrs	r1, r2
        intrinsics::volatile_store(dst, src);
	5029      	str	r1, [r5, r0]
	f64f 1104 	movw	r1, #63748	@ 0xf904
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
        intrinsics::volatile_load(src)
	586a      	ldr	r2, [r5, r1]
	f042 4200 	orr.w	r2, r2, #2147483648	@ 0x80000000
	f042 0201 	orr.w	r2, r2, #1
        intrinsics::volatile_store(dst, src);
	506a      	str	r2, [r5, r1]
	f64f 1118 	movw	r1, #63768	@ 0xf918
	f6cf 71ff 	movt	r1, #65535	@ 0xffff
        intrinsics::volatile_load(src)
	586a      	ldr	r2, [r5, r1]
                self.w.bits =
	f2c0 1200 	movt	r2, #256	@ 0x100
        intrinsics::volatile_store(dst, src);
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
        intrinsics::volatile_load(src)
	586a      	ldr	r2, [r5, r1]

        // Fix incorrect TGFM bit position until https://github.com/stm32-rs/stm32-rs/pull/689
        // is released and used by HALs.
        eth_mmc
            .mmctimr
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 21)) });
	f442 1200 	orr.w	r2, r2, #2097152	@ 0x200000
        intrinsics::volatile_store(dst, src);
	506a      	str	r2, [r5, r1]
	f04f 3203 	mov.w	r2, #50529027	@ 0x3030303
        intrinsics::volatile_load(src)
	5829      	ldr	r1, [r5, r0]
                self.w.bits = (self.w.bits & !(1 << { OF })) | ((<$U>::from(value) & 1) << { OF });
	f441 4190 	orr.w	r1, r1, #18432	@ 0x4800
        intrinsics::volatile_store(dst, src);
	5029      	str	r1, [r5, r0]
	2500      	movs	r5, #0
	2002      	movs	r0, #2
            Ok(t) => t,
	f509 61c2 	add.w	r1, r9, #1552	@ 0x610
	e881 0421 	stmia.w	r1, {r0, r5, sl}
	f24e 0110 	movw	r1, #57360	@ 0xe010
	f2ce 0100 	movt	r1, #57344	@ 0xe000
	f8c9 5624 	str.w	r5, [r9, #1572]	@ 0x624
	f8c9 5620 	str.w	r5, [r9, #1568]	@ 0x620
	f8c9 061c 	str.w	r0, [r9, #1564]	@ 0x61c
	f8c9 b60c 	str.w	fp, [r9, #1548]	@ 0x60c
        intrinsics::volatile_load(src)
	6860      	ldr	r0, [r4, #4]
	f440 3080 	orr.w	r0, r0, #65536	@ 0x10000
	f040 0041 	orr.w	r0, r0, #65	@ 0x41
        intrinsics::volatile_store(dst, src);
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
        Interface {
	f8c9 0640 	str.w	r0, [r9, #1600]	@ 0x640
                hardware_addr: config.hardware_addr,
	f8c9 1790 	str.w	r1, [r9, #1936]	@ 0x790
	f240 51f2 	movw	r1, #1522	@ 0x5f2
        Interface {
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
    pub unsafe fn push_unchecked(&mut self, item: T) {
        // NOTE(ptr::write) the memory slot that we are about to write to is uninitialized. We
        // use `ptr::write` to avoid running `T`'s destructor on the uninitialized memory
        debug_assert!(!self.is_full());

        *self.buffer.get_unchecked_mut(self.len) = MaybeUninit::new(item);
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	2018      	movs	r0, #24

        self.len += 1;
	f8c9 4718 	str.w	r4, [r9, #1816]	@ 0x718
        *self.buffer.get_unchecked_mut(self.len) = MaybeUninit::new(item);
	f88e 0a18 	strb.w	r0, [lr, #2584]	@ 0xa18
	f64f 70fe 	movw	r0, #65534	@ 0xfffe
	f2c0 00ff 	movt	r0, #255	@ 0xff
	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
	300c      	adds	r0, #12
	f8c9 071c 	str.w	r0, [r9, #1820]	@ 0x71c
	f000 01f0 	and.w	r1, r0, #240	@ 0xf0
	f88e 5a8e 	strb.w	r5, [lr, #2702]	@ 0xa8e
        Address::from_bits(u32::from_be_bytes(bytes))
    }

    /// Query whether the address is an unicast address.
    fn x_is_unicast(&self) -> bool {
        !(self.is_broadcast() || self.is_multicast() || self.is_unspecified())
	29e0      	cmp	r1, #224	@ 0xe0
            if len > self.len {
                return;
            }
            let remaining_len = self.len - len;
            let s = ptr::slice_from_raw_parts_mut(self.as_mut_ptr().add(len), remaining_len);
            self.len = len;
	f8c9 5708 	str.w	r5, [r9, #1800]	@ 0x708
	bf18      	it	ne
	f110 0101 	addsne.w	r1, r0, #1
	d11b      	bne.n	<ip::__cortex_m_rt_main+0xf70>

    /// Return the IP address of this CIDR block.
    pub const fn address(&self) -> Address {
        match *self {
            #[cfg(feature = "proto-ipv4")]
            Cidr::Ipv4(cidr) => Address::Ipv4(cidr.address()),
	f8c9 08f0 	str.w	r0, [r9, #2288]	@ 0x8f0
    }

    fn check_ip_addrs(addrs: &[IpCidr]) {
        for cidr in addrs {
            if !cidr.address().is_unicast() && !cidr.address().is_unspecified() {
                panic!("IP address {} is not unicast", cidr.address())
	f647 405d 	movw	r0, #31837	@ 0x7c5d
	f10d 0e08 	add.w	lr, sp, #8
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f50e 51df 	add.w	r1, lr, #7136	@ 0x1be0
	f10d 0e08 	add.w	lr, sp, #8
	f64a 52b4 	movw	r2, #44468	@ 0xadb4
	f8c9 1af0 	str.w	r1, [r9, #2800]	@ 0xaf0
	f8c9 0af4 	str.w	r0, [r9, #2804]	@ 0xaf4
	f649 2053 	movw	r0, #39507	@ 0x9a53
	f50e 51ef 	add.w	r1, lr, #7648	@ 0x1de0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd ff6a 	bl	<core::panicking::panic_fmt>
	f10d 0e10 	add.w	lr, sp, #16
    let mut sockets = [SocketStorage::EMPTY];
	f24a 71a0 	movw	r1, #42912	@ 0xa7a0
	f50e 58d4 	add.w	r8, lr, #6784	@ 0x1a80
	f6c0 0100 	movt	r1, #2048	@ 0x800
	f44f 72a8 	mov.w	r2, #336	@ 0x150
	4640      	mov	r0, r8
	f003 fcef 	bl	<__aeabi_memcpy8>
	f10d 0e08 	add.w	lr, sp, #8
    let mut server_rx_buffer = [0; 512];
	f44f 7100 	mov.w	r1, #512	@ 0x200
	f50e 56df 	add.w	r6, lr, #7136	@ 0x1be0
    pub fn new<SocketsT>(sockets: SocketsT) -> SocketSet<'a>
    where
        SocketsT: Into<ManagedSlice<'a, SocketStorage<'a>>>,
    {
        let sockets = sockets.into();
        SocketSet { sockets }
	f8c9 88e8 	str.w	r8, [r9, #2280]	@ 0x8e8
	f8c9 48ec 	str.w	r4, [r9, #2284]	@ 0x8ec
	f44f 7b00 	mov.w	fp, #512	@ 0x200
	4630      	mov	r0, r6
	f003 fc83 	bl	<__aeabi_memclr4>
	f10d 0e08 	add.w	lr, sp, #8
    let mut server_tx_buffer = [0; 512];
	f44f 7100 	mov.w	r1, #512	@ 0x200
	f50e 50ef 	add.w	r0, lr, #7648	@ 0x1de0
	f003 fc7b 	bl	<__aeabi_memclr4>
        matches!(*self, Some(_))
	f8d9 0798 	ldr.w	r0, [r9, #1944]	@ 0x798
	f04f 0a00 	mov.w	sl, #0
	f8d9 179c 	ldr.w	r1, [r9, #1948]	@ 0x79c
	f080 0003 	eor.w	r0, r0, #3
	e947 aa0e 	strd	sl, sl, [r7, #-56]	@ 0x38
	4308      	orrs	r0, r1
	e947 aa10 	strd	sl, sl, [r7, #-64]	@ 0x40
	e947 aa12 	strd	sl, sl, [r7, #-72]	@ 0x48
	e947 aa14 	strd	sl, sl, [r7, #-80]	@ 0x50
        if rx_capacity > (1 << 30) {
            panic!("receiving buffer too large, cannot exceed 1 GiB")
        }
        let rx_cap_log2 = mem::size_of::<usize>() * 8 - rx_capacity.leading_zeros() as usize;

        Socket {
	e947 aa0a 	strd	sl, sl, [r7, #-40]	@ 0x28
	e947 aa08 	strd	sl, sl, [r7, #-32]
	e947 aa18 	strd	sl, sl, [r7, #-96]	@ 0x60
	e947 aa16 	strd	sl, sl, [r7, #-88]	@ 0x58
        }

        let socket = socket.upcast();

        for (index, slot) in self.sockets.iter_mut().enumerate() {
            if slot.inner.is_none() {
	f040 8223 	bne.w	<ip::__cortex_m_rt_main+0x1434>
	f242 7010 	movw	r0, #10000	@ 0x2710
            *slot = SocketStorage {
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
	e09b      	b.n	<ip::__cortex_m_rt_main+0x1240>
	42800000 	.word	0x42800000
        let rx_cap_log2 =
            mem::size_of::<usize>() * 8 - self.rx_buffer.capacity().leading_zeros() as usize;

        self.state = State::Closed;
        self.timer = Timer::new();
        self.rtte = RttEstimator::default();
	2064      	movs	r0, #100	@ 0x64
	f44f 7196 	mov.w	r1, #300	@ 0x12c
	e9c5 1008 	strd	r1, r0, [r5, #32]
        }
    }

    /// Clear the ring buffer.
    pub fn clear(&mut self) {
        self.read_at = 0;
	f105 00d0 	add.w	r0, r5, #208	@ 0xd0
	2128      	movs	r1, #40	@ 0x28
	e9c5 aa00 	strd	sl, sl, [r5]
	f8c5 a018 	str.w	sl, [r5, #24]
        self.timer = Timer::new();
	e9c5 aa20 	strd	sl, sl, [r5, #128]	@ 0x80
	e9c5 aa22 	strd	sl, sl, [r5, #136]	@ 0x88
        self.assembler = Assembler::new();
        self.tx_buffer.clear();
        self.rx_buffer.clear();
        self.rx_fin_received = false;
	f885 a130 	strb.w	sl, [r5, #304]	@ 0x130
	e9c5 aa30 	strd	sl, sl, [r5, #192]	@ 0xc0
        self.rtte = RttEstimator::default();
	f885 a028 	strb.w	sl, [r5, #40]	@ 0x28
impl<'a, T: 'a> Deref for ManagedSlice<'a, T> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        match self {
            &ManagedSlice::Borrowed(ref value) => value,
	f8d5 40bc 	ldr.w	r4, [r5, #188]	@ 0xbc
        self.tuple = None;
        self.local_seq_no = TcpSeqNumber::default();
        self.remote_seq_no = TcpSeqNumber::default();
        self.remote_last_seq = TcpSeqNumber::default();
        self.remote_last_ack = None;
        self.remote_last_win = 0;
	f8a5 a12e 	strh.w	sl, [r5, #302]	@ 0x12e
        self.remote_last_ack = None;
	f8c5 a098 	str.w	sl, [r5, #152]	@ 0x98
        self.remote_win_len = 0;
        self.remote_win_scale = None;
	f885 a12c 	strb.w	sl, [r5, #300]	@ 0x12c
        self.local_seq_no = TcpSeqNumber::default();
	e9c5 aa40 	strd	sl, sl, [r5, #256]	@ 0x100
	e9c5 aa42 	strd	sl, sl, [r5, #264]	@ 0x108
	f002 fe83 	bl	<__aeabi_memclr8>
                return Err(ListenError::InvalidState);
            }
        }

        self.reset();
        self.listen_endpoint = local_endpoint;
	2050      	movs	r0, #80	@ 0x50
        #[must_use = "this returns the result of the operation, \
                      without modifying the original"]
        #[rustc_const_stable(feature = "const_saturating_int_methods", since = "1.47.0")]
        #[inline(always)]
        pub const fn saturating_sub(self, rhs: Self) -> Self {
            intrinsics::saturating_sub(self, rhs)
	2210      	movs	r2, #16
	e9c5 0a2c 	strd	r0, sl, [r5, #176]	@ 0xb0
        self.remote_mss = DEFAULT_MSS;
	f44f 7006 	mov.w	r0, #536	@ 0x218
	f8c5 0110 	str.w	r0, [r5, #272]	@ 0x110
            return intrinsics::ctlz(self as $ActualT);
	fab4 f084 	clz	r0, r4
            intrinsics::saturating_sub(self, rhs)
	f1c0 0110 	rsb	r1, r0, #16
	4282      	cmp	r2, r0
                defmt::info!("Listening at {}:80...", IP_ADDRESS);
	f240 002c 	movw	r0, #44	@ 0x2c
	f04f 0901 	mov.w	r9, #1
	f2c0 0000 	movt	r0, #0
        self.challenge_ack_timer = Instant::from_secs(0);
	e9c5 aa3e 	strd	sl, sl, [r5, #248]	@ 0xf8
        self.ack_delay_timer = AckDelayTimer::Idle;
	e9c5 aa1c 	strd	sl, sl, [r5, #112]	@ 0x70
        self.remote_last_ts = None;
	e9c5 aa14 	strd	sl, sl, [r5, #80]	@ 0x50
    fn set_state(&mut self, state: State) {
        if self.state != state {
            tcp_trace!("state={}=>{}", self.state, state);
        }

        self.state = state;
	f885 9133 	strb.w	r9, [r5, #307]	@ 0x133
        self.tuple = None;
	f8a5 a11c 	strh.w	sl, [r5, #284]	@ 0x11c
	bf38      	it	cc
	4651      	movcc	r1, sl
        self.remote_win_shift = rx_cap_log2.saturating_sub(16) as u8;
	f885 1134 	strb.w	r1, [r5, #308]	@ 0x134
	f7fe fc0f 	bl	<defmt::export::acquire_and_header>
        // transmute them to arrays of bytes
        #[inline]
        pub const fn to_ne_bytes(self) -> [u8; size_of::<Self>()] {
            // SAFETY: integers are plain old datatypes so we can always transmute them to
            // arrays of bytes
            unsafe { mem::transmute(self) }
	f240 0007 	movw	r0, #7
	f1a7 0414 	sub.w	r4, r7, #20
	f2c0 0000 	movt	r0, #0
#[inline(always)]
pub fn write(bytes: &[u8]) {
    extern "Rust" {
        fn _defmt_write(bytes: &[u8]);
    }
    unsafe { _defmt_write(bytes) }
	2102      	movs	r1, #2
	f827 0c14 	strh.w	r0, [r7, #-20]
	4620      	mov	r0, r4
	f7fe fdb5 	bl	<_defmt_write>
	f240 0032 	movw	r0, #50	@ 0x32
	2102      	movs	r1, #2
	f2c0 0000 	movt	r0, #0
	f827 0c14 	strh.w	r0, [r7, #-20]
	4620      	mov	r0, r4
	f7fe fdab 	bl	<_defmt_write>
	f240 0502 	movw	r5, #2
	4620      	mov	r0, r4
	f2c0 0500 	movt	r5, #0
	2102      	movs	r1, #2
	f827 5c14 	strh.w	r5, [r7, #-20]
	f7fe fda1 	bl	<_defmt_write>
	200a      	movs	r0, #10
	2101      	movs	r1, #1
	f807 0c14 	strb.w	r0, [r7, #-20]
	4620      	mov	r0, r4
	f7fe fd9a 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f827 5c14 	strh.w	r5, [r7, #-20]
	f7fe fd94 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2101      	movs	r1, #1
	f807 ac14 	strb.w	sl, [r7, #-20]
	f7fe fd8e 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f827 5c14 	strh.w	r5, [r7, #-20]
	f7fe fd88 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2101      	movs	r1, #1
	f807 ac14 	strb.w	sl, [r7, #-20]
	f7fe fd82 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f827 5c14 	strh.w	r5, [r7, #-20]
	f7fe fd7c 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2101      	movs	r1, #1
	f807 9c14 	strb.w	r9, [r7, #-20]
	f7fe fd76 	bl	<_defmt_write>
	4620      	mov	r0, r4
	2102      	movs	r1, #2
	f827 ac14 	strh.w	sl, [r7, #-20]
	f7fe fd70 	bl	<_defmt_write>
    _defmt_release()
	f7fe fcf6 	bl	<_defmt_release>
	f002 fe06 	bl	<__primask_r>
	4604      	mov	r4, r0
	f002 fdf6 	bl	<__cpsid>
	07e0      	lsls	r0, r4, #31
        let time: u64 = cortex_m::interrupt::free(|cs| *TIME.borrow(cs).borrow());
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
        iface.poll(Instant::from_millis(time as i64), &mut dma, &mut sockets);
	f10d 0e04 	add.w	lr, sp, #4
    }

    /// Create a new `Instant` from a number of milliseconds.
    pub fn from_millis<T: Into<i64>>(millis: T) -> Instant {
        Instant {
            micros: millis.into() * 1000,
	fba5 b001 	umull	fp, r0, r5, r1
	fb09 0401 	mla	r4, r9, r1, r0
	f50d 50df 	add.w	r0, sp, #7136	@ 0x1be0
	f50e 51c8 	add.w	r1, lr, #6400	@ 0x1900
	e9cd 1000 	strd	r1, r0, [sp]
	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
	465a      	mov	r2, fp
	4623      	mov	r3, r4
	f7fb f9fc 	bl	<smoltcp::iface::interface::Interface::poll>
        let socket = sockets.get_mut::<TcpSocket>(server_handle);
	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
    ///
    /// # Panics
    /// This function may panic if the handle does not belong to this socket set
    /// or the socket has the wrong type.
    pub fn get_mut<T: AnySocket<'a>>(&mut self, handle: SocketHandle) -> &mut T {
        match self.sockets[handle.0].inner.as_mut() {
	2900      	cmp	r1, #0
	f000 816f 	beq.w	<ip::__cortex_m_rt_main+0x1578>
	f8d6 58e8 	ldr.w	r5, [r6, #2280]	@ 0x8e8
        match *self {
	6828      	ldr	r0, [r5, #0]
	43c1      	mvns	r1, r0
	0789      	lsls	r1, r1, #30
	f000 80ff 	beq.w	<ip::__cortex_m_rt_main+0x14a6>
	2802      	cmp	r0, #2
	f000 8107 	beq.w	<ip::__cortex_m_rt_main+0x14bc>
        match self.state {
	f895 0133 	ldrb.w	r0, [r5, #307]	@ 0x133
        if !socket.is_listening() && !socket.is_open() {
	2800      	cmp	r0, #0
	bf18      	it	ne
	280a      	cmpne	r0, #10
	f43f af28 	beq.w	<ip::__cortex_m_rt_main+0x110c>
        match self.state {
	2807      	cmp	r0, #7
	bf18      	it	ne
	2804      	cmpne	r0, #4
	d1bd      	bne.n	<ip::__cortex_m_rt_main+0x1240>
        }
    }

    /// Return the current number of elements in the ring buffer.
    pub fn len(&self) -> usize {
        self.length
	f8d5 60d4 	ldr.w	r6, [r5, #212]	@ 0xd4
    /// than the size of the slice passed into it.
    pub fn enqueue_many_with<'b, R, F>(&'b mut self, f: F) -> (usize, R)
    where
        F: FnOnce(&'b mut [T]) -> (usize, R),
    {
        if self.length == 0 {
	2e00      	cmp	r6, #0
            // Ring is currently empty. Reset `read_at` to optimize
            // for contiguous space.
            self.read_at = 0;
	bf08      	it	eq
	f8c5 a0d0 	streq.w	sl, [r5, #208]	@ 0xd0
	f8d5 90cc 	ldr.w	r9, [r5, #204]	@ 0xcc
        if len > 0 {
	f1b9 0f00 	cmp.w	r9, #0
	d007      	beq.n	<ip::__cortex_m_rt_main+0x12ea>
            (self.read_at + idx) % len
	f8d5 00d0 	ldr.w	r0, [r5, #208]	@ 0xd0
	4430      	add	r0, r6
	fbb0 f1f9 	udiv	r1, r0, r9
	fb01 0019 	mls	r0, r1, r9, r0
	e000      	b.n	<ip::__cortex_m_rt_main+0x12ec>
	2000      	movs	r0, #0
        self.capacity() - self.len()
	eba9 0806 	sub.w	r8, r9, r6
        cmp::min(self.window(), self.capacity() - self.get_idx(self.length))
	eba9 0100 	sub.w	r1, r9, r0
        if other < self { other } else { self }
	4541      	cmp	r1, r8
	bf38      	it	cc
	4688      	movcc	r8, r1
        }

        let write_at = self.get_idx(self.length);
        let max_size = self.contiguous_window();
        let (size, result) = f(&mut self.storage[write_at..write_at + max_size]);
	eb18 0100 	adds.w	r1, r8, r0
            if self < rhs {
	f080 80e8 	bcs.w	<ip::__cortex_m_rt_main+0x14d2>
	4549      	cmp	r1, r9
	f200 80e5 	bhi.w	<ip::__cortex_m_rt_main+0x14d2>
}

impl<'a, T: 'a> DerefMut for ManagedSlice<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            &mut ManagedSlice::Borrowed(ref mut value) => value,
	f8d5 10c8 	ldr.w	r1, [r5, #200]	@ 0xc8
    }
	f1b8 0f06 	cmp.w	r8, #6
	bf28      	it	cs
	f04f 0806 	movcs.w	r8, #6
	9106      	str	r1, [sp, #24]
    offset: usize,
    len: usize,
) -> *mut [T] {
    let ptr = ptr as *mut T;
    // SAFETY: The caller already checked these preconditions
    let ptr = unsafe { crate::intrinsics::offset(ptr, offset) };
	4408      	add	r0, r1
    unsafe { crate::intrinsics::copy_nonoverlapping(src, dst, count) }
	f64a 01f0 	movw	r1, #43248	@ 0xa8f0
	f6c0 0100 	movt	r1, #2048	@ 0x800
	4642      	mov	r2, r8
	f003 fb1a 	bl	<__aeabi_memcpy>
        assert!(size <= max_size);
        self.length += size;
	eb18 0206 	adds.w	r2, r8, r6
            self.read_at = 0;
	bf08      	it	eq
	f8c5 a0d0 	streq.w	sl, [r5, #208]	@ 0xd0
        if len > 0 {
	f1b9 0f00 	cmp.w	r9, #0
	9407      	str	r4, [sp, #28]
	d007      	beq.n	<ip::__cortex_m_rt_main+0x134a>
            (self.read_at + idx) % len
	f8d5 00d0 	ldr.w	r0, [r5, #208]	@ 0xd0
	4410      	add	r0, r2
	fbb0 f1f9 	udiv	r1, r0, r9
	fb01 0019 	mls	r0, r1, r9, r0
	e000      	b.n	<ip::__cortex_m_rt_main+0x134c>
	2000      	movs	r0, #0
        self.capacity() - self.len()
	eba9 0402 	sub.w	r4, r9, r2
        cmp::min(self.window(), self.capacity() - self.get_idx(self.length))
	eba9 0100 	sub.w	r1, r9, r0
        if other < self { other } else { self }
	42a1      	cmp	r1, r4
	bf38      	it	cc
	460c      	movcc	r4, r1
        let (size, result) = f(&mut self.storage[write_at..write_at + max_size]);
	1821      	adds	r1, r4, r0
	f080 80b9 	bcs.w	<ip::__cortex_m_rt_main+0x14d2>
	4549      	cmp	r1, r9
	f200 80b6 	bhi.w	<ip::__cortex_m_rt_main+0x14d2>
        if self.start > slice.len() {
            slice_index_fail(self.start, slice.len(), slice.len())
        }
        // SAFETY: `self` is checked to be valid and in bounds above.
        unsafe {
            let new_len = crate::intrinsics::unchecked_sub(slice.len(), self.start);
	f1c8 0106 	rsb	r1, r8, #6
	46b1      	mov	r9, r6
    }
	42a1      	cmp	r1, r4
	bf38      	it	cc
	460c      	movcc	r4, r1
    let ptr = unsafe { crate::intrinsics::offset(ptr, offset) };
	9906      	ldr	r1, [sp, #24]
	4616      	mov	r6, r2
	4622      	mov	r2, r4
	4408      	add	r0, r1
    let ptr = unsafe { crate::intrinsics::offset(ptr, offset) };
	f64a 01f0 	movw	r1, #43248	@ 0xa8f0
	f6c0 0100 	movt	r1, #2048	@ 0x800
	4441      	add	r1, r8
	f003 faea 	bl	<__aeabi_memcpy>
        self.length += size;
	19a0      	adds	r0, r4, r6
        if size > 0 {
	f1b9 0f00 	cmp.w	r9, #0
	f8c5 00d4 	str.w	r0, [r5, #212]	@ 0xd4
	d104      	bne.n	<ip::__cortex_m_rt_main+0x139e>
	ea54 0008 	orrs.w	r0, r4, r8
                self.remote_last_ts = None
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
                    while sockets.get::<TcpSocket>(server_handle).send_queue() != 0 {
	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
        match self.sockets[handle.0].inner.as_ref() {
	2900      	cmp	r1, #0
	f000 80b9 	beq.w	<ip::__cortex_m_rt_main+0x1538>
	f8d6 08e8 	ldr.w	r0, [r6, #2280]	@ 0x8e8
        match *self {
	6801      	ldr	r1, [r0, #0]
	43ca      	mvns	r2, r1
	0792      	lsls	r2, r2, #30
	d045      	beq.n	<ip::__cortex_m_rt_main+0x145e>
	2902      	cmp	r1, #2
	d04e      	beq.n	<ip::__cortex_m_rt_main+0x1474>
        self.length
	f8d0 20d4 	ldr.w	r2, [r0, #212]	@ 0xd4
	b182      	cbz	r2, <ip::__cortex_m_rt_main+0x13fe>
                        iface.poll(
	f10d 0e04 	add.w	lr, sp, #4
	4622      	mov	r2, r4
	f50e 50c8 	add.w	r0, lr, #6400	@ 0x1900
	e9cd 0900 	strd	r0, r9, [sp]
	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
	462b      	mov	r3, r5
	f7fb f94a 	bl	<smoltcp::iface::interface::Interface::poll>
                    while sockets.get::<TcpSocket>(server_handle).send_queue() != 0 {
	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
	2900      	cmp	r1, #0
	d1e4      	bne.n	<ip::__cortex_m_rt_main+0x13c6>
	e09c      	b.n	<ip::__cortex_m_rt_main+0x1538>
        match *self {
	f001 0103 	and.w	r1, r1, #3
	2903      	cmp	r1, #3
	d04f      	beq.n	<ip::__cortex_m_rt_main+0x14a6>
	2902      	cmp	r1, #2
	d058      	beq.n	<ip::__cortex_m_rt_main+0x14bc>
        self.state = state;
	f880 a133 	strb.w	sl, [r0, #307]	@ 0x133
                    defmt::info!("Transmitted hello! Closing socket...");
	f240 002d 	movw	r0, #45	@ 0x2d
	f2c0 0000 	movt	r0, #0
	f7fe fae0 	bl	<defmt::export::acquire_header_and_release>
                    iface.poll(Instant::from_millis(time as i64), &mut dma, &mut sockets);
	f10d 0e04 	add.w	lr, sp, #4
	9b07      	ldr	r3, [sp, #28]
	f50e 50c8 	add.w	r0, lr, #6400	@ 0x1900
	e9cd 0900 	strd	r0, r9, [sp]
	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
	465a      	mov	r2, fp
	f7fb f92b 	bl	<smoltcp::iface::interface::Interface::poll>
                }
	e705      	b.n	<ip::__cortex_m_rt_main+0x1240>
            ManagedSlice::Borrowed(_) => panic!("adding a socket to a full SocketSet"),
	f649 10c0 	movw	r0, #39360	@ 0x99c0
	f24a 424c 	movw	r2, #42060	@ 0xa44c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2147      	movs	r1, #71	@ 0x47
	f7fd fcfd 	bl	<core::panicking::panic_fmt>
            96..=191 => (128.0, 0b1101),
	f1a0 0360 	sub.w	r3, r0, #96	@ 0x60
	2b60      	cmp	r3, #96	@ 0x60
	d21b      	bcs.n	<ip::__cortex_m_rt_main+0x148a>
	ed9f 2a55 	vldr	s4, [pc, #340]	@ <ip::__cortex_m_rt_main+0x15a8>
	f04f 0ed0 	mov.w	lr, #208	@ 0xd0
	f7fe bec6 	b.w	<ip::__cortex_m_rt_main+0x1ea>
            None => panic!("handle does not refer to a valid socket"),
	f24a 4098 	movw	r0, #42136	@ 0xa498
	f24a 42c0 	movw	r2, #42176	@ 0xa4c0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	214f      	movs	r1, #79	@ 0x4f
	f7fd fce8 	bl	<core::panicking::panic_fmt>
            None => expect_failed(msg),
	f24a 405c 	movw	r0, #42076	@ 0xa45c
	f24a 42d0 	movw	r2, #42192	@ 0xa4d0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2129      	movs	r1, #41	@ 0x29
	f7fe f916 	bl	<core::option::expect_failed>
            192..=383 => (256.0, 0b1110),
	38c0      	subs	r0, #192	@ 0xc0
	a347      	add	r3, pc, #284	@ (adr r3, <ip::__cortex_m_rt_main+0x15ac>)
	f04f 0ef0 	mov.w	lr, #240	@ 0xf0
	28c0      	cmp	r0, #192	@ 0xc0
	bf38      	it	cc
	3304      	addcc	r3, #4
	ed93 2a00 	vldr	s4, [r3]
	bf38      	it	cc
	f04f 0ee0 	movcc.w	lr, #224	@ 0xe0
	f7fe bea2 	b.w	<ip::__cortex_m_rt_main+0x1ea>
            Some(item) => T::downcast_mut(&mut item.socket)
                .expect("handle refers to a socket of a wrong type"),
            None => panic!("handle does not refer to a valid socket"),
	f24a 4098 	movw	r0, #42136	@ 0xa498
	f24a 42f0 	movw	r2, #42224	@ 0xa4f0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	214f      	movs	r1, #79	@ 0x4f
	f7fd fcc4 	bl	<core::panicking::panic_fmt>
	f24a 405c 	movw	r0, #42076	@ 0xa45c
	f24a 5200 	movw	r2, #42240	@ 0xa500
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2129      	movs	r1, #41	@ 0x29
	f7fe f8f2 	bl	<core::option::expect_failed>
	f64a 63fc 	movw	r3, #44796	@ 0xaefc
	464a      	mov	r2, r9
	f6c0 0300 	movt	r3, #2048	@ 0x800
	f7fd fc41 	bl	<core::slice::index::slice_index_fail>
        assert!(sysclk <= 216_000_000);
	f24b 10e0 	movw	r0, #45536	@ 0xb1e0
	f24b 2208 	movw	r2, #45576	@ 0xb208
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2127      	movs	r1, #39	@ 0x27
	f7fd fcc9 	bl	<core::panicking::panic>
            0 => unreachable!(),
	f24b 1098 	movw	r0, #45464	@ 0xb198
	f24b 2218 	movw	r2, #45592	@ 0xb218
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2128      	movs	r1, #40	@ 0x28
	f7fd fcbe 	bl	<core::panicking::panic>
            0 => unreachable!(),
	f24b 1098 	movw	r0, #45464	@ 0xb198
	f24b 2228 	movw	r2, #45608	@ 0xb228
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2128      	movs	r1, #40	@ 0x28
	f7fd fcb3 	bl	<core::panicking::panic>
            0 => unreachable!(),
	f24b 1098 	movw	r0, #45464	@ 0xb198
	f24b 2238 	movw	r2, #45624	@ 0xb238
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	2128      	movs	r1, #40	@ 0x28
	f7fd fca8 	bl	<core::panicking::panic>
        match self.sockets[handle.0].inner.as_ref() {
	f24a 4288 	movw	r2, #42120	@ 0xa488
	2000      	movs	r0, #0
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd fc8d 	bl	<core::panicking::panic_bounds_check>
	f10d 0e08 	add.w	lr, sp, #8
            panic!("couldn't calculate {} from {}", sysclk, base_clk);
	f244 000f 	movw	r0, #16399	@ 0x400f
	f50e 51ef 	add.w	r1, lr, #7648	@ 0x1de0
	f6c0 0000 	movt	r0, #2048	@ 0x800
	900b      	str	r0, [sp, #44]	@ 0x2c
	f24b 12c0 	movw	r2, #45504	@ 0xb1c0
	e9cd 0109 	strd	r0, r1, [sp, #36]	@ 0x24
	f50d 604a 	add.w	r0, sp, #3232	@ 0xca0
	9008      	str	r0, [sp, #32]
	f24a 4001 	movw	r0, #41985	@ 0xa401
	a908      	add	r1, sp, #32
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd fc66 	bl	<core::panicking::panic_fmt>
        match self.sockets[handle.0].inner.as_mut() {
	f24a 42e0 	movw	r2, #42208	@ 0xa4e0
	2000      	movs	r0, #0
	f6c0 0200 	movt	r2, #2048	@ 0x800
	f7fd fc6d 	bl	<core::panicking::panic_bounds_check>
	f002 fc58 	bl	<__cpsie>
            Some(val) => val,
            None => unwrap_failed(),
	f24a 707c 	movw	r0, #42876	@ 0xa77c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fd fbdc 	bl	<core::option::unwrap_failed>
	f002 fc50 	bl	<__cpsie>
	f24a 708c 	movw	r0, #42892	@ 0xa78c
	f6c0 0000 	movt	r0, #2048	@ 0x800
	f7fd fbd4 	bl	<core::option::unwrap_failed>
	bf00      	nop
	43000000 	.word	0x43000000
	44000000 	.word	0x44000000
	43800000 	.word	0x43800000
