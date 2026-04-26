
./working:     file format elf32-littlearm
./working
architecture: armv7e-m, flags 0x00000112:
EXEC_P, HAS_SYMS, D_PAGED
start address 0x080001f9

Program Header:
    LOAD off    0x00010000 vaddr 0x08000000 paddr 0x08000000 align 2**16
         filesz 0x000001f8 memsz 0x000001f8 flags r--
    LOAD off    0x000101f8 vaddr 0x080001f8 paddr 0x080001f8 align 2**16
         filesz 0x00009638 memsz 0x00009638 flags r-x
    LOAD off    0x00019830 vaddr 0x08009830 paddr 0x08009830 align 2**16
         filesz 0x00001918 memsz 0x00001918 flags r--
    LOAD off    0x00020000 vaddr 0x20000000 paddr 0x0800b148 align 2**16
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
  1 .text         00009638  080001f8  080001f8  000101f8  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
  2 .rodata       00001918  08009830  08009830  00019830  2**3
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
  3 .data         00000038  20000000  0800b148  00020000  2**2
                  CONTENTS, ALLOC, LOAD, DATA
  4 .gnu.sgstubs  00000000  0800b180  0800b180  00020040  2**5
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
  5 .bss          0000003c  20000038  20000038  00030038  2**3
                  ALLOC
  6 .uninit       00000400  20000074  20000074  00030038  2**2
                  ALLOC
  7 .defmt        00000042  00000000  00000000  00030038  2**0
                  CONTENTS, READONLY
  8 .debug_loc    00024a28  00000000  00000000  0003007a  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
  9 .debug_abbrev 000015c0  00000000  00000000  00054aa2  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 10 .debug_info   0006fa6a  00000000  00000000  00056062  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 11 .debug_aranges 00001608  00000000  00000000  000c5acc  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 12 .debug_ranges 0000a9d0  00000000  00000000  000c70d4  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 13 .debug_str    0006ae3a  00000000  00000000  000d1aa4  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 14 .comment      0000008b  00000000  00000000  0013c8de  2**0
                  CONTENTS, READONLY
 15 .ARM.attributes 00000038  00000000  00000000  0013c969  2**0
                  CONTENTS, READONLY
 16 .debug_frame  00000ec8  00000000  00000000  0013c9a4  2**2
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 17 .debug_line   000159aa  00000000  00000000  0013d86c  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 18 .debug_pubnames 000002be  00000000  00000000  00153216  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
 19 .debug_pubtypes 00000047  00000000  00000000  001534d4  2**0
                  CONTENTS, READONLY, DEBUGGING, OCTETS
SYMBOL TABLE:
00000000 l    df *ABS*	00000000 ip.db5b21dddbb36540-cgu.3
08009810 l     F .text	00000018 HardFaultTrampoline
08000250 l     F .text	0000010c smoltcp::wire::arp::Repr::emit
08009890 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.8
080036e2 l     F .text	00000032 core::slice::index::slice_index_fail
08009860 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.5
08009840 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.3
08009850 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.4
08009830 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.2
08009870 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.6
08009880 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.7
080098a0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.9
080037e0 l     F .text	00000028 core::panicking::panic_bounds_check
080098b0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.10
0800035c l     F .text	00000dc0 smoltcp::iface::interface::InterfaceInner::dispatch_ip
0800435a l     F .text	00000026 defmt::export::acquire_header_and_release
0800830a l     F .text	0000013e smoltcp::iface::route::Routes::lookup
0800af04 l     O .rodata	0000002c .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.7
0800af30 l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.8
08003808 l     F .text	0000000c core::panicking::panic
0800803e l     F .text	000002cc smoltcp::wire::ip::checksum::data
08007b7c l     F .text	0000017e smoltcp::wire::ipv4::Repr::emit
0800a9d8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.41
080061a8 l     F .text	00000224 smoltcp::wire::tcp::TcpOption::emit
0800aa68 l     O .rodata	00000024 .Lanon.ee555cc9a30366a9abf098ddf788bd84.55
0800aa8c l     O .rodata	00000024 .Lanon.ee555cc9a30366a9abf098ddf788bd84.56
08004338 l     F .text	00000022 defmt::export::acquire_and_header
08004294 l     F .text	000000a4 defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format
0800a410 l     O .rodata	00000036 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.22
0800a448 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.24
0800afb0 l     O .rodata	0000001d .Lanon.00a2111443151d169d68e8fc2cf30f9e.21
0800afd0 l     O .rodata	00000010 .Lanon.00a2111443151d169d68e8fc2cf30f9e.22
0800a8e4 l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.15
0800a8f4 l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.17
0800ad04 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.15
0800ad44 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.21
0800acc4 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.5
0800acf4 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.11
0800ace4 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.9
0800ae50 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.12
0800ae80 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.15
0800aa08 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.44
0800ae70 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.14
0800a9f8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.43
0800ae90 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.19
0800aa38 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.47
0800aa28 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.46
0800aa48 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.48
0800aea0 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.29
08003ffc l     F .text	0000000e core::slice::copy_from_slice_impl::len_mismatch_fail
0800ae60 l     O .rodata	00000010 .Lanon.08d70b627e08cc39a48f4d9327c4d667.13
0800aab0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.58
0800abd8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.91
0800abb8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.87
0800abc8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.88
0800aa18 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.45
0800a9e8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.42
0800ac08 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.96
0800ac28 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.98
0800abf8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.95
0800ac18 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.97
0800af90 l     O .rodata	00000010 .Lanon.00a2111443151d169d68e8fc2cf30f9e.17
08005f52 l     F .text	00000086 core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>
0800a904 l     O .rodata	00000010 .Lanon.dd1935691476038951f4dc0f5cac2ac5.19
080036ce l     F .text	00000014 core::option::unwrap_failed
0800acd4 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.8
0800ad34 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.19
0800ad54 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.26
0800ad14 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.16
0800af50 l     O .rodata	00000010 .Lanon.00a2111443151d169d68e8fc2cf30f9e.5
0800abe8 l     O .rodata	00000010 .Lanon.e728c747789d4d3550167230b35740f6.92
0800111c l     F .text	000022c6 smoltcp::iface::interface::Interface::poll
080087ba l     F .text	00000012 stm32_eth::dma::rx::RxRing::demand_poll
08008448 l     F .text	00000372 smoltcp::iface::neighbor::Cache::fill
08007996 l     F .text	000001b6 smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply
08007e2e l     F .text	00000092 smoltcp::socket::udp::Socket::accepts
08007ec0 l     F .text	0000017e smoltcp::socket::udp::Socket::process
08006124 l     F .text	00000084 <core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold
0800648c l     F .text	0000121e smoltcp::socket::tcp::Socket::process
080076aa l     F .text	00000096 smoltcp::socket::tcp::Socket::rst_reply
080063cc l     F .text	000000c0 smoltcp::socket::tcp::Socket::seq_to_transmit
0800ab70 l     O .rodata	00000038 .Lanon.ee555cc9a30366a9abf098ddf788bd84.70
0800aba8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.71
080037c4 l     F .text	0000001c core::panicking::panic_fmt
0800aff0 l     O .rodata	00000041 .Lanon.86296f10aa77425962719a19169a9c1e.2
0800b034 l     O .rodata	00000010 .Lanon.86296f10aa77425962719a19169a9c1e.4
08004036 l     F .text	00000026 core::option::expect_failed
0800b044 l     O .rodata	00000042 .Lanon.86296f10aa77425962719a19169a9c1e.5
0800b088 l     O .rodata	00000010 .Lanon.86296f10aa77425962719a19169a9c1e.6
0800af60 l     O .rodata	00000010 .Lanon.00a2111443151d169d68e8fc2cf30f9e.7
0800a8b4 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.20
0800a8d4 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.23
0800a458 l     O .rodata	00000016 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.25
0800a470 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.26
0800a880 l     O .rodata	00000022 .Lanon.02e056d4c889a085f771c01290e74e2f.18
0800a8a4 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.19
0800a850 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.13
0800a8c4 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.22
0800a994 l     O .rodata	00000033 .Lanon.ee555cc9a30366a9abf098ddf788bd84.23
0800a9c8 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.25
0800a860 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.15
0800afa0 l     O .rodata	00000010 .Lanon.00a2111443151d169d68e8fc2cf30f9e.19
0800af70 l     O .rodata	00000010 .Lanon.00a2111443151d169d68e8fc2cf30f9e.14
0800ad24 l     O .rodata	00000010 .Lanon.7ae0fc733917b56e83876efbea07ac2a.17
0800adb4 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.12
0800409a l     F .text	00000014 core::panicking::panic_const::panic_const_rem_by_zero
0800af80 l     O .rodata	00000010 .Lanon.00a2111443151d169d68e8fc2cf30f9e.16
0800a870 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.17
0800ad64 l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.21
0800ad74 l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.22
0800ad84 l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.23
0800ad94 l     O .rodata	00000010 .Lanon.e649bb90f52bb047098cd49f541ed508.24
0800aca4 l     O .rodata	00000010 .Lanon.ee56ee3d56b2a23f692cb60d33ec53f4.8
0800ab40 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.67
0800a924 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.2
0800ab20 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.65
0800ab30 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.66
0800aa58 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.54
080033e2 l     F .text	0000010c core::fmt::write
080034f0 l     F .text	0000019c core::fmt::Formatter::pad_integral
0800368c l     F .text	00000042 core::fmt::Formatter::pad_integral::write_prefix
0800a548 l     O .rodata	0000002b .Lanon.dc388a968c23fa524085dd94a83f5751.251
08003714 l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
08003740 l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
08003798 l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
0800376c l     F .text	0000002c core::slice::index::slice_index_fail::do_panic::runtime
08003efa l     F .text	00000102 core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt
08005fd8 l     F .text	00000088 __rustc::rust_begin_unwind
08003814 l     F .text	00000014 <&T as core::fmt::Display>::fmt
08003828 l     F .text	00000652 core::fmt::Formatter::pad
08003e7a l     F .text	00000014 core::panicking::panic_const::panic_const_div_by_zero
0800a573 l     O .rodata	00000019 .Lanon.dc388a968c23fa524085dd94a83f5751.271
08003e8e l     F .text	00000010 <&T as core::fmt::Debug>::fmt
08003e9e l     F .text	0000005c core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt
0800a480 l     O .rodata	000000c8 .Lanon.dc388a968c23fa524085dd94a83f5751.14
0800400a l     F .text	0000002c core::slice::copy_from_slice_impl::len_mismatch_fail::do_panic::runtime
0800405c l     F .text	0000003e core::result::unwrap_failed
0800a58c l     O .rodata	00000039 .Lanon.dc388a968c23fa524085dd94a83f5751.273
080040ae l     F .text	000000ce <core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt
0800a5cc l     O .rodata	00000018 .Lanon.dc388a968c23fa524085dd94a83f5751.363
0800a5e4 l     O .rodata	00000010 .Lanon.dc388a968c23fa524085dd94a83f5751.365
0800a808 l     O .rodata	0000002b .Lanon.02e056d4c889a085f771c01290e74e2f.8
0800a5f4 l     O .rodata	00000010 .Lanon.dc388a968c23fa524085dd94a83f5751.366
0800a604 l     O .rodata	00000010 .Lanon.dc388a968c23fa524085dd94a83f5751.369
0800417c l     F .text	0000001a <core::fmt::Error as core::fmt::Debug>::fmt
0800a5c5 l     O .rodata	00000005 .Lanon.dc388a968c23fa524085dd94a83f5751.303
08004196 l     F .text	00000032 <core::net::display_buffer::DisplayBuffer<_> as core::fmt::Write>::write_str
080041c8 l     F .text	000000a8 core::fmt::Write::write_char
08004270 l     F .text	00000018 core::fmt::Write::write_fmt
08004380 l     F .text	00000010 <defmt::export::FmtWrite as core::fmt::Write>::write_str
08004390 l     F .text	00000088 core::fmt::Write::write_char
08004418 l     F .text	00000018 core::fmt::Write::write_fmt
0800a624 l     O .rodata	00000018 .Lanon.ea5d5760b73fa5b90415fc99172c09b2.1
08004430 l     F .text	00000082 defmt_rtt::channel::Channel::blocking_write
080044b2 l     F .text	0000005c defmt_rtt::channel::Channel::nonblocking_write
20000040 l     O .bss	00000034 .L_MergedGlobals
0800a63d l     O .rodata	0000001e .Lanon.1d905ed0c947f0695d8369569f054732.0
0800a65c l     O .rodata	00000010 .Lanon.1d905ed0c947f0695d8369569f054732.2
0800a66c l     O .rodata	00000010 .Lanon.1d905ed0c947f0695d8369569f054732.4
08004980 l     F .text	000015b8 ip::__cortex_m_rt_main
0800b0d0 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.6
0800afe0 l     O .rodata	00000010 .Lanon.00a2111443151d169d68e8fc2cf30f9e.24
0800a7f8 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.7
0800a834 l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.9
08007b4c l     F .text	0000002e <smoltcp::wire::ip::Address as core::fmt::Display>::fmt
0800acb4 l     O .rodata	00000010 .Lanon.ee56ee3d56b2a23f692cb60d33ec53f4.14
0800a6a0 l     O .rodata	00000150 .Lanon.02e056d4c889a085f771c01290e74e2f.5
0800a7f0 l     O .rodata	00000006 .Lanon.02e056d4c889a085f771c01290e74e2f.6
080098c0 l     O .rodata	00000023 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.11
0800a34c l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.13
0800a398 l     O .rodata	00000027 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.16
0800a3c0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.17
0800a35c l     O .rodata	00000029 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.14
0800a3d0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.18
0800a3f0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.20
0800a400 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.21
0800adfc l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.17
0800b0e0 l     O .rodata	00000027 .Lanon.4bc4785c9ada05daa616a70a22937c0f.10
0800b108 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.11
0800b098 l     O .rodata	00000028 .Lanon.4bc4785c9ada05daa616a70a22937c0f.3
0800b118 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.12
0800b128 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.13
0800b138 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.14
0800a388 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.15
0800b0c0 l     O .rodata	00000010 .Lanon.4bc4785c9ada05daa616a70a22937c0f.5
0800a3e0 l     O .rodata	00000010 .Lanon.8aa7df76f1befd364c3104ad4c2811fa.19
0800a67c l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.2
0800a68c l     O .rodata	00000010 .Lanon.02e056d4c889a085f771c01290e74e2f.3
08005f38 l     F .text	0000001a <stm32_eth::mac::WrongClock as core::fmt::Debug>::fmt
0800a844 l     O .rodata	0000000a .Lanon.02e056d4c889a085f771c01290e74e2f.10
20000038 l     O .bss	00000001 panic_probe::imp::panic::PANICKED
0800a914 l     O .rodata	00000010 .Lanon.6fd57bafcc6bbd1a0f0c6a4a6d894bad.0
0800a63c l     O .rodata	00000001 .Lanon.ea5d5760b73fa5b90415fc99172c09b2.2
08006060 l     F .text	00000018 panic_probe::hard_fault
08006078 l     F .text	000000ac <&T as core::fmt::Display>::fmt
0800a614 l     O .rodata	0000000c .Lanon.dc388a968c23fa524085dd94a83f5751.408
0800a620 l     O .rodata	00000002 .Lanon.dc388a968c23fa524085dd94a83f5751.409
0800ab50 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.68
0800ab60 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.69
0800aad0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.60
0800ab00 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.63
0800a984 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.6
0800a974 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.4
0800aaf0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.62
0800ab10 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.64
0800aac0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.59
0800aae0 l     O .rodata	00000010 .Lanon.ee555cc9a30366a9abf098ddf788bd84.61
0800ac38 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.29
08007d5e l     F .text	00000066 defmt::export::fmt
08007d30 l     F .text	0000002e defmt::export::fmt
0800ac58 l     O .rodata	00000001 .Lanon.c1140054cd359bfdcd454e5b7dad9548.32
08007cfa l     F .text	00000036 defmt::export::fmt
08007948 l     F .text	0000004e smoltcp::socket::tcp::Socket::immediate_ack_to_transmit
08007740 l     F .text	00000208 smoltcp::socket::tcp::Socket::ack_reply
0800ac48 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.31
0800ac94 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.36
0800adc4 l     O .rodata	00000025 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.14
0800adec l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.15
0800ada4 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.11
0800ac59 l     O .rodata	00000028 .Lanon.c1140054cd359bfdcd454e5b7dad9548.33
0800ac84 l     O .rodata	00000010 .Lanon.c1140054cd359bfdcd454e5b7dad9548.35
0800af40 l     O .rodata	00000010 .Lanon.d4b94c958ea1f8000f4d53dc8b85c559.8
08007dc4 l     F .text	0000006a smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with
0800ae40 l     O .rodata	00000010 .Lanon.c1510acb27196742b5d924b6a5d99da6.17
0800aeb0 l     O .rodata	00000023 .Lanon.f0ac2cabe1edd51d584ebf3fae05aeec.16
0800aed4 l     O .rodata	00000010 .Lanon.f0ac2cabe1edd51d584ebf3fae05aeec.18
0800aee4 l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.4
0800ae0c l     O .rodata	00000022 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.20
0800ae30 l     O .rodata	00000010 .Lanon.321b12b0c8d62f4934ae814c6c7478a1.22
0800aef4 l     O .rodata	00000010 .Lanon.9cfaa6f313d9130c3bf3213837ff6d35.6
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
080087f0 l     F .text	000000ae .hidden __aeabi_memclr8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.250
0800889e l     F .text	0000009c .hidden __aeabi_memcpy4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.012
0800893a l     F .text	000002e6 .hidden compiler_builtins::mem::memcpy
08008c20 l     F .text	00000618 .hidden compiler_builtins::mem::memmove
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.275
08009238 l     F .text	0000000c .hidden __aeabi_memmove8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.279
08009244 l     F .text	000000ae .hidden __aeabi_memclr4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.288
080092f2 l     F .text	0000000c .hidden __aeabi_memcpy
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.320
080092fe l     F .text	0000009c .hidden __aeabi_memcpy8
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.373
0800939a l     F .text	000000d0 .hidden __aeabi_memclr
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.413
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.362
08009480 l     F .text	00000030 .hidden __udivmoddi4
00000000 l    df *ABS*	00000000 compiler_builtins.e1cc8e2a576cb2af-cgu.017
080094b0 l     F .text	00000360 .hidden compiler_builtins::int::specialized_div_rem::u64_div_rem
08000004 g     O .vector_table	00000004 __RESET_VECTOR
080001f8 g     F .text	0000003e Reset
08000008 g     O .vector_table	00000038 __EXCEPTIONS
08004288 g     F .text	00000000 DefaultHandler
08000040 g     O .vector_table	000001b8 __INTERRUPTS
0800450e g     F .text	0000009e _defmt_acquire
080045ac g     F .text	000000f0 _defmt_release
0800428e g     F .text	00000006 __defmt_default_timestamp
08009828 g     F .text	00000000 HardFault
0800428e g     F .text	00000000 __pre_init
08004978 g     F .text	00000008 main
00000022 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"Enable the `proto-ipv4-fragmentation` feature for fragmentation support.","disambiguator":"5448626954131158173","crate_name":"smoltcp"}
00000024 g     O .defmt	00000001 {"package":"smoltcp","tag":"defmt_debug","data":"address {} not in neighbor cache, sending ARP request","disambiguator":"12066069798907769471","crate_name":"smoltcp"}
00000007 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=__internal_FormatSequence}","disambiguator":"8408127155438382822","crate_name":"defmt"}
0800469c g     F .text	0000019c _defmt_write
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
0800946c  w    F .text	00000014 __aeabi_uldivmod
080087e6 g     F .text	00000006 __primask_r
080087cc g     F .text	00000004 __cpsid
080087d0 g     F .text	00000004 __cpsie
08004288 g     F .text	00000006 DefaultHandler_
0800428e g     F .text	00000006 DefaultPreInit
08009828 g     F .text	00000006 HardFault_
00000032 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_write","data":"{}.{}.{}.{}","disambiguator":"14742771906021295210","crate_name":"defmt"}
00000002 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=u8}","disambiguator":"16912398427198947700","crate_name":"defmt"}
0800428e g     F .text	00000000 _defmt_timestamp
20000008 g     O .data	00000030 _SEGGER_RTT
08004838 g     F .text	0000010e ETH
08004946 g     F .text	00000032 SysTick
080087d4 g     F .text	0000000c __delay
080087e0 g     F .text	00000006 __dsb
0000002b g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Enabling ethernet...","disambiguator":"10404236363425019797","crate_name":"ip"}
0000002c g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Listening at {}:80...","disambiguator":"3686426055769507929","crate_name":"ip"}
0000002d g     O .defmt	00000001 {"package":"stm32-eth","tag":"defmt_info","data":"Transmitted hello! Closing socket...","disambiguator":"11635372039424199218","crate_name":"ip"}
0000002e g     O .defmt	00000001 {"package":"panic-probe","tag":"defmt_error","data":"{}","disambiguator":"3281125884690968605","crate_name":"panic_probe"}
00000001 g     O .defmt	00000001 {"package":"defmt","tag":"defmt_prim","data":"{=__internal_Display}","disambiguator":"14158116662542268607","crate_name":"defmt"}
080087ec g     F .text	00000004 __udf
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
08004288 g     F .text	00000000 NonMaskableInt
08004288 g     F .text	00000000 MemoryManagement
08004288 g     F .text	00000000 BusFault
08004288 g     F .text	00000000 UsageFault
08004288 g     F .text	00000000 SVCall
08004288 g     F .text	00000000 DebugMonitor
08004288 g     F .text	00000000 PendSV
00000030 g     O .defmt	00000001 _defmt_encoding_ = rzcobs
00000031 g     O .defmt	00000001 _defmt_version_ = 4
08004288 g     F .text	00000000 WWDG
08004288 g     F .text	00000000 PVD
08004288 g     F .text	00000000 TAMP_STAMP
08004288 g     F .text	00000000 RTC_WKUP
08004288 g     F .text	00000000 FLASH
08004288 g     F .text	00000000 RCC
08004288 g     F .text	00000000 EXTI0
08004288 g     F .text	00000000 EXTI1
08004288 g     F .text	00000000 EXTI2
08004288 g     F .text	00000000 EXTI3
08004288 g     F .text	00000000 EXTI4
08004288 g     F .text	00000000 DMA1_STREAM0
08004288 g     F .text	00000000 DMA1_STREAM1
08004288 g     F .text	00000000 DMA1_STREAM2
08004288 g     F .text	00000000 DMA1_STREAM3
08004288 g     F .text	00000000 DMA1_STREAM4
08004288 g     F .text	00000000 DMA1_STREAM5
08004288 g     F .text	00000000 DMA1_STREAM6
08004288 g     F .text	00000000 ADC
08004288 g     F .text	00000000 CAN1_TX
08004288 g     F .text	00000000 CAN1_RX0
08004288 g     F .text	00000000 CAN1_RX1
08004288 g     F .text	00000000 CAN1_SCE
08004288 g     F .text	00000000 EXTI9_5
08004288 g     F .text	00000000 TIM1_BRK_TIM9
08004288 g     F .text	00000000 TIM1_UP_TIM10
08004288 g     F .text	00000000 TIM1_TRG_COM_TIM11
08004288 g     F .text	00000000 TIM1_CC
08004288 g     F .text	00000000 TIM2
08004288 g     F .text	00000000 TIM3
08004288 g     F .text	00000000 TIM4
08004288 g     F .text	00000000 I2C1_EV
08004288 g     F .text	00000000 I2C1_ER
08004288 g     F .text	00000000 I2C2_EV
08004288 g     F .text	00000000 I2C2_ER
08004288 g     F .text	00000000 SPI1
08004288 g     F .text	00000000 SPI2
08004288 g     F .text	00000000 USART1
08004288 g     F .text	00000000 USART2
08004288 g     F .text	00000000 USART3
08004288 g     F .text	00000000 EXTI15_10
08004288 g     F .text	00000000 RTC_ALARM
08004288 g     F .text	00000000 OTG_FS_WKUP
08004288 g     F .text	00000000 TIM8_BRK_TIM12
08004288 g     F .text	00000000 TIM8_UP_TIM13
08004288 g     F .text	00000000 TIM8_TRG_COM_TIM14
08004288 g     F .text	00000000 TIM8_CC
08004288 g     F .text	00000000 DMA1_STREAM7
08004288 g     F .text	00000000 FMC
08004288 g     F .text	00000000 SDMMC1
08004288 g     F .text	00000000 TIM5
08004288 g     F .text	00000000 SPI3
08004288 g     F .text	00000000 UART4
08004288 g     F .text	00000000 UART5
08004288 g     F .text	00000000 TIM6_DAC
08004288 g     F .text	00000000 TIM7
08004288 g     F .text	00000000 DMA2_STREAM0
08004288 g     F .text	00000000 DMA2_STREAM1
08004288 g     F .text	00000000 DMA2_STREAM2
08004288 g     F .text	00000000 DMA2_STREAM3
08004288 g     F .text	00000000 DMA2_STREAM4
08004288 g     F .text	00000000 ETH_WKUP
08004288 g     F .text	00000000 CAN2_TX
08004288 g     F .text	00000000 CAN2_RX0
08004288 g     F .text	00000000 CAN2_RX1
08004288 g     F .text	00000000 CAN2_SCE
08004288 g     F .text	00000000 OTG_FS
08004288 g     F .text	00000000 DMA2_STREAM5
08004288 g     F .text	00000000 DMA2_STREAM6
08004288 g     F .text	00000000 DMA2_STREAM7
08004288 g     F .text	00000000 USART6
08004288 g     F .text	00000000 I2C3_EV
08004288 g     F .text	00000000 I2C3_ER
08004288 g     F .text	00000000 OTG_HS_EP1_OUT
08004288 g     F .text	00000000 OTG_HS_EP1_IN
08004288 g     F .text	00000000 OTG_HS_WKUP
08004288 g     F .text	00000000 OTG_HS
08004288 g     F .text	00000000 DCMI
08004288 g     F .text	00000000 CRYP
08004288 g     F .text	00000000 HASH_RNG
08004288 g     F .text	00000000 FPU
08004288 g     F .text	00000000 UART7
08004288 g     F .text	00000000 UART8
08004288 g     F .text	00000000 SPI4
08004288 g     F .text	00000000 SPI5
08004288 g     F .text	00000000 SPI6
08004288 g     F .text	00000000 SAI1
08004288 g     F .text	00000000 LTDC
08004288 g     F .text	00000000 LTDC_ER
08004288 g     F .text	00000000 DMA2D
08004288 g     F .text	00000000 SAI2
08004288 g     F .text	00000000 QUADSPI
08004288 g     F .text	00000000 LP_TIMER1
08004288 g     F .text	00000000 HDMI_CEC
08004288 g     F .text	00000000 I2C4_EV
08004288 g     F .text	00000000 I2C4_ER
08004288 g     F .text	00000000 SPDIFRX
08004288 g     F .text	00000000 DSIHOST
08004288 g     F .text	00000000 DFSDM1_FLT0
08004288 g     F .text	00000000 DFSDM1_FLT1
08004288 g     F .text	00000000 DFSDM1_FLT2
08004288 g     F .text	00000000 DFSDM1_FLT3
08004288 g     F .text	00000000 SDMMC2
08004288 g     F .text	00000000 CAN3_TX
08004288 g     F .text	00000000 CAN3_RX0
08004288 g     F .text	00000000 CAN3_RX1
08004288 g     F .text	00000000 CAN3_SCE
08004288 g     F .text	00000000 JPEG
08004288 g     F .text	00000000 MDIOS
20000038 g       .bss	00000000 __sbss
20000074 g       .bss	00000000 __ebss
20000000 g       .data	00000000 __sdata
20000038 g       .data	00000000 __edata
0800b148 g       *ABS*	00000000 __sidata
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
08009830 g       .text	00000000 __etext
08009830 g       .rodata	00000000 __srodata
0800b148 g       .rodata	00000000 __erodata
0800b180 g       .gnu.sgstubs	00000000 __veneer_base
0800b180 g       .gnu.sgstubs	00000000 __veneer_limit
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
 80001f8:	f004 f849 	bl	800428e <DefaultPreInit>
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
 8000230:	f004 fba2 	bl	8004978 <main>
 8000234:	de00      	udf	#0
 8000236:	0000      	movs	r0, r0
 8000238:	20000038 	.word	0x20000038
 800023c:	20000074 	.word	0x20000074
 8000240:	20000000 	.word	0x20000000
 8000244:	20000038 	.word	0x20000038
 8000248:	0800b148 	.word	0x0800b148
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
 80002c2:	f649 0390 	movw	r3, #39056	@ 0x9890
 80002c6:	2018      	movs	r0, #24
 80002c8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80002cc:	211c      	movs	r1, #28
 80002ce:	f003 fa08 	bl	80036e2 <core::slice::index::slice_index_fail>
 80002d2:	2301      	movs	r3, #1
 80002d4:	ba5b      	rev16	r3, r3
 80002d6:	2a0d      	cmp	r2, #13
 80002d8:	80cb      	strh	r3, [r1, #6]
 80002da:	d8dc      	bhi.n	8000296 <smoltcp::wire::arp::Repr::emit+0x46>
 80002dc:	f649 0360 	movw	r3, #39008	@ 0x9860
 80002e0:	2008      	movs	r0, #8
 80002e2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80002e6:	210e      	movs	r1, #14
 80002e8:	f003 f9fb 	bl	80036e2 <core::slice::index::slice_index_fail>
 80002ec:	f649 0340 	movw	r3, #38976	@ 0x9840
 80002f0:	2000      	movs	r0, #0
 80002f2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80002f6:	2102      	movs	r1, #2
 80002f8:	f003 f9f3 	bl	80036e2 <core::slice::index::slice_index_fail>
 80002fc:	f649 0350 	movw	r3, #38992	@ 0x9850
 8000300:	2002      	movs	r0, #2
 8000302:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000306:	2104      	movs	r1, #4
 8000308:	f003 f9eb 	bl	80036e2 <core::slice::index::slice_index_fail>
 800030c:	f649 0330 	movw	r3, #38960	@ 0x9830
 8000310:	2006      	movs	r0, #6
 8000312:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000316:	2108      	movs	r1, #8
 8000318:	f003 f9e3 	bl	80036e2 <core::slice::index::slice_index_fail>
 800031c:	f649 0370 	movw	r3, #39024	@ 0x9870
 8000320:	200e      	movs	r0, #14
 8000322:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000326:	2112      	movs	r1, #18
 8000328:	f003 f9db 	bl	80036e2 <core::slice::index::slice_index_fail>
 800032c:	f649 0380 	movw	r3, #39040	@ 0x9880
 8000330:	2012      	movs	r0, #18
 8000332:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000336:	2118      	movs	r1, #24
 8000338:	f003 f9d3 	bl	80036e2 <core::slice::index::slice_index_fail>
 800033c:	f649 02a0 	movw	r2, #39072	@ 0x98a0
 8000340:	2004      	movs	r0, #4
 8000342:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000346:	2104      	movs	r1, #4
 8000348:	f003 fa4a 	bl	80037e0 <core::panicking::panic_bounds_check>
 800034c:	f649 02b0 	movw	r2, #39088	@ 0x98b0
 8000350:	2005      	movs	r0, #5
 8000352:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000356:	2105      	movs	r1, #5
 8000358:	f003 fa42 	bl	80037e0 <core::panicking::panic_bounds_check>

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
 8000460:	f003 ff7b 	bl	800435a <defmt::export::acquire_header_and_release>
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
 80005ea:	f007 fe8e 	bl	800830a <smoltcp::iface::route::Routes::lookup>
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
 8000632:	f64a 7004 	movw	r0, #44804	@ 0xaf04
 8000636:	f64a 7230 	movw	r2, #44848	@ 0xaf30
 800063a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800063e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000642:	212c      	movs	r1, #44	@ 0x2c
 8000644:	f003 f8e0 	bl	8003808 <core::panicking::panic>
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
 8000702:	f007 fc9c 	bl	800803e <smoltcp::wire::ip::checksum::data>
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
 8000788:	f008 fdb3 	bl	80092f2 <__aeabi_memcpy>
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
 80007e6:	f007 fc2a 	bl	800803e <smoltcp::wire::ip::checksum::data>
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
 8000832:	f008 f834 	bl	800889e <__aeabi_memcpy4>
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
 80009be:	f007 f8dd 	bl	8007b7c <smoltcp::wire::ipv4::Repr::emit>
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
 8000a0c:	f007 f8b6 	bl	8007b7c <smoltcp::wire::ipv4::Repr::emit>
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
 8000a5c:	f008 fc49 	bl	80092f2 <__aeabi_memcpy>
 8000a60:	2000      	movs	r0, #0
 8000a62:	f018 0ffd 	tst.w	r8, #253	@ 0xfd
 8000a66:	d107      	bne.n	8000a78 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x71c>
 8000a68:	84a8      	strh	r0, [r5, #36]	@ 0x24
 8000a6a:	4650      	mov	r0, sl
 8000a6c:	4649      	mov	r1, r9
 8000a6e:	f007 fae6 	bl	800803e <smoltcp::wire::ip::checksum::data>
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
 8000aac:	f64a 13d8 	movw	r3, #43480	@ 0xa9d8
 8000ab0:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ab4:	2014      	movs	r0, #20
 8000ab6:	f002 fe14 	bl	80036e2 <core::slice::index::slice_index_fail>
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
 8000ad8:	f005 fb66 	bl	80061a8 <smoltcp::wire::tcp::TcpOption::emit>
 8000adc:	4603      	mov	r3, r0
 8000ade:	460a      	mov	r2, r1
 8000ae0:	f1bb 0f00 	cmp.w	fp, #0
 8000ae4:	d10e      	bne.n	8000b04 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7a8>
 8000ae6:	f8dd b040 	ldr.w	fp, [sp, #64]	@ 0x40
 8000aea:	f1ba 0f00 	cmp.w	sl, #0
 8000aee:	d01a      	beq.n	8000b26 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x7ca>
 8000af0:	f64a 2468 	movw	r4, #43624	@ 0xaa68
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
 8000b14:	f005 fb48 	bl	80061a8 <smoltcp::wire::tcp::TcpOption::emit>
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
 8000b3e:	f005 fb33 	bl	80061a8 <smoltcp::wire::tcp::TcpOption::emit>
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
 8000b5c:	f005 fb24 	bl	80061a8 <smoltcp::wire::tcp::TcpOption::emit>
 8000b60:	4603      	mov	r3, r0
 8000b62:	460a      	mov	r2, r1
 8000b64:	9c11      	ldr	r4, [sp, #68]	@ 0x44
 8000b66:	b132      	cbz	r2, 8000b76 <smoltcp::iface::interface::InterfaceInner::dispatch_ip+0x81a>
 8000b68:	f64a 208c 	movw	r0, #43660	@ 0xaa8c
 8000b6c:	4619      	mov	r1, r3
 8000b6e:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8000b72:	f005 fb19 	bl	80061a8 <smoltcp::wire::tcp::TcpOption::emit>
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
 8000b9c:	f008 fba9 	bl	80092f2 <__aeabi_memcpy>
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
 8000bfa:	f007 fa20 	bl	800803e <smoltcp::wire::ip::checksum::data>
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
 8000cee:	f003 fb23 	bl	8004338 <defmt::export::acquire_and_header>
 8000cf2:	f240 0007 	movw	r0, #7
 8000cf6:	2102      	movs	r1, #2
 8000cf8:	f2c0 0000 	movt	r0, #0
 8000cfc:	f8ad 004c 	strh.w	r0, [sp, #76]	@ 0x4c
 8000d00:	a813      	add	r0, sp, #76	@ 0x4c
 8000d02:	f003 fccb 	bl	800469c <_defmt_write>
 8000d06:	a812      	add	r0, sp, #72	@ 0x48
 8000d08:	f003 fac4 	bl	8004294 <defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>
 8000d0c:	a813      	add	r0, sp, #76	@ 0x4c
 8000d0e:	2600      	movs	r6, #0
 8000d10:	2102      	movs	r1, #2
 8000d12:	f8ad 604c 	strh.w	r6, [sp, #76]	@ 0x4c
 8000d16:	f003 fcc1 	bl	800469c <_defmt_write>
 8000d1a:	f003 fc47 	bl	80045ac <_defmt_release>
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
 8000e4e:	f24a 4010 	movw	r0, #42000	@ 0xa410
 8000e52:	f24a 4248 	movw	r2, #42056	@ 0xa448
 8000e56:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8000e5a:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000e5e:	2136      	movs	r1, #54	@ 0x36
 8000e60:	f002 fcd2 	bl	8003808 <core::panicking::panic>
 8000e64:	f64a 70b0 	movw	r0, #44976	@ 0xafb0
 8000e68:	f64a 72d0 	movw	r2, #45008	@ 0xafd0
 8000e6c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8000e70:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8000e74:	211d      	movs	r1, #29
 8000e76:	f002 fcc7 	bl	8003808 <core::panicking::panic>
 8000e7a:	f64a 03e4 	movw	r3, #43236	@ 0xa8e4
 8000e7e:	2006      	movs	r0, #6
 8000e80:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000e84:	210c      	movs	r1, #12
 8000e86:	4662      	mov	r2, ip
 8000e88:	f002 fc2b 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000e8c:	f64a 03f4 	movw	r3, #43252	@ 0xa8f4
 8000e90:	200c      	movs	r0, #12
 8000e92:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000e96:	210e      	movs	r1, #14
 8000e98:	4662      	mov	r2, ip
 8000e9a:	f002 fc22 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000e9e:	f64a 5304 	movw	r3, #44292	@ 0xad04
 8000ea2:	2002      	movs	r0, #2
 8000ea4:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ea8:	2104      	movs	r1, #4
 8000eaa:	4672      	mov	r2, lr
 8000eac:	f002 fc19 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000eb0:	f64a 5344 	movw	r3, #44356	@ 0xad44
 8000eb4:	2004      	movs	r0, #4
 8000eb6:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000eba:	2106      	movs	r1, #6
 8000ebc:	4672      	mov	r2, lr
 8000ebe:	f002 fc10 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000ec2:	f64a 43c4 	movw	r3, #44228	@ 0xacc4
 8000ec6:	2006      	movs	r0, #6
 8000ec8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ecc:	2108      	movs	r1, #8
 8000ece:	4672      	mov	r2, lr
 8000ed0:	f002 fc07 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000ed4:	f64a 43f4 	movw	r3, #44276	@ 0xacf4
 8000ed8:	200c      	movs	r0, #12
 8000eda:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ede:	2110      	movs	r1, #16
 8000ee0:	4672      	mov	r2, lr
 8000ee2:	f002 fbfe 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000ee6:	f64a 43e4 	movw	r3, #44260	@ 0xace4
 8000eea:	2010      	movs	r0, #16
 8000eec:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ef0:	2114      	movs	r1, #20
 8000ef2:	4672      	mov	r2, lr
 8000ef4:	f002 fbf5 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000ef8:	f64a 6350 	movw	r3, #44624	@ 0xae50
 8000efc:	2008      	movs	r0, #8
 8000efe:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f02:	4631      	mov	r1, r6
 8000f04:	464a      	mov	r2, r9
 8000f06:	f002 fbec 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000f0a:	f64a 6380 	movw	r3, #44672	@ 0xae80
 8000f0e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f12:	2000      	movs	r0, #0
 8000f14:	2102      	movs	r1, #2
 8000f16:	464a      	mov	r2, r9
 8000f18:	f002 fbe3 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000f1c:	f64a 2308 	movw	r3, #43528	@ 0xaa08
 8000f20:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f24:	2000      	movs	r0, #0
 8000f26:	2102      	movs	r1, #2
 8000f28:	464a      	mov	r2, r9
 8000f2a:	f002 fbda 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000f2e:	f64a 6370 	movw	r3, #44656	@ 0xae70
 8000f32:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f36:	2002      	movs	r0, #2
 8000f38:	2104      	movs	r1, #4
 8000f3a:	464a      	mov	r2, r9
 8000f3c:	f002 fbd1 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000f40:	f64a 13f8 	movw	r3, #43512	@ 0xa9f8
 8000f44:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f48:	2002      	movs	r0, #2
 8000f4a:	2104      	movs	r1, #4
 8000f4c:	464a      	mov	r2, r9
 8000f4e:	f002 fbc8 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000f52:	f64a 6390 	movw	r3, #44688	@ 0xae90
 8000f56:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f5a:	2004      	movs	r0, #4
 8000f5c:	2106      	movs	r1, #6
 8000f5e:	464a      	mov	r2, r9
 8000f60:	f002 fbbf 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000f64:	f64a 2338 	movw	r3, #43576	@ 0xaa38
 8000f68:	2004      	movs	r0, #4
 8000f6a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f6e:	2108      	movs	r1, #8
 8000f70:	464a      	mov	r2, r9
 8000f72:	f002 fbb6 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000f76:	f64a 2328 	movw	r3, #43560	@ 0xaa28
 8000f7a:	2008      	movs	r0, #8
 8000f7c:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f80:	210c      	movs	r1, #12
 8000f82:	464a      	mov	r2, r9
 8000f84:	f002 fbad 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000f88:	9a11      	ldr	r2, [sp, #68]	@ 0x44
 8000f8a:	f64a 2348 	movw	r3, #43592	@ 0xaa48
 8000f8e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000f92:	200e      	movs	r0, #14
 8000f94:	2110      	movs	r1, #16
 8000f96:	f002 fba4 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000f9a:	f64a 63a0 	movw	r3, #44704	@ 0xaea0
 8000f9e:	4611      	mov	r1, r2
 8000fa0:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fa4:	461a      	mov	r2, r3
 8000fa6:	f003 f829 	bl	8003ffc <core::slice::copy_from_slice_impl::len_mismatch_fail>
 8000faa:	f64a 6360 	movw	r3, #44640	@ 0xae60
 8000fae:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fb2:	2006      	movs	r0, #6
 8000fb4:	2108      	movs	r1, #8
 8000fb6:	464a      	mov	r2, r9
 8000fb8:	f002 fb93 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000fbc:	f64a 23b0 	movw	r3, #43696	@ 0xaab0
 8000fc0:	4611      	mov	r1, r2
 8000fc2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fc6:	2000      	movs	r0, #0
 8000fc8:	4632      	mov	r2, r6
 8000fca:	f002 fb8a 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000fce:	f64a 33d8 	movw	r3, #43992	@ 0xabd8
 8000fd2:	2008      	movs	r0, #8
 8000fd4:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fd8:	4649      	mov	r1, r9
 8000fda:	464a      	mov	r2, r9
 8000fdc:	f002 fb81 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000fe0:	f64a 33b8 	movw	r3, #43960	@ 0xabb8
 8000fe4:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000fe8:	2004      	movs	r0, #4
 8000fea:	2106      	movs	r1, #6
 8000fec:	464a      	mov	r2, r9
 8000fee:	f002 fb78 	bl	80036e2 <core::slice::index::slice_index_fail>
 8000ff2:	f64a 33c8 	movw	r3, #43976	@ 0xabc8
 8000ff6:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8000ffa:	2006      	movs	r0, #6
 8000ffc:	2108      	movs	r1, #8
 8000ffe:	464a      	mov	r2, r9
 8001000:	f002 fb6f 	bl	80036e2 <core::slice::index::slice_index_fail>
 8001004:	f64a 2318 	movw	r3, #43544	@ 0xaa18
 8001008:	2012      	movs	r0, #18
 800100a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800100e:	2114      	movs	r1, #20
 8001010:	4622      	mov	r2, r4
 8001012:	f002 fb66 	bl	80036e2 <core::slice::index::slice_index_fail>
 8001016:	f64a 13e8 	movw	r3, #43496	@ 0xa9e8
 800101a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800101e:	4621      	mov	r1, r4
 8001020:	4622      	mov	r2, r4
 8001022:	f002 fb5e 	bl	80036e2 <core::slice::index::slice_index_fail>
 8001026:	f64a 4308 	movw	r3, #44040	@ 0xac08
 800102a:	2014      	movs	r0, #20
 800102c:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8001030:	4621      	mov	r1, r4
 8001032:	4622      	mov	r2, r4
 8001034:	f002 fb55 	bl	80036e2 <core::slice::index::slice_index_fail>
 8001038:	f64a 4328 	movw	r3, #44072	@ 0xac28
 800103c:	2014      	movs	r0, #20
 800103e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8001042:	4631      	mov	r1, r6
 8001044:	4632      	mov	r2, r6
 8001046:	f002 fb4c 	bl	80036e2 <core::slice::index::slice_index_fail>
 800104a:	f64a 33f8 	movw	r3, #44024	@ 0xabf8
 800104e:	4610      	mov	r0, r2
 8001050:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8001054:	4631      	mov	r1, r6
 8001056:	461a      	mov	r2, r3
 8001058:	f002 ffd0 	bl	8003ffc <core::slice::copy_from_slice_impl::len_mismatch_fail>
 800105c:	f64a 4218 	movw	r2, #44056	@ 0xac18
 8001060:	4621      	mov	r1, r4
 8001062:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8001066:	f002 ffc9 	bl	8003ffc <core::slice::copy_from_slice_impl::len_mismatch_fail>
 800106a:	f64a 7290 	movw	r2, #44944	@ 0xaf90
 800106e:	4630      	mov	r0, r6
 8001070:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8001074:	f002 fbb4 	bl	80037e0 <core::panicking::panic_bounds_check>
 8001078:	2002      	movs	r0, #2
 800107a:	9013      	str	r0, [sp, #76]	@ 0x4c
 800107c:	a813      	add	r0, sp, #76	@ 0x4c
 800107e:	f004 ff68 	bl	8005f52 <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>
 8001082:	f64a 1004 	movw	r0, #43268	@ 0xa904
 8001086:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800108a:	f002 fb20 	bl	80036ce <core::option::unwrap_failed>
 800108e:	f64a 42d4 	movw	r2, #44244	@ 0xacd4
 8001092:	2000      	movs	r0, #0
 8001094:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8001098:	2100      	movs	r1, #0
 800109a:	f002 fba1 	bl	80037e0 <core::panicking::panic_bounds_check>
 800109e:	f64a 5234 	movw	r2, #44340	@ 0xad34
 80010a2:	2001      	movs	r0, #1
 80010a4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010a8:	2101      	movs	r1, #1
 80010aa:	f002 fb99 	bl	80037e0 <core::panicking::panic_bounds_check>
 80010ae:	f64a 5254 	movw	r2, #44372	@ 0xad54
 80010b2:	2008      	movs	r0, #8
 80010b4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010b8:	2108      	movs	r1, #8
 80010ba:	f002 fb91 	bl	80037e0 <core::panicking::panic_bounds_check>
 80010be:	f64a 5214 	movw	r2, #44308	@ 0xad14
 80010c2:	2009      	movs	r0, #9
 80010c4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010c8:	2109      	movs	r1, #9
 80010ca:	f002 fb89 	bl	80037e0 <core::panicking::panic_bounds_check>
 80010ce:	f64a 7250 	movw	r2, #44880	@ 0xaf50
 80010d2:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010d6:	f002 fb83 	bl	80037e0 <core::panicking::panic_bounds_check>
 80010da:	f64a 32e8 	movw	r2, #44008	@ 0xabe8
 80010de:	2001      	movs	r0, #1
 80010e0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010e4:	4649      	mov	r1, r9
 80010e6:	f002 fb7b 	bl	80037e0 <core::panicking::panic_bounds_check>
 80010ea:	f64a 7290 	movw	r2, #44944	@ 0xaf90
 80010ee:	4620      	mov	r0, r4
 80010f0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80010f4:	f002 fb74 	bl	80037e0 <core::panicking::panic_bounds_check>
 80010f8:	2002      	movs	r0, #2
 80010fa:	9028      	str	r0, [sp, #160]	@ 0xa0
 80010fc:	a828      	add	r0, sp, #160	@ 0xa0
 80010fe:	f004 ff28 	bl	8005f52 <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>
 8001102:	f64a 1004 	movw	r0, #43268	@ 0xa904
 8001106:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800110a:	f002 fae0 	bl	80036ce <core::option::unwrap_failed>
 800110e:	f64a 7250 	movw	r2, #44880	@ 0xaf50
 8001112:	4620      	mov	r0, r4
 8001114:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8001118:	f002 fb62 	bl	80037e0 <core::panicking::panic_bounds_check>

0800111c <smoltcp::iface::interface::Interface::poll>:
 800111c:	b5f0      	push	{r4, r5, r6, r7, lr}
 800111e:	af03      	add	r7, sp, #12
 8001120:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8001124:	f5ad 7d5b 	sub.w	sp, sp, #876	@ 0x36c
 8001128:	aebc      	add	r6, sp, #752	@ 0x2f0
 800112a:	f500 78b8 	add.w	r8, r0, #368	@ 0x170
 800112e:	1c71      	adds	r1, r6, #1
 8001130:	913d      	str	r1, [sp, #244]	@ 0xf4
 8001132:	a98a      	add	r1, sp, #552	@ 0x228
 8001134:	e9c0 2304 	strd	r2, r3, [r0, #16]
 8001138:	f101 0510 	add.w	r5, r1, #16
 800113c:	9534      	str	r5, [sp, #208]	@ 0xd0
 800113e:	f106 0554 	add.w	r5, r6, #84	@ 0x54
 8001142:	9530      	str	r5, [sp, #192]	@ 0xc0
 8001144:	ada3      	add	r5, sp, #652	@ 0x28c
 8001146:	9048      	str	r0, [sp, #288]	@ 0x120
 8001148:	f105 0454 	add.w	r4, r5, #84	@ 0x54
 800114c:	942f      	str	r4, [sp, #188]	@ 0xbc
 800114e:	f101 040d 	add.w	r4, r1, #13
 8001152:	3124      	adds	r1, #36	@ 0x24
 8001154:	9131      	str	r1, [sp, #196]	@ 0xc4
 8001156:	a952      	add	r1, sp, #328	@ 0x148
 8001158:	3102      	adds	r1, #2
 800115a:	9138      	str	r1, [sp, #224]	@ 0xe0
 800115c:	f500 7180 	add.w	r1, r0, #256	@ 0x100
 8001160:	9133      	str	r1, [sp, #204]	@ 0xcc
 8001162:	f100 0120 	add.w	r1, r0, #32
 8001166:	913c      	str	r1, [sp, #240]	@ 0xf0
 8001168:	f100 01f4 	add.w	r1, r0, #244	@ 0xf4
 800116c:	9142      	str	r1, [sp, #264]	@ 0x108
 800116e:	f500 71b4 	add.w	r1, r0, #360	@ 0x168
 8001172:	a855      	add	r0, sp, #340	@ 0x154
 8001174:	9144      	str	r1, [sp, #272]	@ 0x110
 8001176:	f100 011c 	add.w	r1, r0, #28
 800117a:	9140      	str	r1, [sp, #256]	@ 0x100
 800117c:	f100 0113 	add.w	r1, r0, #19
 8001180:	3006      	adds	r0, #6
 8001182:	903e      	str	r0, [sp, #248]	@ 0xf8
 8001184:	f105 000f 	add.w	r0, r5, #15
 8001188:	9037      	str	r0, [sp, #220]	@ 0xdc
 800118a:	1ca8      	adds	r0, r5, #2
 800118c:	9036      	str	r0, [sp, #216]	@ 0xd8
 800118e:	f106 001c 	add.w	r0, r6, #28
 8001192:	903b      	str	r0, [sp, #236]	@ 0xec
 8001194:	f106 0013 	add.w	r0, r6, #19
 8001198:	903a      	str	r0, [sp, #232]	@ 0xe8
 800119a:	1db0      	adds	r0, r6, #6
 800119c:	9039      	str	r0, [sp, #228]	@ 0xe4
 800119e:	f8d7 9008 	ldr.w	r9, [r7, #8]
 80011a2:	9432      	str	r4, [sp, #200]	@ 0xc8
 80011a4:	f109 0a0c 	add.w	sl, r9, #12
 80011a8:	e9cd 2345 	strd	r2, r3, [sp, #276]	@ 0x114
 80011ac:	913f      	str	r1, [sp, #252]	@ 0xfc
 80011ae:	f8cd a104 	str.w	sl, [sp, #260]	@ 0x104
 80011b2:	f8cd 811c 	str.w	r8, [sp, #284]	@ 0x11c
 80011b6:	2000      	movs	r0, #0
 80011b8:	9043      	str	r0, [sp, #268]	@ 0x10c
 80011ba:	2000      	movs	r0, #0
 80011bc:	9035      	str	r0, [sp, #212]	@ 0xd4
 80011be:	e005      	b.n	80011cc <smoltcp::iface::interface::Interface::poll+0xb0>
 80011c0:	f240 002f 	movw	r0, #47	@ 0x2f
 80011c4:	f2c0 0000 	movt	r0, #0
 80011c8:	f003 f8c7 	bl	800435a <defmt::export::acquire_header_and_release>
 80011cc:	e9d9 1004 	ldrd	r1, r0, [r9, #16]
 80011d0:	4288      	cmp	r0, r1
 80011d2:	f082 806a 	bcs.w	80032aa <smoltcp::iface::interface::Interface::poll+0x218e>
 80011d6:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 80011da:	4348      	muls	r0, r1
 80011dc:	f8da 1000 	ldr.w	r1, [sl]
 80011e0:	5808      	ldr	r0, [r1, r0]
 80011e2:	2800      	cmp	r0, #0
 80011e4:	f101 81d8 	bmi.w	8002598 <smoltcp::iface::interface::Interface::poll+0x147c>
 80011e8:	f249 0014 	movw	r0, #36884	@ 0x9014
 80011ec:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 80011f0:	6800      	ldr	r0, [r0, #0]
 80011f2:	f3c0 4042 	ubfx	r0, r0, #17, #3
 80011f6:	2807      	cmp	r0, #7
 80011f8:	d87c      	bhi.n	80012f4 <smoltcp::iface::interface::Interface::poll+0x1d8>
 80011fa:	2101      	movs	r1, #1
 80011fc:	fa01 f000 	lsl.w	r0, r1, r0
 8001200:	f010 0faa 	tst.w	r0, #170	@ 0xaa
 8001204:	bf08      	it	eq
 8001206:	f007 fad8 	bleq	80087ba <stm32_eth::dma::rx::RxRing::demand_poll>
 800120a:	e9d9 6501 	ldrd	r6, r5, [r9, #4]
 800120e:	42b5      	cmp	r5, r6
 8001210:	f082 8051 	bcs.w	80032b6 <smoltcp::iface::interface::Interface::poll+0x219a>
 8001214:	f44f 60c8 	mov.w	r0, #1600	@ 0x640
 8001218:	f8d9 b000 	ldr.w	fp, [r9]
 800121c:	4368      	muls	r0, r5
 800121e:	f85b 0000 	ldr.w	r0, [fp, r0]
 8001222:	2800      	cmp	r0, #0
 8001224:	f101 81b8 	bmi.w	8002598 <smoltcp::iface::interface::Interface::poll+0x147c>
 8001228:	f8d9 4018 	ldr.w	r4, [r9, #24]
 800122c:	f8cd a130 	str.w	sl, [sp, #304]	@ 0x130
 8001230:	1c60      	adds	r0, r4, #1
 8001232:	f8c9 0018 	str.w	r0, [r9, #24]
 8001236:	2000      	movs	r0, #0
 8001238:	9449      	str	r4, [sp, #292]	@ 0x124
 800123a:	904a      	str	r0, [sp, #296]	@ 0x128
 800123c:	a849      	add	r0, sp, #292	@ 0x124
 800123e:	9051      	str	r0, [sp, #324]	@ 0x144
 8001240:	68f8      	ldr	r0, [r7, #12]
 8001242:	9050      	str	r0, [sp, #320]	@ 0x140
 8001244:	f249 0014 	movw	r0, #36884	@ 0x9014
 8001248:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 800124c:	6800      	ldr	r0, [r0, #0]
 800124e:	9948      	ldr	r1, [sp, #288]	@ 0x120
 8001250:	e9cd 884e 	strd	r8, r8, [sp, #312]	@ 0x138
 8001254:	f3c0 4042 	ubfx	r0, r0, #17, #3
 8001258:	914d      	str	r1, [sp, #308]	@ 0x134
 800125a:	2807      	cmp	r0, #7
 800125c:	d852      	bhi.n	8001304 <smoltcp::iface::interface::Interface::poll+0x1e8>
 800125e:	2101      	movs	r1, #1
 8001260:	fa01 f000 	lsl.w	r0, r1, r0
 8001264:	f010 0faa 	tst.w	r0, #170	@ 0xaa
 8001268:	bf08      	it	eq
 800126a:	f007 faa6 	bleq	80087ba <stm32_eth::dma::rx::RxRing::demand_poll>
 800126e:	f44f 60c8 	mov.w	r0, #1600	@ 0x640
 8001272:	fb05 b000 	mla	r0, r5, r0, fp
 8001276:	6801      	ldr	r1, [r0, #0]
 8001278:	2900      	cmp	r1, #0
 800127a:	d4a1      	bmi.n	80011c0 <smoltcp::iface::interface::Interface::poll+0xa4>
 800127c:	6801      	ldr	r1, [r0, #0]
 800127e:	f411 4100 	ands.w	r1, r1, #32768	@ 0x8000
 8001282:	d106      	bne.n	8001292 <smoltcp::iface::interface::Interface::poll+0x176>
 8001284:	6802      	ldr	r2, [r0, #0]
 8001286:	0592      	lsls	r2, r2, #22
 8001288:	bf44      	itt	mi
 800128a:	6802      	ldrmi	r2, [r0, #0]
 800128c:	ea5f 52c2 	movsmi.w	r2, r2, lsl #23
 8001290:	d43b      	bmi.n	800130a <smoltcp::iface::interface::Interface::poll+0x1ee>
 8001292:	6a02      	ldr	r2, [r0, #32]
 8001294:	2a00      	cmp	r2, #0
 8001296:	f001 879e 	beq.w	80031d6 <smoltcp::iface::interface::Interface::poll+0x20ba>
 800129a:	e9d0 2309 	ldrd	r2, r3, [r0, #36]	@ 0x24
 800129e:	2b00      	cmp	r3, #0
 80012a0:	6082      	str	r2, [r0, #8]
 80012a2:	f001 87a3 	beq.w	80031ec <smoltcp::iface::interface::Interface::poll+0x20d0>
 80012a6:	6ac2      	ldr	r2, [r0, #44]	@ 0x2c
 80012a8:	0bcc      	lsrs	r4, r1, #15
 80012aa:	60c2      	str	r2, [r0, #12]
 80012ac:	f04f 4200 	mov.w	r2, #2147483648	@ 0x80000000
 80012b0:	f3bf 8f5f 	dmb	sy
 80012b4:	6002      	str	r2, [r0, #0]
 80012b6:	2c02      	cmp	r4, #2
 80012b8:	f3bf 8f5f 	dmb	sy
 80012bc:	f43f af80 	beq.w	80011c0 <smoltcp::iface::interface::Interface::poll+0xa4>
 80012c0:	f240 002a 	movw	r0, #42	@ 0x2a
 80012c4:	f2c0 0000 	movt	r0, #0
 80012c8:	f003 f836 	bl	8004338 <defmt::export::acquire_and_header>
 80012cc:	f240 0041 	movw	r0, #65	@ 0x41
 80012d0:	adbc      	add	r5, sp, #752	@ 0x2f0
 80012d2:	f2c0 0000 	movt	r0, #0
 80012d6:	2102      	movs	r1, #2
 80012d8:	f8ad 02f0 	strh.w	r0, [sp, #752]	@ 0x2f0
 80012dc:	4628      	mov	r0, r5
 80012de:	f003 f9dd 	bl	800469c <_defmt_write>
 80012e2:	4628      	mov	r0, r5
 80012e4:	2101      	movs	r1, #1
 80012e6:	f88d 42f0 	strb.w	r4, [sp, #752]	@ 0x2f0
 80012ea:	f003 f9d7 	bl	800469c <_defmt_write>
 80012ee:	f003 f95d 	bl	80045ac <_defmt_release>
 80012f2:	e76b      	b.n	80011cc <smoltcp::iface::interface::Interface::poll+0xb0>
 80012f4:	f007 fa61 	bl	80087ba <stm32_eth::dma::rx::RxRing::demand_poll>
 80012f8:	e9d9 6501 	ldrd	r6, r5, [r9, #4]
 80012fc:	42b5      	cmp	r5, r6
 80012fe:	d389      	bcc.n	8001214 <smoltcp::iface::interface::Interface::poll+0xf8>
 8001300:	f001 bfd9 	b.w	80032b6 <smoltcp::iface::interface::Interface::poll+0x219a>
 8001304:	f007 fa59 	bl	80087ba <stm32_eth::dma::rx::RxRing::demand_poll>
 8001308:	e7b1      	b.n	800126e <smoltcp::iface::interface::Interface::poll+0x152>
 800130a:	f8d0 c000 	ldr.w	ip, [r0]
 800130e:	f8d0 e000 	ldr.w	lr, [r0]
 8001312:	2200      	movs	r2, #0
 8001314:	69c1      	ldr	r1, [r0, #28]
 8001316:	f8d0 a018 	ldr.w	sl, [r0, #24]
 800131a:	f02a 4300 	bic.w	r3, sl, #2147483648	@ 0x80000000
 800131e:	ea43 73c1 	orr.w	r3, r3, r1, lsl #31
 8001322:	f1d3 0900 	rsbs	r9, r3, #0
 8001326:	eb62 0851 	sbc.w	r8, r2, r1, lsr #1
 800132a:	f1ba 0f00 	cmp.w	sl, #0
 800132e:	bf5c      	itt	pl
 8001330:	4699      	movpl	r9, r3
 8001332:	ea4f 0851 	movpl.w	r8, r1, lsr #1
 8001336:	ea5f 610e 	movs.w	r1, lr, lsl #24
 800133a:	f8d7 a008 	ldr.w	sl, [r7, #8]
 800133e:	bf41      	itttt	mi
 8001340:	6801      	ldrmi	r1, [r0, #0]
 8001342:	ea5f 51c1 	movsmi.w	r1, r1, lsl #23
 8001346:	f04f 0b00 	movmi.w	fp, #0
 800134a:	f04f 0e01 	movmi.w	lr, #1
 800134e:	bf5c      	itt	pl
 8001350:	f04f 0e00 	movpl.w	lr, #0
 8001354:	f04f 0b00 	movpl.w	fp, #0
 8001358:	e9da 1201 	ldrd	r1, r2, [sl, #4]
 800135c:	3201      	adds	r2, #1
 800135e:	e9c0 b90f 	strd	fp, r9, [r0, #60]	@ 0x3c
 8001362:	fbb2 f3f6 	udiv	r3, r2, r6
 8001366:	428d      	cmp	r5, r1
 8001368:	fb03 2216 	mls	r2, r3, r6, r2
 800136c:	f04f 0301 	mov.w	r3, #1
 8001370:	f100 0630 	add.w	r6, r0, #48	@ 0x30
 8001374:	e886 4018 	stmia.w	r6, {r3, r4, lr}
 8001378:	f8c0 8044 	str.w	r8, [r0, #68]	@ 0x44
 800137c:	f8ca 2008 	str.w	r2, [sl, #8]
 8001380:	f081 87af 	bcs.w	80032e2 <smoltcp::iface::interface::Interface::poll+0x21c6>
 8001384:	f8dd 811c 	ldr.w	r8, [sp, #284]	@ 0x11c
 8001388:	f3cc 410d 	ubfx	r1, ip, #16, #14
 800138c:	f240 50f3 	movw	r0, #1523	@ 0x5f3
 8001390:	4281      	cmp	r1, r0
 8001392:	f081 8736 	bcs.w	8003202 <smoltcp::iface::interface::Interface::poll+0x20e6>
 8001396:	f44f 62c8 	mov.w	r2, #1600	@ 0x640
 800139a:	f8da 0000 	ldr.w	r0, [sl]
 800139e:	fb05 0602 	mla	r6, r5, r2, r0
 80013a2:	46d1      	mov	r9, sl
 80013a4:	b1e1      	cbz	r1, 80013e0 <smoltcp::iface::interface::Interface::poll+0x2c4>
 80013a6:	f106 0248 	add.w	r2, r6, #72	@ 0x48
 80013aa:	290e      	cmp	r1, #14
 80013ac:	4610      	mov	r0, r2
 80013ae:	bf38      	it	cc
 80013b0:	2000      	movcc	r0, #0
 80013b2:	f8dd a104 	ldr.w	sl, [sp, #260]	@ 0x104
 80013b6:	d315      	bcc.n	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 80013b8:	68fb      	ldr	r3, [r7, #12]
 80013ba:	f8dd c124 	ldr.w	ip, [sp, #292]	@ 0x124
 80013be:	e9d3 b800 	ldrd	fp, r8, [r3]
 80013c2:	6813      	ldr	r3, [r2, #0]
 80013c4:	07dd      	lsls	r5, r3, #31
 80013c6:	d121      	bne.n	800140c <smoltcp::iface::interface::Interface::poll+0x2f0>
 80013c8:	9c44      	ldr	r4, [sp, #272]	@ 0x110
 80013ca:	8892      	ldrh	r2, [r2, #4]
 80013cc:	88a5      	ldrh	r5, [r4, #4]
 80013ce:	6824      	ldr	r4, [r4, #0]
 80013d0:	406a      	eors	r2, r5
 80013d2:	4063      	eors	r3, r4
 80013d4:	b292      	uxth	r2, r2
 80013d6:	431a      	orrs	r2, r3
 80013d8:	d018      	beq.n	800140c <smoltcp::iface::interface::Interface::poll+0x2f0>
 80013da:	f8dd 811c 	ldr.w	r8, [sp, #284]	@ 0x11c
 80013de:	e001      	b.n	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 80013e0:	f8dd a104 	ldr.w	sl, [sp, #260]	@ 0x104
 80013e4:	6a30      	ldr	r0, [r6, #32]
 80013e6:	2800      	cmp	r0, #0
 80013e8:	f001 86f5 	beq.w	80031d6 <smoltcp::iface::interface::Interface::poll+0x20ba>
 80013ec:	e9d6 0109 	ldrd	r0, r1, [r6, #36]	@ 0x24
 80013f0:	2900      	cmp	r1, #0
 80013f2:	60b0      	str	r0, [r6, #8]
 80013f4:	f001 86fa 	beq.w	80031ec <smoltcp::iface::interface::Interface::poll+0x20d0>
 80013f8:	6af0      	ldr	r0, [r6, #44]	@ 0x2c
 80013fa:	60f0      	str	r0, [r6, #12]
 80013fc:	f04f 4000 	mov.w	r0, #2147483648	@ 0x80000000
 8001400:	f3bf 8f5f 	dmb	sy
 8001404:	6030      	str	r0, [r6, #0]
 8001406:	f3bf 8f5f 	dmb	sy
 800140a:	e6df      	b.n	80011cc <smoltcp::iface::interface::Interface::poll+0xb0>
 800140c:	f8b6 2054 	ldrh.w	r2, [r6, #84]	@ 0x54
 8001410:	ba12      	rev	r2, r2
 8001412:	0c12      	lsrs	r2, r2, #16
 8001414:	f5b2 6f00 	cmp.w	r2, #2048	@ 0x800
 8001418:	d078      	beq.n	800150c <smoltcp::iface::interface::Interface::poll+0x3f0>
 800141a:	f8dd 811c 	ldr.w	r8, [sp, #284]	@ 0x11c
 800141e:	f640 0006 	movw	r0, #2054	@ 0x806
 8001422:	4282      	cmp	r2, r0
 8001424:	d1de      	bne.n	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 8001426:	f1a1 020e 	sub.w	r2, r1, #14
 800142a:	2a08      	cmp	r2, #8
 800142c:	d3da      	bcc.n	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 800142e:	f896 005a 	ldrb.w	r0, [r6, #90]	@ 0x5a
 8001432:	f896 105b 	ldrb.w	r1, [r6, #91]	@ 0x5b
 8001436:	4408      	add	r0, r1
 8001438:	3004      	adds	r0, #4
 800143a:	ebb2 0f40 	cmp.w	r2, r0, lsl #1
 800143e:	d3d1      	bcc.n	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 8001440:	f896 005a 	ldrb.w	r0, [r6, #90]	@ 0x5a
 8001444:	f896 105b 	ldrb.w	r1, [r6, #91]	@ 0x5b
 8001448:	180b      	adds	r3, r1, r0
 800144a:	3304      	adds	r3, #4
 800144c:	ebb2 0f43 	cmp.w	r2, r3, lsl #1
 8001450:	d3c8      	bcc.n	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 8001452:	2904      	cmp	r1, #4
 8001454:	bf08      	it	eq
 8001456:	2806      	cmpeq	r0, #6
 8001458:	d1c4      	bne.n	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 800145a:	f8b6 3056 	ldrh.w	r3, [r6, #86]	@ 0x56
 800145e:	f5b3 7f80 	cmp.w	r3, #256	@ 0x100
 8001462:	bf04      	itt	eq
 8001464:	f8b6 3058 	ldrheq.w	r3, [r6, #88]	@ 0x58
 8001468:	2b08      	cmpeq	r3, #8
 800146a:	d1bb      	bne.n	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 800146c:	2a0d      	cmp	r2, #13
 800146e:	f241 8745 	bls.w	80032fc <smoltcp::iface::interface::Interface::poll+0x21e0>
 8001472:	310e      	adds	r1, #14
 8001474:	4291      	cmp	r1, r2
 8001476:	f201 8749 	bhi.w	800330c <smoltcp::iface::interface::Interface::poll+0x21f0>
 800147a:	180d      	adds	r5, r1, r0
 800147c:	4295      	cmp	r5, r2
 800147e:	f201 874c 	bhi.w	800331a <smoltcp::iface::interface::Interface::poll+0x21fe>
 8001482:	2a1b      	cmp	r2, #27
 8001484:	f241 8751 	bls.w	800332a <smoltcp::iface::interface::Interface::poll+0x220e>
 8001488:	9848      	ldr	r0, [sp, #288]	@ 0x120
 800148a:	4631      	mov	r1, r6
 800148c:	f896 2063 	ldrb.w	r2, [r6, #99]	@ 0x63
 8001490:	e9d0 ce04 	ldrd	ip, lr, [r0, #16]
 8001494:	f8d0 00f0 	ldr.w	r0, [r0, #240]	@ 0xf0
 8001498:	922c      	str	r2, [sp, #176]	@ 0xb0
 800149a:	f8d6 205f 	ldr.w	r2, [r6, #95]	@ 0x5f
 800149e:	9d42      	ldr	r5, [sp, #264]	@ 0x108
 80014a0:	eb00 0880 	add.w	r8, r0, r0, lsl #2
 80014a4:	922b      	str	r2, [sp, #172]	@ 0xac
 80014a6:	f896 205e 	ldrb.w	r2, [r6, #94]	@ 0x5e
 80014aa:	eb05 0308 	add.w	r3, r5, r8
 80014ae:	f8b6 b05c 	ldrh.w	fp, [r6, #92]	@ 0x5c
 80014b2:	6e76      	ldr	r6, [r6, #100]	@ 0x64
 80014b4:	f8d1 406e 	ldr.w	r4, [r1, #110]	@ 0x6e
 80014b8:	922d      	str	r2, [sp, #180]	@ 0xb4
 80014ba:	4642      	mov	r2, r8
 80014bc:	912e      	str	r1, [sp, #184]	@ 0xb8
 80014be:	2a00      	cmp	r2, #0
 80014c0:	f000 80a4 	beq.w	800160c <smoltcp::iface::interface::Interface::poll+0x4f0>
 80014c4:	4629      	mov	r1, r5
 80014c6:	f851 0b05 	ldr.w	r0, [r1], #5
 80014ca:	42a0      	cmp	r0, r4
 80014cc:	f000 80a3 	beq.w	8001616 <smoltcp::iface::interface::Interface::poll+0x4fa>
 80014d0:	4299      	cmp	r1, r3
 80014d2:	f000 809b 	beq.w	800160c <smoltcp::iface::interface::Interface::poll+0x4f0>
 80014d6:	f8d5 0005 	ldr.w	r0, [r5, #5]
 80014da:	42a0      	cmp	r0, r4
 80014dc:	f000 809b 	beq.w	8001616 <smoltcp::iface::interface::Interface::poll+0x4fa>
 80014e0:	f105 000a 	add.w	r0, r5, #10
 80014e4:	4298      	cmp	r0, r3
 80014e6:	f000 8091 	beq.w	800160c <smoltcp::iface::interface::Interface::poll+0x4f0>
 80014ea:	f8d5 000a 	ldr.w	r0, [r5, #10]
 80014ee:	42a0      	cmp	r0, r4
 80014f0:	f000 8091 	beq.w	8001616 <smoltcp::iface::interface::Interface::poll+0x4fa>
 80014f4:	f105 000f 	add.w	r0, r5, #15
 80014f8:	4298      	cmp	r0, r3
 80014fa:	f000 8087 	beq.w	800160c <smoltcp::iface::interface::Interface::poll+0x4f0>
 80014fe:	f8d5 000f 	ldr.w	r0, [r5, #15]
 8001502:	3a14      	subs	r2, #20
 8001504:	3514      	adds	r5, #20
 8001506:	42a0      	cmp	r0, r4
 8001508:	d1d9      	bne.n	80014be <smoltcp::iface::interface::Interface::poll+0x3a2>
 800150a:	e084      	b.n	8001616 <smoltcp::iface::interface::Interface::poll+0x4fa>
 800150c:	f1a1 050e 	sub.w	r5, r1, #14
 8001510:	2d14      	cmp	r5, #20
 8001512:	f4ff af62 	bcc.w	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 8001516:	f100 030e 	add.w	r3, r0, #14
 800151a:	213c      	movs	r1, #60	@ 0x3c
 800151c:	7818      	ldrb	r0, [r3, #0]
 800151e:	ea01 0280 	and.w	r2, r1, r0, lsl #2
 8001522:	4295      	cmp	r5, r2
 8001524:	f4ff af59 	bcc.w	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 8001528:	f8b6 1058 	ldrh.w	r1, [r6, #88]	@ 0x58
 800152c:	ba09      	rev	r1, r1
 800152e:	ebb2 4f11 	cmp.w	r2, r1, lsr #16
 8001532:	f63f af52 	bhi.w	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 8001536:	0c09      	lsrs	r1, r1, #16
 8001538:	428d      	cmp	r5, r1
 800153a:	f4ff af4e 	bcc.w	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 800153e:	f8b6 1058 	ldrh.w	r1, [r6, #88]	@ 0x58
 8001542:	ba09      	rev	r1, r1
 8001544:	ebb2 4f11 	cmp.w	r2, r1, lsr #16
 8001548:	f63f af47 	bhi.w	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 800154c:	0c0c      	lsrs	r4, r1, #16
 800154e:	42a5      	cmp	r5, r4
 8001550:	f4ff af43 	bcc.w	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 8001554:	f000 01f0 	and.w	r1, r0, #240	@ 0xf0
 8001558:	2940      	cmp	r1, #64	@ 0x40
 800155a:	f47f af3e 	bne.w	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 800155e:	f8b6 1052 	ldrh.w	r1, [r6, #82]	@ 0x52
 8001562:	912a      	str	r1, [sp, #168]	@ 0xa8
 8001564:	f8d6 104e 	ldr.w	r1, [r6, #78]	@ 0x4e
 8001568:	912b      	str	r1, [sp, #172]	@ 0xac
 800156a:	9948      	ldr	r1, [sp, #288]	@ 0x120
 800156c:	7a09      	ldrb	r1, [r1, #8]
 800156e:	2901      	cmp	r1, #1
 8001570:	d813      	bhi.n	800159a <smoltcp::iface::interface::Interface::poll+0x47e>
 8001572:	0080      	lsls	r0, r0, #2
 8001574:	e9cd 242d 	strd	r2, r4, [sp, #180]	@ 0xb4
 8001578:	b2c1      	uxtb	r1, r0
 800157a:	428d      	cmp	r5, r1
 800157c:	f0c1 86a3 	bcc.w	80032c6 <smoltcp::iface::interface::Interface::poll+0x21aa>
 8001580:	4618      	mov	r0, r3
 8001582:	4664      	mov	r4, ip
 8001584:	932c      	str	r3, [sp, #176]	@ 0xb0
 8001586:	f006 fd5a 	bl	800803e <smoltcp::wire::ip::checksum::data>
 800158a:	46a4      	mov	ip, r4
 800158c:	9b2c      	ldr	r3, [sp, #176]	@ 0xb0
 800158e:	e9dd 242d 	ldrd	r2, r4, [sp, #180]	@ 0xb4
 8001592:	43c0      	mvns	r0, r0
 8001594:	0400      	lsls	r0, r0, #16
 8001596:	f47f af20 	bne.w	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 800159a:	f8b6 005c 	ldrh.w	r0, [r6, #92]	@ 0x5c
 800159e:	f06f 01c0 	mvn.w	r1, #192	@ 0xc0
 80015a2:	4208      	tst	r0, r1
 80015a4:	f47f af19 	bne.w	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 80015a8:	942e      	str	r4, [sp, #184]	@ 0xb8
 80015aa:	f896 005f 	ldrb.w	r0, [r6, #95]	@ 0x5f
 80015ae:	f8d6 4066 	ldr.w	r4, [r6, #102]	@ 0x66
 80015b2:	f8d6 e062 	ldr.w	lr, [r6, #98]	@ 0x62
 80015b6:	283c      	cmp	r0, #60	@ 0x3c
 80015b8:	932c      	str	r3, [sp, #176]	@ 0xb0
 80015ba:	f04f 030c 	mov.w	r3, #12
 80015be:	f8cd b098 	str.w	fp, [sp, #152]	@ 0x98
 80015c2:	f200 8083 	bhi.w	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 80015c6:	e8df f000 	tbb	[pc, r0]
 80015ca:	1f1f      	.short	0x1f1f
 80015cc:	8181811f 	.word	0x8181811f
 80015d0:	8181817a 	.word	0x8181817a
 80015d4:	81818181 	.word	0x81818181
 80015d8:	7c818181 	.word	0x7c818181
 80015dc:	81818181 	.word	0x81818181
 80015e0:	81818181 	.word	0x81818181
 80015e4:	81818181 	.word	0x81818181
 80015e8:	81818181 	.word	0x81818181
 80015ec:	81818181 	.word	0x81818181
 80015f0:	81818181 	.word	0x81818181
 80015f4:	81767281 	.word	0x81767281
 80015f8:	81818181 	.word	0x81818181
 80015fc:	81818078 	.word	0x81818078
 8001600:	81818181 	.word	0x81818181
 8001604:	0074707e 	.word	0x0074707e
 8001608:	4603      	mov	r3, r0
 800160a:	e05f      	b.n	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 800160c:	9848      	ldr	r0, [sp, #288]	@ 0x120
 800160e:	f890 016e 	ldrb.w	r0, [r0, #366]	@ 0x16e
 8001612:	2801      	cmp	r0, #1
 8001614:	d118      	bne.n	8001648 <smoltcp::iface::interface::Interface::poll+0x52c>
 8001616:	f5bb 7f80 	cmp.w	fp, #256	@ 0x100
 800161a:	bf18      	it	ne
 800161c:	f5bb 7f00 	cmpne.w	fp, #512	@ 0x200
 8001620:	d10c      	bne.n	800163c <smoltcp::iface::interface::Interface::poll+0x520>
 8001622:	f006 00f0 	and.w	r0, r6, #240	@ 0xf0
 8001626:	28e0      	cmp	r0, #224	@ 0xe0
 8001628:	d003      	beq.n	8001632 <smoltcp::iface::interface::Interface::poll+0x516>
 800162a:	1c70      	adds	r0, r6, #1
 800162c:	4635      	mov	r5, r6
 800162e:	2801      	cmp	r0, #1
 8001630:	d80e      	bhi.n	8001650 <smoltcp::iface::interface::Interface::poll+0x534>
 8001632:	f240 0026 	movw	r0, #38	@ 0x26
 8001636:	f2c0 0000 	movt	r0, #0
 800163a:	e003      	b.n	8001644 <smoltcp::iface::interface::Interface::poll+0x528>
 800163c:	f240 0028 	movw	r0, #40	@ 0x28
 8001640:	f2c0 0000 	movt	r0, #0
 8001644:	f002 fe89 	bl	800435a <defmt::export::acquire_header_and_release>
 8001648:	f8dd 811c 	ldr.w	r8, [sp, #284]	@ 0x11c
 800164c:	9e2e      	ldr	r6, [sp, #184]	@ 0xb8
 800164e:	e6c9      	b.n	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 8001650:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 8001652:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 8001654:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 8001656:	0209      	lsls	r1, r1, #8
 8001658:	ea41 6112 	orr.w	r1, r1, r2, lsr #24
 800165c:	ea40 2002 	orr.w	r0, r0, r2, lsl #8
 8001660:	43c9      	mvns	r1, r1
 8001662:	b289      	uxth	r1, r1
 8001664:	ea61 0000 	orn	r0, r1, r0
 8001668:	2800      	cmp	r0, #0
 800166a:	d0e2      	beq.n	8001632 <smoltcp::iface::interface::Interface::poll+0x516>
 800166c:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 800166e:	f010 0001 	ands.w	r0, r0, #1
 8001672:	d1de      	bne.n	8001632 <smoltcp::iface::interface::Interface::poll+0x516>
 8001674:	9942      	ldr	r1, [sp, #264]	@ 0x108
 8001676:	e00f      	b.n	8001698 <smoltcp::iface::interface::Interface::poll+0x57c>
 8001678:	4250      	negs	r0, r2
 800167a:	f04f 32ff 	mov.w	r2, #4294967295	@ 0xffffffff
 800167e:	f000 001f 	and.w	r0, r0, #31
 8001682:	fa02 f000 	lsl.w	r0, r2, r0
 8001686:	ba02      	rev	r2, r0
 8001688:	6808      	ldr	r0, [r1, #0]
 800168a:	3105      	adds	r1, #5
 800168c:	f1a8 0805 	sub.w	r8, r8, #5
 8001690:	4068      	eors	r0, r5
 8001692:	4202      	tst	r2, r0
 8001694:	f000 80ee 	beq.w	8001874 <smoltcp::iface::interface::Interface::poll+0x758>
 8001698:	f1b8 0f00 	cmp.w	r8, #0
 800169c:	f000 80e5 	beq.w	800186a <smoltcp::iface::interface::Interface::poll+0x74e>
 80016a0:	790a      	ldrb	r2, [r1, #4]
 80016a2:	2a00      	cmp	r2, #0
 80016a4:	d1e8      	bne.n	8001678 <smoltcp::iface::interface::Interface::poll+0x55c>
 80016a6:	2200      	movs	r2, #0
 80016a8:	e7ee      	b.n	8001688 <smoltcp::iface::interface::Interface::poll+0x56c>
 80016aa:	230a      	movs	r3, #10
 80016ac:	e00e      	b.n	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 80016ae:	2305      	movs	r3, #5
 80016b0:	e00c      	b.n	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 80016b2:	230b      	movs	r3, #11
 80016b4:	e00a      	b.n	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 80016b6:	2306      	movs	r3, #6
 80016b8:	e008      	b.n	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 80016ba:	2307      	movs	r3, #7
 80016bc:	e006      	b.n	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 80016be:	2303      	movs	r3, #3
 80016c0:	e004      	b.n	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 80016c2:	2304      	movs	r3, #4
 80016c4:	e002      	b.n	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 80016c6:	2309      	movs	r3, #9
 80016c8:	e000      	b.n	80016cc <smoltcp::iface::interface::Interface::poll+0x5b0>
 80016ca:	2308      	movs	r3, #8
 80016cc:	992e      	ldr	r1, [sp, #184]	@ 0xb8
 80016ce:	922d      	str	r2, [sp, #180]	@ 0xb4
 80016d0:	1a8a      	subs	r2, r1, r2
 80016d2:	9928      	ldr	r1, [sp, #160]	@ 0xa0
 80016d4:	f896 b05e 	ldrb.w	fp, [r6, #94]	@ 0x5e
 80016d8:	f021 01ff 	bic.w	r1, r1, #255	@ 0xff
 80016dc:	962e      	str	r6, [sp, #184]	@ 0xb8
 80016de:	4401      	add	r1, r0
 80016e0:	f00e 00f0 	and.w	r0, lr, #240	@ 0xf0
 80016e4:	28e0      	cmp	r0, #224	@ 0xe0
 80016e6:	9429      	str	r4, [sp, #164]	@ 0xa4
 80016e8:	e9cd e127 	strd	lr, r1, [sp, #156]	@ 0x9c
 80016ec:	bf18      	it	ne
 80016ee:	f11e 0001 	addsne.w	r0, lr, #1
 80016f2:	d107      	bne.n	8001704 <smoltcp::iface::interface::Interface::poll+0x5e8>
 80016f4:	f240 0029 	movw	r0, #41	@ 0x29
 80016f8:	f2c0 0000 	movt	r0, #0
 80016fc:	f002 fe2d 	bl	800435a <defmt::export::acquire_header_and_release>
 8001700:	9e2e      	ldr	r6, [sp, #184]	@ 0xb8
 8001702:	e66a      	b.n	80013da <smoltcp::iface::interface::Interface::poll+0x2be>
 8001704:	f1be 0f00 	cmp.w	lr, #0
 8001708:	e9cd b222 	strd	fp, r2, [sp, #136]	@ 0x88
 800170c:	9325      	str	r3, [sp, #148]	@ 0x94
 800170e:	d101      	bne.n	8001714 <smoltcp::iface::interface::Interface::poll+0x5f8>
 8001710:	2601      	movs	r6, #1
 8001712:	e031      	b.n	8001778 <smoltcp::iface::interface::Interface::poll+0x65c>
 8001714:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8001716:	46e3      	mov	fp, ip
 8001718:	f8d0 00f0 	ldr.w	r0, [r0, #240]	@ 0xf0
 800171c:	b338      	cbz	r0, 800176e <smoltcp::iface::interface::Interface::poll+0x652>
 800171e:	9942      	ldr	r1, [sp, #264]	@ 0x108
 8001720:	eb00 0080 	add.w	r0, r0, r0, lsl #2
 8001724:	eb01 0c00 	add.w	ip, r1, r0
 8001728:	e00f      	b.n	800174a <smoltcp::iface::interface::Interface::poll+0x62e>
 800172a:	2000      	movs	r0, #0
 800172c:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 8001730:	f002 021f 	and.w	r2, r2, #31
 8001734:	4018      	ands	r0, r3
 8001736:	fa24 f202 	lsr.w	r2, r4, r2
 800173a:	ba12      	rev	r2, r2
 800173c:	4310      	orrs	r0, r2
 800173e:	4586      	cmp	lr, r0
 8001740:	f000 808b 	beq.w	800185a <smoltcp::iface::interface::Interface::poll+0x73e>
 8001744:	3105      	adds	r1, #5
 8001746:	4561      	cmp	r1, ip
 8001748:	d011      	beq.n	800176e <smoltcp::iface::interface::Interface::poll+0x652>
 800174a:	790a      	ldrb	r2, [r1, #4]
 800174c:	680b      	ldr	r3, [r1, #0]
 800174e:	2a00      	cmp	r2, #0
 8001750:	d0eb      	beq.n	800172a <smoltcp::iface::interface::Interface::poll+0x60e>
 8001752:	f1a2 001f 	sub.w	r0, r2, #31
 8001756:	b2c0      	uxtb	r0, r0
 8001758:	2802      	cmp	r0, #2
 800175a:	d3f3      	bcc.n	8001744 <smoltcp::iface::interface::Interface::poll+0x628>
 800175c:	4250      	negs	r0, r2
 800175e:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 8001762:	f000 001f 	and.w	r0, r0, #31
 8001766:	fa04 f000 	lsl.w	r0, r4, r0
 800176a:	ba00      	rev	r0, r0
 800176c:	e7e0      	b.n	8001730 <smoltcp::iface::interface::Interface::poll+0x614>
 800176e:	fabe f08e 	clz	r0, lr
 8001772:	9c29      	ldr	r4, [sp, #164]	@ 0xa4
 8001774:	46dc      	mov	ip, fp
 8001776:	0946      	lsrs	r6, r0, #5
 8001778:	e9dd 202d 	ldrd	r2, r0, [sp, #180]	@ 0xb4
 800177c:	f8b0 0058 	ldrh.w	r0, [r0, #88]	@ 0x58
 8001780:	ba00      	rev	r0, r0
 8001782:	0c01      	lsrs	r1, r0, #16
 8001784:	ebb2 4f10 	cmp.w	r2, r0, lsr #16
 8001788:	f201 85de 	bhi.w	8003348 <smoltcp::iface::interface::Interface::poll+0x222c>
 800178c:	428d      	cmp	r5, r1
 800178e:	f0c1 85db 	bcc.w	8003348 <smoltcp::iface::interface::Interface::poll+0x222c>
 8001792:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8001794:	9b42      	ldr	r3, [sp, #264]	@ 0x108
 8001796:	e9cd 6c20 	strd	r6, ip, [sp, #128]	@ 0x80
 800179a:	f8d0 e0f0 	ldr.w	lr, [r0, #240]	@ 0xf0
 800179e:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 80017a0:	1885      	adds	r5, r0, r2
 80017a2:	1a88      	subs	r0, r1, r2
 80017a4:	eb0e 028e 	add.w	r2, lr, lr, lsl #2
 80017a8:	902d      	str	r0, [sp, #180]	@ 0xb4
 80017aa:	1899      	adds	r1, r3, r2
 80017ac:	9524      	str	r5, [sp, #144]	@ 0x90
 80017ae:	952c      	str	r5, [sp, #176]	@ 0xb0
 80017b0:	b302      	cbz	r2, 80017f4 <smoltcp::iface::interface::Interface::poll+0x6d8>
 80017b2:	4618      	mov	r0, r3
 80017b4:	f850 5b05 	ldr.w	r5, [r0], #5
 80017b8:	42a5      	cmp	r5, r4
 80017ba:	f000 814d 	beq.w	8001a58 <smoltcp::iface::interface::Interface::poll+0x93c>
 80017be:	4288      	cmp	r0, r1
 80017c0:	d018      	beq.n	80017f4 <smoltcp::iface::interface::Interface::poll+0x6d8>
 80017c2:	f8d3 0005 	ldr.w	r0, [r3, #5]
 80017c6:	42a0      	cmp	r0, r4
 80017c8:	f000 8146 	beq.w	8001a58 <smoltcp::iface::interface::Interface::poll+0x93c>
 80017cc:	f103 000a 	add.w	r0, r3, #10
 80017d0:	4288      	cmp	r0, r1
 80017d2:	d00f      	beq.n	80017f4 <smoltcp::iface::interface::Interface::poll+0x6d8>
 80017d4:	f8d3 000a 	ldr.w	r0, [r3, #10]
 80017d8:	42a0      	cmp	r0, r4
 80017da:	f000 813d 	beq.w	8001a58 <smoltcp::iface::interface::Interface::poll+0x93c>
 80017de:	f103 000f 	add.w	r0, r3, #15
 80017e2:	4288      	cmp	r0, r1
 80017e4:	d006      	beq.n	80017f4 <smoltcp::iface::interface::Interface::poll+0x6d8>
 80017e6:	f8d3 000f 	ldr.w	r0, [r3, #15]
 80017ea:	3a14      	subs	r2, #20
 80017ec:	3314      	adds	r3, #20
 80017ee:	42a0      	cmp	r0, r4
 80017f0:	d1de      	bne.n	80017b0 <smoltcp::iface::interface::Interface::poll+0x694>
 80017f2:	e131      	b.n	8001a58 <smoltcp::iface::interface::Interface::poll+0x93c>
 80017f4:	f114 0c01 	adds.w	ip, r4, #1
 80017f8:	f000 8137 	beq.w	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 80017fc:	9a29      	ldr	r2, [sp, #164]	@ 0xa4
 80017fe:	20e0      	movs	r0, #224	@ 0xe0
 8001800:	f2c0 1000 	movt	r0, #256	@ 0x100
 8001804:	4282      	cmp	r2, r0
 8001806:	f000 8130 	beq.w	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 800180a:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 800180e:	f1be 0f00 	cmp.w	lr, #0
 8001812:	f000 80d0 	beq.w	80019b6 <smoltcp::iface::interface::Interface::poll+0x89a>
 8001816:	9b42      	ldr	r3, [sp, #264]	@ 0x108
 8001818:	e00f      	b.n	800183a <smoltcp::iface::interface::Interface::poll+0x71e>
 800181a:	2000      	movs	r0, #0
 800181c:	4010      	ands	r0, r2
 800181e:	f005 021f 	and.w	r2, r5, #31
 8001822:	fa24 f202 	lsr.w	r2, r4, r2
 8001826:	ba12      	rev	r2, r2
 8001828:	4310      	orrs	r0, r2
 800182a:	9a29      	ldr	r2, [sp, #164]	@ 0xa4
 800182c:	4282      	cmp	r2, r0
 800182e:	f000 810e 	beq.w	8001a4e <smoltcp::iface::interface::Interface::poll+0x932>
 8001832:	3305      	adds	r3, #5
 8001834:	428b      	cmp	r3, r1
 8001836:	f000 80be 	beq.w	80019b6 <smoltcp::iface::interface::Interface::poll+0x89a>
 800183a:	791d      	ldrb	r5, [r3, #4]
 800183c:	681a      	ldr	r2, [r3, #0]
 800183e:	2d00      	cmp	r5, #0
 8001840:	d0eb      	beq.n	800181a <smoltcp::iface::interface::Interface::poll+0x6fe>
 8001842:	f1a5 001f 	sub.w	r0, r5, #31
 8001846:	b2c0      	uxtb	r0, r0
 8001848:	2802      	cmp	r0, #2
 800184a:	d3f2      	bcc.n	8001832 <smoltcp::iface::interface::Interface::poll+0x716>
 800184c:	4268      	negs	r0, r5
 800184e:	f000 001f 	and.w	r0, r0, #31
 8001852:	fa04 f000 	lsl.w	r0, r4, r0
 8001856:	ba00      	rev	r0, r0
 8001858:	e7e0      	b.n	800181c <smoltcp::iface::interface::Interface::poll+0x700>
 800185a:	9c29      	ldr	r4, [sp, #164]	@ 0xa4
 800185c:	f1be 0f00 	cmp.w	lr, #0
 8001860:	f47f af48 	bne.w	80016f4 <smoltcp::iface::interface::Interface::poll+0x5d8>
 8001864:	46dc      	mov	ip, fp
 8001866:	2601      	movs	r6, #1
 8001868:	e786      	b.n	8001778 <smoltcp::iface::interface::Interface::poll+0x65c>
 800186a:	f240 0027 	movw	r0, #39	@ 0x27
 800186e:	f2c0 0000 	movt	r0, #0
 8001872:	e6e7      	b.n	8001644 <smoltcp::iface::interface::Interface::poll+0x528>
 8001874:	983d      	ldr	r0, [sp, #244]	@ 0xf4
 8001876:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 8001878:	9b2b      	ldr	r3, [sp, #172]	@ 0xac
 800187a:	6003      	str	r3, [r0, #0]
 800187c:	7101      	strb	r1, [r0, #4]
 800187e:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 8001880:	f88d 02f0 	strb.w	r0, [sp, #752]	@ 0x2f0
 8001884:	b2c8      	uxtb	r0, r1
 8001886:	0e19      	lsrs	r1, r3, #24
 8001888:	ea41 2300 	orr.w	r3, r1, r0, lsl #8
 800188c:	9abc      	ldr	r2, [sp, #752]	@ 0x2f0
 800188e:	4629      	mov	r1, r5
 8001890:	983c      	ldr	r0, [sp, #240]	@ 0xf0
 8001892:	e9cd ce00 	strd	ip, lr, [sp]
 8001896:	f006 fdd7 	bl	8008448 <smoltcp::iface::neighbor::Cache::fill>
 800189a:	f5bb 7f80 	cmp.w	fp, #256	@ 0x100
 800189e:	f040 809a 	bne.w	80019d6 <smoltcp::iface::interface::Interface::poll+0x8ba>
 80018a2:	9944      	ldr	r1, [sp, #272]	@ 0x110
 80018a4:	9a38      	ldr	r2, [sp, #224]	@ 0xe0
 80018a6:	f8cd 4296 	str.w	r4, [sp, #662]	@ 0x296
 80018aa:	6808      	ldr	r0, [r1, #0]
 80018ac:	8889      	ldrh	r1, [r1, #4]
 80018ae:	8091      	strh	r1, [r2, #4]
 80018b0:	6010      	str	r0, [r2, #0]
 80018b2:	9a36      	ldr	r2, [sp, #216]	@ 0xd8
 80018b4:	e9dd 0152 	ldrd	r0, r1, [sp, #328]	@ 0x148
 80018b8:	96a8      	str	r6, [sp, #672]	@ 0x2a0
 80018ba:	6010      	str	r0, [r2, #0]
 80018bc:	9837      	ldr	r0, [sp, #220]	@ 0xdc
 80018be:	6051      	str	r1, [r2, #4]
 80018c0:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 80018c2:	7101      	strb	r1, [r0, #4]
 80018c4:	992b      	ldr	r1, [sp, #172]	@ 0xac
 80018c6:	6001      	str	r1, [r0, #0]
 80018c8:	2001      	movs	r0, #1
 80018ca:	e9d9 1504 	ldrd	r1, r5, [r9, #16]
 80018ce:	f8ad 028c 	strh.w	r0, [sp, #652]	@ 0x28c
 80018d2:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 80018d4:	428d      	cmp	r5, r1
 80018d6:	f88d 029a 	strb.w	r0, [sp, #666]	@ 0x29a
 80018da:	f081 8546 	bcs.w	800336a <smoltcp::iface::interface::Interface::poll+0x224e>
 80018de:	f44f 60c6 	mov.w	r0, #1584	@ 0x630
 80018e2:	fb05 f900 	mul.w	r9, r5, r0
 80018e6:	9841      	ldr	r0, [sp, #260]	@ 0x104
 80018e8:	6800      	ldr	r0, [r0, #0]
 80018ea:	f850 2009 	ldr.w	r2, [r0, r9]
 80018ee:	2a00      	cmp	r2, #0
 80018f0:	f101 8542 	bmi.w	8003378 <smoltcp::iface::interface::Interface::poll+0x225c>
 80018f4:	1c6a      	adds	r2, r5, #1
 80018f6:	f8d7 8008 	ldr.w	r8, [r7, #8]
 80018fa:	fbb2 f3f1 	udiv	r3, r2, r1
 80018fe:	fb03 2111 	mls	r1, r3, r1, r2
 8001902:	f44f 62c6 	mov.w	r2, #1584	@ 0x630
 8001906:	9b2c      	ldr	r3, [sp, #176]	@ 0xb0
 8001908:	fb05 0002 	mla	r0, r5, r2, r0
 800190c:	9a44      	ldr	r2, [sp, #272]	@ 0x110
 800190e:	f8c8 1014 	str.w	r1, [r8, #20]
 8001912:	6811      	ldr	r1, [r2, #0]
 8001914:	8892      	ldrh	r2, [r2, #4]
 8001916:	f8c0 103e 	str.w	r1, [r0, #62]	@ 0x3e
 800191a:	f44f 61c1 	mov.w	r1, #1544	@ 0x608
 800191e:	f880 303d 	strb.w	r3, [r0, #61]	@ 0x3d
 8001922:	9b2b      	ldr	r3, [sp, #172]	@ 0xac
 8001924:	f8a0 1044 	strh.w	r1, [r0, #68]	@ 0x44
 8001928:	992d      	ldr	r1, [sp, #180]	@ 0xb4
 800192a:	f8a0 2042 	strh.w	r2, [r0, #66]	@ 0x42
 800192e:	221c      	movs	r2, #28
 8001930:	f880 1038 	strb.w	r1, [r0, #56]	@ 0x38
 8001934:	f100 0146 	add.w	r1, r0, #70	@ 0x46
 8001938:	f8c0 3039 	str.w	r3, [r0, #57]	@ 0x39
 800193c:	a8a3      	add	r0, sp, #652	@ 0x28c
 800193e:	f7fe fc87 	bl	8000250 <smoltcp::wire::arp::Repr::emit>
 8001942:	f8d8 1010 	ldr.w	r1, [r8, #16]
 8001946:	428d      	cmp	r5, r1
 8001948:	f081 8521 	bcs.w	800338e <smoltcp::iface::interface::Interface::poll+0x2272>
 800194c:	f8dd a104 	ldr.w	sl, [sp, #260]	@ 0x104
 8001950:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 8001954:	232a      	movs	r3, #42	@ 0x2a
 8001956:	f8da 0000 	ldr.w	r0, [sl]
 800195a:	fb05 0101 	mla	r1, r5, r1, r0
 800195e:	684a      	ldr	r2, [r1, #4]
 8001960:	e9d1 ec0a 	ldrd	lr, ip, [r1, #40]	@ 0x28
 8001964:	f363 020b 	bfi	r2, r3, #0, #12
 8001968:	604a      	str	r2, [r1, #4]
 800196a:	2200      	movs	r2, #0
 800196c:	f8c1 e008 	str.w	lr, [r1, #8]
 8001970:	620a      	str	r2, [r1, #32]
 8001972:	2300      	movs	r3, #0
 8001974:	f8c1 c00c 	str.w	ip, [r1, #12]
 8001978:	f2cf 03d0 	movt	r3, #61648	@ 0xf0d0
 800197c:	f3bf 8f5f 	dmb	sy
 8001980:	f891 1030 	ldrb.w	r1, [r1, #48]	@ 0x30
 8001984:	ea43 5141 	orr.w	r1, r3, r1, lsl #21
 8001988:	f840 1009 	str.w	r1, [r0, r9]
 800198c:	f249 0014 	movw	r0, #36884	@ 0x9014
 8001990:	f3bf 8f5f 	dmb	sy
 8001994:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 8001998:	f8d7 9008 	ldr.w	r9, [r7, #8]
 800199c:	f840 2c10 	str.w	r2, [r0, #-16]
 80019a0:	2001      	movs	r0, #1
 80019a2:	e9cd 601b 	strd	r6, r0, [sp, #108]	@ 0x6c
 80019a6:	982b      	ldr	r0, [sp, #172]	@ 0xac
 80019a8:	901f      	str	r0, [sp, #124]	@ 0x7c
 80019aa:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 80019ac:	901e      	str	r0, [sp, #120]	@ 0x78
 80019ae:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 80019b0:	9043      	str	r0, [sp, #268]	@ 0x10c
 80019b2:	941d      	str	r4, [sp, #116]	@ 0x74
 80019b4:	e648      	b.n	8001648 <smoltcp::iface::interface::Interface::poll+0x52c>
 80019b6:	9829      	ldr	r0, [sp, #164]	@ 0xa4
 80019b8:	f000 00f0 	and.w	r0, r0, #240	@ 0xf0
 80019bc:	28e0      	cmp	r0, #224	@ 0xe0
 80019be:	bf1e      	ittt	ne
 80019c0:	9848      	ldrne	r0, [sp, #288]	@ 0x120
 80019c2:	f890 016e 	ldrbne.w	r0, [r0, #366]	@ 0x16e
 80019c6:	ea5f 70c0 	movsne.w	r0, r0, lsl #31
 80019ca:	d107      	bne.n	80019dc <smoltcp::iface::interface::Interface::poll+0x8c0>
 80019cc:	f8d7 9008 	ldr.w	r9, [r7, #8]
 80019d0:	f8dd a104 	ldr.w	sl, [sp, #260]	@ 0x104
 80019d4:	e694      	b.n	8001700 <smoltcp::iface::interface::Interface::poll+0x5e4>
 80019d6:	f8dd a104 	ldr.w	sl, [sp, #260]	@ 0x104
 80019da:	e635      	b.n	8001648 <smoltcp::iface::interface::Interface::poll+0x52c>
 80019dc:	f1bc 0f02 	cmp.w	ip, #2
 80019e0:	d3f4      	bcc.n	80019cc <smoltcp::iface::interface::Interface::poll+0x8b0>
 80019e2:	9848      	ldr	r0, [sp, #288]	@ 0x120
 80019e4:	9a29      	ldr	r2, [sp, #164]	@ 0xa4
 80019e6:	e9d0 0104 	ldrd	r0, r1, [r0, #16]
 80019ea:	e9cd 0100 	strd	r0, r1, [sp]
 80019ee:	a8bc      	add	r0, sp, #752	@ 0x2f0
 80019f0:	9933      	ldr	r1, [sp, #204]	@ 0xcc
 80019f2:	f006 fc8a 	bl	800830a <smoltcp::iface::route::Routes::lookup>
 80019f6:	f89d 02f0 	ldrb.w	r0, [sp, #752]	@ 0x2f0
 80019fa:	2801      	cmp	r0, #1
 80019fc:	d1e6      	bne.n	80019cc <smoltcp::iface::interface::Interface::poll+0x8b0>
 80019fe:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8001a00:	9d42      	ldr	r5, [sp, #264]	@ 0x108
 8001a02:	f8dd 12f1 	ldr.w	r1, [sp, #753]	@ 0x2f1
 8001a06:	f8d0 e0f0 	ldr.w	lr, [r0, #240]	@ 0xf0
 8001a0a:	eb0e 028e 	add.w	r2, lr, lr, lsl #2
 8001a0e:	18ab      	adds	r3, r5, r2
 8001a10:	2a00      	cmp	r2, #0
 8001a12:	d0db      	beq.n	80019cc <smoltcp::iface::interface::Interface::poll+0x8b0>
 8001a14:	4628      	mov	r0, r5
 8001a16:	f850 4b05 	ldr.w	r4, [r0], #5
 8001a1a:	428c      	cmp	r4, r1
 8001a1c:	d017      	beq.n	8001a4e <smoltcp::iface::interface::Interface::poll+0x932>
 8001a1e:	4298      	cmp	r0, r3
 8001a20:	d0d4      	beq.n	80019cc <smoltcp::iface::interface::Interface::poll+0x8b0>
 8001a22:	f8d5 0005 	ldr.w	r0, [r5, #5]
 8001a26:	4288      	cmp	r0, r1
 8001a28:	d011      	beq.n	8001a4e <smoltcp::iface::interface::Interface::poll+0x932>
 8001a2a:	f105 000a 	add.w	r0, r5, #10
 8001a2e:	4298      	cmp	r0, r3
 8001a30:	d0cc      	beq.n	80019cc <smoltcp::iface::interface::Interface::poll+0x8b0>
 8001a32:	f8d5 000a 	ldr.w	r0, [r5, #10]
 8001a36:	4288      	cmp	r0, r1
 8001a38:	d009      	beq.n	8001a4e <smoltcp::iface::interface::Interface::poll+0x932>
 8001a3a:	f105 000f 	add.w	r0, r5, #15
 8001a3e:	4298      	cmp	r0, r3
 8001a40:	d0c4      	beq.n	80019cc <smoltcp::iface::interface::Interface::poll+0x8b0>
 8001a42:	f8d5 000f 	ldr.w	r0, [r5, #15]
 8001a46:	3a14      	subs	r2, #20
 8001a48:	3514      	adds	r5, #20
 8001a4a:	4288      	cmp	r0, r1
 8001a4c:	d1e0      	bne.n	8001a10 <smoltcp::iface::interface::Interface::poll+0x8f4>
 8001a4e:	f8d7 9008 	ldr.w	r9, [r7, #8]
 8001a52:	f8dd a104 	ldr.w	sl, [sp, #260]	@ 0x104
 8001a56:	9c29      	ldr	r4, [sp, #164]	@ 0xa4
 8001a58:	f004 00f0 	and.w	r0, r4, #240	@ 0xf0
 8001a5c:	28e0      	cmp	r0, #224	@ 0xe0
 8001a5e:	d004      	beq.n	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 8001a60:	9829      	ldr	r0, [sp, #164]	@ 0xa4
 8001a62:	3001      	adds	r0, #1
 8001a64:	2802      	cmp	r0, #2
 8001a66:	f080 8157 	bcs.w	8001d18 <smoltcp::iface::interface::Interface::poll+0xbfc>
 8001a6a:	9925      	ldr	r1, [sp, #148]	@ 0x94
 8001a6c:	2904      	cmp	r1, #4
 8001a6e:	f000 80b8 	beq.w	8001be2 <smoltcp::iface::interface::Interface::poll+0xac6>
 8001a72:	2903      	cmp	r1, #3
 8001a74:	d052      	beq.n	8001b1c <smoltcp::iface::interface::Interface::poll+0xa00>
 8001a76:	2901      	cmp	r1, #1
 8001a78:	f040 80c9 	bne.w	8001c0e <smoltcp::iface::interface::Interface::poll+0xaf2>
 8001a7c:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 8001a7e:	9924      	ldr	r1, [sp, #144]	@ 0x90
 8001a80:	2808      	cmp	r0, #8
 8001a82:	bf38      	it	cc
 8001a84:	2100      	movcc	r1, #0
 8001a86:	2807      	cmp	r0, #7
 8001a88:	9124      	str	r1, [sp, #144]	@ 0x90
 8001a8a:	f240 80b9 	bls.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001a8e:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8001a90:	7ac0      	ldrb	r0, [r0, #11]
 8001a92:	2801      	cmp	r0, #1
 8001a94:	d807      	bhi.n	8001aa6 <smoltcp::iface::interface::Interface::poll+0x98a>
 8001a96:	e9dd 012c 	ldrd	r0, r1, [sp, #176]	@ 0xb0
 8001a9a:	f006 fad0 	bl	800803e <smoltcp::wire::ip::checksum::data>
 8001a9e:	43c0      	mvns	r0, r0
 8001aa0:	0400      	lsls	r0, r0, #16
 8001aa2:	f040 80ad 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001aa6:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001aa8:	7800      	ldrb	r0, [r0, #0]
 8001aaa:	280b      	cmp	r0, #11
 8001aac:	f200 80a8 	bhi.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001ab0:	e8df f010 	tbh	[pc, r0, lsl #1]
 8001ab4:	00a60218 	.word	0x00a60218
 8001ab8:	000c00a6 	.word	0x000c00a6
 8001abc:	00a600a6 	.word	0x00a600a6
 8001ac0:	00a600a6 	.word	0x00a600a6
 8001ac4:	00a6022b 	.word	0x00a6022b
 8001ac8:	000c00a6 	.word	0x000c00a6
 8001acc:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 8001ace:	f1a0 0108 	sub.w	r1, r0, #8
 8001ad2:	2914      	cmp	r1, #20
 8001ad4:	f0c0 8094 	bcc.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001ad8:	9824      	ldr	r0, [sp, #144]	@ 0x90
 8001ada:	233c      	movs	r3, #60	@ 0x3c
 8001adc:	3008      	adds	r0, #8
 8001ade:	7802      	ldrb	r2, [r0, #0]
 8001ae0:	ea03 0282 	and.w	r2, r3, r2, lsl #2
 8001ae4:	4291      	cmp	r1, r2
 8001ae6:	f0c0 808b 	bcc.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001aea:	9b2c      	ldr	r3, [sp, #176]	@ 0xb0
 8001aec:	895b      	ldrh	r3, [r3, #10]
 8001aee:	ba1b      	rev	r3, r3
 8001af0:	ebb2 4f13 	cmp.w	r2, r3, lsr #16
 8001af4:	f200 8084 	bhi.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001af8:	0c1b      	lsrs	r3, r3, #16
 8001afa:	4299      	cmp	r1, r3
 8001afc:	bf24      	itt	cs
 8001afe:	1a89      	subcs	r1, r1, r2
 8001b00:	2908      	cmpcs	r1, #8
 8001b02:	d37d      	bcc.n	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001b04:	992c      	ldr	r1, [sp, #176]	@ 0xb0
 8001b06:	4410      	add	r0, r2
 8001b08:	901a      	str	r0, [sp, #104]	@ 0x68
 8001b0a:	2004      	movs	r0, #4
 8001b0c:	9055      	str	r0, [sp, #340]	@ 0x154
 8001b0e:	9815      	ldr	r0, [sp, #84]	@ 0x54
 8001b10:	7849      	ldrb	r1, [r1, #1]
 8001b12:	f020 00ff 	bic.w	r0, r0, #255	@ 0xff
 8001b16:	4408      	add	r0, r1
 8001b18:	9015      	str	r0, [sp, #84]	@ 0x54
 8001b1a:	e073      	b.n	8001c04 <smoltcp::iface::interface::Interface::poll+0xae8>
 8001b1c:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 8001b1e:	2814      	cmp	r0, #20
 8001b20:	d36e      	bcc.n	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001b22:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001b24:	213c      	movs	r1, #60	@ 0x3c
 8001b26:	8985      	ldrh	r5, [r0, #12]
 8001b28:	b2e8      	uxtb	r0, r5
 8001b2a:	2850      	cmp	r0, #80	@ 0x50
 8001b2c:	f04f 0000 	mov.w	r0, #0
 8001b30:	ea01 0695 	and.w	r6, r1, r5, lsr #2
 8001b34:	bf38      	it	cc
 8001b36:	2001      	movcc	r0, #1
 8001b38:	992d      	ldr	r1, [sp, #180]	@ 0xb4
 8001b3a:	42b1      	cmp	r1, r6
 8001b3c:	f04f 0100 	mov.w	r1, #0
 8001b40:	bf38      	it	cc
 8001b42:	2101      	movcc	r1, #1
 8001b44:	4308      	orrs	r0, r1
 8001b46:	2801      	cmp	r0, #1
 8001b48:	d05a      	beq.n	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001b4a:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001b4c:	8800      	ldrh	r0, [r0, #0]
 8001b4e:	2800      	cmp	r0, #0
 8001b50:	bf1e      	ittt	ne
 8001b52:	982c      	ldrne	r0, [sp, #176]	@ 0xb0
 8001b54:	8840      	ldrhne	r0, [r0, #2]
 8001b56:	2800      	cmpne	r0, #0
 8001b58:	d052      	beq.n	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001b5a:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8001b5c:	7a80      	ldrb	r0, [r0, #10]
 8001b5e:	2801      	cmp	r0, #1
 8001b60:	d832      	bhi.n	8001bc8 <smoltcp::iface::interface::Interface::poll+0xaac>
 8001b62:	9829      	ldr	r0, [sp, #164]	@ 0xa4
 8001b64:	ba00      	rev	r0, r0
 8001b66:	0c01      	lsrs	r1, r0, #16
 8001b68:	fa11 f080 	uxtah	r0, r1, r0
 8001b6c:	0c01      	lsrs	r1, r0, #16
 8001b6e:	fa11 f080 	uxtah	r0, r1, r0
 8001b72:	9927      	ldr	r1, [sp, #156]	@ 0x9c
 8001b74:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001b78:	ba09      	rev	r1, r1
 8001b7a:	b280      	uxth	r0, r0
 8001b7c:	0c0a      	lsrs	r2, r1, #16
 8001b7e:	fa12 f181 	uxtah	r1, r2, r1
 8001b82:	0c0a      	lsrs	r2, r1, #16
 8001b84:	fa12 f181 	uxtah	r1, r2, r1
 8001b88:	eb01 4111 	add.w	r1, r1, r1, lsr #16
 8001b8c:	fa10 f081 	uxtah	r0, r0, r1
 8001b90:	992d      	ldr	r1, [sp, #180]	@ 0xb4
 8001b92:	1d8b      	adds	r3, r1, #6
 8001b94:	0c1a      	lsrs	r2, r3, #16
 8001b96:	fa12 f283 	uxtah	r2, r2, r3
 8001b9a:	eb02 4212 	add.w	r2, r2, r2, lsr #16
 8001b9e:	fa10 f082 	uxtah	r0, r0, r2
 8001ba2:	0c02      	lsrs	r2, r0, #16
 8001ba4:	fa12 f080 	uxtah	r0, r2, r0
 8001ba8:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001bac:	b284      	uxth	r4, r0
 8001bae:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001bb0:	f006 fa45 	bl	800803e <smoltcp::wire::ip::checksum::data>
 8001bb4:	fa14 f080 	uxtah	r0, r4, r0
 8001bb8:	0c01      	lsrs	r1, r0, #16
 8001bba:	fa11 f080 	uxtah	r0, r1, r0
 8001bbe:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001bc2:	43c0      	mvns	r0, r0
 8001bc4:	0400      	lsls	r0, r0, #16
 8001bc6:	d11b      	bne.n	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001bc8:	05a8      	lsls	r0, r5, #22
 8001bca:	f100 8218 	bmi.w	8001ffe <smoltcp::iface::interface::Interface::poll+0xee2>
 8001bce:	05e8      	lsls	r0, r5, #23
 8001bd0:	f100 821b 	bmi.w	800200a <smoltcp::iface::interface::Interface::poll+0xeee>
 8001bd4:	0568      	lsls	r0, r5, #21
 8001bd6:	f04f 0004 	mov.w	r0, #4
 8001bda:	bf58      	it	pl
 8001bdc:	f3c5 20c0 	ubfxpl	r0, r5, #11, #1
 8001be0:	e217      	b.n	8002012 <smoltcp::iface::interface::Interface::poll+0xef6>
 8001be2:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 8001be4:	2808      	cmp	r0, #8
 8001be6:	d30b      	bcc.n	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001be8:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001bea:	8880      	ldrh	r0, [r0, #4]
 8001bec:	ba00      	rev	r0, r0
 8001bee:	ea4f 4910 	mov.w	r9, r0, lsr #16
 8001bf2:	f1b9 0f08 	cmp.w	r9, #8
 8001bf6:	bf24      	itt	cs
 8001bf8:	982d      	ldrcs	r0, [sp, #180]	@ 0xb4
 8001bfa:	4548      	cmpcs	r0, r9
 8001bfc:	f080 80e8 	bcs.w	8001dd0 <smoltcp::iface::interface::Interface::poll+0xcb4>
 8001c00:	2004      	movs	r0, #4
 8001c02:	9055      	str	r0, [sp, #340]	@ 0x154
 8001c04:	f8d7 9008 	ldr.w	r9, [r7, #8]
 8001c08:	f8dd a104 	ldr.w	sl, [sp, #260]	@ 0x104
 8001c0c:	e021      	b.n	8001c52 <smoltcp::iface::interface::Interface::poll+0xb36>
 8001c0e:	9828      	ldr	r0, [sp, #160]	@ 0xa0
 8001c10:	f8ad 01d2 	strh.w	r0, [sp, #466]	@ 0x1d2
 8001c14:	9822      	ldr	r0, [sp, #136]	@ 0x88
 8001c16:	f88d 01d0 	strb.w	r0, [sp, #464]	@ 0x1d0
 8001c1a:	9823      	ldr	r0, [sp, #140]	@ 0x8c
 8001c1c:	9073      	str	r0, [sp, #460]	@ 0x1cc
 8001c1e:	f240 2002 	movw	r0, #514	@ 0x202
 8001c22:	f8ad 01b8 	strh.w	r0, [sp, #440]	@ 0x1b8
 8001c26:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001c28:	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
 8001c2a:	9b29      	ldr	r3, [sp, #164]	@ 0xa4
 8001c2c:	9a27      	ldr	r2, [sp, #156]	@ 0x9c
 8001c2e:	f5b6 7f04 	cmp.w	r6, #528	@ 0x210
 8001c32:	906f      	str	r0, [sp, #444]	@ 0x1bc
 8001c34:	f44f 7004 	mov.w	r0, #528	@ 0x210
 8001c38:	f88d 11d1 	strb.w	r1, [sp, #465]	@ 0x1d1
 8001c3c:	9372      	str	r3, [sp, #456]	@ 0x1c8
 8001c3e:	9271      	str	r2, [sp, #452]	@ 0x1c4
 8001c40:	bf28      	it	cs
 8001c42:	4606      	movcs	r6, r0
 8001c44:	a86e      	add	r0, sp, #440	@ 0x1b8
 8001c46:	9948      	ldr	r1, [sp, #288]	@ 0x120
 8001c48:	9000      	str	r0, [sp, #0]
 8001c4a:	a855      	add	r0, sp, #340	@ 0x154
 8001c4c:	9670      	str	r6, [sp, #448]	@ 0x1c0
 8001c4e:	f005 fea2 	bl	8007996 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>
 8001c52:	9c55      	ldr	r4, [sp, #340]	@ 0x154
 8001c54:	f8dd 811c 	ldr.w	r8, [sp, #284]	@ 0x11c
 8001c58:	9e2e      	ldr	r6, [sp, #184]	@ 0xb8
 8001c5a:	2c04      	cmp	r4, #4
 8001c5c:	f43f abc2 	beq.w	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 8001c60:	993e      	ldr	r1, [sp, #248]	@ 0xf8
 8001c62:	f8dd 0162 	ldr.w	r0, [sp, #354]	@ 0x162
 8001c66:	901d      	str	r0, [sp, #116]	@ 0x74
 8001c68:	f89d 0166 	ldrb.w	r0, [sp, #358]	@ 0x166
 8001c6c:	9043      	str	r0, [sp, #268]	@ 0x10c
 8001c6e:	6808      	ldr	r0, [r1, #0]
 8001c70:	6849      	ldr	r1, [r1, #4]
 8001c72:	9a3f      	ldr	r2, [sp, #252]	@ 0xfc
 8001c74:	f8bd b158 	ldrh.w	fp, [sp, #344]	@ 0x158
 8001c78:	e9cd 0152 	strd	r0, r1, [sp, #328]	@ 0x148
 8001c7c:	a8a3      	add	r0, sp, #652	@ 0x28c
 8001c7e:	9940      	ldr	r1, [sp, #256]	@ 0x100
 8001c80:	7913      	ldrb	r3, [r2, #4]
 8001c82:	6812      	ldr	r2, [r2, #0]
 8001c84:	921f      	str	r2, [sp, #124]	@ 0x7c
 8001c86:	2248      	movs	r2, #72	@ 0x48
 8001c88:	931e      	str	r3, [sp, #120]	@ 0x78
 8001c8a:	9d5b      	ldr	r5, [sp, #364]	@ 0x16c
 8001c8c:	f006 fe07 	bl	800889e <__aeabi_memcpy4>
 8001c90:	2c05      	cmp	r4, #5
 8001c92:	e9cd 5b1b 	strd	r5, fp, [sp, #108]	@ 0x6c
 8001c96:	d03c      	beq.n	8001d12 <smoltcp::iface::interface::Interface::poll+0xbf6>
 8001c98:	9a39      	ldr	r2, [sp, #228]	@ 0xe4
 8001c9a:	e9dd 0152 	ldrd	r0, r1, [sp, #328]	@ 0x148
 8001c9e:	f8ad b2f4 	strh.w	fp, [sp, #756]	@ 0x2f4
 8001ca2:	6010      	str	r0, [r2, #0]
 8001ca4:	983a      	ldr	r0, [sp, #232]	@ 0xe8
 8001ca6:	6051      	str	r1, [r2, #4]
 8001ca8:	2248      	movs	r2, #72	@ 0x48
 8001caa:	991f      	ldr	r1, [sp, #124]	@ 0x7c
 8001cac:	6001      	str	r1, [r0, #0]
 8001cae:	991e      	ldr	r1, [sp, #120]	@ 0x78
 8001cb0:	7101      	strb	r1, [r0, #4]
 8001cb2:	a9a3      	add	r1, sp, #652	@ 0x28c
 8001cb4:	9843      	ldr	r0, [sp, #268]	@ 0x10c
 8001cb6:	f88d 0302 	strb.w	r0, [sp, #770]	@ 0x302
 8001cba:	981d      	ldr	r0, [sp, #116]	@ 0x74
 8001cbc:	f8cd 02fe 	str.w	r0, [sp, #766]	@ 0x2fe
 8001cc0:	983b      	ldr	r0, [sp, #236]	@ 0xec
 8001cc2:	94bc      	str	r4, [sp, #752]	@ 0x2f0
 8001cc4:	95c2      	str	r5, [sp, #776]	@ 0x308
 8001cc6:	f006 fdea 	bl	800889e <__aeabi_memcpy4>
 8001cca:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8001ccc:	a94a      	add	r1, sp, #296	@ 0x128
 8001cce:	abbc      	add	r3, sp, #752	@ 0x2f0
 8001cd0:	2200      	movs	r2, #0
 8001cd2:	f7fe fb43 	bl	800035c <smoltcp::iface::interface::InterfaceInner::dispatch_ip>
 8001cd6:	b2c4      	uxtb	r4, r0
 8001cd8:	2c02      	cmp	r4, #2
 8001cda:	d01a      	beq.n	8001d12 <smoltcp::iface::interface::Interface::poll+0xbf6>
 8001cdc:	f240 0023 	movw	r0, #35	@ 0x23
 8001ce0:	f2c0 0000 	movt	r0, #0
 8001ce4:	f002 fb28 	bl	8004338 <defmt::export::acquire_and_header>
 8001ce8:	f240 003e 	movw	r0, #62	@ 0x3e
 8001cec:	adbc      	add	r5, sp, #752	@ 0x2f0
 8001cee:	f2c0 0000 	movt	r0, #0
 8001cf2:	2102      	movs	r1, #2
 8001cf4:	f8ad 02f0 	strh.w	r0, [sp, #752]	@ 0x2f0
 8001cf8:	4628      	mov	r0, r5
 8001cfa:	f002 fccf 	bl	800469c <_defmt_write>
 8001cfe:	f004 0001 	and.w	r0, r4, #1
 8001d02:	f88d 02f0 	strb.w	r0, [sp, #752]	@ 0x2f0
 8001d06:	4628      	mov	r0, r5
 8001d08:	2101      	movs	r1, #1
 8001d0a:	f002 fcc7 	bl	800469c <_defmt_write>
 8001d0e:	f002 fc4d 	bl	80045ac <_defmt_release>
 8001d12:	9e2e      	ldr	r6, [sp, #184]	@ 0xb8
 8001d14:	f7ff bb66 	b.w	80013e4 <smoltcp::iface::interface::Interface::poll+0x2c8>
 8001d18:	f1be 0f00 	cmp.w	lr, #0
 8001d1c:	d027      	beq.n	8001d6e <smoltcp::iface::interface::Interface::poll+0xc52>
 8001d1e:	9942      	ldr	r1, [sp, #264]	@ 0x108
 8001d20:	eb0e 008e 	add.w	r0, lr, lr, lsl #2
 8001d24:	4408      	add	r0, r1
 8001d26:	e010      	b.n	8001d4a <smoltcp::iface::interface::Interface::poll+0xc2e>
 8001d28:	2500      	movs	r5, #0
 8001d2a:	f002 021f 	and.w	r2, r2, #31
 8001d2e:	402b      	ands	r3, r5
 8001d30:	f04f 35ff 	mov.w	r5, #4294967295	@ 0xffffffff
 8001d34:	9c29      	ldr	r4, [sp, #164]	@ 0xa4
 8001d36:	fa25 f202 	lsr.w	r2, r5, r2
 8001d3a:	ba12      	rev	r2, r2
 8001d3c:	431a      	orrs	r2, r3
 8001d3e:	4294      	cmp	r4, r2
 8001d40:	f43f ae93 	beq.w	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 8001d44:	3105      	adds	r1, #5
 8001d46:	4281      	cmp	r1, r0
 8001d48:	d011      	beq.n	8001d6e <smoltcp::iface::interface::Interface::poll+0xc52>
 8001d4a:	790a      	ldrb	r2, [r1, #4]
 8001d4c:	680b      	ldr	r3, [r1, #0]
 8001d4e:	2a00      	cmp	r2, #0
 8001d50:	d0ea      	beq.n	8001d28 <smoltcp::iface::interface::Interface::poll+0xc0c>
 8001d52:	f1a2 051f 	sub.w	r5, r2, #31
 8001d56:	b2ed      	uxtb	r5, r5
 8001d58:	2d02      	cmp	r5, #2
 8001d5a:	d3f3      	bcc.n	8001d44 <smoltcp::iface::interface::Interface::poll+0xc28>
 8001d5c:	4255      	negs	r5, r2
 8001d5e:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 8001d62:	f005 051f 	and.w	r5, r5, #31
 8001d66:	fa04 f505 	lsl.w	r5, r4, r5
 8001d6a:	ba2d      	rev	r5, r5
 8001d6c:	e7dd      	b.n	8001d2a <smoltcp::iface::interface::Interface::poll+0xc0e>
 8001d6e:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8001d70:	2200      	movs	r2, #0
 8001d72:	e9d0 ec04 	ldrd	lr, ip, [r0, #16]
 8001d76:	f8d0 00e0 	ldr.w	r0, [r0, #224]	@ 0xe0
 8001d7a:	eba0 0080 	sub.w	r0, r0, r0, lsl #2
 8001d7e:	00c3      	lsls	r3, r0, #3
 8001d80:	1898      	adds	r0, r3, r2
 8001d82:	f43f ae72 	beq.w	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 8001d86:	9d48      	ldr	r5, [sp, #288]	@ 0x120
 8001d88:	9927      	ldr	r1, [sp, #156]	@ 0x9c
 8001d8a:	4415      	add	r5, r2
 8001d8c:	6a2c      	ldr	r4, [r5, #32]
 8001d8e:	428c      	cmp	r4, r1
 8001d90:	f000 8106 	beq.w	8001fa0 <smoltcp::iface::interface::Interface::poll+0xe84>
 8001d94:	f110 0418 	adds.w	r4, r0, #24
 8001d98:	f43f ae67 	beq.w	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 8001d9c:	6bac      	ldr	r4, [r5, #56]	@ 0x38
 8001d9e:	9927      	ldr	r1, [sp, #156]	@ 0x9c
 8001da0:	428c      	cmp	r4, r1
 8001da2:	f000 8100 	beq.w	8001fa6 <smoltcp::iface::interface::Interface::poll+0xe8a>
 8001da6:	f110 0430 	adds.w	r4, r0, #48	@ 0x30
 8001daa:	f43f ae5e 	beq.w	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 8001dae:	6d2c      	ldr	r4, [r5, #80]	@ 0x50
 8001db0:	9927      	ldr	r1, [sp, #156]	@ 0x9c
 8001db2:	428c      	cmp	r4, r1
 8001db4:	f000 80fa 	beq.w	8001fac <smoltcp::iface::interface::Interface::poll+0xe90>
 8001db8:	3048      	adds	r0, #72	@ 0x48
 8001dba:	f43f ae56 	beq.w	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 8001dbe:	6ea8      	ldr	r0, [r5, #104]	@ 0x68
 8001dc0:	3260      	adds	r2, #96	@ 0x60
 8001dc2:	9d27      	ldr	r5, [sp, #156]	@ 0x9c
 8001dc4:	42a8      	cmp	r0, r5
 8001dc6:	d1db      	bne.n	8001d80 <smoltcp::iface::interface::Interface::poll+0xc64>
 8001dc8:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8001dca:	4410      	add	r0, r2
 8001dcc:	3008      	adds	r0, #8
 8001dce:	e0ef      	b.n	8001fb0 <smoltcp::iface::interface::Interface::poll+0xe94>
 8001dd0:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001dd2:	8845      	ldrh	r5, [r0, #2]
 8001dd4:	2d00      	cmp	r5, #0
 8001dd6:	f43f af13 	beq.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001dda:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8001ddc:	7a40      	ldrb	r0, [r0, #9]
 8001dde:	2801      	cmp	r0, #1
 8001de0:	d837      	bhi.n	8001e52 <smoltcp::iface::interface::Interface::poll+0xd36>
 8001de2:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001de4:	88c0      	ldrh	r0, [r0, #6]
 8001de6:	b3a0      	cbz	r0, 8001e52 <smoltcp::iface::interface::Interface::poll+0xd36>
 8001de8:	9829      	ldr	r0, [sp, #164]	@ 0xa4
 8001dea:	ba00      	rev	r0, r0
 8001dec:	0c01      	lsrs	r1, r0, #16
 8001dee:	fa11 f080 	uxtah	r0, r1, r0
 8001df2:	0c01      	lsrs	r1, r0, #16
 8001df4:	fa11 f080 	uxtah	r0, r1, r0
 8001df8:	f109 0111 	add.w	r1, r9, #17
 8001dfc:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001e00:	0c0a      	lsrs	r2, r1, #16
 8001e02:	fa12 f181 	uxtah	r1, r2, r1
 8001e06:	b280      	uxth	r0, r0
 8001e08:	eb01 4111 	add.w	r1, r1, r1, lsr #16
 8001e0c:	fa10 f081 	uxtah	r0, r0, r1
 8001e10:	9927      	ldr	r1, [sp, #156]	@ 0x9c
 8001e12:	ba09      	rev	r1, r1
 8001e14:	0c0a      	lsrs	r2, r1, #16
 8001e16:	fa12 f181 	uxtah	r1, r2, r1
 8001e1a:	0c0a      	lsrs	r2, r1, #16
 8001e1c:	fa12 f181 	uxtah	r1, r2, r1
 8001e20:	eb01 4111 	add.w	r1, r1, r1, lsr #16
 8001e24:	fa10 f081 	uxtah	r0, r0, r1
 8001e28:	0c01      	lsrs	r1, r0, #16
 8001e2a:	fa11 f080 	uxtah	r0, r1, r0
 8001e2e:	4649      	mov	r1, r9
 8001e30:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001e34:	b284      	uxth	r4, r0
 8001e36:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001e38:	f006 f901 	bl	800803e <smoltcp::wire::ip::checksum::data>
 8001e3c:	fa14 f080 	uxtah	r0, r4, r0
 8001e40:	0c01      	lsrs	r1, r0, #16
 8001e42:	fa11 f080 	uxtah	r0, r1, r0
 8001e46:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 8001e4a:	43c0      	mvns	r0, r0
 8001e4c:	0400      	lsls	r0, r0, #16
 8001e4e:	f47f aed7 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001e52:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001e54:	f44f 71a8 	mov.w	r1, #336	@ 0x150
 8001e58:	9a26      	ldr	r2, [sp, #152]	@ 0x98
 8001e5a:	fb08 2401 	mla	r4, r8, r1, r2
 8001e5e:	ba6e      	rev16	r6, r5
 8001e60:	8800      	ldrh	r0, [r0, #0]
 8001e62:	ba00      	rev	r0, r0
 8001e64:	ea4f 4810 	mov.w	r8, r0, lsr #16
 8001e68:	e00c      	b.n	8001e84 <smoltcp::iface::interface::Interface::poll+0xd68>
 8001e6a:	9026      	str	r0, [sp, #152]	@ 0x98
 8001e6c:	4608      	mov	r0, r1
 8001e6e:	f100 0508 	add.w	r5, r0, #8
 8001e72:	9948      	ldr	r1, [sp, #288]	@ 0x120
 8001e74:	9a29      	ldr	r2, [sp, #164]	@ 0xa4
 8001e76:	4633      	mov	r3, r6
 8001e78:	4628      	mov	r0, r5
 8001e7a:	f005 ffd8 	bl	8007e2e <smoltcp::socket::udp::Socket::accepts>
 8001e7e:	2800      	cmp	r0, #0
 8001e80:	f040 80af 	bne.w	8001fe2 <smoltcp::iface::interface::Interface::poll+0xec6>
 8001e84:	9926      	ldr	r1, [sp, #152]	@ 0x98
 8001e86:	42a1      	cmp	r1, r4
 8001e88:	d066      	beq.n	8001f58 <smoltcp::iface::interface::Interface::poll+0xe3c>
 8001e8a:	e9d1 0200 	ldrd	r0, r2, [r1]
 8001e8e:	f080 0002 	eor.w	r0, r0, #2
 8001e92:	4310      	orrs	r0, r2
 8001e94:	f501 70a8 	add.w	r0, r1, #336	@ 0x150
 8001e98:	d0e7      	beq.n	8001e6a <smoltcp::iface::interface::Interface::poll+0xd4e>
 8001e9a:	42a0      	cmp	r0, r4
 8001e9c:	d05c      	beq.n	8001f58 <smoltcp::iface::interface::Interface::poll+0xe3c>
 8001e9e:	e9d1 2354 	ldrd	r2, r3, [r1, #336]	@ 0x150
 8001ea2:	f082 0202 	eor.w	r2, r2, #2
 8001ea6:	431a      	orrs	r2, r3
 8001ea8:	f501 7228 	add.w	r2, r1, #672	@ 0x2a0
 8001eac:	d101      	bne.n	8001eb2 <smoltcp::iface::interface::Interface::poll+0xd96>
 8001eae:	9226      	str	r2, [sp, #152]	@ 0x98
 8001eb0:	e7dd      	b.n	8001e6e <smoltcp::iface::interface::Interface::poll+0xd52>
 8001eb2:	42a2      	cmp	r2, r4
 8001eb4:	d050      	beq.n	8001f58 <smoltcp::iface::interface::Interface::poll+0xe3c>
 8001eb6:	e9d1 03a8 	ldrd	r0, r3, [r1, #672]	@ 0x2a0
 8001eba:	f080 0002 	eor.w	r0, r0, #2
 8001ebe:	4318      	orrs	r0, r3
 8001ec0:	f501 707c 	add.w	r0, r1, #1008	@ 0x3f0
 8001ec4:	d102      	bne.n	8001ecc <smoltcp::iface::interface::Interface::poll+0xdb0>
 8001ec6:	9026      	str	r0, [sp, #152]	@ 0x98
 8001ec8:	4610      	mov	r0, r2
 8001eca:	e7d0      	b.n	8001e6e <smoltcp::iface::interface::Interface::poll+0xd52>
 8001ecc:	42a0      	cmp	r0, r4
 8001ece:	d043      	beq.n	8001f58 <smoltcp::iface::interface::Interface::poll+0xe3c>
 8001ed0:	e9d1 23fc 	ldrd	r2, r3, [r1, #1008]	@ 0x3f0
 8001ed4:	f501 61a8 	add.w	r1, r1, #1344	@ 0x540
 8001ed8:	9126      	str	r1, [sp, #152]	@ 0x98
 8001eda:	f082 0202 	eor.w	r2, r2, #2
 8001ede:	431a      	orrs	r2, r3
 8001ee0:	d1d0      	bne.n	8001e84 <smoltcp::iface::interface::Interface::poll+0xd68>
 8001ee2:	e7c4      	b.n	8001e6e <smoltcp::iface::interface::Interface::poll+0xd52>
 8001ee4:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001ee6:	7840      	ldrb	r0, [r0, #1]
 8001ee8:	2800      	cmp	r0, #0
 8001eea:	f47f ae89 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001eee:	9a2c      	ldr	r2, [sp, #176]	@ 0xb0
 8001ef0:	2104      	movs	r1, #4
 8001ef2:	9155      	str	r1, [sp, #340]	@ 0x154
 8001ef4:	88d0      	ldrh	r0, [r2, #6]
 8001ef6:	8891      	ldrh	r1, [r2, #4]
 8001ef8:	ba00      	rev	r0, r0
 8001efa:	9a1a      	ldr	r2, [sp, #104]	@ 0x68
 8001efc:	eac2 4220 	pkhtb	r2, r2, r0, asr #16
 8001f00:	ba08      	rev	r0, r1
 8001f02:	921a      	str	r2, [sp, #104]	@ 0x68
 8001f04:	0c00      	lsrs	r0, r0, #16
 8001f06:	9015      	str	r0, [sp, #84]	@ 0x54
 8001f08:	e67c      	b.n	8001c04 <smoltcp::iface::interface::Interface::poll+0xae8>
 8001f0a:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001f0c:	7840      	ldrb	r0, [r0, #1]
 8001f0e:	2800      	cmp	r0, #0
 8001f10:	f47f ae76 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001f14:	982d      	ldr	r0, [sp, #180]	@ 0xb4
 8001f16:	2101      	movs	r1, #1
 8001f18:	9a2c      	ldr	r2, [sp, #176]	@ 0xb0
 8001f1a:	3808      	subs	r0, #8
 8001f1c:	90bf      	str	r0, [sp, #764]	@ 0x2fc
 8001f1e:	9824      	ldr	r0, [sp, #144]	@ 0x90
 8001f20:	9b29      	ldr	r3, [sp, #164]	@ 0xa4
 8001f22:	3008      	adds	r0, #8
 8001f24:	90be      	str	r0, [sp, #760]	@ 0x2f8
 8001f26:	88d0      	ldrh	r0, [r2, #6]
 8001f28:	f88d 12f0 	strb.w	r1, [sp, #752]	@ 0x2f0
 8001f2c:	ba04      	rev	r4, r0
 8001f2e:	8891      	ldrh	r1, [r2, #4]
 8001f30:	9a27      	ldr	r2, [sp, #156]	@ 0x9c
 8001f32:	0c20      	lsrs	r0, r4, #16
 8001f34:	f8ad 02f4 	strh.w	r0, [sp, #756]	@ 0x2f4
 8001f38:	ba08      	rev	r0, r1
 8001f3a:	9948      	ldr	r1, [sp, #288]	@ 0x120
 8001f3c:	0c00      	lsrs	r0, r0, #16
 8001f3e:	9015      	str	r0, [sp, #84]	@ 0x54
 8001f40:	f8ad 02f2 	strh.w	r0, [sp, #754]	@ 0x2f2
 8001f44:	a8bc      	add	r0, sp, #752	@ 0x2f0
 8001f46:	9000      	str	r0, [sp, #0]
 8001f48:	a855      	add	r0, sp, #340	@ 0x154
 8001f4a:	f005 fd24 	bl	8007996 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>
 8001f4e:	981a      	ldr	r0, [sp, #104]	@ 0x68
 8001f50:	eac0 4024 	pkhtb	r0, r0, r4, asr #16
 8001f54:	901a      	str	r0, [sp, #104]	@ 0x68
 8001f56:	e655      	b.n	8001c04 <smoltcp::iface::interface::Interface::poll+0xae8>
 8001f58:	9828      	ldr	r0, [sp, #160]	@ 0xa0
 8001f5a:	f8ad 030a 	strh.w	r0, [sp, #778]	@ 0x30a
 8001f5e:	2004      	movs	r0, #4
 8001f60:	f88d 0309 	strb.w	r0, [sp, #777]	@ 0x309
 8001f64:	9822      	ldr	r0, [sp, #136]	@ 0x88
 8001f66:	f88d 0308 	strb.w	r0, [sp, #776]	@ 0x308
 8001f6a:	9823      	ldr	r0, [sp, #140]	@ 0x8c
 8001f6c:	90c1      	str	r0, [sp, #772]	@ 0x304
 8001f6e:	f240 3002 	movw	r0, #770	@ 0x302
 8001f72:	f8ad 02f0 	strh.w	r0, [sp, #752]	@ 0x2f0
 8001f76:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8001f78:	9e2d      	ldr	r6, [sp, #180]	@ 0xb4
 8001f7a:	9b29      	ldr	r3, [sp, #164]	@ 0xa4
 8001f7c:	9a27      	ldr	r2, [sp, #156]	@ 0x9c
 8001f7e:	f5b6 7f04 	cmp.w	r6, #528	@ 0x210
 8001f82:	90bd      	str	r0, [sp, #756]	@ 0x2f4
 8001f84:	f44f 7004 	mov.w	r0, #528	@ 0x210
 8001f88:	93c0      	str	r3, [sp, #768]	@ 0x300
 8001f8a:	92bf      	str	r2, [sp, #764]	@ 0x2fc
 8001f8c:	bf28      	it	cs
 8001f8e:	4606      	movcs	r6, r0
 8001f90:	a8bc      	add	r0, sp, #752	@ 0x2f0
 8001f92:	9948      	ldr	r1, [sp, #288]	@ 0x120
 8001f94:	9000      	str	r0, [sp, #0]
 8001f96:	a855      	add	r0, sp, #340	@ 0x154
 8001f98:	96be      	str	r6, [sp, #760]	@ 0x2f8
 8001f9a:	f005 fcfc 	bl	8007996 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>
 8001f9e:	e631      	b.n	8001c04 <smoltcp::iface::interface::Interface::poll+0xae8>
 8001fa0:	f105 0020 	add.w	r0, r5, #32
 8001fa4:	e004      	b.n	8001fb0 <smoltcp::iface::interface::Interface::poll+0xe94>
 8001fa6:	f105 0038 	add.w	r0, r5, #56	@ 0x38
 8001faa:	e001      	b.n	8001fb0 <smoltcp::iface::interface::Interface::poll+0xe94>
 8001fac:	f105 0050 	add.w	r0, r5, #80	@ 0x50
 8001fb0:	992a      	ldr	r1, [sp, #168]	@ 0xa8
 8001fb2:	8a82      	ldrh	r2, [r0, #20]
 8001fb4:	6903      	ldr	r3, [r0, #16]
 8001fb6:	b28d      	uxth	r5, r1
 8001fb8:	992b      	ldr	r1, [sp, #172]	@ 0xac
 8001fba:	f8d7 9008 	ldr.w	r9, [r7, #8]
 8001fbe:	406a      	eors	r2, r5
 8001fc0:	f8dd a104 	ldr.w	sl, [sp, #260]	@ 0x104
 8001fc4:	404b      	eors	r3, r1
 8001fc6:	431a      	orrs	r2, r3
 8001fc8:	f47f ad4f 	bne.w	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 8001fcc:	f248 7200 	movw	r2, #34560	@ 0x8700
 8001fd0:	f2c0 3293 	movt	r2, #915	@ 0x393
 8001fd4:	eb1e 0102 	adds.w	r1, lr, r2
 8001fd8:	f14c 0200 	adc.w	r2, ip, #0
 8001fdc:	e9c0 1202 	strd	r1, r2, [r0, #8]
 8001fe0:	e543      	b.n	8001a6a <smoltcp::iface::interface::Interface::poll+0x94e>
 8001fe2:	9924      	ldr	r1, [sp, #144]	@ 0x90
 8001fe4:	f1a9 0008 	sub.w	r0, r9, #8
 8001fe8:	9a27      	ldr	r2, [sp, #156]	@ 0x9c
 8001fea:	3108      	adds	r1, #8
 8001fec:	9b29      	ldr	r3, [sp, #164]	@ 0xa4
 8001fee:	9002      	str	r0, [sp, #8]
 8001ff0:	4628      	mov	r0, r5
 8001ff2:	e9cd 8100 	strd	r8, r1, [sp]
 8001ff6:	9921      	ldr	r1, [sp, #132]	@ 0x84
 8001ff8:	f005 ff62 	bl	8007ec0 <smoltcp::socket::udp::Socket::process>
 8001ffc:	e600      	b.n	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8001ffe:	f415 6fa0 	tst.w	r5, #1280	@ 0x500
 8002002:	f47f adfd 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8002006:	2002      	movs	r0, #2
 8002008:	e003      	b.n	8002012 <smoltcp::iface::interface::Interface::poll+0xef6>
 800200a:	0568      	lsls	r0, r5, #21
 800200c:	f53f adf8 	bmi.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8002010:	2003      	movs	r0, #3
 8002012:	9014      	str	r0, [sp, #80]	@ 0x50
 8002014:	04e8      	lsls	r0, r5, #19
 8002016:	902a      	str	r0, [sp, #168]	@ 0xa8
 8002018:	d402      	bmi.n	8002020 <smoltcp::iface::interface::Interface::poll+0xf04>
 800201a:	2000      	movs	r0, #0
 800201c:	9013      	str	r0, [sp, #76]	@ 0x4c
 800201e:	e005      	b.n	800202c <smoltcp::iface::interface::Interface::poll+0xf10>
 8002020:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8002022:	6880      	ldr	r0, [r0, #8]
 8002024:	ba00      	rev	r0, r0
 8002026:	9004      	str	r0, [sp, #16]
 8002028:	2001      	movs	r0, #1
 800202a:	9013      	str	r0, [sp, #76]	@ 0x4c
 800202c:	3e14      	subs	r6, #20
 800202e:	f000 81a1 	beq.w	8002374 <smoltcp::iface::interface::Interface::poll+0x1258>
 8002032:	9824      	ldr	r0, [sp, #144]	@ 0x90
 8002034:	f04f 0a00 	mov.w	sl, #0
 8002038:	f04f 0b00 	mov.w	fp, #0
 800203c:	f100 0914 	add.w	r9, r0, #20
 8002040:	2000      	movs	r0, #0
 8002042:	9017      	str	r0, [sp, #92]	@ 0x5c
 8002044:	2000      	movs	r0, #0
 8002046:	9018      	str	r0, [sp, #96]	@ 0x60
 8002048:	2000      	movs	r0, #0
 800204a:	9019      	str	r0, [sp, #100]	@ 0x64
 800204c:	9021      	str	r0, [sp, #132]	@ 0x84
 800204e:	9016      	str	r0, [sp, #88]	@ 0x58
 8002050:	9025      	str	r0, [sp, #148]	@ 0x94
 8002052:	e002      	b.n	800205a <smoltcp::iface::interface::Interface::poll+0xf3e>
 8002054:	2e00      	cmp	r6, #0
 8002056:	f000 819c 	beq.w	8002392 <smoltcp::iface::interface::Interface::poll+0x1276>
 800205a:	f899 0000 	ldrb.w	r0, [r9]
 800205e:	b148      	cbz	r0, 8002074 <smoltcp::iface::interface::Interface::poll+0xf58>
 8002060:	2801      	cmp	r0, #1
 8002062:	d130      	bne.n	80020c6 <smoltcp::iface::interface::Interface::poll+0xfaa>
 8002064:	2501      	movs	r5, #1
 8002066:	2103      	movs	r1, #3
 8002068:	2000      	movs	r0, #0
 800206a:	f04f 0c00 	mov.w	ip, #0
 800206e:	2200      	movs	r2, #0
 8002070:	2300      	movs	r3, #0
 8002072:	e004      	b.n	800207e <smoltcp::iface::interface::Interface::poll+0xf62>
 8002074:	2501      	movs	r5, #1
 8002076:	2102      	movs	r1, #2
 8002078:	4602      	mov	r2, r0
 800207a:	4684      	mov	ip, r0
 800207c:	4603      	mov	r3, r0
 800207e:	42ae      	cmp	r6, r5
 8002080:	f0c1 816a 	bcc.w	8003358 <smoltcp::iface::interface::Interface::poll+0x223c>
 8002084:	290a      	cmp	r1, #10
 8002086:	f43f adbb 	beq.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 800208a:	021b      	lsls	r3, r3, #8
 800208c:	44a9      	add	r9, r5
 800208e:	b29b      	uxth	r3, r3
 8002090:	431a      	orrs	r2, r3
 8002092:	2305      	movs	r3, #5
 8002094:	2901      	cmp	r1, #1
 8002096:	eba6 0605 	sub.w	r6, r6, r5
 800209a:	bf88      	it	hi
 800209c:	1e8b      	subhi	r3, r1, #2
 800209e:	ea42 020c 	orr.w	r2, r2, ip
 80020a2:	e8df f013 	tbh	[pc, r3, lsl #1]
 80020a6:	0176      	.short	0x0176
 80020a8:	000c000b 	.word	0x000c000b
 80020ac:	00080049 	.word	0x00080049
 80020b0:	00ee00db 	.word	0x00ee00db
 80020b4:	000b      	.short	0x000b
 80020b6:	2001      	movs	r0, #1
 80020b8:	9017      	str	r0, [sp, #92]	@ 0x5c
 80020ba:	e7cb      	b.n	8002054 <smoltcp::iface::interface::Interface::poll+0xf38>
 80020bc:	e7ca      	b.n	8002054 <smoltcp::iface::interface::Interface::poll+0xf38>
 80020be:	2001      	movs	r0, #1
 80020c0:	920a      	str	r2, [sp, #40]	@ 0x28
 80020c2:	9018      	str	r0, [sp, #96]	@ 0x60
 80020c4:	e7c6      	b.n	8002054 <smoltcp::iface::interface::Interface::poll+0xf38>
 80020c6:	2e01      	cmp	r6, #1
 80020c8:	f43f ad9a 	beq.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 80020cc:	f899 2001 	ldrb.w	r2, [r9, #1]
 80020d0:	2100      	movs	r1, #0
 80020d2:	2300      	movs	r3, #0
 80020d4:	f109 0402 	add.w	r4, r9, #2
 80020d8:	b2d5      	uxtb	r5, r2
 80020da:	42ae      	cmp	r6, r5
 80020dc:	bf38      	it	cc
 80020de:	2101      	movcc	r1, #1
 80020e0:	2d02      	cmp	r5, #2
 80020e2:	bf38      	it	cc
 80020e4:	2301      	movcc	r3, #1
 80020e6:	4319      	orrs	r1, r3
 80020e8:	4621      	mov	r1, r4
 80020ea:	bf18      	it	ne
 80020ec:	2100      	movne	r1, #0
 80020ee:	f47f ad87 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 80020f2:	f06f 0301 	mvn.w	r3, #1
 80020f6:	fa53 f282 	uxtab	r2, r3, r2
 80020fa:	922b      	str	r2, [sp, #172]	@ 0xac
 80020fc:	1e82      	subs	r2, r0, #2
 80020fe:	2a06      	cmp	r2, #6
 8002100:	f200 812f 	bhi.w	8002362 <smoltcp::iface::interface::Interface::poll+0x1246>
 8002104:	e8df f012 	tbh	[pc, r2, lsl #1]
 8002108:	00c70007 	.word	0x00c70007
 800210c:	00d600d0 	.word	0x00d600d0
 8002110:	012d012d 	.word	0x012d012d
 8002114:	0111      	.short	0x0111
 8002116:	2d04      	cmp	r5, #4
 8002118:	f47f ad72 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 800211c:	982b      	ldr	r0, [sp, #172]	@ 0xac
 800211e:	2801      	cmp	r0, #1
 8002120:	f241 813c 	bls.w	800339c <smoltcp::iface::interface::Interface::poll+0x2280>
 8002124:	8820      	ldrh	r0, [r4, #0]
 8002126:	2104      	movs	r1, #4
 8002128:	2200      	movs	r2, #0
 800212a:	2504      	movs	r5, #4
 800212c:	ba00      	rev	r0, r0
 800212e:	0e03      	lsrs	r3, r0, #24
 8002130:	f3c0 4c07 	ubfx	ip, r0, #16, #8
 8002134:	2000      	movs	r0, #0
 8002136:	e7a2      	b.n	800207e <smoltcp::iface::interface::Interface::poll+0xf62>
 8002138:	f1bc 0f0e 	cmp.w	ip, #14
 800213c:	f240 80a7 	bls.w	800228e <smoltcp::iface::interface::Interface::poll+0x1172>
 8002140:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8002142:	8841      	ldrh	r1, [r0, #2]
 8002144:	8800      	ldrh	r0, [r0, #0]
 8002146:	9003      	str	r0, [sp, #12]
 8002148:	f240 000a 	movw	r0, #10
 800214c:	f2c0 0000 	movt	r0, #0
 8002150:	912b      	str	r1, [sp, #172]	@ 0xac
 8002152:	f002 f8f1 	bl	8004338 <defmt::export::acquire_and_header>
 8002156:	ad8a      	add	r5, sp, #552	@ 0x228
 8002158:	f240 0a07 	movw	sl, #7
 800215c:	f2c0 0a00 	movt	sl, #0
 8002160:	2102      	movs	r1, #2
 8002162:	4628      	mov	r0, r5
 8002164:	f8ad a228 	strh.w	sl, [sp, #552]	@ 0x228
 8002168:	f002 fa98 	bl	800469c <_defmt_write>
 800216c:	f240 0b3f 	movw	fp, #63	@ 0x3f
 8002170:	9827      	ldr	r0, [sp, #156]	@ 0x9c
 8002172:	f2c0 0b00 	movt	fp, #0
 8002176:	908a      	str	r0, [sp, #552]	@ 0x228
 8002178:	f8ad b360 	strh.w	fp, [sp, #864]	@ 0x360
 800217c:	f50d 7b58 	add.w	fp, sp, #864	@ 0x360
 8002180:	2102      	movs	r1, #2
 8002182:	4658      	mov	r0, fp
 8002184:	f002 fa8a 	bl	800469c <_defmt_write>
 8002188:	4658      	mov	r0, fp
 800218a:	2102      	movs	r1, #2
 800218c:	f8ad a360 	strh.w	sl, [sp, #864]	@ 0x360
 8002190:	f002 fa84 	bl	800469c <_defmt_write>
 8002194:	4628      	mov	r0, r5
 8002196:	f002 f87d 	bl	8004294 <defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>
 800219a:	2400      	movs	r4, #0
 800219c:	4658      	mov	r0, fp
 800219e:	2102      	movs	r1, #2
 80021a0:	f8ad 4360 	strh.w	r4, [sp, #864]	@ 0x360
 80021a4:	f002 fa7a 	bl	800469c <_defmt_write>
 80021a8:	4628      	mov	r0, r5
 80021aa:	2102      	movs	r1, #2
 80021ac:	f8ad 4228 	strh.w	r4, [sp, #552]	@ 0x228
 80021b0:	f002 fa74 	bl	800469c <_defmt_write>
 80021b4:	f240 0004 	movw	r0, #4
 80021b8:	2102      	movs	r1, #2
 80021ba:	f2c0 0000 	movt	r0, #0
 80021be:	f8ad 0228 	strh.w	r0, [sp, #552]	@ 0x228
 80021c2:	4628      	mov	r0, r5
 80021c4:	f002 fa6a 	bl	800469c <_defmt_write>
 80021c8:	9803      	ldr	r0, [sp, #12]
 80021ca:	2102      	movs	r1, #2
 80021cc:	ba00      	rev	r0, r0
 80021ce:	0c00      	lsrs	r0, r0, #16
 80021d0:	f8ad 0228 	strh.w	r0, [sp, #552]	@ 0x228
 80021d4:	4628      	mov	r0, r5
 80021d6:	f002 fa61 	bl	800469c <_defmt_write>
 80021da:	4628      	mov	r0, r5
 80021dc:	2102      	movs	r1, #2
 80021de:	f8ad a228 	strh.w	sl, [sp, #552]	@ 0x228
 80021e2:	f002 fa5b 	bl	800469c <_defmt_write>
 80021e6:	9829      	ldr	r0, [sp, #164]	@ 0xa4
 80021e8:	2102      	movs	r1, #2
 80021ea:	908a      	str	r0, [sp, #552]	@ 0x228
 80021ec:	f240 003f 	movw	r0, #63	@ 0x3f
 80021f0:	f2c0 0000 	movt	r0, #0
 80021f4:	f8ad 0360 	strh.w	r0, [sp, #864]	@ 0x360
 80021f8:	4658      	mov	r0, fp
 80021fa:	f002 fa4f 	bl	800469c <_defmt_write>
 80021fe:	4658      	mov	r0, fp
 8002200:	2102      	movs	r1, #2
 8002202:	f8ad a360 	strh.w	sl, [sp, #864]	@ 0x360
 8002206:	f002 fa49 	bl	800469c <_defmt_write>
 800220a:	4628      	mov	r0, r5
 800220c:	f002 f842 	bl	8004294 <defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>
 8002210:	4658      	mov	r0, fp
 8002212:	2102      	movs	r1, #2
 8002214:	f8ad 4360 	strh.w	r4, [sp, #864]	@ 0x360
 8002218:	f002 fa40 	bl	800469c <_defmt_write>
 800221c:	4628      	mov	r0, r5
 800221e:	2102      	movs	r1, #2
 8002220:	f8ad 4228 	strh.w	r4, [sp, #552]	@ 0x228
 8002224:	f002 fa3a 	bl	800469c <_defmt_write>
 8002228:	f240 0004 	movw	r0, #4
 800222c:	2102      	movs	r1, #2
 800222e:	f2c0 0000 	movt	r0, #0
 8002232:	f8ad 0228 	strh.w	r0, [sp, #552]	@ 0x228
 8002236:	4628      	mov	r0, r5
 8002238:	f002 fa30 	bl	800469c <_defmt_write>
 800223c:	982b      	ldr	r0, [sp, #172]	@ 0xac
 800223e:	2102      	movs	r1, #2
 8002240:	ba00      	rev	r0, r0
 8002242:	0c00      	lsrs	r0, r0, #16
 8002244:	f8ad 0228 	strh.w	r0, [sp, #552]	@ 0x228
 8002248:	4628      	mov	r0, r5
 800224a:	f002 fa27 	bl	800469c <_defmt_write>
 800224e:	f002 f9ad 	bl	80045ac <_defmt_release>
 8002252:	f04f 0a01 	mov.w	sl, #1
 8002256:	f04f 0b0e 	mov.w	fp, #14
 800225a:	e6fb      	b.n	8002054 <smoltcp::iface::interface::Interface::poll+0xf38>
 800225c:	9b35      	ldr	r3, [sp, #212]	@ 0xd4
 800225e:	9212      	str	r2, [sp, #72]	@ 0x48
 8002260:	9119      	str	r1, [sp, #100]	@ 0x64
 8002262:	ea40 2003 	orr.w	r0, r0, r3, lsl #8
 8002266:	9021      	str	r0, [sp, #132]	@ 0x84
 8002268:	982b      	ldr	r0, [sp, #172]	@ 0xac
 800226a:	9011      	str	r0, [sp, #68]	@ 0x44
 800226c:	9806      	ldr	r0, [sp, #24]
 800226e:	9010      	str	r0, [sp, #64]	@ 0x40
 8002270:	9807      	ldr	r0, [sp, #28]
 8002272:	9016      	str	r0, [sp, #88]	@ 0x58
 8002274:	9809      	ldr	r0, [sp, #36]	@ 0x24
 8002276:	900b      	str	r0, [sp, #44]	@ 0x2c
 8002278:	9805      	ldr	r0, [sp, #20]
 800227a:	900d      	str	r0, [sp, #52]	@ 0x34
 800227c:	9808      	ldr	r0, [sp, #32]
 800227e:	900c      	str	r0, [sp, #48]	@ 0x30
 8002280:	e6e8      	b.n	8002054 <smoltcp::iface::interface::Interface::poll+0xf38>
 8002282:	2001      	movs	r0, #1
 8002284:	9025      	str	r0, [sp, #148]	@ 0x94
 8002286:	982b      	ldr	r0, [sp, #172]	@ 0xac
 8002288:	e9cd 020e 	strd	r0, r2, [sp, #56]	@ 0x38
 800228c:	e6e2      	b.n	8002054 <smoltcp::iface::interface::Interface::poll+0xf38>
 800228e:	f04f 0a01 	mov.w	sl, #1
 8002292:	46e3      	mov	fp, ip
 8002294:	e6de      	b.n	8002054 <smoltcp::iface::interface::Interface::poll+0xf38>
 8002296:	2d03      	cmp	r5, #3
 8002298:	f47f acb2 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 800229c:	2503      	movs	r5, #3
 800229e:	f894 c000 	ldrb.w	ip, [r4]
 80022a2:	2105      	movs	r1, #5
 80022a4:	2000      	movs	r0, #0
 80022a6:	e6e2      	b.n	800206e <smoltcp::iface::interface::Interface::poll+0xf52>
 80022a8:	2d02      	cmp	r5, #2
 80022aa:	f47f aca9 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 80022ae:	2502      	movs	r5, #2
 80022b0:	2106      	movs	r1, #6
 80022b2:	e6d9      	b.n	8002068 <smoltcp::iface::interface::Interface::poll+0xf4c>
 80022b4:	2d0a      	cmp	r5, #10
 80022b6:	f4ff aca3 	bcc.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 80022ba:	f005 0007 	and.w	r0, r5, #7
 80022be:	2802      	cmp	r0, #2
 80022c0:	f47f ac9e 	bne.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 80022c4:	2d1a      	cmp	r5, #26
 80022c6:	d905      	bls.n	80022d4 <smoltcp::iface::interface::Interface::poll+0x11b8>
 80022c8:	f240 0009 	movw	r0, #9
 80022cc:	f2c0 0000 	movt	r0, #0
 80022d0:	f002 f843 	bl	800435a <defmt::export::acquire_header_and_release>
 80022d4:	2000      	movs	r0, #0
 80022d6:	9931      	ldr	r1, [sp, #196]	@ 0xc4
 80022d8:	9090      	str	r0, [sp, #576]	@ 0x240
 80022da:	aad8      	add	r2, sp, #864	@ 0x360
 80022dc:	908d      	str	r0, [sp, #564]	@ 0x234
 80022de:	908a      	str	r0, [sp, #552]	@ 0x228
 80022e0:	90da      	str	r0, [sp, #872]	@ 0x368
 80022e2:	982b      	ldr	r0, [sp, #172]	@ 0xac
 80022e4:	e9cd 40d8 	strd	r4, r0, [sp, #864]	@ 0x360
 80022e8:	a88a      	add	r0, sp, #552	@ 0x228
 80022ea:	f003 ff1b 	bl	8006124 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold>
 80022ee:	9a8c      	ldr	r2, [sp, #560]	@ 0x230
 80022f0:	922b      	str	r2, [sp, #172]	@ 0xac
 80022f2:	9a32      	ldr	r2, [sp, #200]	@ 0xc8
 80022f4:	e9dd 108a 	ldrd	r1, r0, [sp, #552]	@ 0x228
 80022f8:	fa5f fc80 	uxtb.w	ip, r0
 80022fc:	7893      	ldrb	r3, [r2, #2]
 80022fe:	8814      	ldrh	r4, [r2, #0]
 8002300:	9a92      	ldr	r2, [sp, #584]	@ 0x248
 8002302:	ea44 4303 	orr.w	r3, r4, r3, lsl #16
 8002306:	9209      	str	r2, [sp, #36]	@ 0x24
 8002308:	f64f 72ff 	movw	r2, #65535	@ 0xffff
 800230c:	9335      	str	r3, [sp, #212]	@ 0xd4
 800230e:	ea20 0202 	bic.w	r2, r0, r2
 8002312:	0a03      	lsrs	r3, r0, #8
 8002314:	9891      	ldr	r0, [sp, #580]	@ 0x244
 8002316:	9006      	str	r0, [sp, #24]
 8002318:	9890      	ldr	r0, [sp, #576]	@ 0x240
 800231a:	9007      	str	r0, [sp, #28]
 800231c:	988f      	ldr	r0, [sp, #572]	@ 0x23c
 800231e:	9c8e      	ldr	r4, [sp, #568]	@ 0x238
 8002320:	9008      	str	r0, [sp, #32]
 8002322:	f89d 0234 	ldrb.w	r0, [sp, #564]	@ 0x234
 8002326:	9405      	str	r4, [sp, #20]
 8002328:	e6a9      	b.n	800207e <smoltcp::iface::interface::Interface::poll+0xf62>
 800232a:	2d0a      	cmp	r5, #10
 800232c:	d119      	bne.n	8002362 <smoltcp::iface::interface::Interface::poll+0x1246>
 800232e:	982b      	ldr	r0, [sp, #172]	@ 0xac
 8002330:	2803      	cmp	r0, #3
 8002332:	f241 803c 	bls.w	80033ae <smoltcp::iface::interface::Interface::poll+0x2292>
 8002336:	982b      	ldr	r0, [sp, #172]	@ 0xac
 8002338:	2807      	cmp	r0, #7
 800233a:	f241 8041 	bls.w	80033c0 <smoltcp::iface::interface::Interface::poll+0x22a4>
 800233e:	f8d9 0002 	ldr.w	r0, [r9, #2]
 8002342:	f64f 72ff 	movw	r2, #65535	@ 0xffff
 8002346:	f8d9 1006 	ldr.w	r1, [r9, #6]
 800234a:	250a      	movs	r5, #10
 800234c:	ba00      	rev	r0, r0
 800234e:	ba09      	rev	r1, r1
 8002350:	912b      	str	r1, [sp, #172]	@ 0xac
 8002352:	ea20 0202 	bic.w	r2, r0, r2
 8002356:	0a03      	lsrs	r3, r0, #8
 8002358:	fa5f fc80 	uxtb.w	ip, r0
 800235c:	2108      	movs	r1, #8
 800235e:	2000      	movs	r0, #0
 8002360:	e68d      	b.n	800207e <smoltcp::iface::interface::Interface::poll+0xf62>
 8002362:	f64f 72ff 	movw	r2, #65535	@ 0xffff
 8002366:	0a0b      	lsrs	r3, r1, #8
 8002368:	ea21 0202 	bic.w	r2, r1, r2
 800236c:	fa5f fc81 	uxtb.w	ip, r1
 8002370:	2109      	movs	r1, #9
 8002372:	e684      	b.n	800207e <smoltcp::iface::interface::Interface::poll+0xf62>
 8002374:	2000      	movs	r0, #0
 8002376:	f04f 0b00 	mov.w	fp, #0
 800237a:	9025      	str	r0, [sp, #148]	@ 0x94
 800237c:	f04f 0a00 	mov.w	sl, #0
 8002380:	2000      	movs	r0, #0
 8002382:	9016      	str	r0, [sp, #88]	@ 0x58
 8002384:	9021      	str	r0, [sp, #132]	@ 0x84
 8002386:	9019      	str	r0, [sp, #100]	@ 0x64
 8002388:	2000      	movs	r0, #0
 800238a:	9018      	str	r0, [sp, #96]	@ 0x60
 800238c:	2000      	movs	r0, #0
 800238e:	9017      	str	r0, [sp, #92]	@ 0x5c
 8002390:	e001      	b.n	8002396 <smoltcp::iface::interface::Interface::poll+0x127a>
 8002392:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8002394:	8985      	ldrh	r5, [r0, #12]
 8002396:	203c      	movs	r0, #60	@ 0x3c
 8002398:	992d      	ldr	r1, [sp, #180]	@ 0xb4
 800239a:	ea00 0095 	and.w	r0, r0, r5, lsr #2
 800239e:	4281      	cmp	r1, r0
 80023a0:	f0c1 8017 	bcc.w	80033d2 <smoltcp::iface::interface::Interface::poll+0x22b6>
 80023a4:	f44f 71a8 	mov.w	r1, #336	@ 0x150
 80023a8:	9a26      	ldr	r2, [sp, #152]	@ 0x98
 80023aa:	fb08 2201 	mla	r2, r8, r1, r2
 80023ae:	9914      	ldr	r1, [sp, #80]	@ 0x50
 80023b0:	f88d 1225 	strb.w	r1, [sp, #549]	@ 0x225
 80023b4:	9917      	ldr	r1, [sp, #92]	@ 0x5c
 80023b6:	9b04      	ldr	r3, [sp, #16]
 80023b8:	f001 0101 	and.w	r1, r1, #1
 80023bc:	f88d 1224 	strb.w	r1, [sp, #548]	@ 0x224
 80023c0:	f00a 0101 	and.w	r1, sl, #1
 80023c4:	f88d 121c 	strb.w	r1, [sp, #540]	@ 0x21c
 80023c8:	992d      	ldr	r1, [sp, #180]	@ 0xb4
 80023ca:	9376      	str	r3, [sp, #472]	@ 0x1d8
 80023cc:	1a09      	subs	r1, r1, r0
 80023ce:	9185      	str	r1, [sp, #532]	@ 0x214
 80023d0:	9924      	ldr	r1, [sp, #144]	@ 0x90
 80023d2:	9b13      	ldr	r3, [sp, #76]	@ 0x4c
 80023d4:	4408      	add	r0, r1
 80023d6:	9084      	str	r0, [sp, #528]	@ 0x210
 80023d8:	980a      	ldr	r0, [sp, #40]	@ 0x28
 80023da:	f8ad 020e 	strh.w	r0, [sp, #526]	@ 0x20e
 80023de:	9818      	ldr	r0, [sp, #96]	@ 0x60
 80023e0:	f8ad 020c 	strh.w	r0, [sp, #524]	@ 0x20c
 80023e4:	980e      	ldr	r0, [sp, #56]	@ 0x38
 80023e6:	9082      	str	r0, [sp, #520]	@ 0x208
 80023e8:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 80023ea:	9081      	str	r0, [sp, #516]	@ 0x204
 80023ec:	9825      	ldr	r0, [sp, #148]	@ 0x94
 80023ee:	9080      	str	r0, [sp, #512]	@ 0x200
 80023f0:	980b      	ldr	r0, [sp, #44]	@ 0x2c
 80023f2:	907f      	str	r0, [sp, #508]	@ 0x1fc
 80023f4:	9810      	ldr	r0, [sp, #64]	@ 0x40
 80023f6:	907e      	str	r0, [sp, #504]	@ 0x1f8
 80023f8:	9816      	ldr	r0, [sp, #88]	@ 0x58
 80023fa:	907d      	str	r0, [sp, #500]	@ 0x1f4
 80023fc:	980c      	ldr	r0, [sp, #48]	@ 0x30
 80023fe:	907c      	str	r0, [sp, #496]	@ 0x1f0
 8002400:	980d      	ldr	r0, [sp, #52]	@ 0x34
 8002402:	907b      	str	r0, [sp, #492]	@ 0x1ec
 8002404:	9821      	ldr	r0, [sp, #132]	@ 0x84
 8002406:	907a      	str	r0, [sp, #488]	@ 0x1e8
 8002408:	9811      	ldr	r0, [sp, #68]	@ 0x44
 800240a:	9079      	str	r0, [sp, #484]	@ 0x1e4
 800240c:	9812      	ldr	r0, [sp, #72]	@ 0x48
 800240e:	9078      	str	r0, [sp, #480]	@ 0x1e0
 8002410:	9819      	ldr	r0, [sp, #100]	@ 0x64
 8002412:	9077      	str	r0, [sp, #476]	@ 0x1dc
 8002414:	982c      	ldr	r0, [sp, #176]	@ 0xb0
 8002416:	f88d b21d 	strb.w	fp, [sp, #541]	@ 0x21d
 800241a:	9375      	str	r3, [sp, #468]	@ 0x1d4
 800241c:	6845      	ldr	r5, [r0, #4]
 800241e:	8801      	ldrh	r1, [r0, #0]
 8002420:	89c4      	ldrh	r4, [r0, #14]
 8002422:	ba2d      	rev	r5, r5
 8002424:	f8b0 e002 	ldrh.w	lr, [r0, #2]
 8002428:	ba09      	rev	r1, r1
 800242a:	9586      	str	r5, [sp, #536]	@ 0x218
 800242c:	ba25      	rev	r5, r4
 800242e:	fa9e f89e 	rev16.w	r8, lr
 8002432:	ea4f 4c11 	mov.w	ip, r1, lsr #16
 8002436:	0c2d      	lsrs	r5, r5, #16
 8002438:	f8ad 8220 	strh.w	r8, [sp, #544]	@ 0x220
 800243c:	f8ad 5222 	strh.w	r5, [sp, #546]	@ 0x222
 8002440:	f8ad c21e 	strh.w	ip, [sp, #542]	@ 0x21e
 8002444:	9826      	ldr	r0, [sp, #152]	@ 0x98
 8002446:	4290      	cmp	r0, r2
 8002448:	f000 8080 	beq.w	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 800244c:	9826      	ldr	r0, [sp, #152]	@ 0x98
 800244e:	e9d0 5400 	ldrd	r5, r4, [r0]
 8002452:	f500 71a8 	add.w	r1, r0, #336	@ 0x150
 8002456:	3d02      	subs	r5, #2
 8002458:	f174 0500 	sbcs.w	r5, r4, #0
 800245c:	d202      	bcs.n	8002464 <smoltcp::iface::interface::Interface::poll+0x1348>
 800245e:	460c      	mov	r4, r1
 8002460:	9926      	ldr	r1, [sp, #152]	@ 0x98
 8002462:	e029      	b.n	80024b8 <smoltcp::iface::interface::Interface::poll+0x139c>
 8002464:	4291      	cmp	r1, r2
 8002466:	d071      	beq.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 8002468:	9b26      	ldr	r3, [sp, #152]	@ 0x98
 800246a:	e9d3 4054 	ldrd	r4, r0, [r3, #336]	@ 0x150
 800246e:	f503 7528 	add.w	r5, r3, #672	@ 0x2a0
 8002472:	3c02      	subs	r4, #2
 8002474:	f170 0000 	sbcs.w	r0, r0, #0
 8002478:	d201      	bcs.n	800247e <smoltcp::iface::interface::Interface::poll+0x1362>
 800247a:	462c      	mov	r4, r5
 800247c:	e01c      	b.n	80024b8 <smoltcp::iface::interface::Interface::poll+0x139c>
 800247e:	4295      	cmp	r5, r2
 8002480:	d064      	beq.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 8002482:	9926      	ldr	r1, [sp, #152]	@ 0x98
 8002484:	e9d1 04a8 	ldrd	r0, r4, [r1, #672]	@ 0x2a0
 8002488:	f501 717c 	add.w	r1, r1, #1008	@ 0x3f0
 800248c:	3802      	subs	r0, #2
 800248e:	f174 0000 	sbcs.w	r0, r4, #0
 8002492:	d202      	bcs.n	800249a <smoltcp::iface::interface::Interface::poll+0x137e>
 8002494:	460c      	mov	r4, r1
 8002496:	4629      	mov	r1, r5
 8002498:	e00e      	b.n	80024b8 <smoltcp::iface::interface::Interface::poll+0x139c>
 800249a:	4291      	cmp	r1, r2
 800249c:	d056      	beq.n	800254c <smoltcp::iface::interface::Interface::poll+0x1430>
 800249e:	9b26      	ldr	r3, [sp, #152]	@ 0x98
 80024a0:	e9d3 04fc 	ldrd	r0, r4, [r3, #1008]	@ 0x3f0
 80024a4:	f503 63a8 	add.w	r3, r3, #1344	@ 0x540
 80024a8:	9326      	str	r3, [sp, #152]	@ 0x98
 80024aa:	f1d0 0001 	rsbs	r0, r0, #1
 80024ae:	f04f 0000 	mov.w	r0, #0
 80024b2:	41a0      	sbcs	r0, r4
 80024b4:	461c      	mov	r4, r3
 80024b6:	d3c5      	bcc.n	8002444 <smoltcp::iface::interface::Interface::poll+0x1328>
 80024b8:	f891 5133 	ldrb.w	r5, [r1, #307]	@ 0x133
 80024bc:	9426      	str	r4, [sp, #152]	@ 0x98
 80024be:	2d00      	cmp	r5, #0
 80024c0:	d0c0      	beq.n	8002444 <smoltcp::iface::interface::Interface::poll+0x1328>
 80024c2:	2d01      	cmp	r5, #1
 80024c4:	d103      	bne.n	80024ce <smoltcp::iface::interface::Interface::poll+0x13b2>
 80024c6:	982a      	ldr	r0, [sp, #168]	@ 0xa8
 80024c8:	9426      	str	r4, [sp, #152]	@ 0x98
 80024ca:	2800      	cmp	r0, #0
 80024cc:	d4ba      	bmi.n	8002444 <smoltcp::iface::interface::Interface::poll+0x1328>
 80024ce:	f8b1 011c 	ldrh.w	r0, [r1, #284]	@ 0x11c
 80024d2:	b1a0      	cbz	r0, 80024fe <smoltcp::iface::interface::Interface::poll+0x13e2>
 80024d4:	f8d1 011e 	ldr.w	r0, [r1, #286]	@ 0x11e
 80024d8:	9b29      	ldr	r3, [sp, #164]	@ 0xa4
 80024da:	9426      	str	r4, [sp, #152]	@ 0x98
 80024dc:	4283      	cmp	r3, r0
 80024de:	d1b1      	bne.n	8002444 <smoltcp::iface::interface::Interface::poll+0x1328>
 80024e0:	f8b1 0122 	ldrh.w	r0, [r1, #290]	@ 0x122
 80024e4:	9426      	str	r4, [sp, #152]	@ 0x98
 80024e6:	4580      	cmp	r8, r0
 80024e8:	d1ac      	bne.n	8002444 <smoltcp::iface::interface::Interface::poll+0x1328>
 80024ea:	f8d1 0124 	ldr.w	r0, [r1, #292]	@ 0x124
 80024ee:	9b27      	ldr	r3, [sp, #156]	@ 0x9c
 80024f0:	9426      	str	r4, [sp, #152]	@ 0x98
 80024f2:	4283      	cmp	r3, r0
 80024f4:	d1a6      	bne.n	8002444 <smoltcp::iface::interface::Interface::poll+0x1328>
 80024f6:	f8b1 0128 	ldrh.w	r0, [r1, #296]	@ 0x128
 80024fa:	4584      	cmp	ip, r0
 80024fc:	e012      	b.n	8002524 <smoltcp::iface::interface::Interface::poll+0x1408>
 80024fe:	f8d1 00b3 	ldr.w	r0, [r1, #179]	@ 0xb3
 8002502:	9b29      	ldr	r3, [sp, #164]	@ 0xa4
 8002504:	f891 50b2 	ldrb.w	r5, [r1, #178]	@ 0xb2
 8002508:	1a18      	subs	r0, r3, r0
 800250a:	bf18      	it	ne
 800250c:	2001      	movne	r0, #1
 800250e:	4205      	tst	r5, r0
 8002510:	9426      	str	r4, [sp, #152]	@ 0x98
 8002512:	d197      	bne.n	8002444 <smoltcp::iface::interface::Interface::poll+0x1328>
 8002514:	f1be 0f00 	cmp.w	lr, #0
 8002518:	9426      	str	r4, [sp, #152]	@ 0x98
 800251a:	f43f af93 	beq.w	8002444 <smoltcp::iface::interface::Interface::poll+0x1328>
 800251e:	f8b1 00b0 	ldrh.w	r0, [r1, #176]	@ 0xb0
 8002522:	4580      	cmp	r8, r0
 8002524:	9426      	str	r4, [sp, #152]	@ 0x98
 8002526:	f47f af8d 	bne.w	8002444 <smoltcp::iface::interface::Interface::poll+0x1328>
 800252a:	a875      	add	r0, sp, #468	@ 0x1d4
 800252c:	9a29      	ldr	r2, [sp, #164]	@ 0xa4
 800252e:	9b27      	ldr	r3, [sp, #156]	@ 0x9c
 8002530:	e9cd 2000 	strd	r2, r0, [sp]
 8002534:	a88a      	add	r0, sp, #552	@ 0x228
 8002536:	9a48      	ldr	r2, [sp, #288]	@ 0x120
 8002538:	f003 ffa8 	bl	800648c <smoltcp::socket::tcp::Socket::process>
 800253c:	988e      	ldr	r0, [sp, #568]	@ 0x238
 800253e:	2802      	cmp	r0, #2
 8002540:	f43f ab5e 	beq.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 8002544:	ab8a      	add	r3, sp, #552	@ 0x228
 8002546:	9d2f      	ldr	r5, [sp, #188]	@ 0xbc
 8002548:	aca3      	add	r4, sp, #652	@ 0x28c
 800254a:	e017      	b.n	800257c <smoltcp::iface::interface::Interface::poll+0x1460>
 800254c:	9814      	ldr	r0, [sp, #80]	@ 0x50
 800254e:	9929      	ldr	r1, [sp, #164]	@ 0xa4
 8002550:	3804      	subs	r0, #4
 8002552:	fab0 f080 	clz	r0, r0
 8002556:	fab1 f181 	clz	r1, r1
 800255a:	0949      	lsrs	r1, r1, #5
 800255c:	0940      	lsrs	r0, r0, #5
 800255e:	4308      	orrs	r0, r1
 8002560:	9920      	ldr	r1, [sp, #128]	@ 0x80
 8002562:	4308      	orrs	r0, r1
 8002564:	2801      	cmp	r0, #1
 8002566:	f43f ab4b 	beq.w	8001c00 <smoltcp::iface::interface::Interface::poll+0xae4>
 800256a:	9927      	ldr	r1, [sp, #156]	@ 0x9c
 800256c:	a88a      	add	r0, sp, #552	@ 0x228
 800256e:	9a29      	ldr	r2, [sp, #164]	@ 0xa4
 8002570:	ab75      	add	r3, sp, #468	@ 0x1d4
 8002572:	f005 f89a 	bl	80076aa <smoltcp::socket::tcp::Socket::rst_reply>
 8002576:	ab8a      	add	r3, sp, #552	@ 0x228
 8002578:	acbc      	add	r4, sp, #752	@ 0x2f0
 800257a:	9d30      	ldr	r5, [sp, #192]	@ 0xc0
 800257c:	cb0f      	ldmia	r3, {r0, r1, r2, r3}
 800257e:	c50f      	stmia	r5!, {r0, r1, r2, r3}
 8002580:	9934      	ldr	r1, [sp, #208]	@ 0xd0
 8002582:	4620      	mov	r0, r4
 8002584:	2254      	movs	r2, #84	@ 0x54
 8002586:	f006 f98a 	bl	800889e <__aeabi_memcpy4>
 800258a:	a855      	add	r0, sp, #340	@ 0x154
 800258c:	4621      	mov	r1, r4
 800258e:	2264      	movs	r2, #100	@ 0x64
 8002590:	f006 f985 	bl	800889e <__aeabi_memcpy4>
 8002594:	f7ff bb36 	b.w	8001c04 <smoltcp::iface::interface::Interface::poll+0xae8>
 8002598:	68f8      	ldr	r0, [r7, #12]
 800259a:	f44f 71a8 	mov.w	r1, #336	@ 0x150
 800259e:	e9d0 5000 	ldrd	r5, r0, [r0]
 80025a2:	fb00 5001 	mla	r0, r0, r1, r5
 80025a6:	e9dd 2145 	ldrd	r2, r1, [sp, #276]	@ 0x114
 80025aa:	9047      	str	r0, [sp, #284]	@ 0x11c
 80025ac:	9848      	ldr	r0, [sp, #288]	@ 0x120
 80025ae:	e9c0 2104 	strd	r2, r1, [r0, #16]
 80025b2:	a8bc      	add	r0, sp, #752	@ 0x2f0
 80025b4:	3008      	adds	r0, #8
 80025b6:	9036      	str	r0, [sp, #216]	@ 0xd8
 80025b8:	9847      	ldr	r0, [sp, #284]	@ 0x11c
 80025ba:	4285      	cmp	r5, r0
 80025bc:	f000 8606 	beq.w	80031cc <smoltcp::iface::interface::Interface::poll+0x20b0>
 80025c0:	46a9      	mov	r9, r5
 80025c2:	e8f5 0154 	ldrd	r0, r1, [r5], #336	@ 0x150
 80025c6:	f080 0003 	eor.w	r0, r0, #3
 80025ca:	4308      	orrs	r0, r1
 80025cc:	d0f4      	beq.n	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 80025ce:	f899 0138 	ldrb.w	r0, [r9, #312]	@ 0x138
 80025d2:	2801      	cmp	r0, #1
 80025d4:	f040 8089 	bne.w	80026ea <smoltcp::iface::interface::Interface::poll+0x15ce>
 80025d8:	9848      	ldr	r0, [sp, #288]	@ 0x120
 80025da:	46a8      	mov	r8, r5
 80025dc:	f8d9 2139 	ldr.w	r2, [r9, #313]	@ 0x139
 80025e0:	f04f 34ff 	mov.w	r4, #4294967295	@ 0xffffffff
 80025e4:	e9d0 ab04 	ldrd	sl, fp, [r0, #16]
 80025e8:	f8d0 00f0 	ldr.w	r0, [r0, #240]	@ 0xf0
 80025ec:	e9d9 c550 	ldrd	ip, r5, [r9, #320]	@ 0x140
 80025f0:	9942      	ldr	r1, [sp, #264]	@ 0x108
 80025f2:	eb00 0080 	add.w	r0, r0, r0, lsl #2
 80025f6:	e00b      	b.n	8002610 <smoltcp::iface::interface::Interface::poll+0x14f4>
 80025f8:	425b      	negs	r3, r3
 80025fa:	f003 031f 	and.w	r3, r3, #31
 80025fe:	fa04 f303 	lsl.w	r3, r4, r3
 8002602:	ba1b      	rev	r3, r3
 8002604:	680e      	ldr	r6, [r1, #0]
 8002606:	3105      	adds	r1, #5
 8002608:	3805      	subs	r0, #5
 800260a:	4056      	eors	r6, r2
 800260c:	4233      	tst	r3, r6
 800260e:	d016      	beq.n	800263e <smoltcp::iface::interface::Interface::poll+0x1522>
 8002610:	b120      	cbz	r0, 800261c <smoltcp::iface::interface::Interface::poll+0x1500>
 8002612:	790b      	ldrb	r3, [r1, #4]
 8002614:	2b00      	cmp	r3, #0
 8002616:	d1ef      	bne.n	80025f8 <smoltcp::iface::interface::Interface::poll+0x14dc>
 8002618:	2300      	movs	r3, #0
 800261a:	e7f3      	b.n	8002604 <smoltcp::iface::interface::Interface::poll+0x14e8>
 800261c:	1c50      	adds	r0, r2, #1
 800261e:	f000 85b9 	beq.w	8003194 <smoltcp::iface::interface::Interface::poll+0x2078>
 8002622:	9933      	ldr	r1, [sp, #204]	@ 0xcc
 8002624:	a8bc      	add	r0, sp, #752	@ 0x2f0
 8002626:	e9cd ab00 	strd	sl, fp, [sp]
 800262a:	4664      	mov	r4, ip
 800262c:	f005 fe6d 	bl	800830a <smoltcp::iface::route::Routes::lookup>
 8002630:	f89d 02f0 	ldrb.w	r0, [sp, #752]	@ 0x2f0
 8002634:	46a4      	mov	ip, r4
 8002636:	2801      	cmp	r0, #1
 8002638:	d150      	bne.n	80026dc <smoltcp::iface::interface::Interface::poll+0x15c0>
 800263a:	f8dd 22f1 	ldr.w	r2, [sp, #753]	@ 0x2f1
 800263e:	1c50      	adds	r0, r2, #1
 8002640:	f000 85a8 	beq.w	8003194 <smoltcp::iface::interface::Interface::poll+0x2078>
 8002644:	f002 00f0 	and.w	r0, r2, #240	@ 0xf0
 8002648:	28e0      	cmp	r0, #224	@ 0xe0
 800264a:	bf1c      	itt	ne
 800264c:	b2d0      	uxtbne	r0, r2
 800264e:	ea50 2012 	orrsne.w	r0, r0, r2, lsr #8
 8002652:	f000 859f 	beq.w	8003194 <smoltcp::iface::interface::Interface::poll+0x2078>
 8002656:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8002658:	f8d0 00e0 	ldr.w	r0, [r0, #224]	@ 0xe0
 800265c:	eba0 0080 	sub.w	r0, r0, r0, lsl #2
 8002660:	00c1      	lsls	r1, r0, #3
 8002662:	2000      	movs	r0, #0
 8002664:	180e      	adds	r6, r1, r0
 8002666:	d039      	beq.n	80026dc <smoltcp::iface::interface::Interface::poll+0x15c0>
 8002668:	9b48      	ldr	r3, [sp, #288]	@ 0x120
 800266a:	4403      	add	r3, r0
 800266c:	6a1c      	ldr	r4, [r3, #32]
 800266e:	4294      	cmp	r4, r2
 8002670:	d01d      	beq.n	80026ae <smoltcp::iface::interface::Interface::poll+0x1592>
 8002672:	f116 0418 	adds.w	r4, r6, #24
 8002676:	d031      	beq.n	80026dc <smoltcp::iface::interface::Interface::poll+0x15c0>
 8002678:	6b9c      	ldr	r4, [r3, #56]	@ 0x38
 800267a:	4294      	cmp	r4, r2
 800267c:	d025      	beq.n	80026ca <smoltcp::iface::interface::Interface::poll+0x15ae>
 800267e:	f116 0430 	adds.w	r4, r6, #48	@ 0x30
 8002682:	d02b      	beq.n	80026dc <smoltcp::iface::interface::Interface::poll+0x15c0>
 8002684:	6d1c      	ldr	r4, [r3, #80]	@ 0x50
 8002686:	4294      	cmp	r4, r2
 8002688:	f000 818a 	beq.w	80029a0 <smoltcp::iface::interface::Interface::poll+0x1884>
 800268c:	3648      	adds	r6, #72	@ 0x48
 800268e:	d025      	beq.n	80026dc <smoltcp::iface::interface::Interface::poll+0x15c0>
 8002690:	6e9b      	ldr	r3, [r3, #104]	@ 0x68
 8002692:	3060      	adds	r0, #96	@ 0x60
 8002694:	4293      	cmp	r3, r2
 8002696:	d1e5      	bne.n	8002664 <smoltcp::iface::interface::Interface::poll+0x1548>
 8002698:	9948      	ldr	r1, [sp, #288]	@ 0x120
 800269a:	4408      	add	r0, r1
 800269c:	3008      	adds	r0, #8
 800269e:	e9d0 0102 	ldrd	r0, r1, [r0, #8]
 80026a2:	ebba 0000 	subs.w	r0, sl, r0
 80026a6:	eb7b 0001 	sbcs.w	r0, fp, r1
 80026aa:	db09      	blt.n	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 80026ac:	e016      	b.n	80026dc <smoltcp::iface::interface::Interface::poll+0x15c0>
 80026ae:	f103 0020 	add.w	r0, r3, #32
 80026b2:	e9d0 0102 	ldrd	r0, r1, [r0, #8]
 80026b6:	ebba 0000 	subs.w	r0, sl, r0
 80026ba:	eb7b 0001 	sbcs.w	r0, fp, r1
 80026be:	da0d      	bge.n	80026dc <smoltcp::iface::interface::Interface::poll+0x15c0>
 80026c0:	2000      	movs	r0, #0
 80026c2:	4645      	mov	r5, r8
 80026c4:	f889 0138 	strb.w	r0, [r9, #312]	@ 0x138
 80026c8:	e00f      	b.n	80026ea <smoltcp::iface::interface::Interface::poll+0x15ce>
 80026ca:	f103 0038 	add.w	r0, r3, #56	@ 0x38
 80026ce:	e9d0 0102 	ldrd	r0, r1, [r0, #8]
 80026d2:	ebba 0000 	subs.w	r0, sl, r0
 80026d6:	eb7b 0001 	sbcs.w	r0, fp, r1
 80026da:	dbf1      	blt.n	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 80026dc:	ebba 000c 	subs.w	r0, sl, ip
 80026e0:	eb7b 0005 	sbcs.w	r0, fp, r5
 80026e4:	4645      	mov	r5, r8
 80026e6:	f6ff af67 	blt.w	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 80026ea:	e9d9 0100 	ldrd	r0, r1, [r9]
 80026ee:	f080 0002 	eor.w	r0, r0, #2
 80026f2:	4308      	orrs	r0, r1
 80026f4:	d140      	bne.n	8002778 <smoltcp::iface::interface::Interface::poll+0x165c>
 80026f6:	f899 0050 	ldrb.w	r0, [r9, #80]	@ 0x50
 80026fa:	f8d9 b03c 	ldr.w	fp, [r9, #60]	@ 0x3c
 80026fe:	f899 e051 	ldrb.w	lr, [r9, #81]	@ 0x51
 8002702:	2800      	cmp	r0, #0
 8002704:	bf08      	it	eq
 8002706:	f04f 0e40 	moveq.w	lr, #64	@ 0x40
 800270a:	f1bb 0f00 	cmp.w	fp, #0
 800270e:	f000 80d7 	beq.w	80028c0 <smoltcp::iface::interface::Interface::poll+0x17a4>
 8002712:	f8d9 1034 	ldr.w	r1, [r9, #52]	@ 0x34
 8002716:	2900      	cmp	r1, #0
 8002718:	f000 85dd 	beq.w	80032d6 <smoltcp::iface::interface::Interface::poll+0x21ba>
 800271c:	f8d9 0038 	ldr.w	r0, [r9, #56]	@ 0x38
 8002720:	4288      	cmp	r0, r1
 8002722:	f080 85e5 	bcs.w	80032f0 <smoltcp::iface::interface::Interface::poll+0x21d4>
 8002726:	f8d9 a030 	ldr.w	sl, [r9, #48]	@ 0x30
 800272a:	eb00 0480 	add.w	r4, r0, r0, lsl #2
 800272e:	e9d9 2302 	ldrd	r2, r3, [r9, #8]
 8002732:	9244      	str	r2, [sp, #272]	@ 0x110
 8002734:	eb0a 0284 	add.w	r2, sl, r4, lsl #2
 8002738:	933d      	str	r3, [sp, #244]	@ 0xf4
 800273a:	7b92      	ldrb	r2, [r2, #14]
 800273c:	2a02      	cmp	r2, #2
 800273e:	d12b      	bne.n	8002798 <smoltcp::iface::interface::Interface::poll+0x167c>
 8002740:	f109 0844 	add.w	r8, r9, #68	@ 0x44
 8002744:	46ac      	mov	ip, r5
 8002746:	e898 0124 	ldmia.w	r8, {r2, r5, r8}
 800274a:	1b53      	subs	r3, r2, r5
 800274c:	4543      	cmp	r3, r8
 800274e:	bf28      	it	cs
 8002750:	4643      	movcs	r3, r8
 8002752:	195e      	adds	r6, r3, r5
 8002754:	f080 8579 	bcs.w	800324a <smoltcp::iface::interface::Interface::poll+0x212e>
 8002758:	4296      	cmp	r6, r2
 800275a:	f200 8576 	bhi.w	800324a <smoltcp::iface::interface::Interface::poll+0x212e>
 800275e:	f85a 6024 	ldr.w	r6, [sl, r4, lsl #2]
 8002762:	42b3      	cmp	r3, r6
 8002764:	bf38      	it	cc
 8002766:	461e      	movcc	r6, r3
 8002768:	2a00      	cmp	r2, #0
 800276a:	d061      	beq.n	8002830 <smoltcp::iface::interface::Interface::poll+0x1714>
 800276c:	1973      	adds	r3, r6, r5
 800276e:	fbb3 f4f2 	udiv	r4, r3, r2
 8002772:	fb04 3212 	mls	r2, r4, r2, r3
 8002776:	e05c      	b.n	8002832 <smoltcp::iface::interface::Interface::poll+0x1716>
 8002778:	f8b9 011c 	ldrh.w	r0, [r9, #284]	@ 0x11c
 800277c:	2800      	cmp	r0, #0
 800277e:	f43f af1b 	beq.w	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 8002782:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8002784:	e9d9 2314 	ldrd	r2, r3, [r9, #80]	@ 0x50
 8002788:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 800278c:	431a      	orrs	r2, r3
 800278e:	9546      	str	r5, [sp, #280]	@ 0x118
 8002790:	d005      	beq.n	800279e <smoltcp::iface::interface::Interface::poll+0x1682>
 8002792:	e9d9 2316 	ldrd	r2, r3, [r9, #88]	@ 0x58
 8002796:	e00c      	b.n	80027b2 <smoltcp::iface::interface::Interface::poll+0x1696>
 8002798:	f8dd a110 	ldr.w	sl, [sp, #272]	@ 0x110
 800279c:	e05a      	b.n	8002854 <smoltcp::iface::interface::Interface::poll+0x1738>
 800279e:	2201      	movs	r2, #1
 80027a0:	f8c9 005c 	str.w	r0, [r9, #92]	@ 0x5c
 80027a4:	f8c9 2050 	str.w	r2, [r9, #80]	@ 0x50
 80027a8:	2200      	movs	r2, #0
 80027aa:	e9c9 2115 	strd	r2, r1, [r9, #84]	@ 0x54
 80027ae:	460a      	mov	r2, r1
 80027b0:	4603      	mov	r3, r0
 80027b2:	e9d9 540e 	ldrd	r5, r4, [r9, #56]	@ 0x38
 80027b6:	f8d9 6030 	ldr.w	r6, [r9, #48]	@ 0x30
 80027ba:	1952      	adds	r2, r2, r5
 80027bc:	4163      	adcs	r3, r4
 80027be:	07f6      	lsls	r6, r6, #31
 80027c0:	d00c      	beq.n	80027dc <smoltcp::iface::interface::Interface::poll+0x16c0>
 80027c2:	1a89      	subs	r1, r1, r2
 80027c4:	4198      	sbcs	r0, r3
 80027c6:	db09      	blt.n	80027dc <smoltcp::iface::interface::Interface::poll+0x16c0>
 80027c8:	f240 0019 	movw	r0, #25
 80027cc:	f2c0 0000 	movt	r0, #0
 80027d0:	f001 fdc3 	bl	800435a <defmt::export::acquire_header_and_release>
 80027d4:	2000      	movs	r0, #0
 80027d6:	f889 0133 	strb.w	r0, [r9, #307]	@ 0x133
 80027da:	e172      	b.n	8002ac2 <smoltcp::iface::interface::Interface::poll+0x19a6>
 80027dc:	9848      	ldr	r0, [sp, #288]	@ 0x120
 80027de:	68c1      	ldr	r1, [r0, #12]
 80027e0:	4648      	mov	r0, r9
 80027e2:	f003 fdf3 	bl	80063cc <smoltcp::socket::tcp::Socket::seq_to_transmit>
 80027e6:	2800      	cmp	r0, #0
 80027e8:	f040 816b 	bne.w	8002ac2 <smoltcp::iface::interface::Interface::poll+0x19a6>
 80027ec:	f8d9 0080 	ldr.w	r0, [r9, #128]	@ 0x80
 80027f0:	2802      	cmp	r0, #2
 80027f2:	f000 80e0 	beq.w	80029b6 <smoltcp::iface::interface::Interface::poll+0x189a>
 80027f6:	2801      	cmp	r0, #1
 80027f8:	f040 8163 	bne.w	8002ac2 <smoltcp::iface::interface::Interface::poll+0x19a6>
 80027fc:	9848      	ldr	r0, [sp, #288]	@ 0x120
 80027fe:	e9d9 3222 	ldrd	r3, r2, [r9, #136]	@ 0x88
 8002802:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 8002806:	1ace      	subs	r6, r1, r3
 8002808:	eb70 0602 	sbcs.w	r6, r0, r2
 800280c:	f2c0 8159 	blt.w	8002ac2 <smoltcp::iface::interface::Interface::poll+0x19a6>
 8002810:	1ac9      	subs	r1, r1, r3
 8002812:	e9d9 6424 	ldrd	r6, r4, [r9, #144]	@ 0x90
 8002816:	4190      	sbcs	r0, r2
 8002818:	ea81 71e0 	eor.w	r1, r1, r0, asr #31
 800281c:	ea80 72e0 	eor.w	r2, r0, r0, asr #31
 8002820:	ebb1 71e0 	subs.w	r1, r1, r0, asr #31
 8002824:	eb62 70e0 	sbc.w	r0, r2, r0, asr #31
 8002828:	1875      	adds	r5, r6, r1
 800282a:	eb44 0600 	adc.w	r6, r4, r0
 800282e:	e0c4      	b.n	80029ba <smoltcp::iface::interface::Interface::poll+0x189e>
 8002830:	2200      	movs	r2, #0
 8002832:	3001      	adds	r0, #1
 8002834:	f8dd a110 	ldr.w	sl, [sp, #272]	@ 0x110
 8002838:	fbb0 f3f1 	udiv	r3, r0, r1
 800283c:	f1bb 0b01 	subs.w	fp, fp, #1
 8002840:	fb03 0011 	mls	r0, r3, r1, r0
 8002844:	eba8 0106 	sub.w	r1, r8, r6
 8002848:	4665      	mov	r5, ip
 800284a:	e9c9 2112 	strd	r2, r1, [r9, #72]	@ 0x48
 800284e:	e9c9 0b0e 	strd	r0, fp, [r9, #56]	@ 0x38
 8002852:	d035      	beq.n	80028c0 <smoltcp::iface::interface::Interface::poll+0x17a4>
 8002854:	f8d9 3034 	ldr.w	r3, [r9, #52]	@ 0x34
 8002858:	2b00      	cmp	r3, #0
 800285a:	f000 853c 	beq.w	80032d6 <smoltcp::iface::interface::Interface::poll+0x21ba>
 800285e:	f8d9 0038 	ldr.w	r0, [r9, #56]	@ 0x38
 8002862:	9546      	str	r5, [sp, #280]	@ 0x118
 8002864:	4298      	cmp	r0, r3
 8002866:	f080 8568 	bcs.w	800333a <smoltcp::iface::interface::Interface::poll+0x221e>
 800286a:	e9d9 5811 	ldrd	r5, r8, [r9, #68]	@ 0x44
 800286e:	f8d9 404c 	ldr.w	r4, [r9, #76]	@ 0x4c
 8002872:	eba5 0608 	sub.w	r6, r5, r8
 8002876:	42a6      	cmp	r6, r4
 8002878:	bf28      	it	cs
 800287a:	4626      	movcs	r6, r4
 800287c:	eb16 0108 	adds.w	r1, r6, r8
 8002880:	f080 84c8 	bcs.w	8003214 <smoltcp::iface::interface::Interface::poll+0x20f8>
 8002884:	42a9      	cmp	r1, r5
 8002886:	f200 84c5 	bhi.w	8003214 <smoltcp::iface::interface::Interface::poll+0x20f8>
 800288a:	f8d9 2030 	ldr.w	r2, [r9, #48]	@ 0x30
 800288e:	eb00 0180 	add.w	r1, r0, r0, lsl #2
 8002892:	f8cd b10c 	str.w	fp, [sp, #268]	@ 0x10c
 8002896:	eb02 0b81 	add.w	fp, r2, r1, lsl #2
 800289a:	f89b 200e 	ldrb.w	r2, [fp, #14]
 800289e:	2a02      	cmp	r2, #2
 80028a0:	f000 84ec 	beq.w	800327c <smoltcp::iface::interface::Interface::poll+0x2160>
 80028a4:	f8db 1000 	ldr.w	r1, [fp]
 80028a8:	42b1      	cmp	r1, r6
 80028aa:	f200 84bb 	bhi.w	8003224 <smoltcp::iface::interface::Interface::poll+0x2108>
 80028ae:	f8d9 c040 	ldr.w	ip, [r9, #64]	@ 0x40
 80028b2:	07d2      	lsls	r2, r2, #31
 80028b4:	aa3e      	add	r2, sp, #248	@ 0xf8
 80028b6:	c219      	stmia	r2!, {r0, r3, r4}
 80028b8:	d007      	beq.n	80028ca <smoltcp::iface::interface::Interface::poll+0x17ae>
 80028ba:	f8db 300f 	ldr.w	r3, [fp, #15]
 80028be:	e015      	b.n	80028ec <smoltcp::iface::interface::Interface::poll+0x17d0>
 80028c0:	2203      	movs	r2, #3
 80028c2:	f04f 0c00 	mov.w	ip, #0
 80028c6:	f000 bc1f 	b.w	8003108 <smoltcp::iface::interface::Interface::poll+0x1fec>
 80028ca:	ea5f 32ca 	movs.w	r2, sl, lsl #15
 80028ce:	d408      	bmi.n	80028e2 <smoltcp::iface::interface::Interface::poll+0x17c6>
 80028d0:	9a48      	ldr	r2, [sp, #288]	@ 0x120
 80028d2:	f8d2 20f0 	ldr.w	r2, [r2, #240]	@ 0xf0
 80028d6:	2a00      	cmp	r2, #0
 80028d8:	f000 83fa 	beq.w	80030d0 <smoltcp::iface::interface::Interface::poll+0x1fb4>
 80028dc:	9a42      	ldr	r2, [sp, #264]	@ 0x108
 80028de:	6813      	ldr	r3, [r2, #0]
 80028e0:	e004      	b.n	80028ec <smoltcp::iface::interface::Interface::poll+0x17d0>
 80028e2:	983d      	ldr	r0, [sp, #244]	@ 0xf4
 80028e4:	ea4f 621a 	mov.w	r2, sl, lsr #24
 80028e8:	ea42 2300 	orr.w	r3, r2, r0, lsl #8
 80028ec:	eb0c 0008 	add.w	r0, ip, r8
 80028f0:	e9db 2a01 	ldrd	r2, sl, [fp, #4]
 80028f4:	f8bb c00c 	ldrh.w	ip, [fp, #12]
 80028f8:	2404      	movs	r4, #4
 80028fa:	e9cd 01bf 	strd	r0, r1, [sp, #764]	@ 0x2fc
 80028fe:	2003      	movs	r0, #3
 8002900:	e9cd 20bd 	strd	r2, r0, [sp, #756]	@ 0x2f4
 8002904:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8002906:	90bc      	str	r0, [sp, #752]	@ 0x2f0
 8002908:	68b8      	ldr	r0, [r7, #8]
 800290a:	f88d 4359 	strb.w	r4, [sp, #857]	@ 0x359
 800290e:	f101 0408 	add.w	r4, r1, #8
 8002912:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 8002916:	e9cd 3ad3 	strd	r3, sl, [sp, #844]	@ 0x34c
 800291a:	9b44      	ldr	r3, [sp, #272]	@ 0x110
 800291c:	4288      	cmp	r0, r1
 800291e:	f88d e358 	strb.w	lr, [sp, #856]	@ 0x358
 8002922:	94d5      	str	r4, [sp, #852]	@ 0x354
 8002924:	f8cd a114 	str.w	sl, [sp, #276]	@ 0x114
 8002928:	f8ad c306 	strh.w	ip, [sp, #774]	@ 0x306
 800292c:	f8ad 3304 	strh.w	r3, [sp, #772]	@ 0x304
 8002930:	f080 84bb 	bcs.w	80032aa <smoltcp::iface::interface::Interface::poll+0x218e>
 8002934:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 8002938:	9b41      	ldr	r3, [sp, #260]	@ 0x104
 800293a:	4348      	muls	r0, r1
 800293c:	6819      	ldr	r1, [r3, #0]
 800293e:	5808      	ldr	r0, [r1, r0]
 8002940:	2800      	cmp	r0, #0
 8002942:	d415      	bmi.n	8002970 <smoltcp::iface::interface::Interface::poll+0x1854>
 8002944:	2000      	movs	r0, #0
 8002946:	93a5      	str	r3, [sp, #660]	@ 0x294
 8002948:	90a3      	str	r0, [sp, #652]	@ 0x28c
 800294a:	a9a3      	add	r1, sp, #652	@ 0x28c
 800294c:	9848      	ldr	r0, [sp, #288]	@ 0x120
 800294e:	9b36      	ldr	r3, [sp, #216]	@ 0xd8
 8002950:	f7fd fd04 	bl	800035c <smoltcp::iface::interface::InterfaceInner::dispatch_ip>
 8002954:	b2c0      	uxtb	r0, r0
 8002956:	2802      	cmp	r0, #2
 8002958:	d112      	bne.n	8002980 <smoltcp::iface::interface::Interface::poll+0x1864>
 800295a:	f8db 1000 	ldr.w	r1, [fp]
 800295e:	f04f 0c01 	mov.w	ip, #1
 8002962:	9c40      	ldr	r4, [sp, #256]	@ 0x100
 8002964:	9843      	ldr	r0, [sp, #268]	@ 0x10c
 8002966:	42b1      	cmp	r1, r6
 8002968:	f240 83b8 	bls.w	80030dc <smoltcp::iface::interface::Interface::poll+0x1fc0>
 800296c:	f000 bc75 	b.w	800325a <smoltcp::iface::interface::Interface::poll+0x213e>
 8002970:	f240 0025 	movw	r0, #37	@ 0x25
 8002974:	f2c0 0000 	movt	r0, #0
 8002978:	f001 fcef 	bl	800435a <defmt::export::acquire_header_and_release>
 800297c:	2200      	movs	r2, #0
 800297e:	e000      	b.n	8002982 <smoltcp::iface::interface::Interface::poll+0x1866>
 8002980:	2201      	movs	r2, #1
 8002982:	9c40      	ldr	r4, [sp, #256]	@ 0x100
 8002984:	2100      	movs	r1, #0
 8002986:	9843      	ldr	r0, [sp, #268]	@ 0x10c
 8002988:	f04f 0c01 	mov.w	ip, #1
 800298c:	2d00      	cmp	r5, #0
 800298e:	f000 83a9 	beq.w	80030e4 <smoltcp::iface::interface::Interface::poll+0x1fc8>
 8002992:	eb01 0308 	add.w	r3, r1, r8
 8002996:	fbb3 f6f5 	udiv	r6, r3, r5
 800299a:	fb06 3315 	mls	r3, r6, r5, r3
 800299e:	e3a2      	b.n	80030e6 <smoltcp::iface::interface::Interface::poll+0x1fca>
 80029a0:	f103 0050 	add.w	r0, r3, #80	@ 0x50
 80029a4:	e9d0 0102 	ldrd	r0, r1, [r0, #8]
 80029a8:	ebba 0000 	subs.w	r0, sl, r0
 80029ac:	eb7b 0001 	sbcs.w	r0, fp, r1
 80029b0:	f6ff ae86 	blt.w	80026c0 <smoltcp::iface::interface::Interface::poll+0x15a4>
 80029b4:	e692      	b.n	80026dc <smoltcp::iface::interface::Interface::poll+0x15c0>
 80029b6:	2500      	movs	r5, #0
 80029b8:	2600      	movs	r6, #0
 80029ba:	f240 0015 	movw	r0, #21
 80029be:	f2c0 0000 	movt	r0, #0
 80029c2:	f001 fcb9 	bl	8004338 <defmt::export::acquire_and_header>
 80029c6:	f240 0007 	movw	r0, #7
 80029ca:	acbc      	add	r4, sp, #752	@ 0x2f0
 80029cc:	f2c0 0000 	movt	r0, #0
 80029d0:	2102      	movs	r1, #2
 80029d2:	f8ad 02f0 	strh.w	r0, [sp, #752]	@ 0x2f0
 80029d6:	4620      	mov	r0, r4
 80029d8:	f001 fe60 	bl	800469c <_defmt_write>
 80029dc:	f240 0040 	movw	r0, #64	@ 0x40
 80029e0:	2102      	movs	r1, #2
 80029e2:	f2c0 0000 	movt	r0, #0
 80029e6:	f8ad 02f0 	strh.w	r0, [sp, #752]	@ 0x2f0
 80029ea:	4620      	mov	r0, r4
 80029ec:	f001 fe56 	bl	800469c <_defmt_write>
 80029f0:	f240 0805 	movw	r8, #5
 80029f4:	4620      	mov	r0, r4
 80029f6:	f2c0 0800 	movt	r8, #0
 80029fa:	2102      	movs	r1, #2
 80029fc:	f8ad 82f0 	strh.w	r8, [sp, #752]	@ 0x2f0
 8002a00:	f001 fe4c 	bl	800469c <_defmt_write>
 8002a04:	f244 2240 	movw	r2, #16960	@ 0x4240
 8002a08:	4628      	mov	r0, r5
 8002a0a:	4631      	mov	r1, r6
 8002a0c:	f2c0 020f 	movt	r2, #15
 8002a10:	2300      	movs	r3, #0
 8002a12:	f006 fd2b 	bl	800946c <__aeabi_uldivmod>
 8002a16:	e9cd 01bc 	strd	r0, r1, [sp, #752]	@ 0x2f0
 8002a1a:	4620      	mov	r0, r4
 8002a1c:	2108      	movs	r1, #8
 8002a1e:	f001 fe3d 	bl	800469c <_defmt_write>
 8002a22:	4620      	mov	r0, r4
 8002a24:	2102      	movs	r1, #2
 8002a26:	f8ad 82f0 	strh.w	r8, [sp, #752]	@ 0x2f0
 8002a2a:	f001 fe37 	bl	800469c <_defmt_write>
 8002a2e:	4628      	mov	r0, r5
 8002a30:	4631      	mov	r1, r6
 8002a32:	f44f 727a 	mov.w	r2, #1000	@ 0x3e8
 8002a36:	2300      	movs	r3, #0
 8002a38:	f006 fd18 	bl	800946c <__aeabi_uldivmod>
 8002a3c:	f44f 727a 	mov.w	r2, #1000	@ 0x3e8
 8002a40:	2300      	movs	r3, #0
 8002a42:	f006 fd13 	bl	800946c <__aeabi_uldivmod>
 8002a46:	4620      	mov	r0, r4
 8002a48:	2108      	movs	r1, #8
 8002a4a:	e9cd 23bc 	strd	r2, r3, [sp, #752]	@ 0x2f0
 8002a4e:	f001 fe25 	bl	800469c <_defmt_write>
 8002a52:	2500      	movs	r5, #0
 8002a54:	4620      	mov	r0, r4
 8002a56:	2102      	movs	r1, #2
 8002a58:	f8ad 52f0 	strh.w	r5, [sp, #752]	@ 0x2f0
 8002a5c:	f001 fe1e 	bl	800469c <_defmt_write>
 8002a60:	f001 fda4 	bl	80045ac <_defmt_release>
 8002a64:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8002a66:	e9d9 6312 	ldrd	r6, r3, [r9, #72]	@ 0x48
 8002a6a:	e9d0 0104 	ldrd	r0, r1, [r0, #16]
 8002a6e:	f8d9 4100 	ldr.w	r4, [r9, #256]	@ 0x100
 8002a72:	1980      	adds	r0, r0, r6
 8002a74:	f999 2028 	ldrsb.w	r2, [r9, #40]	@ 0x28
 8002a78:	f8c9 4108 	str.w	r4, [r9, #264]	@ 0x108
 8002a7c:	f04f 0401 	mov.w	r4, #1
 8002a80:	4159      	adcs	r1, r3
 8002a82:	e9d9 ce10 	ldrd	ip, lr, [r9, #64]	@ 0x40
 8002a86:	fa82 f254 	uqadd8	r2, r2, r4
 8002a8a:	e9c9 5500 	strd	r5, r5, [r9]
 8002a8e:	e9c9 0124 	strd	r0, r1, [r9, #144]	@ 0x90
 8002a92:	b2d0      	uxtb	r0, r2
 8002a94:	2803      	cmp	r0, #3
 8002a96:	f889 2028 	strb.w	r2, [r9, #40]	@ 0x28
 8002a9a:	e9c9 5520 	strd	r5, r5, [r9, #128]	@ 0x80
 8002a9e:	e9c9 ce22 	strd	ip, lr, [r9, #136]	@ 0x88
 8002aa2:	d30e      	bcc.n	8002ac2 <smoltcp::iface::interface::Interface::poll+0x19a6>
 8002aa4:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8002aa8:	2100      	movs	r1, #0
 8002aaa:	f889 1028 	strb.w	r1, [r9, #40]	@ 0x28
 8002aae:	f242 7210 	movw	r2, #10000	@ 0x2710
 8002ab2:	0041      	lsls	r1, r0, #1
 8002ab4:	4291      	cmp	r1, r2
 8002ab6:	f242 7110 	movw	r1, #10000	@ 0x2710
 8002aba:	bf38      	it	cc
 8002abc:	0041      	lslcc	r1, r0, #1
 8002abe:	f8c9 1020 	str.w	r1, [r9, #32]
 8002ac2:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8002ac4:	68c1      	ldr	r1, [r0, #12]
 8002ac6:	4648      	mov	r0, r9
 8002ac8:	f003 fc80 	bl	80063cc <smoltcp::socket::tcp::Socket::seq_to_transmit>
 8002acc:	2800      	cmp	r0, #0
 8002ace:	d03e      	beq.n	8002b4e <smoltcp::iface::interface::Interface::poll+0x1a32>
 8002ad0:	f8b9 011c 	ldrh.w	r0, [r9, #284]	@ 0x11c
 8002ad4:	07c0      	lsls	r0, r0, #31
 8002ad6:	f000 83cb 	beq.w	8003270 <smoltcp::iface::interface::Interface::poll+0x2154>
 8002ada:	f899 112a 	ldrb.w	r1, [r9, #298]	@ 0x12a
 8002ade:	f44f 7a50 	mov.w	sl, #832	@ 0x340
 8002ae2:	f8d9 40c4 	ldr.w	r4, [r9, #196]	@ 0xc4
 8002ae6:	2900      	cmp	r1, #0
 8002ae8:	f899 012b 	ldrb.w	r0, [r9, #299]	@ 0x12b
 8002aec:	bf18      	it	ne
 8002aee:	f500 7a40 	addne.w	sl, r0, #768	@ 0x300
 8002af2:	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
 8002af6:	f340 8358 	ble.w	80031aa <smoltcp::iface::interface::Interface::poll+0x208e>
 8002afa:	f8d9 011e 	ldr.w	r0, [r9, #286]	@ 0x11e
 8002afe:	903b      	str	r0, [sp, #236]	@ 0xec
 8002b00:	f8d9 0124 	ldr.w	r0, [r9, #292]	@ 0x124
 8002b04:	903f      	str	r0, [sp, #252]	@ 0xfc
 8002b06:	f8b9 0122 	ldrh.w	r0, [r9, #290]	@ 0x122
 8002b0a:	903a      	str	r0, [sp, #232]	@ 0xe8
 8002b0c:	f8b9 0128 	ldrh.w	r0, [r9, #296]	@ 0x128
 8002b10:	f899 1134 	ldrb.w	r1, [r9, #308]	@ 0x134
 8002b14:	9039      	str	r0, [sp, #228]	@ 0xe4
 8002b16:	f8d9 00bc 	ldr.w	r0, [r9, #188]	@ 0xbc
 8002b1a:	f001 011f 	and.w	r1, r1, #31
 8002b1e:	e9d9 5641 	ldrd	r5, r6, [r9, #260]	@ 0x104
 8002b22:	1b00      	subs	r0, r0, r4
 8002b24:	f8d9 2114 	ldr.w	r2, [r9, #276]	@ 0x114
 8002b28:	fa20 f101 	lsr.w	r1, r0, r1
 8002b2c:	f64f 70ff 	movw	r0, #65535	@ 0xffff
 8002b30:	4281      	cmp	r1, r0
 8002b32:	bf28      	it	cs
 8002b34:	4601      	movcs	r1, r0
 8002b36:	2a00      	cmp	r2, #0
 8002b38:	9140      	str	r1, [sp, #256]	@ 0x100
 8002b3a:	923c      	str	r2, [sp, #240]	@ 0xf0
 8002b3c:	f000 80c1 	beq.w	8002cc2 <smoltcp::iface::interface::Interface::poll+0x1ba6>
 8002b40:	f8d9 0118 	ldr.w	r0, [r9, #280]	@ 0x118
 8002b44:	9035      	str	r0, [sp, #212]	@ 0xd4
 8002b46:	4790      	blx	r2
 8002b48:	9034      	str	r0, [sp, #208]	@ 0xd0
 8002b4a:	2001      	movs	r0, #1
 8002b4c:	e0ba      	b.n	8002cc4 <smoltcp::iface::interface::Interface::poll+0x1ba8>
 8002b4e:	f8d9 1098 	ldr.w	r1, [r9, #152]	@ 0x98
 8002b52:	2901      	cmp	r1, #1
 8002b54:	d10f      	bne.n	8002b76 <smoltcp::iface::interface::Interface::poll+0x1a5a>
 8002b56:	f8d9 00c4 	ldr.w	r0, [r9, #196]	@ 0xc4
 8002b5a:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8002b5e:	f340 8324 	ble.w	80031aa <smoltcp::iface::interface::Interface::poll+0x208e>
 8002b62:	f8d9 3104 	ldr.w	r3, [r9, #260]	@ 0x104
 8002b66:	f8d9 209c 	ldr.w	r2, [r9, #156]	@ 0x9c
 8002b6a:	4418      	add	r0, r3
 8002b6c:	1a10      	subs	r0, r2, r0
 8002b6e:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8002b72:	f340 8145 	ble.w	8002e00 <smoltcp::iface::interface::Interface::poll+0x1ce4>
 8002b76:	f899 0133 	ldrb.w	r0, [r9, #307]	@ 0x133
 8002b7a:	1e82      	subs	r2, r0, #2
 8002b7c:	2a04      	cmp	r2, #4
 8002b7e:	d834      	bhi.n	8002bea <smoltcp::iface::interface::Interface::poll+0x1ace>
 8002b80:	f899 2134 	ldrb.w	r2, [r9, #308]	@ 0x134
 8002b84:	f8d9 30bc 	ldr.w	r3, [r9, #188]	@ 0xbc
 8002b88:	f8d9 50c4 	ldr.w	r5, [r9, #196]	@ 0xc4
 8002b8c:	f002 021f 	and.w	r2, r2, #31
 8002b90:	1b5b      	subs	r3, r3, r5
 8002b92:	fa23 f602 	lsr.w	r6, r3, r2
 8002b96:	f64f 73ff 	movw	r3, #65535	@ 0xffff
 8002b9a:	429e      	cmp	r6, r3
 8002b9c:	bf38      	it	cc
 8002b9e:	4633      	movcc	r3, r6
 8002ba0:	b319      	cbz	r1, 8002bea <smoltcp::iface::interface::Interface::poll+0x1ace>
 8002ba2:	f1b5 3fff 	cmp.w	r5, #4294967295	@ 0xffffffff
 8002ba6:	bfc2      	ittt	gt
 8002ba8:	f8b9 112e 	ldrhgt.w	r1, [r9, #302]	@ 0x12e
 8002bac:	4091      	lslgt	r1, r2
 8002bae:	f1b1 3fff 	cmpgt.w	r1, #4294967295	@ 0xffffffff
 8002bb2:	f340 82fa 	ble.w	80031aa <smoltcp::iface::interface::Interface::poll+0x208e>
 8002bb6:	f8d9 409c 	ldr.w	r4, [r9, #156]	@ 0x9c
 8002bba:	f8d9 c104 	ldr.w	ip, [r9, #260]	@ 0x104
 8002bbe:	4421      	add	r1, r4
 8002bc0:	4465      	add	r5, ip
 8002bc2:	1b49      	subs	r1, r1, r5
 8002bc4:	f1b1 3fff 	cmp.w	r1, #4294967295	@ 0xffffffff
 8002bc8:	f340 835e 	ble.w	8003288 <smoltcp::iface::interface::Interface::poll+0x216c>
 8002bcc:	b16e      	cbz	r6, 8002bea <smoltcp::iface::interface::Interface::poll+0x1ace>
 8002bce:	40d1      	lsrs	r1, r2
 8002bd0:	f64f 72ff 	movw	r2, #65535	@ 0xffff
 8002bd4:	4291      	cmp	r1, r2
 8002bd6:	bf28      	it	cs
 8002bd8:	4611      	movcs	r1, r2
 8002bda:	ebb1 0f53 	cmp.w	r1, r3, lsr #1
 8002bde:	f04f 0100 	mov.w	r1, #0
 8002be2:	bf98      	it	ls
 8002be4:	2101      	movls	r1, #1
 8002be6:	b121      	cbz	r1, 8002bf2 <smoltcp::iface::interface::Interface::poll+0x1ad6>
 8002be8:	e772      	b.n	8002ad0 <smoltcp::iface::interface::Interface::poll+0x19b4>
 8002bea:	2100      	movs	r1, #0
 8002bec:	2900      	cmp	r1, #0
 8002bee:	f47f af6f 	bne.w	8002ad0 <smoltcp::iface::interface::Interface::poll+0x19b4>
 8002bf2:	2800      	cmp	r0, #0
 8002bf4:	f43f af6c 	beq.w	8002ad0 <smoltcp::iface::interface::Interface::poll+0x19b4>
 8002bf8:	4648      	mov	r0, r9
 8002bfa:	9948      	ldr	r1, [sp, #288]	@ 0x120
 8002bfc:	f850 5f80 	ldr.w	r5, [r0, #128]!
 8002c00:	f100 0c04 	add.w	ip, r0, #4
 8002c04:	e9d1 2e04 	ldrd	r2, lr, [r1, #16]
 8002c08:	e89c 1050 	ldmia.w	ip, {r4, r6, ip}
 8002c0c:	ea55 0304 	orrs.w	r3, r5, r4
 8002c10:	d108      	bne.n	8002c24 <smoltcp::iface::interface::Interface::poll+0x1b08>
 8002c12:	07f3      	lsls	r3, r6, #31
 8002c14:	d006      	beq.n	8002c24 <smoltcp::iface::interface::Interface::poll+0x1b08>
 8002c16:	e9d0 3104 	ldrd	r3, r1, [r0, #16]
 8002c1a:	1ad3      	subs	r3, r2, r3
 8002c1c:	eb7e 0101 	sbcs.w	r1, lr, r1
 8002c20:	f6bf af56 	bge.w	8002ad0 <smoltcp::iface::interface::Interface::poll+0x19b4>
 8002c24:	f085 0103 	eor.w	r1, r5, #3
 8002c28:	9d46      	ldr	r5, [sp, #280]	@ 0x118
 8002c2a:	4321      	orrs	r1, r4
 8002c2c:	f47f acc4 	bne.w	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 8002c30:	1b91      	subs	r1, r2, r6
 8002c32:	eb7e 010c 	sbcs.w	r1, lr, ip
 8002c36:	f6ff acbf 	blt.w	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 8002c3a:	462e      	mov	r6, r5
 8002c3c:	2500      	movs	r5, #0
 8002c3e:	e9c0 5500 	strd	r5, r5, [r0]
 8002c42:	f44f 7196 	mov.w	r1, #300	@ 0x12c
 8002c46:	e9c0 5502 	strd	r5, r5, [r0, #8]
 8002c4a:	2064      	movs	r0, #100	@ 0x64
 8002c4c:	e9c9 1008 	strd	r1, r0, [r9, #32]
 8002c50:	f109 00d0 	add.w	r0, r9, #208	@ 0xd0
 8002c54:	2128      	movs	r1, #40	@ 0x28
 8002c56:	e9c9 5500 	strd	r5, r5, [r9]
 8002c5a:	f889 5133 	strb.w	r5, [r9, #307]	@ 0x133
 8002c5e:	f8c9 5018 	str.w	r5, [r9, #24]
 8002c62:	f889 5130 	strb.w	r5, [r9, #304]	@ 0x130
 8002c66:	e9c9 5530 	strd	r5, r5, [r9, #192]	@ 0xc0
 8002c6a:	f889 5028 	strb.w	r5, [r9, #40]	@ 0x28
 8002c6e:	f8d9 40bc 	ldr.w	r4, [r9, #188]	@ 0xbc
 8002c72:	f8a9 511c 	strh.w	r5, [r9, #284]	@ 0x11c
 8002c76:	f889 50b2 	strb.w	r5, [r9, #178]	@ 0xb2
 8002c7a:	f8a9 50b0 	strh.w	r5, [r9, #176]	@ 0xb0
 8002c7e:	f8a9 512e 	strh.w	r5, [r9, #302]	@ 0x12e
 8002c82:	f8c9 5098 	str.w	r5, [r9, #152]	@ 0x98
 8002c86:	f889 512c 	strb.w	r5, [r9, #300]	@ 0x12c
 8002c8a:	e9c9 5540 	strd	r5, r5, [r9, #256]	@ 0x100
 8002c8e:	e9c9 5542 	strd	r5, r5, [r9, #264]	@ 0x108
 8002c92:	f005 fdad 	bl	80087f0 <__aeabi_memclr8>
 8002c96:	f44f 7006 	mov.w	r0, #536	@ 0x218
 8002c9a:	e9c9 553e 	strd	r5, r5, [r9, #248]	@ 0xf8
 8002c9e:	f8c9 0110 	str.w	r0, [r9, #272]	@ 0x110
 8002ca2:	fab4 f084 	clz	r0, r4
 8002ca6:	f1c0 0110 	rsb	r1, r0, #16
 8002caa:	2210      	movs	r2, #16
 8002cac:	e9c9 551c 	strd	r5, r5, [r9, #112]	@ 0x70
 8002cb0:	4282      	cmp	r2, r0
 8002cb2:	e9c9 5514 	strd	r5, r5, [r9, #80]	@ 0x50
 8002cb6:	bf38      	it	cc
 8002cb8:	4629      	movcc	r1, r5
 8002cba:	4635      	mov	r5, r6
 8002cbc:	f889 1134 	strb.w	r1, [r9, #308]	@ 0x134
 8002cc0:	e47a      	b.n	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 8002cc2:	2000      	movs	r0, #0
 8002cc4:	9038      	str	r0, [sp, #224]	@ 0xe0
 8002cc6:	1960      	adds	r0, r4, r5
 8002cc8:	f899 4133 	ldrb.w	r4, [r9, #307]	@ 0x133
 8002ccc:	f04f 0800 	mov.w	r8, #0
 8002cd0:	903e      	str	r0, [sp, #248]	@ 0xf8
 8002cd2:	2001      	movs	r0, #1
 8002cd4:	963d      	str	r6, [sp, #244]	@ 0xf4
 8002cd6:	2101      	movs	r1, #1
 8002cd8:	9043      	str	r0, [sp, #268]	@ 0x10c
 8002cda:	2000      	movs	r0, #0
 8002cdc:	2300      	movs	r3, #0
 8002cde:	2600      	movs	r6, #0
 8002ce0:	9044      	str	r0, [sp, #272]	@ 0x110
 8002ce2:	f04f 0b01 	mov.w	fp, #1
 8002ce6:	f04f 0e00 	mov.w	lr, #0
 8002cea:	9d46      	ldr	r5, [sp, #280]	@ 0x118
 8002cec:	e8df f004 	tbb	[pc, r4]
 8002cf0:	4b4b069a 	.word	0x4b4b069a
 8002cf4:	07b10707 	.word	0x07b10707
 8002cf8:	00b10707 	.word	0x00b10707
 8002cfc:	e45c      	b.n	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 8002cfe:	f8d9 010c 	ldr.w	r0, [r9, #268]	@ 0x10c
 8002d02:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8002d06:	f340 8250 	ble.w	80031aa <smoltcp::iface::interface::Interface::poll+0x208e>
 8002d0a:	f8d9 2100 	ldr.w	r2, [r9, #256]	@ 0x100
 8002d0e:	9948      	ldr	r1, [sp, #288]	@ 0x120
 8002d10:	f8d9 3108 	ldr.w	r3, [r9, #264]	@ 0x108
 8002d14:	4410      	add	r0, r2
 8002d16:	f8d9 5110 	ldr.w	r5, [r9, #272]	@ 0x110
 8002d1a:	1ac0      	subs	r0, r0, r3
 8002d1c:	68ce      	ldr	r6, [r1, #12]
 8002d1e:	ea20 71e0 	bic.w	r1, r0, r0, asr #31
 8002d22:	f1a6 0036 	sub.w	r0, r6, #54	@ 0x36
 8002d26:	428d      	cmp	r5, r1
 8002d28:	bf38      	it	cc
 8002d2a:	4629      	movcc	r1, r5
 8002d2c:	4288      	cmp	r0, r1
 8002d2e:	bf38      	it	cc
 8002d30:	4601      	movcc	r1, r0
 8002d32:	1a9b      	subs	r3, r3, r2
 8002d34:	f100 82a8 	bmi.w	8003288 <smoltcp::iface::interface::Interface::poll+0x216c>
 8002d38:	f8d9 20cc 	ldr.w	r2, [r9, #204]	@ 0xcc
 8002d3c:	2a00      	cmp	r2, #0
 8002d3e:	d03c      	beq.n	8002dba <smoltcp::iface::interface::Interface::poll+0x1c9e>
 8002d40:	f8d9 00d0 	ldr.w	r0, [r9, #208]	@ 0xd0
 8002d44:	4418      	add	r0, r3
 8002d46:	fbb0 f6f2 	udiv	r6, r0, r2
 8002d4a:	fb06 0012 	mls	r0, r6, r2, r0
 8002d4e:	f8d9 50d4 	ldr.w	r5, [r9, #212]	@ 0xd4
 8002d52:	429d      	cmp	r5, r3
 8002d54:	d236      	bcs.n	8002dc4 <smoltcp::iface::interface::Interface::poll+0x1ca8>
 8002d56:	2101      	movs	r1, #1
 8002d58:	eb08 0003 	add.w	r0, r8, r3
 8002d5c:	42a8      	cmp	r0, r5
 8002d5e:	d148      	bne.n	8002df2 <smoltcp::iface::interface::Interface::poll+0x1cd6>
 8002d60:	3c04      	subs	r4, #4
 8002d62:	2001      	movs	r0, #1
 8002d64:	2300      	movs	r3, #0
 8002d66:	2c05      	cmp	r4, #5
 8002d68:	9043      	str	r0, [sp, #268]	@ 0x10c
 8002d6a:	d845      	bhi.n	8002df8 <smoltcp::iface::interface::Interface::poll+0x1cdc>
 8002d6c:	2200      	movs	r2, #0
 8002d6e:	2600      	movs	r6, #0
 8002d70:	9244      	str	r2, [sp, #272]	@ 0x110
 8002d72:	e8df f014 	tbh	[pc, r4, lsl #1]
 8002d76:	01ec      	.short	0x01ec
 8002d78:	006e0006 	.word	0x006e0006
 8002d7c:	000601ec 	.word	0x000601ec
 8002d80:	0006      	.short	0x0006
 8002d82:	2003      	movs	r0, #3
 8002d84:	e04f      	b.n	8002e26 <smoltcp::iface::interface::Interface::poll+0x1d0a>
 8002d86:	f8d9 00bc 	ldr.w	r0, [r9, #188]	@ 0xbc
 8002d8a:	f8d9 10c4 	ldr.w	r1, [r9, #196]	@ 0xc4
 8002d8e:	f8d9 2100 	ldr.w	r2, [r9, #256]	@ 0x100
 8002d92:	1a41      	subs	r1, r0, r1
 8002d94:	f64f 70ff 	movw	r0, #65535	@ 0xffff
 8002d98:	4281      	cmp	r1, r0
 8002d9a:	923d      	str	r2, [sp, #244]	@ 0xf4
 8002d9c:	bf28      	it	cs
 8002d9e:	4601      	movcs	r1, r0
 8002da0:	2c02      	cmp	r4, #2
 8002da2:	9140      	str	r1, [sp, #256]	@ 0x100
 8002da4:	d145      	bne.n	8002e32 <smoltcp::iface::interface::Interface::poll+0x1d16>
 8002da6:	f899 0134 	ldrb.w	r0, [r9, #308]	@ 0x134
 8002daa:	2101      	movs	r1, #1
 8002dac:	9030      	str	r0, [sp, #192]	@ 0xc0
 8002dae:	2000      	movs	r0, #0
 8002db0:	9043      	str	r0, [sp, #268]	@ 0x10c
 8002db2:	2002      	movs	r0, #2
 8002db4:	2301      	movs	r3, #1
 8002db6:	2601      	movs	r6, #1
 8002db8:	e046      	b.n	8002e48 <smoltcp::iface::interface::Interface::poll+0x1d2c>
 8002dba:	2000      	movs	r0, #0
 8002dbc:	f8d9 50d4 	ldr.w	r5, [r9, #212]	@ 0xd4
 8002dc0:	429d      	cmp	r5, r3
 8002dc2:	d3c8      	bcc.n	8002d56 <smoltcp::iface::interface::Interface::poll+0x1c3a>
 8002dc4:	eba5 0803 	sub.w	r8, r5, r3
 8002dc8:	4541      	cmp	r1, r8
 8002dca:	bf38      	it	cc
 8002dcc:	4688      	movcc	r8, r1
 8002dce:	1a11      	subs	r1, r2, r0
 8002dd0:	4588      	cmp	r8, r1
 8002dd2:	bf28      	it	cs
 8002dd4:	4688      	movcs	r8, r1
 8002dd6:	eb18 0100 	adds.w	r1, r8, r0
 8002dda:	f080 8260 	bcs.w	800329e <smoltcp::iface::interface::Interface::poll+0x2182>
 8002dde:	4291      	cmp	r1, r2
 8002de0:	f200 825d 	bhi.w	800329e <smoltcp::iface::interface::Interface::poll+0x2182>
 8002de4:	f8d9 10c8 	ldr.w	r1, [r9, #200]	@ 0xc8
 8002de8:	4401      	add	r1, r0
 8002dea:	eb08 0003 	add.w	r0, r8, r3
 8002dee:	42a8      	cmp	r0, r5
 8002df0:	d0b6      	beq.n	8002d60 <smoltcp::iface::interface::Interface::poll+0x1c44>
 8002df2:	2001      	movs	r0, #1
 8002df4:	2300      	movs	r3, #0
 8002df6:	9043      	str	r0, [sp, #268]	@ 0x10c
 8002df8:	2600      	movs	r6, #0
 8002dfa:	2000      	movs	r0, #0
 8002dfc:	9044      	str	r0, [sp, #272]	@ 0x110
 8002dfe:	e028      	b.n	8002e52 <smoltcp::iface::interface::Interface::poll+0x1d36>
 8002e00:	e9d9 021c 	ldrd	r0, r2, [r9, #112]	@ 0x70
 8002e04:	f080 0001 	eor.w	r0, r0, #1
 8002e08:	4310      	orrs	r0, r2
 8002e0a:	f47f ae61 	bne.w	8002ad0 <smoltcp::iface::interface::Interface::poll+0x19b4>
 8002e0e:	9b48      	ldr	r3, [sp, #288]	@ 0x120
 8002e10:	e9d9 021e 	ldrd	r0, r2, [r9, #120]	@ 0x78
 8002e14:	e9d3 3604 	ldrd	r3, r6, [r3, #16]
 8002e18:	1a18      	subs	r0, r3, r0
 8002e1a:	eb76 0002 	sbcs.w	r0, r6, r2
 8002e1e:	f6bf ae57 	bge.w	8002ad0 <smoltcp::iface::interface::Interface::poll+0x19b4>
 8002e22:	e6a8      	b.n	8002b76 <smoltcp::iface::interface::Interface::poll+0x1a5a>
 8002e24:	2004      	movs	r0, #4
 8002e26:	9044      	str	r0, [sp, #272]	@ 0x110
 8002e28:	2001      	movs	r0, #1
 8002e2a:	9043      	str	r0, [sp, #268]	@ 0x10c
 8002e2c:	f04f 0b00 	mov.w	fp, #0
 8002e30:	e00f      	b.n	8002e52 <smoltcp::iface::interface::Interface::poll+0x1d36>
 8002e32:	2001      	movs	r0, #1
 8002e34:	f899 312c 	ldrb.w	r3, [r9, #300]	@ 0x12c
 8002e38:	9043      	str	r0, [sp, #268]	@ 0x10c
 8002e3a:	2101      	movs	r1, #1
 8002e3c:	f899 0134 	ldrb.w	r0, [r9, #308]	@ 0x134
 8002e40:	9030      	str	r0, [sp, #192]	@ 0xc0
 8002e42:	2002      	movs	r0, #2
 8002e44:	f899 6131 	ldrb.w	r6, [r9, #305]	@ 0x131
 8002e48:	9044      	str	r0, [sp, #272]	@ 0x110
 8002e4a:	f04f 0b00 	mov.w	fp, #0
 8002e4e:	f04f 0e01 	mov.w	lr, #1
 8002e52:	9637      	str	r6, [sp, #220]	@ 0xdc
 8002e54:	f109 0680 	add.w	r6, r9, #128	@ 0x80
 8002e58:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8002e5a:	468c      	mov	ip, r1
 8002e5c:	ce70      	ldmia	r6, {r4, r5, r6}
 8002e5e:	9345      	str	r3, [sp, #276]	@ 0x114
 8002e60:	4325      	orrs	r5, r4
 8002e62:	e9d9 1224 	ldrd	r1, r2, [r9, #144]	@ 0x90
 8002e66:	fab5 f585 	clz	r5, r5
 8002e6a:	e9d0 3004 	ldrd	r3, r0, [r0, #16]
 8002e6e:	096d      	lsrs	r5, r5, #5
 8002e70:	402e      	ands	r6, r5
 8002e72:	1a59      	subs	r1, r3, r1
 8002e74:	4190      	sbcs	r0, r2
 8002e76:	fab8 f188 	clz	r1, r8
 8002e7a:	f04f 0000 	mov.w	r0, #0
 8002e7e:	bfa8      	it	ge
 8002e80:	2001      	movge	r0, #1
 8002e82:	0949      	lsrs	r1, r1, #5
 8002e84:	4030      	ands	r0, r6
 8002e86:	ea00 000b 	and.w	r0, r0, fp
 8002e8a:	ea10 0201 	ands.w	r2, r0, r1
 8002e8e:	9232      	str	r2, [sp, #200]	@ 0xc8
 8002e90:	d00a      	beq.n	8002ea8 <smoltcp::iface::interface::Interface::poll+0x1d8c>
 8002e92:	f8dd b0f4 	ldr.w	fp, [sp, #244]	@ 0xf4
 8002e96:	f649 3c88 	movw	ip, #39816	@ 0x9b88
 8002e9a:	f04f 0801 	mov.w	r8, #1
 8002e9e:	f6c0 0c00 	movt	ip, #2048	@ 0x800
 8002ea2:	f1ab 0b01 	sub.w	fp, fp, #1
 8002ea6:	e017      	b.n	8002ed8 <smoltcp::iface::interface::Interface::poll+0x1dbc>
 8002ea8:	f8dd b0f4 	ldr.w	fp, [sp, #244]	@ 0xf4
 8002eac:	f1b8 0f00 	cmp.w	r8, #0
 8002eb0:	d008      	beq.n	8002ec4 <smoltcp::iface::interface::Interface::poll+0x1da8>
 8002eb2:	f8d9 0100 	ldr.w	r0, [r9, #256]	@ 0x100
 8002eb6:	f8d9 1108 	ldr.w	r1, [r9, #264]	@ 0x108
 8002eba:	1a08      	subs	r0, r1, r0
 8002ebc:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8002ec0:	f340 81e2 	ble.w	8003288 <smoltcp::iface::interface::Interface::poll+0x216c>
 8002ec4:	f1be 0f00 	cmp.w	lr, #0
 8002ec8:	d006      	beq.n	8002ed8 <smoltcp::iface::interface::Interface::poll+0x1dbc>
 8002eca:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8002ecc:	2318      	movs	r3, #24
 8002ece:	8980      	ldrh	r0, [r0, #12]
 8002ed0:	3836      	subs	r0, #54	@ 0x36
 8002ed2:	9031      	str	r0, [sp, #196]	@ 0xc4
 8002ed4:	2001      	movs	r0, #1
 8002ed6:	e001      	b.n	8002edc <smoltcp::iface::interface::Interface::poll+0x1dc0>
 8002ed8:	2000      	movs	r0, #0
 8002eda:	2314      	movs	r3, #20
 8002edc:	9944      	ldr	r1, [sp, #272]	@ 0x110
 8002ede:	f88d 1349 	strb.w	r1, [sp, #841]	@ 0x349
 8002ee2:	9940      	ldr	r1, [sp, #256]	@ 0x100
 8002ee4:	f8ad 1346 	strh.w	r1, [sp, #838]	@ 0x346
 8002ee8:	9939      	ldr	r1, [sp, #228]	@ 0xe4
 8002eea:	f8ad 1344 	strh.w	r1, [sp, #836]	@ 0x344
 8002eee:	993a      	ldr	r1, [sp, #232]	@ 0xe8
 8002ef0:	f8ad 1342 	strh.w	r1, [sp, #834]	@ 0x342
 8002ef4:	9930      	ldr	r1, [sp, #192]	@ 0xc0
 8002ef6:	f8ad 0330 	strh.w	r0, [sp, #816]	@ 0x330
 8002efa:	9835      	ldr	r0, [sp, #212]	@ 0xd4
 8002efc:	f88d 1341 	strb.w	r1, [sp, #833]	@ 0x341
 8002f00:	9931      	ldr	r1, [sp, #196]	@ 0xc4
 8002f02:	90cb      	str	r0, [sp, #812]	@ 0x32c
 8002f04:	9834      	ldr	r0, [sp, #208]	@ 0xd0
 8002f06:	f8ad 1332 	strh.w	r1, [sp, #818]	@ 0x332
 8002f0a:	90ca      	str	r0, [sp, #808]	@ 0x328
 8002f0c:	9838      	ldr	r0, [sp, #224]	@ 0xe0
 8002f0e:	993e      	ldr	r1, [sp, #248]	@ 0xf8
 8002f10:	9a45      	ldr	r2, [sp, #276]	@ 0x114
 8002f12:	9e37      	ldr	r6, [sp, #220]	@ 0xdc
 8002f14:	90c9      	str	r0, [sp, #804]	@ 0x324
 8002f16:	2000      	movs	r0, #0
 8002f18:	91bf      	str	r1, [sp, #764]	@ 0x2fc
 8002f1a:	2a00      	cmp	r2, #0
 8002f1c:	9943      	ldr	r1, [sp, #268]	@ 0x10c
 8002f1e:	91be      	str	r1, [sp, #760]	@ 0x2f8
 8002f20:	9c3f      	ldr	r4, [sp, #252]	@ 0xfc
 8002f22:	993b      	ldr	r1, [sp, #236]	@ 0xec
 8002f24:	90c6      	str	r0, [sp, #792]	@ 0x318
 8002f26:	90c3      	str	r0, [sp, #780]	@ 0x30c
 8002f28:	90c0      	str	r0, [sp, #768]	@ 0x300
 8002f2a:	90bd      	str	r0, [sp, #756]	@ 0x2f4
 8002f2c:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8002f2e:	9d46      	ldr	r5, [sp, #280]	@ 0x118
 8002f30:	f88d 6348 	strb.w	r6, [sp, #840]	@ 0x348
 8002f34:	f88d 2340 	strb.w	r2, [sp, #832]	@ 0x340
 8002f38:	f8cd b33c 	str.w	fp, [sp, #828]	@ 0x33c
 8002f3c:	e9cd c8cd 	strd	ip, r8, [sp, #820]	@ 0x334
 8002f40:	f8cd a358 	str.w	sl, [sp, #856]	@ 0x358
 8002f44:	94d4      	str	r4, [sp, #848]	@ 0x350
 8002f46:	91d3      	str	r1, [sp, #844]	@ 0x34c
 8002f48:	90bc      	str	r0, [sp, #752]	@ 0x2f0
 8002f4a:	bf18      	it	ne
 8002f4c:	f043 0303 	orrne.w	r3, r3, #3
 8002f50:	993c      	ldr	r1, [sp, #240]	@ 0xf0
 8002f52:	eb03 0046 	add.w	r0, r3, r6, lsl #1
 8002f56:	2900      	cmp	r1, #0
 8002f58:	bf18      	it	ne
 8002f5a:	300a      	addne	r0, #10
 8002f5c:	3003      	adds	r0, #3
 8002f5e:	f020 0003 	bic.w	r0, r0, #3
 8002f62:	4440      	add	r0, r8
 8002f64:	90d5      	str	r0, [sp, #852]	@ 0x354
 8002f66:	68b8      	ldr	r0, [r7, #8]
 8002f68:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 8002f6c:	4288      	cmp	r0, r1
 8002f6e:	f080 819c 	bcs.w	80032aa <smoltcp::iface::interface::Interface::poll+0x218e>
 8002f72:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 8002f76:	9a41      	ldr	r2, [sp, #260]	@ 0x104
 8002f78:	4348      	muls	r0, r1
 8002f7a:	6811      	ldr	r1, [r2, #0]
 8002f7c:	5808      	ldr	r0, [r1, r0]
 8002f7e:	2800      	cmp	r0, #0
 8002f80:	f100 811e 	bmi.w	80031c0 <smoltcp::iface::interface::Interface::poll+0x20a4>
 8002f84:	2000      	movs	r0, #0
 8002f86:	9b36      	ldr	r3, [sp, #216]	@ 0xd8
 8002f88:	90a3      	str	r0, [sp, #652]	@ 0x28c
 8002f8a:	a9a3      	add	r1, sp, #652	@ 0x28c
 8002f8c:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8002f8e:	92a5      	str	r2, [sp, #660]	@ 0x294
 8002f90:	2200      	movs	r2, #0
 8002f92:	f7fd f9e3 	bl	800035c <smoltcp::iface::interface::InterfaceInner::dispatch_ip>
 8002f96:	b2c0      	uxtb	r0, r0
 8002f98:	2802      	cmp	r0, #2
 8002f9a:	d13d      	bne.n	8003018 <smoltcp::iface::interface::Interface::poll+0x1efc>
 8002f9c:	e9d9 0120 	ldrd	r0, r1, [r9, #128]	@ 0x80
 8002fa0:	4308      	orrs	r0, r1
 8002fa2:	d10c      	bne.n	8002fbe <smoltcp::iface::interface::Interface::poll+0x1ea2>
 8002fa4:	9e48      	ldr	r6, [sp, #288]	@ 0x120
 8002fa6:	f109 0340 	add.w	r3, r9, #64	@ 0x40
 8002faa:	f109 0c88 	add.w	ip, r9, #136	@ 0x88
 8002fae:	e9d6 6504 	ldrd	r6, r5, [r6, #16]
 8002fb2:	cb0f      	ldmia	r3, {r0, r1, r2, r3}
 8002fb4:	1992      	adds	r2, r2, r6
 8002fb6:	416b      	adcs	r3, r5
 8002fb8:	9d46      	ldr	r5, [sp, #280]	@ 0x118
 8002fba:	e88c 000f 	stmia.w	ip, {r0, r1, r2, r3}
 8002fbe:	2000      	movs	r0, #0
 8002fc0:	9445      	str	r4, [sp, #276]	@ 0x114
 8002fc2:	e9c9 001c 	strd	r0, r0, [r9, #112]	@ 0x70
 8002fc6:	9832      	ldr	r0, [sp, #200]	@ 0xc8
 8002fc8:	2800      	cmp	r0, #0
 8002fca:	f47f aaf5 	bne.w	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 8002fce:	9844      	ldr	r0, [sp, #272]	@ 0x110
 8002fd0:	f000 0106 	and.w	r1, r0, #6
 8002fd4:	4640      	mov	r0, r8
 8002fd6:	2902      	cmp	r1, #2
 8002fd8:	bf08      	it	eq
 8002fda:	3001      	addeq	r0, #1
 8002fdc:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8002fe0:	f340 80e3 	ble.w	80031aa <smoltcp::iface::interface::Interface::poll+0x208e>
 8002fe4:	9a40      	ldr	r2, [sp, #256]	@ 0x100
 8002fe6:	4458      	add	r0, fp
 8002fe8:	f8a9 212e 	strh.w	r2, [r9, #302]	@ 0x12e
 8002fec:	2902      	cmp	r1, #2
 8002fee:	9a3e      	ldr	r2, [sp, #248]	@ 0xf8
 8002ff0:	f04f 0100 	mov.w	r1, #0
 8002ff4:	9b43      	ldr	r3, [sp, #268]	@ 0x10c
 8002ff6:	f8c9 0108 	str.w	r0, [r9, #264]	@ 0x108
 8002ffa:	e9c9 3226 	strd	r3, r2, [r9, #152]	@ 0x98
 8002ffe:	bf08      	it	eq
 8003000:	f04f 31ff 	moveq.w	r1, #4294967295	@ 0xffffffff
 8003004:	4588      	cmp	r8, r1
 8003006:	d109      	bne.n	800301c <smoltcp::iface::interface::Interface::poll+0x1f00>
 8003008:	9848      	ldr	r0, [sp, #288]	@ 0x120
 800300a:	68c1      	ldr	r1, [r0, #12]
 800300c:	4648      	mov	r0, r9
 800300e:	f003 f9dd 	bl	80063cc <smoltcp::socket::tcp::Socket::seq_to_transmit>
 8003012:	f8dd 80fc 	ldr.w	r8, [sp, #252]	@ 0xfc
 8003016:	e04d      	b.n	80030b4 <smoltcp::iface::interface::Interface::poll+0x1f98>
 8003018:	4622      	mov	r2, r4
 800301a:	e084      	b.n	8003126 <smoltcp::iface::interface::Interface::poll+0x200a>
 800301c:	9948      	ldr	r1, [sp, #288]	@ 0x120
 800301e:	f8dd 80fc 	ldr.w	r8, [sp, #252]	@ 0xfc
 8003022:	e9d1 2104 	ldrd	r2, r1, [r1, #16]
 8003026:	f8d9 3018 	ldr.w	r3, [r9, #24]
 800302a:	2b01      	cmp	r3, #1
 800302c:	d104      	bne.n	8003038 <smoltcp::iface::interface::Interface::poll+0x1f1c>
 800302e:	f8d9 301c 	ldr.w	r3, [r9, #28]
 8003032:	1ac3      	subs	r3, r0, r3
 8003034:	2b01      	cmp	r3, #1
 8003036:	db0e      	blt.n	8003056 <smoltcp::iface::interface::Interface::poll+0x1f3a>
 8003038:	e9d9 3600 	ldrd	r3, r6, [r9]
 800303c:	2401      	movs	r4, #1
 800303e:	e9c9 4006 	strd	r4, r0, [r9, #24]
 8003042:	4333      	orrs	r3, r6
 8003044:	d107      	bne.n	8003056 <smoltcp::iface::interface::Interface::poll+0x1f3a>
 8003046:	2301      	movs	r3, #1
 8003048:	e9c9 1003 	strd	r1, r0, [r9, #12]
 800304c:	f8c9 3000 	str.w	r3, [r9]
 8003050:	2300      	movs	r3, #0
 8003052:	e9c9 3201 	strd	r3, r2, [r9, #4]
 8003056:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8003058:	68c1      	ldr	r1, [r0, #12]
 800305a:	4648      	mov	r0, r9
 800305c:	f003 f9b6 	bl	80063cc <smoltcp::socket::tcp::Socket::seq_to_transmit>
 8003060:	bb40      	cbnz	r0, 80030b4 <smoltcp::iface::interface::Interface::poll+0x1f98>
 8003062:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8003064:	f8d9 2080 	ldr.w	r2, [r9, #128]	@ 0x80
 8003068:	e9d0 1004 	ldrd	r1, r0, [r0, #16]
 800306c:	e8df f002 	tbb	[pc, r2]
 8003070:	22027a02 	.word	0x22027a02
 8003074:	e9d9 2308 	ldrd	r2, r3, [r9, #32]
 8003078:	009e      	lsls	r6, r3, #2
 800307a:	2e05      	cmp	r6, #5
 800307c:	f04f 0605 	mov.w	r6, #5
 8003080:	bf88      	it	hi
 8003082:	009e      	lslhi	r6, r3, #2
 8003084:	4432      	add	r2, r6
 8003086:	2a0a      	cmp	r2, #10
 8003088:	bf98      	it	ls
 800308a:	220a      	movls	r2, #10
 800308c:	f242 7310 	movw	r3, #10000	@ 0x2710
 8003090:	429a      	cmp	r2, r3
 8003092:	bf28      	it	cs
 8003094:	461a      	movcs	r2, r3
 8003096:	f44f 737a 	mov.w	r3, #1000	@ 0x3e8
 800309a:	435a      	muls	r2, r3
 800309c:	2301      	movs	r3, #1
 800309e:	f8c9 3080 	str.w	r3, [r9, #128]	@ 0x80
 80030a2:	2300      	movs	r3, #0
 80030a4:	1889      	adds	r1, r1, r2
 80030a6:	f140 0000 	adc.w	r0, r0, #0
 80030aa:	e9c9 3121 	strd	r3, r1, [r9, #132]	@ 0x84
 80030ae:	f109 018c 	add.w	r1, r9, #140	@ 0x8c
 80030b2:	c10d      	stmia	r1!, {r0, r2, r3}
 80030b4:	f899 0133 	ldrb.w	r0, [r9, #307]	@ 0x133
 80030b8:	f8cd 8114 	str.w	r8, [sp, #276]	@ 0x114
 80030bc:	2800      	cmp	r0, #0
 80030be:	f47f aa7b 	bne.w	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 80030c2:	2000      	movs	r0, #0
 80030c4:	f8a9 011c 	strh.w	r0, [r9, #284]	@ 0x11c
 80030c8:	983f      	ldr	r0, [sp, #252]	@ 0xfc
 80030ca:	9045      	str	r0, [sp, #276]	@ 0x114
 80030cc:	f7ff ba74 	b.w	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 80030d0:	f04f 0c00 	mov.w	ip, #0
 80030d4:	9843      	ldr	r0, [sp, #268]	@ 0x10c
 80030d6:	42b1      	cmp	r1, r6
 80030d8:	f200 80bf 	bhi.w	800325a <smoltcp::iface::interface::Interface::poll+0x213e>
 80030dc:	2202      	movs	r2, #2
 80030de:	2d00      	cmp	r5, #0
 80030e0:	f47f ac57 	bne.w	8002992 <smoltcp::iface::interface::Interface::poll+0x1876>
 80030e4:	2300      	movs	r3, #0
 80030e6:	9d46      	ldr	r5, [sp, #280]	@ 0x118
 80030e8:	1a61      	subs	r1, r4, r1
 80030ea:	2a02      	cmp	r2, #2
 80030ec:	e9c9 3112 	strd	r3, r1, [r9, #72]	@ 0x48
 80030f0:	d10a      	bne.n	8003108 <smoltcp::iface::interface::Interface::poll+0x1fec>
 80030f2:	993e      	ldr	r1, [sp, #248]	@ 0xf8
 80030f4:	9b3f      	ldr	r3, [sp, #252]	@ 0xfc
 80030f6:	3101      	adds	r1, #1
 80030f8:	fbb1 f2f3 	udiv	r2, r1, r3
 80030fc:	fb02 1113 	mls	r1, r2, r3, r1
 8003100:	1e42      	subs	r2, r0, #1
 8003102:	e9c9 120e 	strd	r1, r2, [r9, #56]	@ 0x38
 8003106:	2202      	movs	r2, #2
 8003108:	2a02      	cmp	r2, #2
 800310a:	f04f 0102 	mov.w	r1, #2
 800310e:	bf28      	it	cs
 8003110:	460a      	movcs	r2, r1
 8003112:	2a02      	cmp	r2, #2
 8003114:	f43f aa50 	beq.w	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 8003118:	07d1      	lsls	r1, r2, #31
 800311a:	d057      	beq.n	80031cc <smoltcp::iface::interface::Interface::poll+0x20b0>
 800311c:	9a45      	ldr	r2, [sp, #276]	@ 0x114
 800311e:	f1bc 0f00 	cmp.w	ip, #0
 8003122:	f000 8087 	beq.w	8003234 <smoltcp::iface::interface::Interface::poll+0x2118>
 8003126:	9848      	ldr	r0, [sp, #288]	@ 0x120
 8003128:	9245      	str	r2, [sp, #276]	@ 0x114
 800312a:	e9d0 0104 	ldrd	r0, r1, [r0, #16]
 800312e:	f8c9 2139 	str.w	r2, [r9, #313]	@ 0x139
 8003132:	2201      	movs	r2, #1
 8003134:	f889 2138 	strb.w	r2, [r9, #312]	@ 0x138
 8003138:	f244 2240 	movw	r2, #16960	@ 0x4240
 800313c:	f2c0 020f 	movt	r2, #15
 8003140:	1880      	adds	r0, r0, r2
 8003142:	f141 0100 	adc.w	r1, r1, #0
 8003146:	e9c9 0150 	strd	r0, r1, [r9, #320]	@ 0x140
 800314a:	f7ff ba35 	b.w	80025b8 <smoltcp::iface::interface::Interface::poll+0x149c>
 800314e:	f1b8 0f00 	cmp.w	r8, #0
 8003152:	d002      	beq.n	800315a <smoltcp::iface::interface::Interface::poll+0x203e>
 8003154:	2001      	movs	r0, #1
 8003156:	9043      	str	r0, [sp, #268]	@ 0x10c
 8003158:	e650      	b.n	8002dfc <smoltcp::iface::interface::Interface::poll+0x1ce0>
 800315a:	2001      	movs	r0, #1
 800315c:	f04f 0800 	mov.w	r8, #0
 8003160:	9043      	str	r0, [sp, #268]	@ 0x10c
 8003162:	e64a      	b.n	8002dfa <smoltcp::iface::interface::Interface::poll+0x1cde>
 8003164:	e9d9 2322 	ldrd	r2, r3, [r9, #136]	@ 0x88
 8003168:	1a8a      	subs	r2, r1, r2
 800316a:	eb70 0203 	sbcs.w	r2, r0, r3
 800316e:	dba1      	blt.n	80030b4 <smoltcp::iface::interface::Interface::poll+0x1f98>
 8003170:	e9d9 2324 	ldrd	r2, r3, [r9, #144]	@ 0x90
 8003174:	005e      	lsls	r6, r3, #1
 8003176:	1889      	adds	r1, r1, r2
 8003178:	4158      	adcs	r0, r3
 800317a:	2301      	movs	r3, #1
 800317c:	f8c9 3080 	str.w	r3, [r9, #128]	@ 0x80
 8003180:	2300      	movs	r3, #0
 8003182:	e9c9 3121 	strd	r3, r1, [r9, #132]	@ 0x84
 8003186:	f109 018c 	add.w	r1, r9, #140	@ 0x8c
 800318a:	ea46 76d2 	orr.w	r6, r6, r2, lsr #31
 800318e:	0052      	lsls	r2, r2, #1
 8003190:	c145      	stmia	r1!, {r0, r2, r6}
 8003192:	e78f      	b.n	80030b4 <smoltcp::iface::interface::Interface::poll+0x1f98>
 8003194:	f64a 7004 	movw	r0, #44804	@ 0xaf04
 8003198:	f64a 7230 	movw	r2, #44848	@ 0xaf30
 800319c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80031a0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80031a4:	212c      	movs	r1, #44	@ 0x2c
 80031a6:	f000 fb2f 	bl	8003808 <core::panicking::panic>
 80031aa:	f64a 3070 	movw	r0, #43888	@ 0xab70
 80031ae:	f64a 32a8 	movw	r2, #43944	@ 0xaba8
 80031b2:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80031b6:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80031ba:	2171      	movs	r1, #113	@ 0x71
 80031bc:	f000 fb02 	bl	80037c4 <core::panicking::panic_fmt>
 80031c0:	f240 0025 	movw	r0, #37	@ 0x25
 80031c4:	f2c0 0000 	movt	r0, #0
 80031c8:	f001 f8c7 	bl	800435a <defmt::export::acquire_header_and_release>
 80031cc:	f50d 7d5b 	add.w	sp, sp, #876	@ 0x36c
 80031d0:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80031d4:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80031d6:	f64a 70f0 	movw	r0, #45040	@ 0xaff0
 80031da:	f24b 0234 	movw	r2, #45108	@ 0xb034
 80031de:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80031e2:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80031e6:	2141      	movs	r1, #65	@ 0x41
 80031e8:	f000 ff25 	bl	8004036 <core::option::expect_failed>
 80031ec:	f24b 0044 	movw	r0, #45124	@ 0xb044
 80031f0:	f24b 0288 	movw	r2, #45192	@ 0xb088
 80031f4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80031f8:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80031fc:	2142      	movs	r1, #66	@ 0x42
 80031fe:	f000 ff1a 	bl	8004036 <core::option::expect_failed>
 8003202:	f64a 7360 	movw	r3, #44896	@ 0xaf60
 8003206:	2000      	movs	r0, #0
 8003208:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800320c:	f240 52f2 	movw	r2, #1522	@ 0x5f2
 8003210:	f000 fa67 	bl	80036e2 <core::slice::index::slice_index_fail>
 8003214:	f64a 03b4 	movw	r3, #43188	@ 0xa8b4
 8003218:	4640      	mov	r0, r8
 800321a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800321e:	462a      	mov	r2, r5
 8003220:	f000 fa5f 	bl	80036e2 <core::slice::index::slice_index_fail>
 8003224:	f64a 03d4 	movw	r3, #43220	@ 0xa8d4
 8003228:	2000      	movs	r0, #0
 800322a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800322e:	4632      	mov	r2, r6
 8003230:	f000 fa57 	bl	80036e2 <core::slice::index::slice_index_fail>
 8003234:	f24a 4058 	movw	r0, #42072	@ 0xa458
 8003238:	f24a 4270 	movw	r2, #42096	@ 0xa470
 800323c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003240:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003244:	2116      	movs	r1, #22
 8003246:	f000 fef6 	bl	8004036 <core::option::expect_failed>
 800324a:	f64a 03b4 	movw	r3, #43188	@ 0xa8b4
 800324e:	4628      	mov	r0, r5
 8003250:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003254:	4631      	mov	r1, r6
 8003256:	f000 fa44 	bl	80036e2 <core::slice::index::slice_index_fail>
 800325a:	f64a 0080 	movw	r0, #43136	@ 0xa880
 800325e:	f64a 02a4 	movw	r2, #43172	@ 0xa8a4
 8003262:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003266:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800326a:	2122      	movs	r1, #34	@ 0x22
 800326c:	f000 facc 	bl	8003808 <core::panicking::panic>
 8003270:	f64a 0050 	movw	r0, #43088	@ 0xa850
 8003274:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003278:	f000 fa29 	bl	80036ce <core::option::unwrap_failed>
 800327c:	f64a 00c4 	movw	r0, #43204	@ 0xa8c4
 8003280:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003284:	f000 fa23 	bl	80036ce <core::option::unwrap_failed>
 8003288:	f64a 1094 	movw	r0, #43412	@ 0xa994
 800328c:	f64a 12c8 	movw	r2, #43464	@ 0xa9c8
 8003290:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003294:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003298:	2167      	movs	r1, #103	@ 0x67
 800329a:	f000 fa93 	bl	80037c4 <core::panicking::panic_fmt>
 800329e:	f64a 0360 	movw	r3, #43104	@ 0xa860
 80032a2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80032a6:	f000 fa1c 	bl	80036e2 <core::slice::index::slice_index_fail>
 80032aa:	f64a 72a0 	movw	r2, #44960	@ 0xafa0
 80032ae:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80032b2:	f000 fa95 	bl	80037e0 <core::panicking::panic_bounds_check>
 80032b6:	f64a 7270 	movw	r2, #44912	@ 0xaf70
 80032ba:	4628      	mov	r0, r5
 80032bc:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80032c0:	4631      	mov	r1, r6
 80032c2:	f000 fa8d 	bl	80037e0 <core::panicking::panic_bounds_check>
 80032c6:	f64a 5324 	movw	r3, #44324	@ 0xad24
 80032ca:	2000      	movs	r0, #0
 80032cc:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80032d0:	462a      	mov	r2, r5
 80032d2:	f000 fa06 	bl	80036e2 <core::slice::index::slice_index_fail>
 80032d6:	f64a 50b4 	movw	r0, #44468	@ 0xadb4
 80032da:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80032de:	f000 fedc 	bl	800409a <core::panicking::panic_const::panic_const_rem_by_zero>
 80032e2:	f64a 7280 	movw	r2, #44928	@ 0xaf80
 80032e6:	4628      	mov	r0, r5
 80032e8:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80032ec:	f000 fa78 	bl	80037e0 <core::panicking::panic_bounds_check>
 80032f0:	f64a 0270 	movw	r2, #43120	@ 0xa870
 80032f4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80032f8:	f000 fa72 	bl	80037e0 <core::panicking::panic_bounds_check>
 80032fc:	f64a 5364 	movw	r3, #44388	@ 0xad64
 8003300:	2008      	movs	r0, #8
 8003302:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003306:	210e      	movs	r1, #14
 8003308:	f000 f9eb 	bl	80036e2 <core::slice::index::slice_index_fail>
 800330c:	f64a 5374 	movw	r3, #44404	@ 0xad74
 8003310:	200e      	movs	r0, #14
 8003312:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003316:	f000 f9e4 	bl	80036e2 <core::slice::index::slice_index_fail>
 800331a:	f64a 5384 	movw	r3, #44420	@ 0xad84
 800331e:	4608      	mov	r0, r1
 8003320:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003324:	4629      	mov	r1, r5
 8003326:	f000 f9dc 	bl	80036e2 <core::slice::index::slice_index_fail>
 800332a:	f64a 5394 	movw	r3, #44436	@ 0xad94
 800332e:	2018      	movs	r0, #24
 8003330:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003334:	211c      	movs	r1, #28
 8003336:	f000 f9d4 	bl	80036e2 <core::slice::index::slice_index_fail>
 800333a:	f64a 0270 	movw	r2, #43120	@ 0xa870
 800333e:	4619      	mov	r1, r3
 8003340:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003344:	f000 fa4c 	bl	80037e0 <core::panicking::panic_bounds_check>
 8003348:	f64a 43a4 	movw	r3, #44196	@ 0xaca4
 800334c:	4610      	mov	r0, r2
 800334e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003352:	462a      	mov	r2, r5
 8003354:	f000 f9c5 	bl	80036e2 <core::slice::index::slice_index_fail>
 8003358:	f64a 3340 	movw	r3, #43840	@ 0xab40
 800335c:	4628      	mov	r0, r5
 800335e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8003362:	4631      	mov	r1, r6
 8003364:	4632      	mov	r2, r6
 8003366:	f000 f9bc 	bl	80036e2 <core::slice::index::slice_index_fail>
 800336a:	f64a 7290 	movw	r2, #44944	@ 0xaf90
 800336e:	4628      	mov	r0, r5
 8003370:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003374:	f000 fa34 	bl	80037e0 <core::panicking::panic_bounds_check>
 8003378:	2002      	movs	r0, #2
 800337a:	908a      	str	r0, [sp, #552]	@ 0x228
 800337c:	a88a      	add	r0, sp, #552	@ 0x228
 800337e:	f002 fde8 	bl	8005f52 <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>
 8003382:	f64a 1004 	movw	r0, #43268	@ 0xa904
 8003386:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800338a:	f000 f9a0 	bl	80036ce <core::option::unwrap_failed>
 800338e:	f64a 7250 	movw	r2, #44880	@ 0xaf50
 8003392:	4628      	mov	r0, r5
 8003394:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8003398:	f000 fa22 	bl	80037e0 <core::panicking::panic_bounds_check>
 800339c:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 800339e:	f64a 1324 	movw	r3, #43300	@ 0xa924
 80033a2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80033a6:	2000      	movs	r0, #0
 80033a8:	2102      	movs	r1, #2
 80033aa:	f000 f99a 	bl	80036e2 <core::slice::index::slice_index_fail>
 80033ae:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 80033b0:	f64a 3320 	movw	r3, #43808	@ 0xab20
 80033b4:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80033b8:	2000      	movs	r0, #0
 80033ba:	2104      	movs	r1, #4
 80033bc:	f000 f991 	bl	80036e2 <core::slice::index::slice_index_fail>
 80033c0:	9a2b      	ldr	r2, [sp, #172]	@ 0xac
 80033c2:	f64a 3330 	movw	r3, #43824	@ 0xab30
 80033c6:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80033ca:	2004      	movs	r0, #4
 80033cc:	2108      	movs	r1, #8
 80033ce:	f000 f988 	bl	80036e2 <core::slice::index::slice_index_fail>
 80033d2:	992d      	ldr	r1, [sp, #180]	@ 0xb4
 80033d4:	f64a 2358 	movw	r3, #43608	@ 0xaa58
 80033d8:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80033dc:	460a      	mov	r2, r1
 80033de:	f000 f980 	bl	80036e2 <core::slice::index::slice_index_fail>

080033e2 <core::fmt::write>:
 80033e2:	b5f0      	push	{r4, r5, r6, r7, lr}
 80033e4:	af03      	add	r7, sp, #12
 80033e6:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 80033ea:	b085      	sub	sp, #20
 80033ec:	4606      	mov	r6, r0
 80033ee:	469b      	mov	fp, r3
 80033f0:	468a      	mov	sl, r1
 80033f2:	07d8      	lsls	r0, r3, #31
 80033f4:	d164      	bne.n	80034c0 <core::fmt::write+0xde>
 80033f6:	7815      	ldrb	r5, [r2, #0]
 80033f8:	2000      	movs	r0, #0
 80033fa:	2d00      	cmp	r5, #0
 80033fc:	d06e      	beq.n	80034dc <core::fmt::write+0xfa>
 80033fe:	f8da 900c 	ldr.w	r9, [sl, #12]
 8003402:	f04f 0800 	mov.w	r8, #0
 8003406:	e002      	b.n	800340e <core::fmt::write+0x2c>
 8003408:	7815      	ldrb	r5, [r2, #0]
 800340a:	2d00      	cmp	r5, #0
 800340c:	d06a      	beq.n	80034e4 <core::fmt::write+0x102>
 800340e:	1c54      	adds	r4, r2, #1
 8003410:	b268      	sxtb	r0, r5
 8003412:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8003416:	dd07      	ble.n	8003428 <core::fmt::write+0x46>
 8003418:	4630      	mov	r0, r6
 800341a:	4621      	mov	r1, r4
 800341c:	462a      	mov	r2, r5
 800341e:	47c8      	blx	r9
 8003420:	2800      	cmp	r0, #0
 8003422:	d15a      	bne.n	80034da <core::fmt::write+0xf8>
 8003424:	1962      	adds	r2, r4, r5
 8003426:	e7ef      	b.n	8003408 <core::fmt::write+0x26>
 8003428:	2d80      	cmp	r5, #128	@ 0x80
 800342a:	d009      	beq.n	8003440 <core::fmt::write+0x5e>
 800342c:	2dc0      	cmp	r5, #192	@ 0xc0
 800342e:	d112      	bne.n	8003456 <core::fmt::write+0x74>
 8003430:	f85b 0038 	ldr.w	r0, [fp, r8, lsl #3]
 8003434:	2100      	movs	r1, #0
 8003436:	9104      	str	r1, [sp, #16]
 8003438:	2120      	movs	r1, #32
 800343a:	f2c6 0100 	movt	r1, #24576	@ 0x6000
 800343e:	e031      	b.n	80034a4 <core::fmt::write+0xc2>
 8003440:	f8b2 4001 	ldrh.w	r4, [r2, #1]
 8003444:	1cd5      	adds	r5, r2, #3
 8003446:	4630      	mov	r0, r6
 8003448:	4629      	mov	r1, r5
 800344a:	4622      	mov	r2, r4
 800344c:	47c8      	blx	r9
 800344e:	2800      	cmp	r0, #0
 8003450:	d143      	bne.n	80034da <core::fmt::write+0xf8>
 8003452:	192a      	adds	r2, r5, r4
 8003454:	e7d8      	b.n	8003408 <core::fmt::write+0x26>
 8003456:	07e8      	lsls	r0, r5, #31
 8003458:	d103      	bne.n	8003462 <core::fmt::write+0x80>
 800345a:	2120      	movs	r1, #32
 800345c:	f2c6 0100 	movt	r1, #24576	@ 0x6000
 8003460:	e002      	b.n	8003468 <core::fmt::write+0x86>
 8003462:	f8d2 1001 	ldr.w	r1, [r2, #1]
 8003466:	1d54      	adds	r4, r2, #5
 8003468:	07a8      	lsls	r0, r5, #30
 800346a:	bf4c      	ite	mi
 800346c:	f834 2b02 	ldrhmi.w	r2, [r4], #2
 8003470:	2200      	movpl	r2, #0
 8003472:	0768      	lsls	r0, r5, #29
 8003474:	bf4c      	ite	mi
 8003476:	f834 3b02 	ldrhmi.w	r3, [r4], #2
 800347a:	2300      	movpl	r3, #0
 800347c:	0728      	lsls	r0, r5, #28
 800347e:	bf48      	it	mi
 8003480:	f834 8b02 	ldrhmi.w	r8, [r4], #2
 8003484:	06e8      	lsls	r0, r5, #27
 8003486:	bf44      	itt	mi
 8003488:	eb0b 00c2 	addmi.w	r0, fp, r2, lsl #3
 800348c:	8882      	ldrhmi	r2, [r0, #4]
 800348e:	06a8      	lsls	r0, r5, #26
 8003490:	bf44      	itt	mi
 8003492:	eb0b 00c3 	addmi.w	r0, fp, r3, lsl #3
 8003496:	8883      	ldrhmi	r3, [r0, #4]
 8003498:	f85b 0038 	ldr.w	r0, [fp, r8, lsl #3]
 800349c:	f8ad 3012 	strh.w	r3, [sp, #18]
 80034a0:	f8ad 2010 	strh.w	r2, [sp, #16]
 80034a4:	9103      	str	r1, [sp, #12]
 80034a6:	eb0b 01c8 	add.w	r1, fp, r8, lsl #3
 80034aa:	f8cd a008 	str.w	sl, [sp, #8]
 80034ae:	684a      	ldr	r2, [r1, #4]
 80034b0:	a901      	add	r1, sp, #4
 80034b2:	9601      	str	r6, [sp, #4]
 80034b4:	4790      	blx	r2
 80034b6:	b980      	cbnz	r0, 80034da <core::fmt::write+0xf8>
 80034b8:	f108 0801 	add.w	r8, r8, #1
 80034bc:	4622      	mov	r2, r4
 80034be:	e7a3      	b.n	8003408 <core::fmt::write+0x26>
 80034c0:	ea4f 035b 	mov.w	r3, fp, lsr #1
 80034c4:	4611      	mov	r1, r2
 80034c6:	f8da c00c 	ldr.w	ip, [sl, #12]
 80034ca:	4630      	mov	r0, r6
 80034cc:	461a      	mov	r2, r3
 80034ce:	b005      	add	sp, #20
 80034d0:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80034d4:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 80034d8:	4760      	bx	ip
 80034da:	2001      	movs	r0, #1
 80034dc:	b005      	add	sp, #20
 80034de:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80034e2:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80034e4:	2000      	movs	r0, #0
 80034e6:	b005      	add	sp, #20
 80034e8:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80034ec:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80034ee:	d4d4      	bmi.n	800349a <core::fmt::write+0xb8>

080034f0 <core::fmt::Formatter::pad_integral>:
 80034f0:	b5f0      	push	{r4, r5, r6, r7, lr}
 80034f2:	af03      	add	r7, sp, #12
 80034f4:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 80034f8:	b085      	sub	sp, #20
 80034fa:	6886      	ldr	r6, [r0, #8]
 80034fc:	460c      	mov	r4, r1
 80034fe:	f8b0 800c 	ldrh.w	r8, [r0, #12]
 8003502:	4692      	mov	sl, r2
 8003504:	f416 1100 	ands.w	r1, r6, #2097152	@ 0x200000
 8003508:	f04f 0b2b 	mov.w	fp, #43	@ 0x2b
 800350c:	f3c6 53c0 	ubfx	r3, r6, #23, #1
 8003510:	bf08      	it	eq
 8003512:	f44f 1b88 	moveq.w	fp, #1114112	@ 0x110000
 8003516:	eb02 5951 	add.w	r9, r2, r1, lsr #21
 800351a:	45c1      	cmp	r9, r8
 800351c:	d211      	bcs.n	8003542 <core::fmt::Formatter::pad_integral+0x52>
 800351e:	01f1      	lsls	r1, r6, #7
 8003520:	d41d      	bmi.n	800355e <core::fmt::Formatter::pad_integral+0x6e>
 8003522:	f3c6 7141 	ubfx	r1, r6, #29, #2
 8003526:	9304      	str	r3, [sp, #16]
 8003528:	9403      	str	r4, [sp, #12]
 800352a:	eba8 0309 	sub.w	r3, r8, r9
 800352e:	f36f 565f 	bfc	r6, #21, #11
 8003532:	f04f 0800 	mov.w	r8, #0
 8003536:	e8df f001 	tbb	[pc, r1]
 800353a:	025b      	.short	0x025b
 800353c:	0258      	.short	0x0258
 800353e:	4698      	mov	r8, r3
 8003540:	e056      	b.n	80035f0 <core::fmt::Formatter::pad_integral+0x100>
 8003542:	4626      	mov	r6, r4
 8003544:	e9d0 4500 	ldrd	r4, r5, [r0]
 8003548:	4629      	mov	r1, r5
 800354a:	465a      	mov	r2, fp
 800354c:	4620      	mov	r0, r4
 800354e:	f000 f89d 	bl	800368c <core::fmt::Formatter::pad_integral::write_prefix>
 8003552:	b300      	cbz	r0, 8003596 <core::fmt::Formatter::pad_integral+0xa6>
 8003554:	2001      	movs	r0, #1
 8003556:	b005      	add	sp, #20
 8003558:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800355c:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800355e:	e9d0 6102 	ldrd	r6, r1, [r0, #8]
 8003562:	465a      	mov	r2, fp
 8003564:	f8cd a010 	str.w	sl, [sp, #16]
 8003568:	9102      	str	r1, [sp, #8]
 800356a:	2100      	movs	r1, #0
 800356c:	f6c9 71e0 	movt	r1, #40928	@ 0x9fe0
 8003570:	e9d0 a500 	ldrd	sl, r5, [r0]
 8003574:	4031      	ands	r1, r6
 8003576:	9003      	str	r0, [sp, #12]
 8003578:	f041 5100 	orr.w	r1, r1, #536870912	@ 0x20000000
 800357c:	f041 0130 	orr.w	r1, r1, #48	@ 0x30
 8003580:	6081      	str	r1, [r0, #8]
 8003582:	4650      	mov	r0, sl
 8003584:	4629      	mov	r1, r5
 8003586:	f000 f881 	bl	800368c <core::fmt::Formatter::pad_integral::write_prefix>
 800358a:	b170      	cbz	r0, 80035aa <core::fmt::Formatter::pad_integral+0xba>
 800358c:	2001      	movs	r0, #1
 800358e:	b005      	add	sp, #20
 8003590:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003594:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003596:	68eb      	ldr	r3, [r5, #12]
 8003598:	4620      	mov	r0, r4
 800359a:	4631      	mov	r1, r6
 800359c:	4652      	mov	r2, sl
 800359e:	b005      	add	sp, #20
 80035a0:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80035a4:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 80035a8:	4718      	bx	r3
 80035aa:	eba8 0009 	sub.w	r0, r8, r9
 80035ae:	9601      	str	r6, [sp, #4]
 80035b0:	46a3      	mov	fp, r4
 80035b2:	2600      	movs	r6, #0
 80035b4:	b284      	uxth	r4, r0
 80035b6:	b2b0      	uxth	r0, r6
 80035b8:	42a0      	cmp	r0, r4
 80035ba:	d20b      	bcs.n	80035d4 <core::fmt::Formatter::pad_integral+0xe4>
 80035bc:	692a      	ldr	r2, [r5, #16]
 80035be:	4650      	mov	r0, sl
 80035c0:	2130      	movs	r1, #48	@ 0x30
 80035c2:	4790      	blx	r2
 80035c4:	3601      	adds	r6, #1
 80035c6:	2800      	cmp	r0, #0
 80035c8:	d0f5      	beq.n	80035b6 <core::fmt::Formatter::pad_integral+0xc6>
 80035ca:	2001      	movs	r0, #1
 80035cc:	b005      	add	sp, #20
 80035ce:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80035d2:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80035d4:	9a04      	ldr	r2, [sp, #16]
 80035d6:	4650      	mov	r0, sl
 80035d8:	68eb      	ldr	r3, [r5, #12]
 80035da:	4659      	mov	r1, fp
 80035dc:	4798      	blx	r3
 80035de:	b398      	cbz	r0, 8003648 <core::fmt::Formatter::pad_integral+0x158>
 80035e0:	2001      	movs	r0, #1
 80035e2:	b005      	add	sp, #20
 80035e4:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80035e8:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80035ea:	b299      	uxth	r1, r3
 80035ec:	ea4f 0851 	mov.w	r8, r1, lsr #1
 80035f0:	e9d0 9500 	ldrd	r9, r5, [r0]
 80035f4:	2400      	movs	r4, #0
 80035f6:	9302      	str	r3, [sp, #8]
 80035f8:	fa1f f088 	uxth.w	r0, r8
 80035fc:	b2a1      	uxth	r1, r4
 80035fe:	4281      	cmp	r1, r0
 8003600:	d20b      	bcs.n	800361a <core::fmt::Formatter::pad_integral+0x12a>
 8003602:	692a      	ldr	r2, [r5, #16]
 8003604:	4648      	mov	r0, r9
 8003606:	4631      	mov	r1, r6
 8003608:	4790      	blx	r2
 800360a:	3401      	adds	r4, #1
 800360c:	2800      	cmp	r0, #0
 800360e:	d0f3      	beq.n	80035f8 <core::fmt::Formatter::pad_integral+0x108>
 8003610:	2001      	movs	r0, #1
 8003612:	b005      	add	sp, #20
 8003614:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003618:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800361a:	9b04      	ldr	r3, [sp, #16]
 800361c:	4648      	mov	r0, r9
 800361e:	4629      	mov	r1, r5
 8003620:	465a      	mov	r2, fp
 8003622:	f000 f833 	bl	800368c <core::fmt::Formatter::pad_integral::write_prefix>
 8003626:	b120      	cbz	r0, 8003632 <core::fmt::Formatter::pad_integral+0x142>
 8003628:	2001      	movs	r0, #1
 800362a:	b005      	add	sp, #20
 800362c:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003630:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003632:	9903      	ldr	r1, [sp, #12]
 8003634:	4648      	mov	r0, r9
 8003636:	68eb      	ldr	r3, [r5, #12]
 8003638:	4652      	mov	r2, sl
 800363a:	4798      	blx	r3
 800363c:	b170      	cbz	r0, 800365c <core::fmt::Formatter::pad_integral+0x16c>
 800363e:	2001      	movs	r0, #1
 8003640:	b005      	add	sp, #20
 8003642:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003646:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003648:	9803      	ldr	r0, [sp, #12]
 800364a:	e9dd 1201 	ldrd	r1, r2, [sp, #4]
 800364e:	e9c0 1202 	strd	r1, r2, [r0, #8]
 8003652:	2000      	movs	r0, #0
 8003654:	b005      	add	sp, #20
 8003656:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800365a:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800365c:	9802      	ldr	r0, [sp, #8]
 800365e:	2400      	movs	r4, #0
 8003660:	eba0 0008 	sub.w	r0, r0, r8
 8003664:	fa1f f880 	uxth.w	r8, r0
 8003668:	b2a0      	uxth	r0, r4
 800366a:	4540      	cmp	r0, r8
 800366c:	d209      	bcs.n	8003682 <core::fmt::Formatter::pad_integral+0x192>
 800366e:	692a      	ldr	r2, [r5, #16]
 8003670:	4648      	mov	r0, r9
 8003672:	4631      	mov	r1, r6
 8003674:	4790      	blx	r2
 8003676:	3401      	adds	r4, #1
 8003678:	4601      	mov	r1, r0
 800367a:	2001      	movs	r0, #1
 800367c:	2900      	cmp	r1, #0
 800367e:	d0f3      	beq.n	8003668 <core::fmt::Formatter::pad_integral+0x178>
 8003680:	e769      	b.n	8003556 <core::fmt::Formatter::pad_integral+0x66>
 8003682:	2000      	movs	r0, #0
 8003684:	b005      	add	sp, #20
 8003686:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800368a:	bdf0      	pop	{r4, r5, r6, r7, pc}

0800368c <core::fmt::Formatter::pad_integral::write_prefix>:
 800368c:	b5f0      	push	{r4, r5, r6, r7, lr}
 800368e:	af03      	add	r7, sp, #12
 8003690:	f84d bd04 	str.w	fp, [sp, #-4]!
 8003694:	461c      	mov	r4, r3
 8003696:	460d      	mov	r5, r1
 8003698:	f5b2 1f88 	cmp.w	r2, #1114112	@ 0x110000
 800369c:	d00a      	beq.n	80036b4 <core::fmt::Formatter::pad_integral::write_prefix+0x28>
 800369e:	692b      	ldr	r3, [r5, #16]
 80036a0:	4611      	mov	r1, r2
 80036a2:	4606      	mov	r6, r0
 80036a4:	4798      	blx	r3
 80036a6:	4601      	mov	r1, r0
 80036a8:	4630      	mov	r0, r6
 80036aa:	b119      	cbz	r1, 80036b4 <core::fmt::Formatter::pad_integral::write_prefix+0x28>
 80036ac:	2001      	movs	r0, #1
 80036ae:	f85d bb04 	ldr.w	fp, [sp], #4
 80036b2:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80036b4:	b13c      	cbz	r4, 80036c6 <core::fmt::Formatter::pad_integral::write_prefix+0x3a>
 80036b6:	68eb      	ldr	r3, [r5, #12]
 80036b8:	4621      	mov	r1, r4
 80036ba:	2200      	movs	r2, #0
 80036bc:	f85d bb04 	ldr.w	fp, [sp], #4
 80036c0:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 80036c4:	4718      	bx	r3
 80036c6:	2000      	movs	r0, #0
 80036c8:	f85d bb04 	ldr.w	fp, [sp], #4
 80036cc:	bdf0      	pop	{r4, r5, r6, r7, pc}

080036ce <core::option::unwrap_failed>:
 80036ce:	b580      	push	{r7, lr}
 80036d0:	466f      	mov	r7, sp
 80036d2:	4602      	mov	r2, r0
 80036d4:	f24a 5048 	movw	r0, #42312	@ 0xa548
 80036d8:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80036dc:	212b      	movs	r1, #43	@ 0x2b
 80036de:	f000 f893 	bl	8003808 <core::panicking::panic>

080036e2 <core::slice::index::slice_index_fail>:
 80036e2:	b580      	push	{r7, lr}
 80036e4:	466f      	mov	r7, sp
 80036e6:	4290      	cmp	r0, r2
 80036e8:	d903      	bls.n	80036f2 <core::slice::index::slice_index_fail+0x10>
 80036ea:	4611      	mov	r1, r2
 80036ec:	461a      	mov	r2, r3
 80036ee:	f000 f811 	bl	8003714 <core::slice::index::slice_index_fail::do_panic::runtime>
 80036f2:	4291      	cmp	r1, r2
 80036f4:	d904      	bls.n	8003700 <core::slice::index::slice_index_fail+0x1e>
 80036f6:	4608      	mov	r0, r1
 80036f8:	4611      	mov	r1, r2
 80036fa:	461a      	mov	r2, r3
 80036fc:	f000 f820 	bl	8003740 <core::slice::index::slice_index_fail::do_panic::runtime>
 8003700:	4288      	cmp	r0, r1
 8003702:	d902      	bls.n	800370a <core::slice::index::slice_index_fail+0x28>
 8003704:	461a      	mov	r2, r3
 8003706:	f000 f847 	bl	8003798 <core::slice::index::slice_index_fail::do_panic::runtime>
 800370a:	4608      	mov	r0, r1
 800370c:	4611      	mov	r1, r2
 800370e:	461a      	mov	r2, r3
 8003710:	f000 f82c 	bl	800376c <core::slice::index::slice_index_fail::do_panic::runtime>

08003714 <core::slice::index::slice_index_fail::do_panic::runtime>:
 8003714:	b580      	push	{r7, lr}
 8003716:	466f      	mov	r7, sp
 8003718:	b086      	sub	sp, #24
 800371a:	e9cd 0100 	strd	r0, r1, [sp]
 800371e:	f643 60fb 	movw	r0, #16123	@ 0x3efb
 8003722:	a901      	add	r1, sp, #4
 8003724:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003728:	9005      	str	r0, [sp, #20]
 800372a:	e9cd 0103 	strd	r0, r1, [sp, #12]
 800372e:	4668      	mov	r0, sp
 8003730:	9002      	str	r0, [sp, #8]
 8003732:	f24a 00db 	movw	r0, #41179	@ 0xa0db
 8003736:	a902      	add	r1, sp, #8
 8003738:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800373c:	f000 f842 	bl	80037c4 <core::panicking::panic_fmt>

08003740 <core::slice::index::slice_index_fail::do_panic::runtime>:
 8003740:	b580      	push	{r7, lr}
 8003742:	466f      	mov	r7, sp
 8003744:	b086      	sub	sp, #24
 8003746:	e9cd 0100 	strd	r0, r1, [sp]
 800374a:	f643 60fb 	movw	r0, #16123	@ 0x3efb
 800374e:	a901      	add	r1, sp, #4
 8003750:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003754:	9005      	str	r0, [sp, #20]
 8003756:	e9cd 0103 	strd	r0, r1, [sp, #12]
 800375a:	4668      	mov	r0, sp
 800375c:	9002      	str	r0, [sp, #8]
 800375e:	f24a 0032 	movw	r0, #41010	@ 0xa032
 8003762:	a902      	add	r1, sp, #8
 8003764:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003768:	f000 f82c 	bl	80037c4 <core::panicking::panic_fmt>

0800376c <core::slice::index::slice_index_fail::do_panic::runtime>:
 800376c:	b580      	push	{r7, lr}
 800376e:	466f      	mov	r7, sp
 8003770:	b086      	sub	sp, #24
 8003772:	e9cd 0100 	strd	r0, r1, [sp]
 8003776:	f643 60fb 	movw	r0, #16123	@ 0x3efb
 800377a:	a901      	add	r1, sp, #4
 800377c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003780:	9005      	str	r0, [sp, #20]
 8003782:	e9cd 0103 	strd	r0, r1, [sp, #12]
 8003786:	4668      	mov	r0, sp
 8003788:	9002      	str	r0, [sp, #8]
 800378a:	f24a 0032 	movw	r0, #41010	@ 0xa032
 800378e:	a902      	add	r1, sp, #8
 8003790:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003794:	f000 f816 	bl	80037c4 <core::panicking::panic_fmt>

08003798 <core::slice::index::slice_index_fail::do_panic::runtime>:
 8003798:	b580      	push	{r7, lr}
 800379a:	466f      	mov	r7, sp
 800379c:	b086      	sub	sp, #24
 800379e:	e9cd 0100 	strd	r0, r1, [sp]
 80037a2:	f643 60fb 	movw	r0, #16123	@ 0x3efb
 80037a6:	a901      	add	r1, sp, #4
 80037a8:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80037ac:	9005      	str	r0, [sp, #20]
 80037ae:	e9cd 0103 	strd	r0, r1, [sp, #12]
 80037b2:	4668      	mov	r0, sp
 80037b4:	9002      	str	r0, [sp, #8]
 80037b6:	f649 5098 	movw	r0, #40344	@ 0x9d98
 80037ba:	a902      	add	r1, sp, #8
 80037bc:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80037c0:	f000 f800 	bl	80037c4 <core::panicking::panic_fmt>

080037c4 <core::panicking::panic_fmt>:
 80037c4:	b580      	push	{r7, lr}
 80037c6:	466f      	mov	r7, sp
 80037c8:	b086      	sub	sp, #24
 80037ca:	e9cd 0101 	strd	r0, r1, [sp, #4]
 80037ce:	2001      	movs	r0, #1
 80037d0:	f8ad 0014 	strh.w	r0, [sp, #20]
 80037d4:	a801      	add	r0, sp, #4
 80037d6:	9003      	str	r0, [sp, #12]
 80037d8:	a803      	add	r0, sp, #12
 80037da:	9204      	str	r2, [sp, #16]
 80037dc:	f002 fbfc 	bl	8005fd8 <__rustc::rust_begin_unwind>

080037e0 <core::panicking::panic_bounds_check>:
 80037e0:	b5fe      	push	{r1, r2, r3, r4, r5, r6, r7, lr}
 80037e2:	af06      	add	r7, sp, #24
 80037e4:	e9cd 0100 	strd	r0, r1, [sp]
 80037e8:	4669      	mov	r1, sp
 80037ea:	4805      	ldr	r0, [pc, #20]	@ (8003800 <core::panicking::panic_bounds_check+0x20>)
 80037ec:	9005      	str	r0, [sp, #20]
 80037ee:	e9cd 0103 	strd	r0, r1, [sp, #12]
 80037f2:	a801      	add	r0, sp, #4
 80037f4:	9002      	str	r0, [sp, #8]
 80037f6:	a902      	add	r1, sp, #8
 80037f8:	4802      	ldr	r0, [pc, #8]	@ (8003804 <core::panicking::panic_bounds_check+0x24>)
 80037fa:	f7ff ffe3 	bl	80037c4 <core::panicking::panic_fmt>
 80037fe:	bf00      	nop
 8003800:	08003efb 	.word	0x08003efb
 8003804:	08009ee4 	.word	0x08009ee4

08003808 <core::panicking::panic>:
 8003808:	b580      	push	{r7, lr}
 800380a:	466f      	mov	r7, sp
 800380c:	0049      	lsls	r1, r1, #1
 800380e:	3101      	adds	r1, #1
 8003810:	f7ff ffd8 	bl	80037c4 <core::panicking::panic_fmt>

08003814 <<&T as core::fmt::Display>::fmt>:
 8003814:	b580      	push	{r7, lr}
 8003816:	466f      	mov	r7, sp
 8003818:	460b      	mov	r3, r1
 800381a:	e9d0 1200 	ldrd	r1, r2, [r0]
 800381e:	4618      	mov	r0, r3
 8003820:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8003824:	f000 b800 	b.w	8003828 <core::fmt::Formatter::pad>

08003828 <core::fmt::Formatter::pad>:
 8003828:	b5f0      	push	{r4, r5, r6, r7, lr}
 800382a:	af03      	add	r7, sp, #12
 800382c:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8003830:	b08d      	sub	sp, #52	@ 0x34
 8003832:	f8d0 b008 	ldr.w	fp, [r0, #8]
 8003836:	4690      	mov	r8, r2
 8003838:	460c      	mov	r4, r1
 800383a:	f01b 5fc0 	tst.w	fp, #402653184	@ 0x18000000
 800383e:	f000 8311 	beq.w	8003e64 <core::fmt::Formatter::pad+0x63c>
 8003842:	ea5f 01cb 	movs.w	r1, fp, lsl #3
 8003846:	d43c      	bmi.n	80038c2 <core::fmt::Formatter::pad+0x9a>
 8003848:	f1b8 0f10 	cmp.w	r8, #16
 800384c:	d25a      	bcs.n	8003904 <core::fmt::Formatter::pad+0xdc>
 800384e:	f1b8 0f00 	cmp.w	r8, #0
 8003852:	d079      	beq.n	8003948 <core::fmt::Formatter::pad+0x120>
 8003854:	f008 0c03 	and.w	ip, r8, #3
 8003858:	4681      	mov	r9, r0
 800385a:	ea5f 0198 	movs.w	r1, r8, lsr #2
 800385e:	f04f 0200 	mov.w	r2, #0
 8003862:	f000 82df 	beq.w	8003e24 <core::fmt::Formatter::pad+0x5fc>
 8003866:	f001 0103 	and.w	r1, r1, #3
 800386a:	46a6      	mov	lr, r4
 800386c:	f06f 0603 	mvn.w	r6, #3
 8003870:	2300      	movs	r3, #0
 8003872:	eba2 0a81 	sub.w	sl, r2, r1, lsl #2
 8003876:	1c62      	adds	r2, r4, #1
 8003878:	1991      	adds	r1, r2, r6
 800387a:	3604      	adds	r6, #4
 800387c:	f991 4003 	ldrsb.w	r4, [r1, #3]
 8003880:	f991 5006 	ldrsb.w	r5, [r1, #6]
 8003884:	f991 0005 	ldrsb.w	r0, [r1, #5]
 8003888:	f114 0f41 	cmn.w	r4, #65	@ 0x41
 800388c:	f991 1004 	ldrsb.w	r1, [r1, #4]
 8003890:	bfc8      	it	gt
 8003892:	3301      	addgt	r3, #1
 8003894:	f111 0f41 	cmn.w	r1, #65	@ 0x41
 8003898:	bfc8      	it	gt
 800389a:	3301      	addgt	r3, #1
 800389c:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 80038a0:	bfc8      	it	gt
 80038a2:	3301      	addgt	r3, #1
 80038a4:	f115 0f41 	cmn.w	r5, #65	@ 0x41
 80038a8:	eb0a 0006 	add.w	r0, sl, r6
 80038ac:	bfc8      	it	gt
 80038ae:	3301      	addgt	r3, #1
 80038b0:	3004      	adds	r0, #4
 80038b2:	d1e1      	bne.n	8003878 <core::fmt::Formatter::pad+0x50>
 80038b4:	1d32      	adds	r2, r6, #4
 80038b6:	4674      	mov	r4, lr
 80038b8:	f1bc 0f00 	cmp.w	ip, #0
 80038bc:	f040 82b6 	bne.w	8003e2c <core::fmt::Formatter::pad+0x604>
 80038c0:	e2cc      	b.n	8003e5c <core::fmt::Formatter::pad+0x634>
 80038c2:	f8b0 c00e 	ldrh.w	ip, [r0, #14]
 80038c6:	f1bc 0f00 	cmp.w	ip, #0
 80038ca:	d02c      	beq.n	8003926 <core::fmt::Formatter::pad+0xfe>
 80038cc:	eb04 0208 	add.w	r2, r4, r8
 80038d0:	f04f 0800 	mov.w	r8, #0
 80038d4:	4626      	mov	r6, r4
 80038d6:	4661      	mov	r1, ip
 80038d8:	e007      	b.n	80038ea <core::fmt::Formatter::pad+0xc2>
 80038da:	2ef0      	cmp	r6, #240	@ 0xf0
 80038dc:	bf2c      	ite	cs
 80038de:	1d1e      	addcs	r6, r3, #4
 80038e0:	1cde      	addcc	r6, r3, #3
 80038e2:	1af3      	subs	r3, r6, r3
 80038e4:	3901      	subs	r1, #1
 80038e6:	4498      	add	r8, r3
 80038e8:	d01f      	beq.n	800392a <core::fmt::Formatter::pad+0x102>
 80038ea:	4296      	cmp	r6, r2
 80038ec:	d01e      	beq.n	800392c <core::fmt::Formatter::pad+0x104>
 80038ee:	4633      	mov	r3, r6
 80038f0:	f916 5b01 	ldrsb.w	r5, [r6], #1
 80038f4:	f1b5 3fff 	cmp.w	r5, #4294967295	@ 0xffffffff
 80038f8:	dcf3      	bgt.n	80038e2 <core::fmt::Formatter::pad+0xba>
 80038fa:	b2ee      	uxtb	r6, r5
 80038fc:	2ee0      	cmp	r6, #224	@ 0xe0
 80038fe:	d2ec      	bcs.n	80038da <core::fmt::Formatter::pad+0xb2>
 8003900:	1c9e      	adds	r6, r3, #2
 8003902:	e7ee      	b.n	80038e2 <core::fmt::Formatter::pad+0xba>
 8003904:	9000      	str	r0, [sp, #0]
 8003906:	1ce0      	adds	r0, r4, #3
 8003908:	4622      	mov	r2, r4
 800390a:	f020 0403 	bic.w	r4, r0, #3
 800390e:	ebb4 0e02 	subs.w	lr, r4, r2
 8003912:	f8cd 800c 	str.w	r8, [sp, #12]
 8003916:	eba8 080e 	sub.w	r8, r8, lr
 800391a:	9201      	str	r2, [sp, #4]
 800391c:	f008 0c03 	and.w	ip, r8, #3
 8003920:	d10b      	bne.n	800393a <core::fmt::Formatter::pad+0x112>
 8003922:	2100      	movs	r1, #0
 8003924:	e057      	b.n	80039d6 <core::fmt::Formatter::pad+0x1ae>
 8003926:	f04f 0800 	mov.w	r8, #0
 800392a:	2100      	movs	r1, #0
 800392c:	ebac 0301 	sub.w	r3, ip, r1
 8003930:	8981      	ldrh	r1, [r0, #12]
 8003932:	428b      	cmp	r3, r1
 8003934:	f0c0 8232 	bcc.w	8003d9c <core::fmt::Formatter::pad+0x574>
 8003938:	e294      	b.n	8003e64 <core::fmt::Formatter::pad+0x63c>
 800393a:	1b11      	subs	r1, r2, r4
 800393c:	f111 0f04 	cmn.w	r1, #4
 8003940:	d90a      	bls.n	8003958 <core::fmt::Formatter::pad+0x130>
 8003942:	2200      	movs	r2, #0
 8003944:	2100      	movs	r1, #0
 8003946:	e02c      	b.n	80039a2 <core::fmt::Formatter::pad+0x17a>
 8003948:	2300      	movs	r3, #0
 800394a:	f04f 0800 	mov.w	r8, #0
 800394e:	8981      	ldrh	r1, [r0, #12]
 8003950:	428b      	cmp	r3, r1
 8003952:	f0c0 8223 	bcc.w	8003d9c <core::fmt::Formatter::pad+0x574>
 8003956:	e285      	b.n	8003e64 <core::fmt::Formatter::pad+0x63c>
 8003958:	f102 0901 	add.w	r9, r2, #1
 800395c:	2100      	movs	r1, #0
 800395e:	f06f 0503 	mvn.w	r5, #3
 8003962:	eb09 0205 	add.w	r2, r9, r5
 8003966:	f992 6003 	ldrsb.w	r6, [r2, #3]
 800396a:	f992 0006 	ldrsb.w	r0, [r2, #6]
 800396e:	f992 3005 	ldrsb.w	r3, [r2, #5]
 8003972:	f116 0f41 	cmn.w	r6, #65	@ 0x41
 8003976:	f992 2004 	ldrsb.w	r2, [r2, #4]
 800397a:	bfc8      	it	gt
 800397c:	3101      	addgt	r1, #1
 800397e:	f112 0f41 	cmn.w	r2, #65	@ 0x41
 8003982:	bfc8      	it	gt
 8003984:	3101      	addgt	r1, #1
 8003986:	f113 0f41 	cmn.w	r3, #65	@ 0x41
 800398a:	bfc8      	it	gt
 800398c:	3101      	addgt	r1, #1
 800398e:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003992:	f105 0004 	add.w	r0, r5, #4
 8003996:	bfc8      	it	gt
 8003998:	3101      	addgt	r1, #1
 800399a:	f115 0208 	adds.w	r2, r5, #8
 800399e:	4605      	mov	r5, r0
 80039a0:	d1df      	bne.n	8003962 <core::fmt::Formatter::pad+0x13a>
 80039a2:	9801      	ldr	r0, [sp, #4]
 80039a4:	5680      	ldrsb	r0, [r0, r2]
 80039a6:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 80039aa:	bfc8      	it	gt
 80039ac:	3101      	addgt	r1, #1
 80039ae:	f1be 0f01 	cmp.w	lr, #1
 80039b2:	d010      	beq.n	80039d6 <core::fmt::Formatter::pad+0x1ae>
 80039b4:	9801      	ldr	r0, [sp, #4]
 80039b6:	4402      	add	r2, r0
 80039b8:	f992 0001 	ldrsb.w	r0, [r2, #1]
 80039bc:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 80039c0:	bfc8      	it	gt
 80039c2:	3101      	addgt	r1, #1
 80039c4:	f1be 0f02 	cmp.w	lr, #2
 80039c8:	d005      	beq.n	80039d6 <core::fmt::Formatter::pad+0x1ae>
 80039ca:	f992 0002 	ldrsb.w	r0, [r2, #2]
 80039ce:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 80039d2:	bfc8      	it	gt
 80039d4:	3101      	addgt	r1, #1
 80039d6:	f64f 72fc 	movw	r2, #65532	@ 0xfffc
 80039da:	ea4f 0e98 	mov.w	lr, r8, lsr #2
 80039de:	f6c1 72ff 	movt	r2, #8191	@ 0x1fff
 80039e2:	2000      	movs	r0, #0
 80039e4:	f1bc 0f00 	cmp.w	ip, #0
 80039e8:	d01f      	beq.n	8003a2a <core::fmt::Formatter::pad+0x202>
 80039ea:	f102 42c0 	add.w	r2, r2, #1610612736	@ 0x60000000
 80039ee:	ea02 0208 	and.w	r2, r2, r8
 80039f2:	4422      	add	r2, r4
 80039f4:	f992 3000 	ldrsb.w	r3, [r2]
 80039f8:	f113 0f41 	cmn.w	r3, #65	@ 0x41
 80039fc:	bfc8      	it	gt
 80039fe:	2001      	movgt	r0, #1
 8003a00:	f8dd 800c 	ldr.w	r8, [sp, #12]
 8003a04:	f1bc 0f01 	cmp.w	ip, #1
 8003a08:	d011      	beq.n	8003a2e <core::fmt::Formatter::pad+0x206>
 8003a0a:	f992 3001 	ldrsb.w	r3, [r2, #1]
 8003a0e:	f113 0f41 	cmn.w	r3, #65	@ 0x41
 8003a12:	bfc8      	it	gt
 8003a14:	3001      	addgt	r0, #1
 8003a16:	f1bc 0f02 	cmp.w	ip, #2
 8003a1a:	d008      	beq.n	8003a2e <core::fmt::Formatter::pad+0x206>
 8003a1c:	f992 2002 	ldrsb.w	r2, [r2, #2]
 8003a20:	f112 0f41 	cmn.w	r2, #65	@ 0x41
 8003a24:	bfc8      	it	gt
 8003a26:	3001      	addgt	r0, #1
 8003a28:	e001      	b.n	8003a2e <core::fmt::Formatter::pad+0x206>
 8003a2a:	f8dd 800c 	ldr.w	r8, [sp, #12]
 8003a2e:	1843      	adds	r3, r0, r1
 8003a30:	f8cd b008 	str.w	fp, [sp, #8]
 8003a34:	e014      	b.n	8003a60 <core::fmt::Formatter::pad+0x238>
 8003a36:	f04f 0900 	mov.w	r9, #0
 8003a3a:	fa3f f189 	uxtb16	r1, r9
 8003a3e:	fa3f f299 	uxtb16	r2, r9, ror #8
 8003a42:	4411      	add	r1, r2
 8003a44:	9d07      	ldr	r5, [sp, #28]
 8003a46:	9b08      	ldr	r3, [sp, #32]
 8003a48:	eb01 4101 	add.w	r1, r1, r1, lsl #16
 8003a4c:	ebae 0e05 	sub.w	lr, lr, r5
 8003a50:	eb0c 0485 	add.w	r4, ip, r5, lsl #2
 8003a54:	f015 0003 	ands.w	r0, r5, #3
 8003a58:	eb03 4311 	add.w	r3, r3, r1, lsr #16
 8003a5c:	f040 816c 	bne.w	8003d38 <core::fmt::Formatter::pad+0x510>
 8003a60:	f1be 0f00 	cmp.w	lr, #0
 8003a64:	f000 8166 	beq.w	8003d34 <core::fmt::Formatter::pad+0x50c>
 8003a68:	f1be 0fc0 	cmp.w	lr, #192	@ 0xc0
 8003a6c:	4671      	mov	r1, lr
 8003a6e:	f44f 707c 	mov.w	r0, #1008	@ 0x3f0
 8003a72:	9308      	str	r3, [sp, #32]
 8003a74:	bf28      	it	cs
 8003a76:	21c0      	movcs	r1, #192	@ 0xc0
 8003a78:	ea10 0081 	ands.w	r0, r0, r1, lsl #2
 8003a7c:	46a4      	mov	ip, r4
 8003a7e:	9107      	str	r1, [sp, #28]
 8003a80:	d0d9      	beq.n	8003a36 <core::fmt::Formatter::pad+0x20e>
 8003a82:	eb0c 0400 	add.w	r4, ip, r0
 8003a86:	0088      	lsls	r0, r1, #2
 8003a88:	3810      	subs	r0, #16
 8003a8a:	2101      	movs	r1, #1
 8003a8c:	f10c 0a10 	add.w	sl, ip, #16
 8003a90:	f8cd e018 	str.w	lr, [sp, #24]
 8003a94:	eb01 1110 	add.w	r1, r1, r0, lsr #4
 8003a98:	2830      	cmp	r0, #48	@ 0x30
 8003a9a:	e9cd 1c04 	strd	r1, ip, [sp, #16]
 8003a9e:	d203      	bcs.n	8003aa8 <core::fmt::Formatter::pad+0x280>
 8003aa0:	f04f 0900 	mov.w	r9, #0
 8003aa4:	4663      	mov	r3, ip
 8003aa6:	e0c5      	b.n	8003c34 <core::fmt::Formatter::pad+0x40c>
 8003aa8:	f64f 70fc 	movw	r0, #65532	@ 0xfffc
 8003aac:	f04f 0900 	mov.w	r9, #0
 8003ab0:	f6c1 70ff 	movt	r0, #8191	@ 0x1fff
 8003ab4:	ea01 0e00 	and.w	lr, r1, r0
 8003ab8:	46d0      	mov	r8, sl
 8003aba:	4666      	mov	r6, ip
 8003abc:	9409      	str	r4, [sp, #36]	@ 0x24
 8003abe:	4645      	mov	r5, r8
 8003ac0:	45a0      	cmp	r8, r4
 8003ac2:	bf18      	it	ne
 8003ac4:	3510      	addne	r5, #16
 8003ac6:	462a      	mov	r2, r5
 8003ac8:	42a5      	cmp	r5, r4
 8003aca:	bf18      	it	ne
 8003acc:	3210      	addne	r2, #16
 8003ace:	4613      	mov	r3, r2
 8003ad0:	42a2      	cmp	r2, r4
 8003ad2:	bf18      	it	ne
 8003ad4:	3310      	addne	r3, #16
 8003ad6:	469a      	mov	sl, r3
 8003ad8:	42a3      	cmp	r3, r4
 8003ada:	bf18      	it	ne
 8003adc:	f10a 0a10 	addne.w	sl, sl, #16
 8003ae0:	e9d2 4c00 	ldrd	r4, ip, [r2]
 8003ae4:	f1be 0e04 	subs.w	lr, lr, #4
 8003ae8:	6890      	ldr	r0, [r2, #8]
 8003aea:	900b      	str	r0, [sp, #44]	@ 0x2c
 8003aec:	68d0      	ldr	r0, [r2, #12]
 8003aee:	ea6f 0204 	mvn.w	r2, r4
 8003af2:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003af6:	900c      	str	r0, [sp, #48]	@ 0x30
 8003af8:	ea42 1294 	orr.w	r2, r2, r4, lsr #6
 8003afc:	f022 30fe 	bic.w	r0, r2, #4278124286	@ 0xfefefefe
 8003b00:	e9d6 2400 	ldrd	r2, r4, [r6]
 8003b04:	900a      	str	r0, [sp, #40]	@ 0x28
 8003b06:	e9d6 0602 	ldrd	r0, r6, [r6, #8]
 8003b0a:	ea6f 0b02 	mvn.w	fp, r2
 8003b0e:	ea4f 11db 	mov.w	r1, fp, lsr #7
 8003b12:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003b16:	ea6f 0204 	mvn.w	r2, r4
 8003b1a:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003b1e:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003b22:	ea42 1294 	orr.w	r2, r2, r4, lsr #6
 8003b26:	4449      	add	r1, r9
 8003b28:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003b2c:	4411      	add	r1, r2
 8003b2e:	ea6f 0200 	mvn.w	r2, r0
 8003b32:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003b36:	ea42 1090 	orr.w	r0, r2, r0, lsr #6
 8003b3a:	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
 8003b3e:	4408      	add	r0, r1
 8003b40:	ea6f 0106 	mvn.w	r1, r6
 8003b44:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003b48:	ea41 1196 	orr.w	r1, r1, r6, lsr #6
 8003b4c:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003b50:	eb01 0900 	add.w	r9, r1, r0
 8003b54:	e898 0056 	ldmia.w	r8, {r1, r2, r4, r6}
 8003b58:	46d0      	mov	r8, sl
 8003b5a:	ea6f 0001 	mvn.w	r0, r1
 8003b5e:	ea4f 10d0 	mov.w	r0, r0, lsr #7
 8003b62:	ea40 1091 	orr.w	r0, r0, r1, lsr #6
 8003b66:	ea6f 0102 	mvn.w	r1, r2
 8003b6a:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003b6e:	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
 8003b72:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003b76:	4448      	add	r0, r9
 8003b78:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003b7c:	4408      	add	r0, r1
 8003b7e:	ea6f 0104 	mvn.w	r1, r4
 8003b82:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003b86:	ea41 1194 	orr.w	r1, r1, r4, lsr #6
 8003b8a:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003b8e:	4408      	add	r0, r1
 8003b90:	ea6f 0106 	mvn.w	r1, r6
 8003b94:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003b98:	ea41 1196 	orr.w	r1, r1, r6, lsr #6
 8003b9c:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003ba0:	4408      	add	r0, r1
 8003ba2:	cd36      	ldmia	r5, {r1, r2, r4, r5}
 8003ba4:	ea6f 0601 	mvn.w	r6, r1
 8003ba8:	ea4f 16d6 	mov.w	r6, r6, lsr #7
 8003bac:	ea46 1191 	orr.w	r1, r6, r1, lsr #6
 8003bb0:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003bb4:	461e      	mov	r6, r3
 8003bb6:	4408      	add	r0, r1
 8003bb8:	ea6f 0102 	mvn.w	r1, r2
 8003bbc:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003bc0:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003bc4:	9a0b      	ldr	r2, [sp, #44]	@ 0x2c
 8003bc6:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003bca:	4408      	add	r0, r1
 8003bcc:	ea6f 0104 	mvn.w	r1, r4
 8003bd0:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003bd4:	ea41 1194 	orr.w	r1, r1, r4, lsr #6
 8003bd8:	9c09      	ldr	r4, [sp, #36]	@ 0x24
 8003bda:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003bde:	4408      	add	r0, r1
 8003be0:	ea6f 0105 	mvn.w	r1, r5
 8003be4:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003be8:	ea41 1195 	orr.w	r1, r1, r5, lsr #6
 8003bec:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003bf0:	4408      	add	r0, r1
 8003bf2:	990a      	ldr	r1, [sp, #40]	@ 0x28
 8003bf4:	4408      	add	r0, r1
 8003bf6:	ea6f 010c 	mvn.w	r1, ip
 8003bfa:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003bfe:	ea41 119c 	orr.w	r1, r1, ip, lsr #6
 8003c02:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003c06:	4408      	add	r0, r1
 8003c08:	ea6f 0102 	mvn.w	r1, r2
 8003c0c:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003c10:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003c14:	9a0c      	ldr	r2, [sp, #48]	@ 0x30
 8003c16:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003c1a:	4408      	add	r0, r1
 8003c1c:	ea6f 0102 	mvn.w	r1, r2
 8003c20:	ea4f 11d1 	mov.w	r1, r1, lsr #7
 8003c24:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003c28:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003c2c:	eb01 0900 	add.w	r9, r1, r0
 8003c30:	f47f af45 	bne.w	8003abe <core::fmt::Formatter::pad+0x296>
 8003c34:	9804      	ldr	r0, [sp, #16]
 8003c36:	e9dd b802 	ldrd	fp, r8, [sp, #8]
 8003c3a:	e9dd ce05 	ldrd	ip, lr, [sp, #20]
 8003c3e:	f010 0003 	ands.w	r0, r0, #3
 8003c42:	f43f aefa 	beq.w	8003a3a <core::fmt::Formatter::pad+0x212>
 8003c46:	e893 0046 	ldmia.w	r3, {r1, r2, r6}
 8003c4a:	45a2      	cmp	sl, r4
 8003c4c:	68db      	ldr	r3, [r3, #12]
 8003c4e:	ea6f 0501 	mvn.w	r5, r1
 8003c52:	ea4f 15d5 	mov.w	r5, r5, lsr #7
 8003c56:	ea45 1191 	orr.w	r1, r5, r1, lsr #6
 8003c5a:	ea6f 0502 	mvn.w	r5, r2
 8003c5e:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003c62:	ea4f 15d5 	mov.w	r5, r5, lsr #7
 8003c66:	ea45 1292 	orr.w	r2, r5, r2, lsr #6
 8003c6a:	4449      	add	r1, r9
 8003c6c:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003c70:	4411      	add	r1, r2
 8003c72:	ea6f 0206 	mvn.w	r2, r6
 8003c76:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003c7a:	ea42 1296 	orr.w	r2, r2, r6, lsr #6
 8003c7e:	4656      	mov	r6, sl
 8003c80:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003c84:	bf18      	it	ne
 8003c86:	3610      	addne	r6, #16
 8003c88:	4411      	add	r1, r2
 8003c8a:	43da      	mvns	r2, r3
 8003c8c:	09d2      	lsrs	r2, r2, #7
 8003c8e:	2801      	cmp	r0, #1
 8003c90:	ea42 1293 	orr.w	r2, r2, r3, lsr #6
 8003c94:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003c98:	eb02 0901 	add.w	r9, r2, r1
 8003c9c:	f43f aecd 	beq.w	8003a3a <core::fmt::Formatter::pad+0x212>
 8003ca0:	e89a 002e 	ldmia.w	sl, {r1, r2, r3, r5}
 8003ca4:	2802      	cmp	r0, #2
 8003ca6:	ea6f 0401 	mvn.w	r4, r1
 8003caa:	ea4f 14d4 	mov.w	r4, r4, lsr #7
 8003cae:	ea44 1191 	orr.w	r1, r4, r1, lsr #6
 8003cb2:	ea6f 0402 	mvn.w	r4, r2
 8003cb6:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003cba:	ea4f 14d4 	mov.w	r4, r4, lsr #7
 8003cbe:	ea44 1292 	orr.w	r2, r4, r2, lsr #6
 8003cc2:	4449      	add	r1, r9
 8003cc4:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003cc8:	4411      	add	r1, r2
 8003cca:	ea6f 0203 	mvn.w	r2, r3
 8003cce:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003cd2:	ea42 1293 	orr.w	r2, r2, r3, lsr #6
 8003cd6:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003cda:	4411      	add	r1, r2
 8003cdc:	ea6f 0205 	mvn.w	r2, r5
 8003ce0:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003ce4:	ea42 1295 	orr.w	r2, r2, r5, lsr #6
 8003ce8:	f022 32fe 	bic.w	r2, r2, #4278124286	@ 0xfefefefe
 8003cec:	eb02 0901 	add.w	r9, r2, r1
 8003cf0:	f43f aea3 	beq.w	8003a3a <core::fmt::Formatter::pad+0x212>
 8003cf4:	e896 000f 	ldmia.w	r6, {r0, r1, r2, r3}
 8003cf8:	43c6      	mvns	r6, r0
 8003cfa:	09f6      	lsrs	r6, r6, #7
 8003cfc:	ea46 1090 	orr.w	r0, r6, r0, lsr #6
 8003d00:	43ce      	mvns	r6, r1
 8003d02:	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
 8003d06:	09f6      	lsrs	r6, r6, #7
 8003d08:	ea46 1191 	orr.w	r1, r6, r1, lsr #6
 8003d0c:	4448      	add	r0, r9
 8003d0e:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003d12:	4408      	add	r0, r1
 8003d14:	43d1      	mvns	r1, r2
 8003d16:	09c9      	lsrs	r1, r1, #7
 8003d18:	ea41 1192 	orr.w	r1, r1, r2, lsr #6
 8003d1c:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003d20:	4408      	add	r0, r1
 8003d22:	43d9      	mvns	r1, r3
 8003d24:	09c9      	lsrs	r1, r1, #7
 8003d26:	ea41 1193 	orr.w	r1, r1, r3, lsr #6
 8003d2a:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003d2e:	eb01 0900 	add.w	r9, r1, r0
 8003d32:	e682      	b.n	8003a3a <core::fmt::Formatter::pad+0x212>
 8003d34:	9c01      	ldr	r4, [sp, #4]
 8003d36:	e02d      	b.n	8003d94 <core::fmt::Formatter::pad+0x56c>
 8003d38:	f005 06fc 	and.w	r6, r5, #252	@ 0xfc
 8003d3c:	9c01      	ldr	r4, [sp, #4]
 8003d3e:	2801      	cmp	r0, #1
 8003d40:	f85c 1026 	ldr.w	r1, [ip, r6, lsl #2]
 8003d44:	ea6f 0201 	mvn.w	r2, r1
 8003d48:	ea4f 12d2 	mov.w	r2, r2, lsr #7
 8003d4c:	ea42 1191 	orr.w	r1, r2, r1, lsr #6
 8003d50:	f021 32fe 	bic.w	r2, r1, #4278124286	@ 0xfefefefe
 8003d54:	d015      	beq.n	8003d82 <core::fmt::Formatter::pad+0x55a>
 8003d56:	eb0c 0586 	add.w	r5, ip, r6, lsl #2
 8003d5a:	2802      	cmp	r0, #2
 8003d5c:	6869      	ldr	r1, [r5, #4]
 8003d5e:	ea6f 0601 	mvn.w	r6, r1
 8003d62:	ea4f 16d6 	mov.w	r6, r6, lsr #7
 8003d66:	ea46 1191 	orr.w	r1, r6, r1, lsr #6
 8003d6a:	f021 31fe 	bic.w	r1, r1, #4278124286	@ 0xfefefefe
 8003d6e:	440a      	add	r2, r1
 8003d70:	d007      	beq.n	8003d82 <core::fmt::Formatter::pad+0x55a>
 8003d72:	68a8      	ldr	r0, [r5, #8]
 8003d74:	43c1      	mvns	r1, r0
 8003d76:	09c9      	lsrs	r1, r1, #7
 8003d78:	ea41 1090 	orr.w	r0, r1, r0, lsr #6
 8003d7c:	f020 30fe 	bic.w	r0, r0, #4278124286	@ 0xfefefefe
 8003d80:	4402      	add	r2, r0
 8003d82:	fa3f f082 	uxtb16	r0, r2
 8003d86:	fa3f f192 	uxtb16	r1, r2, ror #8
 8003d8a:	4408      	add	r0, r1
 8003d8c:	eb00 4000 	add.w	r0, r0, r0, lsl #16
 8003d90:	eb03 4310 	add.w	r3, r3, r0, lsr #16
 8003d94:	9800      	ldr	r0, [sp, #0]
 8003d96:	8981      	ldrh	r1, [r0, #12]
 8003d98:	428b      	cmp	r3, r1
 8003d9a:	d263      	bcs.n	8003e64 <core::fmt::Formatter::pad+0x63c>
 8003d9c:	f3cb 7241 	ubfx	r2, fp, #29, #2
 8003da0:	f8cd 800c 	str.w	r8, [sp, #12]
 8003da4:	eba1 0803 	sub.w	r8, r1, r3
 8003da8:	f36f 5b5f 	bfc	fp, #21, #11
 8003dac:	f04f 0900 	mov.w	r9, #0
 8003db0:	46a2      	mov	sl, r4
 8003db2:	e8df f002 	tbb	[pc, r2]
 8003db6:	0208      	.short	0x0208
 8003db8:	0804      	.short	0x0804
 8003dba:	46c1      	mov	r9, r8
 8003dbc:	e003      	b.n	8003dc6 <core::fmt::Formatter::pad+0x59e>
 8003dbe:	fa1f f188 	uxth.w	r1, r8
 8003dc2:	ea4f 0951 	mov.w	r9, r1, lsr #1
 8003dc6:	e9d0 5600 	ldrd	r5, r6, [r0]
 8003dca:	2400      	movs	r4, #0
 8003dcc:	fa1f f089 	uxth.w	r0, r9
 8003dd0:	b2a1      	uxth	r1, r4
 8003dd2:	4281      	cmp	r1, r0
 8003dd4:	d207      	bcs.n	8003de6 <core::fmt::Formatter::pad+0x5be>
 8003dd6:	6932      	ldr	r2, [r6, #16]
 8003dd8:	4628      	mov	r0, r5
 8003dda:	4659      	mov	r1, fp
 8003ddc:	4790      	blx	r2
 8003dde:	3401      	adds	r4, #1
 8003de0:	2800      	cmp	r0, #0
 8003de2:	d0f3      	beq.n	8003dcc <core::fmt::Formatter::pad+0x5a4>
 8003de4:	e014      	b.n	8003e10 <core::fmt::Formatter::pad+0x5e8>
 8003de6:	9a03      	ldr	r2, [sp, #12]
 8003de8:	4628      	mov	r0, r5
 8003dea:	68f3      	ldr	r3, [r6, #12]
 8003dec:	4651      	mov	r1, sl
 8003dee:	4798      	blx	r3
 8003df0:	b970      	cbnz	r0, 8003e10 <core::fmt::Formatter::pad+0x5e8>
 8003df2:	eba8 0009 	sub.w	r0, r8, r9
 8003df6:	2400      	movs	r4, #0
 8003df8:	fa1f f880 	uxth.w	r8, r0
 8003dfc:	b2a0      	uxth	r0, r4
 8003dfe:	4540      	cmp	r0, r8
 8003e00:	d20b      	bcs.n	8003e1a <core::fmt::Formatter::pad+0x5f2>
 8003e02:	6932      	ldr	r2, [r6, #16]
 8003e04:	4628      	mov	r0, r5
 8003e06:	4659      	mov	r1, fp
 8003e08:	4790      	blx	r2
 8003e0a:	3401      	adds	r4, #1
 8003e0c:	2800      	cmp	r0, #0
 8003e0e:	d0f5      	beq.n	8003dfc <core::fmt::Formatter::pad+0x5d4>
 8003e10:	2001      	movs	r0, #1
 8003e12:	b00d      	add	sp, #52	@ 0x34
 8003e14:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003e18:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003e1a:	2000      	movs	r0, #0
 8003e1c:	b00d      	add	sp, #52	@ 0x34
 8003e1e:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003e22:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8003e24:	2300      	movs	r3, #0
 8003e26:	f1bc 0f00 	cmp.w	ip, #0
 8003e2a:	d017      	beq.n	8003e5c <core::fmt::Formatter::pad+0x634>
 8003e2c:	56a0      	ldrsb	r0, [r4, r2]
 8003e2e:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003e32:	bfc8      	it	gt
 8003e34:	3301      	addgt	r3, #1
 8003e36:	f1bc 0f01 	cmp.w	ip, #1
 8003e3a:	d00f      	beq.n	8003e5c <core::fmt::Formatter::pad+0x634>
 8003e3c:	4422      	add	r2, r4
 8003e3e:	f992 0001 	ldrsb.w	r0, [r2, #1]
 8003e42:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003e46:	bfc8      	it	gt
 8003e48:	3301      	addgt	r3, #1
 8003e4a:	f1bc 0f02 	cmp.w	ip, #2
 8003e4e:	d005      	beq.n	8003e5c <core::fmt::Formatter::pad+0x634>
 8003e50:	f992 0002 	ldrsb.w	r0, [r2, #2]
 8003e54:	f110 0f41 	cmn.w	r0, #65	@ 0x41
 8003e58:	bfc8      	it	gt
 8003e5a:	3301      	addgt	r3, #1
 8003e5c:	4648      	mov	r0, r9
 8003e5e:	8981      	ldrh	r1, [r0, #12]
 8003e60:	428b      	cmp	r3, r1
 8003e62:	d39b      	bcc.n	8003d9c <core::fmt::Formatter::pad+0x574>
 8003e64:	e9d0 0100 	ldrd	r0, r1, [r0]
 8003e68:	4642      	mov	r2, r8
 8003e6a:	68cb      	ldr	r3, [r1, #12]
 8003e6c:	4621      	mov	r1, r4
 8003e6e:	b00d      	add	sp, #52	@ 0x34
 8003e70:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8003e74:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 8003e78:	4718      	bx	r3

08003e7a <core::panicking::panic_const::panic_const_div_by_zero>:
 8003e7a:	b580      	push	{r7, lr}
 8003e7c:	466f      	mov	r7, sp
 8003e7e:	4602      	mov	r2, r0
 8003e80:	f24a 5073 	movw	r0, #42355	@ 0xa573
 8003e84:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8003e88:	2133      	movs	r1, #51	@ 0x33
 8003e8a:	f7ff fc9b 	bl	80037c4 <core::panicking::panic_fmt>

08003e8e <<&T as core::fmt::Debug>::fmt>:
 8003e8e:	b580      	push	{r7, lr}
 8003e90:	466f      	mov	r7, sp
 8003e92:	e9d0 0200 	ldrd	r0, r2, [r0]
 8003e96:	68d2      	ldr	r2, [r2, #12]
 8003e98:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8003e9c:	4710      	bx	r2

08003e9e <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt>:
 8003e9e:	b580      	push	{r7, lr}
 8003ea0:	466f      	mov	r7, sp
 8003ea2:	b082      	sub	sp, #8
 8003ea4:	7802      	ldrb	r2, [r0, #0]
 8003ea6:	f24a 4c80 	movw	ip, #42112	@ 0xa480
 8003eaa:	460b      	mov	r3, r1
 8003eac:	f6c0 0c00 	movt	ip, #2048	@ 0x800
 8003eb0:	2a0a      	cmp	r2, #10
 8003eb2:	d30d      	bcc.n	8003ed0 <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x32>
 8003eb4:	2029      	movs	r0, #41	@ 0x29
 8003eb6:	4350      	muls	r0, r2
 8003eb8:	0b01      	lsrs	r1, r0, #12
 8003eba:	2064      	movs	r0, #100	@ 0x64
 8003ebc:	fb01 2010 	mls	r0, r1, r0, r2
 8003ec0:	b2c0      	uxtb	r0, r0
 8003ec2:	f83c 0010 	ldrh.w	r0, [ip, r0, lsl #1]
 8003ec6:	f827 0c02 	strh.w	r0, [r7, #-2]
 8003eca:	2001      	movs	r0, #1
 8003ecc:	b91a      	cbnz	r2, 8003ed6 <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x38>
 8003ece:	e003      	b.n	8003ed8 <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x3a>
 8003ed0:	2003      	movs	r0, #3
 8003ed2:	4611      	mov	r1, r2
 8003ed4:	b102      	cbz	r2, 8003ed8 <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x3a>
 8003ed6:	b139      	cbz	r1, 8003ee8 <core::fmt::num::imp::<impl core::fmt::Display for u8>::fmt+0x4a>
 8003ed8:	f001 017f 	and.w	r1, r1, #127	@ 0x7f
 8003edc:	3801      	subs	r0, #1
 8003ede:	1efa      	subs	r2, r7, #3
 8003ee0:	eb0c 0141 	add.w	r1, ip, r1, lsl #1
 8003ee4:	7849      	ldrb	r1, [r1, #1]
 8003ee6:	5411      	strb	r1, [r2, r0]
 8003ee8:	1ef9      	subs	r1, r7, #3
 8003eea:	f1c0 0203 	rsb	r2, r0, #3
 8003eee:	4401      	add	r1, r0
 8003ef0:	4618      	mov	r0, r3
 8003ef2:	f7ff fafd 	bl	80034f0 <core::fmt::Formatter::pad_integral>
 8003ef6:	b002      	add	sp, #8
 8003ef8:	bd80      	pop	{r7, pc}

08003efa <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt>:
 8003efa:	b5f0      	push	{r4, r5, r6, r7, lr}
 8003efc:	af03      	add	r7, sp, #12
 8003efe:	f84d bd04 	str.w	fp, [sp, #-4]!
 8003f02:	b084      	sub	sp, #16
 8003f04:	6803      	ldr	r3, [r0, #0]
 8003f06:	f24a 4e80 	movw	lr, #42112	@ 0xa480
 8003f0a:	468c      	mov	ip, r1
 8003f0c:	f6c0 0e00 	movt	lr, #2048	@ 0x800
 8003f10:	08d8      	lsrs	r0, r3, #3
 8003f12:	287c      	cmp	r0, #124	@ 0x7c
 8003f14:	d943      	bls.n	8003f9e <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xa4>
 8003f16:	f241 7059 	movw	r0, #5977	@ 0x1759
 8003f1a:	f2cd 10b7 	movt	r0, #53687	@ 0xd1b7
 8003f1e:	fba3 0100 	umull	r0, r1, r3, r0
 8003f22:	f242 7010 	movw	r0, #10000	@ 0x2710
 8003f26:	0b49      	lsrs	r1, r1, #13
 8003f28:	fb01 3510 	mls	r5, r1, r0, r3
 8003f2c:	b2aa      	uxth	r2, r5
 8003f2e:	0894      	lsrs	r4, r2, #2
 8003f30:	f241 427b 	movw	r2, #5243	@ 0x147b
 8003f34:	4354      	muls	r4, r2
 8003f36:	0c66      	lsrs	r6, r4, #17
 8003f38:	2464      	movs	r4, #100	@ 0x64
 8003f3a:	fb06 5514 	mls	r5, r6, r4, r5
 8003f3e:	f83e 6016 	ldrh.w	r6, [lr, r6, lsl #1]
 8003f42:	f827 6c14 	strh.w	r6, [r7, #-20]
 8003f46:	b2ad      	uxth	r5, r5
 8003f48:	f83e 5015 	ldrh.w	r5, [lr, r5, lsl #1]
 8003f4c:	f827 5c12 	strh.w	r5, [r7, #-18]
 8003f50:	f249 657f 	movw	r5, #38527	@ 0x967f
 8003f54:	f2c0 0598 	movt	r5, #152	@ 0x98
 8003f58:	42ab      	cmp	r3, r5
 8003f5a:	d936      	bls.n	8003fca <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xd0>
 8003f5c:	f648 55b9 	movw	r5, #36281	@ 0x8db9
 8003f60:	f2c0 0506 	movt	r5, #6
 8003f64:	fba1 5605 	umull	r5, r6, r1, r5
 8003f68:	fb06 1010 	mls	r0, r6, r0, r1
 8003f6c:	fb00 f102 	mul.w	r1, r0, r2
 8003f70:	f643 3289 	movw	r2, #15241	@ 0x3b89
 8003f74:	f2c5 52e6 	movt	r2, #21990	@ 0x55e6
 8003f78:	0cc9      	lsrs	r1, r1, #19
 8003f7a:	fb01 0014 	mls	r0, r1, r4, r0
 8003f7e:	f83e 1011 	ldrh.w	r1, [lr, r1, lsl #1]
 8003f82:	fba3 2402 	umull	r2, r4, r3, r2
 8003f86:	f827 1c18 	strh.w	r1, [r7, #-24]
 8003f8a:	2202      	movs	r2, #2
 8003f8c:	b280      	uxth	r0, r0
 8003f8e:	f83e 0010 	ldrh.w	r0, [lr, r0, lsl #1]
 8003f92:	0e61      	lsrs	r1, r4, #25
 8003f94:	f827 0c16 	strh.w	r0, [r7, #-22]
 8003f98:	2909      	cmp	r1, #9
 8003f9a:	d804      	bhi.n	8003fa6 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xac>
 8003f9c:	e018      	b.n	8003fd0 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xd6>
 8003f9e:	220a      	movs	r2, #10
 8003fa0:	4619      	mov	r1, r3
 8003fa2:	2909      	cmp	r1, #9
 8003fa4:	d914      	bls.n	8003fd0 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xd6>
 8003fa6:	b288      	uxth	r0, r1
 8003fa8:	f241 447b 	movw	r4, #5243	@ 0x147b
 8003fac:	0880      	lsrs	r0, r0, #2
 8003fae:	3a02      	subs	r2, #2
 8003fb0:	4360      	muls	r0, r4
 8003fb2:	2464      	movs	r4, #100	@ 0x64
 8003fb4:	0c40      	lsrs	r0, r0, #17
 8003fb6:	fb00 1114 	mls	r1, r0, r4, r1
 8003fba:	f1a7 041a 	sub.w	r4, r7, #26
 8003fbe:	b289      	uxth	r1, r1
 8003fc0:	f83e 1011 	ldrh.w	r1, [lr, r1, lsl #1]
 8003fc4:	52a1      	strh	r1, [r4, r2]
 8003fc6:	b92b      	cbnz	r3, 8003fd4 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xda>
 8003fc8:	e005      	b.n	8003fd6 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xdc>
 8003fca:	2206      	movs	r2, #6
 8003fcc:	2909      	cmp	r1, #9
 8003fce:	d8ea      	bhi.n	8003fa6 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xac>
 8003fd0:	4608      	mov	r0, r1
 8003fd2:	b103      	cbz	r3, 8003fd6 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xdc>
 8003fd4:	b130      	cbz	r0, 8003fe4 <core::fmt::num::imp::<impl core::fmt::Display for u32>::fmt+0xea>
 8003fd6:	eb0e 0040 	add.w	r0, lr, r0, lsl #1
 8003fda:	3a01      	subs	r2, #1
 8003fdc:	f1a7 011a 	sub.w	r1, r7, #26
 8003fe0:	7840      	ldrb	r0, [r0, #1]
 8003fe2:	5488      	strb	r0, [r1, r2]
 8003fe4:	f1a7 001a 	sub.w	r0, r7, #26
 8003fe8:	1881      	adds	r1, r0, r2
 8003fea:	f1c2 020a 	rsb	r2, r2, #10
 8003fee:	4660      	mov	r0, ip
 8003ff0:	f7ff fa7e 	bl	80034f0 <core::fmt::Formatter::pad_integral>
 8003ff4:	b004      	add	sp, #16
 8003ff6:	f85d bb04 	ldr.w	fp, [sp], #4
 8003ffa:	bdf0      	pop	{r4, r5, r6, r7, pc}

08003ffc <core::slice::copy_from_slice_impl::len_mismatch_fail>:
 8003ffc:	b580      	push	{r7, lr}
 8003ffe:	466f      	mov	r7, sp
 8004000:	4603      	mov	r3, r0
 8004002:	4608      	mov	r0, r1
 8004004:	4619      	mov	r1, r3
 8004006:	f000 f800 	bl	800400a <core::slice::copy_from_slice_impl::len_mismatch_fail::do_panic::runtime>

0800400a <core::slice::copy_from_slice_impl::len_mismatch_fail::do_panic::runtime>:
 800400a:	b580      	push	{r7, lr}
 800400c:	466f      	mov	r7, sp
 800400e:	b086      	sub	sp, #24
 8004010:	e9cd 0100 	strd	r0, r1, [sp]
 8004014:	f643 60fb 	movw	r0, #16123	@ 0x3efb
 8004018:	a901      	add	r1, sp, #4
 800401a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800401e:	9005      	str	r0, [sp, #20]
 8004020:	e9cd 0103 	strd	r0, r1, [sp, #12]
 8004024:	4668      	mov	r0, sp
 8004026:	9002      	str	r0, [sp, #8]
 8004028:	f649 401a 	movw	r0, #39962	@ 0x9c1a
 800402c:	a902      	add	r1, sp, #8
 800402e:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004032:	f7ff fbc7 	bl	80037c4 <core::panicking::panic_fmt>

08004036 <core::option::expect_failed>:
 8004036:	b580      	push	{r7, lr}
 8004038:	466f      	mov	r7, sp
 800403a:	b084      	sub	sp, #16
 800403c:	e9cd 0100 	strd	r0, r1, [sp]
 8004040:	f643 0015 	movw	r0, #14357	@ 0x3815
 8004044:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004048:	a902      	add	r1, sp, #8
 800404a:	9003      	str	r0, [sp, #12]
 800404c:	4668      	mov	r0, sp
 800404e:	9002      	str	r0, [sp, #8]
 8004050:	f649 40d1 	movw	r0, #40145	@ 0x9cd1
 8004054:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004058:	f7ff fbb4 	bl	80037c4 <core::panicking::panic_fmt>

0800405c <core::result::unwrap_failed>:
 800405c:	b580      	push	{r7, lr}
 800405e:	466f      	mov	r7, sp
 8004060:	b088      	sub	sp, #32
 8004062:	469c      	mov	ip, r3
 8004064:	232b      	movs	r3, #43	@ 0x2b
 8004066:	e9cd 0300 	strd	r0, r3, [sp]
 800406a:	f643 608f 	movw	r0, #16015	@ 0x3e8f
 800406e:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004072:	e9cd 1202 	strd	r1, r2, [sp, #8]
 8004076:	9007      	str	r0, [sp, #28]
 8004078:	a802      	add	r0, sp, #8
 800407a:	9006      	str	r0, [sp, #24]
 800407c:	f643 0015 	movw	r0, #14357	@ 0x3815
 8004080:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004084:	a904      	add	r1, sp, #16
 8004086:	9005      	str	r0, [sp, #20]
 8004088:	4668      	mov	r0, sp
 800408a:	9004      	str	r0, [sp, #16]
 800408c:	f649 00e3 	movw	r0, #39139	@ 0x98e3
 8004090:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004094:	4662      	mov	r2, ip
 8004096:	f7ff fb95 	bl	80037c4 <core::panicking::panic_fmt>

0800409a <core::panicking::panic_const::panic_const_rem_by_zero>:
 800409a:	b580      	push	{r7, lr}
 800409c:	466f      	mov	r7, sp
 800409e:	4602      	mov	r2, r0
 80040a0:	f24a 508c 	movw	r0, #42380	@ 0xa58c
 80040a4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80040a8:	2173      	movs	r1, #115	@ 0x73
 80040aa:	f7ff fb8b 	bl	80037c4 <core::panicking::panic_fmt>

080040ae <<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt>:
 80040ae:	b5b0      	push	{r4, r5, r7, lr}
 80040b0:	af02      	add	r7, sp, #8
 80040b2:	b090      	sub	sp, #64	@ 0x40
 80040b4:	460c      	mov	r4, r1
 80040b6:	7ac9      	ldrb	r1, [r1, #11]
 80040b8:	6800      	ldr	r0, [r0, #0]
 80040ba:	f011 0f18 	tst.w	r1, #24
 80040be:	9001      	str	r0, [sp, #4]
 80040c0:	d028      	beq.n	8004114 <<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt+0x66>
 80040c2:	2000      	movs	r0, #0
 80040c4:	a901      	add	r1, sp, #4
 80040c6:	9002      	str	r0, [sp, #8]
 80040c8:	f643 609f 	movw	r0, #16031	@ 0x3e9f
 80040cc:	1cca      	adds	r2, r1, #3
 80040ce:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80040d2:	ad02      	add	r5, sp, #8
 80040d4:	e9cd 1007 	strd	r1, r0, [sp, #28]
 80040d8:	e9cd 020c 	strd	r0, r2, [sp, #48]	@ 0x30
 80040dc:	1c8a      	adds	r2, r1, #2
 80040de:	ab07      	add	r3, sp, #28
 80040e0:	900e      	str	r0, [sp, #56]	@ 0x38
 80040e2:	e9cd 020a 	strd	r0, r2, [sp, #40]	@ 0x28
 80040e6:	1c4a      	adds	r2, r1, #1
 80040e8:	f24a 51cc 	movw	r1, #42444	@ 0xa5cc
 80040ec:	9209      	str	r2, [sp, #36]	@ 0x24
 80040ee:	f24a 0269 	movw	r2, #41065	@ 0xa069
 80040f2:	f6c0 0100 	movt	r1, #2048	@ 0x800
 80040f6:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80040fa:	4628      	mov	r0, r5
 80040fc:	f7ff f971 	bl	80033e2 <core::fmt::write>
 8004100:	bb60      	cbnz	r0, 800415c <<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt+0xae>
 8004102:	9a02      	ldr	r2, [sp, #8]
 8004104:	2a10      	cmp	r2, #16
 8004106:	d220      	bcs.n	800414a <<core::net::ip_addr::Ipv4Addr as core::fmt::Display>::fmt+0x9c>
 8004108:	1d29      	adds	r1, r5, #4
 800410a:	4620      	mov	r0, r4
 800410c:	f7ff fb8c 	bl	8003828 <core::fmt::Formatter::pad>
 8004110:	b010      	add	sp, #64	@ 0x40
 8004112:	bdb0      	pop	{r4, r5, r7, pc}
 8004114:	ab01      	add	r3, sp, #4
 8004116:	f643 629f 	movw	r2, #16031	@ 0x3e9f
 800411a:	1cdd      	adds	r5, r3, #3
 800411c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8004120:	e9d4 0100 	ldrd	r0, r1, [r4]
 8004124:	e9cd 250c 	strd	r2, r5, [sp, #48]	@ 0x30
 8004128:	1c9d      	adds	r5, r3, #2
 800412a:	920e      	str	r2, [sp, #56]	@ 0x38
 800412c:	e9cd 250a 	strd	r2, r5, [sp, #40]	@ 0x28
 8004130:	1c5d      	adds	r5, r3, #1
 8004132:	e9cd 3207 	strd	r3, r2, [sp, #28]
 8004136:	f24a 0269 	movw	r2, #41065	@ 0xa069
 800413a:	ab07      	add	r3, sp, #28
 800413c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8004140:	9509      	str	r5, [sp, #36]	@ 0x24
 8004142:	f7ff f94e 	bl	80033e2 <core::fmt::write>
 8004146:	b010      	add	sp, #64	@ 0x40
 8004148:	bdb0      	pop	{r4, r5, r7, pc}
 800414a:	f24a 53e4 	movw	r3, #42468	@ 0xa5e4
 800414e:	4611      	mov	r1, r2
 8004150:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8004154:	2000      	movs	r0, #0
 8004156:	220f      	movs	r2, #15
 8004158:	f7ff fac3 	bl	80036e2 <core::slice::index::slice_index_fail>
 800415c:	f64a 0008 	movw	r0, #43016	@ 0xa808
 8004160:	f24a 52f4 	movw	r2, #42484	@ 0xa5f4
 8004164:	f24a 6304 	movw	r3, #42500	@ 0xa604
 8004168:	f1a7 0109 	sub.w	r1, r7, #9
 800416c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004170:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8004174:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8004178:	f7ff ff70 	bl	800405c <core::result::unwrap_failed>

0800417c <<core::fmt::Error as core::fmt::Debug>::fmt>:
 800417c:	b580      	push	{r7, lr}
 800417e:	466f      	mov	r7, sp
 8004180:	e9d1 0200 	ldrd	r0, r2, [r1]
 8004184:	f24a 51c5 	movw	r1, #42437	@ 0xa5c5
 8004188:	68d3      	ldr	r3, [r2, #12]
 800418a:	f6c0 0100 	movt	r1, #2048	@ 0x800
 800418e:	2205      	movs	r2, #5
 8004190:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8004194:	4718      	bx	r3

08004196 <<core::net::display_buffer::DisplayBuffer<_> as core::fmt::Write>::write_str>:
 8004196:	b5f0      	push	{r4, r5, r6, r7, lr}
 8004198:	af03      	add	r7, sp, #12
 800419a:	f84d bd04 	str.w	fp, [sp, #-4]!
 800419e:	6803      	ldr	r3, [r0, #0]
 80041a0:	2400      	movs	r4, #0
 80041a2:	189e      	adds	r6, r3, r2
 80041a4:	f144 0500 	adc.w	r5, r4, #0
 80041a8:	2e0f      	cmp	r6, #15
 80041aa:	bf88      	it	hi
 80041ac:	2401      	movhi	r4, #1
 80041ae:	432c      	orrs	r4, r5
 80041b0:	d106      	bne.n	80041c0 <<core::net::display_buffer::DisplayBuffer<_> as core::fmt::Write>::write_str+0x2a>
 80041b2:	4403      	add	r3, r0
 80041b4:	4605      	mov	r5, r0
 80041b6:	3304      	adds	r3, #4
 80041b8:	4618      	mov	r0, r3
 80041ba:	f005 f89a 	bl	80092f2 <__aeabi_memcpy>
 80041be:	602e      	str	r6, [r5, #0]
 80041c0:	4620      	mov	r0, r4
 80041c2:	f85d bb04 	ldr.w	fp, [sp], #4
 80041c6:	bdf0      	pop	{r4, r5, r6, r7, pc}

080041c8 <core::fmt::Write::write_char>:
 80041c8:	b5f0      	push	{r4, r5, r6, r7, lr}
 80041ca:	af03      	add	r7, sp, #12
 80041cc:	f84d bd04 	str.w	fp, [sp, #-4]!
 80041d0:	b082      	sub	sp, #8
 80041d2:	2200      	movs	r2, #0
 80041d4:	2980      	cmp	r1, #128	@ 0x80
 80041d6:	9201      	str	r2, [sp, #4]
 80041d8:	d203      	bcs.n	80041e2 <core::fmt::Write::write_char+0x1a>
 80041da:	f88d 1004 	strb.w	r1, [sp, #4]
 80041de:	2201      	movs	r2, #1
 80041e0:	e02f      	b.n	8004242 <core::fmt::Write::write_char+0x7a>
 80041e2:	f64f 7cfe 	movw	ip, #65534	@ 0xfffe
 80041e6:	460b      	mov	r3, r1
 80041e8:	f2c0 3cff 	movt	ip, #1023	@ 0x3ff
 80041ec:	098a      	lsrs	r2, r1, #6
 80041ee:	f36c 139f 	bfi	r3, ip, #6, #26
 80041f2:	f5b1 6f00 	cmp.w	r1, #2048	@ 0x800
 80041f6:	d207      	bcs.n	8004208 <core::fmt::Write::write_char+0x40>
 80041f8:	f042 01c0 	orr.w	r1, r2, #192	@ 0xc0
 80041fc:	f88d 3005 	strb.w	r3, [sp, #5]
 8004200:	f88d 1004 	strb.w	r1, [sp, #4]
 8004204:	2202      	movs	r2, #2
 8004206:	e01c      	b.n	8004242 <core::fmt::Write::write_char+0x7a>
 8004208:	f36c 129f 	bfi	r2, ip, #6, #26
 800420c:	0b0c      	lsrs	r4, r1, #12
 800420e:	f5b1 3f80 	cmp.w	r1, #65536	@ 0x10000
 8004212:	d207      	bcs.n	8004224 <core::fmt::Write::write_char+0x5c>
 8004214:	f88d 2005 	strb.w	r2, [sp, #5]
 8004218:	2203      	movs	r2, #3
 800421a:	f044 01e0 	orr.w	r1, r4, #224	@ 0xe0
 800421e:	f88d 3006 	strb.w	r3, [sp, #6]
 8004222:	e00c      	b.n	800423e <core::fmt::Write::write_char+0x76>
 8004224:	f06f 050f 	mvn.w	r5, #15
 8004228:	ea45 4191 	orr.w	r1, r5, r1, lsr #18
 800422c:	f36c 149f 	bfi	r4, ip, #6, #26
 8004230:	f88d 2006 	strb.w	r2, [sp, #6]
 8004234:	f88d 3007 	strb.w	r3, [sp, #7]
 8004238:	2204      	movs	r2, #4
 800423a:	f88d 4005 	strb.w	r4, [sp, #5]
 800423e:	f88d 1004 	strb.w	r1, [sp, #4]
 8004242:	6801      	ldr	r1, [r0, #0]
 8004244:	2300      	movs	r3, #0
 8004246:	188d      	adds	r5, r1, r2
 8004248:	f143 0400 	adc.w	r4, r3, #0
 800424c:	2d0f      	cmp	r5, #15
 800424e:	bf88      	it	hi
 8004250:	2301      	movhi	r3, #1
 8004252:	431c      	orrs	r4, r3
 8004254:	d107      	bne.n	8004266 <core::fmt::Write::write_char+0x9e>
 8004256:	4401      	add	r1, r0
 8004258:	4606      	mov	r6, r0
 800425a:	1d0b      	adds	r3, r1, #4
 800425c:	a901      	add	r1, sp, #4
 800425e:	4618      	mov	r0, r3
 8004260:	f005 f847 	bl	80092f2 <__aeabi_memcpy>
 8004264:	6035      	str	r5, [r6, #0]
 8004266:	4620      	mov	r0, r4
 8004268:	b002      	add	sp, #8
 800426a:	f85d bb04 	ldr.w	fp, [sp], #4
 800426e:	bdf0      	pop	{r4, r5, r6, r7, pc}

08004270 <core::fmt::Write::write_fmt>:
 8004270:	b580      	push	{r7, lr}
 8004272:	466f      	mov	r7, sp
 8004274:	4613      	mov	r3, r2
 8004276:	460a      	mov	r2, r1
 8004278:	f24a 51cc 	movw	r1, #42444	@ 0xa5cc
 800427c:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8004280:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8004284:	f7ff b8ad 	b.w	80033e2 <core::fmt::write>

08004288 <DefaultHandler_>:
 8004288:	b580      	push	{r7, lr}
 800428a:	466f      	mov	r7, sp
 800428c:	e7fe      	b.n	800428c <DefaultHandler_+0x4>

0800428e <DefaultPreInit>:
 800428e:	b580      	push	{r7, lr}
 8004290:	466f      	mov	r7, sp
 8004292:	bd80      	pop	{r7, pc}

08004294 <defmt::impls::core_::net::<impl defmt::traits::Format for core::net::ip_addr::Ipv4Addr>::format>:
 8004294:	b5f0      	push	{r4, r5, r6, r7, lr}
 8004296:	af03      	add	r7, sp, #12
 8004298:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 800429c:	b086      	sub	sp, #24
 800429e:	7805      	ldrb	r5, [r0, #0]
 80042a0:	2102      	movs	r1, #2
 80042a2:	7844      	ldrb	r4, [r0, #1]
 80042a4:	f890 9002 	ldrb.w	r9, [r0, #2]
 80042a8:	f890 8003 	ldrb.w	r8, [r0, #3]
 80042ac:	f240 0032 	movw	r0, #50	@ 0x32
 80042b0:	f2c0 0000 	movt	r0, #0
 80042b4:	f827 0c2a 	strh.w	r0, [r7, #-42]
 80042b8:	f1a7 002a 	sub.w	r0, r7, #42	@ 0x2a
 80042bc:	f000 f9ee 	bl	800469c <_defmt_write>
 80042c0:	f240 0602 	movw	r6, #2
 80042c4:	a802      	add	r0, sp, #8
 80042c6:	f2c0 0600 	movt	r6, #0
 80042ca:	2102      	movs	r1, #2
 80042cc:	f8ad 6008 	strh.w	r6, [sp, #8]
 80042d0:	f000 f9e4 	bl	800469c <_defmt_write>
 80042d4:	f1a7 0025 	sub.w	r0, r7, #37	@ 0x25
 80042d8:	2101      	movs	r1, #1
 80042da:	f807 5c25 	strb.w	r5, [r7, #-37]
 80042de:	f000 f9dd 	bl	800469c <_defmt_write>
 80042e2:	a803      	add	r0, sp, #12
 80042e4:	2102      	movs	r1, #2
 80042e6:	f8ad 600c 	strh.w	r6, [sp, #12]
 80042ea:	f000 f9d7 	bl	800469c <_defmt_write>
 80042ee:	f1a7 0021 	sub.w	r0, r7, #33	@ 0x21
 80042f2:	2101      	movs	r1, #1
 80042f4:	f807 4c21 	strb.w	r4, [r7, #-33]
 80042f8:	f000 f9d0 	bl	800469c <_defmt_write>
 80042fc:	a804      	add	r0, sp, #16
 80042fe:	2102      	movs	r1, #2
 8004300:	f8ad 6010 	strh.w	r6, [sp, #16]
 8004304:	f000 f9ca 	bl	800469c <_defmt_write>
 8004308:	f1a7 001d 	sub.w	r0, r7, #29
 800430c:	2101      	movs	r1, #1
 800430e:	f807 9c1d 	strb.w	r9, [r7, #-29]
 8004312:	f000 f9c3 	bl	800469c <_defmt_write>
 8004316:	a805      	add	r0, sp, #20
 8004318:	2102      	movs	r1, #2
 800431a:	f8ad 6014 	strh.w	r6, [sp, #20]
 800431e:	f000 f9bd 	bl	800469c <_defmt_write>
 8004322:	f1a7 0019 	sub.w	r0, r7, #25
 8004326:	2101      	movs	r1, #1
 8004328:	f807 8c19 	strb.w	r8, [r7, #-25]
 800432c:	f000 f9b6 	bl	800469c <_defmt_write>
 8004330:	b006      	add	sp, #24
 8004332:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8004336:	bdf0      	pop	{r4, r5, r6, r7, pc}

08004338 <defmt::export::acquire_and_header>:
 8004338:	b5d0      	push	{r4, r6, r7, lr}
 800433a:	af02      	add	r7, sp, #8
 800433c:	b082      	sub	sp, #8
 800433e:	4604      	mov	r4, r0
 8004340:	f000 f8e5 	bl	800450e <_defmt_acquire>
 8004344:	f1a7 000a 	sub.w	r0, r7, #10
 8004348:	2102      	movs	r1, #2
 800434a:	f827 4c0a 	strh.w	r4, [r7, #-10]
 800434e:	f000 f9a5 	bl	800469c <_defmt_write>
 8004352:	f7ff ff9c 	bl	800428e <DefaultPreInit>
 8004356:	b002      	add	sp, #8
 8004358:	bdd0      	pop	{r4, r6, r7, pc}

0800435a <defmt::export::acquire_header_and_release>:
 800435a:	b5d0      	push	{r4, r6, r7, lr}
 800435c:	af02      	add	r7, sp, #8
 800435e:	b082      	sub	sp, #8
 8004360:	4604      	mov	r4, r0
 8004362:	f000 f8d4 	bl	800450e <_defmt_acquire>
 8004366:	f1a7 000a 	sub.w	r0, r7, #10
 800436a:	2102      	movs	r1, #2
 800436c:	f827 4c0a 	strh.w	r4, [r7, #-10]
 8004370:	f000 f994 	bl	800469c <_defmt_write>
 8004374:	f7ff ff8b 	bl	800428e <DefaultPreInit>
 8004378:	f000 f918 	bl	80045ac <_defmt_release>
 800437c:	b002      	add	sp, #8
 800437e:	bdd0      	pop	{r4, r6, r7, pc}

08004380 <<defmt::export::FmtWrite as core::fmt::Write>::write_str>:
 8004380:	b580      	push	{r7, lr}
 8004382:	466f      	mov	r7, sp
 8004384:	4608      	mov	r0, r1
 8004386:	4611      	mov	r1, r2
 8004388:	f000 f988 	bl	800469c <_defmt_write>
 800438c:	2000      	movs	r0, #0
 800438e:	bd80      	pop	{r7, pc}

08004390 <core::fmt::Write::write_char>:
 8004390:	b580      	push	{r7, lr}
 8004392:	466f      	mov	r7, sp
 8004394:	b082      	sub	sp, #8
 8004396:	2000      	movs	r0, #0
 8004398:	2980      	cmp	r1, #128	@ 0x80
 800439a:	9001      	str	r0, [sp, #4]
 800439c:	d203      	bcs.n	80043a6 <core::fmt::Write::write_char+0x16>
 800439e:	f88d 1004 	strb.w	r1, [sp, #4]
 80043a2:	2101      	movs	r1, #1
 80043a4:	e032      	b.n	800440c <core::fmt::Write::write_char+0x7c>
 80043a6:	f64f 7cfe 	movw	ip, #65534	@ 0xfffe
 80043aa:	460a      	mov	r2, r1
 80043ac:	f2c0 3cff 	movt	ip, #1023	@ 0x3ff
 80043b0:	0988      	lsrs	r0, r1, #6
 80043b2:	f36c 129f 	bfi	r2, ip, #6, #26
 80043b6:	f5b1 6f00 	cmp.w	r1, #2048	@ 0x800
 80043ba:	d207      	bcs.n	80043cc <core::fmt::Write::write_char+0x3c>
 80043bc:	f040 00c0 	orr.w	r0, r0, #192	@ 0xc0
 80043c0:	f88d 2005 	strb.w	r2, [sp, #5]
 80043c4:	f88d 0004 	strb.w	r0, [sp, #4]
 80043c8:	2102      	movs	r1, #2
 80043ca:	e01f      	b.n	800440c <core::fmt::Write::write_char+0x7c>
 80043cc:	f36c 109f 	bfi	r0, ip, #6, #26
 80043d0:	ea4f 3e11 	mov.w	lr, r1, lsr #12
 80043d4:	f5b1 3f80 	cmp.w	r1, #65536	@ 0x10000
 80043d8:	d209      	bcs.n	80043ee <core::fmt::Write::write_char+0x5e>
 80043da:	2103      	movs	r1, #3
 80043dc:	f88d 0005 	strb.w	r0, [sp, #5]
 80043e0:	f04e 00e0 	orr.w	r0, lr, #224	@ 0xe0
 80043e4:	f88d 2006 	strb.w	r2, [sp, #6]
 80043e8:	f88d 0004 	strb.w	r0, [sp, #4]
 80043ec:	e00e      	b.n	800440c <core::fmt::Write::write_char+0x7c>
 80043ee:	f06f 030f 	mvn.w	r3, #15
 80043f2:	ea43 4191 	orr.w	r1, r3, r1, lsr #18
 80043f6:	f36c 1e9f 	bfi	lr, ip, #6, #26
 80043fa:	f88d 1004 	strb.w	r1, [sp, #4]
 80043fe:	f88d 2007 	strb.w	r2, [sp, #7]
 8004402:	2104      	movs	r1, #4
 8004404:	f88d 0006 	strb.w	r0, [sp, #6]
 8004408:	f88d e005 	strb.w	lr, [sp, #5]
 800440c:	a801      	add	r0, sp, #4
 800440e:	f000 f945 	bl	800469c <_defmt_write>
 8004412:	2000      	movs	r0, #0
 8004414:	b002      	add	sp, #8
 8004416:	bd80      	pop	{r7, pc}

08004418 <core::fmt::Write::write_fmt>:
 8004418:	b580      	push	{r7, lr}
 800441a:	466f      	mov	r7, sp
 800441c:	4613      	mov	r3, r2
 800441e:	460a      	mov	r2, r1
 8004420:	f24a 6124 	movw	r1, #42532	@ 0xa624
 8004424:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8004428:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 800442c:	f7fe bfd9 	b.w	80033e2 <core::fmt::write>

08004430 <defmt_rtt::channel::Channel::blocking_write>:
 8004430:	b5f0      	push	{r4, r5, r6, r7, lr}
 8004432:	af03      	add	r7, sp, #12
 8004434:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 8004438:	b152      	cbz	r2, 8004450 <defmt_rtt::channel::Channel::blocking_write+0x20>
 800443a:	4605      	mov	r5, r0
 800443c:	4614      	mov	r4, r2
 800443e:	6900      	ldr	r0, [r0, #16]
 8004440:	68ea      	ldr	r2, [r5, #12]
 8004442:	f3bf 8f5f 	dmb	sy
 8004446:	4290      	cmp	r0, r2
 8004448:	d907      	bls.n	800445a <defmt_rtt::channel::Channel::blocking_write+0x2a>
 800444a:	43d3      	mvns	r3, r2
 800444c:	4418      	add	r0, r3
 800444e:	b968      	cbnz	r0, 800446c <defmt_rtt::channel::Channel::blocking_write+0x3c>
 8004450:	2400      	movs	r4, #0
 8004452:	4620      	mov	r0, r4
 8004454:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8004458:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800445a:	2800      	cmp	r0, #0
 800445c:	bf06      	itte	eq
 800445e:	f240 30ff 	movweq	r0, #1023	@ 0x3ff
 8004462:	1a80      	subeq	r0, r0, r2
 8004464:	f5c2 6080 	rsbne	r0, r2, #1024	@ 0x400
 8004468:	2800      	cmp	r0, #0
 800446a:	d0f1      	beq.n	8004450 <defmt_rtt::channel::Channel::blocking_write+0x20>
 800446c:	42a0      	cmp	r0, r4
 800446e:	bf38      	it	cc
 8004470:	4604      	movcc	r4, r0
 8004472:	18a6      	adds	r6, r4, r2
 8004474:	f5b6 6f80 	cmp.w	r6, #1024	@ 0x400
 8004478:	d90d      	bls.n	8004496 <defmt_rtt::channel::Channel::blocking_write+0x66>
 800447a:	6868      	ldr	r0, [r5, #4]
 800447c:	f5c2 6980 	rsb	r9, r2, #1024	@ 0x400
 8004480:	4688      	mov	r8, r1
 8004482:	4410      	add	r0, r2
 8004484:	464a      	mov	r2, r9
 8004486:	f004 ff34 	bl	80092f2 <__aeabi_memcpy>
 800448a:	6868      	ldr	r0, [r5, #4]
 800448c:	eb08 0109 	add.w	r1, r8, r9
 8004490:	eba4 0209 	sub.w	r2, r4, r9
 8004494:	e002      	b.n	800449c <defmt_rtt::channel::Channel::blocking_write+0x6c>
 8004496:	6868      	ldr	r0, [r5, #4]
 8004498:	4410      	add	r0, r2
 800449a:	4622      	mov	r2, r4
 800449c:	f004 ff29 	bl	80092f2 <__aeabi_memcpy>
 80044a0:	f36f 269f 	bfc	r6, #10, #22
 80044a4:	f3bf 8f5f 	dmb	sy
 80044a8:	60ee      	str	r6, [r5, #12]
 80044aa:	4620      	mov	r0, r4
 80044ac:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80044b0:	bdf0      	pop	{r4, r5, r6, r7, pc}

080044b2 <defmt_rtt::channel::Channel::nonblocking_write>:
 80044b2:	b5f0      	push	{r4, r5, r6, r7, lr}
 80044b4:	af03      	add	r7, sp, #12
 80044b6:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 80044ba:	4614      	mov	r4, r2
 80044bc:	68c2      	ldr	r2, [r0, #12]
 80044be:	f5b4 6f80 	cmp.w	r4, #1024	@ 0x400
 80044c2:	bf28      	it	cs
 80044c4:	f44f 6480 	movcs.w	r4, #1024	@ 0x400
 80044c8:	1916      	adds	r6, r2, r4
 80044ca:	4605      	mov	r5, r0
 80044cc:	f5b6 6f80 	cmp.w	r6, #1024	@ 0x400
 80044d0:	f3bf 8f5f 	dmb	sy
 80044d4:	d90d      	bls.n	80044f2 <defmt_rtt::channel::Channel::nonblocking_write+0x40>
 80044d6:	6868      	ldr	r0, [r5, #4]
 80044d8:	f5c2 6980 	rsb	r9, r2, #1024	@ 0x400
 80044dc:	4688      	mov	r8, r1
 80044de:	4410      	add	r0, r2
 80044e0:	464a      	mov	r2, r9
 80044e2:	f004 ff06 	bl	80092f2 <__aeabi_memcpy>
 80044e6:	6868      	ldr	r0, [r5, #4]
 80044e8:	eb08 0109 	add.w	r1, r8, r9
 80044ec:	eba4 0209 	sub.w	r2, r4, r9
 80044f0:	e002      	b.n	80044f8 <defmt_rtt::channel::Channel::nonblocking_write+0x46>
 80044f2:	6868      	ldr	r0, [r5, #4]
 80044f4:	4410      	add	r0, r2
 80044f6:	4622      	mov	r2, r4
 80044f8:	f004 fefb 	bl	80092f2 <__aeabi_memcpy>
 80044fc:	f3bf 8f5f 	dmb	sy
 8004500:	f36f 269f 	bfc	r6, #10, #22
 8004504:	60ee      	str	r6, [r5, #12]
 8004506:	4620      	mov	r0, r4
 8004508:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 800450c:	bdf0      	pop	{r4, r5, r6, r7, pc}

0800450e <_defmt_acquire>:
 800450e:	b5f0      	push	{r4, r5, r6, r7, lr}
 8004510:	af03      	add	r7, sp, #12
 8004512:	f84d bd04 	str.w	fp, [sp, #-4]!
 8004516:	b082      	sub	sp, #8
 8004518:	f004 f965 	bl	80087e6 <__primask_r>
 800451c:	4604      	mov	r4, r0
 800451e:	f004 f955 	bl	80087cc <__cpsid>
 8004522:	f240 0040 	movw	r0, #64	@ 0x40
 8004526:	f2c2 0000 	movt	r0, #8192	@ 0x2000
 800452a:	7841      	ldrb	r1, [r0, #1]
 800452c:	bb59      	cbnz	r1, 8004586 <_defmt_acquire+0x78>
 800452e:	2101      	movs	r1, #1
 8004530:	ea21 0304 	bic.w	r3, r1, r4
 8004534:	7041      	strb	r1, [r0, #1]
 8004536:	7882      	ldrb	r2, [r0, #2]
 8004538:	7003      	strb	r3, [r0, #0]
 800453a:	bb02      	cbnz	r2, 800457e <_defmt_acquire+0x70>
 800453c:	7081      	strb	r1, [r0, #2]
 800453e:	f240 0508 	movw	r5, #8
 8004542:	2000      	movs	r0, #0
 8004544:	f2c2 0500 	movt	r5, #8192	@ 0x2000
 8004548:	f807 0c11 	strb.w	r0, [r7, #-17]
 800454c:	f244 46b3 	movw	r6, #17587	@ 0x44b3
 8004550:	6ae8      	ldr	r0, [r5, #44]	@ 0x2c
 8004552:	f1a7 0411 	sub.w	r4, r7, #17
 8004556:	f244 4131 	movw	r1, #17457	@ 0x4431
 800455a:	f6c0 0600 	movt	r6, #2048	@ 0x800
 800455e:	f000 0003 	and.w	r0, r0, #3
 8004562:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8004566:	2802      	cmp	r0, #2
 8004568:	bf08      	it	eq
 800456a:	460e      	moveq	r6, r1
 800456c:	f105 0018 	add.w	r0, r5, #24
 8004570:	4621      	mov	r1, r4
 8004572:	2201      	movs	r2, #1
 8004574:	47b0      	blx	r6
 8004576:	2800      	cmp	r0, #0
 8004578:	d0f8      	beq.n	800456c <_defmt_acquire+0x5e>
 800457a:	2801      	cmp	r0, #1
 800457c:	d10e      	bne.n	800459c <_defmt_acquire+0x8e>
 800457e:	b002      	add	sp, #8
 8004580:	f85d bb04 	ldr.w	fp, [sp], #4
 8004584:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8004586:	f24a 603d 	movw	r0, #42557	@ 0xa63d
 800458a:	f24a 625c 	movw	r2, #42588	@ 0xa65c
 800458e:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004592:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8004596:	213d      	movs	r1, #61	@ 0x3d
 8004598:	f7ff f914 	bl	80037c4 <core::panicking::panic_fmt>
 800459c:	f24a 636c 	movw	r3, #42604	@ 0xa66c
 80045a0:	2101      	movs	r1, #1
 80045a2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80045a6:	2201      	movs	r2, #1
 80045a8:	f7ff f89b 	bl	80036e2 <core::slice::index::slice_index_fail>

080045ac <_defmt_release>:
 80045ac:	b5f0      	push	{r4, r5, r6, r7, lr}
 80045ae:	af03      	add	r7, sp, #12
 80045b0:	e92d 0700 	stmdb	sp!, {r8, r9, sl}
 80045b4:	b082      	sub	sp, #8
 80045b6:	f240 0840 	movw	r8, #64	@ 0x40
 80045ba:	f240 0608 	movw	r6, #8
 80045be:	f2c2 0800 	movt	r8, #8192	@ 0x2000
 80045c2:	f244 4a31 	movw	sl, #17457	@ 0x4431
 80045c6:	f898 0003 	ldrb.w	r0, [r8, #3]
 80045ca:	f244 45b3 	movw	r5, #17587	@ 0x44b3
 80045ce:	f2c2 0600 	movt	r6, #8192	@ 0x2000
 80045d2:	f6c0 0a00 	movt	sl, #2048	@ 0x800
 80045d6:	f6c0 0500 	movt	r5, #2048	@ 0x800
 80045da:	b3a0      	cbz	r0, 8004646 <_defmt_release+0x9a>
 80045dc:	2807      	cmp	r0, #7
 80045de:	d21b      	bcs.n	8004618 <_defmt_release+0x6c>
 80045e0:	f04f 32ff 	mov.w	r2, #4294967295	@ 0xffffffff
 80045e4:	f898 1004 	ldrb.w	r1, [r8, #4]
 80045e8:	fa02 f000 	lsl.w	r0, r2, r0
 80045ec:	f1a7 091a 	sub.w	r9, r7, #26
 80045f0:	4308      	orrs	r0, r1
 80045f2:	462c      	mov	r4, r5
 80045f4:	f000 007f 	and.w	r0, r0, #127	@ 0x7f
 80045f8:	f807 0c1a 	strb.w	r0, [r7, #-26]
 80045fc:	6af0      	ldr	r0, [r6, #44]	@ 0x2c
 80045fe:	f000 0003 	and.w	r0, r0, #3
 8004602:	2802      	cmp	r0, #2
 8004604:	bf08      	it	eq
 8004606:	4654      	moveq	r4, sl
 8004608:	f106 0018 	add.w	r0, r6, #24
 800460c:	4649      	mov	r1, r9
 800460e:	2201      	movs	r2, #1
 8004610:	47a0      	blx	r4
 8004612:	2800      	cmp	r0, #0
 8004614:	d0f8      	beq.n	8004608 <_defmt_release+0x5c>
 8004616:	e014      	b.n	8004642 <_defmt_release+0x96>
 8004618:	3079      	adds	r0, #121	@ 0x79
 800461a:	f1a7 091b 	sub.w	r9, r7, #27
 800461e:	f040 0080 	orr.w	r0, r0, #128	@ 0x80
 8004622:	f807 0c1b 	strb.w	r0, [r7, #-27]
 8004626:	6af0      	ldr	r0, [r6, #44]	@ 0x2c
 8004628:	462c      	mov	r4, r5
 800462a:	f000 0003 	and.w	r0, r0, #3
 800462e:	2802      	cmp	r0, #2
 8004630:	bf08      	it	eq
 8004632:	4654      	moveq	r4, sl
 8004634:	f106 0018 	add.w	r0, r6, #24
 8004638:	4649      	mov	r1, r9
 800463a:	2201      	movs	r2, #1
 800463c:	47a0      	blx	r4
 800463e:	2800      	cmp	r0, #0
 8004640:	d0f8      	beq.n	8004634 <_defmt_release+0x88>
 8004642:	2801      	cmp	r0, #1
 8004644:	d122      	bne.n	800468c <_defmt_release+0xe0>
 8004646:	2000      	movs	r0, #0
 8004648:	f1a7 0419 	sub.w	r4, r7, #25
 800464c:	f807 0c19 	strb.w	r0, [r7, #-25]
 8004650:	6af0      	ldr	r0, [r6, #44]	@ 0x2c
 8004652:	f000 0003 	and.w	r0, r0, #3
 8004656:	2802      	cmp	r0, #2
 8004658:	bf08      	it	eq
 800465a:	4655      	moveq	r5, sl
 800465c:	f106 0018 	add.w	r0, r6, #24
 8004660:	4621      	mov	r1, r4
 8004662:	2201      	movs	r2, #1
 8004664:	47a8      	blx	r5
 8004666:	2800      	cmp	r0, #0
 8004668:	d0f8      	beq.n	800465c <_defmt_release+0xb0>
 800466a:	2801      	cmp	r0, #1
 800466c:	d10e      	bne.n	800468c <_defmt_release+0xe0>
 800466e:	2000      	movs	r0, #0
 8004670:	f8a8 0003 	strh.w	r0, [r8, #3]
 8004674:	f888 0001 	strb.w	r0, [r8, #1]
 8004678:	f898 0000 	ldrb.w	r0, [r8]
 800467c:	07c0      	lsls	r0, r0, #31
 800467e:	bf18      	it	ne
 8004680:	f004 f8a6 	blne	80087d0 <__cpsie>
 8004684:	b002      	add	sp, #8
 8004686:	e8bd 0700 	ldmia.w	sp!, {r8, r9, sl}
 800468a:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800468c:	f24a 636c 	movw	r3, #42604	@ 0xa66c
 8004690:	2101      	movs	r1, #1
 8004692:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8004696:	2201      	movs	r2, #1
 8004698:	f7ff f823 	bl	80036e2 <core::slice::index::slice_index_fail>

0800469c <_defmt_write>:
 800469c:	b5f0      	push	{r4, r5, r6, r7, lr}
 800469e:	af03      	add	r7, sp, #12
 80046a0:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 80046a4:	b083      	sub	sp, #12
 80046a6:	2900      	cmp	r1, #0
 80046a8:	f000 80ba 	beq.w	8004820 <_defmt_write+0x184>
 80046ac:	f240 0840 	movw	r8, #64	@ 0x40
 80046b0:	4604      	mov	r4, r0
 80046b2:	f2c2 0800 	movt	r8, #8192	@ 0x2000
 80046b6:	eb00 0a01 	add.w	sl, r0, r1
 80046ba:	f898 0003 	ldrb.w	r0, [r8, #3]
 80046be:	f240 0908 	movw	r9, #8
 80046c2:	f244 4bb3 	movw	fp, #17587	@ 0x44b3
 80046c6:	f2c2 0900 	movt	r9, #8192	@ 0x2000
 80046ca:	f6c0 0b00 	movt	fp, #2048	@ 0x800
 80046ce:	e008      	b.n	80046e2 <_defmt_write+0x46>
 80046d0:	2801      	cmp	r0, #1
 80046d2:	f040 80a9 	bne.w	8004828 <_defmt_write+0x18c>
 80046d6:	2000      	movs	r0, #0
 80046d8:	f8a8 0003 	strh.w	r0, [r8, #3]
 80046dc:	4554      	cmp	r4, sl
 80046de:	f000 809f 	beq.w	8004820 <_defmt_write+0x184>
 80046e2:	f814 1b01 	ldrb.w	r1, [r4], #1
 80046e6:	b2c2      	uxtb	r2, r0
 80046e8:	2a07      	cmp	r2, #7
 80046ea:	d21f      	bcs.n	800472c <_defmt_write+0x90>
 80046ec:	2900      	cmp	r1, #0
 80046ee:	d057      	beq.n	80047a0 <_defmt_write+0x104>
 80046f0:	f807 1c21 	strb.w	r1, [r7, #-33]
 80046f4:	465d      	mov	r5, fp
 80046f6:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 80046fa:	f000 0003 	and.w	r0, r0, #3
 80046fe:	2802      	cmp	r0, #2
 8004700:	f244 4031 	movw	r0, #17457	@ 0x4431
 8004704:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004708:	bf08      	it	eq
 800470a:	4605      	moveq	r5, r0
 800470c:	f109 0018 	add.w	r0, r9, #24
 8004710:	f1a7 0121 	sub.w	r1, r7, #33	@ 0x21
 8004714:	2201      	movs	r2, #1
 8004716:	47a8      	blx	r5
 8004718:	2800      	cmp	r0, #0
 800471a:	d0f7      	beq.n	800470c <_defmt_write+0x70>
 800471c:	2801      	cmp	r0, #1
 800471e:	f040 8083 	bne.w	8004828 <_defmt_write+0x18c>
 8004722:	f898 0003 	ldrb.w	r0, [r8, #3]
 8004726:	f898 1004 	ldrb.w	r1, [r8, #4]
 800472a:	e041      	b.n	80047b0 <_defmt_write+0x114>
 800472c:	2900      	cmp	r1, #0
 800472e:	d05d      	beq.n	80047ec <_defmt_write+0x150>
 8004730:	f807 1c1e 	strb.w	r1, [r7, #-30]
 8004734:	465d      	mov	r5, fp
 8004736:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 800473a:	f000 0003 	and.w	r0, r0, #3
 800473e:	2802      	cmp	r0, #2
 8004740:	f244 4031 	movw	r0, #17457	@ 0x4431
 8004744:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004748:	bf08      	it	eq
 800474a:	4605      	moveq	r5, r0
 800474c:	f109 0618 	add.w	r6, r9, #24
 8004750:	f1a7 011e 	sub.w	r1, r7, #30
 8004754:	2201      	movs	r2, #1
 8004756:	4630      	mov	r0, r6
 8004758:	47a8      	blx	r5
 800475a:	2800      	cmp	r0, #0
 800475c:	d0f6      	beq.n	800474c <_defmt_write+0xb0>
 800475e:	2801      	cmp	r0, #1
 8004760:	d162      	bne.n	8004828 <_defmt_write+0x18c>
 8004762:	f898 0003 	ldrb.w	r0, [r8, #3]
 8004766:	3001      	adds	r0, #1
 8004768:	f888 0003 	strb.w	r0, [r8, #3]
 800476c:	b2c1      	uxtb	r1, r0
 800476e:	2986      	cmp	r1, #134	@ 0x86
 8004770:	d1b4      	bne.n	80046dc <_defmt_write+0x40>
 8004772:	20ff      	movs	r0, #255	@ 0xff
 8004774:	465d      	mov	r5, fp
 8004776:	f807 0c1d 	strb.w	r0, [r7, #-29]
 800477a:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 800477e:	f000 0003 	and.w	r0, r0, #3
 8004782:	2802      	cmp	r0, #2
 8004784:	f244 4031 	movw	r0, #17457	@ 0x4431
 8004788:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800478c:	bf08      	it	eq
 800478e:	4605      	moveq	r5, r0
 8004790:	f1a7 011d 	sub.w	r1, r7, #29
 8004794:	4630      	mov	r0, r6
 8004796:	2201      	movs	r2, #1
 8004798:	47a8      	blx	r5
 800479a:	2800      	cmp	r0, #0
 800479c:	d0f8      	beq.n	8004790 <_defmt_write+0xf4>
 800479e:	e797      	b.n	80046d0 <_defmt_write+0x34>
 80047a0:	f898 1004 	ldrb.w	r1, [r8, #4]
 80047a4:	2301      	movs	r3, #1
 80047a6:	fa03 f202 	lsl.w	r2, r3, r2
 80047aa:	4311      	orrs	r1, r2
 80047ac:	f888 1004 	strb.w	r1, [r8, #4]
 80047b0:	3001      	adds	r0, #1
 80047b2:	f888 0003 	strb.w	r0, [r8, #3]
 80047b6:	b2c2      	uxtb	r2, r0
 80047b8:	2a07      	cmp	r2, #7
 80047ba:	d18f      	bne.n	80046dc <_defmt_write+0x40>
 80047bc:	060a      	lsls	r2, r1, #24
 80047be:	d08d      	beq.n	80046dc <_defmt_write+0x40>
 80047c0:	f88d 1008 	strb.w	r1, [sp, #8]
 80047c4:	465d      	mov	r5, fp
 80047c6:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 80047ca:	f000 0003 	and.w	r0, r0, #3
 80047ce:	2802      	cmp	r0, #2
 80047d0:	f244 4031 	movw	r0, #17457	@ 0x4431
 80047d4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80047d8:	bf08      	it	eq
 80047da:	4605      	moveq	r5, r0
 80047dc:	f109 0018 	add.w	r0, r9, #24
 80047e0:	a902      	add	r1, sp, #8
 80047e2:	2201      	movs	r2, #1
 80047e4:	47a8      	blx	r5
 80047e6:	2800      	cmp	r0, #0
 80047e8:	d0f8      	beq.n	80047dc <_defmt_write+0x140>
 80047ea:	e771      	b.n	80046d0 <_defmt_write+0x34>
 80047ec:	3079      	adds	r0, #121	@ 0x79
 80047ee:	465d      	mov	r5, fp
 80047f0:	f040 0080 	orr.w	r0, r0, #128	@ 0x80
 80047f4:	f807 0c1f 	strb.w	r0, [r7, #-31]
 80047f8:	f8d9 002c 	ldr.w	r0, [r9, #44]	@ 0x2c
 80047fc:	f000 0003 	and.w	r0, r0, #3
 8004800:	2802      	cmp	r0, #2
 8004802:	f244 4031 	movw	r0, #17457	@ 0x4431
 8004806:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800480a:	bf08      	it	eq
 800480c:	4605      	moveq	r5, r0
 800480e:	f109 0018 	add.w	r0, r9, #24
 8004812:	f1a7 011f 	sub.w	r1, r7, #31
 8004816:	2201      	movs	r2, #1
 8004818:	47a8      	blx	r5
 800481a:	2800      	cmp	r0, #0
 800481c:	d0f7      	beq.n	800480e <_defmt_write+0x172>
 800481e:	e757      	b.n	80046d0 <_defmt_write+0x34>
 8004820:	b003      	add	sp, #12
 8004822:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8004826:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8004828:	f24a 636c 	movw	r3, #42604	@ 0xa66c
 800482c:	2101      	movs	r1, #1
 800482e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8004832:	2201      	movs	r2, #1
 8004834:	f7fe ff55 	bl	80036e2 <core::slice::index::slice_index_fail>

08004838 <ETH>:
 8004838:	b5b0      	push	{r4, r5, r7, lr}
 800483a:	af02      	add	r7, sp, #8
 800483c:	f003 ffd3 	bl	80087e6 <__primask_r>
 8004840:	4604      	mov	r4, r0
 8004842:	f003 ffc3 	bl	80087cc <__cpsid>
 8004846:	07e0      	lsls	r0, r4, #31
 8004848:	bf08      	it	eq
 800484a:	f003 ffc1 	bleq	80087d0 <__cpsie>
 800484e:	f249 0014 	movw	r0, #36884	@ 0x9014
 8004852:	2141      	movs	r1, #65	@ 0x41
 8004854:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 8004858:	f240 0440 	movw	r4, #64	@ 0x40
 800485c:	6805      	ldr	r5, [r0, #0]
 800485e:	f2c0 0101 	movt	r1, #1
 8004862:	f2c2 0400 	movt	r4, #8192	@ 0x2000
 8004866:	6001      	str	r1, [r0, #0]
 8004868:	07e8      	lsls	r0, r5, #31
 800486a:	d01d      	beq.n	80048a8 <ETH+0x70>
 800486c:	f3bf 8f5f 	dmb	sy
 8004870:	e854 0f09 	ldrex	r0, [r4, #36]	@ 0x24
 8004874:	f040 0102 	orr.w	r1, r0, #2
 8004878:	e844 1209 	strex	r2, r1, [r4, #36]	@ 0x24
 800487c:	2a00      	cmp	r2, #0
 800487e:	d1f7      	bne.n	8004870 <ETH+0x38>
 8004880:	f3bf 8f5f 	dmb	sy
 8004884:	b980      	cbnz	r0, 80048a8 <ETH+0x70>
 8004886:	e9d4 1007 	ldrd	r1, r0, [r4, #28]
 800488a:	2200      	movs	r2, #0
 800488c:	61e2      	str	r2, [r4, #28]
 800488e:	f3bf 8f5f 	dmb	sy
 8004892:	e854 2f09 	ldrex	r2, [r4, #36]	@ 0x24
 8004896:	f022 0202 	bic.w	r2, r2, #2
 800489a:	e844 2309 	strex	r3, r2, [r4, #36]	@ 0x24
 800489e:	2b00      	cmp	r3, #0
 80048a0:	d1f7      	bne.n	8004892 <ETH+0x5a>
 80048a2:	b109      	cbz	r1, 80048a8 <ETH+0x70>
 80048a4:	6849      	ldr	r1, [r1, #4]
 80048a6:	4788      	blx	r1
 80048a8:	0668      	lsls	r0, r5, #25
 80048aa:	d51e      	bpl.n	80048ea <ETH+0xb2>
 80048ac:	f3bf 8f5f 	dmb	sy
 80048b0:	e854 0f06 	ldrex	r0, [r4, #24]
 80048b4:	f040 0102 	orr.w	r1, r0, #2
 80048b8:	e844 1206 	strex	r2, r1, [r4, #24]
 80048bc:	2a00      	cmp	r2, #0
 80048be:	d1f7      	bne.n	80048b0 <ETH+0x78>
 80048c0:	f3bf 8f5f 	dmb	sy
 80048c4:	b988      	cbnz	r0, 80048ea <ETH+0xb2>
 80048c6:	e9d4 1004 	ldrd	r1, r0, [r4, #16]
 80048ca:	2200      	movs	r2, #0
 80048cc:	6122      	str	r2, [r4, #16]
 80048ce:	f3bf 8f5f 	dmb	sy
 80048d2:	e854 2f06 	ldrex	r2, [r4, #24]
 80048d6:	f022 0202 	bic.w	r2, r2, #2
 80048da:	e844 2306 	strex	r3, r2, [r4, #24]
 80048de:	2b00      	cmp	r3, #0
 80048e0:	d1f7      	bne.n	80048d2 <ETH+0x9a>
 80048e2:	2900      	cmp	r1, #0
 80048e4:	bf1c      	itt	ne
 80048e6:	6849      	ldrne	r1, [r1, #4]
 80048e8:	4788      	blxne	r1
 80048ea:	f248 0138 	movw	r1, #32824	@ 0x8038
 80048ee:	f2c4 0102 	movt	r1, #16386	@ 0x4002
 80048f2:	6808      	ldr	r0, [r1, #0]
 80048f4:	0580      	lsls	r0, r0, #22
 80048f6:	bf44      	itt	mi
 80048f8:	f44f 7000 	movmi.w	r0, #512	@ 0x200
 80048fc:	6048      	strmi	r0, [r1, #4]
 80048fe:	f3bf 8f5f 	dmb	sy
 8004902:	e854 0f0c 	ldrex	r0, [r4, #48]	@ 0x30
 8004906:	f040 0202 	orr.w	r2, r0, #2
 800490a:	e844 230c 	strex	r3, r2, [r4, #48]	@ 0x30
 800490e:	2b00      	cmp	r3, #0
 8004910:	d1f7      	bne.n	8004902 <ETH+0xca>
 8004912:	f3bf 8f5f 	dmb	sy
 8004916:	b110      	cbz	r0, 800491e <ETH+0xe6>
 8004918:	f8d1 06f0 	ldr.w	r0, [r1, #1776]	@ 0x6f0
 800491c:	bdb0      	pop	{r4, r5, r7, pc}
 800491e:	e9d4 200a 	ldrd	r2, r0, [r4, #40]	@ 0x28
 8004922:	2300      	movs	r3, #0
 8004924:	62a3      	str	r3, [r4, #40]	@ 0x28
 8004926:	f3bf 8f5f 	dmb	sy
 800492a:	e854 3f0c 	ldrex	r3, [r4, #48]	@ 0x30
 800492e:	f023 0302 	bic.w	r3, r3, #2
 8004932:	e844 350c 	strex	r5, r3, [r4, #48]	@ 0x30
 8004936:	2d00      	cmp	r5, #0
 8004938:	d1f7      	bne.n	800492a <ETH+0xf2>
 800493a:	2a00      	cmp	r2, #0
 800493c:	d0ec      	beq.n	8004918 <ETH+0xe0>
 800493e:	6851      	ldr	r1, [r2, #4]
 8004940:	e8bd 40b0 	ldmia.w	sp!, {r4, r5, r7, lr}
 8004944:	4708      	bx	r1

08004946 <SysTick>:
 8004946:	b5d0      	push	{r4, r6, r7, lr}
 8004948:	af02      	add	r7, sp, #8
 800494a:	f003 ff4c 	bl	80087e6 <__primask_r>
 800494e:	4604      	mov	r4, r0
 8004950:	f003 ff3c 	bl	80087cc <__cpsid>
 8004954:	f240 0040 	movw	r0, #64	@ 0x40
 8004958:	f2c2 0000 	movt	r0, #8192	@ 0x2000
 800495c:	e9d0 1202 	ldrd	r1, r2, [r0, #8]
 8004960:	3101      	adds	r1, #1
 8004962:	f142 0200 	adc.w	r2, r2, #0
 8004966:	e9c0 1202 	strd	r1, r2, [r0, #8]
 800496a:	07e0      	lsls	r0, r4, #31
 800496c:	bf18      	it	ne
 800496e:	bdd0      	popne	{r4, r6, r7, pc}
 8004970:	e8bd 40d0 	ldmia.w	sp!, {r4, r6, r7, lr}
 8004974:	f003 bf2c 	b.w	80087d0 <__cpsie>

08004978 <main>:
 8004978:	b580      	push	{r7, lr}
 800497a:	466f      	mov	r7, sp
 800497c:	f000 f800 	bl	8004980 <ip::__cortex_m_rt_main>

08004980 <ip::__cortex_m_rt_main>:
 8004980:	b5b0      	push	{r4, r5, r7, lr}
 8004982:	af02      	add	r7, sp, #8
 8004984:	f5ad 5d01 	sub.w	sp, sp, #8256	@ 0x2040
 8004988:	f003 ff2d 	bl	80087e6 <__primask_r>
 800498c:	4604      	mov	r4, r0
 800498e:	f003 ff1d 	bl	80087cc <__cpsid>
 8004992:	f240 0540 	movw	r5, #64	@ 0x40
 8004996:	07e1      	lsls	r1, r4, #31
 8004998:	f2c2 0500 	movt	r5, #8192	@ 0x2000
 800499c:	79a8      	ldrb	r0, [r5, #6]
 800499e:	d107      	bne.n	80049b0 <ip::__cortex_m_rt_main+0x30>
 80049a0:	07c0      	lsls	r0, r0, #31
 80049a2:	f041 82b2 	bne.w	8005f0a <ip::__cortex_m_rt_main+0x158a>
 80049a6:	2001      	movs	r0, #1
 80049a8:	71a8      	strb	r0, [r5, #6]
 80049aa:	f003 ff11 	bl	80087d0 <__cpsie>
 80049ae:	e004      	b.n	80049ba <ip::__cortex_m_rt_main+0x3a>
 80049b0:	07c0      	lsls	r0, r0, #31
 80049b2:	f041 82ac 	bne.w	8005f0e <ip::__cortex_m_rt_main+0x158e>
 80049b6:	2001      	movs	r0, #1
 80049b8:	71a8      	strb	r0, [r5, #6]
 80049ba:	f003 ff14 	bl	80087e6 <__primask_r>
 80049be:	4604      	mov	r4, r0
 80049c0:	f003 ff04 	bl	80087cc <__cpsid>
 80049c4:	7968      	ldrb	r0, [r5, #5]
 80049c6:	07e1      	lsls	r1, r4, #31
 80049c8:	d107      	bne.n	80049da <ip::__cortex_m_rt_main+0x5a>
 80049ca:	2800      	cmp	r0, #0
 80049cc:	f041 82a5 	bne.w	8005f1a <ip::__cortex_m_rt_main+0x159a>
 80049d0:	2001      	movs	r0, #1
 80049d2:	7168      	strb	r0, [r5, #5]
 80049d4:	f003 fefc 	bl	80087d0 <__cpsie>
 80049d8:	e004      	b.n	80049e4 <ip::__cortex_m_rt_main+0x64>
 80049da:	2800      	cmp	r0, #0
 80049dc:	f041 829f 	bne.w	8005f1e <ip::__cortex_m_rt_main+0x159e>
 80049e0:	2001      	movs	r0, #1
 80049e2:	7168      	strb	r0, [r5, #5]
 80049e4:	f10d 0e18 	add.w	lr, sp, #24
 80049e8:	f64d 0c00 	movw	ip, #55296	@ 0xd800
 80049ec:	f24f 0480 	movw	r4, #61568	@ 0xf080
 80049f0:	f64c 4300 	movw	r3, #52224	@ 0xcc00
 80049f4:	f245 0903 	movw	r9, #20483	@ 0x5003
 80049f8:	f24a 0b07 	movw	fp, #40967	@ 0xa007
 80049fc:	f50e 5097 	add.w	r0, lr, #4832	@ 0x12e0
 8004a00:	f24a 1e20 	movw	lr, #41248	@ 0xa120
 8004a04:	f2c0 0e07 	movt	lr, #7
 8004a08:	f2c0 5cb8 	movt	ip, #1464	@ 0x5b8
 8004a0c:	f04f 0a02 	mov.w	sl, #2
 8004a10:	f04f 4800 	mov.w	r8, #2147483648	@ 0x80000000
 8004a14:	f2c0 24fa 	movt	r4, #762	@ 0x2fa
 8004a18:	f6c1 13bf 	movt	r3, #6591	@ 0x19bf
 8004a1c:	f2cf 498e 	movt	r9, #62606	@ 0xf48e
 8004a20:	f6ce 1b1c 	movt	fp, #59676	@ 0xe91c
 8004a24:	f8c0 eaf0 	str.w	lr, [r0, #2800]	@ 0xaf0
 8004a28:	f8cd cca0 	str.w	ip, [sp, #3232]	@ 0xca0
 8004a2c:	e001      	b.n	8004a32 <ip::__cortex_m_rt_main+0xb2>
 8004a2e:	f10a 0a01 	add.w	sl, sl, #1
 8004a32:	f1ba 0f09 	cmp.w	sl, #9
 8004a36:	f001 8248 	beq.w	8005eca <ip::__cortex_m_rt_main+0x154a>
 8004a3a:	f1ba 0f04 	cmp.w	sl, #4
 8004a3e:	d3f6      	bcc.n	8004a2e <ip::__cortex_m_rt_main+0xae>
 8004a40:	fbb8 f0fa 	udiv	r0, r8, sl
 8004a44:	f44f 72d8 	mov.w	r2, #432	@ 0x1b0
 8004a48:	3001      	adds	r0, #1
 8004a4a:	0840      	lsrs	r0, r0, #1
 8004a4c:	fba0 010e 	umull	r0, r1, r0, lr
 8004a50:	e003      	b.n	8004a5a <ip::__cortex_m_rt_main+0xda>
 8004a52:	1e55      	subs	r5, r2, #1
 8004a54:	2a33      	cmp	r2, #51	@ 0x33
 8004a56:	462a      	mov	r2, r5
 8004a58:	d3e9      	bcc.n	8004a2e <ip::__cortex_m_rt_main+0xae>
 8004a5a:	fba0 5602 	umull	r5, r6, r0, r2
 8004a5e:	fb01 6602 	mla	r6, r1, r2, r6
 8004a62:	0ead      	lsrs	r5, r5, #26
 8004a64:	ea45 1586 	orr.w	r5, r5, r6, lsl #6
 8004a68:	42a5      	cmp	r5, r4
 8004a6a:	d3e0      	bcc.n	8004a2e <ip::__cortex_m_rt_main+0xae>
 8004a6c:	f025 050f 	bic.w	r5, r5, #15
 8004a70:	429d      	cmp	r5, r3
 8004a72:	d8ee      	bhi.n	8004a52 <ip::__cortex_m_rt_main+0xd2>
 8004a74:	eb05 060b 	add.w	r6, r5, fp
 8004a78:	2e0f      	cmp	r6, #15
 8004a7a:	bf24      	itt	cs
 8004a7c:	444d      	addcs	r5, r9
 8004a7e:	2d07      	cmpcs	r5, #7
 8004a80:	d2e7      	bcs.n	8004a52 <ip::__cortex_m_rt_main+0xd2>
 8004a82:	ea5f 600a 	movs.w	r0, sl, lsl #24
 8004a86:	d105      	bne.n	8004a94 <ip::__cortex_m_rt_main+0x114>
 8004a88:	f24b 00d0 	movw	r0, #45264	@ 0xb0d0
 8004a8c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8004a90:	f7ff f9f3 	bl	8003e7a <core::panicking::panic_const::panic_const_div_by_zero>
 8004a94:	fa5f f08a 	uxtb.w	r0, sl
 8004a98:	f04f 4100 	mov.w	r1, #2147483648	@ 0x80000000
 8004a9c:	fbb1 f0f0 	udiv	r0, r1, r0
 8004aa0:	b293      	uxth	r3, r2
 8004aa2:	3001      	adds	r0, #1
 8004aa4:	2e0f      	cmp	r6, #15
 8004aa6:	ea4f 0050 	mov.w	r0, r0, lsr #1
 8004aaa:	fba0 010e 	umull	r0, r1, r0, lr
 8004aae:	fba0 0503 	umull	r0, r5, r0, r3
 8004ab2:	fb01 5103 	mla	r1, r1, r3, r5
 8004ab6:	f04f 051d 	mov.w	r5, #29
 8004aba:	bf38      	it	cc
 8004abc:	251c      	movcc	r5, #28
 8004abe:	f085 041f 	eor.w	r4, r5, #31
 8004ac2:	0f80      	lsrs	r0, r0, #30
 8004ac4:	ea40 0081 	orr.w	r0, r0, r1, lsl #2
 8004ac8:	0f8b      	lsrs	r3, r1, #30
 8004aca:	40ab      	lsls	r3, r5
 8004acc:	0841      	lsrs	r1, r0, #1
 8004ace:	40a8      	lsls	r0, r5
 8004ad0:	40e1      	lsrs	r1, r4
 8004ad2:	0e80      	lsrs	r0, r0, #26
 8004ad4:	ea40 1081 	orr.w	r0, r0, r1, lsl #6
 8004ad8:	430b      	orrs	r3, r1
 8004ada:	f020 010c 	bic.w	r1, r0, #12
 8004ade:	f24e 6001 	movw	r0, #58881	@ 0xe601
 8004ae2:	f6c0 40df 	movt	r0, #3295	@ 0xcdf
 8004ae6:	0e9b      	lsrs	r3, r3, #26
 8004ae8:	1a08      	subs	r0, r1, r0
 8004aea:	f173 0000 	sbcs.w	r0, r3, #0
 8004aee:	f081 81b9 	bcs.w	8005e64 <ip::__cortex_m_rt_main+0x14e4>
 8004af2:	ee00 1a10 	vmov	s0, r1
 8004af6:	4561      	cmp	r1, ip
 8004af8:	bf38      	it	cc
 8004afa:	468c      	movcc	ip, r1
 8004afc:	eeb8 1a40 	vcvt.f32.u32	s2, s0
 8004b00:	ee00 ca10 	vmov	s0, ip
 8004b04:	eeb8 0a40 	vcvt.f32.u32	s0, s0
 8004b08:	eeb1 2a41 	vneg.f32	s4, s2
 8004b0c:	ee82 2a00 	vdiv.f32	s4, s4, s0
 8004b10:	eebd 3ac2 	vcvt.s32.f32	s6, s4
 8004b14:	eebf 0a00 	vmov.f32	s0, #240	@ 0xbf800000 -1.0
 8004b18:	eeb8 3ac3 	vcvt.f32.s32	s6, s6
 8004b1c:	eeb4 2a43 	vcmp.f32	s4, s6
 8004b20:	ee33 4a00 	vadd.f32	s8, s6, s0
 8004b24:	eef1 fa10 	vmrs	APSR_nzcv, fpscr
 8004b28:	bf48      	it	mi
 8004b2a:	eeb0 3a44 	vmovmi.f32	s6, s8
 8004b2e:	eeb1 2a43 	vneg.f32	s4, s6
 8004b32:	eebc 2ac2 	vcvt.u32.f32	s4, s4
 8004b36:	ee12 0a10 	vmov	r0, s4
 8004b3a:	2801      	cmp	r0, #1
 8004b3c:	d011      	beq.n	8004b62 <ip::__cortex_m_rt_main+0x1e2>
 8004b3e:	2802      	cmp	r0, #2
 8004b40:	d00a      	beq.n	8004b58 <ip::__cortex_m_rt_main+0x1d8>
 8004b42:	2800      	cmp	r0, #0
 8004b44:	f001 8199 	beq.w	8005e7a <ip::__cortex_m_rt_main+0x14fa>
 8004b48:	1ec3      	subs	r3, r0, #3
 8004b4a:	2b03      	cmp	r3, #3
 8004b4c:	d26f      	bcs.n	8004c2e <ip::__cortex_m_rt_main+0x2ae>
 8004b4e:	eeb1 2a00 	vmov.f32	s4, #16	@ 0x40800000  4.0
 8004b52:	f04f 0e90 	mov.w	lr, #144	@ 0x90
 8004b56:	e008      	b.n	8004b6a <ip::__cortex_m_rt_main+0x1ea>
 8004b58:	eeb0 2a00 	vmov.f32	s4, #0	@ 0x40000000  2.0
 8004b5c:	f04f 0e80 	mov.w	lr, #128	@ 0x80
 8004b60:	e003      	b.n	8004b6a <ip::__cortex_m_rt_main+0x1ea>
 8004b62:	eeb7 2a00 	vmov.f32	s4, #112	@ 0x3f800000  1.0
 8004b66:	f04f 0e00 	mov.w	lr, #0
 8004b6a:	ee81 1a02 	vdiv.f32	s2, s2, s4
 8004b6e:	f249 5c00 	movw	ip, #38144	@ 0x9500
 8004b72:	f64f 1080 	movw	r0, #63872	@ 0xf980
 8004b76:	f24f 3300 	movw	r3, #62208	@ 0xf300
 8004b7a:	f6c0 2cba 	movt	ip, #2746	@ 0xaba
 8004b7e:	f10c 0901 	add.w	r9, ip, #1
 8004b82:	f2c0 3037 	movt	r0, #823	@ 0x337
 8004b86:	f2c0 636f 	movt	r3, #1647	@ 0x66f
 8004b8a:	eebd 2ac1 	vcvt.s32.f32	s4, s2
 8004b8e:	eeb8 2ac2 	vcvt.f32.s32	s4, s4
 8004b92:	eeb4 1a42 	vcmp.f32	s2, s4
 8004b96:	ee32 3a00 	vadd.f32	s6, s4, s0
 8004b9a:	eef1 fa10 	vmrs	APSR_nzcv, fpscr
 8004b9e:	bf48      	it	mi
 8004ba0:	eeb0 2a43 	vmovmi.f32	s4, s6
 8004ba4:	eebc 1ac2 	vcvt.u32.f32	s2, s4
 8004ba8:	4549      	cmp	r1, r9
 8004baa:	bf3c      	itt	cc
 8004bac:	f24a 5040 	movwcc	r0, #42304	@ 0xa540
 8004bb0:	f2c0 20ae 	movtcc	r0, #686	@ 0x2ae
 8004bb4:	bf3c      	itt	cc
 8004bb6:	f644 2380 	movwcc	r3, #19072	@ 0x4a80
 8004bba:	f2c0 535d 	movtcc	r3, #1373	@ 0x55d
 8004bbe:	ee11 5a10 	vmov	r5, s2
 8004bc2:	eeb8 1a41 	vcvt.f32.u32	s2, s2
 8004bc6:	eeb1 1a41 	vneg.f32	s2, s2
 8004bca:	4285      	cmp	r5, r0
 8004bcc:	bf38      	it	cc
 8004bce:	4628      	movcc	r0, r5
 8004bd0:	ee02 0a10 	vmov	s4, r0
 8004bd4:	eeb8 2a42 	vcvt.f32.u32	s4, s4
 8004bd8:	ee81 2a02 	vdiv.f32	s4, s2, s4
 8004bdc:	eebd 3ac2 	vcvt.s32.f32	s6, s4
 8004be0:	eeb8 3ac3 	vcvt.f32.s32	s6, s6
 8004be4:	eeb4 2a43 	vcmp.f32	s4, s6
 8004be8:	ee33 4a00 	vadd.f32	s8, s6, s0
 8004bec:	eef1 fa10 	vmrs	APSR_nzcv, fpscr
 8004bf0:	bf48      	it	mi
 8004bf2:	eeb0 3a44 	vmovmi.f32	s6, s8
 8004bf6:	eeb1 2a43 	vneg.f32	s4, s6
 8004bfa:	429d      	cmp	r5, r3
 8004bfc:	bf38      	it	cc
 8004bfe:	462b      	movcc	r3, r5
 8004c00:	eebc 2ac2 	vcvt.u32.f32	s4, s4
 8004c04:	ee12 0a10 	vmov	r0, s4
 8004c08:	2801      	cmp	r0, #1
 8004c0a:	d00d      	beq.n	8004c28 <ip::__cortex_m_rt_main+0x2a8>
 8004c0c:	2802      	cmp	r0, #2
 8004c0e:	d008      	beq.n	8004c22 <ip::__cortex_m_rt_main+0x2a2>
 8004c10:	2800      	cmp	r0, #0
 8004c12:	f001 813d 	beq.w	8005e90 <ip::__cortex_m_rt_main+0x1510>
 8004c16:	1ec4      	subs	r4, r0, #3
 8004c18:	2c04      	cmp	r4, #4
 8004c1a:	d210      	bcs.n	8004c3e <ip::__cortex_m_rt_main+0x2be>
 8004c1c:	f44f 58a0 	mov.w	r8, #5120	@ 0x1400
 8004c20:	e014      	b.n	8004c4c <ip::__cortex_m_rt_main+0x2cc>
 8004c22:	f44f 5880 	mov.w	r8, #4096	@ 0x1000
 8004c26:	e011      	b.n	8004c4c <ip::__cortex_m_rt_main+0x2cc>
 8004c28:	f04f 0800 	mov.w	r8, #0
 8004c2c:	e00e      	b.n	8004c4c <ip::__cortex_m_rt_main+0x2cc>
 8004c2e:	1f83      	subs	r3, r0, #6
 8004c30:	2b06      	cmp	r3, #6
 8004c32:	d272      	bcs.n	8004d1a <ip::__cortex_m_rt_main+0x39a>
 8004c34:	eeb2 2a00 	vmov.f32	s4, #32	@ 0x41000000  8.0
 8004c38:	f04f 0ea0 	mov.w	lr, #160	@ 0xa0
 8004c3c:	e795      	b.n	8004b6a <ip::__cortex_m_rt_main+0x1ea>
 8004c3e:	3807      	subs	r0, #7
 8004c40:	f44f 58e0 	mov.w	r8, #7168	@ 0x1c00
 8004c44:	2806      	cmp	r0, #6
 8004c46:	bf38      	it	cc
 8004c48:	f44f 58c0 	movcc.w	r8, #6144	@ 0x1800
 8004c4c:	ee02 3a10 	vmov	s4, r3
 8004c50:	eeb8 2a42 	vcvt.f32.u32	s4, s4
 8004c54:	ee81 1a02 	vdiv.f32	s2, s2, s4
 8004c58:	eebd 2ac1 	vcvt.s32.f32	s4, s2
 8004c5c:	eeb8 2ac2 	vcvt.f32.s32	s4, s4
 8004c60:	eeb4 1a42 	vcmp.f32	s2, s4
 8004c64:	ee32 0a00 	vadd.f32	s0, s4, s0
 8004c68:	eef1 fa10 	vmrs	APSR_nzcv, fpscr
 8004c6c:	bf48      	it	mi
 8004c6e:	eeb0 2a40 	vmovmi.f32	s4, s0
 8004c72:	eeb1 0a42 	vneg.f32	s0, s4
 8004c76:	9507      	str	r5, [sp, #28]
 8004c78:	eebc 0ac0 	vcvt.u32.f32	s0, s0
 8004c7c:	ee10 0a10 	vmov	r0, s0
 8004c80:	2801      	cmp	r0, #1
 8004c82:	d019      	beq.n	8004cb8 <ip::__cortex_m_rt_main+0x338>
 8004c84:	2802      	cmp	r0, #2
 8004c86:	d00e      	beq.n	8004ca6 <ip::__cortex_m_rt_main+0x326>
 8004c88:	2800      	cmp	r0, #0
 8004c8a:	f001 810c 	beq.w	8005ea6 <ip::__cortex_m_rt_main+0x1526>
 8004c8e:	1ec3      	subs	r3, r0, #3
 8004c90:	2b04      	cmp	r3, #4
 8004c92:	d21b      	bcs.n	8004ccc <ip::__cortex_m_rt_main+0x34c>
 8004c94:	f44f 4b20 	mov.w	fp, #40960	@ 0xa000
 8004c98:	f24c 3081 	movw	r0, #50049	@ 0xc381
 8004c9c:	f2c0 10c9 	movt	r0, #457	@ 0x1c9
 8004ca0:	4281      	cmp	r1, r0
 8004ca2:	d311      	bcc.n	8004cc8 <ip::__cortex_m_rt_main+0x348>
 8004ca4:	e01f      	b.n	8004ce6 <ip::__cortex_m_rt_main+0x366>
 8004ca6:	f44f 4b00 	mov.w	fp, #32768	@ 0x8000
 8004caa:	f24c 3081 	movw	r0, #50049	@ 0xc381
 8004cae:	f2c0 10c9 	movt	r0, #457	@ 0x1c9
 8004cb2:	4281      	cmp	r1, r0
 8004cb4:	d308      	bcc.n	8004cc8 <ip::__cortex_m_rt_main+0x348>
 8004cb6:	e016      	b.n	8004ce6 <ip::__cortex_m_rt_main+0x366>
 8004cb8:	f04f 0b00 	mov.w	fp, #0
 8004cbc:	f24c 3081 	movw	r0, #50049	@ 0xc381
 8004cc0:	f2c0 10c9 	movt	r0, #457	@ 0x1c9
 8004cc4:	4281      	cmp	r1, r0
 8004cc6:	d20e      	bcs.n	8004ce6 <ip::__cortex_m_rt_main+0x366>
 8004cc8:	2300      	movs	r3, #0
 8004cca:	e043      	b.n	8004d54 <ip::__cortex_m_rt_main+0x3d4>
 8004ccc:	3807      	subs	r0, #7
 8004cce:	f44f 4b60 	mov.w	fp, #57344	@ 0xe000
 8004cd2:	2806      	cmp	r0, #6
 8004cd4:	bf38      	it	cc
 8004cd6:	f44f 4b40 	movcc.w	fp, #49152	@ 0xc000
 8004cda:	f24c 3081 	movw	r0, #50049	@ 0xc381
 8004cde:	f2c0 10c9 	movt	r0, #457	@ 0x1c9
 8004ce2:	4281      	cmp	r1, r0
 8004ce4:	d3f0      	bcc.n	8004cc8 <ip::__cortex_m_rt_main+0x348>
 8004ce6:	f248 7000 	movw	r0, #34560	@ 0x8700
 8004cea:	f2c0 3093 	movt	r0, #915	@ 0x393
 8004cee:	3001      	adds	r0, #1
 8004cf0:	4281      	cmp	r1, r0
 8004cf2:	d201      	bcs.n	8004cf8 <ip::__cortex_m_rt_main+0x378>
 8004cf4:	2301      	movs	r3, #1
 8004cf6:	e02d      	b.n	8004d54 <ip::__cortex_m_rt_main+0x3d4>
 8004cf8:	f644 2080 	movw	r0, #19072	@ 0x4a80
 8004cfc:	f2c0 505d 	movt	r0, #1373	@ 0x55d
 8004d00:	3001      	adds	r0, #1
 8004d02:	4281      	cmp	r1, r0
 8004d04:	d201      	bcs.n	8004d0a <ip::__cortex_m_rt_main+0x38a>
 8004d06:	2302      	movs	r3, #2
 8004d08:	e024      	b.n	8004d54 <ip::__cortex_m_rt_main+0x3d4>
 8004d0a:	f640 6001 	movw	r0, #3585	@ 0xe01
 8004d0e:	f2c0 7027 	movt	r0, #1831	@ 0x727
 8004d12:	4281      	cmp	r1, r0
 8004d14:	d20b      	bcs.n	8004d2e <ip::__cortex_m_rt_main+0x3ae>
 8004d16:	2303      	movs	r3, #3
 8004d18:	e01c      	b.n	8004d54 <ip::__cortex_m_rt_main+0x3d4>
 8004d1a:	f1a0 030c 	sub.w	r3, r0, #12
 8004d1e:	2b1c      	cmp	r3, #28
 8004d20:	f080 850f 	bcs.w	8005742 <ip::__cortex_m_rt_main+0xdc2>
 8004d24:	eeb3 2a00 	vmov.f32	s4, #48	@ 0x41800000  16.0
 8004d28:	f04f 0eb0 	mov.w	lr, #176	@ 0xb0
 8004d2c:	e71d      	b.n	8004b6a <ip::__cortex_m_rt_main+0x1ea>
 8004d2e:	f24d 1080 	movw	r0, #53632	@ 0xd180
 8004d32:	2305      	movs	r3, #5
 8004d34:	f6c0 00f0 	movt	r0, #2288	@ 0x8f0
 8004d38:	3001      	adds	r0, #1
 8004d3a:	4281      	cmp	r1, r0
 8004d3c:	bf38      	it	cc
 8004d3e:	2304      	movcc	r3, #4
 8004d40:	4549      	cmp	r1, r9
 8004d42:	d307      	bcc.n	8004d54 <ip::__cortex_m_rt_main+0x3d4>
 8004d44:	f645 0081 	movw	r0, #22657	@ 0x5881
 8004d48:	2307      	movs	r3, #7
 8004d4a:	f6c0 4084 	movt	r0, #3204	@ 0xc84
 8004d4e:	4281      	cmp	r1, r0
 8004d50:	bf38      	it	cc
 8004d52:	2306      	movcc	r3, #6
 8004d54:	f643 0910 	movw	r9, #14352	@ 0x3810
 8004d58:	f2c4 0902 	movt	r9, #16386	@ 0x4002
 8004d5c:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004d60:	f040 0001 	orr.w	r0, r0, #1
 8004d64:	f849 0c10 	str.w	r0, [r9, #-16]
 8004d68:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004d6c:	0780      	lsls	r0, r0, #30
 8004d6e:	d40c      	bmi.n	8004d8a <ip::__cortex_m_rt_main+0x40a>
 8004d70:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004d74:	0780      	lsls	r0, r0, #30
 8004d76:	bf5c      	itt	pl
 8004d78:	f859 0c10 	ldrpl.w	r0, [r9, #-16]
 8004d7c:	ea5f 7080 	movspl.w	r0, r0, lsl #30
 8004d80:	d403      	bmi.n	8004d8a <ip::__cortex_m_rt_main+0x40a>
 8004d82:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004d86:	0780      	lsls	r0, r0, #30
 8004d88:	d5ee      	bpl.n	8004d68 <ip::__cortex_m_rt_main+0x3e8>
 8004d8a:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8004d8e:	2500      	movs	r5, #0
 8004d90:	f020 0003 	bic.w	r0, r0, #3
 8004d94:	f849 0c08 	str.w	r0, [r9, #-8]
 8004d98:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004d9c:	f440 2080 	orr.w	r0, r0, #262144	@ 0x40000
 8004da0:	f849 0c10 	str.w	r0, [r9, #-16]
 8004da4:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004da8:	f440 3080 	orr.w	r0, r0, #65536	@ 0x10000
 8004dac:	f849 0c10 	str.w	r0, [r9, #-16]
 8004db0:	f647 2001 	movw	r0, #31233	@ 0x7a01
 8004db4:	f6c0 2003 	movt	r0, #2563	@ 0xa03
 8004db8:	4281      	cmp	r1, r0
 8004dba:	f244 4001 	movw	r0, #17409	@ 0x4401
 8004dbe:	bf38      	it	cc
 8004dc0:	2501      	movcc	r5, #1
 8004dc2:	f6c0 0095 	movt	r0, #2197	@ 0x895
 8004dc6:	4281      	cmp	r1, r0
 8004dc8:	bf38      	it	cc
 8004dca:	2502      	movcc	r5, #2
 8004dcc:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004dd0:	0380      	lsls	r0, r0, #14
 8004dd2:	d40c      	bmi.n	8004dee <ip::__cortex_m_rt_main+0x46e>
 8004dd4:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004dd8:	0380      	lsls	r0, r0, #14
 8004dda:	bf5c      	itt	pl
 8004ddc:	f859 0c10 	ldrpl.w	r0, [r9, #-16]
 8004de0:	ea5f 3080 	movspl.w	r0, r0, lsl #14
 8004de4:	d403      	bmi.n	8004dee <ip::__cortex_m_rt_main+0x46e>
 8004de6:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004dea:	0380      	lsls	r0, r0, #14
 8004dec:	d5ee      	bpl.n	8004dcc <ip::__cortex_m_rt_main+0x44c>
 8004dee:	f859 0c10 	ldr.w	r0, [r9, #-16]
 8004df2:	2e0f      	cmp	r6, #15
 8004df4:	f020 7080 	bic.w	r0, r0, #16777216	@ 0x1000000
 8004df8:	f849 0c10 	str.w	r0, [r9, #-16]
 8004dfc:	f647 70c0 	movw	r0, #32704	@ 0x7fc0
 8004e00:	f859 4c0c 	ldr.w	r4, [r9, #-12]
 8004e04:	ea00 1082 	and.w	r0, r0, r2, lsl #6
 8004e08:	f248 0200 	movw	r2, #32768	@ 0x8000
 8004e0c:	f2cf 02bc 	movt	r2, #61628	@ 0xf0bc
 8004e10:	ea02 0204 	and.w	r2, r2, r4
 8004e14:	f00a 043f 	and.w	r4, sl, #63	@ 0x3f
 8004e18:	4422      	add	r2, r4
 8004e1a:	4410      	add	r0, r2
 8004e1c:	bf38      	it	cc
 8004e1e:	f500 3080 	addcc.w	r0, r0, #65536	@ 0x10000
 8004e22:	2d02      	cmp	r5, #2
 8004e24:	f100 7010 	add.w	r0, r0, #37748736	@ 0x2400000
 8004e28:	f849 0c0c 	str.w	r0, [r9, #-12]
 8004e2c:	f8d9 0030 	ldr.w	r0, [r9, #48]	@ 0x30
 8004e30:	f040 5080 	orr.w	r0, r0, #268435456	@ 0x10000000
 8004e34:	f8c9 0030 	str.w	r0, [r9, #48]	@ 0x30
 8004e38:	f247 0004 	movw	r0, #28676	@ 0x7004
 8004e3c:	f2c4 0000 	movt	r0, #16384	@ 0x4000
 8004e40:	f850 2c04 	ldr.w	r2, [r0, #-4]
 8004e44:	d003      	beq.n	8004e4e <ip::__cortex_m_rt_main+0x4ce>
 8004e46:	2d01      	cmp	r5, #1
 8004e48:	d105      	bne.n	8004e56 <ip::__cortex_m_rt_main+0x4d6>
 8004e4a:	2402      	movs	r4, #2
 8004e4c:	e000      	b.n	8004e50 <ip::__cortex_m_rt_main+0x4d0>
 8004e4e:	2401      	movs	r4, #1
 8004e50:	f364 328f 	bfi	r2, r4, #14, #2
 8004e54:	e001      	b.n	8004e5a <ip::__cortex_m_rt_main+0x4da>
 8004e56:	f442 4240 	orr.w	r2, r2, #49152	@ 0xc000
 8004e5a:	f840 2c04 	str.w	r2, [r0, #-4]
 8004e5e:	f859 2c10 	ldr.w	r2, [r9, #-16]
 8004e62:	f042 7280 	orr.w	r2, r2, #16777216	@ 0x1000000
 8004e66:	f849 2c10 	str.w	r2, [r9, #-16]
 8004e6a:	f859 2c10 	ldr.w	r2, [r9, #-16]
 8004e6e:	0192      	lsls	r2, r2, #6
 8004e70:	d40c      	bmi.n	8004e8c <ip::__cortex_m_rt_main+0x50c>
 8004e72:	f859 2c10 	ldr.w	r2, [r9, #-16]
 8004e76:	0192      	lsls	r2, r2, #6
 8004e78:	bf5c      	itt	pl
 8004e7a:	f859 2c10 	ldrpl.w	r2, [r9, #-16]
 8004e7e:	ea5f 1282 	movspl.w	r2, r2, lsl #6
 8004e82:	d403      	bmi.n	8004e8c <ip::__cortex_m_rt_main+0x50c>
 8004e84:	f859 2c10 	ldr.w	r2, [r9, #-16]
 8004e88:	0192      	lsls	r2, r2, #6
 8004e8a:	d5ee      	bpl.n	8004e6a <ip::__cortex_m_rt_main+0x4ea>
 8004e8c:	4561      	cmp	r1, ip
 8004e8e:	d925      	bls.n	8004edc <ip::__cortex_m_rt_main+0x55c>
 8004e90:	f850 1c04 	ldr.w	r1, [r0, #-4]
 8004e94:	f441 3180 	orr.w	r1, r1, #65536	@ 0x10000
 8004e98:	f840 1c04 	str.w	r1, [r0, #-4]
 8004e9c:	6801      	ldr	r1, [r0, #0]
 8004e9e:	03c9      	lsls	r1, r1, #15
 8004ea0:	d409      	bmi.n	8004eb6 <ip::__cortex_m_rt_main+0x536>
 8004ea2:	6801      	ldr	r1, [r0, #0]
 8004ea4:	03c9      	lsls	r1, r1, #15
 8004ea6:	bf5c      	itt	pl
 8004ea8:	6801      	ldrpl	r1, [r0, #0]
 8004eaa:	ea5f 31c1 	movspl.w	r1, r1, lsl #15
 8004eae:	d402      	bmi.n	8004eb6 <ip::__cortex_m_rt_main+0x536>
 8004eb0:	6801      	ldr	r1, [r0, #0]
 8004eb2:	03c9      	lsls	r1, r1, #15
 8004eb4:	d5f2      	bpl.n	8004e9c <ip::__cortex_m_rt_main+0x51c>
 8004eb6:	f850 1c04 	ldr.w	r1, [r0, #-4]
 8004eba:	f441 3100 	orr.w	r1, r1, #131072	@ 0x20000
 8004ebe:	f840 1c04 	str.w	r1, [r0, #-4]
 8004ec2:	6801      	ldr	r1, [r0, #0]
 8004ec4:	0389      	lsls	r1, r1, #14
 8004ec6:	d409      	bmi.n	8004edc <ip::__cortex_m_rt_main+0x55c>
 8004ec8:	6801      	ldr	r1, [r0, #0]
 8004eca:	0389      	lsls	r1, r1, #14
 8004ecc:	bf5c      	itt	pl
 8004ece:	6801      	ldrpl	r1, [r0, #0]
 8004ed0:	ea5f 3181 	movspl.w	r1, r1, lsl #14
 8004ed4:	d402      	bmi.n	8004edc <ip::__cortex_m_rt_main+0x55c>
 8004ed6:	6801      	ldr	r1, [r0, #0]
 8004ed8:	0389      	lsls	r1, r1, #14
 8004eda:	d5f2      	bpl.n	8004ec2 <ip::__cortex_m_rt_main+0x542>
 8004edc:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8004ee0:	f64f 71ff 	movw	r1, #65535	@ 0xffff
 8004ee4:	f2c0 019f 	movt	r1, #159	@ 0x9f
 8004ee8:	f64f 42f0 	movw	r2, #64752	@ 0xfcf0
 8004eec:	4008      	ands	r0, r1
 8004eee:	f849 0c08 	str.w	r0, [r9, #-8]
 8004ef2:	f8c9 33f0 	str.w	r3, [r9, #1008]	@ 0x3f0
 8004ef6:	ea48 000e 	orr.w	r0, r8, lr
 8004efa:	f859 1c08 	ldr.w	r1, [r9, #-8]
 8004efe:	ea40 000b 	orr.w	r0, r0, fp
 8004f02:	4391      	bics	r1, r2
 8004f04:	4308      	orrs	r0, r1
 8004f06:	f849 0c08 	str.w	r0, [r9, #-8]
 8004f0a:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8004f0e:	2102      	movs	r1, #2
 8004f10:	f361 0001 	bfi	r0, r1, #0, #2
 8004f14:	f849 0c08 	str.w	r0, [r9, #-8]
 8004f18:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8004f1c:	f000 000c 	and.w	r0, r0, #12
 8004f20:	2808      	cmp	r0, #8
 8004f22:	d011      	beq.n	8004f48 <ip::__cortex_m_rt_main+0x5c8>
 8004f24:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8004f28:	f000 000c 	and.w	r0, r0, #12
 8004f2c:	2808      	cmp	r0, #8
 8004f2e:	bf1e      	ittt	ne
 8004f30:	f859 0c08 	ldrne.w	r0, [r9, #-8]
 8004f34:	f000 000c 	andne.w	r0, r0, #12
 8004f38:	2808      	cmpne	r0, #8
 8004f3a:	d005      	beq.n	8004f48 <ip::__cortex_m_rt_main+0x5c8>
 8004f3c:	f859 0c08 	ldr.w	r0, [r9, #-8]
 8004f40:	f000 000c 	and.w	r0, r0, #12
 8004f44:	2808      	cmp	r0, #8
 8004f46:	d1e7      	bne.n	8004f18 <ip::__cortex_m_rt_main+0x598>
 8004f48:	2010      	movs	r0, #16
 8004f4a:	f003 fc43 	bl	80087d4 <__delay>
 8004f4e:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8004f52:	f040 0001 	orr.w	r0, r0, #1
 8004f56:	f8c9 0020 	str.w	r0, [r9, #32]
 8004f5a:	f003 fc41 	bl	80087e0 <__dsb>
 8004f5e:	f8d9 0000 	ldr.w	r0, [r9]
 8004f62:	f040 0001 	orr.w	r0, r0, #1
 8004f66:	f8c9 0000 	str.w	r0, [r9]
 8004f6a:	f8d9 0000 	ldr.w	r0, [r9]
 8004f6e:	f020 0001 	bic.w	r0, r0, #1
 8004f72:	f8c9 0000 	str.w	r0, [r9]
 8004f76:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8004f7a:	f040 0002 	orr.w	r0, r0, #2
 8004f7e:	f8c9 0020 	str.w	r0, [r9, #32]
 8004f82:	f003 fc2d 	bl	80087e0 <__dsb>
 8004f86:	f8d9 0000 	ldr.w	r0, [r9]
 8004f8a:	f040 0002 	orr.w	r0, r0, #2
 8004f8e:	f8c9 0000 	str.w	r0, [r9]
 8004f92:	f8d9 0000 	ldr.w	r0, [r9]
 8004f96:	f020 0002 	bic.w	r0, r0, #2
 8004f9a:	f8c9 0000 	str.w	r0, [r9]
 8004f9e:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8004fa2:	f040 0004 	orr.w	r0, r0, #4
 8004fa6:	f8c9 0020 	str.w	r0, [r9, #32]
 8004faa:	f003 fc19 	bl	80087e0 <__dsb>
 8004fae:	f8d9 0000 	ldr.w	r0, [r9]
 8004fb2:	f040 0004 	orr.w	r0, r0, #4
 8004fb6:	f8c9 0000 	str.w	r0, [r9]
 8004fba:	f8d9 0000 	ldr.w	r0, [r9]
 8004fbe:	f020 0004 	bic.w	r0, r0, #4
 8004fc2:	f8c9 0000 	str.w	r0, [r9]
 8004fc6:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8004fca:	f040 0040 	orr.w	r0, r0, #64	@ 0x40
 8004fce:	f8c9 0020 	str.w	r0, [r9, #32]
 8004fd2:	f003 fc05 	bl	80087e0 <__dsb>
 8004fd6:	f8d9 0000 	ldr.w	r0, [r9]
 8004fda:	f24e 0210 	movw	r2, #57360	@ 0xe010
 8004fde:	f2ce 0200 	movt	r2, #57344	@ 0xe000
 8004fe2:	f64f 71fe 	movw	r1, #65534	@ 0xfffe
 8004fe6:	f040 0040 	orr.w	r0, r0, #64	@ 0x40
 8004fea:	f8c9 0000 	str.w	r0, [r9]
 8004fee:	f8d9 0000 	ldr.w	r0, [r9]
 8004ff2:	f2c0 01ff 	movt	r1, #255	@ 0xff
 8004ff6:	f020 0040 	bic.w	r0, r0, #64	@ 0x40
 8004ffa:	f8c9 0000 	str.w	r0, [r9]
 8004ffe:	68d0      	ldr	r0, [r2, #12]
 8005000:	4008      	ands	r0, r1
 8005002:	f649 119a 	movw	r1, #39322	@ 0x999a
 8005006:	f6c1 1199 	movt	r1, #6553	@ 0x1999
 800500a:	fba0 0101 	umull	r0, r1, r0, r1
 800500e:	6051      	str	r1, [r2, #4]
 8005010:	6810      	ldr	r0, [r2, #0]
 8005012:	f040 0001 	orr.w	r0, r0, #1
 8005016:	6010      	str	r0, [r2, #0]
 8005018:	6810      	ldr	r0, [r2, #0]
 800501a:	f040 0002 	orr.w	r0, r0, #2
 800501e:	6010      	str	r0, [r2, #0]
 8005020:	f240 002b 	movw	r0, #43	@ 0x2b
 8005024:	f2c0 0000 	movt	r0, #0
 8005028:	f7ff f997 	bl	800435a <defmt::export::acquire_header_and_release>
 800502c:	2500      	movs	r5, #0
 800502e:	f640 0400 	movw	r4, #2048	@ 0x800
 8005032:	f2c4 0502 	movt	r5, #16386	@ 0x4002
 8005036:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 800503a:	68e8      	ldr	r0, [r5, #12]
 800503c:	f640 0208 	movw	r2, #2056	@ 0x808
 8005040:	f2c4 0202 	movt	r2, #16386	@ 0x4002
 8005044:	f641 0a00 	movw	sl, #6144	@ 0x1800
 8005048:	f020 000c 	bic.w	r0, r0, #12
 800504c:	60e8      	str	r0, [r5, #12]
 800504e:	6828      	ldr	r0, [r5, #0]
 8005050:	f2c4 0a02 	movt	sl, #16386	@ 0x4002
 8005054:	230b      	movs	r3, #11
 8005056:	2602      	movs	r6, #2
 8005058:	f020 000c 	bic.w	r0, r0, #12
 800505c:	6028      	str	r0, [r5, #0]
 800505e:	68e8      	ldr	r0, [r5, #12]
 8005060:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005064:	f04f 0800 	mov.w	r8, #0
 8005068:	f50d 6b4a 	add.w	fp, sp, #3232	@ 0xca0
 800506c:	f420 4040 	bic.w	r0, r0, #49152	@ 0xc000
 8005070:	60e8      	str	r0, [r5, #12]
 8005072:	6828      	ldr	r0, [r5, #0]
 8005074:	f420 4040 	bic.w	r0, r0, #49152	@ 0xc000
 8005078:	6028      	str	r0, [r5, #0]
 800507a:	f64f 400c 	movw	r0, #64524	@ 0xfc0c
 800507e:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 8005082:	5821      	ldr	r1, [r4, r0]
 8005084:	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
 8005088:	5021      	str	r1, [r4, r0]
 800508a:	f8d5 1400 	ldr.w	r1, [r5, #1024]	@ 0x400
 800508e:	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
 8005092:	f8c5 1400 	str.w	r1, [r5, #1024]	@ 0x400
 8005096:	6851      	ldr	r1, [r2, #4]
 8005098:	f421 7140 	bic.w	r1, r1, #768	@ 0x300
 800509c:	6051      	str	r1, [r2, #4]
 800509e:	6821      	ldr	r1, [r4, #0]
 80050a0:	f421 7140 	bic.w	r1, r1, #768	@ 0x300
 80050a4:	6021      	str	r1, [r4, #0]
 80050a6:	6851      	ldr	r1, [r2, #4]
 80050a8:	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
 80050ac:	6051      	str	r1, [r2, #4]
 80050ae:	6821      	ldr	r1, [r4, #0]
 80050b0:	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
 80050b4:	6021      	str	r1, [r4, #0]
 80050b6:	f8da 100c 	ldr.w	r1, [sl, #12]
 80050ba:	f421 0140 	bic.w	r1, r1, #12582912	@ 0xc00000
 80050be:	f8ca 100c 	str.w	r1, [sl, #12]
 80050c2:	f8da 1000 	ldr.w	r1, [sl]
 80050c6:	f421 0140 	bic.w	r1, r1, #12582912	@ 0xc00000
 80050ca:	f8ca 1000 	str.w	r1, [sl]
 80050ce:	f8da 100c 	ldr.w	r1, [sl, #12]
 80050d2:	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
 80050d6:	f8ca 100c 	str.w	r1, [sl, #12]
 80050da:	f8da 1000 	ldr.w	r1, [sl]
 80050de:	f021 6140 	bic.w	r1, r1, #201326592	@ 0xc000000
 80050e2:	f8ca 1000 	str.w	r1, [sl]
 80050e6:	6a29      	ldr	r1, [r5, #32]
 80050e8:	f363 210b 	bfi	r1, r3, #8, #4
 80050ec:	6229      	str	r1, [r5, #32]
 80050ee:	6829      	ldr	r1, [r5, #0]
 80050f0:	f366 1105 	bfi	r1, r6, #4, #2
 80050f4:	6029      	str	r1, [r5, #0]
 80050f6:	68a9      	ldr	r1, [r5, #8]
 80050f8:	f041 0130 	orr.w	r1, r1, #48	@ 0x30
 80050fc:	60a9      	str	r1, [r5, #8]
 80050fe:	6991      	ldr	r1, [r2, #24]
 8005100:	f363 1107 	bfi	r1, r3, #4, #4
 8005104:	6191      	str	r1, [r2, #24]
 8005106:	6821      	ldr	r1, [r4, #0]
 8005108:	f366 0183 	bfi	r1, r6, #2, #2
 800510c:	6021      	str	r1, [r4, #0]
 800510e:	6811      	ldr	r1, [r2, #0]
 8005110:	f041 010c 	orr.w	r1, r1, #12
 8005114:	6011      	str	r1, [r2, #0]
 8005116:	f64f 4118 	movw	r1, #64536	@ 0xfc18
 800511a:	f44f 1200 	mov.w	r2, #2097152	@ 0x200000
 800511e:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 8005122:	5062      	str	r2, [r4, r1]
 8005124:	5821      	ldr	r1, [r4, r0]
 8005126:	f421 6140 	bic.w	r1, r1, #3072	@ 0xc00
 800512a:	5021      	str	r1, [r4, r0]
 800512c:	f64f 4004 	movw	r0, #64516	@ 0xfc04
 8005130:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 8005134:	5821      	ldr	r1, [r4, r0]
 8005136:	f021 0120 	bic.w	r1, r1, #32
 800513a:	5021      	str	r1, [r4, r0]
 800513c:	2101      	movs	r1, #1
 800513e:	f8d5 0400 	ldr.w	r0, [r5, #1024]	@ 0x400
 8005142:	f361 208b 	bfi	r0, r1, #10, #2
 8005146:	f8c5 0400 	str.w	r0, [r5, #1024]	@ 0x400
 800514a:	f8ce 82ec 	str.w	r8, [lr, #748]	@ 0x2ec
 800514e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005152:	f50b 60cd 	add.w	r0, fp, #1640	@ 0x668
 8005156:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 800515a:	f8ce 82e8 	str.w	r8, [lr, #744]	@ 0x2e8
 800515e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005162:	f8cd 8660 	str.w	r8, [sp, #1632]	@ 0x660
 8005166:	f8ce 82e4 	str.w	r8, [lr, #740]	@ 0x2e4
 800516a:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800516e:	f8cd 8664 	str.w	r8, [sp, #1636]	@ 0x664
 8005172:	f8ce 82e0 	str.w	r8, [lr, #736]	@ 0x2e0
 8005176:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800517a:	f8cd 8668 	str.w	r8, [sp, #1640]	@ 0x668
 800517e:	f8ce 82dc 	str.w	r8, [lr, #732]	@ 0x2dc
 8005182:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005186:	f8cd 866c 	str.w	r8, [sp, #1644]	@ 0x66c
 800518a:	f8ce 82d8 	str.w	r8, [lr, #728]	@ 0x2d8
 800518e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005192:	f8cd 8670 	str.w	r8, [sp, #1648]	@ 0x670
 8005196:	f8ce 82d4 	str.w	r8, [lr, #724]	@ 0x2d4
 800519a:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800519e:	f8cd 8674 	str.w	r8, [sp, #1652]	@ 0x674
 80051a2:	f8cd 8678 	str.w	r8, [sp, #1656]	@ 0x678
 80051a6:	f8cd 867c 	str.w	r8, [sp, #1660]	@ 0x67c
 80051aa:	f8ce 82d0 	str.w	r8, [lr, #720]	@ 0x2d0
 80051ae:	9006      	str	r0, [sp, #24]
 80051b0:	f003 fb1e 	bl	80087f0 <__aeabi_memclr8>
 80051b4:	ae08      	add	r6, sp, #32
 80051b6:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 80051ba:	f506 60d1 	add.w	r0, r6, #1672	@ 0x688
 80051be:	9005      	str	r0, [sp, #20]
 80051c0:	f003 fb16 	bl	80087f0 <__aeabi_memclr8>
 80051c4:	4630      	mov	r0, r6
 80051c6:	2124      	movs	r1, #36	@ 0x24
 80051c8:	e9cd 8816 	strd	r8, r8, [sp, #88]	@ 0x58
 80051cc:	f8cd 8050 	str.w	r8, [sp, #80]	@ 0x50
 80051d0:	f8cd 8048 	str.w	r8, [sp, #72]	@ 0x48
 80051d4:	f003 fb0c 	bl	80087f0 <__aeabi_memclr8>
 80051d8:	f106 0048 	add.w	r0, r6, #72	@ 0x48
 80051dc:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 80051e0:	9003      	str	r0, [sp, #12]
 80051e2:	f003 fb05 	bl	80087f0 <__aeabi_memclr8>
 80051e6:	4658      	mov	r0, fp
 80051e8:	2124      	movs	r1, #36	@ 0x24
 80051ea:	f8cd 869c 	str.w	r8, [sp, #1692]	@ 0x69c
 80051ee:	f8cd 8698 	str.w	r8, [sp, #1688]	@ 0x698
 80051f2:	f8cd 8690 	str.w	r8, [sp, #1680]	@ 0x690
 80051f6:	f8cd 8688 	str.w	r8, [sp, #1672]	@ 0x688
 80051fa:	f8cd 8680 	str.w	r8, [sp, #1664]	@ 0x680
 80051fe:	f88d 8cd0 	strb.w	r8, [sp, #3280]	@ 0xcd0
 8005202:	f8cd 8ccc 	str.w	r8, [sp, #3276]	@ 0xccc
 8005206:	f8cd 8cc8 	str.w	r8, [sp, #3272]	@ 0xcc8
 800520a:	f003 faf1 	bl	80087f0 <__aeabi_memclr8>
 800520e:	f10b 0638 	add.w	r6, fp, #56	@ 0x38
 8005212:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 8005216:	4630      	mov	r0, r6
 8005218:	f003 faea 	bl	80087f0 <__aeabi_memclr8>
 800521c:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005220:	210b      	movs	r1, #11
 8005222:	2302      	movs	r3, #2
 8005224:	220b      	movs	r2, #11
 8005226:	f88e 8300 	strb.w	r8, [lr, #768]	@ 0x300
 800522a:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800522e:	f8ce 82fc 	str.w	r8, [lr, #764]	@ 0x2fc
 8005232:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005236:	f8ce 82f8 	str.w	r8, [lr, #760]	@ 0x2f8
 800523a:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800523e:	f8ce 82f0 	str.w	r8, [lr, #752]	@ 0x2f0
 8005242:	6a28      	ldr	r0, [r5, #32]
 8005244:	f361 1007 	bfi	r0, r1, #4, #4
 8005248:	6228      	str	r0, [r5, #32]
 800524a:	6828      	ldr	r0, [r5, #0]
 800524c:	f363 0083 	bfi	r0, r3, #2, #2
 8005250:	6028      	str	r0, [r5, #0]
 8005252:	68a8      	ldr	r0, [r5, #8]
 8005254:	f040 000c 	orr.w	r0, r0, #12
 8005258:	60a8      	str	r0, [r5, #8]
 800525a:	6a28      	ldr	r0, [r5, #32]
 800525c:	f361 701f 	bfi	r0, r1, #28, #4
 8005260:	6228      	str	r0, [r5, #32]
 8005262:	6828      	ldr	r0, [r5, #0]
 8005264:	f363 308f 	bfi	r0, r3, #14, #2
 8005268:	6028      	str	r0, [r5, #0]
 800526a:	68a8      	ldr	r0, [r5, #8]
 800526c:	f440 4040 	orr.w	r0, r0, #49152	@ 0xc000
 8005270:	60a8      	str	r0, [r5, #8]
 8005272:	f8da 0024 	ldr.w	r0, [sl, #36]	@ 0x24
 8005276:	f362 300f 	bfi	r0, r2, #12, #4
 800527a:	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
 800527e:	f8da 0000 	ldr.w	r0, [sl]
 8005282:	f363 5097 	bfi	r0, r3, #22, #2
 8005286:	f8ca 0000 	str.w	r0, [sl]
 800528a:	f8da 0008 	ldr.w	r0, [sl, #8]
 800528e:	f440 0040 	orr.w	r0, r0, #12582912	@ 0xc00000
 8005292:	f8ca 0008 	str.w	r0, [sl, #8]
 8005296:	f8da 0024 	ldr.w	r0, [sl, #36]	@ 0x24
 800529a:	f362 5017 	bfi	r0, r2, #20, #4
 800529e:	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
 80052a2:	f8da 0000 	ldr.w	r0, [sl]
 80052a6:	f363 609b 	bfi	r0, r3, #26, #2
 80052aa:	f8ca 0000 	str.w	r0, [sl]
 80052ae:	f8da 0008 	ldr.w	r0, [sl, #8]
 80052b2:	f040 6040 	orr.w	r0, r0, #201326592	@ 0xc000000
 80052b6:	f8ca 0008 	str.w	r0, [sl, #8]
 80052ba:	f64f 4024 	movw	r0, #64548	@ 0xfc24
 80052be:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 80052c2:	5821      	ldr	r1, [r4, r0]
 80052c4:	f362 5117 	bfi	r1, r2, #20, #4
 80052c8:	5021      	str	r1, [r4, r0]
 80052ca:	f8d5 0400 	ldr.w	r0, [r5, #1024]	@ 0x400
 80052ce:	f363 609b 	bfi	r0, r3, #26, #2
 80052d2:	f8c5 0400 	str.w	r0, [r5, #1024]	@ 0x400
 80052d6:	f64f 4008 	movw	r0, #64520	@ 0xfc08
 80052da:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 80052de:	5821      	ldr	r1, [r4, r0]
 80052e0:	f041 6140 	orr.w	r1, r1, #201326592	@ 0xc000000
 80052e4:	5021      	str	r1, [r4, r0]
 80052e6:	f640 0108 	movw	r1, #2056	@ 0x808
 80052ea:	f2c4 0102 	movt	r1, #16386	@ 0x4002
 80052ee:	6988      	ldr	r0, [r1, #24]
 80052f0:	f362 4013 	bfi	r0, r2, #16, #4
 80052f4:	6188      	str	r0, [r1, #24]
 80052f6:	6820      	ldr	r0, [r4, #0]
 80052f8:	f363 2009 	bfi	r0, r3, #8, #2
 80052fc:	6020      	str	r0, [r4, #0]
 80052fe:	6808      	ldr	r0, [r1, #0]
 8005300:	f440 7040 	orr.w	r0, r0, #768	@ 0x300
 8005304:	6008      	str	r0, [r1, #0]
 8005306:	6988      	ldr	r0, [r1, #24]
 8005308:	f362 5017 	bfi	r0, r2, #20, #4
 800530c:	6188      	str	r0, [r1, #24]
 800530e:	6820      	ldr	r0, [r4, #0]
 8005310:	f363 208b 	bfi	r0, r3, #10, #2
 8005314:	6020      	str	r0, [r4, #0]
 8005316:	6808      	ldr	r0, [r1, #0]
 8005318:	f440 6040 	orr.w	r0, r0, #3072	@ 0xc00
 800531c:	6008      	str	r0, [r1, #0]
 800531e:	f003 fa62 	bl	80087e6 <__primask_r>
 8005322:	4680      	mov	r8, r0
 8005324:	f003 fa52 	bl	80087cc <__cpsid>
 8005328:	f8d9 0034 	ldr.w	r0, [r9, #52]	@ 0x34
 800532c:	ea5f 71c8 	movs.w	r1, r8, lsl #31
 8005330:	f440 4080 	orr.w	r0, r0, #16384	@ 0x4000
 8005334:	f8c9 0034 	str.w	r0, [r9, #52]	@ 0x34
 8005338:	f8d9 0020 	ldr.w	r0, [r9, #32]
 800533c:	9604      	str	r6, [sp, #16]
 800533e:	d126      	bne.n	800538e <ip::__cortex_m_rt_main+0xa0e>
 8005340:	0180      	lsls	r0, r0, #6
 8005342:	f8dd 801c 	ldr.w	r8, [sp, #28]
 8005346:	bf42      	ittt	mi
 8005348:	f8d9 0020 	ldrmi.w	r0, [r9, #32]
 800534c:	f020 7000 	bicmi.w	r0, r0, #33554432	@ 0x2000000
 8005350:	f8c9 0020 	strmi.w	r0, [r9, #32]
 8005354:	f643 0004 	movw	r0, #14340	@ 0x3804
 8005358:	f2c4 0001 	movt	r0, #16385	@ 0x4001
 800535c:	6801      	ldr	r1, [r0, #0]
 800535e:	f441 0100 	orr.w	r1, r1, #8388608	@ 0x800000
 8005362:	6001      	str	r1, [r0, #0]
 8005364:	f8d9 0020 	ldr.w	r0, [r9, #32]
 8005368:	f040 6060 	orr.w	r0, r0, #234881024	@ 0xe000000
 800536c:	f8c9 0020 	str.w	r0, [r9, #32]
 8005370:	f8d9 0000 	ldr.w	r0, [r9]
 8005374:	f040 7000 	orr.w	r0, r0, #33554432	@ 0x2000000
 8005378:	f8c9 0000 	str.w	r0, [r9]
 800537c:	f8d9 0000 	ldr.w	r0, [r9]
 8005380:	f020 7000 	bic.w	r0, r0, #33554432	@ 0x2000000
 8005384:	f8c9 0000 	str.w	r0, [r9]
 8005388:	f003 fa22 	bl	80087d0 <__cpsie>
 800538c:	e023      	b.n	80053d6 <ip::__cortex_m_rt_main+0xa56>
 800538e:	0180      	lsls	r0, r0, #6
 8005390:	f8dd 801c 	ldr.w	r8, [sp, #28]
 8005394:	bf42      	ittt	mi
 8005396:	f8d9 0020 	ldrmi.w	r0, [r9, #32]
 800539a:	f020 7000 	bicmi.w	r0, r0, #33554432	@ 0x2000000
 800539e:	f8c9 0020 	strmi.w	r0, [r9, #32]
 80053a2:	f643 0004 	movw	r0, #14340	@ 0x3804
 80053a6:	f2c4 0001 	movt	r0, #16385	@ 0x4001
 80053aa:	6801      	ldr	r1, [r0, #0]
 80053ac:	f441 0100 	orr.w	r1, r1, #8388608	@ 0x800000
 80053b0:	6001      	str	r1, [r0, #0]
 80053b2:	f8d9 0020 	ldr.w	r0, [r9, #32]
 80053b6:	f040 6060 	orr.w	r0, r0, #234881024	@ 0xe000000
 80053ba:	f8c9 0020 	str.w	r0, [r9, #32]
 80053be:	f8d9 0000 	ldr.w	r0, [r9]
 80053c2:	f040 7000 	orr.w	r0, r0, #33554432	@ 0x2000000
 80053c6:	f8c9 0000 	str.w	r0, [r9]
 80053ca:	f8d9 0000 	ldr.w	r0, [r9]
 80053ce:	f020 7000 	bic.w	r0, r0, #33554432	@ 0x2000000
 80053d2:	f8c9 0000 	str.w	r0, [r9]
 80053d6:	f249 0418 	movw	r4, #36888	@ 0x9018
 80053da:	f10d 0e18 	add.w	lr, sp, #24
 80053de:	9b05      	ldr	r3, [sp, #20]
 80053e0:	f2c4 0402 	movt	r4, #16386	@ 0x4002
 80053e4:	9e03      	ldr	r6, [sp, #12]
 80053e6:	f50d 6a4a 	add.w	sl, sp, #3232	@ 0xca0
 80053ea:	f854 0c18 	ldr.w	r0, [r4, #-24]
 80053ee:	f10d 0b20 	add.w	fp, sp, #32
 80053f2:	f50e 5997 	add.w	r9, lr, #4832	@ 0x12e0
 80053f6:	f040 0001 	orr.w	r0, r0, #1
 80053fa:	f844 0c18 	str.w	r0, [r4, #-24]
 80053fe:	f854 0c18 	ldr.w	r0, [r4, #-24]
 8005402:	07c0      	lsls	r0, r0, #31
 8005404:	d00c      	beq.n	8005420 <ip::__cortex_m_rt_main+0xaa0>
 8005406:	f854 0c18 	ldr.w	r0, [r4, #-24]
 800540a:	07c0      	lsls	r0, r0, #31
 800540c:	bf1c      	itt	ne
 800540e:	f854 0c18 	ldrne.w	r0, [r4, #-24]
 8005412:	ea5f 70c0 	movsne.w	r0, r0, lsl #31
 8005416:	d003      	beq.n	8005420 <ip::__cortex_m_rt_main+0xaa0>
 8005418:	f854 0c18 	ldr.w	r0, [r4, #-24]
 800541c:	07c0      	lsls	r0, r0, #31
 800541e:	d1ee      	bne.n	80053fe <ip::__cortex_m_rt_main+0xa7e>
 8005420:	6820      	ldr	r0, [r4, #0]
 8005422:	217f      	movs	r1, #127	@ 0x7f
 8005424:	f6cf 5100 	movt	r1, #64768	@ 0xfd00
 8005428:	f240 52f2 	movw	r2, #1522	@ 0x5f2
 800542c:	f040 60e4 	orr.w	r0, r0, #119537664	@ 0x7200000
 8005430:	2501      	movs	r5, #1
 8005432:	f040 0084 	orr.w	r0, r0, #132	@ 0x84
 8005436:	6020      	str	r0, [r4, #0]
 8005438:	f854 0c18 	ldr.w	r0, [r4, #-24]
 800543c:	4008      	ands	r0, r1
 800543e:	f246 0180 	movw	r1, #24704	@ 0x6080
 8005442:	f2c0 21c1 	movt	r1, #705	@ 0x2c1
 8005446:	4308      	orrs	r0, r1
 8005448:	f844 0c18 	str.w	r0, [r4, #-24]
 800544c:	f44f 4080 	mov.w	r0, #16384	@ 0x4000
 8005450:	9009      	str	r0, [sp, #36]	@ 0x24
 8005452:	960a      	str	r6, [sp, #40]	@ 0x28
 8005454:	9909      	ldr	r1, [sp, #36]	@ 0x24
 8005456:	9611      	str	r6, [sp, #68]	@ 0x44
 8005458:	f362 010b 	bfi	r1, r2, #0, #12
 800545c:	9109      	str	r1, [sp, #36]	@ 0x24
 800545e:	f50b 61c8 	add.w	r1, fp, #1600	@ 0x640
 8005462:	910b      	str	r1, [sp, #44]	@ 0x2c
 8005464:	960a      	str	r6, [sp, #40]	@ 0x28
 8005466:	2600      	movs	r6, #0
 8005468:	9510      	str	r5, [sp, #64]	@ 0x40
 800546a:	e9cd 5112 	strd	r5, r1, [sp, #72]	@ 0x48
 800546e:	910b      	str	r1, [sp, #44]	@ 0x2c
 8005470:	f04f 4100 	mov.w	r1, #2147483648	@ 0x80000000
 8005474:	f3bf 8f5f 	dmb	sy
 8005478:	9108      	str	r1, [sp, #32]
 800547a:	f3bf 8f5f 	dmb	sy
 800547e:	f8cd 0664 	str.w	r0, [sp, #1636]	@ 0x664
 8005482:	f8cd 3668 	str.w	r3, [sp, #1640]	@ 0x668
 8005486:	f8dd 0664 	ldr.w	r0, [sp, #1636]	@ 0x664
 800548a:	f362 000b 	bfi	r0, r2, #0, #12
 800548e:	f8cd 0664 	str.w	r0, [sp, #1636]	@ 0x664
 8005492:	f8cd 666c 	str.w	r6, [sp, #1644]	@ 0x66c
 8005496:	f8dd 0664 	ldr.w	r0, [sp, #1636]	@ 0x664
 800549a:	f440 4000 	orr.w	r0, r0, #32768	@ 0x8000
 800549e:	f8cd 0664 	str.w	r0, [sp, #1636]	@ 0x664
 80054a2:	f8cd 3668 	str.w	r3, [sp, #1640]	@ 0x668
 80054a6:	f8cd 3684 	str.w	r3, [sp, #1668]	@ 0x684
 80054aa:	f8cd 5680 	str.w	r5, [sp, #1664]	@ 0x680
 80054ae:	f8cd 668c 	str.w	r6, [sp, #1676]	@ 0x68c
 80054b2:	f8cd 5688 	str.w	r5, [sp, #1672]	@ 0x688
 80054b6:	f8cd 666c 	str.w	r6, [sp, #1644]	@ 0x66c
 80054ba:	f3bf 8f5f 	dmb	sy
 80054be:	f8cd 1660 	str.w	r1, [sp, #1632]	@ 0x660
 80054c2:	f3bf 8f5f 	dmb	sy
 80054c6:	f844 bc0c 	str.w	fp, [r4, #-12]
 80054ca:	6820      	ldr	r0, [r4, #0]
 80054cc:	f040 0002 	orr.w	r0, r0, #2
 80054d0:	6020      	str	r0, [r4, #0]
 80054d2:	f003 f972 	bl	80087ba <stm32_eth::dma::rx::RxRing::demand_poll>
 80054d6:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 80054da:	f8cd 6ca0 	str.w	r6, [sp, #3232]	@ 0xca0
 80054de:	f8cd 6ca4 	str.w	r6, [sp, #3236]	@ 0xca4
 80054e2:	f50a 60c6 	add.w	r0, sl, #1584	@ 0x630
 80054e6:	f8cd 6ca8 	str.w	r6, [sp, #3240]	@ 0xca8
 80054ea:	f44f 7100 	mov.w	r1, #512	@ 0x200
 80054ee:	f8cd 6cac 	str.w	r6, [sp, #3244]	@ 0xcac
 80054f2:	f8cd 6cb0 	str.w	r6, [sp, #3248]	@ 0xcb0
 80054f6:	f8cd 6cb4 	str.w	r6, [sp, #3252]	@ 0xcb4
 80054fa:	f8cd 6cb8 	str.w	r6, [sp, #3256]	@ 0xcb8
 80054fe:	f8cd 6cbc 	str.w	r6, [sp, #3260]	@ 0xcbc
 8005502:	f8ce 62d0 	str.w	r6, [lr, #720]	@ 0x2d0
 8005506:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800550a:	f8ce 62d4 	str.w	r6, [lr, #724]	@ 0x2d4
 800550e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005512:	f8ce 62d8 	str.w	r6, [lr, #728]	@ 0x2d8
 8005516:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800551a:	f8ce 62dc 	str.w	r6, [lr, #732]	@ 0x2dc
 800551e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005522:	f8ce 62e0 	str.w	r6, [lr, #736]	@ 0x2e0
 8005526:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800552a:	f8ce 62e4 	str.w	r6, [lr, #740]	@ 0x2e4
 800552e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005532:	f8cd 0ccc 	str.w	r0, [sp, #3276]	@ 0xccc
 8005536:	f8ce 62e8 	str.w	r6, [lr, #744]	@ 0x2e8
 800553a:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800553e:	9804      	ldr	r0, [sp, #16]
 8005540:	f88e 5300 	strb.w	r5, [lr, #768]	@ 0x300
 8005544:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005548:	f8cd 0cc8 	str.w	r0, [sp, #3272]	@ 0xcc8
 800554c:	f248 7500 	movw	r5, #34560	@ 0x8700
 8005550:	f8ce 62ec 	str.w	r6, [lr, #748]	@ 0x2ec
 8005554:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005558:	9806      	ldr	r0, [sp, #24]
 800555a:	f2c4 0502 	movt	r5, #16386	@ 0x4002
 800555e:	f8ce 62fc 	str.w	r6, [lr, #764]	@ 0x2fc
 8005562:	f8c9 0000 	str.w	r0, [r9]
 8005566:	f844 ac08 	str.w	sl, [r4, #-8]
 800556a:	f3bf 8f5f 	dmb	sy
 800556e:	6820      	ldr	r0, [r4, #0]
 8005570:	f440 5000 	orr.w	r0, r0, #8192	@ 0x2000
 8005574:	6020      	str	r0, [r4, #0]
 8005576:	f64f 103c 	movw	r0, #63804	@ 0xf93c
 800557a:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 800557e:	5029      	str	r1, [r5, r0]
 8005580:	ea5f 0058 	movs.w	r0, r8, lsr #1
 8005584:	d105      	bne.n	8005592 <ip::__cortex_m_rt_main+0xc12>
 8005586:	f64a 70e0 	movw	r0, #45024	@ 0xafe0
 800558a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800558e:	f7fe fc74 	bl	8003e7a <core::panicking::panic_const::panic_const_div_by_zero>
 8005592:	f06f 4100 	mvn.w	r1, #2147483648	@ 0x80000000
 8005596:	eb01 0298 	add.w	r2, r1, r8, lsr #2
 800559a:	fbb2 f2f0 	udiv	r2, r2, r0
 800559e:	f248 7604 	movw	r6, #34564	@ 0x8704
 80055a2:	fbb1 f0f2 	udiv	r0, r1, r2
 80055a6:	f04f 31ff 	mov.w	r1, #4294967295	@ 0xffffffff
 80055aa:	fba0 0101 	umull	r0, r1, r0, r1
 80055ae:	f242 1303 	movw	r3, #8451	@ 0x2103
 80055b2:	b2d2      	uxtb	r2, r2
 80055b4:	f2c4 0602 	movt	r6, #16386	@ 0x4002
 80055b8:	602b      	str	r3, [r5, #0]
 80055ba:	2300      	movs	r3, #0
 80055bc:	6032      	str	r2, [r6, #0]
 80055be:	4642      	mov	r2, r8
 80055c0:	f003 ff54 	bl	800946c <__aeabi_uldivmod>
 80055c4:	6170      	str	r0, [r6, #20]
 80055c6:	6828      	ldr	r0, [r5, #0]
 80055c8:	0680      	lsls	r0, r0, #26
 80055ca:	d509      	bpl.n	80055e0 <ip::__cortex_m_rt_main+0xc60>
 80055cc:	6828      	ldr	r0, [r5, #0]
 80055ce:	0680      	lsls	r0, r0, #26
 80055d0:	bf44      	itt	mi
 80055d2:	6828      	ldrmi	r0, [r5, #0]
 80055d4:	ea5f 6080 	movsmi.w	r0, r0, lsl #26
 80055d8:	d502      	bpl.n	80055e0 <ip::__cortex_m_rt_main+0xc60>
 80055da:	6828      	ldr	r0, [r5, #0]
 80055dc:	0680      	lsls	r0, r0, #26
 80055de:	d4f2      	bmi.n	80055c6 <ip::__cortex_m_rt_main+0xc46>
 80055e0:	6828      	ldr	r0, [r5, #0]
 80055e2:	f040 0020 	orr.w	r0, r0, #32
 80055e6:	6028      	str	r0, [r5, #0]
 80055e8:	6828      	ldr	r0, [r5, #0]
 80055ea:	0680      	lsls	r0, r0, #26
 80055ec:	d509      	bpl.n	8005602 <ip::__cortex_m_rt_main+0xc82>
 80055ee:	6828      	ldr	r0, [r5, #0]
 80055f0:	0680      	lsls	r0, r0, #26
 80055f2:	bf44      	itt	mi
 80055f4:	6828      	ldrmi	r0, [r5, #0]
 80055f6:	ea5f 6080 	movsmi.w	r0, r0, lsl #26
 80055fa:	d502      	bpl.n	8005602 <ip::__cortex_m_rt_main+0xc82>
 80055fc:	6828      	ldr	r0, [r5, #0]
 80055fe:	0680      	lsls	r0, r0, #26
 8005600:	d4f2      	bmi.n	80055e8 <ip::__cortex_m_rt_main+0xc68>
 8005602:	2000      	movs	r0, #0
 8005604:	60f0      	str	r0, [r6, #12]
 8005606:	6130      	str	r0, [r6, #16]
 8005608:	6828      	ldr	r0, [r5, #0]
 800560a:	0740      	lsls	r0, r0, #29
 800560c:	d509      	bpl.n	8005622 <ip::__cortex_m_rt_main+0xca2>
 800560e:	6828      	ldr	r0, [r5, #0]
 8005610:	0740      	lsls	r0, r0, #29
 8005612:	bf44      	itt	mi
 8005614:	6828      	ldrmi	r0, [r5, #0]
 8005616:	ea5f 7040 	movsmi.w	r0, r0, lsl #29
 800561a:	d502      	bpl.n	8005622 <ip::__cortex_m_rt_main+0xca2>
 800561c:	6828      	ldr	r0, [r5, #0]
 800561e:	0740      	lsls	r0, r0, #29
 8005620:	d4f2      	bmi.n	8005608 <ip::__cortex_m_rt_main+0xc88>
 8005622:	6828      	ldr	r0, [r5, #0]
 8005624:	f040 0004 	orr.w	r0, r0, #4
 8005628:	6028      	str	r0, [r5, #0]
 800562a:	6828      	ldr	r0, [r5, #0]
 800562c:	0740      	lsls	r0, r0, #29
 800562e:	d509      	bpl.n	8005644 <ip::__cortex_m_rt_main+0xcc4>
 8005630:	6828      	ldr	r0, [r5, #0]
 8005632:	0740      	lsls	r0, r0, #29
 8005634:	bf44      	itt	mi
 8005636:	6828      	ldrmi	r0, [r5, #0]
 8005638:	ea5f 7040 	movsmi.w	r0, r0, lsl #29
 800563c:	d502      	bpl.n	8005644 <ip::__cortex_m_rt_main+0xcc4>
 800563e:	6828      	ldr	r0, [r5, #0]
 8005640:	0740      	lsls	r0, r0, #29
 8005642:	d4f2      	bmi.n	800562a <ip::__cortex_m_rt_main+0xcaa>
 8005644:	f647 0040 	movw	r0, #30784	@ 0x7840
 8005648:	f2c0 107d 	movt	r0, #381	@ 0x17d
 800564c:	4580      	cmp	r8, r0
 800564e:	d260      	bcs.n	8005712 <ip::__cortex_m_rt_main+0xd92>
 8005650:	6820      	ldr	r0, [r4, #0]
 8005652:	f420 5000 	bic.w	r0, r0, #8192	@ 0x2000
 8005656:	6020      	str	r0, [r4, #0]
 8005658:	f854 0c04 	ldr.w	r0, [r4, #-4]
 800565c:	f3c0 5002 	ubfx	r0, r0, #20, #3
 8005660:	3801      	subs	r0, #1
 8005662:	2802      	cmp	r0, #2
 8005664:	d814      	bhi.n	8005690 <ip::__cortex_m_rt_main+0xd10>
 8005666:	f854 0c04 	ldr.w	r0, [r4, #-4]
 800566a:	f3c0 5002 	ubfx	r0, r0, #20, #3
 800566e:	3801      	subs	r0, #1
 8005670:	2802      	cmp	r0, #2
 8005672:	bf9f      	itttt	ls
 8005674:	f854 0c04 	ldrls.w	r0, [r4, #-4]
 8005678:	f3c0 5002 	ubfxls	r0, r0, #20, #3
 800567c:	3801      	subls	r0, #1
 800567e:	2802      	cmpls	r0, #2
 8005680:	d806      	bhi.n	8005690 <ip::__cortex_m_rt_main+0xd10>
 8005682:	f854 0c04 	ldr.w	r0, [r4, #-4]
 8005686:	f3c0 5002 	ubfx	r0, r0, #20, #3
 800568a:	3801      	subs	r0, #1
 800568c:	2803      	cmp	r0, #3
 800568e:	d3e3      	bcc.n	8005658 <ip::__cortex_m_rt_main+0xcd8>
 8005690:	6820      	ldr	r0, [r4, #0]
 8005692:	f020 0002 	bic.w	r0, r0, #2
 8005696:	6020      	str	r0, [r4, #0]
 8005698:	2001      	movs	r0, #1
 800569a:	f854 1c04 	ldr.w	r1, [r4, #-4]
 800569e:	f3c1 4142 	ubfx	r1, r1, #17, #3
 80056a2:	2907      	cmp	r1, #7
 80056a4:	d825      	bhi.n	80056f2 <ip::__cortex_m_rt_main+0xd72>
 80056a6:	fa00 f101 	lsl.w	r1, r0, r1
 80056aa:	f011 0faa 	tst.w	r1, #170	@ 0xaa
 80056ae:	d020      	beq.n	80056f2 <ip::__cortex_m_rt_main+0xd72>
 80056b0:	f854 1c04 	ldr.w	r1, [r4, #-4]
 80056b4:	f3c1 4142 	ubfx	r1, r1, #17, #3
 80056b8:	2907      	cmp	r1, #7
 80056ba:	d81a      	bhi.n	80056f2 <ip::__cortex_m_rt_main+0xd72>
 80056bc:	fa00 f101 	lsl.w	r1, r0, r1
 80056c0:	f011 0faa 	tst.w	r1, #170	@ 0xaa
 80056c4:	d015      	beq.n	80056f2 <ip::__cortex_m_rt_main+0xd72>
 80056c6:	f854 1c04 	ldr.w	r1, [r4, #-4]
 80056ca:	f3c1 4142 	ubfx	r1, r1, #17, #3
 80056ce:	2907      	cmp	r1, #7
 80056d0:	d80f      	bhi.n	80056f2 <ip::__cortex_m_rt_main+0xd72>
 80056d2:	fa00 f101 	lsl.w	r1, r0, r1
 80056d6:	f011 0faa 	tst.w	r1, #170	@ 0xaa
 80056da:	d00a      	beq.n	80056f2 <ip::__cortex_m_rt_main+0xd72>
 80056dc:	f854 1c04 	ldr.w	r1, [r4, #-4]
 80056e0:	f3c1 4142 	ubfx	r1, r1, #17, #3
 80056e4:	2907      	cmp	r1, #7
 80056e6:	d804      	bhi.n	80056f2 <ip::__cortex_m_rt_main+0xd72>
 80056e8:	fa00 f101 	lsl.w	r1, r0, r1
 80056ec:	f011 0faa 	tst.w	r1, #170	@ 0xaa
 80056f0:	d1d3      	bne.n	800569a <ip::__cortex_m_rt_main+0xd1a>
 80056f2:	f64a 0008 	movw	r0, #43016	@ 0xa808
 80056f6:	f24a 72f8 	movw	r2, #43000	@ 0xa7f8
 80056fa:	f64a 0334 	movw	r3, #43060	@ 0xa834
 80056fe:	f1a7 0111 	sub.w	r1, r7, #17
 8005702:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005706:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800570a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800570e:	f7fe fca5 	bl	800405c <core::result::unwrap_failed>
 8005712:	f640 60c0 	movw	r0, #3776	@ 0xec0
 8005716:	f2c0 2016 	movt	r0, #534	@ 0x216
 800571a:	4580      	cmp	r8, r0
 800571c:	d201      	bcs.n	8005722 <ip::__cortex_m_rt_main+0xda2>
 800571e:	2008      	movs	r0, #8
 8005720:	e022      	b.n	8005768 <ip::__cortex_m_rt_main+0xde8>
 8005722:	f248 7000 	movw	r0, #34560	@ 0x8700
 8005726:	f2c0 3093 	movt	r0, #915	@ 0x393
 800572a:	4580      	cmp	r8, r0
 800572c:	d201      	bcs.n	8005732 <ip::__cortex_m_rt_main+0xdb2>
 800572e:	200c      	movs	r0, #12
 8005730:	e01a      	b.n	8005768 <ip::__cortex_m_rt_main+0xde8>
 8005732:	f24e 1000 	movw	r0, #57600	@ 0xe100
 8005736:	f2c0 50f5 	movt	r0, #1525	@ 0x5f5
 800573a:	4580      	cmp	r8, r0
 800573c:	d20c      	bcs.n	8005758 <ip::__cortex_m_rt_main+0xdd8>
 800573e:	2000      	movs	r0, #0
 8005740:	e012      	b.n	8005768 <ip::__cortex_m_rt_main+0xde8>
 8005742:	f1a0 0328 	sub.w	r3, r0, #40	@ 0x28
 8005746:	2b38      	cmp	r3, #56	@ 0x38
 8005748:	f080 8341 	bcs.w	8005dce <ip::__cortex_m_rt_main+0x144e>
 800574c:	ed9f 2acf 	vldr	s4, [pc, #828]	@ 8005a8c <ip::__cortex_m_rt_main+0x110c>
 8005750:	f04f 0ec0 	mov.w	lr, #192	@ 0xc0
 8005754:	f7ff ba09 	b.w	8004b6a <ip::__cortex_m_rt_main+0x1ea>
 8005758:	f24d 1180 	movw	r1, #53632	@ 0xd180
 800575c:	2010      	movs	r0, #16
 800575e:	f6c0 01f0 	movt	r1, #2288	@ 0x8f0
 8005762:	4588      	cmp	r8, r1
 8005764:	bf38      	it	cc
 8005766:	2004      	movcc	r0, #4
 8005768:	f64f 1110 	movw	r1, #63760	@ 0xf910
 800576c:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005770:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 8005774:	f50d 56c9 	add.w	r6, sp, #6432	@ 0x1920
 8005778:	586a      	ldr	r2, [r5, r1]
 800577a:	f022 021c 	bic.w	r2, r2, #28
 800577e:	4310      	orrs	r0, r2
 8005780:	5068      	str	r0, [r5, r1]
 8005782:	f64f 1000 	movw	r0, #63744	@ 0xf900
 8005786:	f6cf 70ff 	movt	r0, #65535	@ 0xffff
 800578a:	f644 628c 	movw	r2, #20108	@ 0x4e8c
 800578e:	5829      	ldr	r1, [r5, r0]
 8005790:	f2c0 2200 	movt	r2, #512	@ 0x200
 8005794:	4311      	orrs	r1, r2
 8005796:	5029      	str	r1, [r5, r0]
 8005798:	f64f 1104 	movw	r1, #63748	@ 0xf904
 800579c:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 80057a0:	586a      	ldr	r2, [r5, r1]
 80057a2:	f042 4200 	orr.w	r2, r2, #2147483648	@ 0x80000000
 80057a6:	f042 0201 	orr.w	r2, r2, #1
 80057aa:	506a      	str	r2, [r5, r1]
 80057ac:	f64f 1118 	movw	r1, #63768	@ 0xf918
 80057b0:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 80057b4:	586a      	ldr	r2, [r5, r1]
 80057b6:	f2c0 1200 	movt	r2, #256	@ 0x100
 80057ba:	506a      	str	r2, [r5, r1]
 80057bc:	f64f 210c 	movw	r1, #64012	@ 0xfa0c
 80057c0:	2260      	movs	r2, #96	@ 0x60
 80057c2:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 80057c6:	f2c0 0202 	movt	r2, #2
 80057ca:	506a      	str	r2, [r5, r1]
 80057cc:	f64f 2110 	movw	r1, #64016	@ 0xfa10
 80057d0:	f6cf 71ff 	movt	r1, #65535	@ 0xffff
 80057d4:	f44f 1203 	mov.w	r2, #2146304	@ 0x20c000
 80057d8:	506a      	str	r2, [r5, r1]
 80057da:	586a      	ldr	r2, [r5, r1]
 80057dc:	f442 1200 	orr.w	r2, r2, #2097152	@ 0x200000
 80057e0:	506a      	str	r2, [r5, r1]
 80057e2:	f04f 3203 	mov.w	r2, #50529027	@ 0x3030303
 80057e6:	5829      	ldr	r1, [r5, r0]
 80057e8:	f441 4190 	orr.w	r1, r1, #18432	@ 0x4800
 80057ec:	5029      	str	r1, [r5, r0]
 80057ee:	2500      	movs	r5, #0
 80057f0:	2002      	movs	r0, #2
 80057f2:	f509 61c2 	add.w	r1, r9, #1552	@ 0x610
 80057f6:	e881 0421 	stmia.w	r1, {r0, r5, sl}
 80057fa:	f24e 0110 	movw	r1, #57360	@ 0xe010
 80057fe:	f2ce 0100 	movt	r1, #57344	@ 0xe000
 8005802:	f8c9 5624 	str.w	r5, [r9, #1572]	@ 0x624
 8005806:	f8c9 5620 	str.w	r5, [r9, #1568]	@ 0x620
 800580a:	f8c9 061c 	str.w	r0, [r9, #1564]	@ 0x61c
 800580e:	f8c9 b60c 	str.w	fp, [r9, #1548]	@ 0x60c
 8005812:	6860      	ldr	r0, [r4, #4]
 8005814:	f440 3080 	orr.w	r0, r0, #65536	@ 0x10000
 8005818:	f040 0041 	orr.w	r0, r0, #65	@ 0x41
 800581c:	6060      	str	r0, [r4, #4]
 800581e:	f04f 5000 	mov.w	r0, #536870912	@ 0x20000000
 8005822:	2401      	movs	r4, #1
 8005824:	f8c1 00f4 	str.w	r0, [r1, #244]	@ 0xf4
 8005828:	f64e 71be 	movw	r1, #61374	@ 0xefbe
 800582c:	f64e 7039 	movw	r0, #61241	@ 0xef39
 8005830:	f8ae 1a8c 	strh.w	r1, [lr, #2700]	@ 0xa8c
 8005834:	2100      	movs	r1, #0
 8005836:	f2c7 5090 	movt	r0, #30096	@ 0x7590
 800583a:	f6ca 51de 	movt	r1, #44510	@ 0xadde
 800583e:	f8c9 0640 	str.w	r0, [r9, #1600]	@ 0x640
 8005842:	f8c9 1790 	str.w	r1, [r9, #1936]	@ 0x790
 8005846:	f240 51f2 	movw	r1, #1522	@ 0x5f2
 800584a:	f106 0020 	add.w	r0, r6, #32
 800584e:	f8c9 1634 	str.w	r1, [r9, #1588]	@ 0x634
 8005852:	21c0      	movs	r1, #192	@ 0xc0
 8005854:	f8c9 5710 	str.w	r5, [r9, #1808]	@ 0x710
 8005858:	f8c9 5714 	str.w	r5, [r9, #1812]	@ 0x714
 800585c:	f8c9 5718 	str.w	r5, [r9, #1816]	@ 0x718
 8005860:	f8c9 462c 	str.w	r4, [r9, #1580]	@ 0x62c
 8005864:	f8c9 2630 	str.w	r2, [r9, #1584]	@ 0x630
 8005868:	f8c9 5638 	str.w	r5, [r9, #1592]	@ 0x638
 800586c:	f8c9 563c 	str.w	r5, [r9, #1596]	@ 0x63c
 8005870:	f8c9 5644 	str.w	r5, [r9, #1604]	@ 0x644
 8005874:	f8c9 4628 	str.w	r4, [r9, #1576]	@ 0x628
 8005878:	f002 ffba 	bl	80087f0 <__aeabi_memclr8>
 800587c:	f506 7080 	add.w	r0, r6, #256	@ 0x100
 8005880:	2164      	movs	r1, #100	@ 0x64
 8005882:	f002 ffb5 	bl	80087f0 <__aeabi_memclr8>
 8005886:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800588a:	2018      	movs	r0, #24
 800588c:	f8c9 4718 	str.w	r4, [r9, #1816]	@ 0x718
 8005890:	f88e 0a18 	strb.w	r0, [lr, #2584]	@ 0xa18
 8005894:	f64f 70fe 	movw	r0, #65534	@ 0xfffe
 8005898:	f2c0 00ff 	movt	r0, #255	@ 0xff
 800589c:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 80058a0:	300c      	adds	r0, #12
 80058a2:	f8c9 071c 	str.w	r0, [r9, #1820]	@ 0x71c
 80058a6:	f000 01f0 	and.w	r1, r0, #240	@ 0xf0
 80058aa:	f88e 5a8e 	strb.w	r5, [lr, #2702]	@ 0xa8e
 80058ae:	29e0      	cmp	r1, #224	@ 0xe0
 80058b0:	f8c9 5708 	str.w	r5, [r9, #1800]	@ 0x708
 80058b4:	bf18      	it	ne
 80058b6:	f110 0101 	addsne.w	r1, r0, #1
 80058ba:	d11b      	bne.n	80058f4 <ip::__cortex_m_rt_main+0xf74>
 80058bc:	f8c9 08f0 	str.w	r0, [r9, #2288]	@ 0x8f0
 80058c0:	f647 304d 	movw	r0, #31565	@ 0x7b4d
 80058c4:	f10d 0e08 	add.w	lr, sp, #8
 80058c8:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80058cc:	f50e 51df 	add.w	r1, lr, #7136	@ 0x1be0
 80058d0:	f10d 0e08 	add.w	lr, sp, #8
 80058d4:	f64a 42b4 	movw	r2, #44212	@ 0xacb4
 80058d8:	f8c9 1af0 	str.w	r1, [r9, #2800]	@ 0xaf0
 80058dc:	f8c9 0af4 	str.w	r0, [r9, #2804]	@ 0xaf4
 80058e0:	f649 1053 	movw	r0, #39251	@ 0x9953
 80058e4:	f50e 51ef 	add.w	r1, lr, #7648	@ 0x1de0
 80058e8:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80058ec:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80058f0:	f7fd ff68 	bl	80037c4 <core::panicking::panic_fmt>
 80058f4:	f10d 0e10 	add.w	lr, sp, #16
 80058f8:	f24a 61a0 	movw	r1, #42656	@ 0xa6a0
 80058fc:	f50e 58d4 	add.w	r8, lr, #6784	@ 0x1a80
 8005900:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8005904:	f44f 72a8 	mov.w	r2, #336	@ 0x150
 8005908:	4640      	mov	r0, r8
 800590a:	f003 fcf8 	bl	80092fe <__aeabi_memcpy8>
 800590e:	f10d 0e08 	add.w	lr, sp, #8
 8005912:	f44f 7100 	mov.w	r1, #512	@ 0x200
 8005916:	f50e 56df 	add.w	r6, lr, #7136	@ 0x1be0
 800591a:	f8c9 88e8 	str.w	r8, [r9, #2280]	@ 0x8e8
 800591e:	f8c9 48ec 	str.w	r4, [r9, #2284]	@ 0x8ec
 8005922:	f44f 7b00 	mov.w	fp, #512	@ 0x200
 8005926:	4630      	mov	r0, r6
 8005928:	f003 fc8c 	bl	8009244 <__aeabi_memclr4>
 800592c:	f10d 0e08 	add.w	lr, sp, #8
 8005930:	f44f 7100 	mov.w	r1, #512	@ 0x200
 8005934:	f50e 50ef 	add.w	r0, lr, #7648	@ 0x1de0
 8005938:	f003 fc84 	bl	8009244 <__aeabi_memclr4>
 800593c:	f8d9 0798 	ldr.w	r0, [r9, #1944]	@ 0x798
 8005940:	f04f 0a00 	mov.w	sl, #0
 8005944:	f8d9 179c 	ldr.w	r1, [r9, #1948]	@ 0x79c
 8005948:	f080 0003 	eor.w	r0, r0, #3
 800594c:	e947 aa0e 	strd	sl, sl, [r7, #-56]	@ 0x38
 8005950:	4308      	orrs	r0, r1
 8005952:	e947 aa10 	strd	sl, sl, [r7, #-64]	@ 0x40
 8005956:	e947 aa12 	strd	sl, sl, [r7, #-72]	@ 0x48
 800595a:	e947 aa14 	strd	sl, sl, [r7, #-80]	@ 0x50
 800595e:	e947 aa0a 	strd	sl, sl, [r7, #-40]	@ 0x28
 8005962:	e947 aa08 	strd	sl, sl, [r7, #-32]
 8005966:	e947 aa18 	strd	sl, sl, [r7, #-96]	@ 0x60
 800596a:	e947 aa16 	strd	sl, sl, [r7, #-88]	@ 0x58
 800596e:	f040 8223 	bne.w	8005db8 <ip::__cortex_m_rt_main+0x1438>
 8005972:	f242 7010 	movw	r0, #10000	@ 0x2710
 8005976:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 800597a:	f44f 7196 	mov.w	r1, #300	@ 0x12c
 800597e:	f8c9 0800 	str.w	r0, [r9, #2048]	@ 0x800
 8005982:	2064      	movs	r0, #100	@ 0x64
 8005984:	f8c9 17b8 	str.w	r1, [r9, #1976]	@ 0x7b8
 8005988:	f1a7 0130 	sub.w	r1, r7, #48	@ 0x30
 800598c:	f88e aab8 	strb.w	sl, [lr, #2744]	@ 0xab8
 8005990:	f8c9 47f8 	str.w	r4, [r9, #2040]	@ 0x7f8
 8005994:	46de      	mov	lr, fp
 8005996:	f8c9 a7fc 	str.w	sl, [r9, #2044]	@ 0x7fc
 800599a:	46b3      	mov	fp, r6
 800599c:	f8c9 a804 	str.w	sl, [r9, #2052]	@ 0x804
 80059a0:	f108 0c78 	add.w	ip, r8, #120	@ 0x78
 80059a4:	f8c9 a808 	str.w	sl, [r9, #2056]	@ 0x808
 80059a8:	f8c9 a80c 	str.w	sl, [r9, #2060]	@ 0x80c
 80059ac:	f8c9 a7e8 	str.w	sl, [r9, #2024]	@ 0x7e8
 80059b0:	f8c9 a7ec 	str.w	sl, [r9, #2028]	@ 0x7ec
 80059b4:	f8c9 a7d8 	str.w	sl, [r9, #2008]	@ 0x7d8
 80059b8:	f8c9 a7dc 	str.w	sl, [r9, #2012]	@ 0x7dc
 80059bc:	f8c9 a7c8 	str.w	sl, [r9, #1992]	@ 0x7c8
 80059c0:	f8c9 a7cc 	str.w	sl, [r9, #1996]	@ 0x7cc
 80059c4:	f8c9 a798 	str.w	sl, [r9, #1944]	@ 0x798
 80059c8:	f8c9 a79c 	str.w	sl, [r9, #1948]	@ 0x79c
 80059cc:	f8c9 07bc 	str.w	r0, [r9, #1980]	@ 0x7bc
 80059d0:	f8c9 a7b0 	str.w	sl, [r9, #1968]	@ 0x7b0
 80059d4:	e891 007d 	ldmia.w	r1, {r0, r2, r3, r4, r5, r6}
 80059d8:	f1a7 0150 	sub.w	r1, r7, #80	@ 0x50
 80059dc:	e88c 007d 	stmia.w	ip, {r0, r2, r3, r4, r5, r6}
 80059e0:	a802      	add	r0, sp, #8
 80059e2:	f500 50ef 	add.w	r0, r0, #7648	@ 0x1de0
 80059e6:	f8c9 e854 	str.w	lr, [r9, #2132]	@ 0x854
 80059ea:	f8c9 e864 	str.w	lr, [r9, #2148]	@ 0x864
 80059ee:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 80059f2:	f8c9 0860 	str.w	r0, [r9, #2144]	@ 0x860
 80059f6:	f108 00d8 	add.w	r0, r8, #216	@ 0xd8
 80059fa:	f88e ab42 	strb.w	sl, [lr, #2882]	@ 0xb42
 80059fe:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005a02:	f8c9 b850 	str.w	fp, [r9, #2128]	@ 0x850
 8005a06:	f240 0840 	movw	r8, #64	@ 0x40
 8005a0a:	f8ae ab40 	strh.w	sl, [lr, #2880]	@ 0xb40
 8005a0e:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005a12:	f8c9 a858 	str.w	sl, [r9, #2136]	@ 0x858
 8005a16:	f2c2 0800 	movt	r8, #8192	@ 0x2000
 8005a1a:	f8c9 a85c 	str.w	sl, [r9, #2140]	@ 0x85c
 8005a1e:	f8c9 a868 	str.w	sl, [r9, #2152]	@ 0x868
 8005a22:	f8c9 a86c 	str.w	sl, [r9, #2156]	@ 0x86c
 8005a26:	f8c9 a840 	str.w	sl, [r9, #2112]	@ 0x840
 8005a2a:	f8c9 a838 	str.w	sl, [r9, #2104]	@ 0x838
 8005a2e:	f8c9 a830 	str.w	sl, [r9, #2096]	@ 0x830
 8005a32:	c96c      	ldmia	r1!, {r2, r3, r5, r6}
 8005a34:	c06c      	stmia	r0!, {r2, r3, r5, r6}
 8005a36:	e891 006c 	ldmia.w	r1, {r2, r3, r5, r6}
 8005a3a:	c06c      	stmia	r0!, {r2, r3, r5, r6}
 8005a3c:	f1a7 0360 	sub.w	r3, r7, #96	@ 0x60
 8005a40:	2601      	movs	r6, #1
 8005a42:	f88e abc8 	strb.w	sl, [lr, #3016]	@ 0xbc8
 8005a46:	f50d 5e80 	add.w	lr, sp, #4096	@ 0x1000
 8005a4a:	f8c9 68ca 	str.w	r6, [r9, #2250]	@ 0x8ca
 8005a4e:	f44f 7606 	mov.w	r6, #536	@ 0x218
 8005a52:	cb0f      	ldmia	r3, {r0, r1, r2, r3}
 8005a54:	f8c9 68a8 	str.w	r6, [r9, #2216]	@ 0x8a8
 8005a58:	464e      	mov	r6, r9
 8005a5a:	f8c9 a8c6 	str.w	sl, [r9, #2246]	@ 0x8c6
 8005a5e:	f8c9 a8c2 	str.w	sl, [r9, #2242]	@ 0x8c2
 8005a62:	f8c9 a8e0 	str.w	sl, [r9, #2272]	@ 0x8e0
 8005a66:	f8ae abac 	strh.w	sl, [lr, #2988]	@ 0xbac
 8005a6a:	f8c9 a8b0 	str.w	sl, [r9, #2224]	@ 0x8b0
 8005a6e:	f8c9 a890 	str.w	sl, [r9, #2192]	@ 0x890
 8005a72:	f8c9 a894 	str.w	sl, [r9, #2196]	@ 0x894
 8005a76:	f8c9 0898 	str.w	r0, [r9, #2200]	@ 0x898
 8005a7a:	f8c9 189c 	str.w	r1, [r9, #2204]	@ 0x89c
 8005a7e:	f8c9 28a0 	str.w	r2, [r9, #2208]	@ 0x8a0
 8005a82:	f8c9 38a4 	str.w	r3, [r9, #2212]	@ 0x8a4
 8005a86:	f8c9 a8ac 	str.w	sl, [r9, #2220]	@ 0x8ac
 8005a8a:	e09b      	b.n	8005bc4 <ip::__cortex_m_rt_main+0x1244>
 8005a8c:	42800000 	.word	0x42800000
 8005a90:	2064      	movs	r0, #100	@ 0x64
 8005a92:	f44f 7196 	mov.w	r1, #300	@ 0x12c
 8005a96:	e9c5 1008 	strd	r1, r0, [r5, #32]
 8005a9a:	f105 00d0 	add.w	r0, r5, #208	@ 0xd0
 8005a9e:	2128      	movs	r1, #40	@ 0x28
 8005aa0:	e9c5 aa00 	strd	sl, sl, [r5]
 8005aa4:	f8c5 a018 	str.w	sl, [r5, #24]
 8005aa8:	e9c5 aa20 	strd	sl, sl, [r5, #128]	@ 0x80
 8005aac:	e9c5 aa22 	strd	sl, sl, [r5, #136]	@ 0x88
 8005ab0:	f885 a130 	strb.w	sl, [r5, #304]	@ 0x130
 8005ab4:	e9c5 aa30 	strd	sl, sl, [r5, #192]	@ 0xc0
 8005ab8:	f885 a028 	strb.w	sl, [r5, #40]	@ 0x28
 8005abc:	f8d5 40bc 	ldr.w	r4, [r5, #188]	@ 0xbc
 8005ac0:	f8a5 a12e 	strh.w	sl, [r5, #302]	@ 0x12e
 8005ac4:	f8c5 a098 	str.w	sl, [r5, #152]	@ 0x98
 8005ac8:	f885 a12c 	strb.w	sl, [r5, #300]	@ 0x12c
 8005acc:	e9c5 aa40 	strd	sl, sl, [r5, #256]	@ 0x100
 8005ad0:	e9c5 aa42 	strd	sl, sl, [r5, #264]	@ 0x108
 8005ad4:	f002 fe8c 	bl	80087f0 <__aeabi_memclr8>
 8005ad8:	2050      	movs	r0, #80	@ 0x50
 8005ada:	2210      	movs	r2, #16
 8005adc:	e9c5 0a2c 	strd	r0, sl, [r5, #176]	@ 0xb0
 8005ae0:	f44f 7006 	mov.w	r0, #536	@ 0x218
 8005ae4:	f8c5 0110 	str.w	r0, [r5, #272]	@ 0x110
 8005ae8:	fab4 f084 	clz	r0, r4
 8005aec:	f1c0 0110 	rsb	r1, r0, #16
 8005af0:	4282      	cmp	r2, r0
 8005af2:	f240 002c 	movw	r0, #44	@ 0x2c
 8005af6:	f04f 0901 	mov.w	r9, #1
 8005afa:	f2c0 0000 	movt	r0, #0
 8005afe:	e9c5 aa3e 	strd	sl, sl, [r5, #248]	@ 0xf8
 8005b02:	e9c5 aa1c 	strd	sl, sl, [r5, #112]	@ 0x70
 8005b06:	e9c5 aa14 	strd	sl, sl, [r5, #80]	@ 0x50
 8005b0a:	f885 9133 	strb.w	r9, [r5, #307]	@ 0x133
 8005b0e:	f8a5 a11c 	strh.w	sl, [r5, #284]	@ 0x11c
 8005b12:	bf38      	it	cc
 8005b14:	4651      	movcc	r1, sl
 8005b16:	f885 1134 	strb.w	r1, [r5, #308]	@ 0x134
 8005b1a:	f7fe fc0d 	bl	8004338 <defmt::export::acquire_and_header>
 8005b1e:	f240 0007 	movw	r0, #7
 8005b22:	f1a7 0414 	sub.w	r4, r7, #20
 8005b26:	f2c0 0000 	movt	r0, #0
 8005b2a:	2102      	movs	r1, #2
 8005b2c:	f827 0c14 	strh.w	r0, [r7, #-20]
 8005b30:	4620      	mov	r0, r4
 8005b32:	f7fe fdb3 	bl	800469c <_defmt_write>
 8005b36:	f240 0032 	movw	r0, #50	@ 0x32
 8005b3a:	2102      	movs	r1, #2
 8005b3c:	f2c0 0000 	movt	r0, #0
 8005b40:	f827 0c14 	strh.w	r0, [r7, #-20]
 8005b44:	4620      	mov	r0, r4
 8005b46:	f7fe fda9 	bl	800469c <_defmt_write>
 8005b4a:	f240 0502 	movw	r5, #2
 8005b4e:	4620      	mov	r0, r4
 8005b50:	f2c0 0500 	movt	r5, #0
 8005b54:	2102      	movs	r1, #2
 8005b56:	f827 5c14 	strh.w	r5, [r7, #-20]
 8005b5a:	f7fe fd9f 	bl	800469c <_defmt_write>
 8005b5e:	200a      	movs	r0, #10
 8005b60:	2101      	movs	r1, #1
 8005b62:	f807 0c14 	strb.w	r0, [r7, #-20]
 8005b66:	4620      	mov	r0, r4
 8005b68:	f7fe fd98 	bl	800469c <_defmt_write>
 8005b6c:	4620      	mov	r0, r4
 8005b6e:	2102      	movs	r1, #2
 8005b70:	f827 5c14 	strh.w	r5, [r7, #-20]
 8005b74:	f7fe fd92 	bl	800469c <_defmt_write>
 8005b78:	4620      	mov	r0, r4
 8005b7a:	2101      	movs	r1, #1
 8005b7c:	f807 ac14 	strb.w	sl, [r7, #-20]
 8005b80:	f7fe fd8c 	bl	800469c <_defmt_write>
 8005b84:	4620      	mov	r0, r4
 8005b86:	2102      	movs	r1, #2
 8005b88:	f827 5c14 	strh.w	r5, [r7, #-20]
 8005b8c:	f7fe fd86 	bl	800469c <_defmt_write>
 8005b90:	4620      	mov	r0, r4
 8005b92:	2101      	movs	r1, #1
 8005b94:	f807 ac14 	strb.w	sl, [r7, #-20]
 8005b98:	f7fe fd80 	bl	800469c <_defmt_write>
 8005b9c:	4620      	mov	r0, r4
 8005b9e:	2102      	movs	r1, #2
 8005ba0:	f827 5c14 	strh.w	r5, [r7, #-20]
 8005ba4:	f7fe fd7a 	bl	800469c <_defmt_write>
 8005ba8:	4620      	mov	r0, r4
 8005baa:	2101      	movs	r1, #1
 8005bac:	f807 9c14 	strb.w	r9, [r7, #-20]
 8005bb0:	f7fe fd74 	bl	800469c <_defmt_write>
 8005bb4:	4620      	mov	r0, r4
 8005bb6:	2102      	movs	r1, #2
 8005bb8:	f827 ac14 	strh.w	sl, [r7, #-20]
 8005bbc:	f7fe fd6e 	bl	800469c <_defmt_write>
 8005bc0:	f7fe fcf4 	bl	80045ac <_defmt_release>
 8005bc4:	f002 fe0f 	bl	80087e6 <__primask_r>
 8005bc8:	4604      	mov	r4, r0
 8005bca:	f002 fdff 	bl	80087cc <__cpsid>
 8005bce:	07e0      	lsls	r0, r4, #31
 8005bd0:	e9d8 5902 	ldrd	r5, r9, [r8, #8]
 8005bd4:	bf08      	it	eq
 8005bd6:	f002 fdfb 	bleq	80087d0 <__cpsie>
 8005bda:	f002 fe04 	bl	80087e6 <__primask_r>
 8005bde:	4604      	mov	r4, r0
 8005be0:	f002 fdf4 	bl	80087cc <__cpsid>
 8005be4:	07e0      	lsls	r0, r4, #31
 8005be6:	bf08      	it	eq
 8005be8:	f002 fdf2 	bleq	80087d0 <__cpsie>
 8005bec:	f44f 717a 	mov.w	r1, #1000	@ 0x3e8
 8005bf0:	f10d 0e04 	add.w	lr, sp, #4
 8005bf4:	fba5 b001 	umull	fp, r0, r5, r1
 8005bf8:	fb09 0401 	mla	r4, r9, r1, r0
 8005bfc:	f50d 50df 	add.w	r0, sp, #7136	@ 0x1be0
 8005c00:	f50e 51c8 	add.w	r1, lr, #6400	@ 0x1900
 8005c04:	e9cd 1000 	strd	r1, r0, [sp]
 8005c08:	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
 8005c0c:	465a      	mov	r2, fp
 8005c0e:	4623      	mov	r3, r4
 8005c10:	f7fb fa84 	bl	800111c <smoltcp::iface::interface::Interface::poll>
 8005c14:	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
 8005c18:	2900      	cmp	r1, #0
 8005c1a:	f000 816f 	beq.w	8005efc <ip::__cortex_m_rt_main+0x157c>
 8005c1e:	f8d6 58e8 	ldr.w	r5, [r6, #2280]	@ 0x8e8
 8005c22:	6828      	ldr	r0, [r5, #0]
 8005c24:	43c1      	mvns	r1, r0
 8005c26:	0789      	lsls	r1, r1, #30
 8005c28:	f000 80ff 	beq.w	8005e2a <ip::__cortex_m_rt_main+0x14aa>
 8005c2c:	2802      	cmp	r0, #2
 8005c2e:	f000 8107 	beq.w	8005e40 <ip::__cortex_m_rt_main+0x14c0>
 8005c32:	f895 0133 	ldrb.w	r0, [r5, #307]	@ 0x133
 8005c36:	2800      	cmp	r0, #0
 8005c38:	bf18      	it	ne
 8005c3a:	280a      	cmpne	r0, #10
 8005c3c:	f43f af28 	beq.w	8005a90 <ip::__cortex_m_rt_main+0x1110>
 8005c40:	2807      	cmp	r0, #7
 8005c42:	bf18      	it	ne
 8005c44:	2804      	cmpne	r0, #4
 8005c46:	d1bd      	bne.n	8005bc4 <ip::__cortex_m_rt_main+0x1244>
 8005c48:	f8d5 60d4 	ldr.w	r6, [r5, #212]	@ 0xd4
 8005c4c:	2e00      	cmp	r6, #0
 8005c4e:	bf08      	it	eq
 8005c50:	f8c5 a0d0 	streq.w	sl, [r5, #208]	@ 0xd0
 8005c54:	f8d5 90cc 	ldr.w	r9, [r5, #204]	@ 0xcc
 8005c58:	f1b9 0f00 	cmp.w	r9, #0
 8005c5c:	d007      	beq.n	8005c6e <ip::__cortex_m_rt_main+0x12ee>
 8005c5e:	f8d5 00d0 	ldr.w	r0, [r5, #208]	@ 0xd0
 8005c62:	4430      	add	r0, r6
 8005c64:	fbb0 f1f9 	udiv	r1, r0, r9
 8005c68:	fb01 0019 	mls	r0, r1, r9, r0
 8005c6c:	e000      	b.n	8005c70 <ip::__cortex_m_rt_main+0x12f0>
 8005c6e:	2000      	movs	r0, #0
 8005c70:	eba9 0806 	sub.w	r8, r9, r6
 8005c74:	eba9 0100 	sub.w	r1, r9, r0
 8005c78:	4541      	cmp	r1, r8
 8005c7a:	bf38      	it	cc
 8005c7c:	4688      	movcc	r8, r1
 8005c7e:	eb18 0100 	adds.w	r1, r8, r0
 8005c82:	f080 80e8 	bcs.w	8005e56 <ip::__cortex_m_rt_main+0x14d6>
 8005c86:	4549      	cmp	r1, r9
 8005c88:	f200 80e5 	bhi.w	8005e56 <ip::__cortex_m_rt_main+0x14d6>
 8005c8c:	f8d5 10c8 	ldr.w	r1, [r5, #200]	@ 0xc8
 8005c90:	f1b8 0f06 	cmp.w	r8, #6
 8005c94:	bf28      	it	cs
 8005c96:	f04f 0806 	movcs.w	r8, #6
 8005c9a:	9106      	str	r1, [sp, #24]
 8005c9c:	4408      	add	r0, r1
 8005c9e:	f24a 71f0 	movw	r1, #42992	@ 0xa7f0
 8005ca2:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8005ca6:	4642      	mov	r2, r8
 8005ca8:	f003 fb23 	bl	80092f2 <__aeabi_memcpy>
 8005cac:	eb18 0206 	adds.w	r2, r8, r6
 8005cb0:	bf08      	it	eq
 8005cb2:	f8c5 a0d0 	streq.w	sl, [r5, #208]	@ 0xd0
 8005cb6:	f1b9 0f00 	cmp.w	r9, #0
 8005cba:	9407      	str	r4, [sp, #28]
 8005cbc:	d007      	beq.n	8005cce <ip::__cortex_m_rt_main+0x134e>
 8005cbe:	f8d5 00d0 	ldr.w	r0, [r5, #208]	@ 0xd0
 8005cc2:	4410      	add	r0, r2
 8005cc4:	fbb0 f1f9 	udiv	r1, r0, r9
 8005cc8:	fb01 0019 	mls	r0, r1, r9, r0
 8005ccc:	e000      	b.n	8005cd0 <ip::__cortex_m_rt_main+0x1350>
 8005cce:	2000      	movs	r0, #0
 8005cd0:	eba9 0402 	sub.w	r4, r9, r2
 8005cd4:	eba9 0100 	sub.w	r1, r9, r0
 8005cd8:	42a1      	cmp	r1, r4
 8005cda:	bf38      	it	cc
 8005cdc:	460c      	movcc	r4, r1
 8005cde:	1821      	adds	r1, r4, r0
 8005ce0:	f080 80b9 	bcs.w	8005e56 <ip::__cortex_m_rt_main+0x14d6>
 8005ce4:	4549      	cmp	r1, r9
 8005ce6:	f200 80b6 	bhi.w	8005e56 <ip::__cortex_m_rt_main+0x14d6>
 8005cea:	f1c8 0106 	rsb	r1, r8, #6
 8005cee:	46b1      	mov	r9, r6
 8005cf0:	42a1      	cmp	r1, r4
 8005cf2:	bf38      	it	cc
 8005cf4:	460c      	movcc	r4, r1
 8005cf6:	9906      	ldr	r1, [sp, #24]
 8005cf8:	4616      	mov	r6, r2
 8005cfa:	4622      	mov	r2, r4
 8005cfc:	4408      	add	r0, r1
 8005cfe:	f24a 71f0 	movw	r1, #42992	@ 0xa7f0
 8005d02:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8005d06:	4441      	add	r1, r8
 8005d08:	f003 faf3 	bl	80092f2 <__aeabi_memcpy>
 8005d0c:	19a0      	adds	r0, r4, r6
 8005d0e:	f1b9 0f00 	cmp.w	r9, #0
 8005d12:	f8c5 00d4 	str.w	r0, [r5, #212]	@ 0xd4
 8005d16:	d104      	bne.n	8005d22 <ip::__cortex_m_rt_main+0x13a2>
 8005d18:	ea54 0008 	orrs.w	r0, r4, r8
 8005d1c:	bf18      	it	ne
 8005d1e:	e9c5 aa14 	strdne	sl, sl, [r5, #80]	@ 0x50
 8005d22:	9807      	ldr	r0, [sp, #28]
 8005d24:	f51b 747a 	adds.w	r4, fp, #1000	@ 0x3e8
 8005d28:	f10d 0e18 	add.w	lr, sp, #24
 8005d2c:	f240 0840 	movw	r8, #64	@ 0x40
 8005d30:	f140 0500 	adc.w	r5, r0, #0
 8005d34:	f50e 5697 	add.w	r6, lr, #4832	@ 0x12e0
 8005d38:	f50d 59df 	add.w	r9, sp, #7136	@ 0x1be0
 8005d3c:	f2c2 0800 	movt	r8, #8192	@ 0x2000
 8005d40:	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
 8005d44:	2900      	cmp	r1, #0
 8005d46:	f000 80b9 	beq.w	8005ebc <ip::__cortex_m_rt_main+0x153c>
 8005d4a:	f8d6 08e8 	ldr.w	r0, [r6, #2280]	@ 0x8e8
 8005d4e:	6801      	ldr	r1, [r0, #0]
 8005d50:	43ca      	mvns	r2, r1
 8005d52:	0792      	lsls	r2, r2, #30
 8005d54:	d045      	beq.n	8005de2 <ip::__cortex_m_rt_main+0x1462>
 8005d56:	2902      	cmp	r1, #2
 8005d58:	d04e      	beq.n	8005df8 <ip::__cortex_m_rt_main+0x1478>
 8005d5a:	f8d0 20d4 	ldr.w	r2, [r0, #212]	@ 0xd4
 8005d5e:	b182      	cbz	r2, 8005d82 <ip::__cortex_m_rt_main+0x1402>
 8005d60:	f10d 0e04 	add.w	lr, sp, #4
 8005d64:	4622      	mov	r2, r4
 8005d66:	f50e 50c8 	add.w	r0, lr, #6400	@ 0x1900
 8005d6a:	e9cd 0900 	strd	r0, r9, [sp]
 8005d6e:	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
 8005d72:	462b      	mov	r3, r5
 8005d74:	f7fb f9d2 	bl	800111c <smoltcp::iface::interface::Interface::poll>
 8005d78:	f8d6 18ec 	ldr.w	r1, [r6, #2284]	@ 0x8ec
 8005d7c:	2900      	cmp	r1, #0
 8005d7e:	d1e4      	bne.n	8005d4a <ip::__cortex_m_rt_main+0x13ca>
 8005d80:	e09c      	b.n	8005ebc <ip::__cortex_m_rt_main+0x153c>
 8005d82:	f001 0103 	and.w	r1, r1, #3
 8005d86:	2903      	cmp	r1, #3
 8005d88:	d04f      	beq.n	8005e2a <ip::__cortex_m_rt_main+0x14aa>
 8005d8a:	2902      	cmp	r1, #2
 8005d8c:	d058      	beq.n	8005e40 <ip::__cortex_m_rt_main+0x14c0>
 8005d8e:	f880 a133 	strb.w	sl, [r0, #307]	@ 0x133
 8005d92:	f240 002d 	movw	r0, #45	@ 0x2d
 8005d96:	f2c0 0000 	movt	r0, #0
 8005d9a:	f7fe fade 	bl	800435a <defmt::export::acquire_header_and_release>
 8005d9e:	f10d 0e04 	add.w	lr, sp, #4
 8005da2:	9b07      	ldr	r3, [sp, #28]
 8005da4:	f50e 50c8 	add.w	r0, lr, #6400	@ 0x1900
 8005da8:	e9cd 0900 	strd	r0, r9, [sp]
 8005dac:	f50d 50c9 	add.w	r0, sp, #6432	@ 0x1920
 8005db0:	465a      	mov	r2, fp
 8005db2:	f7fb f9b3 	bl	800111c <smoltcp::iface::interface::Interface::poll>
 8005db6:	e705      	b.n	8005bc4 <ip::__cortex_m_rt_main+0x1244>
 8005db8:	f649 00c0 	movw	r0, #39104	@ 0x98c0
 8005dbc:	f24a 324c 	movw	r2, #41804	@ 0xa34c
 8005dc0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005dc4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005dc8:	2147      	movs	r1, #71	@ 0x47
 8005dca:	f7fd fcfb 	bl	80037c4 <core::panicking::panic_fmt>
 8005dce:	f1a0 0360 	sub.w	r3, r0, #96	@ 0x60
 8005dd2:	2b60      	cmp	r3, #96	@ 0x60
 8005dd4:	d21b      	bcs.n	8005e0e <ip::__cortex_m_rt_main+0x148e>
 8005dd6:	ed9f 2a55 	vldr	s4, [pc, #340]	@ 8005f2c <ip::__cortex_m_rt_main+0x15ac>
 8005dda:	f04f 0ed0 	mov.w	lr, #208	@ 0xd0
 8005dde:	f7fe bec4 	b.w	8004b6a <ip::__cortex_m_rt_main+0x1ea>
 8005de2:	f24a 3098 	movw	r0, #41880	@ 0xa398
 8005de6:	f24a 32c0 	movw	r2, #41920	@ 0xa3c0
 8005dea:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005dee:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005df2:	214f      	movs	r1, #79	@ 0x4f
 8005df4:	f7fd fce6 	bl	80037c4 <core::panicking::panic_fmt>
 8005df8:	f24a 305c 	movw	r0, #41820	@ 0xa35c
 8005dfc:	f24a 32d0 	movw	r2, #41936	@ 0xa3d0
 8005e00:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005e04:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005e08:	2129      	movs	r1, #41	@ 0x29
 8005e0a:	f7fe f914 	bl	8004036 <core::option::expect_failed>
 8005e0e:	38c0      	subs	r0, #192	@ 0xc0
 8005e10:	a347      	add	r3, pc, #284	@ (adr r3, 8005f30 <ip::__cortex_m_rt_main+0x15b0>)
 8005e12:	f04f 0ef0 	mov.w	lr, #240	@ 0xf0
 8005e16:	28c0      	cmp	r0, #192	@ 0xc0
 8005e18:	bf38      	it	cc
 8005e1a:	3304      	addcc	r3, #4
 8005e1c:	ed93 2a00 	vldr	s4, [r3]
 8005e20:	bf38      	it	cc
 8005e22:	f04f 0ee0 	movcc.w	lr, #224	@ 0xe0
 8005e26:	f7fe bea0 	b.w	8004b6a <ip::__cortex_m_rt_main+0x1ea>
 8005e2a:	f24a 3098 	movw	r0, #41880	@ 0xa398
 8005e2e:	f24a 32f0 	movw	r2, #41968	@ 0xa3f0
 8005e32:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005e36:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005e3a:	214f      	movs	r1, #79	@ 0x4f
 8005e3c:	f7fd fcc2 	bl	80037c4 <core::panicking::panic_fmt>
 8005e40:	f24a 305c 	movw	r0, #41820	@ 0xa35c
 8005e44:	f24a 4200 	movw	r2, #41984	@ 0xa400
 8005e48:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005e4c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005e50:	2129      	movs	r1, #41	@ 0x29
 8005e52:	f7fe f8f0 	bl	8004036 <core::option::expect_failed>
 8005e56:	f64a 53fc 	movw	r3, #44540	@ 0xadfc
 8005e5a:	464a      	mov	r2, r9
 8005e5c:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8005e60:	f7fd fc3f 	bl	80036e2 <core::slice::index::slice_index_fail>
 8005e64:	f24b 00e0 	movw	r0, #45280	@ 0xb0e0
 8005e68:	f24b 1208 	movw	r2, #45320	@ 0xb108
 8005e6c:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005e70:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005e74:	2127      	movs	r1, #39	@ 0x27
 8005e76:	f7fd fcc7 	bl	8003808 <core::panicking::panic>
 8005e7a:	f24b 0098 	movw	r0, #45208	@ 0xb098
 8005e7e:	f24b 1218 	movw	r2, #45336	@ 0xb118
 8005e82:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005e86:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005e8a:	2128      	movs	r1, #40	@ 0x28
 8005e8c:	f7fd fcbc 	bl	8003808 <core::panicking::panic>
 8005e90:	f24b 0098 	movw	r0, #45208	@ 0xb098
 8005e94:	f24b 1228 	movw	r2, #45352	@ 0xb128
 8005e98:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005e9c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005ea0:	2128      	movs	r1, #40	@ 0x28
 8005ea2:	f7fd fcb1 	bl	8003808 <core::panicking::panic>
 8005ea6:	f24b 0098 	movw	r0, #45208	@ 0xb098
 8005eaa:	f24b 1238 	movw	r2, #45368	@ 0xb138
 8005eae:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005eb2:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005eb6:	2128      	movs	r1, #40	@ 0x28
 8005eb8:	f7fd fca6 	bl	8003808 <core::panicking::panic>
 8005ebc:	f24a 3288 	movw	r2, #41864	@ 0xa388
 8005ec0:	2000      	movs	r0, #0
 8005ec2:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005ec6:	f7fd fc8b 	bl	80037e0 <core::panicking::panic_bounds_check>
 8005eca:	f10d 0e08 	add.w	lr, sp, #8
 8005ece:	f643 60fb 	movw	r0, #16123	@ 0x3efb
 8005ed2:	f50e 51ef 	add.w	r1, lr, #7648	@ 0x1de0
 8005ed6:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005eda:	900b      	str	r0, [sp, #44]	@ 0x2c
 8005edc:	f24b 02c0 	movw	r2, #45248	@ 0xb0c0
 8005ee0:	e9cd 0109 	strd	r0, r1, [sp, #36]	@ 0x24
 8005ee4:	f50d 604a 	add.w	r0, sp, #3232	@ 0xca0
 8005ee8:	9008      	str	r0, [sp, #32]
 8005eea:	f24a 3001 	movw	r0, #41729	@ 0xa301
 8005eee:	a908      	add	r1, sp, #32
 8005ef0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005ef4:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005ef8:	f7fd fc64 	bl	80037c4 <core::panicking::panic_fmt>
 8005efc:	f24a 32e0 	movw	r2, #41952	@ 0xa3e0
 8005f00:	2000      	movs	r0, #0
 8005f02:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005f06:	f7fd fc6b 	bl	80037e0 <core::panicking::panic_bounds_check>
 8005f0a:	f002 fc61 	bl	80087d0 <__cpsie>
 8005f0e:	f24a 607c 	movw	r0, #42620	@ 0xa67c
 8005f12:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005f16:	f7fd fbda 	bl	80036ce <core::option::unwrap_failed>
 8005f1a:	f002 fc59 	bl	80087d0 <__cpsie>
 8005f1e:	f24a 608c 	movw	r0, #42636	@ 0xa68c
 8005f22:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8005f26:	f7fd fbd2 	bl	80036ce <core::option::unwrap_failed>
 8005f2a:	bf00      	nop
 8005f2c:	43000000 	.word	0x43000000
 8005f30:	44000000 	.word	0x44000000
 8005f34:	43800000 	.word	0x43800000

08005f38 <<stm32_eth::mac::WrongClock as core::fmt::Debug>::fmt>:
 8005f38:	b580      	push	{r7, lr}
 8005f3a:	466f      	mov	r7, sp
 8005f3c:	e9d1 0200 	ldrd	r0, r2, [r1]
 8005f40:	f64a 0144 	movw	r1, #43076	@ 0xa844
 8005f44:	68d3      	ldr	r3, [r2, #12]
 8005f46:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8005f4a:	220a      	movs	r2, #10
 8005f4c:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8005f50:	4718      	bx	r3

08005f52 <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>>:
 8005f52:	b5b0      	push	{r4, r5, r7, lr}
 8005f54:	af02      	add	r7, sp, #8
 8005f56:	6802      	ldr	r2, [r0, #0]
 8005f58:	2a02      	cmp	r2, #2
 8005f5a:	bf08      	it	eq
 8005f5c:	bdb0      	popeq	{r4, r5, r7, pc}
 8005f5e:	e9d0 3c02 	ldrd	r3, ip, [r0, #8]
 8005f62:	6859      	ldr	r1, [r3, #4]
 8005f64:	458c      	cmp	ip, r1
 8005f66:	d230      	bcs.n	8005fca <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>+0x78>
 8005f68:	f44f 61c6 	mov.w	r1, #1584	@ 0x630
 8005f6c:	681b      	ldr	r3, [r3, #0]
 8005f6e:	fb0c 3101 	mla	r1, ip, r1, r3
 8005f72:	f8d0 e010 	ldr.w	lr, [r0, #16]
 8005f76:	2500      	movs	r5, #0
 8005f78:	f2cf 05d0 	movt	r5, #61648	@ 0xf0d0
 8005f7c:	07d2      	lsls	r2, r2, #31
 8005f7e:	d007      	beq.n	8005f90 <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>+0x3e>
 8005f80:	6840      	ldr	r0, [r0, #4]
 8005f82:	f04f 0c01 	mov.w	ip, #1
 8005f86:	f105 7500 	add.w	r5, r5, #33554432	@ 0x2000000
 8005f8a:	e9c1 c008 	strd	ip, r0, [r1, #32]
 8005f8e:	e001      	b.n	8005f94 <core::ptr::drop_in_place<core::result::Result<stm32_eth::dma::tx::TxPacket,stm32_eth::dma::tx::TxError>>+0x42>
 8005f90:	2000      	movs	r0, #0
 8005f92:	6208      	str	r0, [r1, #32]
 8005f94:	6848      	ldr	r0, [r1, #4]
 8005f96:	f640 74ff 	movw	r4, #4095	@ 0xfff
 8005f9a:	e9d1 320a 	ldrd	r3, r2, [r1, #40]	@ 0x28
 8005f9e:	43a0      	bics	r0, r4
 8005fa0:	ea40 000e 	orr.w	r0, r0, lr
 8005fa4:	6048      	str	r0, [r1, #4]
 8005fa6:	608b      	str	r3, [r1, #8]
 8005fa8:	60ca      	str	r2, [r1, #12]
 8005faa:	f3bf 8f5f 	dmb	sy
 8005fae:	f891 0030 	ldrb.w	r0, [r1, #48]	@ 0x30
 8005fb2:	ea45 5040 	orr.w	r0, r5, r0, lsl #21
 8005fb6:	6008      	str	r0, [r1, #0]
 8005fb8:	f249 0004 	movw	r0, #36868	@ 0x9004
 8005fbc:	2100      	movs	r1, #0
 8005fbe:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 8005fc2:	f3bf 8f5f 	dmb	sy
 8005fc6:	6001      	str	r1, [r0, #0]
 8005fc8:	bdb0      	pop	{r4, r5, r7, pc}
 8005fca:	f64a 7250 	movw	r2, #44880	@ 0xaf50
 8005fce:	4660      	mov	r0, ip
 8005fd0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8005fd4:	f7fd fc04 	bl	80037e0 <core::panicking::panic_bounds_check>

08005fd8 <__rustc::rust_begin_unwind>:
 8005fd8:	b580      	push	{r7, lr}
 8005fda:	466f      	mov	r7, sp
 8005fdc:	b086      	sub	sp, #24
 8005fde:	4604      	mov	r4, r0
 8005fe0:	f002 fbf4 	bl	80087cc <__cpsid>
 8005fe4:	f240 0038 	movw	r0, #56	@ 0x38
 8005fe8:	f2c2 0000 	movt	r0, #8192	@ 0x2000
 8005fec:	7801      	ldrb	r1, [r0, #0]
 8005fee:	bba9      	cbnz	r1, 800605c <__rustc::rust_begin_unwind+0x84>
 8005ff0:	2101      	movs	r1, #1
 8005ff2:	7001      	strb	r1, [r0, #0]
 8005ff4:	f240 002e 	movw	r0, #46	@ 0x2e
 8005ff8:	f2c0 0000 	movt	r0, #0
 8005ffc:	9400      	str	r4, [sp, #0]
 8005ffe:	f7fe f99b 	bl	8004338 <defmt::export::acquire_and_header>
 8006002:	f240 0001 	movw	r0, #1
 8006006:	2102      	movs	r1, #2
 8006008:	f2c0 0000 	movt	r0, #0
 800600c:	f8ad 000c 	strh.w	r0, [sp, #12]
 8006010:	a803      	add	r0, sp, #12
 8006012:	f7fe fb43 	bl	800469c <_defmt_write>
 8006016:	f64a 1014 	movw	r0, #43284	@ 0xa914
 800601a:	f24a 6124 	movw	r1, #42532	@ 0xa624
 800601e:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8006022:	f649 42d1 	movw	r2, #40145	@ 0x9cd1
 8006026:	9002      	str	r0, [sp, #8]
 8006028:	4668      	mov	r0, sp
 800602a:	9001      	str	r0, [sp, #4]
 800602c:	f643 608f 	movw	r0, #16015	@ 0x3e8f
 8006030:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8006034:	ab03      	add	r3, sp, #12
 8006036:	9004      	str	r0, [sp, #16]
 8006038:	a801      	add	r0, sp, #4
 800603a:	9003      	str	r0, [sp, #12]
 800603c:	1e78      	subs	r0, r7, #1
 800603e:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8006042:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8006046:	f7fd f9cc 	bl	80033e2 <core::fmt::write>
 800604a:	f24a 603c 	movw	r0, #42556	@ 0xa63c
 800604e:	2101      	movs	r1, #1
 8006050:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8006054:	f7fe fb22 	bl	800469c <_defmt_write>
 8006058:	f7fe faa8 	bl	80045ac <_defmt_release>
 800605c:	f000 f800 	bl	8006060 <panic_probe::hard_fault>

08006060 <panic_probe::hard_fault>:
 8006060:	b580      	push	{r7, lr}
 8006062:	466f      	mov	r7, sp
 8006064:	f64e 5024 	movw	r0, #60708	@ 0xed24
 8006068:	f2ce 0000 	movt	r0, #57344	@ 0xe000
 800606c:	6801      	ldr	r1, [r0, #0]
 800606e:	f421 2180 	bic.w	r1, r1, #262144	@ 0x40000
 8006072:	6001      	str	r1, [r0, #0]
 8006074:	f002 fbba 	bl	80087ec <__udf>

08006078 <<&T as core::fmt::Display>::fmt>:
 8006078:	b5f0      	push	{r4, r5, r6, r7, lr}
 800607a:	af03      	add	r7, sp, #12
 800607c:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 8006080:	b088      	sub	sp, #32
 8006082:	e9d1 5400 	ldrd	r5, r4, [r1]
 8006086:	f24a 6114 	movw	r1, #42516	@ 0xa614
 800608a:	6800      	ldr	r0, [r0, #0]
 800608c:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8006090:	f8d4 900c 	ldr.w	r9, [r4, #12]
 8006094:	220c      	movs	r2, #12
 8006096:	e9d0 8600 	ldrd	r8, r6, [r0]
 800609a:	4628      	mov	r0, r5
 800609c:	47c8      	blx	r9
 800609e:	b120      	cbz	r0, 80060aa <<&T as core::fmt::Display>::fmt+0x32>
 80060a0:	2001      	movs	r0, #1
 80060a2:	b008      	add	sp, #32
 80060a4:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80060a8:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80060aa:	e9d6 0100 	ldrd	r0, r1, [r6]
 80060ae:	f24a 321f 	movw	r2, #41759	@ 0xa31f
 80060b2:	e9cd 0100 	strd	r0, r1, [sp]
 80060b6:	f643 60fb 	movw	r0, #16123	@ 0x3efb
 80060ba:	f106 010c 	add.w	r1, r6, #12
 80060be:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80060c2:	9007      	str	r0, [sp, #28]
 80060c4:	ab02      	add	r3, sp, #8
 80060c6:	e9cd 0105 	strd	r0, r1, [sp, #20]
 80060ca:	f106 0008 	add.w	r0, r6, #8
 80060ce:	9004      	str	r0, [sp, #16]
 80060d0:	f643 0015 	movw	r0, #14357	@ 0x3815
 80060d4:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80060d8:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80060dc:	9003      	str	r0, [sp, #12]
 80060de:	4668      	mov	r0, sp
 80060e0:	9002      	str	r0, [sp, #8]
 80060e2:	4628      	mov	r0, r5
 80060e4:	4621      	mov	r1, r4
 80060e6:	f7fd f97c 	bl	80033e2 <core::fmt::write>
 80060ea:	b120      	cbz	r0, 80060f6 <<&T as core::fmt::Display>::fmt+0x7e>
 80060ec:	2001      	movs	r0, #1
 80060ee:	b008      	add	sp, #32
 80060f0:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80060f4:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80060f6:	f24a 6120 	movw	r1, #42528	@ 0xa620
 80060fa:	4628      	mov	r0, r5
 80060fc:	f6c0 0100 	movt	r1, #2048	@ 0x800
 8006100:	2202      	movs	r2, #2
 8006102:	47c8      	blx	r9
 8006104:	b120      	cbz	r0, 8006110 <<&T as core::fmt::Display>::fmt+0x98>
 8006106:	2001      	movs	r0, #1
 8006108:	b008      	add	sp, #32
 800610a:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 800610e:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8006110:	e9d8 2300 	ldrd	r2, r3, [r8]
 8006114:	4628      	mov	r0, r5
 8006116:	4621      	mov	r1, r4
 8006118:	f7fd f963 	bl	80033e2 <core::fmt::write>
 800611c:	b008      	add	sp, #32
 800611e:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8006122:	bdf0      	pop	{r4, r5, r6, r7, pc}

08006124 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold>:
 8006124:	b5b0      	push	{r4, r5, r7, lr}
 8006126:	af02      	add	r7, sp, #8
 8006128:	4288      	cmp	r0, r1
 800612a:	d02c      	beq.n	8006186 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x62>
 800612c:	f64a 23ab 	movw	r3, #43691	@ 0xaaab
 8006130:	1a09      	subs	r1, r1, r0
 8006132:	f6ca 23aa 	movt	r3, #43690	@ 0xaaaa
 8006136:	6815      	ldr	r5, [r2, #0]
 8006138:	fba1 1303 	umull	r1, r3, r1, r3
 800613c:	4611      	mov	r1, r2
 800613e:	688c      	ldr	r4, [r1, #8]
 8006140:	6852      	ldr	r2, [r2, #4]
 8006142:	f105 0e04 	add.w	lr, r5, #4
 8006146:	08d9      	lsrs	r1, r3, #3
 8006148:	1d03      	adds	r3, r0, #4
 800614a:	00e0      	lsls	r0, r4, #3
 800614c:	e007      	b.n	800615e <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x3a>
 800614e:	2400      	movs	r4, #0
 8006150:	f843 4c04 	str.w	r4, [r3, #-4]
 8006154:	3008      	adds	r0, #8
 8006156:	e8e3 5c03 	strd	r5, ip, [r3], #12
 800615a:	3901      	subs	r1, #1
 800615c:	d013      	beq.n	8006186 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x62>
 800615e:	4290      	cmp	r0, r2
 8006160:	d2f5      	bcs.n	800614e <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x2a>
 8006162:	f100 0c04 	add.w	ip, r0, #4
 8006166:	4594      	cmp	ip, r2
 8006168:	d817      	bhi.n	800619a <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x76>
 800616a:	1dc4      	adds	r4, r0, #7
 800616c:	4294      	cmp	r4, r2
 800616e:	d20b      	bcs.n	8006188 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x64>
 8006170:	eb0e 0400 	add.w	r4, lr, r0
 8006174:	f85e 5000 	ldr.w	r5, [lr, r0]
 8006178:	f854 4c04 	ldr.w	r4, [r4, #-4]
 800617c:	fa95 fc85 	rev.w	ip, r5
 8006180:	ba25      	rev	r5, r4
 8006182:	2401      	movs	r4, #1
 8006184:	e7e4      	b.n	8006150 <<core::slice::iter::IterMut<T> as core::iter::traits::iterator::Iterator>::fold+0x2c>
 8006186:	bdb0      	pop	{r4, r5, r7, pc}
 8006188:	f64a 3350 	movw	r3, #43856	@ 0xab50
 800618c:	f100 0108 	add.w	r1, r0, #8
 8006190:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8006194:	4660      	mov	r0, ip
 8006196:	f7fd faa4 	bl	80036e2 <core::slice::index::slice_index_fail>
 800619a:	f64a 3360 	movw	r3, #43872	@ 0xab60
 800619e:	4661      	mov	r1, ip
 80061a0:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80061a4:	f7fd fa9d 	bl	80036e2 <core::slice::index::slice_index_fail>

080061a8 <smoltcp::wire::tcp::TcpOption::emit>:
 80061a8:	b5f0      	push	{r4, r5, r6, r7, lr}
 80061aa:	af03      	add	r7, sp, #12
 80061ac:	f84d bd04 	str.w	fp, [sp, #-4]!
 80061b0:	6803      	ldr	r3, [r0, #0]
 80061b2:	2405      	movs	r4, #5
 80061b4:	2504      	movs	r5, #4
 80061b6:	2b01      	cmp	r3, #1
 80061b8:	bf88      	it	hi
 80061ba:	1e9c      	subhi	r4, r3, #2
 80061bc:	e8df f004 	tbb	[pc, r4]
 80061c0:	13251904 	.word	0x13251904
 80061c4:	17391f0f 	.word	0x17391f0f
 80061c8:	b13a      	cbz	r2, 80061da <smoltcp::wire::tcp::TcpOption::emit+0x32>
 80061ca:	4608      	mov	r0, r1
 80061cc:	460c      	mov	r4, r1
 80061ce:	4611      	mov	r1, r2
 80061d0:	4615      	mov	r5, r2
 80061d2:	f003 f8e2 	bl	800939a <__aeabi_memclr>
 80061d6:	4621      	mov	r1, r4
 80061d8:	462a      	mov	r2, r5
 80061da:	2501      	movs	r5, #1
 80061dc:	e07d      	b.n	80062da <smoltcp::wire::tcp::TcpOption::emit+0x132>
 80061de:	2502      	movs	r5, #2
 80061e0:	2a01      	cmp	r2, #1
 80061e2:	d814      	bhi.n	800620e <smoltcp::wire::tcp::TcpOption::emit+0x66>
 80061e4:	e028      	b.n	8006238 <smoltcp::wire::tcp::TcpOption::emit+0x90>
 80061e6:	2503      	movs	r5, #3
 80061e8:	2a01      	cmp	r2, #1
 80061ea:	d810      	bhi.n	800620e <smoltcp::wire::tcp::TcpOption::emit+0x66>
 80061ec:	e024      	b.n	8006238 <smoltcp::wire::tcp::TcpOption::emit+0x90>
 80061ee:	6886      	ldr	r6, [r0, #8]
 80061f0:	e00a      	b.n	8006208 <smoltcp::wire::tcp::TcpOption::emit+0x60>
 80061f2:	2a00      	cmp	r2, #0
 80061f4:	f000 80da 	beq.w	80063ac <smoltcp::wire::tcp::TcpOption::emit+0x204>
 80061f8:	2501      	movs	r5, #1
 80061fa:	700d      	strb	r5, [r1, #0]
 80061fc:	e06d      	b.n	80062da <smoltcp::wire::tcp::TcpOption::emit+0x132>
 80061fe:	68c6      	ldr	r6, [r0, #12]
 8006200:	6985      	ldr	r5, [r0, #24]
 8006202:	441e      	add	r6, r3
 8006204:	442e      	add	r6, r5
 8006206:	00f6      	lsls	r6, r6, #3
 8006208:	1cb5      	adds	r5, r6, #2
 800620a:	2a01      	cmp	r2, #1
 800620c:	d914      	bls.n	8006238 <smoltcp::wire::tcp::TcpOption::emit+0x90>
 800620e:	3c02      	subs	r4, #2
 8006210:	704d      	strb	r5, [r1, #1]
 8006212:	e8df f004 	tbb	[pc, r4]
 8006216:	4a03      	.short	0x4a03
 8006218:	521a312e 	.word	0x521a312e
 800621c:	2302      	movs	r3, #2
 800621e:	700b      	strb	r3, [r1, #0]
 8006220:	f002 033e 	and.w	r3, r2, #62	@ 0x3e
 8006224:	2b02      	cmp	r3, #2
 8006226:	f000 80a7 	beq.w	8006378 <smoltcp::wire::tcp::TcpOption::emit+0x1d0>
 800622a:	8880      	ldrh	r0, [r0, #4]
 800622c:	ba40      	rev16	r0, r0
 800622e:	8048      	strh	r0, [r1, #2]
 8006230:	e053      	b.n	80062da <smoltcp::wire::tcp::TcpOption::emit+0x132>
 8006232:	250a      	movs	r5, #10
 8006234:	2a01      	cmp	r2, #1
 8006236:	d8ea      	bhi.n	800620e <smoltcp::wire::tcp::TcpOption::emit+0x66>
 8006238:	f64a 23d0 	movw	r3, #43728	@ 0xaad0
 800623c:	2001      	movs	r0, #1
 800623e:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8006242:	4611      	mov	r1, r2
 8006244:	461a      	mov	r2, r3
 8006246:	f7fd facb 	bl	80037e0 <core::panicking::panic_bounds_check>
 800624a:	2608      	movs	r6, #8
 800624c:	e9d0 3001 	ldrd	r3, r0, [r0, #4]
 8006250:	700e      	strb	r6, [r1, #0]
 8006252:	1f96      	subs	r6, r2, #6
 8006254:	f116 0f04 	cmn.w	r6, #4
 8006258:	bf3f      	itttt	cc
 800625a:	ba1b      	revcc	r3, r3
 800625c:	f8c1 3002 	strcc.w	r3, [r1, #2]
 8006260:	f1a2 030a 	subcc.w	r3, r2, #10
 8006264:	f113 0f04 	cmncc.w	r3, #4
 8006268:	d278      	bcs.n	800635c <smoltcp::wire::tcp::TcpOption::emit+0x1b4>
 800626a:	ba00      	rev	r0, r0
 800626c:	f8c1 0006 	str.w	r0, [r1, #6]
 8006270:	e033      	b.n	80062da <smoltcp::wire::tcp::TcpOption::emit+0x132>
 8006272:	2004      	movs	r0, #4
 8006274:	7008      	strb	r0, [r1, #0]
 8006276:	e030      	b.n	80062da <smoltcp::wire::tcp::TcpOption::emit+0x132>
 8006278:	2605      	movs	r6, #5
 800627a:	700e      	strb	r6, [r1, #0]
 800627c:	b3e3      	cbz	r3, 80062f8 <smoltcp::wire::tcp::TcpOption::emit+0x150>
 800627e:	1f93      	subs	r3, r2, #6
 8006280:	f113 0f04 	cmn.w	r3, #4
 8006284:	bf3f      	itttt	cc
 8006286:	e9d0 6301 	ldrdcc	r6, r3, [r0, #4]
 800628a:	ba36      	revcc	r6, r6
 800628c:	f8c1 6002 	strcc.w	r6, [r1, #2]
 8006290:	f1a2 060a 	subcc.w	r6, r2, #10
 8006294:	bf38      	it	cc
 8006296:	f116 0f04 	cmncc.w	r6, #4
 800629a:	d25f      	bcs.n	800635c <smoltcp::wire::tcp::TcpOption::emit+0x1b4>
 800629c:	ba1b      	rev	r3, r3
 800629e:	f8c1 3006 	str.w	r3, [r1, #6]
 80062a2:	2301      	movs	r3, #1
 80062a4:	68c6      	ldr	r6, [r0, #12]
 80062a6:	bb56      	cbnz	r6, 80062fe <smoltcp::wire::tcp::TcpOption::emit+0x156>
 80062a8:	e041      	b.n	800632e <smoltcp::wire::tcp::TcpOption::emit+0x186>
 80062aa:	7900      	ldrb	r0, [r0, #4]
 80062ac:	2303      	movs	r3, #3
 80062ae:	2a02      	cmp	r2, #2
 80062b0:	700b      	strb	r3, [r1, #0]
 80062b2:	f000 8083 	beq.w	80063bc <smoltcp::wire::tcp::TcpOption::emit+0x214>
 80062b6:	7088      	strb	r0, [r1, #2]
 80062b8:	e00f      	b.n	80062da <smoltcp::wire::tcp::TcpOption::emit+0x132>
 80062ba:	e9d0 c301 	ldrd	ip, r3, [r0, #4]
 80062be:	7b00      	ldrb	r0, [r0, #12]
 80062c0:	7008      	strb	r0, [r1, #0]
 80062c2:	1e90      	subs	r0, r2, #2
 80062c4:	4298      	cmp	r0, r3
 80062c6:	d162      	bne.n	800638e <smoltcp::wire::tcp::TcpOption::emit+0x1e6>
 80062c8:	1c88      	adds	r0, r1, #2
 80062ca:	4614      	mov	r4, r2
 80062cc:	460e      	mov	r6, r1
 80062ce:	4661      	mov	r1, ip
 80062d0:	461a      	mov	r2, r3
 80062d2:	f003 f80e 	bl	80092f2 <__aeabi_memcpy>
 80062d6:	4631      	mov	r1, r6
 80062d8:	4622      	mov	r2, r4
 80062da:	42aa      	cmp	r2, r5
 80062dc:	bf21      	itttt	cs
 80062de:	1948      	addcs	r0, r1, r5
 80062e0:	1b51      	subcs	r1, r2, r5
 80062e2:	f85d bb04 	ldrcs.w	fp, [sp], #4
 80062e6:	bdf0      	popcs	{r4, r5, r6, r7, pc}
 80062e8:	f64a 3300 	movw	r3, #43776	@ 0xab00
 80062ec:	4628      	mov	r0, r5
 80062ee:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80062f2:	4611      	mov	r1, r2
 80062f4:	f7fd f9f5 	bl	80036e2 <core::slice::index::slice_index_fail>
 80062f8:	2300      	movs	r3, #0
 80062fa:	68c6      	ldr	r6, [r0, #12]
 80062fc:	b1be      	cbz	r6, 800632e <smoltcp::wire::tcp::TcpOption::emit+0x186>
 80062fe:	00dc      	lsls	r4, r3, #3
 8006300:	f104 0c02 	add.w	ip, r4, #2
 8006304:	4562      	cmp	r2, ip
 8006306:	d349      	bcc.n	800639c <smoltcp::wire::tcp::TcpOption::emit+0x1f4>
 8006308:	eba2 060c 	sub.w	r6, r2, ip
 800630c:	2e04      	cmp	r6, #4
 800630e:	bf21      	itttt	cs
 8006310:	e9d0 6e04 	ldrdcs	r6, lr, [r0, #16]
 8006314:	ba36      	revcs	r6, r6
 8006316:	f841 600c 	strcs.w	r6, [r1, ip]
 800631a:	f044 0406 	orrcs.w	r4, r4, #6
 800631e:	bf24      	itt	cs
 8006320:	1b16      	subcs	r6, r2, r4
 8006322:	2e04      	cmpcs	r6, #4
 8006324:	d31a      	bcc.n	800635c <smoltcp::wire::tcp::TcpOption::emit+0x1b4>
 8006326:	3301      	adds	r3, #1
 8006328:	fa9e f68e 	rev.w	r6, lr
 800632c:	510e      	str	r6, [r1, r4]
 800632e:	6986      	ldr	r6, [r0, #24]
 8006330:	2e00      	cmp	r6, #0
 8006332:	d0d2      	beq.n	80062da <smoltcp::wire::tcp::TcpOption::emit+0x132>
 8006334:	00db      	lsls	r3, r3, #3
 8006336:	f103 0c02 	add.w	ip, r3, #2
 800633a:	4562      	cmp	r2, ip
 800633c:	d32e      	bcc.n	800639c <smoltcp::wire::tcp::TcpOption::emit+0x1f4>
 800633e:	eba2 060c 	sub.w	r6, r2, ip
 8006342:	2e04      	cmp	r6, #4
 8006344:	bf21      	itttt	cs
 8006346:	e9d0 6007 	ldrdcs	r6, r0, [r0, #28]
 800634a:	ba36      	revcs	r6, r6
 800634c:	f841 600c 	strcs.w	r6, [r1, ip]
 8006350:	f043 0306 	orrcs.w	r3, r3, #6
 8006354:	bf24      	itt	cs
 8006356:	1ad6      	subcs	r6, r2, r3
 8006358:	2e04      	cmpcs	r6, #4
 800635a:	d20a      	bcs.n	8006372 <smoltcp::wire::tcp::TcpOption::emit+0x1ca>
 800635c:	f64a 1034 	movw	r0, #43316	@ 0xa934
 8006360:	f64a 1284 	movw	r2, #43396	@ 0xa984
 8006364:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8006368:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800636c:	2120      	movs	r1, #32
 800636e:	f7fd fa4b 	bl	8003808 <core::panicking::panic>
 8006372:	ba00      	rev	r0, r0
 8006374:	50c8      	str	r0, [r1, r3]
 8006376:	e7b0      	b.n	80062da <smoltcp::wire::tcp::TcpOption::emit+0x132>
 8006378:	f64a 1054 	movw	r0, #43348	@ 0xa954
 800637c:	f64a 1274 	movw	r2, #43380	@ 0xa974
 8006380:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8006384:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8006388:	2120      	movs	r1, #32
 800638a:	f7fd fa3d 	bl	8003808 <core::panicking::panic>
 800638e:	f64a 22f0 	movw	r2, #43760	@ 0xaaf0
 8006392:	4619      	mov	r1, r3
 8006394:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8006398:	f7fd fe30 	bl	8003ffc <core::slice::copy_from_slice_impl::len_mismatch_fail>
 800639c:	f64a 3310 	movw	r3, #43792	@ 0xab10
 80063a0:	4660      	mov	r0, ip
 80063a2:	f6c0 0300 	movt	r3, #2048	@ 0x800
 80063a6:	4611      	mov	r1, r2
 80063a8:	f7fd f99b 	bl	80036e2 <core::slice::index::slice_index_fail>
 80063ac:	f64a 22c0 	movw	r2, #43712	@ 0xaac0
 80063b0:	2000      	movs	r0, #0
 80063b2:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80063b6:	2100      	movs	r1, #0
 80063b8:	f7fd fa12 	bl	80037e0 <core::panicking::panic_bounds_check>
 80063bc:	f64a 22e0 	movw	r2, #43744	@ 0xaae0
 80063c0:	2002      	movs	r0, #2
 80063c2:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80063c6:	2102      	movs	r1, #2
 80063c8:	f7fd fa0a 	bl	80037e0 <core::panicking::panic_bounds_check>

080063cc <smoltcp::socket::tcp::Socket::seq_to_transmit>:
 80063cc:	b5b0      	push	{r4, r5, r7, lr}
 80063ce:	af02      	add	r7, sp, #8
 80063d0:	f8b0 211c 	ldrh.w	r2, [r0, #284]	@ 0x11c
 80063d4:	07d2      	lsls	r2, r2, #31
 80063d6:	d053      	beq.n	8006480 <smoltcp::socket::tcp::Socket::seq_to_transmit+0xb4>
 80063d8:	f8d0 3110 	ldr.w	r3, [r0, #272]	@ 0x110
 80063dc:	3936      	subs	r1, #54	@ 0x36
 80063de:	f8d0 5100 	ldr.w	r5, [r0, #256]	@ 0x100
 80063e2:	f8d0 e108 	ldr.w	lr, [r0, #264]	@ 0x108
 80063e6:	428b      	cmp	r3, r1
 80063e8:	f890 2133 	ldrb.w	r2, [r0, #307]	@ 0x133
 80063ec:	bf38      	it	cc
 80063ee:	4619      	movcc	r1, r3
 80063f0:	45ae      	cmp	lr, r5
 80063f2:	bf04      	itt	eq
 80063f4:	f002 030e 	andeq.w	r3, r2, #14
 80063f8:	2b02      	cmpeq	r3, #2
 80063fa:	d024      	beq.n	8006446 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x7a>
 80063fc:	f8d0 40d4 	ldr.w	r4, [r0, #212]	@ 0xd4
 8006400:	f8d0 310c 	ldr.w	r3, [r0, #268]	@ 0x10c
 8006404:	429c      	cmp	r4, r3
 8006406:	bf38      	it	cc
 8006408:	4623      	movcc	r3, r4
 800640a:	f1b3 3fff 	cmp.w	r3, #4294967295	@ 0xffffffff
 800640e:	dd2c      	ble.n	800646a <smoltcp::socket::tcp::Socket::seq_to_transmit+0x9e>
 8006410:	442b      	add	r3, r5
 8006412:	2a09      	cmp	r2, #9
 8006414:	eba3 0c0e 	sub.w	ip, r3, lr
 8006418:	d817      	bhi.n	800644a <smoltcp::socket::tcp::Socket::seq_to_transmit+0x7e>
 800641a:	2301      	movs	r3, #1
 800641c:	fa03 f202 	lsl.w	r2, r3, r2
 8006420:	f412 7f48 	tst.w	r2, #800	@ 0x320
 8006424:	d011      	beq.n	800644a <smoltcp::socket::tcp::Socket::seq_to_transmit+0x7e>
 8006426:	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
 800642a:	dd1e      	ble.n	800646a <smoltcp::socket::tcp::Socket::seq_to_transmit+0x9e>
 800642c:	1960      	adds	r0, r4, r5
 800642e:	ebae 0000 	sub.w	r0, lr, r0
 8006432:	fab0 f080 	clz	r0, r0
 8006436:	0940      	lsrs	r0, r0, #5
 8006438:	2100      	movs	r1, #0
 800643a:	f1bc 0f00 	cmp.w	ip, #0
 800643e:	bfc8      	it	gt
 8006440:	2101      	movgt	r1, #1
 8006442:	4308      	orrs	r0, r1
 8006444:	bdb0      	pop	{r4, r5, r7, pc}
 8006446:	2001      	movs	r0, #1
 8006448:	bdb0      	pop	{r4, r5, r7, pc}
 800644a:	2200      	movs	r2, #0
 800644c:	45ae      	cmp	lr, r5
 800644e:	d00a      	beq.n	8006466 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x9a>
 8006450:	ea2c 73ec 	bic.w	r3, ip, ip, asr #31
 8006454:	428b      	cmp	r3, r1
 8006456:	d206      	bcs.n	8006466 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x9a>
 8006458:	f890 0132 	ldrb.w	r0, [r0, #306]	@ 0x132
 800645c:	07c0      	lsls	r0, r0, #31
 800645e:	4610      	mov	r0, r2
 8006460:	bf18      	it	ne
 8006462:	bdb0      	popne	{r4, r5, r7, pc}
 8006464:	e7e8      	b.n	8006438 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x6c>
 8006466:	4610      	mov	r0, r2
 8006468:	e7e6      	b.n	8006438 <smoltcp::socket::tcp::Socket::seq_to_transmit+0x6c>
 800646a:	f64a 3070 	movw	r0, #43888	@ 0xab70
 800646e:	f64a 32a8 	movw	r2, #43944	@ 0xaba8
 8006472:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8006476:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800647a:	2171      	movs	r1, #113	@ 0x71
 800647c:	f7fd f9a2 	bl	80037c4 <core::panicking::panic_fmt>
 8006480:	f64a 4038 	movw	r0, #44088	@ 0xac38
 8006484:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8006488:	f7fd f921 	bl	80036ce <core::option::unwrap_failed>

0800648c <smoltcp::socket::tcp::Socket::process>:
 800648c:	b5f0      	push	{r4, r5, r6, r7, lr}
 800648e:	af03      	add	r7, sp, #12
 8006490:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8006494:	b091      	sub	sp, #68	@ 0x44
 8006496:	f891 9133 	ldrb.w	r9, [r1, #307]	@ 0x133
 800649a:	4682      	mov	sl, r0
 800649c:	200c      	movs	r0, #12
 800649e:	68fd      	ldr	r5, [r7, #12]
 80064a0:	469b      	mov	fp, r3
 80064a2:	f8d7 8008 	ldr.w	r8, [r7, #8]
 80064a6:	fa20 f009 	lsr.w	r0, r0, r9
 80064aa:	f000 0301 	and.w	r3, r0, #1
 80064ae:	f44f 7048 	mov.w	r0, #800	@ 0x320
 80064b2:	f895 4051 	ldrb.w	r4, [r5, #81]	@ 0x51
 80064b6:	fa20 f609 	lsr.w	r6, r0, r9
 80064ba:	e9d5 0c00 	ldrd	r0, ip, [r5]
 80064be:	f1b9 0f02 	cmp.w	r9, #2
 80064c2:	d113      	bne.n	80064ec <smoltcp::socket::tcp::Socket::process+0x60>
 80064c4:	b35c      	cbz	r4, 800651e <smoltcp::socket::tcp::Socket::process+0x92>
 80064c6:	2c02      	cmp	r4, #2
 80064c8:	d01c      	beq.n	8006504 <smoltcp::socket::tcp::Socket::process+0x78>
 80064ca:	2c04      	cmp	r4, #4
 80064cc:	d133      	bne.n	8006536 <smoltcp::socket::tcp::Socket::process+0xaa>
 80064ce:	2800      	cmp	r0, #0
 80064d0:	f000 80d2 	beq.w	8006678 <smoltcp::socket::tcp::Socket::process+0x1ec>
 80064d4:	4686      	mov	lr, r0
 80064d6:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 80064da:	3001      	adds	r0, #1
 80064dc:	4584      	cmp	ip, r0
 80064de:	4670      	mov	r0, lr
 80064e0:	d031      	beq.n	8006546 <smoltcp::socket::tcp::Socket::process+0xba>
 80064e2:	f240 001d 	movw	r0, #29
 80064e6:	f2c0 0000 	movt	r0, #0
 80064ea:	e101      	b.n	80066f0 <smoltcp::socket::tcp::Socket::process+0x264>
 80064ec:	2c04      	cmp	r4, #4
 80064ee:	d02a      	beq.n	8006546 <smoltcp::socket::tcp::Socket::process+0xba>
 80064f0:	f1b9 0f01 	cmp.w	r9, #1
 80064f4:	d024      	beq.n	8006540 <smoltcp::socket::tcp::Socket::process+0xb4>
 80064f6:	f1b9 0f02 	cmp.w	r9, #2
 80064fa:	f040 809c 	bne.w	8006636 <smoltcp::socket::tcp::Socket::process+0x1aa>
 80064fe:	b174      	cbz	r4, 800651e <smoltcp::socket::tcp::Socket::process+0x92>
 8006500:	2c02      	cmp	r4, #2
 8006502:	d118      	bne.n	8006536 <smoltcp::socket::tcp::Socket::process+0xaa>
 8006504:	b1f8      	cbz	r0, 8006546 <smoltcp::socket::tcp::Socket::process+0xba>
 8006506:	4686      	mov	lr, r0
 8006508:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 800650c:	3001      	adds	r0, #1
 800650e:	4584      	cmp	ip, r0
 8006510:	4670      	mov	r0, lr
 8006512:	d018      	beq.n	8006546 <smoltcp::socket::tcp::Socket::process+0xba>
 8006514:	f240 001e 	movw	r0, #30
 8006518:	f2c0 0000 	movt	r0, #0
 800651c:	e12c      	b.n	8006778 <smoltcp::socket::tcp::Socket::process+0x2ec>
 800651e:	b150      	cbz	r0, 8006536 <smoltcp::socket::tcp::Socket::process+0xaa>
 8006520:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 8006524:	3001      	adds	r0, #1
 8006526:	4584      	cmp	ip, r0
 8006528:	f040 8122 	bne.w	8006770 <smoltcp::socket::tcp::Socket::process+0x2e4>
 800652c:	f240 000e 	movw	r0, #14
 8006530:	f2c0 0000 	movt	r0, #0
 8006534:	e0dc      	b.n	80066f0 <smoltcp::socket::tcp::Socket::process+0x264>
 8006536:	f240 000d 	movw	r0, #13
 800653a:	f2c0 0000 	movt	r0, #0
 800653e:	e0d7      	b.n	80066f0 <smoltcp::socket::tcp::Socket::process+0x264>
 8006540:	2800      	cmp	r0, #0
 8006542:	f041 8070 	bne.w	8007626 <smoltcp::socket::tcp::Socket::process+0x119a>
 8006546:	900d      	str	r0, [sp, #52]	@ 0x34
 8006548:	f8d1 00c4 	ldr.w	r0, [r1, #196]	@ 0xc4
 800654c:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8006550:	dd0e      	ble.n	8006570 <smoltcp::socket::tcp::Socket::process+0xe4>
 8006552:	e9cd 6305 	strd	r6, r3, [sp, #20]
 8006556:	f8cd 801c 	str.w	r8, [sp, #28]
 800655a:	f8d1 80bc 	ldr.w	r8, [r1, #188]	@ 0xbc
 800655e:	f1b8 3fff 	cmp.w	r8, #4294967295	@ 0xffffffff
 8006562:	bfc1      	itttt	gt
 8006564:	920a      	strgt	r2, [sp, #40]	@ 0x28
 8006566:	6c2b      	ldrgt	r3, [r5, #64]	@ 0x40
 8006568:	930b      	strgt	r3, [sp, #44]	@ 0x2c
 800656a:	f1b3 3fff 	cmpgt.w	r3, #4294967295	@ 0xffffffff
 800656e:	dc0a      	bgt.n	8006586 <smoltcp::socket::tcp::Socket::process+0xfa>
 8006570:	f64a 3070 	movw	r0, #43888	@ 0xab70
 8006574:	f64a 32a8 	movw	r2, #43944	@ 0xaba8
 8006578:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800657c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8006580:	2171      	movs	r1, #113	@ 0x71
 8006582:	f7fd f91f 	bl	80037c4 <core::panicking::panic_fmt>
 8006586:	f8d1 6104 	ldr.w	r6, [r1, #260]	@ 0x104
 800658a:	46a6      	mov	lr, r4
 800658c:	6c6c      	ldr	r4, [r5, #68]	@ 0x44
 800658e:	f1a9 0301 	sub.w	r3, r9, #1
 8006592:	f8cd c03c 	str.w	ip, [sp, #60]	@ 0x3c
 8006596:	eb00 0c06 	add.w	ip, r0, r6
 800659a:	2b02      	cmp	r3, #2
 800659c:	f8cd a030 	str.w	sl, [sp, #48]	@ 0x30
 80065a0:	950e      	str	r5, [sp, #56]	@ 0x38
 80065a2:	f8cd b020 	str.w	fp, [sp, #32]
 80065a6:	d239      	bcs.n	800661c <smoltcp::socket::tcp::Socket::process+0x190>
 80065a8:	9603      	str	r6, [sp, #12]
 80065aa:	46e2      	mov	sl, ip
 80065ac:	2501      	movs	r5, #1
 80065ae:	2300      	movs	r3, #0
 80065b0:	f04f 0c00 	mov.w	ip, #0
 80065b4:	4676      	mov	r6, lr
 80065b6:	9409      	str	r4, [sp, #36]	@ 0x24
 80065b8:	980d      	ldr	r0, [sp, #52]	@ 0x34
 80065ba:	f04f 0b00 	mov.w	fp, #0
 80065be:	2e04      	cmp	r6, #4
 80065c0:	f000 81f0 	beq.w	80069a4 <smoltcp::socket::tcp::Socket::process+0x518>
 80065c4:	2400      	movs	r4, #0
 80065c6:	f04f 0800 	mov.w	r8, #0
 80065ca:	2800      	cmp	r0, #0
 80065cc:	f000 81ed 	beq.w	80069aa <smoltcp::socket::tcp::Socket::process+0x51e>
 80065d0:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 80065d4:	9a06      	ldr	r2, [sp, #24]
 80065d6:	4410      	add	r0, r2
 80065d8:	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
 80065da:	1a14      	subs	r4, r2, r0
 80065dc:	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
 80065e0:	f340 80fc 	ble.w	80067dc <smoltcp::socket::tcp::Socket::process+0x350>
 80065e4:	9805      	ldr	r0, [sp, #20]
 80065e6:	07c0      	lsls	r0, r0, #31
 80065e8:	d00b      	beq.n	8006602 <smoltcp::socket::tcp::Socket::process+0x176>
 80065ea:	f8d1 00d4 	ldr.w	r0, [r1, #212]	@ 0xd4
 80065ee:	3001      	adds	r0, #1
 80065f0:	42a0      	cmp	r0, r4
 80065f2:	eba0 0004 	sub.w	r0, r0, r4
 80065f6:	fab0 f080 	clz	r0, r0
 80065fa:	bf08      	it	eq
 80065fc:	3c01      	subeq	r4, #1
 80065fe:	ea4f 1850 	mov.w	r8, r0, lsr #5
 8006602:	f8d1 0108 	ldr.w	r0, [r1, #264]	@ 0x108
 8006606:	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
 8006608:	1a80      	subs	r0, r0, r2
 800660a:	fab0 f080 	clz	r0, r0
 800660e:	ea4f 1b50 	mov.w	fp, r0, lsr #5
 8006612:	6808      	ldr	r0, [r1, #0]
 8006614:	07c0      	lsls	r0, r0, #31
 8006616:	f040 80e6 	bne.w	80067e6 <smoltcp::socket::tcp::Socket::process+0x35a>
 800661a:	e1c6      	b.n	80069aa <smoltcp::socket::tcp::Socket::process+0x51e>
 800661c:	9b0b      	ldr	r3, [sp, #44]	@ 0x2c
 800661e:	eb08 0b06 	add.w	fp, r8, r6
 8006622:	191d      	adds	r5, r3, r4
 8006624:	b36b      	cbz	r3, 8006682 <smoltcp::socket::tcp::Socket::process+0x1f6>
 8006626:	4540      	cmp	r0, r8
 8006628:	d178      	bne.n	800671c <smoltcp::socket::tcp::Socket::process+0x290>
 800662a:	f240 0011 	movw	r0, #17
 800662e:	460e      	mov	r6, r1
 8006630:	f2c0 0000 	movt	r0, #0
 8006634:	e02f      	b.n	8006696 <smoltcp::socket::tcp::Socket::process+0x20a>
 8006636:	2800      	cmp	r0, #0
 8006638:	d056      	beq.n	80066e8 <smoltcp::socket::tcp::Socket::process+0x25c>
 800663a:	469e      	mov	lr, r3
 800663c:	f1b9 0f03 	cmp.w	r9, #3
 8006640:	900d      	str	r0, [sp, #52]	@ 0x34
 8006642:	f040 811d 	bne.w	8006880 <smoltcp::socket::tcp::Socket::process+0x3f4>
 8006646:	f8d1 0100 	ldr.w	r0, [r1, #256]	@ 0x100
 800664a:	4673      	mov	r3, lr
 800664c:	3001      	adds	r0, #1
 800664e:	4584      	cmp	ip, r0
 8006650:	980d      	ldr	r0, [sp, #52]	@ 0x34
 8006652:	f43f af78 	beq.w	8006546 <smoltcp::socket::tcp::Socket::process+0xba>
 8006656:	f240 001b 	movw	r0, #27
 800665a:	f2c0 0000 	movt	r0, #0
 800665e:	f7fd fe7c 	bl	800435a <defmt::export::acquire_header_and_release>
 8006662:	4650      	mov	r0, sl
 8006664:	4659      	mov	r1, fp
 8006666:	4642      	mov	r2, r8
 8006668:	462b      	mov	r3, r5
 800666a:	b011      	add	sp, #68	@ 0x44
 800666c:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8006670:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 8006674:	f001 b819 	b.w	80076aa <smoltcp::socket::tcp::Socket::rst_reply>
 8006678:	f240 001c 	movw	r0, #28
 800667c:	f2c0 0000 	movt	r0, #0
 8006680:	e036      	b.n	80066f0 <smoltcp::socket::tcp::Socket::process+0x264>
 8006682:	f1ac 0301 	sub.w	r3, ip, #1
 8006686:	429c      	cmp	r4, r3
 8006688:	f040 80ef 	bne.w	800686a <smoltcp::socket::tcp::Socket::process+0x3de>
 800668c:	f240 0012 	movw	r0, #18
 8006690:	460e      	mov	r6, r1
 8006692:	f2c0 0000 	movt	r0, #0
 8006696:	f7fd fe60 	bl	800435a <defmt::export::acquire_header_and_release>
 800669a:	980a      	ldr	r0, [sp, #40]	@ 0x28
 800669c:	4631      	mov	r1, r6
 800669e:	f8dd e038 	ldr.w	lr, [sp, #56]	@ 0x38
 80066a2:	e9d0 4504 	ldrd	r4, r5, [r0, #16]
 80066a6:	980c      	ldr	r0, [sp, #48]	@ 0x30
 80066a8:	9a08      	ldr	r2, [sp, #32]
 80066aa:	f896 3133 	ldrb.w	r3, [r6, #307]	@ 0x133
 80066ae:	2b0a      	cmp	r3, #10
 80066b0:	d10e      	bne.n	80066d0 <smoltcp::socket::tcp::Socket::process+0x244>
 80066b2:	f249 6380 	movw	r3, #38528	@ 0x9680
 80066b6:	f04f 0c00 	mov.w	ip, #0
 80066ba:	f2c0 0398 	movt	r3, #152	@ 0x98
 80066be:	f04f 0803 	mov.w	r8, #3
 80066c2:	191b      	adds	r3, r3, r4
 80066c4:	e9c1 8c20 	strd	r8, ip, [r1, #128]	@ 0x80
 80066c8:	f145 0600 	adc.w	r6, r5, #0
 80066cc:	e9c1 3622 	strd	r3, r6, [r1, #136]	@ 0x88
 80066d0:	e9d1 363e 	ldrd	r3, r6, [r1, #248]	@ 0xf8
 80066d4:	1ae3      	subs	r3, r4, r3
 80066d6:	eb75 0306 	sbcs.w	r3, r5, r6
 80066da:	da12      	bge.n	8006702 <smoltcp::socket::tcp::Socket::process+0x276>
 80066dc:	2102      	movs	r1, #2
 80066de:	6101      	str	r1, [r0, #16]
 80066e0:	b011      	add	sp, #68	@ 0x44
 80066e2:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80066e6:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80066e8:	f240 0010 	movw	r0, #16
 80066ec:	f2c0 0000 	movt	r0, #0
 80066f0:	f7fd fe33 	bl	800435a <defmt::export::acquire_header_and_release>
 80066f4:	2002      	movs	r0, #2
 80066f6:	f8ca 0010 	str.w	r0, [sl, #16]
 80066fa:	b011      	add	sp, #68	@ 0x44
 80066fc:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8006700:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8006702:	f244 2340 	movw	r3, #16960	@ 0x4240
 8006706:	f2c0 030f 	movt	r3, #15
 800670a:	18e6      	adds	r6, r4, r3
 800670c:	f145 0300 	adc.w	r3, r5, #0
 8006710:	e9c1 633e 	strd	r6, r3, [r1, #248]	@ 0xf8
 8006714:	f8c7 e008 	str.w	lr, [r7, #8]
 8006718:	f000 bf6b 	b.w	80075f2 <smoltcp::socket::tcp::Socket::process+0x1166>
 800671c:	ebac 0004 	sub.w	r0, ip, r4
 8006720:	2800      	cmp	r0, #0
 8006722:	dc04      	bgt.n	800672e <smoltcp::socket::tcp::Socket::process+0x2a2>
 8006724:	4623      	mov	r3, r4
 8006726:	ebb4 020b 	subs.w	r2, r4, fp
 800672a:	f100 8109 	bmi.w	8006940 <smoltcp::socket::tcp::Socket::process+0x4b4>
 800672e:	ebac 0205 	sub.w	r2, ip, r5
 8006732:	f1b2 3fff 	cmp.w	r2, #4294967295	@ 0xffffffff
 8006736:	bfdc      	itt	le
 8006738:	eba5 020b 	suble.w	r2, r5, fp
 800673c:	2a01      	cmple	r2, #1
 800673e:	f2c0 80fb 	blt.w	8006938 <smoltcp::socket::tcp::Socket::process+0x4ac>
 8006742:	f240 0016 	movw	r0, #22
 8006746:	460e      	mov	r6, r1
 8006748:	f2c0 0000 	movt	r0, #0
 800674c:	46e0      	mov	r8, ip
 800674e:	f7fd fdf3 	bl	8004338 <defmt::export::acquire_and_header>
 8006752:	4620      	mov	r0, r4
 8006754:	f001 fb03 	bl	8007d5e <defmt::export::fmt>
 8006758:	4628      	mov	r0, r5
 800675a:	f001 fb00 	bl	8007d5e <defmt::export::fmt>
 800675e:	4640      	mov	r0, r8
 8006760:	f001 fafd 	bl	8007d5e <defmt::export::fmt>
 8006764:	4658      	mov	r0, fp
 8006766:	f001 fafa 	bl	8007d5e <defmt::export::fmt>
 800676a:	f7fd ff1f 	bl	80045ac <_defmt_release>
 800676e:	e794      	b.n	800669a <smoltcp::socket::tcp::Socket::process+0x20e>
 8006770:	f240 000f 	movw	r0, #15
 8006774:	f2c0 0000 	movt	r0, #0
 8006778:	4666      	mov	r6, ip
 800677a:	f7fd fdee 	bl	800435a <defmt::export::acquire_header_and_release>
 800677e:	2000      	movs	r0, #0
 8006780:	2101      	movs	r1, #1
 8006782:	f88a 0058 	strb.w	r0, [sl, #88]	@ 0x58
 8006786:	f04f 6280 	mov.w	r2, #67108864	@ 0x4000000
 800678a:	f8aa 0048 	strh.w	r0, [sl, #72]	@ 0x48
 800678e:	f8ca 003c 	str.w	r0, [sl, #60]	@ 0x3c
 8006792:	f8ca 0030 	str.w	r0, [sl, #48]	@ 0x30
 8006796:	f8ca 0024 	str.w	r0, [sl, #36]	@ 0x24
 800679a:	f8ca 0018 	str.w	r0, [sl, #24]
 800679e:	f8ca 0050 	str.w	r0, [sl, #80]	@ 0x50
 80067a2:	f8ca 0010 	str.w	r0, [sl, #16]
 80067a6:	f44f 7050 	mov.w	r0, #832	@ 0x340
 80067aa:	f8aa 000c 	strh.w	r0, [sl, #12]
 80067ae:	f8d5 004a 	ldr.w	r0, [r5, #74]	@ 0x4a
 80067b2:	f8ca 104c 	str.w	r1, [sl, #76]	@ 0x4c
 80067b6:	2114      	movs	r1, #20
 80067b8:	f8ca 205e 	str.w	r2, [sl, #94]	@ 0x5e
 80067bc:	ea4f 4030 	mov.w	r0, r0, ror #16
 80067c0:	f8ca 6054 	str.w	r6, [sl, #84]	@ 0x54
 80067c4:	f8ca 8000 	str.w	r8, [sl]
 80067c8:	f8ca b004 	str.w	fp, [sl, #4]
 80067cc:	f8ca 1008 	str.w	r1, [sl, #8]
 80067d0:	f8ca 005a 	str.w	r0, [sl, #90]	@ 0x5a
 80067d4:	b011      	add	sp, #68	@ 0x44
 80067d6:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80067da:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80067dc:	2400      	movs	r4, #0
 80067de:	6808      	ldr	r0, [r1, #0]
 80067e0:	07c0      	lsls	r0, r0, #31
 80067e2:	f000 80e2 	beq.w	80069aa <smoltcp::socket::tcp::Socket::process+0x51e>
 80067e6:	6908      	ldr	r0, [r1, #16]
 80067e8:	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
 80067ea:	1a10      	subs	r0, r2, r0
 80067ec:	f100 80dd 	bmi.w	80069aa <smoltcp::socket::tcp::Socket::process+0x51e>
 80067f0:	e9cd 6500 	strd	r6, r5, [sp]
 80067f4:	f8cd c010 	str.w	ip, [sp, #16]
 80067f8:	9302      	str	r3, [sp, #8]
 80067fa:	980a      	ldr	r0, [sp, #40]	@ 0x28
 80067fc:	e9d1 3502 	ldrd	r3, r5, [r1, #8]
 8006800:	e9d0 0c04 	ldrd	r0, ip, [r0, #16]
 8006804:	e9d1 6208 	ldrd	r6, r2, [r1, #32]
 8006808:	e9cd 6205 	strd	r6, r2, [sp, #20]
 800680c:	2200      	movs	r2, #0
 800680e:	1ac0      	subs	r0, r0, r3
 8006810:	e9c1 2200 	strd	r2, r2, [r1]
 8006814:	f881 2028 	strb.w	r2, [r1, #40]	@ 0x28
 8006818:	eb6c 0205 	sbc.w	r2, ip, r5
 800681c:	460d      	mov	r5, r1
 800681e:	ea80 70e2 	eor.w	r0, r0, r2, asr #31
 8006822:	ea82 73e2 	eor.w	r3, r2, r2, asr #31
 8006826:	ebb0 70e2 	subs.w	r0, r0, r2, asr #31
 800682a:	eb63 72e2 	sbc.w	r2, r3, r2, asr #31
 800682e:	2300      	movs	r3, #0
 8006830:	4611      	mov	r1, r2
 8006832:	f44f 727a 	mov.w	r2, #1000	@ 0x3e8
 8006836:	f002 fe19 	bl	800946c <__aeabi_uldivmod>
 800683a:	9a05      	ldr	r2, [sp, #20]
 800683c:	4629      	mov	r1, r5
 800683e:	ebc2 02c2 	rsb	r2, r2, r2, lsl #3
 8006842:	4402      	add	r2, r0
 8006844:	3207      	adds	r2, #7
 8006846:	08d2      	lsrs	r2, r2, #3
 8006848:	1a10      	subs	r0, r2, r0
 800684a:	622a      	str	r2, [r5, #32]
 800684c:	bf48      	it	mi
 800684e:	4240      	negmi	r0, r0
 8006850:	9a06      	ldr	r2, [sp, #24]
 8006852:	9b02      	ldr	r3, [sp, #8]
 8006854:	e9dd 6500 	ldrd	r6, r5, [sp]
 8006858:	eb02 0242 	add.w	r2, r2, r2, lsl #1
 800685c:	f8dd c010 	ldr.w	ip, [sp, #16]
 8006860:	4410      	add	r0, r2
 8006862:	3003      	adds	r0, #3
 8006864:	0880      	lsrs	r0, r0, #2
 8006866:	6248      	str	r0, [r1, #36]	@ 0x24
 8006868:	e09f      	b.n	80069aa <smoltcp::socket::tcp::Socket::process+0x51e>
 800686a:	4540      	cmp	r0, r8
 800686c:	d156      	bne.n	800691c <smoltcp::socket::tcp::Socket::process+0x490>
 800686e:	4663      	mov	r3, ip
 8006870:	45a4      	cmp	ip, r4
 8006872:	d065      	beq.n	8006940 <smoltcp::socket::tcp::Socket::process+0x4b4>
 8006874:	f240 0021 	movw	r0, #33	@ 0x21
 8006878:	460e      	mov	r6, r1
 800687a:	f2c0 0000 	movt	r0, #0
 800687e:	e70a      	b.n	8006696 <smoltcp::socket::tcp::Socket::process+0x20a>
 8006880:	f006 0001 	and.w	r0, r6, #1
 8006884:	f8d1 30d4 	ldr.w	r3, [r1, #212]	@ 0xd4
 8006888:	4470      	add	r0, lr
 800688a:	f8cd 801c 	str.w	r8, [sp, #28]
 800688e:	4418      	add	r0, r3
 8006890:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8006894:	f77f ae6c 	ble.w	8006570 <smoltcp::socket::tcp::Socket::process+0xe4>
 8006898:	f8d1 3100 	ldr.w	r3, [r1, #256]	@ 0x100
 800689c:	eb03 080e 	add.w	r8, r3, lr
 80068a0:	4418      	add	r0, r3
 80068a2:	900f      	str	r0, [sp, #60]	@ 0x3c
 80068a4:	ebbc 0008 	subs.w	r0, ip, r8
 80068a8:	f100 830f 	bmi.w	8006eca <smoltcp::socket::tcp::Socket::process+0xa3e>
 80068ac:	f8cd 8038 	str.w	r8, [sp, #56]	@ 0x38
 80068b0:	4673      	mov	r3, lr
 80068b2:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 80068b4:	f8dd 801c 	ldr.w	r8, [sp, #28]
 80068b8:	ebac 0000 	sub.w	r0, ip, r0
 80068bc:	2800      	cmp	r0, #0
 80068be:	980d      	ldr	r0, [sp, #52]	@ 0x34
 80068c0:	f77f ae41 	ble.w	8006546 <smoltcp::socket::tcp::Socket::process+0xba>
 80068c4:	f240 001a 	movw	r0, #26
 80068c8:	4688      	mov	r8, r1
 80068ca:	f2c0 0000 	movt	r0, #0
 80068ce:	4664      	mov	r4, ip
 80068d0:	4691      	mov	r9, r2
 80068d2:	f7fd fd31 	bl	8004338 <defmt::export::acquire_and_header>
 80068d6:	4620      	mov	r0, r4
 80068d8:	f001 fa41 	bl	8007d5e <defmt::export::fmt>
 80068dc:	980e      	ldr	r0, [sp, #56]	@ 0x38
 80068de:	f001 fa3e 	bl	8007d5e <defmt::export::fmt>
 80068e2:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 80068e4:	f001 fa3b 	bl	8007d5e <defmt::export::fmt>
 80068e8:	f7fd fe60 	bl	80045ac <_defmt_release>
 80068ec:	e9d9 2004 	ldrd	r2, r0, [r9, #16]
 80068f0:	e9d8 363e 	ldrd	r3, r6, [r8, #248]	@ 0xf8
 80068f4:	1ad3      	subs	r3, r2, r3
 80068f6:	eb70 0306 	sbcs.w	r3, r0, r6
 80068fa:	f6ff aefb 	blt.w	80066f4 <smoltcp::socket::tcp::Socket::process+0x268>
 80068fe:	f244 2340 	movw	r3, #16960	@ 0x4240
 8006902:	4641      	mov	r1, r8
 8006904:	f2c0 030f 	movt	r3, #15
 8006908:	18d2      	adds	r2, r2, r3
 800690a:	f140 0000 	adc.w	r0, r0, #0
 800690e:	e9c1 203e 	strd	r2, r0, [r1, #248]	@ 0xf8
 8006912:	4650      	mov	r0, sl
 8006914:	60bd      	str	r5, [r7, #8]
 8006916:	465a      	mov	r2, fp
 8006918:	f000 be6b 	b.w	80075f2 <smoltcp::socket::tcp::Socket::process+0x1166>
 800691c:	ebac 0004 	sub.w	r0, ip, r4
 8006920:	2800      	cmp	r0, #0
 8006922:	dc03      	bgt.n	800692c <smoltcp::socket::tcp::Socket::process+0x4a0>
 8006924:	4623      	mov	r3, r4
 8006926:	ebb4 000b 	subs.w	r0, r4, fp
 800692a:	d409      	bmi.n	8006940 <smoltcp::socket::tcp::Socket::process+0x4b4>
 800692c:	f240 0020 	movw	r0, #32
 8006930:	460e      	mov	r6, r1
 8006932:	f2c0 0000 	movt	r0, #0
 8006936:	e6ae      	b.n	8006696 <smoltcp::socket::tcp::Socket::process+0x20a>
 8006938:	4623      	mov	r3, r4
 800693a:	2800      	cmp	r0, #0
 800693c:	bfc8      	it	gt
 800693e:	4663      	movgt	r3, ip
 8006940:	2001      	movs	r0, #1
 8006942:	e9c1 0428 	strd	r0, r4, [r1, #160]	@ 0xa0
 8006946:	1b18      	subs	r0, r3, r4
 8006948:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 800694c:	f340 8660 	ble.w	8007610 <smoltcp::socket::tcp::Socket::process+0x1184>
 8006950:	ebbb 0205 	subs.w	r2, fp, r5
 8006954:	9603      	str	r6, [sp, #12]
 8006956:	bf48      	it	mi
 8006958:	465d      	movmi	r5, fp
 800695a:	1b2a      	subs	r2, r5, r4
 800695c:	f1b2 3fff 	cmp.w	r2, #4294967295	@ 0xffffffff
 8006960:	f340 8656 	ble.w	8007610 <smoltcp::socket::tcp::Socket::process+0x1184>
 8006964:	46e2      	mov	sl, ip
 8006966:	f8dd c038 	ldr.w	ip, [sp, #56]	@ 0x38
 800696a:	4282      	cmp	r2, r0
 800696c:	f0c0 8666 	bcc.w	800763c <smoltcp::socket::tcp::Socket::process+0x11b0>
 8006970:	9d0b      	ldr	r5, [sp, #44]	@ 0x2c
 8006972:	42aa      	cmp	r2, r5
 8006974:	f200 8662 	bhi.w	800763c <smoltcp::socket::tcp::Socket::process+0x11b0>
 8006978:	eba3 030a 	sub.w	r3, r3, sl
 800697c:	9409      	str	r4, [sp, #36]	@ 0x24
 800697e:	f1b3 3fff 	cmp.w	r3, #4294967295	@ 0xffffffff
 8006982:	f340 8645 	ble.w	8007610 <smoltcp::socket::tcp::Socket::process+0x1184>
 8006986:	461d      	mov	r5, r3
 8006988:	f8dc 303c 	ldr.w	r3, [ip, #60]	@ 0x3c
 800698c:	eba2 0c00 	sub.w	ip, r2, r0
 8006990:	4676      	mov	r6, lr
 8006992:	4418      	add	r0, r3
 8006994:	462b      	mov	r3, r5
 8006996:	4605      	mov	r5, r0
 8006998:	980d      	ldr	r0, [sp, #52]	@ 0x34
 800699a:	f04f 0b00 	mov.w	fp, #0
 800699e:	2e04      	cmp	r6, #4
 80069a0:	f47f ae10 	bne.w	80065c4 <smoltcp::socket::tcp::Socket::process+0x138>
 80069a4:	2400      	movs	r4, #0
 80069a6:	f04f 0800 	mov.w	r8, #0
 80069aa:	1e72      	subs	r2, r6, #1
 80069ac:	bf18      	it	ne
 80069ae:	4632      	movne	r2, r6
 80069b0:	9809      	ldr	r0, [sp, #36]	@ 0x24
 80069b2:	ebba 0000 	subs.w	r0, sl, r0
 80069b6:	4610      	mov	r0, r2
 80069b8:	bf48      	it	mi
 80069ba:	2000      	movmi	r0, #0
 80069bc:	2a03      	cmp	r2, #3
 80069be:	bf18      	it	ne
 80069c0:	4610      	movne	r0, r2
 80069c2:	f1b9 0f01 	cmp.w	r9, #1
 80069c6:	d039      	beq.n	8006a3c <smoltcp::socket::tcp::Socket::process+0x5b0>
 80069c8:	f1b9 0f03 	cmp.w	r9, #3
 80069cc:	b2c0      	uxtb	r0, r0
 80069ce:	bf08      	it	eq
 80069d0:	2804      	cmpeq	r0, #4
 80069d2:	f000 8082 	beq.w	8006ada <smoltcp::socket::tcp::Socket::process+0x64e>
 80069d6:	e8df f010 	tbh	[pc, r0, lsl #1]
 80069da:	0005      	.short	0x0005
 80069dc:	00980039 	.word	0x00980039
 80069e0:	008c00eb 	.word	0x008c00eb
 80069e4:	f1a9 0003 	sub.w	r0, r9, #3
 80069e8:	2806      	cmp	r0, #6
 80069ea:	d82f      	bhi.n	8006a4c <smoltcp::socket::tcp::Socket::process+0x5c0>
 80069ec:	e9cd 5301 	strd	r5, r3, [sp, #4]
 80069f0:	f8cd c010 	str.w	ip, [sp, #16]
 80069f4:	9406      	str	r4, [sp, #24]
 80069f6:	e8df f010 	tbh	[pc, r0, lsl #1]
 80069fa:	02c9      	.short	0x02c9
 80069fc:	02a902e7 	.word	0x02a902e7
 8006a00:	00070007 	.word	0x00070007
 8006a04:	0435030a 	.word	0x0435030a
 8006a08:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006a0c:	4634      	mov	r4, r6
 8006a0e:	6c8b      	ldr	r3, [r1, #72]	@ 0x48
 8006a10:	f04f 0e00 	mov.w	lr, #0
 8006a14:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8006a18:	6cce      	ldr	r6, [r1, #76]	@ 0x4c
 8006a1a:	195b      	adds	r3, r3, r5
 8006a1c:	f8d1 c040 	ldr.w	ip, [r1, #64]	@ 0x40
 8006a20:	4170      	adcs	r0, r6
 8006a22:	6c4a      	ldr	r2, [r1, #68]	@ 0x44
 8006a24:	4626      	mov	r6, r4
 8006a26:	f8c1 e080 	str.w	lr, [r1, #128]	@ 0x80
 8006a2a:	f8c1 c088 	str.w	ip, [r1, #136]	@ 0x88
 8006a2e:	f8c1 e084 	str.w	lr, [r1, #132]	@ 0x84
 8006a32:	f8c1 208c 	str.w	r2, [r1, #140]	@ 0x8c
 8006a36:	f8c1 3090 	str.w	r3, [r1, #144]	@ 0x90
 8006a3a:	e2c2      	b.n	8006fc2 <smoltcp::socket::tcp::Socket::process+0xb36>
 8006a3c:	b2c0      	uxtb	r0, r0
 8006a3e:	e8df f010 	tbh	[pc, r0, lsl #1]
 8006a42:	0005      	.short	0x0005
 8006a44:	00ef0005 	.word	0x00ef0005
 8006a48:	005d0005 	.word	0x005d0005
 8006a4c:	f240 001f 	movw	r0, #31
 8006a50:	f2c0 0000 	movt	r0, #0
 8006a54:	f7fd fc70 	bl	8004338 <defmt::export::acquire_and_header>
 8006a58:	f240 0907 	movw	r9, #7
 8006a5c:	a810      	add	r0, sp, #64	@ 0x40
 8006a5e:	f2c0 0900 	movt	r9, #0
 8006a62:	2102      	movs	r1, #2
 8006a64:	f8ad 9040 	strh.w	r9, [sp, #64]	@ 0x40
 8006a68:	f7fd fe18 	bl	800469c <_defmt_write>
 8006a6c:	f240 003c 	movw	r0, #60	@ 0x3c
 8006a70:	2102      	movs	r1, #2
 8006a72:	f2c0 0000 	movt	r0, #0
 8006a76:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006a7a:	a810      	add	r0, sp, #64	@ 0x40
 8006a7c:	f7fd fe0e 	bl	800469c <_defmt_write>
 8006a80:	9c0e      	ldr	r4, [sp, #56]	@ 0x38
 8006a82:	f240 0a04 	movw	sl, #4
 8006a86:	a810      	add	r0, sp, #64	@ 0x40
 8006a88:	f2c0 0a00 	movt	sl, #0
 8006a8c:	2102      	movs	r1, #2
 8006a8e:	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
 8006a92:	f8b4 504a 	ldrh.w	r5, [r4, #74]	@ 0x4a
 8006a96:	f7fd fe01 	bl	800469c <_defmt_write>
 8006a9a:	a810      	add	r0, sp, #64	@ 0x40
 8006a9c:	2102      	movs	r1, #2
 8006a9e:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006aa2:	f7fd fdfb 	bl	800469c <_defmt_write>
 8006aa6:	a810      	add	r0, sp, #64	@ 0x40
 8006aa8:	2102      	movs	r1, #2
 8006aaa:	f8b4 504c 	ldrh.w	r5, [r4, #76]	@ 0x4c
 8006aae:	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
 8006ab2:	f7fd fdf3 	bl	800469c <_defmt_write>
 8006ab6:	a810      	add	r0, sp, #64	@ 0x40
 8006ab8:	2102      	movs	r1, #2
 8006aba:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006abe:	f7fd fded 	bl	800469c <_defmt_write>
 8006ac2:	e8df f016 	tbh	[pc, r6, lsl #1]
 8006ac6:	0149      	.short	0x0149
 8006ac8:	013a0005 	.word	0x013a0005
 8006acc:	0135013f 	.word	0x0135013f
 8006ad0:	f240 0037 	movw	r0, #55	@ 0x37
 8006ad4:	f2c0 0000 	movt	r0, #0
 8006ad8:	e138      	b.n	8006d4c <smoltcp::socket::tcp::Socket::process+0x8c0>
 8006ada:	f8b1 20b0 	ldrh.w	r2, [r1, #176]	@ 0xb0
 8006ade:	2a00      	cmp	r2, #0
 8006ae0:	f43f af79 	beq.w	80069d6 <smoltcp::socket::tcp::Socket::process+0x54a>
 8006ae4:	2001      	movs	r0, #1
 8006ae6:	f881 0133 	strb.w	r0, [r1, #307]	@ 0x133
 8006aea:	2000      	movs	r0, #0
 8006aec:	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
 8006af0:	e004      	b.n	8006afc <smoltcp::socket::tcp::Socket::process+0x670>
 8006af2:	2000      	movs	r0, #0
 8006af4:	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
 8006af8:	f881 0133 	strb.w	r0, [r1, #307]	@ 0x133
 8006afc:	990c      	ldr	r1, [sp, #48]	@ 0x30
 8006afe:	2002      	movs	r0, #2
 8006b00:	6108      	str	r0, [r1, #16]
 8006b02:	b011      	add	sp, #68	@ 0x44
 8006b04:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8006b08:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8006b0a:	f1b9 0f01 	cmp.w	r9, #1
 8006b0e:	f000 8087 	beq.w	8006c20 <smoltcp::socket::tcp::Socket::process+0x794>
 8006b12:	f1b9 0f02 	cmp.w	r9, #2
 8006b16:	d199      	bne.n	8006a4c <smoltcp::socket::tcp::Socket::process+0x5c0>
 8006b18:	f8cd c010 	str.w	ip, [sp, #16]
 8006b1c:	9501      	str	r5, [sp, #4]
 8006b1e:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8006b22:	f8dd a024 	ldr.w	sl, [sp, #36]	@ 0x24
 8006b26:	f8bb 0038 	ldrh.w	r0, [fp, #56]	@ 0x38
 8006b2a:	b128      	cbz	r0, 8006b38 <smoltcp::socket::tcp::Socket::process+0x6ac>
 8006b2c:	f8bb 003a 	ldrh.w	r0, [fp, #58]	@ 0x3a
 8006b30:	2800      	cmp	r0, #0
 8006b32:	d0e3      	beq.n	8006afc <smoltcp::socket::tcp::Socket::process+0x670>
 8006b34:	f8c1 0110 	str.w	r0, [r1, #272]	@ 0x110
 8006b38:	46b0      	mov	r8, r6
 8006b3a:	f10a 0601 	add.w	r6, sl, #1
 8006b3e:	f89b 0048 	ldrb.w	r0, [fp, #72]	@ 0x48
 8006b42:	2501      	movs	r5, #1
 8006b44:	f8c1 6104 	str.w	r6, [r1, #260]	@ 0x104
 8006b48:	f04f 0e00 	mov.w	lr, #0
 8006b4c:	f8d1 6100 	ldr.w	r6, [r1, #256]	@ 0x100
 8006b50:	2800      	cmp	r0, #0
 8006b52:	9406      	str	r4, [sp, #24]
 8006b54:	9302      	str	r3, [sp, #8]
 8006b56:	f89b 2049 	ldrb.w	r2, [fp, #73]	@ 0x49
 8006b5a:	f89b 3050 	ldrb.w	r3, [fp, #80]	@ 0x50
 8006b5e:	f881 212d 	strb.w	r2, [r1, #301]	@ 0x12d
 8006b62:	f106 0201 	add.w	r2, r6, #1
 8006b66:	f8c1 2108 	str.w	r2, [r1, #264]	@ 0x108
 8006b6a:	f04f 0204 	mov.w	r2, #4
 8006b6e:	e9c1 5a26 	strd	r5, sl, [r1, #152]	@ 0x98
 8006b72:	f881 3131 	strb.w	r3, [r1, #305]	@ 0x131
 8006b76:	f881 012c 	strb.w	r0, [r1, #300]	@ 0x12c
 8006b7a:	bf04      	itt	eq
 8006b7c:	2000      	moveq	r0, #0
 8006b7e:	f881 0134 	strbeq.w	r0, [r1, #308]	@ 0x134
 8006b82:	f8db 002c 	ldr.w	r0, [fp, #44]	@ 0x2c
 8006b86:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006b8a:	2800      	cmp	r0, #0
 8006b8c:	bf04      	itt	eq
 8006b8e:	2000      	moveq	r0, #0
 8006b90:	f8c1 0114 	streq.w	r0, [r1, #276]	@ 0x114
 8006b94:	9b0d      	ldr	r3, [sp, #52]	@ 0x34
 8006b96:	e9d1 4612 	ldrd	r4, r6, [r1, #72]	@ 0x48
 8006b9a:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8006b9e:	2b00      	cmp	r3, #0
 8006ba0:	e9d1 c910 	ldrd	ip, r9, [r1, #64]	@ 0x40
 8006ba4:	bf08      	it	eq
 8006ba6:	2203      	moveq	r2, #3
 8006ba8:	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
 8006bac:	192a      	adds	r2, r5, r4
 8006bae:	e0b0      	b.n	8006d12 <smoltcp::socket::tcp::Socket::process+0x886>
 8006bb0:	f1a9 0003 	sub.w	r0, r9, #3
 8006bb4:	2803      	cmp	r0, #3
 8006bb6:	f63f af49 	bhi.w	8006a4c <smoltcp::socket::tcp::Socket::process+0x5c0>
 8006bba:	e9cd 5301 	strd	r5, r3, [sp, #4]
 8006bbe:	f8cd c010 	str.w	ip, [sp, #16]
 8006bc2:	9406      	str	r4, [sp, #24]
 8006bc4:	e8df f010 	tbh	[pc, r0, lsl #1]
 8006bc8:	00040004 	.word	0x00040004
 8006bcc:	01b10195 	.word	0x01b10195
 8006bd0:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006bd4:	4634      	mov	r4, r6
 8006bd6:	2607      	movs	r6, #7
 8006bd8:	9803      	ldr	r0, [sp, #12]
 8006bda:	e9da 8e04 	ldrd	r8, lr, [sl, #16]
 8006bde:	f04f 0c00 	mov.w	ip, #0
 8006be2:	6c8b      	ldr	r3, [r1, #72]	@ 0x48
 8006be4:	f881 6133 	strb.w	r6, [r1, #307]	@ 0x133
 8006be8:	2601      	movs	r6, #1
 8006bea:	6cca      	ldr	r2, [r1, #76]	@ 0x4c
 8006bec:	eb13 0308 	adds.w	r3, r3, r8
 8006bf0:	f881 6130 	strb.w	r6, [r1, #304]	@ 0x130
 8006bf4:	f100 0601 	add.w	r6, r0, #1
 8006bf8:	6c0d      	ldr	r5, [r1, #64]	@ 0x40
 8006bfa:	eb42 020e 	adc.w	r2, r2, lr
 8006bfe:	6c48      	ldr	r0, [r1, #68]	@ 0x44
 8006c00:	f8c1 6104 	str.w	r6, [r1, #260]	@ 0x104
 8006c04:	4626      	mov	r6, r4
 8006c06:	f8c1 c080 	str.w	ip, [r1, #128]	@ 0x80
 8006c0a:	f8c1 5088 	str.w	r5, [r1, #136]	@ 0x88
 8006c0e:	f8c1 3090 	str.w	r3, [r1, #144]	@ 0x90
 8006c12:	f8c1 c084 	str.w	ip, [r1, #132]	@ 0x84
 8006c16:	f8c1 008c 	str.w	r0, [r1, #140]	@ 0x8c
 8006c1a:	f8c1 2094 	str.w	r2, [r1, #148]	@ 0x94
 8006c1e:	e212      	b.n	8007046 <smoltcp::socket::tcp::Socket::process+0xbba>
 8006c20:	f8cd c010 	str.w	ip, [sp, #16]
 8006c24:	e9cd 5301 	strd	r5, r3, [sp, #4]
 8006c28:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8006c2c:	e9dd 3a08 	ldrd	r3, sl, [sp, #32]
 8006c30:	f8bb 0038 	ldrh.w	r0, [fp, #56]	@ 0x38
 8006c34:	b130      	cbz	r0, 8006c44 <smoltcp::socket::tcp::Socket::process+0x7b8>
 8006c36:	f8bb 003a 	ldrh.w	r0, [fp, #58]	@ 0x3a
 8006c3a:	2800      	cmp	r0, #0
 8006c3c:	f43f af5e 	beq.w	8006afc <smoltcp::socket::tcp::Socket::process+0x670>
 8006c40:	f8c1 0110 	str.w	r0, [r1, #272]	@ 0x110
 8006c44:	f8dd e028 	ldr.w	lr, [sp, #40]	@ 0x28
 8006c48:	2001      	movs	r0, #1
 8006c4a:	f8c1 3124 	str.w	r3, [r1, #292]	@ 0x124
 8006c4e:	46b0      	mov	r8, r6
 8006c50:	9b07      	ldr	r3, [sp, #28]
 8006c52:	f10a 0601 	add.w	r6, sl, #1
 8006c56:	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
 8006c5a:	f8c1 311e 	str.w	r3, [r1, #286]	@ 0x11e
 8006c5e:	f8bb 304a 	ldrh.w	r3, [fp, #74]	@ 0x4a
 8006c62:	e9de 0206 	ldrd	r0, r2, [lr, #24]
 8006c66:	f8a1 3128 	strh.w	r3, [r1, #296]	@ 0x128
 8006c6a:	f246 131d 	movw	r3, #24861	@ 0x611d
 8006c6e:	f8bb 504c 	ldrh.w	r5, [fp, #76]	@ 0x4c
 8006c72:	f6c3 4339 	movt	r3, #15417	@ 0x3c39
 8006c76:	f8c1 6104 	str.w	r6, [r1, #260]	@ 0x104
 8006c7a:	fb02 fc03 	mul.w	ip, r2, r3
 8006c7e:	f8a1 5122 	strh.w	r5, [r1, #290]	@ 0x122
 8006c82:	fba0 6503 	umull	r6, r5, r0, r3
 8006c86:	f64f 43ec 	movw	r3, #64748	@ 0xfcec
 8006c8a:	f6cb 332e 	movt	r3, #47918	@ 0xbb2e
 8006c8e:	f89b 2050 	ldrb.w	r2, [fp, #80]	@ 0x50
 8006c92:	fb00 5003 	mla	r0, r0, r3, r5
 8006c96:	f881 2131 	strb.w	r2, [r1, #305]	@ 0x131
 8006c9a:	f64e 7239 	movw	r2, #61241	@ 0xef39
 8006c9e:	f2c7 5290 	movt	r2, #30096	@ 0x7590
 8006ca2:	f89b 5049 	ldrb.w	r5, [fp, #73]	@ 0x49
 8006ca6:	1992      	adds	r2, r2, r6
 8006ca8:	f04f 061d 	mov.w	r6, #29
 8006cac:	f881 512d 	strb.w	r5, [r1, #301]	@ 0x12d
 8006cb0:	eb40 000c 	adc.w	r0, r0, ip
 8006cb4:	f89b 3048 	ldrb.w	r3, [fp, #72]	@ 0x48
 8006cb8:	f881 312c 	strb.w	r3, [r1, #300]	@ 0x12c
 8006cbc:	eba6 7650 	sub.w	r6, r6, r0, lsr #29
 8006cc0:	e9ce 2006 	strd	r2, r0, [lr, #24]
 8006cc4:	f086 051f 	eor.w	r5, r6, #31
 8006cc8:	0040      	lsls	r0, r0, #1
 8006cca:	40f2      	lsrs	r2, r6
 8006ccc:	2b00      	cmp	r3, #0
 8006cce:	fa00 f005 	lsl.w	r0, r0, r5
 8006cd2:	f04f 0e00 	mov.w	lr, #0
 8006cd6:	ea40 0002 	orr.w	r0, r0, r2
 8006cda:	f8c1 0108 	str.w	r0, [r1, #264]	@ 0x108
 8006cde:	f8c1 0100 	str.w	r0, [r1, #256]	@ 0x100
 8006ce2:	bf04      	itt	eq
 8006ce4:	2000      	moveq	r0, #0
 8006ce6:	f881 0134 	strbeq.w	r0, [r1, #308]	@ 0x134
 8006cea:	f8db 002c 	ldr.w	r0, [fp, #44]	@ 0x2c
 8006cee:	2203      	movs	r2, #3
 8006cf0:	9406      	str	r4, [sp, #24]
 8006cf2:	2800      	cmp	r0, #0
 8006cf4:	bf04      	itt	eq
 8006cf6:	2000      	moveq	r0, #0
 8006cf8:	f8c1 0114 	streq.w	r0, [r1, #276]	@ 0x114
 8006cfc:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006d00:	e9d1 3612 	ldrd	r3, r6, [r1, #72]	@ 0x48
 8006d04:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8006d08:	e9d1 c910 	ldrd	ip, r9, [r1, #64]	@ 0x40
 8006d0c:	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
 8006d10:	18ea      	adds	r2, r5, r3
 8006d12:	4170      	adcs	r0, r6
 8006d14:	f8c1 e080 	str.w	lr, [r1, #128]	@ 0x80
 8006d18:	f8c1 c088 	str.w	ip, [r1, #136]	@ 0x88
 8006d1c:	4646      	mov	r6, r8
 8006d1e:	f8c1 2090 	str.w	r2, [r1, #144]	@ 0x90
 8006d22:	f8c1 e084 	str.w	lr, [r1, #132]	@ 0x84
 8006d26:	f8c1 908c 	str.w	r9, [r1, #140]	@ 0x8c
 8006d2a:	f8c1 0094 	str.w	r0, [r1, #148]	@ 0x94
 8006d2e:	e18c      	b.n	800704a <smoltcp::socket::tcp::Socket::process+0xbbe>
 8006d30:	f240 0038 	movw	r0, #56	@ 0x38
 8006d34:	f2c0 0000 	movt	r0, #0
 8006d38:	e008      	b.n	8006d4c <smoltcp::socket::tcp::Socket::process+0x8c0>
 8006d3a:	f240 003a 	movw	r0, #58	@ 0x3a
 8006d3e:	f2c0 0000 	movt	r0, #0
 8006d42:	e003      	b.n	8006d4c <smoltcp::socket::tcp::Socket::process+0x8c0>
 8006d44:	f240 0034 	movw	r0, #52	@ 0x34
 8006d48:	f2c0 0000 	movt	r0, #0
 8006d4c:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006d50:	a810      	add	r0, sp, #64	@ 0x40
 8006d52:	2102      	movs	r1, #2
 8006d54:	f7fd fca2 	bl	800469c <_defmt_write>
 8006d58:	f240 0039 	movw	r0, #57	@ 0x39
 8006d5c:	2102      	movs	r1, #2
 8006d5e:	f2c0 0000 	movt	r0, #0
 8006d62:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006d66:	a810      	add	r0, sp, #64	@ 0x40
 8006d68:	f7fd fc98 	bl	800469c <_defmt_write>
 8006d6c:	a810      	add	r0, sp, #64	@ 0x40
 8006d6e:	2102      	movs	r1, #2
 8006d70:	f8ad 9040 	strh.w	r9, [sp, #64]	@ 0x40
 8006d74:	f7fd fc92 	bl	800469c <_defmt_write>
 8006d78:	f240 0b3d 	movw	fp, #61	@ 0x3d
 8006d7c:	a810      	add	r0, sp, #64	@ 0x40
 8006d7e:	f2c0 0b00 	movt	fp, #0
 8006d82:	2102      	movs	r1, #2
 8006d84:	f8ad b040 	strh.w	fp, [sp, #64]	@ 0x40
 8006d88:	f7fd fc88 	bl	800469c <_defmt_write>
 8006d8c:	f240 0508 	movw	r5, #8
 8006d90:	a810      	add	r0, sp, #64	@ 0x40
 8006d92:	f2c0 0500 	movt	r5, #0
 8006d96:	2102      	movs	r1, #2
 8006d98:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006d9c:	f7fd fc7e 	bl	800469c <_defmt_write>
 8006da0:	9809      	ldr	r0, [sp, #36]	@ 0x24
 8006da2:	2104      	movs	r1, #4
 8006da4:	9010      	str	r0, [sp, #64]	@ 0x40
 8006da6:	a810      	add	r0, sp, #64	@ 0x40
 8006da8:	f7fd fc78 	bl	800469c <_defmt_write>
 8006dac:	a810      	add	r0, sp, #64	@ 0x40
 8006dae:	2600      	movs	r6, #0
 8006db0:	2102      	movs	r1, #2
 8006db2:	f8ad 6040 	strh.w	r6, [sp, #64]	@ 0x40
 8006db6:	f7fd fc71 	bl	800469c <_defmt_write>
 8006dba:	f8dd 8030 	ldr.w	r8, [sp, #48]	@ 0x30
 8006dbe:	980d      	ldr	r0, [sp, #52]	@ 0x34
 8006dc0:	b338      	cbz	r0, 8006e12 <smoltcp::socket::tcp::Socket::process+0x986>
 8006dc2:	f240 0033 	movw	r0, #51	@ 0x33
 8006dc6:	2102      	movs	r1, #2
 8006dc8:	f2c0 0000 	movt	r0, #0
 8006dcc:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006dd0:	a810      	add	r0, sp, #64	@ 0x40
 8006dd2:	f7fd fc63 	bl	800469c <_defmt_write>
 8006dd6:	a810      	add	r0, sp, #64	@ 0x40
 8006dd8:	2102      	movs	r1, #2
 8006dda:	f8ad 9040 	strh.w	r9, [sp, #64]	@ 0x40
 8006dde:	f7fd fc5d 	bl	800469c <_defmt_write>
 8006de2:	a810      	add	r0, sp, #64	@ 0x40
 8006de4:	2102      	movs	r1, #2
 8006de6:	f8ad b040 	strh.w	fp, [sp, #64]	@ 0x40
 8006dea:	f7fd fc57 	bl	800469c <_defmt_write>
 8006dee:	a810      	add	r0, sp, #64	@ 0x40
 8006df0:	2102      	movs	r1, #2
 8006df2:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006df6:	f7fd fc51 	bl	800469c <_defmt_write>
 8006dfa:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 8006dfc:	2104      	movs	r1, #4
 8006dfe:	9010      	str	r0, [sp, #64]	@ 0x40
 8006e00:	a810      	add	r0, sp, #64	@ 0x40
 8006e02:	f7fd fc4b 	bl	800469c <_defmt_write>
 8006e06:	a810      	add	r0, sp, #64	@ 0x40
 8006e08:	2102      	movs	r1, #2
 8006e0a:	f8ad 6040 	strh.w	r6, [sp, #64]	@ 0x40
 8006e0e:	f7fd fc45 	bl	800469c <_defmt_write>
 8006e12:	f240 003b 	movw	r0, #59	@ 0x3b
 8006e16:	2102      	movs	r1, #2
 8006e18:	f2c0 0000 	movt	r0, #0
 8006e1c:	2602      	movs	r6, #2
 8006e1e:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006e22:	a810      	add	r0, sp, #64	@ 0x40
 8006e24:	f7fd fc3a 	bl	800469c <_defmt_write>
 8006e28:	a810      	add	r0, sp, #64	@ 0x40
 8006e2a:	2102      	movs	r1, #2
 8006e2c:	f8b4 504e 	ldrh.w	r5, [r4, #78]	@ 0x4e
 8006e30:	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
 8006e34:	f7fd fc32 	bl	800469c <_defmt_write>
 8006e38:	a810      	add	r0, sp, #64	@ 0x40
 8006e3a:	2102      	movs	r1, #2
 8006e3c:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006e40:	f7fd fc2c 	bl	800469c <_defmt_write>
 8006e44:	f240 0035 	movw	r0, #53	@ 0x35
 8006e48:	2102      	movs	r1, #2
 8006e4a:	f2c0 0000 	movt	r0, #0
 8006e4e:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006e52:	a810      	add	r0, sp, #64	@ 0x40
 8006e54:	f7fd fc22 	bl	800469c <_defmt_write>
 8006e58:	f240 0006 	movw	r0, #6
 8006e5c:	2102      	movs	r1, #2
 8006e5e:	f2c0 0000 	movt	r0, #0
 8006e62:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006e66:	a810      	add	r0, sp, #64	@ 0x40
 8006e68:	f7fd fc18 	bl	800469c <_defmt_write>
 8006e6c:	980b      	ldr	r0, [sp, #44]	@ 0x2c
 8006e6e:	2104      	movs	r1, #4
 8006e70:	9010      	str	r0, [sp, #64]	@ 0x40
 8006e72:	a810      	add	r0, sp, #64	@ 0x40
 8006e74:	f7fd fc12 	bl	800469c <_defmt_write>
 8006e78:	8f20      	ldrh	r0, [r4, #56]	@ 0x38
 8006e7a:	2801      	cmp	r0, #1
 8006e7c:	d116      	bne.n	8006eac <smoltcp::socket::tcp::Socket::process+0xa20>
 8006e7e:	f240 0036 	movw	r0, #54	@ 0x36
 8006e82:	2102      	movs	r1, #2
 8006e84:	f2c0 0000 	movt	r0, #0
 8006e88:	8f65      	ldrh	r5, [r4, #58]	@ 0x3a
 8006e8a:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006e8e:	a810      	add	r0, sp, #64	@ 0x40
 8006e90:	f7fd fc04 	bl	800469c <_defmt_write>
 8006e94:	a810      	add	r0, sp, #64	@ 0x40
 8006e96:	2102      	movs	r1, #2
 8006e98:	f8ad a040 	strh.w	sl, [sp, #64]	@ 0x40
 8006e9c:	f7fd fbfe 	bl	800469c <_defmt_write>
 8006ea0:	a810      	add	r0, sp, #64	@ 0x40
 8006ea2:	2102      	movs	r1, #2
 8006ea4:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8006ea8:	f7fd fbf8 	bl	800469c <_defmt_write>
 8006eac:	2000      	movs	r0, #0
 8006eae:	2102      	movs	r1, #2
 8006eb0:	f8ad 0040 	strh.w	r0, [sp, #64]	@ 0x40
 8006eb4:	a810      	add	r0, sp, #64	@ 0x40
 8006eb6:	f7fd fbf1 	bl	800469c <_defmt_write>
 8006eba:	f7fd fb77 	bl	80045ac <_defmt_release>
 8006ebe:	f8c8 6010 	str.w	r6, [r8, #16]
 8006ec2:	b011      	add	sp, #68	@ 0x44
 8006ec4:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8006ec8:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8006eca:	f240 000c 	movw	r0, #12
 8006ece:	4665      	mov	r5, ip
 8006ed0:	f2c0 0000 	movt	r0, #0
 8006ed4:	f7fd fa30 	bl	8004338 <defmt::export::acquire_and_header>
 8006ed8:	4628      	mov	r0, r5
 8006eda:	f000 ff40 	bl	8007d5e <defmt::export::fmt>
 8006ede:	4640      	mov	r0, r8
 8006ee0:	f000 ff3d 	bl	8007d5e <defmt::export::fmt>
 8006ee4:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 8006ee6:	f000 ff3a 	bl	8007d5e <defmt::export::fmt>
 8006eea:	f7fd fb5f 	bl	80045ac <_defmt_release>
 8006eee:	f7ff bc01 	b.w	80066f4 <smoltcp::socket::tcp::Socket::process+0x268>
 8006ef2:	2001      	movs	r0, #1
 8006ef4:	f1b8 0f00 	cmp.w	r8, #0
 8006ef8:	f881 0130 	strb.w	r0, [r1, #304]	@ 0x130
 8006efc:	9803      	ldr	r0, [sp, #12]
 8006efe:	f100 0001 	add.w	r0, r0, #1
 8006f02:	f8c1 0104 	str.w	r0, [r1, #260]	@ 0x104
 8006f06:	f040 8086 	bne.w	8007016 <smoltcp::socket::tcp::Socket::process+0xb8a>
 8006f0a:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006f0e:	2208      	movs	r2, #8
 8006f10:	e9d1 3412 	ldrd	r3, r4, [r1, #72]	@ 0x48
 8006f14:	f04f 0e00 	mov.w	lr, #0
 8006f18:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8006f1c:	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
 8006f20:	18ea      	adds	r2, r5, r3
 8006f22:	e9d1 c810 	ldrd	ip, r8, [r1, #64]	@ 0x40
 8006f26:	4160      	adcs	r0, r4
 8006f28:	e041      	b.n	8006fae <smoltcp::socket::tcp::Socket::process+0xb22>
 8006f2a:	250a      	movs	r5, #10
 8006f2c:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006f30:	f881 5133 	strb.w	r5, [r1, #307]	@ 0x133
 8006f34:	2501      	movs	r5, #1
 8006f36:	f881 5130 	strb.w	r5, [r1, #304]	@ 0x130
 8006f3a:	2000      	movs	r0, #0
 8006f3c:	9d03      	ldr	r5, [sp, #12]
 8006f3e:	2403      	movs	r4, #3
 8006f40:	e9da 2304 	ldrd	r2, r3, [sl, #16]
 8006f44:	3501      	adds	r5, #1
 8006f46:	f8c1 5104 	str.w	r5, [r1, #260]	@ 0x104
 8006f4a:	e06d      	b.n	8007028 <smoltcp::socket::tcp::Socket::process+0xb9c>
 8006f4c:	f1b8 0f00 	cmp.w	r8, #0
 8006f50:	bf1c      	itt	ne
 8006f52:	2006      	movne	r0, #6
 8006f54:	f881 0133 	strbne.w	r0, [r1, #307]	@ 0x133
 8006f58:	f1bb 0f00 	cmp.w	fp, #0
 8006f5c:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8006f60:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006f64:	d071      	beq.n	800704a <smoltcp::socket::tcp::Socket::process+0xbbe>
 8006f66:	6c8b      	ldr	r3, [r1, #72]	@ 0x48
 8006f68:	4634      	mov	r4, r6
 8006f6a:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8006f6e:	f04f 0e00 	mov.w	lr, #0
 8006f72:	6cce      	ldr	r6, [r1, #76]	@ 0x4c
 8006f74:	195b      	adds	r3, r3, r5
 8006f76:	e9d1 c210 	ldrd	ip, r2, [r1, #64]	@ 0x40
 8006f7a:	4170      	adcs	r0, r6
 8006f7c:	4626      	mov	r6, r4
 8006f7e:	e9c1 ee20 	strd	lr, lr, [r1, #128]	@ 0x80
 8006f82:	e9c1 c222 	strd	ip, r2, [r1, #136]	@ 0x88
 8006f86:	e9c1 3024 	strd	r3, r0, [r1, #144]	@ 0x90
 8006f8a:	e05e      	b.n	800704a <smoltcp::socket::tcp::Socket::process+0xbbe>
 8006f8c:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006f90:	4634      	mov	r4, r6
 8006f92:	6c8b      	ldr	r3, [r1, #72]	@ 0x48
 8006f94:	2204      	movs	r2, #4
 8006f96:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 8006f9a:	f04f 0e00 	mov.w	lr, #0
 8006f9e:	6cce      	ldr	r6, [r1, #76]	@ 0x4c
 8006fa0:	f881 2133 	strb.w	r2, [r1, #307]	@ 0x133
 8006fa4:	18ea      	adds	r2, r5, r3
 8006fa6:	4170      	adcs	r0, r6
 8006fa8:	e9d1 c810 	ldrd	ip, r8, [r1, #64]	@ 0x40
 8006fac:	4626      	mov	r6, r4
 8006fae:	f8c1 e080 	str.w	lr, [r1, #128]	@ 0x80
 8006fb2:	f8c1 c088 	str.w	ip, [r1, #136]	@ 0x88
 8006fb6:	f8c1 e084 	str.w	lr, [r1, #132]	@ 0x84
 8006fba:	f8c1 808c 	str.w	r8, [r1, #140]	@ 0x8c
 8006fbe:	f8c1 2090 	str.w	r2, [r1, #144]	@ 0x90
 8006fc2:	f8c1 0094 	str.w	r0, [r1, #148]	@ 0x94
 8006fc6:	e03e      	b.n	8007046 <smoltcp::socket::tcp::Socket::process+0xbba>
 8006fc8:	f8d1 0080 	ldr.w	r0, [r1, #128]	@ 0x80
 8006fcc:	f04f 0c00 	mov.w	ip, #0
 8006fd0:	1ec2      	subs	r2, r0, #3
 8006fd2:	f112 0f02 	cmn.w	r2, #2
 8006fd6:	f04f 0200 	mov.w	r2, #0
 8006fda:	bf38      	it	cc
 8006fdc:	2201      	movcc	r2, #1
 8006fde:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8006fe2:	ea52 020b 	orrs.w	r2, r2, fp
 8006fe6:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8006fea:	d02e      	beq.n	800704a <smoltcp::socket::tcp::Socket::process+0xbbe>
 8006fec:	4634      	mov	r4, r6
 8006fee:	e9d1 6512 	ldrd	r6, r5, [r1, #72]	@ 0x48
 8006ff2:	e9da 0204 	ldrd	r0, r2, [sl, #16]
 8006ff6:	e9d1 e310 	ldrd	lr, r3, [r1, #64]	@ 0x40
 8006ffa:	1980      	adds	r0, r0, r6
 8006ffc:	4626      	mov	r6, r4
 8006ffe:	416a      	adcs	r2, r5
 8007000:	e9c1 cc20 	strd	ip, ip, [r1, #128]	@ 0x80
 8007004:	e9c1 e322 	strd	lr, r3, [r1, #136]	@ 0x88
 8007008:	e9c1 0224 	strd	r0, r2, [r1, #144]	@ 0x90
 800700c:	e01d      	b.n	800704a <smoltcp::socket::tcp::Socket::process+0xbbe>
 800700e:	f1b8 0f00 	cmp.w	r8, #0
 8007012:	f000 813d 	beq.w	8007290 <smoltcp::socket::tcp::Socket::process+0xe04>
 8007016:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 800701a:	250a      	movs	r5, #10
 800701c:	2000      	movs	r0, #0
 800701e:	2403      	movs	r4, #3
 8007020:	e9da 2304 	ldrd	r2, r3, [sl, #16]
 8007024:	f881 5133 	strb.w	r5, [r1, #307]	@ 0x133
 8007028:	f249 6580 	movw	r5, #38528	@ 0x9680
 800702c:	f8c1 4080 	str.w	r4, [r1, #128]	@ 0x80
 8007030:	f2c0 0598 	movt	r5, #152	@ 0x98
 8007034:	1952      	adds	r2, r2, r5
 8007036:	f143 0300 	adc.w	r3, r3, #0
 800703a:	f8c1 2088 	str.w	r2, [r1, #136]	@ 0x88
 800703e:	f8c1 0084 	str.w	r0, [r1, #132]	@ 0x84
 8007042:	f8c1 308c 	str.w	r3, [r1, #140]	@ 0x8c
 8007046:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 800704a:	e9da 3904 	ldrd	r3, r9, [sl, #16]
 800704e:	2200      	movs	r2, #0
 8007050:	2001      	movs	r0, #1
 8007052:	2e02      	cmp	r6, #2
 8007054:	e9c1 0214 	strd	r0, r2, [r1, #80]	@ 0x50
 8007058:	930a      	str	r3, [sp, #40]	@ 0x28
 800705a:	e9c1 3916 	strd	r3, r9, [r1, #88]	@ 0x58
 800705e:	d007      	beq.n	8007070 <smoltcp::socket::tcp::Socket::process+0xbe4>
 8007060:	f891 212c 	ldrb.w	r2, [r1, #300]	@ 0x12c
 8007064:	f891 012d 	ldrb.w	r0, [r1, #301]	@ 0x12d
 8007068:	2a00      	cmp	r2, #0
 800706a:	bf18      	it	ne
 800706c:	f000 021f 	andne.w	r2, r0, #31
 8007070:	f8bb 304e 	ldrh.w	r3, [fp, #78]	@ 0x4e
 8007074:	9d06      	ldr	r5, [sp, #24]
 8007076:	f8d1 010c 	ldr.w	r0, [r1, #268]	@ 0x10c
 800707a:	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
 800707e:	fa03 f202 	lsl.w	r2, r3, r2
 8007082:	9c04      	ldr	r4, [sp, #16]
 8007084:	f8c1 210c 	str.w	r2, [r1, #268]	@ 0x10c
 8007088:	b1ad      	cbz	r5, 80070b6 <smoltcp::socket::tcp::Socket::process+0xc2a>
 800708a:	f8d1 30d4 	ldr.w	r3, [r1, #212]	@ 0xd4
 800708e:	42ab      	cmp	r3, r5
 8007090:	f0c0 82dc 	bcc.w	800764c <smoltcp::socket::tcp::Socket::process+0x11c0>
 8007094:	1b5e      	subs	r6, r3, r5
 8007096:	f8d1 30cc 	ldr.w	r3, [r1, #204]	@ 0xcc
 800709a:	f8c1 60d4 	str.w	r6, [r1, #212]	@ 0xd4
 800709e:	b13b      	cbz	r3, 80070b0 <smoltcp::socket::tcp::Socket::process+0xc24>
 80070a0:	f8d1 60d0 	ldr.w	r6, [r1, #208]	@ 0xd0
 80070a4:	442e      	add	r6, r5
 80070a6:	fbb6 f5f3 	udiv	r5, r6, r3
 80070aa:	fb05 6313 	mls	r3, r5, r3, r6
 80070ae:	e000      	b.n	80070b2 <smoltcp::socket::tcp::Socket::process+0xc26>
 80070b0:	2300      	movs	r3, #0
 80070b2:	f8c1 30d0 	str.w	r3, [r1, #208]	@ 0xd0
 80070b6:	9b0d      	ldr	r3, [sp, #52]	@ 0x34
 80070b8:	2b00      	cmp	r3, #0
 80070ba:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 80070bc:	d06e      	beq.n	800719c <smoltcp::socket::tcp::Socket::process+0xd10>
 80070be:	9e0b      	ldr	r6, [sp, #44]	@ 0x2c
 80070c0:	2e00      	cmp	r6, #0
 80070c2:	d14f      	bne.n	8007164 <smoltcp::socket::tcp::Socket::process+0xcd8>
 80070c4:	f8d1 30a8 	ldr.w	r3, [r1, #168]	@ 0xa8
 80070c8:	2b00      	cmp	r3, #0
 80070ca:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 80070cc:	d04a      	beq.n	8007164 <smoltcp::socket::tcp::Socket::process+0xcd8>
 80070ce:	f8d1 30ac 	ldr.w	r3, [r1, #172]	@ 0xac
 80070d2:	9e0f      	ldr	r6, [sp, #60]	@ 0x3c
 80070d4:	42b3      	cmp	r3, r6
 80070d6:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 80070d8:	bf08      	it	eq
 80070da:	4282      	cmpeq	r2, r0
 80070dc:	d142      	bne.n	8007164 <smoltcp::socket::tcp::Socket::process+0xcd8>
 80070de:	f8d1 0108 	ldr.w	r0, [r1, #264]	@ 0x108
 80070e2:	1a18      	subs	r0, r3, r0
 80070e4:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 80070e8:	dc3c      	bgt.n	8007164 <smoltcp::socket::tcp::Socket::process+0xcd8>
 80070ea:	f991 0135 	ldrsb.w	r0, [r1, #309]	@ 0x135
 80070ee:	f04f 0801 	mov.w	r8, #1
 80070f2:	4625      	mov	r5, r4
 80070f4:	460c      	mov	r4, r1
 80070f6:	fa80 f658 	uqadd8	r6, r0, r8
 80070fa:	f240 0013 	movw	r0, #19
 80070fe:	f2c0 0000 	movt	r0, #0
 8007102:	f881 6135 	strb.w	r6, [r1, #309]	@ 0x135
 8007106:	f7fd f917 	bl	8004338 <defmt::export::acquire_and_header>
 800710a:	980f      	ldr	r0, [sp, #60]	@ 0x3c
 800710c:	f000 fe27 	bl	8007d5e <defmt::export::fmt>
 8007110:	f894 0135 	ldrb.w	r0, [r4, #309]	@ 0x135
 8007114:	f000 fe0c 	bl	8007d30 <defmt::export::fmt>
 8007118:	b2f1      	uxtb	r1, r6
 800711a:	29ff      	cmp	r1, #255	@ 0xff
 800711c:	f06f 01fe 	mvn.w	r1, #254	@ 0xfe
 8007120:	f64a 4058 	movw	r0, #44120	@ 0xac58
 8007124:	fa51 f186 	uxtab	r1, r1, r6
 8007128:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800712c:	fab1 f181 	clz	r1, r1
 8007130:	bf18      	it	ne
 8007132:	4640      	movne	r0, r8
 8007134:	0949      	lsrs	r1, r1, #5
 8007136:	f000 fde0 	bl	8007cfa <defmt::export::fmt>
 800713a:	f7fd fa37 	bl	80045ac <_defmt_release>
 800713e:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 8007140:	4621      	mov	r1, r4
 8007142:	f894 0135 	ldrb.w	r0, [r4, #309]	@ 0x135
 8007146:	2803      	cmp	r0, #3
 8007148:	d11f      	bne.n	800718a <smoltcp::socket::tcp::Socket::process+0xcfe>
 800714a:	2000      	movs	r0, #0
 800714c:	2202      	movs	r2, #2
 800714e:	e9c1 2020 	strd	r2, r0, [r1, #128]	@ 0x80
 8007152:	f240 0018 	movw	r0, #24
 8007156:	f2c0 0000 	movt	r0, #0
 800715a:	f7fd f8fe 	bl	800435a <defmt::export::acquire_header_and_release>
 800715e:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 8007160:	4621      	mov	r1, r4
 8007162:	e012      	b.n	800718a <smoltcp::socket::tcp::Socket::process+0xcfe>
 8007164:	f891 0135 	ldrb.w	r0, [r1, #309]	@ 0x135
 8007168:	4625      	mov	r5, r4
 800716a:	b158      	cbz	r0, 8007184 <smoltcp::socket::tcp::Socket::process+0xcf8>
 800716c:	2000      	movs	r0, #0
 800716e:	460c      	mov	r4, r1
 8007170:	f881 0135 	strb.w	r0, [r1, #309]	@ 0x135
 8007174:	f240 0014 	movw	r0, #20
 8007178:	f2c0 0000 	movt	r0, #0
 800717c:	f7fd f8ed 	bl	800435a <defmt::export::acquire_header_and_release>
 8007180:	9b0f      	ldr	r3, [sp, #60]	@ 0x3c
 8007182:	4621      	mov	r1, r4
 8007184:	2001      	movs	r0, #1
 8007186:	e9c1 032a 	strd	r0, r3, [r1, #168]	@ 0xa8
 800718a:	f8d1 0108 	ldr.w	r0, [r1, #264]	@ 0x108
 800718e:	462c      	mov	r4, r5
 8007190:	f8c1 3100 	str.w	r3, [r1, #256]	@ 0x100
 8007194:	1ac0      	subs	r0, r0, r3
 8007196:	bf48      	it	mi
 8007198:	f8c1 3108 	strmi.w	r3, [r1, #264]	@ 0x108
 800719c:	980e      	ldr	r0, [sp, #56]	@ 0x38
 800719e:	9b02      	ldr	r3, [sp, #8]
 80071a0:	6ac0      	ldr	r0, [r0, #44]	@ 0x2c
 80071a2:	2800      	cmp	r0, #0
 80071a4:	bf1e      	ittt	ne
 80071a6:	980e      	ldrne	r0, [sp, #56]	@ 0x38
 80071a8:	6b00      	ldrne	r0, [r0, #48]	@ 0x30
 80071aa:	f8c1 0118 	strne.w	r0, [r1, #280]	@ 0x118
 80071ae:	2c00      	cmp	r4, #0
 80071b0:	f43f aaa0 	beq.w	80066f4 <smoltcp::socket::tcp::Socket::process+0x268>
 80071b4:	4688      	mov	r8, r1
 80071b6:	9404      	str	r4, [sp, #16]
 80071b8:	f858 6fd8 	ldr.w	r6, [r8, #216]!
 80071bc:	f8cd 9024 	str.w	r9, [sp, #36]	@ 0x24
 80071c0:	f8d8 0004 	ldr.w	r0, [r8, #4]
 80071c4:	900f      	str	r0, [sp, #60]	@ 0x3c
 80071c6:	b933      	cbnz	r3, 80071d6 <smoltcp::socket::tcp::Socket::process+0xd4a>
 80071c8:	42a6      	cmp	r6, r4
 80071ca:	d904      	bls.n	80071d6 <smoltcp::socket::tcp::Socket::process+0xd4a>
 80071cc:	46a4      	mov	ip, r4
 80071ce:	1b30      	subs	r0, r6, r4
 80071d0:	f8c8 0000 	str.w	r0, [r8]
 80071d4:	e15a      	b.n	800748c <smoltcp::socket::tcp::Socket::process+0x1000>
 80071d6:	b398      	cbz	r0, 8007240 <smoltcp::socket::tcp::Socket::process+0xdb4>
 80071d8:	eb06 0e00 	add.w	lr, r6, r0
 80071dc:	4573      	cmp	r3, lr
 80071de:	d934      	bls.n	800724a <smoltcp::socket::tcp::Socket::process+0xdbe>
 80071e0:	f8d1 50e4 	ldr.w	r5, [r1, #228]	@ 0xe4
 80071e4:	eba3 090e 	sub.w	r9, r3, lr
 80071e8:	f101 0be0 	add.w	fp, r1, #224	@ 0xe0
 80071ec:	b355      	cbz	r5, 8007244 <smoltcp::socket::tcp::Socket::process+0xdb8>
 80071ee:	f8db 3000 	ldr.w	r3, [fp]
 80071f2:	eb03 0c05 	add.w	ip, r3, r5
 80071f6:	45e1      	cmp	r9, ip
 80071f8:	d942      	bls.n	8007280 <smoltcp::socket::tcp::Socket::process+0xdf4>
 80071fa:	f8d1 50ec 	ldr.w	r5, [r1, #236]	@ 0xec
 80071fe:	eba9 090c 	sub.w	r9, r9, ip
 8007202:	f101 0be8 	add.w	fp, r1, #232	@ 0xe8
 8007206:	2d00      	cmp	r5, #0
 8007208:	d050      	beq.n	80072ac <smoltcp::socket::tcp::Socket::process+0xe20>
 800720a:	f8db 3000 	ldr.w	r3, [fp]
 800720e:	eb03 0c05 	add.w	ip, r3, r5
 8007212:	45e1      	cmp	r9, ip
 8007214:	d94e      	bls.n	80072b4 <smoltcp::socket::tcp::Socket::process+0xe28>
 8007216:	f8d1 50f4 	ldr.w	r5, [r1, #244]	@ 0xf4
 800721a:	eba9 090c 	sub.w	r9, r9, ip
 800721e:	f101 0bf0 	add.w	fp, r1, #240	@ 0xf0
 8007222:	2d00      	cmp	r5, #0
 8007224:	d042      	beq.n	80072ac <smoltcp::socket::tcp::Socket::process+0xe20>
 8007226:	f8db 3000 	ldr.w	r3, [fp]
 800722a:	eb03 0c05 	add.w	ip, r3, r5
 800722e:	45e1      	cmp	r9, ip
 8007230:	d853      	bhi.n	80072da <smoltcp::socket::tcp::Socket::process+0xe4e>
 8007232:	2203      	movs	r2, #3
 8007234:	920d      	str	r2, [sp, #52]	@ 0x34
 8007236:	2201      	movs	r2, #1
 8007238:	9206      	str	r2, [sp, #24]
 800723a:	2200      	movs	r2, #0
 800723c:	920b      	str	r2, [sp, #44]	@ 0x2c
 800723e:	e040      	b.n	80072c2 <smoltcp::socket::tcp::Socket::process+0xe36>
 8007240:	4699      	mov	r9, r3
 8007242:	46c3      	mov	fp, r8
 8007244:	e9cb 9400 	strd	r9, r4, [fp]
 8007248:	e0fb      	b.n	8007442 <smoltcp::socket::tcp::Socket::process+0xfb6>
 800724a:	2200      	movs	r2, #0
 800724c:	4699      	mov	r9, r3
 800724e:	9206      	str	r2, [sp, #24]
 8007250:	2201      	movs	r2, #1
 8007252:	920b      	str	r2, [sp, #44]	@ 0x2c
 8007254:	4605      	mov	r5, r0
 8007256:	9205      	str	r2, [sp, #20]
 8007258:	2200      	movs	r2, #0
 800725a:	4633      	mov	r3, r6
 800725c:	46f4      	mov	ip, lr
 800725e:	920d      	str	r2, [sp, #52]	@ 0x34
 8007260:	46c3      	mov	fp, r8
 8007262:	e02f      	b.n	80072c4 <smoltcp::socket::tcp::Socket::process+0xe38>
 8007264:	f1b8 0f00 	cmp.w	r8, #0
 8007268:	f43f abce 	beq.w	8006a08 <smoltcp::socket::tcp::Socket::process+0x57c>
 800726c:	2000      	movs	r0, #0
 800726e:	f8dd b038 	ldr.w	fp, [sp, #56]	@ 0x38
 8007272:	f8a1 011c 	strh.w	r0, [r1, #284]	@ 0x11c
 8007276:	f881 0133 	strb.w	r0, [r1, #307]	@ 0x133
 800727a:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 800727e:	e6e4      	b.n	800704a <smoltcp::socket::tcp::Socket::process+0xbbe>
 8007280:	2201      	movs	r2, #1
 8007282:	920b      	str	r2, [sp, #44]	@ 0x2c
 8007284:	2200      	movs	r2, #0
 8007286:	9206      	str	r2, [sp, #24]
 8007288:	2201      	movs	r2, #1
 800728a:	9205      	str	r2, [sp, #20]
 800728c:	920d      	str	r2, [sp, #52]	@ 0x34
 800728e:	e019      	b.n	80072c4 <smoltcp::socket::tcp::Socket::process+0xe38>
 8007290:	f8dd a028 	ldr.w	sl, [sp, #40]	@ 0x28
 8007294:	f04f 0e00 	mov.w	lr, #0
 8007298:	e9d1 3412 	ldrd	r3, r4, [r1, #72]	@ 0x48
 800729c:	e9da 5004 	ldrd	r5, r0, [sl, #16]
 80072a0:	e9d1 c210 	ldrd	ip, r2, [r1, #64]	@ 0x40
 80072a4:	195b      	adds	r3, r3, r5
 80072a6:	4160      	adcs	r0, r4
 80072a8:	f7ff bbbd 	b.w	8006a26 <smoltcp::socket::tcp::Socket::process+0x59a>
 80072ac:	9b02      	ldr	r3, [sp, #8]
 80072ae:	e9cb 9400 	strd	r9, r4, [fp]
 80072b2:	e0c6      	b.n	8007442 <smoltcp::socket::tcp::Socket::process+0xfb6>
 80072b4:	2202      	movs	r2, #2
 80072b6:	920d      	str	r2, [sp, #52]	@ 0x34
 80072b8:	2200      	movs	r2, #0
 80072ba:	9206      	str	r2, [sp, #24]
 80072bc:	2201      	movs	r2, #1
 80072be:	920b      	str	r2, [sp, #44]	@ 0x2c
 80072c0:	2200      	movs	r2, #0
 80072c2:	9205      	str	r2, [sp, #20]
 80072c4:	eb09 0a04 	add.w	sl, r9, r4
 80072c8:	4599      	cmp	r9, r3
 80072ca:	d231      	bcs.n	8007330 <smoltcp::socket::tcp::Socket::process+0xea4>
 80072cc:	459a      	cmp	sl, r3
 80072ce:	d231      	bcs.n	8007334 <smoltcp::socket::tcp::Socket::process+0xea8>
 80072d0:	f8d1 00f4 	ldr.w	r0, [r1, #244]	@ 0xf4
 80072d4:	2800      	cmp	r0, #0
 80072d6:	f000 8096 	beq.w	8007406 <smoltcp::socket::tcp::Socket::process+0xf7a>
 80072da:	f240 000b 	movw	r0, #11
 80072de:	f2c0 0000 	movt	r0, #0
 80072e2:	f7fd f829 	bl	8004338 <defmt::export::acquire_and_header>
 80072e6:	f240 0506 	movw	r5, #6
 80072ea:	a810      	add	r0, sp, #64	@ 0x40
 80072ec:	f2c0 0500 	movt	r5, #0
 80072f0:	2102      	movs	r1, #2
 80072f2:	4626      	mov	r6, r4
 80072f4:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 80072f8:	2402      	movs	r4, #2
 80072fa:	f7fd f9cf 	bl	800469c <_defmt_write>
 80072fe:	a810      	add	r0, sp, #64	@ 0x40
 8007300:	2104      	movs	r1, #4
 8007302:	9610      	str	r6, [sp, #64]	@ 0x40
 8007304:	f7fd f9ca 	bl	800469c <_defmt_write>
 8007308:	a810      	add	r0, sp, #64	@ 0x40
 800730a:	2102      	movs	r1, #2
 800730c:	f8ad 5040 	strh.w	r5, [sp, #64]	@ 0x40
 8007310:	f7fd f9c4 	bl	800469c <_defmt_write>
 8007314:	9802      	ldr	r0, [sp, #8]
 8007316:	2104      	movs	r1, #4
 8007318:	9010      	str	r0, [sp, #64]	@ 0x40
 800731a:	a810      	add	r0, sp, #64	@ 0x40
 800731c:	f7fd f9be 	bl	800469c <_defmt_write>
 8007320:	f7fd f944 	bl	80045ac <_defmt_release>
 8007324:	980c      	ldr	r0, [sp, #48]	@ 0x30
 8007326:	6104      	str	r4, [r0, #16]
 8007328:	b011      	add	sp, #68	@ 0x44
 800732a:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800732e:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007330:	4699      	mov	r9, r3
 8007332:	e003      	b.n	800733c <smoltcp::socket::tcp::Socket::process+0xeb0>
 8007334:	ebac 0509 	sub.w	r5, ip, r9
 8007338:	e9cb 9500 	strd	r9, r5, [fp]
 800733c:	f8dd e034 	ldr.w	lr, [sp, #52]	@ 0x34
 8007340:	980b      	ldr	r0, [sp, #44]	@ 0x2c
 8007342:	f10e 0c01 	add.w	ip, lr, #1
 8007346:	b1b0      	cbz	r0, 8007376 <smoltcp::socket::tcp::Socket::process+0xeea>
 8007348:	4672      	mov	r2, lr
 800734a:	4666      	mov	r6, ip
 800734c:	eb01 03c2 	add.w	r3, r1, r2, lsl #3
 8007350:	f8d3 40e4 	ldr.w	r4, [r3, #228]	@ 0xe4
 8007354:	b18c      	cbz	r4, 800737a <smoltcp::socket::tcp::Socket::process+0xeee>
 8007356:	f8d3 30e0 	ldr.w	r3, [r3, #224]	@ 0xe0
 800735a:	442b      	add	r3, r5
 800735c:	eb03 0009 	add.w	r0, r3, r9
 8007360:	4582      	cmp	sl, r0
 8007362:	d30b      	bcc.n	800737c <smoltcp::socket::tcp::Socket::process+0xef0>
 8007364:	191d      	adds	r5, r3, r4
 8007366:	3201      	adds	r2, #1
 8007368:	3601      	adds	r6, #1
 800736a:	2a03      	cmp	r2, #3
 800736c:	f8cb 5004 	str.w	r5, [fp, #4]
 8007370:	d1ec      	bne.n	800734c <smoltcp::socket::tcp::Socket::process+0xec0>
 8007372:	2604      	movs	r6, #4
 8007374:	e002      	b.n	800737c <smoltcp::socket::tcp::Socket::process+0xef0>
 8007376:	9b02      	ldr	r3, [sp, #8]
 8007378:	e02e      	b.n	80073d8 <smoltcp::socket::tcp::Socket::process+0xf4c>
 800737a:	1c56      	adds	r6, r2, #1
 800737c:	9b02      	ldr	r3, [sp, #8]
 800737e:	ea6f 020e 	mvn.w	r2, lr
 8007382:	1992      	adds	r2, r2, r6
 8007384:	d028      	beq.n	80073d8 <smoltcp::socket::tcp::Socket::process+0xf4c>
 8007386:	f8cd c02c 	str.w	ip, [sp, #44]	@ 0x2c
 800738a:	eb01 09ce 	add.w	r9, r1, lr, lsl #3
 800738e:	eb01 0cc6 	add.w	ip, r1, r6, lsl #3
 8007392:	f08e 0203 	eor.w	r2, lr, #3
 8007396:	2500      	movs	r5, #0
 8007398:	9302      	str	r3, [sp, #8]
 800739a:	e008      	b.n	80073ae <smoltcp::socket::tcp::Socket::process+0xf22>
 800739c:	eb0c 00c5 	add.w	r0, ip, r5, lsl #3
 80073a0:	e9d0 3e36 	ldrd	r3, lr, [r0, #216]	@ 0xd8
 80073a4:	3501      	adds	r5, #1
 80073a6:	e9c4 3e38 	strd	r3, lr, [r4, #224]	@ 0xe0
 80073aa:	42aa      	cmp	r2, r5
 80073ac:	d00f      	beq.n	80073ce <smoltcp::socket::tcp::Socket::process+0xf42>
 80073ae:	eb09 04c5 	add.w	r4, r9, r5, lsl #3
 80073b2:	f8d4 00e4 	ldr.w	r0, [r4, #228]	@ 0xe4
 80073b6:	b150      	cbz	r0, 80073ce <smoltcp::socket::tcp::Socket::process+0xf42>
 80073b8:	1970      	adds	r0, r6, r5
 80073ba:	2803      	cmp	r0, #3
 80073bc:	d9ee      	bls.n	800739c <smoltcp::socket::tcp::Socket::process+0xf10>
 80073be:	2300      	movs	r3, #0
 80073c0:	f04f 0e00 	mov.w	lr, #0
 80073c4:	3501      	adds	r5, #1
 80073c6:	e9c4 3e38 	strd	r3, lr, [r4, #224]	@ 0xe0
 80073ca:	42aa      	cmp	r2, r5
 80073cc:	d1ef      	bne.n	80073ae <smoltcp::socket::tcp::Socket::process+0xf22>
 80073ce:	e9db 9500 	ldrd	r9, r5, [fp]
 80073d2:	9b02      	ldr	r3, [sp, #8]
 80073d4:	f8dd c02c 	ldr.w	ip, [sp, #44]	@ 0x2c
 80073d8:	eb05 0209 	add.w	r2, r5, r9
 80073dc:	4592      	cmp	sl, r2
 80073de:	d930      	bls.n	8007442 <smoltcp::socket::tcp::Socket::process+0xfb6>
 80073e0:	ebaa 0202 	sub.w	r2, sl, r2
 80073e4:	1950      	adds	r0, r2, r5
 80073e6:	f8cb 0004 	str.w	r0, [fp, #4]
 80073ea:	9806      	ldr	r0, [sp, #24]
 80073ec:	bb48      	cbnz	r0, 8007442 <smoltcp::socket::tcp::Socket::process+0xfb6>
 80073ee:	eb08 00cc 	add.w	r0, r8, ip, lsl #3
 80073f2:	461e      	mov	r6, r3
 80073f4:	6843      	ldr	r3, [r0, #4]
 80073f6:	2b00      	cmp	r3, #0
 80073f8:	4633      	mov	r3, r6
 80073fa:	bf1f      	itttt	ne
 80073fc:	6803      	ldrne	r3, [r0, #0]
 80073fe:	1a9a      	subne	r2, r3, r2
 8007400:	4633      	movne	r3, r6
 8007402:	6002      	strne	r2, [r0, #0]
 8007404:	e01d      	b.n	8007442 <smoltcp::socket::tcp::Socket::process+0xfb6>
 8007406:	9806      	ldr	r0, [sp, #24]
 8007408:	2800      	cmp	r0, #0
 800740a:	f040 8144 	bne.w	8007696 <smoltcp::socket::tcp::Socket::process+0x120a>
 800740e:	e9d1 033a 	ldrd	r0, r3, [r1, #232]	@ 0xe8
 8007412:	e9c1 033c 	strd	r0, r3, [r1, #240]	@ 0xf0
 8007416:	9b02      	ldr	r3, [sp, #8]
 8007418:	9805      	ldr	r0, [sp, #20]
 800741a:	b140      	cbz	r0, 800742e <smoltcp::socket::tcp::Socket::process+0xfa2>
 800741c:	e9d1 0238 	ldrd	r0, r2, [r1, #224]	@ 0xe0
 8007420:	4573      	cmp	r3, lr
 8007422:	e9c1 023a 	strd	r0, r2, [r1, #232]	@ 0xe8
 8007426:	bf9c      	itt	ls
 8007428:	980f      	ldrls	r0, [sp, #60]	@ 0x3c
 800742a:	e9c1 6038 	strdls	r6, r0, [r1, #224]	@ 0xe0
 800742e:	f8db 0008 	ldr.w	r0, [fp, #8]
 8007432:	f8cb 4004 	str.w	r4, [fp, #4]
 8007436:	eba0 000a 	sub.w	r0, r0, sl
 800743a:	f8cb 9000 	str.w	r9, [fp]
 800743e:	f8cb 0008 	str.w	r0, [fp, #8]
 8007442:	f8d1 00d8 	ldr.w	r0, [r1, #216]	@ 0xd8
 8007446:	f04f 0c00 	mov.w	ip, #0
 800744a:	b9e8      	cbnz	r0, 8007488 <smoltcp::socket::tcp::Socket::process+0xffc>
 800744c:	f8d1 00dc 	ldr.w	r0, [r1, #220]	@ 0xdc
 8007450:	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
 8007454:	b1d0      	cbz	r0, 800748c <smoltcp::socket::tcp::Socket::process+0x1000>
 8007456:	461d      	mov	r5, r3
 8007458:	e9d1 2338 	ldrd	r2, r3, [r1, #224]	@ 0xe0
 800745c:	e9c1 2336 	strd	r2, r3, [r1, #216]	@ 0xd8
 8007460:	2b00      	cmp	r3, #0
 8007462:	bf1e      	ittt	ne
 8007464:	e9d1 233a 	ldrdne	r2, r3, [r1, #232]	@ 0xe8
 8007468:	e9c1 2338 	strdne	r2, r3, [r1, #224]	@ 0xe0
 800746c:	2b00      	cmpne	r3, #0
 800746e:	d008      	beq.n	8007482 <smoltcp::socket::tcp::Socket::process+0xff6>
 8007470:	e9d1 233c 	ldrd	r2, r3, [r1, #240]	@ 0xf0
 8007474:	2600      	movs	r6, #0
 8007476:	f101 0ce8 	add.w	ip, r1, #232	@ 0xe8
 800747a:	f8c1 60f4 	str.w	r6, [r1, #244]	@ 0xf4
 800747e:	e88c 004c 	stmia.w	ip, {r2, r3, r6}
 8007482:	4684      	mov	ip, r0
 8007484:	462b      	mov	r3, r5
 8007486:	e001      	b.n	800748c <smoltcp::socket::tcp::Socket::process+0x1000>
 8007488:	f8dd a030 	ldr.w	sl, [sp, #48]	@ 0x30
 800748c:	f8d1 40bc 	ldr.w	r4, [r1, #188]	@ 0xbc
 8007490:	f8d1 50c4 	ldr.w	r5, [r1, #196]	@ 0xc4
 8007494:	b144      	cbz	r4, 80074a8 <smoltcp::socket::tcp::Socket::process+0x101c>
 8007496:	f8d1 00c0 	ldr.w	r0, [r1, #192]	@ 0xc0
 800749a:	18ea      	adds	r2, r5, r3
 800749c:	4410      	add	r0, r2
 800749e:	fbb0 f2f4 	udiv	r2, r0, r4
 80074a2:	fb02 0014 	mls	r0, r2, r4, r0
 80074a6:	e000      	b.n	80074aa <smoltcp::socket::tcp::Socket::process+0x101e>
 80074a8:	2000      	movs	r0, #0
 80074aa:	eba4 0805 	sub.w	r8, r4, r5
 80074ae:	f8cd c034 	str.w	ip, [sp, #52]	@ 0x34
 80074b2:	4598      	cmp	r8, r3
 80074b4:	d205      	bcs.n	80074c2 <smoltcp::socket::tcp::Socket::process+0x1036>
 80074b6:	46d1      	mov	r9, sl
 80074b8:	469b      	mov	fp, r3
 80074ba:	468a      	mov	sl, r1
 80074bc:	2600      	movs	r6, #0
 80074be:	2001      	movs	r0, #1
 80074c0:	e015      	b.n	80074ee <smoltcp::socket::tcp::Socket::process+0x1062>
 80074c2:	9a04      	ldr	r2, [sp, #16]
 80074c4:	eba8 0603 	sub.w	r6, r8, r3
 80074c8:	42b2      	cmp	r2, r6
 80074ca:	bf38      	it	cc
 80074cc:	4616      	movcc	r6, r2
 80074ce:	1a22      	subs	r2, r4, r0
 80074d0:	4296      	cmp	r6, r2
 80074d2:	bf28      	it	cs
 80074d4:	4616      	movcs	r6, r2
 80074d6:	1832      	adds	r2, r6, r0
 80074d8:	f080 80c3 	bcs.w	8007662 <smoltcp::socket::tcp::Socket::process+0x11d6>
 80074dc:	42a2      	cmp	r2, r4
 80074de:	f200 80c0 	bhi.w	8007662 <smoltcp::socket::tcp::Socket::process+0x11d6>
 80074e2:	46d1      	mov	r9, sl
 80074e4:	468a      	mov	sl, r1
 80074e6:	f8d1 10b8 	ldr.w	r1, [r1, #184]	@ 0xb8
 80074ea:	469b      	mov	fp, r3
 80074ec:	4408      	add	r0, r1
 80074ee:	9901      	ldr	r1, [sp, #4]
 80074f0:	4632      	mov	r2, r6
 80074f2:	f001 fefe 	bl	80092f2 <__aeabi_memcpy>
 80074f6:	eb06 010b 	add.w	r1, r6, fp
 80074fa:	b14c      	cbz	r4, 8007510 <smoltcp::socket::tcp::Socket::process+0x1084>
 80074fc:	f8da 00c0 	ldr.w	r0, [sl, #192]	@ 0xc0
 8007500:	194a      	adds	r2, r1, r5
 8007502:	4653      	mov	r3, sl
 8007504:	4410      	add	r0, r2
 8007506:	fbb0 f2f4 	udiv	r2, r0, r4
 800750a:	fb02 0014 	mls	r0, r2, r4, r0
 800750e:	e001      	b.n	8007514 <smoltcp::socket::tcp::Socket::process+0x1088>
 8007510:	2000      	movs	r0, #0
 8007512:	4653      	mov	r3, sl
 8007514:	4588      	cmp	r8, r1
 8007516:	d203      	bcs.n	8007520 <smoltcp::socket::tcp::Socket::process+0x1094>
 8007518:	461c      	mov	r4, r3
 800751a:	2200      	movs	r2, #0
 800751c:	2001      	movs	r0, #1
 800751e:	e015      	b.n	800754c <smoltcp::socket::tcp::Socket::process+0x10c0>
 8007520:	9a04      	ldr	r2, [sp, #16]
 8007522:	1b95      	subs	r5, r2, r6
 8007524:	eba8 0201 	sub.w	r2, r8, r1
 8007528:	4295      	cmp	r5, r2
 800752a:	eba4 0100 	sub.w	r1, r4, r0
 800752e:	bf38      	it	cc
 8007530:	462a      	movcc	r2, r5
 8007532:	428a      	cmp	r2, r1
 8007534:	bf28      	it	cs
 8007536:	460a      	movcs	r2, r1
 8007538:	1811      	adds	r1, r2, r0
 800753a:	f080 809a 	bcs.w	8007672 <smoltcp::socket::tcp::Socket::process+0x11e6>
 800753e:	42a1      	cmp	r1, r4
 8007540:	f200 8097 	bhi.w	8007672 <smoltcp::socket::tcp::Socket::process+0x11e6>
 8007544:	f8d3 10b8 	ldr.w	r1, [r3, #184]	@ 0xb8
 8007548:	461c      	mov	r4, r3
 800754a:	4408      	add	r0, r1
 800754c:	9901      	ldr	r1, [sp, #4]
 800754e:	4431      	add	r1, r6
 8007550:	f001 fecf 	bl	80092f2 <__aeabi_memcpy>
 8007554:	9b0d      	ldr	r3, [sp, #52]	@ 0x34
 8007556:	4621      	mov	r1, r4
 8007558:	b153      	cbz	r3, 8007570 <smoltcp::socket::tcp::Socket::process+0x10e4>
 800755a:	f8d1 20bc 	ldr.w	r2, [r1, #188]	@ 0xbc
 800755e:	f8d1 00c4 	ldr.w	r0, [r1, #196]	@ 0xc4
 8007562:	1a12      	subs	r2, r2, r0
 8007564:	4293      	cmp	r3, r2
 8007566:	f200 808b 	bhi.w	8007680 <smoltcp::socket::tcp::Socket::process+0x11f4>
 800756a:	4418      	add	r0, r3
 800756c:	f8c1 00c4 	str.w	r0, [r1, #196]	@ 0xc4
 8007570:	6e08      	ldr	r0, [r1, #96]	@ 0x60
 8007572:	07c0      	lsls	r0, r0, #31
 8007574:	d034      	beq.n	80075e0 <smoltcp::socket::tcp::Socket::process+0x1154>
 8007576:	f8d1 0098 	ldr.w	r0, [r1, #152]	@ 0x98
 800757a:	2801      	cmp	r0, #1
 800757c:	d130      	bne.n	80075e0 <smoltcp::socket::tcp::Socket::process+0x1154>
 800757e:	f8d1 00c4 	ldr.w	r0, [r1, #196]	@ 0xc4
 8007582:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8007586:	f77e aff3 	ble.w	8006570 <smoltcp::socket::tcp::Socket::process+0xe4>
 800758a:	f8d1 3104 	ldr.w	r3, [r1, #260]	@ 0x104
 800758e:	f8d1 209c 	ldr.w	r2, [r1, #156]	@ 0x9c
 8007592:	4418      	add	r0, r3
 8007594:	1a10      	subs	r0, r2, r0
 8007596:	d523      	bpl.n	80075e0 <smoltcp::socket::tcp::Socket::process+0x1154>
 8007598:	6f08      	ldr	r0, [r1, #112]	@ 0x70
 800759a:	2802      	cmp	r0, #2
 800759c:	d00a      	beq.n	80075b4 <smoltcp::socket::tcp::Socket::process+0x1128>
 800759e:	2801      	cmp	r0, #1
 80075a0:	d10b      	bne.n	80075ba <smoltcp::socket::tcp::Socket::process+0x112e>
 80075a2:	4608      	mov	r0, r1
 80075a4:	460c      	mov	r4, r1
 80075a6:	f000 f9cf 	bl	8007948 <smoltcp::socket::tcp::Socket::immediate_ack_to_transmit>
 80075aa:	b180      	cbz	r0, 80075ce <smoltcp::socket::tcp::Socket::process+0x1142>
 80075ac:	2000      	movs	r0, #0
 80075ae:	2202      	movs	r2, #2
 80075b0:	4621      	mov	r1, r4
 80075b2:	e011      	b.n	80075d8 <smoltcp::socket::tcp::Socket::process+0x114c>
 80075b4:	2000      	movs	r0, #0
 80075b6:	2202      	movs	r2, #2
 80075b8:	e00e      	b.n	80075d8 <smoltcp::socket::tcp::Socket::process+0x114c>
 80075ba:	e9d1 201a 	ldrd	r2, r0, [r1, #104]	@ 0x68
 80075be:	9b0a      	ldr	r3, [sp, #40]	@ 0x28
 80075c0:	189b      	adds	r3, r3, r2
 80075c2:	9a09      	ldr	r2, [sp, #36]	@ 0x24
 80075c4:	eb40 0602 	adc.w	r6, r0, r2
 80075c8:	2000      	movs	r0, #0
 80075ca:	2201      	movs	r2, #1
 80075cc:	e004      	b.n	80075d8 <smoltcp::socket::tcp::Socket::process+0x114c>
 80075ce:	4621      	mov	r1, r4
 80075d0:	e9d4 201c 	ldrd	r2, r0, [r4, #112]	@ 0x70
 80075d4:	e9d4 361e 	ldrd	r3, r6, [r4, #120]	@ 0x78
 80075d8:	e9c1 201c 	strd	r2, r0, [r1, #112]	@ 0x70
 80075dc:	e9c1 361e 	strd	r3, r6, [r1, #120]	@ 0x78
 80075e0:	f8d1 00dc 	ldr.w	r0, [r1, #220]	@ 0xdc
 80075e4:	9a0f      	ldr	r2, [sp, #60]	@ 0x3c
 80075e6:	4310      	orrs	r0, r2
 80075e8:	d00b      	beq.n	8007602 <smoltcp::socket::tcp::Socket::process+0x1176>
 80075ea:	980e      	ldr	r0, [sp, #56]	@ 0x38
 80075ec:	60b8      	str	r0, [r7, #8]
 80075ee:	4648      	mov	r0, r9
 80075f0:	9a08      	ldr	r2, [sp, #32]
 80075f2:	9b07      	ldr	r3, [sp, #28]
 80075f4:	b011      	add	sp, #68	@ 0x44
 80075f6:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80075fa:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 80075fe:	f000 b89f 	b.w	8007740 <smoltcp::socket::tcp::Socket::ack_reply>
 8007602:	2002      	movs	r0, #2
 8007604:	f8c9 0010 	str.w	r0, [r9, #16]
 8007608:	b011      	add	sp, #68	@ 0x44
 800760a:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 800760e:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007610:	f64a 1094 	movw	r0, #43412	@ 0xa994
 8007614:	f64a 12c8 	movw	r2, #43464	@ 0xa9c8
 8007618:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800761c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007620:	2167      	movs	r1, #103	@ 0x67
 8007622:	f7fc f8cf 	bl	80037c4 <core::panicking::panic_fmt>
 8007626:	f24b 0098 	movw	r0, #45208	@ 0xb098
 800762a:	f64a 4248 	movw	r2, #44104	@ 0xac48
 800762e:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8007632:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007636:	2128      	movs	r1, #40	@ 0x28
 8007638:	f7fc f8e6 	bl	8003808 <core::panicking::panic>
 800763c:	4611      	mov	r1, r2
 800763e:	9a0b      	ldr	r2, [sp, #44]	@ 0x2c
 8007640:	f64a 4394 	movw	r3, #44180	@ 0xac94
 8007644:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007648:	f7fc f84b 	bl	80036e2 <core::slice::index::slice_index_fail>
 800764c:	f64a 50c4 	movw	r0, #44484	@ 0xadc4
 8007650:	f64a 52ec 	movw	r2, #44524	@ 0xadec
 8007654:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8007658:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800765c:	2125      	movs	r1, #37	@ 0x25
 800765e:	f7fc f8d3 	bl	8003808 <core::panicking::panic>
 8007662:	f64a 53a4 	movw	r3, #44452	@ 0xada4
 8007666:	4611      	mov	r1, r2
 8007668:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800766c:	4622      	mov	r2, r4
 800766e:	f7fc f838 	bl	80036e2 <core::slice::index::slice_index_fail>
 8007672:	f64a 53a4 	movw	r3, #44452	@ 0xada4
 8007676:	4622      	mov	r2, r4
 8007678:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800767c:	f7fc f831 	bl	80036e2 <core::slice::index::slice_index_fail>
 8007680:	f64a 4059 	movw	r0, #44121	@ 0xac59
 8007684:	f64a 4284 	movw	r2, #44164	@ 0xac84
 8007688:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800768c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007690:	2128      	movs	r1, #40	@ 0x28
 8007692:	f7fc f8b9 	bl	8003808 <core::panicking::panic>
 8007696:	f64a 7240 	movw	r2, #44864	@ 0xaf40
 800769a:	2004      	movs	r0, #4
 800769c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80076a0:	2104      	movs	r1, #4
 80076a2:	e9cb 9400 	strd	r9, r4, [fp]
 80076a6:	f7fc f89b 	bl	80037e0 <core::panicking::panic_bounds_check>

080076aa <smoltcp::socket::tcp::Socket::rst_reply>:
 80076aa:	b5f0      	push	{r4, r5, r6, r7, lr}
 80076ac:	af03      	add	r7, sp, #12
 80076ae:	f84d 8d04 	str.w	r8, [sp, #-4]!
 80076b2:	f8b3 c04c 	ldrh.w	ip, [r3, #76]	@ 0x4c
 80076b6:	f8b3 e04a 	ldrh.w	lr, [r3, #74]	@ 0x4a
 80076ba:	681c      	ldr	r4, [r3, #0]
 80076bc:	2c01      	cmp	r4, #1
 80076be:	d102      	bne.n	80076c6 <smoltcp::socket::tcp::Socket::rst_reply+0x1c>
 80076c0:	685d      	ldr	r5, [r3, #4]
 80076c2:	2400      	movs	r4, #0
 80076c4:	e010      	b.n	80076e8 <smoltcp::socket::tcp::Socket::rst_reply+0x3e>
 80076c6:	f893 4051 	ldrb.w	r4, [r3, #81]	@ 0x51
 80076ca:	2c02      	cmp	r4, #2
 80076cc:	d10a      	bne.n	80076e4 <smoltcp::socket::tcp::Socket::rst_reply+0x3a>
 80076ce:	6c1c      	ldr	r4, [r3, #64]	@ 0x40
 80076d0:	3401      	adds	r4, #1
 80076d2:	f1b4 3fff 	cmp.w	r4, #4294967295	@ 0xffffffff
 80076d6:	dd28      	ble.n	800772a <smoltcp::socket::tcp::Socket::rst_reply+0x80>
 80076d8:	6c5b      	ldr	r3, [r3, #68]	@ 0x44
 80076da:	2500      	movs	r5, #0
 80076dc:	eb04 0803 	add.w	r8, r4, r3
 80076e0:	2401      	movs	r4, #1
 80076e2:	e001      	b.n	80076e8 <smoltcp::socket::tcp::Socket::rst_reply+0x3e>
 80076e4:	2500      	movs	r5, #0
 80076e6:	2400      	movs	r4, #0
 80076e8:	f04f 6680 	mov.w	r6, #67108864	@ 0x4000000
 80076ec:	2301      	movs	r3, #1
 80076ee:	f8c0 605e 	str.w	r6, [r0, #94]	@ 0x5e
 80076f2:	2600      	movs	r6, #0
 80076f4:	e9c0 3613 	strd	r3, r6, [r0, #76]	@ 0x4c
 80076f8:	f44f 7350 	mov.w	r3, #832	@ 0x340
 80076fc:	8183      	strh	r3, [r0, #12]
 80076fe:	2314      	movs	r3, #20
 8007700:	f8a0 e05c 	strh.w	lr, [r0, #92]	@ 0x5c
 8007704:	f8a0 c05a 	strh.w	ip, [r0, #90]	@ 0x5a
 8007708:	f880 6058 	strb.w	r6, [r0, #88]	@ 0x58
 800770c:	6545      	str	r5, [r0, #84]	@ 0x54
 800770e:	f8a0 6048 	strh.w	r6, [r0, #72]	@ 0x48
 8007712:	63c6      	str	r6, [r0, #60]	@ 0x3c
 8007714:	6306      	str	r6, [r0, #48]	@ 0x30
 8007716:	6246      	str	r6, [r0, #36]	@ 0x24
 8007718:	e9c0 4804 	strd	r4, r8, [r0, #16]
 800771c:	6186      	str	r6, [r0, #24]
 800771e:	e9c0 2100 	strd	r2, r1, [r0]
 8007722:	6083      	str	r3, [r0, #8]
 8007724:	f85d 8b04 	ldr.w	r8, [sp], #4
 8007728:	bdf0      	pop	{r4, r5, r6, r7, pc}
 800772a:	f64a 3070 	movw	r0, #43888	@ 0xab70
 800772e:	f64a 32a8 	movw	r2, #43944	@ 0xaba8
 8007732:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8007736:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800773a:	2171      	movs	r1, #113	@ 0x71
 800773c:	f7fc f842 	bl	80037c4 <core::panicking::panic_fmt>

08007740 <smoltcp::socket::tcp::Socket::ack_reply>:
 8007740:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007742:	af03      	add	r7, sp, #12
 8007744:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8007748:	b089      	sub	sp, #36	@ 0x24
 800774a:	4604      	mov	r4, r0
 800774c:	68b8      	ldr	r0, [r7, #8]
 800774e:	4689      	mov	r9, r1
 8007750:	6ac1      	ldr	r1, [r0, #44]	@ 0x2c
 8007752:	f8b0 b04c 	ldrh.w	fp, [r0, #76]	@ 0x4c
 8007756:	f8b0 504a 	ldrh.w	r5, [r0, #74]	@ 0x4a
 800775a:	07c9      	lsls	r1, r1, #31
 800775c:	bf1c      	itt	ne
 800775e:	f8d9 1114 	ldrne.w	r1, [r9, #276]	@ 0x114
 8007762:	2900      	cmpne	r1, #0
 8007764:	f040 8085 	bne.w	8007872 <smoltcp::socket::tcp::Socket::ack_reply+0x132>
 8007768:	2600      	movs	r6, #0
 800776a:	2117      	movs	r1, #23
 800776c:	f8d9 00c4 	ldr.w	r0, [r9, #196]	@ 0xc4
 8007770:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 8007774:	f340 808d 	ble.w	8007892 <smoltcp::socket::tcp::Socket::ack_reply+0x152>
 8007778:	e9cd 6106 	strd	r6, r1, [sp, #24]
 800777c:	f04f 0c01 	mov.w	ip, #1
 8007780:	e9d9 1641 	ldrd	r1, r6, [r9, #260]	@ 0x104
 8007784:	f64f 78ff 	movw	r8, #65535	@ 0xffff
 8007788:	9508      	str	r5, [sp, #32]
 800778a:	9605      	str	r6, [sp, #20]
 800778c:	eb00 0a01 	add.w	sl, r0, r1
 8007790:	f8d9 50bc 	ldr.w	r5, [r9, #188]	@ 0xbc
 8007794:	f899 6134 	ldrb.w	r6, [r9, #308]	@ 0x134
 8007798:	1a28      	subs	r0, r5, r0
 800779a:	f899 e131 	ldrb.w	lr, [r9, #305]	@ 0x131
 800779e:	f006 011f 	and.w	r1, r6, #31
 80077a2:	e9c9 ca26 	strd	ip, sl, [r9, #152]	@ 0x98
 80077a6:	40c8      	lsrs	r0, r1
 80077a8:	4540      	cmp	r0, r8
 80077aa:	bf38      	it	cc
 80077ac:	4680      	movcc	r8, r0
 80077ae:	f1be 0f00 	cmp.w	lr, #0
 80077b2:	f8a9 812e 	strh.w	r8, [r9, #302]	@ 0x12e
 80077b6:	f000 8093 	beq.w	80078e0 <smoltcp::socket::tcp::Socket::ack_reply+0x1a0>
 80077ba:	f240 0017 	movw	r0, #23
 80077be:	e9cd 3201 	strd	r3, r2, [sp, #4]
 80077c2:	f2c0 0000 	movt	r0, #0
 80077c6:	f7fc fdc8 	bl	800435a <defmt::export::acquire_header_and_release>
 80077ca:	f8d9 00a0 	ldr.w	r0, [r9, #160]	@ 0xa0
 80077ce:	2801      	cmp	r0, #1
 80077d0:	d147      	bne.n	8007862 <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 80077d2:	f8d9 c0a4 	ldr.w	ip, [r9, #164]	@ 0xa4
 80077d6:	f109 00d8 	add.w	r0, r9, #216	@ 0xd8
 80077da:	2600      	movs	r6, #0
 80077dc:	2200      	movs	r2, #0
 80077de:	e000      	b.n	80077e2 <smoltcp::socket::tcp::Socket::ack_reply+0xa2>
 80077e0:	3201      	adds	r2, #1
 80077e2:	2a03      	cmp	r2, #3
 80077e4:	d83d      	bhi.n	8007862 <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 80077e6:	eb00 03c2 	add.w	r3, r0, r2, lsl #3
 80077ea:	f850 1032 	ldr.w	r1, [r0, r2, lsl #3]
 80077ee:	685d      	ldr	r5, [r3, #4]
 80077f0:	1873      	adds	r3, r6, r1
 80077f2:	195e      	adds	r6, r3, r5
 80077f4:	42b3      	cmp	r3, r6
 80077f6:	d325      	bcc.n	8007844 <smoltcp::socket::tcp::Socket::ack_reply+0x104>
 80077f8:	1c51      	adds	r1, r2, #1
 80077fa:	2904      	cmp	r1, #4
 80077fc:	d031      	beq.n	8007862 <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 80077fe:	eb00 05c1 	add.w	r5, r0, r1, lsl #3
 8007802:	f850 6031 	ldr.w	r6, [r0, r1, lsl #3]
 8007806:	686d      	ldr	r5, [r5, #4]
 8007808:	4433      	add	r3, r6
 800780a:	195e      	adds	r6, r3, r5
 800780c:	42b3      	cmp	r3, r6
 800780e:	d30b      	bcc.n	8007828 <smoltcp::socket::tcp::Socket::ack_reply+0xe8>
 8007810:	1c91      	adds	r1, r2, #2
 8007812:	2904      	cmp	r1, #4
 8007814:	d025      	beq.n	8007862 <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 8007816:	eb00 05c1 	add.w	r5, r0, r1, lsl #3
 800781a:	f850 6031 	ldr.w	r6, [r0, r1, lsl #3]
 800781e:	686d      	ldr	r5, [r5, #4]
 8007820:	4433      	add	r3, r6
 8007822:	195e      	adds	r6, r3, r5
 8007824:	42b3      	cmp	r3, r6
 8007826:	d201      	bcs.n	800782c <smoltcp::socket::tcp::Socket::ack_reply+0xec>
 8007828:	460a      	mov	r2, r1
 800782a:	e00b      	b.n	8007844 <smoltcp::socket::tcp::Socket::ack_reply+0x104>
 800782c:	3203      	adds	r2, #3
 800782e:	2a04      	cmp	r2, #4
 8007830:	d017      	beq.n	8007862 <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 8007832:	eb00 06c2 	add.w	r6, r0, r2, lsl #3
 8007836:	f850 1032 	ldr.w	r1, [r0, r2, lsl #3]
 800783a:	6876      	ldr	r6, [r6, #4]
 800783c:	440b      	add	r3, r1
 800783e:	441e      	add	r6, r3
 8007840:	42b3      	cmp	r3, r6
 8007842:	d20e      	bcs.n	8007862 <smoltcp::socket::tcp::Socket::ack_reply+0x122>
 8007844:	eb0a 0503 	add.w	r5, sl, r3
 8007848:	4565      	cmp	r5, ip
 800784a:	d8c9      	bhi.n	80077e0 <smoltcp::socket::tcp::Socket::ack_reply+0xa0>
 800784c:	eb0a 0106 	add.w	r1, sl, r6
 8007850:	4561      	cmp	r1, ip
 8007852:	d3c5      	bcc.n	80077e0 <smoltcp::socket::tcp::Socket::ack_reply+0xa0>
 8007854:	f04f 090a 	mov.w	r9, #10
 8007858:	f04f 0c01 	mov.w	ip, #1
 800785c:	e9dd 3201 	ldrd	r3, r2, [sp, #4]
 8007860:	e042      	b.n	80078e8 <smoltcp::socket::tcp::Socket::ack_reply+0x1a8>
 8007862:	e9d9 0136 	ldrd	r0, r1, [r9, #216]	@ 0xd8
 8007866:	4401      	add	r1, r0
 8007868:	4288      	cmp	r0, r1
 800786a:	d21d      	bcs.n	80078a8 <smoltcp::socket::tcp::Socket::ack_reply+0x168>
 800786c:	e9dd 3201 	ldrd	r3, r2, [sp, #4]
 8007870:	e02e      	b.n	80078d0 <smoltcp::socket::tcp::Socket::ack_reply+0x190>
 8007872:	6b00      	ldr	r0, [r0, #48]	@ 0x30
 8007874:	4690      	mov	r8, r2
 8007876:	9004      	str	r0, [sp, #16]
 8007878:	461e      	mov	r6, r3
 800787a:	4788      	blx	r1
 800787c:	4633      	mov	r3, r6
 800787e:	4642      	mov	r2, r8
 8007880:	2601      	movs	r6, #1
 8007882:	2121      	movs	r1, #33	@ 0x21
 8007884:	9003      	str	r0, [sp, #12]
 8007886:	f8d9 00c4 	ldr.w	r0, [r9, #196]	@ 0xc4
 800788a:	f1b0 3fff 	cmp.w	r0, #4294967295	@ 0xffffffff
 800788e:	f73f af73 	bgt.w	8007778 <smoltcp::socket::tcp::Socket::ack_reply+0x38>
 8007892:	f64a 3070 	movw	r0, #43888	@ 0xab70
 8007896:	f64a 32a8 	movw	r2, #43944	@ 0xaba8
 800789a:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800789e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80078a2:	2171      	movs	r1, #113	@ 0x71
 80078a4:	f7fb ff8e 	bl	80037c4 <core::panicking::panic_fmt>
 80078a8:	e9d9 1238 	ldrd	r1, r2, [r9, #224]	@ 0xe0
 80078ac:	4408      	add	r0, r1
 80078ae:	1881      	adds	r1, r0, r2
 80078b0:	e9dd 3201 	ldrd	r3, r2, [sp, #4]
 80078b4:	4288      	cmp	r0, r1
 80078b6:	d30b      	bcc.n	80078d0 <smoltcp::socket::tcp::Socket::ack_reply+0x190>
 80078b8:	e9d9 163a 	ldrd	r1, r6, [r9, #232]	@ 0xe8
 80078bc:	4408      	add	r0, r1
 80078be:	1981      	adds	r1, r0, r6
 80078c0:	4288      	cmp	r0, r1
 80078c2:	d305      	bcc.n	80078d0 <smoltcp::socket::tcp::Socket::ack_reply+0x190>
 80078c4:	e9d9 163c 	ldrd	r1, r6, [r9, #240]	@ 0xf0
 80078c8:	4408      	add	r0, r1
 80078ca:	1981      	adds	r1, r0, r6
 80078cc:	4288      	cmp	r0, r1
 80078ce:	d207      	bcs.n	80078e0 <smoltcp::socket::tcp::Socket::ack_reply+0x1a0>
 80078d0:	4451      	add	r1, sl
 80078d2:	eb0a 0500 	add.w	r5, sl, r0
 80078d6:	f04f 0c01 	mov.w	ip, #1
 80078da:	f04f 090a 	mov.w	r9, #10
 80078de:	e003      	b.n	80078e8 <smoltcp::socket::tcp::Socket::ack_reply+0x1a8>
 80078e0:	f04f 0c00 	mov.w	ip, #0
 80078e4:	f04f 0900 	mov.w	r9, #0
 80078e8:	9808      	ldr	r0, [sp, #32]
 80078ea:	2600      	movs	r6, #0
 80078ec:	f8a4 005c 	strh.w	r0, [r4, #92]	@ 0x5c
 80078f0:	f04f 0e01 	mov.w	lr, #1
 80078f4:	9805      	ldr	r0, [sp, #20]
 80078f6:	6560      	str	r0, [r4, #84]	@ 0x54
 80078f8:	9806      	ldr	r0, [sp, #24]
 80078fa:	63e0      	str	r0, [r4, #60]	@ 0x3c
 80078fc:	9803      	ldr	r0, [sp, #12]
 80078fe:	6420      	str	r0, [r4, #64]	@ 0x40
 8007900:	9804      	ldr	r0, [sp, #16]
 8007902:	6460      	str	r0, [r4, #68]	@ 0x44
 8007904:	9807      	ldr	r0, [sp, #28]
 8007906:	e9c4 3200 	strd	r3, r2, [r4]
 800790a:	f104 0208 	add.w	r2, r4, #8
 800790e:	4448      	add	r0, r9
 8007910:	e9c4 1608 	strd	r1, r6, [r4, #32]
 8007914:	f000 007c 	and.w	r0, r0, #124	@ 0x7c
 8007918:	f44f 7150 	mov.w	r1, #832	@ 0x340
 800791c:	f8a4 6060 	strh.w	r6, [r4, #96]	@ 0x60
 8007920:	f8a4 805e 	strh.w	r8, [r4, #94]	@ 0x5e
 8007924:	f8a4 b05a 	strh.w	fp, [r4, #90]	@ 0x5a
 8007928:	f884 6058 	strb.w	r6, [r4, #88]	@ 0x58
 800792c:	e9c4 e613 	strd	lr, r6, [r4, #76]	@ 0x4c
 8007930:	f8a4 6048 	strh.w	r6, [r4, #72]	@ 0x48
 8007934:	6326      	str	r6, [r4, #48]	@ 0x30
 8007936:	e882 4003 	stmia.w	r2, {r0, r1, lr}
 800793a:	e9c4 ac05 	strd	sl, ip, [r4, #20]
 800793e:	61e5      	str	r5, [r4, #28]
 8007940:	b009      	add	sp, #36	@ 0x24
 8007942:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007946:	bdf0      	pop	{r4, r5, r6, r7, pc}

08007948 <smoltcp::socket::tcp::Socket::immediate_ack_to_transmit>:
 8007948:	b580      	push	{r7, lr}
 800794a:	466f      	mov	r7, sp
 800794c:	f8d0 1098 	ldr.w	r1, [r0, #152]	@ 0x98
 8007950:	2901      	cmp	r1, #1
 8007952:	bf1c      	itt	ne
 8007954:	2000      	movne	r0, #0
 8007956:	bd80      	popne	{r7, pc}
 8007958:	f8d0 1110 	ldr.w	r1, [r0, #272]	@ 0x110
 800795c:	f1b1 3fff 	cmp.w	r1, #4294967295	@ 0xffffffff
 8007960:	bfc1      	itttt	gt
 8007962:	f8d0 20c4 	ldrgt.w	r2, [r0, #196]	@ 0xc4
 8007966:	f1b2 3fff 	cmpgt.w	r2, #4294967295	@ 0xffffffff
 800796a:	f8d0 309c 	ldrgt.w	r3, [r0, #156]	@ 0x9c
 800796e:	f8d0 0104 	ldrgt.w	r0, [r0, #260]	@ 0x104
 8007972:	bfc1      	itttt	gt
 8007974:	4410      	addgt	r0, r2
 8007976:	4419      	addgt	r1, r3
 8007978:	1a08      	subgt	r0, r1, r0
 800797a:	0fc0      	lsrgt	r0, r0, #31
 800797c:	bfc8      	it	gt
 800797e:	bd80      	popgt	{r7, pc}
 8007980:	f64a 3070 	movw	r0, #43888	@ 0xab70
 8007984:	f64a 32a8 	movw	r2, #43944	@ 0xaba8
 8007988:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800798c:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007990:	2171      	movs	r1, #113	@ 0x71
 8007992:	f7fb ff17 	bl	80037c4 <core::panicking::panic_fmt>

08007996 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply>:
 8007996:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007998:	af03      	add	r7, sp, #12
 800799a:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 800799e:	f002 06f0 	and.w	r6, r2, #240	@ 0xf0
 80079a2:	2ee0      	cmp	r6, #224	@ 0xe0
 80079a4:	d002      	beq.n	80079ac <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 80079a6:	1c56      	adds	r6, r2, #1
 80079a8:	2e02      	cmp	r6, #2
 80079aa:	d204      	bcs.n	80079b6 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x20>
 80079ac:	2104      	movs	r1, #4
 80079ae:	6001      	str	r1, [r0, #0]
 80079b0:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80079b4:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80079b6:	f8d1 c0f0 	ldr.w	ip, [r1, #240]	@ 0xf0
 80079ba:	f1bc 0f00 	cmp.w	ip, #0
 80079be:	eb0c 068c 	add.w	r6, ip, ip, lsl #2
 80079c2:	440e      	add	r6, r1
 80079c4:	f106 0ef4 	add.w	lr, r6, #244	@ 0xf4
 80079c8:	d023      	beq.n	8007a12 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x7c>
 80079ca:	f101 04f4 	add.w	r4, r1, #244	@ 0xf4
 80079ce:	f04f 38ff 	mov.w	r8, #4294967295	@ 0xffffffff
 80079d2:	e00d      	b.n	80079f0 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x5a>
 80079d4:	2500      	movs	r5, #0
 80079d6:	f006 061f 	and.w	r6, r6, #31
 80079da:	ea05 0509 	and.w	r5, r5, r9
 80079de:	fa28 f606 	lsr.w	r6, r8, r6
 80079e2:	ba36      	rev	r6, r6
 80079e4:	4335      	orrs	r5, r6
 80079e6:	42aa      	cmp	r2, r5
 80079e8:	d0e0      	beq.n	80079ac <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 80079ea:	3405      	adds	r4, #5
 80079ec:	4574      	cmp	r4, lr
 80079ee:	d010      	beq.n	8007a12 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x7c>
 80079f0:	7926      	ldrb	r6, [r4, #4]
 80079f2:	f8d4 9000 	ldr.w	r9, [r4]
 80079f6:	2e00      	cmp	r6, #0
 80079f8:	d0ec      	beq.n	80079d4 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x3e>
 80079fa:	f1a6 051f 	sub.w	r5, r6, #31
 80079fe:	b2ed      	uxtb	r5, r5
 8007a00:	2d02      	cmp	r5, #2
 8007a02:	d3f2      	bcc.n	80079ea <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x54>
 8007a04:	4275      	negs	r5, r6
 8007a06:	f005 051f 	and.w	r5, r5, #31
 8007a0a:	fa08 f505 	lsl.w	r5, r8, r5
 8007a0e:	ba2d      	rev	r5, r5
 8007a10:	e7e1      	b.n	80079d6 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x40>
 8007a12:	f8d7 9008 	ldr.w	r9, [r7, #8]
 8007a16:	f003 04f0 	and.w	r4, r3, #240	@ 0xf0
 8007a1a:	2ce0      	cmp	r4, #224	@ 0xe0
 8007a1c:	d003      	beq.n	8007a26 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x90>
 8007a1e:	f113 0801 	adds.w	r8, r3, #1
 8007a22:	d070      	beq.n	8007b06 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x170>
 8007a24:	bb33      	cbnz	r3, 8007a74 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xde>
 8007a26:	f1bc 0f00 	cmp.w	ip, #0
 8007a2a:	d0bf      	beq.n	80079ac <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007a2c:	f101 05f4 	add.w	r5, r1, #244	@ 0xf4
 8007a30:	f04f 38ff 	mov.w	r8, #4294967295	@ 0xffffffff
 8007a34:	e00d      	b.n	8007a52 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xbc>
 8007a36:	2600      	movs	r6, #0
 8007a38:	f004 041f 	and.w	r4, r4, #31
 8007a3c:	ea06 060a 	and.w	r6, r6, sl
 8007a40:	fa28 f404 	lsr.w	r4, r8, r4
 8007a44:	ba24      	rev	r4, r4
 8007a46:	4334      	orrs	r4, r6
 8007a48:	42a3      	cmp	r3, r4
 8007a4a:	d05c      	beq.n	8007b06 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x170>
 8007a4c:	3505      	adds	r5, #5
 8007a4e:	4575      	cmp	r5, lr
 8007a50:	d0ac      	beq.n	80079ac <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007a52:	792c      	ldrb	r4, [r5, #4]
 8007a54:	f8d5 a000 	ldr.w	sl, [r5]
 8007a58:	2c00      	cmp	r4, #0
 8007a5a:	d0ec      	beq.n	8007a36 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xa0>
 8007a5c:	f1a4 061f 	sub.w	r6, r4, #31
 8007a60:	b2f6      	uxtb	r6, r6
 8007a62:	2e02      	cmp	r6, #2
 8007a64:	d3f2      	bcc.n	8007a4c <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xb6>
 8007a66:	4266      	negs	r6, r4
 8007a68:	f006 061f 	and.w	r6, r6, #31
 8007a6c:	fa08 f606 	lsl.w	r6, r8, r6
 8007a70:	ba36      	rev	r6, r6
 8007a72:	e7e1      	b.n	8007a38 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xa2>
 8007a74:	f1bc 0f00 	cmp.w	ip, #0
 8007a78:	d023      	beq.n	8007ac2 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x12c>
 8007a7a:	f101 06f4 	add.w	r6, r1, #244	@ 0xf4
 8007a7e:	f04f 3aff 	mov.w	sl, #4294967295	@ 0xffffffff
 8007a82:	e00d      	b.n	8007aa0 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x10a>
 8007a84:	2500      	movs	r5, #0
 8007a86:	f004 041f 	and.w	r4, r4, #31
 8007a8a:	ea05 050b 	and.w	r5, r5, fp
 8007a8e:	fa2a f404 	lsr.w	r4, sl, r4
 8007a92:	ba24      	rev	r4, r4
 8007a94:	432c      	orrs	r4, r5
 8007a96:	42a3      	cmp	r3, r4
 8007a98:	d032      	beq.n	8007b00 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16a>
 8007a9a:	3605      	adds	r6, #5
 8007a9c:	4576      	cmp	r6, lr
 8007a9e:	d010      	beq.n	8007ac2 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x12c>
 8007aa0:	7934      	ldrb	r4, [r6, #4]
 8007aa2:	f8d6 b000 	ldr.w	fp, [r6]
 8007aa6:	2c00      	cmp	r4, #0
 8007aa8:	d0ec      	beq.n	8007a84 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xee>
 8007aaa:	f1a4 051f 	sub.w	r5, r4, #31
 8007aae:	b2ed      	uxtb	r5, r5
 8007ab0:	2d02      	cmp	r5, #2
 8007ab2:	d3f2      	bcc.n	8007a9a <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x104>
 8007ab4:	4265      	negs	r5, r4
 8007ab6:	f005 051f 	and.w	r5, r5, #31
 8007aba:	fa0a f505 	lsl.w	r5, sl, r5
 8007abe:	ba2d      	rev	r5, r5
 8007ac0:	e7e1      	b.n	8007a86 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0xf0>
 8007ac2:	2102      	movs	r1, #2
 8007ac4:	f899 8000 	ldrb.w	r8, [r9]
 8007ac8:	6001      	str	r1, [r0, #0]
 8007aca:	1d04      	adds	r4, r0, #4
 8007acc:	e9d9 ec02 	ldrd	lr, ip, [r9, #8]
 8007ad0:	f1b8 0f02 	cmp.w	r8, #2
 8007ad4:	e8b9 0062 	ldmia.w	r9!, {r1, r5, r6}
 8007ad8:	c462      	stmia	r4!, {r1, r5, r6}
 8007ada:	e899 0462 	ldmia.w	r9, {r1, r5, r6, sl}
 8007ade:	e884 0462 	stmia.w	r4, {r1, r5, r6, sl}
 8007ae2:	f44f 71a0 	mov.w	r1, #320	@ 0x140
 8007ae6:	f8a0 1060 	strh.w	r1, [r0, #96]	@ 0x60
 8007aea:	f10e 011c 	add.w	r1, lr, #28
 8007aee:	bf38      	it	cc
 8007af0:	f10c 0108 	addcc.w	r1, ip, #8
 8007af4:	6543      	str	r3, [r0, #84]	@ 0x54
 8007af6:	6582      	str	r2, [r0, #88]	@ 0x58
 8007af8:	65c1      	str	r1, [r0, #92]	@ 0x5c
 8007afa:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007afe:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007b00:	f1b8 0f00 	cmp.w	r8, #0
 8007b04:	d18f      	bne.n	8007a26 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x90>
 8007b06:	f899 3000 	ldrb.w	r3, [r9]
 8007b0a:	2b01      	cmp	r3, #1
 8007b0c:	f47f af4e 	bne.w	80079ac <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007b10:	f1bc 0f00 	cmp.w	ip, #0
 8007b14:	f43f af4a 	beq.w	80079ac <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x16>
 8007b18:	2302      	movs	r3, #2
 8007b1a:	f8d1 c0f4 	ldr.w	ip, [r1, #244]	@ 0xf4
 8007b1e:	6003      	str	r3, [r0, #0]
 8007b20:	1d06      	adds	r6, r0, #4
 8007b22:	f8d9 e00c 	ldr.w	lr, [r9, #12]
 8007b26:	e8b9 0032 	ldmia.w	r9!, {r1, r4, r5}
 8007b2a:	c632      	stmia	r6!, {r1, r4, r5}
 8007b2c:	e899 003a 	ldmia.w	r9, {r1, r3, r4, r5}
 8007b30:	c63a      	stmia	r6!, {r1, r3, r4, r5}
 8007b32:	f44f 71a0 	mov.w	r1, #320	@ 0x140
 8007b36:	f8a0 1060 	strh.w	r1, [r0, #96]	@ 0x60
 8007b3a:	f10e 0108 	add.w	r1, lr, #8
 8007b3e:	f8c0 c054 	str.w	ip, [r0, #84]	@ 0x54
 8007b42:	6582      	str	r2, [r0, #88]	@ 0x58
 8007b44:	65c1      	str	r1, [r0, #92]	@ 0x5c
 8007b46:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007b4a:	bdf0      	pop	{r4, r5, r6, r7, pc}

08007b4c <<smoltcp::wire::ip::Address as core::fmt::Display>::fmt>:
 8007b4c:	b580      	push	{r7, lr}
 8007b4e:	466f      	mov	r7, sp
 8007b50:	b084      	sub	sp, #16
 8007b52:	f244 02af 	movw	r2, #16559	@ 0x40af
 8007b56:	6800      	ldr	r0, [r0, #0]
 8007b58:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007b5c:	9001      	str	r0, [sp, #4]
 8007b5e:	9203      	str	r2, [sp, #12]
 8007b60:	aa01      	add	r2, sp, #4
 8007b62:	e9d1 0100 	ldrd	r0, r1, [r1]
 8007b66:	ab02      	add	r3, sp, #8
 8007b68:	9202      	str	r2, [sp, #8]
 8007b6a:	f649 42d1 	movw	r2, #40145	@ 0x9cd1
 8007b6e:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007b72:	f7fb fc36 	bl	80033e2 <core::fmt::write>
 8007b76:	b004      	add	sp, #16
 8007b78:	bd80      	pop	{r7, pc}
 8007b7a:	d4d4      	bmi.n	8007b26 <smoltcp::iface::interface::ipv4::<impl smoltcp::iface::interface::InterfaceInner>::icmpv4_reply+0x190>

08007b7c <smoltcp::wire::ipv4::Repr::emit>:
 8007b7c:	b5d0      	push	{r4, r6, r7, lr}
 8007b7e:	af02      	add	r7, sp, #8
 8007b80:	2a00      	cmp	r2, #0
 8007b82:	f000 809a 	beq.w	8007cba <smoltcp::wire::ipv4::Repr::emit+0x13e>
 8007b86:	f04f 0c45 	mov.w	ip, #69	@ 0x45
 8007b8a:	2a01      	cmp	r2, #1
 8007b8c:	f881 c000 	strb.w	ip, [r1]
 8007b90:	f000 809b 	beq.w	8007cca <smoltcp::wire::ipv4::Repr::emit+0x14e>
 8007b94:	f04f 0c00 	mov.w	ip, #0
 8007b98:	2a03      	cmp	r2, #3
 8007b9a:	f881 c001 	strb.w	ip, [r1, #1]
 8007b9e:	d964      	bls.n	8007c6a <smoltcp::wire::ipv4::Repr::emit+0xee>
 8007ba0:	f8b0 c008 	ldrh.w	ip, [r0, #8]
 8007ba4:	2a05      	cmp	r2, #5
 8007ba6:	f10c 0c14 	add.w	ip, ip, #20
 8007baa:	fa9c fc8c 	rev.w	ip, ip
 8007bae:	ea4f 4c1c 	mov.w	ip, ip, lsr #16
 8007bb2:	f8a1 c002 	strh.w	ip, [r1, #2]
 8007bb6:	d960      	bls.n	8007c7a <smoltcp::wire::ipv4::Repr::emit+0xfe>
 8007bb8:	f04f 0c00 	mov.w	ip, #0
 8007bbc:	2a07      	cmp	r2, #7
 8007bbe:	f8a1 c004 	strh.w	ip, [r1, #4]
 8007bc2:	d962      	bls.n	8007c8a <smoltcp::wire::ipv4::Repr::emit+0x10e>
 8007bc4:	f04f 0c40 	mov.w	ip, #64	@ 0x40
 8007bc8:	2a08      	cmp	r2, #8
 8007bca:	f8a1 c006 	strh.w	ip, [r1, #6]
 8007bce:	f000 8084 	beq.w	8007cda <smoltcp::wire::ipv4::Repr::emit+0x15e>
 8007bd2:	f890 c00d 	ldrb.w	ip, [r0, #13]
 8007bd6:	f890 e00c 	ldrb.w	lr, [r0, #12]
 8007bda:	f881 e008 	strb.w	lr, [r1, #8]
 8007bde:	e8df f00c 	tbb	[pc, ip]
 8007be2:	2424      	.short	0x2424
 8007be4:	190d0724 	.word	0x190d0724
 8007be8:	1322161c 	.word	0x1322161c
 8007bec:	00100a1f 	.word	0x00100a1f
 8007bf0:	f04f 0c06 	mov.w	ip, #6
 8007bf4:	e019      	b.n	8007c2a <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007bf6:	f04f 0c3c 	mov.w	ip, #60	@ 0x3c
 8007bfa:	e016      	b.n	8007c2a <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007bfc:	f04f 0c11 	mov.w	ip, #17
 8007c00:	e013      	b.n	8007c2a <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007c02:	f890 c00e 	ldrb.w	ip, [r0, #14]
 8007c06:	e010      	b.n	8007c2a <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007c08:	f04f 0c3a 	mov.w	ip, #58	@ 0x3a
 8007c0c:	e00d      	b.n	8007c2a <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007c0e:	f04f 0c32 	mov.w	ip, #50	@ 0x32
 8007c12:	e00a      	b.n	8007c2a <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007c14:	f04f 0c2b 	mov.w	ip, #43	@ 0x2b
 8007c18:	e007      	b.n	8007c2a <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007c1a:	f04f 0c2c 	mov.w	ip, #44	@ 0x2c
 8007c1e:	e004      	b.n	8007c2a <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007c20:	f04f 0c3b 	mov.w	ip, #59	@ 0x3b
 8007c24:	e001      	b.n	8007c2a <smoltcp::wire::ipv4::Repr::emit+0xae>
 8007c26:	f04f 0c33 	mov.w	ip, #51	@ 0x33
 8007c2a:	2a09      	cmp	r2, #9
 8007c2c:	d95d      	bls.n	8007cea <smoltcp::wire::ipv4::Repr::emit+0x16e>
 8007c2e:	2a0f      	cmp	r2, #15
 8007c30:	f881 c009 	strb.w	ip, [r1, #9]
 8007c34:	d931      	bls.n	8007c9a <smoltcp::wire::ipv4::Repr::emit+0x11e>
 8007c36:	f8d0 c000 	ldr.w	ip, [r0]
 8007c3a:	2a13      	cmp	r2, #19
 8007c3c:	f8c1 c00c 	str.w	ip, [r1, #12]
 8007c40:	d933      	bls.n	8007caa <smoltcp::wire::ipv4::Repr::emit+0x12e>
 8007c42:	6840      	ldr	r0, [r0, #4]
 8007c44:	f013 0ffd 	tst.w	r3, #253	@ 0xfd
 8007c48:	6108      	str	r0, [r1, #16]
 8007c4a:	f04f 0000 	mov.w	r0, #0
 8007c4e:	d10a      	bne.n	8007c66 <smoltcp::wire::ipv4::Repr::emit+0xea>
 8007c50:	8148      	strh	r0, [r1, #10]
 8007c52:	4608      	mov	r0, r1
 8007c54:	460c      	mov	r4, r1
 8007c56:	2114      	movs	r1, #20
 8007c58:	f000 f9f1 	bl	800803e <smoltcp::wire::ip::checksum::data>
 8007c5c:	43c0      	mvns	r0, r0
 8007c5e:	ba00      	rev	r0, r0
 8007c60:	0c00      	lsrs	r0, r0, #16
 8007c62:	8160      	strh	r0, [r4, #10]
 8007c64:	bdd0      	pop	{r4, r6, r7, pc}
 8007c66:	8148      	strh	r0, [r1, #10]
 8007c68:	bdd0      	pop	{r4, r6, r7, pc}
 8007c6a:	f64a 5304 	movw	r3, #44292	@ 0xad04
 8007c6e:	2002      	movs	r0, #2
 8007c70:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007c74:	2104      	movs	r1, #4
 8007c76:	f7fb fd34 	bl	80036e2 <core::slice::index::slice_index_fail>
 8007c7a:	f64a 5344 	movw	r3, #44356	@ 0xad44
 8007c7e:	2004      	movs	r0, #4
 8007c80:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007c84:	2106      	movs	r1, #6
 8007c86:	f7fb fd2c 	bl	80036e2 <core::slice::index::slice_index_fail>
 8007c8a:	f64a 43c4 	movw	r3, #44228	@ 0xacc4
 8007c8e:	2006      	movs	r0, #6
 8007c90:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007c94:	2108      	movs	r1, #8
 8007c96:	f7fb fd24 	bl	80036e2 <core::slice::index::slice_index_fail>
 8007c9a:	f64a 43f4 	movw	r3, #44276	@ 0xacf4
 8007c9e:	200c      	movs	r0, #12
 8007ca0:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007ca4:	2110      	movs	r1, #16
 8007ca6:	f7fb fd1c 	bl	80036e2 <core::slice::index::slice_index_fail>
 8007caa:	f64a 43e4 	movw	r3, #44260	@ 0xace4
 8007cae:	2010      	movs	r0, #16
 8007cb0:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007cb4:	2114      	movs	r1, #20
 8007cb6:	f7fb fd14 	bl	80036e2 <core::slice::index::slice_index_fail>
 8007cba:	f64a 42d4 	movw	r2, #44244	@ 0xacd4
 8007cbe:	2000      	movs	r0, #0
 8007cc0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007cc4:	2100      	movs	r1, #0
 8007cc6:	f7fb fd8b 	bl	80037e0 <core::panicking::panic_bounds_check>
 8007cca:	f64a 5234 	movw	r2, #44340	@ 0xad34
 8007cce:	2001      	movs	r0, #1
 8007cd0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007cd4:	2101      	movs	r1, #1
 8007cd6:	f7fb fd83 	bl	80037e0 <core::panicking::panic_bounds_check>
 8007cda:	f64a 5254 	movw	r2, #44372	@ 0xad54
 8007cde:	2008      	movs	r0, #8
 8007ce0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007ce4:	2108      	movs	r1, #8
 8007ce6:	f7fb fd7b 	bl	80037e0 <core::panicking::panic_bounds_check>
 8007cea:	f64a 5214 	movw	r2, #44308	@ 0xad14
 8007cee:	2009      	movs	r0, #9
 8007cf0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8007cf4:	2109      	movs	r1, #9
 8007cf6:	f7fb fd73 	bl	80037e0 <core::panicking::panic_bounds_check>

08007cfa <defmt::export::fmt>:
 8007cfa:	b5b0      	push	{r4, r5, r7, lr}
 8007cfc:	af02      	add	r7, sp, #8
 8007cfe:	b082      	sub	sp, #8
 8007d00:	4605      	mov	r5, r0
 8007d02:	f240 0003 	movw	r0, #3
 8007d06:	f2c0 0000 	movt	r0, #0
 8007d0a:	460c      	mov	r4, r1
 8007d0c:	f827 0c0e 	strh.w	r0, [r7, #-14]
 8007d10:	f1a7 000e 	sub.w	r0, r7, #14
 8007d14:	2102      	movs	r1, #2
 8007d16:	f7fc fcc1 	bl	800469c <_defmt_write>
 8007d1a:	a801      	add	r0, sp, #4
 8007d1c:	2104      	movs	r1, #4
 8007d1e:	9401      	str	r4, [sp, #4]
 8007d20:	f7fc fcbc 	bl	800469c <_defmt_write>
 8007d24:	4628      	mov	r0, r5
 8007d26:	4621      	mov	r1, r4
 8007d28:	f7fc fcb8 	bl	800469c <_defmt_write>
 8007d2c:	b002      	add	sp, #8
 8007d2e:	bdb0      	pop	{r4, r5, r7, pc}

08007d30 <defmt::export::fmt>:
 8007d30:	b5d0      	push	{r4, r6, r7, lr}
 8007d32:	af02      	add	r7, sp, #8
 8007d34:	b082      	sub	sp, #8
 8007d36:	4604      	mov	r4, r0
 8007d38:	f240 0002 	movw	r0, #2
 8007d3c:	f2c0 0000 	movt	r0, #0
 8007d40:	2102      	movs	r1, #2
 8007d42:	f8ad 0004 	strh.w	r0, [sp, #4]
 8007d46:	a801      	add	r0, sp, #4
 8007d48:	f7fc fca8 	bl	800469c <_defmt_write>
 8007d4c:	f1a7 0009 	sub.w	r0, r7, #9
 8007d50:	2101      	movs	r1, #1
 8007d52:	f807 4c09 	strb.w	r4, [r7, #-9]
 8007d56:	f7fc fca1 	bl	800469c <_defmt_write>
 8007d5a:	b002      	add	sp, #8
 8007d5c:	bdd0      	pop	{r4, r6, r7, pc}

08007d5e <defmt::export::fmt>:
 8007d5e:	b5d0      	push	{r4, r6, r7, lr}
 8007d60:	af02      	add	r7, sp, #8
 8007d62:	b084      	sub	sp, #16
 8007d64:	4604      	mov	r4, r0
 8007d66:	f240 0007 	movw	r0, #7
 8007d6a:	f2c0 0000 	movt	r0, #0
 8007d6e:	2102      	movs	r1, #2
 8007d70:	f827 0c16 	strh.w	r0, [r7, #-22]
 8007d74:	f1a7 0016 	sub.w	r0, r7, #22
 8007d78:	f7fc fc90 	bl	800469c <_defmt_write>
 8007d7c:	f240 003d 	movw	r0, #61	@ 0x3d
 8007d80:	2102      	movs	r1, #2
 8007d82:	f2c0 0000 	movt	r0, #0
 8007d86:	f8ad 0004 	strh.w	r0, [sp, #4]
 8007d8a:	a801      	add	r0, sp, #4
 8007d8c:	f7fc fc86 	bl	800469c <_defmt_write>
 8007d90:	f240 0008 	movw	r0, #8
 8007d94:	2102      	movs	r1, #2
 8007d96:	f2c0 0000 	movt	r0, #0
 8007d9a:	f827 0c12 	strh.w	r0, [r7, #-18]
 8007d9e:	f1a7 0012 	sub.w	r0, r7, #18
 8007da2:	f7fc fc7b 	bl	800469c <_defmt_write>
 8007da6:	a802      	add	r0, sp, #8
 8007da8:	2104      	movs	r1, #4
 8007daa:	9402      	str	r4, [sp, #8]
 8007dac:	f7fc fc76 	bl	800469c <_defmt_write>
 8007db0:	2000      	movs	r0, #0
 8007db2:	2102      	movs	r1, #2
 8007db4:	f827 0c0a 	strh.w	r0, [r7, #-10]
 8007db8:	f1a7 000a 	sub.w	r0, r7, #10
 8007dbc:	f7fc fc6e 	bl	800469c <_defmt_write>
 8007dc0:	b004      	add	sp, #16
 8007dc2:	bdd0      	pop	{r4, r6, r7, pc}

08007dc4 <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with>:
 8007dc4:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007dc6:	af03      	add	r7, sp, #12
 8007dc8:	f84d bd04 	str.w	fp, [sp, #-4]!
 8007dcc:	68ce      	ldr	r6, [r1, #12]
 8007dce:	2e00      	cmp	r6, #0
 8007dd0:	bf04      	itt	eq
 8007dd2:	2500      	moveq	r5, #0
 8007dd4:	608d      	streq	r5, [r1, #8]
 8007dd6:	684c      	ldr	r4, [r1, #4]
 8007dd8:	b134      	cbz	r4, 8007de8 <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x24>
 8007dda:	688d      	ldr	r5, [r1, #8]
 8007ddc:	4435      	add	r5, r6
 8007dde:	fbb5 f3f4 	udiv	r3, r5, r4
 8007de2:	fb03 5c14 	mls	ip, r3, r4, r5
 8007de6:	e001      	b.n	8007dec <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x28>
 8007de8:	f04f 0c00 	mov.w	ip, #0
 8007dec:	1ba5      	subs	r5, r4, r6
 8007dee:	eba4 030c 	sub.w	r3, r4, ip
 8007df2:	42ab      	cmp	r3, r5
 8007df4:	bf38      	it	cc
 8007df6:	461d      	movcc	r5, r3
 8007df8:	eb15 0e0c 	adds.w	lr, r5, ip
 8007dfc:	d20e      	bcs.n	8007e1c <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x58>
 8007dfe:	45a6      	cmp	lr, r4
 8007e00:	d80c      	bhi.n	8007e1c <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with+0x58>
 8007e02:	680b      	ldr	r3, [r1, #0]
 8007e04:	4295      	cmp	r5, r2
 8007e06:	bf38      	it	cc
 8007e08:	462a      	movcc	r2, r5
 8007e0a:	6082      	str	r2, [r0, #8]
 8007e0c:	4463      	add	r3, ip
 8007e0e:	e9c0 2300 	strd	r2, r3, [r0]
 8007e12:	1990      	adds	r0, r2, r6
 8007e14:	60c8      	str	r0, [r1, #12]
 8007e16:	f85d bb04 	ldr.w	fp, [sp], #4
 8007e1a:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007e1c:	f64a 53fc 	movw	r3, #44540	@ 0xadfc
 8007e20:	4660      	mov	r0, ip
 8007e22:	f6c0 0300 	movt	r3, #2048	@ 0x800
 8007e26:	4671      	mov	r1, lr
 8007e28:	4622      	mov	r2, r4
 8007e2a:	f7fb fc5a 	bl	80036e2 <core::slice::index::slice_index_fail>

08007e2e <smoltcp::socket::udp::Socket::accepts>:
 8007e2e:	4684      	mov	ip, r0
 8007e30:	8800      	ldrh	r0, [r0, #0]
 8007e32:	b29b      	uxth	r3, r3
 8007e34:	4298      	cmp	r0, r3
 8007e36:	d10d      	bne.n	8007e54 <smoltcp::socket::udp::Socket::accepts+0x26>
 8007e38:	f89c 3002 	ldrb.w	r3, [ip, #2]
 8007e3c:	2001      	movs	r0, #1
 8007e3e:	2b01      	cmp	r3, #1
 8007e40:	bf18      	it	ne
 8007e42:	4770      	bxne	lr
 8007e44:	f8dc 3003 	ldr.w	r3, [ip, #3]
 8007e48:	4293      	cmp	r3, r2
 8007e4a:	bf18      	it	ne
 8007e4c:	f112 0301 	addsne.w	r3, r2, #1
 8007e50:	d102      	bne.n	8007e58 <smoltcp::socket::udp::Socket::accepts+0x2a>
 8007e52:	4770      	bx	lr
 8007e54:	2000      	movs	r0, #0
 8007e56:	4770      	bx	lr
 8007e58:	b5d0      	push	{r4, r6, r7, lr}
 8007e5a:	af02      	add	r7, sp, #8
 8007e5c:	f8d1 00f0 	ldr.w	r0, [r1, #240]	@ 0xf0
 8007e60:	b328      	cbz	r0, 8007eae <smoltcp::socket::udp::Socket::accepts+0x80>
 8007e62:	eb00 0080 	add.w	r0, r0, r0, lsl #2
 8007e66:	f04f 3cff 	mov.w	ip, #4294967295	@ 0xffffffff
 8007e6a:	4408      	add	r0, r1
 8007e6c:	31f4      	adds	r1, #244	@ 0xf4
 8007e6e:	f100 0ef4 	add.w	lr, r0, #244	@ 0xf4
 8007e72:	e00c      	b.n	8007e8e <smoltcp::socket::udp::Socket::accepts+0x60>
 8007e74:	2400      	movs	r4, #0
 8007e76:	f003 031f 	and.w	r3, r3, #31
 8007e7a:	4020      	ands	r0, r4
 8007e7c:	fa2c f303 	lsr.w	r3, ip, r3
 8007e80:	ba1b      	rev	r3, r3
 8007e82:	4318      	orrs	r0, r3
 8007e84:	4282      	cmp	r2, r0
 8007e86:	d019      	beq.n	8007ebc <smoltcp::socket::udp::Socket::accepts+0x8e>
 8007e88:	3105      	adds	r1, #5
 8007e8a:	4571      	cmp	r1, lr
 8007e8c:	d00f      	beq.n	8007eae <smoltcp::socket::udp::Socket::accepts+0x80>
 8007e8e:	790b      	ldrb	r3, [r1, #4]
 8007e90:	6808      	ldr	r0, [r1, #0]
 8007e92:	2b00      	cmp	r3, #0
 8007e94:	d0ee      	beq.n	8007e74 <smoltcp::socket::udp::Socket::accepts+0x46>
 8007e96:	f1a3 041f 	sub.w	r4, r3, #31
 8007e9a:	b2e4      	uxtb	r4, r4
 8007e9c:	2c02      	cmp	r4, #2
 8007e9e:	d3f3      	bcc.n	8007e88 <smoltcp::socket::udp::Socket::accepts+0x5a>
 8007ea0:	425c      	negs	r4, r3
 8007ea2:	f004 041f 	and.w	r4, r4, #31
 8007ea6:	fa0c f404 	lsl.w	r4, ip, r4
 8007eaa:	ba24      	rev	r4, r4
 8007eac:	e7e3      	b.n	8007e76 <smoltcp::socket::udp::Socket::accepts+0x48>
 8007eae:	f002 00f0 	and.w	r0, r2, #240	@ 0xf0
 8007eb2:	38e0      	subs	r0, #224	@ 0xe0
 8007eb4:	fab0 f080 	clz	r0, r0
 8007eb8:	0940      	lsrs	r0, r0, #5
 8007eba:	bdd0      	pop	{r4, r6, r7, pc}
 8007ebc:	2001      	movs	r0, #1
 8007ebe:	bdd0      	pop	{r4, r6, r7, pc}

08007ec0 <smoltcp::socket::udp::Socket::process>:
 8007ec0:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007ec2:	af03      	add	r7, sp, #12
 8007ec4:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8007ec8:	b085      	sub	sp, #20
 8007eca:	f8d7 a010 	ldr.w	sl, [r7, #16]
 8007ece:	69c5      	ldr	r5, [r0, #28]
 8007ed0:	4555      	cmp	r5, sl
 8007ed2:	d34f      	bcc.n	8007f74 <smoltcp::socket::udp::Socket::process+0xb4>
 8007ed4:	f8d0 b00c 	ldr.w	fp, [r0, #12]
 8007ed8:	f8d0 c014 	ldr.w	ip, [r0, #20]
 8007edc:	45e3      	cmp	fp, ip
 8007ede:	d049      	beq.n	8007f74 <smoltcp::socket::udp::Socket::process+0xb4>
 8007ee0:	6a44      	ldr	r4, [r0, #36]	@ 0x24
 8007ee2:	e9d7 9802 	ldrd	r9, r8, [r7, #8]
 8007ee6:	eba5 0e04 	sub.w	lr, r5, r4
 8007eea:	2c00      	cmp	r4, #0
 8007eec:	bf04      	itt	eq
 8007eee:	2600      	moveq	r6, #0
 8007ef0:	6206      	streq	r6, [r0, #32]
 8007ef2:	b135      	cbz	r5, 8007f02 <smoltcp::socket::udp::Socket::process+0x42>
 8007ef4:	6a06      	ldr	r6, [r0, #32]
 8007ef6:	4434      	add	r4, r6
 8007ef8:	fbb4 f6f5 	udiv	r6, r4, r5
 8007efc:	fb06 4415 	mls	r4, r6, r5, r4
 8007f00:	e000      	b.n	8007f04 <smoltcp::socket::udp::Socket::process+0x44>
 8007f02:	2400      	movs	r4, #0
 8007f04:	1b2c      	subs	r4, r5, r4
 8007f06:	4675      	mov	r5, lr
 8007f08:	4574      	cmp	r4, lr
 8007f0a:	bf38      	it	cc
 8007f0c:	4625      	movcc	r5, r4
 8007f0e:	45d6      	cmp	lr, sl
 8007f10:	d330      	bcc.n	8007f74 <smoltcp::socket::udp::Socket::process+0xb4>
 8007f12:	4554      	cmp	r4, sl
 8007f14:	d22c      	bcs.n	8007f70 <smoltcp::socket::udp::Socket::process+0xb0>
 8007f16:	ebae 0605 	sub.w	r6, lr, r5
 8007f1a:	4556      	cmp	r6, sl
 8007f1c:	d32a      	bcc.n	8007f74 <smoltcp::socket::udp::Socket::process+0xb4>
 8007f1e:	f1bb 0f00 	cmp.w	fp, #0
 8007f22:	e9cd 1300 	strd	r1, r3, [sp]
 8007f26:	f000 8084 	beq.w	8008032 <smoltcp::socket::udp::Socket::process+0x172>
 8007f2a:	6903      	ldr	r3, [r0, #16]
 8007f2c:	4614      	mov	r4, r2
 8007f2e:	6882      	ldr	r2, [r0, #8]
 8007f30:	f100 0118 	add.w	r1, r0, #24
 8007f34:	4463      	add	r3, ip
 8007f36:	fbb3 f6fb 	udiv	r6, r3, fp
 8007f3a:	fb06 331b 	mls	r3, r6, fp, r3
 8007f3e:	f10c 0601 	add.w	r6, ip, #1
 8007f42:	6146      	str	r6, [r0, #20]
 8007f44:	4606      	mov	r6, r0
 8007f46:	eb03 0383 	add.w	r3, r3, r3, lsl #2
 8007f4a:	f842 5023 	str.w	r5, [r2, r3, lsl #2]
 8007f4e:	eb02 0283 	add.w	r2, r2, r3, lsl #2
 8007f52:	2302      	movs	r3, #2
 8007f54:	7393      	strb	r3, [r2, #14]
 8007f56:	aa02      	add	r2, sp, #8
 8007f58:	4610      	mov	r0, r2
 8007f5a:	462a      	mov	r2, r5
 8007f5c:	f7ff ff32 	bl	8007dc4 <smoltcp::storage::ring_buffer::RingBuffer<T>::enqueue_many_with>
 8007f60:	4630      	mov	r0, r6
 8007f62:	f8d6 b00c 	ldr.w	fp, [r6, #12]
 8007f66:	f8d6 c014 	ldr.w	ip, [r6, #20]
 8007f6a:	4622      	mov	r2, r4
 8007f6c:	e9dd 1300 	ldrd	r1, r3, [sp]
 8007f70:	45e3      	cmp	fp, ip
 8007f72:	d103      	bne.n	8007f7c <smoltcp::socket::udp::Socket::process+0xbc>
 8007f74:	b005      	add	sp, #20
 8007f76:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8007f7a:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007f7c:	f1bb 0f00 	cmp.w	fp, #0
 8007f80:	d057      	beq.n	8008032 <smoltcp::socket::udp::Socket::process+0x172>
 8007f82:	6905      	ldr	r5, [r0, #16]
 8007f84:	6884      	ldr	r4, [r0, #8]
 8007f86:	4465      	add	r5, ip
 8007f88:	f8d0 e024 	ldr.w	lr, [r0, #36]	@ 0x24
 8007f8c:	fbb5 f6fb 	udiv	r6, r5, fp
 8007f90:	fb06 551b 	mls	r5, r6, fp, r5
 8007f94:	f10c 0601 	add.w	r6, ip, #1
 8007f98:	6146      	str	r6, [r0, #20]
 8007f9a:	f1be 0f00 	cmp.w	lr, #0
 8007f9e:	eb05 0585 	add.w	r5, r5, r5, lsl #2
 8007fa2:	eb04 0685 	add.w	r6, r4, r5, lsl #2
 8007fa6:	f844 a025 	str.w	sl, [r4, r5, lsl #2]
 8007faa:	f8c6 300f 	str.w	r3, [r6, #15]
 8007fae:	f04f 0301 	mov.w	r3, #1
 8007fb2:	73b3      	strb	r3, [r6, #14]
 8007fb4:	f8a6 900c 	strh.w	r9, [r6, #12]
 8007fb8:	e9c6 1201 	strd	r1, r2, [r6, #4]
 8007fbc:	bf04      	itt	eq
 8007fbe:	2100      	moveq	r1, #0
 8007fc0:	6201      	streq	r1, [r0, #32]
 8007fc2:	69c2      	ldr	r2, [r0, #28]
 8007fc4:	b132      	cbz	r2, 8007fd4 <smoltcp::socket::udp::Socket::process+0x114>
 8007fc6:	6a01      	ldr	r1, [r0, #32]
 8007fc8:	4471      	add	r1, lr
 8007fca:	fbb1 f3f2 	udiv	r3, r1, r2
 8007fce:	fb03 1512 	mls	r5, r3, r2, r1
 8007fd2:	e000      	b.n	8007fd6 <smoltcp::socket::udp::Socket::process+0x116>
 8007fd4:	2500      	movs	r5, #0
 8007fd6:	eba2 030e 	sub.w	r3, r2, lr
 8007fda:	1b51      	subs	r1, r2, r5
 8007fdc:	4299      	cmp	r1, r3
 8007fde:	bf38      	it	cc
 8007fe0:	460b      	movcc	r3, r1
 8007fe2:	1959      	adds	r1, r3, r5
 8007fe4:	d21e      	bcs.n	8008024 <smoltcp::socket::udp::Socket::process+0x164>
 8007fe6:	4291      	cmp	r1, r2
 8007fe8:	d81c      	bhi.n	8008024 <smoltcp::socket::udp::Socket::process+0x164>
 8007fea:	6982      	ldr	r2, [r0, #24]
 8007fec:	4553      	cmp	r3, sl
 8007fee:	4651      	mov	r1, sl
 8007ff0:	bf38      	it	cc
 8007ff2:	4619      	movcc	r1, r3
 8007ff4:	459a      	cmp	sl, r3
 8007ff6:	eb01 060e 	add.w	r6, r1, lr
 8007ffa:	6246      	str	r6, [r0, #36]	@ 0x24
 8007ffc:	bf9f      	itttt	ls
 8007ffe:	1950      	addls	r0, r2, r5
 8008000:	4641      	movls	r1, r8
 8008002:	4652      	movls	r2, sl
 8008004:	b005      	addls	sp, #20
 8008006:	bf9e      	ittt	ls
 8008008:	e8bd 0f00 	ldmials.w	sp!, {r8, r9, sl, fp}
 800800c:	e8bd 40f0 	ldmials.w	sp!, {r4, r5, r6, r7, lr}
 8008010:	f001 b96f 	bls.w	80092f2 <__aeabi_memcpy>
 8008014:	f64a 6240 	movw	r2, #44608	@ 0xae40
 8008018:	4608      	mov	r0, r1
 800801a:	f6c0 0200 	movt	r2, #2048	@ 0x800
 800801e:	4651      	mov	r1, sl
 8008020:	f7fb ffec 	bl	8003ffc <core::slice::copy_from_slice_impl::len_mismatch_fail>
 8008024:	f64a 53fc 	movw	r3, #44540	@ 0xadfc
 8008028:	4628      	mov	r0, r5
 800802a:	f6c0 0300 	movt	r3, #2048	@ 0x800
 800802e:	f7fb fb58 	bl	80036e2 <core::slice::index::slice_index_fail>
 8008032:	f64a 50b4 	movw	r0, #44468	@ 0xadb4
 8008036:	f6c0 0000 	movt	r0, #2048	@ 0x800
 800803a:	f7fc f82e 	bl	800409a <core::panicking::panic_const::panic_const_rem_by_zero>

0800803e <smoltcp::wire::ip::checksum::data>:
 800803e:	b5f0      	push	{r4, r5, r6, r7, lr}
 8008040:	af03      	add	r7, sp, #12
 8008042:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 8008046:	2920      	cmp	r1, #32
 8008048:	f0c0 80b5 	bcc.w	80081b6 <smoltcp::wire::ip::checksum::data+0x178>
 800804c:	f1a1 0c20 	sub.w	ip, r1, #32
 8008050:	ea5f 125c 	movs.w	r2, ip, lsr #5
 8008054:	f000 80b1 	beq.w	80081ba <smoltcp::wire::ip::checksum::data+0x17c>
 8008058:	3201      	adds	r2, #1
 800805a:	f022 0301 	bic.w	r3, r2, #1
 800805e:	2200      	movs	r2, #0
 8008060:	f830 6b40 	ldrh.w	r6, [r0], #64
 8008064:	3940      	subs	r1, #64	@ 0x40
 8008066:	3b02      	subs	r3, #2
 8008068:	f830 4c3e 	ldrh.w	r4, [r0, #-62]
 800806c:	ba36      	rev	r6, r6
 800806e:	f830 5c3c 	ldrh.w	r5, [r0, #-60]
 8008072:	ba24      	rev	r4, r4
 8008074:	eb02 4216 	add.w	r2, r2, r6, lsr #16
 8008078:	f830 6c3a 	ldrh.w	r6, [r0, #-58]
 800807c:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008080:	ba2c      	rev	r4, r5
 8008082:	f830 5c38 	ldrh.w	r5, [r0, #-56]
 8008086:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800808a:	ba34      	rev	r4, r6
 800808c:	f830 6c36 	ldrh.w	r6, [r0, #-54]
 8008090:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008094:	ba2c      	rev	r4, r5
 8008096:	f830 5c34 	ldrh.w	r5, [r0, #-52]
 800809a:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800809e:	ba34      	rev	r4, r6
 80080a0:	f830 6c32 	ldrh.w	r6, [r0, #-50]
 80080a4:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80080a8:	ba2c      	rev	r4, r5
 80080aa:	f830 5c30 	ldrh.w	r5, [r0, #-48]
 80080ae:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80080b2:	ba34      	rev	r4, r6
 80080b4:	f830 6c2e 	ldrh.w	r6, [r0, #-46]
 80080b8:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80080bc:	ba2c      	rev	r4, r5
 80080be:	f830 5c2c 	ldrh.w	r5, [r0, #-44]
 80080c2:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80080c6:	ba34      	rev	r4, r6
 80080c8:	f830 6c2a 	ldrh.w	r6, [r0, #-42]
 80080cc:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80080d0:	ba2c      	rev	r4, r5
 80080d2:	f830 5c28 	ldrh.w	r5, [r0, #-40]
 80080d6:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80080da:	ba34      	rev	r4, r6
 80080dc:	f830 6c26 	ldrh.w	r6, [r0, #-38]
 80080e0:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80080e4:	ba2c      	rev	r4, r5
 80080e6:	f830 5c24 	ldrh.w	r5, [r0, #-36]
 80080ea:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80080ee:	ba34      	rev	r4, r6
 80080f0:	f830 6c22 	ldrh.w	r6, [r0, #-34]
 80080f4:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80080f8:	ba2c      	rev	r4, r5
 80080fa:	f830 5c20 	ldrh.w	r5, [r0, #-32]
 80080fe:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008102:	ba34      	rev	r4, r6
 8008104:	f830 6c1e 	ldrh.w	r6, [r0, #-30]
 8008108:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800810c:	ba2c      	rev	r4, r5
 800810e:	f830 5c1c 	ldrh.w	r5, [r0, #-28]
 8008112:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008116:	ba34      	rev	r4, r6
 8008118:	f830 6c1a 	ldrh.w	r6, [r0, #-26]
 800811c:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008120:	ba2c      	rev	r4, r5
 8008122:	f830 5c18 	ldrh.w	r5, [r0, #-24]
 8008126:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800812a:	ba34      	rev	r4, r6
 800812c:	f830 6c16 	ldrh.w	r6, [r0, #-22]
 8008130:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008134:	ba2c      	rev	r4, r5
 8008136:	f830 5c14 	ldrh.w	r5, [r0, #-20]
 800813a:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800813e:	ba34      	rev	r4, r6
 8008140:	f830 6c12 	ldrh.w	r6, [r0, #-18]
 8008144:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008148:	ba2c      	rev	r4, r5
 800814a:	f830 5c10 	ldrh.w	r5, [r0, #-16]
 800814e:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008152:	ba34      	rev	r4, r6
 8008154:	f830 6c0e 	ldrh.w	r6, [r0, #-14]
 8008158:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800815c:	ba2c      	rev	r4, r5
 800815e:	f830 5c0c 	ldrh.w	r5, [r0, #-12]
 8008162:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008166:	ba34      	rev	r4, r6
 8008168:	f830 6c0a 	ldrh.w	r6, [r0, #-10]
 800816c:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008170:	ba2c      	rev	r4, r5
 8008172:	f830 5c08 	ldrh.w	r5, [r0, #-8]
 8008176:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800817a:	ba34      	rev	r4, r6
 800817c:	f830 9c06 	ldrh.w	r9, [r0, #-6]
 8008180:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 8008184:	ba2c      	rev	r4, r5
 8008186:	f830 8c04 	ldrh.w	r8, [r0, #-4]
 800818a:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800818e:	fa99 f489 	rev.w	r4, r9
 8008192:	f830 ec02 	ldrh.w	lr, [r0, #-2]
 8008196:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 800819a:	fa98 f488 	rev.w	r4, r8
 800819e:	fa9e f68e 	rev.w	r6, lr
 80081a2:	eb02 4214 	add.w	r2, r2, r4, lsr #16
 80081a6:	eb02 4216 	add.w	r2, r2, r6, lsr #16
 80081aa:	f47f af59 	bne.w	8008060 <smoltcp::wire::ip::checksum::data+0x22>
 80081ae:	ea5f 638c 	movs.w	r3, ip, lsl #26
 80081b2:	d506      	bpl.n	80081c2 <smoltcp::wire::ip::checksum::data+0x184>
 80081b4:	e058      	b.n	8008268 <smoltcp::wire::ip::checksum::data+0x22a>
 80081b6:	2200      	movs	r2, #0
 80081b8:	e056      	b.n	8008268 <smoltcp::wire::ip::checksum::data+0x22a>
 80081ba:	2200      	movs	r2, #0
 80081bc:	ea5f 638c 	movs.w	r3, ip, lsl #26
 80081c0:	d452      	bmi.n	8008268 <smoltcp::wire::ip::checksum::data+0x22a>
 80081c2:	f830 6b20 	ldrh.w	r6, [r0], #32
 80081c6:	3920      	subs	r1, #32
 80081c8:	f830 3c1e 	ldrh.w	r3, [r0, #-30]
 80081cc:	ba36      	rev	r6, r6
 80081ce:	f830 5c1c 	ldrh.w	r5, [r0, #-28]
 80081d2:	ba1b      	rev	r3, r3
 80081d4:	eb02 4216 	add.w	r2, r2, r6, lsr #16
 80081d8:	f830 6c1a 	ldrh.w	r6, [r0, #-26]
 80081dc:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 80081e0:	ba2b      	rev	r3, r5
 80081e2:	f830 5c18 	ldrh.w	r5, [r0, #-24]
 80081e6:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 80081ea:	ba33      	rev	r3, r6
 80081ec:	f830 6c16 	ldrh.w	r6, [r0, #-22]
 80081f0:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 80081f4:	ba2b      	rev	r3, r5
 80081f6:	f830 5c14 	ldrh.w	r5, [r0, #-20]
 80081fa:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 80081fe:	ba33      	rev	r3, r6
 8008200:	f830 6c12 	ldrh.w	r6, [r0, #-18]
 8008204:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008208:	ba2b      	rev	r3, r5
 800820a:	f830 5c10 	ldrh.w	r5, [r0, #-16]
 800820e:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008212:	ba33      	rev	r3, r6
 8008214:	f830 6c0e 	ldrh.w	r6, [r0, #-14]
 8008218:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 800821c:	ba2b      	rev	r3, r5
 800821e:	f830 5c0c 	ldrh.w	r5, [r0, #-12]
 8008222:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008226:	ba33      	rev	r3, r6
 8008228:	f830 6c0a 	ldrh.w	r6, [r0, #-10]
 800822c:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008230:	ba2b      	rev	r3, r5
 8008232:	f830 5c08 	ldrh.w	r5, [r0, #-8]
 8008236:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 800823a:	ba33      	rev	r3, r6
 800823c:	f830 4c06 	ldrh.w	r4, [r0, #-6]
 8008240:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008244:	ba2b      	rev	r3, r5
 8008246:	f830 ec04 	ldrh.w	lr, [r0, #-4]
 800824a:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 800824e:	ba23      	rev	r3, r4
 8008250:	f830 cc02 	ldrh.w	ip, [r0, #-2]
 8008254:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008258:	fa9e f38e 	rev.w	r3, lr
 800825c:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008260:	fa9c f38c 	rev.w	r3, ip
 8008264:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008268:	2902      	cmp	r1, #2
 800826a:	d306      	bcc.n	800827a <smoltcp::wire::ip::checksum::data+0x23c>
 800826c:	1e8d      	subs	r5, r1, #2
 800826e:	43eb      	mvns	r3, r5
 8008270:	f013 0f06 	tst.w	r3, #6
 8008274:	d103      	bne.n	800827e <smoltcp::wire::ip::checksum::data+0x240>
 8008276:	4603      	mov	r3, r0
 8008278:	e01a      	b.n	80082b0 <smoltcp::wire::ip::checksum::data+0x272>
 800827a:	4603      	mov	r3, r0
 800827c:	e02f      	b.n	80082de <smoltcp::wire::ip::checksum::data+0x2a0>
 800827e:	4603      	mov	r3, r0
 8008280:	f015 0f06 	tst.w	r5, #6
 8008284:	f833 6b02 	ldrh.w	r6, [r3], #2
 8008288:	ba36      	rev	r6, r6
 800828a:	eb02 4216 	add.w	r2, r2, r6, lsr #16
 800828e:	d00c      	beq.n	80082aa <smoltcp::wire::ip::checksum::data+0x26c>
 8008290:	8843      	ldrh	r3, [r0, #2]
 8008292:	ba1b      	rev	r3, r3
 8008294:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008298:	f005 0306 	and.w	r3, r5, #6
 800829c:	2b02      	cmp	r3, #2
 800829e:	d12b      	bne.n	80082f8 <smoltcp::wire::ip::checksum::data+0x2ba>
 80082a0:	1d03      	adds	r3, r0, #4
 80082a2:	3904      	subs	r1, #4
 80082a4:	2d06      	cmp	r5, #6
 80082a6:	d203      	bcs.n	80082b0 <smoltcp::wire::ip::checksum::data+0x272>
 80082a8:	e019      	b.n	80082de <smoltcp::wire::ip::checksum::data+0x2a0>
 80082aa:	4629      	mov	r1, r5
 80082ac:	2d06      	cmp	r5, #6
 80082ae:	d316      	bcc.n	80082de <smoltcp::wire::ip::checksum::data+0x2a0>
 80082b0:	f833 0b08 	ldrh.w	r0, [r3], #8
 80082b4:	3908      	subs	r1, #8
 80082b6:	2901      	cmp	r1, #1
 80082b8:	ba00      	rev	r0, r0
 80082ba:	f833 4c06 	ldrh.w	r4, [r3, #-6]
 80082be:	f833 5c04 	ldrh.w	r5, [r3, #-4]
 80082c2:	eb02 4010 	add.w	r0, r2, r0, lsr #16
 80082c6:	ba22      	rev	r2, r4
 80082c8:	f833 6c02 	ldrh.w	r6, [r3, #-2]
 80082cc:	eb00 4012 	add.w	r0, r0, r2, lsr #16
 80082d0:	ba2a      	rev	r2, r5
 80082d2:	eb00 4012 	add.w	r0, r0, r2, lsr #16
 80082d6:	ba32      	rev	r2, r6
 80082d8:	eb00 4212 	add.w	r2, r0, r2, lsr #16
 80082dc:	d8e8      	bhi.n	80082b0 <smoltcp::wire::ip::checksum::data+0x272>
 80082de:	2900      	cmp	r1, #0
 80082e0:	bf1c      	itt	ne
 80082e2:	7818      	ldrbne	r0, [r3, #0]
 80082e4:	eb02 2200 	addne.w	r2, r2, r0, lsl #8
 80082e8:	0c10      	lsrs	r0, r2, #16
 80082ea:	fa10 f082 	uxtah	r0, r0, r2
 80082ee:	eb00 4010 	add.w	r0, r0, r0, lsr #16
 80082f2:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80082f6:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80082f8:	8883      	ldrh	r3, [r0, #4]
 80082fa:	3906      	subs	r1, #6
 80082fc:	ba1b      	rev	r3, r3
 80082fe:	eb02 4213 	add.w	r2, r2, r3, lsr #16
 8008302:	1d83      	adds	r3, r0, #6
 8008304:	2d06      	cmp	r5, #6
 8008306:	d2d3      	bcs.n	80082b0 <smoltcp::wire::ip::checksum::data+0x272>
 8008308:	e7e9      	b.n	80082de <smoltcp::wire::ip::checksum::data+0x2a0>

0800830a <smoltcp::iface::route::Routes::lookup>:
 800830a:	b5f0      	push	{r4, r5, r6, r7, lr}
 800830c:	af03      	add	r7, sp, #12
 800830e:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8008312:	b081      	sub	sp, #4
 8008314:	1c53      	adds	r3, r2, #1
 8008316:	d007      	beq.n	8008328 <smoltcp::iface::route::Routes::lookup+0x1e>
 8008318:	f002 03f0 	and.w	r3, r2, #240	@ 0xf0
 800831c:	2be0      	cmp	r3, #224	@ 0xe0
 800831e:	bf1c      	itt	ne
 8008320:	b2d3      	uxtbne	r3, r2
 8008322:	ea53 2312 	orrsne.w	r3, r3, r2, lsr #8
 8008326:	d10a      	bne.n	800833e <smoltcp::iface::route::Routes::lookup+0x34>
 8008328:	f64a 60b0 	movw	r0, #44720	@ 0xaeb0
 800832c:	f64a 62d4 	movw	r2, #44756	@ 0xaed4
 8008330:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8008334:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8008338:	2123      	movs	r1, #35	@ 0x23
 800833a:	f7fb fa65 	bl	8003808 <core::panicking::panic>
 800833e:	6e0b      	ldr	r3, [r1, #96]	@ 0x60
 8008340:	2b00      	cmp	r3, #0
 8008342:	d072      	beq.n	800842a <smoltcp::iface::route::Routes::lookup+0x120>
 8008344:	eb03 0343 	add.w	r3, r3, r3, lsl #1
 8008348:	e9d7 ec02 	ldrd	lr, ip, [r7, #8]
 800834c:	2600      	movs	r6, #0
 800834e:	f04f 3aff 	mov.w	sl, #4294967295	@ 0xffffffff
 8008352:	eb01 1903 	add.w	r9, r1, r3, lsl #4
 8008356:	ea4f 1803 	mov.w	r8, r3, lsl #4
 800835a:	f1a8 0330 	sub.w	r3, r8, #48	@ 0x30
 800835e:	9300      	str	r3, [sp, #0]
 8008360:	e007      	b.n	8008372 <smoltcp::iface::route::Routes::lookup+0x68>
 8008362:	2400      	movs	r4, #0
 8008364:	6a5d      	ldr	r5, [r3, #36]	@ 0x24
 8008366:	4055      	eors	r5, r2
 8008368:	422c      	tst	r4, r5
 800836a:	d01d      	beq.n	80083a8 <smoltcp::iface::route::Routes::lookup+0x9e>
 800836c:	3630      	adds	r6, #48	@ 0x30
 800836e:	45b0      	cmp	r8, r6
 8008370:	d05b      	beq.n	800842a <smoltcp::iface::route::Routes::lookup+0x120>
 8008372:	198b      	adds	r3, r1, r6
 8008374:	691c      	ldr	r4, [r3, #16]
 8008376:	07e4      	lsls	r4, r4, #31
 8008378:	d006      	beq.n	8008388 <smoltcp::iface::route::Routes::lookup+0x7e>
 800837a:	e9d3 4506 	ldrd	r4, r5, [r3, #24]
 800837e:	ebb4 040e 	subs.w	r4, r4, lr
 8008382:	eb75 040c 	sbcs.w	r4, r5, ip
 8008386:	dbf1      	blt.n	800836c <smoltcp::iface::route::Routes::lookup+0x62>
 8008388:	f893 b028 	ldrb.w	fp, [r3, #40]	@ 0x28
 800838c:	f1bb 0f00 	cmp.w	fp, #0
 8008390:	d0e7      	beq.n	8008362 <smoltcp::iface::route::Routes::lookup+0x58>
 8008392:	f1cb 0400 	rsb	r4, fp, #0
 8008396:	f004 041f 	and.w	r4, r4, #31
 800839a:	fa0a f404 	lsl.w	r4, sl, r4
 800839e:	ba24      	rev	r4, r4
 80083a0:	6a5d      	ldr	r5, [r3, #36]	@ 0x24
 80083a2:	4055      	eors	r5, r2
 80083a4:	422c      	tst	r4, r5
 80083a6:	d1e1      	bne.n	800836c <smoltcp::iface::route::Routes::lookup+0x62>
 80083a8:	f103 0130 	add.w	r1, r3, #48	@ 0x30
 80083ac:	4549      	cmp	r1, r9
 80083ae:	d042      	beq.n	8008436 <smoltcp::iface::route::Routes::lookup+0x12c>
 80083b0:	9900      	ldr	r1, [sp, #0]
 80083b2:	f64a 25ab 	movw	r5, #43691	@ 0xaaab
 80083b6:	f6ca 25aa 	movt	r5, #43690	@ 0xaaaa
 80083ba:	f04f 38ff 	mov.w	r8, #4294967295	@ 0xffffffff
 80083be:	1b8c      	subs	r4, r1, r6
 80083c0:	f103 0158 	add.w	r1, r3, #88	@ 0x58
 80083c4:	fba4 4505 	umull	r4, r5, r4, r5
 80083c8:	096d      	lsrs	r5, r5, #5
 80083ca:	e00a      	b.n	80083e2 <smoltcp::iface::route::Routes::lookup+0xd8>
 80083cc:	fa5f f68b 	uxtb.w	r6, fp
 80083d0:	42a6      	cmp	r6, r4
 80083d2:	bf8c      	ite	hi
 80083d4:	4634      	movhi	r4, r6
 80083d6:	f1a1 0328 	subls.w	r3, r1, #40	@ 0x28
 80083da:	46a3      	mov	fp, r4
 80083dc:	3130      	adds	r1, #48	@ 0x30
 80083de:	3d01      	subs	r5, #1
 80083e0:	d029      	beq.n	8008436 <smoltcp::iface::route::Routes::lookup+0x12c>
 80083e2:	f851 4c18 	ldr.w	r4, [r1, #-24]
 80083e6:	07e4      	lsls	r4, r4, #31
 80083e8:	d006      	beq.n	80083f8 <smoltcp::iface::route::Routes::lookup+0xee>
 80083ea:	e951 4604 	ldrd	r4, r6, [r1, #-16]
 80083ee:	ebb4 040e 	subs.w	r4, r4, lr
 80083f2:	eb76 040c 	sbcs.w	r4, r6, ip
 80083f6:	dbf1      	blt.n	80083dc <smoltcp::iface::route::Routes::lookup+0xd2>
 80083f8:	780c      	ldrb	r4, [r1, #0]
 80083fa:	b16c      	cbz	r4, 8008418 <smoltcp::iface::route::Routes::lookup+0x10e>
 80083fc:	4266      	negs	r6, r4
 80083fe:	f006 061f 	and.w	r6, r6, #31
 8008402:	fa08 f606 	lsl.w	r6, r8, r6
 8008406:	fa96 f986 	rev.w	r9, r6
 800840a:	f851 6c04 	ldr.w	r6, [r1, #-4]
 800840e:	4056      	eors	r6, r2
 8008410:	ea19 0f06 	tst.w	r9, r6
 8008414:	d1e2      	bne.n	80083dc <smoltcp::iface::route::Routes::lookup+0xd2>
 8008416:	e7d9      	b.n	80083cc <smoltcp::iface::route::Routes::lookup+0xc2>
 8008418:	f04f 0900 	mov.w	r9, #0
 800841c:	f851 6c04 	ldr.w	r6, [r1, #-4]
 8008420:	4056      	eors	r6, r2
 8008422:	ea19 0f06 	tst.w	r9, r6
 8008426:	d1d9      	bne.n	80083dc <smoltcp::iface::route::Routes::lookup+0xd2>
 8008428:	e7d0      	b.n	80083cc <smoltcp::iface::route::Routes::lookup+0xc2>
 800842a:	2100      	movs	r1, #0
 800842c:	7001      	strb	r1, [r0, #0]
 800842e:	b001      	add	sp, #4
 8008430:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8008434:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8008436:	6a19      	ldr	r1, [r3, #32]
 8008438:	f8c0 1001 	str.w	r1, [r0, #1]
 800843c:	2101      	movs	r1, #1
 800843e:	7001      	strb	r1, [r0, #0]
 8008440:	b001      	add	sp, #4
 8008442:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8008446:	bdf0      	pop	{r4, r5, r6, r7, pc}

08008448 <smoltcp::iface::neighbor::Cache::fill>:
 8008448:	b5f0      	push	{r4, r5, r6, r7, lr}
 800844a:	af03      	add	r7, sp, #12
 800844c:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8008450:	b089      	sub	sp, #36	@ 0x24
 8008452:	f8d0 60c0 	ldr.w	r6, [r0, #192]	@ 0xc0
 8008456:	f248 7400 	movw	r4, #34560	@ 0x8700
 800845a:	f8d7 c008 	ldr.w	ip, [r7, #8]
 800845e:	f2c0 3493 	movt	r4, #915	@ 0x393
 8008462:	68fd      	ldr	r5, [r7, #12]
 8008464:	fa1f fe83 	uxth.w	lr, r3
 8008468:	eb06 0846 	add.w	r8, r6, r6, lsl #1
 800846c:	eb1c 0a04 	adds.w	sl, ip, r4
 8008470:	eba6 0386 	sub.w	r3, r6, r6, lsl #2
 8008474:	f145 0c00 	adc.w	ip, r5, #0
 8008478:	eb00 0bc8 	add.w	fp, r0, r8, lsl #3
 800847c:	9607      	str	r6, [sp, #28]
 800847e:	ea4f 09c3 	mov.w	r9, r3, lsl #3
 8008482:	2300      	movs	r3, #0
 8008484:	eb19 0403 	adds.w	r4, r9, r3
 8008488:	d01a      	beq.n	80084c0 <smoltcp::iface::neighbor::Cache::fill+0x78>
 800848a:	58c6      	ldr	r6, [r0, r3]
 800848c:	18c5      	adds	r5, r0, r3
 800848e:	428e      	cmp	r6, r1
 8008490:	f000 8082 	beq.w	8008598 <smoltcp::iface::neighbor::Cache::fill+0x150>
 8008494:	f114 0618 	adds.w	r6, r4, #24
 8008498:	d012      	beq.n	80084c0 <smoltcp::iface::neighbor::Cache::fill+0x78>
 800849a:	69ae      	ldr	r6, [r5, #24]
 800849c:	428e      	cmp	r6, r1
 800849e:	d078      	beq.n	8008592 <smoltcp::iface::neighbor::Cache::fill+0x14a>
 80084a0:	f114 0630 	adds.w	r6, r4, #48	@ 0x30
 80084a4:	d00c      	beq.n	80084c0 <smoltcp::iface::neighbor::Cache::fill+0x78>
 80084a6:	6b2e      	ldr	r6, [r5, #48]	@ 0x30
 80084a8:	428e      	cmp	r6, r1
 80084aa:	d074      	beq.n	8008596 <smoltcp::iface::neighbor::Cache::fill+0x14e>
 80084ac:	3448      	adds	r4, #72	@ 0x48
 80084ae:	d007      	beq.n	80084c0 <smoltcp::iface::neighbor::Cache::fill+0x78>
 80084b0:	6cac      	ldr	r4, [r5, #72]	@ 0x48
 80084b2:	3360      	adds	r3, #96	@ 0x60
 80084b4:	428c      	cmp	r4, r1
 80084b6:	d1e5      	bne.n	8008484 <smoltcp::iface::neighbor::Cache::fill+0x3c>
 80084b8:	4418      	add	r0, r3
 80084ba:	f1a0 0518 	sub.w	r5, r0, #24
 80084be:	e06b      	b.n	8008598 <smoltcp::iface::neighbor::Cache::fill+0x150>
 80084c0:	9b07      	ldr	r3, [sp, #28]
 80084c2:	2b07      	cmp	r3, #7
 80084c4:	d80a      	bhi.n	80084dc <smoltcp::iface::neighbor::Cache::fill+0x94>
 80084c6:	f8cb a008 	str.w	sl, [fp, #8]
 80084ca:	f8cb 1000 	str.w	r1, [fp]
 80084ce:	e9cb 2e04 	strd	r2, lr, [fp, #16]
 80084d2:	f8d0 10c0 	ldr.w	r1, [r0, #192]	@ 0xc0
 80084d6:	f8cb c00c 	str.w	ip, [fp, #12]
 80084da:	e146      	b.n	800876a <smoltcp::iface::neighbor::Cache::fill+0x322>
 80084dc:	f06f 032f 	mvn.w	r3, #47	@ 0x2f
 80084e0:	f64a 24ab 	movw	r4, #43691	@ 0xaaab
 80084e4:	eb03 03c8 	add.w	r3, r3, r8, lsl #3
 80084e8:	f6ca 24aa 	movt	r4, #43690	@ 0xaaaa
 80084ec:	f8cd b020 	str.w	fp, [sp, #32]
 80084f0:	fba3 4304 	umull	r4, r3, r3, r4
 80084f4:	9104      	str	r1, [sp, #16]
 80084f6:	e9d0 4502 	ldrd	r4, r5, [r0, #8]
 80084fa:	f8cd a008 	str.w	sl, [sp, #8]
 80084fe:	f8cd c00c 	str.w	ip, [sp, #12]
 8008502:	f8cd e004 	str.w	lr, [sp, #4]
 8008506:	e9cd 0205 	strd	r0, r2, [sp, #20]
 800850a:	ea6f 1613 	mvn.w	r6, r3, lsr #4
 800850e:	07b6      	lsls	r6, r6, #30
 8008510:	f100 0618 	add.w	r6, r0, #24
 8008514:	d105      	bne.n	8008522 <smoltcp::iface::neighbor::Cache::fill+0xda>
 8008516:	46b3      	mov	fp, r6
 8008518:	4603      	mov	r3, r0
 800851a:	4606      	mov	r6, r0
 800851c:	46a2      	mov	sl, r4
 800851e:	46a9      	mov	r9, r5
 8008520:	e058      	b.n	80085d4 <smoltcp::iface::neighbor::Cache::fill+0x18c>
 8008522:	e9d0 b808 	ldrd	fp, r8, [r0, #32]
 8008526:	ea4f 1a13 	mov.w	sl, r3, lsr #4
 800852a:	4601      	mov	r1, r0
 800852c:	4650      	mov	r0, sl
 800852e:	ebb4 030b 	subs.w	r3, r4, fp
 8008532:	46c1      	mov	r9, r8
 8008534:	eb75 0308 	sbcs.w	r3, r5, r8
 8008538:	46da      	mov	sl, fp
 800853a:	bfb8      	it	lt
 800853c:	46a9      	movlt	r9, r5
 800853e:	bfb8      	it	lt
 8008540:	46a2      	movlt	sl, r4
 8008542:	ebbb 0304 	subs.w	r3, fp, r4
 8008546:	f101 0b30 	add.w	fp, r1, #48	@ 0x30
 800854a:	eb78 0305 	sbcs.w	r3, r8, r5
 800854e:	4633      	mov	r3, r6
 8008550:	bfa8      	it	ge
 8008552:	460b      	movge	r3, r1
 8008554:	0784      	lsls	r4, r0, #30
 8008556:	d03d      	beq.n	80085d4 <smoltcp::iface::neighbor::Cache::fill+0x18c>
 8008558:	e9d1 460e 	ldrd	r4, r6, [r1, #56]	@ 0x38
 800855c:	f000 0003 	and.w	r0, r0, #3
 8008560:	ebba 0504 	subs.w	r5, sl, r4
 8008564:	eb79 0506 	sbcs.w	r5, r9, r6
 8008568:	46a0      	mov	r8, r4
 800856a:	4635      	mov	r5, r6
 800856c:	bfbc      	itt	lt
 800856e:	464d      	movlt	r5, r9
 8008570:	46d0      	movlt	r8, sl
 8008572:	ebb4 040a 	subs.w	r4, r4, sl
 8008576:	eb76 0409 	sbcs.w	r4, r6, r9
 800857a:	f101 0648 	add.w	r6, r1, #72	@ 0x48
 800857e:	bfb8      	it	lt
 8008580:	465b      	movlt	r3, fp
 8008582:	2801      	cmp	r0, #1
 8008584:	d113      	bne.n	80085ae <smoltcp::iface::neighbor::Cache::fill+0x166>
 8008586:	4658      	mov	r0, fp
 8008588:	46b3      	mov	fp, r6
 800858a:	46c2      	mov	sl, r8
 800858c:	46a9      	mov	r9, r5
 800858e:	4606      	mov	r6, r0
 8008590:	e020      	b.n	80085d4 <smoltcp::iface::neighbor::Cache::fill+0x18c>
 8008592:	3518      	adds	r5, #24
 8008594:	e000      	b.n	8008598 <smoltcp::iface::neighbor::Cache::fill+0x150>
 8008596:	3530      	adds	r5, #48	@ 0x30
 8008598:	f8c5 e014 	str.w	lr, [r5, #20]
 800859c:	612a      	str	r2, [r5, #16]
 800859e:	f8c5 c00c 	str.w	ip, [r5, #12]
 80085a2:	f8c5 a008 	str.w	sl, [r5, #8]
 80085a6:	b009      	add	sp, #36	@ 0x24
 80085a8:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 80085ac:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80085ae:	e9d1 2b14 	ldrd	r2, fp, [r1, #80]	@ 0x50
 80085b2:	46d9      	mov	r9, fp
 80085b4:	ebb8 0402 	subs.w	r4, r8, r2
 80085b8:	eb75 040b 	sbcs.w	r4, r5, fp
 80085bc:	4692      	mov	sl, r2
 80085be:	bfbc      	itt	lt
 80085c0:	46a9      	movlt	r9, r5
 80085c2:	46c2      	movlt	sl, r8
 80085c4:	ebb2 0208 	subs.w	r2, r2, r8
 80085c8:	eb7b 0205 	sbcs.w	r2, fp, r5
 80085cc:	f101 0b60 	add.w	fp, r1, #96	@ 0x60
 80085d0:	bfb8      	it	lt
 80085d2:	4633      	movlt	r3, r6
 80085d4:	f04f 0800 	mov.w	r8, #0
 80085d8:	e9d6 4508 	ldrd	r4, r5, [r6, #32]
 80085dc:	eb0b 0608 	add.w	r6, fp, r8
 80085e0:	f108 0860 	add.w	r8, r8, #96	@ 0x60
 80085e4:	ebb4 020a 	subs.w	r2, r4, sl
 80085e8:	eb75 0209 	sbcs.w	r2, r5, r9
 80085ec:	bfb8      	it	lt
 80085ee:	4633      	movlt	r3, r6
 80085f0:	ebba 0204 	subs.w	r2, sl, r4
 80085f4:	eb79 0205 	sbcs.w	r2, r9, r5
 80085f8:	bfbc      	itt	lt
 80085fa:	464d      	movlt	r5, r9
 80085fc:	4654      	movlt	r4, sl
 80085fe:	e9d6 9208 	ldrd	r9, r2, [r6, #32]
 8008602:	e9d6 ac0e 	ldrd	sl, ip, [r6, #56]	@ 0x38
 8008606:	ebb9 0e04 	subs.w	lr, r9, r4
 800860a:	eb72 0005 	sbcs.w	r0, r2, r5
 800860e:	bfb8      	it	lt
 8008610:	f106 0318 	addlt.w	r3, r6, #24
 8008614:	ebb4 0009 	subs.w	r0, r4, r9
 8008618:	eb75 0002 	sbcs.w	r0, r5, r2
 800861c:	bfa4      	itt	ge
 800861e:	4615      	movge	r5, r2
 8008620:	464c      	movge	r4, r9
 8008622:	ebba 0004 	subs.w	r0, sl, r4
 8008626:	eb7c 0005 	sbcs.w	r0, ip, r5
 800862a:	bfb8      	it	lt
 800862c:	f106 0330 	addlt.w	r3, r6, #48	@ 0x30
 8008630:	ebb4 000a 	subs.w	r0, r4, sl
 8008634:	eb75 000c 	sbcs.w	r0, r5, ip
 8008638:	bfa8      	it	ge
 800863a:	4665      	movge	r5, ip
 800863c:	e9d6 2014 	ldrd	r2, r0, [r6, #80]	@ 0x50
 8008640:	bfa8      	it	ge
 8008642:	4654      	movge	r4, sl
 8008644:	3648      	adds	r6, #72	@ 0x48
 8008646:	1b11      	subs	r1, r2, r4
 8008648:	eb70 0105 	sbcs.w	r1, r0, r5
 800864c:	bfb8      	it	lt
 800864e:	4633      	movlt	r3, r6
 8008650:	1aa1      	subs	r1, r4, r2
 8008652:	eb75 0100 	sbcs.w	r1, r5, r0
 8008656:	bfa4      	itt	ge
 8008658:	4605      	movge	r5, r0
 800865a:	4614      	movge	r4, r2
 800865c:	9908      	ldr	r1, [sp, #32]
 800865e:	eb0b 0008 	add.w	r0, fp, r8
 8008662:	46a2      	mov	sl, r4
 8008664:	46a9      	mov	r9, r5
 8008666:	4288      	cmp	r0, r1
 8008668:	d1b6      	bne.n	80085d8 <smoltcp::iface::neighbor::Cache::fill+0x190>
 800866a:	9905      	ldr	r1, [sp, #20]
 800866c:	681c      	ldr	r4, [r3, #0]
 800866e:	2300      	movs	r3, #0
 8008670:	f8dd 8018 	ldr.w	r8, [sp, #24]
 8008674:	460d      	mov	r5, r1
 8008676:	9a04      	ldr	r2, [sp, #16]
 8008678:	9e08      	ldr	r6, [sp, #32]
 800867a:	6828      	ldr	r0, [r5, #0]
 800867c:	42a0      	cmp	r0, r4
 800867e:	d023      	beq.n	80086c8 <smoltcp::iface::neighbor::Cache::fill+0x280>
 8008680:	3518      	adds	r5, #24
 8008682:	42b5      	cmp	r5, r6
 8008684:	d012      	beq.n	80086ac <smoltcp::iface::neighbor::Cache::fill+0x264>
 8008686:	6828      	ldr	r0, [r5, #0]
 8008688:	42a0      	cmp	r0, r4
 800868a:	d015      	beq.n	80086b8 <smoltcp::iface::neighbor::Cache::fill+0x270>
 800868c:	3518      	adds	r5, #24
 800868e:	42b5      	cmp	r5, r6
 8008690:	d00c      	beq.n	80086ac <smoltcp::iface::neighbor::Cache::fill+0x264>
 8008692:	6828      	ldr	r0, [r5, #0]
 8008694:	42a0      	cmp	r0, r4
 8008696:	d012      	beq.n	80086be <smoltcp::iface::neighbor::Cache::fill+0x276>
 8008698:	3518      	adds	r5, #24
 800869a:	42b5      	cmp	r5, r6
 800869c:	d006      	beq.n	80086ac <smoltcp::iface::neighbor::Cache::fill+0x264>
 800869e:	6828      	ldr	r0, [r5, #0]
 80086a0:	42a0      	cmp	r0, r4
 80086a2:	d00f      	beq.n	80086c4 <smoltcp::iface::neighbor::Cache::fill+0x27c>
 80086a4:	3518      	adds	r5, #24
 80086a6:	3304      	adds	r3, #4
 80086a8:	42b5      	cmp	r5, r6
 80086aa:	d1e6      	bne.n	800867a <smoltcp::iface::neighbor::Cache::fill+0x232>
 80086ac:	f64a 60e4 	movw	r0, #44772	@ 0xaee4
 80086b0:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80086b4:	f7fb f80b 	bl	80036ce <core::option::unwrap_failed>
 80086b8:	f043 0301 	orr.w	r3, r3, #1
 80086bc:	e004      	b.n	80086c8 <smoltcp::iface::neighbor::Cache::fill+0x280>
 80086be:	f043 0302 	orr.w	r3, r3, #2
 80086c2:	e001      	b.n	80086c8 <smoltcp::iface::neighbor::Cache::fill+0x280>
 80086c4:	f043 0303 	orr.w	r3, r3, #3
 80086c8:	9807      	ldr	r0, [sp, #28]
 80086ca:	4283      	cmp	r3, r0
 80086cc:	d254      	bcs.n	8008778 <smoltcp::iface::neighbor::Cache::fill+0x330>
 80086ce:	eb03 0043 	add.w	r0, r3, r3, lsl #1
 80086d2:	f1a6 0318 	sub.w	r3, r6, #24
 80086d6:	4615      	mov	r5, r2
 80086d8:	460c      	mov	r4, r1
 80086da:	eb01 00c0 	add.w	r0, r1, r0, lsl #3
 80086de:	4619      	mov	r1, r3
 80086e0:	2218      	movs	r2, #24
 80086e2:	f000 fda9 	bl	8009238 <__aeabi_memmove8>
 80086e6:	f8d4 20c0 	ldr.w	r2, [r4, #192]	@ 0xc0
 80086ea:	4629      	mov	r1, r5
 80086ec:	4620      	mov	r0, r4
 80086ee:	46c4      	mov	ip, r8
 80086f0:	f1a2 0e01 	sub.w	lr, r2, #1
 80086f4:	f8c4 e0c0 	str.w	lr, [r4, #192]	@ 0xc0
 80086f8:	eba2 0282 	sub.w	r2, r2, r2, lsl #2
 80086fc:	eb0e 034e 	add.w	r3, lr, lr, lsl #1
 8008700:	00d2      	lsls	r2, r2, #3
 8008702:	eb04 09c3 	add.w	r9, r4, r3, lsl #3
 8008706:	2400      	movs	r4, #0
 8008708:	1916      	adds	r6, r2, r4
 800870a:	f116 0518 	adds.w	r5, r6, #24
 800870e:	d01a      	beq.n	8008746 <smoltcp::iface::neighbor::Cache::fill+0x2fe>
 8008710:	5903      	ldr	r3, [r0, r4]
 8008712:	1905      	adds	r5, r0, r4
 8008714:	428b      	cmp	r3, r1
 8008716:	d03d      	beq.n	8008794 <smoltcp::iface::neighbor::Cache::fill+0x34c>
 8008718:	f116 0330 	adds.w	r3, r6, #48	@ 0x30
 800871c:	d013      	beq.n	8008746 <smoltcp::iface::neighbor::Cache::fill+0x2fe>
 800871e:	69ab      	ldr	r3, [r5, #24]
 8008720:	428b      	cmp	r3, r1
 8008722:	d034      	beq.n	800878e <smoltcp::iface::neighbor::Cache::fill+0x346>
 8008724:	f116 0348 	adds.w	r3, r6, #72	@ 0x48
 8008728:	d00d      	beq.n	8008746 <smoltcp::iface::neighbor::Cache::fill+0x2fe>
 800872a:	6b2b      	ldr	r3, [r5, #48]	@ 0x30
 800872c:	428b      	cmp	r3, r1
 800872e:	d030      	beq.n	8008792 <smoltcp::iface::neighbor::Cache::fill+0x34a>
 8008730:	f116 0360 	adds.w	r3, r6, #96	@ 0x60
 8008734:	d007      	beq.n	8008746 <smoltcp::iface::neighbor::Cache::fill+0x2fe>
 8008736:	6cab      	ldr	r3, [r5, #72]	@ 0x48
 8008738:	3460      	adds	r4, #96	@ 0x60
 800873a:	428b      	cmp	r3, r1
 800873c:	d1e4      	bne.n	8008708 <smoltcp::iface::neighbor::Cache::fill+0x2c0>
 800873e:	4420      	add	r0, r4
 8008740:	f1a0 0518 	sub.w	r5, r0, #24
 8008744:	e026      	b.n	8008794 <smoltcp::iface::neighbor::Cache::fill+0x34c>
 8008746:	f1be 0f07 	cmp.w	lr, #7
 800874a:	d82b      	bhi.n	80087a4 <smoltcp::iface::neighbor::Cache::fill+0x35c>
 800874c:	9a02      	ldr	r2, [sp, #8]
 800874e:	f8c9 2008 	str.w	r2, [r9, #8]
 8008752:	f8c9 1000 	str.w	r1, [r9]
 8008756:	9901      	ldr	r1, [sp, #4]
 8008758:	f8c9 c010 	str.w	ip, [r9, #16]
 800875c:	9a03      	ldr	r2, [sp, #12]
 800875e:	f8c9 1014 	str.w	r1, [r9, #20]
 8008762:	f8d0 10c0 	ldr.w	r1, [r0, #192]	@ 0xc0
 8008766:	f8c9 200c 	str.w	r2, [r9, #12]
 800876a:	3101      	adds	r1, #1
 800876c:	f8c0 10c0 	str.w	r1, [r0, #192]	@ 0xc0
 8008770:	b009      	add	sp, #36	@ 0x24
 8008772:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8008776:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8008778:	f64a 600c 	movw	r0, #44556	@ 0xae0c
 800877c:	f64a 6230 	movw	r2, #44592	@ 0xae30
 8008780:	f6c0 0000 	movt	r0, #2048	@ 0x800
 8008784:	f6c0 0200 	movt	r2, #2048	@ 0x800
 8008788:	2122      	movs	r1, #34	@ 0x22
 800878a:	f7fb f83d 	bl	8003808 <core::panicking::panic>
 800878e:	3518      	adds	r5, #24
 8008790:	e000      	b.n	8008794 <smoltcp::iface::neighbor::Cache::fill+0x34c>
 8008792:	3530      	adds	r5, #48	@ 0x30
 8008794:	9801      	ldr	r0, [sp, #4]
 8008796:	6168      	str	r0, [r5, #20]
 8008798:	9803      	ldr	r0, [sp, #12]
 800879a:	60e8      	str	r0, [r5, #12]
 800879c:	9802      	ldr	r0, [sp, #8]
 800879e:	f8c5 8010 	str.w	r8, [r5, #16]
 80087a2:	60a8      	str	r0, [r5, #8]
 80087a4:	f24b 0098 	movw	r0, #45208	@ 0xb098
 80087a8:	f64a 62f4 	movw	r2, #44788	@ 0xaef4
 80087ac:	f6c0 0000 	movt	r0, #2048	@ 0x800
 80087b0:	f6c0 0200 	movt	r2, #2048	@ 0x800
 80087b4:	2128      	movs	r1, #40	@ 0x28
 80087b6:	f7fb f827 	bl	8003808 <core::panicking::panic>

080087ba <stm32_eth::dma::rx::RxRing::demand_poll>:
 80087ba:	b580      	push	{r7, lr}
 80087bc:	466f      	mov	r7, sp
 80087be:	f249 0008 	movw	r0, #36872	@ 0x9008
 80087c2:	2101      	movs	r1, #1
 80087c4:	f2c4 0002 	movt	r0, #16386	@ 0x4002
 80087c8:	6001      	str	r1, [r0, #0]
 80087ca:	bd80      	pop	{r7, pc}

080087cc <__cpsid>:
 80087cc:	b672      	cpsid	i
 80087ce:	4770      	bx	lr

080087d0 <__cpsie>:
 80087d0:	b662      	cpsie	i
 80087d2:	4770      	bx	lr

080087d4 <__delay>:
 80087d4:	2101      	movs	r1, #1
 80087d6:	eb01 0050 	add.w	r0, r1, r0, lsr #1
 80087da:	3801      	subs	r0, #1
 80087dc:	d1fd      	bne.n	80087da <__delay+0x6>
 80087de:	4770      	bx	lr

080087e0 <__dsb>:
 80087e0:	f3bf 8f4f 	dsb	sy
 80087e4:	4770      	bx	lr

080087e6 <__primask_r>:
 80087e6:	f3ef 8010 	mrs	r0, PRIMASK
 80087ea:	4770      	bx	lr

080087ec <__udf>:
 80087ec:	de00      	udf	#0
 80087ee:	defe      	udf	#254	@ 0xfe

080087f0 <__aeabi_memclr8>:
 80087f0:	b580      	push	{r7, lr}
 80087f2:	466f      	mov	r7, sp
 80087f4:	2904      	cmp	r1, #4
 80087f6:	d305      	bcc.n	8008804 <__aeabi_memclr8+0x14>
 80087f8:	1f0b      	subs	r3, r1, #4
 80087fa:	43da      	mvns	r2, r3
 80087fc:	f012 0f0c 	tst.w	r2, #12
 8008800:	d102      	bne.n	8008808 <__aeabi_memclr8+0x18>
 8008802:	e01a      	b.n	800883a <__aeabi_memclr8+0x4a>
 8008804:	4602      	mov	r2, r0
 8008806:	e024      	b.n	8008852 <__aeabi_memclr8+0x62>
 8008808:	f04f 0c00 	mov.w	ip, #0
 800880c:	4602      	mov	r2, r0
 800880e:	f842 cb04 	str.w	ip, [r2], #4
 8008812:	f013 0f0c 	tst.w	r3, #12
 8008816:	d008      	beq.n	800882a <__aeabi_memclr8+0x3a>
 8008818:	f003 020c 	and.w	r2, r3, #12
 800881c:	f8c0 c004 	str.w	ip, [r0, #4]
 8008820:	2a04      	cmp	r2, #4
 8008822:	d105      	bne.n	8008830 <__aeabi_memclr8+0x40>
 8008824:	3908      	subs	r1, #8
 8008826:	3008      	adds	r0, #8
 8008828:	e006      	b.n	8008838 <__aeabi_memclr8+0x48>
 800882a:	4619      	mov	r1, r3
 800882c:	4610      	mov	r0, r2
 800882e:	e004      	b.n	800883a <__aeabi_memclr8+0x4a>
 8008830:	2200      	movs	r2, #0
 8008832:	390c      	subs	r1, #12
 8008834:	6082      	str	r2, [r0, #8]
 8008836:	300c      	adds	r0, #12
 8008838:	4602      	mov	r2, r0
 800883a:	2b0c      	cmp	r3, #12
 800883c:	d309      	bcc.n	8008852 <__aeabi_memclr8+0x62>
 800883e:	2300      	movs	r3, #0
 8008840:	4602      	mov	r2, r0
 8008842:	3910      	subs	r1, #16
 8008844:	e9c2 3300 	strd	r3, r3, [r2]
 8008848:	e9c2 3302 	strd	r3, r3, [r2, #8]
 800884c:	3210      	adds	r2, #16
 800884e:	2903      	cmp	r1, #3
 8008850:	d8f7      	bhi.n	8008842 <__aeabi_memclr8+0x52>
 8008852:	1850      	adds	r0, r2, r1
 8008854:	4282      	cmp	r2, r0
 8008856:	d221      	bcs.n	800889c <__aeabi_memclr8+0xac>
 8008858:	f1a1 0c01 	sub.w	ip, r1, #1
 800885c:	f011 0303 	ands.w	r3, r1, #3
 8008860:	d00c      	beq.n	800887c <__aeabi_memclr8+0x8c>
 8008862:	f04f 0e00 	mov.w	lr, #0
 8008866:	4611      	mov	r1, r2
 8008868:	f801 eb01 	strb.w	lr, [r1], #1
 800886c:	2b01      	cmp	r3, #1
 800886e:	d00a      	beq.n	8008886 <__aeabi_memclr8+0x96>
 8008870:	2b02      	cmp	r3, #2
 8008872:	f882 e001 	strb.w	lr, [r2, #1]
 8008876:	d103      	bne.n	8008880 <__aeabi_memclr8+0x90>
 8008878:	1c91      	adds	r1, r2, #2
 800887a:	e004      	b.n	8008886 <__aeabi_memclr8+0x96>
 800887c:	4611      	mov	r1, r2
 800887e:	e002      	b.n	8008886 <__aeabi_memclr8+0x96>
 8008880:	2100      	movs	r1, #0
 8008882:	7091      	strb	r1, [r2, #2]
 8008884:	1cd1      	adds	r1, r2, #3
 8008886:	f1bc 0f03 	cmp.w	ip, #3
 800888a:	bf38      	it	cc
 800888c:	bd80      	popcc	{r7, pc}
 800888e:	3904      	subs	r1, #4
 8008890:	2200      	movs	r2, #0
 8008892:	f841 2f04 	str.w	r2, [r1, #4]!
 8008896:	1d0b      	adds	r3, r1, #4
 8008898:	4283      	cmp	r3, r0
 800889a:	d1fa      	bne.n	8008892 <__aeabi_memclr8+0xa2>
 800889c:	bd80      	pop	{r7, pc}

0800889e <__aeabi_memcpy4>:
 800889e:	2a04      	cmp	r2, #4
 80088a0:	d309      	bcc.n	80088b6 <__aeabi_memcpy4+0x18>
 80088a2:	b5d0      	push	{r4, r6, r7, lr}
 80088a4:	af02      	add	r7, sp, #8
 80088a6:	f1a2 0e04 	sub.w	lr, r2, #4
 80088aa:	ea6f 030e 	mvn.w	r3, lr
 80088ae:	f013 0f0c 	tst.w	r3, #12
 80088b2:	d106      	bne.n	80088c2 <__aeabi_memcpy4+0x24>
 80088b4:	e023      	b.n	80088fe <__aeabi_memcpy4+0x60>
 80088b6:	460b      	mov	r3, r1
 80088b8:	4684      	mov	ip, r0
 80088ba:	4660      	mov	r0, ip
 80088bc:	4619      	mov	r1, r3
 80088be:	f000 b83c 	b.w	800893a <compiler_builtins::mem::memcpy>
 80088c2:	460b      	mov	r3, r1
 80088c4:	4684      	mov	ip, r0
 80088c6:	f853 4b04 	ldr.w	r4, [r3], #4
 80088ca:	f01e 0f0c 	tst.w	lr, #12
 80088ce:	f84c 4b04 	str.w	r4, [ip], #4
 80088d2:	d009      	beq.n	80088e8 <__aeabi_memcpy4+0x4a>
 80088d4:	684b      	ldr	r3, [r1, #4]
 80088d6:	6043      	str	r3, [r0, #4]
 80088d8:	f00e 030c 	and.w	r3, lr, #12
 80088dc:	2b04      	cmp	r3, #4
 80088de:	d107      	bne.n	80088f0 <__aeabi_memcpy4+0x52>
 80088e0:	3a08      	subs	r2, #8
 80088e2:	3108      	adds	r1, #8
 80088e4:	3008      	adds	r0, #8
 80088e6:	e008      	b.n	80088fa <__aeabi_memcpy4+0x5c>
 80088e8:	4672      	mov	r2, lr
 80088ea:	4660      	mov	r0, ip
 80088ec:	4619      	mov	r1, r3
 80088ee:	e006      	b.n	80088fe <__aeabi_memcpy4+0x60>
 80088f0:	688b      	ldr	r3, [r1, #8]
 80088f2:	3a0c      	subs	r2, #12
 80088f4:	6083      	str	r3, [r0, #8]
 80088f6:	310c      	adds	r1, #12
 80088f8:	300c      	adds	r0, #12
 80088fa:	4684      	mov	ip, r0
 80088fc:	460b      	mov	r3, r1
 80088fe:	f1be 0f0c 	cmp.w	lr, #12
 8008902:	d314      	bcc.n	800892e <__aeabi_memcpy4+0x90>
 8008904:	4684      	mov	ip, r0
 8008906:	460b      	mov	r3, r1
 8008908:	6818      	ldr	r0, [r3, #0]
 800890a:	3a10      	subs	r2, #16
 800890c:	f8cc 0000 	str.w	r0, [ip]
 8008910:	2a03      	cmp	r2, #3
 8008912:	6858      	ldr	r0, [r3, #4]
 8008914:	f8cc 0004 	str.w	r0, [ip, #4]
 8008918:	6898      	ldr	r0, [r3, #8]
 800891a:	f8cc 0008 	str.w	r0, [ip, #8]
 800891e:	68d8      	ldr	r0, [r3, #12]
 8008920:	f103 0310 	add.w	r3, r3, #16
 8008924:	f8cc 000c 	str.w	r0, [ip, #12]
 8008928:	f10c 0c10 	add.w	ip, ip, #16
 800892c:	d8ec      	bhi.n	8008908 <__aeabi_memcpy4+0x6a>
 800892e:	e8bd 40d0 	ldmia.w	sp!, {r4, r6, r7, lr}
 8008932:	4660      	mov	r0, ip
 8008934:	4619      	mov	r1, r3
 8008936:	f000 b800 	b.w	800893a <compiler_builtins::mem::memcpy>

0800893a <compiler_builtins::mem::memcpy>:
 800893a:	b5f0      	push	{r4, r5, r6, r7, lr}
 800893c:	af03      	add	r7, sp, #12
 800893e:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8008942:	b08c      	sub	sp, #48	@ 0x30
 8008944:	2a10      	cmp	r2, #16
 8008946:	d31e      	bcc.n	8008986 <compiler_builtins::mem::memcpy+0x4c>
 8008948:	4243      	negs	r3, r0
 800894a:	f003 0803 	and.w	r8, r3, #3
 800894e:	eb00 0308 	add.w	r3, r0, r8
 8008952:	4298      	cmp	r0, r3
 8008954:	d233      	bcs.n	80089be <compiler_builtins::mem::memcpy+0x84>
 8008956:	f1a8 0c01 	sub.w	ip, r8, #1
 800895a:	4606      	mov	r6, r0
 800895c:	460c      	mov	r4, r1
 800895e:	f1b8 0f00 	cmp.w	r8, #0
 8008962:	d01a      	beq.n	800899a <compiler_builtins::mem::memcpy+0x60>
 8008964:	460c      	mov	r4, r1
 8008966:	4606      	mov	r6, r0
 8008968:	f814 eb01 	ldrb.w	lr, [r4], #1
 800896c:	f1b8 0f01 	cmp.w	r8, #1
 8008970:	f806 eb01 	strb.w	lr, [r6], #1
 8008974:	d011      	beq.n	800899a <compiler_builtins::mem::memcpy+0x60>
 8008976:	784e      	ldrb	r6, [r1, #1]
 8008978:	f1b8 0f02 	cmp.w	r8, #2
 800897c:	7046      	strb	r6, [r0, #1]
 800897e:	d108      	bne.n	8008992 <compiler_builtins::mem::memcpy+0x58>
 8008980:	1c8c      	adds	r4, r1, #2
 8008982:	1c86      	adds	r6, r0, #2
 8008984:	e009      	b.n	800899a <compiler_builtins::mem::memcpy+0x60>
 8008986:	4684      	mov	ip, r0
 8008988:	eb0c 0302 	add.w	r3, ip, r2
 800898c:	459c      	cmp	ip, r3
 800898e:	d340      	bcc.n	8008a12 <compiler_builtins::mem::memcpy+0xd8>
 8008990:	e06c      	b.n	8008a6c <compiler_builtins::mem::memcpy+0x132>
 8008992:	788e      	ldrb	r6, [r1, #2]
 8008994:	1ccc      	adds	r4, r1, #3
 8008996:	7086      	strb	r6, [r0, #2]
 8008998:	1cc6      	adds	r6, r0, #3
 800899a:	f1bc 0f03 	cmp.w	ip, #3
 800899e:	d30e      	bcc.n	80089be <compiler_builtins::mem::memcpy+0x84>
 80089a0:	3c04      	subs	r4, #4
 80089a2:	3e04      	subs	r6, #4
 80089a4:	f814 5f04 	ldrb.w	r5, [r4, #4]!
 80089a8:	f806 5f04 	strb.w	r5, [r6, #4]!
 80089ac:	7865      	ldrb	r5, [r4, #1]
 80089ae:	7075      	strb	r5, [r6, #1]
 80089b0:	78a5      	ldrb	r5, [r4, #2]
 80089b2:	70b5      	strb	r5, [r6, #2]
 80089b4:	78e5      	ldrb	r5, [r4, #3]
 80089b6:	70f5      	strb	r5, [r6, #3]
 80089b8:	1d35      	adds	r5, r6, #4
 80089ba:	429d      	cmp	r5, r3
 80089bc:	d1f2      	bne.n	80089a4 <compiler_builtins::mem::memcpy+0x6a>
 80089be:	eba2 0e08 	sub.w	lr, r2, r8
 80089c2:	eb01 0408 	add.w	r4, r1, r8
 80089c6:	f02e 0203 	bic.w	r2, lr, #3
 80089ca:	f014 0603 	ands.w	r6, r4, #3
 80089ce:	eb03 0c02 	add.w	ip, r3, r2
 80089d2:	d159      	bne.n	8008a88 <compiler_builtins::mem::memcpy+0x14e>
 80089d4:	4563      	cmp	r3, ip
 80089d6:	d215      	bcs.n	8008a04 <compiler_builtins::mem::memcpy+0xca>
 80089d8:	4621      	mov	r1, r4
 80089da:	680d      	ldr	r5, [r1, #0]
 80089dc:	f843 5b04 	str.w	r5, [r3], #4
 80089e0:	4563      	cmp	r3, ip
 80089e2:	d20f      	bcs.n	8008a04 <compiler_builtins::mem::memcpy+0xca>
 80089e4:	684d      	ldr	r5, [r1, #4]
 80089e6:	f843 5b04 	str.w	r5, [r3], #4
 80089ea:	4563      	cmp	r3, ip
 80089ec:	bf3e      	ittt	cc
 80089ee:	688d      	ldrcc	r5, [r1, #8]
 80089f0:	f843 5b04 	strcc.w	r5, [r3], #4
 80089f4:	4563      	cmpcc	r3, ip
 80089f6:	d205      	bcs.n	8008a04 <compiler_builtins::mem::memcpy+0xca>
 80089f8:	68cd      	ldr	r5, [r1, #12]
 80089fa:	3110      	adds	r1, #16
 80089fc:	f843 5b04 	str.w	r5, [r3], #4
 8008a00:	4563      	cmp	r3, ip
 8008a02:	d3ea      	bcc.n	80089da <compiler_builtins::mem::memcpy+0xa0>
 8008a04:	18a1      	adds	r1, r4, r2
 8008a06:	f00e 0203 	and.w	r2, lr, #3
 8008a0a:	eb0c 0302 	add.w	r3, ip, r2
 8008a0e:	459c      	cmp	ip, r3
 8008a10:	d22c      	bcs.n	8008a6c <compiler_builtins::mem::memcpy+0x132>
 8008a12:	f1a2 0e01 	sub.w	lr, r2, #1
 8008a16:	f012 0403 	ands.w	r4, r2, #3
 8008a1a:	d013      	beq.n	8008a44 <compiler_builtins::mem::memcpy+0x10a>
 8008a1c:	460a      	mov	r2, r1
 8008a1e:	4665      	mov	r5, ip
 8008a20:	f812 6b01 	ldrb.w	r6, [r2], #1
 8008a24:	2c01      	cmp	r4, #1
 8008a26:	f805 6b01 	strb.w	r6, [r5], #1
 8008a2a:	d00d      	beq.n	8008a48 <compiler_builtins::mem::memcpy+0x10e>
 8008a2c:	784a      	ldrb	r2, [r1, #1]
 8008a2e:	2c02      	cmp	r4, #2
 8008a30:	f88c 2001 	strb.w	r2, [ip, #1]
 8008a34:	d11e      	bne.n	8008a74 <compiler_builtins::mem::memcpy+0x13a>
 8008a36:	1c8a      	adds	r2, r1, #2
 8008a38:	f10c 0502 	add.w	r5, ip, #2
 8008a3c:	f1be 0f03 	cmp.w	lr, #3
 8008a40:	d205      	bcs.n	8008a4e <compiler_builtins::mem::memcpy+0x114>
 8008a42:	e013      	b.n	8008a6c <compiler_builtins::mem::memcpy+0x132>
 8008a44:	4665      	mov	r5, ip
 8008a46:	460a      	mov	r2, r1
 8008a48:	f1be 0f03 	cmp.w	lr, #3
 8008a4c:	d30e      	bcc.n	8008a6c <compiler_builtins::mem::memcpy+0x132>
 8008a4e:	1f11      	subs	r1, r2, #4
 8008a50:	1f2a      	subs	r2, r5, #4
 8008a52:	f811 6f04 	ldrb.w	r6, [r1, #4]!
 8008a56:	f802 6f04 	strb.w	r6, [r2, #4]!
 8008a5a:	784e      	ldrb	r6, [r1, #1]
 8008a5c:	7056      	strb	r6, [r2, #1]
 8008a5e:	788e      	ldrb	r6, [r1, #2]
 8008a60:	7096      	strb	r6, [r2, #2]
 8008a62:	78ce      	ldrb	r6, [r1, #3]
 8008a64:	70d6      	strb	r6, [r2, #3]
 8008a66:	1d16      	adds	r6, r2, #4
 8008a68:	429e      	cmp	r6, r3
 8008a6a:	d1f2      	bne.n	8008a52 <compiler_builtins::mem::memcpy+0x118>
 8008a6c:	b00c      	add	sp, #48	@ 0x30
 8008a6e:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8008a72:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8008a74:	788a      	ldrb	r2, [r1, #2]
 8008a76:	f10c 0503 	add.w	r5, ip, #3
 8008a7a:	f88c 2002 	strb.w	r2, [ip, #2]
 8008a7e:	1cca      	adds	r2, r1, #3
 8008a80:	f1be 0f03 	cmp.w	lr, #3
 8008a84:	d2e3      	bcs.n	8008a4e <compiler_builtins::mem::memcpy+0x114>
 8008a86:	e7f1      	b.n	8008a6c <compiler_builtins::mem::memcpy+0x132>
 8008a88:	f04f 0900 	mov.w	r9, #0
 8008a8c:	f1c6 0a04 	rsb	sl, r6, #4
 8008a90:	ad0b      	add	r5, sp, #44	@ 0x2c
 8008a92:	f8cd 902c 	str.w	r9, [sp, #44]	@ 0x2c
 8008a96:	eb05 0b06 	add.w	fp, r5, r6
 8008a9a:	ea5f 75ca 	movs.w	r5, sl, lsl #31
 8008a9e:	bf1e      	ittt	ne
 8008aa0:	7825      	ldrbne	r5, [r4, #0]
 8008aa2:	f88b 5000 	strbne.w	r5, [fp]
 8008aa6:	f04f 0901 	movne.w	r9, #1
 8008aaa:	1ba5      	subs	r5, r4, r6
 8008aac:	9507      	str	r5, [sp, #28]
 8008aae:	00f5      	lsls	r5, r6, #3
 8008ab0:	9501      	str	r5, [sp, #4]
 8008ab2:	ea5f 758a 	movs.w	r5, sl, lsl #30
 8008ab6:	bf44      	itt	mi
 8008ab8:	f834 5009 	ldrhmi.w	r5, [r4, r9]
 8008abc:	f82b 5009 	strhmi.w	r5, [fp, r9]
 8008ac0:	1d1d      	adds	r5, r3, #4
 8008ac2:	f8dd b02c 	ldr.w	fp, [sp, #44]	@ 0x2c
 8008ac6:	4565      	cmp	r5, ip
 8008ac8:	9d01      	ldr	r5, [sp, #4]
 8008aca:	46aa      	mov	sl, r5
 8008acc:	f1c5 0500 	rsb	r5, r5, #0
 8008ad0:	d25c      	bcs.n	8008b8c <compiler_builtins::mem::memcpy+0x252>
 8008ad2:	4273      	negs	r3, r6
 8008ad4:	9500      	str	r5, [sp, #0]
 8008ad6:	eb01 0903 	add.w	r9, r1, r3
 8008ada:	f005 0118 	and.w	r1, r5, #24
 8008ade:	4605      	mov	r5, r0
 8008ae0:	9107      	str	r1, [sp, #28]
 8008ae2:	f8cd 9010 	str.w	r9, [sp, #16]
 8008ae6:	44c1      	add	r9, r8
 8008ae8:	9b07      	ldr	r3, [sp, #28]
 8008aea:	fa2b fb0a 	lsr.w	fp, fp, sl
 8008aee:	f8d9 1004 	ldr.w	r1, [r9, #4]
 8008af2:	9108      	str	r1, [sp, #32]
 8008af4:	fa01 f303 	lsl.w	r3, r1, r3
 8008af8:	eb05 0108 	add.w	r1, r5, r8
 8008afc:	ea43 030b 	orr.w	r3, r3, fp
 8008b00:	468b      	mov	fp, r1
 8008b02:	f84b 3b08 	str.w	r3, [fp], #8
 8008b06:	45e3      	cmp	fp, ip
 8008b08:	d242      	bcs.n	8008b90 <compiler_builtins::mem::memcpy+0x256>
 8008b0a:	9b08      	ldr	r3, [sp, #32]
 8008b0c:	9503      	str	r5, [sp, #12]
 8008b0e:	f8cd 9018 	str.w	r9, [sp, #24]
 8008b12:	fa23 f50a 	lsr.w	r5, r3, sl
 8008b16:	f8d9 9008 	ldr.w	r9, [r9, #8]
 8008b1a:	9b07      	ldr	r3, [sp, #28]
 8008b1c:	9105      	str	r1, [sp, #20]
 8008b1e:	fa09 f303 	lsl.w	r3, r9, r3
 8008b22:	432b      	orrs	r3, r5
 8008b24:	604b      	str	r3, [r1, #4]
 8008b26:	f101 030c 	add.w	r3, r1, #12
 8008b2a:	4563      	cmp	r3, ip
 8008b2c:	d236      	bcs.n	8008b9c <compiler_builtins::mem::memcpy+0x262>
 8008b2e:	9d06      	ldr	r5, [sp, #24]
 8008b30:	68ed      	ldr	r5, [r5, #12]
 8008b32:	9508      	str	r5, [sp, #32]
 8008b34:	fa29 f50a 	lsr.w	r5, r9, sl
 8008b38:	9502      	str	r5, [sp, #8]
 8008b3a:	e9dd 5107 	ldrd	r5, r1, [sp, #28]
 8008b3e:	fa01 f905 	lsl.w	r9, r1, r5
 8008b42:	9905      	ldr	r1, [sp, #20]
 8008b44:	9d02      	ldr	r5, [sp, #8]
 8008b46:	3110      	adds	r1, #16
 8008b48:	ea45 0509 	orr.w	r5, r5, r9
 8008b4c:	4561      	cmp	r1, ip
 8008b4e:	f8cb 5000 	str.w	r5, [fp]
 8008b52:	d22d      	bcs.n	8008bb0 <compiler_builtins::mem::memcpy+0x276>
 8008b54:	9906      	ldr	r1, [sp, #24]
 8008b56:	9d07      	ldr	r5, [sp, #28]
 8008b58:	f8dd 9010 	ldr.w	r9, [sp, #16]
 8008b5c:	f8d1 b010 	ldr.w	fp, [r1, #16]
 8008b60:	9908      	ldr	r1, [sp, #32]
 8008b62:	f109 0910 	add.w	r9, r9, #16
 8008b66:	fa0b f505 	lsl.w	r5, fp, r5
 8008b6a:	fa21 f10a 	lsr.w	r1, r1, sl
 8008b6e:	4329      	orrs	r1, r5
 8008b70:	9d03      	ldr	r5, [sp, #12]
 8008b72:	6019      	str	r1, [r3, #0]
 8008b74:	3510      	adds	r5, #16
 8008b76:	eb05 0308 	add.w	r3, r5, r8
 8008b7a:	1d19      	adds	r1, r3, #4
 8008b7c:	4561      	cmp	r1, ip
 8008b7e:	d3b0      	bcc.n	8008ae2 <compiler_builtins::mem::memcpy+0x1a8>
 8008b80:	eb09 0108 	add.w	r1, r9, r8
 8008b84:	9107      	str	r1, [sp, #28]
 8008b86:	f8dd a000 	ldr.w	sl, [sp]
 8008b8a:	e018      	b.n	8008bbe <compiler_builtins::mem::memcpy+0x284>
 8008b8c:	46aa      	mov	sl, r5
 8008b8e:	e016      	b.n	8008bbe <compiler_builtins::mem::memcpy+0x284>
 8008b90:	460b      	mov	r3, r1
 8008b92:	f109 0104 	add.w	r1, r9, #4
 8008b96:	9107      	str	r1, [sp, #28]
 8008b98:	3304      	adds	r3, #4
 8008b9a:	e00c      	b.n	8008bb6 <compiler_builtins::mem::memcpy+0x27c>
 8008b9c:	9906      	ldr	r1, [sp, #24]
 8008b9e:	46cb      	mov	fp, r9
 8008ba0:	f8dd a000 	ldr.w	sl, [sp]
 8008ba4:	3108      	adds	r1, #8
 8008ba6:	9107      	str	r1, [sp, #28]
 8008ba8:	9905      	ldr	r1, [sp, #20]
 8008baa:	f101 0308 	add.w	r3, r1, #8
 8008bae:	e006      	b.n	8008bbe <compiler_builtins::mem::memcpy+0x284>
 8008bb0:	9906      	ldr	r1, [sp, #24]
 8008bb2:	310c      	adds	r1, #12
 8008bb4:	9107      	str	r1, [sp, #28]
 8008bb6:	f8dd a000 	ldr.w	sl, [sp]
 8008bba:	f8dd b020 	ldr.w	fp, [sp, #32]
 8008bbe:	2500      	movs	r5, #0
 8008bc0:	2e01      	cmp	r6, #1
 8008bc2:	f88d 5028 	strb.w	r5, [sp, #40]	@ 0x28
 8008bc6:	f807 5c26 	strb.w	r5, [r7, #-38]
 8008bca:	d105      	bne.n	8008bd8 <compiler_builtins::mem::memcpy+0x29e>
 8008bcc:	f10d 0828 	add.w	r8, sp, #40	@ 0x28
 8008bd0:	f04f 0900 	mov.w	r9, #0
 8008bd4:	2100      	movs	r1, #0
 8008bd6:	e009      	b.n	8008bec <compiler_builtins::mem::memcpy+0x2b2>
 8008bd8:	9907      	ldr	r1, [sp, #28]
 8008bda:	f1a7 0826 	sub.w	r8, r7, #38	@ 0x26
 8008bde:	790d      	ldrb	r5, [r1, #4]
 8008be0:	7949      	ldrb	r1, [r1, #5]
 8008be2:	f88d 5028 	strb.w	r5, [sp, #40]	@ 0x28
 8008be6:	ea4f 2901 	mov.w	r9, r1, lsl #8
 8008bea:	2102      	movs	r1, #2
 8008bec:	07e6      	lsls	r6, r4, #31
 8008bee:	d101      	bne.n	8008bf4 <compiler_builtins::mem::memcpy+0x2ba>
 8008bf0:	2100      	movs	r1, #0
 8008bf2:	e009      	b.n	8008c08 <compiler_builtins::mem::memcpy+0x2ce>
 8008bf4:	9d07      	ldr	r5, [sp, #28]
 8008bf6:	3504      	adds	r5, #4
 8008bf8:	5c69      	ldrb	r1, [r5, r1]
 8008bfa:	f888 1000 	strb.w	r1, [r8]
 8008bfe:	f817 1c26 	ldrb.w	r1, [r7, #-38]
 8008c02:	f89d 5028 	ldrb.w	r5, [sp, #40]	@ 0x28
 8008c06:	0409      	lsls	r1, r1, #16
 8008c08:	ea41 0109 	orr.w	r1, r1, r9
 8008c0c:	9e01      	ldr	r6, [sp, #4]
 8008c0e:	4329      	orrs	r1, r5
 8008c10:	f00a 0518 	and.w	r5, sl, #24
 8008c14:	40a9      	lsls	r1, r5
 8008c16:	fa2b f606 	lsr.w	r6, fp, r6
 8008c1a:	4331      	orrs	r1, r6
 8008c1c:	6019      	str	r1, [r3, #0]
 8008c1e:	e6f1      	b.n	8008a04 <compiler_builtins::mem::memcpy+0xca>

08008c20 <compiler_builtins::mem::memmove>:
 8008c20:	b5f0      	push	{r4, r5, r6, r7, lr}
 8008c22:	af03      	add	r7, sp, #12
 8008c24:	e92d 0f00 	stmdb	sp!, {r8, r9, sl, fp}
 8008c28:	b08e      	sub	sp, #56	@ 0x38
 8008c2a:	1a43      	subs	r3, r0, r1
 8008c2c:	4293      	cmp	r3, r2
 8008c2e:	d22b      	bcs.n	8008c88 <compiler_builtins::mem::memmove+0x68>
 8008c30:	eb01 0902 	add.w	r9, r1, r2
 8008c34:	eb00 0c02 	add.w	ip, r0, r2
 8008c38:	2a10      	cmp	r2, #16
 8008c3a:	d34a      	bcc.n	8008cd2 <compiler_builtins::mem::memmove+0xb2>
 8008c3c:	f00c 0a03 	and.w	sl, ip, #3
 8008c40:	f02c 0503 	bic.w	r5, ip, #3
 8008c44:	f1ca 0b00 	rsb	fp, sl, #0
 8008c48:	4565      	cmp	r5, ip
 8008c4a:	d264      	bcs.n	8008d16 <compiler_builtins::mem::memmove+0xf6>
 8008c4c:	f1aa 0e01 	sub.w	lr, sl, #1
 8008c50:	f1ba 0f00 	cmp.w	sl, #0
 8008c54:	d049      	beq.n	8008cea <compiler_builtins::mem::memmove+0xca>
 8008c56:	46c8      	mov	r8, r9
 8008c58:	4663      	mov	r3, ip
 8008c5a:	f818 4d01 	ldrb.w	r4, [r8, #-1]!
 8008c5e:	f1ba 0f01 	cmp.w	sl, #1
 8008c62:	f803 4d01 	strb.w	r4, [r3, #-1]!
 8008c66:	d042      	beq.n	8008cee <compiler_builtins::mem::memmove+0xce>
 8008c68:	46c8      	mov	r8, r9
 8008c6a:	4663      	mov	r3, ip
 8008c6c:	f818 4d02 	ldrb.w	r4, [r8, #-2]!
 8008c70:	f1ba 0f02 	cmp.w	sl, #2
 8008c74:	f803 4d02 	strb.w	r4, [r3, #-2]!
 8008c78:	bf1f      	itttt	ne
 8008c7a:	46c8      	movne	r8, r9
 8008c7c:	f818 4d03 	ldrbne.w	r4, [r8, #-3]!
 8008c80:	4663      	movne	r3, ip
 8008c82:	f803 4d03 	strbne.w	r4, [r3, #-3]!
 8008c86:	e032      	b.n	8008cee <compiler_builtins::mem::memmove+0xce>
 8008c88:	2a10      	cmp	r2, #16
 8008c8a:	d327      	bcc.n	8008cdc <compiler_builtins::mem::memmove+0xbc>
 8008c8c:	4243      	negs	r3, r0
 8008c8e:	f003 0803 	and.w	r8, r3, #3
 8008c92:	eb00 0308 	add.w	r3, r0, r8
 8008c96:	4298      	cmp	r0, r3
 8008c98:	f080 80c3 	bcs.w	8008e22 <compiler_builtins::mem::memmove+0x202>
 8008c9c:	f1a8 0c01 	sub.w	ip, r8, #1
 8008ca0:	4606      	mov	r6, r0
 8008ca2:	460d      	mov	r5, r1
 8008ca4:	f1b8 0f00 	cmp.w	r8, #0
 8008ca8:	f000 80a9 	beq.w	8008dfe <compiler_builtins::mem::memmove+0x1de>
 8008cac:	460d      	mov	r5, r1
 8008cae:	4606      	mov	r6, r0
 8008cb0:	f815 eb01 	ldrb.w	lr, [r5], #1
 8008cb4:	f1b8 0f01 	cmp.w	r8, #1
 8008cb8:	f806 eb01 	strb.w	lr, [r6], #1
 8008cbc:	f000 809f 	beq.w	8008dfe <compiler_builtins::mem::memmove+0x1de>
 8008cc0:	784e      	ldrb	r6, [r1, #1]
 8008cc2:	f1b8 0f02 	cmp.w	r8, #2
 8008cc6:	7046      	strb	r6, [r0, #1]
 8008cc8:	f040 8095 	bne.w	8008df6 <compiler_builtins::mem::memmove+0x1d6>
 8008ccc:	1c8d      	adds	r5, r1, #2
 8008cce:	1c86      	adds	r6, r0, #2
 8008cd0:	e095      	b.n	8008dfe <compiler_builtins::mem::memmove+0x1de>
 8008cd2:	4663      	mov	r3, ip
 8008cd4:	1a99      	subs	r1, r3, r2
 8008cd6:	4299      	cmp	r1, r3
 8008cd8:	d35b      	bcc.n	8008d92 <compiler_builtins::mem::memmove+0x172>
 8008cda:	e0f9      	b.n	8008ed0 <compiler_builtins::mem::memmove+0x2b0>
 8008cdc:	4684      	mov	ip, r0
 8008cde:	eb0c 0302 	add.w	r3, ip, r2
 8008ce2:	459c      	cmp	ip, r3
 8008ce4:	f0c0 80c7 	bcc.w	8008e76 <compiler_builtins::mem::memmove+0x256>
 8008ce8:	e0f2      	b.n	8008ed0 <compiler_builtins::mem::memmove+0x2b0>
 8008cea:	4663      	mov	r3, ip
 8008cec:	46c8      	mov	r8, r9
 8008cee:	f1be 0f03 	cmp.w	lr, #3
 8008cf2:	d310      	bcc.n	8008d16 <compiler_builtins::mem::memmove+0xf6>
 8008cf4:	f1a8 0604 	sub.w	r6, r8, #4
 8008cf8:	78f4      	ldrb	r4, [r6, #3]
 8008cfa:	f803 4c01 	strb.w	r4, [r3, #-1]
 8008cfe:	78b4      	ldrb	r4, [r6, #2]
 8008d00:	f803 4c02 	strb.w	r4, [r3, #-2]
 8008d04:	7874      	ldrb	r4, [r6, #1]
 8008d06:	f803 4c03 	strb.w	r4, [r3, #-3]
 8008d0a:	f816 4904 	ldrb.w	r4, [r6], #-4
 8008d0e:	f803 4d04 	strb.w	r4, [r3, #-4]!
 8008d12:	429d      	cmp	r5, r3
 8008d14:	d3f0      	bcc.n	8008cf8 <compiler_builtins::mem::memmove+0xd8>
 8008d16:	eba2 0e0a 	sub.w	lr, r2, sl
 8008d1a:	eb09 0a0b 	add.w	sl, r9, fp
 8008d1e:	f02e 0403 	bic.w	r4, lr, #3
 8008d22:	1b2b      	subs	r3, r5, r4
 8008d24:	f1c4 0800 	rsb	r8, r4, #0
 8008d28:	f01a 0403 	ands.w	r4, sl, #3
 8008d2c:	f040 80de 	bne.w	8008eec <compiler_builtins::mem::memmove+0x2cc>
 8008d30:	42ab      	cmp	r3, r5
 8008d32:	d226      	bcs.n	8008d82 <compiler_builtins::mem::memmove+0x162>
 8008d34:	f1a9 0110 	sub.w	r1, r9, #16
 8008d38:	f1ab 0904 	sub.w	r9, fp, #4
 8008d3c:	eb01 060b 	add.w	r6, r1, fp
 8008d40:	eb0c 050b 	add.w	r5, ip, fp
 8008d44:	68f4      	ldr	r4, [r6, #12]
 8008d46:	f845 4c04 	str.w	r4, [r5, #-4]
 8008d4a:	eb0c 0409 	add.w	r4, ip, r9
 8008d4e:	42a3      	cmp	r3, r4
 8008d50:	d217      	bcs.n	8008d82 <compiler_builtins::mem::memmove+0x162>
 8008d52:	68b2      	ldr	r2, [r6, #8]
 8008d54:	f845 2c08 	str.w	r2, [r5, #-8]
 8008d58:	1f22      	subs	r2, r4, #4
 8008d5a:	4293      	cmp	r3, r2
 8008d5c:	bf3f      	itttt	cc
 8008d5e:	6872      	ldrcc	r2, [r6, #4]
 8008d60:	f845 2c0c 	strcc.w	r2, [r5, #-12]
 8008d64:	f1a4 0208 	subcc.w	r2, r4, #8
 8008d68:	4293      	cmpcc	r3, r2
 8008d6a:	d20a      	bcs.n	8008d82 <compiler_builtins::mem::memmove+0x162>
 8008d6c:	f851 200b 	ldr.w	r2, [r1, fp]
 8008d70:	f1ac 0c10 	sub.w	ip, ip, #16
 8008d74:	3910      	subs	r1, #16
 8008d76:	f845 2c10 	str.w	r2, [r5, #-16]
 8008d7a:	eb0c 020b 	add.w	r2, ip, fp
 8008d7e:	4293      	cmp	r3, r2
 8008d80:	d3dc      	bcc.n	8008d3c <compiler_builtins::mem::memmove+0x11c>
 8008d82:	eb0a 0908 	add.w	r9, sl, r8
 8008d86:	f00e 0203 	and.w	r2, lr, #3
 8008d8a:	1a99      	subs	r1, r3, r2
 8008d8c:	4299      	cmp	r1, r3
 8008d8e:	f080 809f 	bcs.w	8008ed0 <compiler_builtins::mem::memmove+0x2b0>
 8008d92:	f1a2 0c01 	sub.w	ip, r2, #1
 8008d96:	f012 0403 	ands.w	r4, r2, #3
 8008d9a:	d013      	beq.n	8008dc4 <compiler_builtins::mem::memmove+0x1a4>
 8008d9c:	464d      	mov	r5, r9
 8008d9e:	461a      	mov	r2, r3
 8008da0:	f815 6d01 	ldrb.w	r6, [r5, #-1]!
 8008da4:	2c01      	cmp	r4, #1
 8008da6:	f802 6d01 	strb.w	r6, [r2, #-1]!
 8008daa:	d00d      	beq.n	8008dc8 <compiler_builtins::mem::memmove+0x1a8>
 8008dac:	464d      	mov	r5, r9
 8008dae:	461a      	mov	r2, r3
 8008db0:	f815 6d02 	ldrb.w	r6, [r5, #-2]!
 8008db4:	2c02      	cmp	r4, #2
 8008db6:	f802 6d02 	strb.w	r6, [r2, #-2]!
 8008dba:	d005      	beq.n	8008dc8 <compiler_builtins::mem::memmove+0x1a8>
 8008dbc:	f819 2d03 	ldrb.w	r2, [r9, #-3]!
 8008dc0:	f803 2d03 	strb.w	r2, [r3, #-3]!
 8008dc4:	464d      	mov	r5, r9
 8008dc6:	461a      	mov	r2, r3
 8008dc8:	f1bc 0f03 	cmp.w	ip, #3
 8008dcc:	f0c0 8080 	bcc.w	8008ed0 <compiler_builtins::mem::memmove+0x2b0>
 8008dd0:	1eab      	subs	r3, r5, #2
 8008dd2:	785e      	ldrb	r6, [r3, #1]
 8008dd4:	f802 6c01 	strb.w	r6, [r2, #-1]
 8008dd8:	781e      	ldrb	r6, [r3, #0]
 8008dda:	f802 6c02 	strb.w	r6, [r2, #-2]
 8008dde:	f813 6c01 	ldrb.w	r6, [r3, #-1]
 8008de2:	f802 6c03 	strb.w	r6, [r2, #-3]
 8008de6:	f813 6c02 	ldrb.w	r6, [r3, #-2]
 8008dea:	3b04      	subs	r3, #4
 8008dec:	f802 6d04 	strb.w	r6, [r2, #-4]!
 8008df0:	4291      	cmp	r1, r2
 8008df2:	d3ee      	bcc.n	8008dd2 <compiler_builtins::mem::memmove+0x1b2>
 8008df4:	e06c      	b.n	8008ed0 <compiler_builtins::mem::memmove+0x2b0>
 8008df6:	788e      	ldrb	r6, [r1, #2]
 8008df8:	1ccd      	adds	r5, r1, #3
 8008dfa:	7086      	strb	r6, [r0, #2]
 8008dfc:	1cc6      	adds	r6, r0, #3
 8008dfe:	f1bc 0f03 	cmp.w	ip, #3
 8008e02:	d30e      	bcc.n	8008e22 <compiler_builtins::mem::memmove+0x202>
 8008e04:	3d04      	subs	r5, #4
 8008e06:	3e04      	subs	r6, #4
 8008e08:	f815 4f04 	ldrb.w	r4, [r5, #4]!
 8008e0c:	f806 4f04 	strb.w	r4, [r6, #4]!
 8008e10:	786c      	ldrb	r4, [r5, #1]
 8008e12:	7074      	strb	r4, [r6, #1]
 8008e14:	78ac      	ldrb	r4, [r5, #2]
 8008e16:	70b4      	strb	r4, [r6, #2]
 8008e18:	78ec      	ldrb	r4, [r5, #3]
 8008e1a:	70f4      	strb	r4, [r6, #3]
 8008e1c:	1d34      	adds	r4, r6, #4
 8008e1e:	429c      	cmp	r4, r3
 8008e20:	d1f2      	bne.n	8008e08 <compiler_builtins::mem::memmove+0x1e8>
 8008e22:	eba2 0208 	sub.w	r2, r2, r8
 8008e26:	eb01 0508 	add.w	r5, r1, r8
 8008e2a:	f022 0603 	bic.w	r6, r2, #3
 8008e2e:	f015 0903 	ands.w	r9, r5, #3
 8008e32:	eb03 0c06 	add.w	ip, r3, r6
 8008e36:	d16d      	bne.n	8008f14 <compiler_builtins::mem::memmove+0x2f4>
 8008e38:	4563      	cmp	r3, ip
 8008e3a:	d215      	bcs.n	8008e68 <compiler_builtins::mem::memmove+0x248>
 8008e3c:	4629      	mov	r1, r5
 8008e3e:	680c      	ldr	r4, [r1, #0]
 8008e40:	f843 4b04 	str.w	r4, [r3], #4
 8008e44:	4563      	cmp	r3, ip
 8008e46:	d20f      	bcs.n	8008e68 <compiler_builtins::mem::memmove+0x248>
 8008e48:	684c      	ldr	r4, [r1, #4]
 8008e4a:	f843 4b04 	str.w	r4, [r3], #4
 8008e4e:	4563      	cmp	r3, ip
 8008e50:	bf3e      	ittt	cc
 8008e52:	688c      	ldrcc	r4, [r1, #8]
 8008e54:	f843 4b04 	strcc.w	r4, [r3], #4
 8008e58:	4563      	cmpcc	r3, ip
 8008e5a:	d205      	bcs.n	8008e68 <compiler_builtins::mem::memmove+0x248>
 8008e5c:	68cc      	ldr	r4, [r1, #12]
 8008e5e:	3110      	adds	r1, #16
 8008e60:	f843 4b04 	str.w	r4, [r3], #4
 8008e64:	4563      	cmp	r3, ip
 8008e66:	d3ea      	bcc.n	8008e3e <compiler_builtins::mem::memmove+0x21e>
 8008e68:	19a9      	adds	r1, r5, r6
 8008e6a:	f002 0203 	and.w	r2, r2, #3
 8008e6e:	eb0c 0302 	add.w	r3, ip, r2
 8008e72:	459c      	cmp	ip, r3
 8008e74:	d22c      	bcs.n	8008ed0 <compiler_builtins::mem::memmove+0x2b0>
 8008e76:	f1a2 0e01 	sub.w	lr, r2, #1
 8008e7a:	f012 0403 	ands.w	r4, r2, #3
 8008e7e:	d013      	beq.n	8008ea8 <compiler_builtins::mem::memmove+0x288>
 8008e80:	460a      	mov	r2, r1
 8008e82:	4665      	mov	r5, ip
 8008e84:	f812 6b01 	ldrb.w	r6, [r2], #1
 8008e88:	2c01      	cmp	r4, #1
 8008e8a:	f805 6b01 	strb.w	r6, [r5], #1
 8008e8e:	d00d      	beq.n	8008eac <compiler_builtins::mem::memmove+0x28c>
 8008e90:	784a      	ldrb	r2, [r1, #1]
 8008e92:	2c02      	cmp	r4, #2
 8008e94:	f88c 2001 	strb.w	r2, [ip, #1]
 8008e98:	d11e      	bne.n	8008ed8 <compiler_builtins::mem::memmove+0x2b8>
 8008e9a:	1c8a      	adds	r2, r1, #2
 8008e9c:	f10c 0502 	add.w	r5, ip, #2
 8008ea0:	f1be 0f03 	cmp.w	lr, #3
 8008ea4:	d205      	bcs.n	8008eb2 <compiler_builtins::mem::memmove+0x292>
 8008ea6:	e013      	b.n	8008ed0 <compiler_builtins::mem::memmove+0x2b0>
 8008ea8:	4665      	mov	r5, ip
 8008eaa:	460a      	mov	r2, r1
 8008eac:	f1be 0f03 	cmp.w	lr, #3
 8008eb0:	d30e      	bcc.n	8008ed0 <compiler_builtins::mem::memmove+0x2b0>
 8008eb2:	1f11      	subs	r1, r2, #4
 8008eb4:	1f2a      	subs	r2, r5, #4
 8008eb6:	f811 6f04 	ldrb.w	r6, [r1, #4]!
 8008eba:	f802 6f04 	strb.w	r6, [r2, #4]!
 8008ebe:	784e      	ldrb	r6, [r1, #1]
 8008ec0:	7056      	strb	r6, [r2, #1]
 8008ec2:	788e      	ldrb	r6, [r1, #2]
 8008ec4:	7096      	strb	r6, [r2, #2]
 8008ec6:	78ce      	ldrb	r6, [r1, #3]
 8008ec8:	70d6      	strb	r6, [r2, #3]
 8008eca:	1d16      	adds	r6, r2, #4
 8008ecc:	429e      	cmp	r6, r3
 8008ece:	d1f2      	bne.n	8008eb6 <compiler_builtins::mem::memmove+0x296>
 8008ed0:	b00e      	add	sp, #56	@ 0x38
 8008ed2:	e8bd 0f00 	ldmia.w	sp!, {r8, r9, sl, fp}
 8008ed6:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8008ed8:	788a      	ldrb	r2, [r1, #2]
 8008eda:	f10c 0503 	add.w	r5, ip, #3
 8008ede:	f88c 2002 	strb.w	r2, [ip, #2]
 8008ee2:	1cca      	adds	r2, r1, #3
 8008ee4:	f1be 0f03 	cmp.w	lr, #3
 8008ee8:	d2e3      	bcs.n	8008eb2 <compiler_builtins::mem::memmove+0x292>
 8008eea:	e7f1      	b.n	8008ed0 <compiler_builtins::mem::memmove+0x2b0>
 8008eec:	ebaa 0604 	sub.w	r6, sl, r4
 8008ef0:	f04f 0900 	mov.w	r9, #0
 8008ef4:	9607      	str	r6, [sp, #28]
 8008ef6:	00e6      	lsls	r6, r4, #3
 8008ef8:	2c01      	cmp	r4, #1
 8008efa:	f88d 9028 	strb.w	r9, [sp, #40]	@ 0x28
 8008efe:	f807 9c2e 	strb.w	r9, [r7, #-46]
 8008f02:	9606      	str	r6, [sp, #24]
 8008f04:	9402      	str	r4, [sp, #8]
 8008f06:	f040 808e 	bne.w	8009026 <compiler_builtins::mem::memmove+0x406>
 8008f0a:	ae0a      	add	r6, sp, #40	@ 0x28
 8008f0c:	9605      	str	r6, [sp, #20]
 8008f0e:	2600      	movs	r6, #0
 8008f10:	9608      	str	r6, [sp, #32]
 8008f12:	e099      	b.n	8009048 <compiler_builtins::mem::memmove+0x428>
 8008f14:	f04f 0b00 	mov.w	fp, #0
 8008f18:	f1c9 0a04 	rsb	sl, r9, #4
 8008f1c:	ac0d      	add	r4, sp, #52	@ 0x34
 8008f1e:	f8cd b034 	str.w	fp, [sp, #52]	@ 0x34
 8008f22:	eb04 0e09 	add.w	lr, r4, r9
 8008f26:	ea5f 74ca 	movs.w	r4, sl, lsl #31
 8008f2a:	bf1e      	ittt	ne
 8008f2c:	782c      	ldrbne	r4, [r5, #0]
 8008f2e:	f88e 4000 	strbne.w	r4, [lr]
 8008f32:	f04f 0b01 	movne.w	fp, #1
 8008f36:	eba5 0409 	sub.w	r4, r5, r9
 8008f3a:	9407      	str	r4, [sp, #28]
 8008f3c:	ea4f 04c9 	mov.w	r4, r9, lsl #3
 8008f40:	9401      	str	r4, [sp, #4]
 8008f42:	ea5f 748a 	movs.w	r4, sl, lsl #30
 8008f46:	bf44      	itt	mi
 8008f48:	f835 400b 	ldrhmi.w	r4, [r5, fp]
 8008f4c:	f82e 400b 	strhmi.w	r4, [lr, fp]
 8008f50:	9c0d      	ldr	r4, [sp, #52]	@ 0x34
 8008f52:	f8dd a004 	ldr.w	sl, [sp, #4]
 8008f56:	9408      	str	r4, [sp, #32]
 8008f58:	1d1c      	adds	r4, r3, #4
 8008f5a:	4564      	cmp	r4, ip
 8008f5c:	f1ca 0400 	rsb	r4, sl, #0
 8008f60:	9400      	str	r4, [sp, #0]
 8008f62:	f080 8105 	bcs.w	8009170 <compiler_builtins::mem::memmove+0x550>
 8008f66:	f1c9 0300 	rsb	r3, r9, #0
 8008f6a:	f8dd b020 	ldr.w	fp, [sp, #32]
 8008f6e:	eb01 0e03 	add.w	lr, r1, r3
 8008f72:	f004 0118 	and.w	r1, r4, #24
 8008f76:	4604      	mov	r4, r0
 8008f78:	9107      	str	r1, [sp, #28]
 8008f7a:	f8cd e010 	str.w	lr, [sp, #16]
 8008f7e:	44c6      	add	lr, r8
 8008f80:	9b07      	ldr	r3, [sp, #28]
 8008f82:	fa2b fb0a 	lsr.w	fp, fp, sl
 8008f86:	f8de 1004 	ldr.w	r1, [lr, #4]
 8008f8a:	9108      	str	r1, [sp, #32]
 8008f8c:	9403      	str	r4, [sp, #12]
 8008f8e:	fa01 f303 	lsl.w	r3, r1, r3
 8008f92:	eb04 0108 	add.w	r1, r4, r8
 8008f96:	ea43 030b 	orr.w	r3, r3, fp
 8008f9a:	468b      	mov	fp, r1
 8008f9c:	f84b 3b08 	str.w	r3, [fp], #8
 8008fa0:	45e3      	cmp	fp, ip
 8008fa2:	f080 80c6 	bcs.w	8009132 <compiler_builtins::mem::memmove+0x512>
 8008fa6:	9b08      	ldr	r3, [sp, #32]
 8008fa8:	f8cd e018 	str.w	lr, [sp, #24]
 8008fac:	f8de e008 	ldr.w	lr, [lr, #8]
 8008fb0:	fa23 f40a 	lsr.w	r4, r3, sl
 8008fb4:	9b07      	ldr	r3, [sp, #28]
 8008fb6:	9105      	str	r1, [sp, #20]
 8008fb8:	fa0e f303 	lsl.w	r3, lr, r3
 8008fbc:	4323      	orrs	r3, r4
 8008fbe:	604b      	str	r3, [r1, #4]
 8008fc0:	f101 030c 	add.w	r3, r1, #12
 8008fc4:	4563      	cmp	r3, ip
 8008fc6:	f080 80be 	bcs.w	8009146 <compiler_builtins::mem::memmove+0x526>
 8008fca:	9c06      	ldr	r4, [sp, #24]
 8008fcc:	68e4      	ldr	r4, [r4, #12]
 8008fce:	9408      	str	r4, [sp, #32]
 8008fd0:	fa2e f40a 	lsr.w	r4, lr, sl
 8008fd4:	9402      	str	r4, [sp, #8]
 8008fd6:	e9dd 4107 	ldrd	r4, r1, [sp, #28]
 8008fda:	fa01 fe04 	lsl.w	lr, r1, r4
 8008fde:	9905      	ldr	r1, [sp, #20]
 8008fe0:	9c02      	ldr	r4, [sp, #8]
 8008fe2:	3110      	adds	r1, #16
 8008fe4:	ea44 040e 	orr.w	r4, r4, lr
 8008fe8:	4561      	cmp	r1, ip
 8008fea:	f8cb 4000 	str.w	r4, [fp]
 8008fee:	f080 80bc 	bcs.w	800916a <compiler_builtins::mem::memmove+0x54a>
 8008ff2:	9906      	ldr	r1, [sp, #24]
 8008ff4:	9c07      	ldr	r4, [sp, #28]
 8008ff6:	f8dd e010 	ldr.w	lr, [sp, #16]
 8008ffa:	f8d1 b010 	ldr.w	fp, [r1, #16]
 8008ffe:	9908      	ldr	r1, [sp, #32]
 8009000:	f10e 0e10 	add.w	lr, lr, #16
 8009004:	fa0b f404 	lsl.w	r4, fp, r4
 8009008:	fa21 f10a 	lsr.w	r1, r1, sl
 800900c:	4321      	orrs	r1, r4
 800900e:	9c03      	ldr	r4, [sp, #12]
 8009010:	6019      	str	r1, [r3, #0]
 8009012:	3410      	adds	r4, #16
 8009014:	eb04 0308 	add.w	r3, r4, r8
 8009018:	1d19      	adds	r1, r3, #4
 800901a:	4561      	cmp	r1, ip
 800901c:	d3ad      	bcc.n	8008f7a <compiler_builtins::mem::memmove+0x35a>
 800901e:	eb0e 0108 	add.w	r1, lr, r8
 8009022:	9107      	str	r1, [sp, #28]
 8009024:	e0a6      	b.n	8009174 <compiler_builtins::mem::memmove+0x554>
 8009026:	9e07      	ldr	r6, [sp, #28]
 8009028:	f896 9000 	ldrb.w	r9, [r6]
 800902c:	7876      	ldrb	r6, [r6, #1]
 800902e:	9608      	str	r6, [sp, #32]
 8009030:	ea5f 76ca 	movs.w	r6, sl, lsl #31
 8009034:	f88d 9028 	strb.w	r9, [sp, #40]	@ 0x28
 8009038:	d101      	bne.n	800903e <compiler_builtins::mem::memmove+0x41e>
 800903a:	2400      	movs	r4, #0
 800903c:	e00e      	b.n	800905c <compiler_builtins::mem::memmove+0x43c>
 800903e:	f04f 0902 	mov.w	r9, #2
 8009042:	f1a7 062e 	sub.w	r6, r7, #46	@ 0x2e
 8009046:	9605      	str	r6, [sp, #20]
 8009048:	9e07      	ldr	r6, [sp, #28]
 800904a:	9c05      	ldr	r4, [sp, #20]
 800904c:	f816 6009 	ldrb.w	r6, [r6, r9]
 8009050:	7026      	strb	r6, [r4, #0]
 8009052:	f817 6c2e 	ldrb.w	r6, [r7, #-46]
 8009056:	f89d 9028 	ldrb.w	r9, [sp, #40]	@ 0x28
 800905a:	0434      	lsls	r4, r6, #16
 800905c:	9e08      	ldr	r6, [sp, #32]
 800905e:	ea44 2606 	orr.w	r6, r4, r6, lsl #8
 8009062:	9c02      	ldr	r4, [sp, #8]
 8009064:	ea46 0609 	orr.w	r6, r6, r9
 8009068:	9605      	str	r6, [sp, #20]
 800906a:	1d1e      	adds	r6, r3, #4
 800906c:	9608      	str	r6, [sp, #32]
 800906e:	42ae      	cmp	r6, r5
 8009070:	9e06      	ldr	r6, [sp, #24]
 8009072:	f1c6 0900 	rsb	r9, r6, #0
 8009076:	d259      	bcs.n	800912c <compiler_builtins::mem::memmove+0x50c>
 8009078:	1b12      	subs	r2, r2, r4
 800907a:	f8cd 9004 	str.w	r9, [sp, #4]
 800907e:	4411      	add	r1, r2
 8009080:	f1ab 0204 	sub.w	r2, fp, #4
 8009084:	9203      	str	r2, [sp, #12]
 8009086:	f009 0218 	and.w	r2, r9, #24
 800908a:	9207      	str	r2, [sp, #28]
 800908c:	9a05      	ldr	r2, [sp, #20]
 800908e:	eb01 090b 	add.w	r9, r1, fp
 8009092:	9104      	str	r1, [sp, #16]
 8009094:	9907      	ldr	r1, [sp, #28]
 8009096:	eb0c 060b 	add.w	r6, ip, fp
 800909a:	f859 4c04 	ldr.w	r4, [r9, #-4]
 800909e:	9d06      	ldr	r5, [sp, #24]
 80090a0:	fa02 f101 	lsl.w	r1, r2, r1
 80090a4:	fa24 f205 	lsr.w	r2, r4, r5
 80090a8:	4311      	orrs	r1, r2
 80090aa:	f846 1c04 	str.w	r1, [r6, #-4]
 80090ae:	9903      	ldr	r1, [sp, #12]
 80090b0:	eb0c 0201 	add.w	r2, ip, r1
 80090b4:	9908      	ldr	r1, [sp, #32]
 80090b6:	4291      	cmp	r1, r2
 80090b8:	d240      	bcs.n	800913c <compiler_builtins::mem::memmove+0x51c>
 80090ba:	f859 1c08 	ldr.w	r1, [r9, #-8]
 80090be:	9105      	str	r1, [sp, #20]
 80090c0:	9907      	ldr	r1, [sp, #28]
 80090c2:	408c      	lsls	r4, r1
 80090c4:	9905      	ldr	r1, [sp, #20]
 80090c6:	fa21 f505 	lsr.w	r5, r1, r5
 80090ca:	9908      	ldr	r1, [sp, #32]
 80090cc:	4325      	orrs	r5, r4
 80090ce:	f846 5c08 	str.w	r5, [r6, #-8]
 80090d2:	1f15      	subs	r5, r2, #4
 80090d4:	42a9      	cmp	r1, r5
 80090d6:	d23e      	bcs.n	8009156 <compiler_builtins::mem::memmove+0x536>
 80090d8:	9907      	ldr	r1, [sp, #28]
 80090da:	9d05      	ldr	r5, [sp, #20]
 80090dc:	f859 4c0c 	ldr.w	r4, [r9, #-12]
 80090e0:	fa05 f101 	lsl.w	r1, r5, r1
 80090e4:	9d06      	ldr	r5, [sp, #24]
 80090e6:	fa24 f505 	lsr.w	r5, r4, r5
 80090ea:	4329      	orrs	r1, r5
 80090ec:	f846 1c0c 	str.w	r1, [r6, #-12]
 80090f0:	f1a2 0508 	sub.w	r5, r2, #8
 80090f4:	9908      	ldr	r1, [sp, #32]
 80090f6:	42a9      	cmp	r1, r5
 80090f8:	d275      	bcs.n	80091e6 <compiler_builtins::mem::memmove+0x5c6>
 80090fa:	9907      	ldr	r1, [sp, #28]
 80090fc:	f1ac 0c10 	sub.w	ip, ip, #16
 8009100:	f859 9c10 	ldr.w	r9, [r9, #-16]
 8009104:	eb0c 050b 	add.w	r5, ip, fp
 8009108:	9a06      	ldr	r2, [sp, #24]
 800910a:	fa04 f101 	lsl.w	r1, r4, r1
 800910e:	fa29 f202 	lsr.w	r2, r9, r2
 8009112:	4311      	orrs	r1, r2
 8009114:	f846 1c10 	str.w	r1, [r6, #-16]
 8009118:	9904      	ldr	r1, [sp, #16]
 800911a:	9a08      	ldr	r2, [sp, #32]
 800911c:	3910      	subs	r1, #16
 800911e:	42aa      	cmp	r2, r5
 8009120:	464a      	mov	r2, r9
 8009122:	d3b4      	bcc.n	800908e <compiler_builtins::mem::memmove+0x46e>
 8009124:	4459      	add	r1, fp
 8009126:	4693      	mov	fp, r2
 8009128:	9107      	str	r1, [sp, #28]
 800912a:	e01b      	b.n	8009164 <compiler_builtins::mem::memmove+0x544>
 800912c:	f8dd b014 	ldr.w	fp, [sp, #20]
 8009130:	e062      	b.n	80091f8 <compiler_builtins::mem::memmove+0x5d8>
 8009132:	460b      	mov	r3, r1
 8009134:	f10e 0104 	add.w	r1, lr, #4
 8009138:	3304      	adds	r3, #4
 800913a:	e018      	b.n	800916e <compiler_builtins::mem::memmove+0x54e>
 800913c:	e9dd 6103 	ldrd	r6, r1, [sp, #12]
 8009140:	4615      	mov	r5, r2
 8009142:	4431      	add	r1, r6
 8009144:	e053      	b.n	80091ee <compiler_builtins::mem::memmove+0x5ce>
 8009146:	9906      	ldr	r1, [sp, #24]
 8009148:	46f3      	mov	fp, lr
 800914a:	3108      	adds	r1, #8
 800914c:	9107      	str	r1, [sp, #28]
 800914e:	9905      	ldr	r1, [sp, #20]
 8009150:	f101 0308 	add.w	r3, r1, #8
 8009154:	e00e      	b.n	8009174 <compiler_builtins::mem::memmove+0x554>
 8009156:	e9dd 6203 	ldrd	r6, r2, [sp, #12]
 800915a:	f8dd b014 	ldr.w	fp, [sp, #20]
 800915e:	4432      	add	r2, r6
 8009160:	3a04      	subs	r2, #4
 8009162:	9207      	str	r2, [sp, #28]
 8009164:	e9dd 9401 	ldrd	r9, r4, [sp, #4]
 8009168:	e046      	b.n	80091f8 <compiler_builtins::mem::memmove+0x5d8>
 800916a:	9906      	ldr	r1, [sp, #24]
 800916c:	310c      	adds	r1, #12
 800916e:	9107      	str	r1, [sp, #28]
 8009170:	f8dd b020 	ldr.w	fp, [sp, #32]
 8009174:	2400      	movs	r4, #0
 8009176:	f1b9 0f01 	cmp.w	r9, #1
 800917a:	f88d 402c 	strb.w	r4, [sp, #44]	@ 0x2c
 800917e:	f807 4c2a 	strb.w	r4, [r7, #-42]
 8009182:	d106      	bne.n	8009192 <compiler_builtins::mem::memmove+0x572>
 8009184:	f10d 092c 	add.w	r9, sp, #44	@ 0x2c
 8009188:	f04f 0800 	mov.w	r8, #0
 800918c:	f04f 0e00 	mov.w	lr, #0
 8009190:	e00a      	b.n	80091a8 <compiler_builtins::mem::memmove+0x588>
 8009192:	9907      	ldr	r1, [sp, #28]
 8009194:	f1a7 092a 	sub.w	r9, r7, #42	@ 0x2a
 8009198:	f04f 0e02 	mov.w	lr, #2
 800919c:	790c      	ldrb	r4, [r1, #4]
 800919e:	7949      	ldrb	r1, [r1, #5]
 80091a0:	f88d 402c 	strb.w	r4, [sp, #44]	@ 0x2c
 80091a4:	ea4f 2801 	mov.w	r8, r1, lsl #8
 80091a8:	07e9      	lsls	r1, r5, #31
 80091aa:	d102      	bne.n	80091b2 <compiler_builtins::mem::memmove+0x592>
 80091ac:	f04f 0e00 	mov.w	lr, #0
 80091b0:	e00b      	b.n	80091ca <compiler_builtins::mem::memmove+0x5aa>
 80091b2:	9907      	ldr	r1, [sp, #28]
 80091b4:	3104      	adds	r1, #4
 80091b6:	f811 100e 	ldrb.w	r1, [r1, lr]
 80091ba:	f889 1000 	strb.w	r1, [r9]
 80091be:	f817 1c2a 	ldrb.w	r1, [r7, #-42]
 80091c2:	f89d 402c 	ldrb.w	r4, [sp, #44]	@ 0x2c
 80091c6:	ea4f 4e01 	mov.w	lr, r1, lsl #16
 80091ca:	9901      	ldr	r1, [sp, #4]
 80091cc:	fa2b f901 	lsr.w	r9, fp, r1
 80091d0:	ea48 010e 	orr.w	r1, r8, lr
 80091d4:	4321      	orrs	r1, r4
 80091d6:	9c00      	ldr	r4, [sp, #0]
 80091d8:	f004 0418 	and.w	r4, r4, #24
 80091dc:	40a1      	lsls	r1, r4
 80091de:	ea41 0109 	orr.w	r1, r1, r9
 80091e2:	6019      	str	r1, [r3, #0]
 80091e4:	e640      	b.n	8008e68 <compiler_builtins::mem::memmove+0x248>
 80091e6:	e9dd 2103 	ldrd	r2, r1, [sp, #12]
 80091ea:	4411      	add	r1, r2
 80091ec:	3908      	subs	r1, #8
 80091ee:	9107      	str	r1, [sp, #28]
 80091f0:	46a3      	mov	fp, r4
 80091f2:	f8dd 9004 	ldr.w	r9, [sp, #4]
 80091f6:	9c02      	ldr	r4, [sp, #8]
 80091f8:	a90c      	add	r1, sp, #48	@ 0x30
 80091fa:	2600      	movs	r6, #0
 80091fc:	eb01 0c04 	add.w	ip, r1, r4
 8009200:	9907      	ldr	r1, [sp, #28]
 8009202:	960c      	str	r6, [sp, #48]	@ 0x30
 8009204:	4421      	add	r1, r4
 8009206:	1f0a      	subs	r2, r1, #4
 8009208:	f1c4 0104 	rsb	r1, r4, #4
 800920c:	07cc      	lsls	r4, r1, #31
 800920e:	bf1e      	ittt	ne
 8009210:	7814      	ldrbne	r4, [r2, #0]
 8009212:	f88c 4000 	strbne.w	r4, [ip]
 8009216:	2601      	movne	r6, #1
 8009218:	0789      	lsls	r1, r1, #30
 800921a:	bf44      	itt	mi
 800921c:	5b91      	ldrhmi	r1, [r2, r6]
 800921e:	f82c 1006 	strhmi.w	r1, [ip, r6]
 8009222:	990c      	ldr	r1, [sp, #48]	@ 0x30
 8009224:	9a06      	ldr	r2, [sp, #24]
 8009226:	40d1      	lsrs	r1, r2
 8009228:	f009 0218 	and.w	r2, r9, #24
 800922c:	fa0b f202 	lsl.w	r2, fp, r2
 8009230:	4311      	orrs	r1, r2
 8009232:	f845 1c04 	str.w	r1, [r5, #-4]
 8009236:	e5a4      	b.n	8008d82 <compiler_builtins::mem::memmove+0x162>

08009238 <__aeabi_memmove8>:
 8009238:	b580      	push	{r7, lr}
 800923a:	466f      	mov	r7, sp
 800923c:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8009240:	f7ff bcee 	b.w	8008c20 <compiler_builtins::mem::memmove>

08009244 <__aeabi_memclr4>:
 8009244:	b580      	push	{r7, lr}
 8009246:	466f      	mov	r7, sp
 8009248:	2904      	cmp	r1, #4
 800924a:	d305      	bcc.n	8009258 <__aeabi_memclr4+0x14>
 800924c:	1f0b      	subs	r3, r1, #4
 800924e:	43da      	mvns	r2, r3
 8009250:	f012 0f0c 	tst.w	r2, #12
 8009254:	d102      	bne.n	800925c <__aeabi_memclr4+0x18>
 8009256:	e01a      	b.n	800928e <__aeabi_memclr4+0x4a>
 8009258:	4602      	mov	r2, r0
 800925a:	e024      	b.n	80092a6 <__aeabi_memclr4+0x62>
 800925c:	f04f 0c00 	mov.w	ip, #0
 8009260:	4602      	mov	r2, r0
 8009262:	f842 cb04 	str.w	ip, [r2], #4
 8009266:	f013 0f0c 	tst.w	r3, #12
 800926a:	d008      	beq.n	800927e <__aeabi_memclr4+0x3a>
 800926c:	f003 020c 	and.w	r2, r3, #12
 8009270:	f8c0 c004 	str.w	ip, [r0, #4]
 8009274:	2a04      	cmp	r2, #4
 8009276:	d105      	bne.n	8009284 <__aeabi_memclr4+0x40>
 8009278:	3908      	subs	r1, #8
 800927a:	3008      	adds	r0, #8
 800927c:	e006      	b.n	800928c <__aeabi_memclr4+0x48>
 800927e:	4619      	mov	r1, r3
 8009280:	4610      	mov	r0, r2
 8009282:	e004      	b.n	800928e <__aeabi_memclr4+0x4a>
 8009284:	2200      	movs	r2, #0
 8009286:	390c      	subs	r1, #12
 8009288:	6082      	str	r2, [r0, #8]
 800928a:	300c      	adds	r0, #12
 800928c:	4602      	mov	r2, r0
 800928e:	2b0c      	cmp	r3, #12
 8009290:	d309      	bcc.n	80092a6 <__aeabi_memclr4+0x62>
 8009292:	2300      	movs	r3, #0
 8009294:	4602      	mov	r2, r0
 8009296:	3910      	subs	r1, #16
 8009298:	e9c2 3300 	strd	r3, r3, [r2]
 800929c:	e9c2 3302 	strd	r3, r3, [r2, #8]
 80092a0:	3210      	adds	r2, #16
 80092a2:	2903      	cmp	r1, #3
 80092a4:	d8f7      	bhi.n	8009296 <__aeabi_memclr4+0x52>
 80092a6:	1850      	adds	r0, r2, r1
 80092a8:	4282      	cmp	r2, r0
 80092aa:	d221      	bcs.n	80092f0 <__aeabi_memclr4+0xac>
 80092ac:	f1a1 0c01 	sub.w	ip, r1, #1
 80092b0:	f011 0303 	ands.w	r3, r1, #3
 80092b4:	d00c      	beq.n	80092d0 <__aeabi_memclr4+0x8c>
 80092b6:	f04f 0e00 	mov.w	lr, #0
 80092ba:	4611      	mov	r1, r2
 80092bc:	f801 eb01 	strb.w	lr, [r1], #1
 80092c0:	2b01      	cmp	r3, #1
 80092c2:	d00a      	beq.n	80092da <__aeabi_memclr4+0x96>
 80092c4:	2b02      	cmp	r3, #2
 80092c6:	f882 e001 	strb.w	lr, [r2, #1]
 80092ca:	d103      	bne.n	80092d4 <__aeabi_memclr4+0x90>
 80092cc:	1c91      	adds	r1, r2, #2
 80092ce:	e004      	b.n	80092da <__aeabi_memclr4+0x96>
 80092d0:	4611      	mov	r1, r2
 80092d2:	e002      	b.n	80092da <__aeabi_memclr4+0x96>
 80092d4:	2100      	movs	r1, #0
 80092d6:	7091      	strb	r1, [r2, #2]
 80092d8:	1cd1      	adds	r1, r2, #3
 80092da:	f1bc 0f03 	cmp.w	ip, #3
 80092de:	bf38      	it	cc
 80092e0:	bd80      	popcc	{r7, pc}
 80092e2:	3904      	subs	r1, #4
 80092e4:	2200      	movs	r2, #0
 80092e6:	f841 2f04 	str.w	r2, [r1, #4]!
 80092ea:	1d0b      	adds	r3, r1, #4
 80092ec:	4283      	cmp	r3, r0
 80092ee:	d1fa      	bne.n	80092e6 <__aeabi_memclr4+0xa2>
 80092f0:	bd80      	pop	{r7, pc}

080092f2 <__aeabi_memcpy>:
 80092f2:	b580      	push	{r7, lr}
 80092f4:	466f      	mov	r7, sp
 80092f6:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 80092fa:	f7ff bb1e 	b.w	800893a <compiler_builtins::mem::memcpy>

080092fe <__aeabi_memcpy8>:
 80092fe:	2a04      	cmp	r2, #4
 8009300:	d309      	bcc.n	8009316 <__aeabi_memcpy8+0x18>
 8009302:	b5d0      	push	{r4, r6, r7, lr}
 8009304:	af02      	add	r7, sp, #8
 8009306:	f1a2 0e04 	sub.w	lr, r2, #4
 800930a:	ea6f 030e 	mvn.w	r3, lr
 800930e:	f013 0f0c 	tst.w	r3, #12
 8009312:	d106      	bne.n	8009322 <__aeabi_memcpy8+0x24>
 8009314:	e023      	b.n	800935e <__aeabi_memcpy8+0x60>
 8009316:	460b      	mov	r3, r1
 8009318:	4684      	mov	ip, r0
 800931a:	4660      	mov	r0, ip
 800931c:	4619      	mov	r1, r3
 800931e:	f7ff bb0c 	b.w	800893a <compiler_builtins::mem::memcpy>
 8009322:	460b      	mov	r3, r1
 8009324:	4684      	mov	ip, r0
 8009326:	f853 4b04 	ldr.w	r4, [r3], #4
 800932a:	f01e 0f0c 	tst.w	lr, #12
 800932e:	f84c 4b04 	str.w	r4, [ip], #4
 8009332:	d009      	beq.n	8009348 <__aeabi_memcpy8+0x4a>
 8009334:	684b      	ldr	r3, [r1, #4]
 8009336:	6043      	str	r3, [r0, #4]
 8009338:	f00e 030c 	and.w	r3, lr, #12
 800933c:	2b04      	cmp	r3, #4
 800933e:	d107      	bne.n	8009350 <__aeabi_memcpy8+0x52>
 8009340:	3a08      	subs	r2, #8
 8009342:	3108      	adds	r1, #8
 8009344:	3008      	adds	r0, #8
 8009346:	e008      	b.n	800935a <__aeabi_memcpy8+0x5c>
 8009348:	4672      	mov	r2, lr
 800934a:	4660      	mov	r0, ip
 800934c:	4619      	mov	r1, r3
 800934e:	e006      	b.n	800935e <__aeabi_memcpy8+0x60>
 8009350:	688b      	ldr	r3, [r1, #8]
 8009352:	3a0c      	subs	r2, #12
 8009354:	6083      	str	r3, [r0, #8]
 8009356:	310c      	adds	r1, #12
 8009358:	300c      	adds	r0, #12
 800935a:	4684      	mov	ip, r0
 800935c:	460b      	mov	r3, r1
 800935e:	f1be 0f0c 	cmp.w	lr, #12
 8009362:	d314      	bcc.n	800938e <__aeabi_memcpy8+0x90>
 8009364:	4684      	mov	ip, r0
 8009366:	460b      	mov	r3, r1
 8009368:	6818      	ldr	r0, [r3, #0]
 800936a:	3a10      	subs	r2, #16
 800936c:	f8cc 0000 	str.w	r0, [ip]
 8009370:	2a03      	cmp	r2, #3
 8009372:	6858      	ldr	r0, [r3, #4]
 8009374:	f8cc 0004 	str.w	r0, [ip, #4]
 8009378:	6898      	ldr	r0, [r3, #8]
 800937a:	f8cc 0008 	str.w	r0, [ip, #8]
 800937e:	68d8      	ldr	r0, [r3, #12]
 8009380:	f103 0310 	add.w	r3, r3, #16
 8009384:	f8cc 000c 	str.w	r0, [ip, #12]
 8009388:	f10c 0c10 	add.w	ip, ip, #16
 800938c:	d8ec      	bhi.n	8009368 <__aeabi_memcpy8+0x6a>
 800938e:	e8bd 40d0 	ldmia.w	sp!, {r4, r6, r7, lr}
 8009392:	4660      	mov	r0, ip
 8009394:	4619      	mov	r1, r3
 8009396:	f7ff bad0 	b.w	800893a <compiler_builtins::mem::memcpy>

0800939a <__aeabi_memclr>:
 800939a:	b5b0      	push	{r4, r5, r7, lr}
 800939c:	af02      	add	r7, sp, #8
 800939e:	2910      	cmp	r1, #16
 80093a0:	d33f      	bcc.n	8009422 <__aeabi_memclr+0x88>
 80093a2:	4242      	negs	r2, r0
 80093a4:	f002 0503 	and.w	r5, r2, #3
 80093a8:	1942      	adds	r2, r0, r5
 80093aa:	4290      	cmp	r0, r2
 80093ac:	d220      	bcs.n	80093f0 <__aeabi_memclr+0x56>
 80093ae:	f1a5 0c01 	sub.w	ip, r5, #1
 80093b2:	b18d      	cbz	r5, 80093d8 <__aeabi_memclr+0x3e>
 80093b4:	2300      	movs	r3, #0
 80093b6:	4686      	mov	lr, r0
 80093b8:	f80e 3b01 	strb.w	r3, [lr], #1
 80093bc:	2d01      	cmp	r5, #1
 80093be:	d007      	beq.n	80093d0 <__aeabi_memclr+0x36>
 80093c0:	7043      	strb	r3, [r0, #1]
 80093c2:	2d02      	cmp	r5, #2
 80093c4:	bf1a      	itte	ne
 80093c6:	7083      	strbne	r3, [r0, #2]
 80093c8:	f100 0e03 	addne.w	lr, r0, #3
 80093cc:	f100 0e02 	addeq.w	lr, r0, #2
 80093d0:	f1bc 0f03 	cmp.w	ip, #3
 80093d4:	d204      	bcs.n	80093e0 <__aeabi_memclr+0x46>
 80093d6:	e00b      	b.n	80093f0 <__aeabi_memclr+0x56>
 80093d8:	4686      	mov	lr, r0
 80093da:	f1bc 0f03 	cmp.w	ip, #3
 80093de:	d307      	bcc.n	80093f0 <__aeabi_memclr+0x56>
 80093e0:	f1ae 0004 	sub.w	r0, lr, #4
 80093e4:	2300      	movs	r3, #0
 80093e6:	f840 3f04 	str.w	r3, [r0, #4]!
 80093ea:	1d04      	adds	r4, r0, #4
 80093ec:	4294      	cmp	r4, r2
 80093ee:	d1fa      	bne.n	80093e6 <__aeabi_memclr+0x4c>
 80093f0:	1b49      	subs	r1, r1, r5
 80093f2:	f021 0003 	bic.w	r0, r1, #3
 80093f6:	4410      	add	r0, r2
 80093f8:	4282      	cmp	r2, r0
 80093fa:	d210      	bcs.n	800941e <__aeabi_memclr+0x84>
 80093fc:	2300      	movs	r3, #0
 80093fe:	f842 3b04 	str.w	r3, [r2], #4
 8009402:	4282      	cmp	r2, r0
 8009404:	d20b      	bcs.n	800941e <__aeabi_memclr+0x84>
 8009406:	f842 3b04 	str.w	r3, [r2], #4
 800940a:	4282      	cmp	r2, r0
 800940c:	bf3c      	itt	cc
 800940e:	f842 3b04 	strcc.w	r3, [r2], #4
 8009412:	4282      	cmpcc	r2, r0
 8009414:	d203      	bcs.n	800941e <__aeabi_memclr+0x84>
 8009416:	f842 3b04 	str.w	r3, [r2], #4
 800941a:	4282      	cmp	r2, r0
 800941c:	d3ef      	bcc.n	80093fe <__aeabi_memclr+0x64>
 800941e:	f001 0103 	and.w	r1, r1, #3
 8009422:	1842      	adds	r2, r0, r1
 8009424:	4290      	cmp	r0, r2
 8009426:	d21f      	bcs.n	8009468 <__aeabi_memclr+0xce>
 8009428:	1e4b      	subs	r3, r1, #1
 800942a:	f011 0403 	ands.w	r4, r1, #3
 800942e:	d00c      	beq.n	800944a <__aeabi_memclr+0xb0>
 8009430:	f04f 0c00 	mov.w	ip, #0
 8009434:	4601      	mov	r1, r0
 8009436:	f801 cb01 	strb.w	ip, [r1], #1
 800943a:	2c01      	cmp	r4, #1
 800943c:	d00a      	beq.n	8009454 <__aeabi_memclr+0xba>
 800943e:	2c02      	cmp	r4, #2
 8009440:	f880 c001 	strb.w	ip, [r0, #1]
 8009444:	d103      	bne.n	800944e <__aeabi_memclr+0xb4>
 8009446:	1c81      	adds	r1, r0, #2
 8009448:	e004      	b.n	8009454 <__aeabi_memclr+0xba>
 800944a:	4601      	mov	r1, r0
 800944c:	e002      	b.n	8009454 <__aeabi_memclr+0xba>
 800944e:	2100      	movs	r1, #0
 8009450:	7081      	strb	r1, [r0, #2]
 8009452:	1cc1      	adds	r1, r0, #3
 8009454:	2b03      	cmp	r3, #3
 8009456:	bf38      	it	cc
 8009458:	bdb0      	popcc	{r4, r5, r7, pc}
 800945a:	1f08      	subs	r0, r1, #4
 800945c:	2100      	movs	r1, #0
 800945e:	f840 1f04 	str.w	r1, [r0, #4]!
 8009462:	1d03      	adds	r3, r0, #4
 8009464:	4293      	cmp	r3, r2
 8009466:	d1fa      	bne.n	800945e <__aeabi_memclr+0xc4>
 8009468:	bdb0      	pop	{r4, r5, r7, pc}
 800946a:	d4d4      	bmi.n	8009416 <__aeabi_memclr+0x7c>

0800946c <__aeabi_uldivmod>:
 800946c:	b510      	push	{r4, lr}
 800946e:	b084      	sub	sp, #16
 8009470:	ac02      	add	r4, sp, #8
 8009472:	9400      	str	r4, [sp, #0]
 8009474:	f000 f804 	bl	8009480 <__udivmoddi4>
 8009478:	9a02      	ldr	r2, [sp, #8]
 800947a:	9b03      	ldr	r3, [sp, #12]
 800947c:	b004      	add	sp, #16
 800947e:	bd10      	pop	{r4, pc}

08009480 <__udivmoddi4>:
 8009480:	b580      	push	{r7, lr}
 8009482:	466f      	mov	r7, sp
 8009484:	b086      	sub	sp, #24
 8009486:	4684      	mov	ip, r0
 8009488:	a802      	add	r0, sp, #8
 800948a:	e9cd 2300 	strd	r2, r3, [sp]
 800948e:	4662      	mov	r2, ip
 8009490:	460b      	mov	r3, r1
 8009492:	f000 f80d 	bl	80094b0 <compiler_builtins::int::specialized_div_rem::u64_div_rem>
 8009496:	f8d7 c008 	ldr.w	ip, [r7, #8]
 800949a:	e9dd 0102 	ldrd	r0, r1, [sp, #8]
 800949e:	f1bc 0f00 	cmp.w	ip, #0
 80094a2:	bf1c      	itt	ne
 80094a4:	e9dd 3204 	ldrdne	r3, r2, [sp, #16]
 80094a8:	e9cc 3200 	strdne	r3, r2, [ip]
 80094ac:	b006      	add	sp, #24
 80094ae:	bd80      	pop	{r7, pc}

080094b0 <compiler_builtins::int::specialized_div_rem::u64_div_rem>:
 80094b0:	b5f0      	push	{r4, r5, r6, r7, lr}
 80094b2:	af03      	add	r7, sp, #12
 80094b4:	e92d 0b00 	stmdb	sp!, {r8, r9, fp}
 80094b8:	e9d7 e802 	ldrd	lr, r8, [r7, #8]
 80094bc:	f1be 0f00 	cmp.w	lr, #0
 80094c0:	d072      	beq.n	80095a8 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xf8>
 80094c2:	f1b8 0f00 	cmp.w	r8, #0
 80094c6:	d16f      	bne.n	80095a8 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xf8>
 80094c8:	2b00      	cmp	r3, #0
 80094ca:	f000 80fc 	beq.w	80096c6 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x216>
 80094ce:	4573      	cmp	r3, lr
 80094d0:	f080 8107 	bcs.w	80096e2 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x232>
 80094d4:	fab3 f183 	clz	r1, r3
 80094d8:	fabe f68e 	clz	r6, lr
 80094dc:	1a71      	subs	r1, r6, r1
 80094de:	f101 0620 	add.w	r6, r1, #32
 80094e2:	bf08      	it	eq
 80094e4:	261f      	moveq	r6, #31
 80094e6:	f1c6 0520 	rsb	r5, r6, #32
 80094ea:	fa08 f106 	lsl.w	r1, r8, r6
 80094ee:	fa0e f806 	lsl.w	r8, lr, r6
 80094f2:	fa2e f505 	lsr.w	r5, lr, r5
 80094f6:	4329      	orrs	r1, r5
 80094f8:	f1b6 0520 	subs.w	r5, r6, #32
 80094fc:	f006 061f 	and.w	r6, r6, #31
 8009500:	bf58      	it	pl
 8009502:	fa0e f105 	lslpl.w	r1, lr, r5
 8009506:	f04f 0501 	mov.w	r5, #1
 800950a:	fa05 fc06 	lsl.w	ip, r5, r6
 800950e:	f04f 0500 	mov.w	r5, #0
 8009512:	bf58      	it	pl
 8009514:	f04f 0800 	movpl.w	r8, #0
 8009518:	e008      	b.n	800952c <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x7c>
 800951a:	4622      	mov	r2, r4
 800951c:	4633      	mov	r3, r6
 800951e:	ea4f 1418 	mov.w	r4, r8, lsr #4
 8009522:	ea44 7801 	orr.w	r8, r4, r1, lsl #28
 8009526:	ea4f 1c1c 	mov.w	ip, ip, lsr #4
 800952a:	0909      	lsrs	r1, r1, #4
 800952c:	ebb2 0408 	subs.w	r4, r2, r8
 8009530:	eb63 0601 	sbc.w	r6, r3, r1
 8009534:	2e00      	cmp	r6, #0
 8009536:	d403      	bmi.n	8009540 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x90>
 8009538:	ea45 050c 	orr.w	r5, r5, ip
 800953c:	d102      	bne.n	8009544 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x94>
 800953e:	e02d      	b.n	800959c <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xec>
 8009540:	4614      	mov	r4, r2
 8009542:	461e      	mov	r6, r3
 8009544:	ea5f 0351 	movs.w	r3, r1, lsr #1
 8009548:	ea4f 0238 	mov.w	r2, r8, rrx
 800954c:	1aa2      	subs	r2, r4, r2
 800954e:	eb66 0303 	sbc.w	r3, r6, r3
 8009552:	2b00      	cmp	r3, #0
 8009554:	d404      	bmi.n	8009560 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xb0>
 8009556:	ea45 055c 	orr.w	r5, r5, ip, lsr #1
 800955a:	4614      	mov	r4, r2
 800955c:	d102      	bne.n	8009564 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xb4>
 800955e:	e01d      	b.n	800959c <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xec>
 8009560:	4622      	mov	r2, r4
 8009562:	4633      	mov	r3, r6
 8009564:	ea4f 0498 	mov.w	r4, r8, lsr #2
 8009568:	ea44 7481 	orr.w	r4, r4, r1, lsl #30
 800956c:	1b14      	subs	r4, r2, r4
 800956e:	eb63 0691 	sbc.w	r6, r3, r1, lsr #2
 8009572:	2e00      	cmp	r6, #0
 8009574:	d403      	bmi.n	800957e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xce>
 8009576:	ea45 059c 	orr.w	r5, r5, ip, lsr #2
 800957a:	d102      	bne.n	8009582 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xd2>
 800957c:	e00e      	b.n	800959c <compiler_builtins::int::specialized_div_rem::u64_div_rem+0xec>
 800957e:	4614      	mov	r4, r2
 8009580:	461e      	mov	r6, r3
 8009582:	ea4f 02d8 	mov.w	r2, r8, lsr #3
 8009586:	ea42 7241 	orr.w	r2, r2, r1, lsl #29
 800958a:	1aa2      	subs	r2, r4, r2
 800958c:	eb66 03d1 	sbc.w	r3, r6, r1, lsr #3
 8009590:	2b00      	cmp	r3, #0
 8009592:	d4c2      	bmi.n	800951a <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x6a>
 8009594:	ea45 05dc 	orr.w	r5, r5, ip, lsr #3
 8009598:	4614      	mov	r4, r2
 800959a:	d1c0      	bne.n	800951e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x6e>
 800959c:	fbb4 f1fe 	udiv	r1, r4, lr
 80095a0:	fb01 461e 	mls	r6, r1, lr, r4
 80095a4:	4329      	orrs	r1, r5
 80095a6:	e092      	b.n	80096ce <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x21e>
 80095a8:	ebb2 060e 	subs.w	r6, r2, lr
 80095ac:	f04f 0100 	mov.w	r1, #0
 80095b0:	eb73 0608 	sbcs.w	r6, r3, r8
 80095b4:	d37c      	bcc.n	80096b0 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x200>
 80095b6:	2b00      	cmp	r3, #0
 80095b8:	d07a      	beq.n	80096b0 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x200>
 80095ba:	fab3 f183 	clz	r1, r3
 80095be:	fab8 f688 	clz	r6, r8
 80095c2:	1a71      	subs	r1, r6, r1
 80095c4:	f001 063f 	and.w	r6, r1, #63	@ 0x3f
 80095c8:	f001 011f 	and.w	r1, r1, #31
 80095cc:	f1c6 0420 	rsb	r4, r6, #32
 80095d0:	fa08 f506 	lsl.w	r5, r8, r6
 80095d4:	fa0e fc06 	lsl.w	ip, lr, r6
 80095d8:	fa2e f404 	lsr.w	r4, lr, r4
 80095dc:	432c      	orrs	r4, r5
 80095de:	f1b6 0520 	subs.w	r5, r6, #32
 80095e2:	bf58      	it	pl
 80095e4:	fa0e f405 	lslpl.w	r4, lr, r5
 80095e8:	f04f 0501 	mov.w	r5, #1
 80095ec:	fa05 f901 	lsl.w	r9, r5, r1
 80095f0:	f04f 0100 	mov.w	r1, #0
 80095f4:	bf58      	it	pl
 80095f6:	f04f 0c00 	movpl.w	ip, #0
 80095fa:	e008      	b.n	800960e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x15e>
 80095fc:	4632      	mov	r2, r6
 80095fe:	462b      	mov	r3, r5
 8009600:	ea4f 161c 	mov.w	r6, ip, lsr #4
 8009604:	ea46 7c04 	orr.w	ip, r6, r4, lsl #28
 8009608:	ea4f 1919 	mov.w	r9, r9, lsr #4
 800960c:	0924      	lsrs	r4, r4, #4
 800960e:	ebb2 060c 	subs.w	r6, r2, ip
 8009612:	eb73 0504 	sbcs.w	r5, r3, r4
 8009616:	d407      	bmi.n	8009628 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x178>
 8009618:	ea41 0109 	orr.w	r1, r1, r9
 800961c:	ebb6 020e 	subs.w	r2, r6, lr
 8009620:	eb75 0208 	sbcs.w	r2, r5, r8
 8009624:	d202      	bcs.n	800962c <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x17c>
 8009626:	e03a      	b.n	800969e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ee>
 8009628:	4616      	mov	r6, r2
 800962a:	461d      	mov	r5, r3
 800962c:	ea5f 0354 	movs.w	r3, r4, lsr #1
 8009630:	ea4f 023c 	mov.w	r2, ip, rrx
 8009634:	1ab2      	subs	r2, r6, r2
 8009636:	eb75 0303 	sbcs.w	r3, r5, r3
 800963a:	d409      	bmi.n	8009650 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1a0>
 800963c:	ebb2 050e 	subs.w	r5, r2, lr
 8009640:	ea41 0159 	orr.w	r1, r1, r9, lsr #1
 8009644:	eb73 0508 	sbcs.w	r5, r3, r8
 8009648:	4616      	mov	r6, r2
 800964a:	461d      	mov	r5, r3
 800964c:	d202      	bcs.n	8009654 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1a4>
 800964e:	e026      	b.n	800969e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ee>
 8009650:	4632      	mov	r2, r6
 8009652:	462b      	mov	r3, r5
 8009654:	ea4f 059c 	mov.w	r5, ip, lsr #2
 8009658:	ea45 7584 	orr.w	r5, r5, r4, lsl #30
 800965c:	1b56      	subs	r6, r2, r5
 800965e:	eb63 0594 	sbc.w	r5, r3, r4, lsr #2
 8009662:	2d00      	cmp	r5, #0
 8009664:	d407      	bmi.n	8009676 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1c6>
 8009666:	ea41 0199 	orr.w	r1, r1, r9, lsr #2
 800966a:	ebb6 020e 	subs.w	r2, r6, lr
 800966e:	eb75 0208 	sbcs.w	r2, r5, r8
 8009672:	d202      	bcs.n	800967a <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ca>
 8009674:	e013      	b.n	800969e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x1ee>
 8009676:	4616      	mov	r6, r2
 8009678:	461d      	mov	r5, r3
 800967a:	ea4f 02dc 	mov.w	r2, ip, lsr #3
 800967e:	ea42 7244 	orr.w	r2, r2, r4, lsl #29
 8009682:	1ab2      	subs	r2, r6, r2
 8009684:	eb65 03d4 	sbc.w	r3, r5, r4, lsr #3
 8009688:	2b00      	cmp	r3, #0
 800968a:	d4b7      	bmi.n	80095fc <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x14c>
 800968c:	ebb2 050e 	subs.w	r5, r2, lr
 8009690:	ea41 01d9 	orr.w	r1, r1, r9, lsr #3
 8009694:	eb73 0508 	sbcs.w	r5, r3, r8
 8009698:	4616      	mov	r6, r2
 800969a:	461d      	mov	r5, r3
 800969c:	d2b0      	bcs.n	8009600 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x150>
 800969e:	f04f 0c00 	mov.w	ip, #0
 80096a2:	e9c0 1c00 	strd	r1, ip, [r0]
 80096a6:	e9c0 6502 	strd	r6, r5, [r0, #8]
 80096aa:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80096ae:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80096b0:	f04f 0c00 	mov.w	ip, #0
 80096b4:	4616      	mov	r6, r2
 80096b6:	461d      	mov	r5, r3
 80096b8:	e9c0 1c00 	strd	r1, ip, [r0]
 80096bc:	e9c0 6502 	strd	r6, r5, [r0, #8]
 80096c0:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80096c4:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80096c6:	fbb2 f1fe 	udiv	r1, r2, lr
 80096ca:	fb01 261e 	mls	r6, r1, lr, r2
 80096ce:	f04f 0c00 	mov.w	ip, #0
 80096d2:	2500      	movs	r5, #0
 80096d4:	e9c0 1c00 	strd	r1, ip, [r0]
 80096d8:	e9c0 6502 	strd	r6, r5, [r0, #8]
 80096dc:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80096e0:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80096e2:	d10d      	bne.n	8009700 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x250>
 80096e4:	fbb2 f1f3 	udiv	r1, r2, r3
 80096e8:	2500      	movs	r5, #0
 80096ea:	fb01 2613 	mls	r6, r1, r3, r2
 80096ee:	f04f 0c01 	mov.w	ip, #1
 80096f2:	e9c0 1c00 	strd	r1, ip, [r0]
 80096f6:	e9c0 6502 	strd	r6, r5, [r0, #8]
 80096fa:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 80096fe:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8009700:	fbb3 fcfe 	udiv	ip, r3, lr
 8009704:	f5be 3f80 	cmp.w	lr, #65536	@ 0x10000
 8009708:	fb0c 351e 	mls	r5, ip, lr, r3
 800970c:	d21a      	bcs.n	8009744 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x294>
 800970e:	0429      	lsls	r1, r5, #16
 8009710:	2500      	movs	r5, #0
 8009712:	ea41 4112 	orr.w	r1, r1, r2, lsr #16
 8009716:	fbb1 f3fe 	udiv	r3, r1, lr
 800971a:	fb03 f10e 	mul.w	r1, r3, lr
 800971e:	ea4c 4c13 	orr.w	ip, ip, r3, lsr #16
 8009722:	ebc1 4112 	rsb	r1, r1, r2, lsr #16
 8009726:	eac2 4101 	pkhbt	r1, r2, r1, lsl #16
 800972a:	fbb1 f2fe 	udiv	r2, r1, lr
 800972e:	fb02 161e 	mls	r6, r2, lr, r1
 8009732:	ea42 4103 	orr.w	r1, r2, r3, lsl #16
 8009736:	e9c0 1c00 	strd	r1, ip, [r0]
 800973a:	e9c0 6502 	strd	r6, r5, [r0, #8]
 800973e:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 8009742:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8009744:	ebb2 010e 	subs.w	r1, r2, lr
 8009748:	eb75 0108 	sbcs.w	r1, r5, r8
 800974c:	d208      	bcs.n	8009760 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2b0>
 800974e:	2100      	movs	r1, #0
 8009750:	4616      	mov	r6, r2
 8009752:	e9c0 1c00 	strd	r1, ip, [r0]
 8009756:	e9c0 6502 	strd	r6, r5, [r0, #8]
 800975a:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 800975e:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8009760:	ea4f 71c8 	mov.w	r1, r8, lsl #31
 8009764:	ea41 035e 	orr.w	r3, r1, lr, lsr #1
 8009768:	ea4f 79ce 	mov.w	r9, lr, lsl #31
 800976c:	f04f 4800 	mov.w	r8, #2147483648	@ 0x80000000
 8009770:	2100      	movs	r1, #0
 8009772:	e008      	b.n	8009786 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2d6>
 8009774:	4622      	mov	r2, r4
 8009776:	4635      	mov	r5, r6
 8009778:	ea4f 1419 	mov.w	r4, r9, lsr #4
 800977c:	ea44 7903 	orr.w	r9, r4, r3, lsl #28
 8009780:	ea4f 1818 	mov.w	r8, r8, lsr #4
 8009784:	091b      	lsrs	r3, r3, #4
 8009786:	ebb2 0409 	subs.w	r4, r2, r9
 800978a:	eb65 0603 	sbc.w	r6, r5, r3
 800978e:	2e00      	cmp	r6, #0
 8009790:	d403      	bmi.n	800979a <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2ea>
 8009792:	ea41 0108 	orr.w	r1, r1, r8
 8009796:	d102      	bne.n	800979e <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2ee>
 8009798:	e02d      	b.n	80097f6 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x346>
 800979a:	4614      	mov	r4, r2
 800979c:	462e      	mov	r6, r5
 800979e:	ea5f 0553 	movs.w	r5, r3, lsr #1
 80097a2:	ea4f 0239 	mov.w	r2, r9, rrx
 80097a6:	1aa2      	subs	r2, r4, r2
 80097a8:	eb66 0505 	sbc.w	r5, r6, r5
 80097ac:	2d00      	cmp	r5, #0
 80097ae:	d404      	bmi.n	80097ba <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x30a>
 80097b0:	ea41 0158 	orr.w	r1, r1, r8, lsr #1
 80097b4:	4614      	mov	r4, r2
 80097b6:	d102      	bne.n	80097be <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x30e>
 80097b8:	e01d      	b.n	80097f6 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x346>
 80097ba:	4622      	mov	r2, r4
 80097bc:	4635      	mov	r5, r6
 80097be:	ea4f 0499 	mov.w	r4, r9, lsr #2
 80097c2:	ea44 7483 	orr.w	r4, r4, r3, lsl #30
 80097c6:	1b14      	subs	r4, r2, r4
 80097c8:	eb65 0693 	sbc.w	r6, r5, r3, lsr #2
 80097cc:	2e00      	cmp	r6, #0
 80097ce:	d403      	bmi.n	80097d8 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x328>
 80097d0:	ea41 0198 	orr.w	r1, r1, r8, lsr #2
 80097d4:	d102      	bne.n	80097dc <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x32c>
 80097d6:	e00e      	b.n	80097f6 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x346>
 80097d8:	4614      	mov	r4, r2
 80097da:	462e      	mov	r6, r5
 80097dc:	ea4f 02d9 	mov.w	r2, r9, lsr #3
 80097e0:	ea42 7243 	orr.w	r2, r2, r3, lsl #29
 80097e4:	1aa2      	subs	r2, r4, r2
 80097e6:	eb66 05d3 	sbc.w	r5, r6, r3, lsr #3
 80097ea:	2d00      	cmp	r5, #0
 80097ec:	d4c2      	bmi.n	8009774 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2c4>
 80097ee:	ea41 01d8 	orr.w	r1, r1, r8, lsr #3
 80097f2:	4614      	mov	r4, r2
 80097f4:	d1c0      	bne.n	8009778 <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x2c8>
 80097f6:	fbb4 f2fe 	udiv	r2, r4, lr
 80097fa:	2500      	movs	r5, #0
 80097fc:	fb02 461e 	mls	r6, r2, lr, r4
 8009800:	4311      	orrs	r1, r2
 8009802:	e9c0 1c00 	strd	r1, ip, [r0]
 8009806:	e9c0 6502 	strd	r6, r5, [r0, #8]
 800980a:	e8bd 0b00 	ldmia.w	sp!, {r8, r9, fp}
 800980e:	bdf0      	pop	{r4, r5, r6, r7, pc}

08009810 <HardFaultTrampoline>:
 8009810:	4670      	mov	r0, lr
 8009812:	2104      	movs	r1, #4
 8009814:	4208      	tst	r0, r1
 8009816:	d103      	bne.n	8009820 <HardFaultTrampoline+0x10>
 8009818:	f3ef 8008 	mrs	r0, MSP
 800981c:	f000 b804 	b.w	8009828 <HardFault_>
 8009820:	f3ef 8009 	mrs	r0, PSP
 8009824:	f000 b800 	b.w	8009828 <HardFault_>

08009828 <HardFault_>:
 8009828:	b580      	push	{r7, lr}
 800982a:	466f      	mov	r7, sp
 800982c:	e7fe      	b.n	800982c <HardFault_+0x4>
 800982e:	d4d4      	bmi.n	80097da <compiler_builtins::int::specialized_div_rem::u64_div_rem+0x32a>
