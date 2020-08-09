import sys
import socket
import struct
import threading
import time

try:
    addr = sys.argv[1]
    port = int(sys.argv[2])
except (ValueError, IndexError):
    print('usage: test.py ipaddr port')
    sys.exit(1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', port))
sock.sendto(b'...', (addr, port))


class result:
    npackets = 0
    nbytes = 0
    endtime = 0
    rtstart = 0

    @classmethod
    def out(cls):
        endtime = time.time() - cls.rtstart
        print('[%9f] got: %s packets => %.1f MBit/s' %
              (endtime, cls.npackets, 8 * cls.nbytes / float(endtime * 1e6)))


def get():
    lastno = None
    while True:
        data, _ = sock.recvfrom(2000)
        seqno, time = struct.unpack('<IQ', data[:12])
        result.endtime = time
        result.npackets += 1
        result.nbytes += len(data) + 66  # consider Eth/IP/UDP headers
        if not (lastno is None or seqno == lastno + 1):
            print("!!! missing packet: %s %s" % (seqno, lastno))
        lastno = seqno


def periodic_out():
    while True:
        time.sleep(0.5)
        result.out()


periodic_thread = threading.Thread(target=periodic_out)
periodic_thread.daemon = True
result.rtstart = time.time()
periodic_thread.start()
get()
