#!/usr/bin/env python3
"""PSP LCDC Capture bridge.

Video hot path is intentionally receive-only: EP81/EP82 halves are paired by
sequence and forwarded to the browser in native 512x272 PSP stride without
cropping, pixel conversion, or full-frame concatenation in Python.

/video and /audio use separate WebSockets so large video frames never sit in
front of low-latency audio.  The local OBS page owns visible 480px cropping,
pixel conversion, and latest-only presentation on browser animation ticks.
"""
from __future__ import annotations
import argparse
import base64
from collections import deque
import hashlib
from pathlib import Path
import socket
import struct
import threading
import time

VID, PID = 0x054C, 0x01C9
EP1, EP2, IFACE = 0x81, 0x82, 0
MAGIC = 0x3036434C
TAG = 0x324D5246  # FRM2
VERSION = 2
HEADER = struct.Struct('<IIIII BBH')
assert HEADER.size == 24
USB_W, H = 512, 272
BYTES16, BYTES32 = USB_W * H * 2, USB_W * H * 4
USB_EP_READ_CAP = BYTES32 // 2 + 64
AUDIO_MAGIC, AUDIO_VERSION = 0x31445541, 1
AUDIO_WIRE = struct.Struct('<IIIHBBHHHH')
assert AUDIO_WIRE.size == 24
AUDIO_RX_CAP = AUDIO_WIRE.size + 512 * 4 + 64
AUDIO_WS_MARKER = 0xA0
_AUDIO_MARKER_BYTE = bytes((AUDIO_WS_MARKER,))
HERE = Path(__file__).resolve().parent
WS_AUDIO_MAX_AGE_S = 0.14
WS_HOST, WS_PORT = '127.0.0.1', 8766

latest_lock = threading.Condition()
latest_packet = None
latest_seq = -1
# Keep at most roughly 100-160 ms of audio, never a full second of stale PCM.
audio_queue = deque(maxlen=20)
audio_seq = 0
stop_flag = False


def parse_header(data: bytes, endpoint: int):
    if len(data) != HEADER.size:
        return None
    magic, tag, seq, total, p0, fmt, index, version = HEADER.unpack(data)
    expected = BYTES32 if fmt == 3 else BYTES16
    if (magic != MAGIC or tag != TAG or version != VERSION or fmt > 3
            or total != expected or p0 != expected // 2 - 64
            or index != (1 if endpoint == EP1 else 2)):
        return None
    return {'seq': seq, 'total': total, 'p0': p0, 'fmt': fmt}


def is_audio_packet(data: bytes):
    if len(data) < AUDIO_WIRE.size:
        return False
    magic, sequence, tick, frames, source, channel, lv, rv, flags, version = AUDIO_WIRE.unpack_from(data)
    if (magic != AUDIO_MAGIC or version != AUDIO_VERSION or source not in (1, 2)
            or not 1 <= frames <= 512 or flags & ~1
            or (source == 1 and (channel != 255 or flags != 0))
            or (source == 2 and channel > 10)):
        return False
    expected = AUDIO_WIRE.size + frames * (2 if flags & 1 else 4)
    return len(data) == expected


def data_lengths(fmt: int):
    total = BYTES32 if fmt == 3 else BYTES16
    first = total // 2 - 64
    return first, total - first




class Reassembler:
    """Per-endpoint FIFO headers, seq-based pairing and bounded storage."""
    def __init__(self):
        self.headerq = {EP1: deque(), EP2: deque()}
        self.partial = {}
        self.completed = 0
        self.discarded = 0
        self.last_seq = None
        self.errors = 0
        self.bytes_rx = 0
        self.frames_by_fmt = [0] * 4

    def feed(self, ep: int, payload: bytes):
        if ep not in (EP1, EP2):
            self.errors += 1
            return None
        self.bytes_rx += len(payload)
        if len(payload) == HEADER.size:
            hdr = parse_header(payload, ep)
            if hdr is None:
                self.errors += 1
                return None
            self.headerq[ep].append(hdr)
            return None
        if not self.headerq[ep]:
            self.errors += 1
            return None
        hdr = self.headerq[ep].popleft()
        first, second = data_lengths(hdr['fmt'])
        if len(payload) != (first if ep == EP1 else second):
            self.errors += 1
            self.partial.pop(hdr['seq'], None)
            return None
        seq = hdr['seq']
        parts = self.partial.setdefault(seq, {})
        if 'fmt' in parts and parts['fmt'] != hdr['fmt']:
            self.errors += 1
            del self.partial[seq]
            return None
        parts['fmt'] = hdr['fmt']
        parts[ep] = payload
        # In case a malformed/late endpoint disappears, do not grow indefinitely.
        if len(self.partial) > 12:
            oldest = next(iter(self.partial))
            self.partial.pop(oldest, None)
            self.discarded += 1
        if EP1 not in parts or EP2 not in parts:
            return None
        del self.partial[seq]
        if self.last_seq is not None and ((seq - self.last_seq) & 0xFFFFFFFF) > 0x80000000:
            self.discarded += 1
            return None
        self.last_seq = seq
        self.completed += 1
        self.frames_by_fmt[hdr['fmt']] += 1
        # Keep endpoint halves separate. WebSocket framing can stream one logical
        # message across several TCP writes, so Python never needs a full-frame
        # concatenate/crop copy here.
        return seq, hdr['fmt'], parts[EP1], parts[EP2]


def open_usb(ctx):
    h = None
    try:
        h = ctx.openByVendorIDAndProductID(VID, PID, skip_on_error=True)
        if h is None:
            return None
        try:
            h.setAutoDetachKernelDriver(True)
        except Exception:
            pass
        h.claimInterface(IFACE)
        eps = {e.getAddress() for conf in h.getDevice().iterConfigurations()
               for interface in conf.iterInterfaces()
               for setting in interface.iterSettings()
               for e in setting.iterEndpoints()}
        if EP1 not in eps or EP2 not in eps:
            raise RuntimeError(f'EP81/82 required; found {sorted(eps)}')
        return h
    except Exception:
        if h is not None:
            try: h.close()
            except Exception: pass
        raise


def close_usb(h):
    if h is None:
        return
    try: h.releaseInterface(IFACE)
    except Exception: pass
    try: h.close()
    except Exception: pass


def usb_worker(depth=8):
    global latest_seq, latest_packet, audio_seq
    import usb1
    with usb1.USBContext() as ctx:
        while not stop_flag:
            h = None
            try:
                try:
                    h = open_usb(ctx)
                except Exception as exc:
                    print('USB open:', exc)
                    time.sleep(0.5)
                    continue
                if h is None:
                    time.sleep(0.25)
                    continue
                print('USB CONNECTED: EP81+EP82 video; AUD1 multiplexed on EP82')
                with latest_lock:
                    audio_queue.clear()
                    latest_lock.notify_all()
                rx = Reassembler()
                transfers = []
                alive = [True]
                stat_t = [time.perf_counter()]
                stat_n = [0]
                stat_bytes = [0]
                audio_stat_packets = [0]
                audio_stat_bytes = [0]

                def done(t):
                    global latest_seq, latest_packet, audio_seq
                    state = t.getStatus()
                    if state == usb1.TRANSFER_COMPLETED:
                        size = t.getActualLength()
                        data = bytes(t.getBuffer()[:size])
                        # EP82 carries both ordinary video transfers and AUD1 packets.
                        # Audio packets are self-framing short Bulk requests, so peel them
                        # off before passing the transfer to the video reassembler.
                        audio_item = False
                        if t.getEndpoint() == EP2 and size <= AUDIO_RX_CAP and size >= AUDIO_WIRE.size:
                            try:
                                if struct.unpack_from('<I', data, 0)[0] == AUDIO_MAGIC:
                                    audio_item = is_audio_packet(data)
                            except (struct.error, ValueError):
                                audio_item = False
                        if audio_item:
                            with latest_lock:
                                audio_seq += 1
                                audio_queue.append((audio_seq, time.perf_counter(), data))
                                latest_lock.notify_all()
                            audio_stat_packets[0] += 1
                            audio_stat_bytes[0] += size
                            item = None
                        else:
                            item = rx.feed(t.getEndpoint(), data)
                            stat_bytes[0] += size
                            if item is not None:
                                seq, fmt, first_half, second_half = item
                                with latest_lock:
                                    latest_seq = seq
                                    latest_packet = (fmt, first_half, second_half)
                                    latest_lock.notify_all()
                                stat_n[0] += 1
                        now = time.perf_counter()
                        span = now - stat_t[0]
                        if span >= 1.0:
                            fps = stat_n[0] / span
                            mb = stat_bytes[0] / span / 1e6
                            print(f'\rUSB {fps:5.1f} fps  {mb:5.2f} MB/s  '
                                  f'32bpp={rx.frames_by_fmt[3]} total={rx.completed} '
                                  f'errors={rx.errors} dropped={rx.discarded}  audio={audio_stat_packets[0]}/s ({audio_stat_bytes[0]/span/1e6:.3f} MB/s)',
                                  end='', flush=True)
                            stat_t[0] = now
                            stat_n[0] = stat_bytes[0] = 0
                            audio_stat_packets[0] = audio_stat_bytes[0] = 0
                    elif state == usb1.TRANSFER_TIMED_OUT:
                        pass
                    elif state != usb1.TRANSFER_CANCELLED:
                        alive[0] = False
                        print(f'\nUSB transfer status={state}; reconnect')
                    if alive[0] and not stop_flag:
                        try: t.submit()
                        except usb1.USBError as exc:
                            alive[0] = False
                            print('\nUSB submit:', exc)

                for ep in (EP1, EP2):
                    for _ in range(depth):
                        t = h.getTransfer()
                        t.setBulk(ep, USB_EP_READ_CAP, callback=done, timeout=1000)
                        transfers.append(t)
                for t in transfers: t.submit()
                try:
                    while alive[0] and not stop_flag:
                        try: ctx.handleEvents()
                        except usb1.USBErrorInterrupted: pass
                        except usb1.USBError as exc:
                            print('\nUSB events:', exc)
                            break
                finally:
                    alive[0] = False
                    for t in transfers:
                        if t.isSubmitted():
                            try:t.cancel()
                            except usb1.USBError:pass
                    deadline = time.perf_counter() + 1.0
                    while any(t.isSubmitted() for t in transfers) and time.perf_counter() < deadline:
                        try:ctx.handleEvents()
                        except usb1.USBError:break
            finally:
                close_usb(h)
            time.sleep(0.25)


def ws_accept_value(key):
    sha = hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()
    return base64.b64encode(sha).decode()


def ws_header(n):
    if n < 126: return bytes((0x82, n))
    if n < 65536: return bytes((0x82,126)) + struct.pack('!H',n)
    return bytes((0x82,127)) + struct.pack('!Q',n)


_FMT_BYTE = (b"\x00", b"\x01", b"\x02", b"\x03")

def send_native_video(conn, packet):
    """Send one native-stride frame without concatenating its two USB halves."""
    fmt, first_half, second_half = packet
    total = 1 + len(first_half) + len(second_half)
    conn.sendall(ws_header(total))
    conn.sendall(_FMT_BYTE[fmt])
    conn.sendall(first_half)
    conn.sendall(second_half)

def send_audio_packet(conn, packet):
    """Send AUD1 marker + packet as one WebSocket message without concatenation."""
    conn.sendall(ws_header(1 + len(packet)))
    conn.sendall(_AUDIO_MARKER_BYTE)
    conn.sendall(packet)


def ws_client(conn, addr):
    """One WebSocket per medium; no audio head-of-line behind video frames."""
    try:
        conn.settimeout(5.0)
        req = bytearray()
        while b'\r\n\r\n' not in req and len(req) < 16384:
            buf = conn.recv(4096)
            if not buf:
                return
            req.extend(buf)
        header = req.decode('latin1', errors='replace')
        path = header.split(' ', 2)[1].split('?', 1)[0] if header.startswith('GET ') else ''
        if path not in ('/video', '/audio'):
            conn.sendall(b'HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n')
            return
        key = next((line.split(':',1)[1].strip() for line in header.split('\r\n')
                    if line.lower().startswith('sec-websocket-key:')), None)
        if key is None:
            return
        resp = ('HTTP/1.1 101 Switching Protocols\r\n'
                'Upgrade: websocket\r\nConnection: Upgrade\r\n'
                f'Sec-WebSocket-Accept: {ws_accept_value(key)}\r\n\r\n').encode()
        conn.sendall(resp)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # Separate streams prevent large video frames from blocking PCM.
        stream = 'audio' if path == '/audio' else 'video'
        conn.settimeout(0.35 if stream == 'audio' else None)
        print(f'\nOBS {stream.upper()} CONNECTED {addr}')
        if stream == 'video':
            sent = -1
            while not stop_flag:
                with latest_lock:
                    latest_lock.wait_for(lambda: stop_flag or
                         (latest_packet is not None and latest_seq != sent), timeout=0.5)
                    if stop_flag:
                        break
                    seq, packet = latest_seq, latest_packet
                if packet is not None and seq != sent:
                    send_native_video(conn, packet)
                    sent = seq
        elif stream == 'audio':
            sent = audio_seq
            while not stop_flag:
                with latest_lock:
                    latest_lock.wait_for(lambda: stop_flag or audio_seq != sent,
                                         timeout=0.5)
                    if stop_flag:
                        break
                    newest_seq = audio_seq
                    cutoff = time.perf_counter() - WS_AUDIO_MAX_AGE_S
                    pending = [(s, p) for s, recv_at, p in audio_queue
                               if s > sent and recv_at >= cutoff]
                # Always advance past missed packets: audio must stay near live.
                sent = newest_seq
                for _, pcm_packet in pending:
                    send_audio_packet(conn, pcm_packet)
    except (ConnectionError, OSError) as exc:
        print(f'\nOBS {locals().get("stream", "unknown")} WS error: {type(exc).__name__}: {exc}')
    except Exception as exc:
        print(f'\nOBS WS internal error: {type(exc).__name__}: {exc}')
    finally:
        try:
            conn.close()
        except Exception:
            pass
        print('\nOBS STREAM DISCONNECTED')


def ws_server():
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        server.bind((WS_HOST,WS_PORT))
        server.listen(4)
        server.settimeout(1)
        while not stop_flag:
            try:conn,addr=server.accept()
            except socket.timeout:continue
            threading.Thread(target=ws_client,args=(conn,addr),daemon=True).start()


def main():
    global stop_flag
    parser = argparse.ArgumentParser()
    parser.add_argument('--depth', type=int, choices=[2,4,8], default=8)
    args = parser.parse_args()

    print('PSP LCDC Capture')
    print('OBS LOCAL FILE:', HERE/'obs.html')

    # Only the WebSocket port is required. obs.html is a local file.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        try:
            probe.bind((WS_HOST, WS_PORT))
        except OSError as exc:
            parser.error(f'{WS_HOST}:{WS_PORT} busy — close other bridge instances first ({exc})')

    threading.Thread(target=ws_server, daemon=True).start()
    threading.Thread(target=usb_worker, args=(args.depth,), daemon=True).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_flag=True
        with latest_lock:
            latest_lock.notify_all()
        print('\nStopped')


if __name__ == '__main__':
    main()
