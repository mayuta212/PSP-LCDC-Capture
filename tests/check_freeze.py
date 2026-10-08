from pathlib import Path
import importlib.util
import socket
import struct
import threading

root = Path(__file__).resolve().parents[1]
main = (root/'psp/main.c').read_text()
audio = (root/'psp/audio_capture.c').read_text()
ah = (root/'psp/audio_capture.h').read_text()
pops = (root/'psp/audio_pops.c').read_text()
bridge = (root/'pc/bridge.py').read_text(encoding='utf-8')
obs = (root/'pc/obs.html').read_text(encoding='utf-8')
make = (root/'psp/Makefile').read_text()

# Frozen PSP policy.
assert 'PSP_LCDC_CAPTURE_20261008' in main
assert 'LCDC60_BANK_WAIT_US' not in main and 'BANK_WAIT_US' not in make
assert 'sceIoOpen' not in main and 'LCDC60_AV.LOG' not in main and 'trace_value' not in main
assert 'sceDisplayGetCurrentHcount' not in main and 'LCDC60_LATE_HCOUNT_MAX' not in main
assert 'PDIAPP' not in main
assert '"LCDC60AudioUSB",audio_thread,0x30,0x2000' in main
assert '"LCDC60Dual32",capture_thread,0x38,0x4000' in main
assert '"LCDC60VideoSend",video_sender_thread,0x3C,0x1800' in main
assert 'g_ready_bank' in main and 'game_store_ready' in main
assert 'max_attempts=(g_mode==LCDC_MODE_GAME)?1:2' in main
assert 'audio_waiting_packets()==0 && game_claim_direct_submit()' in main
assert 'sctrlHENSetStartModuleHandler(lcdc60_start_module_handler)' in main
assert 'g_audio_rescan_pending=1' in main

# A02 coverage remains broad.
for name in ['NID_OUTPUT','NID_OUTPUT_BLOCKING','NID_PANNED','NID_PANNED_BLOCKING',
             'NID_OUTPUT2_BLOCKING','NID_SRC_BLOCKING','NID_VAUDIO_BLOCKING']:
    assert name in audio
assert 'install_unique' in audio and 'g_hook_mask' in audio
assert 'audio_waiting_packets(void)' in audio and 'u32 audio_waiting_packets(void);' in ah

# Production cleanup: diagnostic-only exported counters are gone.
for dead in ['audio_psp_hooks_ready','audio_psp_scan_count','audio_psp_alias_conflicts',
             'audio_psp_path_events','audio_sent(void)','audio_lost(void)',
             'audio_pops_diag','audio_pops_lost','audio_pops_samples','audio_pops_registrations']:
    assert dead not in main + audio + ah + pops


# POPS frozen hot path keeps only registration state + PCM ring state.
for dead in ['g_diag_', 'sample_calls', 'last_sample', 'or_accum', 'xor_accum', 'wrapper_cb']:
    assert dead not in pops
assert 'volatile uint32_t ring_index;' in pops
assert 'volatile uint32_t samples[SAMPLE_RING];' in pops
assert '0x8D0A0008U' in pops and '0xAD62000CU' in pops and '0xAD0A0008U' in pops

# PC is local-file + two websocket streams only.
assert 'http.server' not in bridge and 'HTTP_PORT' not in bridge and 'OBS_HTML' not in bridge
assert "path not in ('/video', '/audio')" in bridge
assert "path == '/'" not in bridge and "stream == 'mixed'" not in bridge
assert 'LCDC60_AV_PC.log' not in bridge and 'log_event' not in bridge
assert 'def send_native_video' in bridge
assert 'conn.sendall(first_half)' in bridge and 'conn.sendall(second_half)' in bridge
assert 'def send_audio_packet' in bridge and 'conn.sendall(_AUDIO_MARKER_BYTE)' in bridge
assert 'make_obs_packet' not in bridge
assert 'const USB_W = 512, W = 480' in obs
assert 'ws.onmessage = e => acceptVideo(e.data);' in obs
assert 'requestAnimationFrame(presentVideo);' in obs
assert 'const LEAD_MS = 0, MAX_AHEAD_MS = 36;' in obs
assert 'const PSP_LEAD_MS = 12, PSP_MAX_AHEAD_MS = 64;' in obs

# Cheap structural sanity for edited C sources.
def balanced(text, a, b):
    depth = 0
    i = 0
    in_str = in_chr = in_line = in_block = False
    esc = False
    while i < len(text):
        c = text[i]
        n = text[i+1] if i+1 < len(text) else ''
        if in_line:
            if c == '\n': in_line = False
        elif in_block:
            if c == '*' and n == '/': in_block = False; i += 1
        elif in_str:
            if esc: esc = False
            elif c == '\\': esc = True
            elif c == '"': in_str = False
        elif in_chr:
            if esc: esc = False
            elif c == '\\': esc = True
            elif c == "'": in_chr = False
        else:
            if c == '/' and n == '/': in_line = True; i += 1
            elif c == '/' and n == '*': in_block = True; i += 1
            elif c == '"': in_str = True
            elif c == "'": in_chr = True
            elif c == a: depth += 1
            elif c == b:
                depth -= 1
                if depth < 0: return False
        i += 1
    return depth == 0 and not in_block and not in_str and not in_chr

for text in (main, audio, pops):
    assert balanced(text, '{', '}')
    assert balanced(text, '(', ')')

# WebSocket fragmented-send helpers must still form exactly one binary message.
spec = importlib.util.spec_from_file_location('lcdc_bridge', root/'pc/bridge.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

def recv_exact(sock, n):
    out = bytearray()
    while len(out) < n:
        part = sock.recv(n-len(out))
        assert part
        out.extend(part)
    return bytes(out)

def recv_ws(sock):
    h = recv_exact(sock, 2)
    assert h[0] == 0x82
    n = h[1] & 0x7f
    if n == 126: n = struct.unpack('!H', recv_exact(sock, 2))[0]
    elif n == 127: n = struct.unpack('!Q', recv_exact(sock, 8))[0]
    return recv_exact(sock, n)

s1, s2 = socket.socketpair()
try:
    th = threading.Thread(target=mod.send_native_video, args=(s1, (3, b'abc', b'defg')))
    th.start(); assert recv_ws(s2) == b'\x03abcdefg'; th.join()
    th = threading.Thread(target=mod.send_audio_packet, args=(s1, b'AUD1test'))
    th.start(); assert recv_ws(s2) == b'\xA0AUD1test'; th.join()
finally:
    s1.close(); s2.close()

print('PSP LCDC Capture static checks PASS')
