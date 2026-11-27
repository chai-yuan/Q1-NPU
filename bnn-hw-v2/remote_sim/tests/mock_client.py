#!/usr/bin/env python3
"""Minimal SoC MMIO TCP client for bnn-hw-v2/remote_sim.

Drives bnn_soc_top exactly like an ARM host processor on Zynq: verifies CORE_ID,
configures DATA_BASE, writes 64-bit instructions via CMD_LO / CMD_HI, triggers
CTRL_STATUS.LAUNCH, and polls CTRL_STATUS.DONE while serving the simulator's
64-bit AXI4 DMA memory traffic.

Wire format (little-endian, 16-byte header + payload):
    magic  u32 = 0x424E4E53 ("BNNS")
    type   u8
    flags  u8
    seq    u16
    addr   u32
    length u32
"""

import argparse
import os
import socket
import struct
import subprocess
import sys
import time

MAGIC = 0x424E4E53
HEADER = struct.Struct("<IBBHII")

TYPE_READ = 0x02
TYPE_WRITE = 0x03
TYPE_QUIT = 0x04
TYPE_READ_BURST = 0x05
TYPE_MMIO_WRITE = 0x10
TYPE_MMIO_READ = 0x11
TYPE_READ_RSP = 0x81
TYPE_QUIT_ACK = 0x83
TYPE_READ_BURST_RSP = 0x85
TYPE_MMIO_WRITE_RSP = 0x90
TYPE_MMIO_READ_RSP = 0x91

# bnn_soc_ctrl MMIO register map
REG_CTRL_STATUS = 0x00
REG_CYCLE_CNT = 0x04
REG_CMD_LO = 0x08
REG_CMD_HI = 0x0C
REG_DATA_BASE_LO = 0x10
REG_DATA_BASE_HI = 0x14
REG_CMD_DONE_CNT = 0x18
REG_ERR_CODE = 0x1C
REG_CORE_ID = 0x20

CORE_MAGIC_ID = 0x424E4E32  # "BNN2"
CTRL_LAUNCH = 1 << 0
CTRL_DONE = 1 << 1
CTRL_ERROR = 1 << 3
CTRL_SOFT_RESET = 1 << 4


class ProtocolError(RuntimeError):
    pass


def recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise EOFError("connection closed by simulator")
        data += chunk
    return bytes(data)


def recv_message(sock):
    header = recv_exact(sock, HEADER.size)
    magic, mtype, flags, seq, addr, length = HEADER.unpack(header)
    if magic != MAGIC:
        raise ProtocolError(f"bad magic 0x{magic:08x}")
    return {
        "type": mtype,
        "flags": flags,
        "seq": seq,
        "addr": addr,
        "length": length,
        "payload": recv_exact(sock, length) if length else b"",
    }


def send_message(sock, mtype, seq=0, addr=0, flags=0, payload=b""):
    sock.sendall(HEADER.pack(MAGIC, mtype, flags, seq, addr, len(payload)) + payload)


def parse_commands(path):
    commands = []
    with open(path, "r", encoding="ascii") as stream:
        for line in stream:
            token = line.split("#", 1)[0].strip()
            if not token:
                continue
            if token[:2].lower() == "0x":
                token = token[2:]
            commands.append(int(token, 16))
    return commands


def load_memory(path):
    with open(path, "rb") as stream:
        image = stream.read()
    memory = bytearray(max(1 << 20, (len(image) + 7) & ~7))
    memory[: len(image)] = image
    return memory, len(image)


def choose_port():
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def start_server(server, port):
    process = subprocess.Popen(
        [server, "--port", str(port)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    for line in process.stdout:
        if "listening on" in line:
            return process
        if not line.strip():
            continue
        print(f"server: {line.rstrip()}")
    output = process.stdout.read()
    process.wait(timeout=5)
    raise RuntimeError(
        f"simulator exited before listening (code {process.returncode}): {output}"
    )


def connect(port, timeout=10.0):
    deadline = time.monotonic() + timeout
    while True:
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=5.0)
            sock.settimeout(60.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            return sock
        except OSError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.05)


def apply_write(memory, message, data_base, highest):
    address = message["addr"]
    if address < data_base:
        raise ProtocolError(f"write below data_base at 0x{address:x}")
    offset = address - data_base
    payload = message["payload"]
    if offset + len(payload) > len(memory):
        raise ProtocolError(f"write beyond memory at 0x{address:x}")
    for index, value in enumerate(payload):
        if message["flags"] & (1 << index):
            memory[offset + index] = value
    return max(highest, offset + len(payload))


def serve_memory_message(sock, memory, data_base, message, highest):
    if message["type"] == TYPE_READ:
        address = message["addr"]
        if address < data_base:
            raise ProtocolError(f"read below data_base at 0x{address:x}")
        offset = address - data_base
        length = 8
        if offset + length > len(memory):
            raise ProtocolError(f"read beyond memory at 0x{address:x}")
        send_message(
            sock,
            TYPE_READ_RSP,
            seq=message["seq"],
            addr=address,
            payload=bytes(memory[offset : offset + length]),
        )
        return highest
    if message["type"] == TYPE_READ_BURST:
        if len(message["payload"]) != 4:
            raise ProtocolError("burst read request without a byte count")
        address = message["addr"]
        if address < data_base:
            raise ProtocolError(f"burst read below data_base at 0x{address:x}")
        offset = address - data_base
        length = struct.unpack("<I", message["payload"])[0]
        if length == 0 or offset + length > len(memory):
            raise ProtocolError(f"burst read beyond memory at 0x{address:x}")
        send_message(
            sock,
            TYPE_READ_BURST_RSP,
            seq=message["seq"],
            addr=address,
            payload=bytes(memory[offset : offset + length]),
        )
        return highest
    if message["type"] == TYPE_WRITE:
        return apply_write(memory, message, data_base, highest)
    raise ProtocolError(f"unexpected message type 0x{message['type']:02x}")


class MmioSession:
    def __init__(self, sock, memory, data_base=0):
        self.sock = sock
        self.memory = memory
        self.data_base = data_base
        self.highest = 0
        self.seq = 0

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    def write32(self, offset, value, wstrb=0xF):
        seq = self.next_seq()
        send_message(
            self.sock,
            TYPE_MMIO_WRITE,
            seq=seq,
            addr=offset,
            flags=wstrb & 0xF,
            payload=struct.pack("<I", value & 0xFFFFFFFF),
        )
        while True:
            msg = recv_message(self.sock)
            if msg["type"] == TYPE_MMIO_WRITE_RSP:
                if msg["seq"] != seq:
                    raise ProtocolError(f"MMIO write seq mismatch: {msg['seq']} != {seq}")
                if msg["flags"] != 0:
                    raise ProtocolError(f"MMIO write BRESP error: {msg['flags']}")
                return
            self.highest = serve_memory_message(
                self.sock, self.memory, self.data_base, msg, self.highest
            )

    def read32(self, offset):
        seq = self.next_seq()
        send_message(self.sock, TYPE_MMIO_READ, seq=seq, addr=offset)
        while True:
            msg = recv_message(self.sock)
            if msg["type"] == TYPE_MMIO_READ_RSP:
                if msg["seq"] != seq:
                    raise ProtocolError(f"MMIO read seq mismatch: {msg['seq']} != {seq}")
                if msg["flags"] != 0:
                    raise ProtocolError(f"MMIO read RRESP error: {msg['flags']}")
                if len(msg["payload"]) != 4:
                    raise ProtocolError("MMIO read response missing 4-byte payload")
                return struct.unpack("<I", msg["payload"])[0]
            self.highest = serve_memory_message(
                self.sock, self.memory, self.data_base, msg, self.highest
            )

    def quit(self):
        send_message(self.sock, TYPE_QUIT)
        while True:
            msg = recv_message(self.sock)
            if msg["type"] == TYPE_QUIT_ACK:
                return
            self.highest = serve_memory_message(
                self.sock, self.memory, self.data_base, msg, self.highest
            )


def run(sock, commands, memory, data_base=0):
    """Drive bnn_soc_top via AXI4-Lite MMIO registers and verify completion."""
    session = MmioSession(sock, memory, data_base=data_base)

    core_id = session.read32(REG_CORE_ID)
    if core_id != CORE_MAGIC_ID:
        raise ProtocolError(f"unexpected CORE_ID 0x{core_id:08x}")

    session.write32(REG_CTRL_STATUS, CTRL_SOFT_RESET)
    session.write32(REG_DATA_BASE_LO, data_base & 0xFFFFFFFF)
    session.write32(REG_DATA_BASE_HI, (data_base >> 32) & 0xFFFFFFFF)

    for index, inst in enumerate(commands):
        session.write32(REG_CMD_LO, inst & 0xFFFFFFFF)
        session.write32(REG_CMD_HI, (inst >> 32) & 0xFFFFFFFF)
        session.write32(REG_CTRL_STATUS, CTRL_LAUNCH)
        while True:
            status = session.read32(REG_CTRL_STATUS)
            if status & CTRL_ERROR:
                err_code = session.read32(REG_ERR_CODE)
                raise ProtocolError(
                    f"command {index}: SoC reported ERROR (ERR_CODE=0x{err_code:x})"
                )
            if status & CTRL_DONE:
                break

    done_cnt = session.read32(REG_CMD_DONE_CNT)
    if done_cnt != len(commands):
        raise ProtocolError(
            f"CMD_DONE_CNT mismatch: got {done_cnt}, expected {len(commands)}"
        )
    hw_cycles = session.read32(REG_CYCLE_CNT)
    print(f"soc_ctrl: CORE_ID=0x{core_id:08x}, CMD_DONE_CNT={done_cnt}, CYCLE_CNT={hw_cycles}")

    session.quit()
    return session.highest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", required=True, help="path to the remote_sim binary")
    parser.add_argument("--port", type=int, default=0, help="TCP port (0 = pick free)")
    parser.add_argument("--commands", required=True, help="commands.hex")
    parser.add_argument("--memory", required=True, help="initial memory image")
    parser.add_argument("--output", required=True, help="output memory image")
    parser.add_argument("--reference", help="optional reference memory image")
    parser.add_argument(
        "--data-base",
        type=lambda s: int(s, 0),
        default=0,
        help="optional physical DATA_BASE offset (e.g. 0x10000ff0)",
    )
    args = parser.parse_args()

    commands = parse_commands(args.commands)
    memory, initial_size = load_memory(args.memory)
    port = args.port or choose_port()

    server = start_server(args.server, port)
    try:
        sock = connect(port)
        with sock:
            highest = run(sock, commands, memory, data_base=args.data_base)
        server.wait(timeout=30)
        output = server.stdout.read()
        if output.strip():
            print(output.rstrip())
        if server.returncode != 0:
            raise RuntimeError(f"simulator exited with code {server.returncode}")
    finally:
        if server.poll() is None:
            server.kill()
            server.wait()

    output_size = min(len(memory), max(initial_size, highest))
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "wb") as stream:
        stream.write(memory[:output_size])
    print(f"commands: {len(commands)}, wrote {output_size} bytes -> {args.output}")

    if args.reference:
        with open(args.reference, "rb") as stream:
            reference = stream.read()
        if reference != memory[:output_size]:
            limit = min(len(reference), output_size)
            mismatch = next(
                (i for i in range(limit) if reference[i] != memory[i]), None
            )
            detail = "size mismatch" if mismatch is None else f"first mismatch at {mismatch}"
            raise SystemExit(f"FAIL: {detail} (reference {len(reference)} bytes)")
        print(f"PASS: output matches {args.reference}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
