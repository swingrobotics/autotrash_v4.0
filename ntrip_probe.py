import base64
import json
import os
import socket
import time


CONFIG_PATH = os.environ.get(
    "NTRIP_CONFIG_PATH",
    "/home/gnss/camera-stream/ntrip-config.json",
)


def gps_fix():
    with socket.create_connection(("127.0.0.1", 2947), timeout=5) as gpsd_socket:
        gpsd_socket.sendall(b'?WATCH={"enable":true,"json":true};\n')
        with gpsd_socket.makefile("r", encoding="utf-8") as stream:
            for line in stream:
                report = json.loads(line)
                if report.get("class") == "TPV" and report.get("mode", 0) >= 2:
                    return report
    raise RuntimeError("No GNSS fix available")


def coordinate(value, latitude):
    width = 2 if latitude else 3
    absolute = abs(float(value))
    degrees = int(absolute)
    minutes = (absolute - degrees) * 60
    return f"{degrees:0{width}d}{minutes:09.6f}"


def gga_sentence(fix):
    now = time.gmtime()
    body = (
        f"GPGGA,{time.strftime('%H%M%S', now)}.00,"
        f"{coordinate(fix['lat'], True)},{'N' if fix['lat'] >= 0 else 'S'},"
        f"{coordinate(fix['lon'], False)},{'E' if fix['lon'] >= 0 else 'W'},"
        f"1,12,1.0,{float(fix.get('altMSL', fix.get('alt', 0.0))):.3f},M,0.0,M,,"
    )
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f"${body}*{checksum:02X}\r\n".encode("ascii")


def rtcm_types(data):
    message_types = []
    offset = 0
    while offset + 6 <= len(data):
        if data[offset] != 0xD3:
            offset += 1
            continue
        length = ((data[offset + 1] & 0x03) << 8) | data[offset + 2]
        frame_length = 3 + length + 3
        if offset + frame_length > len(data):
            break
        payload = data[offset + 3 : offset + 3 + length]
        if len(payload) >= 2:
            message_types.append((payload[0] << 4) | (payload[1] >> 4))
        offset += frame_length
    return sorted(set(message_types))


def main():
    with open(CONFIG_PATH, "r", encoding="utf-8") as config_file:
        config = json.load(config_file)
    fix = gps_fix()
    credentials = base64.b64encode(
        f"{config.get('username', '')}:{config.get('password', '')}".encode("utf-8")
    ).decode("ascii")
    request = (
        f"GET /{config['mountpoint'].lstrip('/')} HTTP/1.1\r\n"
        f"Host: {config['host']}:{int(config['port'])}\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "User-Agent: NTRIP GNSS-Probe/1.0\r\n"
        f"Authorization: Basic {credentials}\r\n"
        "Connection: close\r\n\r\n"
    ).encode("ascii")

    with socket.create_connection((config["host"], int(config["port"])), timeout=8) as caster:
        caster.settimeout(3)
        caster.sendall(request)
        response = b""
        payload = b""
        while len(response) < 16384:
            chunk = caster.recv(1024)
            if not chunk:
                break
            response += chunk
            if response.startswith(b"ICY 200 OK\r\n"):
                status, _, payload = response.partition(b"\r\n")
                break
            if b"\r\n\r\n" in response:
                header, _, payload = response.partition(b"\r\n\r\n")
                status = header.split(b"\r\n", 1)[0]
                break
        else:
            raise RuntimeError("Invalid caster response")

        print(f"caster_status={status.decode('ascii', errors='replace')}")
        if b" 200 " not in status and not status.startswith(b"ICY 200"):
            return
        caster.sendall(gga_sentence(fix))
        received = bytearray(payload)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            try:
                received.extend(caster.recv(4096))
            except socket.timeout:
                continue
        print(f"rtcm_bytes={len(received)}")
        print(f"rtcm_types={rtcm_types(received)}")


if __name__ == "__main__":
    main()
