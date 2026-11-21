"""PlatformIO upload over Wi-Fi: copy the .hex to the robot's Orange Pi and flash the Teensy there.

Used by [env:teensy41_wifi] (upload_protocol = custom). Also runs on its own:
    python tools/wifi_upload.py .pio/build/teensy41_wifi/firmware.hex

Env: ROBOT_HOST          ssh host (default "robot", see ~/.ssh/config)
     ROBOT_FLASH_DRY_RUN 1 = run every check on the Pi but do not flash
"""
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REMOTE_DIR = "sbr-tools"          # relative to the Pi user's home
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=8"]


def run(cmd):
    print("> " + " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def main():
    if len(sys.argv) != 2:
        print("usage: wifi_upload.py firmware.hex")
        return 2
    hex_path = sys.argv[1]
    host = os.environ.get("ROBOT_HOST", "").strip() or "robot"
    dry = os.environ.get("ROBOT_FLASH_DRY_RUN", "").strip().lower() in ("1", "true", "yes", "on")
    if not os.path.isfile(hex_path):
        print("wifi_upload: no such file: " + hex_path)
        return 2
    print("wifi_upload: %s (%d KB) -> %s%s" % (hex_path, os.path.getsize(hex_path) // 1024, host,
                                             "  [DRY RUN]" if dry else ""), flush=True)
    t0 = time.time()

    # pi_flash.sh with LF line endings no matter how git checked it out
    with open(os.path.join(HERE, "pi_flash.sh"), "rb") as f:
        script = f.read().replace(b"\r\n", b"\n")
    tmp = tempfile.mkdtemp(prefix="sbr3-upload-")
    script_path = os.path.join(tmp, "pi_flash.sh")
    with open(script_path, "wb") as f:
        f.write(script)

    try:
        if run(["ssh"] + SSH_OPTS + [host, "mkdir -p " + REMOTE_DIR]) != 0:
            print("wifi_upload: cannot reach %s over ssh (robot on? same Wi-Fi? try: ssh %s)" % (host, host))
            return 1
        if run(["scp", "-q"] + SSH_OPTS + [hex_path, script_path, "%s:%s/" % (host, REMOTE_DIR)]) != 0:
            print("wifi_upload: copying to the Pi failed")
            return 1
        remote = "bash {d}/pi_flash.sh {opt}{d}/{hex}".format(
            d=REMOTE_DIR, opt="--dry-run " if dry else "", hex=os.path.basename(hex_path))
        rc = run(["ssh"] + SSH_OPTS + [host, remote])
    finally:
        try:
            os.remove(script_path)
            os.rmdir(tmp)
        except OSError:
            pass

    took = time.time() - t0
    if rc == 0:
        print("wifi_upload: %s in %.1fs" % ("dry run OK" if dry else "uploaded", took))
    else:
        print("wifi_upload: FAILED (exit %d) after %.1fs - see the [pi_flash] lines above. "
              "USB fallback: pio run -e teensy41 -t upload" % (rc, took))
    return rc


if __name__ == "__main__":
    sys.exit(main())
