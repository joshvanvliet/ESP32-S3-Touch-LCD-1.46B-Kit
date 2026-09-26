"""Replay 1,100 deterministic frames and compare the pre-TE renderer's pixels."""
import hashlib
import pathlib
import subprocess
import tempfile

test = pathlib.Path(__file__).resolve().parent
main = test.parents[1] / "main"
expected = "1ab3ab123a5ca73354a28670226ac3fed47df16ae428d1f70b3469c48a67fccf"
with tempfile.TemporaryDirectory(prefix="face-render-") as output:
    exe = pathlib.Path(output) / "replay.exe"
    subprocess.run([
        "gcc", "-std=c11", "-O2", "-I", str(test / "stubs"), "-I", str(main),
        str(test / "replay.c"),
        *(str(main / name) for name in (
            "app_face_blit.c", "app_face_dirty.c", "app_face_canvas.c",
            "app_face_math.c", "app_face_config.c")),
        "-lm", "-o", str(exe),
    ], check=True)
    result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    normalized = "\n".join(result.stdout.splitlines()) + "\n"
    assert len(result.stdout.splitlines()) == 1100
    assert hashlib.sha256(normalized.encode()).hexdigest() == expected, "Rendered pixels changed"
    print(result.stderr.strip())
    print("All framebuffer and panel pixel hashes match the pre-TE reference.")
