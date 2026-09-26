"""Run transport regression checks with a host GCC compiler; no ESP-IDF needed."""
import pathlib
import subprocess
import tempfile

test = pathlib.Path(__file__).resolve().parent
main = test.parents[1] / "main"
with tempfile.TemporaryDirectory(prefix="lcd-blit-") as output:
    executable = pathlib.Path(output) / "test_blit.exe"
    subprocess.run([
        "gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(test / "stubs"), "-I", str(main),
        str(test / "test_blit.c"), str(main / "app_face_blit.c"),
        str(main / "app_face_dirty.c"), "-lm", "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
