import pathlib
import subprocess
import tempfile

test = pathlib.Path(__file__).resolve().parent
driver = test.parents[1] / "main" / "LCD_Driver"
with tempfile.TemporaryDirectory(prefix="qspi-io-") as output:
    executable = pathlib.Path(output) / "test_qspi.exe"
    subprocess.run([
        "gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(test / "stubs"), "-I", str(driver),
        str(test / "test_qspi.c"), str(driver / "app_lcd_qspi_io.c"),
        "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
