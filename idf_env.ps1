# Sets up a correct, working ESP-IDF environment (GCC toolchain) and runs idf.py
# Usage: .\idf_env.ps1 set-target esp32s3
#        .\idf_env.ps1 build
#        .\idf_env.ps1 -p COM3 flash monitor

$env:IDF_PATH = "C:\esp\v6.1\esp-idf"
$env:IDF_TOOLCHAIN = "gcc"
$env:ESP_IDF_VERSION = "6.1"
$env:IDF_TOOLS_PATH = "C:\Espressif\tools"
$env:IDF_PYTHON_ENV_PATH = "C:\Espressif\tools\python\v6.1\venv"

$env:PATH = "C:\Espressif\tools\ninja\1.12.1;" + `
            "C:\Espressif\tools\cmake\4.0.3\bin;" + `
            "C:\Espressif\tools\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin;" + `
            "C:\Espressif\tools\dfu-util\0.11\dfu-util-0.11-win64;" + `
            "C:\Espressif\tools\openocd-esp32\v0.12.0-esp32-20260703\openocd-esp32\bin;" + `
            $env:PATH

& "C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe" "C:\esp\v6.1\esp-idf\tools\idf.py" @args
