# Compilation Guide for ArduRover 4.6.3 (Pixhawk 4)

This guide documents the steps to compile ArduRover 4.6.3 on a modern Linux system (specifically Ubuntu 24.04) which may encounter compatibility issues with Python 3.12 and missing toolchains.

## 1. Prerequisites

### 1.1 Source Code
Checkout the specific version and submodules:
```bash
git checkout Rover-4.6.3
git submodule update --init --recursive
```

### 1.2 ARM Toolchain
The standard `install-prereqs-ubuntu.sh` may fail on newer Ubuntu releases. Manually install the ARM GCC toolchain:
```bash
cd /opt
sudo wget https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
sudo tar xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
sudo rm gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
```
Add to your PATH:
```bash
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
```

## 2. Handling Python 3.12 Compatibility
ArduPilot 4.6.3's Waf build system depends on the `imp` module, which was removed in Python 3.12. Use the following wrapper to inject a mock `imp` module at runtime:

**Create `waf_python312_fix.py`**:
```python
import types, importlib.util, sys, os

# Create a mock imp module to support Waf on Python 3.12
imp = types.ModuleType('imp')
sys.modules['imp'] = imp
imp.new_module = lambda name: types.ModuleType(name)
def load_source(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod
imp.load_source = load_source

# Setup paths for Waf
cwd = os.getcwd()
wafdir = os.path.join(cwd, 'modules/waf')
sys.path.insert(0, wafdir)

# Import and run Waf entry point
from waflib import Scripting
# WAFVERSION can be found in modules/waf/waflib/Context.py
Scripting.waf_entry_point(cwd, '2.0.27', wafdir)
```

## 3. Build Commands

### 3.1 Configure
Configure for Holybro Pixhawk 4:
```bash
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
python3 waf_python312_fix.py configure --board Pixhawk4
```

### 3.2 Compile
Compile Rover with MAVLink 2.0 enabled:
```bash
export MAVLINK20=1
python3 waf_python312_fix.py rover
```

## 4. Output
The resulting firmware files are located in:
`build/Pixhawk4/bin/ardurover.apj`
