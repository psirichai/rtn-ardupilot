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
