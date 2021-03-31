import warnings
warnings.warn("The all module is deprecated. It will be removed in the next release.", DeprecationWarning, stacklevel=2)


import inspect
import bipedal_locomotion_framework.bindings as blf

for blf_module in [__import__("bipedal_locomotion_framework.bindings." + module[0], globals(), locals(), ['*']) for module in inspect.getmembers(blf, inspect.ismodule)]:
    for k in dir(blf_module):
        locals()[k] = getattr(blf_module, k)


# Delete the imports
del inspect
del blf
del warnings
del blf_module
