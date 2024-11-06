from setuptools import setup
from setuptools import Extension
from Cython.Build import cythonize
cy_opts = {'compiler_directives': {'language_level': '3'}}
import numpy as np

ext_mods = [Extension(
    'wrapper_module_3', ['wrapper_module_3.pyx', 'wrapped_code_3.c'],
    include_dirs=[np.get_include()],
    library_dirs=[],
    libraries=[],
    extra_compile_args=['-std=c99'],
    extra_link_args=[]
)]
setup(ext_modules=cythonize(ext_mods, **cy_opts))
