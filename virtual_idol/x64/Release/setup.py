from distutils.core import setup
from Cython.Build import cythonize


setup(
    name="dll_test",
    ext_modules=cythonize("dlltest.py")
)