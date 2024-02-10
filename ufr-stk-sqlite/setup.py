from setuptools import setup, Extension
from setuptools.command.install import install

with open("README.md", "r") as fh:
    long_description = fh.read()

module = Extension (
    'urf.libgw_sqlite'
    , sources = ['src/lt_gw_sqlite.c']
    , language = "c"
    , extra_compile_args = ['-fPIC']
    , py_limited_api=True  # for use .abi3.so instead cpython-310-linux.so
    , libraries = ['lt_api', 'sqlite3']
)

setup (
    name="urf-gw-sqlite",
    version="0.0.1",
    author="Felipe Bombardelli",
    author_email="felipebombardelli@gmail.com",
    url="https://github.com/bombark/linktree",
    description="",
    long_description=long_description,
    long_description_content_type="text/markdown",

    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
    ],
    python_requires='>=3.6',

    # configure compilation for c code
    ext_modules=[module],
    # include_package_data=True,
)