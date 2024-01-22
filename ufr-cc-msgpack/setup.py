from setuptools import setup, Extension

with open("README.md", "r") as fh:
    long_description = fh.read()

encoder = Extension (
    'urf.libec_msgpack'
    , sources = ['src/enc_msgpack.c']
    , language = "c"
    , extra_compile_args = ['-fPIC']
    , py_limited_api=True  # for use .abi3.so instead cpython-310-linux.so
    , libraries = ['lt_api', 'msgpackc']
)

decoder = Extension (
    'urf.libdc_msgpack'
    , sources = ['src/dec_msgpack.c']
    , language = "c"
    , extra_compile_args = ['-fPIC']
    , py_limited_api=True  # for use .abi3.so instead cpython-310-linux.so
    , libraries = ['lt_api', 'msgpackc']
)

setup (
    name="urf-cc-msgpack",
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
    ext_modules=[encoder, decoder],
    include_package_data=True,
)