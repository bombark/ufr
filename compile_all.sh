#!/bin/bash

function ufr_fatal() {
    echo "ERROR"
    exit 1
}

function compile_module() {
    pushd $1
    mkdir -p build;
    cmake -S . -B build
    if [[ $? != 0 ]]; then
        echo "opa"
        rm -rf ./build
        mkdir -p build
        cmake -S . -B build || ufr_fatal
    fi
    cd build
    make || ufr_fatal;
	sudo make install
    popd
}

# Core
compile_module "ufr-api"
compile_module "ufr-cc-std"
compile_module "ufr-gtw-posix"

# Basic
compile_module "ufr-cc-msgpack"
compile_module "ufr-gtw-mqtt"
compile_module "ufr-gtw-zmq"
compile_module "ufr-gtw-sqlite"
# compile_module "ufr-gtw-gtk"
# compile_module "ufr-stk-ros_humble"

# Applications
# compile_module "vri-nav-simple"
# compile_module "vri-slam-simple"
compile_module "vri-base-pioneer-webots"
# compile_module "vri-base-pioneer"
