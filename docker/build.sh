#!/bin/bash -ex

apt install -y libfss-client libsmm-asset

cd /src
./autogen.sh
./configure
make

