# Install protobuf for Win64 and MSYS-2020

Create and run Shell-Script with **MSYS-2020**:
```bash
#!/bin/bash
# Protobuf 3.3.0
cd C:/tmp
curl -LO https://github.com/protocolbuffers/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.zip
unzip -qq protobuf-cpp-3.3.0.zip
cd protobuf-3.3.0
export CC="gcc -std=c++17 -s"
export CXX="g++ -std=c++17 -s"
mkdir -p build-dir && cd build-dir && rm -r *
../configure --prefix=C:/protobuf-3.3 --disable-shared
make -j12 && make install
```
