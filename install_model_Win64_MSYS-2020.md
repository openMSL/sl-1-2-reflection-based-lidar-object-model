Build the model by running the following bash script in MSYS-2020:
   ```bash
   #!/bin/bash
   export CC="gcc -std=c++17 -s -static-libgcc -static-libstdc++ -static"
   export CXX="g++ -std=c++17 -s -static-libgcc -static-libstdc++ -static"
   mkdir -p build-MSYS && cd build-MSYS && rm -r *
   mkdir -p install-osi
   "C:/Program Files/CMake/bin/cmake.exe" \
      -G "MSYS Makefiles" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=install-osi \
      -DFMU_INSTALL_DIR:PATH=C:/tmp \
      -DPROTOBUF_PROTOC_EXECUTABLE=C:/protobuf-3.3/bin/protoc.exe \
      -DPROTOBUF_FOUND=C:/protobuf-3.3 \
      -DProtobuf_INCLUDE_DIR=C:/protobuf-3.3/include \
      -DPROTOBUF_LIBRARY=C:/protobuf-3.3/lib/libprotobuf.a \
      ..
   make -j12
   make install
   ```
