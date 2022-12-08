# Install model for Win64 and Visual Studio 2017

1. Create and run Shell-Script (e.g. with cmd.exe):
    ```bash
    #!/bin/bash
    mkdir -p build-VS2017 && cd build-VS2017 && rm -r *
    mkdir -p install-osi
    "C:/Program Files/CMake/bin/cmake.exe" \
        -G "Visual Studio 15 2017" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=install-osi \
        -DFMU_INSTALL_DIR:PATH=C:/tmp \
        -DCMAKE_GENERATOR_PLATFORM:INTERNAL=x64 \
        -DCMAKE_SHARED_LINKER_FLAGS:STRING=/machine:x64 \
        -DCMAKE_STATIC_LINKER_FLAGS:STRING=/machine:x64 \
        -DCMAKE_EXE_LINKER_FLAGS:STRING=/machine:x64 \
        -DCMAKE_MODULE_LINKER_FLAGS:STRING=/machine:x64 \
        -DProtobuf_INCLUDE_DIR=C:/protobuf-3.14/include \
        -DPROTOBUF_PROTOC_EXECUTABLE=C:/protobuf-3.14/bin/protoc.exe \
        -DProtobuf_LIBRARY_DEBUG=C:/protobuf-3.14/lib/libprotobufd.lib \
        -DProtobuf_LIBRARY_RELEASE=C:/protobuf-3.14/lib/libprotobuf.lib \
        -DProtobuf_LITE_LIBRARY_DEBUG=C:/protobuf-3.14/lib/libprotobuf-lited.lib \
        -DProtobuf_LITE_LIBRARY_RELEASE=C:/protobuf-3.14/lib/libprotobuf-lite.lib \
        -DProtobuf_PROTOC_LIBRARY_DEBUG=C:/protobuf-3.14/lib/libprotocd.lib \
        -DProtobuf_PROTOC_LIBRARY_RELEASE=C:/protobuf-3.14/lib/libprotoc.lib \
        ..
    ```
3. Open ReflectionBasedLidarModel.sln located in build-dir from above with **Visual Studio 2017**
4. Build Solution for *Release* AND for *Debug*
5. Build INSTALL for *Release* AND for *Debug*
