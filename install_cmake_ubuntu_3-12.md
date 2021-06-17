#### Download and compile:
```bash
$ wget http://www.cmake.org/files/v3.12/cmake-3.12.1.tar.gz
$ tar -xvzf cmake-3.12.1.tar.gz
$ cd cmake-3.12.1/
$ ./configure
$ make
```

Make's install command installs cmake by default in /usr/local/bin/cmake, shared files are installed into /usr/local/share/cmake-3.10.

Now it's time to create a backup, in case you need to roll back to the old version:
```bash
$ /usr/bin/cmake --version
cmake version 3.10.1

CMake suite maintained and supported by Kitware (kitware.com/cmake). 

$ sudo cp -p /usr/bin/cmake{,.3.10.1}

$ ll /usr/bin/cmake*
-rwxr-xr-x 1 root root 16509675 Dez 22  2017 /usr/local/bin/cmake
-rwxr-xr-x 1 root root 16509675 Dez 22  2017 /usr/local/bin/cmake.3.10.1
```

To install (copy) the binary and libraries to the new destination, run:
```bash
$ sudo make install
```

If you haven't already installed a newer cmake installation, run the following command to tell Ubuntu that the cmake command is now being replaced by an alternative installation:
```bash
$ sudo update-alternatives --install /usr/bin/cmake cmake /usr/local/bin/cmake 1 --force
```

If you already have a custom cmake version installed (in my case I still had the 3.10.1 version active), the update-alternatives command is not necessary.
The make install command will replace the existing binary in /usr/local/bin/cmake. This can be verified using:
```bash
$ cmake --version
cmake version 3.12.1

CMake suite maintained and supported by Kitware (kitware.com/cmake).
```
