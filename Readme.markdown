battlehill
==========

Battlehill communicates with the Link2's co-processor to coordinate motor/servo actuation and digital/analog sensor input.
It is the daemon component of libbattlecreek.

Daylite
-------

Battlehill communicates over daylite. For message documentation, see the bsonbind files in libbattlecreek/src
and the libbattlecreek Readme.

Dependencies
============
* cmake > 2.8
* c++ compiler with C++11 support
* [libbattlecreek](https://github.com/kipr/libbattlecreek)
* [daylite](https://github.com/kipr/daylite)

Building
========
```
mkdir build
cd build
cmake ..
make
make install
```

Useful cmake arguments:
* `-DCMAKE_INSTALL_PREFIX=<install prefex>`, recommended for Windows
* `-DCMAKE_PREFIX_PATH=<dependencies prefix path>`, useful when building for Windows to tell cmake where the dependencies were installed
