hwm
===

Build libroad, following those instructions.

Then point to that build, and configure this:

```
$ export LIBROAD_DIR=<your path to libroad>
$ autoreconf -i
$ CXXFLAGS="-std=c++11" ./configure --with-libroad-cflags="-I$LIBROAD_DIR/libroad/" --with-libroad-libs="$LIBROAD_DIR/libroad/libroad.la $LIBROAD_DIR/libroad/libroad_image.la $LIBROAD_DIR/libroad/libroad_visual.la"
$ make -j 8
```
or

```
$ export LIBROAD_DIR=<your path to libroad>
$ autoreconf -i
$ CXXFLAGS="-std=c++11" ./configure --with-libroad-cflags="-I$LIBROAD_DIR/" --with-libroad-libs="$LIBROAD_DIR/libroad/libroad.la $LIBROAD_DIR/libroad/libroad_image.la $LIBROAD_DIR/libroad/libroad_visual.la"
$ make -j 8
```

