#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/martin/Code/Master/autoCalib_ws/src/cv_bridge"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/usr/local/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/usr/local/lib/python3/dist-packages:/home/martin/Code/Master/autoCalib_ws/src/cmake-build-debug/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/martin/Code/Master/autoCalib_ws/src/cmake-build-debug" \
    "/usr/bin/python3" \
    "/home/martin/Code/Master/autoCalib_ws/src/cv_bridge/setup.py" \
    egg_info --egg-base /home/martin/Code/Master/autoCalib_ws/src/cmake-build-debug/cv_bridge \
    build --build-base "/home/martin/Code/Master/autoCalib_ws/src/cmake-build-debug/cv_bridge" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/usr/local" --install-scripts="/usr/local/bin"
