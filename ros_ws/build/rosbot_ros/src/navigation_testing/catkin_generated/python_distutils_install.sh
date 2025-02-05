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

echo_and_run cd "/home/hao/ros_ws/src/rosbot_ros/src/navigation_testing"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hao/ros_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hao/ros_ws/install/lib/python3/dist-packages:/home/hao/ros_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hao/ros_ws/build" \
    "/usr/bin/python3" \
    "/home/hao/ros_ws/src/rosbot_ros/src/navigation_testing/setup.py" \
     \
    build --build-base "/home/hao/ros_ws/build/rosbot_ros/src/navigation_testing" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hao/ros_ws/install" --install-scripts="/home/hao/ros_ws/install/bin"
