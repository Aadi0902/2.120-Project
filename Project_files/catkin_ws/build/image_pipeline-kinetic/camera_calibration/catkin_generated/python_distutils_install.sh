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

echo_and_run cd "/home/robot/2.120-Project/Project_files/catkin_ws/src/image_pipeline-kinetic/camera_calibration"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/robot/2.120-Project/Project_files/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/robot/2.120-Project/Project_files/catkin_ws/install/lib/python2.7/dist-packages:/home/robot/2.120-Project/Project_files/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/robot/2.120-Project/Project_files/catkin_ws/build" \
    "/usr/bin/python2" \
    "/home/robot/2.120-Project/Project_files/catkin_ws/src/image_pipeline-kinetic/camera_calibration/setup.py" \
     \
    build --build-base "/home/robot/2.120-Project/Project_files/catkin_ws/build/image_pipeline-kinetic/camera_calibration" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/robot/2.120-Project/Project_files/catkin_ws/install" --install-scripts="/home/robot/2.120-Project/Project_files/catkin_ws/install/bin"
