#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

if [ $# -lt 2 ]; then
    echo "Usage: $0 [ROS1|ROS2|humble] [Livox-SDK2 install prefix] [extra cmake_prefix_path]"
    echo "eg: $0 ROS1 $HOME/Documents/slam_devel $HOME/Documents/slam_devel"
    echo "/opt/ros/noetic is often not need to pass in the prefix_path argument."
    exit
fi

if [ $# -ge 3 ]; then
    prefix_path="$CMAKE_PREFIX_PATH;$3"
else
    prefix_path=$CMAKE_PREFIX_PATH
fi
echo "prefix path: $prefix_path"

if [ $# -ge 2 ]; then
    install_prefix="$2"
else
    install_prefix=$CMAKE_INSTALL_PREFIX
fi
echo "install prefix: $install_prefix"

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

ROS_VERSION=""
ROS_HUMBLE=""

# Set working ROS version
if [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "$1" = "humble" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_HUMBLE=${VERSION_HUMBLE}
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    echo "Invalid Argument"
    exit
fi
echo "ROS version is: "$ROS_VERSION

# clear `build/` folder.
# TODO: Do not clear these folders, if the last build is based on the same ROS version.
rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/
# clear src/CMakeLists.txt if it exists.
if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

# exit

# substitute the files/folders: CMakeList.txt, package.xml(s)
if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS1.xml package.xml
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS2.xml package.xml
    cp -rf launch_ROS2/ launch/
fi

# build
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    cd ../../
    cmd="catkin build -DROS_EDITION=${VERSION_ROS1} -DCMAKE_INSTALL_PREFIX="$install_prefix" -DCMAKE_PREFIX_PATH="$prefix_path""
    echo $cmd
    $cmd
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    cd ../../
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2} -DHUMBLE_ROS=${ROS_HUMBLE}
fi
popd > /dev/null

# remove the substituted folders/files
if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm -rf launch/
fi

popd > /dev/null
