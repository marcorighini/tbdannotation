#!/usr/bin/env bash

JOBS=4

while [ "$1" != "" ]; do
  case $1 in
    -j | --jobs ) shift
    JOBS=$1
    ;;
    -c | --ccompiler ) shift
    CC="-DCMAKE_C_COMPILER=$1"
    ;;
    -p | --cppcompiler ) shift
    CXX="-DCMAKE_CXX_COMPILER=$1"
    ;;
    * )
    #If we exit here we break the first argument behaviour
    #echo "$1"
    echo "${USAGE}"
    ;;
  esac
  shift
done

# google docet - jump within the directory which contains this script
SOURCE="${BASH_SOURCE[0]}"

while [ -h "$SOURCE" ]; do
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done

# set aside the base dir for future references
DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"

### Submodules

# Init submodules
git submodule update --init

# Configure and build OpenCV
cd deps/opencv && git checkout tags/3.2.0 && cd $DIR
cd deps/opencv_contrib && git checkout abf44fc && cd $DIR
mkdir -p $DIR/deps/opencv/build
cd $DIR/deps/opencv/build
cmake $CXX $CC -DCMAKE_BUILD_TYPE=Release -DOPENCV_EXTRA_MODULES_PATH=$DIR/deps/opencv_contrib/modules .. && make -j$JOBS
  #-DBUILD_SHARED_LIBS=OFF

# Configure and build Boost
cd $DIR
cd deps/boost && git checkout tags/boost-1.63.0 && cd $DIR
cd $DIR/deps/boost
git submodule update --init
mkdir -p $DIR/deps/boost/build
cd $DIR/deps/boost
./bootstrap.sh --prefix=build --with-libraries=serialization,filesystem,thread,program_options
./b2 headers
#./b2 link=static
./b2 install


cd $DIR
