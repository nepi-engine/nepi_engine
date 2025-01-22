##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##
OPENCV_PKG=opencv # Desktop
#OPENCV_PKG=opencv4 # Jetson

g++ --std=c++11 -Ofast -DUNDERWATER_IMG_ENHANCER_TEST_APP `pkg-config --cflags $OPENCV_PKG` -I/usr/include/opencv4/ -I../../include ./underwater_img_enhancer.cpp `pkg-config --libs $OPENCV_PKG` -o underwater_img_enhancer_test_app
