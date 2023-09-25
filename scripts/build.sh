rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DTARGET=$1 -DSUBTREE=TRUE ..
make