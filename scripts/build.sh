rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DMCU=$1 -DSUBTREE=TRUE ..
make