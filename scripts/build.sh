cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_TARGET=$1 -DSUBTREE=TRUE ..
make