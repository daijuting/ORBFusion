echo "Configuring and building Thirdparty/DBoW2 ..."

cd src/SlamEngine/Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../../../../

mkdir build
cd build
cmake ../src
make -j8
