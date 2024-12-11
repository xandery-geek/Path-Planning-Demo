# update the .ts files
lupdate src -ts lang/en_US.ts lang/zh_CN.ts

# build the project
cd build
cmake ..
cmake --build .