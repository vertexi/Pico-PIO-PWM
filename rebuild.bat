RMDIR /S /Q build
cmake -G "NMake Makefiles" . -B build
cd build
nmake
cd ../
