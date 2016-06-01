# This is the compilation for testing the simulator
g++ -pipe -Wall -DUSE_SDL -O3 -msse2 -ggdb3 -g -std=c++11 RLNNACST.cpp Utils.cpp ./libfastsim/*.cpp -I/usr/include/SDL -lSDL -o RLNNACST -lboost_system
