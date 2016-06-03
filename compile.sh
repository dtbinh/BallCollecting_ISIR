reset
# This is the compilation for testing the simulator
#g++ -pipe -Wall -DUSE_SDL -O3 -msse2 -ggdb3 -g -std=c++11 RLNNACST.cpp Utils.cpp ./libfastsim/*.cpp -I/usr/include/SDL -lSDL -o RLNNACST -lboost_system


#This is for the entire experiment
g++ -pipe -Wall -DUSE_SDL -O3 -msse2 -ggdb3 -g -std=c++11 *.cpp ./libfastsim/*.cpp -I/usr/include/SDL -I/usr/include/boost -lSDL -l/usr/lib/x86_64-linux-gnu/ -o RLNNACST -lboost_system
