reset
rm RLNNACST
# This is the compilation for testing the simulator
# g++ -pipe -Wall -DUSE_SDL -O3 -msse2 -ggdb3 -g -std=c++11 RLNNACST.cpp Utils.cpp ./libfastsim/*.cpp -I/usr/include/SDL -lSDL -o RLNNACST -lboost_system


#This is for the entire experiment
g++ -pipe -Wall -DUSE_SDL -O3 -msse2 -ggdb3 -g -std=c++11 *.cpp ./libfastsim/*.cpp \
-I/usr/include/SDL -I/usr/include/boost -lfann -lSDL \
-lboost_program_options -lboost_filesystem -lboost_system -L/usr/lib/x86_64-linux-gnu -L/usr/local/lib \
-o RLNNACST

./RLNNACST --map b_a2_c2.pbm --instances 0 --actions actions_s15_n17 --states states_h1
