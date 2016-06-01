#ifndef VISU
#define VISU
#endif

//#include "run_rl.hpp"
//#include "run_rl2.hpp"
#include "collectball2.hpp"
#include <iostream>
#include <tuple>
#include <stdint.h>
#include <SDL/SDL_keyboard.h>
#include <SDL.h>
#include <stdlib.h>     //for using the function sleep
#include <algorithm>

struct FSTParam {

    static constexpr size_t nb_motors       = 3;

//					wall    bal  bas   sw    bump  car  swa
    static constexpr int sensor_size = 3 +   2  +  2  +  2  +  2  +  1 + 1;

    static constexpr int strandmin = 52;
    static constexpr int strandmax = 109;
};

/*
int main(int argc, char **argv)
{

    //define program options
    namespace po = boost::program_options;
    po::options_description desc("Allowed RL FST options");
    desc.add_options()
    ("map", po::value<std::vector<std::string>>()->multitoken(), "set arena maps for evolution")
    ("instances", po::value<std::vector<unsigned int>>()->multitoken(), "set instances of position on the map");

    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    //po::store(po::parse_command_line(argc, argv).options(desc).allow_unregistered(), vm);
    po::store(parsed, vm);
    po::notify(vm);

    if (vm.count("map")) {
        std::cout << "Arena map was set to {";
        for(string i : vm["map"].as<std::vector<string>>() )
            std::cout << i << ", ";
        std::cout <<"} "  << std::endl;
    } else {
        std::cout << "Arena maps was not set." << std::endl;
        return 1;
    }

    if(vm.count("instances")) {
        std::cout << "Arena instances was set to {";
        for(unsigned int i : vm["instances"].as<std::vector<unsigned int>>() )
            std::cout << i << ", ";

        std::cout <<"} "  << std::endl;
    } else {
        std::cout << "Arena instances position was not set." << std::endl;
        return 1;
    }

    for(string i : vm["map"].as<std::vector<string>>() )
        LOG_FILE("config", "maps " << i);

    for(unsigned int i : vm["instances"].as<std::vector<unsigned int>>() )
        LOG_FILE("config", "instances " << i);


    ParamGame *GameInstance = new ParamGame();

    GameInstance->injectArgs(vm["map"].as<std::vector<std::string>>(), vm["instances"].as<std::vector<unsigned int>>());

    //run --original, by Matthieu
    RL_run<FSTParam, FSTSim, Simulator, arena_instance_id> runner;

    runner.run(argc, argv, desc);

    GameFactory::endInstance();
    ParamGame::endInstance();
    

    // Trial by Omar in order to have manual control over the simulator.
    RL_run2<FSTParam, FSTSim, Simulator, arena_instance_id> runner(0, 0, false, 0);
    runner.initializeEnvironment(argc, argv, desc);
    runner.runAllSteps();
}

*/

// Testing the collectball task with SDL keyboard actions
int main (){
    cout << "koko wawa" << endl;
    string mapName("b_a2_c2.pbm");
    const char *mapNameChar = mapName.c_str();

    CollectBall mycollector;
    cout << "Object established" << endl;
    mycollector.simuInitInside(0, mapNameChar);
    cout << "simuInitInside established" << endl;

    double move_left = 0.0;
    double move_right = 0.0;
    vector <double> movement(2);
    Uint8* keys;
    vector <int> goalsAchieved;
    for (long long int i = 0; i < 10000000; ++i)
    {
        cout << "Step number = " << i << endl;
        int numkey;
        SDL_PumpEvents();
        keys = SDL_GetKeyState(&numkey);
        // cout << "Keys readings are there : " << keys << endl;

        if (keys[SDLK_s]) {
            move_left = 0.0;
            move_right = 0.0;
        }
        else if (keys[SDLK_UP]) {
            // move_left = 1.0;
            // move_right = 1.0;
            move_left = 0.2;
            move_right = 0.2;
        }
        else if (keys[SDLK_DOWN]) {
            // move_left = -1.0;
            // move_right = -1.0;
            move_left = -0.2;
            move_right = -0.2;
        }
        else if (keys[SDLK_LEFT]) {
            // move_left = 1.0;
            // move_right = -1.0;
            move_left = 0.2;
            move_right = -0.2;   
        }
        else if (keys[SDLK_RIGHT]) {
            // move_left = -1.0;
            // move_right = 1.0;
            move_left = -0.2;
            move_right = 0.2;
        }
        else{
            move_left = 0.0;
            move_right = 0.0;
        }
        // cout << "If condition is done" << endl;

        movement[0] = move_left;
        movement[1] = move_right;
        mycollector.step_simu(movement);

        // cout << "simulator steps has been taken" << endl;
        for (int x = 0; x < 5; x++){
            double reward = mycollector.computeReward(x);
            if (reward == 100.0){
                if (std::find (goalsAchieved.begin(), goalsAchieved.end(), 30) == goalsAchieved.end())
                    goalsAchieved.push_back(x);
            }
        }
        cout << "----------------------------" << endl;
    }

    for (auto &item: goalsAchieved)
        cout << "Reward No." << item << " , has been achieved" << endl;

    return 0;
}