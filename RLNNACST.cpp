#define RLNN
//only use in CollectBall to active ball_collected function
#define STORETRACE

//#define BOLTZMANN

// #define SARSA
// #define NO_PARALLEL

// #define DYNA

// #define RANDOM

// #define NO_PARALLEL

// #define HUMAN_CONTROL

#define NO_SHAPING

#define ACTION_TIME
//#define OFFLINE_LEARNING

#include "gamefactory.hpp"
//#include "run_rl.hpp"
#include "run_rl2.hpp"
#include <iostream>
#include <tuple>

struct FSTParam {

    static constexpr size_t nb_motors       = 3;

//					wall    bal  bas   sw    bump  car  swa
    static constexpr int sensor_size = 3 +   2  +  2  +  2  +  2  +  1 + 1;

    static constexpr int strandmin = 52;
    static constexpr int strandmax = 109;
};


class RewardCollectBall {
public:
    RewardCollectBall(CollectBall* _simu):simu(_simu),
        begin_carry(ParamGame::max_nb_ball, false),
        switch_already_pu(false) {
    }

    ~RewardCollectBall() {
        if(simu != nullptr)
            delete simu;
    }

    CollectBall& operator()() {
        return *simu;
    }

    void deleteSimu() {
        delete simu;
        simu = nullptr;
    }

    bool end() {
        return simu->end();
    }

private:
    CollectBall* simu;

public:
    vector<bool> begin_carry;
    bool switch_already_pu;
};

typedef boost::shared_ptr<RewardCollectBall> Simulator;

struct FSTSim {

    static void computeIniInputs(std::vector<double>* inputs, const Simulator& simu, const std::vector<double>& ac) {
        // Update of the sensors
        size_t nb_lasers = simu->operator()().robot().get_lasers().size();
        size_t nb_light_sensors = simu->operator()().robot().get_light_sensors().size();

        int index = 0;
        // *** set inputs ***
        for (size_t j = 0; j < nb_lasers; j++)
        {
            double d = simu->operator()().robot().get_lasers()[j].get_dist();
            double range = simu->operator()().robot().get_lasers()[j].get_range();
            if(d == -1.f || d >= range)
                inputs->operator[](index++) = 0.f;
            else
                inputs->operator[](index++) = (d == -1.f || d >= range ? 0 : 1 - d / range);
        }

        for (size_t j = 0; j < nb_light_sensors; j+=2)
        {
            double actL = (double) simu->operator()().robot().get_light_sensors()[j].get_activated();
            double actR = (double) simu->operator()().robot().get_light_sensors()[j+1].get_activated();
            inputs->operator[](index+j) = actL;
            inputs->operator[](index+j+1) = actR;
        }

        index += nb_light_sensors;

        inputs->operator[](index+0) = (double) simu->operator()().robot().get_left_bumper();
        inputs->operator[](index+1) = (double) simu->operator()().robot().get_right_bumper();
        inputs->operator[](index+2) = (double) simu->operator()().robot_carrying_ball();

        index = index + 3;

        inputs->operator[](index) = (double) !simu->operator()().never_pull_switch();
        index++;

        for(int i=index; i< index + ac.size(); i++)
            inputs->operator[](i) = ac.at( i - index );
    }

//    static void singletonInit() {
//        GameFactory::getInstance();
//    }

    static arena_instance_id instanceIteratorBegin() {
        return {0,0};
    }

    static int instanceToInt(const arena_instance_id& instance) {
        return *instance;
    }

    static bool instanceIteratorEnd(const arena_instance_id& instance) {
        ParamGame *GameInstance = new ParamGame();
        bool result = instance <= GameInstance->getEndInstance();
        delete GameInstance;
        return result;
    }

    static Simulator simuInit() {
        return nullptr;
    }

    static Simulator simuInitInside(Simulator s, const arena_instance_id& instance) {
	endInstanceInside(s);
        GameFactory *GameFactoryInstance = new GameFactory();
        ParamGame *GameInstance = new ParamGame();
        CollectBall* simu1 = GameFactoryInstance->create(GameInstance->getArenaKey(instance.map_id), GameInstance->getPositionInstance(instance.position_id));
        Simulator simulation(new RewardCollectBall(simu1));

        assert(simulation->operator()().well_init());
#ifdef VISU
        simulation->operator()().init_view(true);
#endif
        simulation->operator()().refresh_robot();

        delete GameFactoryInstance;
        delete GameInstance;

        return simulation;
    }

    static void endInstanceInside(Simulator& s) {
        if(s.get() != nullptr) {
            s->deleteSimu();
            s.reset();
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //ADDED BY OMAR - see more about the robot behaviour
    //What to see:
    // - Is the robot near a basket?
    // - Is the robot near a switch?
    // - Is the robot near a ball?
    // - Is the robot carrying a ball?
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    static double computeReward(Simulator& simu, int rewardChoice){
        /*
            In order to handle the case of different 'hardcoded' reward functions, I will make a switch case between different
            reward functions
            0 : Collect ball and then put it in the basket --> the traditional reward.
            1 : Collect the ball
            2 : See the ball
            3 : See the basket
            4 : See a switch
            5 : See a wall - no particular reason at the moment
        */
        bool bdrop = false;
        switch (rewardChoice){
            case 0:
                bdrop = simu->operator()().ball_collected();
                break;

            case 1:
                bdrop = simu->operator()().robot_carrying_ball();
                break;

            case 2:{
                int robot_near_ball_id = simu->operator()().robot_near_ball(); // I think it will return the number of ball it is near to. Otherwise, -1. I can make a boolean version of it
                    if (robot_near_ball_id != -1)
                        bdrop = true;

                    break;
            }
            case 3:
                bdrop = simu->operator()().robot_near_basket();
                break;

            case 4:
                bdrop = simu->operator()().robot_near_switch();
                break;

            default:{
                bdrop = false;
                break;
            }
        }

        if (bdrop == true)
            return 100;
        else
            return 0.0;

    }

    static std::tuple <bool,bool,bool,bool,bool,bool,float> robotWorldStatus (Simulator& simu){
        int robot_near_ball_id = simu->operator()().robot_near_ball(); // I think it will return the number of ball it is near to. Otherwise, -1. I can make a boolean version of it
        bool robot_near_ball = false;
        if (robot_near_ball_id != -1)
            robot_near_ball = true;

        bool robot_near_switch = simu->operator()().robot_near_switch();
        bool robot_near_basket = simu->operator()().robot_near_basket();
        bool robot_carrying_ball = simu->operator()().robot_carrying_ball();
        bool stuck_too_long = simu->operator()().stuck_too_long();
        bool never_pull_switch = simu->operator()().never_pull_switch();
        float robot_distance_basket = simu->operator()().robot_distance_basket();

        return std::make_tuple(robot_near_ball, robot_near_switch, robot_near_basket, robot_carrying_ball, stuck_too_long, never_pull_switch, robot_distance_basket);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    static void step_simu(Simulator& simu, const std::vector<double>& outputs) {
        simu->operator()().step(sml::Utils::transform(outputs[0], 0.f, 1.f, -ParamGame::motor_power, ParamGame::motor_power),
                                sml::Utils::transform(outputs[1], 0.f, 1.f, -ParamGame::motor_power, ParamGame::motor_power),
                                outputs[2] >= 0.5f);

        simu->operator()().refresh();
#ifdef VISU
        simu->operator()().refresh_view();
#endif
    }

    static double simu_perf(Simulator& simu) {
        return simu->operator()().number_collected_balls();
    }

    static int simu_max_episode() {
        return ParamGame::step_limit;
    }

//    static void endInstance(Simulator&) {
//
//    }

};


int main(int argc, char **argv)
{

//     static tbb::task_scheduler_init init(32);

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
    /*
    RL_run<FSTParam, FSTSim, Simulator, arena_instance_id> runner;

    runner.run(argc, argv, desc);

    GameFactory::endInstance();
    ParamGame::endInstance();
    */

    // Trial by Omar in order to have manual control over the simulator.
    RL_run2<FSTParam, FSTSim, Simulator, arena_instance_id> runner(0, 0, false, 0);
    runner.initializeEnvironment(argc, argv, desc);
    runner.runAllSteps();
}

