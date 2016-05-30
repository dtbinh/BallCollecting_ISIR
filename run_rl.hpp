#ifndef RUN_RL_HPP
#define RUN_RL_HPP

#define RLNN

#ifdef VISU
#define NO_PARALLEL
#endif

#ifdef VALGRIND
#define NO_PARALLEL
#endif

#include <ctime>
#include <ratio>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <boost/program_options.hpp>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/queuing_mutex.h>
#include <tbb/compat/condition_variable>
#include <tbb/task_scheduler_init.h>
#include <thread>

#include "bib/Seed.hpp"
#include "bib/Logger.hpp"

#include <sml/SarsaNN.hpp>
#include <sml/DynaNN.hpp>
#include <sml/ActionFactory.hpp>
#include <sml/StateExtractor.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <sstream> // Added by Omar in order to make more robust logging!
#include <tuple>


/*
    This is for the options
*/
#include <sml/Options.hpp>
#include <sml/TerminationCondition.hpp>

typedef boost::shared_ptr<vector<double>> EnvState;

using namespace sml;
using tbb::interface5::condition_variable;

struct RLAllParam {
    double starting_epsilon;
    double ending_epsilon;
    double stepness_epsilon;

    int max_episod;

    int timestep_decision_min;
    int timestep_decision_max;

    sml::RLParam DefaultParam;

    int number_parallel_mean;

    int history_length_max;

    int nbInputsTotal(int nb_max_inputs_on_time) const{
	return nb_max_inputs_on_time * history_length_max;
    }
};

template <typename SimuParam, typename Simulator, typename Environnement, typename InstanceIterator>
class RL_run {
    static constexpr int nb_max_inputs_on_time = SimuParam::sensor_size + SimuParam::nb_motors;

    typedef tbb::queuing_mutex Mutex;

    RLAllParam rlparam;

    struct Average {
        //needed by synchro
        int populate = 0;

        //variance collected ball
        list<double> score;   	// score -> mean score / score variance
        list<double> reward_sum;	// sum R(s,a) -> mean sum / sum variance
        list<int> step_sum;	// number of step

        list<double> personal_best_score;
        list<double> personal_best_reward_sum;
        list<int> personal_best_step_sum;
    };

    static std::vector<double>* computeInputs(const Environnement& simu, std::vector<double>* last_input, const std::vector<double>& ac, RLAllParam* rlparam) {
        // Omar: What does this function do exactly??
        int nbInputsTotal = rlparam->nbInputsTotal(nb_max_inputs_on_time);
	std::vector<double>* inputs = new std::vector<double>(nbInputsTotal);

        Simulator::computeIniInputs(inputs, simu, ac);

        for(int i=0; i < nbInputsTotal - nb_max_inputs_on_time; i++)
            inputs->operator[](i + nb_max_inputs_on_time) = last_input->at(i);

        return inputs;
    }

    struct AverageRun {

        Mutex* mutex;
        Average* avg;
        unsigned int number_thread;
        tbb::mutex* sync_mutex;
        condition_variable* my_condition;
        std::string loading_file;
        double* bestRewardEpisode;
        RLAllParam* rlparam;
	int* stop_simulation;

#ifndef RANDSTATE
        StateExtractor* stx;
#endif

#ifdef NO_PARALLEL
        void operator()() {
#else
        void operator()(const tbb::blocked_range<int>& range) const {
#endif

#if !defined(RANDACLINEAR) && !defined(RANDACFIXED)
            const list_tlaction& actions = sml::ActionFactory::getInstance()->getActions();
            int nb_actions = sml::ActionFactory::getInstance()->getActionsNumber();
#else
            Mutex::scoped_lock lock(*mutex);

#ifdef LOADFILE
            sml::ActionFactory::getInstance()->read(loading_file+".ac");
#else
#if defined(RANDACLINEAR)
            sml::ActionFactory::getInstance()->randomLinearAction(SimuParam::nb_motors, rlparam->timestep_decision_min, rlparam->timestep_decision_max);
#else
            sml::ActionFactory::getInstance()->randomFixedAction(SimuParam::nb_motors, rlparam->timestep_decision_min, rlparam->timestep_decision_max);
#endif
#endif
            const list_tlaction& actions(sml::ActionFactory::getInstance()->getActions());

            int nb_actions = sml::ActionFactory::getInstance()->getActionsNumber();

            lock.release();
#endif

            int num_options = 3;
//            sml::ActionTemplate a( {"effectors"}, {nb_actions + num_options});

#ifdef RANDSTATE
            StateExtractor stx_in(SimuParam::sensor_size, SimuParam::nb_motors);
            StateExtractor* stx = &stx_in;

//            stx->initParser(rlparam->nbInputsTotal(nb_max_inputs_on_time), (rand() % (int) (rlparam->nbInputsTotal(nb_max_inputs_on_time))));
            //Random informed
	    stx->initParser(rlparam->nbInputsTotal(nb_max_inputs_on_time), SimuParam::strandmin + (rand() % (SimuParam::strandmax - SimuParam::strandmin)  )  );
#endif



#ifdef ACTION_TIME

            vector<int> actions_time(nb_actions);
            for(int i=0; i < nb_actions; i++)
                actions_time[i] = actions[i].temporal_extension;
                //actions_time[i] = 15;

            sml::SarsaNN<EnvState> algo(&a, rlparam->DefaultParam, stx->getNumberInput(), actions_time);
#else
            sml::SarsaNN<EnvState> algo(&a, rlparam->DefaultParam, stx->getNumberInput());
#endif

#ifdef LOADFILE
            algo.read(loading_file);
#endif
            sml::Policy<EnvState>* bestEpisode = nullptr;
            double my_best_score = 0;
            double my_best_reward = INT_MIN;
            int my_best_step_sum = INT_MAX;

            Environnement simu = Simulator::simuInit();

#ifdef VALGRIND
            for(int episode=0; episode < 20; episode++) {
#else
//#ifdef TESTPERF
            //for(int episode=0; episode < 10; episode++) { //This is the original Matthieu line
//            for(int episode=0; episode < 50; episode++) {
//#else
        /*
            Testing options
            - In order for this hack to work, what I did is:
            -- I will increase the number of actions in the "ActionTemplate" initialization with the number of options "num_options".
            --
        */
        sml::TerminationCondition<EnvState> cond0;
        sml::Option<EnvState, sml::TerminationCondition<EnvState>> option1(cond0, &a, rlparam, stx->getNumberInput());


            for(int episode=0; episode < rlparam->max_episod; episode++) {

            //for(int episode=0; episode < 2; episode++) {
//#endif
#endif
                double cball = 0;
                double reward_sum = 0;
                int step_sum = 0;

                //This is the decay in the epsilon with time (less exploration with time)
                algo.getParams().epsilon =  (rlparam->starting_epsilon - rlparam->ending_epsilon)*pow(rlparam->stepness_epsilon, 1+episode) + rlparam->ending_epsilon;

#if defined(LOADFILE) && ! defined(TESTPERF)
                algo.getParams().epsilon =  rlparam->ending_epsilon;
#endif

                ///////////////------------------ Added by Omar
                int num_actions = 0;
                //////////////////////////////////////////////////

                for(InstanceIterator instance = Simulator::instanceIteratorBegin(); Simulator::instanceIteratorEnd(instance); instance++)
                {
                    #ifdef TESTPERF
                    #endif // TESTPERF
                    simu = Simulator::simuInitInside(simu, instance);

                    DAction ainit(&a, {0});
                    DAction* ac = &ainit;

                    std::vector<double> motor_init(SimuParam::nb_motors, 0.5);
                    std::vector<double>* tmp_inp_ini = new vector<double>(rlparam->nbInputsTotal(nb_max_inputs_on_time));
                    Simulator::computeIniInputs(tmp_inp_ini, simu, motor_init);

                    for(int i=nb_max_inputs_on_time; i < tmp_inp_ini->size(); i++)
                        tmp_inp_ini->operator[](i) = tmp_inp_ini->at(i % nb_max_inputs_on_time);

                    boost::shared_ptr< std::vector<double> > complete_input(tmp_inp_ini);
                    boost::shared_ptr< std::vector<double> > _inputs(stx->parse(*complete_input));
                    algo.startEpisode(_inputs, *ac);

                    int step;
                    double reward = Simulator::computeReward(simu); //This is the number of balls collected. It is in RLNNACST
                    double rreward = 0;
                    double powgamma = 1.d;

                    for(step=0; true ;) {
                    num_actions ++;

                        _inputs.reset(stx->parse(*complete_input));

#ifndef TESTPERF
                        ac = algo.learn(_inputs, reward, simu->end());
#else //TESTPERF
                        ac = algo.decision(_inputs, true);
#endif
                        if(simu->end() || step >= Simulator::simu_max_episode())
                            break;

                        int timestep_decision = actions.at(ac->get(0)).temporal_extension;
                        reward = 0;
                        for(int timestep=0; timestep < timestep_decision; timestep ++) {

                            boost::scoped_ptr< std::vector<double> > outputs(ActionFactory::computeOutputs(ac, timestep, actions));
                            Simulator::step_simu(simu, *outputs);
                            double lreward = Simulator::computeReward(simu);
                            reward += lreward * pow(rlparam->DefaultParam.gamma, timestep);

                            rreward += lreward * powgamma;
                            powgamma *= rlparam->DefaultParam.gamma;

                            if(timestep >= timestep_decision - rlparam->history_length_max)
                                complete_input.reset(RL_run::computeInputs(simu, complete_input.get(), *outputs, rlparam));

                            if(simu->end()){
                            	timestep_decision = timestep + 1;
                            	break;
                            }
                        }

                        step += timestep_decision;
                    }

                    step_sum += step;

                    cball += Simulator::simu_perf(simu);

                    algo.endEpisode(rreward, Simulator::instanceToInt(instance));

                    reward_sum += rreward;
                }

                Simulator::endInstanceInside(simu);

                if(my_best_reward < reward_sum) {
                    my_best_reward = reward_sum;
                    my_best_score = cball;
                    my_best_step_sum = step_sum;
                }

                Mutex::scoped_lock lock(*mutex);//acquire

                if(reward_sum > *bestRewardEpisode) { //Although he updates the policy, but in case of TESTPERF, he doesn't write it - check line 508
                    if(bestEpisode != nullptr)
                        delete bestEpisode;
                    bestEpisode = algo.copyPolicy();
                    *bestRewardEpisode = reward_sum;
                }

                avg->score.push_back(cball);
                avg->reward_sum.push_back(reward_sum);
                avg->step_sum.push_back(step_sum);

                avg->personal_best_score.push_back(my_best_score);
                avg->personal_best_reward_sum.push_back(my_best_reward);
                avg->personal_best_step_sum.push_back(my_best_step_sum);

                avg->populate ++;

                bool final_thread = avg->populate == number_thread;

                lock.release();


                if(final_thread) {
                    Mutex::scoped_lock lock(*mutex);//acquire

                    //statistics
                    std::pair<double, double> mv_score = Utils::mean_and_var<std::list<double>>(avg->score);
                    std::pair<double, double> mv_reward_sum = Utils::mean_and_var<std::list<double>>(avg->reward_sum);
                    std::pair<double, double> mv_step_sum = Utils::mean_and_var<std::list<int>>(avg->step_sum);

                    std::pair<double, double> mv_personal_best_score = Utils::mean_and_var<std::list<double>>(avg->personal_best_score);
                    std::pair<double, double> mv_personal_best_reward_sum = Utils::mean_and_var<std::list<double>>(avg->personal_best_reward_sum);
                    std::pair<double, double> mv_personal_best_step_sum = Utils::mean_and_var<std::list<int>>(avg->personal_best_step_sum);


#if !defined(VISU) && defined(NDEBUG)
                    if(episode % 10 == 0)
#endif
                    {
                        int hs = algo.history_size();
                        double ws = algo.weight_sum();
                        double ms = algo.mse();
                        LOG_DEBUG(std::left << std::setw(7) << std::setfill(' ') << episode <<
                                  std::left << std::setw(4) << std::setfill(' ') << std::setprecision(2) << mv_score.first <<
                                  std::left << std::setw(10) << std::setfill(' ') << mv_reward_sum.first <<
                                  std::left << std::setw(7) << std::setfill(' ') << std::setprecision(3) << algo.getParams().epsilon <<
                                  std::left << std::setw(10) << std::setfill(' ') << hs <<
                                  std::left << std::setw(13) << std::setfill(' ') << ws <<
                                  std::left << std::setw(13) << std::setfill(' ') << ms) ;

                    }

		    if(mv_score.first == 0.)
			(*stop_simulation)++;
		    else (*stop_simulation) = 0;

//		    if((*stop_simulation) >= 600 && episode > 2000){
//			LOG_DEBUG("useless simulation perf 0 during 200 epochs");
//			LOG_FILE("time_elapsed", "-1");
//			exit(1);
//		    }
#ifdef TESTPERF //This ifdef is added by Omar, in order just to have different log files names
                    LOG_FILE("statRL_NN_TESTPERF.dat", episode << " " <<
                             mv_score.first << " " << mv_score.second << " " <<
                             mv_personal_best_score.first << " " <<  mv_personal_best_score.second << " " <<
                             mv_reward_sum.first << " " << mv_reward_sum.second << " " <<
                             mv_personal_best_reward_sum.first << " " << mv_personal_best_reward_sum.second << " " <<
                             mv_step_sum.first << " " << mv_step_sum.second << " " <<
                             mv_personal_best_step_sum.first << " " <<  mv_personal_best_step_sum.second << " " <<
                             number_thread);

#else
                    LOG_FILE("statRL_NN_TESTPERF.dat", episode << " " <<
                             mv_score.first << " " << mv_score.second << " " <<
                             mv_personal_best_score.first << " " <<  mv_personal_best_score.second << " " <<
                             mv_reward_sum.first << " " << mv_reward_sum.second << " " <<
                             mv_personal_best_reward_sum.first << " " << mv_personal_best_reward_sum.second << " " <<
                             mv_step_sum.first << " " << mv_step_sum.second << " " <<
                             mv_personal_best_step_sum.first << " " <<  mv_personal_best_step_sum.second << " " <<
                             number_thread);
#endif // TESTPERF

                    avg->score.clear();
                    avg->reward_sum.clear();
                    avg->step_sum.clear();

                    avg->personal_best_score.clear();
                    avg->personal_best_reward_sum.clear();
                    avg->personal_best_step_sum.clear();

                    avg->populate = 0;
                    lock.release();

                    my_condition->notify_all();
                } else {
                    tbb::interface5::unique_lock<tbb::mutex> ul(*sync_mutex);
                    my_condition->wait(ul);
                }

                //after everybody sync
#ifndef TESTPERF
                if(episode % 200 == 0 && episode > 0 && bestEpisode != nullptr) {

                    Mutex::scoped_lock lock(*mutex);//acquire
                    if(*bestRewardEpisode == my_best_reward) {
                        std::string filename("episode_");
                        std::string filename2 = std::to_string(episode);
                        std::string extension(".dat");

                        bestEpisode->write(filename + filename2 + extension);

#if defined(RANDACLINEAR) || defined(RANDACFIXED)
                        std::string filename3(".ac");
                        ActionFactory::getInstance()->write(actions, filename + filename2 + extension + filename3);
#endif

#ifdef RANDSTATE
			std::string filename4(".st");
			stx->write(filename + filename2 + extension + filename4);
#endif

                    }
                    lock.release();
                }
#endif

#ifdef RANDOM
                exit(0);
#endif
            }

            Simulator::endInstance(simu);

            if(bestEpisode != nullptr)
                delete bestEpisode;
        }
    };

public:

    void run(int argc, char **argv,
             const boost::program_options::options_description& add_opts = boost::program_options::options_description())
    {

        //init singleton
        bib::Seed::getInstance();

        Simulator::singletonInit();




        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini("configRL.ini", pt);
#ifdef BOLTZMANN
	rlparam.DefaultParam.temperature = pt.get<double>("reinforcement_learning.temperature");
#endif
        rlparam.starting_epsilon = pt.get<double>("reinforcement_learning.starting_epsilon");
        rlparam.ending_epsilon = pt.get<double>("reinforcement_learning.ending_epsilon");
        rlparam.stepness_epsilon = pt.get<double>("reinforcement_learning.stepness_epsilon");
        rlparam.DefaultParam.gamma = pt.get<double>("reinforcement_learning.gamma");

        rlparam.DefaultParam.epsilon=0;
        rlparam.DefaultParam.alpha = pt.get<double>("reinforcement_learning.alpha");

        rlparam.DefaultParam.hidden_unit = pt.get<int>("neural_network.hidden_unit");
        rlparam.DefaultParam.activation = pt.get<std::string>("neural_network.activation_function_hidden");
        rlparam.DefaultParam.activation_stepness = pt.get<double>("neural_network.activation_steepness_hidden");

        rlparam.DefaultParam.repeat_replay = pt.get<int>("trace.replay");

        rlparam.max_episod = pt.get<int>("simulation.max_episod");
        rlparam.number_parallel_mean = pt.get<int>("simulation.number_of_mean");

	rlparam.timestep_decision_min = pt.get<int>("action.extension_min");
	rlparam.timestep_decision_max = pt.get<int>("action.extension_max");
#ifdef RANDSTATE
	rlparam.history_length_max = pt.get<int>("state.history_max");
	rlparam.DefaultParam.memory_size = rlparam.history_length_max;
#endif

#ifdef BOLTZMANN
        LOG_INFO("BOLTZMANN policy");
#else
        LOG_INFO("GREEDY policy");
#endif

        //define program options
        namespace po = boost::program_options;
        po::options_description desc("Allowed RLAdhoc options");
        desc.add(add_opts);
        desc.add_options()
#ifdef LOADFILE
        ("load", po::value<std::string>(), "set the policy to load")
#endif
        ("actions", po::value<std::string>(), "set the actions file to load")
        ("na", po::value<int>(), "set the number of actions to load")
        ("states", po::value<std::string>(), "set the states file to load");


        po::variables_map vm;
        po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
        //po::store(po::parse_command_line(argc, argv).options(desc).allow_unregistered(), vm);
        po::store(parsed, vm);
        po::notify(vm);

#ifdef LOADFILE
        if(!vm.count("load")) {
            std::cout << "Policy not set." << std::endl;
            exit(1);
        }
#endif

#if !defined(RANDACLINEAR) && !defined(RANDACFIXED)
        if(!vm.count("actions")) {
            std::cout << "Actions file not set. What are my actions?" << std::endl;
            exit(1);
        }
#endif

        if(!vm.count("na")) {
            std::cout << "How many actions should I load? " << std::endl;
            exit(1);
        }

#if !defined(RANDACLINEAR) && !defined(RANDACFIXED)
        sml::ActionFactory::getInstance()->injectArgs(vm["actions"].as<std::string>(), SimuParam::nb_motors, vm["na"].as<int>());
#else
        sml::ActionFactory::getInstance()->injectArgs(vm["na"].as<int>());
#endif


#ifndef RANDSTATE
        StateExtractor stx(SimuParam::sensor_size, SimuParam::nb_motors);
        if(vm.count("states")) {
            stx.initParser(vm["states"].as<std::string>());
            std::cout << "Number of input state set to " << stx.getNumberInput() << std::endl;
        } else {
            std::cout << "State file not set. What are my states?" << std::endl;
            exit(1);
        }

	rlparam.history_length_max = ceil(((double)stx.maxIndex()+1)/((double)(SimuParam::sensor_size + SimuParam::nb_motors)));
	rlparam.DefaultParam.memory_size = rlparam.history_length_max;
#endif

#ifdef NO_PARALLEL
        rlparam.number_parallel_mean = 1;
#endif

        Average avg;
        Mutex mutex;
        tbb::mutex sync_mutex;
        condition_variable my_condition;
        double bestRewardEpisode = -5000000000;
	int stop_simulation = 0;

        AverageRun avgr;
        avgr.avg = &avg;
        avgr.number_thread = rlparam.number_parallel_mean;
        avgr.mutex = &mutex;
        avgr.sync_mutex = &sync_mutex;
        avgr.my_condition = &my_condition;
        avgr.bestRewardEpisode = &bestRewardEpisode;
        avgr.rlparam = &rlparam;
	avgr.stop_simulation = &stop_simulation;

#ifndef RANDSTATE
        avgr.stx = &stx;
#endif


#ifdef LOADFILE
        avgr.loading_file = vm["load"].as<std::string>();
#endif

//
// CONFIG FILE
//
#ifndef SARSA
        LOG_FILE("config", "NO DEF SARSA");
#else
        LOG_FILE("config", "DEF SARSA");
#endif
#if defined(RANDACLINEAR) || defined(RANDACFIXED)
        LOG_FILE("config", "DEF RANDOM AC");
#else
        LOG_FILE("config", "NO DEF RANDOM AC");
#endif
#ifndef RANDSTATE
        LOG_FILE("config", "NO DEF RANDOM STATE");
#else
        LOG_FILE("config", "DEF RANDOM STATE");
#endif
        LOG_FILE("config", "HIDDEN UNITS : " << rlparam.DefaultParam.hidden_unit);
        LOG_FILE("config", "I [ " << rlparam.timestep_decision_min << " ; " << rlparam.timestep_decision_max << " ]");
        LOG_FILE("config", "GAMMA : " << rlparam.DefaultParam.gamma );
        LOG_FILE("config", "ALPHA : " << rlparam.DefaultParam.alpha );
        LOG_FILE("config", "EPSILON start" << rlparam.starting_epsilon);
        LOG_FILE("config", "EPSILON end" << rlparam.ending_epsilon);
        LOG_FILE("config", "repeat_replay : " << rlparam.DefaultParam.repeat_replay );
        LOG_FILE("config", "memory_size : " << rlparam.DefaultParam.memory_size );
        LOG_FILE("config", "history state length :" << rlparam.history_length_max);
//        LOG_FILE("config", "number_random_state :" << SimuParam::number_random_state);
#ifndef RANDSTATE
        LOG_FILE("config", "state nf input : " << stx.getNumberInput());
        LOG_FILE("config", "state file : " << vm["states"].as<std::string>() );
#endif
        LOG_FILE("config", "action number : " << sml::ActionFactory::getInstance()->getActionsNumber());
#if !defined(RANDACLINEAR) && !defined(RANDACFIXED)
        LOG_FILE("config", "action file : " << vm["actions"].as<std::string>());
#endif
        LOG_FILE("config", "number of thread : " << rlparam.number_parallel_mean );

	using namespace std::chrono;
	high_resolution_clock::time_point begin = high_resolution_clock::now();

	unsigned concurentThreadsSupported = std::thread::hardware_concurrency();
	static tbb::task_scheduler_init* init;
	if(concurentThreadsSupported == 0 ) //error
	   init = new tbb::task_scheduler_init(rlparam.number_parallel_mean);
	else init = new tbb::task_scheduler_init(rlparam.number_parallel_mean + concurentThreadsSupported);


#ifdef NO_PARALLEL
        avgr.operator()();
#else
        tbb::parallel_for(tbb::blocked_range<int>(0, rlparam.number_parallel_mean, 1), avgr);

	high_resolution_clock::time_point end = high_resolution_clock::now();
	duration<double> time_span = duration_cast<duration<double>>( end - begin);
	LOG_FILE("time_elapsed", ""<<(double) (time_span.count() / 60.f)); //in minutes

        init->terminate();
#endif

        ActionFactory::endInstance();
        bib::Seed::endInstance();
        bib::Logger::endInstance();
        delete init;
    }


};

#endif
