//#ifndef RUN_RL_HPP
//#define RUN_RL_HPP

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

#include "SarsaNN.hpp" // This is temporarily, for testing purpose
#include "ActionFactory.hpp"
#include "StateExtractor.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <sstream> // Added by Omar in order to make more robust logging!
#include <tuple>

typedef boost::shared_ptr<vector<double>> EnvState;

using namespace sml;

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
class RL_run2 {
public:
    static constexpr int nb_max_inputs_on_time = SimuParam::sensor_size + SimuParam::nb_motors;

    struct Average {
      /*
        The purpose of this datastructure is to collect data during the run, in order to perform
        statistics on it later.
      */
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

    RL_run2(int RewardNumber, int FinalGoal, bool UseOptions, int NumOptions){
        this->RewardNumber = RewardNumber;
        this->FinalGoal = FinalGoal;
        this->ActionFactoryInstance = new ActionFactory(3); // 3 is the number of motors
        this->nb_options = NumOptions;
        this->optionsUsed = UseOptions; // This variable will decide if opt
    }

    ~RL_run2(){
        delete ActionFactoryInstance;
    }

    void initializeEnvironment(int argc, char **argv,
             const boost::program_options::options_description& add_opts = boost::program_options::options_description()){
        /********************************************************************************************
            Read the configuration file - get the RL parameters
            +
            Load the relevant information (states, actions, map,..etc).
        ********************************************************************************************/
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini("configRL.ini", pt);
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

        namespace po = boost::program_options;
        po::options_description desc("Allowed RLAdhoc options");
        desc.add(add_opts);
        desc.add_options()
        #ifdef LOADFILE
        ("load", po::value<std::string>(), "set the policy to load")
        #endif
        ("actions", po::value<std::string>(), "set the actions file to load")
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

        if(!vm.count("actions")) {
            std::cout << "Actions file not set. What are my actions?" << std::endl;
            exit(1);
        }

        // Inject the loaded action into the actions factory

        // Extract the states from the states files
        this->stx = new StateExtractor(SimuParam::sensor_size, SimuParam::nb_motors);
        if(vm.count("states")) {
            this->stx->initParser(vm["states"].as<std::string>());
            std::cout << "Number of input state set to " << this->stx->getNumberInput() << std::endl;
        } else {
            std::cout << "State file not set. What are my states?" << std::endl;
            exit(1);
        }


        // TODO: Add the facilities to 'LOG' the summary about the RL run.

        /********************************************************************************************
            Get the primitive actions from the file
        ********************************************************************************************/
        this->primitive_actions = ActionFactoryInstance->read(vm["actions"].as<std::string>());
        this->nb_primitive_actions = ActionFactoryInstance->getActionsNumber();

        for (int i = 0; i < this->nb_primitive_actions ; i++) // Note that this is for primitive actions only at the moment
          actions_time.push_back(this->primitive_actions[i].temporal_extension);

        // I must add here a for loop to get the max timing for options
        // if (this->optionsUsed == true)
        //   actions_time[i].push_back(this->primitive_actions[i].temporal_extension);

        /********************************************************************************************
            Build the policy function (with value function approximator inside)
        ********************************************************************************************/
        if (this->optionsUsed == true)
          this->algo = new sml::SarsaNN<EnvState> (this->nb_options + this->nb_primitive_actions, rlparam.DefaultParam, this->stx->getNumberInput(), this->actions_time);
        else
          this->algo = new sml::SarsaNN<EnvState> (this->nb_primitive_actions, rlparam.DefaultParam, this->stx->getNumberInput(), this->actions_time);
        #ifdef LOADFILE
          this->algo->read(loading_file);
        #endif

        /********************************************************************************************
          Get an instance of the environment simulator
        ********************************************************************************************/
        this->simu = Simulator::simuInit();
        // At the end of all this processing, you will have:
        // - list_tlaction this->primitive_actions --> list of primitive actions
        // - this->algo --> your SARSA policy
        // nb_primitive_actions --> number of primitive actions
        // this->simu --> A nullptr instance of the simulator
    }

    void runOneTimeStep(int timedecision){
        /*
            This will progress the simulator by one time step only
        */
    }

    void runOneEpisode(int episode){
      /*
          This will progress the simulator for an entire episode
      */
      this->cball = 0;
      this->reward_sum = 0;
      this->step_sum = 0;

      //This is the decay in the epsilon with time (less exploration with time)
      this->algo->getParams().epsilon =  (rlparam.starting_epsilon - rlparam.ending_epsilon)*pow(rlparam.stepness_epsilon, 1+episode) + rlparam.ending_epsilon;

      #if defined(LOADFILE) && ! defined(TESTPERF)
      algo->getParams().epsilon =  rlparam.ending_epsilon;
      #endif

      this->simu = Simulator::simuInitInside(this->simu, instance);

      int ac = 0; // This is the index of the current action
      ////////
      std::vector<double> motor_init(SimuParam::nb_motors, 0.5);
      std::vector<double>* tmp_inp_ini = new vector<double>(rlparam.nbInputsTotal(nb_max_inputs_on_time));
      Simulator::computeIniInputs(tmp_inp_ini, simu, motor_init);

      for(int i=nb_max_inputs_on_time; i < tmp_inp_ini->size(); i++)
          tmp_inp_ini->operator[](i) = tmp_inp_ini->at(i % nb_max_inputs_on_time);

      boost::shared_ptr< std::vector<double> > complete_input(tmp_inp_ini);
      boost::shared_ptr< std::vector<double> > _inputs(this->stx->parse(*complete_input));

      this->algo->startEpisode(_inputs, ac);

      this->reward = Simulator::computeReward(this->simu, this->RewardNumber); //This is the number of balls collected. It is in RLNNACST
      this->rreward = 0;
      this->powgamma = 1.d;
      int step;
      for (step = 0; true; ){
          _inputs.reset(this->stx->parse(*complete_input));
          #ifndef TESTPERF
          ac = algo->learn(_inputs, reward, simu->end()); // The end here should take a parameter in the future, indicating the end condition
          #else //TESTPERF
          ac = algo->decision(_inputs, true);
          #endif

          if(simu->end() || step >= Simulator::simu_max_episode())
              break;

          int timestep_decision = 0;
          if (ac < this->nb_primitive_actions){ // This means we selected a primitve action
              timestep_decision = this->primitive_actions.at(ac).temporal_extension;
          }
          else{ // This means we selected an option, so I need its correct timing (this will be the max time allowed for an option)

          }
          this->reward = 0;

          for(int timestep=0; timestep < timestep_decision; timestep ++) {

              boost::scoped_ptr< std::vector<double> > outputs(ActionFactory::computeOutputs(ac, timestep, primitive_actions)); //This should deal with primitive actions only
              Simulator::step_simu(this->simu, *outputs);
              double lreward = Simulator::computeReward(this->simu, this->RewardNumber);
              this->reward += lreward * pow(rlparam.DefaultParam.gamma, timestep);
              this->rreward += lreward * powgamma;
              this->powgamma *= rlparam.DefaultParam.gamma;

              if(simu->end()){ //In the end() here, I must give it the condition number (for the options sake)
                  timestep_decision = timestep + 1;
                  break;
              }
          }
          step += timestep_decision;
      }
      step_sum += step;

      cball += Simulator::simu_perf(simu);
      this->algo->endEpisode(rreward, Simulator::instanceToInt(instance)); //TODO: I need to add the instance to the code.
    }

    void runAllSteps(){
        /*
            This will run the simulator till either the time budget expires, or the finalGoal condition is satisfied.
        */
        this->instance = Simulator::instanceIteratorBegin(); // since this is only one instance
        for(int episode=0; episode < rlparam.max_episod; episode++) {
          this->runOneEpisode(episode);
          this->avg.score.push_back(cball);
          this->avg.reward_sum.push_back(reward_sum);
          this->avg.step_sum.push_back(step_sum);

          //this->avg.personal_best_score.push_back(my_best_score);
          //this->avg.personal_best_reward_sum.push_back(my_best_reward);
          //this->avg.personal_best_step_sum.push_back(my_best_step_sum);

          this->avg.populate++;
        }
    }

private:
    int RewardNumber; //This will determine the reward function to be used
    int FinalGoal; // This to determine the 'end of simulation' function to be used
    totalActionList primitive_actions; // TODO: I am not sure if this syntax is correct or not
    int nb_primitive_actions;
    std::string loading_file; // This is the file to load the action from
    int nb_options; // This is the number of options used
    bool optionsUsed; // This variable will decide if options will be used in this RL problem or not
    sml::ActionFactory *ActionFactoryInstance;
    vector<int> actions_time;
    sml::SarsaNN<EnvState> *algo;
    Environnement simu;
    InstanceIterator instance;

    // These variables are more related to the learning process itself.
    double reward;
    double rreward;
    double powgamma;
    Average avg;
    RLAllParam rlparam;
    StateExtractor *stx;
    double cball;
    double reward_sum;
    int step_sum;
};
