#ifndef SARSANN_H
#define SARSANN_H

//#include <typeinfo>

#include "Utils.hpp"
#include <set>
#include <vector>

#include "doublefann.h" //--> This will make the NN output as a double --> fann_type == double
#include <boost/graph/graph_concepts.hpp>

#include <iostream>
#include <string>
#include <sstream> // Added by Omar in order to make more robust logging!
#include <deque> // For the experience replay


#define MAX_REPLAY_FOR_ON_TRAJECTORY 20

using std::pair;
using namespace std;


namespace sml {

struct RLParam {
    double epsilon;
    double alpha;
    double gamma;
    int repeat_replay;
    int memory_size;

    int hidden_unit;
    double activation_stepness;
    std::string activation;
};

  struct LearnReturn {
      int ac;
      bool gotGreedy;
  };

  struct Experience_replay
  {
      typedef std::vector<double> experience;

  };

using namespace sml;
typedef struct fann* NN;

template <class State>
class SarsaNN
{
/*
  For now, I will not make any history reply --> I need something basic that I can debug, and I need also to understand how history reply works.
*/

public:
    SarsaNN(int num_actions_options, RLParam param, unsigned int size_input_state , std::vector<int> _actions_time) :
        param(param), num_actions_options(num_actions_options), neural_networks(num_actions_options),size_input_state(size_input_state), actions_time(_actions_time)

    {
        // Note that num_actions_options is the number of NN we have
        this->Q_Table = new double [this->num_actions_options]; // This will contain the Q-values from each of the neural networks

        for(int i=0; i < this->num_actions_options ; i++) {
            neural_networks[i] = fann_create_standard(3, size_input_state, param.hidden_unit, 1);

            if(param.activation == "tanh")
                fann_set_activation_function_hidden(neural_networks[i], FANN_SIGMOID_SYMMETRIC);
            else if(param.activation == "sigmoid")
                fann_set_activation_function_hidden(neural_networks[i], FANN_SIGMOID);
            else if(param.activation == "linear")
                fann_set_activation_function_hidden(neural_networks[i], FANN_LINEAR);
            else {
                LOG_ERROR("activation function for hidden layer of the neural network unknown : " << param.activation_stepness);
                exit(1);
            }

            fann_set_activation_steepness_hidden(neural_networks[i], param.activation_stepness);
            fann_set_activation_function_output(neural_networks[i], FANN_LINEAR);//Linear cause Q(s,a) isn't normalized

            fann_set_learning_momentum(neural_networks[i], 0.);
            fann_set_train_error_function(neural_networks[i], FANN_ERRORFUNC_LINEAR);
	    fann_set_training_algorithm(neural_networks[i], FANN_TRAIN_INCREMENTAL);


            fann_set_train_stop_function(neural_networks[i], FANN_STOPFUNC_MSE);
            fann_set_learning_rate(neural_networks[i], this->param.alpha);
        }

        //LOG_DEBUG(weight_sum());
    }

    ~SarsaNN() {
        for(int i=0; i < this->num_actions_options ; i++)
            fann_destroy(neural_networks[i]);

        delete [] Q_Table;
    }


    void startEpisode(const State& s, const int a) {
      //Added by Omar for debugging
        q_max << "max=";
        q_avg << "avg=";
        /////////////////////////////////////////
        lastAction = a;
        lastState = s;

        //start an new episod
        computeQa(s);
        internal_step = 0;
    }

    void endEpisode(double rr) {
        #ifdef TESTPERF
        std::cout << q_max.str() << std::endl;
        std::cout << q_avg.str() << std::endl;
        #endif // TESTPERF
        q_max.str(std::string()); //Clearing the stringstream
        q_avg.str(std::string()); //Clearing the stringstream
    }

    int decision(const State& state, bool greedy) {
        if(greedy && sml::Utils::rand01(this->param.epsilon)  ) {
            q_max << "???,";
            q_avg << "???,";
            return rand() % (int)this->num_actions_options;
        }

        computeQa(state);

        return this->Q_Table_MAX();

    }

//     long internal_step = 0;
    LearnReturn _learn(const State& state, double r, bool goal)
    {
        int a = this->lastAction;

        double delta = r;

        // For all a in A(s')
        computeQa(state);

        bool gotGreedy = false;

        int ap = this->Q_Table_MAX();
        if( sml::Utils::rand01(this->param.epsilon)) {
            ap = rand() % this->num_actions_options;
            gotGreedy = true;
        }

        if(!goal) {
            // Notice here that each action can have different time
            delta = delta + pow(this->param.gamma, actions_time[a]) * this->Q_Table[ap];
        }
        else{
            cout << "Goal achieved : " << delta << endl;
        }


        fann_type inputs[size_input_state];
        fann_type out[1];

        // cout << "Last state = " ;
        for(int i=0; i < size_input_state ; i++){
            inputs[i] = lastState->at(i);
            // cout << inputs[i] << " , ";
        }
        // cout << endl;
        out[0] = delta;

        // if(internal_step > 0)
        fann_train(neural_networks[a], inputs, out); //So, you only train the NN of the current selected action

        // take action a, observe reward, and next state
        // delete this->lastAction;
        this->lastAction = ap;
        lastState = state;

        // cout << "Current reward (delta) = " << delta << endl;

        /*
        This is for debugging purpose
        */
        if (goal){
            computeQa(state);
            this->printQvalues();
        }
        return {ap, gotGreedy};
    }

    double mse() {
#ifndef NDEBUG
        double s=0.;
        for(int i=0; i < this->num_actions_options ; i++) {
            s += fann_get_MSE(neural_networks[i]);
            fann_reset_MSE(neural_networks[i]);
        }
        return s/this->num_actions_options;
#else
        return 0;
#endif
    }

    void write(const string& chemin) {
        for(int i=0; i < this->num_actions_options; i++) {
            fann_save(neural_networks[i], (chemin+"."+std::to_string(i)).c_str());
        }
    }

    void read(const string& chemin) {
        for(int i=0; i<this->num_actions_options; i++) {
            neural_networks[i] = fann_create_from_file((chemin+"."+std::to_string(i)).c_str());
        }
    }

    double weight_sum() {
        #ifndef NDEBUG
        double sum = 0.f;
        for(int i=0; i < this->num_actions_options; i++) {
            struct fann_connection* connections = (struct fann_connection*) calloc(fann_get_total_connections(neural_networks[i]), sizeof(struct fann_connection));

            for(int j=0; j<fann_get_total_connections(neural_networks[i]); j++)
                connections[j].weight=0;

            fann_get_connection_array(neural_networks[i], connections);

            for(int j=0; j<fann_get_total_connections(neural_networks[i]); j++)
                sum += std::fabs(connections[j].weight);

            free(connections);
        }


        return sum/this->num_actions_options;
        #else
        return 0;
        #endif
    }

    RLParam& getParams() {
        return param;
    }

    int learn(const State& s, double reward, bool goal)
    {
        return _learn(s, reward, goal).ac;
    }

    void printQvalues(){
        for (int i = 0 ; i < this->num_actions_options ; i++){
            cout << "Q_table[" << i << "] = " << this->Q_Table[i] << endl;
        }
    }

protected:

    void computeQa(const State& state) {
        //std::cout << "COMPUTE QA" << std::endl;
        fann_type inputs[size_input_state];

        for(int i=0; i < size_input_state ; i++)
            inputs[i] = state->at(i);

        //Let's run all the NN here in series, not in parallel
        for (int NN_num = 0; NN_num < this->num_actions_options ; NN_num++){
          fann_type* out = fann_run(this->neural_networks[NN_num], inputs);
          this->Q_Table[NN_num] = out[0];
        }

    }

    int Q_Table_MAX(){
      /* This will get the argmax element in the Q-Table */
      //TODO: I should modify the return value, so that I can return the Q-value itself as well
      int arg = 0;
      fann_type max_output = this->Q_Table[0];
        for (int i = 1 ; i < this->num_actions_options ; i++){
            if (this->Q_Table[i] > max_output){
              max_output = this->Q_Table[i];
              arg = i;
            }
        }
        return arg;
    }

    int Q_Table_MIN(){
      /* This will get the argmin element in the Q-Table */
      //TODO: I should modify the return value, so that I can return the Q-value itself as well
      int arg = 0;
      fann_type min_output = this->Q_Table[0];
        for (int i = 1 ; i < this->num_actions_options ; i++){
            if (this->Q_Table[i] < min_output){
              min_output = this->Q_Table[i];
              arg = i;
            }
        }
        return arg;
    }

protected:
    RLParam param;
    int num_actions_options;
    fann_type *Q_Table; // This will replace the QTable data structure
    vector<NN> neural_networks;
    unsigned int size_input_state;


    int lastAction;
    State lastState;
    int internal_step;


    //Added by Omar for debugging purposes
    std::stringstream q_max;
    std::stringstream q_avg;


    std::vector<int> actions_time;
//    RLAllParam rlparam;
};

}

#endif // SARSANN_H
