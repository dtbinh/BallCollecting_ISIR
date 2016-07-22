/*
This is an implementation of SARSA, based on Linear-FA, from
http://artint.info/html/ArtInt_272.html
But, I will have a different LinearFA for each action - Like what Matthieu did before
*/

//#include <typeinfo>

#include "Utils.hpp"
#include <set>
#include <vector>

#include <boost/graph/graph_concepts.hpp>

#include <iostream>
#include <string>
#include <sstream> // Added by Omar in order to make more robust logging!
#include <deque> // For the experience replay
#include <algorithm>    // std::min_element, std::max_element



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

using namespace sml;

template <class State>
class LinearFA
{
/*
  For now, I will not make any history reply --> I need something basic that I can debug, and I need also to understand how history reply works.
*/

typedef tuple<State, int, double, State> experience;
// struct Experience_replay
// {
//     std::vector<State> experience_dataset;
//     int experience_depth = 30;
//     bool reward_achieved = false;
//     void insert_experience (State state1, int action_num, double reward, State state2){
        
//         Here, I record an entire experience in the form of (s, a, r, s')
        
//         experience_dataset.push_back(input);
//         while (experience_dataset.size() > experience_depth){
//             experience_dataset.erase(experience_dataset.begin());
//         }
//     }

    
// };

typedef vector<double> LinearModel;

public:
    LinearFA(int num_actions_options, RLParam param, unsigned int size_input_state , std::vector<int> _actions_time) :
        param(param), num_actions_options(num_actions_options), AllLinearModels(num_actions_options), size_input_state(size_input_state), actions_time(_actions_time)

    {
        // Note that num_actions_options is the number of NN we have
        this->Q_Table = new double [this->num_actions_options]; // This will contain the Q-values from each of the neural networks

        for (int action_index = 0; action_index < this->num_actions_options; action_index ++){
            for (int state_index = 0; state_index < this->size_input_state + 1; state_index ++){ // The extra 1 is for the bias weight (W0)
                // I will initialize the weights to zero here. It could be important to make random initiailization (not sure)
                // this->AllLinearModels[action_index].push_back(0.0);
                double my_rand = ((double) rand() / (RAND_MAX));
                // this->AllLinearModels[action_index].push_back(my_rand);
                this->AllLinearModels[action_index].push_back(0.0);
            }
        }
    }

    ~LinearFA() {
        delete [] Q_Table;
    }


    void startEpisode(const State& s, const int a) {
        //Added by Omar for debugging
        q_max << "max=";
        q_avg << "avg=";
        /////////////////////////////////////////
        this->lastAction = a;
        this->lastState = s;

        //start an new episod
        computeQa(s);
        internal_step = 0;
        this->printModelParameters();
    }

    void endEpisode(double rr) {
        #ifdef TESTPERF
        std::cout << q_max.str() << std::endl;
        std::cout << q_avg.str() << std::endl;
        #endif // TESTPERF
        q_max.str(std::string()); //Clearing the stringstream
        q_avg.str(std::string()); //Clearing the stringstream

        // this->printModelParameters();
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

    LearnReturn _learn(const State& state, double r, bool goal)
    {
        /*
        carry out action a 
        observe reward r and state s' 
        select action a' (using a policy based on Qw) 
        let δ= r+γQw(s',a')-Qw(s,a) 
        for i=0 to n do 
            wi ←wi + ηδFi(s,a) 
         
        s ←s' 
        a ←a' 
        */

        computeQa(state);

        bool gotGreedy = false;

        int next_action = this->Q_Table_MAX();
        if( sml::Utils::rand01(this->param.epsilon)) {
            next_action = rand() % this->num_actions_options;
            gotGreedy = true;
        }

        /*
        This equation is from
        http://computing.dcu.ie/~humphrys/PhD/ch4.html#4.3.2
        Note however, that is different from 
        http://artint.info/html/ArtInt_272.html
        */
        double delta = r;
        double prev_qValue = this->computeQa_rtnQValue(this->lastState, this->lastAction);
        // delta = r + (this->param.gamma * this->Q_Table[next_action]) - prev_qValue;
        delta = r + (this->param.gamma * this->Q_Table[next_action]);

        double inputs[size_input_state];
        double out = delta;

        cout << "Sensory input: " ;
        for(int i=0; i < size_input_state ; i++){
            inputs[i] = this->lastState->at(i);
            cout << inputs[i] << " , ";
        }
        cout << endl;

        // fann_train(neural_networks[this->lastAction], inputs, out); //So, you only train the NN of the current selected action
        // Train the linear model here
        // http://artint.info/html/ArtInt_272.html
        
        this->linear_model_train(this->AllLinearModels[this->lastAction] , inputs , delta);

        // take action a, observe reward, and next state
        this->lastAction = next_action;
        this->lastState = state;

        /*
        This is for debugging purpose
        */
        // if (goal){
        //     computeQa(state);
        //     this->printQvalues();
        // }
        cout << "reward=" << r << endl;
        cout << "prev_qValue=" << prev_qValue << endl;
        cout << "this->Q_Table[next_action]=" << this->Q_Table[next_action] << endl;
        cout << "this->param.gamma=" << this->param.gamma << endl;
        cout << "delta=" << delta << endl;
        cout << "next_action=" << next_action << endl;
        this->printModelParameters();
        cout << "----------------------------------------------------------------" << endl;
        return {next_action, gotGreedy};
    }


    void write(const string& chemin) {
    }

    void read(const string& chemin) {
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
    void printModelParameters (){
        for (int i = 0; i < this->num_actions_options; i++){
            cout << "Model #." << i << endl;
            for (int x = 0; x < size_input_state + 1; x++){
                // cout << "x : " << x << " --> ";
                cout << this->AllLinearModels[i][x] << " , ";
            }
            cout << endl;
        }
    }
    void linear_model_train (LinearModel &my_model, double * inputArray, double & delta){
        /*
        I will fix the step size to be 1 for now, till further notice
        */
        // ERROR : I am not updating the value of the offset wi
        for(int i=1; i < size_input_state + 1; i++){
            my_model[i] = my_model[i] + 0.05 * delta * inputArray[i - 1];
        }
    }

    double computeQa_rtnQValue(const State& state, int action) {
        double inputs[size_input_state];
        double *Q_Table_temp; // This will replace the QTable data structure
        Q_Table_temp = new double[this->num_actions_options];

        for(int i=0; i < size_input_state ; i++)
            inputs[i] = state->at(i);

        //Let's run all the NN here in series, not in parallel
        for (int model_num = 0; model_num < this->num_actions_options ; model_num++){
            double out = this->linear_model_run(AllLinearModels[model_num], inputs);
            Q_Table_temp[model_num] = out;
        }

        int argmax = distance(Q_Table_temp, std::max_element(Q_Table_temp, Q_Table_temp+ this->num_actions_options));
        double maxQvalue = Q_Table_temp[argmax];
        delete [] Q_Table_temp;
        return maxQvalue;
    } 

    void computeQa(const State& state) {
        double inputs[size_input_state];

        for(int i=0; i < size_input_state ; i++)
            inputs[i] = state->at(i);

        //Let's run all the NN here in series, not in parallel
        for (int model_num = 0; model_num < this->num_actions_options ; model_num++){
            double out = this->linear_model_run(AllLinearModels[model_num], inputs);
            this->Q_Table[model_num] = out;
        }

    }

    double linear_model_run(LinearModel &my_model, double * input){
        double output = 0.0;
        output += my_model[0];
        for (int i = 1; i < size_input_state+1; i++){
            output += my_model[i] * input[i-1];
        }
        return output;
    }

    int Q_Table_MAX(){
        /* This will get the argmax element in the Q-Table */
        
        // A neat way to get the index of the max element of an array
        // http://stackoverflow.com/questions/2953491/finding-the-position-of-the-max-element
        return distance(this->Q_Table, std::max_element(this->Q_Table, this->Q_Table + this->num_actions_options));
        
    }

    int Q_Table_MIN(){
        /* This will get the argmin element in the Q-Table */
        return distance(this->Q_Table, std::min_element(this->Q_Table, this->Q_Table + this->num_actions_options));
    }

protected:
    RLParam param;
    int num_actions_options;
    double *Q_Table; // This will replace the QTable data structure
    vector <LinearModel> AllLinearModels;
    unsigned int size_input_state;


    int lastAction;
    State lastState;
    int internal_step;


    //Added by Omar for debugging purposes
    std::stringstream q_max;
    std::stringstream q_avg;


    std::vector<int> actions_time;
    // Linear models characteristic
    int num_weights;
};

}