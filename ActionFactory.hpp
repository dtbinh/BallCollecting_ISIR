#ifndef ACTIONFACTORY_H
#define ACTIONFACTORY_H

#include "ActionFactory.hpp"
#include "Utils.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <exception>

namespace sml {

typedef std::vector <double> singleMotorCommand;
typedef std::vector <singleMotorCommand> singleAction;
typedef std::vector <singleAction> totalActionList;

void split(const std::string &s, char delim, std::vector<double> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        std::stringstream ss_substring(item);
        double currentAction;
        ss_substring >> currentAction;
        //elems.push_back(item);
        elems.push_back(currentAction);
    }
    //return elems;
}

class actionExtractor{
public:
    actionExtractor(int motors_num){
        this->motors_num = motors_num;
    };
    ~actionExtractor(){};
    //vector <double> read (string filename){
    totalActionList read (string filename){
        ifstream inputFile(filename);
        //assert(inputFile.good());

        std::string fileLine;
        int lineNum = 1;
        while (std::getline(inputFile, fileLine)) {
            split (fileLine, ' ', this->oneMotorCmd);

            if (lineNum % this->motors_num != 0){
                this->oneCompleteAction.push_back(this->oneMotorCmd);
            }
            else{
                this->oneCompleteAction.push_back(this->oneMotorCmd);
                this->allActions.push_back(this->oneCompleteAction);
                this->oneCompleteAction.clear();
            }

            lineNum ++;

            oneMotorCmd.clear();
        }

        cout << "Number of all the actions = " << this->allActions.size() << endl;

        inputFile.close();
        return this->allActions;
    }

    int getActionsNumber (){
        assert (allActions.size() > 0);
        return allActions.size();
    }

    void save (string filename, vector< vector<double> > actionList){
        ofstream outputFile(filename);
        for (auto &action: actionList){
            for (auto &motor: action){
                for (auto &value: motor){
                    cout << value << " " ;
                }
                cout << endl;
            }
        }
        outputFile.close();
    }

    std::vector<double>* computeOutputs(const int ac_id, int timestep, const totalActionList& actionsList) {
    // This function compute the value of the action at a given timestep.
    const singleAction& action = actionsList.at(ac_id);

    std::vector<double>* outputs = new std::vector<double>(nb_motors, 0.5);

    for(int motor=0; motor < this -> motors_num; motor++)
        outputs->operator[](motor) = action[motor][0] * timestep + action[motor][1];

    return outputs;
}

private:
    int motors_num;
    singleMotorCommand oneMotorCmd;
    singleAction oneCompleteAction;
    totalActionList allActions;
};

}
#endif // ACTIONFACTORY_H
