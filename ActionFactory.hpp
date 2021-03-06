#ifndef ACTIONFACTORY_H
#define ACTIONFACTORY_H

#include "ActionFactory.hpp"
#include "Utils.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <exception>
#include <assert.h>

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

class ActionFactory{
public:
    ActionFactory(int nb_motors){
        this->nb_motors = nb_motors;
    };
    ~ActionFactory(){};
    //vector <double> read (string filename){
    totalActionList read (string filename){
        ifstream inputFile(filename);
        assert(inputFile.good());

        std::string fileLine;
        int lineNum = 1;
        while (std::getline(inputFile, fileLine)) {
            split (fileLine, ' ', this->oneMotorCmd);

            if (lineNum % this->nb_motors != 0){
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

    void save (string filename, totalActionList actionList){
        ofstream outputFile(filename);
        for (auto &action: actionList){
            for (auto &motor: action){
                for (auto &value: motor){
                    outputFile << value << " " ;
                }
                outputFile << endl;
            }
        }
        outputFile.close();
    }

    std::vector<double>* computeOutputs(const int ac_id, const int timestep, const totalActionList& actionsList) {
    // This function compute the value of the action at a given timestep.
    const singleAction& action = actionsList.at(ac_id);

    std::vector<double>* outputs = new std::vector<double>(this->nb_motors, 0.5);

    // cout << "Time step - from computeOutputs = " << timestep << endl;
    // cout << "Calculated outputs = " ;
    for(int motor=0; motor < this->nb_motors; motor++){
        outputs->operator[](motor) = action[motor][0] * (double) timestep + action[motor][1];
        // cout << action[motor][0] << ", " << action[motor][1] << " --> " << outputs->operator[](motor) << endl;
    }
    // cout << endl;


    return outputs;
    }

    void printAllActions()const{
        for (auto &action: this->allActions){
            for (auto &motor: action){
                for (auto &value: motor){
                    cout << value << " " ;
                }
                cout << endl;
            }
        }
    }

private:
    int nb_motors;
    singleMotorCommand oneMotorCmd;
    singleAction oneCompleteAction;
    totalActionList allActions;
};

}
#endif // ACTIONFACTORY_H
