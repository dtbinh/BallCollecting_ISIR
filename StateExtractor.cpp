#include "StateExtractor.hpp"
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <exception>
#include <algorithm>
#include "Logger.hpp"

using namespace std;

class malformedfile : public std::exception {
    virtual const char* what() const throw()
    {
        return "The state file is malformed !";
    }
} malformedfile_ex;

StateExtractor::StateExtractor(int _sensor_size, int _motor_size): sensor_size(_sensor_size), motor_size(_motor_size) {

}

void StateExtractor::initParser(const std::string& file) {
    using std::getline;

    if ( !boost::filesystem::exists( file ) )
    {
        std::cout << "Can't find the state file! (" << file << ")" << std::endl;
        exit(1);
    }
    valid_indexes.clear();

    std::ifstream myfile(file);

    std::string line;

    while(getline(myfile, line))
            valid_indexes.push_back(stoi(line));

    myfile.close();

}

void StateExtractor::write(const std::string& file) const{
   std::ofstream myfile(file , std::ofstream::out);

   for(auto ind : valid_indexes){
	myfile << ind << std::endl;
   }

   myfile.close();
}

void StateExtractor::initParser(int numberRandMax, int possibility){
    assert(numberRandMax > possibility);

    valid_indexes.clear();

    std::vector<int> randomized(numberRandMax, 0);
    for(int i=0;i<numberRandMax; i++)
      randomized[i] = i;

    std::random_shuffle(randomized.begin(), randomized.end());

    for(int i=0;i < possibility;i++)
      valid_indexes.push_back(randomized[i]);

}

int StateExtractor::getNumberInput() const {
    return valid_indexes.size();
}

int StateExtractor::maxIndex() const{
//     LOG_DEBUG(valid_indexes.size() << " " << valid_indexes[valid_indexes.size()-1]);
    return *std::max_element(valid_indexes.cbegin(), valid_indexes.cend());
}

std::vector<double>* StateExtractor::parse(const std::vector<double>& input) const {
    //std::cout << "getNumberInput = " << getNumberInput() << std::endl;

    std::vector<double>* output = new std::vector<double>(getNumberInput());

    // cout << "Parsing inputs -- input size : " << input.size() << " , number of inputs : " << this->getNumberInput() << endl;

    int index_output=0;
    //std::cout << "inputs = ";
    for(int i = 0; i < getNumberInput(); i++) {
      output->operator[](index_output) = input.at(valid_indexes[i]);
      index_output++;
      //std::cout << input.at(valid_indexes[i]) << " ";
    }
    //std::cout << std::endl;

    return output;
}
