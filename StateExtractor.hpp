#ifndef STATEEXTRACTOR_HPP
#define STATEEXTRACTOR_HPP

#include <string>
#include <vector>

class StateExtractor{
  
public:
  StateExtractor(int sensor_size, int motor_size);
  void initParser(const std::string&);
  void initParser(int numberRandMax, int possibility);
  int getNumberInput() const;
  int maxIndex() const;
  void write(const std::string&) const;
  
  std::vector<double>* parse(const std::vector<double>& input) const;

private:
  int number_action;
  int sensor_size;
  int motor_size;
  std::vector<int> valid_indexes;
//   std::vector<int> recognized_actions;
};

#endif
