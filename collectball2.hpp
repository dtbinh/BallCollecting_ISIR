#ifndef VISU
#define VISU
#endif

#include <iostream>
#include <stdlib.h>     /* srand, rand */
#include <tuple>
#include <vector>
#include <list>
#include <ctime> // Needed for the true randomization
#include <cstdlib>

#include "libfastsim/laser.hpp"
#include "libfastsim/map.hpp"
#include "libfastsim/misc.hpp"
#include "libfastsim/posture.hpp"
#include "libfastsim/robot.hpp"
#include "libfastsim/display.hpp"
#include "libfastsim/goal.hpp"
#include "libfastsim/radar.hpp"

#include <stdint.h>
#include <SDL/SDL_keyboard.h>
#include <SDL.h>

#include <math.h>       /* pow */
#include "Utils.hpp"
#include <boost/shared_ptr.hpp>


using namespace std;
using namespace fastsim;
using std::pair;

//switch color
#define SWITCH 4

// ball color
#define BALL 3

//basket color
#define BASKET 2

struct basket {
    float x;
    float y;
    float radius;
};

class CollectBall{
//TODO: I will have to make the the environment configurations adjustable by the same 'instances' scheme matthieu was using --> more robust.
public:
    static constexpr float env_size = 500.;
    static constexpr float motor_power = 5.;
    static constexpr int stuck_history = 100;
    static constexpr float dist_stuck = 10.f;
    static constexpr int step_limit = 10000;
    static constexpr unsigned int max_nb_ball = 4;

    typedef std::pair<int,int> objectPosition;

    CollectBall (){
    };

    void setMap (const char *mapFileName){
        this->m = boost::shared_ptr<Map>(new Map(mapFileName, this->env_size));
    };

    void add_robot_position (int id_point) {
        const pair<float, float>& pos = this->get_predefined_points(id_point);  
        cout << "robot pos : " << pos.first << " , " << pos.second << endl;
        //this->robot->set_pos(fastsim::Posture(pos.first, pos.second, 0.1));
        this->robot = new Robot(20.0f, Posture(pos.first, pos.second, 0.1));
    };

    void add_balls(int id_point) {
        const pair<float, float>& pos = this->get_predefined_points(id_point);  
        fastsim::Map::ill_sw_t s1 = fastsim::Map::ill_sw_t(new fastsim::IlluminatedSwitch(BALL, 12, pos.first, pos.second, true));
        this->m->add_illuminated_switch(s1);
    }

    void add_hidden_balls(int id_point) {
        const pair<float, float>& pos = this->get_predefined_points(id_point);  
        fastsim::Map::ill_sw_t s1 = fastsim::Map::ill_sw_t(new fastsim::IlluminatedSwitch(BALL, 12, pos.first, pos.second, false));
        this->m->add_illuminated_switch(s1);
    }

    void add_basket(int id_point) {
        const pair<float, float>& pos = this->get_predefined_points(id_point);  
        float radius = this->robot->get_radius()*1.3;

        this->_basket.x = pos.first;
        this->_basket.y = pos.second;
        this->_basket.radius = radius;

        Map::ill_sw_t s9 = Map::ill_sw_t(new IlluminatedSwitch(BASKET, radius, pos.first, pos.second, true));
        this->m->add_illuminated_switch(s9);
    }

    void add_switch(int id_point) {
        const pair<float, float>& pos = this->get_predefined_points(id_point);  
        float radius = this->robot->get_radius()*0.8;

        this->_switch.x = pos.first;
        this->_switch.y = pos.second;
        this->_switch.radius = radius;
        Map::ill_sw_t s9 = Map::ill_sw_t(new IlluminatedSwitch(SWITCH, radius, pos.first, pos.second, true));
        this->m->add_illuminated_switch(s9);
    }

    void getCurrentBalls() {
        for (size_t k = 0; k < this->m->get_illuminated_switches().size(); k++)
            if (this->m->get_illuminated_switches()[k]->get_color() == BALL) {
                if(this->m->get_illuminated_switches()[k]->get_on())
                    this->_active_balls.push_back(k);
                else
                    this->_hidden_balls.push_back(k);
            }
    }

    void define_robot_sensors() {
        using namespace fastsim;
        // Sensors:
        // * 3 lasers range sensors

        //right
        this->robot->add_laser(Laser(M_PI / 4.0, 8.f*this->robot->get_radius()*2.f));

        // left
        this->robot->add_laser(Laser(-M_PI / 4.0, 8.f*this->robot->get_radius()*2.f));

        //middle
        this->robot->add_laser(Laser(0.0f, 8.f*this->robot->get_radius()*2.f));

        // * 2 ball sensors (ball=light of color 0)

        //right
        this->robot->add_light_sensor(LightSensor(BALL,  M_PI / 4.0, M_PI / 2.0 + 0.01));

        //left
        this->robot->add_light_sensor(LightSensor(BALL, -M_PI / 4.0, M_PI / 2.0 + 0.01));
        // * 2 basket sensors (basket surrounded by lights of color 1)
        //right
        this->robot->add_light_sensor(LightSensor(BASKET,  M_PI / 4.0, M_PI / 2.0 + 0.01));
        // left
        this->robot->add_light_sensor(LightSensor(BASKET, -M_PI / 4.0, M_PI / 2.0 + 0.01));

        // * 2 switch sensors
        //right
        this->robot->add_light_sensor(LightSensor(SWITCH,  M_PI / 4.0, M_PI / 2.0 + 0.01));

        //left
        this->robot->add_light_sensor(LightSensor(SWITCH, -M_PI / 4.0, M_PI / 2.0 + 0.01));

        // * 2 bumpers
        // * 1 carrying object sensor
    }

    bool ball_collected (){

        bool retr = _number_ball_in_basket != _last_number_ball_in_basket;
        _last_number_ball_in_basket = _number_ball_in_basket;
        return retr;
    }

    double computeReward(int rewardChoice){
        /*
            In order to handle the case of different 'hardcoded' reward functions, I will make a switch case between different
            reward functions
            0 : Collect ball and then put it in the basket --> the traditional reward.
            1 : Collect the ball
            2 : See the ball
            3 : See the basket
            4 : See a switch
            5 : All the tasks of the ball collecting task has been done --> I don't think this is useful as a reward function (but good for an end condition)
        */
        bool bdrop = false;
        switch (rewardChoice){
            case 0:
                bdrop = this->ball_collected();
                break;

            case 1:
                bdrop = this->robot_carrying_ball();
                break;

            case 2:{
                if (this->robot_near_ball() != -1)
                    bdrop = true;

                break;
            }
            case 3:
                bdrop = this->robot_near_basket();
                break;

            case 4:
                bdrop = this->robot_near_switch();
                break;

            case 5:
                bdrop = (!_never_pull_switch && _carrying_ball == -1 && _active_balls.size() == 0);
                break;

            default:{
                bdrop = false;
                break;
            }
        }

        if (bdrop == true)
            return 100.0;
        else
            return 0.0;

    }

    bool end(int FinalGoal){
        //return !_never_pull_switch && _carrying_ball == -1 && _active_balls.size() == 0; // This is the normal end for collectball task
        bool bdrop;
        switch (FinalGoal){
            case 0:
                bdrop = this->ball_collected();
                break;

            case 1:
                bdrop = this->robot_carrying_ball();
                break;

            case 2:{
                if (this->robot_near_ball() != -1)
                    bdrop = true;
                break;
            }
            case 3:
                bdrop = this->robot_near_basket();
                break;

            case 4:
                bdrop = this->robot_near_switch();
                break;

            case 5:
                bdrop = (!_never_pull_switch && _carrying_ball == -1 && _active_balls.size() == 0);
                break;

            default:{
                bdrop = false;
                break;
            }
        }

        return bdrop;
    }
    int simu_perf() {

        return this->number_collected_balls();
    }

    int number_collected_balls() const {
        return _number_ball_in_basket;
    }

    void step_simu(const std::vector<double>& outputs) {
        // if ((outputs[0] != 0.0) && (outputs[1] != 0.0))
        // this->step(sml::Utils::transform(outputs[0], 0.f, 1.f, -this->motor_power, this->motor_power/10.0),
        //                         sml::Utils::transform(outputs[1], 0.f, 1.f, -this->motor_power, this->motor_power/10.0));
        this->step (outputs[0], outputs[1]);

    }

    void step(double move_left, double move_right) {
        /*
        The game logic here:
        - Move the robot to the selected position
        - Collect the reading of all the robot sensors
        - Check the following:
            -- If the robot is near the switch, and it has never pulled the switch before ---> make the hidden balls visible.
            -- If the robot is near a ball AND it is NOT carrying a ball --> carry the ball
            -- If the ball is near a basket AND it is carrying a ball --> put the ball in the basket

        */
        // cout << "Inside step function, part 1 " << endl;

        cout << "robot_carrying_ball() = " << robot_carrying_ball() << endl;
        cout << "robot_near_basket() = " << robot_near_basket() << endl;
        cout << "robot_near_ball() = " << robot_near_ball() << endl;
        cout << "robot_near_switch() = " << robot_near_switch() << endl;
        cout << "_number_ball_in_basket = " << this->_number_ball_in_basket << endl;
        cout << "this->_never_pull_switch = " << this->_never_pull_switch << endl;
        
        if(!this->robot_carrying_ball()) {
            // cout << "Condition 1" << endl;
            int possible_ball = robot_near_ball();
            if(possible_ball != -1) {
                this->_carrying_ball = possible_ball;
                this->m->get_illuminated_switches()[possible_ball]->set_on(false);
                this->_active_balls.remove(_carrying_ball);
            }
        }
        else if(this->robot_carrying_ball()) {
            if(robot_near_basket()) {
                this->_number_ball_in_basket ++; //It will just put the ball in the basket

                this->_carrying_ball = -1;
            }
        }
        // If the switched is activated, the hidden balls will be visible
        // cout << "Inside step function, part 2 " << endl;
        if(this->_never_pull_switch && robot_near_switch()) {
            for(list<unsigned int>::const_iterator it = this->_hidden_balls.begin(); it != this->_hidden_balls.end() ; ++it) {
                this->m->get_illuminated_switches()[*it]->set_on(true);
                _active_balls.push_back(*it);
            }
            this->_never_pull_switch=false;
        }
        this->robot->move(move_left, move_right, this->m);
        this->m->update(this->robot->get_pos());
        #ifdef VISU
        this->d->update();
        #endif
    }

    bool robot_carrying_ball() const {

        return _carrying_ball != -1;
    }

    int robot_near_ball() const {
        float rx = this->robot->get_pos().x();
        float ry = this->robot->get_pos().y();

        for(list<unsigned int>::const_iterator it = _active_balls.begin(); it != _active_balls.end() ; ++it)
        {
            unsigned int k = *it;
            float xx = rx-this->m->get_illuminated_switches()[k]->get_x();
            float yy = ry-this->m->get_illuminated_switches()[k]->get_y();
            float dist = sqrtf(xx * xx + yy * yy);
            if(dist < this->robot->get_radius())
                return k;
        }

        return -1;
    }

    bool robot_near_switch() const {
        float rx = this->robot->get_pos().x();
        float ry = this->robot->get_pos().y();

        float xx = rx-_switch.x;
        float yy = ry-_switch.y;
        float dist = sqrtf(xx * xx + yy * yy);
        return dist<=_switch.radius;
    }

    bool robot_near_basket() const {
        float rx = this->robot->get_pos().x();
        float ry = this->robot->get_pos().y();

        float xx = rx-_basket.x;
        float yy = ry-_basket.y;
        float dist = sqrtf(xx * xx + yy * yy);
        return dist<=_basket.radius;
    }

    float robot_distance_basket() const {
    float rx = this->robot->get_pos().x();
        float ry = this->robot->get_pos().y();

        float xx = rx-_basket.x;
        float yy = ry-_basket.y;
        float dist = sqrtf(xx * xx + yy * yy);

    return dist;
    }

    void computeIniInputs(std::vector<double>* inputs, const std::vector<double>& ac) {
        /*
            This function will build the vector of sensory input. At the end of it, the selected action will be added.
            So, the resulting vector, 'inputs', will be (sensory inputs + action)
        */
        // Update of the sensors
        size_t nb_lasers = this->robot->get_lasers().size();
        size_t nb_light_sensors = this->robot->get_light_sensors().size();

        int index = 0;
        // *** set inputs ***
        for (size_t j = 0; j < nb_lasers; j++)
        {
            double d = this->robot->get_lasers()[j].get_dist();
            double range = this->robot->get_lasers()[j].get_range();
            if(d == -1.f || d >= range)
                inputs->operator[](index++) = 0.f;
            else
                inputs->operator[](index++) = (d == -1.f || d >= range ? 0 : 1 - d / range);
        }

        for (size_t j = 0; j < nb_light_sensors; j+=2)
        {
            double actL = (double) this->robot->get_light_sensors()[j].get_activated();
            double actR = (double) this->robot->get_light_sensors()[j+1].get_activated();
            inputs->operator[](index+j) = actL;
            inputs->operator[](index+j+1) = actR;
        }

        index += nb_light_sensors;

        inputs->operator[](index+0) = (double) this->robot->get_left_bumper();
        inputs->operator[](index+1) = (double) this->robot->get_right_bumper();
        inputs->operator[](index+2) = (double) this->robot_carrying_ball();

        index = index + 3;

        inputs->operator[](index) = (double) !this->never_pull_switch();
        index++;

        for(int i=index; i< index + ac.size(); i++)
            inputs->operator[](i) = ac.at( i - index );
    }

    bool never_pull_switch(){
        return _never_pull_switch;
    }

    void simuInitInside(int instance_num, const char *mapFileName) {
        /*
        This function will build the collectball experiment (adding the robots, the switch, the hidden and the active balls,..etc)
        - Build instances
        - Build predefined positions
        - For the selected instance number, add the sensors/basket/switch/robot in the correct places.
        - Update your knowledge about the number of active/hidden balls, and their ID
        */
        this->_last_number_ball_in_basket = 0;
        this->_number_ball_in_basket = 0;
        this->_carrying_ball = -1;
        this->_never_pull_switch = true;
        this->ball_number = 0;
        this->build_instances();
        this->build_predifined_positions();
        cout << "simuInitInside - variables initialized " << endl;
        this->build_instances();
        this->build_predifined_positions();
        cout << "simuInitInside - instances initialized " << endl;
        this->setMap(mapFileName);
        cout << "simuInitInside - map initialized " << endl;
        //for(auto item: this->instances[instance_num]) {
        this->add_robot_position(this->instances[instance_num].find('I')->second);
        for(std::multimap<char, int>::const_iterator it = this->instances[instance_num].begin(); it != this->instances[instance_num].end(); it ++) {
            cout << "item.first " << it->first << endl;
            cout << "item.second " << it->second << endl;
            switch (it->first) {
                case 'B':
                    this->add_balls(it->second);
                    this->ball_number++;
                    break;
                case 'G':
                    this->add_basket(it->second);
                    break;
                case 'O':
                    this->add_hidden_balls(it->second);
                    this->ball_number++;
                    break;
                case 'S':
                    this->add_switch(it->second);
                    break;
                default:
                    std::cout << "ignore " << it->first << " while parsing map position" << std::endl;
                }
        }
        cout << "this->ball_number = " << this->ball_number << endl;
        cout << "this->max_nb_ball = " << this->max_nb_ball << endl;

        cout << "simuInitInside - instance built " << endl;

        assert(this->ball_number == this->max_nb_ball);
        this->getCurrentBalls();

        #ifdef VISU
        // Display the screen
        this->d = new Display(this->m, *this->robot);
        #endif
    }

    void build_instances(){
        /*
        This is the list of possible instances for the game. It will ease the construction of the game itself.
        */
        // I - Initial position
        // G - (goal) Basket position
        // B - Ball position
        // S - Switch
        // O - offline ball (need switch)

        // std::map<int, std::multimap<char, int>> my_instances = {
        //     //instance 0
        //     {   0, {
        //             {'I', 0}, {'G', 12}, {'B', 4}, {'B', 7}, {'S', 14}, {'O', 1}, {'O', 10}
        //         }
        //     },
        //     //instance 1
        //     {   1, {
        //             {'I', 15}, {'G', 12}, {'B', 4}, {'B', 7}, {'S', 14}, {'O', 1}, {'O', 10}
        //         }
        //     },
        //     //instance 2
        //     {   2, {
        //             {'I', 6}, {'G', 10}, {'B', 3}, {'B', 13}, {'S', 1}, {'O', 2}, {'O', 7}
        //         }
        //     }
        // };
        this->instances[0] = {{'I', 0}, {'G', 12}, {'B', 4}, {'B', 7}, {'S', 14}, {'O', 1}, {'O', 10}};
        this->instances[1] = {{'I', 15}, {'G', 12}, {'B', 4}, {'B', 7}, {'S', 14}, {'O', 1}, {'O', 10}};
        this->instances[2] = {{'I', 6}, {'G', 10}, {'B', 3}, {'B', 13}, {'S', 1}, {'O', 2}, {'O', 7}};
        // this->instances = my_instances;
    }

    const pair<float, float>& get_predefined_points(unsigned int i) const {
        return this->predefined_positions[i];
    }

    void build_predifined_positions() {
        vector<float> rank = {0.075, 0.425, 0.575, 0.925};

        for(unsigned int i=0; i< 4; i++)
            for(unsigned int j=0; j< 4; j++) {
                int x = rank[i]*this->env_size;
                int y = rank[j]*this->env_size;

                this->predefined_positions.push_back(pair<float, float>(x, y));
            }
    }

private:
    boost::shared_ptr<Map> m;
    //Robot robot;
    Robot *robot;
    #ifdef VISU
    Display *d;
    #endif
    typedef tuple<float, float> robotMovementInstructions;
    typedef tuple<float, float, float, bool, bool, float> robotSensorReadings;
    typedef tuple<float, float> robotMotorInstruction;
    Uint8* keys;
    list<unsigned int> _active_balls;
    list<unsigned int> _hidden_balls;
    int _carrying_ball;
    int _number_ball_in_basket;
    bool _never_pull_switch;
    std::map<int, std::multimap<char, int>> instances;
    int ball_number;
    int _last_number_ball_in_basket;
    vector< pair<float, float> > predefined_positions;
    basket _basket, _switch;
};