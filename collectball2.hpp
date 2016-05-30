#include <iostream>
#include <stdlib.h>     /* srand, rand */
#include <tuple>
#include <vector>
#include <pair>
#include <list>
#include <ctime> // Needed for the true randomization
#include <cstdlib>
#include "fastsim.hpp"

#include <stdint.h>
#include <SDL/SDL_keyboard.h>
#include <SDL.h>

#include <math.h>       /* pow */

using namespace std;
using namespace fastsim;

class CollectBall{
//TODO: I will have to make the the environment configurations adjustable by the same 'instances' scheme matthieu was using --> more robust.
public:
    typedef std::tuple<int,int> objectPosition;

    CollectBall(const char *mapFileName)
    {
        this->m = boost::shared_ptr<Map>(new Map(mapFileName, 600));
        this->robot = new Robot(20.0f, Posture(200, 200, 0));


        #ifdef VISU
        // Display the screen
        this->d = new Display(this->m, *this->robot);
        #endif
    }
    CollectBall (){
    };

    void setMap (const char *mapFileName){
        this->m = boost::shared_ptr<Map>(new Map(mapFileName, 600));
    };

    void addRobot (double robotDiameter, objectPosition robotInitPos){
        this->robot = new Robot(robotDiameter, Posture(robotInitPos.first, robotInitPos.second, 0));
    };

    void add_balls(objectPosition pos) {
        fastsim::Map::ill_sw_t s1 = fastsim::Map::ill_sw_t(new fastsim::IlluminatedSwitch(BALL, 12, pos.first, pos.second, true));
        this->m->add_illuminated_switch(s1);
    }

    void add_hidden_balls(objectPosition pos) {
        fastsim::Map::ill_sw_t s1 = fastsim::Map::ill_sw_t(new fastsim::IlluminatedSwitch(BALL, 12, pos.first, pos.second, false));
        this->m->add_illuminated_switch(s1);
    }

    void add_basket(int radius, objectPosition pos) {
        float radius = this->robot->get_radius()*1.3;
        Map::ill_sw_t s9 = Map::ill_sw_t(new IlluminatedSwitch(BASKET, radius, pos.first, pos.second, true));
        this->m->add_illuminated_switch(s9);
    }

    void add_switch(int radius, objectPosition pos) {
        float radius = this->robot->get_radius()*0.8;
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

    bool collectball_goal (){

    }

    bool gameOver(){
        // Loop over the number of goals we have
        auto goals = this->m->get_goals();
        auto switches = this->m->get_illuminated_switches();
        auto robotPosition = this->robot->get_pos();
        auto robotPosX = robotPosition.get_x();
        auto robotPosY = robotPosition.get_y();
        auto robotDim = this->robot->get_radius();

        for (size_t i = 0; i < goals.size() ; i++){
            auto goalDim = goals[i].get_diam();
            auto goalX = goals[i].get_x();
            auto goalY = goals[i].get_y();

            auto distance = sqrt(pow(goalX - robotPosX, 2.0) + pow(goalY - robotPosY, 2.0));
            //if ((goals[i].get_y() == robotPosition.get_y()) && (goals[i].get_x() == robotPosition.get_x())){
            if (distance <= (robotDim + goalDim)){
                return true;
            }
        }

        for (size_t i = 0; i < switches.size() ; i++){
            if (switches[i] -> get_on() == true){
                auto switcheDim = switches[i]->get_radius();
                auto switcheX = switches[i]->get_x();
                auto switcheY = switches[i]->get_y();

                auto distance = sqrt(pow(switcheX - robotPosX, 2.0) + pow(switcheY - robotPosY, 2.0));
                //if ((goals[i].get_y() == robotPosition.get_y()) && (goals[i].get_x() == robotPosition.get_x())){
                if (distance <= (robotDim + switcheDim)){
                    switches[i]->set_off();
                    return true;
                }
            }
        }
        return false;
    }

    int number_collected_balls() const {
        return _number_ball_in_basket;
    }

    void printLightSensorsReadings (){
        auto light_sensors = this->robot->get_light_sensors();
        for (size_t i = 0; i < light_sensors.size() ; i++){
            cout << "Light sensor number " << i << " - Activation : " << light_sensors[i].get_activated() << endl;
        }
        cout << "--------------------------------" << endl;
    }


    robotSensorReadings robotStatusWrapping(){
        robotSensorReadings robotStatus(this->robot->get_lasers()[0].get_dist(),
            this->robot->get_lasers()[1].get_dist(),
            this->robot->get_lasers()[2].get_dist(),
            this->robot->get_left_bumper(),
            this->robot->get_left_bumper(),
            this->robot->get_radars()[0].get_inc());
        return robotStatus;
    }

    void step(float move_left, float move_right, bool wanna_carry) {
        /*
        The game logic here:
        - Move the robot to the selected position
        - Collect the reading of all the robot sensors
        - Check the following:
            -- If the robot is near the switch, and it has never pulled the switch before ---> make the hidden balls visible.
            -- If the robot is near a ball AND it is NOT carrying a ball --> carry the ball
            -- If the ball is near a basket AND it is carrying a ball --> put the ball in the basket

        */
        
        this->_lastPostures.push_back(robot().get_pos());

        if(!robot_carrying_ball()) {
            int possible_ball = robot_near_ball();
            if(possible_ball != -1) {
                this->_carrying_ball = possible_ball;
                this->m->get_illuminated_switches()[possible_ball]->set_on(false);
                this->_active_balls.remove(_carrying_ball);
            }
        }
        else if(robot_carrying_ball()) {
            bool avoid = false;
            if(robot_near_basket()) {
                this->_number_ball_in_basket ++;
            } else if (robot_near_ball() == -1 && !robot_near_switch() ) { //if the robot can release anywhere he will carry multiple ball at the same time
                this->m->get_illuminated_switches()[_carrying_ball]->set_on(true);
                _active_balls.push_back(_carrying_ball);
            } else avoid = true;

            if(!avoid) {
                this->m->get_illuminated_switches()[_carrying_ball]->set_pos(robot().get_pos().get_x(), robot().get_pos().get_y());
                this->_carrying_ball = -1;
                arm_movement = true;
            }
        }
        // If the switched is activated, the hidden balls will be visible
        if(this->_never_pull_switch && robot_near_switch()) {
            for(vector<unsigned int>::const_iterator it = this->_hidden_balls.cbegin(); it != this->_hidden_balls.cend() ; ++it) {
                this->m->get_illuminated_switches()[*it]->set_on(true);
                _active_balls.push_back(*it);
            }
            this->_never_pull_switch=false;
        }
    }

    bool robot_carrying_ball() const {
        return _carrying_ball != -1;
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
    int _number_ball_in_basket;
    list <objectPosition> _lastPostures;
    int _carrying_ball;
    int _number_ball_in_basket;
    bool _never_pull_switch;
};