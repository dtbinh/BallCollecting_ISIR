#ifndef COLLECTBALL_H
#define COLLECTBALL_H

//#include <modules/fastsim/simu_fastsim.hpp>
#include "simu.hpp"
#include "defParamGame.hpp"

#include <boost/concept_check.hpp>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <list>
//Added by Omar
#include <stdint.h>
#include <SDL/SDL_keyboard.h>
#include <SDL.h> // Added by Omar in order to handle the Unit8 problem

#ifdef RA
#error RA should not use this file
#endif




struct basket {
    float x;
    float y;
    float radius;
};

using fastsim::Posture;
using std::list;
using boost::scoped_ptr;
using std::auto_ptr;

std::ostream& operator<<(std::ostream& stream, const Posture& p) {
    stream << "{" << p.x() <<", " << p.y() << "}";
    return stream;
}

class CollectBall : public sferes::simu::Fastsim<ParamGame>
{

public:
    CollectBall(const std::string& path):Fastsim(fastsim::Map(path.c_str(), ParamGame::env_size)), _number_stuck(0),
        _carrying_ball(-1), _number_ball_in_basket(0), _lastPostures(), _barycenter_count(0), _last_barycenter(nullptr), _never_pull_switch(true), _arm_delay(0) {

    }

    ~CollectBall() {
//         delete _last_barycenter;
    }

    bool well_init() {
        //all hiden balls are off
        for(list<unsigned int>::const_iterator it = _hidden_balls.cbegin(); it != _hidden_balls.cend() ; ++it)
            if(map()->get_illuminated_switches()[*it]->get_on())
                return false;

        //all active balls are on
        for(list<unsigned int>::const_iterator it = _active_balls.cbegin(); it != _active_balls.cend() ; ++it)
            if(! map()->get_illuminated_switches()[*it]->get_on())
                return false;

        //no hidden ball are in active ball list
        for(list<unsigned int>::const_iterator it = _hidden_balls.cbegin(); it != _hidden_balls.cend() ; ++it)
            if(std::find(_active_balls.begin(), _active_balls.end(), *it) != _active_balls.end())
                return false;

        return  _number_stuck ==0 && _carrying_ball == -1 && _number_ball_in_basket == 0
                && _lastPostures.size() == 0 && _barycenter_count == 0 && _last_barycenter.get() == nullptr && _never_pull_switch;
    }

    void step(float move_left, float move_right, bool wanna_carry) {

#ifdef VISU
        int numkey;
        SDL_PumpEvents();
        keys = SDL_GetKeyState(&numkey);

        if (keys[SDLK_s]) {
            move_left = 0;
            move_right = 0;
        }
        if (keys[SDLK_UP]) {
            move_left=1* ParamGame::motor_power;
            move_right=1* ParamGame::motor_power;
        }
        if (keys[SDLK_DOWN]) {
            move_left=-1* ParamGame::motor_power;
            move_right=-1* ParamGame::motor_power;
        }
        if (keys[SDLK_LEFT]) {
            move_left=1* ParamGame::motor_power;
            move_right=-1* ParamGame::motor_power;
        }
        if (keys[SDLK_RIGHT]) {
            move_left=-1 * ParamGame::motor_power;
            move_right=1* ParamGame::motor_power;
        }
        if (keys[SDLK_SPACE]) {
            wanna_carry = true;
        }
        if (keys[SDLK_RCTRL]) {
            wanna_carry = false;
        }
#endif


        //computations to stop the simulation if it stuck too long
        _lastPostures.push_back(robot().get_pos());
        if(_lastPostures.size() >  ParamGame::stuck_history)
            _lastPostures.pop_front();

        _barycenter_count++;
        if(_barycenter_count > ParamGame::stuck_history) {
            _last_barycenter.reset(create_barycenter(_lastPostures));
            _barycenter_count = 0;
        }

        if(_arm_delay <= 0) {
            bool arm_movement = false;
            if(wanna_carry && !robot_carrying_ball()) {
                int possible_ball = robot_near_ball();
                if(possible_ball != -1) {
                    _carrying_ball = possible_ball;
                    map()->get_illuminated_switches()[possible_ball]->set_on(false);
                    _active_balls.remove(_carrying_ball);
                    arm_movement = true;
                }
            }
            else if(robot_carrying_ball() && !wanna_carry) {
                bool avoid = false;
                if(robot_near_basket()) {
                    _number_ball_in_basket ++;
                } else if (robot_near_ball() == -1 && !robot_near_switch() ) { //if the robot can release anywhere he will carry multiple ball at the same time
                    map()->get_illuminated_switches()[_carrying_ball]->set_on(true);
                    _active_balls.push_back(_carrying_ball);
                } else avoid = true;

                if(!avoid) {
                    map()->get_illuminated_switches()[_carrying_ball]->set_pos(robot().get_pos().get_x(), robot().get_pos().get_y());
                    _carrying_ball = -1;
                    arm_movement = true;
                }
            }

            if(arm_movement)
                _arm_delay = 4;
        }

        //deals with switch
        if(_never_pull_switch && robot_near_switch()) {
            for(list<unsigned int>::const_iterator it = _hidden_balls.cbegin(); it != _hidden_balls.cend() ; ++it) {
                map()->get_illuminated_switches()[*it]->set_on(true);
                _active_balls.push_back(*it);
            }
            _never_pull_switch=false;
        }



        if(_arm_delay <= 0)
            move_robot(move_left, move_right);

        _arm_delay--;

#ifdef VISU
        if (_carrying_ball != -1)
            robot().set_color(5);
        else
            robot().set_color(BALL);
#endif
    }

    void refresh_robot(){
	move_robot(0.,0.);
    }

    bool end() {
        return !_never_pull_switch && _carrying_ball == -1 && _active_balls.size() == 0;
    }

    int carried_ball(){
        return _carrying_ball;
    }

    bool never_pull_switch() {
        return _never_pull_switch;
    }

    void refresh_view() {
        if (!keys[SDLK_LCTRL])
            usleep(50*1000);
        Fastsim::refresh_view();
    }

#ifdef STORETRACE
    //be careful it is not just a getter
    bool ball_collected() {
        bool retr = _number_ball_in_basket != _last_number_ball_in_basket;
        _last_number_ball_in_basket = _number_ball_in_basket;
        return retr;
    }
#endif

    friend std::ostream& operator<< (std::ostream&, const Posture&);
    bool stuck_too_long() const  {
        if(_last_barycenter.get() != nullptr && _barycenter_count == ParamGame::stuck_history) {
            scoped_ptr<Posture> bary(create_barycenter(_lastPostures));
            float dist = _last_barycenter->dist_to(*bary);

            //std::cout << dist << " " << **_last_barycenter  << " " << *bary << std::endl;
            return dist <= ParamGame::dist_stuck;
        }
        return false;
    }

    bool robot_carrying_ball() const {
        return _carrying_ball != -1;
    }

    void add_basket(const basket& b) {
        using namespace fastsim;
        _basket = b;

        Map::ill_sw_t s9 = Map::ill_sw_t(new IlluminatedSwitch(BASKET,b.radius, b.x, b.y, true));
        map()->add_illuminated_switch(s9);
    }

    void add_switch(const basket& b) {
        using namespace fastsim;
        _switch = b;

        Map::ill_sw_t s9 = Map::ill_sw_t(new IlluminatedSwitch(SWITCH,b.radius, b.x, b.y, true));
        map()->add_illuminated_switch(s9);
    }

    int number_collected_balls() const {
        return _number_ball_in_basket;
    }

    void consolidate() {
        for (size_t k = 0; k < map()->get_illuminated_switches().size(); k++)
            if (map()->get_illuminated_switches()[k]->get_color() == BALL) {
                if(map()->get_illuminated_switches()[k]->get_on())
                    _active_balls.push_back(k);
                else
                    _hidden_balls.push_back(k);
            }
    }


    bool robot_near_basket() const {
        float rx = robot().get_pos().x();
        float ry = robot().get_pos().y();

        float xx = rx-_basket.x;
        float yy = ry-_basket.y;
        float dist = sqrtf(xx * xx + yy * yy);
        return dist<=_basket.radius;
    }

    float robot_distance_basket() const {
	float rx = robot().get_pos().x();
        float ry = robot().get_pos().y();

        float xx = rx-_basket.x;
        float yy = ry-_basket.y;
        float dist = sqrtf(xx * xx + yy * yy);

	return dist;
    }
//private : //This is the orignial line by M -- commented by omar

    int robot_near_ball() const {
        float rx = robot().get_pos().x();
        float ry = robot().get_pos().y();

        for(list<unsigned int>::const_iterator it = _active_balls.cbegin(); it != _active_balls.cend() ; ++it)
        {
            unsigned int k = *it;
            float xx = rx-map()->get_illuminated_switches()[k]->get_x();
            float yy = ry-map()->get_illuminated_switches()[k]->get_y();
            float dist = sqrtf(xx * xx + yy * yy);
            if(dist < robot().get_radius())
                return k;
        }

        return -1;
    }

    bool robot_near_switch() const {
        float rx = robot().get_pos().x();
        float ry = robot().get_pos().y();

        float xx = rx-_switch.x;
        float yy = ry-_switch.y;
        float dist = sqrtf(xx * xx + yy * yy);
        return dist<=_switch.radius;
    }


    Posture* create_barycenter(const list<Posture>& _lastPostures) const {
        float x = 0.f;
        float y = 0.f;
        for(list<Posture>::const_iterator it=_lastPostures.cbegin(); it != _lastPostures.cend() ; ++it) {
            x += it->get_x();
            y += it->get_y();
        }

        return new Posture(x/_lastPostures.size(),y/_lastPostures.size(),0);
    }

    int _number_stuck;
    int _carrying_ball;
    int _number_ball_in_basket;
    basket _basket, _switch;
    list<Posture> _lastPostures;
    int _barycenter_count;
    auto_ptr<Posture> _last_barycenter;
    list<unsigned int> _active_balls;
    list<unsigned int> _hidden_balls;
    bool _never_pull_switch;
    int _arm_delay;

    Uint8* keys;

#ifdef STORETRACE
    int _last_number_ball_in_basket = 0;
#endif
};

#endif // COLLECTBALL_H
