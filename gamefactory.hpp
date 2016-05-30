#ifndef GAMEFACTORY_H
#define GAMEFACTORY_H

//#include <sferes/misc.hpp>
//#include "sferes/misc.hpp"
#include "simu.hpp"
#include "collectball.hpp"
#include "defParamGame.hpp"
//#include "bib/Singleton.hpp"
#include <map>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#ifdef RA
#error RA should not use this file
#endif

using std::string;
using std::multimap;

class GameFactory// : public bib::Singleton<GameFactory>
{
    //friend class bib::Singleton<GameFactory>;

public:
    typedef boost::shared_ptr<CollectBall> simu_t;
private:
    typedef boost::shared_ptr<CollectBall> _simu_t;
    typedef std::map<std::string, _simu_t > map_simu_t;
    typedef std::pair < std::string , unsigned int > key_pos_instance;
    typedef std::map< key_pos_instance , _simu_t > inst_map_simu_t;

public:
    GameFactory() {
    }
    CollectBall* create(const string& key, unsigned int instance) {
        inst_map_simu_t::iterator it = instances_positions.find(key_pos_instance(key, instance));
        CollectBall* retrn;
        if(it == instances_positions.end()) {
            _simu_t base_instance = _simu_t(new CollectBall(key));

            define_robot_sensors(base_instance);

            const multimap<char,int>& positions = this->gameParameters.getPositionOfInstance(instance);

            int ball_number = 0;
            for(multimap<char,int>::const_iterator it = positions.cbegin(); it != positions.cend() ; ++it) {
                switch (it->first) {
                case 'I':
                    add_robot_position(base_instance, it->second);
                    break;
                case 'B':
                    add_balls(base_instance, it->second);
                    ball_number++;
                    break;
                case 'G':
                    add_basket(base_instance, it->second);
                    break;
                case 'O':
                    add_hidden_balls(base_instance, it->second);
                    ball_number++;
                    break;
                case 'S':
                    add_switch(base_instance, it->second);
                    break;
                default:
                    std::cout << "ignore " << it->first << " while parsing map position" << std::endl;
                }
            }

            assert(ball_number == ParamGame::max_nb_ball);
            base_instance->consolidate();

            boost::unique_lock< boost::shared_mutex > w_lock(_write_mutex);
            instances_positions.insert(std::pair<key_pos_instance, _simu_t>(key_pos_instance(key, instance), base_instance) );

            retrn = new CollectBall(*base_instance);
        }
        else
            retrn= new CollectBall(*it->second);
        retrn->deep_copy();

        return retrn;
    }

private:
    void define_robot_sensors(_simu_t base_instance) {
        using namespace fastsim;
        // Sensors:
        // * 3 lasers range sensors

        //right
        base_instance->robot().add_laser(Laser(M_PI / 4.0, 8.f*base_instance->robot().get_radius()*2.f));

        // left
        base_instance->robot().add_laser(Laser(-M_PI / 4.0, 8.f*base_instance->robot().get_radius()*2.f));

        //middle
        base_instance->robot().add_laser(Laser(0.0f, 8.f*base_instance->robot().get_radius()*2.f));

        // * 2 ball sensors (ball=light of color 0)

        //right
        base_instance->robot().add_light_sensor(LightSensor(BALL,  M_PI / 4.0, M_PI / 2.0 + 0.01));

        //left
        base_instance->robot().add_light_sensor(LightSensor(BALL, -M_PI / 4.0, M_PI / 2.0 + 0.01));
        // * 2 basket sensors (basket surrounded by lights of color 1)
        //right
        base_instance->robot().add_light_sensor(LightSensor(BASKET,  M_PI / 4.0, M_PI / 2.0 + 0.01));
        // left
        base_instance->robot().add_light_sensor(LightSensor(BASKET, -M_PI / 4.0, M_PI / 2.0 + 0.01));

        // * 2 switch sensors
        //right
        base_instance->robot().add_light_sensor(LightSensor(SWITCH,  M_PI / 4.0, M_PI / 2.0 + 0.01));

        //left
        base_instance->robot().add_light_sensor(LightSensor(SWITCH, -M_PI / 4.0, M_PI / 2.0 + 0.01));

        // * 2 bumpers
        // * 1 carrying object sensor
    }

    void add_robot_position(_simu_t base_instance, int id_point) {
        const pair<float, float>& pos = this->gameParameters.get_predefined_points(id_point);
        base_instance->robot().set_pos(fastsim::Posture(pos.first, pos.second, 0.1));
    }

    void add_balls(_simu_t base_instance, int id_point) {
        const pair<float, float>& pos = this->gameParameters.get_predefined_points(id_point);
        fastsim::Map::ill_sw_t s1 = fastsim::Map::ill_sw_t(new fastsim::IlluminatedSwitch(BALL, 12, pos.first, pos.second, true));
        base_instance->map()->add_illuminated_switch(s1);
    }

    void add_hidden_balls(_simu_t base_instance, int id_point) {
        const pair<float, float>& pos = this->gameParameters.get_predefined_points(id_point);
        fastsim::Map::ill_sw_t s1 = fastsim::Map::ill_sw_t(new fastsim::IlluminatedSwitch(BALL, 12, pos.first, pos.second, false));
        base_instance->map()->add_illuminated_switch(s1);
    }

    void add_basket(_simu_t base_instance, int id_point) {
        const pair<float, float>& pos = this->gameParameters.get_predefined_points(id_point);
        float radius = base_instance->robot().get_radius()*1.3;

        base_instance->add_basket( {pos.first, pos.second, radius});
    }

    void add_switch(_simu_t base_instance, int id_point) {
        const pair<float, float>& pos = this->gameParameters.get_predefined_points(id_point);
        float radius = base_instance->robot().get_radius()*0.8;

        base_instance->add_switch( {pos.first, pos.second, radius});
    }

    inst_map_simu_t instances_positions;
    map_simu_t instances;

    boost::shared_mutex _write_mutex;

    ParamGame gameParameters;
};

#endif // GAMEFACTORY_H
