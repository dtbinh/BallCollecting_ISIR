#ifndef DEFPARAMGAME_H
#define DEFPARAMGAME_H

#include <string>
#include <assert.h>
#include <boost/concept_check.hpp>
#include <vector>
#include <map>
#include "Singleton.hpp"

#ifdef RA
#error RA should not use this file
#endif

//switch color
#define SWITCH 4

// ball color
#define BALL 3

//basket color
#define BASKET 2

using std::string;
using std::pair;
using std::vector;
using std::map;
using std::multimap;

typedef struct _arena_instance_id {
    unsigned int map_id;
    unsigned int position_id;

    bool operator<=(const struct _arena_instance_id& other) const {
        return map_id <= other.map_id && position_id <= other.position_id;
    }

} arena_instance_id;

std::ostream& operator<<(std::ostream& stream, const arena_instance_id& inst) {
    stream << "{" << inst.map_id <<", " << inst.position_id << "}";
    return stream;
}

class ParamGame : public bib::Singleton<ParamGame>
{
    friend class bib::Singleton<ParamGame>;

public:
    static constexpr float env_size = 500.;
    static constexpr float motor_power = 5.;
    static constexpr int stuck_history = 100;
    static constexpr float dist_stuck = 10.f;
    static constexpr int step_limit = 10000;
    static constexpr unsigned int max_nb_ball = 4;

    // I - Initial position
    // G - (goal) Basket position
    // B - Ball position
    // S - Switch
    // O - offline ball (need switch)
    std::map<int, std::multimap<char, int>> instances = {
        //instance 0
        {   0, {
                {'I', 0}, {'G', 12}, {'B', 4}, {'B', 7}, {'S', 14}, {'O', 1}, {'O', 10}
            }
        },
	//instance 1
        {   1, {
                {'I', 15}, {'G', 12}, {'B', 4}, {'B', 7}, {'S', 14}, {'O', 1}, {'O', 10}
            }
        },
        //instance 2
        {   2, {
                {'I', 6}, {'G', 10}, {'B', 3}, {'B', 13}, {'S', 1}, {'O', 2}, {'O', 7}
            }
        }
    };

    ParamGame() {
        build_predifined_positions();
    }

    const string& getArenaKey(int instance) const {
        return _arena_keys[instance];
    }

    const std::multimap<char, int>& getPositionOfInstance(int n_instance) const {
        return instances.find(n_instance)->second;
    }

    const unsigned int& getNumberInstance()  const {
        return _number_instance;
    }

    const _arena_instance_id& getEndInstance()  const {
        return _end_id;
    }

    unsigned int getPositionInstance(unsigned int i)  const {
        return _position_instances[i];
    }

    void injectArgs(const vector<string>& keys, const vector<unsigned int>& vals) {
        _arena_keys = keys;
        _position_instances = vals;

        _end_id = {(unsigned int)_arena_keys.size() - 1, (unsigned int)_position_instances.size() - 1};
	_number_instance = _arena_keys.size() * _position_instances.size();
    }

    const pair<float, float>& get_predefined_points(unsigned int i) const {
        return predefined_positions[i];
    }

private:
    void build_predifined_positions() {
        vector<float> rank = {0.075, 0.425, 0.575, 0.925};

        for(unsigned int i=0; i< 4; i++)
            for(unsigned int j=0; j< 4; j++) {
                int x = rank[i]*ParamGame::env_size;
                int y = rank[j]*ParamGame::env_size;

                predefined_positions.push_back(pair<float, float>(x, y));
            }
    }


    //predifined points on the map
    vector< pair<float, float> > predefined_positions;

    arena_instance_id _end_id;
    unsigned int _number_instance;

    //instance to evaluate
    vector<unsigned int> _position_instances;

    //map to evaluate
    vector<string> _arena_keys;
};

void operator++(arena_instance_id& inst, int) {
    if(inst.position_id == ParamGame::getInstance()->getEndInstance().position_id) {
        inst.map_id ++;
        inst.position_id = 0;
    }
    else
        inst.position_id++;
}

unsigned int operator*(const arena_instance_id& inst) {
    return inst.map_id * (ParamGame::getInstance()->getEndInstance().position_id + 1) + inst.position_id;
}


#endif // DEFPARAMGAME_H
