#include "libfastsim/laser.hpp"
#include "libfastsim/map.hpp"
#include "libfastsim/misc.hpp"
#include "libfastsim/posture.hpp"
#include "libfastsim/robot.hpp"
#include "libfastsim/display.hpp"
#include "libfastsim/goal.hpp"
#include "libfastsim/radar.hpp"
#include "defParamGame.hpp"

#include <boost/shared_ptr.hpp>

class LIBFASTSIM{
    public:
      LIBFASTSIM() :
        _robot(20.0f) {
        _map = boost::shared_ptr<fastsim::Map>(new fastsim::Map(Params::simu::map_name(), 600.0f));
      }
      LIBFASTSIM(const fastsim::Map & m) :
        _robot(20.0f) {
        _map = boost::shared_ptr<fastsim::Map>(new fastsim::Map(m));
      }
      ~LIBFASTSIM() {}
      void init() {
      }
      void refresh() {
        _map->update(_robot.get_pos());
      }
      void init_view(bool dump = false) {
        _display =
          boost::shared_ptr<fastsim::Display>(new fastsim::Display(_map, _robot));
      }
      void set_map(const boost::shared_ptr<fastsim::Map>& map) {
        _map = map;
      }
      void refresh_view() {
        _display->update();
      }
      void refresh_map_view() {
        _display->update_map();
      }
      void switch_map() {
        _map->terrain_switch(Params::simu::alt_map_name());
      }
      void reset_map() {
        _map->terrain_switch(Params::simu::map_name());
      }
      void move_robot(float v1, float v2) {
        _robot.move(v1, v2, _map);
      }

      boost::shared_ptr<fastsim::Map> map() {
        return _map;
      }
      const boost::shared_ptr<fastsim::Map> map() const {
        return _map;
      }

      fastsim::Robot& robot() {
        return _robot;
      }
      const fastsim::Robot& robot() const {
        return _robot;
      }

      fastsim::Display& display() {
        return *_display;
      }
    protected:
      fastsim::Robot _robot;
      boost::shared_ptr<fastsim::Map> _map;
      boost::shared_ptr<fastsim::Display> _display;

};
