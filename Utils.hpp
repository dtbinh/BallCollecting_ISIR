#ifndef UTILS_HPP
#define UTILS_HPP

///
///\file Utils.hpp
///\brief les méthodes utiles
///
///

#include "Logger.hpp"
#include <vector>

using std::vector;

namespace sml {

class Utils
{
public :

///
///\brief Retourner 0 ou 1
    static double rand01();

    static bool rand01(double);

    static double randin(double a, double b);
///
///\brief Retourner la valeur absolue de "x"
///\param x : une valeur
    static double abs(double x);

///
///\brief transformer la valeur x qui appartient [a,b] à [c,d]
///\param x :une valeur
///	  a,b : intervalle [a,b]
///	  c,d: intervalle [c,d]
    static double transform(double x, double a, double b, double c, double d);

    static time_t srand_mili(bool zero=false);

    static double* genNrand(int N, double max);

    template<typename T>
    static std::pair<double, double> mean_and_var(const T& _list) {

        double mean = 0.f;
        for(auto it=_list.cbegin();it != _list.cend(); ++it) {
            double p = *it;
            mean += p;
        }
        mean = (double)(mean/(double)_list.size());

        double variance = 0.f;
        for(auto it=_list.cbegin();it != _list.cend(); ++it) {
            double p = *it;
            variance += p * p;
        }
        variance = (double)(variance/(double)_list.size());
        variance = variance - mean*mean;

        return {mean, variance};
    }

    static unsigned int boltzmann(const vector<double> &, double);

};

}

#endif // UTILS_HPP
