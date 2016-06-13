#include "Utils.hpp"
#include <sys/time.h>
// #include <stdlib.h>
#include <random>
#include <algorithm>

namespace sml {

double Utils::rand01() {
//     LOG_DEBUG("rand");
    return (double)rand()/(double)RAND_MAX;
}

bool Utils::rand01(double limit) {
    if(limit > 0.L) {
        return Utils::rand01() < limit;
    }
    return false;
}

double Utils::randin(double a, double b) {
    assert( b > a);
//     LOG_DEBUG("rand");
    double random = ((double) rand()) / (double) RAND_MAX;
    double diff = b - a;
    double r = random * diff;
    return a + r;
}

double Utils::abs(double x) {
    if(x > 0)
        return x;
    else return - x;
}

// a < x < b => c < X < d
double Utils::transform(double x, double a, double b, double c, double d) {
    if(x < a)
        x=a;
    else if (x > b)
        x=b;

    return c + ((x - a)/(b - a))*(d - c);
}

time_t Utils::srand_mili(bool zero) {
    if(zero) {
        srand(0);
        return 0;
    }
    else {
        timeval t1;
        gettimeofday(&t1, NULL);
        srand(t1.tv_usec * t1.tv_sec);
        return t1.tv_usec * t1.tv_sec;
    }
}


double* Utils::genNrand(int N, double max) {
    double* tab = new double[N];
    tab[0] = 0.;
    for(int i=1; i<N; i++)
        tab[i] = rand01() * max;

    std::sort(tab, tab + N, std::less<double>());
    return tab;
}

// high temperature means high random
unsigned int Utils::boltzmann(const vector<double> & values, double temperature) {
    vector<double> proba(values.size());
    vector<double> rproba(values.size());
    vector<double> rexp(values.size());

    double max_component = values[0]/temperature;
    for(uint i=1; i < values.size();i++)
	if(values[i]/temperature > max_component)
		max_component = values[i]/temperature;

    double sum = 0.;
    for(uint i=0; i<values.size(); i++) {
        rexp[i] = std::exp((values[i]/temperature) - max_component );
        sum += rexp[i];
    }

    double last_prob = 0.;
    for(uint i=0; i<values.size(); i++) {
        rproba[i] = rexp[i]/sum;
        proba[i]= last_prob + rproba[i];
        last_prob = proba[i];
    }

    double number = Utils::rand01();
    unsigned int retrn = RAND_MAX;

    if(number >= 1.f)
        return values.size() - 1;

    for(unsigned int i=0; i<values.size(); i++)
        if(number <= proba[i]) {
            retrn=i;
            break;
        }

    return retrn;
}

}
