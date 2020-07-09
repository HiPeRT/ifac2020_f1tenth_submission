#ifndef FRENET_PLANNER_HPP
#define FRENET_PLANNER_HPP


#include <algorithm>
#include <cfloat>
#include <polynomials.hpp>
#include <cubic_spline_planner.hpp>

struct obstacle{
    double x, y, radius;
};

class FrenetPath{
	public :

		vecD t, d, d_d, d_dd, d_ddd, s, s_d, s_dd, s_ddd, x, y, yaw, ds, c;
		double cd, cv, cf;

};

vector<FrenetPath> calc_frenet_paths(double, double, double, double, double);
vector<FrenetPath> calc_global_paths(vector<FrenetPath> &, double);
vector<FrenetPath> check_path(vector<FrenetPath>	);
FrenetPath frenet_optimal_planning(Spline2D, double, double, double, double, double, vector<obstacle> );
FrenetPath frenet_optimal_planning(Spline2D, double, double, double, double, double, vector<obstacle> , FrenetPath &, FrenetPath &);
vector<double> linspace(double start_in, double end_in, int num_in);
bool check_collision_new(FrenetPath, vector<obstacle> );

#endif
