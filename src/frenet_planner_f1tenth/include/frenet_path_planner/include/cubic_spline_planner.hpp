#ifndef CUBIC_SPLINE_PLANNER_HPP
#define	CUBIC_SPLINE_PLANNER_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

#define NONE -1e9
using namespace std;
using namespace Eigen;

typedef vector<double> vecD;


class Spline
{
	private:
		vecD a, b, c, d, w;
		vecD x, y;
		int nx; 

		MatrixXd calc_A(vecD h); 
		MatrixXd calc_B(vecD h);
		int search_index(double p);

	public:
		
		void init(vecD x_in, vecD y_in);
		double calc(double t); 
		double calcd(double t); 
		double calcdd(double t);
}; 


class Spline2D
{
	private:
	vecD x, y, s, ds;
	Spline sx,sy;

	public:	
		Spline2D(vecD x_in, vecD y_in)
		{
      std::cout << "prima di calc_s\n";
			s = calc_s(x_in, y_in);
      std::cout << "prima di init\n";
			sx.init(s, x_in);
			sy.init(s, y_in);
      std::cout << "dopo di init\n";
			/*x = x_in;
			y = y_in;

			s = calc_s(x, y);

			sx.init(s, x);
			sy.init(s, y);*/
		}


		vecD calc_s(vecD x, vecD y);
		void calc_position(double *x, double *y, double t);
		double calc_curvature(double t);
		double calc_yaw(double t);
		double get_s_last();
}; 

Spline2D calc_spline_course(vecD x, vecD y, vecD &rx, vecD &ry, vecD &ryaw, vecD &rk, double ds);
void printVecD(vecD A);

#endif
