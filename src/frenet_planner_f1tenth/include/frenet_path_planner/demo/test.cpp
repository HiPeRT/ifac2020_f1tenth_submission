#include <matplotlibcpp.h>
#include <frenet_optimal_trajectory.hpp>

using namespace matplotlibcpp;

void quint_quart_test()
{
	double xs = 0;
	double vxs = 1.0;
	double axs = 2.0;
	double xe = 5.0;
	double vxe = 3.0;
	double axe = 4.0;
	double T = 2;
	double dT = 0.2;

	quartic quartic_poly(xs, vxs, axs, vxe, axe, T);
	quintic quintic_poly(xs, vxs, axs, xe, vxe, axe, T);

	cout << quartic_poly.calc_point(dT) << endl;
	cout << quartic_poly.calc_first_derivative(dT) << endl;
	cout << quartic_poly.calc_second_derivative(dT) << endl;
	cout << quartic_poly.calc_third_derivative(dT) << endl;

	cout << quintic_poly.calc_point(dT) << endl;
	cout << quintic_poly.calc_first_derivative(dT) << endl;
	cout << quintic_poly.calc_second_derivative(dT) << endl;
	cout << quintic_poly.calc_third_derivative(dT) << endl;
}

void spline_test()
{
	vecD x, y;
	
	//x = {-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
	//y = {0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0};

  x = {0.0, 10.0, 20.5, 35.0, 70.5};
  y = {0.0, -6.0, 5.0, 6.5, 0.0};
	
	vecD rx, ry, ryaw, rk;
	calc_spline_course(x, y, rx, ry, ryaw, rk, 0.1);
	// printVecD(rx);
	// printVecD(ry);
	// printVecD(ryaw);
	// printVecD(rk);

	// plot
	figure_size(1200, 780);
	plot(x, y, "xb");
	plot(rx, ry, "-r");
	grid(1);
	axis("equal");
	title("Final path");
	save("final_path.png");
	show();
	
}

void path_test()
{
	vecD wx = {0.0, 10.0, 20.5, 35.0, 70.5};
	vecD wy = {0.0, -6.0,  5.0,  6.5,  0.0};

  obstacle ob_a = {
    .x = 20,
    .y = 4.5,
    .radius = 0.5
  };

  obstacle ob_b = {
    .x = 70,
    .y = 0.0,
    .radius = 1
  };

  obstacle ob_c = {
    .x = 100,
    .y = 0.0,
    .radius = 0.2
  };

  std::vector<obstacle> obs{ob_a, ob_b};
  std::cout << obs[0].x << "\n";

	vecD tx, ty, tyaw, tc;	
	Spline2D csp = calc_spline_course(wx, wy, tx, ty, tyaw, tc, 0.1);

	double c_speed = 10.0 / 3.6, c_d = 2.0, c_d_d = 2.0, c_d_dd = 0.0, s0 = 0.0;

	double area = 20.0;

	for(int i = 0; i < 10; i++)
	{
		FrenetPath path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, obs);

		s0 = path.s[1];
		c_d = path.d[1];
		c_d_d = path.d_d[1];
		c_d_dd = path.d_dd[1];
		c_speed = path.s_d[1];

		if(sqrt(pow(path.x[1] - tx[-1], 2) + pow(path.y[1] - ty[-1], 2)) <= 1.0)
		{
			cout << "Goal!\n";
			break;
		}
		// cout << "Printing the path params :\n";
		// cout << s0 << " \n" << c_d << " \n" << c_d_d << " \n" << c_d_dd << " \n" << c_speed << "\n\n";
		// printVecD(path.x);
		// printVecD(path.y);
		//if(i == 9)
		//{
			plot();
			plot(tx, ty);
	        plot(path.x, path.y, "-or");
	        grid(1);	
          //pause(0);
          show();
		//}
	}

	grid(1);
	show();
}

int main()
{
	//quint_quart_test();
	//spline_test();
	
	path_test();
	return 0;
}


