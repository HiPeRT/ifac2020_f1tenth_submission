#include "../include/frenet_optimal_trajectory.hpp"
#include <chrono>
#include <numeric>

// Parameter
double MAX_SPEED = 10; // maximum speed [m/s]
double MAX_ACCEL = 10.0;  // maximum acceleration [m/ss]
double MAX_CURVATURE = 10.0;  // maximum curvature [1/m]
double MAX_ROAD_WIDTH = 3.0;  // maximum road width [m]
double D_ROAD_W = 0.2;//0.1;  // road width sampling length [m]
double DT = 0.1;  // time tick [s]
double MAXT = 7.0;  // max prediction time [m]
double MINT = 7.0;  // min prediction time [m]
double TARGET_SPEED = 6.0;  // target speed [m/s]
double D_T_S = 1;   // target speed sampling length [m/s]
double N_S_SAMPLE = 0.5;// sampling number of target speed
double ROBOT_RADIUS = 2.0;  // robot radius [m]
double MAX_ROAD_WIDTH_LEFT = 3;
double MAX_ROAD_WIDTH_RIGHT = 3;

double safe_distance = 0.2;

//double minV = 3
//double maxV = 4


// cost weights
/*double KJ = 1.0;
double KT = 1.0;
double KD = 0.5;
double KLAT = 1.0;
double KLON = 2.0;*/

double KJ = 0.01;
double KT = 0.1;
double KD = 2.0;
double KLAT = 1.0;
double KLON = 1.0;

std::vector<double> linspace(double start_in, double end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
  {
      linspaced.push_back(start);
      return linspaced;
  }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
  {
      linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
  return linspaced;
}

// generates frenet path parameters including the cost
vector<FrenetPath> calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0)
{
    vector<FrenetPath> frenet_paths;
    
    for(double di = -MAX_ROAD_WIDTH_LEFT; di < MAX_ROAD_WIDTH_RIGHT; di += D_ROAD_W) //di <= MAX_ROAD_WIDTH
    {
	//for(double Ti = MINT; Ti < MAXT; Ti += DT)
	for(double Ti = MINT; Ti <= MAXT + DT; Ti += DT)
	{
	    FrenetPath fp;

	    quintic lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);

	    vector<double> vecT = linspace(0.0, Ti, 15); //15
	    double t;

	    //for(double t = 0.0; t <= Ti + DT; t += DT)  // linspace
	    for (int i=0; i<vecT.size(); i++)
	    {
		t = vecT[i];
		fp.t.push_back(t);
		fp.d.push_back(lat_qp.calc_point(t));
		fp.d_d.push_back(lat_qp.calc_first_derivative(t));
		fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
		fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
	    }

	    double Jp = std::inner_product(fp.d_ddd.begin(), fp.d_ddd.end(), fp.d_ddd.begin(), 0);
	    double minV = TARGET_SPEED - D_T_S*N_S_SAMPLE;
	    double maxV = TARGET_SPEED + D_T_S*N_S_SAMPLE;

	    //for(double tv = minV; tv <= maxV + D_T_S; tv += D_T_S)
	    for(double tv = 3; tv < 4; tv += 1)
	    {
		FrenetPath tfp = fp;
		quartic lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

		for(auto const& t : fp.t)
		{
		    tfp.s.push_back(lon_qp.calc_point(t));
		    tfp.s_d.push_back(lon_qp.calc_first_derivative(t));
		    //tfp.s_dd.push_back(lon_qp.calc_second_derivative(t));
		    tfp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
		}

		double Js = std::inner_product(tfp.s_ddd.begin(), tfp.s_ddd.end(), tfp.s_ddd.begin(), 0);

		double ds = pow((TARGET_SPEED - tfp.s_d.back()), 2);

		tfp.cd = KJ*Jp + KT*Ti + KD*tfp.d.back()*tfp.d.back();
		tfp.cv = KJ*Js + KT*Ti + KD*ds;
		tfp.cf = KLAT*tfp.cd + KLON*tfp.cv;

		frenet_paths.push_back(tfp);
	    }
	}

    }

    return frenet_paths;
}

// convert to global frame 
vector<FrenetPath> calc_global_paths(vector<FrenetPath> fplist, Spline2D csp)
{
    for(auto& fp : fplist)
    {

	for(int i = 0; i < fp.s.size(); i++)
	{
	    double ix, iy;
	    csp.calc_position(&ix, &iy, fp.s[i]);

	    if(ix == NONE)
		break;
	    double iyaw = csp.calc_yaw(fp.s[i]);
	    double di = fp.d[i];

	    //double fx = ix - di*sin(iyaw);
	    //double fy = iy + di*cos(iyaw);

	    double fx = ix + di*cos(iyaw + M_PI/2.0);
	    double fy = iy + di*sin(iyaw + M_PI/2.0);

	    fp.x.push_back(fx);
	    fp.y.push_back(fy);
	}


	/*for(int i = 0; i < fp.x.size() - 1; i++)
	{
	    double dx = fp.x[i + 1] - fp.x[i];
	    double dy = fp.y[i + 1] - fp.y[i];

	    fp.yaw.push_back(atan2(dy, dx));
	    fp.ds.push_back(sqrt(dx*dx + dy*dy));
	}


	for(int i = 0; i < fp.yaw.size() - 1; i++)
	    fp.c.push_back((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i]);*/
    }

    return fplist;
}	

double dist(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

bool check_collision_path(FrenetPath fp, vector<obstacle> obs)
{
    double di;
    for (auto const& ob : obs)
    {
        for (int j=0; j<fp.x.size()/2; j++)
        {
	    //std::cout << fp.x[j] << " " << fp.y[j] << " " << fp.x.size() << endl;
	    di = dist(fp.x[j], fp.y[j], ob.x, ob.y) - ob.radius;
	    if (di < safe_distance)
		return 1;
        }
    }
    return 0;
}

bool check_collision_opp(FrenetPath fp, obstacle ob)
{
    double di;
    for (int j=0; j<fp.x.size()/2; j++)
    {
	di = dist(fp.x[j], fp.y[j], ob.x, ob.y) - ob.radius;
	//std::cout << fp.x[j] << " " << fp.y[j] << " " << fp.x.size() << endl;
	if (di < safe_distance)
	    return 1;
    }
    return 0;
}

vector<FrenetPath> check_path_new(vector<FrenetPath> fplist, vector<obstacle> obs, obstacle opp_obs)
{
	//cout << opp_obs.x << " " << opp_obs.y << " " << opp_obs.radius << endl;
	vector<FrenetPath> fplist_final;
	for(int i = 0; i < fplist.size(); i++)
		if(check_collision_opp(fplist[i], opp_obs)==0)
			if(check_collision_path(fplist[i], obs)==0)
				fplist_final.push_back(fplist[i]);
	return fplist_final;
}

FrenetPath frenet_optimal_planning(Spline2D csp, double s0, double c_speed, double c_d, double c_d_d, double c_d_dd, vector<obstacle> obstacles, FrenetPath &first, FrenetPath &last, obstacle opp_obs, int overtake_strategy)
{

  switch(overtake_strategy)
  {
      /*case 2:
        MAX_ROAD_WIDTH_LEFT = MAX_ROAD_WIDTH;
        MAX_ROAD_WIDTH_RIGHT = 0;
        break;
      case 1:
        MAX_ROAD_WIDTH_LEFT = 0;
        MAX_ROAD_WIDTH_RIGHT = MAX_ROAD_WIDTH;
        break;*/
      default:
        MAX_ROAD_WIDTH_LEFT = MAX_ROAD_WIDTH;
        MAX_ROAD_WIDTH_RIGHT = MAX_ROAD_WIDTH;
        break;
  }

  vector<FrenetPath> fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);

  //cout<<"NUM PATH1: "<<fplist.size()<<endl;
  fplist = calc_global_paths(fplist, csp);

  first = fplist[0];
  last = fplist[fplist.size()-1];

  fplist = check_path_new(fplist, obstacles, opp_obs);
  //cout<<"NUM PATH dopo check: "<<fplist.size()<<endl;
  //std::cout << "path size: " << fplist[0].x.size() << "\n";

  double min_cost = FLT_MAX;

  FrenetPath bestpath;
  bestpath.empty = true;
  for(auto const& fp : fplist)
  {
      if(min_cost >= fp.cf)
      {
	  min_cost = fp.cf;
	  bestpath = fp;
	  bestpath.empty = false;
      }
  }

  return bestpath;
}

FrenetPath frenet_optimal_planning(Spline2D csp, double s0, double c_speed, double c_d, double c_d_d, double c_d_dd, vector<obstacle> obstacles, obstacle opp_obs)
{
    vector<FrenetPath> fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);

    //cout<<"NUM PATH1: "<<fplist.size()<<endl;
    fplist = calc_global_paths(fplist, csp);

    fplist = check_path_new(fplist, obstacles, opp_obs);
    //cout<<"NUM PATH3: "<<fplist.size()<<endl;
    
    double min_cost = FLT_MAX;
    FrenetPath bestpath;

    for(auto const& fp : fplist)
    {
	if(min_cost >= fp.cf)
	{
	    min_cost = fp.cf;
	    bestpath = fp;
	    bestpath.empty = false;
	}
    }
    
    return bestpath;
}
