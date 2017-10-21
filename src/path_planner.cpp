#include <iostream>
#include <vector>
#include <cmath>
#include "spline.h"
#include <map>
#include <string>
#include <iterator>
#include <limits>
#include <algorithm>


using namespace std;

/*** Previously in main.cpp ***/

// For converting back and forth between radians and degrees.
constexpr double pi()
{
	return M_PI;
}
double deg2rad(double x)
{
	return x * pi() / 180;
}
double rad2deg(double x)
{
	return x * 180 / pi();
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x,
		vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x,
		vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
		vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1],
				maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return
	{	frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s,
		vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
			(maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x,y};

}

/*** /Previously in main.cpp ***/


//1 mph is equal to 0.44704 meter/second.
inline double mph2mps(double mph)
{
	return mph * 0.44704;
}

inline double mps2mph(double mps)
{
	return mps / 0.44704;
}

enum closeness_level {not_close, too_close, emergency_too_close};

struct Collision_avoidance
{
	double min_frontal_gap; // Minimum frontal space in meters to keep before reducing speed
	double emergency_min_frontal_gap; // Minimum frontal space in meters to make an emergency brake
	double frontal_safe_gap; // Distance in meter that delimite a safe area around the car to consider a lane change is secure
	double rear_safe_gap;
	closeness_level closeness; // not_close if we are above min_frontal_gap
								// too_close if we are below min_frontal_gap threshold
								// emergency_too_close if we are below emergency_min_frontal_gap threshold
};

struct Velocity_manager
{
	double target_vel; // Reference velocity to target in mph
	// The following var is removed from here due to issues related with the static qualifier
	//static double curr_vel; // This is to avoid the cold start problem (to avoid surpass max jerk)
	double step_up_vel; // Slowly increase velocity byt step_up_vel
	double step_down_vel; // Slowly decrease velocity byt step_down_vel
	double emergency_step_down_vel; // Quickly decrease velocity by emergency_step_down_vel
};

enum finite_state_machine 
{
	KL, 	// Keep Lane
	PLCL, 	// Prepare Lane Change Left
	PLCR, 	// Prepare Lane Change Right
	LCL, 	// Lane Change Left
	LCR		// Lane Chane Right
};

enum e_level
{
	TOTALLY_EMPTY, 	// There is no car in a safe area around my car
	ALMOST_EMPTY, 	// Cannot incorporate right now, need to locate behind the following car)
	NOT_EMPTY 		// There is no drivable empty space in this lane
};


/**
	Given a car in the same lane, it computes the gap between us and the car.
	The gap could be frontal (positive) or rear (negative).
*/
double compute_gap(const vector<double> &car, int size_previous_path, double car_s)
{
	double vx = car[3];
	double vy = car[4];
	double check_speed = sqrt(vx*vx + vy*vy);
	double check_car_s = car[5];

	check_car_s += (double)size_previous_path*0.02*check_speed; // Because we are reusing previous points

	// Is it the car too close?
	double gap = check_car_s - car_s;

	return gap;
}


/**
    Returns the emptiness level of a lane
		@return see enum e_level

*/
e_level emptiness_level(int target_lane, int lane_width, int lanes_available, const vector<double> &other_car, \
	int size_previous_path, double car_s, double frontal_safe_gap, double rear_safe_gap)
{
	double inf = numeric_limits<double>::max();

	// Check if lane has sense
	if(target_lane < 0 || target_lane >= lanes_available)
		return NOT_EMPTY;

	// Check if there is a car in this lane

	double other_car_d = other_car[6];
	double lane_center = target_lane*lane_width + lane_width/2;

	// There is a car
	if(other_car_d < lane_center + lane_width/2 \
			&& other_car_d > lane_center - lane_width/2)
	{
		// Check car_s in respect to my s
		double gap = compute_gap(other_car, size_previous_path, car_s);

		// A positive gap means the car is in front

		if(gap > 0 && gap > frontal_safe_gap)
			return TOTALLY_EMPTY;
		else if(gap < 0 && rear_safe_gap)
			return TOTALLY_EMPTY;
		else
			return NOT_EMPTY;

	}
	// There is no car
	else 
		return TOTALLY_EMPTY;
}


/**
	Returns the cost of changing the lane. It just take into account if the lane exist and if it's our destiny's lane.

*/
/*
void cost_lane_changing(int future_lane, finite_state_machine future_state, map<finite_state_machine, vector<double> > &costs)
{
	double cost, delta_d, delta_s;
	double inf = numeric_limits<double>::max();

	delta_d = future_lane - goal_lane;
	delta_s = goal_s - this->s;

	if(future_lane < 0 || future_lane > lanes_available)
		cost = inf;
	else
		//cost = 1 - exp(-(fabs(delta_d)/delta_s)); TODO FUTURE
		cost = 1;

	costs[future_state].push_back(cost);
}
*/

vector< vector<double> > path_planner(const vector<double> &previous_path_x, const vector<double> &previous_path_y, \
					double car_x, double car_y, double car_yaw, \
					double car_s, vector<double> &map_waypoints_s, \
					vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, \
					const vector< vector<double> > &sensor_fusion)
{
	static double curr_vel = 0.0; // This is to avoid the cold start problem (to avoid surpass max jerk)
	Velocity_manager v_man = {.target_vel = 49., .step_up_vel = 0.7, \
		.step_down_vel = 0.5, .emergency_step_down_vel = 2.0};

	int lanes_available = 3;
	static double desired_lane = 1;
	double lane_width = 4;
	double next_d;
	unsigned int size_previous_path = previous_path_x.size();
	vector<double> ptsx, ptsy; // List of widely spaced waypoints evenly spaced at 30 m

	Collision_avoidance col_avoidance = {.min_frontal_gap = 40,	.emergency_min_frontal_gap = 25, \
		.frontal_safe_gap = col_avoidance.min_frontal_gap, .rear_safe_gap = 15, .closeness = not_close};

	double vx, vy, check_speed, check_car_s;
	double other_car_d;
	double gap;

	static finite_state_machine curr_state = KL;

	vector< pair<e_level, int> > l_elevel_ls, curr_elevel_ls, r_elevel_ls; // To measure emptiness levels in adjacent lanes
	pair<e_level, int> l_elevel, curr_elevel, r_elevel; // To reduce emptiness levels in adjacent lanes in a single value

	int car_id;
	double s_nearest_car;
	int full_transition = 18;
	double step = 1.0/full_transition;
	static int transition = 0;


	if(curr_state != PLCL && curr_state != PLCL)
	{
		/* Get decision */
		/****************/

		// Compute emptiness levels
		for(int i=0; i<sensor_fusion.size(); i++)
		{

			l_elevel_ls.push_back(\
				make_pair(emptiness_level(desired_lane - 1, lane_width, lanes_available, sensor_fusion[i], \
					size_previous_path, car_s, col_avoidance.frontal_safe_gap, col_avoidance.rear_safe_gap), i));

			curr_elevel_ls.push_back(\
				make_pair(emptiness_level(desired_lane 	  , lane_width, lanes_available, sensor_fusion[i], \
					size_previous_path, car_s, col_avoidance.frontal_safe_gap, col_avoidance.rear_safe_gap), i));

			r_elevel_ls.push_back(\
				make_pair(emptiness_level(desired_lane + 1, lane_width, lanes_available, sensor_fusion[i], \
					size_previous_path, car_s, col_avoidance.frontal_safe_gap, col_avoidance.rear_safe_gap), i));
		}


		// Reduce levels to a single value
		auto compare_first = [](pair<e_level, int> i, pair<e_level, int> j) {return i.first < j.first;};

		// TODO For other lanes, I will be interested only in the car with smallest s which is in my safety area
		l_elevel = *max_element(l_elevel_ls.begin(), l_elevel_ls.end(), compare_first);
		curr_elevel = *max_element(curr_elevel_ls.begin(), curr_elevel_ls.end(), compare_first);
		r_elevel = *max_element(r_elevel_ls.begin(), r_elevel_ls.end(), compare_first);



		car_id = curr_elevel.second;
		gap = compute_gap(sensor_fusion[car_id], size_previous_path, car_s);

		// If there is no problem, just continue in lane
		if ( (curr_elevel.first == TOTALLY_EMPTY) \
			|| (curr_elevel.first == ALMOST_EMPTY && (gap < 0)) ) // negative gap means the car is behind
		{
			curr_state = KL;
			col_avoidance.closeness = not_close;
		}
		// A problem happens
		else
		{
			//TODO DEBUG
			cout << "Problem in my lane detected" << endl;
			auto print = [](pair<e_level, int> e){ cout << "(" << e.first << ", " << e.second << "). ";};

			cout << "l_elevel: ";
			print(l_elevel);
			cout << "d: " << sensor_fusion[l_elevel.second][6];
			//for_each(l_elevel_ls.begin(), l_elevel_ls.end(), print);
			cout << endl << "curr_elevel: ";
			print(curr_elevel);
			cout << "d: " << sensor_fusion[curr_elevel.second][6];
			//for_each(curr_elevel_ls.begin(), curr_elevel_ls.end(), print);
			cout << endl << "r_elevel: ";
			print(r_elevel);
			cout << "d: " << sensor_fusion[r_elevel.second][6];
			//for_each(r_elevel_ls.begin(), r_elevel_ls.end(), print);
			cout << endl;
			//TODO DEBUG


			// It's there any lane better?
			if(l_elevel.first < curr_elevel.first)
			{
				curr_state = PLCL;
			}
			else if(r_elevel.first < curr_elevel.first)
			{
				curr_state = PLCR;
			}
			// If it's not better in other places, we should keep
			else
			{
				curr_state = KL;
				col_avoidance.closeness = too_close;

				if(gap < col_avoidance.emergency_min_frontal_gap)
					col_avoidance.closeness = emergency_too_close;

				car_id = curr_elevel.second;
			}

		}
	}
	else if(curr_state == PLCL)
	{
		transition ++;

		if(transition == full_transition)
		{
			curr_state = LCL;
			transition = 0;
		}
	}
	else if(curr_state == PLCR)
	{
		transition ++;

		if(transition == full_transition)
		{
			curr_state = LCR;
			transition = 0;
		}
	}

	/* Execute decision */
	/********************/

	switch(curr_state)
	{
		case KL:

			// TODO DEBUG
			cout << "KL state. ";

			switch(col_avoidance.closeness)
			{
				case not_close:

					curr_vel = fmin(curr_vel + v_man.step_up_vel, v_man.target_vel);

					// TODO DEBUG
					cout << "We are safe." << " curr_vel: " << curr_vel << endl;

					break;

				case too_close:

					curr_vel = fmax(1, curr_vel - v_man.step_down_vel);
					col_avoidance.closeness = too_close;

					// TODO DEBUG
					cout << "Too close. Frontal gap: " << gap << " curr_vel: " << curr_vel << endl;

					break;

				case emergency_too_close:

					curr_vel = fmax(1, curr_vel - v_man.emergency_step_down_vel);
					col_avoidance.closeness = emergency_too_close;

					// TODO DEBUG
					cout << "Emergency. Frontal gap: " << gap << " curr_vel: " << curr_vel << endl;

					break;

				default:
					cout << "There is a bug in switch(col_avoidance.closeness)" << endl;
					exit(-1);
			}

			break;
			
		case PLCL:

			desired_lane -= step;
			transition += step;

			//TODO DEBUG
			cout << "PLCL state" << " curr_vel: " << curr_vel << endl;

			break;

		case PLCR:

			desired_lane += step;
			transition += step;

			//TODO DEBUG
			cout << "PLCR state" << " curr_vel: " << curr_vel << endl;

			break;


		case LCL:

			//TODO DEBUG
			cout << "LCL state" << " curr_vel: " << curr_vel << endl;

			break;

		case LCR:

			//TODO DEBUG
			cout << "LCR state"  << " curr_vel: " << curr_vel << endl;

			break;

		default:
			cout << "There is a bug in switch(curr_state)" << endl;
			exit(-1);
	}

	next_d = desired_lane*lane_width + lane_width/2;

	// TODO DEBUG
	if (curr_state == PLCL || curr_state == PLCR)
		cout << "next_d: " << next_d << endl;




	/* Computing a reference state */
	/******************************/

	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	double prev_ref_x, prev_ref_y;

	// The path is almost empty, we need to use the car as starting reference
	if(size_previous_path < 2)
	{
		// Making a path tangent to the car, going back in time
		prev_ref_x = ref_x - cos(ref_yaw); // video walkthrough said cos(car_yaw) but it's a huge bug
		prev_ref_y = ref_y - sin(ref_yaw);

		ptsx.push_back(prev_ref_x);
		ptsx.push_back(ref_x);

		ptsy.push_back(prev_ref_y);
		ptsy.push_back(ref_y);

	}
	else
	{
		ref_x = previous_path_x[size_previous_path-1];
		prev_ref_x = previous_path_x[size_previous_path-2];

		ref_y = previous_path_y[size_previous_path-1];
		prev_ref_y = previous_path_y[size_previous_path-2];

		ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

		ptsx.push_back(prev_ref_x);
		ptsx.push_back(ref_x);

		ptsy.push_back(prev_ref_y);
		ptsy.push_back(ref_y);
	}

	/* Adding future waypoints to interpolate 
		and changing reference's frame      */
	/***************************************/

	// Adding 30 m evenly spaced points

	vector<double> next_wp0 = getXY(car_s + 30, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s + 60, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s + 90, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);


	// Changing reference's frame to local car's coordinates
	double shift_x, shift_y;

	for(int i=0; i<ptsx.size(); i++)
	{
		shift_x = ptsx[i] - ref_x;
		shift_y = ptsy[i] - ref_y;

		ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
		ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
	}

	/* splines time */
	/****************/

	tk::spline s;

	// TODO DEBUG
	/*
	auto print = [](const double& n) {cout << n << ", ";};
	cout << "ptsx: ";
	for_each(ptsx.begin(), ptsx.end(), print);
	cout << endl << "ptsy: ";
	for_each(ptsy.begin(), ptsy.end(), print);
	cout << endl;
	*/



	s.set_points(ptsx, ptsy);

	/* Prepare fine waypoints from coarse 
		waypoints + previous (fine) path */
	/*************************************/

	vector<double> next_x_vals, next_y_vals;

	for(int i=0; i<size_previous_path; i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);

	double x_add_on = 0;
	
	double N, x_point, y_point, x_ref, y_ref;

	for(int i=1; i <= 50 - size_previous_path; i++)
	{
		N = (target_dist / ( 0.02*mph2mps(curr_vel) )); // Each point is visited every 0.02 seconds
		x_point = x_add_on + target_x/N;
		y_point = s(x_point);

		x_add_on = x_point;

		x_ref = x_point;
		y_ref = y_point;

		// Undoing coordinate transformations
		x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
		y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}


	vector< vector <double> > next_vals = {next_x_vals, next_y_vals};

	return next_vals;
}