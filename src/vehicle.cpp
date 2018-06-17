#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
//#include "cost.h"

/**
 * Initializes Vehicle
 */

#define SAFE_DISTANCE_BEHIND (15)
#define SAFE_DISTANCE_FRONT (30)
#define TRACK_LENGTH (6945.554)
#define MAX_COST (100000.0)
#define LOOK_AHEAD_DISTANCE (100)
#define LOOK_BEHIND_DISTANCE (30)
#define PENALTY_LANE_CHANGE (10)

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v, double a, string state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    
    this->target_speed = 49.5 * 0.44704;

}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int,vector<Vehicle>> predictions) {
    
    vector<string> states = successor_states();
    double cost;
    vector<double> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;
    
    string best_state = select_best_state(states, predictions);
    cout << "best_state: " << best_state << endl;

    this->state = best_state;
    vector<Vehicle> trajectory = generate_trajectory(best_state, predictions);
    
    return trajectory;
}

string Vehicle::select_best_state(vector<string> states, map<int,vector<Vehicle>> predictions)
{
  vector<pair<double, string>> next_states_cost;
  
  for (const string &next_state : states) {
    double cost;
    if (next_state == "LCL") {
      cost = lane_change_cost(predictions, (-1));
    } else if (next_state == "LCR") {
      cost = lane_change_cost(predictions, (1));
    } else if (next_state == "KL") {
      cost = keep_lane_cost(predictions);
    } 
    
    next_states_cost.emplace_back(cost, next_state);
    cout << " Next_state: " << next_state << "   Cost: " << cost << endl;
  } // end of examining all possible next modes
  std::sort(next_states_cost.begin(),
            next_states_cost.end(),
            [](std::pair<double, string> &i, std::pair<double, string> &j) -> bool {
              return i.first < j.first;
            });
  string selected_state = next_states_cost[0].second;
  return selected_state;
  
}


double Vehicle::keep_lane_cost(map<int,vector<Vehicle>> predictions) {
  int currentLane = this->lane;
  return lane_front_cost(predictions, currentLane);
}

double Vehicle::lane_front_cost(map<int,vector<Vehicle>> predictions, const int lane) {
  Vehicle vehicle_ahead;

  double d = TRACK_LENGTH; //max_s

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) 
  {
    d = vehicle_ahead.s - this->s;
  }
  
  if(d < SAFE_DISTANCE_FRONT)
    return MAX_COST;
  
  if(LOOK_AHEAD_DISTANCE > d) {
    return LOOK_AHEAD_DISTANCE - d;
  }
  return 0.0;
}

double Vehicle::lane_behind_cost(map<int,vector<Vehicle>> predictions, const int lane) {

  Vehicle vehicle_behind;
  double d = TRACK_LENGTH;
  
  if (get_vehicle_behind(predictions, lane, vehicle_behind)) 
  {
    d = this->s - vehicle_behind.s;
  }
  
  if(d < SAFE_DISTANCE_BEHIND)
    return MAX_COST;
  
  if(LOOK_BEHIND_DISTANCE > d) {
    return LOOK_BEHIND_DISTANCE - d;
  }

  return 0.0;

}

double Vehicle::lane_change_cost(map<int,vector<Vehicle>> predictions, const int offset) {

  int intended_lane = this->lane + offset; // left = -1, -2.. etc // 0 is current lane // right = +1, +2..

  if ((intended_lane < 0) || (intended_lane > 2)) {
    return MAX_COST;
  }

  double cost = lane_front_cost(predictions, intended_lane) + lane_behind_cost(predictions, intended_lane);

  if (cost >= MAX_COST) {
    return MAX_COST;
  }

  cost += PENALTY_LANE_CHANGE;
  
  return cost;
}

vector<string> Vehicle::successor_states() {
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) 
  {
    states.push_back("LCL");
    states.push_back("LCR");
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
  Given a possible next state, generate the appropriate trajectory to realize the next state.
  */
  vector<Vehicle> trajectory;
  
  if (state.compare("KL") == 0) {
      trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
      trajectory = lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */

    double new_position;
    double new_velocity;
    double new_accel = 0;
    Vehicle vehicle_ahead;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
        new_velocity = vehicle_ahead.v - 2;
    } else {
        new_velocity = this->target_speed;
    }
    new_position = this->s + new_velocity*0.02; 
    return{new_position, new_velocity, new_accel};
    
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    vector<double> kinematics = get_kinematics(predictions, this->lane);
    trajectory.push_back(Vehicle(this->lane, kinematics[0], kinematics[1], kinematics[2], "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    vector<double> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], this->state));
    return trajectory;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -10;
    if(this->lane != lane)
    {
      max_s -= 10;
    }
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        temp_vehicle.s += (double) delta_time*temp_vehicle.v;
        if ((temp_vehicle.lane == lane) && (temp_vehicle.s < this->s) && (temp_vehicle.s - this->s) > max_s) {
            max_s = temp_vehicle.s - this->s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */

  int min_s = 30;
  if(this->lane != lane)
  {
    min_s += 10;
  }
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    temp_vehicle.s += (double) delta_time*temp_vehicle.v;
    if ((temp_vehicle.lane == lane) && ((temp_vehicle.s - this->s) < min_s) && (temp_vehicle.s > this->s)) {
      min_s = temp_vehicle.s - this->s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

// Functions for prediction of vehicles
void Vehicle::increment(int dt = 1) {
  this->s = position_at(dt);
  this->v = speed_at(dt);
  if (this->v > this->target_speed) this->v = this->target_speed;
}

double Vehicle::position_at(int t) {
  double s = this->s + this->v*t + this->a*t*t/2.0;
  return s;
}

double Vehicle::speed_at(int t)
{
  double v = this->v + this->a*t;
  return v;
}

vector<Vehicle> Vehicle::generate_predictions(int duration) {
  /*
  Generates predictions for non-ego vehicles to be used
  in trajectory generation for the ego vehicle.
  */
	vector<Vehicle> predictions;
  for(int i = 0; i < duration; i++) 
  {
    double t = (i*0.02);
    double next_s = position_at(t);
    double next_v = speed_at(t);
    predictions.push_back(Vehicle(this->lane, next_s, next_v, this->a, "KL"));
	}
  return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
  /*
  Sets state and kinematics for ego vehicle using the last state of the trajectory.
  */
  //cout << "trajectory size:" << trajectory.size() << endl;
  Vehicle next_state = trajectory[1];
  
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
}