#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};
  int lane;

  double v;
  double a;
  double target_speed;
  
  int id = -1;
  double x = 0;
  double y = 0;
  double s = 0;
  double d = 0;
  double yaw = 0;
  double speed = 0;
  
  double delta_time;
  
  string state;
  vector<string> available_states;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, double s, double v, double a, string state="CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);
  string select_best_state(vector<string> states, map<int,vector<Vehicle>> predictions);
  vector<string> successor_states();
  double keep_lane_cost(map<int,vector<Vehicle>> predictions);
  double lane_change_cost(map<int,vector<Vehicle>> predictions, const int offset);
  double lane_front_cost(map<int,vector<Vehicle>> predictions, const int lane);
  double lane_behind_cost(map<int,vector<Vehicle>> predictions, const int lane);

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);
  vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);
  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);
  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  void increment(int dt);
  double position_at(int t);
  double speed_at(int t);
  vector<Vehicle> generate_predictions(int);
  void realize_next_state(vector<Vehicle> trajectory);
};

#endif