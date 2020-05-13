#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  //Start in lane 1;
  int lane = 1;
  
  //Have a reference velocity at start
  double ref_vel = 0.0;
  
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          int prev_size = previous_path_x.size();
          
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          //Init variables
          bool too_close = false;
          bool car_front = false;
          bool car_left = false;
          bool car_right = false;

          //Set maximum speed mph so we dont break the law
          double maximum_ref_vel = 49.5;
          double car_front_speed;
          double distance_between;

          for(int i = 0; i < sensor_fusion.size(); i++)
            {
              //Get info from sensor fusion. Note that the detected vehicle velocity is in m/s
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              //Get ego vehicle current lane
              double check_car_d = sensor_fusion[i][6];
              int detected_car_lane = -1;

              //Assign lane ID based on the d value. 0 is far left lane. 
              if ((check_car_d < 4) && (check_car_d > 0))
                  {
                    detected_car_lane = 0;
                  }

              else if ((check_car_d < 8) && (check_car_d > 4))
                  {
                    detected_car_lane = 1;
                  }
              else
                  {
                    detected_car_lane = 2;
                  }



              //Predict ego vehicle S position based on it's velocity
              check_car_s += ((double)prev_size*0.02*check_speed);

              // std::cout << "DETECTED_CAR_LANE :" << detected_car_lane << std::endl;

              // Check vehicle in front
              // First by checking the detected is in the same lane as ego, then check the vehicle is infront of ego instead of behind. 
              // We also don't want to trigger flag when the detected vehicle is still too far away, so we only trigger if the detected vehicle is within 20 meter range. 
              if ((detected_car_lane == lane) && (check_car_s > (car_s - 2)) && ((check_car_s - car_s) < 20))
                  { 
                    //Keep the distance between ego and detected vehicle infront
                    distance_between= check_car_s - car_s;

                    //Keep the relative speed of the detected vehicle infront
                    car_front_speed = check_speed;
                    car_front = true;
                  }
              //Check whether there are vehicles at left and right lane. 
              else if ((lane - detected_car_lane == 1) && ((check_car_s - car_s) > -5) && ((check_car_s - car_s) < 20))
                  { 
                    car_left = true;
                  }
              else if ((lane - detected_car_lane == -1) && ((check_car_s - car_s) > -5) && ((check_car_s - car_s) < 20))
                  {
                     car_right = true;
                  }
            }

          std::cout << "DETECTED CAR FRONT :" << car_front << std::endl;
          std::cout << "DETECTED CAR LEFT :" << car_left << std::endl;
          std::cout << "DETECTED CAR RIGHT :" << car_right << std::endl;
          std::cout << "EGO CAR LANE :" << lane << std::endl;
          std::cout << "CAR FRONT DISTANCE :" << distance_between << std::endl;

          std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

          //Behavious Planning
          //If there is vehicle in front. 
          if (car_front)
          {
            //To avoid jerk but also decelerates hard enough if the vehicle in front brakes suddenly. 
            ref_vel -= 0.724;

            // If there is vehicle in front of ego, the ego will match its speed. 
            // Times 2.24 to convert to mph
            if (ref_vel <= car_front_speed * 2.24)
              {
                ref_vel = car_front_speed * 2.24;
              }

            // To change lane, the distance between the ego and the front vehicle must at least 15.0 meter apart to avoid collision with the front vehicle. 
            if (distance_between > 15.0)
            { 
              //Lane keeping when it is not safe to change when both vehicle exists on both sides. 
              if ((car_left) && (car_right))
              { 
                lane = lane;
              }
              //If there is vehicle on the left, and no vehicle on the right, then it is safe to change to right lane. 
              else if (car_left)
              {  
                //However if the ego is already on the far right lane, ego will stay on lane. 
                if (lane != 2)
                  {
                  lane += 1; 
                  // lane = lane;
                  }
              }
              //If there is vehicle on the right, and no vehicle on the left, then it is safe to change to left lane. 
              else if (car_right)
              { 
                //However if the ego is already on the far left lane, ego will stay on lane. 
                if (lane != 0)
                {
                  lane -= 1;
                  // lane = lane;
                }                
              }
              //If there is no vehicle on both sides, 
              //Priority will given to change to left lane.
              else if ((!car_left) && (!car_right))
              {
                if (lane != 0)
                  {
                    lane -= 1;
                    // lane = lane;
                  }
                else if (lane != 2)
                  {
                    lane += 1;
                    // lane = lane;
                  }
                //Priority to left lane
                else if (lane == 1)
                  {
                    // lane = lane;
                    lane -=1;
                  }
              }
            }
            //Well if the distance_between is less than 15 m, ego vehicle starts to slow down. 
            else
            {
              ref_vel -= 0.724;
            }

          }
          else
          { 
            //To avoid jerk and only limit to maximum speed that has been set earlier. 
            ref_vel += 0.524;
            if (ref_vel > maximum_ref_vel)
            {
            ref_vel = maximum_ref_vel;
            }
          }
          std::cout << "REF VEL :" << ref_vel << std::endl;
          vector<double> ptsx;  
          vector<double> ptsy;  
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          //If previous path size is short. 
          if (prev_size < 2)
          {
            //Use two points that make the path tangent to the ego vehicle
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);               
          }
          // If there is a previous path, 
          else
          {
            //Get the last point of that path as reference
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            //Use two points that make the path tangent to the ego vehicle
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);         
            
          }
          

          // In Frenet add evenly 30 m spaced points ahead of the starting reference. 
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]); 
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
            
          for (int i = 0; i < ptsx.size(); i++)
          {
            //Shift vehicle reference to 0 degrees. 
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            //Applying rotational transformation. Global coordinate to local coordinate. 
            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            
          }

          tk::spline s;
          
          //Get spline
          s.set_points(ptsx, ptsy);
          
          //Define actual coordinates for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          

          //start with the previous path coordinates
          for(int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          double x_add_on = 0;
          
          // Fill up the rest of the path planner after filling it with previous path. 
          // Here we will always output 50 points. 
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) 
          {
            
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
           	double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate it back to global coordinates
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}