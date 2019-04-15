#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

double ref_vel = 0.0; // target velocity
 int lane = 1; // left : 0, middle : 1, right : 2;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          std::cout << "--------------------------------------" << std::endl;
          std::cout << "car_x : " << car_x << std::endl;
          std::cout << "car_y : " << car_y << std::endl;
          std::cout << "car_yaw : " << car_yaw << std::endl;
          std::cout << "car_s : " << car_s << std::endl;
          std::cout << "car_d : " << car_d << std::endl;
          std::cout << "car_speed : " << car_speed  << std::endl;
          
          bool too_close = false; // deacc flag
          int prev_size = previous_path_x.size(); // number of previous path existing
          std::cout << "prev_size : " << prev_size << std::endl;
          
          // if previous path exists
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }   
          
          // sensing aroud
          bool left_lane = true;
          bool right_lane = true;
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            std::cout << "d : " << d << std::endl;
            float s = sensor_fusion[i][5];
            
            // left lane sensing
            if (abs(s - car_s) < 30 &&  d < (2+4*(lane-1)+2) && d > (2+4*(lane-1)-2) )
            {
              left_lane = false;
            }
            
            // right lane sensing
            if (abs(s - car_s) < 30 &&  d < (2+4*(lane+1)+2) && d > (2+4*(lane+1)-2) )
            {
              right_lane = false;
            }
            
            // current lane sensing
            if ( d < (2+4*lane+2) && d > (2+4*lane-2) )
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              // ???
              check_car_s += ((double)prev_size*.02*check_speed);
              
              // other vehicle exists in front of and near ego vehicle 
              if ( (check_car_s > car_s) && ((check_car_s - car_s) < 30 )) 
              {
                too_close = true;
                if (left_lane == true && lane >= 1)
                { 
                  lane = lane - 1;
                }
                else if (right_lane == true && lane <= 1)
                { 
                  lane = lane + 1;
                }
              }
            }
          }
          
          std::cout << "left_lane : " << left_lane << std::endl;
          std::cout << "right_lane : " << right_lane << std::endl;
          std::cout << "lane : " << lane << std::endl;
          
          if(too_close) // preceding vehicle exists
          {
            ref_vel -= .224; //deacc
          }
          else if(ref_vel < 49.5) // under target velocity
          {
            ref_vel += .224; //acc
          }
          
          std::cout << "ref_vel: " << ref_vel << std::endl;
          
          // points for spline 
          vector<double> ptsx;
          vector<double> ptsy;
          
          // current parameters
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // if there are few points
          if(prev_size <2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            // last previous path point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            // two from last previous path point
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            std::cout << "ref_yaw : " << ref_yaw << std::endl;
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // make next way point every 30m (and car_s means s value at end of previous path)
          vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // set end of previous path as origin points O(0,0) and 0 rad
          for (int i= 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = ( shift_x * cos(0-ref_yaw) ) - (shift_y * sin(0-ref_yaw) );
            std::cout << "stpx[" << i << "] : " << ptsx[i] << std::endl;
            ptsy[i] = ( shift_x * sin(0-ref_yaw) ) + ( shift_y * cos(0-ref_yaw) );
            std::cout << "stpy[" << i << "] : " << ptsy[i] << std::endl;
          }
          
          // make spline
          tk::spline s;
          s.set_points(ptsx,ptsy);
          
          // keep previous path
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // make points using the spline
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt( (target_x)*(target_x) +  (target_y)*(target_y) );
          
          double x_add_on = 0;
          
          for (int i = 0; i <= 50 -prev_size; i++)
          {
            double N = (target_dist / (.02*ref_vel/2.24)); // number of points between end of path and target_dist
            double x_point = x_add_on + (target_x)/N; // x_add_on plus one sample
            double y_point = s(x_point); // calcurate by the spline
            
            x_add_on = x_point; // update x_add_on
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // re-convert to global cordinate
            x_point = ( x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw) );
            y_point = ( x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw) );
            
            x_point += ref_x; 
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);            
          }
          
          std::cout << "next_x : " << int(next_x_vals[0]) << "," << int(next_x_vals[1]) << "," << int(next_x_vals[2]) << std::endl;
          std::cout << "next_y : " << int(next_y_vals[0]) << "," << int(next_y_vals[1]) << "," << int(next_y_vals[2]) <<std::endl;
          
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