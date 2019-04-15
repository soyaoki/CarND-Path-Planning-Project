#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
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
          std::cout << "car_speed  : " << car_speed  << std::endl;
          
          
          int desired_d = 2+4*lane; // target d depended lane number
          bool too_close = false; // acc or deacc flag
          int prev_size = previous_path_x.size(); // number of previous path existing
          std::cout << "prev_size : " << prev_size << std::endl;
          
          // if previous path exists
          //if (prev_size > 0)
          //{
            //car_s = end_path_s;
          //}
          
          // keep previous path
          if(prev_size >20){
            for (int i = 0; i < prev_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          }
          
          // sensing aroud
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            if ( d < (2+4*lane+2) && d < (2+4*lane-2) )
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s += ((double)prev_size*.02*check_speed);
              
              if ( (check_car_s > car_s) && ((check_car_s - car_s) < 30 )) 
              {
                too_close = true;
                lane = 0;
              }
            }
          }
          
          if(too_close) // preceding var exists
          {
            ref_vel -= .112; //deacc
          }
          else if(ref_vel < 49.5) // under target velocity
          {
            ref_vel += .112; //acc
          }
            
          std::cout << "ref_vel: " << ref_vel << std::endl;
          
          // build time vector
          vector<double> T;
          int path_points_num = 50;
          for ( int ind = 1; ind < path_points_num+1; ind ++)
          {
            T.push_back(ind*.02); // ego vehicle moves every 0.02
          }
          
          // calculate polynominal to minimize jark
          vector<double> start_s{car_s, car_speed, 0};
          std::cout << "start s :" << start_s[0] << "," << start_s[1] << "," << start_s[2] << std::endl;
          vector<double> end_s{(car_s+T[path_points_num-1]*ref_vel), ref_vel, 0};
          std::cout << "end s :" << end_s[0] << "," << end_s[1] << "," << end_s[2] << std::endl;
          vector<double> start_d{car_d, 0, 0};
          std::cout << "start d :" << start_d[0] << "," << start_d[1] << "," << start_d[2] << std::endl;
          vector<double> end_d{desired_d, 0, 0};
          std::cout << "end d :" << end_d[0] << "," << end_d[1] << "," << end_d[2] << std::endl;
          
          vector<double> s_eq = JMT(start_s, end_s, T[path_points_num-1]);
          std::cout << "s_eq :" << s_eq[0] <<  "," << s_eq[1] << "," <<  s_eq[2] << "," <<  s_eq[4] << "," <<  s_eq[4] << "," <<  s_eq[5] << std::endl;
          vector<double> d_eq = JMT(start_d, end_d, T[path_points_num-1]);
          std::cout << "d_eq :" << d_eq[0] <<  "," <<  d_eq[1] << "," <<  d_eq[2] << "," <<  d_eq[4] << "," <<  d_eq[4] << "," <<  d_eq[5] << std::endl;
          
          if (prev_size < 15){
            
            int t_size = T.size();
            for (int jj = 0; jj < path_points_num-1; jj++)
            {
              double tt = T[jj];
              std::cout << "T :" << tt << std::endl;            
              double s = cul_polynomial(s_eq,tt);
              double d = cul_polynomial(d_eq,tt);
              std::cout << "s :" << s << ", d :"  << d << std::endl;            
              vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(xy[0]);
              std::cout << "x added to path: " << xy[0] << std::endl;
              next_y_vals.push_back(xy[1]);
              std::cout << "y added to path: " << xy[1] << std::endl;
            }
          }
          
          std::cout << "path of : "  << std::endl;
          for (int i =0; i < next_x_vals.size(); i++){
            std::cout << "x : "  << next_x_vals[i] << ", y : "  << next_y_vals[i] << std::endl;
          }
          
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