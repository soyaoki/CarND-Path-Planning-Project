#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

using namespace std;

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
  ofstream log_file;
  log_file.open("path_planning_log.csv");

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&log_file,&max_s]
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
          
          // WAYPOINTS SMOOTHING
          // cout << "smoothing" << endl;
          int id_map_last = map_waypoints_x.size() - 1;
          int _close_way_point_id = ClosestWaypoint(car_x, car_y, map_waypoints_x, map_waypoints_y);

          int id_interp_start = _close_way_point_id - 5;
          int id_interp_end   = _close_way_point_id + 8;

          // double prev_s_offset = s_offset;
          // s_offset = map_waypoints_s[_close_way_point_id];

          // cout << "setting a range for interpolate ... " << endl;
          // cout << "smoothing 2" << endl;
          vector<double> map_x_to_interp, map_y_to_interp, map_s_to_interp;
          double _map_s, _map_x, _map_y;
          // cout << " [-] closest waypoint = " << _close_way_point_id << endl;
          // if (_close_way_point_id == 0) {minus_max_s += 1;}
          // if (id_interp_start == id_map_last - 3) {minus_max_s = 1;}
          for (int map_id=id_interp_start; map_id < id_interp_end; map_id ++) {
            if (map_id > id_map_last) {
              int _map_id = map_id - id_map_last - 1;
              // cout << " [***] map_id > id_map_last, " << _map_id << endl;
              _map_s = map_waypoints_s[_map_id] + max_s;
              _map_x = map_waypoints_x[_map_id];
              _map_y = map_waypoints_y[_map_id];
            }
            else if (map_id < 0) {
              int _map_id = id_map_last + map_id + 1;
              // cout << " [***] map_id < 0, " << _map_id << endl;
              _map_s = map_waypoints_s[_map_id] - max_s;
              _map_x = map_waypoints_x[_map_id];
              _map_y = map_waypoints_y[_map_id];

            }
            else {
              // cout << " [***] else, " << map_id << endl;
              _map_s = map_waypoints_s[map_id];
              _map_x = map_waypoints_x[map_id];
              _map_y = map_waypoints_y[map_id];
            }
            // _map_s -= s_offset;
            map_s_to_interp.push_back(_map_s);
            map_x_to_interp.push_back(_map_x);
            map_y_to_interp.push_back(_map_y);
            // cout << "(s,x,y) = " << _map_s << ", " << _map_x << ", " << _map_y << endl;
          }
          // cout << "set spline" << endl;
          tk::spline x_given_s;
          tk::spline y_given_s;
          x_given_s.set_points(map_s_to_interp, map_x_to_interp);
          y_given_s.set_points(map_s_to_interp, map_y_to_interp);

          // cout << "interpolating starts...." << endl;

          vector<double> map_ss, map_xs, map_ys;
          double _s = map_s_to_interp[0];

          // cout << "interp" << endl;

          while (_s < map_s_to_interp[map_s_to_interp.size()-1]) {
            double _x = x_given_s(_s);
            double _y = y_given_s(_s);
            map_ss.push_back(_s);
            map_xs.push_back(_x);
            map_ys.push_back(_y);
            _s += 0.05;
          }
          std::ofstream single_iteration_log;
          single_iteration_log.open("path_planning_log-single_iteration.csv");
          
          std::cout << "--------------------------------------" << std::endl;
          std::cout << "car_x : " << car_x << std::endl;
          std::cout << "car_y : " << car_y << std::endl;
          std::cout << "car_yaw : " << car_yaw << std::endl;
          std::cout << "car_s : " << car_s << std::endl;
          std::cout << "car_d : " << car_d << std::endl;
          std::cout << "car_speed  : " << car_speed  << std::endl;
          
          double desired_d = 2+4*lane; // target d depended lane number
          bool too_close = false; // acc or deacc flag
          int prev_size = previous_path_x.size(); // number of previous path existing
          std::cout << "prev_size : " << prev_size << std::endl;
          
          // if previous path exists
          if (prev_size > 0)
          {
            car_s = end_path_s;
            car_d = end_path_d;
          }
          
          // keep previous path
//          if(prev_size >20)
//          {
            for (int i = 0; i < prev_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
//          }
          
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
          
          if(too_close) // preceding var exists
          {
            ref_vel -= .224; //deacc
          }
          else if(ref_vel < 48) // under target velocity
          {
            ref_vel += .224; //acc
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
          vector<double> start_s{car_s, car_speed/2.237, 0};
          std::cout << "start s :" << start_s[0] << "," << start_s[1] << "," << start_s[2] << std::endl;
          
          vector<double> end_s{car_s+(T[path_points_num-1]*ref_vel/2.237), ref_vel/2.237, 0};
          std::cout << "end s :" << end_s[0] << "," << end_s[1] << "," << end_s[2] << std::endl;
          
          vector<double> start_d{car_d, 0, 0};
          std::cout << "start d :" << start_d[0] << "," << start_d[1] << "," << start_d[2] << std::endl;
          
          double ref_d;
          if (ref_vel <= 10) // mph
          {
            ref_d = car_d;
          }else
          {
            ref_d = 2+4*lane;
          }
          vector<double> end_d{ref_d, 0, 0};
          std::cout << "end d :" << end_d[0] << "," << end_d[1] << "," << end_d[2] << std::endl;
          
          vector<double> s_eq = JMT(start_s, end_s, T[path_points_num-1]);
          std::cout << "s_eq :" << s_eq[0] <<  "," << s_eq[1] << "," <<  s_eq[2] << "," <<  s_eq[4] << "," <<  s_eq[4] << "," <<  s_eq[5] << std::endl;
          vector<double> d_eq = JMT(start_d, end_d, T[path_points_num-1]);
          std::cout << "d_eq :" << d_eq[0] <<  "," <<  d_eq[1] << "," <<  d_eq[2] << "," <<  d_eq[4] << "," <<  d_eq[4] << "," <<  d_eq[5] << std::endl;
          
//          if (prev_size < 20)
//          {
          if (path_points_num - prev_size > 0)
          {
            int t_size = T.size();
            for (int jj = 0; jj < path_points_num-1; jj++)
//            for (int jj = 0; jj < path_points_num - prev_size; jj++)
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
          
          log_file << "waypoints" << endl << "x, y" << endl;
          single_iteration_log << "waypoints" << endl << "x, y" << endl;

          std::cout << "path of : "  << std::endl;
          for (int i =0; i < next_x_vals.size(); i++)
          {
            std::cout << "x : "  << next_x_vals[i] << ", y : "  << next_y_vals[i] << std::endl;
            
            log_file << next_x_vals[i] << ", " << next_y_vals[i] << endl;            
            
            single_iteration_log << next_x_vals[i] << ", " << next_y_vals[i] << endl;
          }
          log_file << endl;
          single_iteration_log.close();
          
          // ------------- End -------------
          
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
  log_file.close();
}