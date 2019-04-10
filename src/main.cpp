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
#include <algorithm>
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
  int lane = 1;
  double ref_vel = 0;
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &max_s]
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
          int prev_size = previous_path_x.size();

          json msgJson;

          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          bool decrease_speed = false;
          bool front_close = false;
          bool left_close = false;
          bool right_close = false;
          float min_left_s  = max_s;
          float min_right_s = max_s;
          float min_front_s = max_s;
          for(int i=0; i<sensor_fusion.size(); ++i)
          {
            float d = sensor_fusion[i][6];
            if(d>0 && d<12)
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += prev_size*0.02*check_speed;
              int car_lane = (int)d/4;
              if(car_lane == lane)
              {
                if((check_car_s>car_s)&&(check_car_s-car_s<30)) front_close = true;
              }
              else if((check_car_s>car_s-10)&&(check_car_s-car_s<30))
              {
                if(car_lane == lane-1) left_close = true;
                else if(car_lane == lane+1) right_close = true;
              }
              if( (car_lane == lane-1) && (check_car_s>car_s) && min_left_s>check_car_s )  min_left_s=check_car_s;
              else if( (car_lane == lane+1) && (check_car_s>car_s) && min_right_s>check_car_s )  min_right_s=check_car_s;
              else if( (car_lane == lane) && (check_car_s>car_s) && min_front_s>check_car_s )  min_front_s=check_car_s;
            }
          }
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          if(front_close)
          {
            if(lane == 0) // only changing lane to right is possible
            {
              if(!right_close) lane = 1;
              else decrease_speed = true;
            }
            else if(lane == 2) // only changing lane to left is possible
            {
              if(!left_close) lane = 0;
              else decrease_speed = true;
            }
            else if(lane == 1) // changing lane to right or left are possible
            {
              if(!right_close && left_close) lane = 2; // only changing lane to right is possible
              else if(right_close && !left_close) lane = 0; // only changing lane to right is possible
              else if(!right_close && !left_close) //we need to choose one
              {
                if(min_right_s > min_left_s) lane = 2;
                else lane = 0;
              }
              else decrease_speed = true;
            }
          }
          else
          {
            if(lane == 0)
            {
              if(!right_close && min_right_s > min_front_s) lane = 1;
            }
            else if(lane == 2)
            {
              if(!left_close && min_left_s > min_front_s) lane = 1;
            }
            else if(lane == 1) 
            {
              if(!right_close && left_close && min_right_s > min_front_s) lane = 2; 
              else if(right_close && !left_close && min_left_s > min_front_s) lane = 0;
              else if(!right_close && !left_close)
              {
                if(min_right_s > min_left_s && min_right_s > min_front_s) lane = 2;
                else if(min_left_s > min_right_s && min_left_s > min_front_s) lane = 0;
              }
            }
          }
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            //std::cout<<"abbas  "<<prev_car_x<<"  "<<car_x<<"  "<<car_yaw<<std::endl;
            double prev_car_y = car_y - sin(car_yaw);
            next_x_vals.push_back(prev_car_x);
            next_x_vals.push_back(car_x);
            next_y_vals.push_back(prev_car_y);
            next_y_vals.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            next_x_vals.push_back(ref_x_prev);
            next_x_vals.push_back(ref_x);
            next_y_vals.push_back(ref_y_prev);
            next_y_vals.push_back(ref_y);
          }
          
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          next_x_vals.push_back(next_wp0[0]);
          next_x_vals.push_back(next_wp1[0]);
          next_x_vals.push_back(next_wp2[0]);
          
          next_y_vals.push_back(next_wp0[1]);
          next_y_vals.push_back(next_wp1[1]);
          next_y_vals.push_back(next_wp2[1]);
          
          for(int i=0; i<next_x_vals.size(); ++i)
          {
            double shift_x = next_x_vals[i] - ref_x;
            double shift_y = next_y_vals[i] - ref_y;
            
            next_x_vals[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            next_y_vals[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
          }
          /*
          for(int i=0; i<next_x_vals.size(); ++i)
          {
            std::cout<<next_x_vals[i]<<"  "<<next_y_vals[i]<<std::endl;
          }
          std::cout<<std::endl<<std::endl<<std::endl;
          */
          tk::spline s;
          for (int i = 1; i < next_x_vals.size(); ++i) {
            bool swapped = false;
            for (int j = 0; j < next_x_vals.size() - i; j++) {
              if (next_x_vals[j] > next_x_vals[j + 1]) {
                double temp = next_x_vals[j];
                next_x_vals[j] = next_x_vals[j + 1];
                next_x_vals[j + 1] = temp;
                temp = next_y_vals[j];
                next_y_vals[j] = next_y_vals[j + 1];
                next_y_vals[j + 1] = temp;
                swapped = true;
              }
            }
            if (!swapped) break;
          }     
          s.set_points(next_x_vals, next_y_vals);
          
          next_x_vals.clear();
          next_y_vals.clear();
          for(int i=0; i<prev_size; ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0;

          for (int i = 1; i <= 50-prev_size; ++i) {
            if(decrease_speed) ref_vel -= .224;
            else if(ref_vel < 49.5) ref_vel += .224;
            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw) + ref_x;
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw) + ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
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