#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

static const double MIN_TOLERANCE = 0.2;
static const int SAMPLE_SIZE = 2000;
static const double OUT_OF_ROAD_CTE = 3.0;
static const double MAX_SPEED = 100;
static const double MAX_ANGLE = 20.0;
static int sample_count = 0;
static int twiddle_try_count = 0;

static const double START_STEER_KP = 0.13;
static const double START_STEER_KI = 0.002;
static const double START_STEER_KD = 5.5;

static const double START_THROTTLE_KP = 0.1;
static const double START_THROTTLE_KI = 0.0;
static const double START_THROTTLE_KD = 2.0;

static const bool use_twiddle = false;// twiddle tunning use flag

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Resetting the Simulator
void resetSimulator(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main() {
  uWS::Hub h;
  
  /**
   * TODO: Initialize the pid variable.
   */
  PID steer_pid;
  PID throttle_pid;

  steer_pid.Init(START_STEER_KP,START_STEER_KI,START_STEER_KD);
  throttle_pid.Init(START_THROTTLE_KP,START_THROTTLE_KI,START_THROTTLE_KD);

  Twiddle steer_twiddle;
  steer_twiddle.Init(0.5,0.0,0.0);

  Twiddle throttle_twiddle;
  throttle_twiddle.Init(START_THROTTLE_KP,START_THROTTLE_KI,START_THROTTLE_KD);

  bool is_initialized = false;
  static bool achieved_tolerance = false;

  int skip_count = 0; // main loop counter for removing error feedback before connected.

  h.onMessage([&steer_pid,&throttle_pid,&steer_twiddle,&throttle_twiddle,&is_initialized,&skip_count](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());

          double steer_value,throttle_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */  

          skip_count++;
          
          if(skip_count > 3){
            steer_pid.UpdateError(cte);
            steer_value = steer_pid.TotalError(); 
            steer_pid.setAccumulateError(cte);         

            double target_speed = std::max(0.0, MAX_SPEED * ( 1.0 - fabs(angle/MAX_ANGLE*cte) / 4));
            target_speed = std::min(MAX_SPEED, target_speed);
            double err_speed = speed - target_speed;
            throttle_pid.UpdateError(err_speed);
            throttle_value = throttle_pid.TotalError();
            
            bool is_sample_period = (++sample_count % SAMPLE_SIZE == 0);        

            if(use_twiddle && (is_sample_period || fabs(cte) > OUT_OF_ROAD_CTE)) {                        
              double steer_avg_err = steer_pid.getAccumulateError() / sample_count;            
              
              if(fabs(cte) > OUT_OF_ROAD_CTE){ // assigne large error when out of road.
                steer_twiddle.setCurrentError(steer_avg_err+100.0);              
              }else{
                steer_twiddle.setCurrentError(steer_avg_err);              
              }

              std::vector<double> params = steer_twiddle.UpdateParams();              

              if(steer_twiddle.getTolerance() > MIN_TOLERANCE){
                steer_pid.Init(params[0], params[1], params[2]);              

                twiddle_try_count++;

                std::cout << "============================ new Steer PID set =============================" << std::endl;
                std::cout << "Twiddle try num : " << twiddle_try_count << std::endl;

                steer_pid.PrintPIDValue();
                std::cout << "Best err: " << steer_twiddle.getBestError() << "\tcurrent err: " << steer_twiddle.getCurrentError() << std::endl;
                
                std::cout << "==============================================================================" << std::endl << std::endl;             

                resetSimulator(ws);
                sample_count = 0;
                skip_count = 0;
              }          
            } 

            // DEBUG
            // std::cout << "CTE: " << cte << " | Steering: " << steer_value << " | Throttle: " << throttle_value << " | Angle: " << angle << " | target_speed: " << target_speed << std::endl;
          }// end of skip count

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);          

        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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