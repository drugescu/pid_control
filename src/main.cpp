#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

int main() {
  uWS::Hub h;

  PID pidSt, pidSp;
  /**
   * TODO: Initialize the pid variable.
   */
  /*double tau_p = 0.200;
  double tau_i = 0.004;
  double tau_d = 3.000;*/

  double tau_p = 0.070;
  double tau_i = 0.0001;
  double tau_d = 0.2;
  
  int counter = 0;

  pidSt.Init(tau_p, tau_i, tau_d);

  // PID for Speed
  double tau_pS = 0.15;
  double tau_iS = 0.00;
  double tau_dS = 0.00;

  pidSp.Init(tau_pS, tau_iS, tau_dS);
  //pidSp.SetDesired(0.5f);

  h.onMessage([&pidSt, &pidSp, &tau_p, &tau_i, &tau_d, &counter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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

          double steer_value = 0.0f;
          double throttle_value = 0.5f;
          
          counter++;
          if (counter > 150) {

            // Adaptive PID - https://drum.lib.umd.edu/bitstream/handle/1903/5044/MS_90-10.pdf?sequence=1&isAllowed=y

            double fe = fabs(cte);
            double fep = fe/100.0;
            if (fe <= 0.7) // <0.7-0.75, <1.15-1.45, <1.8-1.95, <2.2-2.35, <2.9-2.6, 2.6
              tau_p = 0.020 + fep * 0.75;
            if (fe > 0.7 && fe < 1.15) // 1.8 cte with 1.55 and then 3.30
              tau_p = 0.020 + fep * 1.45;
            else if (fe > 1.15 && fe < 1.8) // 1.8 cte with 1.55 and then 3.30
              tau_p = 0.020 + fep * 1.95;
            else
              tau_p = 0.020 + fep * 2.35;
             
            //tau_p = 0.125;
            if (tau_p > 0.16)  tau_p = 0.16;
            if (tau_p < 0.020) tau_p = 0.020;
            //std::cout << "tau_p = " << tau_p << "\n";
            
            tau_i = 0.0f;
            
            //tau_d = 1.200 + (fabs(cte)/100.0) * 45.0; 
            tau_d = 1.200 + (fabs(cte)/100.0) * (20.0 + (fabs(speed) - 0) * 0.5f); 
            //tau_d = 1.2;
            
            if (tau_d < 0.2) tau_d = 0.2;
            if (tau_d > 5.0) tau_d = 5.0;

            //std::cout << "tau_d = " << tau_d << "\n";
            pidSt.UpdateTaus(tau_p, tau_i, tau_d);
            
          }
          
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          // PID-controllers
          pidSp.UpdateError(fabs(cte));
          steer_value -= pidSt.TotalError();
          throttle_value = 0.45f - pidSp.TotalError(); // 0.55f - totalerror is best
          //throttle_value = 0.80f;
          
          if (throttle_value > 1.0f) throttle_value = 1.0f;
          if (throttle_value < 0.0f) throttle_value = 0.0f;
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
          //          << std::endl;
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
