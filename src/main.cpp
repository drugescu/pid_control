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
  //double tau_pS = 0.3;//0.05;
  //double tau_iS = 0.00;//0.006;
  //double tau_dS = 0.00;
  double tau_pS = 0.1;//0.05;
  double tau_iS = 0.00;//0.006;
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

             
            if (fabs(cte) <= 0.7) // 1.8 cte with 1.55 and then 3.30
              tau_p = 0.020 + (fabs(cte)/100.0) * 0.7;
            if (fabs(cte) > 0.8 && fabs(cte) < 1.15) // 1.8 cte with 1.55 and then 3.30
              tau_p = 0.020 + (fabs(cte)/100.0) * 1.35;
            else if (fabs(cte) > 1.1 && fabs(cte) < 1.8) // 1.8 cte with 1.55 and then 3.30
              tau_p = 0.020 + (fabs(cte)/100.0) * 1.95;
            else if (fabs(cte) > 1.8 && fabs(cte) < 2.2) // 1.8 cte with 1.55 and then 3.30
              tau_p = 0.020 + (fabs(cte)/100.0) * 2.95;
            else if (fabs(cte) > 2.2 && fabs(cte) < 2.9) // 1.8 cte with 1.55 and then 3.30
              tau_p = 0.020 + (fabs(cte)/100.0) * 3.40;
            else
              tau_p = 0.020 + (fabs(cte)/100.0) * 4.15;
             
            //tau_p = 0.125;
            if (tau_p > 0.16)
              tau_p = 0.16;
             if (tau_p < 0.020)
              tau_p = 0.020;
            std::cout << "tau_p = " << tau_p << "\n";
            
            tau_i = 0.0f;
            
            //tau_d = 1.800; // pid on this too from 1.9 to 2.5?
            //tau_d = 2.000 + (fabs(cte)/100.0) * 30.0; // pid on this too from 1.9 to 2.5?
            //tau_d = 1.200 + (fabs(cte)/100.0) * 45.0; // pid on this too from 1.9 to 2.5? at pid throttle
            tau_d = 1.200 + (fabs(cte)/100.0) * (20.0 + (fabs(speed) - 0) * 0.5f); // at pid throttle 0.85 - pidSp at 0.2p // 0.55
            //tau_d = 1.2;
            if (tau_d < 0.2) tau_d = 0.2;
            if (tau_d > 5.0) tau_d = 5.0;

            std::cout << "tau_d = " << tau_d << "\n";
            pidSt.UpdateTaus(tau_p, tau_i, tau_d);
            std::cout << "----------\n";
            
          }
          
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          // PID-controllers
          pidSt.UpdateError(cte);
          //pidSp.UpdateError(cte_speed + cte);
          //pidSp.UpdateError(cte);
          pidSp.UpdateError(fabs(cte));
          steer_value -= pidSt.TotalError();
          throttle_value = 0.70f - pidSp.TotalError(); // 0.55f - totalerror is best
          //throttle_value = 0.80f;
          
          if (throttle_value > 1.0f) throttle_value = 1.0f;
          if (throttle_value < 0.0f) throttle_value = 0.0f;
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;
          
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
