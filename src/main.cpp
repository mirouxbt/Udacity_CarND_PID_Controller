#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
//#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Factors for the PID 
double factor_steer_P    = 0.09;
double factor_steer_D    = 1.3;
double factor_th_P       = 15.0;
double factor_th_D       = 0.0;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char **argv)
{
  uWS::Hub h;



  if (argc == 5) {
    factor_steer_P  = atof(argv[1]);
    factor_steer_D  = atof(argv[2]);
    factor_th_P     = atof(argv[3]);
    factor_th_D     = atof(argv[4]);
  }


  PID pid;
  pid.Init(factor_steer_P, 0.0, factor_steer_D);
  pid.InitHyperParam(0.1,0.0,1.0);
  pid.SaveToFile("PID_steer_params");
  pid.NeedOptimization(false);
  PID pid_prev;
  pid_prev.Init(pid.get_Kp(),pid.get_Ki(), pid.get_Kd());

  PID pid_th;
  pid_th.Init(factor_th_P, 0.0, factor_th_D);
  pid_th.InitHyperParam(15.0,0.0,15.0);
  pid_th.SaveToFile("PID_throttle_params");
  pid_th.NeedOptimization(false);
  PID pid_th_prev;
  pid_th_prev.Init(pid_th.get_Kp(),pid_th.get_Ki(), pid_th.get_Kd());
    
  double prev_cte = 9999.0;
  
  h.onMessage([&pid, &pid_th, &pid_prev, &pid_th_prev, &prev_cte](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle;

          // Update steering PID
          pid.UpdateError(cte);
          
          // Update throttle PID
          // We use the derivative of the error only when
          // we move away from the center line
          if ( prev_cte == 9999.0 ) {
            prev_cte = cte;
          }
          double delta = cte - prev_cte;
          if (cte > 0) {
            if (delta < 0) { delta = 0; }
          }
          else {
            if (delta > 0) { delta = 0; }
          }
          pid_th.UpdateError(abs(delta));
          prev_cte = cte;
          
          
          // If optimization cycle complete, restart
          bool restart = pid.Optimize(cte, speed);
          if (pid.isOptimized()) {
            restart = pid_th.Optimize(abs(cte), speed) || restart;
          }
          if (restart) {
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            return;
          }
          
          
          // If both are optimized and we got a change on any
          // Restart a full optimization cycle
          if (pid.isOptimized() && pid_th.isOptimized()) {
            if (pid != pid_prev || pid_th != pid_th_prev) {
              pid_prev.Init(pid.get_Kp(),pid.get_Ki(), pid.get_Kd());
              pid_th_prev.Init(pid_th.get_Kp(),pid_th.get_Ki(), pid_th.get_Kd());
              
              pid.NeedOptimization(true);
              pid.InitHyperParam(0.1,0.0,1.0);
              
              pid_th.NeedOptimization(true);
              pid_th.InitHyperParam(5.0,0.0,10.0);
            }
          }
          
          // Steer control
          steer_value = -pid.TotalError();
          // Clip value in [-1, 1]
          if (steer_value > 1.0) {
            steer_value = 1.0;
          }
          else if (steer_value < -1.0) {
            steer_value = -1.0;
          }
          
          // Throttle control
          throttle = 1.0 - pid_th.TotalError();
          // Clip value in [-1, 1]
          if (throttle > 1.0) {
            throttle = 1.0;
          }
          else if (throttle < -1.0) {
            throttle = -1.0;
          }

          // DEBUG
          std::cout << "CTE: " << cte << " SV: " << steer_value << " TH: " << throttle << std::endl;
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
