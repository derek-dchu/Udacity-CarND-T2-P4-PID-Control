#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"
#include <math.h>

#define PID_TOL 0.000001
#define N_STEP 20
#define IS_STEER_PID_TUNED true 
#define IS_SPEED_PID_TUNED true
#define MIN_SPEED 15.0
#define SPEED_RANGE 10

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  /**
   * steer_pid is tuned in 14701 steps. Elapsed time: 356.562s
   * Finalized params: 0.437229 -9.99989e-05 0.795459
   */
  Twiddle twiddle;
  if (IS_STEER_PID_TUNED) {
    pid.Init(0.437229, -9.99989e-05, 0.795459);
    twiddle.is_tuned = true;
  } else {
    pid.Init(0,0,0);
    std::vector<double> s_dp {1, 1, .01};
    twiddle.init("steer_pid", pid, PID_TOL, s_dp, N_STEP);
  }

  /**
   * speed_pid is tuned in 12061 steps. Elapsed time: 291.587s
   * Finalized params: 0.213868 0.000957822 0.138551
   */
  PID v_pid;
  Twiddle v_twiddle;
  if (IS_SPEED_PID_TUNED) {
    v_pid.Init(0.213868, 0.000957822, 0.138551);
    v_twiddle.is_tuned = true;
  } else {
    v_pid.Init(0,0,0);
    std::vector<double> v_dp {.1, .1, .001};
    v_twiddle.init("speed_pid", v_pid, PID_TOL, v_dp, N_STEP);
  }

  h.onMessage([&pid, &twiddle, &v_pid, &v_twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          if (!twiddle.is_tuned) {
            pid = twiddle.tune(cte);
          } else {
            pid.UpdateError(cte);
          }

          steer_value = -pid.TotalError();

          if (steer_value > 1) steer_value = 1;
          if (steer_value < -1) steer_value = -1;

          double v_cte = speed - (MIN_SPEED + (1-fabs(steer_value))*SPEED_RANGE);
          if (!v_twiddle.is_tuned) {
            v_pid = v_twiddle.tune(v_cte);
          } else {
            v_pid.UpdateError(v_cte);
          }

          double throttle = -v_pid.TotalError();
          
          if (throttle > 1) throttle = 1;
          if (throttle < -1) throttle = -1;
          throttle *= 1-fabs(steer_value);

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "Speed CTE: " << v_cte << " Throttle Value: " << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
