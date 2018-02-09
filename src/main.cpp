#include <uWS/uWS.h>
#include <algorithm>
#include "json.hpp"
#include "PID.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(const std::string &s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_last_of(']');
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  unsigned long long obs_cnt = 0; // counter of the processed observations

  PID pid_steer;

  // manually sarted with P=0.2 I=0.004 D=3.0 Throttle=0.3 passed
  // twiddle 0.2, 0.004, 3.0, 30, 500, 0.1, 0.01, 1.0, 0.1
  // twiddle 0.41, 0.02, 8.0, 30, 350, 0.1, 0.005, 1.0, 0.2
  // twiddle 0.609, 0.015, 10.0, 30, 350, 0.1, 0.005, 1.0, 0.2
  // double twiddle 0.61, 0.0155, 13.0, 30, 175, 0.1, 0.005, 1.0, 0.2; 2.0, 0.0, 3.0, 30, 150, 0.5, 0.005, 1.0, 0.1
  // double twiddle 0.3, 0.015, 13.0, 30, 50, 0.1, 0.005, 1.0, 0.2; 2.5, 0.0, 6.0, 30, 175, 0.5, 0.005, 1.0, 0.1
  // double twiddle 0.2, 0.0165, 7.0, 30, 0, 0.1, 0.005, 1.0, 0.2; 5.0, 0.0, 8.0, 30, 0, 0.5, 0.005, 1.0, 0.1
  pid_steer.Init(0.15, 0.0165, 5.0, 30, 0, 0.1, 0.005, 1.0, 0.2);

  PID pid_throttle;
  pid_throttle.Init(7.0, 0.0, 0.5, 30, 0, 0.5, 0.005, 1.0, 0.1);

  h.onMessage([&pid_steer, &pid_throttle, &obs_cnt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (!s.empty()) {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // feed new data to the steering angle PID controller
          pid_steer.UpdateError(cte);
          double steering_angle = pid_steer.Correction();

          // the steeper steering angle, the lesser throttle
          pid_throttle.UpdateError(std::max(fabs(steering_angle), fabs(cte)));
          double throttle       = 1.0 + pid_throttle.Correction();
          if (throttle > 1.0) {
            throttle = 1.0;
          } else if (throttle <= 0.0) {
            throttle = 0.3;
          }

          // observation processed
          ++obs_cnt;

          // debug message
          std::cout
              << "\n======[" << obs_cnt << "]======"
              << "\nCross Track Error:               " << cte
              << "\nCurrent Speed:                   " << speed
              << "\nCurrent Steering Angle:          " << angle
              << "\nPID Corrected Steering:          " << steering_angle
              << "\nPID Steering Coefficients:       " <<   "Kp = " << pid_steer.Kp
                                                       << ", Ki = " << pid_steer.Ki
                                                       << ", Kd = " << pid_steer.Kd
              << "\nPID Corrected Throttle:          " << throttle
              << "\nPID Throttle Coefficients:       " <<   "Kp = " << pid_throttle.Kp
                                                       << ", Ki = " << pid_throttle.Ki
                                                       << ", Kd = " << pid_throttle.Kd
              << "\n======[" << obs_cnt << "]======\n"
              << std::endl;

          json msgJson;

          // apply corrections
          msgJson["steering_angle"] = steering_angle;
          msgJson["throttle"]       = throttle;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
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
