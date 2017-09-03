#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"
#include "Poly.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  double latency = 0.1;

  h.onMessage([&mpc,&latency](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];

          v *= 0.447; // convert to m/s
          delta *= -1; // flip steering angle for solver

          // Transform waypoints to car coordinates
          Eigen::VectorXd wypts_x(ptsx.size());
          Eigen::VectorXd wypts_y(ptsx.size());
          for (unsigned int i=0; i < ptsx.size(); i++) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            wypts_x[i] =  x*cos(psi) + y*sin(psi);
            wypts_y[i] = -x*sin(psi) + y*cos(psi);
          }

          // Fit with 3rd order polynomial
          auto coeffs = polyfit(wypts_x, wypts_y, 3);

          // Current errors
          double cte = coeffs[0]; // since we are at the origin
          double epsi = -atan(coeffs[1]); // x=0 (since we are at origin), minus sign because we want angle relative to truth, not to truth relative to car.
          // std::cout << epsi << "\t";

          // Initialize state vector
          Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
          state[3] = v;
          state[4] = cte;
          state[5] = epsi;

          // Solve system
          auto mpc_sol = mpc.Solve(state, coeffs, latency);
          int N = (mpc_sol.size() - 2)/2; // number of solver steps

          // Extract the actuator commands
          double steer_value = mpc_sol[0] / (deg2rad(25)); // range [-1,1]
          double throttle_value = mpc_sol[1];

          /* Create commands to send to simulator */
          json msgJson;

          msgJson["steering_angle"] = -steer_value; // flip steering angle back for simulator
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory : green
          vector<double> mpc_x_vals(N);
          vector<double> mpc_y_vals(N);

          for (int i=0; i<N; i++ ) {
            mpc_x_vals[i] = mpc_sol[2*(i+1)];
            mpc_y_vals[i] = mpc_sol[2*(i+1)+1];
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line : yellow
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (unsigned int i=0; i < ptsx.size(); i++) {
            next_x_vals.push_back(wypts_x[i]);
            next_y_vals.push_back(wypts_y[i]);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // Model latency to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(100));

          // send commands
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
