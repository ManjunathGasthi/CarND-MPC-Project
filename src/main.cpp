#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  double past_steer=0.0;
  double past_acc=0.0;

  h.onMessage([&mpc,&past_steer,&past_acc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          //READ TELEMETRY
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          past_steer = j[1]["steering_angle"];
          past_steer = - past_steer;
          std::cout << "Recovered steering angle: " << past_steer << std::endl;

          past_acc = j[1]["throttle"];

          v = v * 1.61 / 3.6;

          //APPLY LATENCY
          double latency = 0.1;
          double Lf = 2.67;
          double latency_x = px + v * cos(psi) * latency;
          double latency_y = py + v * sin(psi) * latency;
          //It is not necessary to divide by Lf because the value is returned
          //in radians, i.e. already divided by Lf.
          double latency_psi = psi + v / Lf * past_steer * latency;
          double latency_v = v + past_acc * latency;
          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
          px = latency_x;
          py = latency_y;
          psi = latency_psi;
          v = latency_v;

          //Project enough waypoints.
          vector<double> xvals;
          vector<double> yvals;
          std::cout << "Numero de puntos: " << ptsx.size() << std::endl;
          for(int i=0; i<ptsx.size(); i++){
            double dx = ptsx[i]-px;
            double dy = ptsy[i]-py;
            xvals.push_back(dy * sin(psi) + dx * cos(psi));
            yvals.push_back(dy * cos(psi) - dx * sin(psi));
          }
          Eigen::VectorXd xv = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(xvals.data(), xvals.size());
          Eigen::VectorXd yv = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yvals.data(), yvals.size());
          int degree = 3;
          if(xvals.size()<4) std::cout << "ERROR: TOO FEW WAYPOINTS" << std::endl;
/*          if(xvals.size()>=4){
            degree=3;
          }else{
            degree=xvals.size()-1;
          }*/
          auto coeffs = polyfit(xv,yv,degree);

          double cte = polyeval(coeffs, 0);
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi =  - atan(coeffs[1]) ;


          Eigen::VectorXd state(8);
          state << 0.0, 0.0, 0.0, v, cte, epsi, past_steer, past_acc;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          Eigen::VectorXd weights(8);
          weights << 500.0, 80000.0, 3.5, 80.0 * 1.61 / 3.6, 20.0, 1.0, 200.0, 1.0;
//Works 45mph          weights << 50.0, 2000.0, 1.0, 45.0 * 1.61 / 3.6, 20.0, 1.0, 20.0, 1.0;
//Works 50mph          weights << 50.0, 2000.0, 1.0, 50.0 * 1.61 / 3.6, 20.0, 1.0, 20.0, 1.0;
//Works 55mph          weights << 50.0, 2000.0, 2.0, 55.0 * 1.61 / 3.6, 10.0, 1.0, 100.0, 1.0;
//Works 60mph          weights << 25.0, 2000.0, 2.0, 60.0 * 1.61 / 3.6, 10.0, 1.0, 200.0, 1.0;
//Works 65mph          weights << 60.0, 4500.0, 2.0, 65.0 * 1.61 / 3.6, 20.0, 1.0, 200.0, 1.0;
//Works 70mph          weights << 120.0, 12000.0, 2.5, 70.0 * 1.61 / 3.6, 20.0, 1.0, 200.0, 1.0;
//Works 75mph          weights << 150.0, 22000.0, 2.5, 75.0 * 1.61 / 3.6, 20.0, 1.0, 200.0, 1.0;

          auto vars = mpc.Solve(state, coeffs, weights);

/*
          x_vals.push_back(vars[0]);
          y_vals.push_back(vars[1]);
          psi_vals.push_back(vars[2]);
          v_vals.push_back(vars[3]);
          cte_vals.push_back(vars[4]);
          epsi_vals.push_back(vars[5]);

          delta_vals.push_back(vars[6]);
          a_vals.push_back(vars[7]);*/
/*
          std::cout << "x = " << vars[0] << std::endl;
          std::cout << "y = " << vars[1] << std::endl;
          std::cout << "psi = " << vars[2] << std::endl;
          std::cout << "v = " << vars[3] << std::endl;
          std::cout << "cte = " << vars[4] << std::endl;
          std::cout << "epsi = " << vars[5] << std::endl;
          std::cout << "delta = " << vars[6] << std::endl;
          std::cout << "a = " << vars[7] << std::endl;
          std::cout << std::endl;
  */
          double steer_value;
          double throttle_value;

          std::cout << "Sent steering angle: " << vars[6] << std::endl;

          steer_value = - vars[6] / deg2rad(25);

          throttle_value = vars[7];

          if(fabs(throttle_value)>1) throttle_value /= fabs(throttle_value);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
/*
          for(int i = 0; i < xvals.size(); i++){
            mpc_x_vals.push_back(xvals[i]);
            mpc_y_vals.push_back(polyeval(coeffs,xvals[i]));
          }*/
          size_t N = (vars.size()-8)/2;
          for(int i=0; i<N; i++){
            mpc_x_vals.push_back(vars[8+i]);
            mpc_y_vals.push_back(vars[8+N+i]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /*px += vars[0] * cos (psi) - vars[1] * sin (psi);
          py += vars[1] * cos (psi) + vars[0] * sin (psi);
          psi += vars[2];
          xvals.clear();
          yvals.clear();
          for(int i=0; i<ptsx.size(); i++){
            double dx = ptsx[i]-px;
            double dy = ptsy[i]-py;
            xvals.push_back(dy * sin(psi) + dx * cos(psi));
            yvals.push_back(dy * cos(psi) - dx * sin(psi));
          }*/

          for(int i = 0; i < xvals.size(); i++){
            next_x_vals.push_back(xvals[i]);
            next_y_vals.push_back(yvals[i]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
