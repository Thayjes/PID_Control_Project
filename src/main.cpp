#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;
int num_iterations = 0;
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
  
  PID pid_s;
  PID pid_t; // PID controller for throttle
  // TODO: Initialize the pid variable.
    std::cout << "Initializing Parameters" << std::endl;
    /* First Twiddle Attempt
    double Kp_s = 1.19832;
    double Ki_s = 0.0;
    double Kd_s = 4.46997;
     */
    // On run with Twiddle, with throttle fixed at 0.5 and steps = 2000
    // Completes the lap, with throttle = 0.5.
    // On second run with Twiddle, steps = 4000.
    double Kp_s = 0.174381;
    double Ki_s = 0.0002786;
    double Kd_s = 4.46262;
    pid_s.Init(Kp_s, Ki_s, Kd_s);
    // P,I and D for throttle control
    double Kp_t = 0.3;
    double Ki_t = 0.0;
    double Kd_t = 0.01;
    //
    //After twiddle
    Kp_t = 0.381059;
    Ki_t = 0.0;
    Kd_t = 0.022569;
    pid_t.Init(Kp_t, Ki_t, Kd_t);
  h.onMessage([&pid_s, &pid_t](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
      // TODO: Initialize the pid variable.
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
            pid_s.UpdateError(cte);
            steer_value = pid_s.TotalError();
            
            double throttle_value;
            pid_t.UpdateError(cte);
            throttle_value = 0.5 + pid_t.TotalError();
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
            // Twiddle Both
            /*
            if(pid_both.step % (pid_both.num_steps_run + pid_both.num_steps_error) == 0){
                //std::cout << pid_s.num_steps_run + pid_s.num_steps_error << " have passed!" << std::endl;
                std::cout << "Time to run twiddle (BOTH) to update our P, I and D parameters!" << std::endl;
                pid_both.Twiddle(0.2);
                //std::cout << "Setting Step Back to 1" << std::endl;
                pid_both.step = 1;
                //std::cout << "Incrementing num of twiddle iterations" << std::endl;
                ++pid_both.num_iterations;
                // Set all errors back to zero for the new run
                pid_both.p_error = 0.0;
                pid_both.d_error = 0.0;
                pid_both.i_error = 0.0;
                // Reset car back to start of track
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                
            }
            // If we have run a certain number of steps
            // Run twiddle
            // Update the parameter and reset the car back to the start
            // Is this the correct method?
            // Or do we update and continue to drive the car around the track??
            
            if(pid_s.step % (pid_s.num_steps_run + pid_s.num_steps_error) == 0){
                //std::cout << pid_s.num_steps_run + pid_s.num_steps_error << " have passed!" << std::endl;
                std::cout << "Time to run twiddle (STEER) to update our P, I and D parameters!" << std::endl;
                pid_s.Twiddle(0.2);
                //std::cout << "Setting Step Back to 1" << std::endl;
                pid_s.step = 1;
                //std::cout << "Incrementing num of twiddle iterations" << std::endl;
                ++pid_s.num_iterations;
                pid_s.p_error = 0.0;
                pid_s.d_error = 0.0;
                pid_s.i_error = 0.0;
                //std::string reset_msg = "42[\"reset\",{}]";
                //ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                
            }
            
            // Twiddle Throttle Value;
            
            if(pid_t.step % (pid_t.num_steps_run + pid_t.num_steps_error) == 0){
                std::cout << pid_t.num_steps_run + pid_t.num_steps_error << " have passed!" << std::endl;
                std::cout << "Time to run twiddle (THROTTLE) to update our P, I and D parameters!" << std::endl;
                pid_t.Twiddle(0.2);
                //std::cout << "Setting Step Back to 1" << std::endl;
                pid_t.step = 1;
                //std::cout << "Incrementing num of twiddle iterations" << std::endl;
                ++pid_t.num_iterations;
                pid_t.p_error = 0.0;
                pid_t.d_error = 0.0;
                pid_t.i_error = 0.0;
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                
            }
             //*/
            ++pid_s.step;
            ++pid_t.step;
            //++pid_both.step;
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
