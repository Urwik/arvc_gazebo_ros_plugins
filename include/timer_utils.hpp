#pragma once

#include <iostream>
#include <chrono>

namespace arvc{
  class Timer
  {
  private:
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point stop;
    double duration;

  public:
    // Default constructor
    Timer(/* args */){
      duration = 0;
    }


    // Start the timer
    void start_timer(){
      start = std::chrono::high_resolution_clock::now();
    }


    // Stop the timer
    void stop_timer(){
      stop = std::chrono::high_resolution_clock::now();
    }


    /**
     * @brief Get the duration of the timer.
     * @return The duration of the timer.
    */
    int get_duration(){
      duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
      return duration;}

    /**
     * @brief Get the duration of the timer.
     * @return The duration of the timer.
    */
    int get_current_duration(){
      auto current_time = std::chrono::high_resolution_clock::now();
      auto current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start).count();
      return current_duration;
      }

    /**
     * @brief Print the duration of the timer.
    */
    void print_duration(const std::string& _message = ""){
      std::cout << _message << " " << duration << " ms" << std::endl;
    }


    /**
     * @brief Print the current duration of the timer.
    */
    void print_current_duration(const std::string& _message = ""){
      auto current_time = std::chrono::high_resolution_clock::now();
      auto current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start).count();
      std::cout << _message << " " << current_duration << " ms" << std::endl;
    }

  };
} // namespace arvc