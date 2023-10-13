#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
    double* new_array = new double[new_size];
    for(int i = 0; i < length; i++){
        new_array[i] = array[i];
    }
    for(int i = length; i < new_size; i++){
        new_array[i] = 0;
    }
    delete[] array;
    return new_array;
}

double* shrink_array(double* array, int length, int new_size) {
    double* new_array = new double[new_size];
    for(int i = 0; i < new_size; i++){
        new_array[i] = array[i];
    }
    delete[] array;
    return new_array;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
    if(current_size == max_size){
        array = extend_array(array, max_size, max_size+5);
        max_size += 5;
    }
    array[current_size] = element;
    current_size++;
    return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    if(current_size <= max_size-4){
        array = shrink_array(array, max_size, max_size-5);
        max_size -= 5;
    }
    current_size--;
    return array;
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {


  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
    }
  }

  return hit_target;
}

void print_array(double* array, int length) {
  std::cout << "[";
  for(int i = 0; i < length; ++i) {
    if (i != 0) std::cout << ", ";
    std::cout << array[i];
  }
  std::cout << "]";
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
  int* telemetries_status = new int[tot_telemetries];
  for(int i = 0; i < tot_telemetries; i++){
    telemetries_status[i] = 0;
  }
  int telemetries_done = 0;
  while(telemetries_done < tot_telemetries){
      double min_t = -1;
      double min_x = -1;
      double min_y = -1;
      int telemetry_index = 0;
      telemetries_done = 0;
      for(int i = 0; i < tot_telemetries; i++){
        double *telemetry = telemetries[i];
        if(3*telemetries_status[i] < telemetries_sizes[i]){
            if(min_t == -1){
                min_t = telemetry[3*telemetries_status[i]];
                min_x = telemetry[3*telemetries_status[i]+1];
                min_y = telemetry[3*telemetries_status[i]+2];
                telemetry_index = i;
            }
            if(min_t > telemetry[3*telemetries_status[i]]){
              min_t = telemetry[3*telemetries_status[i]];
              min_x = telemetry[3*telemetries_status[i]+1];
              min_y = telemetry[3*telemetries_status[i]+2];
              telemetry_index = i;
          }
        } else {
          telemetries_done++;
        }
      }
      if(telemetries_done >= tot_telemetries){
        break;
      }
      telemetries_status[telemetry_index]++;
      global_telemetry = append_to_array(min_t, global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
      global_telemetry = append_to_array(min_x, global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
      global_telemetry = append_to_array(min_y, global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
    }
  delete[] telemetries_status;

}

