#include "PID.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

using namespace std;

PID::PID() {
  go_pos = true;
  is_optimized = true;
  best_speed = 10.0;
  current_param = 0;
  nCTE = 0;
  avgSpeed = 0.0;
  nCTE_steps = 2500; // At least 2 laps
  prev_cte = 99999999999999.0;
  nImprovement = -1; // The 1st improvement is not real, it is just the baseline
  
  dp[0] = 1.0;
  dp[1] = 1.0;
  dp[2] = 1.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
}

void PID::InitHyperParam(double dKp, double dKi, double dKd) {
  dp[0] = dKp;
  dp[1] = dKi;
  dp[2] = dKd;
}

void PID::NeedOptimization(bool need) {
  is_optimized = !need;
}


void PID::SaveToFile(const string& filename) {
  file_stream.open(filename, ios::out);
}

void PID::UpdateError(double cte) {
  
  // Proportional error
  error = p[0] * cte;
  
  // Integral error clip sum at MAX_CTE_SUM
  cte_list.push_back(cte);
  if (cte_list.size() > MAX_CTE_TO_KEEP) { cte_list.pop_front(); }
  
  double cte_sum = 0.0;
  for (auto n : cte_list) cte_sum += n;
  if ( cte_sum >  MAX_CTE_SUM ) { cte_sum =  MAX_CTE_SUM; }
  if ( cte_sum < -MAX_CTE_SUM ) { cte_sum = -MAX_CTE_SUM; }

  error += p[1] * cte_sum;
  
  // Derivative error
  if (prev_cte == 99999999999999.0) {
    prev_cte = cte;
  }
  
  double delta = cte - prev_cte;
  prev_cte = cte;
  
  error += p[2] * delta;
}

double PID::TotalError() {
  return error;
}

bool PID::Optimize(double cte, double speed) {
  if (is_optimized) { return false; }
  
  nCTE++;

  cout << "CP: " << current_param << endl;
  cout << "p:"   << p[0]   << ',' << p[1]  << ','<< p[2]   << endl;
  cout << "dp:"  << dp[0]  << ',' << dp[1] << ','<< dp[2]  << endl;

  if (nCTE > 100) {
    avgSpeed += speed;
    
    cout << "nCTE: " << nCTE << "/" << nCTE_steps << " aSpeed: "  << avgSpeed/nCTE << " bSpeed: " << best_speed << endl;
    
    // If no chance to get better, stop here
    if ( (avgSpeed + (nCTE_steps - nCTE) * 100.0) / nCTE_steps < best_speed ) {
      nCTE = nCTE_steps;
    }

    // If off track, stop here
    if (abs(cte) > 3.7) {
      nCTE = nCTE_steps;
      avgSpeed = 0.0;
    }

    // If at the target step for evaluation
    if (nCTE == nCTE_steps ) {
      avgSpeed /= nCTE;
      
      // If better
      if (avgSpeed > best_speed) {
        best_speed = avgSpeed;
        go_pos = true;
        nImprovement++;
        
        dp[current_param] *= 1.1;
        
        if (file_stream.is_open()) {
          file_stream << "---- BETTER ----" << endl;
          file_stream << "CP: " << current_param << endl;
          file_stream << "p:"   << setprecision(numeric_limits<double>::max_digits10) << p[0]   << ',' << p[1]  << ','<< p[2]   << endl;
          file_stream << "dp:"  << dp[0]  << ',' << dp[1] << ','<< dp[2]  << endl;
          file_stream << "nCTE: " << nCTE << " aSpeed: "  << avgSpeed/nCTE << " bSpeed: " << best_speed << endl;
          file_stream << "----------------" << endl << endl;
        }
        
        // Go to next param, skip integral parameter
        current_param += 2;
        current_param %= 4;

        p[current_param] += dp[current_param];
      }
      else {
        // If worse and tried to add dp, now try to substract
        if (go_pos) {
          go_pos = false;
          p[current_param] -= 2*dp[current_param];
        }
        else { // if none worked, reduce dp
          go_pos = true;

          p[current_param]  += dp[current_param];
          dp[current_param] *= 0.9;
          
          // Go to next param, skip integral parameter
          current_param += 2;
          current_param %= 4;
          
          // If below tolerance
          if (dp[0] + dp[2] < PARAM_TOL) {
            // If this cycle didn't give us improvement, we are optimized
            if (nImprovement == 0) {
              is_optimized = true;
              if (file_stream.is_open()) {
                file_stream << "---- OPTIMIZATION DONE ----" << endl;
                file_stream << "p:"   << setprecision(numeric_limits<double>::max_digits10) << p[0]   << ',' << p[1]  << ','<< p[2]   << endl;
                file_stream << "dp:"  << dp[0]  << ',' << dp[1] << ','<< dp[2]  << endl;
                file_stream << "nCTE: " << nCTE << " aSpeed: "  << avgSpeed/nCTE << " bSpeed: " << best_speed << endl;
                file_stream << "----------------" << endl << endl;
              }
              return false;
            }
            
            // If we had improvement try a cycle again
            dp[0] = dp[2] = 0.5;
            nImprovement = 0;
          }
          
          p[current_param] += dp[current_param];          
        }
      }
      
      // Reset 
      avgSpeed = 0.0;
      nCTE = 0;
      cte_list.clear();
      
      // One optimization step
      return true;
    }
  }
  
  // Nothing happened
  return false;
}



bool PID::operator ==(const PID& pid) {
  return    pid.p[0] == p[0]
         && pid.p[1] == p[1]
         && pid.p[2] == p[2]
  ;
}

bool PID::operator !=(const PID& pid) {
  return !(operator==(pid));
}
