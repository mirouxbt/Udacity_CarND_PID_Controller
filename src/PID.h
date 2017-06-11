#ifndef PID_H
#define PID_H

#include <deque>
#include <fstream>

using namespace std;


const double MAX_CTE_SUM = 100.0;
const unsigned int MAX_CTE_TO_KEEP = 100;
const double PARAM_TOL = 0.01;

class PID {
  public:
    /*
    * Constructor
    */
    PID(); 

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);
    void InitHyperParam(double dKp, double dKi, double dKd);
    void NeedOptimization(bool need);


    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
    
    /*
     * To optimize the parameters
     */
    bool Optimize(double cte, double speed);
    bool isOptimized() { return is_optimized; }
    
    /*
     * File where to save parameters if we optimize them
     */
     void SaveToFile(const string& filename);
     
     
     bool operator ==(const PID& pid);
     bool operator !=(const PID& pid);
     
     double get_Kp() { return p[0]; }
     double get_Ki() { return p[1]; }
     double get_Kd() { return p[2]; }

  private:
    /*
    * Errors
    */
    double error;

    /*
    * Coefficients Kp, ki, Kd
    */ 
    double p[3];
    
    // Previous CTE, used to compute the derivative part
    double prev_cte;
    
    // Queue to keep the last n CTE, used to compute the integral part
    deque<double> cte_list;
    
    // Hyper Parameters for parameter optimization
    bool go_pos;
    bool is_optimized;
    int nImprovement;

    
    double dp[3]; // dKp, dKi, dKd
    
    double best_speed;
    int current_param;
    int nCTE;
    int nCTE_steps;
    double avgSpeed;
    
    // File to save parameters
    ofstream file_stream;
};

#endif /* PID_H */
