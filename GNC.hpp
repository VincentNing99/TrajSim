#ifndef GNC_hpp
#define GNC_hpp

#include "rocket.hpp"
#include "sim.hpp"
#include "trajectory.hpp"
class Guidance
{
public:
    Guidance(double t , rkt::rocket& obj,
             std::vector<std::vector<double>> table,
             std::vector<std::vector<double>> V_table,
             std::vector<std::vector<double>> P_table);

    void set_euler_angles(std::string file);

    void initialize();

    void IGM_initialize(std::vector<double> vi_curr, std::vector<double> pi_curr,
                        std::vector<double> , double);
    std::vector<double> calc_G(std::vector<double> vi, std::vector<double> pi);
    inline std::vector<double> get_servo_cmd() {return servo_cmd;};
    inline void set_time(double time) {t = time;};
    inline void set_time_to_go(double time_to_go) {this->time_to_go = time_to_go;};
    inline void set_time_to_go() {time_to_go -= guidance_cycle;};
    inline double get_time_to_go() {return time_to_go;};
    inline double get_semi_major_axis() {return a_curr;};
    inline double getRangeAngle() {return range_angle;};
    inline std::vector<std::vector<double>> get_terminal_guidance_rotation_matrix(){return C_et;};
    inline void set_flight_state(flight_states state) {flight_state = state;};
    std::vector<double> get_attitude(double t, std::vector<double> g, std::vector<double> Pi, std::vector<double> Vi, double m);
    std::vector<double> get_Vi_traj(double t);
    std::vector<double> get_Pi_traj(double t);
    std::vector<double> algor_cmd(double t);
    
    bool MECO();
    bool SECO(const std::vector<double>&, const std::vector<double>&, double time_to_go, double t);
    double calc_semi_major_axis(double r, double v);
    void update_time_to_go(double);
    void UpdateTimeToGo3(double del_v, std::vector<double> g, std::vector<double> vt, std::vector<double> vc);
    std::vector<double> IGM_step(std::vector<double> g, std::vector<double> Pi, std::vector<double> Vi, double t, double m);
protected:
    rkt::rocket& rocket;
    flight_states flight_state;
    
    double aiming_azimuth;
    double launch_lattitude;
    double longitude_to_descending_node;
    double injection_angle;
    //orbit elements
    double TA;
    double w;
    double RA;
    double i;
    double e;
    double a_curr;
    double beta_e;
    double beta_t;
    std::vector<std::vector<double>> C_et;
    std::vector<double> Pi_SECO = {x_SECO, y_SECO, z_SECO};
    std::vector<double> Vi_SECO = {vx_SECO, vy_SECO, vz_SECO};
    
    std::vector<double> Pi_TECO = {x_TECO, y_TECO, z_TECO};
    std::vector<double> Vi_TECO = {vx_TECO, vy_TECO, vz_TECO};

    //maximum steering speed

    double v_etaM, v_xiM, v_zetaM;
    double p_etaM, p_xiM, p_zetaM;

    double psi_optimal_v, phi_optimal_v;
    double phi_optimal, psi_optimal;
    double phi_optimal_n, psi_optimal_n;
    double t;
    double ti;
    double t_max;
    double time_to_go;
    double time_to_go_n;
    double tau;
    double t_igm;
    double range_angle;
    
    double eta_c;
    double beta_c;
    
    std::vector<double> euler_angle_cmd;
    std::vector<double> servo_cmd;
    std::vector<std::vector<double>> euler_angles;
    std::vector<std::vector<double>> V_traj;
    std::vector<std::vector<double>> P_traj;
    double Vx_a, Vy_a, Vx_b, Vy_b;
    double u_phi,u_psi;
    //横程诸元
    double dh_dvx = 5.31859760238e-005;
    double dh_dvy = -3.40241925989e-005;
    double dh_dvz = -2.04285714286e-003;
    double dh_dx = 1.62972315339e-007;
    double dh_dy = -1.07907262502e-007;
    double dh_dz = -6.26205906507e-006;
    //法向诸元
    double df_dvx = -6.06790228901e-004;
    double df_dvy =  1.58333333333e-003;
    double df_dvz = 1.13466207915e-004;
    double df_dx = 1.05868458762e-006;
    double df_dy = 6.23011412998e-007;
    double df_dz = -1.05024506217e-007;
    //射程
    double dL_dt = 73.92304;
    double dL_dvx = 323.29553;
    double dL_dvy = 370.12431;
    double dL_dvz = -6.23281;
    double dL_dx = 0.99691;
    double dL_dy = 1.30567;
    double dL_dz = -0.02114;
    double L = 776294;
};

#endif /* GNC_hpp */
