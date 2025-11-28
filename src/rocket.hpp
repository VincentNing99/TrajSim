//
//  rocket.hpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//

#ifndef rocket_hpp
#define rocket_hpp

#include "sim.hpp"

namespace rkt{
    class rocket
    {
    public:
        rocket(){};
        //copy constructor
        rocket(const rocket& other) = default;
        rocket(double t, std::vector<double> Pi, std::vector<double> Vi, std::vector<double> Vb_init, std::vector<double> aero_angle, double m_i, flight_states state, std::vector<double> steering_angle, std::vector<double> R) {
            this->t = t;
            R0 = R;
            f_states = state;
            t = 0.0;
            phi = steering_angle[0];
            psi = steering_angle[1];
            gamma = steering_angle[2];
            x = Pi[0];
            y = Pi[1];
            z = Pi[2];
            x_n = Pi[0];
            y_n = Pi[1];
            z_n = Pi[2];
            x_dot = Vi[0];
            y_dot = Vi[1];
            z_dot = Vi[2];
            x_dot_n = Vi[0];
            y_dot_n = Vi[1];
            z_dot_n = Vi[2];

            Vb = Vb_init;
            rkt_mass = rkt_mass_n = m_i;
            mass_dot = 0.0;
            alpha = aero_angle[0];
            beta = aero_angle[1];

            initialize();
        };
        void initialize();
        void mass_properties();
        void update_states(std::vector<double> k, double step);
        void update_fstates(std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>,double, double);
        void angle_calc(std::vector<double>);

        inline double get_mass(){return rkt_mass;};
        inline std::vector<double> get_attitude(){return {phi,psi,gamma};};
        inline double get_phi() {return phi;};
        inline double get_gamma() {return gamma;};
        inline double get_psi() {return psi;};
        inline double get_attack_angle(){return alpha;};
        inline double get_sideslip_angle(){return beta;};
        inline double get_flightpath_angle(){return gamma;};
        inline flight_states get_fstate(){return f_states;};
        inline void set_fstate(flight_states state) {f_states = state;};
        inline double get_length(){return L;};
        inline double get_COM() {return COM;};
        std::vector<double> get_PI();
        std::vector<double> get_VI();
        std::vector<double> get_Vb(){return Vb;};
        inline double get_VI_mag(){return sqrt(x_dot*x_dot+y_dot*y_dot+z_dot*z_dot);};
        inline double get_r(){return sqrt(x*x+y*y+z*z);}
        inline double get_sigma(){return sigma;};
        inline double get_del_m(){return mass_calc();}
        inline void set_attitude(std::vector<double> attitude){phi = attitude[0]; psi = attitude[1]; gamma = attitude[2];};
        void set_states(std::vector<double> Pi, std::vector<double> Vi, std::vector<double>, double t, double mass);
        void set_table(std::vector<std::vector<double>> table, std::string name);
        double wind(double h, windtype W_type, std::vector<std::vector<double>> table);
        std::vector<double> get_accel();
        windtype w_type;
        long double TK1, TK2, TK3;
        long double t;

    protected:
        //First stage structural properties
        std::vector<std::vector<double>> Table1Jxyz;
        //Second stage ...
        std::vector<std::vector<double>> Table2Jxyz;
        //before fairing separation
        std::vector<std::vector<double>> Table2Jxyz_fs;
        //after fairing separation
        std::vector<std::vector<double>> Table3Jxyz;
        //third stage ...
        std::vector<std::vector<double>> Table4Jxyz;
        //aerodynamics
//        std::vector<std::vector<double>> Table
        //transient engine thrust profile at SE ignition
        std::vector<std::vector<double>> Table2_thrust_on;
        //transient engine thrust profile at engine cutoff
        std::vector<std::vector<double>> Table2_thrust_off;
        flight_states f_states;
        double x_ddot, y_ddot, z_ddot;
        double x_dot, y_dot, z_dot, x_dot_n, y_dot_n, z_dot_n;

        double mass_calc();
        double x,y,z, x_n, y_n, z_n;


        double mass_dot;
        double alpha;// attack angle
        double beta;// sideslip angle
        double phi;// pitch
        double psi;// yaw
        double gamma;// roll
        double gamma_f; // flight path angle
        double sigma; // 弹道偏角
        
        double n[3];
        double Wb[3];

        double Jx, Jy, Jz, COM, delta_COM, L;
        std::vector<double> Vb, Vb_dot, Vb_dot_n, Vb_n;
        
        std::vector<double>  R0;
        double rkt_mass, rkt_mass_n;

        std::vector<double> F_total;
    };
}
#endif /* rocket_hpp */
