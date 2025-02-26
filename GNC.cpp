//
//  GNC.cpp
//  FlightSim
//
//  Created by Vincent Ning on 8/12/24.
//

#include "GNC.hpp"
#include "utilities.hpp"
#include "math.hpp"
#include "trajectory.hpp"

using namespace std;
void Guidance::initialize()
{
    servo_cmd = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    u_phi = 0.0;
    u_psi = 0.0;
    flight_state = rocket.get_fstate();
    
}

void Guidance::IGM_initialize(vector<double> V_initial, vector<double> P_initial,
                              vector<double> steering_angle_initial, double tf)
{
    time_to_go = tf - rocket.t;
    ti = rocket.t;
    this->t = ti;
    t_max = (tf - t) + 10.0;
    time_to_go_n = time_to_go;
    t_igm = 0;

    vector<double> p_ECIt = launch_inertial_to_ECSF(Pi_TECO);
    range_angle = range_angle_initial_AN;
    
    vector<vector<double>> M_phi = {{cos(range_angle), sin(range_angle) , 0},
                                    {-sin(range_angle), cos(range_angle), 0},
                                    {0, 0, 1}};
    
    C_et = multiply_matrices(M_phi, G);
    vector<double> v_terminal = Matrix_multiply_vector(C_et, Vi_TECO);
    vector<double> p_terminal = Matrix_multiply_vector(C_et, p_ECIt);
    v_xiM = v_terminal[0];
    v_etaM = v_terminal[1];
    v_zetaM = v_terminal[2];
    p_xiM = p_terminal[0];
    p_etaM = p_terminal[1];
    p_zetaM = p_terminal[2];
    
    phi_optimal_v = phi_optimal = phi_optimal_n = steering_angle_initial[0];
    psi_optimal_v = psi_optimal = psi_optimal_n = steering_angle_initial[1];

    ofstream rrr("IGMCoefs.csv");
    if(!rrr.is_open()){
        cerr << "error opening file: " << "IGMCoefs.csv" << endl;
    }
    rrr << "k1," 
        << "k2,"
        << "k3,"
        << "k4,"<< "Ay," << "By," << "Cy," << "Dy," << "Ey," << "Ap," << "Bp," << "Cp,"
    << "Dp," << "Ep,";
    rrr << endl;
    
}

void Guidance::set_euler_angles(string file)
{
    euler_angles = read_table(file);
}



vector<double> Guidance::get_attitude(double t, vector<double> g, vector<double> pi, vector<double> vi, double m)
{
    vector<double> result(3);
    vector<size_t> range(2);
    vector<double> IGM_cmd;
    if(t > ti)
    {
        t_igm += 0.01;
    }
    
    switch(flight_state)
    {
        case flight_states::YJFX:
            range = ascending_binary_search(euler_angles, 1, t);
            if(range[0] == range[1])
            {
                result[0] = euler_angles[range[0]][1];
                result[1] = euler_angles[range[0]][2];
                result[2] = euler_angles[range[0]][3];
            }
            else
            {
                result[0] = euler_angles[range[0]][1] + (euler_angles[range[1]][1] - euler_angles[range[0]][1]) / (euler_angles[range[1]][0] - euler_angles[range[0]][0]) * (t - euler_angles[range[0]][0]);
                result[1] = euler_angles[range[0]][2] + (euler_angles[range[1]][2] - euler_angles[range[0]][2]) / (euler_angles[range[1]][0] - euler_angles[range[0]][0]) * (t - euler_angles[range[0]][0]);
                result[2] = euler_angles[range[0]][3] + (euler_angles[range[1]][3] - euler_angles[range[0]][3]) / (euler_angles[range[1]][0] - euler_angles[range[0]][0]) * (t - euler_angles[range[0]][0]);
            }
            result[0] = result[0] * D2R;
            result[1] = result[1] * D2R;
            return result;
            break;
        case flight_states::EJFX:
        case flight_states::SJFX:
            IGM_cmd = IGM_step(g, pi, vi, t, m);
            return IGM_cmd;
            break;
        default:
            break;
    }
    return result;
}

vector<double> Guidance::get_Vi_traj(double t)
{
    vector<double> result(3);
    vector<size_t> range(2);
    range = ascending_binary_search(V_traj, 1, (double) t);
    if(range[0] == range[1])
    {
        result[0] = V_traj[range[0]][1];
        result[1] = V_traj[range[0]][2];
        result[2] = V_traj[range[0]][3];
    }
    else
    {
        result[0] = V_traj[range[0]][1] + (V_traj[range[1]][1] - V_traj[range[0]][1]) / (V_traj[range[1]][0] - V_traj[range[0]][0]) * (t - V_traj[range[0]][0]);
        result[1] = V_traj[range[0]][2] + (V_traj[range[1]][2] - V_traj[range[0]][2]) / (V_traj[range[1]][0] - V_traj[range[0]][0]) * (t - V_traj[range[0]][0]);
        result[2] = V_traj[range[0]][3] + (V_traj[range[1]][3] - V_traj[range[0]][3]) / (V_traj[range[1]][0] - V_traj[range[0]][0]) * (t - V_traj[range[0]][0]);
    }
    return result;
}
vector<double> Guidance::get_Pi_traj( double t)
{
    vector<double> result(3);
    vector<size_t> range(2);
    range = ascending_binary_search(P_traj, 1, (double) t);
    if(range[0] == range[1])
    {
        result[0] = P_traj[range[0]][1];
        result[1] = P_traj[range[0]][2];
        result[2] = P_traj[range[0]][3];
    }
    else
    {
        result[0] = P_traj[range[0]][1] + (P_traj[range[1]][1] - P_traj[range[0]][1]) / (P_traj[range[1]][0] - P_traj[range[0]][0]) * (t - P_traj[range[0]][0]);
        result[1] = P_traj[range[0]][2] + (P_traj[range[1]][2] - P_traj[range[0]][2]) / (P_traj[range[1]][0] - P_traj[range[0]][0]) * (t - P_traj[range[0]][0]);
        result[2] = P_traj[range[0]][3] + (P_traj[range[1]][3] - P_traj[range[0]][3]) / (P_traj[range[1]][0] - P_traj[range[0]][0]) * (t - P_traj[range[0]][0]);
    }
    return result;
}

vector<double> Guidance::algor_cmd( double t)
{
    vector<double> result;
    if(t >=90.0 && t < 140.0)
    {
        vector<double> Vi_traj = get_Vi_traj(t);
        vector<double> Pi_traj = get_Pi_traj(t);
        vector<double> g = calc_G(Vi_traj, Pi_traj);
        double del_theta = g[1];
        double del_H = g[0];
        //法向引导
        if(t >= 110)
        {
            u_phi = -1.2 * del_theta ;
        }
        
        if(t < 110)
        {
            u_psi = -1.2 * del_H;
        }
    } else {
        u_phi = 0.0;
        u_psi = 0.0;
    }
    result = {u_phi,u_psi};
    return result;
}
vector<double> Guidance::calc_G(vector<double> vi, vector<double> pi) {
    double del_z = rocket.get_PI()[2] - pi[2];
    double del_H = del_z;
    double del_theta;
    
    if (abs(vi[0]) < 1e-10 || abs(rocket.get_VI()[0]) < 1e-10) {
        del_theta = 0.0;
    } else {
        del_theta = rocket.get_VI()[1]/rocket.get_VI()[0] - vi[1]/vi[0];
    }
    
    return {del_H, del_theta};
}

bool Guidance::SECO(const vector<double>& pi, const vector<double>& vi, double time_to_go, double t)
{
    // Transform position and velocity from launch inertial to ECSF frame
    vector<double> Pe = Matrix_multiply_vector(C_ea, launch_inertial_to_ECSF(pi));
    vector<double> Ve = Matrix_multiply_vector(C_ea, vi);
    double Pe_mag = vector_mag(Pe);
    double Ve_mag = vector_mag(Ve);
    
    // Compute the current semi-major axis and angular momentum
    a_curr = calc_semi_major_axis(Pe_mag, Ve_mag);
    vector<double> H = cross_product33(Pe, Ve);
    double H_mag = vector_mag(H);
    
    // Compute radial velocity and inclination
    double vr = dot_product(Pe, Ve) / Pe_mag;
    i = acos(H[2] / H_mag);
    
    // Compute right ascension of the ascending node (RA)
    vector<double> N = cross_product33({0, 0, 1}, H);
    double N_mag = vector_mag(N);
    if (N_mag != 0) {
        RA = acos(N[0] / N_mag);
        if (N[1] < 0) {
            RA = 2 * PI - RA;
        }
    } else {
        RA = 0;
    }
    
    // Compute eccentricity vector and magnitude
    vector<double> E = scalar_vector(1 / GM,
                      subtract_vectors(scalar_vector(pow(Ve_mag, 2) - GM / Pe_mag, Pe),
                                       scalar_vector(Pe_mag * vr, Ve)));
    e = vector_mag(E);
    
    // Compute argument of perigee (w) if possible
    if (N_mag != 0.0 && e > Epsilon0) {
        w = acos(dot_product(N, E) / (N_mag * e));
        if (E[2] < 0) {
            w = 2 * PI - w;
        }
    } else {
        w = 0;
    }
    
    // Compute true anomaly (TA)
    if (e > Epsilon0) {
        TA = acos(dot_product(E, Pe) / (e * Pe_mag));
        if (vr < 0) {
            TA = 2 * PI - TA;
        }
    } else {
        // Handle near-circular orbits using the cross product sign
        vector<double> cp = cross_product33(N, Pe);
        if (cp[2] >= 0)
            TA = acos(dot_product(N, Pe) / (N_mag * Pe_mag));
        else
            TA = 2 * PI - acos(dot_product(N, Pe) / (N_mag * Pe_mag));
    }
    
    // Check SECO conditions by comparing current orbital parameters with target values
    if (a_curr - a_terminal > 0 &&
        fabs(e - eccentricity) <= 0.005 &&
        fabs(i - inclination) * R2D <= 0.07)
    {
        // Improved output using std::cout with fixed precision formatting
        cout << fixed << setprecision(6);
        cout << "a = " << a_curr << "  e = " << e
             << "  i = " << i * R2D << "  ω = " << w * R2D
             << "  Ω = " << RA * R2D << "  υ = " << TA * R2D << "\n";
        cout << "δa = " << a_terminal - a_curr << "\n";
        cout << "δe = " << eccentricity - e << "\n";
        cout << "δi = " << (inclination - i) * R2D << "\n";
        cout << "δΩ = " << (right_ascension - (RA + longitude - omega_e * t)) * R2D << "\n";
        cout << "δυ = " << (true_anomaly - TA) * R2D << "\n";
        cout << "δω = " << (arugument_of_perigee - w) * R2D << "\n";
        cout << "δx = " << x_SECO - rocket.get_PI()[0]
             << ", δy = " << y_SECO - rocket.get_PI()[1]
             << ", δz = " << z_SECO - rocket.get_PI()[2]
             << ", δvx = " << vx_SECO - rocket.get_VI()[0]
             << ", δvy = " << vy_SECO - rocket.get_VI()[1]
             << ", δvz = " << vz_SECO - rocket.get_VI()[2] << "\n";
        return true;
    }
    
    // Check if maximum allowable time has elapsed
    if ((t - ti) >= t_max)
    {
        cout << "Fuel depleted, orbit insertion failed\n";
        cout << "t = " << t << "\n";
        cout << "x = " << pi[0] << "  y = " << pi[1] << "  z = " << pi[2] << "\n";
        return true;
    }
    
    return false;
}

Guidance::Guidance(double t, rkt::rocket& obj,
                   std::vector<std::vector<double>> table,
                   std::vector<std::vector<double>> V_table,
                   std::vector<std::vector<double>> P_table)
: P_traj{P_table}, rocket{obj},
      euler_angles{table}, V_traj{V_table}
{

    initialize();
}

double Guidance::calc_semi_major_axis(double r, double v)
{
    double energy = pow(v,2) - 2*GM/r;  // Specific orbital energy
    if (fabs(energy) < Epsilon0) {
        // Parabolic/escape trajectory - semi-major axis is infinite
        a_curr = 1e15;  // Very large value to represent near-infinite
    } else {
        a_curr = -GM / energy;
    }
    return a_curr;
}

vector<double> Guidance::IGM_step(vector<double> g, vector<double> pi, vector<double> vi, double t, double m)
{
    tau = -rocket.get_mass() / rocket.get_del_m();

    // Prevent division by zero and log of invalid values near engine cutoff
    double tau_remaining = tau - time_to_go;
    if (tau_remaining < Epsilon0) {
        tau_remaining = Epsilon0;
        time_to_go = tau - Epsilon0;  // Adjust time_to_go to maintain consistency
    }

    double log_tau_ratio = log(tau / tau_remaining);
    double A = Vex3 * log_tau_ratio;
    double J = Vex3 * (tau * log_tau_ratio - time_to_go);
    double S = Vex3 * (tau_remaining * log_tau_ratio - time_to_go);
    double Q = Vex3 * (0.5 * pow(time_to_go,2) + tau * (tau_remaining * log_tau_ratio - time_to_go));
    
    vector<double> vG = Matrix_multiply_vector(G, vi);
    vector<double> pG = Matrix_multiply_vector(G, launch_inertial_to_ECSF(pi));
    
    vector<vector<double>> M_phi = {{cos(range_angle), sin(range_angle) , 0},
                                    {-sin(range_angle), cos(range_angle), 0},
                                    {0, 0, 1}};
    
    C_et = multiply_matrices(M_phi, G);
    vector<double> p_terminal = Matrix_multiply_vector(C_et, launch_inertial_to_ECSF(Pi_TECO));
    vector<double> v_terminal = Matrix_multiply_vector(C_et, Vi_TECO);
    p_xiM = p_terminal[0];
    p_etaM = p_terminal[1];
    p_zetaM = p_terminal[2];
    v_xiM = v_terminal[0];
    v_etaM = v_terminal[1];
    v_zetaM = v_terminal[2];
    
    beta_e = atan2(-pG[0], pG[1]);
    beta_t = 1 / p_etaM * (vG[0] * time_to_go - S + 0.5 * g[0] * pow(time_to_go, 2));
    range_angle = beta_e - beta_t; //beta_e - beta_t or beta_e + beta_t 取决于飞行方向
    
    vector<double> v_c = Matrix_multiply_vector(C_et, vi);
    vector<double> p_c = Matrix_multiply_vector(C_et, launch_inertial_to_ECSF(pi));
    
    double del_v_xi = v_xiM - v_c[0] - g[0] * time_to_go;
    double del_v_eta = v_etaM - v_c[1] - g[1] * time_to_go;
    double del_v_zeta = v_c[2] - v_zetaM + g[2] * time_to_go;
    double del_v = sqrt(pow(del_v_xi , 2) + pow(del_v_eta , 2) + pow(del_v_zeta , 2));
    UpdateTimeToGo3(del_v, g, v_terminal, v_c);

    // Add iteration limit to prevent infinite loops
    int max_iterations = 100;
    int iteration = 0;
    while(abs(time_to_go - time_to_go_n) >= 1e-5 && iteration < max_iterations)
    {
        del_v_xi = v_xiM - v_c[0] - g[0] * time_to_go;
        del_v_eta = v_etaM - v_c[1] - g[1] * time_to_go;
        del_v_zeta = v_c[2] - v_zetaM + g[2] * time_to_go;
        del_v = sqrt(pow(del_v_xi , 2) + pow(del_v_eta , 2) + pow(del_v_zeta , 2));

        UpdateTimeToGo3(del_v, g, v_terminal, v_c);
        iteration++;

        // Safety check for NaN/infinity
        if (isnan(time_to_go) || isinf(time_to_go)) {
            cerr << "WARNING: time_to_go became NaN/infinity in IGM convergence" << endl;
            break;
        }
    }
    if (iteration >= max_iterations) {
        cerr << "WARNING: IGM convergence did not complete in " << max_iterations << " iterations" << endl;
    }
    
    if (del_v_xi != 0) {
        phi_optimal_v = atan2(del_v_eta , del_v_xi);
    }
    else
    {
        phi_optimal_v = (del_v_eta >= 0) ? M_PI_2 : -M_PI_2 ;
        
    }
    double del_pitch = sqrt(pow(del_v_xi , 2) + pow(del_v_eta , 2));
    if(del_pitch != 0)
    {
//根据箭体和发惯系右手定义，若需要向正z走则需要负偏航角,速度导引角完全是需要速度之间的比值
        psi_optimal_v = asin(del_v_zeta / del_pitch);
    }
    else
    {
        psi_optimal_v = (del_v_zeta >= 0) ? M_PI_2 : -M_PI_2; // Set to ±90 degrees based on del_v_zeta
    }
    
    double Ay = A;
    double By = J;
    double Cy = S * cos(psi_optimal_v);
    double Dy = Q * cos(psi_optimal_v);
    double Ey = p_c[2] + v_c[2] * time_to_go + 0.5 * g[2] * pow(time_to_go , 2) + S * sin(psi_optimal_v);
    double K3 = By * Ey / (By * Cy - Ay * Dy);
    double K4 = Ay * K3 / By;
    
    //cos(psi)可以cancel out
    double Ap = Ay;
    double Bp = By;
    double Cp = Cy * cos(phi_optimal_v);
    double Dp = Q * cos(phi_optimal_v);
    double Ep = -p_etaM + p_c[1] + v_c[1] * time_to_go + 0.5 * g[1] * pow(time_to_go , 2) - S * sin(phi_optimal_v);
    double K1 = Bp * Ep / (Ap * Dp - Bp * Cp);
    double K2 = Ap * K1 / Bp;
    
    if(time_to_go <= 35.0)
    {
        K1 = K2 = 0;
    }
    //can be tweaked
    if(time_to_go <= 15.0)

    {
        K3 = K4 = 0.0;
    }
    
    phi_optimal = phi_optimal_v + K2 * guidance_cycle - K1;
    psi_optimal = psi_optimal_v + K4 * guidance_cycle - K3;
    

    //rotate steering angle back to inertial frame
    
    vector<double> steeringCos_terminal = {cos(psi_optimal) * cos(phi_optimal),
                                 cos(psi_optimal) * sin(phi_optimal),
                                 sin(psi_optimal)};
    vector<double> steeringCos_inertial = Matrix_multiply_vector(transpose_matrix(C_et), steeringCos_terminal);
    
    psi_optimal = asin(steeringCos_inertial[2]);
    phi_optimal = atan2(steeringCos_inertial[1] / cos(psi_optimal), steeringCos_inertial[0] / cos(psi_optimal));
    
    //steering angle constrain
    double dt = guidance_cycle;
    double max_rate = 5 * D2R; // radians per second

    // Add protection against division by zero
    if (dt > 0)
    {
        double del_phi = clamp((phi_optimal - phi_optimal_n) / dt, -max_rate, max_rate);
        double del_psi = clamp((psi_optimal - psi_optimal_n) / dt, -max_rate, max_rate);

        // Apply rate-limited changes
        phi_optimal = phi_optimal_n + del_phi * dt;
        psi_optimal = psi_optimal_n + del_psi * dt;
    }
    else
    {
        // If dt is zero, maintain previous values
        phi_optimal = phi_optimal_n;
        psi_optimal = psi_optimal_n;
    }
    
    if(time_to_go <= 15.0)
    {
        phi_optimal = phi_optimal_n;
    }
    if(time_to_go <= 15.0)
    {
        psi_optimal = psi_optimal_n;
    }


    // Store for next iteration
    phi_optimal_n = phi_optimal;
    psi_optimal_n = psi_optimal;
    
    return {phi_optimal, psi_optimal, 0.0};
}


void Guidance::update_time_to_go(double del_v)
{
    double time_to_go_approx = tau * (1 - exp(-del_v / Vex3));
    time_to_go_n = time_to_go;
    time_to_go = time_to_go_approx;
    
    double A = Vex3 * log(tau / (tau - time_to_go));
    double G = 0.5 * (pow(del_v , 2) / A - A);
    time_to_go_n = time_to_go;
    time_to_go = time_to_go + G * ((tau - time_to_go) / Vex3);
    

    
}


void Guidance::UpdateTimeToGo3(double del_v, vector<double> g, vector<double> vt, vector<double> vc)
{

//    double time_to_go_approx = tau - tau / exp(del_v / Vex3);
//    time_to_go_n = time_to_go;
//    time_to_go = time_to_go_approx;
    
//    double A = Vex3 * log(tau / (tau - time_to_go));
//    double G = 0.5 * (pow(del_v , 2) / A - A);
//    time_to_go_n = time_to_go;
//    time_to_go = time_to_go + G * ((tau - time_to_go) / Vex3);
    
    // Prevent division by zero near engine cutoff
    double tau_remaining = tau - time_to_go;
    if (tau_remaining < Epsilon0) {
        tau_remaining = Epsilon0;
    }

    double A = Vex3 * log(tau / tau_remaining);
    double dAdT = Vex3 / tau_remaining;
    double a1 = dot_product(g, g);
    double b1 = 2 * (dot_product(vc, g) - dot_product(vt, g));
    double c1 = dot_product(vt, vt) + dot_product(vc, vc) - 2 * dot_product(vt, vc);

    // Check for division by zero in h calculation
    double h;
    if (fabs(a1) < Epsilon0) {
        h = time_to_go;  // Keep current value if can't compute
    } else {
        h = -b1 / (2 * a1);
    }

    double a2 = dAdT;
    double b2 = -dAdT * time_to_go + A;
    double c2 = pow(b2,2);
    double a = a1 - pow(a2, 2);
    double b = b1 - 2 * a2 * b2;
    double c = c1 - pow(c2, 2);

    // Check discriminant and denominator for quadratic formula
    double discriminant = pow(b, 2) - 4 * a * c;
    if (discriminant < 0) {
        // No real roots, keep h value
        time_to_go_n = time_to_go;
        time_to_go = h;
    } else if (fabs(a) < Epsilon0) {
        // Linear equation, use h value
        time_to_go_n = time_to_go;
        time_to_go = h;
    } else {
        double sqrt_discriminant = sqrt(discriminant);
        double root1 = (-b + sqrt_discriminant) / (2 * a);
        double root2 = (-b - sqrt_discriminant) / (2 * a);
        time_to_go_n = time_to_go;
        time_to_go = h;
    }
//    if(root1 < 0 && root2 > 0)
//    {
//        time_to_go = root2;
//    }
//    else if(root2 < 0 && root1 > 0)
//    {
//        time_to_go = root1;
//    }
//    else if(root1 < root2)
//    {
//        time_to_go = root2;
//    }
//    else if(root2 < root1)
//    {
//        time_to_go = root1;
//    }
//    else if(root1 == root2)
//    {
//        time_to_go = root1;
//    }
//    else
//    {
//        time_to_go = 0.0;
//    }
}
