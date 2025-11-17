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
    flight_state = rocket.get_fstate();
}

void Guidance::IGM_initialize(const vector<double>& V_initial, const vector<double>& P_initial,
                              const vector<double>& steering_angle_initial, double tf)
{
    time_to_go = tf - rocket.t;
    ti = rocket.t;
    this->t = ti;
    t_max = (tf - t) + 10.0;
    time_to_go_n = time_to_go;

    vector<double> p_ECIt = launch_inertial_to_ECSF(Pi_TECO.to_vector());
    range_angle = range_angle_initial_AN;

    vector<vector<double>> M_phi = {{cos(range_angle), sin(range_angle) , 0},
                                    {-sin(range_angle), cos(range_angle), 0},
                                    {0, 0, 1}};

    C_et = multiply_matrices(M_phi, G);
    vector<double> v_terminal = Matrix_multiply_vector(C_et, Vi_TECO.to_vector());
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



vector<double> Guidance::get_attitude(double t, const vector<double>& g, const vector<double>& pi, const vector<double>& vi, double m)
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
            result = {
                interpolate_column(euler_angles, range, 1, t) * D2R,
                interpolate_column(euler_angles, range, 2, t) * D2R,
                interpolate_column(euler_angles, range, 3, t)
            };
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
    vector<size_t> range = ascending_binary_search(V_traj, 1, (double) t);
    return {
        interpolate_column(V_traj, range, 1, t),
        interpolate_column(V_traj, range, 2, t),
        interpolate_column(V_traj, range, 3, t)
    };
}
vector<double> Guidance::get_Pi_traj( double t)
{
    vector<size_t> range = ascending_binary_search(P_traj, 1, (double) t);
    return {
        interpolate_column(P_traj, range, 1, t),
        interpolate_column(P_traj, range, 2, t),
        interpolate_column(P_traj, range, 3, t)
    };
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
vector<double> Guidance::calc_G(const vector<double>& vi, const vector<double>& pi) {
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

// IGM Helper Functions
IGMCoefficients Guidance::compute_igm_coefficients(double tau, double time_to_go)
{
    IGMCoefficients coef;

    // Prevent division by zero and log of invalid values near engine cutoff
    coef.tau_remaining = tau - time_to_go;
    if (coef.tau_remaining < Epsilon0) {
        coef.tau_remaining = Epsilon0;
        time_to_go = tau - Epsilon0;  // Adjust time_to_go to maintain consistency
    }

    double log_tau_ratio = log(tau / coef.tau_remaining);
    coef.A = Vex3 * log_tau_ratio;
    coef.J = Vex3 * (tau * log_tau_ratio - time_to_go);
    coef.S = Vex3 * (coef.tau_remaining * log_tau_ratio - time_to_go);
    coef.Q = Vex3 * (0.5 * pow(time_to_go, 2) + tau * (coef.tau_remaining * log_tau_ratio - time_to_go));

    return coef;
}

void Guidance::update_terminal_frame(const vector<double>& vi, const vector<double>& pi,
                                     const vector<double>& g, double S)
{
    vector<double> vG = Matrix_multiply_vector(G, vi);
    vector<double> pG = Matrix_multiply_vector(G, launch_inertial_to_ECSF(pi));

    vector<vector<double>> M_phi = {{cos(range_angle), sin(range_angle), 0},
                                    {-sin(range_angle), cos(range_angle), 0},
                                    {0, 0, 1}};

    C_et = multiply_matrices(M_phi, G);
    vector<double> p_terminal = Matrix_multiply_vector(C_et, launch_inertial_to_ECSF(Pi_TECO.to_vector()));
    vector<double> v_terminal = Matrix_multiply_vector(C_et, Vi_TECO.to_vector());

    p_xiM = p_terminal[0];
    p_etaM = p_terminal[1];
    p_zetaM = p_terminal[2];
    v_xiM = v_terminal[0];
    v_etaM = v_terminal[1];
    v_zetaM = v_terminal[2];

    beta_e = atan2(-pG[0], pG[1]);
    beta_t = 1 / p_etaM * (vG[0] * time_to_go - S + 0.5 * g[0] * pow(time_to_go, 2));
    range_angle = beta_e - beta_t;
}

DeltaVelocity Guidance::compute_delta_velocity(const vector<double>& v_c, const vector<double>& v_terminal,
                                               const vector<double>& g, double time_to_go)
{
    DeltaVelocity del_v;
    del_v.del_v_xi = v_terminal[0] - v_c[0] - g[0] * time_to_go;
    del_v.del_v_eta = v_terminal[1] - v_c[1] - g[1] * time_to_go;
    del_v.del_v_zeta = v_c[2] - v_terminal[2] + g[2] * time_to_go;
    del_v.del_v = sqrt(pow(del_v.del_v_xi, 2) + pow(del_v.del_v_eta, 2) + pow(del_v.del_v_zeta, 2));
    return del_v;
}

void Guidance::converge_time_to_go(const vector<double>& g, const vector<double>& v_terminal,
                                   const vector<double>& v_c)
{
    int max_iterations = 100;
    int iteration = 0;

    while(abs(time_to_go - time_to_go_n) >= 1e-5 && iteration < max_iterations)
    {
        DeltaVelocity del_v = compute_delta_velocity(v_c, v_terminal, g, time_to_go);
        UpdateTimeToGo3(del_v.del_v, g, v_terminal, v_c);
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
}

SteeringAngles Guidance::compute_velocity_steering_angles(const DeltaVelocity& del_v)
{
    SteeringAngles angles;

    if (del_v.del_v_xi != 0) {
        angles.phi = atan2(del_v.del_v_eta, del_v.del_v_xi);
    } else {
        angles.phi = (del_v.del_v_eta >= 0) ? M_PI_2 : -M_PI_2;
    }

    double del_pitch = sqrt(pow(del_v.del_v_xi, 2) + pow(del_v.del_v_eta, 2));
    if (del_pitch != 0) {
        angles.psi = asin(del_v.del_v_zeta / del_pitch);
    } else {
        angles.psi = (del_v.del_v_zeta >= 0) ? M_PI_2 : -M_PI_2;
    }

    return angles;
}

SteeringAngles Guidance::apply_position_corrections(const SteeringAngles& velocity_angles,
                                                   const IGMCoefficients& coef,
                                                   const vector<double>& p_c,
                                                   const vector<double>& v_c,
                                                   const vector<double>& g)
{
    SteeringAngles corrected = velocity_angles;

    double Ay = coef.A;
    double By = coef.J;
    double Cy = coef.S * cos(velocity_angles.psi);
    double Dy = coef.Q * cos(velocity_angles.psi);
    double Ey = p_c[2] + v_c[2] * time_to_go + 0.5 * g[2] * pow(time_to_go, 2) + coef.S * sin(velocity_angles.psi);
    double K3 = By * Ey / (By * Cy - Ay * Dy);
    double K4 = Ay * K3 / By;

    double Ap = Ay;
    double Bp = By;
    double Cp = Cy * cos(velocity_angles.phi);
    double Dp = coef.Q * cos(velocity_angles.phi);
    double Ep = -p_etaM + p_c[1] + v_c[1] * time_to_go + 0.5 * g[1] * pow(time_to_go, 2) - coef.S * sin(velocity_angles.phi);
    double K1 = Bp * Ep / (Ap * Dp - Bp * Cp);
    double K2 = Ap * K1 / Bp;

    if (time_to_go <= 35.0) {
        K1 = K2 = 0;
    }
    if (time_to_go <= 15.0) {
        K3 = K4 = 0.0;
    }

    corrected.phi = velocity_angles.phi + K2 * guidance_cycle - K1;
    corrected.psi = velocity_angles.psi + K4 * guidance_cycle - K3;

    return corrected;
}

SteeringAngles Guidance::transform_steering_to_inertial(const SteeringAngles& terminal_angles)
{
    vector<double> steeringCos_terminal = {cos(terminal_angles.psi) * cos(terminal_angles.phi),
                                          cos(terminal_angles.psi) * sin(terminal_angles.phi),
                                          sin(terminal_angles.psi)};
    vector<double> steeringCos_inertial = Matrix_multiply_vector(transpose_matrix(C_et), steeringCos_terminal);

    SteeringAngles inertial;
    inertial.psi = asin(steeringCos_inertial[2]);
    inertial.phi = atan2(steeringCos_inertial[1] / cos(inertial.psi), steeringCos_inertial[0] / cos(inertial.psi));

    return inertial;
}

SteeringAngles Guidance::apply_rate_limiting(const SteeringAngles& commanded,
                                            const SteeringAngles& previous,
                                            double dt, double max_rate)
{
    SteeringAngles limited = commanded;

    if (dt > 0) {
        double del_phi = clamp((commanded.phi - previous.phi) / dt, -max_rate, max_rate);
        double del_psi = clamp((commanded.psi - previous.psi) / dt, -max_rate, max_rate);

        limited.phi = previous.phi + del_phi * dt;
        limited.psi = previous.psi + del_psi * dt;
    } else {
        limited = previous;
    }

    if (time_to_go <= 15.0) {
        limited.phi = previous.phi;
        limited.psi = previous.psi;
    }

    return limited;
}

vector<double> Guidance::IGM_step(const vector<double>& g, const vector<double>& pi, const vector<double>& vi, double t, double m)
{
    tau = -rocket.get_mass() / rocket.get_del_m();

    // 1. Compute IGM coefficients
    IGMCoefficients coef = compute_igm_coefficients(tau, time_to_go);

    // 2. Update terminal guidance frame
    update_terminal_frame(vi, pi, g, coef.S);

    // 3. Transform current state to terminal frame
    vector<double> v_c = Matrix_multiply_vector(C_et, vi);
    vector<double> p_c = Matrix_multiply_vector(C_et, launch_inertial_to_ECSF(pi));
    vector<double> v_terminal = {v_xiM, v_etaM, v_zetaM};

    // 4. Compute initial delta-V and update time-to-go
    DeltaVelocity del_v = compute_delta_velocity(v_c, v_terminal, g, time_to_go);
    UpdateTimeToGo3(del_v.del_v, g, v_terminal, v_c);

    // 5. Converge time-to-go
    converge_time_to_go(g, v_terminal, v_c);

    // 6. Recompute delta-V with converged time-to-go
    del_v = compute_delta_velocity(v_c, v_terminal, g, time_to_go);

    // 7. Compute velocity steering angles
    SteeringAngles velocity_angles = compute_velocity_steering_angles(del_v);
    phi_optimal_v = velocity_angles.phi;
    psi_optimal_v = velocity_angles.psi;

    // 8. Apply position corrections
    SteeringAngles corrected_angles = apply_position_corrections(velocity_angles, coef, p_c, v_c, g);

    // 9. Transform steering angles to inertial frame
    SteeringAngles inertial_angles = transform_steering_to_inertial(corrected_angles);

    // 10. Apply rate limiting
    SteeringAngles previous_angles = {phi_optimal_n, psi_optimal_n};
    double dt = guidance_cycle;
    double max_rate = 5 * D2R; // radians per second
    SteeringAngles final_angles = apply_rate_limiting(inertial_angles, previous_angles, dt, max_rate);

    // Store for next iteration
    phi_optimal = final_angles.phi;
    psi_optimal = final_angles.psi;
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


void Guidance::UpdateTimeToGo3(double del_v, const vector<double>& g, const vector<double>& vt, const vector<double>& vc)
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
