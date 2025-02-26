//
//  snm.cpp
//  FlnghtSnm
//
//  Created by Vincent Nnng on 8/7/24.
//
// #include "rocket.hpp"
#include "sim.hpp"
#include "math.hpp"
#include "rocket.hpp"
#include "gravity.hpp"
#include "aerodynamics.hpp"
#include "Atmosphere_properties.hpp"
#include "GNC.hpp"
#include "engine_model.hpp"
#include "utilities.hpp"
#include "trajectory.hpp"
using namespace std;

  int main() {
    //time
      double t = 3140.3405;
      double t_end = 3344.5194;
      vector<double> Pi_init, Vi_init;
      Pi_init = {-912205.4, -13212262.5,   149163.6};
      Vi_init = {-7440.6404,   953.8138,   294.5883};
      vector<double> steering_angle_initial = {172.7593 * D2R,0.0,0.0};
      
      gravity_model gravity = gravity_model(launch_coordinates, Pi_init, A0, B0);

      vector<double> omega_e_vec = scalar_vector(omega_e, omega_e0);
      vector<double> Ve_init = cross_product33(omega_e_vec, gravity.get_R_vec());
      vector<double> aero_angle = {0.0,0.0};
      
      vector<double> Vb_init = subtract_vectors(Vi_init, Ve_init);

      atmosphere_model atm = atmosphere_model(height_initial);

      rkt::rocket rocket = rkt::rocket(t, Pi_init, Vi_init, Vb_init,
                                        aero_angle, 6731.1, flight_states::SJFX,
                                        steering_angle_initial, gravity.get_R_vec());
      
      
      engine_model engine = engine_model();

      Guidance guidance = Guidance(t,
                                   rocket,
                                   read_table("euler angles.csv"),
                                   read_table("V_inertial.csv"),
                                   read_table("P_inertial.csv"));
      
      guidance.IGM_initialize(Vi_init,
                              Pi_init,
                              steering_angle_initial,
                              t_end);
      
      aerodynamics aero = aerodynamics(read_table("highspeed.csv"),
                                       read_table("lowspeed.csv"),
                                       EffectiveArea, L);
    
       string filename = "rk4_IGM_sim_MSC.csv";
       ofstream record(filename);
       record << "time," << "phi," << "psi," << "gamma,"<< "M," << "Px," << "Py," << "Pz," << "Vx," << "Vy," << "Vz," << "Ma,"
       << "H," << "semi_major_axis," << "time_to_go \n";
       if(!record.is_open()){
           cerr << "error opening file: " << filename << endl;
       }
      while (!guidance.SECO(rocket.get_PI(), rocket.get_VI(), guidance.get_time_to_go(), t))
      {
          //record
          vector<double> data = {t, rocket.get_phi()*R2D, rocket.get_psi()*R2D, rocket.get_gamma()*R2D, rocket.get_mass(), rocket.get_PI()[0], rocket.get_PI()[1], rocket.get_PI()[2], rocket.get_VI()[0],  rocket.get_VI()[1], rocket.get_VI()[2], atm.get_mach_number(), gravity.get_altitude(rocket.get_PI()), guidance.get_semi_major_axis(), guidance.get_time_to_go()};
          for(double value:data)
          {
              record << fixed << setprecision(6) << value << ",";
          }
          record << endl;
          static int step_counter = 0;
          if (step_counter % 1000 == 0) {
              cout << format("Τ': {:.6f} t: {:.6f} φ: {:.6f} ψ: {:.6f} β: {:.6f} δx: {:.6f} δy: {:.6f} δz: {:.6f} Δv: {:.6f} {:.6f} {:.6f}\n",
                           data[14],
                           data[0],
                           data[1],
                           data[2],
                           guidance.getRangeAngle() * R2D,
                           rocket.get_PI()[0] - x_SECO,
                           rocket.get_PI()[1] - y_SECO,
                           rocket.get_PI()[2] - z_SECO,
                           rocket.get_VI()[0] - Vi_terminal[0],
                           rocket.get_VI()[1] - Vi_terminal[1],
                           rocket.get_VI()[2] - Vi_terminal[2]);
          }
          //run till cut-off conditions reached
          rk4(t, step_counter, rocket, gravity, atm, engine, guidance, aero, rocket.get_fstate());
          // Update time
          t += step;
          step_counter++;
      }

       return 0;
}
