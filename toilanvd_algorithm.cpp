// Author(s):         Dung Nguyen
// Maintainer:        Dung Nguyen (vietdung@kaist.ac.kr)

#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>

#include <fstream>
#include <iostream>
#include <cmath>

class my_ai
  : public aiwc::ai_base
{
public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
  {
    // you don't have any information of the game here
  }

private:
  double dist(double x1, double y1, double x2, double y2){
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  double get_spin_angle(double robot_coor[2], double robot_th, double target[2]){
    double robot_angle = (robot_th < 0.0) ? 2*PI + robot_th : robot_th;
    double target_angle = std::acos((target[0] - robot_coor[0]) / dist(target[0], target[1], robot_coor[0], robot_coor[1]));
    if(target[1] >= robot_coor[1]) target_angle = 2*PI - target_angle;
    double robot_spin_angle = robot_angle + target_angle;
    while(robot_spin_angle > 2*PI) robot_spin_angle -= 2*PI;
    if(robot_spin_angle > PI) robot_spin_angle = robot_spin_angle - 2*PI;
    return robot_spin_angle;
  }

  void init()
  {
    // now you have information of the game
    std::cout << "(There is a bug here, should be 0.5 1.3) penalty_area = " << info.penalty_area[0] << " " << info.penalty_area[1] << std::endl;
    std::cout << "(There is a bug here, should be 0.2 0.85) goal_area = " << info.goal_area[0] << " " << info.goal_area[1] << std::endl;
    std::cout << "field = " << info.field[0] << " " << info.field[1] << std::endl;
    std::cout << "goal = " << info.goal[0] << " " << info.goal[1] << std::endl;
    // double resolution_x = info.resolution[0];
    // double resolution_y = info.resolution[1];
  }

  double d2r(double deg) {
    return deg * PI / 180;
  }

  double r2d(double rad) {
    return rad * 180 / PI;
  }

  auto velocity(double l, double r)
  {
    if (std::fabs(l) > info.max_linear_velocity || std::fabs(r) > info.max_linear_velocity) {
      double multiplier;
      if (std::fabs(l) > std::fabs(r))
        multiplier = info.max_linear_velocity / std::fabs(l);
      else
        multiplier = info.max_linear_velocity / std::fabs(r);

      l *= multiplier;
      r *= multiplier;
    }

    return std::make_pair(l, r);
  }

  auto move_to_target_3_phases(int i, double new_dist, double move_forward, double robot_spin_angle){
    if((new_dist <= last_dist[i] || std::fabs(robot_spin_angle) <= ANGLE_DIFF_MAX_TO_MOVE) 
        && phase[i] >= 2) phase[i] = 2;
    //else if(std::fabs(robot_spin_angle) >= ANGLE_DIFF_MIN_TO_STOP && phase[i] > 1) phase[i] = 0;
    else if(std::fabs(robot_spin_angle) > ANGLE_DIFF_MAX_TO_MOVE && phase[i] == 1) phase[i] = 0;

    //std::cout << f.time << " robot_spin_angle = " << robot_spin_angle * 180.0 / PI << ", robot_angle = " << robot_angle * 180.0 / PI << ", phase = " << phase[i];
    //std::cout << ", robot = (" << robot_coor[0] << " , " << robot_coor[1] << ")";

    last_dist[i] = new_dist;
    phase[i] = (phase[i] + 1) % PHASE_MOD;

    if((phase[i]-1+PHASE_MOD)%PHASE_MOD == 0)
      return std::make_pair(AMPLIFIER * (robot_spin_angle * info.axle_length * 0.5 / FRAME_INTERVAL),
                          AMPLIFIER * (-robot_spin_angle * info.axle_length * 0.5 / FRAME_INTERVAL));
    else if((phase[i]-1+PHASE_MOD)%PHASE_MOD == 1)
      return std::make_pair(0.0, 0.0);
    else
      return std::make_pair(move_forward * info.max_linear_velocity * (i == 4 ? 2.0/3.0 : 1.0),
                            move_forward * info.max_linear_velocity * (i == 4 ? 2.0/3.0 : 1.0));
    //std::cout << ", velocity = (" << wheels[0] << " , " << wheels[1] << ")" << std::endl;
  }

  auto move_to_target_curve(double d_e, double sign, double d_th, double damping = 0.35){
    const double mult_lin = 3.5;
    const double mult_ang = 0.4;

    double ka;
    if(d_e > 1) {        ka = 17; }
    else if(d_e > 0.5) { ka = 19; }
    else if(d_e > 0.3) { ka = 21; }
    else if(d_e > 0.2) { ka = 23; }
    else               { ka = 25; }
    ka /= 90;

    if(std::fabs(d_th) > d2r(85)) {
      return velocity(mult_ang * d_th, -mult_ang * d_th);
    }
    else {
      if(d_e < 5.0 && fabs(d_th) < d2r(40)) {
        ka = 0.1;
      }
      ka *= 4;
      return velocity(
         sign * (mult_lin * (1 / (1 + std::exp(-3 * d_e)) - damping)) + mult_ang * ka * d_th,
         sign * (mult_lin * (1 / (1 + std::exp(-3 * d_e)) - damping)) - mult_ang * ka * d_th);
    }
  }

  void update(const aiwc::frame& f)
  {
    if(f.reset_reason == aiwc::GAME_START) {
      std::cout << "Game started : " << f.time << std::endl;
    }
    if(f.reset_reason == aiwc::SCORE_MYTEAM) {
      // yay! my team scored!
      std::cout << "Myteam scored : " << f.time << std::endl;
    }
    else if(f.reset_reason == aiwc::SCORE_OPPONENT) {
      // T_T what have you done
      std::cout << "Opponent scored : " << f.time << std::endl;
    }
    else if(f.reset_reason == aiwc::GAME_END) {
      // game is finished. finish() will be called after you return.
      // now you have about 30 seconds before this process is killed.
      std::cout << "Game ended : " << f.time << std::endl;
      return;
    }

    /* My algorithm */
    if(f.reset_reason != aiwc::NONE){
      last_ball[0] = 0.0; last_ball[1] = 0.0;
      for(int i = 0; i < 5; i++){
        phase[i] = 0;
        last_dist[i] = INF;
      }
    }

    std::array<double, 10> wheels;
    for(int i = 0; i < 10; i++) wheels[i] = 0.0;

    double new_ball[2];
    new_ball[0] = (*f.opt_coordinates).ball.x; new_ball[1] = (*f.opt_coordinates).ball.y;

    for(int i = 0; i <= 4; i++){
      double robot_coor[2];
      robot_coor[0] = (*f.opt_coordinates).robots[MYTEAM][i].x;
      robot_coor[1] = (*f.opt_coordinates).robots[MYTEAM][i].y;
      double robot_th = (*f.opt_coordinates).robots[MYTEAM][i].th;

      double new_dist = dist(robot_coor[0], robot_coor[1], new_ball[0], new_ball[1]);

      double target[2];

      if(i <= 1 || i == 4 
        || (i == 3 && !(*f.opt_coordinates).robots[MYTEAM][1].is_active) 
        || (i == 2 && !(*f.opt_coordinates).robots[MYTEAM][0].is_active)){
        target[0] = TARGET_COEF * new_ball[0] - (TARGET_COEF - 1.0) * last_ball[0];
        target[1] = TARGET_COEF * new_ball[1] - (TARGET_COEF - 1.0) * last_ball[1];

        if(i == 4){
          target[0] = -info.field[0] / 2.0 + GOAL_AREA_LENGTH / 2.0;
          target[1] = new_ball[1];
          if(target[1] < -GOAL_AREA_WIDTH / 2.0) target[1] = -GOAL_AREA_WIDTH / 2.0;
          if(target[1] > GOAL_AREA_WIDTH / 2.0) target[1] = GOAL_AREA_WIDTH / 2.0;
        }
      }
      else{
        target[0] = (*f.opt_coordinates).robots[OPPONENT][4].x; 
        target[1] = (*f.opt_coordinates).robots[OPPONENT][4].y;
      }

      double forward_spin_angle = get_spin_angle(robot_coor, robot_th, target); // spin angle if move forward
      double backward_spin_angle = get_spin_angle(robot_coor, (robot_th < 0.0 ? 1.0 : -1.0) * (PI - std::fabs(robot_th)), target); // spin angle if move backward
      double move_forward = std::fabs(forward_spin_angle) <= std::fabs(backward_spin_angle) ? 1.0 : -1.0;
      double robot_spin_angle = move_forward > 0.0 ? forward_spin_angle : backward_spin_angle;

      if(i % 2) std::tie(wheels[2*i], wheels[2*i+1]) = move_to_target_3_phases(i, new_dist, move_forward, robot_spin_angle);
      else std::tie(wheels[2*i], wheels[2*i+1]) = move_to_target_curve(dist(robot_coor[0], robot_coor[1], target[0], target[1]), move_forward, robot_spin_angle);
    }

    last_ball[0] = (*f.opt_coordinates).ball.x; last_ball[1] = (*f.opt_coordinates).ball.y;
    set_wheel(wheels);
    /* End of my algorithm */
  }

  void finish()
  {
    // You have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");
    ofs << "my_result" << std::endl;
  }

private: // member variable
  const double PI = 3.14159265358979323846;
  const double INF = 1000000000.0;
  const double FRAME_INTERVAL = 0.05; // second
  const double GOAL_AREA_LENGTH = 0.2;
  const double GOAL_AREA_WIDTH = 0.85;
  const double PENALTY_AREA_LENGTH = 0.5;
  const double PENALTY_AREA_WIDTH = 1.3;

  const double TARGET_COEF = 3.0; // if no friction exists, then TARGET_COEF = 3.0
  const double ANGLE_DIFF_MAX_TO_MOVE = PI / 12.0;
  const double GK_ANGLE_AMPLIFIER = 2.0; // not using
  const double ANGLE_DIFF_MIN_TO_STOP = PI / 2.0; // not using
  const double AMPLIFIER = 1.0; // not using
  const int PHASE_MOD = 4;
  const int PHASE_MOD_GK = 4; // not using

  double last_ball[2]; // last ball position
  double last_dist[5]; // last distance far from the ball
  int phase[5]; // 0 = spin, 1 = stop, other = move
};

int main(int argc, char *argv[])
{
  if(argc < 6) {
    std::cerr << "Usage " << argv[0] << " server_ip port realm key datapath" << std::endl;
    return -1;
  }

  const auto& server_ip = std::string(argv[1]);
  const auto& port      = boost::lexical_cast<std::size_t>(argv[2]);
  const auto& realm     = std::string(argv[3]);
  const auto& key       = std::string(argv[4]);
  const auto& datapath  = std::string(argv[5]);

  my_ai ai(server_ip, port, realm, key, datapath);

  ai.run();

  return 0;
}
