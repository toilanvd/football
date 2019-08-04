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
    std::cout << "field = " << info.field[0] << " " << info.field[1] << std::endl;
    std::cout << "(There is a bug here, should be 0.2 0.85) goal_area = " << info.goal_area[0] << " " << info.goal_area[1] << std::endl;
    std::cout << "goal = " << info.goal[0] << " " << info.goal[1] << std::endl;
    // double resolution_x = info.resolution[0];
    // double resolution_y = info.resolution[1];
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
      for(int i = 0; i < 5; i++) phase[i] = 0;
    }

    std::array<double, 10> wheels;
    for(int i = 0; i < 10; i++) wheels[i] = 0.0;

    for(int i = 0; i <= 4; i++){
      double robot_coor[2];
      robot_coor[0] = (*f.opt_coordinates).robots[MYTEAM][i].x;
      robot_coor[1] = (*f.opt_coordinates).robots[MYTEAM][i].y;
      double robot_th = (*f.opt_coordinates).robots[MYTEAM][i].th;

      double new_ball[2];
      new_ball[0] = (*f.opt_coordinates).ball.x; new_ball[1] = (*f.opt_coordinates).ball.y;
      double target[2];
      if(i <= 1 || i == 4 
        || (i == 3 && !(*f.opt_coordinates).robots[MYTEAM][1].is_active) 
        || (i == 2 && !(*f.opt_coordinates).robots[MYTEAM][0].is_active)){
        target[0] = TARGET_COEF * new_ball[0] - (TARGET_COEF - 1.0) * last_ball[0];
        target[1] = TARGET_COEF * new_ball[1] - (TARGET_COEF - 1.0) * last_ball[1];

        if(i == 4){
          double defend_point[2];
          defend_point[0] = -info.field[0] / 2.0 + GOAL_AREA_LENGTH / 2.0;
          defend_point[1] = target[1] * (1.0 + (target[0] - defend_point[0]) / (-info.field[0] / 2.0 - info.goal[0] - target[0]));
          if(defend_point[1] < -GOAL_AREA_WIDTH / 2.0) defend_point[1] = -GOAL_AREA_WIDTH / 2.0;
          if(defend_point[1] > GOAL_AREA_WIDTH / 2.0) defend_point[1] = GOAL_AREA_WIDTH / 2.0;
          target[0] = defend_point[0]; target[1] = defend_point[1];

          //std::cout << f.time << ": ball = (" << new_ball[0] << ", " << new_ball[1] << "), ";
          //std::cout << ", defend_point = (" << defend_point[0] << ", " << defend_point[1] << ")" << std::endl;
        }
      }
      else{
        target[0] = (*f.opt_coordinates).robots[OPPONENT][4].x; 
        target[1] = (*f.opt_coordinates).robots[OPPONENT][4].y;
      }

      double forward_spin_angle = get_spin_angle(robot_coor, robot_th, target); // spin angle if move forward
      double backward_spin_angle = get_spin_angle(robot_coor, (robot_th < 0.0 ? 1.0 : -1.0) * (PI - std::fabs(robot_th)), target); // spin angle if move backward
      bool move_forward = std::fabs(forward_spin_angle) <= std::fabs(backward_spin_angle) ? true : false;
      double robot_spin_angle = move_forward ? forward_spin_angle : backward_spin_angle;

      if(std::fabs(robot_spin_angle) <= ANGLE_DIFF_MAX_TO_MOVE && phase[i] > 1) phase[i] = 2;
      else if(std::fabs(robot_spin_angle) >= ANGLE_DIFF_MIN_TO_STOP && phase[i] > 1) phase[i] = 0;
      else if(std::fabs(robot_spin_angle) > ANGLE_DIFF_MAX_TO_MOVE && phase[i] == 1) phase[i] = 0;

      //std::cout << f.time << " robot_spin_angle = " << robot_spin_angle * 180.0 / PI << ", robot_angle = " << robot_angle * 180.0 / PI << ", phase = " << phase[i];
      //std::cout << ", robot = (" << robot_coor[0] << " , " << robot_coor[1] << ")";

      if(phase[i] == 0){
        wheels[2*i] = AMPLIFIER * (robot_spin_angle * info.axle_length * 0.5 / FRAME_INTERVAL);
        wheels[2*i+1] = AMPLIFIER * (-robot_spin_angle * info.axle_length * 0.5 / FRAME_INTERVAL);
      }
      else if(phase[i] == 1){
        wheels[2*i] = 0.0;
        wheels[2*i+1] = 0.0;
      }
      else{
        wheels[2*i] = (move_forward ? 1.0 : -1.0) * info.max_linear_velocity * ((i == 4) ? 2.0/3.0 : 1.0);
        wheels[2*i+1] = (move_forward ? 1.0 : -1.0) * info.max_linear_velocity * ((i == 4) ? 2.0/3.0 : 1.0);
      }
      //std::cout << ", velocity = (" << wheels[0] << " , " << wheels[1] << ")" << std::endl;

      if(i != 4) phase[i] = (phase[i] + 1) % PHASE_MOD;
      else phase[i] = (phase[i] + 1) % PHASE_MOD_GK;
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
  const double FRAME_INTERVAL = 0.05; // second
  const double GOAL_AREA_LENGTH = 0.2;
  const double GOAL_AREA_WIDTH = 0.85;

  const double TARGET_COEF = 3.0; // if no friction exists, then TARGET_COEF = 3.0
  const double ANGLE_DIFF_MAX_TO_MOVE = PI / 12.0;
  const double ANGLE_DIFF_MIN_TO_STOP = PI / 2.0;
  const double AMPLIFIER = 1.0;
  const int PHASE_MOD = 5;
  const int PHASE_MOD_GK = 4;

  double last_ball[2]; // last ball position
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
