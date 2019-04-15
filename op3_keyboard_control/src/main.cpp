#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>

#include <curses.h>

int main (int argc, char **argv){

  ros::init(argc,argv,"op3_keyboard_control");
  ros::NodeHandle node;

  ros::Publisher get_key_pub = node.advertise<std_msgs::Int32>("op3_keyboard_control", 1000);

  ROS_INFO("Use keyboard to control robot locomotion process");
  ROS_INFO("***Press   R   to start**");
  ROS_INFO("***Press SPACE to stop***");
  ROS_INFO("***Press   Q   to quit***");

  ros::Rate rate(1000); //Hz

  bool quit = false;
  bool is_running = false;

  enum control{
    quit_,
    stop_,
    run_
  };

  while(ros::ok()){
// Curses library code
    WINDOW *w;
    w=initscr();     // init new screen
    cbreak();        // use cbreack call to make terminal send all keystrokes directly
    nodelay(w,true); // non-blocking call for getch

    int msg;
    if(is_running)
      msg = run_;
    else
      msg = stop_;
    if(quit)
      msg = quit_;

    std_msgs::Int32 control_msg;
    control_msg.data = msg;
    get_key_pub.publish(control_msg);

    refresh(); // push from buffer to the real terminal
    endwin();

    if(quit)
      break;

    char input_key=getch();
    if((input_key=='q')||(input_key=='Q')) // ESC == 27
      quit = true;

    if((input_key == 'r')||(input_key == 'R')) // ENTER == 13
      is_running = true;

    if(input_key == 32)
      is_running = false;

    ros::spinOnce();
    rate.sleep();
  }

  //system("clear");

  return 0;
}
