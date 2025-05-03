#include "ExecutiveLoop.hpp"
#include "SensorDataConfig.hpp"

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char *argv[]) {
  // setup performance and time benchmarks.
  //  setup Robot during initialization.
  rclcpp::init(argc, argv);
  SetupRobot initStateandConfig = SetupRobot();

  // ExecutiveLoop Think about object by reference or value passing
  std::cout << "Executive Main Loop Object Creation" << std::endl;
  std::shared_ptr<ExecutiveLoop> mainLoopObject =
      std::make_shared<ExecutiveLoop>();
  std::shared_ptr<SensorsDataConfig> sensorsROScallback =
      std::make_shared<SensorsDataConfig>(mainLoopObject);
  // ExecutiveLoop mainLoopObject = ExecutiveLoop(argc, argv);
  // records false if run has not completed yet.
  bool runStatus = false;
  // these threads functions will have loops that go on for ever
  // these functions will have wait functions just in case with a queue
  // system.

  // std::jthread ReadInputsThread(&ExecutiveLoop::ReadInputs,
  // mainLoopObject); 
  // Creates a new thread for each function.

  std::jthread UpdateStateThread(&ExecutiveLoop::updateState, mainLoopObject);

  std::jthread ExecutiveDecisionLoopThread(&ExecutiveLoop::executeDecisionLoop,
                                           mainLoopObject);
  std::jthread SendThrusterCommandThread(&ExecutiveLoop::sendThrusterCommand,
                                         mainLoopObject);
 
  // std::jthread
  // SendThrusterCommandThread(&ExecutiveLoop::sendThrusterCommand,
  // mainLoopObject);
  std::cout << "User defined threads has ran sucessfully" << std::endl;

  rclcpp::executors::MultiThreadedExecutor SensorsExecutor;
  // auto sensorNode = std::make_shared<SensorsDataConfig>(sensorsROScallback);

  SensorsExecutor.add_node(sensorsROScallback);
  // SensorsExecutor.add_node(mainLoopObject);
  std::cout << "ROS2 runnning" << std::endl;
  SensorsExecutor.spin();
  std::cout << "ROS2 runnning" << std::endl;

  rclcpp::shutdown();
  std::cout << "ROS2 exited." << std::endl;
  /*
    ReadInputsThread.join();
    UpdateStateThread.join();
    ExecutiveDecisionLoopThread.join();
    SendThrusterCommandThread.join();*/

  return 0;
}
#endif // TESTING_EXCLUDE_MAIN