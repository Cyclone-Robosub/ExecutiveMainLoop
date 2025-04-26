#include "ExecutiveLoop.hpp"
#include "SensorData.hpp"

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
  std::shared_ptr<SensorsData> sensorsROScallback =
      std::make_shared<SensorsData>(mainLoopObject);
  // ExecutiveLoop mainLoopObject = ExecutiveLoop(argc, argv);
  // records false if run has not completed yet.
  bool runStatus = false;
  // these threads functions will have loops that go on for ever
  // these functions will have wait functions just in case with a queue
  // system.

  // std::jthread ReadInputsThread(&ExecutiveLoop::ReadInputs,
  // mainLoopObject); Creates a new thread for each node. Need to check if it
  // does

  std::jthread UpdateStateThread(&ExecutiveLoop::updateState, mainLoopObject);

  std::jthread ExecutiveDecisionLoopThread(&ExecutiveLoop::executeDecisionLoop,
                                           mainLoopObject);
  // Note: We can join these two threads above and bottom if Raspberry PI
  // really does not like multithreading.
  // This is now the case ^.
  // std::jthread
  // SendThrusterCommandsThread(&ExecutiveLoop::sendThrusterCommands,
  // mainLoopObject);
  std::cout << "User defined threads has ran successfully" << std::endl;

  rclcpp::executors::MultiThreadedExecutor SensorsExecutor;
  // auto sensorNode = std::make_shared<SensorsData>(sensorsROScallback);

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
    SendThrusterCommandsThread.join();*/

  return 0;
}
#endif  // TESTING_EXCLUDE_MAIN