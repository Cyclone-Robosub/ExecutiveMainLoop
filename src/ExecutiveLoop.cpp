#include "ExecutiveLoop.hpp"

//TODO from William:
// Write docstring thingies for function definitions (i.e. /// @param, /// @brief, etc.)

// start the executive Loop
// Setup for all the functions should be done here.
// Setup everything so that when the threads startup, it can run its tasks.
ExecutiveLoop::ExecutiveLoop(
    std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr,
    std::unique_ptr<Pwm_Command> currentCommand_ptr,
    std::ofstream& stateFile,
    std::ostream& output,
    std::ostream& error ) :
      Node("executive_main_node"),
      commandInterpreter_ptr(std::move(commandInterpreter_ptr)),
      currentCommand_ptr(std::move(currentCommand_ptr)),
      stateFile(stateFile),
      output(output),
      error(error),
      loopIsRunning(false),
      tasksCompleted(true) {}

void ExecutiveLoop::clearQueue() {
  std::lock_guard<std::mutex> QueueLock(Queue_pwm_mutex);
  std::queue<std::unique_ptr<Pwm_Command>> empty;
  std::swap(ManualPWMQueue, empty);
  output << "Manual Command Current Override -> Deleted Queue"
            << std::endl;
}

// these callback functions serve as the "read Input node in the loop"
//Feel free to later push these into the Sensors Class, but make sure ExecutiveLoop can still access through memory its needed fields.
void ExecutiveLoop::ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  std::unique_lock<std::mutex> Manual_Lock(Manual_Mutex);
  isManualEnabled = msg->data;
  if (isManualEnabled) {
    output << "Manual Control Enabled" << std::endl;
    // need to add to here later.
  } else {
    output << "Manual Control Disabled" << std::endl;
    clearQueue();
  }
  Change_Manual.notify_all();
}


//This should only clear the queue
void ExecutiveLoop::ManualOverrideCallback(const std_msgs::msg::Empty::SharedPtr msg) {
  std::unique_lock<std::mutex> Manual_Lock(Manual_Override_mutex);
  isManualOverride = true;
  clearQueue();
}


void ExecutiveLoop::depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg) {
  //  std::lock_guard<std::mutex> lock(mutex_);
  // std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_WAIT_TIME));
  // output << "Got depth ";
  depth_pressure_msg = msg->data;
}


void ExecutiveLoop::imuSensorCallback(const sensor_msgs::msg::Imu &msg) {
  std::lock_guard<std::mutex> CallBacklock(imu_mutex);
  // std::cout << "imu sensor\n";
  angular_velocity_x = msg.angular_velocity.x;
  angular_velocity_y = msg.angular_velocity.y;
  angular_velocity_z = msg.angular_velocity.z;
  linear_acceleration_x = msg.linear_acceleration.x;
  linear_acceleration_y = msg.linear_acceleration.y;
  linear_acceleration_z = msg.linear_acceleration.z;
}


void ExecutiveLoop::magCallback(const sensor_msgs::msg::MagneticField &msg) {
  mag_field_x = msg.magnetic_field.x;
  mag_field_y = msg.magnetic_field.y;
  mag_field_z = msg.magnetic_field.z;
}

// First get the PWM Array. Then allow the duration callback to execute and pair the array with the duration.
// Then push it onto the queue for ExecuteDecision. Notify every time we allow either Duration or Execute to
// use the queue for chain of execution. 
void ExecutiveLoop::PWMArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  output << "Received Int32MultiArray: ";
  int i = 0;
  int setvalue;
  std::unique_lock<std::mutex> pwm_lock(array_duration_sync_mutex);
  // pwm_array receivedArray;
  for (int32_t value : msg->data) {
    setvalue = (int)value;
    given_array.pwm_signals[i] = setvalue;
    output << given_array.pwm_signals[i];
    ++i;
  }
  AllowDurationSync = true;
  output << std::endl;
  SendToDuration_change.notify_all();
  pwm_lock.unlock();
}


void ExecutiveLoop::durationCallback(const std_msgs::msg::Int64::SharedPtr msg) {
  std::unique_lock<std::mutex> duration_lock(array_duration_sync_mutex,
                                              std::defer_lock);
  SendToDuration_change.wait(duration_lock,
                              [this] { return AllowDurationSync; });
  output << "Getting duration" << std::endl;
  auto duration_int_pwm = msg->data;
  std::unique_ptr<Pwm_Command> newCommand;
  std::unique_lock<std::mutex> Queue_sync_lock(Queue_pwm_mutex);
  switch (duration_int_pwm) {
  case -1: // PWM
    newCommand = std::make_unique<Untimed_Command>(given_array);
    ManualPWMQueue.push(std::move(newCommand));
    haltCurrentCommand();
    break;
  default: // TIMED PWM
    std::chrono::milliseconds durationMS = std::chrono::milliseconds(duration_int_pwm * 1000);
    output << durationMS << std::endl;
    newCommand = std::make_unique<Timed_Command>(given_array, durationMS);
    ManualPWMQueue.push(std::move(newCommand));
    ManualPWMQueue.push(std::make_unique<Untimed_Command>(stop_set_array));
    break;
  }
  AllowDurationSync = false;
  PWM_cond_change.notify_all();
  output << "Pushed to queue, Duration: " << duration_int_pwm << std::endl;
}


void ExecutiveLoop::updateState() {
  output << "UpdateState" << std::endl;
  while (loopIsRunning) {
    // Get the variables and put it into the state file.
    // timestamped every 0.1 seconds.
    std::lock_guard<std::mutex> sensorDataLock(sensor_mutex);
    // try ownslock for future testing
    stateFile << getCurrentDateTime() << ",";

    //    std::cout << depth_msg << " updateStateLocation" << " \n";
    stateFile << depth_pressure_msg;
    std::unique_lock<std::mutex> IMUlock(imu_mutex);
    stateFile << " ";
    stateFile << angular_velocity_x << ", " << angular_velocity_y << ", "
              << angular_velocity_z << ", " << linear_acceleration_x << ", "
              << linear_acceleration_y << ", " << linear_acceleration_z << ", ";
    IMUlock.unlock();
    stateFile << mag_field_x << ", " << mag_field_y << ", " << mag_field_z
              << ", PWM :[";

    std::unique_lock<std::mutex> pwmValuesLock(command_mutex);
    for (auto i : (currentCommand_ptr->getPwms().pwm_signals)) {
      stateFile << i << ", ";
    }
    stateFile << "],";
    pwmValuesLock.unlock();
    stateFile << "\n";
    if (stateFile.tellp() > 200) {
      stateFile.flush();
      stateFile.clear();
      stateFile.seekp(0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_WAIT_TIME));
  }
}

void ExecutiveLoop::replaceCurrentCommand() {
  std::unique_lock<std::mutex> CurrentpwmValuesLock(
    command_mutex);
  currentCommand_ptr = std::move(ManualPWMQueue.front());
  ManualPWMQueue.pop();
  std::unique_lock<std::mutex> thrusterCommandLock(thruster_mutex);
  isRunningThrusterCommand = true;
}


void ExecutiveLoop::executeDecisionLoop() {
  while (loopIsRunning) {
    std::unique_lock<std::mutex> Manual_Lock(Manual_Mutex);
    if (isManualEnabled) {
      std::unique_lock<std::mutex> Manual_Override_Lock(Manual_Override_mutex);
      if (isManualOverride) {
        //if the sendThrusterCommand is currently running or has a task.
        if (isRunningThrusterCommand) {
          output << "manual override" << std::endl;
          haltCurrentCommand();
          replaceCurrentCommand();
          //override was successful; revert back.
          isManualOverride = false;
        }
      }
      typeOfExecute = "blind_execute";
      std::unique_lock<std::mutex> QueuepwmValuesLock(Queue_pwm_mutex,
                                                      std::defer_lock);
      PWM_cond_change.wait(QueuepwmValuesLock,
                            [this] { return !(ManualPWMQueue.size() == 0); });
      // Add comment here below and above.
      if (!isRunningThrusterCommand) {
        replaceCurrentCommand();
      }else{
        
      }
    }
  }
  // Need to see William's python code to move foward.
}
// if all decisions/tasks are done, make tasksCompleted true;


// Sends Commands to Thrusters with CommandInterpreter


void ExecutiveLoop::sendThrusterCommand(Pwm_Command& command) {
  while (loopIsRunning) {
    if (isRunningThrusterCommand) {
      output << "Send Thruster Command is doing its job" << std::endl;
      std::unique_lock<std::mutex> current_command_lock(
        command_mutex);
      command.execute(*commandInterpreter_ptr);
      std::unique_lock<std::mutex> statusThruster(thruster_mutex);
      isRunningThrusterCommand = false;
    }
    output << "Finished Thruster Command\n" << std::endl;
  }
}

bool ExecutiveLoop::returnStatus() { return loopIsRunning; }


bool ExecutiveLoop::returntasksCompleted() { return tasksCompleted; }


void ExecutiveLoop::executeFailCommands() {
  std::lock_guard<std::mutex> stateFileLock(sensor_mutex);
  stateFile << std::endl;
  stateFile.close();
  // loopIsRunning = false;
  output << "Shutting down Executive Loop, sensors are still reading."
            << std::endl;
}
/*
void ShutdownRobot(){
  if(FuncFailCommExecuted){
    //shutdown opreations
  }
}*/

std::string ExecutiveLoop::getCurrentDateTime() {
  time_t now = time(0);
  tm *localTime = localtime(&now);
  char buffer[80];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", localTime);
  return std::string(buffer);
}


//If a task is currently running, stop this task.
void ExecutiveLoop::haltCurrentCommand() {
  if (isRunningThrusterCommand) {
    //May have to have a mutex for this ptr.
    commandInterpreter_ptr->interruptTimed_Execute();
    std::lock_guard<std::mutex> statusThrusterLock(thruster_mutex);
    isRunningThrusterCommand = false;
  }
}
