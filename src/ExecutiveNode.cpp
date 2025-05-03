#include "ExecutiveLoop.h"
#include <iostream>

ExecutiveLoop::ExecutiveLoop(std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr, 
                           std::shared_ptr<std::pair<pwm_array, std::chrono::milliseconds>> currentPWMandDuration_ptr, 
                           std::ofstream& stateFile, std::ostream& output, std::ostream& error)
    : Node("executive_main_node"),
      commandInterpreter_ptr(std::move(commandInterpreter_ptr)), 
      currentPWMandDuration_ptr(currentPWMandDuration_ptr), 
      stateFile(stateFile), 
      output(output), 
      error(error), 
      loopIsRunning(true), 
      tasksCompleted(true) {}

void ExecutiveLoop::ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  std::unique_lock<std::mutex> Manual_Lock(Manual_Mutex);
  isManualEnabled = msg->data;
  
  if (isManualEnabled) {
    std::cout << "Manual Control Enabled" << std::endl;
  } else {
    std::cout << "Manual Control Disabled" << std::endl;
    std::lock_guard<std::mutex> QueueLock(Queue_pwm_mutex);
    std::queue<std::pair<pwm_array, std::chrono::milliseconds>> empty;
    std::swap(ManualPWMQueue, empty);
    std::cout << "Manual Command Current Override -> Deleted Queue" << std::endl;
    sizeQueue = 0;
  }
  
  Change_Manual.notify_all();
}

void ExecutiveLoop::ManualOverrideCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  std::unique_lock<std::mutex> Manual_Lock(Manual_Override_mutex);
  isManualOverride = msg->data;
  
  if (isManualOverride) {
    std::lock_guard<std::mutex> QueueLock(Queue_pwm_mutex);
    std::queue<std::pair<pwm_array, std::chrono::milliseconds>> empty;
    std::swap(ManualPWMQueue, empty);
    std::cout << "Manual Command Current Override -> Deleted Queue" << std::endl;
    sizeQueue = 0;
  }
}

void ExecutiveLoop::depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg) {
  depth_pressure_msg = msg->data;
}

void ExecutiveLoop::imuSensorCallback(const sensor_msgs::msg::Imu &msg) {
  std::lock_guard<std::mutex> CallBacklock(imu_mutex);
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

void ExecutiveLoop::PWMArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  std::cout << "Received Int32MultiArray: ";
  int i = 0;
  int setvalue;
  std::unique_lock<std::mutex> pwm_lock(array_duration_sync_mutex);
  
  for (int32_t value : msg->data) {
    setvalue = (int)value;
    given_array.pwm_signals[i] = setvalue;
    std::cout << given_array.pwm_signals[i];
    ++i;
  }
  
  AllowDurationSync = true;
  std::cout << std::endl;
  SendToDuration_change.notify_all();
  pwm_lock.unlock();
}

void ExecutiveLoop::durationCallback(const std_msgs::msg::Int64::SharedPtr msg) {
  std::unique_lock<std::mutex> duration_lock(array_duration_sync_mutex, std::defer_lock);
  SendToDuration_change.wait(duration_lock, [this] { return AllowDurationSync; });
  
  std::cout << "Getting duration" << std::endl;
  auto duration_int_pwm = msg->data;
  std::chrono::milliseconds durationMS;
  bool isgivenTimed = false;
  
  switch (duration_int_pwm) {
    case -1:
      durationMS = std::chrono::milliseconds(9999999999);
      std::cout << durationMS << std::endl;
      break;
    default:
      durationMS = std::chrono::milliseconds(duration_int_pwm * 1000);
      isgivenTimed = true;
      std::cout << durationMS << std::endl;
      break;
  }
  
  std::unique_lock<std::mutex> Queue_sync_lock(Queue_pwm_mutex);
  ManualPWMQueue.push(std::make_pair(given_array, durationMS));
  sizeQueue++;
  
  if(isgivenTimed){
    pwm_array stop_set_array;
    for (int i = 0; i < 8; i++) {
      stop_set_array.pwm_signals[i] = 1500;
    }
    std::pair<pwm_array, std::chrono::milliseconds> stop_set_pair(
        stop_set_array, std::chrono::milliseconds(99999999));
    ManualPWMQueue.push(stop_set_pair);
    sizeQueue++;
  }
  
  AllowDurationSync = false;
  PWM_cond_change.notify_all();
  std::cout << "Pushed to queue, Duration: " << duration_int_pwm << std::endl;
}

void ExecutiveLoop::updateState() {
  std::cout << "UpdateState" << std::endl;
  while (loopIsRunning) {
    std::lock_guard<std::mutex> sensorDataLock(sensor_mutex);
    
    stateFile << getCurrentDateTime() << ",";
    stateFile << depth_pressure_msg;
    
    std::unique_lock<std::mutex> IMUlock(imu_mutex);
    stateFile << " ";
    stateFile << angular_velocity_x << ", " << angular_velocity_y << ", "
              << angular_velocity_z << ", " << linear_acceleration_x << ", "
              << linear_acceleration_y << ", " << linear_acceleration_z << ", ";
    IMUlock.unlock();
    
    stateFile << mag_field_x << ", " << mag_field_y << ", " << mag_field_z
              << ", PWM :[";
    
    std::unique_lock<std::mutex> pwmValuesLock(current_PWM_duration_mutex);
    for (auto i : currentPWMandDuration_ptr->first.pwm_signals) {
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

void ExecutiveLoop::executeDecisionLoop() {
  while (loopIsRunning) {
    std::unique_lock<std::mutex> Manual_Lock(Manual_Mutex);
    
    if (isManualEnabled) {
      std::unique_lock<std::mutex> Manual_Override_Lock(Manual_Override_mutex);
      if (isManualOverride) {
        if (isRunningThrusterCommand) {
          std::cout << "manual override" << std::endl;
          override();
          isManualOverride = false;
        }
      }
      
      typeOfExecute = "blind_execute";
      std::unique_lock<std::mutex> QueuepwmValuesLock(Queue_pwm_mutex, std::defer_lock);
      PWM_cond_change.wait(QueuepwmValuesLock, [this] { return !(sizeQueue == 0); });
      
      if (!isCurrentCommandTimedPWM) {
        override();
        std::cout << "Executive Decision: Replace Current PWM Command" << std::endl;
        
        if (ManualPWMQueue.front().second >= std::chrono::milliseconds(99999999)) {
          isCurrentCommandTimedPWM = false;
        } else {
          isCurrentCommandTimedPWM = true;
        }
        
        std::unique_lock<std::mutex> CurrentpwmValuesLock(current_PWM_duration_mutex);
        currentPWMandDuration_ptr = std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(ManualPWMQueue.front());
        CurrentpwmValuesLock.unlock();
        
        std::unique_lock<std::mutex> thrusterCommandLock(thruster_mutex);
        isRunningThrusterCommand = true;
        thrusterCommandLock.unlock();
        
        ManualPWMQueue.pop();
        sizeQueue--;
      }
      else if (!isRunningThrusterCommand) {
        std::cout << "2 Executor Decision: Needs a Command" << std::endl;
        
        if (ManualPWMQueue.front().second >= std::chrono::milliseconds(9999999)) {
          isCurrentCommandTimedPWM = false;
        } else {
          isCurrentCommandTimedPWM = true;
        }
        
        std::unique_lock<std::mutex> CurrentpwmValuesLock(current_PWM_duration_mutex);
        currentPWMandDuration_ptr = std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(ManualPWMQueue.front());
        
        std::unique_lock<std::mutex> statusThruster(thruster_mutex);
        isRunningThrusterCommand = true;
        statusThruster.unlock();
        
        CurrentpwmValuesLock.unlock();
        std::cout << "3 executor decision: Gave New Command" << std::endl;
        
        ManualPWMQueue.pop();
        sizeQueue--;
      }
    }
  }
}

void ExecutiveLoop::sendThrusterCommand() {
  std::ofstream logFilePins("PWM_LOGS.txt");
  
  while (loopIsRunning) {
    if (typeOfExecute == "blind_execute") {
      CommandComponent commandComponent;
      
      if (isRunningThrusterCommand) {
        std::cout << "Send Thruster Command is doing its job" << std::endl;
        
        std::unique_lock<std::mutex> CurrentpwmValuesLock(current_PWM_duration_mutex);
        commandComponent.thruster_pwms = currentPWMandDuration_ptr->first;
        commandComponent.duration = currentPWMandDuration_ptr->second;
        CurrentpwmValuesLock.unlock();
        
        commandInterpreter_ptr->blind_execute(commandComponent, logFilePins);
        std::cout << "Finished Thruster Command\n" << std::endl;
        
        std::unique_lock<std::mutex> statusThruster(thruster_mutex);
        isRunningThrusterCommand = false;
        statusThruster.unlock();
      }
    }
  }
}

bool ExecutiveLoop::returnStatus() { 
  return loopIsRunning; 
}

bool ExecutiveLoop::returntasksCompleted() { 
  return tasksCompleted; 
}

void ExecutiveLoop::executeFailCommands() {
  std::lock_guard<std::mutex> stateFileLock(sensor_mutex);
  stateFile << std::endl;
  stateFile.close();
  std::cout << "Shutting down Executive Loop, sensors are still reading." << std::endl;
}

std::string ExecutiveLoop::getCurrentDateTime() {
  time_t now = time(0);
  tm *localTime = localtime(&now);
  char buffer[80];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", localTime);
  return std::string(buffer);
}

void ExecutiveLoop::override() {
  if (isRunningThrusterCommand) {
    commandInterpreter_ptr->interruptBlind_Execute();
    
    pwm_array zero_set_array;
    for (int i = 0; i < 8; i++) {
      zero_set_array.pwm_signals[i] = 0;
    }
    
    std::pair<pwm_array, std::chrono::milliseconds> zero_set_pair(
        zero_set_array, std::chrono::milliseconds(100));
    
    std::unique_lock<std::mutex> pwmValuesLock(current_PWM_duration_mutex);
    currentPWMandDuration_ptr = std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(zero_set_pair);
    pwmValuesLock.unlock();
    
    isCurrentCommandTimedPWM = false;
    
    std::lock_guard<std::mutex> statusThrusterLock(thruster_mutex);
    isRunningThrusterCommand = false;
  }
}
