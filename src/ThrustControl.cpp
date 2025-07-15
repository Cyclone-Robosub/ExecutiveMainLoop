#include "PWMController.h"
#include <sstream>

PWMController::PWMController(const std::string& node_name)
    : Node(node_name) {
  
  // Create publishers
  status_pub_ = this->create_publisher<std_msgs::msg::String>("pwm_status", 10);
  current_pwm_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("current_pwm", 10);
  
  // Create subscribers
  manual_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "manual_toggle_switch", 10,
      std::bind(&PWMController::manualControlCallback, this, std::placeholders::_1));
  
  manual_override_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "manualOverride", 10,
      std::bind(&PWMController::manualOverrideCallback, this, std::placeholders::_1));
  
  pwm_array_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "array_Cltool_topic", 10,
      std::bind(&PWMController::pwmArrayCallback, this, std::placeholders::_1));
  
  duration_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "duration_Cltool_topic", 10,
      std::bind(&PWMController::durationCallback, this, std::placeholders::_1));
  
  // Initialize current PWM to neutral
  setNeutralPWM();
  
  RCLCPP_INFO(this->get_logger(), "PWM Controller initialized");
}

bool PWMController::initialize() {
  try {
    // Setup hardware pins
    std::vector<int> physical_pins = {2, 3, 4, 5, 6, 7, 8, 9};
    for (auto pin : physical_pins) {
      thruster_pins_.push_back(new HardwarePwmPin(pin));
    }
    
    // Create command interpreter
    command_interpreter_ = std::make_unique<Command_Interpreter_RPi5>(
        thruster_pins_, std::vector<DigitalPin*>{});
    
    command_interpreter_->initializePins();
    
    // Start execution thread
    execution_thread_ = std::thread(&PWMController::executePWMLoop, this);
    
    publishStatus("PWM Controller initialized and running");
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize PWM Controller: %s", e.what());
    return false;
  }
}

void PWMController::shutdown() {
  is_running_ = false;
  queue_cv_.notify_all();
  duration_cv_.notify_all();
  
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
  
  // Clean up pins
  for (auto pin : thruster_pins_) {
    delete pin;
  }
  thruster_pins_.clear();
  
  publishStatus("PWM Controller shutdown");
  RCLCPP_INFO(this->get_logger(), "PWM Controller shutdown complete");
}

void PWMController::manualControlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  is_manual_enabled_ = msg->data;
  
  if (is_manual_enabled_) {
    RCLCPP_INFO(this->get_logger(), "Manual control enabled");
    publishStatus("Manual control enabled");
  } else {
    RCLCPP_INFO(this->get_logger(), "Manual control disabled");
    publishStatus("Manual control disabled");
    
    // Clear the command queue
    std::queue<std::pair<pwm_array, std::chrono::milliseconds>> empty;
    std::swap(command_queue_, empty);
  }
}

void PWMController::manualOverrideCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  is_manual_override_ = msg->data;
  
  if (is_manual_override_) {
    RCLCPP_INFO(this->get_logger(), "Manual override activated");
    publishStatus("Manual override activated");
    
    // Clear the command queue and override current command
    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::queue<std::pair<pwm_array, std::chrono::milliseconds>> empty;
    std::swap(command_queue_, empty);
    
    if (is_executing_command_) {
      overrideCurrentCommand();
    }
  }
}

void PWMController::pwmArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(sync_mutex_);
  
  if (msg->data.size() != NUM_THRUSTERS) {
    RCLCPP_WARN(this->get_logger(), "Received PWM array with incorrect size: %zu", msg->data.size());
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Received PWM array");
  
  // Copy PWM values to pending array
  for (size_t i = 0; i < msg->data.size(); i++) {
    pending_pwm_array_.pwm_signals[i] = msg->data[i];
  }
  
  allow_duration_sync_ = true;
  duration_cv_.notify_all();
}

void PWMController::durationCallback(const std_msgs::msg::Int64::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(sync_mutex_);
  
  // Wait for PWM array to be received
  duration_cv_.wait(lock, [this] { return allow_duration_sync_; });
  
  auto duration_ms = msg->data;
  std::chrono::milliseconds duration;
  bool is_timed = false;
  
  if (duration_ms == -1) {
    duration = std::chrono::milliseconds(INFINITE_DURATION);
  } else {
    duration = std::chrono::milliseconds(duration_ms * 1000);
    is_timed = true;
  }
  
  RCLCPP_INFO(this->get_logger(), "Received duration: %ld ms", duration_ms);
  
  // Add command to queue
  {
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
    command_queue_.push(std::make_pair(pending_pwm_array_, duration));
    
    // If timed command, add stop command after
    if (is_timed) {
      pwm_array stop_array;
      for (int i = 0; i < NUM_THRUSTERS; i++) {
        stop_array.pwm_signals[i] = NEUTRAL_PWM;
      }
      command_queue_.push(std::make_pair(stop_array, std::chrono::milliseconds(INFINITE_DURATION)));
    }
  }
  
  allow_duration_sync_ = false;
  queue_cv_.notify_all();
}

void PWMController::executePWMLoop() {
  std::ofstream log_file("PWM_LOGS.txt");
  
  while (is_running_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // Wait for commands in queue or shutdown
    queue_cv_.wait(lock, [this] { 
      return !command_queue_.empty() || !is_running_; 
    });
    
    if (!is_running_) {
      break;
    }
    
    if (!is_manual_enabled_) {
      continue;
    }
    
    // Get next command
    if (!command_queue_.empty()) {
      auto command = command_queue_.front();
      command_queue_.pop();
      lock.unlock();
      
      executeCommand(command);
    }
  }
  
  log_file.close();
}

void PWMController::executeCommand(const std::pair<pwm_array, std::chrono::milliseconds>& command) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  
  is_executing_command_ = true;
  current_command_ = command;
  current_pwm_array_ = command.first;
  
  // Publish current PWM values
  publishCurrentPWM();
  
  // Execute command
  CommandComponent component;
  component.thruster_pwms = command.first;
  component.duration = command.second;
  
  try {
    command_interpreter_->blind_execute(component, std::ofstream("PWM_LOGS.txt", std::ios::app));
    publishStatus("Command execution complete");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Command execution failed: %s", e.what());
    publishStatus("Command execution failed");
  }
  
  is_executing_command_ = false;
}

void PWMController::setNeutralPWM() {
  for (int i = 0; i < NUM_THRUSTERS; i++) {
    current_pwm_array_.pwm_signals[i] = NEUTRAL_PWM;
  }
}

void PWMController::overrideCurrentCommand() {
  if (is_executing_command_) {
    command_interpreter_->interruptBlind_Execute();
    
    // Set all thrusters to zero
    pwm_array zero_array;
    for (int i = 0; i < NUM_THRUSTERS; i++) {
      zero_array.pwm_signals[i] = ZERO_PWM;
    }
    
    // Execute zero command briefly
    CommandComponent zero_command;
    zero_command.thruster_pwms = zero_array;
    zero_command.duration = std::chrono::milliseconds(100);
    
    command_interpreter_->blind_execute(zero_command, std::ofstream("PWM_LOGS.txt", std::ios::app));
    
    is_executing_command_ = false;
    publishStatus("Command overridden");
  }
}

void PWMController::publishStatus(const std::string& status) {
  auto msg = std_msgs::msg::String();
  msg.data = status;
  status_pub_->publish(msg);
}

void PWMController::publishCurrentPWM() {
  auto msg = std_msgs::msg::Int32MultiArray();
  msg.data.resize(NUM_THRUSTERS);
  
  for (int i = 0; i < NUM_THRUSTERS; i++) {
    msg.data[i] = current_pwm_array_.pwm_signals[i];
  }
  
  current_pwm_pub_->publish(msg);
}
