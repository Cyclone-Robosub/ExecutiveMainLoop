#include "Pwm_Command.hpp"

void Untimed_Command::execute(Command_Interpreter_RPi5& commandInterpreter) {
    commandInterpreter.untimed_execute(pwms);
}

Untimed_Command::Untimed_Command(pwm_array pwms) : Pwm_Command(pwms) {};

void Timed_Command::execute(Command_Interpreter_RPi5& commandInterpreter) {
    CommandComponent commandComponent = {pwms, duration};
    commandInterpreter.timed_execute(commandComponent);
}

Timed_Command::Timed_Command(pwm_array pwms, std::chrono::milliseconds duration)
    : Pwm_Command(pwms), duration(duration) {}
