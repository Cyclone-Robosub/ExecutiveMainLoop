#include "Pwm_Command.hpp"

void Untimed_Command::execute(Command_Interpreter_RPi5& commandInterpreter) {
    commandInterpreter.untimed_execute(pwms);
}

explicit Untimed_Command::Untimed_Command(pwm_array pwms) : Pwm_Command(pwms) {};

void Timed_Command::execute(Command_Interpreter_RPi5& commandInterpreter) {
    CommandComponent commandComponent = {pwms, duration};
    commandInterpreter.blind_execute(commandComponent);
}

explicit Timed_Command::Timed_Command(pwm_array pwms): Pwm_Command(pwms) {}