#pragma once

#include <chrono>
#include "lib/Executive_Propulsion/lib/Command.h" // for pwm_array.
#include "lib/Executive_Propulsion/lib/Command_Interpreter.h"

class Pwm_Command {
protected:
    pwm_array pwms;
public:
    virtual void execute(Command_Interpreter_RPi5& commandInterpreter);
    explicit Pwm_Command(pwm_array pwms) : pwms(pwms) {};
};

class Untimed_Command : public Pwm_Command {
public:
    void execute(Command_Interpreter_RPi5& commandInterpreter) override;
    explicit Untimed_Command(pwm_array pwms);
};

class Timed_Command : public Pwm_Command {
private:
    std::chrono::milliseconds duration;
public:
    void execute(Command_Interpreter_RPi5& commandInterpreter) override;
    explicit Timed_Command(pwm_array pwms);
};