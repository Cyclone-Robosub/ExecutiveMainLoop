#pragma once

#include <chrono>
#include "Command.hpp"
#include "Command_Interpreter.hpp"

const pwm_array stop_set_array = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

class Pwm_Command {
protected:
    pwm_array pwms;
public:
    virtual void execute(Command_Interpreter_RPi5& commandInterpreter) = 0;
    virtual pwm_array getPwms() {return pwms;};
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
    explicit Timed_Command(pwm_array pwms, std::chrono::milliseconds duration);
};
