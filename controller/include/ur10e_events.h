#pragma once

#include <vector>



// types of error events
enum class Error
{
    traj_action_err,        // Error relating to the joint action server from Universal Robots.
    target_computation_err, // Error relating to the computation of a new target position (probably a bug).
    unknown_state,          // State machine went to an unhandled state.
    robot_wrong_mode,       // The mode reported by the robot is not right for the current state of this controller.
    program_load,           // The robot could not load the program.
    program_play,           // The robot could not play the program.
};

// types of command events
enum class Command
{
    init,                   // initialize the robot
    stop,                   // stop the current trajectory or motion mode
    reset,                  // reset errors and allow further robot control
    start_home,             // start the homing procedure
    start_cart_velocity,    // start the joystick control mode
    start_joint_positiom,   // start the joint position mode
    done,                   // the current task is done, probably return to standstill state
};


// Class to manage events
class EventsManager
{
public:
    std::vector<Error> errors;
    std::vector<Command> commands;

    // register a new error
    inline void add(Error error) {errors.push_back(error);}

    // register a new command
    inline void add(Command command) {commands.push_back(command);}

    // Check if there are errors registered
    inline bool has_error() {return !errors.empty();}

    // check if the given command is in the registered events
    inline bool operator==(const Command& cmd);

    // clear everything except errors
    inline void clear() {commands.clear();}

    // clear errors
    inline void clear_errors() {errors.clear();}
};


inline bool EventsManager::operator==(const Command& cmd)
{
    for (auto& event : commands)
        if (event == cmd)
            return true;
    return false;
}




