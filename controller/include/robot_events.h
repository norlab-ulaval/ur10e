#pragma once

#include <vector>



// data types for error events
enum class Error
{
    none,
    traj_action_err,
    target_computation_err,
    unknown_state,
};

// data types for command events
enum class Command
{
    none,
    init,
    stop,
    reset,
    start_home,
    start_cart_velocity,
    start_joint_positiom,
    done,
};


// Class to manage events
class EventsManager
{
public:
    std::vector<Error> errors;
    std::vector<Command> commands;

    // adding events
    inline void add(Error error) {errors.push_back(error);}
    inline void add(Command command) {commands.push_back(command);}


    // searching operators
    inline bool has_error() {return !errors.empty();}
    inline bool operator==(const Command& cmd);

    // mirror some vector functions
    inline void clear() {commands.clear();}
    inline void clear_errors() {errors.clear();}
};


inline bool EventsManager::operator==(const Command& cmd)
{
    for (auto& event : commands)
        if (event == cmd)
            return true;
    return false;
}




