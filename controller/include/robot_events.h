#pragma once

#include <vector>


// types of errors
enum class EventType
{
    none,
    error,
    warning,
    command,
};


// data types for error events
enum class Error
{

};
struct EventError
{
    EventType type;
    Error code;
};

enum class Warning
{
    ik_not_converge,
};
struct EventWarning
{
    EventType type;
    Warning code;
};

// data types for command events
enum class Command
{
    none,
    stop,
    reset,
    start_cart_velocity,
};
struct EventCommand
{
    EventType type;
    Command command;
};


// data type for a single event
union Event
{
    EventType       type;
    EventError      error;
    EventWarning    warning;
    EventCommand    command;
};


// Class to manage events
class EventsManager
{
public:
    std::vector<Event> events;
    bool has_error = false;
    bool has_warning = false;

    // adding events
    void add(Error error);
    void add(Warning warning);
    void add(Command command);

    // searching operators
    inline bool operator==(const Command& cmd);

    // mirror some vector functions
    inline bool empty() {return events.empty();}
    inline std::vector<Event>::iterator begin() {return events.begin();}
    inline std::vector<Event>::iterator end() {return events.end();}
    inline void clear() {events.clear(); has_error = false; has_warning = false;}
};



/** Check the given event if it is the given command
 *
 * @param e -> event to check
 * @param cmd -> the command to compare against
 */
inline bool operator==(const Event& e, const Command& cmd)
{
    return e.type == EventType::command && e.command.command == cmd;
}

/** Check the given command is in ANY of the events currently logged
 *
 * @param cmd -> the command to find
 */
inline bool EventsManager::operator==(const Command& cmd)
{
    for (auto& event : events)
        if (event == cmd)
            return true;
    return false;
}




