#include "robot_events.h"











void EventsManager::add(Command command)
{
    Event event;
    event.type = EventType::command;
    event.command.command = command;
    events.push_back(event);
}
void EventsManager::add(Error error)
{
    has_error = true;
    Event event;
    event.type = EventType::error;
    event.error.code = error;
    events.push_back(event);
}
