#pragma once

#include <memory>
#include <map>
#include <mutex>

namespace CubliMini {
namespace State {
class StateMachine;

class State
{
   public:
    virtual ~State()                           = default;
    virtual void enter(StateMachine &sm)       = 0;
    virtual void handleEvent(StateMachine &sm) = 0;
    virtual void exit(StateMachine &sm)        = 0;
};

typedef enum STATE_e
{
    IDIE,
};

class StateMachine
{
   public:
    void setState(STATE_e state)
    {
        if(states_.count(state) == 0)
        {
            return;
        }
        if (current_state_)
        {
            current_state_->exit(*this);
        }
        current_state_ = states_[state];
        if (current_state_)
        {
            current_state_->enter(*this);
        }
    }

    void handleEvent()
    {
        if (current_state_)
        {
            current_state_->handleEvent(*this);
        }
    }

   private:
    static std::map<STATE_e, std::shared_ptr<State>> states_;
    std::shared_ptr<State> current_state_;
};

}  // namespace State
}  // namespace CubliMini