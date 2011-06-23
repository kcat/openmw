
#include "actionread.hpp"

#include "../mwgui/window_manager.hpp"

#include "environment.hpp"

namespace MWWorld
{
    ActionRead::ActionRead (const MWWorld::Ptr& object, const std::string& text, bool scroll)
    : mObject (object), mText (text), mScroll (scroll)
    {}

    void ActionRead::execute (Environment& environment)
    {
        /// \todo skill increment.
        environment.mWindowManager->viewDocument (mText, mScroll, mObject);
    }
}
