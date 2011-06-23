#ifndef GAME_MWWORLD_ACTIONREAD_H
#define GAME_MWWORLD_ACTIONREAD_H

#include "action.hpp"
#include "ptr.hpp"

namespace MWWorld
{
    class ActionRead : public Action
    {
            MWWorld::Ptr mObject;
            std::string mText;
            bool mScroll;

        public:

            ActionRead (const MWWorld::Ptr& object, const std::string& text, bool scroll);

            virtual void execute (Environment& environment);
    };
}

#endif
