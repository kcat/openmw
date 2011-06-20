#ifndef MWGUI_BOOKS_H
#define MWGUI_BOOKS_H

#include <components/esm_store/store.hpp>

#include <sstream>
#include <set>
#include <string>
#include <utility>

#include "window_base.hpp"


namespace MWGui
{

    class WindowManager;

    class BookWindow : public WindowBase
    {
        public:

        BookWindow(WindowManager& parWindowManager);



        private:

        MyGUI::WidgetPtr leftPageWidget, rightPageWidget;
        MyGUI::WidgetPtr prevBTNWidget, nextBTNWidget;
        MyGUI::WidgetPtr closeBTNWidget;
    };

}


#endif




