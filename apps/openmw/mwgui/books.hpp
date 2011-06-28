#ifndef MWGUI_BOOKS_H
#define MWGUI_BOOKS_H

#include <MyGUI.h>
#include "widgets.hpp"
#include "window_base.hpp"


namespace MWGui
{
    using namespace MyGUI;

    class WindowManager;

    class BookDialog : public WindowBase
    {
    public:

        BookDialog(WindowManager& parWindowManager);
        void Open(const std::string& text);

        void setPage(int p, const std::string& text);

        // Events
        typedef delegates::CDelegate1<bool> EventHandle_Bool;
        /**
        Event : "Close" or "Take" button clicked\n
        Signature : void method(bool taken)\n
        */
        EventHandle_Bool eventCloseOrTaken;

    private:

        MyGUI::WidgetPtr leftPageWidget, rightPageWidget;
        MyGUI::WidgetPtr prevBTNWidget, nextBTNWidget;
        MyGUI::WidgetPtr closeBTNWidget;

    protected:
        void onOkClicked(MyGUI::Widget* _sender);
    };

}


#endif




