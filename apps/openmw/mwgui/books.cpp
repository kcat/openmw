#include "stats_window.hpp"
#include "window_manager.hpp"

using namespace MWGui;

BookWindow:BookWindow(WindowManager& parWindowManager)
    : WindowBase("openmw_book_window_layout.xml", parWindowManager)
{
    const ESMS::ESMStore &store = mWindowManager.getStore();

    getWidget(leftPageWidget, "LeftPage");
    getWidget(rightPageWidget, "RightPage");
    getWidget(prevBTNWidget, "PrevPageBTN");
    getWidget(nextBTNWidget, "NextPageBTN");
    getWidget(closeBTNWidget, "CloseBTN");
}








