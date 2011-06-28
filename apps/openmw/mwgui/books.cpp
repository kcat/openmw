#include "books.hpp"
#include "window_manager.hpp"

using namespace MWGui;

BookDialog::BookDialog(WindowManager& parWindowManager)
    : WindowBase("openmw_book_window_layout.xml", parWindowManager)
{
    getWidget(leftPageWidget, "LeftPage");
    getWidget(rightPageWidget, "RightPage");
    getWidget(prevBTNWidget, "PrevPageBTN");
    getWidget(nextBTNWidget, "NextPageBTN");
    getWidget(closeBTNWidget, "CloseBTN");

    closeBTNWidget->eventMouseButtonClick = MyGUI::newDelegate(this, &BookDialog::onOkClicked);
}


void BookDialog::Open(const std::string& text)
{
    setPage(1, text);
    setVisible(true);
}


void BookDialog::setPage(int p, const std::string& text){
    if (p == 1) leftPageWidget->setCaption(text);
    else if (p == 2) rightPageWidget->setCaption(text);
}


/*
----------------------------------------------------------------------------------------------------
EVENTS
----------------------------------------------------------------------------------------------------
*/

void BookDialog::onOkClicked(MyGUI::Widget* _sender){
    eventCloseOrTaken(false);
}








