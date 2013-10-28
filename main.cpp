#include "astarwin.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    AStarWin w;
    w.showMaximized();

    return a.exec();
}
