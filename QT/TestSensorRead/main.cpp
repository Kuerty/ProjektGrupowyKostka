#include "dialog.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Dialog w;
    w.setFixedSize(464,388);
    w.setWindowTitle("GUI kostki");
    w.show();

    return a.exec();
}
