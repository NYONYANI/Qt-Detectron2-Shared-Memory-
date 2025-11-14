#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qDebug() << "--- Main: Creating MainWindow object..."; // <-- CHECKPOINT 1
    MainWindow w;
    qDebug() << "--- Main: MainWindow object created. Showing window..."; // <-- CHECKPOINT 6
    w.show();
    return a.exec();
}
