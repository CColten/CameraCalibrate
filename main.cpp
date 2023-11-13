#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w.setWindowTitle("车舱点位标定");
    w.setFixedSize(1280, 800);
    return a.exec();
}
