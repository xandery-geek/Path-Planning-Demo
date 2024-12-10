#include "mainwindow.h"
#include <QApplication>
#include <QTranslator>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QTranslator translator;
    QString locale = QLocale::system().name();

    if (locale.startsWith("zh")) {
        bool ret = translator.load(":/translations/zh_CN.qm");
    } else {
        bool ret = translator.load(":/translations/en_US.qm");
    }
    app.installTranslator(&translator);

    MainWindow w;
    w.show();

    return app.exec();
}
