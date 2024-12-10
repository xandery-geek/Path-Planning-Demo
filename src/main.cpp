#include "mainwindow.h"
#include <QApplication>
#include <QTranslator>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QTranslator translator;
    QString locale = QLocale::system().name();

    if (locale.startsWith("zh")) {
        bool ret = translator.load(":/i18n/lang/zh_CN.qm");
        if (!ret) {
            qDebug() << "load zh_CN.qm failed";
        }
    } else {
        bool ret = translator.load(":/i18n/lang/en_US.qm");
        if (!ret) {
            qDebug() << "load en_US.qm failed";
        }
    }
    app.installTranslator(&translator);

    MainWindow w;
    w.show();

    return app.exec();
}
