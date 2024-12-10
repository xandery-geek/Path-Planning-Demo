#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QScrollArea>
#include <QRadioButton>
#include <QButtonGroup>
#include <QCheckBox>
#include "generatemap.h"
#include "prm.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
private:

    QLabel *width_label_;
    QLabel *height_label_;

    QLabel *start_coordinate_;
    QLabel *end_coordinate_;

    QLineEdit *width_edit_;
    QLineEdit *height_edit_;

    QPushButton *generate_button_;
    QPushButton *start_button_;
    QCheckBox *display_track_;
    QCheckBox *auto_mode_;

    QButtonGroup *radio_group_;
    QRadioButton *distance_button_; //the distance of path is the first factor
    QRadioButton *oil_button_;  //the amount of the usage of oil is the first factor

    QScrollArea *scroll_area_;
    GenerateMap *map_;

    QGroupBox *control_group_;
    QGroupBox *map_group_;

    bool is_start_= false;
    PRM prm;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void initWidget();
    void connectSigal();

private:
    void setStartButton(bool enable);

public slots:

    void onStartEndChange(const QPoint& start, const QPoint& end);
    void onAutoModeChanged(int state);
    void onGenerateButton();
    void onStartButton();
    void onDisplayButton();
    void onEditChange();
    void onAnimationFinished();
};

#endif // MAINWINDOW_H
