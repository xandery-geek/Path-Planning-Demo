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
#include "custom_edit.h"
#include "prm.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
private:
    const int INIT_WIDTH_ = 800;
    const int INIT_HEIGHT_ = 600;
    const int LABEL_WIDTH_ = 100;
    const int EDIT_WIDTH_ = 100;
    const int CIRCLR_BTN_SIZE_ = 40;

    QLabel *width_label_;
    QLabel *height_label_;
    QLabel *trial_num_label_;

    QLabel *start_coordinate_;
    QLabel *end_coordinate_;

    QLabel *map_seed_label_;
    QLabel *prm_seed_label_;

    QLineEdit *width_edit_;
    QLineEdit *height_edit_;
    QLineEdit *trial_num_edit_;

    QLineEdit *map_seed_edit_;
    QLineEdit *prm_seed_edit_;

    QPushButton *generate_button_;
    QPushButton *start_button_;
    QPushButton *report_button_;

    QCheckBox *display_track_;
    QCheckBox *auto_mode_;

    QButtonGroup *radio_group1_;
    QButtonGroup *radio_group2_;
    QRadioButton *distance_button_; //the distance of path is the first factor
    QRadioButton *energy_button_;  //the amount of the usage of oil is the first factor
    QRadioButton *astar_button_;   //use A* algorithm to search path
    QRadioButton *dijkstra_button_;    //use Dijkstra algorithm to search path

    CustomPlainTextEdit *log_viewer_;

    QScrollArea *scroll_area_;
    GenerateMap *map_;

    QGroupBox *control_group_;
    QGroupBox *map_group_;

    bool is_planning_path_ = false;
    bool is_generating_report_ = false;

    PRM prm_;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void initWidget();
    void connectSigal();

private:
    void setStartButton(bool enable);
    void setGenerateButton();
    void setReportButton(bool enable);
    QString generateReport() const;
    void onReportFinished(const QString& report);

public slots:
    void onStartEndChange(const QPoint& start, const QPoint& end);
    void onAutoModeChanged(int state);
    void onGenerateButton();
    void onReportButton();
    void onStartButton();
    void onDisplayButton();
    void onEditChange();
    void onAnimationFinished();
};

#endif // MAINWINDOW_H
