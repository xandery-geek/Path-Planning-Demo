#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSpacerItem>
#include <QMessageBox>
#include <QRegularExpression>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    this->setWindowTitle(QObject::tr("title"));
    this->setWindowIcon(QIcon(QString(":/new/prefix1/image/robot_icon.png")));
    this->resize(800, 600);

    initWidget();
    connectSigal();
}

MainWindow::~MainWindow()
{

}

void MainWindow::initWidget()
{
    width_label_ = new QLabel(QObject::tr("width"));
    width_label_->setFixedWidth(50);

    height_label_ = new QLabel(QObject::tr("height"));
    height_label_->setFixedWidth(50);

    width_edit_ = new QLineEdit("31");
    width_edit_->setFixedWidth(100);

    height_edit_ = new QLineEdit("21");
    height_edit_->setFixedWidth(100);

    QLabel *start_point = new QLabel(QObject::tr("start"));
    start_point->setFixedWidth(50);

    QLabel *end_point = new QLabel(QObject::tr("end"));
    end_point->setFixedWidth(50);

    start_coordinate_ = new QLabel("");
    start_coordinate_->setFixedWidth(100);
    end_coordinate_ = new QLabel("");
    end_coordinate_->setFixedWidth(100);

    generate_button_ = new QPushButton(QObject::tr("generate_map"));
    generate_button_->setFixedWidth(100);

    display_track_ = new QCheckBox(QObject::tr("display_track"));
    display_track_->setFixedWidth(100);
    display_track_->setCheckable(false);

    auto_mode_ = new QCheckBox(QObject::tr("auto_mode"));
    auto_mode_->setFixedWidth(100);

    distance_button_ = new QRadioButton(QObject::tr("distance_priority"));
    distance_button_->setFixedWidth(200);
    distance_button_->setChecked(true);
    energy_button_ = new QRadioButton(QObject::tr("energy_priority"));
    energy_button_->setFixedWidth(200);

    radio_group_ = new QButtonGroup;
    radio_group_->addButton(distance_button_);
    radio_group_->addButton(energy_button_);

    start_button_ = new QPushButton();
    start_button_->setFixedSize(50, 50);
    setStartButton(false);

    control_group_ = new QGroupBox(QObject::tr("control_panel"));
    map_group_ = new QGroupBox(QObject::tr("map"));

    map_ = new GenerateMap();

    scroll_area_ = new QScrollArea;
    //scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    //scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setWidget(map_);

    QHBoxLayout *hrizon_layout1 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout2 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout3 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout4 = new QHBoxLayout;

    hrizon_layout1->addWidget(width_label_);
    hrizon_layout1->addWidget(width_edit_);

    hrizon_layout2->addWidget(height_label_);
    hrizon_layout2->addWidget(height_edit_);

    hrizon_layout3->addWidget(start_point);
    hrizon_layout3->addWidget(start_coordinate_);

    hrizon_layout4->addWidget(end_point);
    hrizon_layout4->addWidget(end_coordinate_);

    QVBoxLayout *vertical_layout1 = new QVBoxLayout;
    QVBoxLayout *vertical_layout2 = new QVBoxLayout;

    vertical_layout1->addLayout(hrizon_layout1);
    vertical_layout1->addLayout(hrizon_layout2);
    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Fixed));
    vertical_layout1->addLayout(hrizon_layout3);
    vertical_layout1->addLayout(hrizon_layout4);
    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Fixed));
    vertical_layout1->addWidget(distance_button_);
    vertical_layout1->addWidget(energy_button_);
    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Fixed));
    vertical_layout1->addWidget(display_track_, Qt::AlignRight);
    vertical_layout1->addWidget(auto_mode_, Qt::AlignCenter);
    vertical_layout1->addWidget(generate_button_, Qt::AlignRight);
    vertical_layout1->addWidget(start_button_, Qt::AlignRight);
    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Expanding));

    vertical_layout2->addWidget(scroll_area_);

    map_group_->setLayout(vertical_layout2);
    control_group_->setLayout(vertical_layout1);

    QHBoxLayout *main_layout = new QHBoxLayout;
    main_layout->addWidget(map_group_);
    main_layout->addWidget(control_group_);
    main_layout->setStretch(0, 4);
    main_layout->setStretch(1, 1);

    QWidget *central_widget = new QWidget;
    central_widget->setLayout(main_layout);

    this->setCentralWidget(central_widget);
}

void MainWindow::connectSigal()
{
    connect(width_edit_, SIGNAL(textChanged(QString)), this, SLOT(onEditChange()));
    connect(height_edit_, SIGNAL(textChanged(QString)), this, SLOT(onEditChange()));
    connect(generate_button_, SIGNAL(clicked(bool)), this, SLOT(onGenerateButton()));
    connect(display_track_, SIGNAL(clicked(bool)), this, SLOT(onDisplayButton()));
    connect(auto_mode_, SIGNAL(stateChanged(int)), this, SLOT(onAutoModeChanged(int)));
    connect(start_button_, SIGNAL(clicked(bool)), this, SLOT(onStartButton()));
    connect(map_, SIGNAL(startEndChanged(QPoint,QPoint)), this, SLOT(onStartEndChange(QPoint,QPoint)));
    connect(map_->animation_group_, SIGNAL(finished()), this, SLOT(onAnimationFinished()));
}

void MainWindow::setStartButton(bool enable)
{
    if(enable)
    {
        is_start_ = false;
        start_button_->setEnabled(true);
        start_button_->setStyleSheet("QPushButton{border-image: url(:/new/prefix1/image/button3.png);}"
                                     "QPushButton:hover{border-image: url(:/new/prefix1/image/button4.png);}"
                                     "QPushButton:pressed{border-image: url(:/new/prefix1/image/button2.png);}");
    }
    else
    {
        start_button_->setDisabled(true);
        start_button_->setStyleSheet("QPushButton{border-image: url(:/new/prefix1/image/button1.png);}");
    }
}

void MainWindow::onStartEndChange(const QPoint &start, const QPoint &end)
{
    if(start.x() != -1)
    {
        start_coordinate_->setText("(" + QString::number(start.x()) + "," + QString::number(start.y()) + ")");
    }
    if(end.x() != -1)
    {
        end_coordinate_->setText("(" + QString::number(end.x()) + "," + QString::number(end.y()) + ")");
    }
}

void MainWindow::onAutoModeChanged(int state)
{
    if(state == Qt::Checked)
    {
        connect(map_, SIGNAL(endChanged()), this, SLOT(onStartButton()));
    }
    else
    {
        disconnect(map_, SIGNAL(endChanged()), this, SLOT(onStartButton()));
    }
}

void MainWindow::onGenerateButton()
{
    //update map
    map_->creatMap(width_edit_->text().toInt(), height_edit_->text().toInt());
    map_->showMap();

    //update status flag
    is_start_ = false;

    //update button status
    start_coordinate_->setText("");
    end_coordinate_->setText("");

    setStartButton(true);   //enable start button
    display_track_->setCheckable(false);    //disable display button
}

void MainWindow::onStartButton()
{
    if(is_start_ == true)
    {
        return;
    }

    //start prm
    if(start_coordinate_->text() == "" || end_coordinate_->text() == "")
    {
        QMessageBox::about(this, "Error", "Please set start and end point");
        return;
    }

    prm.setStartPoint(map_->getStartPoint());    //set start point
    prm.setEndPoint(map_->getEndPoint());      //set end point
    prm.constructGraph(map_->getMapMatrix(), map_->getMapHeight(), map_->getMapWidth());

    if(distance_button_->isChecked())
    {
        prm.searchPath(false);   //search path by PRM, distance first
    }
    else
    {
        prm.searchPath(true);  //search path by PRM, the usage of oil first
    }

    QVector<QPoint> points = prm.getPath();

    if(points.empty())
    {
        QMessageBox::about(this, "Not Find", "Path not find");
    }
    else
    {
        map_->showRobot(points);    //show animation
        setStartButton(false);      //disable start button

        is_start_ = true;
        display_track_->setCheckable(true);
    }
}

void MainWindow::onDisplayButton()
{
    if(display_track_->isChecked()) //show path
    {
        Graph graph;
        QVector<QPoint> points;

        graph = prm.getGraph();
        points = prm.getPath();

        if(points.empty())
        {
            QMessageBox::about(this, "Not Find", "Path not find");
        }
        else
        {
            map_->showPath(graph, points);  // show the graph and path
        }
    }
    else    //delete path track, refreash map
    {
        map_->refreashMap();
        map_->showPoint();
        //map_->showRobot();
    }
}

void MainWindow::onEditChange()
{
    QLineEdit *line_edit = (QLineEdit*)sender();

    QRegularExpression rx("[0-9]+");

    if(!rx.match(line_edit->text()).hasMatch())
    {
       line_edit->setStyleSheet("background-color:red");
    }
    else
    {
       line_edit->setStyleSheet(0);
    }
}

void MainWindow::onAnimationFinished()
{
    setStartButton(true);
}
