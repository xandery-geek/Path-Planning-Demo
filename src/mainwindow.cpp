#include "mainwindow.h"
#include "custom_edit.h"
#include "evaluator.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSpacerItem>
#include <QMessageBox>
#include <QtConcurrent>
#include <QFuture>
#include <QRegularExpression>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    this->setWindowTitle(QObject::tr("title"));
    this->setWindowIcon(QIcon(QString(":/new/prefix1/image/robot_icon.png")));
    this->setMinimumSize(INIT_WIDTH_, INIT_HEIGHT_);
    this->showMaximized();

    initWidget();
    connectSigal();
}

MainWindow::~MainWindow()
{

}

void MainWindow::initWidget()
{
    width_label_ = new QLabel(QObject::tr("width"));
    width_label_->setFixedWidth(LABEL_WIDTH_);

    height_label_ = new QLabel(QObject::tr("height"));
    height_label_->setFixedWidth(LABEL_WIDTH_);

    trial_num_label_ = new QLabel(QObject::tr("trial_num"));
    trial_num_label_->setFixedWidth(LABEL_WIDTH_);

    width_edit_ = new QLineEdit("31");
    width_edit_->setFixedWidth(EDIT_WIDTH_);

    height_edit_ = new QLineEdit("21");
    height_edit_->setFixedWidth(EDIT_WIDTH_);

    trial_num_edit_ = new QLineEdit("10");
    trial_num_edit_->setFixedWidth(EDIT_WIDTH_);

    map_seed_label_ = new QLabel(QObject::tr("map_seed"));
    map_seed_label_->setFixedWidth(LABEL_WIDTH_);

    prm_seed_label_ = new QLabel(QObject::tr("prm_seed"));
    prm_seed_label_->setFixedWidth(LABEL_WIDTH_);

    map_seed_edit_ = new QLineEdit("0");
    map_seed_edit_->setFixedWidth(EDIT_WIDTH_);

    prm_seed_edit_ = new QLineEdit("0");
    prm_seed_edit_->setFixedWidth(EDIT_WIDTH_);

    QLabel *start_point = new QLabel(QObject::tr("start"));
    start_point->setFixedWidth(LABEL_WIDTH_);

    QLabel *end_point = new QLabel(QObject::tr("end"));
    end_point->setFixedWidth(LABEL_WIDTH_);

    start_coordinate_ = new QLabel("");
    start_coordinate_->setFixedWidth(EDIT_WIDTH_);
    end_coordinate_ = new QLabel("");
    end_coordinate_->setFixedWidth(EDIT_WIDTH_);

    distance_button_ = new QRadioButton(QObject::tr("distance_priority"));
    distance_button_->setFixedWidth(LABEL_WIDTH_);
    distance_button_->setChecked(true);
    energy_button_ = new QRadioButton(QObject::tr("energy_priority"));
    energy_button_->setFixedWidth(LABEL_WIDTH_);

    astar_button_ = new QRadioButton(QObject::tr("a_star"));
    astar_button_->setFixedWidth(LABEL_WIDTH_);
    astar_button_->setChecked(true);
    dijkstra_button_ = new QRadioButton(QObject::tr("dijkstra"));
    dijkstra_button_->setFixedWidth(LABEL_WIDTH_);

    radio_group1_ = new QButtonGroup;
    radio_group1_->addButton(distance_button_);
    radio_group1_->addButton(energy_button_);

    radio_group2_ = new QButtonGroup;
    radio_group2_->addButton(astar_button_);
    radio_group2_->addButton(dijkstra_button_);

    display_track_ = new QCheckBox(QObject::tr("display_track"));
    display_track_->setFixedWidth(LABEL_WIDTH_);
    display_track_->setCheckable(false);

    auto_mode_ = new QCheckBox(QObject::tr("auto_mode"));
    auto_mode_->setFixedWidth(LABEL_WIDTH_);

    generate_button_ = new QPushButton();
    generate_button_->setFixedSize(CIRCLR_BTN_SIZE_, CIRCLR_BTN_SIZE_);
    generate_button_->setToolTip(QObject::tr("generate_map"));
    setGenerateButton();

    start_button_ = new QPushButton();
    start_button_->setFixedSize(CIRCLR_BTN_SIZE_, CIRCLR_BTN_SIZE_);
    start_button_->setToolTip(QObject::tr("start_plan"));
    setStartButton(false);

    report_button_ = new QPushButton();
    report_button_->setFixedSize(CIRCLR_BTN_SIZE_, CIRCLR_BTN_SIZE_);
    report_button_->setToolTip(QObject::tr("generate_report"));
    setReportButton(true);

    log_viewer_ = new CustomPlainTextEdit();
    log_viewer_->setReadOnly(true);

    control_group_ = new QGroupBox(QObject::tr("control_panel"));
    map_group_ = new QGroupBox(QObject::tr("map"));

    map_ = new GenerateMap();

    scroll_area_ = new QScrollArea;
    // scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    // scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setWidget(map_);

    QHBoxLayout *hrizon_layout1 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout2 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout3 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout4 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout5 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout6 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout7 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout8 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout9 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout10 = new QHBoxLayout;
    QHBoxLayout *hrizon_layout11 = new QHBoxLayout;

    hrizon_layout1->addWidget(width_label_);
    hrizon_layout1->addWidget(width_edit_);

    hrizon_layout2->addWidget(height_label_);
    hrizon_layout2->addWidget(height_edit_);

    hrizon_layout11->addWidget(trial_num_label_);
    hrizon_layout11->addWidget(trial_num_edit_);

    hrizon_layout3->addWidget(start_point);
    hrizon_layout3->addWidget(start_coordinate_);

    hrizon_layout4->addWidget(end_point);
    hrizon_layout4->addWidget(end_coordinate_);

    hrizon_layout5->addWidget(distance_button_);
    hrizon_layout5->addWidget(energy_button_);

    hrizon_layout6->addWidget(astar_button_);
    hrizon_layout6->addWidget(dijkstra_button_);

    hrizon_layout7->addWidget(display_track_);
    hrizon_layout7->addWidget(auto_mode_);

    hrizon_layout8->addWidget(generate_button_);
    hrizon_layout8->addWidget(start_button_);
    hrizon_layout8->addWidget(report_button_);

    hrizon_layout9->addWidget(map_seed_label_);
    hrizon_layout9->addWidget(map_seed_edit_);

    hrizon_layout10->addWidget(prm_seed_label_);
    hrizon_layout10->addWidget(prm_seed_edit_);

    QVBoxLayout *vertical_layout1 = new QVBoxLayout;
    QVBoxLayout *vertical_layout2 = new QVBoxLayout;

    vertical_layout1->addLayout(hrizon_layout1);
    vertical_layout1->addLayout(hrizon_layout2);
    vertical_layout1->addLayout(hrizon_layout11);

    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Fixed));
    vertical_layout1->addLayout(hrizon_layout9);
    vertical_layout1->addLayout(hrizon_layout10);
    
    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Fixed));
    vertical_layout1->addLayout(hrizon_layout3);
    vertical_layout1->addLayout(hrizon_layout4);
    
    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Fixed));
    vertical_layout1->addLayout(hrizon_layout5);
    vertical_layout1->addLayout(hrizon_layout6);
    
    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Fixed));
    vertical_layout1->addLayout(hrizon_layout7);

    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Fixed));
    vertical_layout1->addLayout(hrizon_layout8);

    vertical_layout1->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Fixed));
    vertical_layout1->addWidget(log_viewer_);

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
    connect(trial_num_edit_, SIGNAL(textChanged(QString)), this, SLOT(onEditChange()));
    connect(map_seed_edit_, SIGNAL(textChanged(QString)), this, SLOT(onEditChange()));
    connect(prm_seed_edit_, SIGNAL(textChanged(QString)), this, SLOT(onEditChange()));
    connect(generate_button_, SIGNAL(clicked(bool)), this, SLOT(onGenerateButton()));
    connect(display_track_, SIGNAL(clicked(bool)), this, SLOT(onDisplayButton()));
    connect(auto_mode_, SIGNAL(stateChanged(int)), this, SLOT(onAutoModeChanged(int)));
    connect(start_button_, SIGNAL(clicked(bool)), this, SLOT(onStartButton()));
    connect(report_button_, SIGNAL(clicked(bool)), this, SLOT(onReportButton()));
    connect(map_, SIGNAL(startEndChanged(QPoint,QPoint)), this, SLOT(onStartEndChange(QPoint,QPoint)));
    connect(map_->animation_group_, SIGNAL(finished()), this, SLOT(onAnimationFinished()));
}

void MainWindow::setStartButton(bool enable)
{
    if(enable)
    {
        is_planning_path_ = false;
        start_button_->setEnabled(true);
        start_button_->setStyleSheet("QPushButton{border-image: url(:/new/prefix1/image/button2.png);}"
                                     "QPushButton:hover{border-image: url(:/new/prefix1/image/button3.png);}"
                                     "QPushButton:pressed{border-image: url(:/new/prefix1/image/button2.png);}");
    }
    else
    {
        is_planning_path_ = true;
        start_button_->setDisabled(true);
        start_button_->setStyleSheet("QPushButton{border-image: url(:/new/prefix1/image/button1.png);}");
    }
}

void MainWindow::setGenerateButton()
{
    generate_button_->setStyleSheet("QPushButton{border-image: url(:/new/prefix1/image/generate1.png);}"
                                    "QPushButton:hover{border-image: url(:/new/prefix1/image/generate2.png);}");
}

void MainWindow::setReportButton(bool enable)
{
    if(enable)
    {
        is_generating_report_ = false;
        report_button_->setEnabled(true);
        report_button_->setStyleSheet("QPushButton{border-image: url(:/new/prefix1/image/report2.png);}"
                                     "QPushButton:hover{border-image: url(:/new/prefix1/image/report3.png);}"
                                     "QPushButton:pressed{border-image: url(:/new/prefix1/image/report2.png);}");
    }
    else
    {
        is_generating_report_ = true;
        report_button_->setDisabled(true);
        report_button_->setStyleSheet("QPushButton{border-image: url(:/new/prefix1/image/report1.png);}");
        
        // Force Qt to process the pending events
        //TODO: This is a workaround to avoid the GUI freezing when generating the report. Maybe there is a better way to do this.
        QCoreApplication::processEvents();
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

QString MainWindow::generateReport() const
{
    Evaluator evaluator(
        width_edit_->text().toInt(),
        height_edit_->text().toInt(),
        trial_num_edit_->text().toInt(),
        map_seed_edit_->text().toInt(),
        prm_seed_edit_->text().toInt(),
        astar_button_->isChecked() ? PRM::Alg_AStar : PRM::Alg_Dijkstra,
        distance_button_->isChecked() ? PRM::Stra_DistanceFirst : PRM::Stra_EnergyFirst
    );

    evaluator.evaluate();
    return evaluator.getResults();
}

void MainWindow::onGenerateButton()
{
    //update map
    log_viewer_->appendPlainText("Generating map...");

    //set random seed
    int map_seed = map_seed_edit_->text().toInt();
    map_->setRandomSeed(map_seed);

    //generate map
    map_->creatMap(width_edit_->text().toInt(), height_edit_->text().toInt());
    map_->showMap();

    //update status flag
    is_planning_path_ = false;

    //update button status
    start_coordinate_->setText("");
    end_coordinate_->setText("");

    setStartButton(true);   //enable start button
    display_track_->setCheckable(false);    //disable display button
}

void MainWindow::onStartButton()
{
    if(is_planning_path_ == true)
    {
        return;
    }

    if(start_coordinate_->text() == "" || end_coordinate_->text() == "")
    {
        log_viewer_->appendPlainText("Please set start and end point");
        QMessageBox::about(this, "Error", "Please set start and end point");
        return;
    }

    bool astar_btn_checked = astar_button_->isChecked();
    bool distance_btn_checked = distance_button_->isChecked();
    
    PRM::Algorithm algorithm = astar_btn_checked? PRM::Alg_AStar : PRM::Alg_Dijkstra;
    PRM::Strategy strategy = distance_btn_checked? PRM::Stra_DistanceFirst : PRM::Stra_EnergyFirst;
    
    log_viewer_->appendPlainText(QString("Searching path...") 
                                + "\nAlgorithm: " + (astar_btn_checked ? "A Star": "Dijkstra") 
                                + "\nStrategy: " + (distance_btn_checked ? "Distance first" : "Energy first"));

    //set random seed
    int prm_seed = prm_seed_edit_->text().toInt();
    prm_.setRandomSeed(prm_seed);
    prm_.setStartPoint(map_->getStartPoint());   //set start point
    prm_.setEndPoint(map_->getEndPoint());      //set end point
    prm_.constructGraph(map_->getMapMatrix(), map_->getMapHeight(), map_->getMapWidth());
    
    try {
        // search path with selected algorithm and strategy
        prm_.searchPath(algorithm, strategy);
    } catch (std::runtime_error &e) {
        QMessageBox::about(this, "Error", e.what());
        return;
    }

    QVector<QPoint> points = prm_.getPath();
    if(points.empty())
    {
        log_viewer_->appendPlainText("Path not find");
        QMessageBox::about(this, "Not Find", "Path not find");
    }
    else
    {
        float cost = prm_.calPathCost(points, strategy);
        log_viewer_->appendPlainText("Path cost: " + QString::number(cost));

        map_->showRobot(points);    //show animation
        setStartButton(false);      //disable start button
        display_track_->setCheckable(true);
    }
}

void MainWindow::onReportButton()
{
    if(is_generating_report_ == true)
    {
        return;
    }
    setReportButton(false);
    log_viewer_->appendPlainText("Generating report...");
    
    QFuture<QString> future = QtConcurrent::run([this]() { return generateReport(); });
        future.then([this](const QString &result) {
            onReportFinished(result); // Handle the result directly
    });
}

void MainWindow::onReportFinished(const QString &report)
{
    setReportButton(true);
    log_viewer_->appendPlainText(report);
}

void MainWindow::onDisplayButton()
{
    if(display_track_->isChecked()) //show path
    {
        Graph graph;
        QVector<QPoint> points;

        graph = prm_.getGraph();
        points = prm_.getPath();

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
