#ifndef GENERATEMAP_H
#define GENERATEMAP_H

#include <QWidget>
#include <QPainter>
#include <QPixmap>
#include <QPoint>
#include <QTimer>
#include <QLabel>
#include <QSequentialAnimationGroup>
#include <QRandomGenerator>
#include "graph.h"

class GenerateMap: public QWidget
{
    Q_OBJECT

private:
    enum Dir_Enum
    {
        D_LEFT = 0,
        D_RIGHT = 1,
        D_UP = 2,
        D_DOWN = 3,
    };

    const int BASE_SIZE_ = 30;  //the size of basic unit of map.
    const int ROBOT_SIZE = 30;  //the size of robot
    const int FLAG_SIZE = 32;   //the size of start and end point
    const int PILE_COEFFICIENT_ = 3;    //pile coefficient of map
    const float EROSING_COEFFICIENT_ = 0.3;  // erosing coefficient of map
    const float SAND_COEFFICIENT_ = 0.05;   // coefficient of sand

    const QPoint LIMIT_WIDTH_;  //the min and max map width
    const QPoint LIMIT_HEIGHT_; //the min and max map height

    int **map_matrix_;
    int width_;
    int height_;
    QPoint start_point_;
    QPoint end_point_;

    QPoint paint_offset_;
    QPoint clicked_pos_;    //the position of left button clicked

    QPainter painter_;

    QPixmap *canvas_;        //canvas

    //resource images
    QPixmap road_image_;
    QPixmap wall_image_;
    QPixmap sand_image_;
    QPixmap start_image_;
    QPixmap end_image_;

    QLabel *robot_label_;

    QTimer *timer_;
    QRandomGenerator *random_gen_;

public:
    QSequentialAnimationGroup *animation_group_;

public:
    GenerateMap(QWidget* parent=nullptr);
    ~GenerateMap();

    void creatMap(int width, int height);    //creat image and save to map_matrix_
    void refreashMap(); //refreash map
    void showMap();     //show generated map in the widget
    void showPoint();   //showe start point and end point
    void showRobot(const QVector<QPoint>& points);   //show the track of robot
    void showPath(const Graph& graph, const QVector<QPoint>& points);    //show the path searched by prm

    const int** getMapMatrix() const;
    int getMapHeight() const;
    int getMapWidth() const;
    const QPoint getStartPoint() const;
    const QPoint getEndPoint() const;

    void setRandomSeed(int seed);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
    void loadImage();   //load resource image
    void setStartPoint(int x, int y);
    void setEndPoint(int x, int y);
    void setMapSize(int width, int height);  //limit the size of map

    void generateMaze(int pos_i, int pos_j);    //generate a maze which acted a basic map
    void pileObstacle(int pos_i, int pos_j, int dir, int pile_coeff, int* count);    //pile up small obstacle to generate large obstacle block
    void removeObstacleRandomly(float erosing_coeff);  //remove some obstacle in maze randomly
    void obstacleClustering();      //clustering, to generate large obstacle block
    void generateSand();    //genereate sand

    const QVector<int> existedRoad(const int **mat, int i, int j);   //check if there is a road
    const QVector<int> existedRoad(const int **mat, int i, int j, int dir);
    QVector<QPoint> *aroundRoad(const int **mat, int i, int j);

    int getMatrixSum(int** mat, int row, int col, int start_i, int start_j, int size);
    void setMatrixValue(int** mat,  int row, int col, int start_i, int start_j, int size, int value);

signals:
    void startEndChanged(const QPoint &start, const QPoint &end);
    void endChanged();

public slots:
    void changeCharacter();
};

#endif // GENERATEMAP_H
