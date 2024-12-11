#include "generatemap.h"
#include <QDateTime>
#include <QTime>
#include <QMouseEvent>
#include <QRegion>
#include <QVector>
#include <QMessageBox>
#include <QColor>
#include <QPropertyAnimation>


GenerateMap::GenerateMap(QWidget *parent)
    :QWidget(parent),
      LIMIT_WIDTH_(11, 200), LIMIT_HEIGHT_(11, 100),
      paint_offset_(0, 0)
{
    map_matrix_ = nullptr;
    canvas_ = nullptr;

    animation_group_ = new QSequentialAnimationGroup(this);
    timer_ = new QTimer;

    robot_label_ = new QLabel(this);
    robot_label_->hide();
    robot_label_->setFixedSize(ROBOT_SIZE, ROBOT_SIZE);
    robot_label_->setStyleSheet("border-image: url(:/new/prefix1/image/robot.png);");

    random_gen_ = new QRandomGenerator(QDateTime::currentMSecsSinceEpoch() % UINT_MAX);

    connect(timer_, SIGNAL(timeout()), this, SLOT(changeCharacter()));
    loadImage();
}

GenerateMap::~GenerateMap()
{
    //release map_matrix_
    for(int i=0; i<height_; i++)
    {
        delete [] map_matrix_[i];
    }

    delete [] map_matrix_;
}

void GenerateMap::loadImage()
{
    road_image_.load(":/new/prefix1/image/road1.png");
    wall_image_.load(":/new/prefix1/image/wall1.png");
    sand_image_.load(":/new/prefix1/image/sand.jpg");
    start_image_.load(":/new/prefix1/image/start.png");
    end_image_.load(":/new/prefix1/image/end.png");
}

void GenerateMap::creatMap(int width, int height)
{
    //release map_matrix_
    if(map_matrix_ != nullptr)
    {
        for(int i=0; i<height_; i++)
        {
            delete [] map_matrix_[i];
        }

        delete [] map_matrix_;
    }

    this->setMapSize(width, height);

    //initlizlize map
    map_matrix_ = new int*[height_];
    for(int i=0; i<height_; i++)
    {
        map_matrix_[i] = new int[width_];
    }

    for(int i=0; i<height_; i++)
    {
        for(int j=0; j<width_;j++)
        {
            if(i % 2 == 0 || j % 2==0)
            {
                map_matrix_[i][j] = 1;  //wall
            }
            else
            {
                map_matrix_[i][j] = 0;  //road
            }
        }
    }

    map_matrix_[1][1] = 2;
    this->generateMaze(1, 1);

    for(int i=0; i<height_; i++)
    {
        for(int j=0; j<width_;j++)
        {
            if(map_matrix_[i][j] == 2)
            {
                map_matrix_[i][j] = 0;
            }
        }
    }

    int count = 1;
    for(int i=0; i<width_*height_*0.03; i++)
    {
        this->pileObstacle(1, 1, D_DOWN, random_gen_->bounded(PILE_COEFFICIENT_+1), &count);
    }

    this->removeObstacleRandomly(EROSING_COEFFICIENT_);
    this->obstacleClustering(); //waiting for improve
    this->generateSand();
}

//depth first search to generate maze
void GenerateMap::generateMaze(int pos_i, int pos_j)
{
    //arrive boarder
    if(pos_j < 0 || pos_j >= width_ || pos_i < 0 || pos_i >= height_)
    {
        return;
    }

    QVector<int> vec = existedRoad((const int**)map_matrix_, pos_i, pos_j);

    //no existed road
    if(vec.size() == 0)
    {
        return;
    }

    //find next road randomly
    for(int i=0; i < vec.size();)
    {
        int index = random_gen_->bounded(vec.size());
        switch(vec[index])
        {
        case D_LEFT:
            if(map_matrix_[pos_i][pos_j-2] != 2)
            {
                map_matrix_[pos_i][pos_j-1] = 2;
                map_matrix_[pos_i][pos_j-2] = 2;
                this->generateMaze(pos_i, pos_j-2);
            }
            break;
        case D_RIGHT:
            if(map_matrix_[pos_i][pos_j+2] != 2)
            {
                map_matrix_[pos_i][pos_j+1] = 2;
                map_matrix_[pos_i][pos_j+2] = 2;
                this->generateMaze(pos_i, pos_j+2);
            }
            break;
        case D_UP:
            if(map_matrix_[pos_i-2][pos_j] != 2)
            {
                map_matrix_[pos_i-1][pos_j] = 2;
                map_matrix_[pos_i-2][pos_j] = 2;
                this->generateMaze(pos_i-2, pos_j);
            }
            break;
        case D_DOWN:
            if(map_matrix_[pos_i+2][pos_j] != 2)
            {
                map_matrix_[pos_i+1][pos_j] = 2;
                map_matrix_[pos_i+2][pos_j] = 2;
                this->generateMaze(pos_i+2, pos_j);
            }
            break;
        }
        vec.remove(index);
    }
}

void GenerateMap::pileObstacle(int pos_i, int pos_j, int dir, int pile_coeff, int* count)
{
    //arrive boarder
    if(pos_j < 0 || pos_j >= width_ || pos_i < 0 || pos_i >= height_)
    {
        return;
    }

    //find road
    int next_i=0, next_j=0;
    QVector<int> vec;
    vec = existedRoad((const int**)map_matrix_, pos_i, pos_j, dir);

    if(vec.size() == 0)
    {
        map_matrix_[pos_i][pos_j] = 1;
        (*count)++;
        return;
    }

    for(int i=0; i<vec.size(); i++)
    {
        switch (vec[i])
        {
        case D_LEFT:
            next_i = pos_i;
            next_j = pos_j-1;
            break;
        case D_RIGHT:
            next_i = pos_i;
            next_j = pos_j+1;
            break;
        case D_UP:
            next_i = pos_i-1;
            next_j = pos_j;
            break;
        case D_DOWN:
            next_i = pos_i+1;
            next_j = pos_j;
            break;
        }

        this->pileObstacle(next_i, next_j, vec[i], pile_coeff, count);
    }
}

void GenerateMap::removeObstacleRandomly(float erosing_coeff)
{
    int index_i = 0, index_j = 0;

    for(int count=0; count < erosing_coeff*width_*height_;)
    {
        index_i = random_gen_->bounded(height_-2) + 1;
        index_j = random_gen_->bounded(width_-2) + 1;

        //remove obstacle which is bordered by road randomly
        if(map_matrix_[index_i][index_j] == 1 &&
          (map_matrix_[index_i-1][index_j] == 0 || map_matrix_[index_i+1][index_j]==0 ||
           map_matrix_[index_i][index_j-1] == 0 || map_matrix_[index_i][index_j+1] == 0))
        {
            map_matrix_[index_i][index_j] = 0;
            count++;
        }
    }
}

void GenerateMap::obstacleClustering()
{
    int core_size = 3;

    int count=0;

    for(int i=1; i< height_-1; i++)
    {
        for(int j=1; j <width_-1; j++)
        {
            count = getMatrixSum(map_matrix_, height_, width_, i, j, core_size);
            if(count > 7)
            {
                map_matrix_[i][j] = 1;
            }
        }
    }

    for(int i=1; i< height_-1; i++)
    {
        for(int j=1; j <width_-1; j++)
        {
            count = getMatrixSum(map_matrix_, height_, width_, i, j, core_size);
            if(count < 3)
            {
                map_matrix_[i][j] = 0;
                //setMatrixValue(map_matrix_, height_, width_, i, j, core_size, 0);
            }
        }
    }
}

void GenerateMap::generateSand()
{
    int n = SAND_COEFFICIENT_*width_*height_;

    int index_i=0, index_j =0;
    int count = 0;
    QVector<QPoint> *vec;

    while(n)
    {
        index_i = random_gen_->bounded(height_-2) + 1;
        index_j = random_gen_->bounded(width_-2) + 1;

        vec = aroundRoad((const int**)map_matrix_, index_i, index_j);

        if(vec->size())
        {
            count = random_gen_->bounded(vec->size());
            while(count--)
            {
                map_matrix_[vec->at(0).x()][vec->at(0).y()] = 2;
                vec->pop_front();
            }

            n--;
        }

        delete vec;
    }
}

const QVector<int> GenerateMap::existedRoad(const int **mat, int i, int j)
{
    QVector<int> vec;

    if(j-2 >= 0 && mat[i][j-2] == 0)
    {
        vec.push_back(D_LEFT);
    }

    if(j+2 < width_ && mat[i][j+2] == 0)
    {
        vec.push_back(D_RIGHT);
    }

    if(i-2 >= 0 && mat[i-2][j] == 0)
    {
        vec.push_back(D_UP);
    }

    if(i+2 < height_ && mat[i+2][j] == 0)
    {
        vec.push_back(D_DOWN);
    }

    return vec;
}

const QVector<int> GenerateMap::existedRoad(const int **mat, int i, int j, int dir)
{
    QVector<int> vec;

    //left
    if(j-1 >= 0 && dir != D_RIGHT && mat[i][j-1] == 0)
    {
        vec.push_back(D_LEFT);
    }

    //right
    if(j+1 < width_ && dir != D_LEFT && mat[i][j+1] == 0)
    {
        vec.push_back(D_RIGHT);
    }

    //up
    if(i-1 > 0 && dir != D_DOWN && mat[i-1][j] == 0)
    {
        vec.push_back(D_UP);
    }

    //down
    if(i+1 > 0 && dir != D_UP && mat[i+1][j] == 0)
    {
        vec.push_back(D_DOWN);
    }

    return vec;
}

QVector<QPoint> *GenerateMap::aroundRoad(const int **mat, int i, int j)
{
    QVector<QPoint> *vec = new QVector<QPoint>;
    for(int u=i-1; u<=i+1;u++)
    {
        for(int v=j-1; v<= j+1; v++)
        {
            if(mat[u][v] == 0)
            {
                vec->push_back(QPoint(u,v));
            }
        }
    }

    return vec;
}

int GenerateMap::getMatrixSum(int **mat, int row, int col, int start_i, int start_j, int size)
{
    int scale = (size-1)/2;

    assert(start_i - scale >= 0 && start_i + scale <= row &&
           start_j - scale >= 0 && start_j + scale <= col);

    int count = 0;
    for(int i = start_i - scale; i <= start_i + scale; i++)
    {
        for(int j = start_j - scale; j <= start_j + scale; j++)
        {
            count += mat[i][j];
        }
    }
    return count;
}

void GenerateMap::setMatrixValue(int **mat, int row, int col, int start_i, int start_j, int size, int value)
{
    int scale = (size-1)/2;

    assert(start_i - scale >= 0 && start_i + scale <= row &&
           start_j - scale >= 0 && start_j + scale <= col);

    for(int i=start_i - scale; i<=start_i + scale; i++)
    {
        for(int j=start_j - scale; j<=start_j + scale; j++)
        {
            mat[i][j] = value;
        }
    }
}

void GenerateMap::changeCharacter()
{

}

void GenerateMap::showMap()
{
    this->resize(width_*BASE_SIZE_, height_*BASE_SIZE_);    //adjust widget size
    this->refreashMap();
    this->robot_label_->hide();

    setStartPoint(-1, -1);
    setEndPoint(-1, -1);
}

void GenerateMap::showRobot(const QVector<QPoint> &points)
{
    animation_group_->clear();
    timer_->stop();

    for(int i=0; i<points.size()-1; i++)
    {
        QPropertyAnimation *animation = new QPropertyAnimation(robot_label_, "geometry");

        animation->setDuration(1000);
        animation->setStartValue(QRect(paint_offset_.x() + points[i].x()*BASE_SIZE_ + (BASE_SIZE_ - ROBOT_SIZE)/2,
                                        paint_offset_.y() + points[i].y()*BASE_SIZE_ + (BASE_SIZE_ - ROBOT_SIZE)/2,
                                       ROBOT_SIZE, ROBOT_SIZE));

        animation->setEndValue(QRect(paint_offset_.x() + points[i+1].x()*BASE_SIZE_ + (BASE_SIZE_ - ROBOT_SIZE)/2,
                                      paint_offset_.y() + points[i+1].y()*BASE_SIZE_ + (BASE_SIZE_ - ROBOT_SIZE)/2,
                                        ROBOT_SIZE, ROBOT_SIZE));

        animation->setEasingCurve(QEasingCurve::Linear);

        animation_group_->addAnimation(animation);
    }
    robot_label_->show();
    animation_group_->start();
}

void GenerateMap::showPath(const Graph &graph, const QVector<QPoint> &points)
{
    if(canvas_ == nullptr)
    {
        return;
    }

    QVector<Graph::Vertex> vertex;
    vertex = graph.getVertex();

    QPainter painter;
    painter.begin(canvas_);

    //draw points
    QPen pen;
    pen.setColor(QColor(0, 0, 255));    //blue
    pen.setWidth(2);
    painter.setPen(pen);

    QVector<Graph::Vertex>::const_iterator it1 = vertex.cbegin();
    for(; it1!=vertex.cend(); it1++)
    {
        painter.drawEllipse(it1->pos.x()*BASE_SIZE_ + BASE_SIZE_/2,
                            it1->pos.y()*BASE_SIZE_+ BASE_SIZE_/2, 5, 5);
    }

    //drow graph
    pen.setColor(QColor(0, 255, 0));    //green
    painter.setPen(pen);

    for(int i=0; i<vertex.size(); i++)
    {
        const Graph::Arc *arc = vertex[i].head_arc;
        while (arc)
        {
            if(arc->vertex_index >= 0 && arc->vertex_index < vertex.size())
            {
                painter.drawLine(QPoint(vertex[i].pos.x()* BASE_SIZE_ + BASE_SIZE_/2 ,
                                        vertex[i].pos.y() * BASE_SIZE_ + BASE_SIZE_/2),
                                 QPoint(vertex[arc->vertex_index].pos.x() * BASE_SIZE_ + BASE_SIZE_/2,
                                        vertex[arc->vertex_index].pos.y() * BASE_SIZE_ + BASE_SIZE_/2));
            }
            arc = arc->next_arc;
        }
    }

    //draw path
    pen.setColor(QColor(255, 0, 0));    //green
    painter.setPen(pen);

    if(points.size() >= 2)
    {
        //queue, first in first out
        QVector<QPoint> points_queue;
        QVector<QPoint>::const_iterator it3 = points.cbegin();

        for(; it3 != points.cend(); it3++)
        {
            points_queue.push_back(QPoint(*it3));
        }

        QPoint start = points_queue.at(0);
        points_queue.pop_front();

        QPoint end;

        while (points_queue.size())
        {
            end = points_queue.at(0);
            points_queue.pop_front();

            painter.drawLine(QPoint(start.x()*BASE_SIZE_ + BASE_SIZE_/2 ,
                                    start.y()*BASE_SIZE_ + BASE_SIZE_/2),
                             QPoint(end.x()*BASE_SIZE_ + BASE_SIZE_/2,
                                    end.y()*BASE_SIZE_ + BASE_SIZE_/2));
            start = end;
        }
    }

    painter.end();
    update();
}

const int **GenerateMap::getMapMatrix() const
{
    return (const int**)map_matrix_;
}

int GenerateMap::getMapHeight() const
{
    return height_;
}

int GenerateMap::getMapWidth() const
{
    return width_;
}

void GenerateMap::showPoint()
{
    if(canvas_ == nullptr)
    {
        return;
    }

    QPainter painter;

    painter.begin(canvas_);

    if(start_point_.x() != -1)
    {
        const int shift_size = FLAG_SIZE/2;
        painter.drawPixmap(start_point_.x()-shift_size, start_point_.y()-shift_size, 
                        FLAG_SIZE, FLAG_SIZE, start_image_);
        update(QRect(start_point_.x(), start_point_.y(), FLAG_SIZE, FLAG_SIZE));
    }

    if(end_point_.x() != -1)
    {
        const int shift_size = FLAG_SIZE/2;
        painter.drawPixmap(end_point_.x()-shift_size, end_point_.y()-shift_size, 
                        FLAG_SIZE, FLAG_SIZE, end_image_);
        update(QRect(end_point_.x(), end_point_.y(), FLAG_SIZE, FLAG_SIZE));
    }

    painter.end();
}

void GenerateMap::refreashMap()
{
    if(map_matrix_ == nullptr)
    {
        return;
    }
    QPainter painter;

    if(canvas_ != nullptr)
    {
        delete canvas_;
    }

    canvas_ = new QPixmap(width_*BASE_SIZE_, height_*BASE_SIZE_);

    painter.begin(canvas_);

    for(int i=0; i<height_; i++)
    {
        for(int j=0; j<width_; j++)
        {
            if(map_matrix_[i][j] == 2)
            {
                painter.drawPixmap(j*BASE_SIZE_, i*BASE_SIZE_, BASE_SIZE_, BASE_SIZE_, sand_image_);
            }
            else if(map_matrix_[i][j] == 1)
            {
                painter.drawPixmap(j*BASE_SIZE_, i*BASE_SIZE_, BASE_SIZE_, BASE_SIZE_, wall_image_);
            }
            else
            {
                painter.drawPixmap(j*BASE_SIZE_, i*BASE_SIZE_, BASE_SIZE_, BASE_SIZE_, road_image_);
            }
        }
    }

    painter.end();
    update();
}

void GenerateMap::paintEvent(QPaintEvent *event)
{
    if(canvas_ == nullptr)
    {
        return;
    }

    painter_.begin(this);
    painter_.drawPixmap(paint_offset_, *canvas_);
    painter_.end();
}

void GenerateMap::mousePressEvent(QMouseEvent *event)
{
    if(canvas_ == nullptr)
    {
        return;
    }

    if(event->button() == Qt::RightButton)
    {
        QPoint pos;

        pos.setX(event->pos().x()-paint_offset_.x());
        pos.setY(event->pos().y()-paint_offset_.y());

        if(map_matrix_[pos.y()/BASE_SIZE_][pos.x()/BASE_SIZE_] == 1)
        {
            QMessageBox::about(this, "Error", "You can't set the start or end point on the wall");
            return;
        }

        if(start_point_.x() == -1)
        {
            setStartPoint(pos.x(), pos.y());
            emit startEndChanged((const QPoint)start_point_, (const QPoint)end_point_);
        }
        else if(end_point_.x() == -1)
        {
            setEndPoint(pos.x(), pos.y());
            emit startEndChanged((const QPoint)start_point_, (const QPoint)end_point_);
            emit endChanged();
        }
        else
        {
            setStartPoint(end_point_.x(), end_point_.y());
            setEndPoint(pos.x(), pos.y());
            emit startEndChanged((const QPoint)start_point_, (const QPoint)end_point_);
            emit endChanged();
        }

        this->refreashMap();
        this->showPoint();
    }
    else if(event->button() == Qt::LeftButton)
    {
        clicked_pos_ = event->pos();
    }
}

void GenerateMap::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        paint_offset_.setX(event->pos().x() - clicked_pos_.x());
        paint_offset_.setY(event->pos().y() - clicked_pos_.y());
        robot_label_->move(end_point_.x()+paint_offset_.x(),
                           end_point_.y()+paint_offset_.y());
        update();
    }
}

void GenerateMap::mouseDoubleClickEvent(QMouseEvent *event)
{
    paint_offset_.setX(0);
    paint_offset_.setY(0);
    robot_label_->move(end_point_.x()+paint_offset_.x(),
                       end_point_.y()+paint_offset_.y());
    update();
}

void GenerateMap::setStartPoint(int x, int y)
{
    start_point_.setX(x);
    start_point_.setY(y);
}

const QPoint GenerateMap::getStartPoint() const
{
    return QPoint(start_point_.x()/BASE_SIZE_, start_point_.y()/BASE_SIZE_);
}

void GenerateMap::setEndPoint(int x, int y)
{
    end_point_.setX(x);
    end_point_.setY(y);
}

const QPoint GenerateMap::getEndPoint() const
{
    return QPoint(end_point_.x()/BASE_SIZE_, end_point_.y()/BASE_SIZE_);
}

void GenerateMap::setRandomSeed(int seed)
{
    if(seed == 0)
    {
        random_gen_->seed(QDateTime::currentMSecsSinceEpoch() % UINT_MAX);
    }
    else
    {
        random_gen_->seed(seed);
    }
}

void GenerateMap::setMapSize(int width, int height)
{
    width_ = width;
    height_ = height;

    if(width_ < LIMIT_WIDTH_.x())
    {
        width_ = LIMIT_WIDTH_.x();
    }
    else if(width_ > LIMIT_WIDTH_.y())
    {
        width_ = LIMIT_WIDTH_.y();
    }

    if(height_ < LIMIT_HEIGHT_.x())
    {
        height_ = LIMIT_HEIGHT_.x();
    }
    else if(height_ > LIMIT_HEIGHT_.y())
    {
        height_ = LIMIT_HEIGHT_.y();
    }

    if(width_%2 == 0)
    {
        width_ += 1;
    }

    if(height_%2 == 0)
    {
        height_ += 1;
    }
}
