#ifndef PRM_H
#define PRM_H

#include <QVector>
#include <QPoint>
#include <QRandomGenerator>
#include <unordered_map>
#include "graph.h"
#include "kdtree.h"


class AStarCost
{
public:
    int index;
    float cost;

    AStarCost()
    {
        index = 0;
        cost = 0.0f;
    }

    AStarCost(int index, float cost)
    {
        this->index = index;
        this->cost = cost;
    }

    bool operator <(const AStarCost& obj) const
    {
        return this->cost < obj.cost;
    }

    bool operator >(const AStarCost& obj) const
    {
        return this->cost > obj.cost;
    }
};

class PRM
{
private:
    const float VERTEX_COEFFICIENT = 0.4;
    const float K_COEFFICIENT = 0.02;
    const float ROAD_WEIGHT = 1;
    const float SAND_WEIGHT = 8;

    QPoint start_;
    QPoint goal_;
    QVector<QPoint> path_;

    Graph prm_graph_;
    KdTree *kd_tree_;
    int vertex_k_;  // the scale of nearest beightbor

    const int **graph_mat_; //the actual map data
    int graph_mat_row_;
    int graph_mat_col_;
    QString stragety_;

    QRandomGenerator *random_gen_;

public:
    PRM(const QString& stragety="AStar");
    ~PRM();

    void constructGraph(const int** mat, int row, int col);
    void generateArc(const QVector<QPoint>& points); //KNN
    void setStrategy(const QString& stragety);
    void setStartPoint(const QPoint& point);
    void setEndPoint(const QPoint& point);
    void searchPath(bool option);
    const QVector<QPoint>& getPath() const;
    const Graph& getGraph() const;

private:
    void AStar(bool option);
    void Dijkstra(bool option);
    bool checkPath(const QPoint& point1, const QPoint& point2);
    float getDistance(const QPoint& point1, const QPoint& point2);
    float getOil(const QPoint& point1, const QPoint& point2);
    QPoint transposePoint(const QPoint& point);
    bool isCrash(const int** mat, int row, int col, const QPoint& point);
    void reconstructPath(const QVector<Graph::Vertex> &vertex, int start, int goal,
                         const std::unordered_map<int, int> &close_list);
};

#endif // PRM_H
