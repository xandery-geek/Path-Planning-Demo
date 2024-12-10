#include "graph.h"
#include <qmath.h>

Graph::Graph()
{

}

Graph::~Graph()
{

}

void Graph::addVertex(const QVector<QPoint> &points)
{
    QVector<QPoint>::const_iterator it = points.cbegin();
    for(; it!=points.cend(); it++)
    {
        graph_.push_back(Vertex(*it));
    }
}

void Graph::addVertex(const QPoint& point)
{
    graph_.push_back(point);
}

const QVector<Graph::Vertex> &Graph::getVertex() const
{
    return graph_;
}

const Graph::Vertex &Graph::getVertex(int index) const
{
    assert(index >=0 && index < graph_.size());

    return graph_[index];
}

int Graph::getVertex(const QPoint &point) const
{
    for(int i=0; i<graph_.size(); i++)
    {
        if(graph_[i].pos == point)
        {
            return i;
        }
    }

    return -1;  //not find
}

void Graph::addArc(int start, int end)
{
    Graph::Arc *arc = graph_[start].head_arc;

    if(arc == nullptr)
    {
        graph_[start].head_arc = new Graph::Arc(end);
        arc = graph_[start].head_arc;
    }
    else
    {
        while(arc->next_arc)
        {
            arc = arc->next_arc;
        }

        arc->next_arc = new Graph::Arc(end);
        arc = arc->next_arc;
    }

    arc->distance = 0.0f;
    arc->oil = 0.0f;
    arc->next_arc = nullptr;
}

void Graph::addArc(int start, int end, float distance, float oil)
{
    Graph::Arc *arc = graph_[start].head_arc;

    if(arc == nullptr)
    {
        graph_[start].head_arc = new Graph::Arc(end);
        arc = graph_[start].head_arc;
    }
    else
    {
        while(arc->next_arc)
        {
            arc = arc->next_arc;
        }

        arc->next_arc = new Graph::Arc(end);
        arc = arc->next_arc;
    }

    arc->distance = distance;
    arc->oil = oil;
    arc->next_arc = nullptr;
}

void Graph::addArc(int start, const QVector<int>& end_points)
{

    Graph::Arc *arc = graph_[start].head_arc;
    while(arc)
    {
        arc = arc->next_arc;
    }

    for(int i=0; i<end_points.size(); i++)
    {
        arc = new Graph::Arc(end_points[i]);
        arc->distance = getDistance(graph_[start].pos, graph_[end_points[i]].pos);
        arc->next_arc = nullptr;

        arc = arc->next_arc;
    }
}

/*
* @retutn:
*/
const Graph::Arc Graph::getArc(int vertex, int index) const
{
    assert(vertex >=0 && vertex < graph_.size());

    int i=0;

    Graph::Arc *arc = graph_[vertex].head_arc;
    while(arc->next_arc && i<index)
    {
        arc = arc->next_arc;
        i++;
    }

    return (*arc);
}

QVector<int> Graph::getNeightbor(int vertex) const
{
    assert(vertex>=0 && vertex < graph_.size());

    QVector<int> vec;

    Graph::Arc *arc = graph_[vertex].head_arc;
    while(arc)
    {
        vec.push_back(arc->vertex_index);
        arc = arc->next_arc;
    }

    return vec;
}

void Graph::destroyGraph()
{
    for(int i=0; i<graph_.size(); i++)
    {
        Graph::Arc *temp;
        Graph::Arc *arc = graph_[i].head_arc;
        if(arc)
        {
            temp = arc;
            arc = arc->next_arc;
            delete temp;
            temp = nullptr;
        }
    }

    graph_.clear();
}

float Graph::getDistance(const QPoint &point1, const QPoint &point2)
{
    return sqrt( pow(point1.x() - point2.x(), 2) + pow(point2.y() - point2.y(), 2));
}

