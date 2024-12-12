#include "evaluator.h"
#include "generatemap.h"
#include <chrono>
#include <ratio>


Evaluator::Evaluator(const int map_width, const int map_height, const int trial_num,
            const int map_seed, const int prm_seed,
            PRM::Algorithm algorithm, PRM::Strategy strategy)
{
    map_width_ = map_width;
    map_height_ = map_height;
    trial_num_ = trial_num;
    map_seed_ = map_seed;
    prm_seed_ = prm_seed;
    algorithm_ = algorithm;
    strategy_ = strategy;

    map_random_gen_ = new QRandomGenerator(map_seed_);
    prm_random_gen_ = new QRandomGenerator(prm_seed_);
    map_ = new GenerateMap();
    
    prm_ = new PRM();
    report_ = new Report();

    report_->addEntry("Map Width", map_width_);
    report_->addEntry("Map Height", map_height_);
    report_->addEntry("Trail Number", trial_num_);
    report_->addEntry("Map Seed", map_seed_);
    report_->addEntry("PRM Seed", prm_seed_);
    report_->addEntry("Algorithm", algorithm_);
    report_->addEntry("Strategy", strategy_);
}


Evaluator::~Evaluator()
{
    if(prm_ != nullptr)
    {
        delete prm_;
    }

    if(report_ != nullptr)
    {
        delete report_;
    }
}

void Evaluator::evaluate() const
{
    map_->setRandomSeed(map_seed_);
    prm_->setRandomSeed(prm_seed_);

    auto search_function = prm_->selectAlgorithm(algorithm_);

    double total_time = 0.0;
    float total_cost = 0.0;
    int failed_num = 0;
    for(int i=0; i<trial_num_;i++){
        //generate map
        map_->creatMap(map_width_, map_height_);

        //get the map matrix
        const int** map_matrix = map_->getMapMatrix();
        int map_height = map_->getMapHeight();
        int map_width = map_->getMapWidth();

        //generate start and end point
        QPoint start_point = QPoint(0, 0);
        QPoint end_point = QPoint(0, 0);

        float distance_threshold = 0.5 * sqrt(pow(map_height, 2) + pow(map_width, 2));
        while (true) {
            start_point = generateValidPoint(map_matrix, map_height, map_width);
            end_point = generateValidPoint(map_matrix, map_height, map_width);

            //check the start and end point
            float distance = sqrt(pow(start_point.x() - end_point.x(), 2) + pow(start_point.y() - end_point.y(), 2));
            
            if (distance >= distance_threshold) {
                break;
            }
        }

        prm_->setStartPoint(start_point);
        prm_->setEndPoint(end_point);
        prm_->constructGraph(map_matrix, map_height, map_width);
        
        //calculate the running time
        auto start = std::chrono::high_resolution_clock::now();

        search_function(strategy_);
        QVector<QPoint> points = prm_->getPath();
        if(points.empty())
        {
            failed_num++;
        }
        else
        {
            float cost = prm_->calPathCost(points, strategy_);
            total_cost += cost;
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        total_time += elapsed.count();
    }

    report_->addEntry("Average Time (ms)", float(total_time / trial_num_));
    report_->addEntry("Average Cost", total_cost / (trial_num_ - failed_num));
    report_->addEntry("Failed Number", failed_num);
}

QString Evaluator::getResults()
{
    return report_->getReportContent();
}

QPoint Evaluator::generateValidPoint(const int **map_matrix, const int map_height, 
                                const int map_width) const
{
    int x1, y1;
    while(true)
    {
        x1 = map_random_gen_->bounded(map_height);
        y1 = map_random_gen_->bounded(map_width);
        if(map_matrix[x1][y1] == 0)
        {
            break;
        }
    }
    QPoint point(x1, y1);
    
    // transpose the point: from the matrix coordinate to the Cartesian coordinate
    return point.transposed();
}