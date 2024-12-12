#ifndef EVALUATOR_H
#define EVALUATOR_H

#include "generatemap.h"
#include "prm.h"
#include <QRandomGenerator>
#include <QString>


class Report {
private:
    // Define a structure to store data entries
    struct Entry {
        std::string label;
        std::string value;
    };

    std::vector<Entry> entries;

public:
    // Add an integer entry
    void addEntry(const std::string& label, int value) {
        entries.push_back({label, std::to_string(value)});
    }

    void addEntry(const std::string& label, float value) {
        entries.push_back({label, std::to_string(value)});
    }

    // Add a string entry
    void addEntry(const std::string& label, const std::string& value) {
        entries.push_back({label, value});
    }

    void clear() {
        entries.clear();
    }

    // Output the report in a formatted style
    QString getReportContent() const {
        QString content;
        for (const auto& entry : entries) {
            content += QString::fromStdString(entry.label) + ": " + QString::fromStdString(entry.value) + "\n";
        }
        return content;
    }
};


class Evaluator {

private:
    /* data */
    int map_width_;
    int map_height_;
    int trial_num_;
    int map_seed_;
    int prm_seed_;
    PRM::Algorithm algorithm_;
    PRM::Strategy strategy_;

    QRandomGenerator *map_random_gen_;
    QRandomGenerator *prm_random_gen_;

    GenerateMap *map_;
    
    PRM *prm_;
    Report *report_;

    QPoint generateValidPoint(const int **map_matrix, const int map_height, 
                            const int map_width) const;
public:
    Evaluator(const int map_width, const int map_height, const int trial_num,
            const int map_seed, const int prm_seed,
            PRM::Algorithm algorithm, PRM::Strategy strategy);
    ~Evaluator();

    void evaluate() const;
    QString getResults();
};

#endif // EVALUATOR_H