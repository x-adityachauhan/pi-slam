// Regression: observation pruning during PreSave must not invalidate iterators.
// Loads an atlas read-only, mutates the in-memory copy after workers stop,
// and deliberately creates both stale-map and absent-keyframe observations.
#include <iostream>
#include <set>
#include <stdexcept>
#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    ORB_SLAM3::System system(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);
    system.Shutdown();
    auto* atlas = system.GetAtlasForExport();
    ORB_SLAM3::Map* fixture = nullptr;
    for (auto* map : atlas->GetAllMaps())
        if (!fixture || map->MapPointsInMap() > fixture->MapPointsInMap()) fixture = map;
    if (!fixture || fixture->MapPointsInMap() < 100) {
        std::cerr << "A nonempty saved atlas fixture is required\n";
        return 1;
    }
    const auto before = fixture->MapPointsInMap();
    const auto keyframes = fixture->GetAllKeyFrames();
    ORB_SLAM3::Map other_map(100000);
    for (size_t i = 0; i < keyframes.size(); ++i) {
        if (i % 3 == 0) continue;
        if (i % 3 == 1) keyframes[i]->UpdateMap(&other_map);
        fixture->EraseKeyFrame(keyframes[i]);
    }
    // Both passes must tolerate points erasing themselves from the map.
    atlas->PreSave();
    atlas->PreSave();
    const auto remaining = fixture->GetAllKeyFrames();
    std::set<ORB_SLAM3::KeyFrame*> valid(remaining.begin(), remaining.end());
    size_t observations = 0;
    for (auto* point : fixture->GetAllMapPoints()) {
        if (point->isBad()) throw std::runtime_error("Bad point survived pruning");
        for (const auto& observation : point->GetObservations()) {
            if (!valid.count(observation.first) || observation.first->GetMap() != fixture)
                throw std::runtime_error("Invalid observation survived pruning");
            ++observations;
        }
    }
    const auto after = fixture->MapPointsInMap();
    if (after >= before || !observations) throw std::runtime_error("Fixture did not exercise pruning");
    std::cout << "SAVE_REGRESSION_PASS: " << before << " -> " << after
              << " points, " << observations << " valid observations; repeated PreSave safe\n";
}
