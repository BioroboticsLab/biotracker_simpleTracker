#ifndef MAPPER_H
#define MAPPER_H

#include <opencv2/opencv.hpp>

#include "FishPose.h"
#include "FishCandidate.h"

#include <biotracker/serialization/TrackedObject.h>

class Mapper {
public:
    Mapper(std::vector<BioTracker::Core::TrackedObject> &trackedObjects, size_t numberOfObjects);

	void map(std::vector<cv::RotatedRect> &contourEllipses, size_t frame);

    std::vector<BioTracker::Core::TrackedObject>& getFishCandidates();

private:
    std::vector<BioTracker::Core::TrackedObject> &m_trackedObjects;
    std::vector<BioTracker::Core::TrackedObject> _fishCandidates;

    size_t _numberOfObjects;
    size_t _lastId;


    std::tuple<size_t, std::shared_ptr<FishPose>> mergeContoursToFishes(size_t fishIndex, size_t frame,
                                                                        std::vector<BioTracker::Core::TrackedObject> &fishes,
                                                                        std::vector<cv::RotatedRect> &contourEllipses);

    std::shared_ptr<FishPose> mergeContoursToFishPose(size_t trackedObjectIndex, size_t frame,
                                                      std::vector<cv::RotatedRect> &contourEllipses);
    std::shared_ptr<FishCandidate> mergeContoursToFishCandidates(size_t trackedObjectIndex, size_t frame,
                                                                 std::vector<cv::RotatedRect> &contourEllipses);

    std::tuple<int , float> getNearestIndexFromFishPoses(FishPose &fishPose,
                                                         const std::vector<cv::RotatedRect> &fishPoses);
};

#endif