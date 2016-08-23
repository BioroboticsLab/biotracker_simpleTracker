#ifndef BIOTRACKER_DEVELOPMENT_MAPPER_H
#define BIOTRACKER_DEVELOPMENT_MAPPER_H

#include <opencv2/opencv.hpp>

#include "TrackedFish.h"
#include "FishCandidate.h"

#include <biotracker/serialization/TrackedObject.h>

class Mapper {
public:
    static const float MAX_TRACK_DISTANCE_PER_FRAME;
    static const float MAX_TRACK_DISTANCE;
    static const int   CANDIDATE_SCORE_THRESHOLD;

    Mapper();

	std::vector<BioTracker::Core::TrackedObject> map(std::vector<BioTracker::Core::TrackedObject> &trackedObjects,
													 std::vector<cv::RotatedRect> &contourEllipses, size_t frame);

private:
    std::vector<BioTracker::Core::TrackedObject> m_trackedObjects;

    float _averageSpeedPx;


    std::shared_ptr<TrackedFish> mergeContoursToTrackedFish(size_t trackedObjectIndex, size_t frame,
                                                            std::vector<cv::RotatedRect> &contourEllipses);
    std::tuple<size_t , float> getNearestIndexFromFishPoses(TrackedFish &fishPose,
                                                            const std::vector<cv::RotatedRect> &fishPoses);
    float estimateOrientationRad(size_t trackedObjectIndex, size_t frame, float *confidence) const;
    float getCurrentSpeed(size_t trackedObjectIndex, size_t frame, size_t smoothingWindow) const;
    std::shared_ptr<TrackedFish> estimateNextPose(size_t trackedObjectIndex, size_t frame) const;
    TrackedFish getPoseForMapping(size_t trackedObjectIndex, size_t frame) const;
    bool correctAngle(size_t trackedObjectIndex, size_t frame, cv::RotatedRect &pose);
    float angleDifference(float alpha, float beta);
};

#endif