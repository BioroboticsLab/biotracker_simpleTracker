#ifndef TrackedFish_H
#define TrackedFish_H

#include <biotracker/serialization/TrackedObject.h>

#include "FishPose.h"

#include <cereal/access.hpp>
#include <opencv2/opencv.hpp>

class TrackedFish : public BioTracker::Core::TrackedObject {
public:
    float estimateOrientationRad(size_t frame, float *confidence);
    float getCurrentSpeed(size_t frame, size_t smoothingWindow);
    std::shared_ptr<FishPose> estimateNextPose(size_t frame);
    FishPose& getPoseForMapping(size_t frame);
    bool correctAngle(size_t frame, cv::RotatedRect &pose);

private:
    float angleDifference(float alpha, float beta);

};

#endif