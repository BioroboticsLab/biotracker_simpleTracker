#include "TrackedFish.h"

using namespace BioTracker::Core;


float TrackedFish::estimateOrientationRad(size_t frame, float *confidence) {
    // can't give estimate if not enough poses available
    if (frame < 3 || !hasValuesAtFrame(frame) || !hasValuesAtFrame(frame - 1) ||
        !hasValuesAtFrame(frame - 2)) return std::numeric_limits<float>::quiet_NaN();

    cv::Point2f nextPoint = get<FishPose>(frame)->last_known_position().center;
    cv::Point2f positionDerivative(0.0f, 0.0f);

    // weights the last poses with falloff^k * pose[end - k] until falloff^k < falloffMargin
    int posesUsed = 0;
    float currentWeight = 1.0f;
    float weightSum = 0.0f;
    const float falloff = 0.9f;
    const float falloffMargin = 0.4f;

    for (int i = static_cast<int>(frame) - 1; i > -1; i--) {
        if(hasValuesAtFrame(static_cast<size_t>(i))){
            // TODO: may want to use position in cm instead of pixel
            cv::Point2f currentPoint = get<FishPose>(static_cast<size_t>(i))->last_known_position().center;
            const cv::Point2f oneStepDerivative = nextPoint - currentPoint;

            positionDerivative += currentWeight * oneStepDerivative;
            weightSum += currentWeight;

            currentWeight *= falloff;
            if (currentWeight < falloffMargin) break;

            nextPoint = currentPoint;
            ++posesUsed;
        }
    }
    // calculate average (weighted) movement of the fish
    if (weightSum != 0.0f) {
        positionDerivative.x /= weightSum;
        positionDerivative.y /= weightSum;
    }
    // use the euclidian distance
    const float distance = std::sqrt(std::pow(positionDerivative.x, 2.0f) + std::pow(positionDerivative.y, 2.0f));

//    const float confidenceDistanceMin = FishPose::_averageSpeed * 0.66f;
    const float confidenceDistanceMax = FishPose::_averageSpeed * 1.33f;
//    // if we have either nearly no data or are very unsure (left movement offsets right movement f.e.), just return nothing
//    if (distance < confidenceDistanceMin)
//        return std::numeric_limits<float>::quiet_NaN();
    *confidence = std::min(distance / confidenceDistanceMax, 1.0f);

    // negative y coordinate to offset open cv coordinate system
    return std::atan2(-positionDerivative.y, positionDerivative.x);
}


float TrackedFish::getCurrentSpeed(size_t frame, size_t smoothingWindow) {
    // don't try to estimate a speed when NO smoothing is possible
    if (frame < 3 || !hasValuesAtFrame(frame) || !hasValuesAtFrame(frame - 1) ||
        !hasValuesAtFrame(frame - 2) || !hasValuesAtFrame(frame - 3)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    // Can only smooth over max. the frames we know.
    // Also, never use the first invalid "pose".
    if (smoothingWindow > frame) smoothingWindow = frame;

    auto calculateSpeed = [](const cv::Point2f &current, const cv::Point2f &previous)
    {
        const cv::Point2f derivation = current - previous;
        const float speed = std::sqrt(derivation.x * derivation.x + derivation.y * derivation.y);
        assert(std::isfinite(speed));
        return speed;
    };

    int totalPoints = 0;
    float totalSpeed = 0.0f;

//    size_t i = getLastFrameNumber().get() - 1;
    size_t i = frame - 1;
    while (hasValuesAtFrame(i) && hasValuesAtFrame(i + 1)) {
        auto currentPose = get<FishPose>(i);
        auto nextPose = get<FishPose>(i + 1);
        const float currentSpeed = calculateSpeed(currentPose->last_known_position().center, nextPose->last_known_position().center);
        totalSpeed += currentSpeed;
        i--;
        totalPoints++;

        if (--smoothingWindow == 0) break;
    }

    // the current speed is the (non-weighted!) average over the last frames
    const float currentSpeed = totalSpeed / static_cast<float>(totalPoints);
    assert(std::isfinite(currentSpeed));
    return currentSpeed;
}

std::shared_ptr<FishPose> TrackedFish::estimateNextPose(size_t frame) {
    // can't estimate next position?
    if (frame < 3 || !hasValuesAtFrame(frame) || !hasValuesAtFrame(frame - 1) ||
        !hasValuesAtFrame(frame - 2) || !hasValuesAtFrame(frame - 3)) {
        return nullptr;
    }

    const auto currentPose = get<FishPose>(frame);
    const float currentAngle = currentPose->angle();
    const size_t smoothingWindow = 3;
    const float currentSpeedPx = getCurrentSpeed(frame, smoothingWindow);

    // safety!
    if (!std::isfinite(currentAngle) || !std::isfinite(currentSpeedPx)) { return nullptr; }

    const cv::Point2f nextPositionPx = currentPose->last_known_position().center
                                       + cv::Point2f(static_cast<float>(currentSpeedPx * std::cos(currentAngle)),
                                                     static_cast<float>(-currentSpeedPx * std::sin(currentAngle)));
    auto retFish = std::make_shared<FishPose>(currentPose->age_of_last_known_position(),
                                              cv::RotatedRect(nextPositionPx, currentPose->last_known_position().size, currentAngle));
//    retFish->setNextPosition(cv::RotatedRect(nextPositionPx, currentPose->last_known_position().size, currentAngle));
    retFish->setAngle(currentAngle);
    return retFish;
}

FishPose& TrackedFish::getPoseForMapping(size_t frame) {
    // try the estimated position first
    std::shared_ptr<FishPose> estimated = estimateNextPose(frame);
    // ok? then use this!
    if (estimated.get())
    {
        assert(std::isfinite(estimated->last_known_position().center.x));
        assert(std::isfinite(estimated->angle()));
        return *estimated;
    }
    // otherwise, just use the current pose
//    assert(m_trackedObjects[trackedObjectIndex].hasValuesAtFrame(frame));
    return *(get<FishPose>(frame));
}

bool TrackedFish::correctAngle(size_t frame, cv::RotatedRect &pose)
{
    assert(hasValuesAtFrame(frame));
    auto fish = get<FishPose>(frame);
    // the current angle is a decent estimation of the direction; however, it might point into the wrong hemisphere
    const float poseOrientation = static_cast<float>(pose.angle * CV_PI / 180.0f);

    // start with the pose orientation for our estimate
    float proposedAngle = poseOrientation;

    // we have more historical data to correct the new angle to at least be more plausible
    float confidence = 0.0f;
    const float historyAngle = estimateOrientationRad(frame, &confidence);
    const float lastConfidentAngle = fish->angle();

    // the current history orientation has a stronger meaning and is preferred
    const float comparisonOrientation = std::isnan(historyAngle) ? lastConfidentAngle : historyAngle;
    // can't correct the angle?
    if (std::isnan(comparisonOrientation)) return false;

    // panic mode - what if nothing was measured?
    if (std::isnan(poseOrientation))
    {
        pose.angle = comparisonOrientation;
        return false;
    }

    const float angleDiff = angleDifference(proposedAngle, comparisonOrientation);

    // if the angles do not lie on the same hemisphere, mirror the proposed angle
    if (!std::isnan(angleDiff) && std::abs(angleDiff) > 0.5f * CV_PI)
    {
        proposedAngle += static_cast<float>(CV_PI);
    }

    // the angle is corrected into the correct hemisphere now;
    // now smooth the angle to reduce the impact of outliers or directly remove a zero-measurement.

    if (std::isnan(lastConfidentAngle)) // nothing to smooth? Then simply assume the movement-angle to be a good first estimate
        proposedAngle = historyAngle;
    else
    {
        // smooth the change in the angle iff the new angle deviates too much from the last one
        const float deviationFromLast = angleDifference(lastConfidentAngle, proposedAngle);
        assert(!std::isnan(deviationFromLast));

        if (std::abs(deviationFromLast) > 0.2f * static_cast<float>(CV_PI))
        {
            if (poseOrientation == 0.0f) // deviation AND zero-angle? Most likely not a decent estimation.
                proposedAngle = lastConfidentAngle;
            else // smooth outliers by a fixed margin
                proposedAngle = lastConfidentAngle - 0.1f * deviationFromLast;
        }
    }
    // angle should be between 0� and 360�
    if (proposedAngle > 2.0f * CV_PI) proposedAngle -= 2.0f * static_cast<float>(CV_PI);
    else if (proposedAngle < 0.0f)    proposedAngle += 2.0f * static_cast<float>(CV_PI);
    assert(!std::isnan(proposedAngle));

    pose.angle = proposedAngle;

    // did we have ANY confident correction?
    if (!std::isnan(lastConfidentAngle)) // if we simply adjusted the last position, assume to be confident
        return true;
    // otherwise, we need to intialize the confident angle.
    // do that when we really are "confident" for the first time..
    const float differenceToHistoryAngle = std::abs(angleDifference(proposedAngle, historyAngle));
    assert(!std::isnan(differenceToHistoryAngle));
    return (differenceToHistoryAngle < 0.25f * static_cast<float>(CV_PI));
}


float TrackedFish::angleDifference(float alpha, float beta)
{
    float difference = alpha - beta;
    while (difference < -CV_PI) difference += 2.0f * static_cast<float>(CV_PI);
    while (difference > +CV_PI) difference -= 2.0f * static_cast<float>(CV_PI);
    return difference;
}