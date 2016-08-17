#include "Mapper.h"

#include <QPainter>

#include <biotracker/Registry.h>

#include <opencv2/opencv.hpp>

const float Mapper::MAX_TRACK_DISTANCE_PER_FRAME = 5;

const float Mapper::MAX_TRACK_DISTANCE = 1000;

const int Mapper::CANDIDATE_SCORE_THRESHOLD = 30;

using namespace BioTracker::Core;
// ================= P U B L I C ====================
Mapper::Mapper() {}

std::vector<TrackedObject> Mapper::map(std::vector<TrackedObject> &trackedObjects, std::vector<cv::RotatedRect> &contourEllipses, std::vector<cv::Point2f> &centers, size_t frame){
    m_trackedObjects = trackedObjects;
    for (TrackedObject& trackedObject : m_trackedObjects) {
        trackedObject.add(frame, mergeContoursToTrackedFish(trackedObject.getId() - 1, frame - 1, contourEllipses, centers));
    }
	return m_trackedObjects;
}

// ================ P R I V A T E ===================

std::shared_ptr<TrackedFish> Mapper::mergeContoursToTrackedFish(size_t trackedObjectIndex, size_t frame, std::vector<cv::RotatedRect> &contourEllipses, std::vector<cv::Point2f> &centers)
{

    auto fish = m_trackedObjects[trackedObjectIndex];

    size_t np(0);
    float score(0);

    TrackedFish tmpFishPose = getPoseForMapping(trackedObjectIndex, frame);
    std::tie(np, score) = getNearestIndexFromFishPoses(tmpFishPose, contourEllipses);
    auto fp = std::make_shared<TrackedFish>();
    fp->setNextPosition(contourEllipses.at(np).center);
    fp->setAngle(contourEllipses.at(np).angle * (static_cast<float>(CV_PI) / 180.0f));

    std::vector<cv::RotatedRect> fps;
    for(size_t i = 0; i < m_trackedObjects.size(); i++)
    {
        if(m_trackedObjects[i].hasValuesAtFrame(frame)){
            TrackedFish tmpFish = getPoseForMapping(i, frame);
            cv::Size2f tmpSize(0.0f, 0.0f);
            cv::RotatedRect tmpRect(tmpFish.last_known_position(), tmpSize, tmpFish.angle());
            fps.push_back(tmpRect);
        }
    }

    size_t fpt;
    float fptScore;
    std::tie(fpt, fptScore) = getNearestIndexFromFishPoses(*(fp.get()), fps);


    if(fpt == trackedObjectIndex)
    {
        auto newFish = std::make_shared<TrackedFish>();
//        TrackedFish newFish();
        newFish->setNextPosition(contourEllipses[np].center);
        newFish->setAngle(contourEllipses[np].angle * (static_cast<float>(CV_PI) / 180.0f));
//        pose.setScore(score);
//        const bool angleConfident = correctAngle(trackedObjectIndex, frame, contourEllipses[np]);
        correctAngle(trackedObjectIndex, frame, contourEllipses[np]);

		centers.erase(centers.begin() + np);
        contourEllipses.erase(contourEllipses.begin() + np);
        return newFish;
    }
    else
    {
        return mergeContoursToTrackedFish(fpt, frame, contourEllipses, centers);
    }
}

std::tuple<size_t , float> Mapper::getNearestIndexFromFishPoses(TrackedFish &fishPose,
                                                                       const std::vector<cv::RotatedRect> &fishPoses)
{
//    double nearestDistance = std::numeric_limits<double>::max();
    float totalProbablility = 0.0f;

    // for readability
    typedef std::tuple<int, float> PoseScoreTuple;
    enum { PoseIndex = 0, ScoreIndex = 1 };

    PoseScoreTuple bestTwo[] = { std::make_tuple(-1, -1.0f), std::make_tuple(-1, -1.0f) };

    for(size_t i = 0; i < fishPoses.size(); i++)
    {
        const cv::RotatedRect &possiblePose = fishPoses[i];
        // this takes angle-direction correction into account
        const float probabilityOfIdentity = fishPose.calculateProbabilityOfIdentity(possiblePose, 0.5f);
        totalProbablility += probabilityOfIdentity;

        // figure out whether the probability is the new highest or second highest
        if (probabilityOfIdentity > std::get<ScoreIndex>(bestTwo[0])) // new best?
        {
            // move current best to second place
            bestTwo[1] = bestTwo[0];
            // and remember new best one
            bestTwo[0] = std::make_tuple(static_cast<int>(i), probabilityOfIdentity);
        }
        else if (probabilityOfIdentity > std::get<ScoreIndex>(bestTwo[1])) // new second place?
        {
            bestTwo[1] = std::make_tuple(static_cast<int>(i), probabilityOfIdentity);
        }
    }
    // score defaults to the probability of identity if only one pose was found
    float score = std::get<ScoreIndex>(bestTwo[0]);

    if (std::get<PoseIndex>(bestTwo[1]) != -1) // second pose found?
    {
        score = score / (score + std::get<ScoreIndex>(bestTwo[1]));
        // this will now always be above 50%, so rescale a bit
        score = 2.0f * (score - 0.5f);
    }

    return std::make_tuple(static_cast<size_t>(std::get<PoseIndex>(bestTwo[0])), score);
}


float Mapper::estimateOrientationRad(size_t trackedObjectIndex, size_t frame, float *confidence) const
{
    auto trackedObject = m_trackedObjects[trackedObjectIndex];
    // can't give estimate if not enough poses available
    if (frame < 3 || !trackedObject.hasValuesAtFrame(frame) || !trackedObject.hasValuesAtFrame(frame - 1) ||
        !trackedObject.hasValuesAtFrame(frame - 2)) return std::numeric_limits<float>::quiet_NaN();

    cv::Point2f nextPoint = trackedObject.get<TrackedFish>(frame)->last_known_position();
    cv::Point2f positionDerivative(0.0f, 0.0f);

    // weights the last poses with falloff^k * pose[end - k] until falloff^k < falloffMargin
    int posesUsed = 0;
    float currentWeight = 1.0f;
    float weightSum = 0.0f;
    const float falloff = 0.9f;
    const float falloffMargin = 0.4f;

    for (int i = static_cast<int>(frame) - 1; i > -1; i--) {
        if(trackedObject.hasValuesAtFrame(i)){
            // TODO: may want to use position in cm instead of pixel
            cv::Point2f currentPoint = trackedObject.get<TrackedFish>(i)->last_known_position();
            const cv::Point2f oneStepDerivative = nextPoint - currentPoint;

            positionDerivative += currentWeight * oneStepDerivative;
            weightSum += currentWeight;

            currentWeight *= falloff;
            if (currentWeight < falloffMargin) break;

            nextPoint = currentPoint;
        }
        ++posesUsed;
    }
    // calculate average (weighted) movement of the fish
    if (weightSum != 0.0f) {
        positionDerivative.x /= weightSum;
        positionDerivative.y /= weightSum;
    }
    // use the euclidian distance in cm
    const float distance = std::sqrt(std::pow(positionDerivative.x, 2.0f) + std::pow(positionDerivative.y, 2.0f));
    // Calculate cm/s.
    // TODO: replace '50' with ms/frame
    const float distanceNormalized = 1000.0f * distance / static_cast<float>(50);
    const float confidenceDistanceMinCm = 2.0f;
    const float confidenceDistanceMaxCm = 6.0f;
    // if we have either nearly no data or are very unsure (left movement offsets right movement f.e.), just return nothing
    if (distanceNormalized < confidenceDistanceMinCm)
        return std::numeric_limits<float>::quiet_NaN();
    *confidence = std::min(distanceNormalized / confidenceDistanceMaxCm, 1.0f);

    // negative y coordinate to offset open cv coordinate system
    return std::atan2(-positionDerivative.y, positionDerivative.x);
}


float Mapper::getCurrentSpeed(size_t trackedObjectIndex, size_t frame, size_t smoothingWindow) const
{
    auto trackedObject = m_trackedObjects[trackedObjectIndex];
    // don't try to estimate a speed when NO smoothing is possible
    if (frame < 4 || !trackedObject.hasValuesAtFrame(frame) || !trackedObject.hasValuesAtFrame(frame - 1) ||
        !trackedObject.hasValuesAtFrame(frame - 2) || !trackedObject.hasValuesAtFrame(frame - 3)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    // Can only smooth over max. the frames we know.
    // Also, never use the first invalid "pose".
    if (smoothingWindow > trackedObject.getLastFrameNumber().get()) smoothingWindow = trackedObject.getLastFrameNumber().get() - 1;

    auto calculateSpeed = [](const cv::Point2f &current, const cv::Point2f &previous)
    {
        const cv::Point2f derivation = current - previous;
        const float speed = std::sqrt(derivation.x * derivation.x + derivation.y * derivation.y);
        assert(std::isfinite(speed));
        return speed;
    };

    const float totalPoints = static_cast<float>(smoothingWindow);
    float totalSpeed = 0.0f;

    size_t i = trackedObject.getLastFrameNumber().get() - 1;
    while (trackedObject.hasValuesAtFrame(i) && trackedObject.hasValuesAtFrame(i + 1)) {
//        assert((iter + 1) != _histComponents.rend());
        auto currentPose = trackedObject.get<TrackedFish>(i);
        auto nextPose = trackedObject.get<TrackedFish>(i + 1);
        const float currentSpeed = calculateSpeed(currentPose->last_known_position(), nextPose->last_known_position());
        totalSpeed += currentSpeed;
        i--;

        if (--smoothingWindow == 0) break;
    }
//    assert(smoothingWindow == 0);

    // the current speed is the (non-weighted!) average over the last frames
    const float currentSpeed = totalSpeed / totalPoints;
    assert(std::isfinite(currentSpeed));
    return currentSpeed;
}

std::shared_ptr<TrackedFish> Mapper::estimateNextPose(size_t trackedObjectIndex, size_t frame) const
{
    auto trackedObject = m_trackedObjects[trackedObjectIndex];
    // can't estimate next position?
    if (frame < 4 || !trackedObject.hasValuesAtFrame(frame) || !trackedObject.hasValuesAtFrame(frame - 1) ||
        !trackedObject.hasValuesAtFrame(frame - 2) || !trackedObject.hasValuesAtFrame(frame - 3)) {
        return nullptr;
    }

    const auto currentPose = trackedObject.get<TrackedFish>(frame);
    const float currentAngle = currentPose->angle();
    const size_t smoothingWindow = 3;
    const float currentSpeedPx = getCurrentSpeed(trackedObjectIndex, frame, smoothingWindow);

    // safety!
    if (!std::isfinite(currentAngle) || !std::isfinite(currentSpeedPx)) { return nullptr; }

    const cv::Point2f nextPositionPx = currentPose->last_known_position()
                                       + cv::Point2f(static_cast<float>(currentSpeedPx * std::cos(currentAngle)),
                                                     static_cast<float>(-currentSpeedPx * std::sin(currentAngle)));
    auto retFish = std::make_shared<TrackedFish>();
    retFish->setNextPosition(nextPositionPx);
    retFish->setAngle(currentAngle);
    return retFish;
}

TrackedFish Mapper::getPoseForMapping(size_t trackedObjectIndex, size_t frame) const
{
    // try the estimated position first
    std::shared_ptr<TrackedFish> estimated = estimateNextPose(trackedObjectIndex, frame);
    // ok? then use this!
    if (estimated.get())
    {
        assert(std::isfinite(estimated->last_known_position().x));
        assert(std::isfinite(estimated->angle()));
        return *estimated;
    }
    // otherwise, just use the current pose
//    assert(m_trackedObjects[trackedObjectIndex].hasValuesAtFrame(frame));
    return *(m_trackedObjects[trackedObjectIndex].get<TrackedFish>(frame));
}

bool Mapper::correctAngle(size_t trackedObjectIndex, size_t frame, cv::RotatedRect &pose)
{
    assert(m_trackedObjects[trackedObjectIndex].hasValuesAtFrame(frame));
    auto fish = m_trackedObjects[trackedObjectIndex].get<TrackedFish>(frame);
    // the current angle is a decent estimation of the direction; however, it might point into the wrong hemisphere
    const float poseOrientation = pose.angle;

    // start with the pose orientation for our estimate
    float proposedAngle = poseOrientation;

    // we have more historical data to correct the new angle to at least be more plausible
    float confidence = 0.0f;
    const float historyAngle = estimateOrientationRad(trackedObjectIndex, frame, &confidence);
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
    if (differenceToHistoryAngle < 0.25f * static_cast<float>(CV_PI))
        return true;
    // neither updating nor a good initialization?
    return false;
}


float Mapper::angleDifference(float alpha, float beta)
{
    float difference = alpha - beta;
    while (difference < -CV_PI) difference += 2.0f * static_cast<float>(CV_PI);
    while (difference > +CV_PI) difference -= 2.0f * static_cast<float>(CV_PI);
    return difference;
}