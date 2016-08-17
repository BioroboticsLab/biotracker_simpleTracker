#include "TrackedFish.h"

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>

void TrackedFish::setNextPosition(cv::Point2f position) {
    _age_of_last_known_position = 1;
    _last_known_position = position;
}

void TrackedFish::setNextPositionUnknown() {
    ++_age_of_last_known_position;
}

cv::Point2f TrackedFish::last_known_position() const {
    return _last_known_position;
}

unsigned TrackedFish::age_of_last_known_position() const {
    return _age_of_last_known_position;
}

void TrackedFish::set_associated_color(const cv::Scalar& color) {
    _associated_color = color;
}

cv::Scalar TrackedFish::associated_color() const {
    return _associated_color;
}
void TrackedFish::setAngle(float angle) {
    _angle = angle;
}

float TrackedFish::angle() {
    return _angle;
}

float TrackedFish::calculateProbabilityOfIdentity(const cv::RotatedRect &second, float angleImportance)
{
    const float distance = static_cast<float>(sqrt(double((_last_known_position.x - second.center.x) *
                                               (_last_known_position.x - second.center.x) +
                                               (_last_known_position.y - second.center.y) *
                                               (_last_known_position.y - second.center.y))));

    // we are not sure about the direction of the angle, so just use the closer one
    const float absAngleDifference = std::min(
            std::abs(angleDifference(_angle, second.angle * (static_cast<float>(CV_PI) / 180.0f))),
            std::abs(angleDifference(_angle + static_cast<float>(CV_PI), second.angle * (static_cast<float>(CV_PI) / 180.0f))));

    auto normalDistributionPdf = [](double sigma, double distance2)
    {
        return std::exp(-(distance2 * distance2) / (2.0 * sigma * sigma));
    };


    auto adjustForFrameSpeed = [](const float &sigma)
    {
        // TODO: need to replace 50 with real ms per frame (depending on fps of video)
        return (sigma / 1000.0f) * static_cast<float>(50);
    };

    // 0.5cm per millisecond sounds good as a ~66% estimate - that is about 18 km/h
    // the factors are a hand-optimized scaling of the distribution's dropoff
    const float distanceSigma = adjustForFrameSpeed(100.0f * 0.5f);
	// TODO: replace 0.0001 with cm/px
    const double distanceIdentity = normalDistributionPdf(distanceSigma, distance * 0.02);

    const double angleSigma = adjustForFrameSpeed(10.0f * static_cast<float>(CV_PI) / 2.0f);
    const double angleIdentity = normalDistributionPdf(angleSigma, absAngleDifference);
    /*std::cout << "distance: \t" << int(100.0f * distanceIdentity) << "\t\t\t angle: " << int(100.0f * angleIdentity) << std::endl;
    std::cout << "\t\t^- " << distance << "\t\t\t^-" << (180.0f * absAngleDifference / CV_PI) << "(" << first.orientation_deg() << " - " << second.orientation_deg() << ")" << std::endl;
    std::cout << "\tSPFs:\t" << FishTrackerThread::instance()->getRealTimePerFrameMs() << std::endl;*/
    return static_cast<float>((1.0f - angleImportance) * distanceIdentity + angleImportance * angleIdentity);
}


float TrackedFish::angleDifference(float alpha, float beta)
{
    float difference = alpha - beta;
    while (difference < -CV_PI) difference += 2.0f * static_cast<float>(CV_PI);
    while (difference > +CV_PI) difference -= 2.0f * static_cast<float>(CV_PI);
    return difference;
}

CEREAL_REGISTER_TYPE(TrackedFish)
