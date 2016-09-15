#include "FishPose.h"

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>

float FishPose::_averageSpeed;
float FishPose::_averageSpeedSigma;


FishPose::FishPose() {}

FishPose::FishPose(size_t age, cv::RotatedRect position) {
    _age_of_last_known_position = age;
    _last_known_position = position;
}

FishPose::FishPose(FishPose& other) {
    _last_known_position = other.last_known_position();
    _age_of_last_known_position = other.age_of_last_known_position();
    _associated_color = other.associated_color();
    _angle = other.angle();
}

void FishPose::setNextPosition(cv::RotatedRect position) {
    _age_of_last_known_position = 1;
    _last_known_position = position;
}

void FishPose::setNextPositionUnknown() {
    ++_age_of_last_known_position;
}

cv::RotatedRect FishPose::last_known_position() const {
    return _last_known_position;
}

size_t FishPose::age_of_last_known_position() const {
    return _age_of_last_known_position;
}

void FishPose::set_associated_color(const cv::Scalar& color) {
    _associated_color = color;
}

cv::Scalar FishPose::associated_color() const {
    return _associated_color;
}
void FishPose::setAngle(float angle) {
    _angle = angle;
}

float FishPose::angle() {
    return _angle;
}

float FishPose::calculateProbabilityOfIdentity(const cv::RotatedRect &second, float &distance, float angleImportance)
{
    distance = static_cast<float>(sqrt(double((_last_known_position.center.x - second.center.x) *
                                               (_last_known_position.center.x - second.center.x) +
                                               (_last_known_position.center.y - second.center.y) *
                                               (_last_known_position.center.y - second.center.y))));

    // we are not sure about the direction of the angle, so just use the closer one
    const float absAngleDifference = std::min(
            std::abs(angleDifference(_angle, static_cast<float>(second.angle * CV_PI / 180.0f))),
            std::abs(angleDifference(static_cast<float>(_angle + CV_PI), static_cast<float>(second.angle * CV_PI / 180.0f))));

    auto normalDistributionPdf = [](double sigma, double distance2)
    {
        return std::exp(-(distance2 * distance2) / (2.0 * sigma * sigma));
    };

    // 0.5cm per millisecond sounds good as a ~66% estimate - that is about 18 km/h
    // the factors are a hand-optimized scaling of the distribution's dropoff
    const float distanceSigma = FishPose::_averageSpeedSigma;
    const double distanceIdentity = normalDistributionPdf(distanceSigma, distance);

    const double angleSigma = 10.0 * CV_PI / 2.0 * 0.05;
//    std::cout << "angleSigma: \t" << angleSigma << std::endl;
    const double angleIdentity = normalDistributionPdf(angleSigma, absAngleDifference);
//    std::cout << "distance: \t" << int(100.0f * distanceIdentity) << "\t\t\t angle: " << int(100.0f * angleIdentity) << std::endl;
//    std::cout << "\t\t^- " << distance << "\t\t\t^-" << (180.0 * absAngleDifference / CV_PI) << "(" << _angle << " - " << second.angle << ")" << std::endl;
    return static_cast<float>((1.0f - angleImportance) * distanceIdentity + angleImportance * angleIdentity);
}


float FishPose::angleDifference(float alpha, float beta)
{
    float difference = alpha - beta;
    while (difference < -CV_PI) difference += 2.0f * static_cast<float>(CV_PI);
    while (difference > +CV_PI) difference -= 2.0f * static_cast<float>(CV_PI);
    return difference;
}

CEREAL_REGISTER_TYPE(FishPose)
