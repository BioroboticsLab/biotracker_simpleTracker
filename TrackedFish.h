#ifndef TrackedFish_H
#define TrackedFish_H

#include <biotracker/serialization/TrackedObject.h>

#include <cereal/access.hpp>
#include <opencv2/opencv.hpp>

class TrackedFish : public BioTracker::Core::ObjectModel {
public:
    TrackedFish() {}
    virtual ~TrackedFish() override {}

    void setNextPosition(cv::Point2f position);
    void setNextPositionUnknown();

    cv::Point2f last_known_position() const;
    unsigned age_of_last_known_position() const;

    void set_associated_color(const cv::Scalar& color);
    cv::Scalar associated_color() const;

    void setAngle(float angle);
    float angle();

    float calculateProbabilityOfIdentity(const cv::RotatedRect &second, float angleImportance);

protected:
    cv::Point2f _last_known_position;
    unsigned    _age_of_last_known_position;
    cv::Scalar  _associated_color;
    float       _angle;

private:
    float angleDifference(float alpha, float beta);

    friend class cereal::access;
    template <class Archive>
    void serialize(Archive& ar)
    {
		ar(CEREAL_NVP(_last_known_position),
		   CEREAL_NVP(_age_of_last_known_position),
		   CEREAL_NVP(_associated_color),
           CEREAL_NVP(_angle));
    }
};

#endif
