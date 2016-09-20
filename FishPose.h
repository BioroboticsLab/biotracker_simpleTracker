#ifndef FishPose_H
#define FishPose_H

#include <biotracker/serialization/TrackedObject.h>

#include <cereal/access.hpp>
#include <opencv2/opencv.hpp>

class FishPose : public BioTracker::Core::ObjectModel {
public:
    FishPose();
    FishPose(size_t age, cv::RotatedRect position);
    FishPose(FishPose& other);
    virtual ~FishPose() override {}

    static float _averageSpeed;
    static float _averageSpeedSigma;

    void setNextPosition(cv::RotatedRect position);
    void setNextPositionUnknown();

    cv::RotatedRect last_known_position() const;
    unsigned age_of_last_known_position() const;

    void set_associated_color(const cv::Scalar& color);
    cv::Scalar associated_color() const;

    void setAngle(float angle);
    float angle();

    float calculateProbabilityOfIdentity(const cv::RotatedRect &second, float &distance, float angleImportance = 0.2f);

protected:
    cv::RotatedRect _last_known_position;
    unsigned        _age_of_last_known_position;
    cv::Scalar      _associated_color;
    float           _angle;

private:
    float angleDifference(float alpha, float beta);

    friend class cereal::access;
    template <class Archive>
    void serialize(Archive& ar)
    {
		ar(CEREAL_NVP(_last_known_position.center),
           CEREAL_NVP(_last_known_position.size),
           CEREAL_NVP(_last_known_position.angle),
		   CEREAL_NVP(_age_of_last_known_position),
		   CEREAL_NVP(_associated_color),
           CEREAL_NVP(_angle));
    }
};

#endif
