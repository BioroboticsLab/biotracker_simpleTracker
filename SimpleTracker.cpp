#include "SimpleTracker.h"

#include <biotracker/Registry.h>

#include <opencv2/opencv.hpp>
#include <QMutexLocker>

class BgSub : public cv::BackgroundSubtractorMOG2 {
public:
    BgSub()
    {
        bShadowDetection = true;
        nShadowDetection = 0;
        fTau = 0.90f;
        nmixtures = 30;
    }
};

const float SimpleTracker::MAX_TRACK_DISTANCE_PER_FRAME = 5;

const float SimpleTracker::MAX_TRACK_DISTANCE = 1000;

const int SimpleTracker::CANDIDATE_SCORE_THRESHOLD = 30;

const int SimpleTracker::MAX_NUMBER_OF_TRACKED_OBJECTS = 5;

struct isYounger {
  bool operator() (const BioTracker::Core::TrackedObject& lhs,
                   const BioTracker::Core::TrackedObject& rhs) const
  {
      return std::dynamic_pointer_cast<TrackedFish>(lhs.top())->age_of_last_known_position() <
             std::dynamic_pointer_cast<TrackedFish>(rhs.top())->age_of_last_known_position();
  }
};

using namespace BioTracker::Core;

extern "C" {
    #ifdef _WIN32
    void __declspec(dllexport) registerTracker() {
    #else
    void registerTracker() {
    #endif
        BioTracker::Core::Registry::getInstance().registerTrackerType<SimpleTracker>("SimpleTracker");
    }
}

SimpleTracker::SimpleTracker(BioTracker::Core::Settings &settings)
    : TrackingAlgorithm(settings)
    , _bg_subtractor(BgSub())
{}

void SimpleTracker::track(size_t frameNumber, const cv::Mat &frame) {
    // TODO history, handle frame number,...
    static cv::RNG rng(12345);

    std::vector<std::vector<cv::Point> > contours;
    cv::Mat foreground;
    cv::Mat background;
    _bg_subtractor.operator ()(frame, foreground);
    _bg_subtractor.getBackgroundImage(background);

    cv::erode(foreground, foreground, cv::Mat());

    cv::dilate(foreground, foreground, cv::Mat());
    cv::dilate(foreground, foreground, cv::Mat());

    cv::findContours(foreground, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    std::vector<cv::Point2f> center(contours.size());
    float radius_dummy;
    for(size_t i = 0; i < contours.size(); i++) {
        cv::minEnclosingCircle(contours[i], center[i], radius_dummy);
    }
    std::vector<cv::Point2f> contourCenters(center.begin(), center.end());
    // Now we know the centers of all the detected contours in the picture (center)

    // TRACKING
    // (1) Find the next contour belonging to each tracked fish (recently found fish first)
    std::sort(m_trackedObjects.begin(), m_trackedObjects.end(), isYounger());
    for (BioTracker::Core::TrackedObject& trackedObject : m_trackedObjects)
    {
        // get object from last frame if available, otherwise the newest one
        std::shared_ptr<BioTracker::Core::ObjectModel> lastTrackedObject =
                trackedObject.count(frameNumber - 1) ?
                    trackedObject.get(frameNumber - 1) :
                    trackedObject.top();
        // dynamic cast to TrackedFish
        auto lastTrackedFish = std::dynamic_pointer_cast<TrackedFish>(
                    lastTrackedObject);
        // deep copy
        std::shared_ptr<TrackedFish> trackedFish =
                std::make_shared<TrackedFish>(*lastTrackedFish);
        unsigned age = trackedFish->age_of_last_known_position();
        float maxRange = std::min(age * MAX_TRACK_DISTANCE_PER_FRAME, MAX_TRACK_DISTANCE);
        cv::Point2f currentPosition = trackedFish->last_known_position();

        cv::Point2f minDistanceContour;
        float minDistance = MAX_TRACK_DISTANCE + 1;
        for (const cv::Point2f& contour : contourCenters) {
            cv::Point distPoint = contour - currentPosition;
            float distance = static_cast<float>(cv::sqrt((distPoint.x * distPoint.x) + (distPoint.y * distPoint.y)));
            if (distance < minDistance) {
                minDistance = distance;
                minDistanceContour = contour;
            }
        }
        if (minDistance < maxRange) {
            trackedFish->setNextPosition(minDistanceContour);
            contourCenters.erase(std::find(contourCenters.begin(), contourCenters.end(), minDistanceContour));
        } else {
            trackedFish->setNextPositionUnknown();
        }
        // store TrackedFish
        trackedObject.add(frameNumber, trackedFish);
    }

    // (2) Try to find contours belonging to fish candidates, promoting to TrackedFish as appropriate
    if (m_trackedObjects.size() < MAX_NUMBER_OF_TRACKED_OBJECTS) {
        std::vector<FishCandidate> candidates_to_promote;
        std::vector<FishCandidate> candidates_to_drop;
        std::sort(m_trackedObjects.begin(), m_trackedObjects.end(), isYounger());
        for (FishCandidate& candidate : _fish_candidates) {
            cv::Point2f currentPosition = candidate.last_known_position();
            float minDistance = MAX_TRACK_DISTANCE + 1;
            cv::Point2f minDistanceContour;
            for (const cv::Point2f& contour : contourCenters) {
                cv::Point2f distPoint = contour - currentPosition;
                float distance = cv::sqrt((distPoint.x * distPoint.x) + (distPoint.y * distPoint.y));
                if (distance < minDistance) {
					minDistance = distance;
                    minDistanceContour = contour;
                }
            }

            if (minDistance < std::min(MAX_TRACK_DISTANCE_PER_FRAME * candidate.age_of_last_known_position(), MAX_TRACK_DISTANCE)) {
                candidate.setNextPosition(minDistanceContour);
                candidate.increaseScore();
                contourCenters.erase(std::find(contourCenters.begin(), contourCenters.end(), minDistanceContour));
                if (candidate.score() > CANDIDATE_SCORE_THRESHOLD) {
                    candidates_to_promote.push_back(candidate);
                }
            } else {
                candidate.decreaseScore();
                candidate.setNextPositionUnknown();
                if (candidate.score() < 0) {
                    candidates_to_drop.push_back(candidate);
                }
            }
        }

        // (2.5) Drop/Promote candidates.
        for (FishCandidate& promoted : candidates_to_promote) {
            BioTracker::Core::TrackedObject newObject(m_trackedObjects.size() + 1);
            auto newFish = std::make_shared<TrackedFish>();
            newFish->setNextPosition(promoted.last_known_position());
            newFish->set_associated_color(cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
            _fish_candidates.erase(std::find(_fish_candidates.begin(), _fish_candidates.end(), promoted));
            newObject.add(frameNumber, newFish);
            m_trackedObjects.push_back(newObject);
        }
        candidates_to_promote.clear();
        for (FishCandidate& dropped : candidates_to_drop) {
            _fish_candidates.erase(std::find(_fish_candidates.begin(), _fish_candidates.end(), dropped));
        }
        candidates_to_drop.clear();

        // (3) Create new candidates for unmatched contours
        for (cv::Point2f& contour : contourCenters) {
            FishCandidate newCandidate;
            newCandidate.setNextPosition(contour);
            _fish_candidates.push_back(newCandidate);
        }
    } else {
        _fish_candidates.clear();
    }

    {
        QMutexLocker locker(&lastFrameLock);
        lastFrame = frame;
    }
}

void SimpleTracker::paint (size_t , BioTracker::Core::ProxyMat & p, const TrackingAlgorithm::View &) {
    auto &image = p.getMat();
    {
        QMutexLocker locker(&lastFrameLock);
        if (lastFrame.empty()) return;
        image = lastFrame;
    }

    for (BioTracker::Core::TrackedObject& trackedObject : m_trackedObjects) {
        std::shared_ptr<TrackedFish> fish =
                std::dynamic_pointer_cast<TrackedFish>(trackedObject.top());
        cv::circle(image, fish->last_known_position(), 3, fish->associated_color(), -1, 8, 0 );
    }

    for (FishCandidate& candidate : _fish_candidates) {
        cv::Scalar color(255, 0, 0);
        cv::circle(image, candidate.last_known_position(), 2, color, -1, 8, 0 );
    }
}

void SimpleTracker::paintOverlay(size_t , QPainter *, const View &) { }


void SimpleTracker::prepareSave() { }

void SimpleTracker::postLoad() { }
// =========== I O = H A N D L I N G ============


// ============== Keyboard ==================

void SimpleTracker::keyPressEvent(QKeyEvent *ev) { }

// ============== Mouse ==================

void SimpleTracker::mousePressEvent(QMouseEvent * e) { }

void SimpleTracker::mouseMoveEvent(QMouseEvent * e) { }

void SimpleTracker::mouseReleaseEvent(QMouseEvent * e) { }

void SimpleTracker::mouseWheelEvent(QWheelEvent *) { }
