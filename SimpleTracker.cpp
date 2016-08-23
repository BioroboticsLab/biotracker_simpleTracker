#include "SimpleTracker.h"

#include <QPainter>
#include <QGridLayout>
#include <QLineEdit>
#include <QSlider>

#include <biotracker/Registry.h>

#include <opencv2/opencv.hpp>
#include <QMutexLocker>

const float SimpleTracker::MAX_TRACK_DISTANCE_PER_FRAME = 5;

const float SimpleTracker::MAX_TRACK_DISTANCE = 1000;

const int SimpleTracker::CANDIDATE_SCORE_THRESHOLD = 30;

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
    , _backgroundInitialized(false)
    , _mapper(Mapper())
    , _averageSpeedPx(10.0f)
    , _numberOfObjects(4)
    , _minContourSize(new QLabel("5", getToolsWidget()))
    , _numberOfErosions(new QLabel("2", getToolsWidget()))
    , _numberOfDilations(new QLabel("1", getToolsWidget()))
    , _backgroundWeight(new QLabel("0.95", getToolsWidget()))
    , _diffThreshold(new QLabel("10", getToolsWidget()))
{
    TrackedFish::_averageSpeedSigma = std::sqrt(-(_averageSpeedPx*_averageSpeedPx/2) * (1/std::log(0.66f)));
    // initialize gui
    auto ui = getToolsWidget();
    auto layout = new QGridLayout();

    auto numberOfObjects = new QLineEdit();
    numberOfObjects->setText(QString::number(_numberOfObjects));
    connect(numberOfObjects, SIGNAL(textChanged(const QString &)), this, SLOT(setNumberOfObjects(const QString &)));
    layout->addWidget(new QLabel("number of objects"), 0, 0, 1, 2);
    layout->addWidget(numberOfObjects, 0, 2, 1, 1);

    auto averageSpeedPx = new QLineEdit();
    averageSpeedPx->setText(QString::number(_averageSpeedPx));
    connect(averageSpeedPx, SIGNAL(textChanged(const QString &)), this, SLOT(setAverageSpeedPx(const QString &)));
    layout->addWidget(new QLabel("average speed (px/frame)"), 1, 0, 1, 2);
    layout->addWidget(averageSpeedPx, 1, 2, 1, 1);

    auto minContourSize = new QSlider(Qt::Horizontal);
    minContourSize->setValue(_minContourSize->text().toInt());
    minContourSize->setMinimum(5);
    minContourSize->setMaximum(20);
    connect(minContourSize, SIGNAL(valueChanged(int)), this, SLOT(setMinContourSize(int)));
    layout->addWidget(new QLabel("minimal contour size"), 2, 0, 1, 2);
    layout->addWidget(_minContourSize, 2, 2, 1, 1);
    layout->addWidget(minContourSize, 3, 0, 1, 3);

    auto numberOfErosions = new QSlider(Qt::Horizontal);
    numberOfErosions->setValue(_numberOfErosions->text().toInt());
    numberOfErosions->setMinimum(0);
    numberOfErosions->setMaximum(5);
    connect(numberOfErosions, SIGNAL(valueChanged(int)), this, SLOT(setNumberOfErosions(int)));
    layout->addWidget(new QLabel("number of erasions"), 4, 0, 1, 2);
    layout->addWidget(_numberOfErosions, 4, 2, 1, 1);
    layout->addWidget(numberOfErosions, 5, 0, 1, 3);

    auto numberOfDilations = new QSlider(Qt::Horizontal);
    numberOfDilations->setValue(_numberOfDilations->text().toInt());
    numberOfDilations->setMinimum(0);
    numberOfDilations->setMaximum(5);
    connect(numberOfDilations, SIGNAL(valueChanged(int)), this, SLOT(setNumberOfDilations(int)));
    layout->addWidget(new QLabel("number of dilations"), 6, 0, 1, 2);
    layout->addWidget(_numberOfDilations, 6, 2, 1, 1);
    layout->addWidget(numberOfDilations, 7, 0, 1, 3);

    auto backgroundWeight = new QSlider(Qt::Horizontal);
    backgroundWeight->setValue(static_cast<int>(_backgroundWeight->text().toFloat() * 100));
    backgroundWeight->setMinimum(0);
    backgroundWeight->setMaximum(100);
    connect(backgroundWeight, SIGNAL(valueChanged(int)), this, SLOT(setBackgroundWeight(int)));
    layout->addWidget(new QLabel("Alpha"), 8, 0, 1, 2);
    layout->addWidget(_backgroundWeight, 8, 2, 1, 1);
    layout->addWidget(backgroundWeight, 9, 0, 1, 3);

    auto diffThreshold = new QSlider(Qt::Horizontal);
    diffThreshold->setValue(_diffThreshold->text().toInt());
    diffThreshold->setMinimum(0);
    diffThreshold->setMaximum(50);
    connect(diffThreshold, SIGNAL(valueChanged(int)), this, SLOT(setDiffThreshold(int)));
    layout->addWidget(new QLabel("Threshold"), 10, 0, 1, 2);
    layout->addWidget(_diffThreshold, 10, 2, 1, 1);
    layout->addWidget(diffThreshold, 11, 0, 1, 3);

    ui->setLayout(layout);
}

const TrackingAlgorithm::View SimpleTracker::ForegroundView {"Foreground"};
const TrackingAlgorithm::View SimpleTracker::BackgroundView {"Background"};

void SimpleTracker::track(size_t frameNumber, const cv::Mat &frame) {
    // TODO history, handle frame number,...
    static cv::RNG rng(12345);

    if(!_backgroundInitialized){
        cv::cvtColor(frame.clone(), _background, CV_RGB2GRAY);
        _backgroundInitialized = true;
    }
    cv::Mat frameGRAY;
    cv::cvtColor(frame, frameGRAY, CV_RGB2GRAY);
    _background = (_background * _backgroundWeight->text().toFloat()) + (frameGRAY * (1.0f - _backgroundWeight->text().toFloat()));

    std::vector<std::vector<cv::Point> > contours;
    cv::Mat foreground;
//    cv::absdiff(frameGRAY, _background, foreground);
    cv::subtract(_background, frameGRAY, foreground);
//    cv::subtract(frameGRAY, _background, foreground);

    for(size_t i = 0; i < _numberOfErosions->text().toUInt(); i++){
        cv::erode(foreground, foreground, cv::Mat());
    }

    for(size_t i = 0; i < _numberOfDilations->text().toUInt(); i++){
        cv::dilate(foreground, foreground, cv::Mat());
    }



    for( size_t i = 0; i < static_cast<size_t>(foreground.cols); i++){
        for( size_t j = 0; j < static_cast<size_t>(foreground.rows); j++){
            if(foreground.at<uchar>(cv::Point(i, j)) < _diffThreshold->text().toUInt()) {
                foreground.at<uchar>(cv::Point(i, j)) = 0;
            }
        }
    }



    cv::findContours(foreground, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    for(size_t i = 0; i < contours.size(); i++) {
        if(contours[i].size() < _minContourSize->text().toUInt()) {
            contours.erase(contours.begin() + i);
            i--;
        }
    }

    std::vector<cv::RotatedRect> contourEllipses(contours.size());

    for(size_t i = 0; i < contours.size(); i++) {
        contourEllipses[i] = cv::fitEllipse(cv::Mat(contours[i]));
    }
    // Now we know the centers of all the detected contours in the picture (center)

    // TRACKING
    // (1) Find the next contour belonging to each tracked fish (recently found fish first)
    std::sort(m_trackedObjects.begin(), m_trackedObjects.end(), isYounger());
    m_trackedObjects = _mapper.map(m_trackedObjects, contourEllipses, frameNumber);

    // (2) Try to find contours belonging to fish candidates, promoting to TrackedFish as appropriate
    if (m_trackedObjects.size() < _numberOfObjects) {
        std::vector<FishCandidate> candidates_to_promote;
        std::vector<FishCandidate> candidates_to_drop;
        std::sort(m_trackedObjects.begin(), m_trackedObjects.end(), isYounger());
        for (FishCandidate& candidate : _fish_candidates) {
            cv::Point2f currentPosition = candidate.last_known_position();
            float minDistance = MAX_TRACK_DISTANCE + 1;
            size_t minDistanceIndex;
            for (size_t i = 0; i < contourEllipses.size(); i++){
                cv::Point2f distPoint = contourEllipses[i].center - currentPosition;
                float distance = cv::sqrt((distPoint.x * distPoint.x) + (distPoint.y * distPoint.y));
                if (distance < minDistance) {
					minDistance = distance;
                    minDistanceIndex = i;
                }
            }

            if (minDistance < std::min(MAX_TRACK_DISTANCE_PER_FRAME * candidate.age_of_last_known_position(), MAX_TRACK_DISTANCE)) {
                candidate.setNextPosition(contourEllipses[minDistanceIndex].center);
                candidate.increaseScore();
				contourEllipses.erase(contourEllipses.begin() + minDistanceIndex);
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
			newFish->setAngle(promoted.angle());
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
		for (cv::RotatedRect& contour : contourEllipses) {
			FishCandidate newCandidate;
			newCandidate.setNextPosition(contour.center);
			newCandidate.setAngle(contour.angle * (static_cast<float>(CV_PI) / 180.0f));
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

void SimpleTracker::paint (size_t , BioTracker::Core::ProxyMat & p, const TrackingAlgorithm::View &view) {
    if(!_backgroundInitialized){
        cv::cvtColor(p.getMat(), _background, CV_RGB2GRAY);
        _backgroundInitialized = true;
    }
    if(view.name == SimpleTracker::ForegroundView.name) {
        cv::Mat frameGRAY;
        cv::cvtColor(p.getMat(), frameGRAY, CV_RGB2GRAY);
        cv::Mat foreground;

//        cv::subtract(frameGRAY, _background, foreground);
        cv::subtract(_background, frameGRAY, foreground);
//        cv::absdiff(frameGRAY, _background, foreground);

        for(size_t i = 0; i < _numberOfErosions->text().toUInt(); i++){
            cv::erode(foreground, foreground, cv::Mat());
        }

        for(size_t i = 0; i < _numberOfDilations->text().toUInt(); i++){
            cv::dilate(foreground, foreground, cv::Mat());
        }

        p.setMat(foreground);
    } else if(view.name == SimpleTracker::BackgroundView.name) {
//        cv::RNG rng(12345);
//
//        cv::Mat frameGRAY;
//        cv::cvtColor(p.getMat(), frameGRAY, CV_RGB2GRAY);
//        cv::Mat foreground;
////        cv::subtract(frameGRAY, _background, foreground);
//        cv::subtract(_background, frameGRAY, foreground);
////        cv::absdiff(frameGRAY, _background, foreground);
//
//        for(size_t i = 0; i < _numberOfErosions->text().toUInt(); i++){
//            cv::erode(foreground, foreground, cv::Mat());
//        }
//
//        for(size_t i = 0; i < _numberOfDilations->text().toUInt(); i++){
//            cv::dilate(foreground, foreground, cv::Mat());
//        }
//
//
//
//        for( size_t i = 0; i < static_cast<size_t>(foreground.cols); i++){
//            for( size_t j = 0; j < static_cast<size_t>(foreground.rows); j++){
//                if(foreground.at<uchar>(cv::Point(i, j)) < _diffThreshold->text().toUInt()) {
//                    foreground.at<uchar>(cv::Point(i, j)) = 0;
//                }
//            }
//        }
//
//
//
//        std::vector<std::vector<cv::Point>> contours;
//        cv::findContours(foreground, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
//        // TODO: erasing small contours really best option?
//        for(size_t i = 0; i < contours.size(); i++) {
//            if(contours[i].size() < _minContourSize->text().toInt()) {
//                contours.erase(contours.begin() + i);
//                i--;
//            }
//        }
//
//        cv::cvtColor(foreground, foreground, cv::COLOR_GRAY2RGB);
//
//        for(size_t i = 0; i < contours.size(); i++) {
//            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );
//            cv::drawContours( foreground, contours, i, color, 2, 8);
//        }
//
//        p.setMat(foreground);
        p.setMat(_background);
    } else {
        auto &image = p.getMat();
        {
            QMutexLocker locker(&lastFrameLock);
            if (lastFrame.empty()) return;
            image = lastFrame;
        }
    }
}

void SimpleTracker::paintOverlay(size_t , QPainter *painter, const View &view) {

    if(view.name == SimpleTracker::ForegroundView.name) {

    } else if(view.name == SimpleTracker::BackgroundView.name) {

    } else {
        for (BioTracker::Core::TrackedObject& trackedObject : m_trackedObjects) {
            std::shared_ptr<TrackedFish> fish =
                    std::dynamic_pointer_cast<TrackedFish>(trackedObject.top());
            cv::Scalar color = fish->associated_color();
            painter->setPen(QPen(QColor(static_cast<int>(color[2]),
                                        static_cast<int>(color[1]),
                                        static_cast<int>(color[0]))
            ));
            painter->drawEllipse(QPointF(fish->last_known_position().x, fish->last_known_position().y), 3, 3);
        }

        for (FishCandidate& candidate : _fish_candidates) {
            painter->setPen(QPen(QColor(0, 0, 255)));
            painter->drawEllipse(QPointF(candidate.last_known_position().x, candidate.last_known_position().y), 2, 2);
        }
    }
}

void SimpleTracker::postConnect() {
    Q_EMIT registerViews({ForegroundView, BackgroundView});
}


void SimpleTracker::prepareSave() { }

void SimpleTracker::postLoad() { }
// =========== I O = H A N D L I N G ============


// ============== Keyboard ===============

void SimpleTracker::keyPressEvent(QKeyEvent *) { }

// ================ Mouse ================

void SimpleTracker::mousePressEvent(QMouseEvent *) { }

void SimpleTracker::mouseMoveEvent(QMouseEvent *) { }

void SimpleTracker::mouseReleaseEvent(QMouseEvent *) { }

void SimpleTracker::mouseWheelEvent(QWheelEvent *) { }

// ============== H E L P E R ====================

// =============== S L O T S =====================

void SimpleTracker::setNumberOfObjects(const QString &newValue){
    _numberOfObjects = newValue.toUInt();
}

void SimpleTracker::setAverageSpeedPx(const QString &newValue){
    _averageSpeedPx = newValue.toFloat();
    TrackedFish::_averageSpeedSigma = std::sqrt(-(_averageSpeedPx*_averageSpeedPx/2) * (1/std::log(0.66f)));
}

void SimpleTracker::setMinContourSize(int newValue){
    _minContourSize->setText(QString::number(newValue));
}

void SimpleTracker::setNumberOfErosions(int newValue){
    _numberOfErosions->setText(QString::number(newValue));
}

void SimpleTracker::setNumberOfDilations(int newValue){
    _numberOfDilations->setText(QString::number(newValue));
}

void SimpleTracker::setDiffThreshold(int newValue){
    _diffThreshold->setText(QString::number(newValue));
}

void SimpleTracker::setBackgroundWeight(int newValue){
    float val = static_cast<float>(newValue) / 100.0f;
    _backgroundWeight->setText(QString::number(val));
}