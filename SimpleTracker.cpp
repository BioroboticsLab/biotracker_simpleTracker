#include "SimpleTracker.h"

#include <QPainter>
#include <QGridLayout>
#include <QLineEdit>
#include <QSlider>

#include "TrackedFish.h"

#include <QGraphicsEllipseItem>

#include <biotracker/Registry.h>

#include <opencv2/opencv.hpp>
#include <QMutexLocker>

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
    , _numberOfObjects(4)
    , _mapper(Mapper(m_trackedObjects, _numberOfObjects))
    , _averageSpeedPx(6.0f)
    , _minContourSize(new QLabel("5", getToolsWidget()))
    , _numberOfErosions(new QLabel("2", getToolsWidget()))
    , _numberOfDilations(new QLabel("1", getToolsWidget()))
    , _backgroundWeight(new QLabel("0.85", getToolsWidget()))
    , _diffThreshold(new QLabel("25", getToolsWidget()))
{
    FishPose::_averageSpeed = _averageSpeedPx;
    FishPose::_averageSpeedSigma = std::sqrt(-(_averageSpeedPx*_averageSpeedPx/2) * (1/std::log(0.66f)));
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
    layout->addWidget(new QLabel("number of erosions"), 4, 0, 1, 2);
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
    if(!_backgroundInitialized){
        cv::cvtColor(frame.clone(), _background, CV_RGB2GRAY);
        _backgroundInitialized = true;
    }
    cv::Mat frameGRAY;
    cv::cvtColor(frame, frameGRAY, CV_RGB2GRAY);
    _background = (_background * _backgroundWeight->text().toFloat()) + (frameGRAY * (1.0f - _backgroundWeight->text().toFloat()));

    std::vector<std::vector<cv::Point>> contours;
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



    for( int i = 0; i < foreground.cols; i++){
        for( int j = 0; j < foreground.rows; j++){
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
    _mapper.map(contourEllipses, frameNumber);

    {
        QMutexLocker locker(&lastFrameLock);
        lastFrame = frame;
    }
}

void SimpleTracker::paint (size_t, BioTracker::Core::ProxyMat & p, const TrackingAlgorithm::View &view) {
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

        for( int i = 0; i < foreground.cols; i++){
            for( int j = 0; j < foreground.rows; j++){
                if(foreground.at<uchar>(cv::Point(i, j)) < _diffThreshold->text().toUInt()) {
                    foreground.at<uchar>(cv::Point(i, j)) = 0;
                }
            }
        }

        p.setMat(foreground);
    } else if(view.name == SimpleTracker::BackgroundView.name) {
//        p.setMat(_background);

        cv::RNG rng(12345);

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



        for( size_t i = 0; i < static_cast<size_t>(foreground.cols); i++){
            for( size_t j = 0; j < static_cast<size_t>(foreground.rows); j++){
                if(foreground.at<uchar>(cv::Point(i, j)) < _diffThreshold->text().toUInt()) {
                    foreground.at<uchar>(cv::Point(i, j)) = 0;
                }
            }
        }



        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(foreground, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        // TODO: erasing small contours really best option?
        for(size_t i = 0; i < contours.size(); i++) {
            if(contours[i].size() < _minContourSize->text().toUInt()) {
                contours.erase(contours.begin() + i);
                i--;
            }
        }

        cv::cvtColor(foreground, foreground, cv::COLOR_GRAY2RGB);

        for(size_t i = 0; i < contours.size(); i++) {
            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );
            cv::drawContours( foreground, contours, i, color, 2, 8);
        }

        p.setMat(foreground);
    } else {
        auto &image = p.getMat();
        {
            QMutexLocker locker(&lastFrameLock);
            if (lastFrame.empty()) return;
            image = lastFrame;
        }
    }
}

void SimpleTracker::paintOverlay(size_t frame, QPainter *painter, const View &view) {
    if(view.name == SimpleTracker::ForegroundView.name) {

    } else if(view.name == SimpleTracker::BackgroundView.name) {

    } else {
        for( size_t i = 0; i < m_trackedObjects.size(); i++){
//        for (BioTracker::Core::TrackedObject& trackedObject : m_trackedObjects) {
            TrackedFish& trackedFish = static_cast<TrackedFish&>(m_trackedObjects.at(i));
            std::shared_ptr<FishPose> fish;
            if(trackedFish.hasValuesAtFrame(frame)){
                fish = trackedFish.get<FishPose>(frame);
            } else {
                continue;
            }
            cv::Scalar color = fish->associated_color();
            painter->setPen(QPen(QColor(static_cast<int>(color[2]),
                                        static_cast<int>(color[1]),
                                        static_cast<int>(color[0])), 3
            ));
            float angleDeg = static_cast<float>(fish->angle() * 180.0f / CV_PI);
            painter->translate(fish->last_known_position().center.x, fish->last_known_position().center.y);
            painter->rotate(angleDeg);
            painter->drawEllipse(QPointF(0, 0), fish->last_known_position().size.width,
                                 fish->last_known_position().size.height);
            painter->rotate(-angleDeg);
            painter->drawText(QRectF(-5,-5,10.0f,10.0f), Qt::AlignCenter, std::to_string(trackedFish.getId()).c_str());
//            painter->drawText(QRectF(-5,-5,10.0f,10.0f), Qt::AlignCenter, std::to_string(i).c_str());
            painter->translate(-fish->last_known_position().center.x, -fish->last_known_position().center.y);


//            fish = trackedFish.estimateNextPose(frame);
//            painter->setPen(QPen(QColor(125, 125, 125), 1));
//            angleDeg = static_cast<float>(fish->angle() * 180.0f / CV_PI);
//            painter->translate(fish->last_known_position().center.x, fish->last_known_position().center.y);
//            painter->rotate(angleDeg);
//            painter->drawEllipse(QPointF(0, 0), fish->last_known_position().size.width,
//                                 fish->last_known_position().size.height);
//            painter->rotate(-angleDeg);
//            painter->drawText(QRectF(-5,-5,10.0f,10.0f), Qt::AlignCenter, std::to_string(i).c_str());
//            painter->translate(-fish->last_known_position().center.x, -fish->last_known_position().center.y);

        }

//        for (TrackedObject& trackedCandidate : _mapper.getFishCandidates()) {
//            std::shared_ptr<FishCandidate> candidate;
//            if(trackedCandidate.hasValuesAtFrame(frame)){
//                candidate = trackedCandidate.get<FishCandidate>(frame);
//            } else {
//                continue;
////                candidate = std::dynamic_pointer_cast<FishCandidate>(trackedCandidate.top());
//            }
//            cv::Scalar color = candidate->associated_color();
//            painter->setPen(QPen(QColor(static_cast<int>(color[2]),
//                                        static_cast<int>(color[1]),
//                                        static_cast<int>(color[0]))
//            ));
//
//
//            float angleDeg = static_cast<float>(candidate->angle() * 180.0f / CV_PI);
//            painter->translate(candidate->last_known_position().center.x, candidate->last_known_position().center.y);
//            painter->rotate(angleDeg);
//            painter->drawEllipse(QPointF(0, 0), candidate->last_known_position().size.width,
//                                 candidate->last_known_position().size.height);
//            painter->rotate(-angleDeg);
//            painter->drawText(QRectF(-5,-5,10.0f,10.0f), Qt::AlignCenter, std::to_string(trackedCandidate.getId()).c_str());
//            painter->translate(-candidate->last_known_position().center.x, -candidate->last_known_position().center.y);
//        }
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
    FishPose::_averageSpeed = _averageSpeedPx;
    FishPose::_averageSpeedSigma = std::sqrt(-(_averageSpeedPx*_averageSpeedPx/2) * (1/std::log(0.66f)));
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
