#include "SimpleTracker.h"

#include <QGridLayout>
#include <QLineEdit>
#include <QSlider>
#include <QGroupBox>
#include <QPushButton>

#include "TrackedFish.h"

#include <QGraphicsEllipseItem>

#include <biotracker/Registry.h>

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
    , _numberOfObjects(6)
    , _averageSpeedPx(75.0f)
    , _minContourSize(new QLabel("5", getToolsWidget()))
    , _maxContourSize(new QLabel("1500", getToolsWidget()))
    , _numberOfErosions(new QLabel("3", getToolsWidget()))
    , _numberOfDilations(new QLabel("1", getToolsWidget()))
    , _backgroundWeight(new QLabel("0.95", getToolsWidget()))
    , _diffThreshold(new QLabel("15", getToolsWidget()))
    , _framesTillPromotion(new QLabel("30", getToolsWidget()))
    , _mapper(new Mapper(m_trackedObjects, _numberOfObjects, _framesTillPromotion->text().toUInt()))
{
    FishPose::_averageSpeed = _averageSpeedPx;
    FishPose::_averageSpeedSigma = std::sqrt(-(_averageSpeedPx*_averageSpeedPx/2) * (1/std::log(0.33f)));
    // initialize gui
    auto ui = getToolsWidget();
    auto layout = new QGridLayout();

    auto numberOfObjects = new QLineEdit();
    numberOfObjects->setText(QString::number(_numberOfObjects));
    connect(numberOfObjects, SIGNAL(textChanged(const QString &)), this, SLOT(setNumberOfObjects(const QString &)));
    layout->addWidget(new QLabel("number of objects"), 0, 0, 1, 2);
    layout->addWidget(numberOfObjects, 0, 2, 1, 1);

    QGroupBox *groupBox = new QGroupBox(tr("Objects are:"));
    _darker = new QRadioButton(tr("Darker"));
    _brighter = new QRadioButton(tr("Brighter"));
    _both = new QRadioButton(tr("Both"));
    _darker->setChecked(true);

    QHBoxLayout *hbox = new QHBoxLayout;
    hbox->addWidget(_darker);
    hbox->addWidget(_brighter);
    hbox->addWidget(_both);
    groupBox->setLayout(hbox);
    layout->addWidget(groupBox, 1, 0, 1, 3);

    auto averageSpeedPx = new QLineEdit();
    averageSpeedPx->setText(QString::number(_averageSpeedPx));
    connect(averageSpeedPx, SIGNAL(textChanged(const QString &)), this, SLOT(setAverageSpeedPx(const QString &)));
    layout->addWidget(new QLabel("average speed (px/frame)"), 2, 0, 1, 2);
    layout->addWidget(averageSpeedPx, 2, 2, 1, 1);

    auto minContourSize = new QSlider(Qt::Horizontal);
    minContourSize->setMinimum(5);
    minContourSize->setMaximum(250);
    minContourSize->setValue(_minContourSize->text().toInt());
    connect(minContourSize, SIGNAL(valueChanged(int)), this, SLOT(setMinContourSize(int)));
    layout->addWidget(new QLabel("minimal contour size"), 3, 0, 1, 2);
    layout->addWidget(_minContourSize, 3, 2, 1, 1);
    layout->addWidget(minContourSize, 4, 0, 1, 3);

    auto maxContourSize = new QSlider(Qt::Horizontal);
    maxContourSize->setMinimum(5);
    maxContourSize->setMaximum(1500);
    maxContourSize->setValue(_maxContourSize->text().toInt());
    connect(maxContourSize, SIGNAL(valueChanged(int)), this, SLOT(setMaxContourSize(int)));
    layout->addWidget(new QLabel("maximal contour size"), 5, 0, 1, 2);
    layout->addWidget(_maxContourSize, 5, 2, 1, 1);
    layout->addWidget(maxContourSize, 6, 0, 1, 3);

    auto numberOfErosions = new QSlider(Qt::Horizontal);
    numberOfErosions->setMinimum(0);
    numberOfErosions->setMaximum(25);
    numberOfErosions->setValue(_numberOfErosions->text().toInt());
    connect(numberOfErosions, SIGNAL(valueChanged(int)), this, SLOT(setNumberOfErosions(int)));
    layout->addWidget(new QLabel("number of erosions"), 7, 0, 1, 2);
    layout->addWidget(_numberOfErosions, 7, 2, 1, 1);
    layout->addWidget(numberOfErosions, 8, 0, 1, 3);

    auto numberOfDilations = new QSlider(Qt::Horizontal);
    numberOfDilations->setMinimum(0);
    numberOfDilations->setMaximum(25);
    numberOfDilations->setValue(_numberOfDilations->text().toInt());
    connect(numberOfDilations, SIGNAL(valueChanged(int)), this, SLOT(setNumberOfDilations(int)));
    layout->addWidget(new QLabel("number of dilations"), 9, 0, 1, 2);
    layout->addWidget(_numberOfDilations, 9, 2, 1, 1);
    layout->addWidget(numberOfDilations, 10, 0, 1, 3);

    auto backgroundWeight = new QSlider(Qt::Horizontal);
    backgroundWeight->setMinimum(0);
    backgroundWeight->setMaximum(100);
    backgroundWeight->setValue(static_cast<int>(_backgroundWeight->text().toFloat() * 100));
    connect(backgroundWeight, SIGNAL(valueChanged(int)), this, SLOT(setBackgroundWeight(int)));
    layout->addWidget(new QLabel("Alpha"), 11, 0, 1, 2);
    layout->addWidget(_backgroundWeight, 11, 2, 1, 1);
    layout->addWidget(backgroundWeight, 12, 0, 1, 3);

    auto diffThreshold = new QSlider(Qt::Horizontal);
    diffThreshold->setMinimum(0);
    diffThreshold->setMaximum(255);
    diffThreshold->setValue(_diffThreshold->text().toInt());
    connect(diffThreshold, SIGNAL(valueChanged(int)), this, SLOT(setDiffThreshold(int)));
    layout->addWidget(new QLabel("Threshold"), 13, 0, 1, 2);
    layout->addWidget(_diffThreshold, 13, 2, 1, 1);
    layout->addWidget(diffThreshold, 14, 0, 1, 3);

    auto framesTillPromotion = new QSlider(Qt::Horizontal);
    framesTillPromotion->setMinimum(0);
    framesTillPromotion->setMaximum(250);
    framesTillPromotion->setValue(_framesTillPromotion->text().toInt());
    connect(framesTillPromotion, SIGNAL(valueChanged(int)), this, SLOT(setFramesTillPromotion(int)));
    layout->addWidget(new QLabel("frames till promotion"), 15, 0, 1, 2);
    layout->addWidget(_framesTillPromotion, 15, 2, 1, 1);
    layout->addWidget(framesTillPromotion, 16, 0, 1, 3);

    auto reset = new QPushButton("reset");
    connect(reset, SIGNAL(clicked()), this, SLOT(reset()));
    layout->addWidget(reset, 17, 0, 1, 3);

    ui->setLayout(layout);
}

const TrackingAlgorithm::View SimpleTracker::ForegroundView {"Foreground"};
const TrackingAlgorithm::View SimpleTracker::BackgroundView {"Background"};

void SimpleTracker::track(size_t frameNumber, const cv::Mat &frame) {
    if(!_backgroundInitialized || _background.rows != frame.rows || _background.cols != frame.cols){
        _background = frame.clone();
        cv::cvtColor(_background, _background, CV_RGB2GRAY);
        _backgroundInitialized = true;
    }
    cv::Mat frameGRAY;
    cv::cvtColor(frame, frameGRAY, CV_RGB2GRAY);
    _background = (_background * _backgroundWeight->text().toFloat()) + (frameGRAY * (1.0f - _backgroundWeight->text().toFloat()));

    _foregroundFrame = frameNumber;
    if(_darker->isChecked()){
        cv::subtract(_background, frameGRAY, _foreground);
    } else if(_brighter->isChecked()){
        cv::subtract(frameGRAY, _background, _foreground);
    } else if(_both->isChecked()){
        cv::absdiff(frameGRAY, _background, _foreground);
    }

    for(size_t i = 0; i < _numberOfErosions->text().toUInt(); i++){
        cv::erode(_foreground, _foreground, cv::Mat());
    }

    for(size_t i = 0; i < _numberOfDilations->text().toUInt(); i++){
        cv::dilate(_foreground, _foreground, cv::Mat());
    }



    for( int i = 0; i < _foreground.cols; i++){
        for( int j = 0; j < _foreground.rows; j++){
            if(_foreground.at<uchar>(cv::Point(i, j)) < _diffThreshold->text().toUInt()) {
                _foreground.at<uchar>(cv::Point(i, j)) = 0;
            }
        }
    }


    std::vector<std::vector<cv::Point>> contours;
    cv::Mat foreground = _foreground.clone();
    cv::findContours(foreground, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    cv::cvtColor(_foreground, _foreground, cv::COLOR_GRAY2RGB);

    for(size_t i = 0; i < contours.size(); i++) {
        if(contours[i].size() < _minContourSize->text().toUInt() || contours[i].size() > _maxContourSize->text().toUInt()) {
            contours.erase(contours.begin() + i);
            i--;
        }
    }

    _ellipses = std::vector<cv::RotatedRect>(contours.size());

    _ellipsesFrame = frameNumber;
    for(size_t i = 0; i < contours.size(); i++) {
        _ellipses[i] = cv::fitEllipse(cv::Mat(contours[i]));
    }
    // Now we know the centers of all the detected contours in the picture (center)

    // TRACKING
    std::vector<cv::RotatedRect> ellipses = _ellipses;
    _mapper->map(ellipses, frameNumber);

    {
        QMutexLocker locker(&lastFrameLock);
        lastFrame = frame;
    }
}

void SimpleTracker::paint (size_t frameNumber, BioTracker::Core::ProxyMat & p, const TrackingAlgorithm::View &view) {
    {
        QMutexLocker locker(&lastFrameLock);
        lastFrame = p.getMat();
    }
    if(!_backgroundInitialized || _background.rows != p.getMat().rows || _background.cols != p.getMat().cols){
        _background = p.getMat();
        cv::cvtColor(_background, _background, CV_RGB2GRAY);
        _backgroundInitialized = true;
    }
    if(view.name == SimpleTracker::ForegroundView.name) {
        if(_foregroundFrame != frameNumber){
            cv::Mat frameGRAY;
            cv::cvtColor(p.getMat(), frameGRAY, CV_RGB2GRAY);

            if(_darker->isChecked()){
                cv::subtract(_background, frameGRAY, _foreground);
            } else if(_brighter->isChecked()){
                cv::subtract(frameGRAY, _background, _foreground);
            } else if(_both->isChecked()){
                cv::absdiff(frameGRAY, _background, _foreground);
            }

            for(size_t i = 0; i < _numberOfErosions->text().toUInt(); i++){
                cv::erode(_foreground, _foreground, cv::Mat());
            }

            for(size_t i = 0; i < _numberOfDilations->text().toUInt(); i++){
                cv::dilate(_foreground, _foreground, cv::Mat());
            }

            for( int i = 0; i < _foreground.cols; i++){
                for( int j = 0; j < _foreground.rows; j++){
                    if(_foreground.at<uchar>(cv::Point(i, j)) < _diffThreshold->text().toUInt()) {
                        _foreground.at<uchar>(cv::Point(i, j)) = 0;
                    }
                }
            }
            cv::cvtColor(_foreground, _foreground, cv::COLOR_GRAY2RGB);
        }

        p.setMat(_foreground);
    } else if(view.name == SimpleTracker::BackgroundView.name) {
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

void SimpleTracker::paintOverlay(size_t frame, QPainter *painter, const View &view) {
    if(view.name == SimpleTracker::ForegroundView.name) {
        if(_ellipsesFrame != frame){
			std::vector<std::vector<cv::Point>> contours;
			cv::Mat foreground = _foreground.clone();

			cv::cvtColor(foreground, foreground, cv::COLOR_RGB2GRAY);
            cv::findContours(foreground, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

            for(size_t i = 0; i < contours.size(); i++) {
                if(contours[i].size() < _minContourSize->text().toUInt() || contours[i].size() > _maxContourSize->text().toUInt()) {
                    contours.erase(contours.begin() + i);
                    i--;
                }
            }

            _ellipses = std::vector<cv::RotatedRect>(contours.size());

            _ellipsesFrame = frame;
            for(size_t i = 0; i < contours.size(); i++) {
                _ellipses[i] = cv::fitEllipse(cv::Mat(contours[i]));
            }
        }

        for( size_t i = 0; i < _ellipses.size(); i++){
            cv::RotatedRect ellipse = _ellipses.at(i);
            painter->setPen(QPen(QColor(0, 0, 255), 3));
            painter->translate(ellipse.center.x, ellipse.center.y);
            painter->rotate(ellipse.angle);
            painter->drawEllipse(QPointF(0, 0), ellipse.size.width, ellipse.size.height);
            painter->rotate(-ellipse.angle);
            painter->translate(-ellipse.center.x, -ellipse.center.y);
        }
    } else if(view.name == SimpleTracker::BackgroundView.name) {

    } else {
        paintTrackedFishes(painter, frame);
    }
}

void SimpleTracker::postConnect() {
    Q_EMIT registerViews({ForegroundView, BackgroundView});
}


void SimpleTracker::prepareSave() { }

void SimpleTracker::postLoad() { }

void SimpleTracker::inputChanged() {
    resetTracks();
}

//=============== H E L P E R S ================

void SimpleTracker::paintTrackedFishes(QPainter *painter, size_t frame){
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
        painter->translate(-fish->last_known_position().center.x, -fish->last_known_position().center.y);
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

void SimpleTracker::resetTracks(){
    m_trackedObjects.clear();
    _backgroundInitialized = false;
    _mapper = new Mapper(m_trackedObjects, _numberOfObjects, _framesTillPromotion->text().toUInt());
}

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
    _mapper->setNumberOfObjects(newValue.toUInt());
}

void SimpleTracker::setAverageSpeedPx(const QString &newValue){
    _averageSpeedPx = newValue.toFloat();
    FishPose::_averageSpeed = _averageSpeedPx;
    FishPose::_averageSpeedSigma = std::sqrt(-(_averageSpeedPx*_averageSpeedPx/2) * (1/std::log(0.33f)));
}

void SimpleTracker::setMinContourSize(int newValue){
    _minContourSize->setText(QString::number(newValue));
    Q_EMIT update();
}

void SimpleTracker::setMaxContourSize(int newValue){
    _maxContourSize->setText(QString::number(newValue));
    Q_EMIT update();
}

void SimpleTracker::setNumberOfErosions(int newValue){
    _numberOfErosions->setText(QString::number(newValue));
    Q_EMIT update();
}

void SimpleTracker::setNumberOfDilations(int newValue){
    _numberOfDilations->setText(QString::number(newValue));
    Q_EMIT update();
}

void SimpleTracker::setDiffThreshold(int newValue){
    _diffThreshold->setText(QString::number(newValue));
    Q_EMIT update();
}

void SimpleTracker::setFramesTillPromotion(int newValue){
    _framesTillPromotion->setText(QString::number(newValue));
    _mapper->setFramesTillPromotion(static_cast<size_t>(newValue));
    Q_EMIT update();
}

void SimpleTracker::setBackgroundWeight(int newValue){
    float val = static_cast<float>(newValue) / 100.0f;
    _backgroundWeight->setText(QString::number(val));
    Q_EMIT update();
}

void SimpleTracker::reset(){
    resetTracks();
    Q_EMIT update();
}