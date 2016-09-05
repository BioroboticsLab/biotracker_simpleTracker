#pragma once

#include <opencv2/opencv.hpp>
#include <QMutex>
#include <QLabel>
#include <QRadioButton>

#include <biotracker/TrackingAlgorithm.h>
#include "FishPose.h"
#include "FishCandidate.h"
#include "Mapper.h"

class SimpleTracker : public BioTracker::Core::TrackingAlgorithm {
Q_OBJECT
public:
    static const View BackgroundView;
    static const View ForegroundView;

	SimpleTracker(BioTracker::Core::Settings &settings);

	void track(size_t frameNumber, const cv::Mat &frame) override;
	void paint(size_t frameNumber, BioTracker::Core::ProxyMat &m, View const &view = OriginalView) override;
	void paintOverlay(size_t frameNumber, QPainter *painter, View const &view = OriginalView) override;

    void postConnect() override;

	void prepareSave() override;
	void postLoad() override;

	void keyPressEvent(QKeyEvent *ev) override;

	//mouse click and move events
	void mouseMoveEvent(QMouseEvent *e);
	void mousePressEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);
	void mouseWheelEvent(QWheelEvent *e);

private:
    bool                        _backgroundInitialized;
    cv::Mat                     _background;
    size_t                      _numberOfObjects;
    Mapper						_mapper;

	QMutex  lastFrameLock;
	cv::Mat lastFrame;

    float    _averageSpeedPx;
	enum { Darker = 0, Brighter = 1, Both = 2 };
    QRadioButton * _darker;
    QRadioButton * _brighter;
    QRadioButton * _both;


    QLabel *    _minContourSize;
    QLabel *    _numberOfErosions;
    QLabel *    _numberOfDilations;
    QLabel *    _backgroundWeight;
    QLabel *    _diffThreshold;

private Q_SLOTS:
    void setNumberOfObjects(const QString &newValue);
    void setAverageSpeedPx(const QString &newValue);
    void setMinContourSize(int newValue);
    void setNumberOfErosions(int newValue);
    void setNumberOfDilations(int newValue);
    void setBackgroundWeight(int newValue);
    void setDiffThreshold(int newValue);
    void substractionTypeChanged();
};
