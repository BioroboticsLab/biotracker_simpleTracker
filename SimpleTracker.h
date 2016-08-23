#ifndef SimpleTracker_H
#define SimpleTracker_H

#include <opencv2/opencv.hpp>
#include <QMutex>
#include <QLabel>

#include <biotracker/TrackingAlgorithm.h>
#include "TrackedFish.h"
#include "FishCandidate.h"
#include "Mapper.h"

class SimpleTracker : public BioTracker::Core::TrackingAlgorithm {
Q_OBJECT
public:
    static const View BackgroundView;
    static const View ForegroundView;

	static const float MAX_TRACK_DISTANCE_PER_FRAME;
	static const float MAX_TRACK_DISTANCE;
	static const int   CANDIDATE_SCORE_THRESHOLD;

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
	std::vector<FishCandidate>  _fish_candidates;
    Mapper						_mapper;

	QMutex  lastFrameLock;
	cv::Mat lastFrame;

    float    _averageSpeedPx;

    size_t      _numberOfObjects;
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
};

#endif
