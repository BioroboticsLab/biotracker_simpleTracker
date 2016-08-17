#ifndef SimpleTracker_H
#define SimpleTracker_H

#include <opencv2/opencv.hpp>
#include <QMutex>

#include <biotracker/TrackingAlgorithm.h>
#include "TrackedFish.h"
#include "FishCandidate.h"
#include "Mapper.h"

class SimpleTracker : public BioTracker::Core::TrackingAlgorithm {
public:
	static const float MAX_TRACK_DISTANCE_PER_FRAME;
	static const float MAX_TRACK_DISTANCE;
	static const int   CANDIDATE_SCORE_THRESHOLD;
	static const int   MAX_NUMBER_OF_TRACKED_OBJECTS;

	SimpleTracker(BioTracker::Core::Settings &settings);

	void track(size_t frameNumber, const cv::Mat &frame) override;
	void paint(size_t frameNumber, BioTracker::Core::ProxyMat &m, View const &view = OriginalView) override;
	void paintOverlay(size_t frameNumber, QPainter *painter, View const &view = OriginalView) override;

	void prepareSave() override;

	void postLoad() override;

	void keyPressEvent(QKeyEvent *ev) override;

	//mouse click and move events
	void mouseMoveEvent(QMouseEvent *e);
	void mousePressEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);
	void mouseWheelEvent(QWheelEvent *e);

private:
	cv::BackgroundSubtractorMOG2 _bg_subtractor;
	std::vector<FishCandidate>		 _fish_candidates;
    Mapper							 _mapper;

	QMutex  lastFrameLock;
	cv::Mat lastFrame;
};

#endif
