#include "Mapper.h"

#include <QPainter>
#include "SimpleTracker.h"
#include "TrackedFish.h"

#include <biotracker/Registry.h>

using namespace BioTracker::Core;
// ================= P U B L I C ====================
Mapper::Mapper(std::vector<TrackedObject> &trackedObjects, size_t numberOfObjects, size_t framesTillPromotion) :
    m_trackedObjects(trackedObjects)
    , _numberOfObjects(numberOfObjects)
    , _framesTillPromotion(framesTillPromotion)
    , _lastId(1)
{
    _fishCandidates = std::vector<TrackedObject>();
}

void Mapper::map(std::vector<cv::RotatedRect> &contourEllipses, size_t frame){
    static cv::RNG rng(12345);
    // (1) Find the next contour belonging to each tracked fish

    std::vector<TrackedObject> fishes(m_trackedObjects);
    for (size_t i = 0; i < fishes.size(); i++){
        if(!fishes.at(i).hasValuesAtFrame(frame - 1)){
            fishes.erase(fishes.begin() + i);
            i--;
        }
    }
    size_t nrOfObjectsInFrame = fishes.size();
    std::vector<std::tuple<size_t, std::shared_ptr<FishPose>>> newFishes;

    while(!fishes.empty() && !contourEllipses.empty()) {
        newFishes.push_back(mergeContoursToFishes(0, frame - 1, fishes, contourEllipses, std::vector<size_t>()));
    }

	for (size_t j = 0; j < m_trackedObjects.size(); j++) {
    //for(size_t i = 0; i < newFishes.size(); i++){
        //for(size_t j = 0; j < m_trackedObjects.size(); j++){
		for (size_t i = 0; i < newFishes.size(); i++) {
			size_t id;
			std::shared_ptr<FishPose> fp;
			std::tie(id, fp) = newFishes[i];
            if(id == m_trackedObjects[j].getId()){
                if(!fp){
                    continue;
                }
                m_trackedObjects[j].add(frame, fp);
            }
        }
//		if (!m_trackedObjects[j].hasValuesAtFrame(frame)) {
//			std::shared_ptr<FishPose> a = std::make_shared<FishPose>(*(m_trackedObjects[j].get<FishPose>(frame - 1).get()));
//			a->setNextPositionUnknown();
//			m_trackedObjects[j].add(frame, a);
//		}
    }

    // (2) Try to find contours belonging to fish candidates, promoting to FishPose as appropriate
    if (nrOfObjectsInFrame < _numberOfObjects) {
        for(size_t i = 0; i < _fishCandidates.size(); i++){
            if(!_fishCandidates[i].hasValuesAtFrame(frame - 1)){
                _fishCandidates.erase(_fishCandidates.begin() + i);
                i--;
            }
        }
        std::vector<TrackedObject> fishCandidates(_fishCandidates);
        std::vector<std::tuple<size_t, std::shared_ptr<FishPose>>> newFishCandidates;

        while(!fishCandidates.empty() && !contourEllipses.empty()) {
            newFishCandidates.push_back(mergeContoursToFishes(0, frame - 1, fishCandidates, contourEllipses,
                                                              std::vector<size_t>()));
        }

        for(size_t j = 0; j < _fishCandidates.size(); j++){
            for(size_t i = 0; i < newFishCandidates.size(); i++){
                size_t id;
                std::shared_ptr<FishPose> fp;
                std::tie(id, fp) = newFishCandidates[i];
                if(id == _fishCandidates[j].getId()){
                    if(!fp){
                        continue;
                    }
                    std::shared_ptr<FishCandidate> a = std::make_shared<FishCandidate>(*fp.get(),
                                                                                       _fishCandidates[j].get<FishCandidate>(frame - 1)->score());
                    a->increaseScore();
                    _fishCandidates[j].add(frame, a);
                }
            }
            if(!_fishCandidates[j].hasValuesAtFrame(frame)){
                std::shared_ptr<FishCandidate> a = std::make_shared<FishCandidate>(*(_fishCandidates[j].get<FishCandidate>(frame-1).get()));
                a->setNextPositionUnknown();
                _fishCandidates[j].add(frame, a);
            }
        }
        // (2.5) Drop/Promote candidates.
        for(int i = 0; i < static_cast<int>(_fishCandidates.size()) && nrOfObjectsInFrame < _numberOfObjects; i++){
            if(_fishCandidates[i].hasValuesAtFrame(frame)){
                // TODO: Score Threshold needed
                int score = _fishCandidates[i].get<FishCandidate>(frame)->score();
                if(static_cast<size_t>(score) >= _framesTillPromotion){
                    std::move(_fishCandidates.begin() + i, _fishCandidates.begin() + i + 1, std::back_inserter(m_trackedObjects));
                    _fishCandidates.erase(_fishCandidates.begin() + i);
                    i--;
                    nrOfObjectsInFrame++;
                } else if (score < 0){
                    _fishCandidates.erase(_fishCandidates.begin() + i);
                    i--;
                }
            } else {
                _fishCandidates.erase(_fishCandidates.begin() + i);
                i--;
            }
        }

        // (3) Create new candidates for unmatched contours
        for (cv::RotatedRect& contour : contourEllipses) {
            BioTracker::Core::TrackedObject newObject(_lastId);
            _lastId++;
            auto newFish = std::make_shared<FishCandidate>();
            newFish->setNextPosition(contour);
            newFish->set_associated_color(cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
            newFish->setAngle(contour.angle * (static_cast<float>(CV_PI) / 180.0f));
            newObject.add(frame, newFish);
            _fishCandidates.push_back(newObject);
        }
    }
    if (nrOfObjectsInFrame >= _numberOfObjects) {
        _fishCandidates.clear();
    }
}


void Mapper::setNumberOfObjects(size_t numberOfObjects){
    _numberOfObjects = numberOfObjects;
}
void Mapper::setFramesTillPromotion(size_t framesTillPromotion){
    _framesTillPromotion = framesTillPromotion;
}

std::vector<BioTracker::Core::TrackedObject>& Mapper::getFishCandidates(){
    return _fishCandidates;
}

// ================ P R I V A T E ===================

std::tuple<size_t, std::shared_ptr<FishPose>> Mapper::mergeContoursToFishes(size_t fishIndex, size_t frame,
                                                                            std::vector<TrackedObject> &fishes,
                                                                            std::vector<cv::RotatedRect> &contourEllipses,
                                                                            std::vector<size_t> alreadyTestedIndizies)
{
    TrackedFish trackedFish = static_cast<TrackedFish&>(fishes.at(fishIndex));
    size_t trackedId = trackedFish.getId();

    int np(-1);
    float score(0);

    std::tie(np, score) = getNearestIndexFromFishPoses(trackedFish.getPoseForMapping(frame), contourEllipses);

    if(np == -1){
        fishes.erase(fishes.begin() + fishIndex);
        return std::make_tuple(trackedId, nullptr);
    }
//    if(score < 0.3){
//        fishes.erase(fishes.begin() + fishIndex);
//        return std::make_tuple(trackedFish.getId(), std::make_shared<FishPose>(*(trackedFish.get<FishPose>(frame).get())));
//    }

    auto fp = std::make_shared<FishPose>(trackedFish.get<FishPose>(frame)->age_of_last_known_position(),
                                         contourEllipses.at(np));
//    fp->setNextPosition(contourEllipses.at(np));
    fp->setAngle(contourEllipses.at(np).angle * (static_cast<float>(CV_PI) / 180.0f));

    if(std::find(alreadyTestedIndizies.begin(), alreadyTestedIndizies.end(), fishIndex) == alreadyTestedIndizies.end()){
        auto newFish = std::make_shared<FishPose>();
        newFish->setNextPosition(contourEllipses[np]);
        newFish->setAngle(contourEllipses[np].angle * (static_cast<float>(CV_PI) / 180.0f));
        newFish->set_associated_color(fishes[fishIndex].get<FishPose>(frame)->associated_color());

        fishes.erase(fishes.begin() + fishIndex);
        contourEllipses.erase(contourEllipses.begin() + np);

        return std::make_tuple(trackedId, newFish);
    }

    std::vector<cv::RotatedRect> fps;
    for(size_t i = 0; i < fishes.size(); i++)
    {
        if(fishes[i].hasValuesAtFrame(frame)){
            TrackedFish& tFish = static_cast<TrackedFish&>(fishes.at(i));
//            FishPose tmpFish = tFish.getPoseForMapping(frame);
            FishPose tmpFish = *(tFish.get<FishPose>(frame).get());
            cv::RotatedRect tmpRect(tmpFish.last_known_position().center, tmpFish.last_known_position().size, tmpFish.angle());
            fps.push_back(tmpRect);
        }
    }

    int fpt;
    float fptScore;
    std::tie(fpt, fptScore) = getNearestIndexFromFishPoses(*(fp.get()), fps);

    if(fpt == static_cast<int>(fishIndex)) {
//        trackedFish.correctAngle(frame, contourEllipses[np]);
        auto newFish = std::make_shared<FishPose>();
        newFish->setNextPosition(contourEllipses[np]);
        newFish->setAngle(contourEllipses[np].angle * (static_cast<float>(CV_PI) / 180.0f));
        newFish->set_associated_color(fishes[fishIndex].get<FishPose>(frame)->associated_color());

        fishes.erase(fishes.begin() + fishIndex);
        contourEllipses.erase(contourEllipses.begin() + np);

        return std::make_tuple(trackedId, newFish);
    } else {
        alreadyTestedIndizies.push_back(fishIndex);
        return mergeContoursToFishes(fpt, frame, fishes, contourEllipses, alreadyTestedIndizies);
    }
}

std::tuple<int , float> Mapper::getNearestIndexFromFishPoses(FishPose &fishPose,
                                                                const std::vector<cv::RotatedRect> &fishPoses)
{
    // for readability
    typedef std::tuple<int, float> PoseScoreTuple;
    enum { PoseIndex = 0, ScoreIndex = 1 };

    PoseScoreTuple bestTwo[] = { std::make_tuple(-1, -1.0f), std::make_tuple(-1, -1.0f) };

    for(size_t i = 0; i < fishPoses.size(); i++)
    {
        const cv::RotatedRect &possiblePose = fishPoses[i];
        // this takes angle-direction correction into account
        float distance = -1.0f;
        const float probabilityOfIdentity = fishPose.calculateProbabilityOfIdentity(possiblePose, distance);

        if(distance > 3 * FishPose::_averageSpeed * fishPose.age_of_last_known_position()){
            continue;
        }

        // figure out whether the probability is the new highest or second highest
        if (probabilityOfIdentity > std::get<ScoreIndex>(bestTwo[0])) // new best?
        {
            // move current best to second place
            bestTwo[1] = bestTwo[0];
            // and remember new best one
            bestTwo[0] = std::make_tuple(static_cast<int>(i), probabilityOfIdentity);
        }
        else if (probabilityOfIdentity > std::get<ScoreIndex>(bestTwo[1])) // new second place?
        {
            bestTwo[1] = std::make_tuple(static_cast<int>(i), probabilityOfIdentity);
        }
    }
    // score defaults to the probability of identity if only one pose was found
    float score = std::get<ScoreIndex>(bestTwo[0]);

    if (std::get<PoseIndex>(bestTwo[1]) != -1) // second pose found?
    {
        score = score / (score + std::get<ScoreIndex>(bestTwo[1]));
        // this will now always be above 50%, so rescale a bit
        score = 2.0f * (score - 0.5f);
    }

    if(std::isnan(score)){
        score = 0;
    }

    return std::make_tuple(std::get<PoseIndex>(bestTwo[0]), score);
}
