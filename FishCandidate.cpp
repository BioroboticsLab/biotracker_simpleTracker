#include "FishCandidate.h"

FishCandidate::FishCandidate()
    : FishPose()
    , _score(1)
{}

FishCandidate::FishCandidate(FishPose& other, int score) : FishPose(other)
{
    _score = score;
}

FishCandidate::FishCandidate(FishCandidate& other) : FishPose(other)
{
    _score = other.score();
}

void FishCandidate::increaseScore() {
    ++_score;
}

void FishCandidate::decreaseScore() {
    _score -= 2;
}

int FishCandidate::score() const {
    return _score;
}

bool FishCandidate::operator==(const FishCandidate& other) const {
    return _last_known_position.center == other._last_known_position.center;
}

