#include "MedianFilter.h"

MedianFilter::MedianFilter(int windowSize) : windowSize_(windowSize), sortedValues_(windowSize_, 0), currentIndex_(0) {}

float MedianFilter::update(float newValue) {
  sortedValues_[currentIndex_] = newValue;
  currentIndex_ = (currentIndex_ + 1) % windowSize_;

  std::vector<float> temp(sortedValues_);
  std::sort(temp.begin(), temp.end());

  return temp[windowSize_ / 2];
}