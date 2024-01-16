#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <vector>
#include <algorithm>

class MedianFilter {
public:
  MedianFilter(int windowSize);
  float update(float newValue);

private:
  int windowSize_;
  std::vector<float> sortedValues_;
  int currentIndex_;
};

#endif // MEDIAN_FILTER_H
