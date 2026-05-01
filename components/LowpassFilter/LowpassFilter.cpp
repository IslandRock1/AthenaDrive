#include <stdio.h>
#include "LowpassFilter.hpp"

LowpassFilter::LowpassFilter(float alpha) {
    _alpha = alpha;
}

float LowpassFilter::update(float value) {
    filtered = filtered * (1.0f - _alpha) + value * _alpha;
    return filtered;
}
