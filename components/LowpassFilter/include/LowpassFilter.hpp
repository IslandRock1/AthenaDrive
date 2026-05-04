
#pragma once

class LowpassFilter {
public:
    LowpassFilter(float alpha);
    float update(float value);
    float getValue();

private:
    float _filtered = 0;
    float _alpha;
};