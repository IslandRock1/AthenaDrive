
#pragma once

class LowpassFilter {
public:
    LowpassFilter(float alpha);
    float update(float value);

private:
    float filtered = 0;
    float _alpha;
};