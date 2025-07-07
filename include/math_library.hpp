#ifndef MATH_LIBRARY_HPP
#define MATH_LIBRARY_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iostream>
#include <vector>

namespace Filter
{
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::: 低通滤波器 ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    class LowPassFilter
    {
    private:
        double alpha_;
        double last_output_;
        bool initialized_;

    public:
        LowPassFilter(double alpha);
        double filter(double input);
        void reset();
    };

    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::: 卡尔曼滤波器 ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    class KalmanFilter
    {
    private:
        double q_;
        double r_;
        double x_;
        double p_;

    public:
        KalmanFilter(double q, double r, double initial_value);
        double filter(double measurement);
        void reset(double initial_value);
    };

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 互补滤波器 :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    double complementaryFilter(double alpha, double fast, double slow);

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 移动平均滤波器 :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    class MovingAverageFilter
    {
    private:
        size_t window_size_;
        std::deque<double> window_;

    public:
        MovingAverageFilter(size_t window_size);
        double filter(double input);
        void reset();
    };

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 中值滤波器 :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    class MedianFilter
    {
    private:
        size_t window_size_;
        std::deque<double> window_;

    public:
        MedianFilter(size_t window_size);
        double filter(double input);
        void reset();
    };
} // namespace Filter

#endif // MATH_LIBRARY_HPP