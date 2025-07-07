#include "math_library.hpp"

namespace Filter
{
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::: 低通滤波器实现 :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    /**
     * 低通滤波器构造函数
     * @param alpha 平滑系数，范围[0.0, 1.0]
     *  - alpha接近0：强滤波，响应慢，适合高频噪声场景
     *  - alpha接近1：弱滤波，响应快，适合快速变化信号
     */
    LowPassFilter::LowPassFilter(double alpha) : alpha_(alpha), initialized_(false) {}

    /**
     * 执行低通滤波
     * @param input 当前输入值
     * @return 滤波后的输出值
     *
     * 算法公式：
     *   y[n] = α·x[n] + (1-α)·y[n-1]
     *
     * 频率响应特性：
     *   截止频率 fc ≈ fs·α/(2π)
     *   其中fs为采样频率
     */
    double LowPassFilter::filter(double input)
    {
        if (!initialized_)
        {
            // 首次调用时用输入值初始化状态，避免输出跳变
            last_output_ = input;
            initialized_ = true;
        }

        // 指数加权移动平均核心计算
        double output = alpha_ * input + (1.0 - alpha_) * last_output_;
        last_output_ = output;
        return output;
    }

    /**
     * 重置滤波器状态
     * 下次调用filter时将重新初始化
     */
    void LowPassFilter::reset()
    {
        initialized_ = false;
    }

    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::: 卡尔曼滤波器实现 :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    /**
     * 卡尔曼滤波器构造函数
     * @param q 过程噪声协方差，反映系统模型不确定性
     * @param r 测量噪声协方差，反映传感器测量不确定性
     * @param initial_value 初始状态估计值
     *
     * 参数调整原则：
     *  - q >> r：更信任测量值，对变化响应快但可能不稳定
     *  - q << r：更信任预测值，输出更平滑但响应慢
     */
    KalmanFilter::KalmanFilter(double q, double r, double initial_value) : q_(q), r_(r), x_(initial_value), p_(1.0) {}

    /**
     * 执行卡尔曼滤波迭代
     * @param measurement 当前测量值
     * @return 融合后的最优状态估计
     */
    double KalmanFilter::filter(double measurement)
    {
        // 预测阶段：仅更新误差协方差（简化模型假设状态转移为1）
        p_ = p_ + q_;

        // 更新阶段：计算增益并融合测量值
        double k = p_ / (p_ + r_);
        x_ = x_ + k * (measurement - x_);
        p_ = (1.0 - k) * p_;

        return x_;
    }

    /**
     * 重置滤波器状态
     * @param initial_value 新的初始状态估计值
     */
    void KalmanFilter::reset(double initial_value)
    {
        x_ = initial_value;
        p_ = 1.0; // 重置误差协方差
    }

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 互补滤波器实现 :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    /**
     * 互补滤波器函数
     * @param alpha 低频信号权重系数 [0.0, 1.0]
     * @param fast 高频信号（如陀螺仪数据）
     * @param slow 低频信号（如加速度计数据）
     * @return 融合后的信号
     *
     * 原理：
     *   利用互补特性融合两种信号：
     *   output = α·slow + (1-α)·fast
     *
     *   典型应用：
     *   融合陀螺仪角速度积分（短期精度高但长期漂移）
     *   和加速度计角度（长期稳定但动态响应差）
     */
    double complementaryFilter(double alpha, double fast, double slow)
    {
        return alpha * slow + (1.0 - alpha) * fast;
    }

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 移动平均滤波器实现 :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    /**
     * 移动平均滤波器构造函数
     * @param window_size 滑动窗口大小
     *
     * 窗口大小选择建议：
     *  - 较小窗口（3-5）：响应快，对变化敏感
     *  - 较大窗口（8-10）：平滑效果好，抑制噪声能力强
     */
    MovingAverageFilter::MovingAverageFilter(size_t window_size)
        : window_size_(window_size) {}

    /**
     * 执行移动平均滤波
     * @param input 当前输入值
     * @return 窗口内数据的平均值
     *
     * 算法复杂度：
     *   O(n)，每次需要遍历窗口内所有元素求和
     *
     * 频率响应特性：
     *   低通滤波特性，截止频率约为 fs/(2·window_size)
     */
    double MovingAverageFilter::filter(double input)
    {
        // 更新滑动窗口
        window_.push_back(input);
        if (window_.size() > window_size_)
            window_.pop_front();

        // 计算窗口内数据的平均值
        double sum = 0.0;
        for (double val : window_)
            sum += val;
        return sum / window_.size();
    }

    /**
     * 重置滤波器状态
     * 清空滑动窗口数据
     */
    void MovingAverageFilter::reset()
    {
        window_.clear();
    }

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 中值滤波器实现 :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    /**
     * 中值滤波器构造函数
     * @param window_size 滑动窗口大小，建议使用奇数
     *
     * 窗口大小选择：
     *  - 奇数窗口（3、5、7）：避免偶数时需要取中间两个数的平均值
     *  - 典型值：3（轻度滤波）、5（中度滤波）、7（强滤波）
     */
    MedianFilter::MedianFilter(size_t window_size)
        : window_size_(window_size) {}

    /**
     * 执行中值滤波
     * @param input 当前输入值
     * @return 窗口内数据的中值
     *
     * 算法特点：
     *  - 有效抑制脉冲噪声（如传感器尖峰干扰）
     *  - 保留信号边缘特性，不会平滑过渡
     *
     * 算法复杂度：
     *  O(n·logn)，主要来自排序操作
     */
    double MedianFilter::filter(double input)
    {
        // 更新滑动窗口
        window_.push_back(input);
        if (window_.size() > window_size_)
            window_.pop_front();

        // 复制窗口数据并排序
        std::vector<double> sorted(window_.begin(), window_.end());
        std::sort(sorted.begin(), sorted.end());

        // 计算中值
        size_t mid = sorted.size() / 2;
        if (sorted.size() % 2 == 0)
            return (sorted[mid - 1] + sorted[mid]) / 2.0; // 偶数个元素取中间两数平均值
        else
            return sorted[mid]; // 奇数个元素直接取中间值
    }

    /**
     * 重置滤波器状态
     * 清空滑动窗口数据
     */
    void MedianFilter::reset()
    {
        window_.clear();
    }
} // namespace Filter