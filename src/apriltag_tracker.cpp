#include "apriltag_tracker.hpp"
#include "sim_camera_module.hpp"

// Gazebo仿真环境中相机图像主题
std::string subscribePtr = "/gazebo/default/iris/base_link/camera/image";

// 获取最新帧函数（仿真模式专用）
cv::Mat get_latest_frame()
{
    return GazeboCamera::Instance()->GetNextFrame(); // 从Gazebo仿真相机获取最新图像帧
}

// 启动跟踪器实现
void AprilTagTracker::GazeboStart(int argc, char *argv[])
{
    // 初始化并启动Gazebo相机
    GazeboCamera::Instance()->init(argc, argv, subscribePtr);
    GazeboCamera::Instance()->start();
}

namespace
{
    /**
     * @brief 计算四边形面积（使用鞋带公式）
     * @param points 四边形四个顶点坐标（按顺时针或逆时针顺序排列）
     * @return 计算得到的四边形面积（绝对值的一半）
     */
    float calculateQuadrilateralArea(const cv::Point2f *points)
    {
        // 提取四个顶点坐标（假设按顺时针顺序排列）
        const float &x0 = points[0].x, &y0 = points[0].y;
        const float &x1 = points[1].x, &y1 = points[1].y;
        const float &x2 = points[2].x, &y2 = points[2].y;
        const float &x3 = points[3].x, &y3 = points[3].y;

        // 应用鞋带公式计算交叉乘积和
        float sum = (x0 * y1 - x1 * y0) +
                    (x1 * y2 - x2 * y1) +
                    (x2 * y3 - x3 * y2) +
                    (x3 * y0 - x0 * y3);

        // 返回绝对值的一半（确保面积为正值）
        return std::abs(sum) * 0.5f;
    }

    /**
     * @brief 验证四边形几何特征（用于过滤无效AprilTag检测）
     * @param points 四边形顶点坐标数组
     * @param image_width 图像宽度（用于坐标范围检查，0表示不检查）
     * @param image_height 图像高度（用于坐标范围检查，0表示不检查）
     * @param max_edge_ratio 允许的最大边长比例（默认2.0）
     * @return true表示几何特征合法，false表示需要过滤
     */
    bool check_quad_geometry(const cv::Point2f *points,
                             int image_width,
                             int image_height,
                             float max_edge_ratio)
    {
        // 1. 空指针防御性编程
        if (points == nullptr)
            return false;

        // 2. 坐标范围有效性检查
        if (image_width > 0 && image_height > 0)
        {
            for (int i = 0; i < 4; ++i)
            {
                // 顶点坐标超出图像边界时视为无效
                if (points[i].x <= 0 || points[i].x >= image_width ||
                    points[i].y <= 0 || points[i].y >= image_height)
                {
                    return false;
                }
            }
        }

        // 3. 计算四条边的长度
        float edges[4];
        for (int i = 0; i < 4; ++i)
        {
            int j = (i + 1) % 4; // 下一个顶点索引（闭合四边形）
            float dx = points[j].x - points[i].x;
            float dy = points[j].y - points[i].y;
            edges[i] = std::hypot(dx, dy); // 使用hypot安全计算欧氏距离
        }

        // 4. 边长比例合法性检查
        float max_edge = *std::max_element(edges, edges + 4);
        float min_edge = *std::min_element(edges, edges + 4);

        // 处理除零错误（极小边长视为退化四边形）
        if (min_edge < 1e-6f)
            return false;

        // 最大边长不超过最小边长的max_edge_ratio倍
        return (max_edge / min_edge <= max_edge_ratio);
    }

    // 多目标检测状态标记（仅在当前编译单元内可见）
    bool detect_twotag = false; // true表示当前帧检测到两个AprilTag目标
}

AprilTagTracker::AprilTagTracker()
{
    tf = tag25h9_create();
    td = apriltag_detector_create();

    if (!td || !tf)
    {
        throw std::runtime_error("未能创建 AprilTag 检测器");
    }
    apriltag_detector_add_family(td, tf); // 将标签家族添加到检测器

    td->quad_decimate = 1.0;
    td->nthreads = 4;
    td->refine_edges = true;
    td->decode_sharpening = 0.75; // 锐化解码区域
    td->quad_sigma = 0.2;         // 增加高斯模糊
}

AprilTagTracker::~AprilTagTracker()
{
    // 释放AprilTag资源（先释放标签家族，再释放检测器）
    if (tf)
    {
        tag25h9_destroy(tf);
    }
    if (td)
    {
        apriltag_detector_destroy(td);
    }
}

/**
 * @brief 主检测函数：从图像中检测AprilTag标签并计算其位置
 * @param frame 输入图像帧（BGR或灰度图）
 * @param drawOverlay 是否绘制检测结果叠加层
 * @return 包含检测结果的AprilTagData结构体
 */
AprilTagData AprilTagTracker::detect(cv::Mat &frame, bool drawOverlay)
{
    // 初始化返回结果（默认未检测到标签）
    AprilTagData result = {false, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0};

    try
    {
        // 输入帧有效性检查
        if (frame.empty())
        {
            std::cerr << "输入帧为空" << std::endl;
            return result;
        }

        // 记录图像尺寸到结果结构体
        result.width = frame.cols;
        result.height = frame.rows;

        // ------------------- 图像预处理阶段 -------------------
        cv::Mat gray, binary;
        if (frame.channels() == 3)
        {
            // BGR彩色图转灰度图（AprilTag检测仅需灰度信息）
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        }
        else
        {
            // 已为灰度图时直接复用
            gray = frame;
        }
        binary = gray; // 直接使用灰度图进行二值化处理

        // 分配AprilTag库所需的图像缓冲区
        image_u8_t *im = image_u8_create(binary.cols, binary.rows);
        if (!im)
        {
            std::cerr << "无法分配图像_u8 缓冲区" << std::endl;
            throw std::bad_alloc(); // 内存分配失败异常
        }

        // 将OpenCV图像数据复制到AprilTag缓冲区
        for (int y = 0; y < binary.rows; ++y)
        {
            memcpy(im->buf + y * im->stride,
                   binary.data + y * binary.step,
                   binary.cols);
        }

        // ------------------- AprilTag检测阶段 -------------------
        // 执行标签检测算法
        zarray_t *detections = apriltag_detector_detect(td, im);

        // 检测结果数量分类处理
        if (zarray_size(detections) > 1)
        {
            // 检测到多个标签时重置计时
            _start = clock();
            detect_twotag = true; // 标记多标签检测状态
        }
        else if (zarray_size(detections) > 0 && !detect_twotag)
        {
            // 检测到单个标签且非多标签模式时重置计时
            _start = clock();
        }
        else
        {
            // 未检测到标签时的超时处理
            if ((double)(clock() - _start) / CLOCKS_PER_SEC > 3)
            {
                // 超过3秒未检测到标签，重置状态
                detect_twotag = false;
                result.iffind = false;
            }
            else
            {
                // 短时间未检测到时返回上一次结果
                result = _last_results;
            }

            // 释放检测资源
            apriltag_detections_destroy(detections);
            image_u8_destroy(im);
            return result;
        }

        // 清空历史面积记录
        _areas.clear();

        // ------------------- 检测结果遍历处理 -------------------
        for (int i = 0; i < zarray_size(detections); ++i)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det); // 提取单个检测结果

            // 提取标签四边形四个顶点坐标（按顺时针顺序）
            const cv::Point2f points[4] =
                {
                    {det->p[3][0], det->p[3][1]}, // 左上顶点
                    {det->p[0][0], det->p[0][1]}, // 右上顶点
                    {det->p[1][0], det->p[1][1]}, // 右下顶点
                    {det->p[2][0], det->p[2][1]}  // 左下顶点
                };

            // 几何验证：检查四边形形状合法性和面积阈值
            if (!check_quad_geometry(points, binary.cols, binary.rows) || calculateQuadrilateralArea(points) < 100)
            {
                continue; // 跳过不合法的检测结果
            }

            // 提取标签中心坐标
            result.x = det->c[0]; // 列坐标 -> x
            result.y = det->c[1]; // 行坐标 -> y
            result.iffind = true; // 标记找到标签

            // 计算与图像中心的偏差
            result.err_x = (result.height / 2.0) - result.y; // X方向偏差
            result.err_y = (result.width / 2.0) - result.x;  // Y方向偏差

            // 相对于图像全宽 / 全高归一化（结果范围 [-0.5, 0.5]）：
            result.norm_err_x = result.err_x / result.width;
            result.norm_err_y = result.err_y / result.height;

            // 计算标签面积并记录
            _areas.push_back(calculateQuadrilateralArea(points));

            // ------------------- 可视化绘制阶段 -------------------
            apriltag_detection_t *best_det = nullptr; // 最佳检测结果指针
            cv::Mat display;                          // 可视化输出图像
            if (drawOverlay)
            {
                display = frame.clone(); // 需要绘制时复制原图

                // 绘制标签四边形边界
                for (int i = 0; i < 4; i++)
                {
                    cv::Point pt1(best_det->p[i][0], best_det->p[i][1]);
                    cv::Point pt2(best_det->p[(i + 1) % 4][0], best_det->p[(i + 1) % 4][1]);
                    cv::line(display, pt1, pt2, cv::Scalar(0, 255, 0), 2); // 绿色边界线
                }

                // 绘制标签中心点（红色圆点）
                cv::circle(display, cv::Point(result.x, result.y), 5, cv::Scalar(0, 0, 255), -1);
            }
        }

        // ------------------- 多目标逻辑处理 -------------------
        if (_areas.size() > 1)
        {
            // 面积差异较大时交换中心点顺序（可能用于目标优先级排序）
            if (_areas.at(0) > _areas.at(1) && fabs(_areas.at(0) - _areas.at(1)) > 100)
            {
                std::swap(result.centers[0], result.centers[1]);
            }
        }
        else if (_areas.size() > 0)
        {
            // 单目标时重置多目标标记
            detect_twotag = false;
        }

        // 保存检测结果
        _last_results = result;

        // 释放检测资源（必须在函数结束前执行）
        apriltag_detections_destroy(detections);
        image_u8_destroy(im);
    }
    // ------------------- 异常处理阶段 -------------------
    catch (const cv::Exception &e)
    {
        // OpenCV特定异常捕获（如格式错误、内存访问错误）
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        result.centers.clear();
        result.width = result.height = 0;
    }
    catch (...)
    {
        // 通用异常捕获（处理其他未知异常）
        std::cerr << "未知异常发生" << std::endl;
        result.centers.clear();
        result.width = result.height = 0;
    }

    return result;
}