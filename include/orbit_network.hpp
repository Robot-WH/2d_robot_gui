#pragma once 
#include <vector>
#include <QPointF>
#include <QPolygonF>
namespace Schedule {
struct OrbitNode {
    OrbitNode() = default; // 使用= default来让编译器生成默认构造函数
    OrbitNode(float state_x, float state_y, float state_yaw, QPolygonF&& path)
        : state_x_(state_x), state_y_(state_y), state_yaw_(state_yaw), path_(std::move(path)) {}
    OrbitNode(OrbitNode&& node)
        : state_x_(node.state_x_), state_y_(node.state_y_), state_yaw_(node.state_yaw_), path_(std::move(node.path_)) {} // 使用初始化列表
    OrbitNode(const OrbitNode& node) = default; // 使用= default来让编译器生成默认复制构造函数
    float state_x_;
    float state_y_;
    float state_yaw_;
    QPolygonF path_;
};

class OrbitNetwork {
public:
    OrbitNetwork() = default;
    void AddOrbitNetNode(float begin_x, float begin_y, float begin_yaw, 
        float end_x, float end_y, float end_yaw, uint8_t type, const std::vector<float>& param);
    const std::vector<OrbitNode>& GetAllNode();  
protected:
    /**
     * @brief 生成直线路径  
     * 
     * @param begin_x 
     * @param begin_y 
     * @param end_x 
     * @param end_y 
     * @param t   除了 begin点外，后面路径点数
     * @param path 
     */
    void generateLinePath(float begin_x, float begin_y, float end_x, 
                                                    float end_y, int t, QPolygonF& path);
    void generateBezierCurvePath();
private:
    std::vector<OrbitNode> orbit_network_;
}; 
};
