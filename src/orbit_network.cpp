#include <iostream>
#include "orbit_network.hpp"
namespace schedule {
////////////////////////////////////////////////////////////////////////////////////////////////////////
void OrbitNetwork::AddOrbitNetNode(float begin_x, float begin_y, float begin_yaw, 
        float end_x, float end_y, float end_yaw, uint8_t type, const std::vector<float>& param) {
    // 根据两点间的距离确定路径点数
    float dis_2 = (begin_x - end_x) * (begin_x - end_x) + (begin_y - end_y) * (begin_y - end_y); 
    int t = 0;
    // 距离 < 10m 认为路径长度 < 30m
    if (dis_2 <100) {
        t = 200;      //  平均路径点间距 < 0.15cm
    } else if (dis_2 < 900) {    // 距离 < 30, 认为路径长度 < 100m   
        t = 500;     //  平均路径点间距 < 0.2cm
    } else {
        t = 1000;    // 最多分1000个点  
    }
    std::cout << "t: " << t << "\n";  
    QPolygonF path;
    path.reserve(t);
    // 直线
    if (type == 1) {
        generateLinePath(begin_x, begin_y, end_x, end_y, t, path);
    } else if (type == 2) {   // 贝塞尔曲线
    } else if (type == 0) {   // 自由路径
    }
    orbit_network_.emplace_back(begin_x, begin_y, begin_yaw, std::move(path));  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void OrbitNetwork::generateLinePath(float begin_x, float begin_y, float end_x, 
                                                                                float end_y, int t, QPolygonF& path) {
    float step = (float)1 / t;  
    float i = 0;    
    // std::cout << "begin_x: " << begin_x << ",begin_y: " << begin_y
    //     << ",end_x: " << end_x << ",end_y: " << end_y << ", step: " << step << "\n";  
    while (i <= 1) {
        path.append(QPointF((1 - i) * begin_x + i * end_x, (1 - i) * begin_y + i * end_y));
        i += step;  
    }
    return; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void OrbitNetwork::generateBezierCurvePath() {

}

////////////////////////////////////////////////////////////////////////////////////////////////////////
const std::vector<OrbitNode>& OrbitNetwork::GetAllNode() {
    return orbit_network_;  
}

}


