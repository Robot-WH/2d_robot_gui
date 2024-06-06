#ifndef roboItem_H
#define roboItem_H
#include <mutex>
#include <QColor>
#include <QCursor>
#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QObject>
#include <QPainter>
#include <QPolygon>
#include <QTimer>
#include <QtMath>
#include <opencv2/highgui/highgui.hpp>
#include "RobotAlgorithm.h"
namespace ros_qt {
enum eRobotColor { blue, red, yellow };

class roboItem : public QObject, public QGraphicsItem {
    Q_OBJECT
public:
    enum class VisualMode {translate_tracking, internal_tracking, driver};
    roboItem();
    QRectF boundingRect() const;
    void wheelEvent(QGraphicsSceneWheelEvent *event);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    //  void hoverMoveEvent(QGraphicsSceneHoverEvent *event);
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget);
    int QColorToInt(const QColor &color);
    const QPointF& GetRoboPos() const;
    float GetScale() const;
    void SetVisualMode(VisualMode mode);
    void setRobotVis(eRobotColor color);
    void SetLaserInverted(bool flag);
    void setRobotSize(QSize size);
    void SetGridMapShow(bool flag);
    void SetGoal();
    void ChangeScale(bool type, const QPointF& center);
    void clearWheelPath();
    QPolygon MapPoints;
    QPolygonF DWALocalPath;
    QPolygonF wheelOdomPath;
    QPolygonF GlobalPlanningPath;
    QPolygonF stableLaserPoints;
    QPolygonF dynamicLaserPoints;
    QPointF RoboPostion;
    QPointF last_RoboPostion;
    QPointF SubGridMapOrigin;
    QSizeF mapSize;
    QImage m_image;
    QImage m_imageMap;
    QTimer timer_update;
    int m_sizeCar = 4;
    double m_roboYaw;
    double m_roboR = 5;
    double map_size = 1;
    double PI = 3.1415926;
    void get_version() { qDebug() << "1.0.0"; }
    void setMax();
    void setMin();
    void setDefaultScale();
    void move(double x, double y);
    QCursor *m_moveCursor = nullptr;
    QCursor *set2DPoseCursor = nullptr;
    QCursor *set2DGoalCursor = nullptr;
    QCursor *m_currCursor = nullptr;
    struct Param {
        float angular_v;
        float linear_v;
    } param;
signals:
    void cursorPos(QPointF);
    void roboPos(QPointF);
    void signalPub2DPose(QPointF,QPointF);
    // void signalPub2DGoal(QPointF,QPointF);
    void signalPub2DGoal(double x, double y, double yaw);
public slots:
    void paintMaps(QImage map);
    void paintSubGridMap(QImage map, QPointF mapOrigin, float res, int width, int height);
    void paintRoboPos(RobotPose pos);
    void paintImage(int, QImage);
    void paintWheelOdomPath(QPolygonF path);
    void paintGlobalPlanningPath(QPolygonF path);
    void paintDWALocalPath(QPolygonF);
    void paintStableLaserScan(QPolygonF);
    void paintDynamicLaserScan(QPolygonF points);
    void slot_set2DPos();
    void slot_set2DGoal();
    void slot_setMoveCamera();

private:
    void drawGridMap(QPainter *painter);
    void drawRoboPos(QPainter *painter);
    void drawNavArrow(QPainter *painter);
    void drawLaserScan(QPainter *painter);
    void drawTools(QPainter *painter);
    void drawPlanningPath(QPainter *painter);
    void drawWheelOdomPath(QPainter *painter);
    void transformMapFromLaserOdomToOdom(QImage& map);
private:
    struct LaserWheelExt {
        float rot_scale = 1;   // 旋转尺度
        float trans_scale = 1;   // 线速度尺度
        float x = 0, y = 0, theta = 0;     // 外参
    } laser_wheel_ext_;
    VisualMode visual_mode_;
    int m_zoomState;
    bool m_isMousePress{false};
    bool laser_upside_down_;   // 雷达颠倒
    bool show_gridmap_flag = false;
    QPixmap robotImg;
    QPixmap arrowImg;
    QPointF m_startPose;
    QPointF m_endPose;
    qreal m_scaleValue = 0.2;
    qreal m_scaleMin = 0.01;
    float map_resolution_ = 0.05;    // 地图局部分辨率
    float expansion_coef_ = 0.2;     // 地图分辨率的放大系数
    std::mutex wheelPath_mt;
    bool set_goal_ = false;
};
}  // namespace ros_qt5_gui_app
#endif  // roboItem_H
