#ifndef roboItem_H
#define roboItem_H

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
  void setRobotSize(QSize size);
  void ChangeScale(bool type, const QPointF& center);
  QPolygon MapPoints;
  QPolygonF plannerPath;
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
 signals:
  void cursorPos(QPointF);
  void roboPos(QPointF);
  void signalPub2DPose(QPointF,QPointF);
  void signalPub2DGoal(QPointF,QPointF);
 public slots:
  void paintMaps(QImage map);
  void paintSubGridMap(QImage map, QPointF mapOrigin, float res, int width, int height);
  void paintRoboPos(RobotPose pos);
  void paintImage(int, QImage);
  void paintPlannerPath(QPolygonF);
  void paintStableLaserScan(QPolygonF);
  void paintDynamicLaserScan(QPolygonF points);
  void slot_set2DPos();
  void slot_set2DGoal();
  void slot_setMoveCamera();

 private:
  void drawMap(QPainter *painter);
  void drawRoboPos(QPainter *painter);
  void drawLaserScan(QPainter *painter);
  void drawPlannerPath(QPainter *painter);
  void drawTools(QPainter *painter);
  void poseLaserOdomToOdom(QPointF& mapOrigin_in_laserOdom);
 private:
  VisualMode visual_mode_;
  int m_zoomState;
  bool m_isMousePress{false};
  bool laser_upside_down_;   // 雷达颠倒
  QPixmap robotImg;
  QPointF m_startPose;
  QPointF m_endPose;
  qreal m_scaleValue = 0.2;
  qreal m_scaleMin = 0.01;
  float map_resolution_ = 0.05;    // 地图局部分辨率
};
}  // namespace ros_qt5_gui_app
#endif  // roboItem_H
