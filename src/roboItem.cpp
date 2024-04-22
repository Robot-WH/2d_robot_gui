#include "roboItem.h"
#include "ui_MainWindow.h"    // build/my_gui下
#include <QDebug>
#include <qelapsedtimer.h>
#include <iostream>
#include <iomanip>
namespace ros_qt {
////////////////////////////////////////////////////////////////////////////////////////////////////////
roboItem::roboItem() {
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setAcceptDrops(true);
  setFlag(ItemAcceptsInputMethod, true);
//  moveBy(0, 0);
  m_moveCursor = new QCursor(QPixmap("://images/cursor_move"), 0, 0);
  set2DPoseCursor = new QCursor(QPixmap("://images/cursor_pos.png"), 0, 0);
  set2DGoalCursor = new QCursor(QPixmap("://images/cursor_pos.png"), 0, 0);
  setRobotVis(eRobotColor::red);
  setDefaultScale();
  laser_upside_down_ = false;   // 雷达颠倒
  visual_mode_ = VisualMode::internal_tracking;
  param.linear_v = 0.5;
  param.angular_v = 0.175;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::setRobotVis(eRobotColor color) {
  switch (color) {
    case eRobotColor::blue: {
      robotImg.load("://images/robot_blue.png");
    } break;
    case eRobotColor::red: {
      robotImg.load("://images/robot_red.png");
    } break;
    case eRobotColor::yellow: {
      robotImg.load("://images/robot_yellow.png");
    } break;
  }
  QMatrix matrix;
  matrix.rotate(90);
  robotImg = robotImg.transformed(matrix, Qt::SmoothTransformation);
  robotImg = robotImg.scaled(robotImg.width() * 2, robotImg.height() * 2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::setRobotSize(QSize size) { robotImg = robotImg.scaled(size); }

////////////////////////////////////////////////////////////////////////////////////////////////////////
int roboItem::QColorToInt(const QColor &color) {
  //将Color 从QColor 转换成 int
  return (int)(((unsigned int)color.blue() << 16) |
               (unsigned short)(((unsigned short)color.green() << 8) |
                                color.red()));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::paintImage(int id, QImage image) { m_image = image; }

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief roboItem::paintStableLaserScan  绘制静态激光点
/// \param points 相对于激光odom坐标系的点
///
void roboItem::paintStableLaserScan(QPolygonF points) {
  // 激光odom系转图元item系
  for (int i = 0; i < points.size(); ++i) {
    // 转到odom系
    poseLaserOdomToOdom(points[i]);
    // 转图元系
    points[i].setX(points[i].x() / map_resolution_);
    points[i].setY(-points[i].y() / map_resolution_);
  }

  stableLaserPoints = points;
  update();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief roboItem::paintStableLaserScan
/// \param points
///
void roboItem::paintDynamicLaserScan(QPolygonF points) {
  // 激光odom系转图元item系
  for (int i = 0; i < points.size(); ++i) {
    // 转到odom系
    poseLaserOdomToOdom(points[i]);
    // 转图元系
    points[i].setX(points[i].x() / map_resolution_);
    points[i].setY(-points[i].y() / map_resolution_);
  }
  dynamicLaserPoints = points;
//  qDebug() << "dynamicLaserPoint size: " << dynamicLaserPoints.size();
  update();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief roboItem::paintPlannerPath
///               palnner规划path绘制
/// \param path
///
void roboItem::paintPlannerPath(QPolygonF path) {
  plannerPath = path;
  update();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::paintWheelOdomPath(QPolygonF path) {
  // std::cout << "paintWheelOdomPath" << std::endl;
  // 转图元item系
  for (int i = 0; i < path.size(); ++i) {
    // 转图元系
    path[i].setX(path[i].x() / map_resolution_);
    path[i].setY(-path[i].y() / map_resolution_);
  }
  wheelOdomPath = path;
  update();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::paintMaps(QImage map) {
  m_imageMap = map;
  update();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief roboItem::paintSubGridMap 绘制子栅格地图
/// \param map
/// \param mapOrigin map左下角相对于激光ODOM坐标系的位置
/// \param res 分辨率
/// \param width 像素宽size
/// \param height 像素高size
///
void roboItem::paintSubGridMap(QImage map, QPointF mapOrigin, float res, int width, int height) {
//  qDebug() << "paintSubGridMap";
//  qDebug() << "width: " << width << ",height: " << height << ", res: " << res;
//  qDebug() << "mapOrigin x: " << mapOrigin.x() << ",y: " << mapOrigin.y();
  if (map_resolution_ != res * expansion_coef_) {
    map_resolution_ = res * expansion_coef_;    // 可视化的分辨率 和 实际物理分辨率 差一个 expansion_coef_的倍数
  }
  // 没有颠倒的情况下原mapOrigin是图片左下角的坐标，而Qt中显示图片的原点坐标系在图片左上角，因此要进行转换
  // 而颠倒时候，由与y反向，所以原来的mapOrigin实际就是Qt显示的坐标原点了
  if (!laser_upside_down_) {
    mapOrigin.setY(mapOrigin.y() + height * res);
  }
  // map 是激光里程计坐标系下的，要把它转换到odom系
  transformMapFromLaserOdomToOdom(map);
  poseLaserOdomToOdom(mapOrigin);
  // 注意，下面这个map_resolution_分辨率不是原始slam算法里的分辨率，
  // 而是qt显示时的可视化分辨率，在上面进行了转换 
  SubGridMapOrigin.setX(mapOrigin.x() / map_resolution_);
  // qt中视图坐标系y的方向和机器人坐标系相反 所以要-,
  SubGridMapOrigin.setY(-mapOrigin.y() / map_resolution_);
  // if (flag) return;  
  QElapsedTimer mstimer;
  mstimer.start();
  // map 是物理尺度分辨率下的图片，显示时要转换到显示尺度分辨率下
  // 这个函数耗时巨大，当expansion_coef_ = 0.1 时，耗时接近200ms,将造成卡顿
  m_imageMap = map.scaled(width / expansion_coef_, height / expansion_coef_);
  float time =(double)mstimer.nsecsElapsed()/(double)1000000;
  qDebug() <<"map.scaled time= " <<time<<"ms";// 输出运行时间（ms）
  update();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief roboItem::poseLaserOdomToOdom 激光odom坐标系转odom坐标系
///               底层算法发送过来的地图原点坐标是相对于激光odom坐标系的，而实际上
///               我们上层应用的是odom坐标系
/// \param mapOrigin_in_laserOdom
///
void roboItem::poseLaserOdomToOdom(QPointF& pose_in_laserOdom) {
    /**
     * @todo: 这里只考虑了颠倒，需要再加入外参
     **/
    if (laser_upside_down_) {
        pose_in_laserOdom.setY(-pose_in_laserOdom.y());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::transformMapFromLaserOdomToOdom(QImage& map) {
  int width = map.width();
  int layer_num = map.height() / 2;
  /**
   * @todo: 这里只考虑了颠倒，需要再加入外参
   **/
  if (laser_upside_down_) {
    for (int row = 0; row < layer_num; ++row) {
      for (int col = 0; col < width; ++col) {
        QRgb pixel = map.pixel(col, row);
        map.setPixel(col, row, map.pixel(col, map.height() - row - 1));
        map.setPixel(col, map.height() - row - 1, pixel);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::paintRoboPos(RobotPose pos) {
 // qDebug()<<"pos:"<<pos.x<<" "<<pos.y<<" "<<pos.theta;
  RoboPostion = QPointF(pos.x / map_resolution_, -pos.y / map_resolution_);   // y坐标颠倒
  m_roboYaw = pos.theta;
  update();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                                          QWidget *widget) {
//    QElapsedTimer mstimer;
//    mstimer.start();
    std::cout << "paint" << "\n";
    if (show_gridmap_flag) {
      drawGridMap(painter);
    }
    drawRoboPos(painter);
  //  drawPlannerPath(painter);
    drawWheelOdomPath(painter);
    drawLaserScan(painter);
  //  drawTools(painter);
//      float time =(double)mstimer.nsecsElapsed()/(double)1000000;
//      qDebug() <<"paint time= " <<time<<"ms";// 输出运行时间（ms）
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::drawTools(QPainter *painter) {
  if (m_currCursor == set2DPoseCursor || m_currCursor == set2DGoalCursor) {
      //绘制箭头
    if (m_startPose.x() != 0 && m_startPose.y() != 0 &&
            m_endPose.x() != 0 && m_endPose.y() != 0) {
        QPen pen(QColor(50, 205, 50, 255), 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        QBrush brush(QColor(50, 205, 50, 255), Qt::SolidPattern);

        painter->setPen(pen);
        painter->setBrush(brush);
        //计算线弧度
        double theta = atan((m_endPose.y() - m_startPose.y()) /
                             (m_endPose.x() - m_startPose.x()));
        //绘制直线
        QPointF startPoint, endPoint;
        startPoint = m_startPose;
        endPoint =m_endPose;
        QLineF line(startPoint, endPoint);
        painter->drawLine(line);
        float angle = atan2(endPoint.y()-startPoint.y(), endPoint.x()-startPoint.x()) + 3.1415926;//
        //绘制三角形
        QPolygonF points;
        points.push_back(endPoint);
        QPointF point1,point2;
        point1.setX(endPoint.x() + 10 * cos(angle - 0.5));//求得箭头点1坐标
        point1.setY(endPoint.y() + 10 * sin(angle - 0.5));
        point2.setX(endPoint.x() + 10 * cos(angle + 0.5));//求得箭头点2坐标
        point2.setY(endPoint.y() + 10 * sin(angle + 0.5));
        points.push_back(point1);
        points.push_back(point2);
        painter->drawPolygon(points);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::drawGridMap(QPainter* painter) {
  // 这个sb drawImage()函数绘制图片时的原点坐标只能是整型，
  // 例如 painter->drawImage(17.6, 15.4, m_imageMap), 那么实际上绘图的原点坐标是(17, 15)
  // 这也是为什么可视化分辨率相比物理分辨率要进行膨胀的原因
//  QElapsedTimer mstimer;
//  mstimer.start();
// drawImage绘图时的原点坐标只能是整数，比如地图的原点应该是(18.6, 17.4)那么实际绘图后
// 的原点是(18，17),因此最终显示的图片会有一个偏差 
  painter->drawImage(SubGridMapOrigin.x(), SubGridMapOrigin.y(), m_imageMap);
//  float time =(double)mstimer.nsecsElapsed()/(double)1000000;
//  qDebug() <<"drawGridMap time= " <<time<<"ms";// 输出运行时间（ms）
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::drawRoboPos(QPainter *painter) {
//  qDebug() << "drawRoboPos";
//  qDebug() << "RoboPostion.x()： "<<RoboPostion.x() << "RoboPostion.y(): " << RoboPostion.y();

  painter->setPen(QPen(QColor(255, 0, 0, 255), 1, Qt::SolidLine, Qt::RoundCap,
                       Qt::RoundJoin));
  painter->save();
  painter->translate(RoboPostion.x(), RoboPostion.y());
  painter->rotate(rad2deg(-m_roboYaw));
  painter->drawPoint(QPoint(0, 0));
  painter->drawPixmap(QPoint(-robotImg.width() / 2, -robotImg.height() / 2),
                      robotImg);

  painter->restore();
  if (visual_mode_ == VisualMode::internal_tracking) {
       emit roboPos(RoboPostion);
  } else if (visual_mode_ == VisualMode::translate_tracking) {
    this->moveBy(m_scaleValue * (last_RoboPostion.x() - RoboPostion.x()),
                                  m_scaleValue * (last_RoboPostion.y() - RoboPostion.y()));
    last_RoboPostion = RoboPostion;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::drawLaserScan(QPainter* painter) {
  //绘制laser
  painter->setPen(QPen(QColor(0, 255, 0, 255), 1 / expansion_coef_));
  painter->drawPoints(stableLaserPoints);

  painter->setPen(QPen(QColor(255, 0, 0, 255), 1 / expansion_coef_));
  painter->drawPoints(dynamicLaserPoints);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::drawPlannerPath(QPainter *painter) {
  //绘制planner Path
  painter->setPen(QPen(QColor(0, 0, 0, 255), 1));
  painter->drawPoints(plannerPath);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::drawWheelOdomPath(QPainter *painter) {
  //绘制planner Path
  painter->setPen(QPen(QColor(0, 255, 255, 255), 1 / expansion_coef_));
  painter->drawPoints(wheelOdomPath);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
const QPointF& roboItem::GetRoboPos() const {
  return RoboPostion;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
float roboItem::GetScale() const {
  return m_scaleValue;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::SetVisualMode(VisualMode mode) {
    visual_mode_ = mode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::SetLaserInverted(bool flag) {
  laser_upside_down_ = flag;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::setMax() {
  m_scaleValue *= 1.1;  //每次放大10%
  setScale(m_scaleValue);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::setMin() {
  m_scaleValue *= 0.9;  //每次缩小10%
  setScale(m_scaleValue);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::setDefaultScale() {
  m_scaleValue = 0.5;
  this->setScale(m_scaleValue);
//  this->moveBy(0, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::SetGridMapShow(bool flag) {
  show_gridmap_flag = flag;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
QRectF roboItem::boundingRect() const {
  //设置当前item绘制区域 (x,y,width,height)
  return QRectF(-m_imageMap.width() * 5, -m_imageMap.height() * 5,
                                m_imageMap.width() * 10, m_imageMap.height() * 10);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::move(double x, double y) { this->moveBy(x, y); }


////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief roboItem::wheelEvent
/// \param event
///
void roboItem::wheelEvent(QGraphicsSceneWheelEvent *event) {
  this->setCursor(Qt::CrossCursor);
  m_currCursor = nullptr;
  if (event->delta() > 0) {
    ChangeScale(1, event->pos());
  } else {
    ChangeScale(0, event->pos());
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief roboItem::changeScale 放大缩小
/// \param type 0 缩小， 1 放大
/// \param center 缩放的参考中心
///
void roboItem::ChangeScale(bool type, const QPointF& center) {
  if (type && (m_scaleValue >= 100)) {
    //最大放大到原始图像的50倍
    return;
  }
  else if (type &&(m_scaleValue <= m_scaleMin)) {
    //图像缩小到最小之后就不继续缩小
    // ResetItemPos();//重置图片大小和位置，使之自适应控件窗口大小
  } else {
    qreal qrealOriginScale = m_scaleValue;
    if (type) {
      //鼠标滚轮向前滚动
      m_scaleValue *= 1.1;  //每次放大10%
    } else {
      m_scaleValue *= 0.9;  //每次缩小10%
    }
    setScale(m_scaleValue);
    if (type) {
      // 放大
      moveBy(-center.x() * qrealOriginScale * 0.1,
             -center.y() * qrealOriginScale * 0.1);  //使图片缩放的效果看起来像是以center为中心进行缩放的
    } else {
      moveBy(center.x() * qrealOriginScale * 0.1,
             center.y() * qrealOriginScale * 0.1);  //使图片缩放的效果看起来像是以center为中心进行缩放的
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::slot_set2DPos() {
  this->setCursor(*set2DPoseCursor);  //设置自定义的鼠标样式
  m_currCursor = set2DPoseCursor;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::slot_set2DGoal() {
  this->setCursor(*set2DGoalCursor);  //设置自定义的鼠标样式
  m_currCursor = set2DGoalCursor;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::slot_setMoveCamera() {
  this->setCursor(*m_moveCursor);  //设置自定义的鼠标样式
  m_currCursor = m_moveCursor;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief roboItem::mousePressEvent
///             事件处理函数，用于处理鼠标按下事件。
///             当鼠标在图元上按下时，系统将触发 mousePressEvent() 函数
/// \param event
///
void roboItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
//    qDebug() << "mousePressEvent";
  if (event->button() == Qt::LeftButton) {
    m_startPose = event->pos();  //鼠标左击时，获取当前鼠标在图片中的坐标，
    // qDebug() << "Press pos x: " << m_startPose.x() << ",y: " << m_startPose.y();
    m_isMousePress = true;  //标记鼠标左键被按下
  } else if (event->button() == Qt::RightButton) {
    // ResetItemPos();//右击鼠标重置大小
  }
//  update();
 }

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
//  qDebug() << "mouseMoveEvent";
  //设置鼠标样式为移动
  if (m_currCursor != m_moveCursor) {
    this->setCursor(*m_moveCursor);  //设置自定义的鼠标样式
    m_currCursor = m_moveCursor;
  }
  //移动图层
  if (m_isMousePress && m_currCursor == m_moveCursor) {
    QPointF point = (event->pos() - m_startPose) * m_scaleValue;
    moveBy(point.x(), point.y());
  }
  m_endPose = event->pos();
}

//void roboItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
//{
//  qDebug() << "hoverMoveEvent";
//  emit cursorPos(event->pos());
//}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
//  qDebug() << "mouseReleaseEvent";
  m_isMousePress = false;  //标记鼠标左键已经抬起
  //如果是选择点位模式 重置
  if (m_currCursor == set2DPoseCursor) {
      emit signalPub2DPose(m_startPose,m_endPose);
      m_currCursor=m_moveCursor;
      this->setCursor(*m_currCursor);
  } else if (m_currCursor == set2DGoalCursor) {
      emit signalPub2DGoal(m_startPose,m_endPose);
      m_currCursor=m_moveCursor;
      this->setCursor(*m_currCursor);
  }
  m_startPose=QPointF();
  m_endPose=QPointF();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void roboItem::clearWheelPath() {
  wheelOdomPath.clear();
}

}  // namespace ros_qt5_gui_app
