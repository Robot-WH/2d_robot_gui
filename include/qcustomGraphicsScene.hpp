#pragma once 
#include <QGraphicsScene>  
#include <QGraphicsSceneMouseEvent>  
#include <QList>  
#include <QPointF>  
  
class QCustomQGraphicsScene : public QGraphicsScene {  
    Q_OBJECT  
public:  
    QCustomQGraphicsScene(QObject *parent = nullptr) : QGraphicsScene(parent) {}  
    void SetGoal() {
      set_goal_ = true;
    }
protected:  
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
        if (set_goal_) {

        } else {
          QGraphicsScene::mousePressEvent(event);
        }
    }

    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
      if (set_goal_) {
        // 如果处与设定导航目标的状态下    那么绘制指向当前触摸位置的箭头

      } else {
        QGraphicsScene::mouseMoveEvent(event);
      }
    }

    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
      if (set_goal_) {
        // 如果处与设定导航目标的状态下   那么发送导航目标
      } else {
        QGraphicsScene::mouseReleaseEvent(event);
      }
    }
  
private:  
    QList<QPointF> m_vetPoints; // 用于存储绘制点  
    bool set_goal_ = false;
};  
