#ifndef MainWindow_H
#define MainWindow_H

#include <QWidget>
#include <QProcess>
#include <QKeyEvent>
#include "qnode.hpp"
#include "roboItem.h"
#include "client/socket_comm.hpp"

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
  explicit MainWindow(int argc, char **argv, QWidget *parent = nullptr);
  ~MainWindow();
signals:
   void Quit();

protected:
  /**
   * @brief getLocalIpAddress 获取本机的IP地址
   * @return
   */
  QString getLocalIpAddress();

  void keyPressEvent(QKeyEvent* event) override;

  /**
   * @brief MainWindow::connection 信号与槽的连接
   */
  void connection();

  /**
   * @brief MainWindow::initUI 初始化UI
   */
  void initUI();

  /**
   * @brief MainWindow::initParm 初始化各种参数
   */
  void initParm();

  /**
   * @brief MainWindow::bringUpQtRosNode
   */
  void bringUpQtRosNode();

  void viewCenterFocusOnRobot();

private slots:
  void on_toolButton_7_clicked();

private slots:
  void on_toolButton_4_clicked();

private slots:
  void on_toolButton_5_clicked();

private slots:
  void on_toolButton_3_clicked();

private slots:
  void on_checkBox_7_stateChanged(int arg1);

private slots:
  void on_pushButton_8_clicked();

private slots:
  void on_pushButton_4_clicked();

private slots:
  void on_pushButton_6_clicked();

private slots:
  void on_checkBox_stateChanged(int arg1);

private slots:
  void on_localgridmap_checkBox_stateChanged(int arg1);

private slots:
  void on_pushButton_reset_clicked();

private slots:
  void on_pushButton_minus_clicked();
  void on_pushButton_plus_clicked();
  void on_pushButton_return_clicked();
  void on_pushButton_clicked();
  /**
   * @brief MainWindow::on_bringup_button_clicked 启动机器人系统
   */
  void on_bringup_button_clicked();

  /**
   * @brief MainWindow::on_server_connect_Button_clicked
   *                连接服务器
   */
  void on_server_connect_Button_clicked();

  void slot_rosShutdown();
  void slot_roboPos(QPointF pos);

private:
  Ui::MainWindow *ui;
  QProcess *frontend_process_;
  QProcess *laser_process_;
  QProcess *hardware_process_;
  QProcess *recordBag_process_;
  ros_qt::QNode qt_ros_node_;

  QGraphicsScene *qgraphicsScene_ = NULL;
  ros_qt::roboItem *roboItem_ = NULL;
  Comm::SocketClient client_;
};

#endif // MainWindow_H
