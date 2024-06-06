#include <QMovie>
#include <QMessageBox>
#include <QDebug>
#include <QNetworkInterface>
#include <QFileDialog>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "MainWindow.h"
#include "ui_MainWindow.h"    // build/my_gui下 
#include "ipc/DataDispatcher.hpp"
#include "proto/control_cmd.pb.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QWidget(parent), ui(new Ui::MainWindow), qt_ros_node_(argc, argv) {
  initUI();    // 初始化UI
  initParm(); // 初始化参数
  // 系统信息输出
  ui->msg_output->setReadOnly(true);
  // 设置最大显示文本容量为100个文本块
  ui->msg_output->document()->setMaximumBlockCount(100);
  navigation_process_ = new QProcess;
  frontend_process_ = new QProcess;
  laser_process_ = new QProcess;
  hardware_process_ = new QProcess;
  recordBag_process_ = new QProcess;
  // 将ROS节点的输出重定向到QTextBrowser控件中
  QObject::connect(frontend_process_, &QProcess::readyReadStandardOutput, [&]() {
      QString output = frontend_process_->readAllStandardOutput();
      ui->msg_output->append(output);
  });
  // 设置QProcess的标准输出通道为MergedChannels
  frontend_process_->setProcessChannelMode(QProcess::MergedChannels);
  bringUpQtRosNode();
  // 信号，槽连接
  connection();
  // this->setFocus();
  this->setFocusPolicy(Qt::ClickFocus);
  qt_ros_node_.setMainWindowUi(ui);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow() {
  qDebug() << "~MainWindow()";
  delete ui;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::initUI() {
  ui->setupUi(this);
  //  Qt::Window: 指示窗口是一个普通窗口，拥有标题栏和边框。
  //  Qt::WindowTitleHint: 显示窗口标题栏。
  //  Qt::CustomizeWindowHint: 禁用默认的窗口装饰，即关闭、最大化和最小化按钮
  //  this->setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);
//  this->setWindowFlags(Qt::FramelessWindowHint);  //去掉标题栏

//  this->setFixedSize(700, 500);
//  // 显示界面的动态背景
////  QMovie* movie = new QMovie("/home/lwh/图片/8059.gif_wh300.gif");
////  ui->label_vedio->setMovie(movie);
//  ui->label_vedio->lower();   // 作为背景
//  ui->label_vedio->setFixedSize(700, 500);
//  ui->label_vedio->setPixmap(QPixmap("/home/lwh/图片/back.jpg"));  // 设置背景图片
////  this->setCentralWidget(backgroundLabel);  // 将QLabel设置为窗口的中心部件
//  ui->label_vedio->setScaledContents(true);  // 设置QLabel的内容自适应缩放
////  movie->setScaledSize(ui->label_vedio->size());  // 将QMovie的缩放大小设置为QLabel的大小
////  movie->start();
////  ui->label_vedio->show();

//  QPalette palette = ui->label_head->palette();  // 获取QLabel的调色板
//  palette.setColor(QPalette::WindowText, Qt::darkMagenta);  // 设置文字颜色为红色
//  ui->label_head->setPalette(palette);  // 应用新的调色板

//  // 标题处的机器人图片
//  ui->label_robot_img->setPixmap(QPixmap("://images/robot.png"));
//  ui->label_robot_img->setScaledContents(true);  // 设置QLabel的内容自适应缩放
  ui->pushButton_plus->setIcon(QIcon(":/images/plus.png"));
  ui->pushButton_plus->setFixedHeight(50);
  ui->pushButton_plus->setFixedWidth(50);
  ui->pushButton_minus->setIcon(QIcon(":/images/minus.png"));
  ui->pushButton_minus->setFixedHeight(50);
  ui->pushButton_minus->setFixedWidth(50);
  ui->pushButton_return->setIcon(QIcon(":/images/set_return.png"));
  ui->pushButton_return->setFixedHeight(50);
  ui->pushButton_return->setFixedWidth(50);

  ui->pushButton_6->setIcon(QIcon(":/images/classes/Group.png"));

  ui->pushButton_8->setEnabled(false);

  ui->toolButton->setIcon(QIcon(":/images/classes/PublishPoint.svg"));
  ui->toolButton->setIconSize(QSize(60, 60));
  ui->toolButton->setFixedHeight(100);
  ui->toolButton->setFixedWidth(100);
  ui->toolButton->setText("记录站点");
  ui->toolButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

  ui->toolButton_2->setIcon(QIcon(":/images/moverotate.svg"));
  ui->toolButton_2->setIconSize(QSize(60, 60));
  ui->toolButton_2->setFixedHeight(100);
  ui->toolButton_2->setFixedWidth(100);
  ui->toolButton_2->setText("设置目标");
  ui->toolButton_2->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

  ui->toolButton_3->setIcon(QIcon(":/images/zoom.svg"));
  ui->toolButton_3->setIconSize(QSize(60, 60));

  QImage img;
  img.load(":/images/control.png");
  QImage scaledImg = img.scaled(60, 60, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ui->toolButton_3->setIcon(QPixmap::fromImage(scaledImg));
  ui->toolButton_3->setIconSize(QPixmap::fromImage(scaledImg).size());
  ui->toolButton_3->setFixedHeight(100);
  ui->toolButton_3->setFixedWidth(100);
  ui->toolButton_3->setText("运行");
  ui->toolButton_3->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

  ui->toolButton_4->setIcon(QIcon(":/images/forbidden.svg"));
  ui->toolButton_4->setIconSize(QSize(60, 60));
  ui->toolButton_4->setFixedHeight(100);
  ui->toolButton_4->setFixedWidth(100);
  ui->toolButton_4->setText("停止");
  ui->toolButton_4->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

  img.load(":/images/return.png");
  scaledImg = img.scaled(60, 60, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ui->toolButton_5->setIcon(QPixmap::fromImage(scaledImg));
  ui->toolButton_5->setIconSize(QPixmap::fromImage(scaledImg).size());
  ui->toolButton_5->setFixedHeight(100);
  ui->toolButton_5->setFixedWidth(100);
  ui->toolButton_5->setText("重置");
  ui->toolButton_5->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

  img.load(":/images/default_package_icon.png");
  scaledImg = img.scaled(60, 60, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ui->toolButton_6->setIcon(QPixmap::fromImage(scaledImg));
  ui->toolButton_6->setIconSize(QPixmap::fromImage(scaledImg).size());
  ui->toolButton_6->setFixedHeight(100);
  ui->toolButton_6->setFixedWidth(100);
  ui->toolButton_6->setText("保存");
  ui->toolButton_6->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

  ui->toolButton_7->setIcon(QIcon(":/images/classes/Views.svg"));
  ui->toolButton_7->setIconSize(QSize(60, 60));
  ui->toolButton_7->setFixedHeight(100);
  ui->toolButton_7->setFixedWidth(100);
  ui->toolButton_7->setText("录制");
  ui->toolButton_7->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

  ui->label_power->setPixmap(QPixmap(":/images/power.png"));
  //视图场景加载
  qgraphicsScene_ =
      // new QCustomQGraphicsScene;  //要用QGraphicsView就必须要有QGraphicsScene搭配着用
      new QGraphicsScene;  //要用QGraphicsView就必须要有QGraphicsScene搭配着用
  qgraphicsScene_->clear();
  //创建item
  roboItem_ = new ros_qt::roboItem();
  //视图添加item
  qgraphicsScene_->addItem(roboItem_);
  // widget添加视图
  ui->mapViz->setScene(qgraphicsScene_);
//  QRectF visibleRect = ui->mapViz->mapToScene(ui->mapViz->viewport()->geometry()).boundingRect();
//  QPointF visibleRect = ui->mapViz->mapToScene(0, 0);
//  QPointF sceneCenter = ui->mapViz->mapToScene(ui->mapViz->viewport()->rect().topLeft());
//  qDebug() << "sceneCenter w:" << sceneCenter.x() << ", h: " << sceneCenter.y();
  // 视图内   显示内容UI设置
  QString styleSheet = "QCheckBox::indicator{width: 22px;height: 22px;color:rgb(0, 191, 255)}\
        QCheckBox{font-size: 18px;color: rgb(0, 191, 255);}\
        QCheckBox::checked{color:rgb(50,205,50);}\
        QCheckBox::unchecked{color:rgb(119, 136, 153);}\
        ";
  ui->localgridmap_checkBox->setStyleSheet(styleSheet);
  ui->checkBox->setStyleSheet(styleSheet);
  ui->checkBox_2->setStyleSheet(styleSheet);
  ui->checkBox_3->setStyleSheet(styleSheet);
  ui->checkBox_4->setStyleSheet(styleSheet);
  ui->checkBox_5->setStyleSheet(styleSheet);
  ui->checkBox_6->setStyleSheet(styleSheet);
  ui->checkBox_7->setStyleSheet(styleSheet);
  // 视图内   视角选择UI设置
  styleSheet = "QRadioButton::indicator{width: 22px;height: 22px;color:rgb(0, 191, 255)}\
      QRadioButton{font-size: 18px;color: rgb(0, 191, 255);}\
      QRadioButton::checked{color:rgb(50,205,50);}\
      QRadioButton::unchecked{color:rgb(119, 136, 153);}\
      ";
  ui->radioButton->setStyleSheet(styleSheet);
  ui->radioButton_2->setStyleSheet(styleSheet);
  ui->radioButton_4->setStyleSheet(styleSheet);
  ui->radioButton_4->setChecked(true);

  ui->tabWidget->setFixedWidth(270);
  // ui->tabWidget->setFixedHeight(400);
  // ui->groupBox_3->setFixedWidth(300);
//  ui->groupBox_10->setFixedWidth(300);
//  ui->groupBox_6->setFixedHeight(250);
  // 放大缩小设置
//  ui->groupBox_5->setFixedWidth(40);
////  ui->groupBox_5->setStyleSheet("QGroupBox { padding: 2px; }");
//  ui->groupBox_5->setStyleSheet("QGroupBox { border: none; }");

  ui->server_connect_Button->setFixedHeight(60);
  ui->server_connect_Button->setFixedWidth(60);
//  // 退出
//  ui->pushButton->setFixedHeight(50);
//  ui->pushButton->setFixedWidth(100);
  // ui->image_label_0->setFixedSize(300, 200);
  ui->groupBox_4->setHidden(1);
  ui->groupBox_6->setFixedHeight(100);
  ui->groupBox_7->setFixedHeight(100);

  ui->label_43->setText(QString::number(roboItem_->param.linear_v));
  ui->label_45->setText(QString::number(roboItem_->param.angular_v));
  // 标定 轮速-laser的数值设置为红色
  QPalette palette = ui->label_28->palette();
  palette.setColor(QPalette::WindowText, Qt::red); // 设置文本颜色为红色
  ui->label_28->setPalette(palette);
  palette = ui->label_30->palette();
  palette.setColor(QPalette::WindowText, Qt::red); // 设置文本颜色为红色
  ui->label_30->setPalette(palette);
  palette = ui->label_32->palette();
  palette.setColor(QPalette::WindowText, Qt::red); // 设置文本颜色为红色
  ui->label_32->setPalette(palette);
  palette = ui->label_34->palette();
  palette.setColor(QPalette::WindowText, Qt::red); // 设置文本颜色为红色
  ui->label_34->setPalette(palette);
  palette = ui->label_35->palette();
  palette.setColor(QPalette::WindowText, Qt::red); // 设置文本颜色为红色
  ui->label_35->setPalette(palette);

  // 传感器状态
  palette = ui->label_laser_state->palette();
  palette.setColor(QPalette::WindowText, Qt::green);
  ui->label_laser_state->setPalette(palette);
  palette = ui->label_wheel_state->palette();
  palette.setColor(QPalette::WindowText, Qt::green);
  ui->label_wheel_state->setPalette(palette);
  palette = ui->label_imu_state->palette();
  palette.setColor(QPalette::WindowText, Qt::green);
  ui->label_imu_state->setPalette(palette);

  palette = ui->label_6->palette();  // 获取QLabel的调色板
  palette.setColor(QPalette::WindowText, Qt::red);  // 设置文字颜色为红色
  ui->label_6->setPalette(palette);  // 应用新的调色板
  ui->label_6->setText("服务器未连接");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::initParm() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::connection() {
  QObject::connect(&qt_ros_node_, &ros_qt::QNode::rosShutdown, this,
                 &MainWindow::slot_rosShutdown);
  QObject::connect(&qt_ros_node_, &ros_qt::QNode::updateStableLaserScan, roboItem_,
          &ros_qt::roboItem::paintStableLaserScan);
  QObject::connect(&qt_ros_node_, &ros_qt::QNode::updateDynamicLaserScan, roboItem_,
          &ros_qt::roboItem::paintDynamicLaserScan);
  connect(&qt_ros_node_, &ros_qt::QNode::updateSubGridMap, roboItem_,
          &ros_qt::roboItem::paintSubGridMap);
  connect(&qt_ros_node_, &ros_qt::QNode::showImage, roboItem_,
          [&](const int& num, const QImage& img) {
        if (ui->groupBox_4->isHidden()) {
          return;
        }
        // // 创建一个QImage（这里只是示例，你可以加载自己的图像）
        // QImage image(200, 200, QImage::Format_RGB888);
        // image.fill(Qt::blue); // 用蓝色填充图像
        QPixmap pixmap = QPixmap::fromImage(img);
        pixmap = pixmap.scaled(ui->image_label_0->size(), Qt::KeepAspectRatio);
        ui->image_label_0->setPixmap(pixmap);
        ui->image_label_0->show();
  });

  connect(roboItem_, &ros_qt::roboItem::signalPub2DGoal , &qt_ros_node_,
          &ros_qt::QNode::pub2DGoal);


  // 机器人pose -> map
  connect(&qt_ros_node_, &ros_qt::QNode::updateRoboPose, roboItem_,
          &ros_qt::roboItem::paintRoboPos);
  // 间断跟踪模式下对robopos的处理
  connect(roboItem_, &ros_qt::roboItem::roboPos , this,
          &MainWindow::slot_roboPos);
  // radioButton
  // QRadioButton 有一个状态切换的信号 toggled，即该信号在状态切换时发送；
  connect(ui->radioButton, &QRadioButton::toggled, [=](bool isChecked) {
      if (isChecked == true) {
        roboItem_->SetVisualMode(ros_qt::roboItem::VisualMode::translate_tracking);
      }
  });
  connect(ui->radioButton_2, &QRadioButton::toggled, [=](bool isChecked){
      if (isChecked == true) {}
  });
  connect(ui->radioButton_4, &QRadioButton::toggled, [=](bool isChecked){
      if (isChecked == true) {
        roboItem_->SetVisualMode(ros_qt::roboItem::VisualMode::internal_tracking);
      }
  });
  // 接收轮速轨迹->绘制轮速轨迹
  connect(&qt_ros_node_, &ros_qt::QNode::wheelOdomPathSignals, roboItem_,
          &ros_qt::roboItem::paintWheelOdomPath);
  // 接收全局规划轨迹->绘制轨迹
  connect(&qt_ros_node_, &ros_qt::QNode::globalPlannerPathSignal, roboItem_,
          &ros_qt::roboItem::paintGlobalPlanningPath);
  // 接收dwa局部规划轨迹->绘制轨迹
  connect(&qt_ros_node_, &ros_qt::QNode::dwaLocalPathSignal, roboItem_,
          &ros_qt::roboItem::paintDWALocalPath);

  ipc::DataDispatcher::GetInstance().Subscribe("ServerMsg",
                                                                                              &MainWindow::serverMsgCallback,
                                                                                              this,
                                                                                              1000,
                                                                                              true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::simulateKeyPress   模拟键盘按下事件
/// \param keyCode
/// \param modifiers
///
void MainWindow::simulateKeyPress(int keyCode, Qt::KeyboardModifiers modifiers) {
    QKeyEvent event(QEvent::KeyPress, keyCode, modifiers);
    QCoreApplication::sendEvent(this, &event);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::serverMsgCallback
/// \param msg
///
void MainWindow::serverMsgCallback(const std::pair<uint8_t, std::string>& msg) {
  // 反序列化
  // std::cout << "接收到信号" << "\n";
  if (msg.first == 1) {
    // std::cout << "接收到控制信号" << "\n";
    comm::cmd::proto::ControlCmd control_cmd_packet;
    if (control_cmd_packet.ParseFromString(msg.second)) {
      // qt_ros_node_.move_base(control_cmd_packet.cmd()[0],
      //                                                       roboItem_->param.linear_v, roboItem_->param.angular_v);
      // std::cout << "control_cmd_packet.cmd()[0]: " << control_cmd_packet.cmd()[0] << "\n";
      char key = control_cmd_packet.cmd()[0];
      if (key == 'a') {
        simulateKeyPress(Qt::Key_A, Qt::NoModifier); // 模拟按下 'A' 键
      } else if (key == 'd') {
        simulateKeyPress(Qt::Key_D, Qt::NoModifier);
      } else if (key == 'w') {
        simulateKeyPress(Qt::Key_W, Qt::NoModifier);
      } else if (key == 'x') {
        simulateKeyPress(Qt::Key_X, Qt::NoModifier);
      } else if (key == 's') {
        simulateKeyPress(Qt::Key_S, Qt::NoModifier);
      } else if (key == 'q') {
        simulateKeyPress(Qt::Key_Q, Qt::NoModifier);
      } else if (key == 'e') {
        simulateKeyPress(Qt::Key_E, Qt::NoModifier);
      } else if (key == 'z') {
        simulateKeyPress(Qt::Key_Z, Qt::NoModifier);
      } else if (key == 'c') {
        simulateKeyPress(Qt::Key_C, Qt::NoModifier);
      } else if (key == '7') {
        simulateKeyPress(Qt::Key_7, Qt::NoModifier);
      } else if (key == '8') {
        simulateKeyPress(Qt::Key_8, Qt::NoModifier);
      } else if (key == '9') {
        simulateKeyPress(Qt::Key_9, Qt::NoModifier);
      } else if (key == '0') {
        simulateKeyPress(Qt::Key_0, Qt::NoModifier);
      }
    } else {
      std::cout << "解析错误" << "\n";
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::bringUpQtRosNode() {
  // 读取本机的IP
  QString hostUrl = getLocalIpAddress();
  if (hostUrl.isEmpty()) {
    hostUrl = "NanoPi-M4";
    // hostUrl = getenv("HOSTNAME");
    // qDebug() << "hostUrl: " << hostUrl;
  }
  QString masterUrl = "http://" + hostUrl + ":11311";
  // qDebug() << "masterUrl: " << masterUrl;
  if (!qt_ros_node_.init(masterUrl.toStdString(), hostUrl.toStdString())) {
    QPalette palette = ui->label_status->palette();  // 获取QLabel的调色板
    palette.setColor(QPalette::WindowText, Qt::red);  // 设置文字颜色为红色
    ui->label_status->setPalette(palette);  // 应用新的调色板
    ui->label_status->setText("ROS 连接失败");
    ui->label_status_img->setPixmap(QPixmap("://images/false.png"));  // 设置背景图片
    ui->label_status_img->setScaledContents(true);  // 设置QLabel的内容自适应缩放
    ui->label_status_img->show();
    ui->toolButton_3->setEnabled(false);
    return;
  } else {
    QPalette palette = ui->label_status->palette();  // 获取QLabel的调色板
    palette.setColor(QPalette::WindowText, Qt::darkGreen);  // 设置文字颜色为红色
    ui->label_status->setPalette(palette);  // 应用新的调色板
    ui->label_status->setText("ROS 连接成功");
    ui->label_status_img->setPixmap(QPixmap("://images/ok.png"));  // 设置背景图片
    ui->label_status_img->setScaledContents(true);  // 设置QLabel的内容自适应缩放
    ui->label_status_img->show();
//    //初始化视频订阅的显示
//    initVideos();
//    //显示话题列表
//    initTopicList();
//    initOthers();
  }
//  ReadSettings();
  return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::keyPressEvent(QKeyEvent* event) {
  if(event->key() == Qt::Key_A) {
    // 区分大小写
    // if (event->text().toLower() == "c") {
    //     // 小写的 'c' 键被按下
    //     qDebug() << "小写的 'c' 键被按下";
    // } else if (event->text().toUpper() == "C") {
    //     // 大写的 'C' 键被按下
    //     qDebug() << "大写的 'C' 键被按下";
    // }
    qt_ros_node_.move_base('a', roboItem_->param.linear_v, roboItem_->param.angular_v);
    ui->label_41->setText("左转");
  } else if(event->key() == Qt::Key_D) {
    qt_ros_node_.move_base('d', roboItem_->param.linear_v, roboItem_->param.angular_v);
    ui->label_41->setText("右转");
  } else if(event->key() == Qt::Key_W) {
    qt_ros_node_.move_base('w', roboItem_->param.linear_v, roboItem_->param.angular_v);
    ui->label_41->setText("前进");
  } else if(event->key() == Qt::Key_X) {
    qt_ros_node_.move_base('x', roboItem_->param.linear_v, roboItem_->param.angular_v);
    ui->label_41->setText("后退");
  } else if(event->key() == Qt::Key_S) {
    qt_ros_node_.move_base('s', roboItem_->param.linear_v, roboItem_->param.angular_v);
    ui->label_41->setText("停止");
  } else if(event->key() == Qt::Key_Q) {
    qt_ros_node_.move_base('q', roboItem_->param.linear_v, roboItem_->param.angular_v);
    ui->label_41->setText("左上");
  } else if(event->key() == Qt::Key_E) {
    qt_ros_node_.move_base('e', roboItem_->param.linear_v, roboItem_->param.angular_v);
    ui->label_41->setText("右上");
  } else if(event->key() == Qt::Key_Z) {
    qt_ros_node_.move_base('z', roboItem_->param.linear_v, roboItem_->param.angular_v);
    ui->label_41->setText("左下");
  } else if(event->key() == Qt::Key_C) {
    qt_ros_node_.move_base('c', roboItem_->param.linear_v, roboItem_->param.angular_v);
    ui->label_41->setText("右下");
  } else if(event->key() == Qt::Key_7) {
    roboItem_->param.linear_v *= 1.1;
    ui->label_43->setText(QString::number(roboItem_->param.linear_v));
  } else if(event->key() == Qt::Key_8) {
    roboItem_->param.linear_v *= 0.9;
    ui->label_43->setText(QString::number(roboItem_->param.linear_v));
  } else if(event->key() == Qt::Key_9) {
    roboItem_->param.angular_v *= 1.1;
    ui->label_45->setText(QString::number(roboItem_->param.angular_v));
  } else if(event->key() == Qt::Key_0) {
    roboItem_->param.angular_v *= 0.9;
    ui->label_45->setText(QString::number(roboItem_->param.angular_v));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_bringup_button_clicked() {

}

////////////////////////////////////////////////////////////////////////////////////////////////////////
QString MainWindow::getLocalIpAddress() {
    QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
    // 遍历所有接口的IP地址
    for (const QHostAddress &ipAddress : ipAddressesList) {
        // 只获取IPv4地址，忽略IPv6地址和回环地址
        if (ipAddress != QHostAddress::LocalHost && ipAddress.toIPv4Address()) {
            return ipAddress.toString();
        }
    }
    // 如果没有找到有效的IP地址，则返回空字符串
    return QString();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_server_connect_Button_clicked() {
    QString ip = ui->line_ip_Edit->text();
    QString port = ui->line_port_Edit->text();
    qDebug() << "ip: " << ip;
    qDebug() << "port: " << port;
    if (qt_ros_node_.SocketClientConnect(port.toInt(), ip.toStdString()) == 1) {
      QPalette palette = ui->label_6->palette();  // 获取QLabel的调色板
      palette.setColor(QPalette::WindowText, Qt::green);  // 设置文字颜色为红色
      ui->label_6->setPalette(palette);  // 应用新的调色板
      ui->label_6->setText("服务器已连接");
      ui->server_connect_Button->setDisabled(true);
    } else {
      ui->label_6->setText("服务器连接失败");
    }
    // // 连接服务器
    // if (::connect(sockfd, (struct sockaddr*)&address, sizeof(address)) < 0) {
    //     QMessageBox::information(nullptr, "提示", "连接服务器失败");
    // } else {
    //     QMessageBox::information(nullptr, "提示", "连接服务器成功！");
    // }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::slot_rosShutdown() {
  qDebug() << "slot_rosShutdown";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::slot_roboPos
/// \param pos item 坐标系下的robot pos
///
void MainWindow::slot_roboPos(QPointF pos) {
  QPointF ScenePos = roboItem_->mapToScene(pos);
  QPointF ViewPos = ui->mapViz->mapFromScene(ScenePos);
  QRect view_rect = ui->mapViz->viewport()->rect();   // 获取视图坐标范围
//  qDebug() << "ViewPos.x(): " << ViewPos.x() << ", y: " << ViewPos.y();
  // 如果机器人移动到了窗口边缘，则要将窗口中心调整到机器人处
  if (ViewPos.x() < view_rect.width() * 0.2 || ViewPos.x() > view_rect.width() * 0.8
        || ViewPos.y() < view_rect.height() * 0.2 || ViewPos.y() > view_rect.height() * 0.8) {
    // 视图中心的item坐标系坐标
    QPointF scene_center = ui->mapViz->mapToScene(view_rect.center());   // 视图坐标系转换到scene坐标系
    QPointF scene_center_itempos = roboItem_->mapFromScene(scene_center);   // 再由scene坐标系转换到Item坐标系
    // 将view的中心移动到机器人上
    float dx = roboItem_->GetScale() * (scene_center_itempos.x() - pos.x());
    float dy = roboItem_->GetScale() * (scene_center_itempos.y() - pos.y());
    roboItem_->move(dx, dy);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_clicked() {
    if (frontend_process_->state() == QProcess::Running) {
        frontend_process_->terminate();
    }
    if (laser_process_->state() == QProcess::Running) {
        laser_process_->terminate();
    }
    if (hardware_process_->state() == QProcess::Running) {
        hardware_process_->terminate();
    }
    if (recordBag_process_->state() == QProcess::Running) {
      recordBag_process_->terminate();
    }
    emit Quit();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::viewCenterFocusOnRobot
///
void MainWindow::viewCenterFocusOnRobot() {
  QPoint viewCenter = ui->mapViz->viewport()->rect().center();   // 获取视图中心的坐标(视图坐标系)
  QPointF sceneCenter = ui->mapViz->mapToScene(viewCenter);   // 视图坐标系转换到scene坐标系
  QPointF itemPoint = roboItem_->mapFromScene(sceneCenter);   // 再由scene坐标系转换到Item坐标系
  const QPointF& robo_pos = roboItem_->GetRoboPos();
  float dx = roboItem_->GetScale() * (itemPoint.x() - robo_pos.x());
  float dy = roboItem_->GetScale() * (itemPoint.y() - robo_pos.y());
  roboItem_->move(dx, dy);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_pushButton_return_clicked
///             将视图中心与机器人对齐
void MainWindow::on_pushButton_return_clicked() {
    viewCenterFocusOnRobot();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_pushButton_plus_clicked
///             放大操作
void MainWindow::on_pushButton_plus_clicked() {
  // 获取放大操作的中心位置(相对于view窗口中心进行放大)
  QPoint viewCenter = ui->mapViz->viewport()->rect().center();   // 获取视图中心的坐标(视图坐标系)
  QPointF sceneCenter = ui->mapViz->mapToScene(viewCenter);   // 视图坐标系转换到scene坐标系
  QPointF itemPoint = roboItem_->mapFromScene(sceneCenter);   // 再由scene坐标系转换到Item坐标系
  roboItem_->ChangeScale(1, roboItem_->GetScale() * itemPoint);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_pushButton_minus_clicked
/// 缩小操作
void MainWindow::on_pushButton_minus_clicked() {
  // 获取缩小操作的中心位置(相对于view窗口中心进行缩小)
  QPoint viewCenter = ui->mapViz->viewport()->rect().center();   // 获取视图中心的坐标(视图坐标系)
  QPointF sceneCenter = ui->mapViz->mapToScene(viewCenter);   // 视图坐标系转换到scene坐标系
  QPointF itemPoint = roboItem_->mapFromScene(sceneCenter);   // 再由scene坐标系转换到Item坐标系
  roboItem_->ChangeScale(0, roboItem_->GetScale() * itemPoint);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_reset_clicked() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_localgridmap_checkBox_stateChanged(int arg1) {
    if (arg1) {
    //  qDebug() << "localgridmap on";
       qt_ros_node_.SetGridMapShowFlag(true);
       roboItem_->SetGridMapShow(true);
    } else {
    //  qDebug() << "localgridmap off";
      qt_ros_node_.SetGridMapShowFlag(false);
      roboItem_->SetGridMapShow(false);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_checkBox_stateChanged(int arg1) {
    if (arg1) {
      ui->groupBox_4->setHidden(0);
      int W = ui->mapViz->width();
      ui->groupBox_4->setFixedWidth(0.3 * W);
      ui->image_label_0->setFixedWidth(ui->groupBox_4->width());
      ui->image_label_0->setFixedHeight(ui->groupBox_4->width());
    } else {
      // ui->groupBox_4->setFixedWidth(0);
      ui->groupBox_4->setHidden(1);
    }
}

/**
 * @brief MainWindow::on_pushButton_6_clicked   设置bag包保存路径
 */
void MainWindow::on_pushButton_6_clicked() {
  // QString file_name = QFileDialog::getOpenFileNames(this,"open file dialog","/home/lwh","Txt files(*.txt)");
  QString file_name = QFileDialog::getExistingDirectory(this,"选择目录","/home");
  ui->lineEdit->setText(file_name);
}

/**
 * @brief MainWindow::on_pushButton_4_clicked 轮速laser标定开始
 */
void MainWindow::on_pushButton_4_clicked() {
  bool res;
  qt_ros_node_.laserWheelCalibCall(1, res);
  if (res) {
    ui->checkBox_7->setChecked(1);
    QMessageBox::information(nullptr, "提示", "开始进行轮速-激光标定！");
  } else {
    QMessageBox::information(nullptr, "提示", "轮速-激光标定已开启！");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_pushButton_8_clicked
/// 保存laser-wheel 标定结果
void MainWindow::on_pushButton_8_clicked() {
  bool res;
  qt_ros_node_.laserWheelCalibCall(2, res);
  ui->checkBox_7->setChecked(0);
  if (res) {
    QMessageBox::information(nullptr, "提示", "保存成功！");
  } else {
    QMessageBox::information(nullptr, "提示", "保存失败！");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_checkBox_7_stateChanged  是否显示轮速轨迹
/// \param arg1
///
void MainWindow::on_checkBox_7_stateChanged(int arg1) {
  if (arg1) {
    qt_ros_node_.SetWheelOdomSubscribe(1);
  } else {
    qt_ros_node_.SetWheelOdomSubscribe(0);
    roboItem_->clearWheelPath();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_toolButton_3_clicked
///   启动
void MainWindow::on_toolButton_3_clicked() {
  // hardware_process_->start("roslaunch", QStringList() << "robot_control" << "robot_control.launch");
  // laser_process_->start("roslaunch", QStringList() << "ydlidar_ros_driver" << "lidar.launch");
  // 延时1s  不然启动有问题
  QTime t;
  t.start();
  while(t.elapsed()<1000)//1000ms = 1s
        QCoreApplication::processEvents();

  // frontend_process_->start("roslaunch", QStringList() << "calib_fusion_2d" << "frontend_view.launch");
  // frontend_process_->start("roslaunch", QStringList() << "calib_fusion_2d" << "dataset_frontend_view.launch");
  frontend_process_->start("roslaunch", QStringList() << "calib_fusion_2d" << "frontend.launch");

  QTime t2;
  t2.start();
  while(t2.elapsed()<1000)//1000ms = 1s
        QCoreApplication::processEvents();
  navigation_process_->start("roslaunch", QStringList() << "move_base" << "nav.launch");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_toolButton_5_clicked
/// 重置
void MainWindow::on_toolButton_5_clicked() {
  // qDebug() << "pushButton_reset";
  qt_ros_node_.SetReset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_toolButton_4_clicked
/// 停止
void MainWindow::on_toolButton_4_clicked() {
  if (frontend_process_->state() == QProcess::Running) {
      frontend_process_->terminate();
  }
  if (navigation_process_->state() == QProcess::Running) {
      navigation_process_->terminate();
  }
  if (laser_process_->state() == QProcess::Running) {
      laser_process_->terminate();
  }
  if (hardware_process_->state() == QProcess::Running) {
      hardware_process_->terminate();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_toolButton_7_clicked
/// 录制bag
void MainWindow::on_toolButton_7_clicked() {
  if (hardware_process_->state() != QProcess::Running) {
      hardware_process_->start("roslaunch", QStringList() << "robot_control" << "robot_control.launch");
  }

  if (laser_process_->state() != QProcess::Running) {
      laser_process_->start("roslaunch", QStringList() << "ydlidar_ros_driver" << "lidar.launch");
  }

  QMessageBox msgBox;  
  msgBox.setWindowTitle("提示");  
  msgBox.setIcon(QMessageBox::Information);  
  msgBox.setSizeGripEnabled(true);  
  // msgBox.setMinimumSize(1600, 800); // 设置你需要的尺寸
  msgBox.setFixedHeight(1000);
  msgBox.setFixedWidth(800);

  if (recordBag_process_->state() != QProcess::Running) {
    // QMessageBox::information(nullptr, "提示", "开始录制Bag包");
    msgBox.setText("开始录制Bag包");  
    msgBox.exec();

    recordBag_process_->start("rosbag", QStringList() << "record" 
      << "--all" << "-o" << "/home/pi/");

    if (!recordBag_process_->waitForStarted(5000)) { // 等待5秒
      // 如果在5秒内进程没有启动，则退出等待
      // QMessageBox::information(nullptr, "提示", "录制失败");
      msgBox.setText("录制失败");  
      msgBox.exec();
      recordBag_process_->close();
    }
  } else {
    // QMessageBox::information(nullptr, "提示", "Bag包录制完成，以保存在/home/pi");
    msgBox.setText("Bag包录制完成，以保存在/home/pi");  
    msgBox.exec();
    recordBag_process_->terminate();
    hardware_process_->terminate();
    laser_process_->terminate();
  }
}


void MainWindow::on_toolButton_2_clicked() {
  roboItem_->SetGoal();
}

