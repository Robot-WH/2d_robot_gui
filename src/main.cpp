
/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QApplication>
#include <QDesktopWidget>
#include <QtGui>

#include "MainWindow.h"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);
  //共享opengl上下文
  //    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts)
  //    QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);
  // app.setOverrideCursor(QCursor(Qt::BlankCursor));  
  MainWindow w(argc, argv);
  // w.setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
  // w.showFullScreen();
     w.show();
//  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  QObject::connect(&w, &MainWindow::Quit, &app, &QCoreApplication::quit);
  int result = app.exec();  // 程序执行  相当于不停的循环

  return result;
}
