
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "qcustomplot.h" // the header file of QCustomPlot. Don't forget to add it to your project, if you use an IDE, so it gets compiled.
#include "Robot.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT
  
public:
  MainWindow(Robot* _robotPtr, QWidget *parent = 0);
  ~MainWindow();
  
  void setupRealtimeDataDemo(QCustomPlot *customPlot, Robot* _robotPtr);
  
private slots:
    //void realtimeDataSlot();
    void realtimeDataSlot(double x, double y);

  
private:
  Ui::MainWindow *ui;
  QString demoName;
  QTimer dataTimer;
  QCPItemTracer *itemDemoPhaseTracer;
  int currentDemoIndex;
};

#endif // MAINWINDOW_H
