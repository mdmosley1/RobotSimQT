
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "qcustomplot.h" // the header file of QCustomPlot. Don't forget to add it to your project, if you use an IDE, so it gets compiled.
#include "Robot.h" // window needs to know about robot so it can receive its pose update signals

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT
  
public:
  MainWindow(Robot* _robotPtr, QWidget *parent = 0);
  ~MainWindow();
  
  void setupPosPlot(QCustomPlot *customPlot, Robot* _robotPtr);
  void setupVelPlot(QCustomPlot *customPlot, Robot* _robotPtr);
  
private slots:
    //void realtimeDataSlot();
    void PlotPosition(double x, double y);
    void PlotVelocity(double);

  
private:
  Ui::MainWindow *ui;
  QString demoName;
  QTimer dataTimer;
  QCPItemTracer *itemDemoPhaseTracer;
  int currentDemoIndex;
};

#endif // MAINWINDOW_H
