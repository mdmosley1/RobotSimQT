#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include "Robot.h"
#include <iostream>

MainWindow::MainWindow(Robot* _robotPtr, QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  setGeometry(400, 250, 542, 390);

  setupPosPlot(ui->customPlotPos, _robotPtr);
  setupVelPlot(ui->customPlotVel, _robotPtr);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupPosPlot(QCustomPlot *customPlotPos, Robot* _robotPtr)
{
  demoName = "Real Time Data Demo";
  customPlotPos->addGraph(); // blue line
  customPlotPos->graph(0)->setPen(QPen(QColor(40, 110, 255)));
  customPlotPos->addGraph(); // red line
  customPlotPos->graph(1)->setPen(QPen(QColor(255, 110, 40)));

  QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
  timeTicker->setTimeFormat("%h:%m:%s");
  customPlotPos->xAxis->setTicker(timeTicker);
  customPlotPos->axisRect()->setupFullAxesBox();
  customPlotPos->yAxis->setRange(0, 1000);
  
  // make left and bottom axes transfer their ranges to right and top axes:
  connect(customPlotPos->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlotPos->xAxis2, SLOT(setRange(QCPRange)));
  connect(customPlotPos->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlotPos->yAxis2, SLOT(setRange(QCPRange)));
  
  // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
  //connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
  connect(_robotPtr, SIGNAL(PositionChanged(double, double)),
          this,      SLOT(PlotPosition(double, double)));
  //dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::setupVelPlot(QCustomPlot *customPlotVel, Robot* _robotPtr)
{
  demoName = "Real Time Data Demo";
  customPlotVel->addGraph(); // blue line
  customPlotVel->graph(0)->setPen(QPen(QColor(40, 110, 255)));

  QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
  timeTicker->setTimeFormat("%h:%m:%s");
  customPlotVel->xAxis->setTicker(timeTicker);
  customPlotVel->axisRect()->setupFullAxesBox();
  customPlotVel->yAxis->setRange(0, 1000);
  
  // make left and bottom axes transfer their ranges to right and top axes:
  connect(customPlotVel->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlotVel->xAxis2, SLOT(setRange(QCPRange)));
  connect(customPlotVel->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlotVel->yAxis2, SLOT(setRange(QCPRange)));
  
  // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
  //connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
  connect(_robotPtr, SIGNAL(UpdateVelocity(double)),
          this,      SLOT(PlotVelocity(double)));
  //dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}



void MainWindow::PlotPosition(double _x, double _y)
{
  static QTime time(QTime::currentTime());
  // calculate two new data points:
  double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
  static double lastPointKey = 0;
  if (key-lastPointKey > 0.002) // at most add point every 2 ms
  {
    // add data to lines:
    ui->customPlotPos->graph(0)->addData(key, _x);
    ui->customPlotPos->graph(1)->addData(key, _y);
    // rescale value (vertical) axis to fit the current data:
    ui->customPlotPos->graph(0)->rescaleValueAxis();
    ui->customPlotPos->graph(1)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->customPlotPos->xAxis->setRange(key, 8, Qt::AlignRight);
  ui->customPlotPos->replot();
  
  // calculate frames per second:
  static double lastFpsKey;
  static int frameCount;
  ++frameCount;
  if (key-lastFpsKey > 2) // average fps over 2 seconds
  {
    ui->statusBar->showMessage(
          QString("%1 FPS, Total Data points: %2")
          .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
          .arg(ui->customPlotPos->graph(0)->data()->size()
               +ui->customPlotPos->graph(1)->data()->size())
          , 0);
    lastFpsKey = key;
    frameCount = 0;
  }
}

void MainWindow::PlotVelocity(double _velocity)
{
  static QTime time(QTime::currentTime());
  // calculate two new data points:
  double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
  static double lastPointKey = 0;
  if (key-lastPointKey > 0.002) // at most add point every 2 ms
  {
    // add data to lines:
    ui->customPlotVel->graph(0)->addData(key, _velocity);
    // rescale value (vertical) axis to fit the current data:
    ui->customPlotVel->graph(0)->rescaleValueAxis();
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->customPlotVel->xAxis->setRange(key, 8, Qt::AlignRight);
  ui->customPlotVel->replot();
  
  // calculate frames per second:
  static double lastFpsKey;
  static int frameCount;
  ++frameCount;
  if (key-lastFpsKey > 2) // average fps over 2 seconds
  {
    ui->statusBar->showMessage(
          QString("%1 FPS, Total Data points: %2")
          .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
          .arg(ui->customPlotVel->graph(0)->data()->size())
          , 0);
    lastFpsKey = key;
    frameCount = 0;
  }
}
