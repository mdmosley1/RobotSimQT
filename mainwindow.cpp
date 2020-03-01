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
  setupCovarPlot(ui->customPlotCovar, _robotPtr);
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
  connect(_robotPtr, SIGNAL(PositionChanged(double, double, double)),
          this,      SLOT(PlotPosition(double, double, double)));
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
  connect(_robotPtr, SIGNAL(UpdateVelocity(double, double)),
          this,      SLOT(PlotVelocity(double,double)));
  //dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::setupCovarPlot(QCustomPlot *customPlotCovar, Robot* _robotPtr)
{
  demoName = "Real Time Data Demo";
  customPlotCovar->addGraph(); // blue line
  customPlotCovar->graph(0)->setPen(QPen(QColor(40, 110, 255)));

  QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
  timeTicker->setTimeFormat("%h:%m:%s");
  customPlotCovar->xAxis->setTicker(timeTicker);
  customPlotCovar->axisRect()->setupFullAxesBox();
  customPlotCovar->yAxis->setRange(0, 1000);
  
  // make left and bottom axes transfer their ranges to right and top axes:
  connect(customPlotCovar->xAxis,
          SIGNAL(rangeChanged(QCPRange)),
          customPlotCovar->xAxis2,
          SLOT(setRange(QCPRange)));
  connect(customPlotCovar->yAxis,
          SIGNAL(rangeChanged(QCPRange)),
          customPlotCovar->yAxis2,
          SLOT(setRange(QCPRange)) );
  
  connect(_robotPtr, SIGNAL(UpdateErrorCovariance(double, double)),
           this,      SLOT( PlotCovariance(double, double)) );
}



void MainWindow::PlotPosition(double _x, double _y, double _simTime)
{
  double key = _simTime; // time elapsed since start of demo, in seconds
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
}

void MainWindow::PlotVelocity(double _velocity, double _simTime)
{
  double key = _simTime; // time elapsed since start of demo, in seconds
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
}

void MainWindow::PlotCovariance(double _covariance, double _simTime)
{
  double key = _simTime; // time elapsed since start of demo, in seconds
  static double lastPointKey = 0;
  if (key-lastPointKey > 0.002) // at most add point every 2 ms
  {
    // add data to lines:
    ui->customPlotCovar->graph(0)->addData(key, _covariance);
    // rescale value (vertical) axis to fit the current data:
    ui->customPlotCovar->graph(0)->rescaleValueAxis();
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->customPlotCovar->xAxis->setRange(key, 8, Qt::AlignRight);
  ui->customPlotCovar->replot();
}
