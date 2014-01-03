/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/controller_interface/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace controller_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    //QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    qnode.init();
    //setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(updateDepth(double)), this, SLOT(updateDepth(double)));
    QObject::connect(&qnode, SIGNAL(updateYaw(double)), this, SLOT(updateYaw(double)));
    
    ui.yawDisplay->setNeedle(new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Ray,false, Qt::red));
    count_depth = 0;
    count_yaw = 0;
    updateYaw(0);
    updateDepth(0);
    /**
     * Setting up plots
     **/
    curve1 = new QwtPlotCurve("Error_depth");
    curve1->setRenderHint(QwtPlotItem::RenderAntialiased);
    curve1->setLegendAttribute(QwtPlotCurve::LegendShowLine, true);
    curve1->setPen(QPen(Qt::red));
    curve1->attach(ui.depthPlot);
    //
    curve2 = new QwtPlotCurve("Error_yaw");
    curve2->setRenderHint(QwtPlotItem::RenderAntialiased);
    curve2->setLegendAttribute(QwtPlotCurve::LegendShowLine, true);
    curve2->setPen(QPen(Qt::red));
    curve2->attach(ui.yawPlot);
}

MainWindow::~MainWindow() {}


void MainWindow::closeEvent(QCloseEvent *event)
{
    qnode.stopThruster();
    QMainWindow::closeEvent(event);
}


/******************************************
** Thruster functionsSpeed
*******************************************/
void MainWindow::on_testThrusters_clicked()
{
    std::cout<<"Thruster tested"<<std::endl;
    qnode.testThrusters();
}
void MainWindow::on_startThrusters_clicked()
{
    std::cout<<"Thruster Started"<<std::endl;
    qnode.stopThruster();
    qnode.runThrusters = true;
}
void MainWindow::on_stopThrusters_clicked()
{
    std::cout<<"Thruster Stoped"<<std::endl;
    qnode.stopThruster();
}

/******************************************
** Depth Controller{}
*******************************************/
void MainWindow::on_startDepthController_clicked()
{
    std::cout<<"Depth Started"<<std::endl;
    qnode.resetDepthController();
    qnode.depth_controller = true;
}
void MainWindow::on_stopDepthController_clicked()
{
    std::cout<<"Depth Stoped"<<std::endl;
    qnode.resetDepthController();
}
void MainWindow::on_depthPlotStart_clicked()
{
    std::cout<<"Depth Plot Started"<<std::endl;
    depth_plot = true;
}
void MainWindow::on_depthPlotStop_clicked()
{
    std::cout<<"Depth Plot Stoped"<<std::endl;
    depth_plot = false;
    QVector<QPointF> q;
    q.push_back(QPointF(0,0));
    error_depth_data.setSamples(q);
    curve1->setData(&error_depth_data);
    ui.depthPlot->replot();
}
void MainWindow::on_kpDepth_valueChanged(double d)
{
    std::cout<<"KpDepth:"<<d<<std::endl;
    qnode.KP_DEPTH = d;
}
//void MainWindow::on_kpDepth_valueChanged(const QString &str)
//{
//    std::cout<<"KpDepth(S):"<<str.toStdString()<<std::endl;
//}
void MainWindow::on_kiDepth_valueChanged(double d)
{
    std::cout<<"KiDepth:"<<d<<std::endl;
    qnode.KI_DEPTH = d;
}
//void MainWindow::on_kiDepth_valueChanged(const QString &str)
//{
//    std::cout<<"KiDepth(S):"<<str.toStdString()<<std::endl;
//}
void MainWindow::on_kdDepth_valueChanged(double d)
{
    std::cout<<"KdDepth:"<<d<<std::endl;
    qnode.KD_DEPTH = d;
}
//void MainWindow::on_kdDepth_valueChanged(const QString &str)
//{
//    std::cout<<"KdDepth(S):"<<str.toStdString()<<std::endl;
//}
void MainWindow::on_setDepth_valueChanged(double d)
{
    std::cout<<"setDepth:"<<d<<std::endl;
    qnode.steady_depth = d;
}
//void MainWindow::on_setDepth_valueChanged(const QString &str)
//{
 //   std::cout<<"setDepth(S):"<<str.toStdString()<<std::endl;
//}

/******************************************
** Yaw Controller{}
*******************************************/
void MainWindow::on_startYawController_clicked()
{
    std::cout<<"Yaw Started"<<std::endl;
    qnode.resetYawController();
    qnode.yaw_controller = true;
}
void MainWindow::on_stopYawController_clicked()
{
    std::cout<<"Yaw Stoped"<<std::endl;
    qnode.resetYawController();
}
void MainWindow::on_yawPlotStart_clicked()
{
    std::cout<<"Yaw plot Started"<<std::endl;
    yaw_plot = true;
}
void MainWindow::on_yawPlotStop_clicked()
{
    std::cout<<"Yaw plot Stoped"<<std::endl;
    yaw_plot = false;
    QVector<QPointF> q;
    q.push_back(QPointF(0,0));
    error_yaw_data.setSamples(q);
    curve2->setData(&error_yaw_data);
    ui.yawPlot->replot();
}
void MainWindow::on_kpYaw_valueChanged(double d)
{
    std::cout<<"KpYaw:"<<d<<std::endl;
    qnode.KP_YAW = d;
}
//void MainWindow::on_kpYaw_valueChanged(const QString &str)
//{
//    std::cout<<"KpYaw(S):"<<str.toStdString()<<std::endl;
//}
void MainWindow::on_kiYaw_valueChanged(double d)
{
    std::cout<<"KiYaw:"<<d<<std::endl;
    qnode.KI_YAW = d;
}
//void MainWindow::on_kiYaw_valueChanged(const QString &str)
//{
//    std::cout<<"KiYaw(S):"<<str.toStdString()<<std::endl;
//}
void MainWindow::on_kdYaw_valueChanged(double d)
{
    std::cout<<"KdYaw:"<<d<<std::endl;
    qnode.KD_YAW = d;
}
//void MainWindow::on_kdYaw_valueChanged(const QString &str)
//{
//    std::cout<<"KdYaw(S):"<<str.toStdString()<<std::endl;
//}
void MainWindow::on_setYaw_valueChanged(double d)
{
    std::cout<<"setYaw:"<<d<<std::endl;
    qnode.steady_angle = d;
}
//void MainWindow::on_setYaw_valueChanged(const QString &str)
//{
//    std::cout<<"setYaw(S):"<<str.toStdString()<<std::endl;
//}

/******************************************
** Speed Controller{}
*******************************************/
void MainWindow::on_startSpeedController_clicked()
{
    std::cout<<"Speed Started"<<std::endl;
    qnode.resetSpeedController();
    qnode.speed_controller = true;
}
void MainWindow::on_stopSpeedController_clicked()
{
    std::cout<<"Speed Stoped"<<std::endl;
    qnode.resetSpeedController();
}
void MainWindow::on_kpSpeed_valueChanged(double d)
{
    std::cout<<"KpSpeed:"<<d<<std::endl;
    qnode.KP_SPEED = d;
}
//void MainWindow::on_kpSpeed_valueChanged(const QString &str)
//{
//    std::cout<<"KpSpeed(S):"<<str.toStdString()<<std::endl;
//}
void MainWindow::on_kiSpeed_valueChanged(double d)
{
    std::cout<<"KiSpeed:"<<d<<std::endl;
    qnode.KI_SPEED = d;
}
//void MainWindow::on_kiSpeed_valueChanged(const QString &str)
//{
//    std::cout<<"KiSpeed(S):"<<str.toStdString()<<std::endl;
//}
void MainWindow::on_kdSpeed_valueChanged(double d)
{
    std::cout<<"KdSpeed:"<<d<<std::endl;
    qnode.KD_SPEED = d;
}
//void MainWindow::on_kdSpeed_valueChanged(const QString &str)
//{
//    std::cout<<"KdSpeed(S):"<<str.toStdString()<<std::endl;
//}
void MainWindow::on_setSpeed_valueChanged(double d)
{
    std::cout<<"setSpeed:"<<d<<std::endl;
    qnode.horizontal_speed = d;
}

void MainWindow::updateDepth(double value)
{
    //ui.depthDisplay->setValue(value*10);
    ui.currentDepth->setText(QString(QString::number(value)));
    ui.errorDepth->setText(QString(QString::number(qnode.error_depth)));
    error_depth_vec.push_back(QPointF(count_depth,value));
    if(count_depth<50)
    {
        
    }
    else
    {
        error_depth_vec.pop_front();
        if(depth_plot)
        {
            error_depth_data.setSamples(error_depth_vec);
            curve1->setData(&error_depth_data);
            ui.depthPlot->replot();
        }
    }
    count_depth++;
}

void MainWindow::updateYaw(double value)
{
    //std::cout<<"Yaw Update"<<value<<std::endl;
    ui.yawDisplay->setValue(value);
    ui.currentYaw->setText(QString(QString::number(value)));
    ui.errorYaw->setText(QString(QString::number(qnode.error_yaw)));
    error_yaw_vec.push_back(QPointF(count_yaw,value));
    if(count_yaw<50)
    {
    }
    else
    {
        error_yaw_vec.pop_front();
        if(yaw_plot)
        {
            //std::cout<<"Yaw Update:"<<count_yaw<<":"<<value<<":"<<error_yaw_vec.size()<<std::endl;
            error_yaw_data.setSamples(error_yaw_vec);
            curve2->setData(&error_yaw_data);
            curve2->detach();
            curve2->attach(ui.yawPlot);
            ui.yawPlot->replot();
        }
    }
    count_yaw++;
}

void MainWindow::updatePlots()
{
}

}  // namespace controller_gui

