/**
 * @file /include/controller_gui/main_window.hpp
 *
 * @brief Qt based gui for controller_gui.
 *
 * @date November 2010
 **/
#ifndef controller_gui_MAIN_WINDOW_H
#define controller_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <qwt_plot_curve.h>
#include <qwt_plot.h>
#include <qwt_compass_rose.h>
#include <qwt_dial_needle.h>
#include <boost/circular_buffer.hpp>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace controller_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	//void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	//void on_actionAbout_triggered();
	//void on_button_connect_clicked(bool check );
	//void on_checkbox_use_environment_stateChanged(int state);
    /******************************************
	** Thruster functions;
	*******************************************/
    void on_testThrusters_clicked();
    void on_startThrusters_clicked();
    void on_stopThrusters_clicked();
    
    /******************************************
	** Depth Controller;
	*******************************************/
    void on_startDepthController_clicked();
    void on_stopDepthController_clicked();
    void on_depthPlotStart_clicked();
    void on_depthPlotStop_clicked();
    void on_kpDepth_valueChanged(double);
    //void on_kpDepth_valueChanged(const QString &);
    void on_kiDepth_valueChanged(double);
    //void on_kiDepth_valueChanged(const QString &);
    void on_kdDepth_valueChanged(double);
    //void on_kdDepth_valueChanged(const QString &);
    void on_setDepth_valueChanged(double);
    //void on_setDepth_valueChanged(const QString &);
    
    /******************************************
	** Yaw Controller;
	*******************************************/
    void on_startYawController_clicked();
    void on_stopYawController_clicked();
    void on_yawPlotStart_clicked();
    void on_yawPlotStop_clicked();
    void on_kpYaw_valueChanged(double);
    //void on_kpYaw_valueChanged(const QString &);
    void on_kiYaw_valueChanged(double);
    //void on_kiYaw_valueChanged(const QString &);
    void on_kdYaw_valueChanged(double);
    //void on_kdYaw_valueChanged(const QString &);
    void on_setYaw_valueChanged(double);
   // void on_setYaw_valueChanged(const QString &);
    
    /******************************************
	** Speed Controller;
	*******************************************/
    void on_startSpeedController_clicked();
    void on_stopSpeedController_clicked();
    void on_kpSpeed_valueChanged(double);
    //void on_kpSpeed_valueChanged(const QString &);
    void on_kiSpeed_valueChanged(double);
   // void on_kiSpeed_valueChanged(const QString &);
    void on_kdSpeed_valueChanged(double);
    //void on_kdSpeed_valueChanged(const QString &);
    void on_setSpeed_valueChanged(double);
    //void on_setSpeed_valueChanged(const QString &);
    
    /******************************************
    ** Manual connections
    *******************************************/
    //void updateLoggingView(); // no idea why this can't connect automatically
    void updatePlots();
    void updateDepth(double value);
    void updateYaw(double value);
    

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    QwtPlotCurve *curve1;
    QwtPlotCurve *curve2;
    QwtPointSeriesData error_depth_data;
    QwtPointSeriesData error_yaw_data;
    QVector<QPointF> error_depth_vec;
    QVector<QPointF> error_yaw_vec;
    int count_depth,count_yaw;
    bool yaw_plot;
    bool depth_plot;
};

}  // namespace controller_gui

#endif // controller_gui_MAIN_WINDOW_H
