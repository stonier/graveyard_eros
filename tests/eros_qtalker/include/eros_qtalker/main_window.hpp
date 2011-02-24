/**
 * @file /include/eros_qtalker/main_window.hpp
 *
 * @brief Qt based gui for eros_qtalker.
 *
 * @date November 2010
 **/
#ifndef eros_qtalker_MAIN_WINDOW_H
#define eros_qtalker_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace eros_qtalker {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function

public slots:
	// Put automatically triggered slots here (because of connectSlotsByName())
	// void on_button_enable_clicked(bool check); // example only

	void on_actionAbout_triggered();

private:
    Ui::MainWindowDesign ui;
};

}  // namespace eros_qtalker

#endif // eros_qtalker_MAIN_WINDOW_H
