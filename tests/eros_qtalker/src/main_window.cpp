/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include "../include/eros_qtalker/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace eros_qtalker {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
// Example of a button slot connection
///**
// * These triggers whenever the button is clicked, regardless of whether it
// * is already checked or not.
// */
//void MainWindow::on_button_disable_arm_clicked(bool check ) {
//	if( check ) {
//		ui.button_disable_arm->setEnabled(false);
//		ui.button_enable_arm->setChecked(false);
//		ui.button_enable_arm->setEnabled(true);
//		arm_node.disable();
//	}
//}
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "eros_qtalker");
    QRect rect = settings.value("geometry", QRect(200, 200, 400, 400)).toRect();
    move(rect.topLeft());
    resize(rect.size());
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "eros_qtalker");
    settings.setValue("geometry", geometry());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	event->accept();
}

}  // namespace eros_qtalker
