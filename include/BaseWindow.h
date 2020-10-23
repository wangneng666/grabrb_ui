

#ifndef RB_UI_BASEWINDOW_H
#define RB_UI_BASEWINDOW_H
//qt库
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QWidget>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QProgressBar>
#include <QPainter>
#include <QDateTime>
#include <QDialog>
#include <QDir>
#include <QMessageBox>
#include <QProcess>
#include <QTimer>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QMutex>
#include "QDebug"
#include "qdebug.h"
#include <QFuture>
//#include <QtConcurrent>
#include <iostream>
#include <ros/node_handle.h>
#include <QGraphicsItem>
#include <QMetaType>

#include "ros/package.h"

using namespace std;

#define  BTN_W 150
#define  BTN_H 50
#define  COMBOX_W 200
#define  COMBOX_H 50
#define  LABLE_STATUS_W 50
#define  LABLE_STATUS_H 50


class BaseWindow: public QMainWindow {
public:
    BaseWindow(ros::NodeHandle* node,QWidget* parent = Q_NULLPTR);
    ~BaseWindow();

public:
    //UI流程
    void initQtVal();
    void initUi(QMainWindow *MainWindow);
    void retranslateUi(QMainWindow *MainWindow);

public:
    //ros节点
    ros::NodeHandle* Node;
    //全局变量
    QString tab_qss;
    QString groupBox_qss;
    QString photoPath;
    QString logPath;
public:
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *vLayout_main;
    QHBoxLayout *hLayout_main_11;
    QLabel *label_main_logo;
    QLabel *label_main_title;
    QHBoxLayout *hLayout_main_12;
    QTabWidget *tabWidget;
    QWidget *tab_main;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *vLayout_tabmain_1;
    QHBoxLayout *hLayout_tabmain_11;
    QGroupBox *gBox_tabmain_status;
    QHBoxLayout *horizontalLayout_5;
    QGridLayout *gLayout_tabmain_status;
    QLabel *label_tabmain_motionBridge;
    QLabel *label_tabmain_perceptionBridge;
    QLabel *label_tabmain_rbConn;
    QLabel *label_tabmain_rbIsWell;
    QLabel *label_tabmain_rbEnable;
    QLabel *label_tabmain_plannerBridge;
    QLabel *label_tabmain_dmBridge;
    QLabel *label_tabmain_gripperConn;
    QLabel *label_tabmain_voiceNode;
    QLabel *label_tabmain_d435iConn;
    QLabel *label_tabmain_kinect2Conn;
    QLabel *label_tabmain_visionBridge;
    QLabel *label_tabmain_senceFinish;
    QLabel *label_tabmain_pickPlaceBridge;
    QLabel *label_tabmain_fsmNode;
    QHBoxLayout *hLayout_tabmain_21;
    QGroupBox *gBox_tabmain_mode;
    QHBoxLayout *horizontalLayout_6;
    QComboBox *cbox_tabmain_chooseMode;
    QHBoxLayout *hLayout_tabmain_31;
    QGroupBox *gBox_tabmain_func;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *btn_tabmain_devConn;
    QPushButton *btn_tabmain_runPrepare;
    QPushButton *btn_tabmain_sysStop;
    QPushButton *btn_tabmain_sysReset;

    QWidget *tab_autoMode;
    QHBoxLayout *horizontalLayout;
    QHBoxLayout *hLayout_tab_autoMode_1;
    QVBoxLayout *vLayout_tab_autoMode_11;
    QGroupBox *gBox_tab_autoMode_img;
    QHBoxLayout *horizontalLayout_15;
    QLabel *labeltab_autoMode_image;
    QVBoxLayout *vLayout_tab_autoMode_12;
    QGroupBox *gBox_tab_autoMode_fsm;
    QHBoxLayout *horizontalLayout_17;
    QGridLayout *gLayout_tab_autoMode_fsm;
    QLabel *label_tab_autoMode_grab;
    QLabel *label_tab_autoMode_updatePCL;
    QLabel *label_tab_autoMode_detect;
    QLabel *label_tab_autoMode_prepare;
    QLabel *label_tab_autoMode_init;
    QLabel *label_tab_autoMode_exit;
    QLabel *label_tab_autoMode_err;
    QLabel *label_tab_autoMode_place;
    QGroupBox *gBox_tab_autoMode_mode;
    QVBoxLayout *verticalLayout;
    QComboBox *cBox_tab_autoMode_mode;
    QComboBox *cBox_tab_autoMode_boxmodel;
    QGroupBox *gBox_tab_autoMode_operate;
    QVBoxLayout *verticalLayout_10;
    QHBoxLayout *hLayout_tab_autoMode_123;
    QPushButton *btn_tab_autoMode_run;
    QPushButton *btn_tab_autoMode_normalstop;
    QPushButton *btn_tab_autoMode_quickstop;

    QWidget *tab_stepMode;
    QHBoxLayout *horizontalLayout_4;
    QHBoxLayout *hLayout_tab_stepMode_1;
    QVBoxLayout *vLayout_tab_stepMode_12;
    QGroupBox *gBox_tab_stepMode_showImg;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_tab_stepMode_showImg;
    QVBoxLayout *vLayout_tab_stepMode_11;
    QGroupBox *gBox_tab_stepMode_status;
    QHBoxLayout *horizontalLayout_11;
    QGridLayout *gLayout_tab_stepMode;
    QLabel *label_tab_stepMode_pickPlaceBridge;
    QLabel *label_tab_stepMode_rd435iConn;
    QLabel *label_tab_stepMode_rbIsWell;
    QLabel *label_tab_stepMode_rbEnable;
    QLabel *label_tab_stepMode_gripperConn;
    QLabel *label_tab_stepMode_senceFinish;
    QLabel *label_tab_stepMode_perceptionBridge;
    QLabel *label_tab_stepMode_dmBridge;
    QLabel *label_tab_stepMode_plannerBridge;
    QLabel *label_tab_stepMode_motionBridge;

    QGroupBox *gBox_tab_stepMode_operate;
    QHBoxLayout *horizontalLayout_12;
    QPushButton *btn_tab_stepMode_goPhotoPose;
    QPushButton *btn_tab_stepMode_detectAndGrab;
    QPushButton *btn_tab_stepMode_goHomePose;
    QPushButton *btn_tab_stepMode_Estop;
    QWidget *tab_debug;
    QVBoxLayout *verticalLayout_5;
    QGroupBox *groupBox_tabdebug_1;
    QHBoxLayout *horizontalLayout_14;
    QPushButton *btn_rbSetEnable;
    QPushButton *btn_rbReset;
    QGroupBox *groupBox_tabdebug_2;
    QHBoxLayout *horizontalLayout_19;
    QPushButton *btn_gripper_open;
    QPushButton *btn_gripper_close;
    QGroupBox *groupBox_tabdebug_3;
    QHBoxLayout *horizontalLayout_20;
    QPushButton *btn_rbGoHomePose;

    QWidget *tab_recorder;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *vLayout_tab_recorder_1;
    QVBoxLayout *vLayout_tab_recorder_11;
    QPlainTextEdit *plainText_tabrecorder;
    QVBoxLayout *vLayout_tab_recorder_2;
    QPushButton *btn_tab_recoder_ouputRecorder;
    QPushButton *btn_tab_recoder_clearRecorder;
    QStatusBar *statusBar;
};


#endif //RB_UI_BASEWINDOW_H
