#ifndef GRABRB_UI_MAINWINDOW_H
#define GRABRB_UI_MAINWINDOW_H

#include "BaseWindow.h"
#include "logmanager.h"
//opencv库
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//消息类型头文件
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "hsr_rosi_device/ClearFaultSrv.h"
#include "hsr_rosi_device/SetEnableSrv.h"
#include "industrial_msgs/RobotStatus.h"
#include "geometry_msgs/Wrench.h"
#include "hsr_rosi_device/setModeSrv.h"
#include "std_srvs/SetBool.h"
#include "hirop_msgs/taskInputCmd.h"
#include "hirop_msgs/taskCmdRet.h"
#include "hirop_msgs/robotError.h"
#include "hirop_msgs/closeGripper.h"
#include "hirop_msgs/openGripper.h"
#include <atomic>

//标准库
#include "ros/ros.h"
#include <iostream>
#include <fstream>
using namespace std;

enum  infoLevel{information,warning};

struct  devDetector{
    string name; //设备名
    int lifeNum; //生命值
    bool real_time;//实时监控与否
    bool status; //状态
    vector<QLabel*> lableList_showStatus;//状态显示lable
};

struct  fsmState{
    string stateName;
    bool status;
    QLabel* lableList_showStatus;//状态显示lable
};

class MainWindow: public BaseWindow {
Q_OBJECT
public:
    MainWindow(ros::NodeHandle* node,QWidget* parent = Q_NULLPTR);
    ~MainWindow();

private:
    map<string, devDetector*> map_devDetector;//运行准备状态监控器
    map<string, fsmState*> map_fsmState;      //状态机状态监控
    vector<string> in_nodeNameList;           //ros节点名称列表
    bool startUpFlag_devconn= false;
    QTimer* Timer_listenStatus;
    QTimer* Timer_listenNodeStatus;
    QMutex lock_showImg;
    QPalette palette;
    bool messagebox_showOnce= false;
    atomic<bool> enable_btn_tab_autoMode;

private:
    //ros消息对象
    ros::ServiceClient RobReset_client;
    ros::ServiceClient RobEnable_client;
    ros::ServiceClient RobSetMode_client;
    ros::ServiceClient fsmCmd_client;///VoiceCtlRob_TaskServerCmd
    ros::ServiceClient getRobotErr_client;
    ros::ServiceClient openGripper_client;
    ros::ServiceClient closeGripper_client;

    ros::Subscriber fsmState_subscriber;
    ros::Subscriber robStatus_subscriber;
    ros::Subscriber peopleDetectImg_subscriber;
    ros::Subscriber personImg_subcriber;
    ros::Subscriber yolo6dImagRes_subcriber;
    ros::Subscriber d435iImagRes_subcriber;
    ros::Subscriber kinect2_subcriber;

public:
    //系统变量初始化
    void SysVarInit();
    //处理所有信号和槽函数
    void signalAndSlot();
    //初始化Ros话题与服务
    void initRosToptic();
    //初始化UI界面状态
    void initUiStatus();
    //输入想要监控的节点名称列表 输出监控状态列表
    void checkNodeAlive(const std::vector<std::string>& in_nodeNameList, std::vector<bool >& out_nodeIsAlive);
    //cv::mat转Qimage
    QImage cvMat2QImage(const cv::Mat &mat) ;
    //lable显示色彩
    void lableShowImag(QLabel* lable,Qt::GlobalColor color);
    void LisionRbErrInfo();
    //
private:
    /********************按钮槽函数**************************************/
    //主界面
    void slot_btn_tabmain_devConn();
    void slot_btn_tabmain_runPrepare();
    void slot_btn_tabmain_sysStop();
    void slot_btn_tabmain_sysReset();

        //自动模式界面
    void slot_btn_tab_autoMode_run();
    void slot_btn_tab_autoMode_normalstop();
    void slot_btn_tab_autoMode_quickstop();
    //手动模式界面
    void slot_btn_tab_stepMode_goPhotoPose();
    void slot_btn_tab_stepMode_detectAndGrab();
    void slot_btn_tab_stepMode_goHomePose();
    void slot_btn_tab_stepMode_Estop();
    //调试界面
    void slot_btn_rbSetEnable();
    void slot_btn_rbReset();
    void slot_btn_gripper_open();
    void slot_btn_gripper_close();
    void slot_btn_rbGoHomePose();
    //日志界面
    void slot_btn_tab_recoder_ouputRecorder();
    void slot_btn_tab_recoder_clearRecorder();

    void slot_timer_updateStatus();
    void slot_timer_listenNodeStatus();
//*********************************************************************************
    //ros节点回调函数
    void callback_robStatus_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status);
    void callback_peopleDetectImg_subscriber(const sensor_msgs::Image::ConstPtr& msg);
    void callback_fsmState_subscriber(const hirop_msgs::taskCmdRet::ConstPtr msg);
    void callback_yolo6dImagRes_subcriber(const sensor_msgs::Image::ConstPtr& msg);
    void callback_d435iImagRes_subcriber(const sensor_msgs::Image::ConstPtr& msg);
    void callback_kinect2_subscriber(const sensor_msgs::Image::ConstPtr& msg);

signals:
    void emitLightColor(vector<QLabel*> label_list,string color);
    void emitQmessageBox(infoLevel level,QString info);
    void emitTextControl(QString text) const;


private slots:
    void displayTextControl(QString text);
    void showLightColor(vector<QLabel*>  label_list,string color);
    void showQmessageBox(infoLevel level,QString info);
    void slot_combox_chooseMode_Clicked(int index);
    void slot_cBox_tab_autoMode_mode_Clicked(int index);
    void slot_cBox_tab_autoMode_boxmodel_Clicked(int index);
};



#endif //GRABRB_UI_MAINWINDOW_H
