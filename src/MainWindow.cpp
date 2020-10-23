#include "MainWindow.h"

MainWindow::MainWindow(ros::NodeHandle *node, QWidget *parent) : BaseWindow(node, parent) {
    //系统变量初始化
    SysVarInit();
    //初始化rostopic
    initRosToptic();
    //信号与槽绑定
    signalAndSlot();
    //初始化状态显示
    initUiStatus();
}

MainWindow::~MainWindow() {

}


void MainWindow::SysVarInit() {
    //连接状态监控
    vector<string> devDetectorName\
    {"rbConn_Detector","rbIsWell_Detector","rbEnable_Detector","gripperConn_Detector","voiceNode_Detector",
     "d435iConn_Detector","kinect2Conn_Detector","senceFinish_Detector","pickPlaceBridge_Detector","fsmNode_Detector",
     "visionBridge_Detector","dmBridge_Detector","plannerBridge_Detector","motionBridge_Detector","perceptionBridge_Detector"
    };
    vector<vector<QLabel*>> devDetectorShowLable\
    {{label_tabmain_rbConn},{label_tabmain_rbIsWell,label_tab_stepMode_rbIsWell},{label_tabmain_rbEnable,label_tab_stepMode_rbEnable},{label_tabmain_gripperConn,label_tab_stepMode_gripperConn},{label_tabmain_voiceNode},
     {label_tabmain_d435iConn,label_tab_stepMode_rd435iConn},{label_tabmain_kinect2Conn},{label_tabmain_senceFinish,label_tab_stepMode_senceFinish},{label_tabmain_pickPlaceBridge,label_tab_stepMode_pickPlaceBridge},{label_tabmain_fsmNode},
     {label_tabmain_visionBridge},{label_tabmain_dmBridge,label_tab_stepMode_dmBridge},{label_tabmain_plannerBridge,label_tab_stepMode_plannerBridge},{label_tabmain_motionBridge,label_tab_stepMode_motionBridge},{label_tabmain_perceptionBridge,label_tab_stepMode_perceptionBridge}
    };
    for (size_t i = 0; i <devDetectorName.size(); ++i) {
        map_devDetector.insert(pair<string ,devDetector*>(devDetectorName[i],new devDetector{devDetectorName[i],0,true,false,devDetectorShowLable[i]}));
    }
//    map_devDetector["gripperConn_Detector"]->real_time= false;//非实时监控
//    map_devDetector["senceFinish_Detector"]->real_time= false;//非实时监控
    //状态机状态监控
    vector<string> fsmStateName\
    {
     "init","prepare","look","detection",
     "pick","place","error","exit",
    };
    vector<QLabel*> fsmStateShowLable\
    {
        label_tab_autoMode_init,label_tab_autoMode_prepare,label_tab_autoMode_updatePCL,label_tab_autoMode_detect,
        label_tab_autoMode_grab,label_tab_autoMode_place,label_tab_autoMode_err,label_tab_autoMode_exit
    };
    for (size_t i = 0; i <fsmStateName.size(); ++i) {
        map_fsmState.insert(pair<string ,fsmState*>(fsmStateName[i],new fsmState{fsmStateName[i],false,fsmStateShowLable[i]}));
    }
    //节点名字监控
    vector<string > nodeName\
    {
     "/voice_assistant","/pickplace_bridge","/vision_bridge",
     "/dm_bridge","/trajectory_planner","/motion_bridge","/hscfsm_bridge","/perception_bridge"
    };
    for (const auto & j : nodeName) {
        in_nodeNameList.push_back(j);
    }
    //定时器启动
    Timer_listenStatus = new QTimer(this);
    Timer_listenStatus->setInterval(1000);
    Timer_listenNodeStatus = new QTimer(this);
    Timer_listenNodeStatus->setInterval(1000);
    //初始化状态机颜色标签
    for (auto &fsmstate : map_fsmState) {
        lableShowImag(fsmstate.second->lableList_showStatus,Qt::lightGray);
    }
}

void MainWindow::initRosToptic() {

    fsmCmd_client = Node->serviceClient<hirop_msgs::taskInputCmd>("/VisualCapture_TaskServerCmd");
    RobReset_client = Node->serviceClient<hsr_rosi_device::ClearFaultSrv>("/clear_robot_fault");
    RobEnable_client = Node->serviceClient<hsr_rosi_device::SetEnableSrv>("/set_robot_enable");
    RobSetMode_client = Node->serviceClient<hsr_rosi_device::setModeSrv>("/set_mode_srv");
    // getRobotErr_client = Node->serviceClient<hirop_msgs::robotError>("getRobotErrorFaultMsg");
    openGripper_client = Node->serviceClient<hirop_msgs::openGripper>("/openGripper");
    closeGripper_client = Node->serviceClient<hirop_msgs::closeGripper>("/closeGripper");

    fsmState_subscriber=Node->subscribe<hirop_msgs::taskCmdRet>("/VisualCapture_state",1000,boost::bind(&MainWindow::callback_fsmState_subscriber,this,_1));
    robStatus_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("robot_status",1,boost::bind(&MainWindow::callback_robStatus_subscriber,this,_1));
    personImg_subcriber=Node->subscribe<sensor_msgs::Image>("/videphoto_feedback",1,boost::bind(&MainWindow::callback_peopleDetectImg_subscriber, this, _1));
    // personImg_subcriber=Node->subscribe<sensor_msgs::Image>("/usb_cam/image_raw",1,boost::bind(&MainWindow::callback_peopleDetectImg_subscriber, this, _1));
    yolo6dImagRes_subcriber=Node->subscribe<sensor_msgs::Image>("/preview_image",1,boost::bind(&MainWindow::callback_yolo6dImagRes_subcriber, this, _1));
    d435iImagRes_subcriber=Node->subscribe<sensor_msgs::Image>("/camera_base/color/image_raw",1,boost::bind(&MainWindow::callback_d435iImagRes_subcriber, this, _1));
    //kinect2_subcriber=Node->subscribe<sensor_msgs::Image>("/kinect2/qhd/image_color",1,boost::bind(&MainWindow::callback_kinect2_subscriber, this, _1));

}

void MainWindow::signalAndSlot() {
    /****信号与槽连接*******/
    //主界面
    connect(btn_tabmain_devConn,&QPushButton::clicked,this,&MainWindow::slot_btn_tabmain_devConn);
    connect(btn_tabmain_runPrepare,&QPushButton::clicked,this,&MainWindow::slot_btn_tabmain_runPrepare);
    connect(btn_tabmain_sysStop,&QPushButton::clicked,this,&MainWindow::slot_btn_tabmain_sysStop);
    connect(btn_tabmain_sysReset,&QPushButton::clicked,this,&MainWindow::slot_btn_tabmain_sysReset);
    connect(cbox_tabmain_chooseMode,SIGNAL(currentIndexChanged(int)), this, SLOT(slot_combox_chooseMode_Clicked(int)));

    //自动模式界面
    connect(btn_tab_autoMode_run,&QPushButton::clicked,this,&MainWindow::slot_btn_tab_autoMode_run);
    connect(btn_tab_autoMode_normalstop,&QPushButton::clicked,this,&MainWindow::slot_btn_tab_autoMode_normalstop);
    connect(btn_tab_autoMode_quickstop,&QPushButton::clicked,this,&MainWindow::slot_btn_tab_autoMode_quickstop);
    connect(cBox_tab_autoMode_mode,SIGNAL(currentIndexChanged(int)), this, SLOT(slot_cBox_tab_autoMode_mode_Clicked(int)));
    connect(cBox_tab_autoMode_boxmodel,SIGNAL(currentIndexChanged(int)), this, SLOT(slot_cBox_tab_autoMode_boxmodel_Clicked(int)));

    //手动模式界面
    connect(btn_tab_stepMode_goPhotoPose,&QPushButton::clicked,this,&MainWindow::slot_btn_tab_stepMode_goPhotoPose);
    connect(btn_tab_stepMode_detectAndGrab,&QPushButton::clicked,this,&MainWindow::slot_btn_tab_stepMode_detectAndGrab);
    connect(btn_tab_stepMode_goHomePose,&QPushButton::clicked,this,&MainWindow::slot_btn_tab_stepMode_goHomePose);
    connect(btn_tab_stepMode_Estop,&QPushButton::clicked,this,&MainWindow::slot_btn_tab_stepMode_Estop);

    //调试界面
    connect(btn_rbSetEnable,&QPushButton::clicked,this,&MainWindow::slot_btn_rbSetEnable);
    connect(btn_rbReset,&QPushButton::clicked,this,&MainWindow::slot_btn_rbReset);
    connect(btn_gripper_open,&QPushButton::clicked,this,&MainWindow::slot_btn_gripper_open);
    connect(btn_gripper_close,&QPushButton::clicked,this,&MainWindow::slot_btn_gripper_close);
    connect(btn_rbGoHomePose,&QPushButton::clicked,this,&MainWindow::slot_btn_rbGoHomePose);

    //日志界面
    connect(btn_tab_recoder_ouputRecorder,&QPushButton::clicked,this,&MainWindow::slot_btn_tab_recoder_ouputRecorder);
    connect(btn_tab_recoder_clearRecorder,&QPushButton::clicked,this,&MainWindow::slot_btn_tab_recoder_clearRecorder);

    qRegisterMetaType<infoLevel>("infoLevel");
    connect(this, &MainWindow::emitLightColor,this, &MainWindow::showLightColor);
    connect(this, SIGNAL(emitQmessageBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)),Qt::QueuedConnection);  //将自定义槽连接到自定义信号
    connect(this, &MainWindow::emitTextControl,this, &MainWindow::displayTextControl);
    connect(Timer_listenStatus, &QTimer::timeout, this, &MainWindow::slot_timer_updateStatus);
    connect(Timer_listenNodeStatus, &QTimer::timeout, this, &MainWindow::slot_timer_listenNodeStatus);
    Timer_listenStatus->start();
    Timer_listenNodeStatus->start();
}

void MainWindow::initUiStatus() {
    emitLightColor(map_devDetector["gripperConn_Detector"]->lableList_showStatus,"red");
    emitLightColor(map_devDetector["senceFinish_Detector"]->lableList_showStatus,"red");
    for (auto &detector : map_devDetector)
    {
        //刷新检测器标签状态
        if (detector.second->status) {
            emit emitLightColor(detector.second->lableList_showStatus, "green");
        } else {
            emit emitLightColor(detector.second->lableList_showStatus, "red");
        }
    }

}

void MainWindow::checkNodeAlive(const std::vector<std::string> &in_nodeNameList, std::vector<bool> &out_nodeIsAlive) {
    if(in_nodeNameList.size()==0)
    {
        return;
    }
    std::vector<bool > tmp_nodeIsAlive(in_nodeNameList.size());
    fill(tmp_nodeIsAlive.begin(),tmp_nodeIsAlive.end(), false);
    ros::V_string curAliveNodes;
    ros::master::getNodes(curAliveNodes);
    for (auto curNodeName :curAliveNodes)
    {
        for (size_t i = 0; i <in_nodeNameList.size(); ++i)
        {
            if(curNodeName==in_nodeNameList[i])
            {
                tmp_nodeIsAlive[i]= true;
            }
        }
    }
    out_nodeIsAlive=tmp_nodeIsAlive;
}

QImage MainWindow::cvMat2QImage(const cv::Mat &mat) {
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
//        image.setNumColors(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
        // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        return QImage();
    }

}

void MainWindow::lableShowImag(QLabel *lable, Qt::GlobalColor color) {
//    lock_showImg.lock();
    palette.setBrush(lable->backgroundRole(),QBrush(color));
    lable->setPalette(palette);
    lable->setAutoFillBackground(true);
//    lock_showImg.unlock();
}

void MainWindow::LisionRbErrInfo() {
    // hirop_msgs::robotError srv;
    // ros::ServiceClient client = Node->serviceClient<hirop_msgs::robotError>("getRobotErrorFaultMsg");
    // // getRobotErr_client.call(srv);
    // uint64_t level=srv.response.errorLevel;
    // int errorLevel=level;
    // string errorMsg=srv.response.errorMsg;
    // string isError=srv.response.isError?"true":"false";
    // string dealMsg=srv.response.dealMsg;
    // QString tmp=QString("errorLevel:%1\nerrorMsg:%2\nisError:%3\ndealMsg:%4").arg(errorLevel).arg(QString().fromStdString(errorMsg)).arg(QString().fromStdString(isError)).arg(QString().fromStdString(dealMsg));
    // if(srv.response.isError){
    //     emit emitQmessageBox(infoLevel::information,tmp);
    // }
}

void MainWindow::slot_timer_updateStatus() {
    for (auto &detector : map_devDetector)
    {
        if(detector.second->real_time)
        {
            if (detector.second->lifeNum > 0) {
                detector.second->lifeNum -= 50;
            } else {
                detector.second->lifeNum = 0;
                detector.second->status = false;
            }
            //刷新检测器标签状态
            if (detector.second->status) {
                emit emitLightColor(detector.second->lableList_showStatus, "green");
            } else {
                emit emitLightColor(detector.second->lableList_showStatus, "red");
            }
        }
    }

    // if(map_devDetector["rbIsWell_Detector"]->status== false){
    //     if(!messagebox_showOnce)
    //     {
    //         LisionRbErrInfo();
    //         messagebox_showOnce= true;
    //     }
    // } else
    // {
    //     messagebox_showOnce= false;
    // }
}

//节点状态监听
void MainWindow::slot_timer_listenNodeStatus() {
    std::vector<bool > out_nodeIsAlive;
    checkNodeAlive(in_nodeNameList,out_nodeIsAlive);

    if(out_nodeIsAlive[0]){
        map_devDetector["voiceNode_Detector"]->lifeNum=100;
        map_devDetector["voiceNode_Detector"]->status=true;
    }
    if(out_nodeIsAlive[1]){
        map_devDetector["pickPlaceBridge_Detector"]->lifeNum=100;
        map_devDetector["pickPlaceBridge_Detector"]->status=true;
    }
    if(out_nodeIsAlive[2]){
        map_devDetector["visionBridge_Detector"]->lifeNum=100;
        map_devDetector["visionBridge_Detector"]->status=true;
    }
    if(out_nodeIsAlive[3]){
        map_devDetector["dmBridge_Detector"]->lifeNum=100;
        map_devDetector["dmBridge_Detector"]->status=true;
    }
    if(out_nodeIsAlive[4]){
        map_devDetector["plannerBridge_Detector"]->lifeNum=100;
        map_devDetector["plannerBridge_Detector"]->status=true;
    }
    if(out_nodeIsAlive[5]){
        map_devDetector["motionBridge_Detector"]->lifeNum=100;
        map_devDetector["motionBridge_Detector"]->status=true;
    }
        if(out_nodeIsAlive[6]){
        map_devDetector["fsmNode_Detector"]->lifeNum=100;
        map_devDetector["fsmNode_Detector"]->status=true;
    }
    if(out_nodeIsAlive[7]){
        map_devDetector["perceptionBridge_Detector"]->lifeNum=100;
        map_devDetector["perceptionBridge_Detector"]->status=true;
    }

}

void MainWindow::showLightColor(vector<QLabel*>  label_list,string color){
    if(color=="red")
    {
        for(auto label:label_list)
        {
            lableShowImag(label,Qt::red);
        }
    } else if(color=="green")
    {
        for(auto label:label_list)
        {
            lableShowImag(label,Qt::green);
        }
    }
}
void MainWindow::showQmessageBox(infoLevel level,QString info){
    switch (level)
    {
        case infoLevel ::information:
            QMessageBox::information(this,"提示",info,QMessageBox::Ok);break;
        case infoLevel ::warning:
            QMessageBox::warning(this,"警告",info,QMessageBox::Ok);break;
    }
}

void MainWindow::callback_fsmState_subscriber(const hirop_msgs::taskCmdRet::ConstPtr msg) {
    if((msg->state=="error")&&(msg->behevior=="initing")){
        LOG("ERRINFO")->logInfoMessage(msg->message.data()->data());
        emit emitQmessageBox(infoLevel::warning,QString().fromStdString(msg->message.data()->data()));
    }

    if(msg->behevior=="initing"){
        for (auto &fsmstate : map_fsmState) {
            if(fsmstate.second->status)
            {
                lableShowImag(fsmstate.second->lableList_showStatus,Qt::red);
                fsmstate.second->status= false;
            }
        }

        for (auto &fsmstate : map_fsmState) {
            if(fsmstate.second->stateName==msg->state)
            {
                lableShowImag(fsmstate.second->lableList_showStatus,Qt::green);
                fsmstate.second->status= true;
            }
        }
    }
//    if(msg->behevior=="quiting"){
//        for (auto &fsmstate : map_fsmState) {
//            if(fsmstate.second->stateName==msg->state){
//                lableShowImag(fsmstate.second->lableList_showStatus,Qt::red);
//                fsmstate.second->status= false;
//            }
//        }
//    }
    // //如果在准备状态并且是声控模式
    // if(msg->state=="prepare"){
    //     //如果是声控模式
    //     if(cBox_tab_autoMode_mode->currentIndex()==0)
    //     {
    //         emit emitQmessageBox(infoLevel::information,QString("请说出你想要的任务，比如“抓牛奶盒”"));

    //     }
    //     //如果非声控模式
    //     if(cBox_tab_autoMode_mode->currentIndex()==1)
    //     {
    //         emit emitQmessageBox(infoLevel::information,QString("请再点击一次启动按钮"));
    //     }
    // }
}

void MainWindow::callback_robStatus_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status) {
    map_devDetector["rbConn_Detector"]->lifeNum=100;
    map_devDetector["rbConn_Detector"]->status= true;
    if(robot_status->in_error.val==0){
        map_devDetector["rbIsWell_Detector"]->lifeNum=100;
        map_devDetector["rbIsWell_Detector"]->status= true;
    } else{
        map_devDetector["rbIsWell_Detector"]->status= false;
    }
    if(robot_status->drives_powered.val==1){
        map_devDetector["rbEnable_Detector"]->lifeNum=100;
        map_devDetector["rbEnable_Detector"]->status= true;
    } else{
        map_devDetector["rbEnable_Detector"]->status= false;
    }
}

void MainWindow::callback_peopleDetectImg_subscriber(const sensor_msgs::Image::ConstPtr &msg) {
    map_devDetector["kinect2Conn_Detector"]->lifeNum=100;
    map_devDetector["kinect2Conn_Detector"]->status= true;

    QPixmap new_pixmap;
    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat mat = ptr->image;
    QImage qimage = cvMat2QImage(mat);
    QPixmap tmp_pixmap = QPixmap::fromImage(qimage);
    new_pixmap = tmp_pixmap.scaled(msg->width, msg->height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
    labeltab_autoMode_image->setPixmap(new_pixmap);
}


void MainWindow::callback_yolo6dImagRes_subcriber(const sensor_msgs::Image::ConstPtr &msg) {
    QPixmap new_pixmap;
    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat mat = ptr->image;
    QImage qimage = cvMat2QImage(mat);
    QPixmap tmp_pixmap = QPixmap::fromImage(qimage);
//    new_pixmap = tmp_pixmap.scaled(msg->width/2, msg->height/2, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
    new_pixmap = tmp_pixmap.scaled(512, 424, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
//    labeltab_autoMode_image->setPixmap(new_pixmap);
    label_tab_stepMode_showImg->setPixmap(new_pixmap);
}

void MainWindow::callback_d435iImagRes_subcriber(const sensor_msgs::Image::ConstPtr &msg) {
    map_devDetector["d435iConn_Detector"]->lifeNum=100;
    map_devDetector["d435iConn_Detector"]->status= true;
  //  d435iImagRes_subcriber.shutdown();
    //sleep(1);
    //d435iImagRes_subcriber=Node->subscribe<sensor_msgs::Image>("/camera_base/color/image_raw",1,boost::bind(&MainWindow::callback_d435iImagRes_subcriber, this, _1));


}

void MainWindow::slot_btn_tabmain_devConn() {
    if(!startUpFlag_devconn){
        startUpFlag_devconn= true;
        system("rosrun grabrb_ui bringup.sh &");
    } else
    {
        emit emitQmessageBox(infoLevel::warning,QString("请不要重复连接设备!"));
    }
}

void MainWindow::slot_btn_tabmain_runPrepare() {

    //加载场景并刷新状态

    //机器人复位并上使能
    hsr_rosi_device::ClearFaultSrv srv_clear;
    RobReset_client.call(srv_clear);
    hsr_rosi_device::SetEnableSrv srv_enable;
    srv_enable.request.enable= true;
    RobEnable_client.call(srv_enable);
    //检测夹爪连接状态

    //
}

void MainWindow::slot_btn_tabmain_sysStop() {
    hsr_rosi_device::SetEnableSrv srv;
    srv.request.enable= false;
    RobEnable_client.call(srv);
}

void MainWindow::slot_btn_tabmain_sysReset() {
//    system("rosrun openni2_tracker voice_shutdown.sh &");
//    system("rosrun openni2_tracker vision_shutdown.sh &");
    for (auto &fsmstate : map_fsmState) {
        lableShowImag(fsmstate.second->lableList_showStatus,Qt::lightGray);
    }
    std::thread t([&]{
        btn_tabmain_sysReset->setEnabled(false);
        system("rosnode kill $(rosnode list |grep -v grabrb_ui ) &");
        sleep(5);
        cout<<"5s休眠完"<<endl;
        system("kill -9 $(ps -ef | grep kinect2* | awk '{print $2}')");
	    system("kill -9 $(ps -ef | grep nodelet | awk '{print $2}')");
        system("echo y| rosrun grabrb_ui clearNode.sh");

        startUpFlag_devconn= false;
        btn_tabmain_sysReset->setEnabled(true);
    });
    t.detach();
}


void MainWindow::slot_btn_tab_autoMode_run() {

    if(map_fsmState["look"]->status||map_fsmState["detection"]->status\
    ||map_fsmState["pick"]->status||map_fsmState["place"]->status)
    {
        enable_btn_tab_autoMode= false;
    } else{
        enable_btn_tab_autoMode= true;
    }

    if(enable_btn_tab_autoMode)
    {
        hirop_msgs::taskInputCmd srv;
        srv.request.taskName="prepare";
        srv.request.behavior="starting";
        srv.request.param.resize(1);
        if(cBox_tab_autoMode_boxmodel->currentIndex()==0){
            srv.request.param[0]="wangzai1";
        }
        if(cBox_tab_autoMode_boxmodel->currentIndex()==1){
            srv.request.param[0]="milk3";
        }
        if(cBox_tab_autoMode_boxmodel->currentIndex()==2){
            srv.request.param[0]="cola1";
        }
        if(!fsmCmd_client.call(srv))
        {
            emit emitQmessageBox(infoLevel::warning,QString("状态机服务连接失败!"));
        }
    }
    else
    {
        emit emitQmessageBox(infoLevel::warning,QString("请不要重复启动"));
    }

}

void MainWindow::slot_btn_tab_autoMode_normalstop() {
    hirop_msgs::taskInputCmd srv;
    srv.request.taskName="exit";
    srv.request.behavior="restart";
    if(!fsmCmd_client.call(srv))
    {
        emit emitQmessageBox(infoLevel::warning,QString("状态机服务连接失败!"));
    }
}

void MainWindow::slot_btn_tab_autoMode_quickstop() {
    hsr_rosi_device::SetEnableSrv srv;
    srv.request.enable= false;
    RobEnable_client.call(srv);
    enable_btn_tab_autoMode=true;
}

void MainWindow::slot_btn_tab_stepMode_goPhotoPose() {
    hirop_msgs::taskInputCmd srv;
    srv.request.behavior="switch";
    srv.request.param.resize(1);
    srv.request.param[0]="1";
    if(!fsmCmd_client.call(srv))
    {
        emit emitQmessageBox(infoLevel::warning,QString("状态机服务连接失败!"));
    }
}

void MainWindow::slot_btn_tab_stepMode_detectAndGrab() {
    hirop_msgs::taskInputCmd srv;
    srv.request.behavior="switch";
    srv.request.param.resize(1);
    srv.request.param[0]="2";
    if(!fsmCmd_client.call(srv))
    {
        emit emitQmessageBox(infoLevel::warning,QString("状态机服务连接失败!"));
    }
}

void MainWindow::slot_btn_tab_stepMode_goHomePose() {
    hirop_msgs::taskInputCmd srv;
    srv.request.behavior="switch";
    srv.request.param.resize(1);
    srv.request.param[0]="3";
    if(!fsmCmd_client.call(srv))
    {
        emit emitQmessageBox(infoLevel::warning,QString("状态机服务连接失败!"));
    }
}

void MainWindow::slot_btn_tab_stepMode_Estop() {
    hsr_rosi_device::SetEnableSrv srv;
    srv.request.enable= false;
    RobEnable_client.call(srv);
}

void MainWindow::slot_btn_rbSetEnable() {
    hsr_rosi_device::SetEnableSrv srv;
    srv.request.enable= true;
    RobEnable_client.call(srv);
}

void MainWindow::slot_btn_rbReset() {
    hsr_rosi_device::ClearFaultSrv srv_clear;
    RobReset_client.call(srv_clear);
}

void MainWindow::slot_btn_gripper_open() {
    hirop_msgs::openGripper srv;
    openGripper_client.call(srv);
}

void MainWindow::slot_btn_gripper_close() {
    hirop_msgs::closeGripper srv;
    closeGripper_client.call(srv);
}

void MainWindow::slot_btn_rbGoHomePose() {

}

void MainWindow::slot_btn_tab_recoder_ouputRecorder() {
    QString file_path = QFileDialog::getOpenFileName(this,"选择文件",logPath, "Files(*.log)");
    QString displayString;
    QFile file(file_path);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return;
//        qDebug()<<"Can't open the file!"<<endl;
    }
    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        QString str(line);
        displayString.append(str);
    }
    file.close();
    plainText_tabrecorder->clear();
    plainText_tabrecorder->setPlainText(displayString);
}

void MainWindow::slot_btn_tab_recoder_clearRecorder() {
    plainText_tabrecorder->clear();
}

void MainWindow::displayTextControl(QString text) {
    plainText_tabrecorder->appendPlainText(text);
}

void MainWindow::slot_combox_chooseMode_Clicked(int index) {
//    switch (index) {
//        case 0:
//            tab_autoMode->setEnabled(false);
//            tab_stepMode->setEnabled(false);
//            break;
//        case 1:
//            tabWidget->removeTab(1);
//            tabWidget->insertTab(1,tab_autoMode, QString("自动模式界面"));
//            break;
//        case 2:
//            tabWidget->removeTab(1);
//            tabWidget->insertTab(1,tab_stepMode, QString("手动模式界面"));
//            break;
//    }
}

void MainWindow::slot_cBox_tab_autoMode_mode_Clicked(int index){
    switch (index) {
        case 0:
            cBox_tab_autoMode_mode->setVisible(true);
            cBox_tab_autoMode_boxmodel->setVisible(false);
            break;
        case 1:
            cBox_tab_autoMode_mode->setVisible(true);
            cBox_tab_autoMode_boxmodel->setVisible(true);
            break;
    }
}

void MainWindow::slot_cBox_tab_autoMode_boxmodel_Clicked(int index){

}

void MainWindow::callback_kinect2_subscriber(const sensor_msgs::Image::ConstPtr &msg) {
    map_devDetector["kinect2Conn_Detector"]->lifeNum=100;
    map_devDetector["kinect2Conn_Detector"]->status= true;
}


