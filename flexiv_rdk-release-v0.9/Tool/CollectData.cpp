/**
 * @copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>
#include <Eigen/Eigen>

using namespace std;

namespace {
    int indexx = 0;
    int collectionFlag = 0;
    std::string planTimer = "0";
    std::string collectionTimer = "0";
    std::string initTimer,nodeTimer,nodeName;
    bool programRunning = false;
    std::string startNode = "Start",stopNode = "Stop",showName;

    //vector for data storage
    std::vector<std::vector<double>> excelData;
    std::vector<std::vector<std::vector<double>>> data_vector;
    std::vector<int> indexx_vector;
    std::vector<std::string> nodeName_vector;
    std::vector<std::string> planTimer_vector;

    std::vector<std::string> nodeNameHeader_vector;
    std::vector<std::string> nodeTimer_vector;
}

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP] [start node name] [stop node name]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "    start node name(optional): node name to start collecting data, if not entered is 'Start'" << std::endl;
    std::cout << "    stop node name(optional): node name to stop collecting data, if not entered is 'Stop'" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

std::vector<std::string> split(const string &str, const string &pattern)
{
    std::vector<std::string> res;
    if(str == "")
        return res;
    string strs = str + pattern;
    size_t pos = strs.find(pattern);

    while(pos != strs.npos)
    {
        string temp = strs.substr(0, pos);
        res.push_back(temp);
        strs = strs.substr(pos+1, strs.size());
        pos = strs.find(pattern);
    }
    return res;
}

void lowPriorityPeriodicTask()
{
    while (true) {
        if (planTimer == collectionTimer){
            programRunning = false;
        }
        else{
            programRunning = true;
        }
        planTimer = collectionTimer;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (programRunning == false){
            if (collectionFlag == 0){
                std::cout<<"Plan Stoped! Wait plan running..."<<std::endl;
            }
        }
        else{
            if (collectionFlag == 1){
                std::cout<<"Plan Info [ PlanTimer : "<<"\033[32m"<<planTimer<<"s" <<"\033[0m"<<"]"
                                   <<"[ NodeTimer : "<<"\033[32m"<<nodeTimer<<"s" <<"\033[0m"<<"]"
                                   <<"[ Nodename : "<<"\033[32m"<<showName<<"\033[0m"<<" ]"<<std::endl;
            }
            else if(collectionFlag == 0){
                std::cout<<"Plan running! Wait start node...[ PlanTimer : "<<"\033[32m"<<planTimer<<"s" <<"\033[0m"<<"]"<<std::endl;
            }
        }
    }
}

// read robot data and push robot data to vector
void highPriorityPeriodicTask(flexiv::Robot& robot)
{
    flexiv::RobotStates robotStates;
    flexiv::PlanInfo planInfo;

    while(planTimer == "0"){
        robot.getRobotStates(robotStates);
        robot.getPlanInfo(planInfo);
        if(!(planInfo.nodePathTimePeriod.empty())){
            planTimer = split(planInfo.nodePathTimePeriod,"::")[0];
        }
    }
    
    
    // robot.setMode(flexiv::Mode::NRT_PLAN_EXECUTION);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // robot.executePlan("Test-MainPlan");

    while (true) {
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        robot.getRobotStates(robotStates);
        robot.getPlanInfo(planInfo);

        collectionTimer = split(planInfo.nodePathTimePeriod,"::")[0];

        if (collectionFlag == 0 ){  
            if (programRunning == true){
                if (planInfo.nodeName == startNode){
                    indexx = 1;
                    std::cout<< "Start collecting data" <<std::endl;
                    collectionFlag = 1;
                    initTimer = collectionTimer;
                    nodeTimer = collectionTimer;
                    nodeName = planInfo.nodeName;
                }
            }
        }

        if (collectionFlag == 1 ){   
            if ((planInfo.nodeName == stopNode)||(programRunning == false)){
                nodeNameHeader_vector.push_back(nodeName);
                nodeTimer_vector.push_back(nodeTimer);
                collectionFlag = 2;
            }
        }

        if (collectionFlag == 1){
            showName = planInfo.nodeName;
            indexx_vector.push_back(indexx);
            indexx=indexx+1;
            planTimer_vector.push_back(to_string(stod(collectionTimer)-stod(initTimer)));
            nodeName_vector.push_back(planInfo.nodeName);
            
            if (nodeName != planInfo.nodeName){
                nodeNameHeader_vector.push_back(nodeName);
                nodeTimer_vector.push_back(to_string(stod(collectionTimer)-stod(nodeTimer)));
                nodeName = planInfo.nodeName;
                nodeTimer = collectionTimer;
            }

            excelData.clear();

            excelData.push_back(flexiv::utility::mQuat2mmDeg(robotStates.flangePose));
            excelData.push_back(flexiv::utility::mQuat2mmDeg(robotStates.tcpPose));
            excelData.push_back(flexiv::utility::ms2mms(robotStates.tcpVel));
            excelData.push_back(robotStates.ftSensorRaw);
            excelData.push_back(robotStates.extWrenchInTcp);
            excelData.push_back(robotStates.q);
            excelData.push_back(robotStates.tauExt);

            data_vector.push_back(excelData);
        }

        if (collectionFlag == 2){
            std::cout<< "Stop collecting! Writing csv file..." <<std::endl;

            time_t rawtime;
            struct tm *ptminfo;
            time(&rawtime);
            ptminfo = localtime(&rawtime);
            std::string date_str = to_string(ptminfo->tm_year + 1900)+"-" +to_string(ptminfo->tm_mon + 1)+ "-" +to_string(ptminfo->tm_mday);
            std::string time_str = to_string(ptminfo->tm_hour)+"-" +to_string(ptminfo->tm_min)+"-" +to_string(ptminfo->tm_sec);
            std::string timeExcel_str = to_string(ptminfo->tm_hour)+":" +to_string(ptminfo->tm_min)+":" +to_string(ptminfo->tm_sec);

            std::string planName = planInfo.assignedPlanName;
            std::string dataFileName = "Data_" + planName + "_" + date_str + "_" + time_str + ".csv";
            std::string timerFileName = "Timer_" + planName + "_" + date_str + ".csv";
            std::string filePath = "./RobotData/";

            if (access(filePath.c_str(), F_OK) == -1) {
                mkdir(filePath.c_str(), S_IRWXU | S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
            }

            // write data file
            std::ofstream MyExcelFile;
            std::string tmpFileName = filePath+dataFileName;
            MyExcelFile.open(tmpFileName);

            //write data file header
            MyExcelFile << "Index"<<",";
            MyExcelFile << "Time_s"<<",";
            MyExcelFile << "NodeName"<<",";
            MyExcelFile << "FLA_X"<<","<<"FLA_Y"<<"," << "FLA_Z"<<"," ;
            MyExcelFile << "FLA_Rx"<<","<<"FLA_Ry"<<"," << "FLA_Rz"<<"," ;
            MyExcelFile << "TCP_X"<<","<<"TCP_Y"<<"," << "TCP_Z"<<"," ;
            MyExcelFile << "TCP_Rx"<<","<<"TCP_Ry"<<"," << "TCP_Rz"<<"," ;
            MyExcelFile << "Vel_X"<<","<<"Vel_Y"<<"," << "Vel_Z"<<"," ;
            MyExcelFile << "Vel_Rx"<<","<<"Vel_Ry"<<"," << "Vel_Rz"<<"," ;
            MyExcelFile << "FtSensor_Fx"<<","<< "FtSensor_Fy"<<","<< "FtSensor_Fz"<<",";
            MyExcelFile << "FtSensor_Mx"<<","<< "FtSensor_My"<<","<< "FtSensor_Mz"<<",";
            MyExcelFile << "JtSensor_Fx"<<","<< "JtSensor_Fy"<<","<< "JtSensor_Fz"<<",";
            MyExcelFile << "JtSensor_Mx"<<","<< "JtSensor_My"<<","<< "JtSensor_Mz"<<",";
            MyExcelFile << "A1_Deg"<<","<< "A2_Deg"<<","<< "A3_Deg"<<","<< "A4_Deg"<<","<< "A5_Deg"<<","<< "A6_Deg"<<","<< "A7_Deg"<<",";
            MyExcelFile << "A1_Nm"<<","<< "A2_Nm"<<","<< "A3_Nm"<<","<< "A4_Nm"<<","<< "A5_Nm"<<","<< "A6_Nm"<<","<< "A7_Nm"<<",";
            MyExcelFile << std::endl; 

            // write data
            while (!data_vector.empty()){
                MyExcelFile << indexx_vector.front()<<",";
                MyExcelFile << planTimer_vector.front()<<",";
                MyExcelFile << nodeName_vector.front()<<","; 
                for (auto i = 0; i < 5; i++){
                    for(int j = 0; j<6; j++){
                        MyExcelFile << data_vector.front()[i][j]<<",";
                    }
                }
                for (auto i = 5; i < 7; i++){
                    for(int j = 0; j<7; j++){
                        MyExcelFile << data_vector.front()[i][j]<<",";
                    }
                }
                MyExcelFile << std::endl; 
                data_vector.erase(data_vector.begin());
                nodeName_vector.erase(nodeName_vector.begin());
                indexx_vector.erase(indexx_vector.begin());
                planTimer_vector.erase(planTimer_vector.begin());
            }
            MyExcelFile.close();
            
            // write timer file
            tmpFileName = filePath+timerFileName;

            // write timer file header
            if (access(tmpFileName.c_str(), F_OK) == -1) {
                MyExcelFile.open(tmpFileName);
                MyExcelFile << "Index"<<","<<"Time"<<",";
                while (!nodeNameHeader_vector.empty()){
                    MyExcelFile << nodeNameHeader_vector.front()<<",";
                    nodeNameHeader_vector.erase(nodeNameHeader_vector.begin());
                }
                MyExcelFile << std::endl;
                MyExcelFile.close();
            }
            
            // caclulate timer index
            int timerIndex = 0;
            std::string tmp;
            ifstream readFile;
            readFile.open(tmpFileName,ios::in);
            while (getline(readFile,tmp,'\n')){
                timerIndex++;
            }
            readFile.close();

            // write timer
            MyExcelFile.open(tmpFileName,ios::app);
            MyExcelFile << timerIndex <<","<< timeExcel_str <<",";
            while (!nodeTimer_vector.empty()){
                    MyExcelFile << nodeTimer_vector.front()<<",";
                    nodeTimer_vector.erase(nodeTimer_vector.begin());
                }
            MyExcelFile << std::endl;
            MyExcelFile.close();

            // write done
            std::cout<< "Write csv file done! File name is : "<< tmpFileName <<std::endl;
            collectionFlag = 0;
        }
    }
}


int main(int argc, char* argv[])
{

    flexiv::Log log;
    std::string cinStr;

    if (argc < 3 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }
    // IP of the robot server
        std::string robotIP = argv[1];
        std::string localIP = argv[2];

    try {
        flexiv::Robot robot(robotIP, localIP);

        if (argc > 3){
            startNode = argv[3];
        }
        if (argc > 4 ){
            stopNode = argv[4];
        }
        std::cout<<std::endl;
        std::cout<<"======================================="<<std::endl;
        std::cout<<"Collection start node name is: "<<"\033[32m"<<startNode<<"\033[0m"<<std::endl;
        std::cout<<"Collection stop node name is: "<<"\033[32m"<<stopNode<<"\033[0m"<<std::endl;
        std::cout<<"======================================="<<std::endl;
        std::cout<<"Start collecting now ? y/n: ";
        cinStr = std::cin.get();

        if (cinStr=="n"){
            std::cout<<"Start node name: ";
            std::cin>>startNode;
            std::cout<<"Stop node name: ";
            std::cin>>stopNode;
            std::cout<<"======================================="<<std::endl;
            std::cout<<"Collection start node name is: "<<startNode<<std::endl;
            std::cout<<"Collection stop node name is: "<<stopNode<<std::endl;
            std::cout<<"======================================="<<std::endl;

            std::cin.get();
            std::cout<<"Start collecting now ? y/n: ";
            cinStr = std::cin.get();

            if (cinStr=="n"){
                return 0;
            }
        }
        std::cout<<"======================================="<<std::endl;

        
        std::thread highPriorityThread(std::bind(highPriorityPeriodicTask, std::ref(robot)));
        std::thread lowPriorityThread(std::bind(lowPriorityPeriodicTask));
        lowPriorityThread.join();
        highPriorityThread.join();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
        }

    return 0;

}
