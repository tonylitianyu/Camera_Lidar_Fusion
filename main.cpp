//
//  main.cpp
//  HelloCPP
//
//  Created by Tianyu Li on 7/15/18.
//  Copyright © 2018 Tianyu Li. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>
#include <vector>
#include "lidarRead.h"
#include <sys/time.h>
#include <thread>
#include <pthread.h>
using namespace std;
using namespace cv;


const double cAngle = 150.0;
const double cPixel = 1920.0;
const double lAngle = 270.0;

vector<int> stringToArray(string text){
    vector<int> distancePoints;
    
    
    std::istringstream ss(text);
    std::string token;
    int first = 0;
    while(std::getline(ss, token, ',')) {
        if (first != 0){
            distancePoints.push_back(stoi(token));
        }
        first++;
    }
    
    
    return distancePoints;
    
    
}


vector<int> findLidarDataRangeOnCamera(string points){
    
    vector<int> distancePoints = stringToArray(points);  //input point data vector;
    int numberOfPoint = 1080;
    for(int i = 0; i < numberOfPoint; i++){
        //cout << i<< "th " << distancePoints[i] << endl; //print data for each of 1080 point;
    }

    
    //calculate useful data range from Lidar for the camera since they have different angle.
    
    double pointsPerDegree = numberOfPoint/lAngle;
    
    double removeDegree = (lAngle - cAngle)/2; //needs to remove most right and most left
    double removePoints = removeDegree * pointsPerDegree;
    
    vector<int> CLFusionPoint;
    
    //decide which Lidar data point will be on the camera, filter out the data.
    for (int i = int(removePoints) + 1; i < (numberOfPoint - int(removePoints)); i++) {
        CLFusionPoint.push_back(distancePoints[i]);
        //cout << i<< "th " << distancePoints[i] << endl; //print data for each in range point.
        
    }
    
    return CLFusionPoint;
    
    
}

// image version
void processData(vector<int> data, string imageName){
    Mat image = imread("sample.png");
    
    
    if (image.empty())
    {
        cout << "Could not open or find the image" << endl;
        cin.get(); //wait for any key press
        return;
    }
    
    //we need to have the height of the lidar view in camera. we will assume a number now.
    int y = 500;
    
    double maximumDetectableDistance = 10000;
    
    
    
    for(int i = y-20; i < y+20; i++){
        for(int j = 0; j < image.cols; j++){
            double distance = double(data[data.size() - 1 - j]);
            
            Vec3b &bgrPixel = image.at<Vec3b>(i, j);
            if(distance < maximumDetectableDistance){
                
                bgrPixel[2] = 1.0 - (distance/maximumDetectableDistance);
                //cout << 1.0 - (distance/maximumDetectableDistance) << endl;
            }
        }
    }
    
    
    
    String windowName = "Fusion of Lidar and Camera";
    
    namedWindow(windowName);
    
    imshow(windowName, image); // Show our image inside the created window.
    
    waitKey(0); // Wait for any keystroke in the window
    
    destroyWindow(windowName); //destroy the created window
}

//video version
void processFrame(vector<int> data, Mat &frame){
    Mat image = frame;
    
    
    if (image.empty())
    {
        cout << "Could not open or find the image" << endl;
        cin.get(); //wait for any key press
        return;
    }
    
    //we need to have the height of the lidar view in camera. we will assume a number now.
    int y = 300;
    
    double maximumDetectableDistance = 300;
    
    double pointsPerDegree = 1080/lAngle;
    
    double pixelPerDegree = cPixel/cAngle;
    
    double pixelPerPoint = pixelPerDegree/pointsPerDegree;
    
    
    int pointCounter = 0;
    for(int i = y-20; i < y+20; i++){
        for(int j = 0; j < image.cols;){
            long currIdx = data.size() - 1 - pointCounter;
            if(currIdx >= 0){
                double distance = double(data[data.size() - 1 - pointCounter]);
                
                cout << currIdx << endl;
                
                Vec3b &bgrPixel = image.at<Vec3b>(i, j);
                if(distance < maximumDetectableDistance){
                    
                    bgrPixel[0] = 255;// * (distance/maximumDetectableDistance);
                    bgrPixel[1] = 255;
                    bgrPixel[2] = 255;
                    
                    //cout << 1.0 - (distance/maximumDetectableDistance) << endl;
                }
                
                pointCounter++;
            }
            j += 3;
        }
    }

}




void getCameraStreaming(){
    VideoCapture stream1(1);
    if(!stream1.isOpened()){
        cout << "no camera"<<endl;
        return;
    }
    
    
    
    ifstream inputFile;
    string timeStamp;
    cout << "start file" << endl;
    inputFile.open("20180724_14h10m15s_lidar_ts.txt");
    
    
    bool isFirst = true;
    while(true && inputFile >> timeStamp){
        cout << "there is camera" <<endl;
        Mat cameraFrame;
        
        stream1.read(cameraFrame);
        vector<int> dataPoints = findLidarDataRangeOnCamera(timeStamp);
        
        //put lidar data on the image
        processFrame(dataPoints, cameraFrame);
        sleep(0.025);
        
        imshow("cam", cameraFrame);
        
        if(isFirst){
            imwrite("/Users/tianyuli/Downloads/firstFrame.png", cameraFrame);
            isFirst = false;
        }
        
        if (waitKey(10) >= 0)
            break;
        
        isFirst = false;
    }
    
    return;
}





//For data logging purpose. It returns a string
//with current time to uniquely name log files
const std::string currentDateTime(){
    time_t        now = time(0);
    struct tm    tstruct;
    char        buf[80];
    tstruct = *localtime(&now);
    strftime(buf,sizeof(buf), "%Y%m%d_%Hh%Mm%Ss",&tstruct);
    return buf;
}
const std::string currentTime(){
    time_t        now = time(0);
    struct tm    tstruct;
    char        buf[80];
    tstruct = *localtime(&now);
    strftime(buf,sizeof(buf), "%Y%m%d,%H%M%S,", &tstruct);
    return buf;
}
int millis(timeval t_start)
{
    struct timeval t;
    gettimeofday(&t,NULL);
    return (t.tv_sec - t_start.tv_sec)*1000 + (t.tv_usec - t_start.tv_usec)/1000;
}
///////



void getLidarData(){
    //DATA_LOGGING
    std::string lidar_filename, intensity_filename;
    std::fstream lidar_file, intensity_file;
    
    //LIDAR
    int previous_lidar_ts = 0, current_ts = 0;
    std::vector<long> lidar_distance (1080);
    std::vector<unsigned short> lidar_intensity(1080);
    pthread_t lidar_thread;
    lidarRead::thdata lidar_data;
    lidar_data.b_loop = 0;
    lidar_data.ip_or_portname = "192.168.1.50";
    lidar_data.connection_type = "-e";
    
    //MISC
    int loop = 1;
    std::string timestamp;
    struct timeval t_start;
    
    lidar_filename.append(currentDateTime());
    intensity_filename = lidar_filename;
    lidar_filename.append("_lidar_ts.txt");
    intensity_filename.append("_intensity_ts.txt");
    lidar_file.open(lidar_filename.c_str(), std::ios_base::out);
    intensity_file.open(intensity_filename.c_str(), std::ios_base::out);
    
    
    pthread_create (&lidar_thread, NULL, &lidarRead::lidarReading, &lidar_data);
    while (!lidar_data.b_loop);
    
    
    
    
    gettimeofday(&t_start,NULL);
    
    while(true)
    {
        timestamp = currentTime() + std::to_string(millis(t_start));
        try
        {
            lidar_data.mtx.lock();
            std::copy(lidar_data.distance.begin(), lidar_data.distance.end()-1, lidar_distance.begin());
            std::copy(lidar_data.intensity.begin(), lidar_data.intensity.end(), lidar_intensity.begin());
            current_ts = lidar_data.timestamp;
            lidar_data.mtx.unlock();
        }
        catch (...) { std::cout << "\n\nCouldn't copy latest LiDAR readings...\n\n";}
        
        if(current_ts != previous_lidar_ts)
        {
            std::cout << std::endl << timestamp << "\nLidar samples:\n";
            for (int i = 0; i < lidar_distance.size(); i += lidar_distance.size()/10)
            {
                std::cout << "[" << i << "]: " << lidar_distance[i] << " | "  << lidar_intensity[i] << "\t";;
            }
            previous_lidar_ts = current_ts;
            lidar_file << timestamp;
            for (int i = 0; i < lidar_distance.size(); i++) //last one is lidar_timestamp
                lidar_file << "," << lidar_distance[i];
            lidar_file << std::endl;
            
            intensity_file << timestamp;
            for (int i = 0; i < lidar_distance.size(); i++) //last one is lidar_timestamp
                intensity_file << "," << lidar_intensity[i];
            intensity_file << std::endl;
        }
    }
    
    
    lidar_file.close();
    intensity_file.close();
    std::cout << lidar_filename << std::endl;
}





int main(int argc, const char * argv[]) {
    // insert code here...

    //getLidarData();
    getCameraStreaming();
    
    
    return 0;
}





