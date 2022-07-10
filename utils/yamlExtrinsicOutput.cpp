/**
* @file        :yamlExtrinsicOutput.cpp
* @brief       :本程序用于曾总提供的3d仿真数据转换为徐工的外参形式（honmogenous matrix）yaml输出
* @details     :由matlab解析的csv文件被OpenCV输出为yaml配置文件
* @date        :2022/06/14 16:11:17
* @author      :cuixingxing(cuixingxing150@gmail.com)
* @version     :1.0
*
* @copyright Copyright (c) 2022
*
*/

#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

/**
* @brief       This is a brief description
* @details     This is the detail description.
* @param[in]   csvFile 读取csv文件中的矩阵数据，比如E:\AllDataAndModels\underParkingLotImages20220606\cameraExtrinsics.csv
* @param[out]  输出csv中的二维矩阵.
* @return      返回值
* @retval      返回值类型
* @par 标识符
*     保留
* @par 其它
*
* @par 修改日志
*      cuixingxing于2022/06/14创建
*/
cv::Mat readCSV(string csvFile) {
    ifstream inFile(csvFile, ios::in);
    if (!inFile) {
        cout << "打开文件失败！" << endl;
        exit(1);
    }
    vector<vector<double> > all_data;
    string lineStr;
    while (getline(inFile, lineStr)) {
        // Now inside each line we need to seperate the cols
        vector<double> values;
        stringstream temp(lineStr);
        string single_value;
        while (getline(temp, single_value, ',')) {
            // convert the string element to a integer value
            values.push_back(atof(single_value.c_str()));
        }
        // add the row to the complete data vector
        all_data.push_back(values);
    }

    // Now add all the data into a Mat element
    Mat vect = Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_64FC1);
    // Loop over vectors and add the data
    for (int row = 0; row < (int)all_data.size(); row++) {
        for (int cols = 0; cols < (int)all_data[0].size(); cols++) {
            vect.at<double>(row, cols) = all_data[row][cols];
        }
    }
    return vect;
}

int main() {
    //  相机系数
    string calibration_time = "06/13/22 14:12:21";
    int image_width = 1920;
    int image_height = 1080;
    cv::Mat cameraIntrinsics = (cv::Mat_<double>(3, 3) << 1046, 0, image_width / 2 - 1, 0, 1046, image_height / 2 - 1, 0, 0, 1);
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0;  // opencv 格式
    cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 4) << k1, k2, p1, p2);

    string cameraExtricsFile = "E:\\AllDataAndModels\\underParkingLotImages20220606\\cameraExtrinsics.csv";
    cv::Mat extrinsicMats = readCSV(cameraExtricsFile);
    int image_count = extrinsicMats.rows;
    FileStorage fs("cam_OpenCV.yml", cv::FileStorage::WRITE);
    fs.write("calibration_time", calibration_time);
    fs.write("image_count", image_count);
    fs.write("image_width", image_width);
    fs.write("image_height", image_height);
    fs.write("camera_matrix", cameraIntrinsics);
    fs.write("distortion_coefficients", distortion_coefficients);
    fs.write("extrinsic_parameters", extrinsicMats);  // 不能够写成4列，filestorage换行仍然是个问题！
    fs.release();

    // 记得手动复制粘贴到extrinsic_parameters的data域下覆盖
    ofstream fid("cam_OpenCV.yml", ios::app);
    // fs << "extrinsic_parameters"
    //    << "[:"; //
    double* dataOri = extrinsicMats.ptr<double>(0);
    for (int ind = 0; ind < extrinsicMats.rows * extrinsicMats.cols; ind++) {
        if ((ind % 4) == 0) {
            fid << endl
                << "      ";
        }
        fid << dataOri[ind] << ",";
    }
    fid.close();

    return 0;
}