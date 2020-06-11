#include <iostream>
#include <vector>
#include <boost/filesystem.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <math.h>
#include <chrono>
#include <random>

using namespace std;
namespace fs = boost::filesystem;

template <class T>
std::string IntToStr(T n);

std::vector<cv::Mat> diff_depth(std::vector<cv::Mat> images, int reference);
int select_reference(std::vector<cv::Mat> images, std::vector<string> imageNames);
int select_reference_min(std::vector<cv::Mat> images, std::vector<string> imageNames);
std::vector<cv::Mat> interference_remove(std::vector<cv::Mat> images, std::vector<cv::Mat> interferenceROI);
cv::Mat medianImages(std::vector<cv::Mat> images,std::vector<cv::Mat> uninterferenceROI);
pcl::PointCloud<pcl::PointXYZRGB> generatePointCloud(cv::Mat medianImage, string, bool withNaN, Eigen::Matrix4d transformCLoud);
float medianPixel(std::vector<float> values);
cv::Mat medianRowsCols(cv::Mat image, int frameBufferSize, Eigen::Matrix4d transform_matrix);
float excludeExtreme(cv::Mat image,int i,bool type);
cv::Mat interpolation(cv::Mat image, cv::Mat binary);
cv::Mat repairInputData(cv::Mat image, cv::Mat interfROI, cv::Mat median, int count);
void saveLUT(cv::Mat depth, std::string path);
cv::Mat generateIslands(cv::Mat image);
pcl::PointCloud<pcl::PointXYZRGB> transformCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, Eigen::Matrix4d transformCLoud);


string path;
float percentage;
float replace_thresh;
int rgbRows=0;
cv::Mat reference_rgb;
cv::Mat _importanceMap, _interferenceBinary;
//cv::Rect myROI(0, 0, 512, 424);
cv::Rect myROI(0, 0, 512, 424);
//cv::Rect myROI(0, 0, 512, 424);



//cv::Rect removeGB(0, 305, 512, 94); //cam0
//cv::Rect removeGB(0, 255, 512, 1); //cam1
//cv::Rect removeGB(0, 240, 512, 94); //lopta
//cv::Rect removeGB(0, 265, 512, 94); //plysak

//cv::Rect removeGB(0, 335, 512, 94); //druhaseria_2
//cv::Rect removeGB(0, 308, 512, 94); //druhaseria_1
cv::Rect removeGB(0, 240, 512, 94); //druhaseria_0

pcl::PointCloud<pcl::PointXYZRGB> medianCloud;
pcl::PointCloud<pcl::PointXYZRGB> original_merge;
Eigen::Matrix4d transform_matrix_left;
Eigen::Matrix4d transform_matrix_right;
Eigen::Matrix4d transform_matrix_central;



int main(int argc, char **argv)
{

    transform_matrix_central (0,0) = 1;
    transform_matrix_central (0,1) = 0;
    transform_matrix_central (0,2) = 0;
    transform_matrix_central (0,3) = 0;
    transform_matrix_central (1,0) = 0;
    transform_matrix_central (1,1) = 1;
    transform_matrix_central (1,2) = 0;
    transform_matrix_central (1,3) = 0;
    transform_matrix_central (2,0) = 0;
    transform_matrix_central (2,1) = 0;
    transform_matrix_central (2,2) = 1;
    transform_matrix_central (2,3) = 0;
    transform_matrix_central (3,0) = 0;
    transform_matrix_central (3,1) = 0;
    transform_matrix_central (3,2) = 0;
    transform_matrix_central (3,3) = 0;

    transform_matrix_left (0,0) = 0.02;
    transform_matrix_left (0,1) = -0.10;
    transform_matrix_left (0,2) = 0.99;
    transform_matrix_left (0,3) = -0.749;
    transform_matrix_left (1,0) = 0.15;
    transform_matrix_left (1,1) = 0.98;
    transform_matrix_left (1,2) = -0.098;
    transform_matrix_left (1,3) = -0.095;
    transform_matrix_left (2,0) = 0.999685378252994;
    transform_matrix_left (2,1) = -0.095;
    transform_matrix_left (2,2) = -0.99;
    transform_matrix_left (2,3) = 0.08;
    transform_matrix_left (3,0) = 0.04;
    transform_matrix_left (3,1) = 0.62;
    transform_matrix_left (3,2) = 0;
    transform_matrix_left (3,3) = 0;

    transform_matrix_right (0,0) = 1;
    transform_matrix_right (0,1) = 0;
    transform_matrix_right (0,2) = 0;
    transform_matrix_right (0,3) = 0;
    transform_matrix_right (1,0) = 0;
    transform_matrix_right (1,1) = 1;
    transform_matrix_right (1,2) = 0;
    transform_matrix_right (1,3) = 0;
    transform_matrix_right (2,0) = 0;
    transform_matrix_right (2,1) = 0;
    transform_matrix_right (2,2) = 1;
    transform_matrix_right (2,3) = 0;
    transform_matrix_right (3,0) = 0;
    transform_matrix_right (3,1) = 0;
    transform_matrix_right (3,2) = 0;
    transform_matrix_right (3,3) = 0;

    Eigen::Matrix4d transform_matrix;


    std::chrono::system_clock::time_point then, now;

    path=argv[1];
    reference_rgb=cv::imread(path+argv[2],cv::IMREAD_COLOR);
//    reference_rgb=reference_rgb(myROI);
    cv::imwrite(path + "/data/ref"+ ".png",reference_rgb);
    cv::Mat image=cv::Mat::zeros(reference_rgb.rows,reference_rgb.cols,CV_32FC1);
    rgbRows=reference_rgb.rows;
    percentage=std::stof(argv[3]);
    replace_thresh=std::stof(argv[4]);
    std::vector<cv::Mat> images;
    std::vector<string> imageNames;
    string ext{".txt"};
    int frameBufferSize=0;  
    std::string object = path;

    if(path.at(path.size()-2) == 1){
        transform_matrix=transform_matrix_central;
    } else if (path.at(path.size()-2) == 0) {
        transform_matrix=transform_matrix_right;
    } else if (path.at(path.size()-2) == 2) {
        transform_matrix=transform_matrix_left;
    }

    auto camera = path.at(path.size()-2);
    cout<<camera<<std::endl;
    std::string delimiter = "/";
    std::string token = object.substr(5, object.find(delimiter));
    size_t pos = 0;
    std::vector<std::string> saveName;
    while ((pos = object.find(delimiter)) != std::string::npos) {
        token = object.substr(0, pos);
        saveName.push_back(token);
        object.erase(0, pos + delimiter.length());
    }


    std::string direct = path + "data";
    boost::filesystem::path dir(direct);

    boost::filesystem::remove_all(direct);
    if(boost::filesystem::create_directory(dir))
    {
        std::cerr<< "Directory Created: "<<direct<<std::endl;
    }

    for(auto & p : fs::directory_iterator( path ))
    {
        if(frameBufferSize==5){
            break;
        } else {
            if(p.path().extension() == ext)
            {


                ifstream depthTxt;
                depthTxt.open(p.path().string());
                std::cout<<p.path().string()<<std::endl;
                string line;
                double counter;
                int rows=0;
                int cols=0;
                while(getline(depthTxt,line))
                {
                    if(rows==424)
                    {
                        cols++;
                        rows=0;
                    }
                    float pixel_value= std::stof(line);
                    image.at<float>(rows, cols, 0) = pixel_value;
                    rows++;
                }
                depthTxt.close();

    //            std::cout << p << std::endl;After thresholding
    //            cv::Mat image=cv::imread(p.path().string(),cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
    //            cv::Rect myROI(255, 125, 100, 175);
    //            cv::Rect myROI(170, 65, 90, 175);
    //            cvtColor(image, image, cv::COLOR_BGR2GRAY);
//                image = generateIslands(image);


    //            cv::Mat image=cv::Mat::zeros(reference_rgb.rows,reference_rgb.cols,CV_32FC1);


    //            image=image(myROI);
    //            (void)generatePointCloud(image,path+"/data/original"+IntToStr(frameBufferSize), true);

                cv::Mat tmpBinary;
                cv::Mat tmpDepth;
                cv::normalize(image, tmpDepth, 0, 255,cv::NORM_MINMAX);
                cv::imwrite(path + "/data/origos_" + p.path().stem().string() + ".png",tmpDepth(myROI));

                cv::inRange(image,10,900,tmpBinary);


                cv::Mat tmpImage;

                image.copyTo(tmpImage,tmpBinary);
                original_merge+=generatePointCloud(tmpImage,path+"/data/original"+p.path().stem().string(), false, transform_matrix);

                images.emplace_back(tmpImage(myROI));
                imageNames.emplace_back(p.path().stem().string());

                cv::normalize(tmpImage, tmpDepth, 0, 255,cv::NORM_MINMAX);
                cv::imwrite(path + "/data/original"+ p.path().stem().string() + ".png",tmpDepth);
                frameBufferSize++;
            }
        }
    }

    pcl::io::savePLYFileASCII("/media/jozefvolak/Maxtor/DIZERTACKA/pre_kuba/inter/" + saveName[5] + "_" + saveName[7] + ".ply",original_merge);

    then=std::chrono::system_clock::now();

//    auto reference = select_reference_min(images, imageNames);
    auto reference = select_reference(images, imageNames);
    auto interferenceROI = diff_depth(images, reference);
    auto uninterferenceROI = interference_remove(images,interferenceROI);

    auto medianImage = medianImages(images,images);


    auto filtered = medianRowsCols(medianImage,frameBufferSize, transform_matrix);
    auto interpolated = interpolation(medianImage, _interferenceBinary);

//    cv::imwrite(path + "/data/interp_mask.hdr",interpolated);

    interpolated = interpolated+filtered;

    cv::Mat binary_median;
    cv::inRange(interpolated,1,1000,binary_median);
    cv::imwrite(path + "/data/median_bin.png",binary_median);

    medianCloud = generatePointCloud(interpolated,path+"/data/median", false,transform_matrix);



        //    cv::normalize(interpolated, interpolated, 0, 255,cv::NORM_MINMAX);
//    cv::imwrite(path + "/data/median.hdr",interpolated);
//    saveLUT(interpolated,path);

    int i=0;
    pcl::PointCloud<pcl::PointXYZRGB> merged_clouds_viewer;
    for(auto image : images)
    {
        pcl::PointCloud<pcl::PointXYZRGB> tmp;
        repairInputData(image,_interferenceBinary,interpolated,i);



//        cv::Mat tmpUnfilterMask, notBin;
//        cv::bitwise_not(_interferenceBinary,notBin);
//        image.copyTo(tmpUnfilterMask, notBin);
         tmp = generatePointCloud(image,path+"/data/uninterfer_"+imageNames[i],false,transform_matrix);
         merged_clouds_viewer += tmp;
         i++;
    }
    pcl::io::savePLYFileASCII("/media/jozefvolak/Maxtor/DIZERTACKA/pre_kuba/filter/imbm/" + saveName[5] + "_" + saveName[7] + ".ply" ,merged_clouds_viewer);


    now=std::chrono::system_clock::now();
    std::cout <<std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() << " ms" << std::endl;

//    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//    viewer.showCloud(cloud);
//    while (!viewer.wasStopped ())
//    {
//    }
}

int select_reference(std::vector<cv::Mat> images, std::vector<string> imageNames){
    //how many pixels are non zero. select image with biggest ROI
    int nonZero[images.size()];
    int interator=0;
    for(auto & image : images){
        nonZero[interator]=cv::countNonZero(image);
//        std::cout<<nonZero[interator]<<" "<<imageNames[interator]<<std::endl;
        interator++;
    }
    int N = sizeof(nonZero) / sizeof(int);
    int position = distance(nonZero, max_element(nonZero, nonZero + (sizeof(nonZero) / sizeof(int))));
//    std::cout<<"Reference: " << nonZero[position]<<" "<<imageNames[position]<<std::endl;
    return position;
}

int select_reference_min(std::vector<cv::Mat> images, std::vector<string> imageNames){
    //select image, where the ROI is minimal
    // count of zero pixels in image
    int nonZero[images.size()];
    int interator=0;
    for(auto & image : images){
        nonZero[interator]=(image.cols*image.rows)-cv::countNonZero(image);
//        std::cout<<nonZero[interator]<<" "<<imageNames[interator]<<std::endl;
        interator++;
    }
    int N = sizeof(nonZero) / sizeof(int);
    int position = distance(nonZero, max_element(nonZero, nonZero + (sizeof(nonZero) / sizeof(int))));
//    std::cout<<"Reference: " << nonZero[position]<<" "<<imageNames[position]<<std::endl;
    return position;
}

std::vector<cv::Mat> diff_depth(std::vector<cv::Mat> images, int reference){
    int i=0;
    std::vector<cv::Mat> interferenceROI;
    auto referenceImage=images[reference];
    for(auto & image : images){
        cv::Mat diffImage = cv::Mat::zeros(referenceImage.rows,referenceImage.cols,referenceImage.type());
        cv::absdiff(referenceImage,image,diffImage);
        cv::Mat tmp;
        cv::normalize(diffImage, tmp, 0, 255,cv::NORM_MINMAX);
        cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,cv::Size( 1*1 + 1, 1*1+1 ),cv::Point( 1, 1 ) );
        cv::dilate( tmp, tmp, element );
//        cv::imwrite(path + "/data/interferenceROI"+IntToStr(i) + ".png",tmp);
        interferenceROI.emplace_back(diffImage);
        i++;
    }
    return interferenceROI;
}

std::vector<cv::Mat> interference_remove(std::vector<cv::Mat> images,std::vector<cv::Mat> interferenceROI){
    int i=0;
    std::vector<cv::Mat> uninterferenceROI;
    for(auto & image : images){
        cv::Mat tmpBinary;
        cv::threshold(interferenceROI[i], tmpBinary, 100, 255, cv::THRESH_BINARY_INV);
        cv::Mat tmp;
        cv::Mat tmpDepth;
        tmpBinary.convertTo(tmpBinary, CV_8U);
//        cv::imwrite(path +"/data/binary"+IntToStr(i) + ".png",interferenceROI[i]);
        image.copyTo(tmpDepth, tmpBinary);
        cv::normalize(tmpDepth, tmp, 0, 255,cv::NORM_MINMAX);
        cv::imwrite(path +"/data/uninterferenceROI"+IntToStr(i) + ".png",tmp);
        uninterferenceROI.emplace_back(tmpDepth);
        i++;
    }
    return uninterferenceROI;
}

cv::Mat medianImages(std::vector<cv::Mat> images,std::vector<cv::Mat> uninterferenceROI){
    cv::Mat medianImage = cv::Mat::zeros(uninterferenceROI[0].rows,uninterferenceROI[0].cols,uninterferenceROI[0].type());
    cv::Mat importanceMap = cv::Mat::zeros(uninterferenceROI[0].rows,uninterferenceROI[0].cols,CV_8U);
    for(int i=0; i<uninterferenceROI[0].cols;i++){  //riadky
        for(int j=0; j<uninterferenceROI[0].rows;j++){ //stlpce
            std::vector<float> values;
            for(auto & image : uninterferenceROI){ //image
                if(image.at<float>(j,i)>1) {
                    values.emplace_back(image.at<float>(j,i));
                }
            }
            if(values.size()>1) {
                importanceMap.at<uchar>(j, i)=static_cast<uint>(values.size());
                sort(values.begin(), values.end());
                if(values.size() % 2 !=0)
                    medianImage.at<float>(j, i)=values[(values.size()-1)/2];
                else
                    medianImage.at<float>(j, i)=(values[floor((values.size()-1)/2)] + values[ceil((values.size()-1)/2)])/2;
            }
        }
    }
    importanceMap.copyTo(_importanceMap);

    cv::Mat tmp;
    cv::normalize(importanceMap, tmp, 0, 255,cv::NORM_MINMAX);
//    cv::imwrite(path+"/data/importanceMap.png",tmp);
    cv::normalize(medianImage, tmp, 0, 255,cv::NORM_MINMAX);
//    cv::imwrite(path+"/data/median.png",tmp);
    return medianImage;
}

cv::Mat medianRowsCols(cv::Mat image, int frameBufferSize, Eigen::Matrix4d transform_matrix){
    struct median_value{
        float value;
        int position;
    };
    cv::Mat filtered;
    std::vector<median_value> col_median;
    std::vector<median_value> row_median;

    for(int i=0; i<image.cols;i++){  //riadky
        median_value value;
        value.value = excludeExtreme(image,i,true);
        value.position = i;
        col_median.emplace_back(value);
    }
    for(int j=0; j<image.rows;j++){
        median_value value;
        value.value = excludeExtreme(image,j,false);
        value.position = j;
        row_median.emplace_back(value);
    }

    cv::Mat tmpBinary, tmpImage;
    cv::inRange(_importanceMap,1,round(frameBufferSize*percentage),_interferenceBinary);
//    cv::imwrite(path+"/data/binary.png",_interferenceBinary);
    cv::bitwise_not(_interferenceBinary,tmpBinary);
//    cv::imwrite(path+"/data/notBinary.png",tmpBinary);
    image.copyTo(filtered,tmpBinary);
    auto cloud = generatePointCloud(filtered,path+"/data/filtered", false,transform_matrix);

//    for(int i=0;i<image.cols;i++){
//        for(int j=0;j<image.rows;j++){
//            auto value=image.at<float>(j,i);
//            std::cout<<i<<" "<<j<<" "<<col_median[i].value<<" "<<row_median[j].value<<" "<<value<<std::endl;
//            if(col_median[i].value-value>15 || row_median[i].value-value>15 ) {
//                filtered.at<float>(j,i)= 0;
//            }
//        }
//    }

//    for (auto median : col_median) {
//        std::cout<<median.value<<" ";
//    }
//    std::cout<<std::endl;

//    for (auto median : row_median) {
//        std::cout<<median.value<<" ";
//    }
//    std::cout<<std::endl;
    cv::imwrite(path+"/data/filtered.png",filtered);
    return filtered;
}

float excludeExtreme(cv::Mat image,int x,bool type){
    std::vector<float> values;
    if(type==false){
        for(int i=0; i<image.rows;i++){
            if(image.at<float>(x,i)>1) {
                values.emplace_back(image.at<float>(x,i));
            }
        }
    } else {
        for(int i=0; i<image.cols;i++){
            if(image.at<float>(i,x)>1) {
                values.emplace_back(image.at<float>(i,x));
            }
        }
    }
    if(values.size()>1)
        return medianPixel(values);
    else
        return 0;
}

float medianPixel(std::vector<float> values){
    sort(values.begin(), values.end());

    if(values.size() % 2 !=0)
        return values[(values.size()-1)/2];
    else
        return (values[floor((values.size()-1)/2)] + values[ceil((values.size()-1)/2)])/2;
}


cv::Mat interpolation(cv::Mat image, cv::Mat binary){

    struct pixelInfo{
        float value;
        cv::Point point;
    };

    std::vector<std::vector<pixelInfo>> points;
    std::vector<pixelInfo> vektorik;

    cv::Mat tmpImageHorizontal = cv::Mat::zeros(image.rows,image.cols,image.type());
    cv::Mat tmpImageVertical = cv::Mat::zeros(image.rows,image.cols,image.type());
    cv::Mat tmpImage = cv::Mat::zeros(image.rows,image.cols,image.type());

    for(int i=0;i<image.rows;i++){
        for(int j=0;j<image.cols;j++){
            if(binary.at<uchar>(i,j)==255){
                pixelInfo data;
                data.value=255;
                data.point.x=j;
                data.point.y=i;
                vektorik.push_back(data);

                if(binary.at<uchar>(i,j+1)==0){
                    points.push_back(vektorik);
                    vektorik.clear();
                }
            }
        }
    }
    std::vector<std::vector<pixelInfo>> interPoints;
    auto size = points.size();
    for(int i=0; i<size; i++){
        auto leftPositionValue = image.at<float>(points[i].front().point.y,points[i].front().point.x-1);
        auto rightPositionValue = image.at<float>(points[i].back().point.y,points[i].back().point.x+1);
        if((int)leftPositionValue!=0 && (int)rightPositionValue!=0) {
            interPoints.push_back(points[i]);
        }
    }
    for(int i=0; i<interPoints.size();i++){
        auto leftPositionValue = image.at<float>(interPoints[i].front().point.y,interPoints[i].front().point.x-1);
        auto rightPositionValue = image.at<float>(interPoints[i].back().point.y,interPoints[i].back().point.x+1);
        auto differenceValue = leftPositionValue - rightPositionValue;
        auto diffX = interPoints[i].back().point.x-interPoints[i].front().point.x+1;
        auto accValue = differenceValue/diffX;
//        std::cout<<interPoints[i].front().point.x<<" "<< interPoints[i].front().point.y<<" "<< interPoints[i].back().point.x<<" "<< interPoints[i].back().point.y <<" "
//                <<leftPositionValue<<" "<<rightPositionValue<<" "<< differenceValue <<" "<<" "<<diffX<<" "<<accValue<<std::endl;
        int k=1;
        for(auto point : interPoints[i]){
            tmpImageHorizontal.at<float>(point.point.y,point.point.x)=round(leftPositionValue-accValue*k);
            k++;
        }
    }
//    cv::imwrite(path+"/data/horizontal.png",tmpImageHorizontal);


    points.clear();
    vektorik.clear();
    interPoints.clear();

    for(int i=0;i<image.cols;i++){
        for(int j=0;j<image.rows;j++){
            if(binary.at<uchar>(j,i)==255){
                pixelInfo data;
                data.value=255;
                data.point.x=i;
                data.point.y=j;
                vektorik.push_back(data);

                if(binary.at<uchar>(j+1,i)==0){
                    points.push_back(vektorik);
                    vektorik.clear();
                }
            }
        }
    }


    size = points.size();
    for(int i=0; i<size; i++){
        auto leftPositionValue = image.at<float>(points[i].front().point.y-1,points[i].front().point.x);
        auto rightPositionValue = image.at<float>(points[i].back().point.y+1,points[i].back().point.x);
        if((int)leftPositionValue!=0 && (int)rightPositionValue!=0) {
            interPoints.push_back(points[i]);
        }
    }

    for(int i=0; i<interPoints.size();i++){
        auto leftPositionValue = image.at<float>(interPoints[i].front().point.y-1,interPoints[i].front().point.x);
        auto rightPositionValue = image.at<float>(interPoints[i].back().point.y+1,interPoints[i].back().point.x);
        auto differenceValue = leftPositionValue - rightPositionValue;
        auto diffY = interPoints[i].back().point.y-interPoints[i].front().point.y+1;
        auto accValue = differenceValue/diffY;
//        std::cout<<interPoints[i].front().point.x<<" "<< interPoints[i].front().point.y<<" "<< interPoints[i].back().point.x<<" "<< interPoints[i].back().point.y <<" "
//                <<leftPositionValue<<" "<<rightPositionValue<<" "<< differenceValue <<" "<<" "<<diffX<<" "<<accValue<<std::endl;
        int k=1;
        if(accValue<10){
            for(auto point : interPoints[i]){
                tmpImageVertical.at<float>(point.point.y,point.point.x)=round(leftPositionValue-accValue*k);
                k++;
            }
        }
    }

    cv::imwrite(path+"/data/vertical.png",tmpImageVertical);


    for(int i=0;i<image.rows;i++){
        for(int j=0;j<image.cols;j++){
            auto leftPositionValue = tmpImageHorizontal.at<float>(i,j);
            auto rightPositionValue = tmpImageVertical.at<float>(i,j);
            if((int)rightPositionValue != 0 && (int)leftPositionValue!=0){
                tmpImage.at<float>(i,j)=round((leftPositionValue+rightPositionValue)/2);
            } else if ((int)rightPositionValue == 0 || (int)leftPositionValue == 0 ){
                tmpImage.at<float>(i,j)=round((leftPositionValue+rightPositionValue));
            }
        }
    }

    return tmpImage;
}


pcl::PointCloud<pcl::PointXYZRGB> generatePointCloud(cv::Mat medianImage,std::string name, bool withNaN, Eigen::Matrix4d transformCLoud){

    transpose(medianImage, medianImage);
    cv::Mat tmpImage;
    transpose(reference_rgb, tmpImage);
    cv::imwrite("depth_med.png",medianImage);
    cv::imwrite("ref.png",tmpImage);

    pcl::PointCloud<pcl::PointXYZRGB> cloud; //(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud.width = static_cast<uint32_t>(medianImage.cols);
    cloud.height = static_cast<uint32_t>(medianImage.rows);
    cloud.is_dense = false;
    cloud.points.resize( cloud.width* cloud.height);

    uint32_t n=0;
    double x=0, y=0, z=0;
    const float fx=363.709,fy=363.709,cx=254.799,cy=209.699;
    cvtColor(tmpImage, tmpImage, cv::COLOR_RGBA2RGB);
//    flip(reference_rgb, reference_rgb, +1);

    for (int x_side = 0; x_side < medianImage.cols; x_side++)
    {
        for (int y_side = 0; y_side < medianImage.rows; y_side++)
        {
            const float icx(cx), icy(cy);
            const float ifx(1/fx), ify(1/fy);
            double Z = medianImage.at<float>(y_side, x_side)/1000.0f;
 //           auto Z=medianImage.data[512*y_side+x_side]/1000.0f;

            if(withNaN){
                x=(x_side + 0.5 - icx)*ifx*Z;
                y=(y_side + 0.5 - icy)*ify*Z;
                z=Z;
            } else {
                if(Z>0)
                {
                    x=(x_side + 0.5 - icx)*ifx*Z;
                    y=(y_side + 0.5 - icy)*ify*Z;
                    z=Z;
                }
                else
                {
                    x=NAN;
                    y=NAN;
                    z=NAN;
                }
            }

            cloud.points[n].x=x;
            cloud.points[n].y=y;
            cloud.points[n].z=z;
//            cloud->points[n].r=maskRGB.at<cv::Vec3b>(x_side,y_side)[2];
//            cloud->points[n].g=maskRGB.at<cv::Vec3b>(x_side,y_side)[1];
//            cloud->points[n].b=maskRGB.at<cv::Vec3b>(x_side,y_side)[0];

            cv::Vec3b tmp_rgb=tmpImage.at<cv::Vec3b>(y_side,x_side);
            if(tmp_rgb[2]==0 && tmp_rgb[1]==0 && tmp_rgb[0]==0)
            {
              tmp_rgb[2]=10;
              tmp_rgb[1]=10;
              tmp_rgb[0]=10;
            }

            if(tmp_rgb[2]>170 && tmp_rgb[1]>170 && tmp_rgb[0]>170)
            {
              tmp_rgb[2]=10;
              tmp_rgb[1]=10;
              tmp_rgb[0]=10;
            }

            cloud.points[n].r=tmp_rgb[2];
            cloud.points[n].g=tmp_rgb[1];
            cloud.points[n].b=tmp_rgb[0];
            n++;
        }
    }

    std::vector<int> removedPoints;
    pcl::removeNaNFromPointCloud(cloud,cloud,removedPoints);
    pcl::io::savePLYFileASCII(name+".ply",cloud);
    cloud = transformCloud(cloud, transformCLoud);

    return cloud;
}

cv::Mat repairInputData(cv::Mat image, cv::Mat interfROI,  cv::Mat median, int count){

//    cv::imwrite(path+"/data/image"+ IntToStr(count) + ".png",image);
//    cv::imwrite(path+"/data/interf"+ IntToStr(count) + ".png",interfROI);

//    ofstream myfile;
//    myfile.open(path + "/data/" + IntToStr(count) +".txt");
    for(int i=0;i<image.rows;i++){
        for(int j=0;j<image.cols;j++){
//            if(interfROI.at<uint>(i,j)>0){
//                std::cout<<interfROI.at<uint>(i,j)<<std::endl;
                auto imagePixel = image.at<float>(i,j);
                auto medianPixel = median.at<float>(i,j);

                auto var = abs(imagePixel-medianPixel);

                if(var>replace_thresh){
//                    image.at<float>(i,j)=medianPixel;
//                    myfile<<1<<"\n";
                    image.at<float>(i,j)=0;

                } else {
//                    myfile<<0<<"\n";
                }

//            }
        }
    }
//    myfile.close();
//    (void)generatePointCloud(image,path+"/data/cloud"+IntToStr(count));
    return image;
}

void saveLUT(cv::Mat depth, std::string path)
{
#define ir_depth_width 512
#define ir_depth_height 424
    int i,j;

    ofstream myfile;
    myfile.open(path + "/data/median.txt");

    for(i = 0; i < depth.cols; i++)
    {
        for(j = 0; j < depth.rows; j++)
        {
            double orig = depth.at<float>(j,i);
            myfile<<orig<<"\n";

        }
    }

    myfile.close();
}

cv::Mat generateIslands(cv::Mat image){

    for(int i=removeGB.x; i < removeGB.width + removeGB.x; i++){
        for(int j=removeGB.y; j<424; j++){
//            std::cout<<i<<" "<<j<<std::endl;
            image.at<float>(j,i)=0;
        }
    }
    return image;
}


pcl::PointCloud<pcl::PointXYZRGB> transformCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, Eigen::Matrix4d transformCLoud){
    pcl::transformPointCloud(cloud,cloud,transformCLoud,true);
    return cloud;
}

template <class T>
std::string IntToStr(T n){
    std::stringstream result;
    result << n;
    return result.str();
}
