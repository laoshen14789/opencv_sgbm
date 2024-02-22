/*        双目测距        */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <pthread.h>

using namespace std;
using namespace cv;

const int imageWidth = 640; // 摄像头的分辨率
const int imageHeight = 480;

Size imageSize = Size(imageWidth, imageHeight);

Mat img;
Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;
Rect m_l_select;
Rect m_r_select;

Rect validROIL; // 图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy; // 映射表
Mat Rl, Rr, Pl, Pr, Q;          // 校正旋转矩阵R，投影矩阵P 重投影矩阵Q
Mat xyz;                        // 三维坐标

int blockSize = 8, mindisparity = 1, ndisparities = 64, img_channels = 3;
Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, blockSize);

queue<cv::Mat> frames; // 先进先出队列
pthread_t get_frame_task;
pthread_mutex_t get_frame_mutex = PTHREAD_MUTEX_INITIALIZER;
int key = 0;

#if 0
/*事先标定好的左相机的内参矩阵
fx 0 cx
0 fy cy
0  0  1
*/
Mat cameraMatrixL = (Mat_<double>(3, 3) << 4.415933973970764e+02, 0.888293291277704, 3.526651555562162e+02, 0, 4.411133787693817e+02, 2.232619941949315e+02, 0, 0, 1.);
// 获得的畸变参数

/*418.523322187048	0	0
-1.26842201390676	421.222568242056	0
344.758267538961	243.318992284899	1 */
// 2

Mat distCoeffL = (Mat_<double>(5, 1) << -0.046491781796315, 0.044020442291142, 2.058356678536538e-04, -0.002391054201273, 0);
//[0.006636837611004,0.050240447649195] [0.006681263320267,0.003130367429418]

/*事先标定好的右相机的内参矩阵
fx 0 cx
0 fy cy
0  0  1
*/
Mat cameraMatrixR = (Mat_<double>(3, 3) << 4.415806910857655e+02, 0.986324854776383, 3.516384751701322e+02, 0, 4.415787207304480e+02, 2.187716974773817e+02, 0, 0, 1);

/*
417.417985082506	0	0
0.498638151824367	419.795432389420	0
309.903372309072	236.256106972796	1
*/
// 2

Mat distCoeffR = (Mat_<double>(5, 1) << -0.025116847461741, -0.009398368454655, -5.469746901740971e-04, -0.001401086804110, 0);
//[-0.038407383078874,0.236392800301615]  [0.004121779274885,0.002296129959664]

Mat T = (Mat_<double>(3, 1) << -84.112786580188410, 0.048738965544393, -1.093135047571078); // T平移向量
//[-1.210187345641146e+02,0.519235426836325,-0.425535566316217]
// 对应Matlab所得T参数
// Mat rec = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//rec旋转向量，对应matlab om参数  我
Mat rec = (Mat_<double>(3, 3) << 0.999813536769747, 2.987693235819144e-04, -0.019308092315418,
           -3.020587502927146e-04, 0.999999940360662, -1.674490481028591e-04,
           0.019308041135257, 1.732500032477791e-04, 0.999813567387418); // rec旋转向量，对应matlab om参数  我

/* 0.999341122700880	0.000660748031451783	-0.0362888948713456
-0.00206388651740061	0.999250989651683	-0.0386419468010579
0.0362361815232777	0.0386913826603732	0.998593969567432 */

// Mat T = (Mat_<double>(3, 1) << -48.4, 0.241, -0.0344);//T平移向量
//[-1.210187345641146e+02,0.519235426836325,-0.425535566316217]
// 对应Matlab所得T参数
#else

/*事先标定好的左相机的内参矩阵
fx 0 cx
0 fy cy
0  0  1
*/
Mat cameraMatrixL = (Mat_<double>(3, 3) << 4.415933973970764e+02, 0.888293291277704, 3.526651555562162e+02, 0, 4.411133787693817e+02, 2.232619941949315e+02, 0, 0, 1.);
// 获得的畸变参数

/*418.523322187048	0	0
-1.26842201390676	421.222568242056	0
344.758267538961	243.318992284899	1 */
// 2

Mat distCoeffL = (Mat_<double>(5, 1) << -0.046491781796315, 0.044020442291142, 2.058356678536538e-04, -0.002391054201273, 0);
//[0.006636837611004,0.050240447649195] [0.006681263320267,0.003130367429418]

/*事先标定好的右相机的内参矩阵
fx 0 cx
0 fy cy
0  0  1
*/
Mat cameraMatrixR = (Mat_<double>(3, 3) << 4.415806910857655e+02, 0.986324854776383, 3.516384751701322e+02, 0, 4.415787207304480e+02, 2.187716974773817e+02, 0, 0, 1);

/*
417.417985082506	0	0
0.498638151824367	419.795432389420	0
309.903372309072	236.256106972796	1
*/
// 2

Mat distCoeffR = (Mat_<double>(5, 1) << -0.025116847461741, -0.009398368454655, -5.469746901740971e-04, -0.001401086804110, 0);
//[-0.038407383078874,0.236392800301615]  [0.004121779274885,0.002296129959664]

Mat T = (Mat_<double>(3, 1) << -84.112786580188410, 0.048738965544393, -1.093135047571078); // T平移向量
//[-1.210187345641146e+02,0.519235426836325,-0.425535566316217]
// 对应Matlab所得T参数
// Mat rec = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//rec旋转向量，对应matlab om参数  我
Mat rec = (Mat_<double>(3, 3) << 0.999813536769747, 2.987693235819144e-04, -0.019308092315418,
           -3.020587502927146e-04, 0.999999940360662, -1.674490481028591e-04,
           0.019308041135257, 1.732500032477791e-04, 0.999813567387418); // rec旋转向量，对应matlab om参数  我

/* 0.999341122700880	0.000660748031451783	-0.0362888948713456
-0.00206388651740061	0.999250989651683	-0.0386419468010579
0.0362361815232777	0.0386913826603732	0.998593969567432 */

// Mat T = (Mat_<double>(3, 1) << -48.4, 0.241, -0.0344);//T平移向量
//[-1.210187345641146e+02,0.519235426836325,-0.425535566316217]
// 对应Matlab所得T参数
#endif

Mat R; // R 旋转矩阵

// 深度图转伪彩
// void convertColor(Mat & image)
// {
//     /*https://blog.csdn.net/qq_51639530/article/details/124462762*/
//     //读取16位深度图（像素范围0～65535），并将其转化为8位（像素范围0～255）
//     double minValue, maxValue;    // 最大值，最小值
//     cv::Point  minIdx, maxIdx;    // 最小值坐标，最大值坐标
//     cv::minMaxLoc(image, &minValue, &maxValue, &minIdx, &maxIdx);

//     image -= minValue;
// //    image = image / (maxValue - minValue);
//     image =image/((numDisparities * 16 + 16)*16.);
//     image *= 255;

//     //使得越近的地方深度值越大，越远的地方深度值越小，以达到伪彩色图近蓝远红的目的。
//     image = 255 - image;

//    // cv2 中的色度图有十几种，其中最常用的是 cv2.COLORMAP_JET，蓝色表示较高的深度值，红色表示较低的深度值。
//    // cv.convertScaleAbs() 函数中的 alpha 的大小与深度图中的有效距离有关，如果像我一样默认深度图中的所有深度值都在有效距离内，并已经手动将16位深度转化为了8位深度，则 alpha 可以设为1。
//     convertScaleAbs(image,image,1);
//     applyColorMap(image,image, COLORMAP_JET);

// }

void get_depth_info(Vec3f *obstacleCoordinates, Vec2i *depthCoordinates, float *depth)
{
    Vec3f point3;
    Point origin;
    float d = 0.0;
    int height = 480;
    int width = 640;
    int stepX = width / 20;
    int stepY = height / 20;

    int sampling_width = 212;
    int sampling_height = 160;
    for (int k = 0; k < 9; k++)
    {
        if (k % 3 == 0 && k != 0)
        {
            sampling_width = 212;
            sampling_height = sampling_height + sampling_height;
        }
        int x = 0, y = 0;
        x = sampling_width - 212;
        while (x < sampling_width)
        {
            x = x + stepX;
            y = sampling_height - 160;
            while (y < sampling_height)
            {
                y = y + stepY;
                int xPixel = 0;
                int yPixel = 0;
                xPixel = (x < 0) ? 0 : ((x > width - 1) ? width - 1 : (int)x);
                yPixel = (y < 0) ? 0 : ((y > height - 1) ? height - 1 : (int)y);

                origin = Point(xPixel, yPixel);
                point3 = xyz.at<Vec3f>(origin);

                d = point3[0] * point3[0] + point3[1] * point3[1] + point3[2] * point3[2];
                d = sqrt(d) / 10.0; // cm
                // cout << "xPixel:" << xPixel << "yPixel:" << yPixel << " d:" << d <<endl;
                if (d <= depth[k] && d > 70.0 && d < 300.0)
                {
                    // cout << "k:" << k << " xPixel:" << xPixel << " yPixel:" << yPixel << " d:" << d << endl;
                    depth[k] = d / 100;
                    obstacleCoordinates[k] = xyz.at<Vec3f>(Point(xPixel, yPixel));
                    depthCoordinates[k] = Point(xPixel, yPixel);
                }
                // depth[k] = depth[k] / 10.0; // cm
            }
        }
        sampling_width = sampling_width + 212;
    }
    // cout << sampling_width << " " << sampling_height << endl;
}
/*****立体匹配*****/
void stereo_match(int, void *)
{
    Vec3f obstacleCoordinates[9];
    Vec2i depthCoordinates[9];
    float depth[9];
    /*
    bm->setBlockSize(2 * blockSize + 5);     //SAD窗口大小，5~21之间为宜
    bm->setROI1(validROIL);
    bm->setROI2(validROIR);
    bm->setPreFilterCap(31);
    bm->setMinDisparity(0);  //最小视差，默认值为0, 可以是负值，int型
    bm->setNumDisparities(numDisparities * 16 + 16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(-1);
    */
    int lineX, lineY;
    int P1 = 8 * img_channels * blockSize * blockSize;
    int P2 = 32 * img_channels * blockSize * blockSize;
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setPreFilterCap(1);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleRange(100);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setDisp12MaxDiff(-1);
    // sgbm->setNumDisparities(1);
    sgbm->setMode(cv::StereoSGBM::MODE_HH);

    Mat disp, disp8;
    sgbm->compute(rectifyImageL, rectifyImageR, disp); // 输入图像必须为灰度图
    disp8 = Mat(disp.rows, disp.cols, CV_8UC1);
    normalize(disp, disp8, 0, 255, NORM_MINMAX, CV_8UC1);
    reprojectImageTo3D(disp, xyz, Q, true); // 在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    // convertScaleAbs(disp8,disp8,1);
    applyColorMap(disp8, disp8, COLORMAP_JET);
    xyz = xyz * 16;

    for (int i = 0; i < 9; i++)
    {
        depth[i] = 301.0;
    }
    get_depth_info(obstacleCoordinates, depthCoordinates, depth);
    cout << "depth: " << depth[8] << "m" << endl;

    for (int i = 1; i < 3; i++)
    {
        lineX = 212 * i;
        line(disp8, Point(lineX, 0), Point(lineX, 480), Scalar(0, 255, 255), 2);
    }
    for (int j = 1; j < 3; j++)
    {

        lineY = j * 160;
        line(disp8, Point(0, lineY), Point(640, lineY), Scalar(0, 255, 255), 2);
    }

    for (int i = 0; i < 9; i++)
    {
        putText(disp8, std::to_string(depth[i]), Point(212 * (0.25 + i % 3), 160 * (0.33 + floor(i / 3))), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
    }

    imshow("disparity", disp8);
}

/*****描述：鼠标操作回调*****/
static void onMouse(int event, int x, int y, int, void *)
{
    Vec3f point3;
    Point origin;              // 鼠标按下的起始点
    Rect selection;            // 定义矩形选框
    bool selectObject = false; // 是否选择对象
    float d;

    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event)
    {
    case EVENT_LBUTTONDOWN: // 鼠标左按钮按下的事件
        d = 0;
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        // cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
        point3 = xyz.at<Vec3f>(origin);
        point3[0];
        // cout << "point3[0]:" << point3[0] << "point3[1]:" << point3[1] << "point3[2]:" << point3[2]<<endl;
        cout << "世界坐标：" << endl;
        cout << "x: " << point3[0] << "  y: " << point3[1] << "  z: " << point3[2] << endl;
        d = point3[0] * point3[0] + point3[1] * point3[1] + point3[2] * point3[2];
        d = sqrt(d); // mm
        // cout << "距离是:" << d << "mm" << endl;

        // d = d / 10.0; // cm

        d = d / 1000.0; // m
        // cout << "距离是:" << d << "m" << endl;
        cout << "x:" << x << "y:" << y << " 11111距离是:" << d << " cm" << endl;

        break;
    case EVENT_LBUTTONUP: // 鼠标左按钮释放的事件
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
            break;
        break;
    case EVENT_RBUTTONDOWN: // 鼠标右按钮按下的事件
        int pointX, pointY;
        cout << "\n"
             << endl;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                d = 0;
                pointX = 106 + (i * 212);
                pointY = 80 + (j * 160);

                origin = Point(pointX, pointY);
                point3 = xyz.at<Vec3f>(origin);
                // cout << "世界坐标：" << endl;
                // cout << "wx: " << point3[0] << "  wy: " << point3[1] << "  wz: " << point3[2] << endl;
                d = point3[0] * point3[0] + point3[1] * point3[1] + point3[2] * point3[2];
                d = sqrt(d);  // mm
                d = d / 10.0; // cm
                cout << "x:" << pointX << "y:" << pointY << " 22222距离是:" << d << " cm" << endl;
            }
        }
        break;

    default:
        break;
    }
}

void *get_frame_handle(void *arg)
{
    /*
    读取图片
    */
    VideoCapture cap;
    Mat frame;
    cap.open(0);
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    if (!cap.isOpened())
    {
        cout << "open failed" << endl;
        return 0;
    }
    while (key != 'q')
    {
        cap >> frame;
        // frame = imread("222.jpg");
        pthread_mutex_lock(&get_frame_mutex);
        frames.push(frame);
        pthread_mutex_unlock(&get_frame_mutex);
    }
    cap.release();
    return NULL;
}

/*****主函数*****/
int main()
{
    Mat frame;
    /*
    立体校正
    */
    Rodrigues(rec, R); // Rodrigues变换
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
                  0, imageSize, &validROIL, &validROIR);
    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

    namedWindow("disparity", WINDOW_AUTOSIZE);
    setMouseCallback("disparity", onMouse, 0);
    pthread_create(&get_frame_task, NULL, get_frame_handle, NULL);
    /*
    读取图片
    */
    while (key != 'q')
    {
        if (!frames.empty())
        {
            pthread_mutex_lock(&get_frame_mutex);
            frame = frames.front();
            frames.pop();
            pthread_mutex_unlock(&get_frame_mutex);

            m_l_select = Rect(0, 0, 640, 480);
            img = frame;
            // img = imread("222.jpg", IMREAD_COLOR);
            // imshow("Image", img);
            rgbImageL = img(m_l_select);
            cvtColor(rgbImageL, grayImageL, COLOR_BGR2GRAY);

            m_r_select = Rect(640, 0, 640, 480);
            rgbImageR = img(m_r_select);
            cvtColor(rgbImageR, grayImageR, COLOR_BGR2GRAY);

            // imshow("ImageL", rgbImageL);
            // imshow("ImageR", rgbImageR);

            /*
            经过remap之后，左右相机的图像已经共面并且行对准了
            */
            remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
            remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

            /*
            把校正结果显示出来
            */
            Mat rgbRectifyImageL, rgbRectifyImageR;
            cvtColor(rectifyImageL, rgbRectifyImageL, COLOR_GRAY2BGR); // 伪彩色图
            cvtColor(rectifyImageR, rgbRectifyImageR, COLOR_GRAY2BGR);

            // 单独显示
            // rectangle(rgbRectifyImageL, validROIL, Scalar(0, 0, 255), 3, 8);
            // rectangle(rgbRectifyImageR, validROIR, Scalar(0, 0, 255), 3, 8);
            // imshow("ImageL After Rectify", rgbRectifyImageL);
            // imshow("ImageR After Rectify", rgbRectifyImageR);

            // 显示在同一张图上
            // Mat canvas;
            // double sf;
            // int w, h;
            // sf = 600. / MAX(imageSize.width, imageSize.height);
            // w = cvRound(imageSize.width * sf);
            // h = cvRound(imageSize.height * sf);
            // canvas.create(h, w * 2, CV_8UC3); // 注意通道

            // 左图像画到画布上
            // Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                             // 得到画布的一部分
            // resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA); // 把图像缩放到跟canvasPart一样大小
            // Rect vroiL(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),           // 获得被截取的区域
            //            cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
            // rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形
            // cout << "Painted ImageL" << endl;

            // 右图像画到画布上
            // canvasPart = canvas(Rect(w, 0, w, h)); // 获得画布的另一部分
            // resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
            // Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
            //            cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
            // rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
            // cout << "Painted ImageR" << endl;

            // 画上对应的线条
            // for (int i = 0; i < canvas.rows; i += 16)
            //     line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
            // imshow("rectified", canvas);

            /*
            立体匹配
            */
            // namedWindow("disparity", WINDOW_AUTOSIZE);

            /*************************调参可视化**********************************************/
            // 创建SAD窗口 Trackbar
            // createTrackbar("BlockSize:\n", "disparity", &blockSize, 8, stereo_match);
            // 创建视差唯一性百分比窗口 Trackbar
            // createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);
            // 创建视差窗口 Trackbar
            // createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);

            // 鼠标响应函数setMouseCallback(窗口名称, 鼠标回调函数, 传给回调函数的参数，一般取0)
            // setMouseCallback("disparity", onMouse, 0);
            stereo_match(0, 0);
            // imshow("frame", frame);
        }
        key = waitKey(30);
    }

    return 0;
}