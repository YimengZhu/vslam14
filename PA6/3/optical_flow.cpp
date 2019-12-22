#include <opencv2/opencv.hpp>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;

// this program shows how to use optical flow
double fx = 718.856;
double baseline = 0.573;

string file_left = "./left.png";  // first image
string file_right = "./right.png";  // second image
string file_dis = "./disparity.png";

void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}


int main(int argc, char **argv) {

    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = imread(file_left, 0);
    Mat img2 = imread(file_right, 0);
    Mat img3 = imread(file_dis, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    vector<Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, cv::Size(8, 8));

    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            int dis = pt1[i].x - pt2[i].x;
            double depth_real = img3.at<uchar>(pt1[i].y, pt1[i].x);
            if (dis > 0)
            cout <<"estimated disparity:  " << dis << "; real disparity: " << depth_real << endl; 
        }
    }


    return 0;
}



