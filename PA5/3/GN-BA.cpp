//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream p2dfs, p3dfs;
    p2dfs.open(p2d_file);
    p3dfs.open(p3d_file);
    
    if(!p2dfs || !p3dfs) return -1;

    string p2dline, p3dline;
    
    while(getline(p2dfs,p2dline) && !p2dline.empty())
    {
        istringstream currLine(p2dline);
        Vector2d curr;
        currLine>>curr[0]>>curr[1];
        p2d.push_back(curr);
    } 
    
    while(getline(p3dfs,p3dline) && !p3dline.empty())
    {
        istringstream currLine(p3dline);
        Vector3d curr;
        currLine>>curr[0]>>curr[1]>>curr[2];
        p3d.push_back(curr);
    } 
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Vector3d cameraCoord = T_esti * p3d[i];
            Vector3d normCoord(p2d[i][0], p2d[i][1], 1);
            Vector3d residual = normCoord - K * cameraCoord / cameraCoord[2];

            Vector2d e(residual[0], residual[1]);
            cost += 0.5 * e.transpose() * e;
	        // END YOUR CODE HERE

	        // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            double x = cameraCoord[0], y = cameraCoord[1], z = cameraCoord[2];
            J(0,0) = -fx/z;
            J(0,2) = fx*x/(z*z);
            J(0,3) = fx*x*y/(z*z);
            J(0,4) = -fx - fx*(x*x)/(z*z);
            J(0,5) =  fx*y/z;
            J(1,1) = -fy/z;
            J(1,2) = fy*y/(z*z);
            J(1,3) = fy+fy*(y*y)/(z*z);
            J(1,4) = -fy*x*y/(z*z);
            J(1,5) = -fy*x/z;
            // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	    // solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3::exp(dx) * T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
