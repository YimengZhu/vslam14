#include <iostream>
#include <fstream>
#include <unistd.h>
#include "bal_type.h"
#include <vector>
#include <string>
#include <pangolin/pangolin.h>

using namespace std;
using std::vector;
using std::string;

class balCamera
{
public:
    balCamera(double* data){
        Eigen::Vector3d so3;
        so3 << data[0], data[1], data[2];
        Eigen::Matrix3d R = Sophus::SO3d::exp(so3).matrix();
        Sophus::SE3d T(R, Eigen::Vector3d(data[3], data[4], data[5]));
        Eigen::Matrix<double, 6, 1> se3 = T.log();
        cam << se3(0, 0), se3(1, 0), se3(2, 0), se3(3, 0), se3(4, 0), se3(5, 0), data[6], data[7], data[8];
    }
    Eigen::Matrix<double, 9, 1> cam;
};

class balPoint
{
public:
    balPoint(double* data) : pt(data){}
    Eigen::Vector3d pt; 
};

class balEdge
{
public:
    balEdge(int _cam, int _pt, double* data) : cam(_cam), pt(_pt), uv(data){}
    int cam;
    int pt; 
    Eigen::Vector2d uv; 
};



class BalProblem {

    private:     
        vector<balCamera> cameras;
        vector<balPoint> points;
        vector<balEdge> edges;
        g2o::SparseOptimizer optimizer;
    
    public:
        BalProblem(const string& filename) {
        ifstream ifs;
        ifs.open(filename, fstream::in);
        if(!ifs.is_open())return;
    
        int camera_num, point_num, edge_num;
        ifs >> camera_num >> point_num >> edge_num;
    
        for(int i = 0 ; i < edge_num ; ++i) {
            int c, p;
            double uv[2];
            ifs >> c >> p >> uv[0] >> uv[1];
            edges.push_back(balEdge(c, p, uv));
        }
    
        for(int i = 0 ; i < camera_num ; ++i) {
            double data[9];
            for(int j = 0 ; j < 9 ; ++j)
                ifs >> data[j];
            cameras.push_back(balCamera(data));
        }
    
        for(int i = 0 ; i < point_num ; ++i) {
            double data[3];
            ifs >> data[0] >> data[1] >> data[2];
            points.push_back((balPoint(data)));
        }
    
    }
    
    void buildProblem() {
        typedef g2o::BlockSolver< g2o::BlockSolverTraits<9,3> > Block; 
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
        dynamic_cast<g2o::LinearSolverCholmod<Block::PoseMatrixType>*>(linearSolver)->setBlockOrdering(true);
    
    
        Block* solver_ptr = new Block (linearSolver);   
        g2o::OptimizationAlgorithmWithHessian* solver = new g2o::OptimizationAlgorithmLevenberg (solver_ptr);
    
        optimizer.setAlgorithm (solver);
    
        int camera_num = cameras.size();
        int point_num = points.size();
        int edge_num = edges.size();
    
        int index = 0;
        for(auto data:cameras) {
            balCameraVertex* camera = new balCameraVertex();
            camera->setEstimate(data.cam);
            camera->setId(index);
            optimizer.addVertex(camera);
            ++index;
        }
    
        for(auto data:points) {
            bal3DVertex* point = new bal3DVertex();
            point->setEstimate(data.pt);
            point->setId(index);
            point->setMarginalized(true);
            optimizer.addVertex(point);
            ++index;
        }
    
        index = 0;
        for(auto data:edges) {
            balCamPtEdge* edge = new balCamPtEdge();
            edge->setVertex(0, dynamic_cast< balCameraVertex* >( optimizer.vertex(data.cam) ));
            edge->setVertex(1, dynamic_cast< bal3DVertex* >(optimizer.vertex(data.pt + camera_num)));
            edge->setMeasurement(data.uv);
            edge->setId(index);
            edge->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(edge);
            ++index;
        }
    }
    
    void solveProblem(int iter) {
        optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(iter);
    }
    
    
    void show() {
        pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
        pangolin::OpenGlRenderState s_cam(
                 pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                 pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
            );
    
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));
    
    
        while (pangolin::ShouldQuit() == false) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
            d_cam.Activate(s_cam);
            glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    
            glPointSize(1);
            glBegin(GL_POINTS);
            int camera_num = cameras.size();
            for (size_t i = 0; i < points.size(); i++) {
                const bal3DVertex* mypoint = dynamic_cast<const bal3DVertex*>(optimizer.vertex(i + camera_num));
                Eigen::Vector3d point_data = mypoint->estimate();
                glColor3f(1.0, 1.0, 1.0);
                glVertex3d(point_data(0, 0), -point_data(1, 0), -point_data(2, 0) );
            }
            glEnd();
    
            pangolin::FinishFrame();
            usleep(5000);
        }
    
    }
};


int main()
{
    BalProblem bal("./data/problem-49-7776-pre.txt");
    bal.buildProblem();
    bal.solveProblem(40);
    bal.show();

    return 0;
}

