#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "./compare.txt";


vector<double> timee, timeg;
vector<Eigen::Vector3d> te, tg;
vector<Eigen::Quaterniond> qe, qg;

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> posesE, 
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> posesG);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> posesE, posesG;

    /// implement pose reading code
    // start your code here (5~10 lines)
	ifstream trjfs;
	trjfs.open(trajectory_file);
	if(!trjfs)return -1;	
    
    string trjLine;
    while(getline(trjfs, trjLine) && !trjLine.empty()){
        stringstream lineStream(trjLine);

        double timee_cur, timeg_cur;
        Eigen::Vector3d te_cur, tg_cur;
        Eigen::Quaterniond qe_cur, qg_cur;

        lineStream >> timee_cur >> te_cur[0] >> te_cur[1] >> te_cur[2] 
            >> qe_cur.x() >> qe_cur.y() >> qe_cur.z() >> qe_cur.w()
            >> timeg_cur >> tg_cur[0] >> tg_cur[1] >> tg_cur[2] 
            >> qg_cur.x() >> qg_cur.y() >> qg_cur.z() >> qg_cur.w();
        
        timee.push_back(timee_cur);
        timeg.push_back(timeg_cur);
        te.push_back(te_cur);
        tg.push_back(tg_cur);
        qe.push_back(qe_cur.normalized());
        qg.push_back(qg_cur.normalized());
        
        posesG.push_back(Sophus::SE3(qg_cur.normalized(), tg_cur));
    }
	trjfs.close();
    
    // compute ICP via SVD
    Eigen::Vector3d centriodG, centriodE;
    for(int i = 0; i < te.size(); i++){
        centriodG += tg[i];
        centriodE += te[i];
    }
    centriodE /= te.size();
    centriodG /= tg.size();
    
    vector<Eigen::Vector3d> te_noCen, tg_noCen;
    for(int i = 0; i < te.size(); i++){
        te_noCen.push_back(te[i] - centriodE);
        tg_noCen.push_back(tg[i] - centriodG);
    }

    Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
    for(int i = 0; i < te_noCen.size(); i++){
        w += tg_noCen[i] * te_noCen[i].transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d u = svd.matrixU();
    Eigen::Matrix3d v = svd.matrixV();

    Eigen::Matrix3d r = u * (v.transpose());
    Eigen::Vector3d t = centriodG - r * centriodE;

    Sophus::SE3 tge(r, t);
    for(int i = 0; i < te.size(); i++){
        Sophus::SE3 poseE(qe[i], te[i]); 
        posesE.push_back(tge * poseE);
    }
    
    // draw trajectory in pangolin
    DrawTrajectory(posesE, posesG);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> posesE,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> posesG) {
    if (posesE.empty() || posesG.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
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
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < posesE.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = posesE[i], p2 = posesE[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < posesG.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = posesG[i], p2 = posesG[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
