#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main() {

	Quaterniond q1(0.55, 0.3, 0.2, 0.2);
	Vector3d t1(0.7, 1.1, 0.2);
	
	Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
	Vector3d t2(-0.1, 0.4, 0.8);

	Vector3d p1(0.5, -0.1, 0.2);
	Vector3d p2;

	q1 = q1.normalized();
	q2 = q2.normalized();

	p2 = q2 * q1.conjugate() * (p1 - t1) + t2;
	cout<<p2<<endl;
	
	return 0;
}

