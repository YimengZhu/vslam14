#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define SIZE 100

using namespace std;
using namespace Eigen;


int main(){

	Matrix<double, SIZE, SIZE> a;
	VectorXd b(SIZE);

	a = MatrixXd::Random(SIZE, SIZE);	
	a = a * a.transpose(); //ensure a is symetric positive definite
	b = MatrixXd::Random(SIZE, 1);

	VectorXd qr(SIZE);
	qr = a.colPivHouseholderQr().solve(b);
	cout << "solution with QR: " << endl << qr.transpose() << endl;

	VectorXd llt(SIZE);
	llt = a.llt().solve(b);
	cout << "solution with cholesky: "<< endl << llt.transpose() << endl; 

	return 0;
}
