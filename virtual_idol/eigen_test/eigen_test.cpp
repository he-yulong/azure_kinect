// eigen_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <k4a/k4a.h>
#include <k4abt.h>

#include <iostream>
#include <Eigen/Dense>
#include <vector>

using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::RowMajor;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::RowVector3f;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

int main()
{
    //Matrix<double, 3, 3> A;               // Fixed rows and cols. Same as Matrix3d.
    //Matrix<double, 3, Dynamic> B;         // Fixed rows, dynamic cols.
    //Matrix<double, Dynamic, Dynamic> C;   // Full dynamic. Same as MatrixXd.
    //Matrix<double, 3, 3, RowMajor> E;     // Row major; default is column-major.
    //Matrix3f P, Q, R;                     // 3x3 float matrix.
    //Vector3f x, y, z;                     // 3x1 float matrix.
    //RowVector3f a, b, c;                  // 1x3 float matrix.
    //VectorXd v;                           // Dynamic column vector of doubles
    // Eigen          // Matlab           // comments
    // cout << x.size();          // length(x)        // vector size
    //    C.rows()          // size(C,1)        // number of rows
    //    C.cols()          // size(C,2)        // number of columns
    //    x(i)              // x(i+1)           // Matlab is 1-based
    //    C(i, j)            // C(i+1,j+1)       //

    // Basic usage
    //A.resize(4, 4);   // Runtime error if assertions are on.
    //B.resize(4, 9);   // Runtime error if assertions are on.
    //A.resize(3, 3);   // Ok; size didn't change.
    //B.resize(3, 9);   // Ok; only dynamic cols changed.

    //A << 1, 2, 3,     // Initialize A. The elements can also be
    //    4, 5, 6,     // matrices, which are stacked along cols
    //    7, 8, 9;     // and then the rows are stacked.
    //B << A, A, A;     // B is three horizontally stacked A's.
    
    //A.fill(10);       // Fill A with all 10's.

    //x << 1, 2, 3;
    //y << 4, 5, 6;
    //z << 3, 4, 0;


    k4a_float3_t ttt{};
    Eigen::Vector3f vvv;
    vvv = (Eigen::Vector3f) ttt.v;
    cout << vvv << endl;
    cout << "-------------" << endl;
    vvv(0) = 6;
    vvv(1) = 7;
    vvv(2) = 8;
    ttt.xyz.x = 4;
    ttt.xyz.y = vvv(1);
    ttt.xyz.z = vvv(2);

    cout << ttt.xyz.x << endl;
    cout << ttt.v[1] << endl;
    cout << ttt.xyz.z << endl;

    cout << endl;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
