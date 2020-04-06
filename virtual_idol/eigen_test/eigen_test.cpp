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

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4atypes.h>
#include <cmath>

struct Vector
{
    float X;
    float Y;
    float Z;

    Vector(float x, float y, float z)
        : X(x)
        , Y(y)
        , Z(z)
    {
    }

    Vector(const k4a_float3_t& v)
        : X(v.xyz.x)
        , Y(v.xyz.y)
        , Z(v.xyz.z)
    {
    }

    float Dot(const Vector& other) const
    {
        return X * other.X + Y * other.Y + Z * other.Z;
    }

    float SquareLength() const
    {
        return X * X + Y * Y + Z * Z;
    }

    float Length() const
    {
        return std::sqrt(SquareLength());
    }

    Vector operator*(float c) const
    {
        return { X * c, Y * c, Z * c };
    }

    Vector operator/(float c) const
    {
        return *this * (1 / c);
    }

    Vector Normalized() const
    {
        return *this / Length();
    }

    float Angle(const Vector& other) const
    {
        return std::acos(Dot(other) / Length() / other.Length());
    }
};

inline Vector operator-(const Vector& v1, const Vector& v2)
{
    return { v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z };
}

inline Vector operator+(const Vector& v1, const Vector& v2)
{
    return { v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z };
}

inline Vector operator*(float c, const Vector& v)
{
    return { v.X * c, v.Y * c, v.Z * c };
}

inline Vector operator*(const Vector& v1, const Vector& v2)
{
    return { v1.Y * v2.Z - v1.Z * v2.Y, v1.Z * v2.X - v1.X * v2.Z, v1.X * v2.Y - v1.Y * v2.X };
}

struct Plane
{
    using Point = Vector;

    Vector Normal;
    Point Origin;
    float C;

    static Plane Create(Vector n, Point p)
    {
        float c = n.X * p.X + n.Y * p.Y + n.Z * p.Z;
        return { n, p, -c };
    }

    static Plane Create(const Point& p1, const Point& p2, const Point& p3)
    {
        Vector v1 = p2 - p1;
        Vector v2 = p2 - p3;
        Vector n = v1 * v2;
        return Create(n, p1);
    }

    Vector ProjectVector(const Vector& v) const
    {
        return v - Normal * (v.Dot(Normal) / Normal.SquareLength());
    }

    Vector ProjectPoint(const Point& p) const
    {
        return Origin + ProjectVector(p - Origin);
    }

    float AbsDistance(const Point& p) const
    {
        return std::abs(p.X * Normal.X + p.Y * Normal.Y + p.Z * Normal.Z + C) / Normal.Length();
    }
};

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


    /*k4a_float3_t ttt{};
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
    cout << ttt.xyz.z << endl;*/
    

    Vector n(0, 0, 1);
    Vector p(0, 0, 0);
    Plane plane = Plane::Create(n, p);

    Vector v1(1, 0, 0);

    



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
