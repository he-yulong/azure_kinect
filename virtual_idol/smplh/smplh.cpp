// smplh.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

int main()
{
    Eigen::MatrixXd pose_pca_basis_matrix(153, 153);
    double* pose_pca_basis = new double[153, 153];
    {
        
        std::ifstream in;
        in = std::ifstream("pose_pca_basis.data", std::ios::in | std::ios::binary);
        in.read((char*)&pose_pca_basis_matrix, sizeof pose_pca_basis_matrix);
        // see how many bytes have been read
        std::cout << in.gcount() << " bytes read\n";
        in.close();
        for (int i = 0; i < 153; i++)
            for (int j = 0; j < 153; j++)
                pose_pca_basis_matrix << pose_pca_basis[i][j];
    }



    //Eigen::VectorXd pose_pca_mean_vector(153);
    //double pose_pca_mean[153] = { 0 };
    //in = std::ifstream("pose_pca_mean.data", std::ios::in | std::ios::binary);
    //in.read((char*)&pose_pca_mean, sizeof pose_pca_mean);
    //// see how many bytes have been read
    //std::cout << in.gcount() << " bytes read\n";
    //in.close();
    //for (int i = 0; i < 153; i++)
    //    pose_pca_mean_vector << pose_pca_mean[i];

    //Eigen::MatrixXd J_regressor_matrix(52, 6890);
    //double J_regressor[52][6890] = { 0 };
    //in = std::ifstream("J_regressor.data", std::ios::in | std::ios::binary);
    //in.read((char*)&J_regressor, sizeof J_regressor);
    //// see how many bytes have been read
    //std::cout << in.gcount() << " bytes read\n";
    //in.close();
    //for (int i = 0; i < 52; i++)
    //    for (int j = 0; j < 6890; j++)
    //        J_regressor_matrix << J_regressor[i][j];


    //std::cout << J_regressor_matrix(0, 0);

    return 0;
}
