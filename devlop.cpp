#include <iostream>
#include <cmath>
#include <limits>
#include <iterator>
#include <algorithm>
#include <vector>

#include "ACSP.hpp"

using namespace ACSP::math;
using namespace ACSP::Controller;
using namespace ACSP::LTI;



int main() {

    constexpr int dim = 2;
    constexpr int out_dim = 1;

    Matrix<double, dim, dim> A({1.0, 0.0010, -0.0010, 0.9900});
    Matrix<double, dim, out_dim> B({0.0, 0.0995});
    Matrix<double, out_dim, dim> C({1.0, 0.0});
    Matrix<double, out_dim, out_dim> D({0.0});
    SquareMatrix<double, dim> G;
    G.setIdentity();
    Matrix<double, out_dim, dim> H;
    H.setZero();

    KalmanFilter kalman(A,B,C,D,G,H);

    SquareMatrix<double, dim> Q;
    SquareMatrix<double, out_dim> R;
    Matrix<double, dim, out_dim> N;
    SquareMatrix<double, dim> P;

    Q = eye<double, dim>() * 0.05;
    R(0,0) = 1;
    N.setZero();
    P.setIdentity();
    kalman.init(Q,R,N,P);


    DiscreteStateSpace<2,1,1> ss;
    ss.A = A;
    ss.B = B;
    ss.C = C;
    ss.D = D;

    double target = 1;



    double t = 0;

    Vector<double, 1> u;
    auto x = kalman.getState();

    while (t < 5)
    {

        u(0) = (target - x(0)) * 100 + (0 - x(1)) * 20;

        ss.setInput(u);

        ss.step(1e-3);
        auto y = ss.getOutput();

        kalman.update(u, y);
        x = kalman.getState();
        t += 1e-3;

        std::cout << "t: " << t << " u:" << u(0) << " y: " << y(0) << " x1: " << x(0) << " x2: " << x(1) << std::endl;

    }
    return EXIT_SUCCESS;
}
