#include <iostream>
#include <cmath>
#include <algorithm>
#include "ACSP.hpp"

#include <fstream>  // 需要包含文件流头文件


using namespace FastMath;
using namespace ACSP::Controller;
using namespace ACSP::LTI;


int main() {
    std::ofstream outputFile("motor_control.txt");  // 创建并打开文件
    if (!outputFile.is_open()) {             // 检查文件是否成功打开
        std::cerr << "Failed to open output file!" << std::endl;
        return 1;  // 返回错误码
    }

    /*
     *       1.25
     *   -----------------------------
     *   s^2 + 0.5s
     * */
    Vector<double, 2> b({1.25});
    Vector<double, 2> a({0,0.5});
    auto tf = SISO::tf(a, b);

    ACSP::Controller::PD pid;

    double w = 4;
    pid.setParam(w*w, 2*w,{-10, 10});

    tf.setInput(1.0);
    double t = 0.0;
    double dt = 1e-3;
    std::cout << "data format: [time],[y],[pid]" << std::endl;
    while (t < 10.0) {
        pid.Enable = true;
        pid.alpha = 1.5;
        pid.Target = 1;
        pid.Feedback = tf.getOutput();
        pid.step(dt);
        tf.setInput(pid.ControlOutput);
        tf.step(dt);
        t += dt;
        outputFile << t << "," << tf.getOutput() << "," << pid.ControlOutput << std::endl;  // 写入文件
        // std::cout << t << "," << tf.getOutput() << "," << pid.ControlOutput << std::endl;
    }
    outputFile.close();  // 显式关闭文件（析构时也会自动关闭）

    return EXIT_SUCCESS;
}

/* matlab 画图程序
clear
clc

opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "output", "pid"];
opts.VariableTypes = ["double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable("motor_control.txt", opts);

%% Convert to output type
data = table2array(data);

%% Clear temporary variables
clear opts

%% data process
t = data(:, 1);
y = data(:, 2);
u = data(:, 3);
%% show
subplot(2,1,1)
plot(t, y, 'b')
grid on
subplot(2,1,2)
plot(t, u, 'r')
xlabel("time")
ylabel("ouput")
 */