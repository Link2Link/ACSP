#ifndef ACSP_PID_HPP
#define ACSP_PID_HPP
#include "FastMath.hpp"
#include "LTI/LTI.hpp"
#include <array>

namespace ACSP::Controller
{
    namespace LTI = ACSP::LTI;

    class PID
    {
    public:
        LTI::StateSpace<2, 1, 1> ss;

        bool Enable;

        // Control Input
        double Target;      // 目标位置
        double Feedback;    // 反馈位置

        // Control Output
        double ControlOutput;   // 控制输出

        double Kp;
        double Ki;
        double Kd;
        double Tf;              // 微分时间常数
        double alpha;           // 积分饱和系数
        std::array<double, 2> bound;    // 控制量限幅

        bool intSatFlag;        // 积分饱和标志
        bool outSatFlag;        // 输出饱和标志

        PID() : Target(0), Feedback(0), ControlOutput(0), Enable(false)
        {
            Kp = 1;
            Ki = 0;
            Kd = 0;
            Tf = 1e-3;
            bound[0] = -100;
            bound[1] = 100;
            alpha = 2.0;        // 默认积分饱和系数2.0，即积分控制的作用是控制器最大输出的2倍
            ss.cleanAll();
        }

        void setParam(const double & Kp, const double & Ki, const double & Kd, const double & Tf, const std::array<double, 2> & bound)
        {
            this->Kp = Kp;
            this->Ki = Ki;
            this->Kd = Kd;
            this->Tf = Tf;
            this->bound[0] = bound[0];
            this->bound[1] = bound[1];
        }

        void step(double dt)
        {
            intSatFlag = false;
            LTI::State<1> u, y;
            u(0) = Target - Feedback; //calc err
            calc_ABCD();
            ss.setInput(u);
            ss.step(dt);
            y = ss.getOutput();
            double out = y(0);
            out = sat(out);

            // 抗积分饱和，会在下一周期起作用，所以积分控制量会略微超过bound * alpha一点
            if (ss.x(0)*Ki < bound[0] * alpha ) {
                ss.x(0) = bound[0] * alpha / Ki ;   // 积分控制作用超过下限，将状态限制在bound*alpha/Ki
                intSatFlag = true;
            }
            if (ss.x(0)*Ki > bound[1] * alpha ) {
                ss.x(0) = bound[1] * alpha / Ki ;
                intSatFlag = true;
            }

            if (Enable)
                ControlOutput = out;
            else
            {
                ControlOutput = 0;
                reset(Feedback);
            }
        }

        void reset(double y)
        {
            ss.cleanAll();
            calc_ABCD();
        }

    protected:
        void calc_ABCD()
        {
            // ESO update
            /*
             * A = [0, 0;
             *      0, -1/Tf]
             */
            ss.A.setZero();
            ss.A(1,1) = -1.0/Tf;

            /*
             * B = [1;
             *      1/Tf]
             */
            ss.B.setZero();
            ss.B(0, 0) = 1.0;
            ss.B(1, 0) = 1.0/Tf;

            /*
             * C = [Ki -Kd/Tf]
             */
            ss.C.setZero();
            ss.C(0, 0) = Ki;
            ss.C(0, 1) = -Kd/Tf;

            /*
             * D = Kp + Kd/Tf
             */
            ss.D.setZero();
            ss.D(0, 0) = Kp + Kd/Tf;
        }
        inline double sat(double in)
        {
            if (bound[0] > bound[1])	// for safety
                return 0;
            if (in < bound[0]) {
                outSatFlag = true;
                return bound[0];
            }

            if (in > bound[1]) {
                outSatFlag = true;
                return bound[1];
            }
            outSatFlag = false;
            return in;
        }
    };

    class PI : public PID {
    public:
        PI() : PID()
        {}

        void setParam(const double & Kp, const double & Ki, const std::array<double, 2> & bound)
        {
            this->Kp = Kp;
            this->Ki = Ki;
            this->bound[0] = bound[0];
            this->bound[1] = bound[1];
        }
    };

    class PD : public PID {
        public:
        PD() : PID(){}
        void setParam(const double & Kp, const double & Kd, const std::array<double, 2> & bound)
        {
            this->Kp = Kp;
            this->Kd = Kd;
            this->bound[0] = bound[0];
            this->bound[1] = bound[1];
        }

    };

}


#endif //ACSP_PID_HPP
