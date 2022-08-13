#ifndef WORLD_H
#define WORLD_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "math_lib.h"
#include <random>

class Map
{
    public:
    Map(int w, int h){
        width = w;
        height = h;
        points.push_back(cv::Point2d(0,0));
        points.push_back(cv::Point2d(0,1));
        points.push_back(cv::Point2d(1,1));
        points.push_back(cv::Point2d(1,2));
        points.push_back(cv::Point2d(2,2));
        points.push_back(cv::Point2d(2,0));
        points.push_back(cv::Point2d(0,0));
    }
    ~Map(){}
    double find_nearest(cv::Point2d p, cv::Point2d& dir)
    {
        double min_dis = 100;
        for(int i = 0; i < points.size()-1; i++)
        {
            cv::Point2d dir1 = points[i+1] - points[i];
            dir1 = calc_normal(dir1);
            cv::Point2d dir2 = points[i] - p;
            double dis = product(dir1, dir2);
            if(fabs(dis) < fabs(min_dis))
            {
                min_dis = dis;
                dir = dir1;
            }
        }
        return min_dis;
    }
    int width;
    int height;
    std::vector<cv::Point2d> points;
};

class Lidar
{
    public:
    Lidar(){
        data.resize(8);
        angle = std::vector<double>{0, M_PI/4, M_PI/2, M_PI*0.75, M_PI, M_PI*1.25, M_PI*1.5, M_PI*1.75};
        scan_point.resize(8);
        scan_dir.push_back(cv::Point2d(1,0));
        scan_dir.push_back(cv::Point2d(1,1));
        scan_dir.push_back(cv::Point2d(0,1));
        scan_dir.push_back(cv::Point2d(-1,1));
        scan_dir.push_back(cv::Point2d(-1,0));
        scan_dir.push_back(cv::Point2d(-1,-1));
        scan_dir.push_back(cv::Point2d(0,-1));
        scan_dir.push_back(cv::Point2d(1,-1));
    }
    ~Lidar(){}
    void measure(double* position, double theta, const Map& map)
    {
        cv::Point2d pos(position[0], position[1]);
        // 雷达按【米】字形扫八个点
        for(int i = 0; i < scan_dir.size(); i++)
        {
            // 首先计算世界坐标系下的扫描方向
            cv::Point2d dir;
            dir.x = std::cos(theta)*scan_dir[i].x - std::sin(theta)*scan_dir[i].y;
            dir.y = std::sin(theta)*scan_dir[i].x + std::cos(theta)*scan_dir[i].y;
            // 计算哪个地图边与扫描方向相交
            std::vector<cv::Point2d> pots;
            for(int j = 0; j < map.points.size(); j++)
            {
                cv::Point2d dir1 = map.points[j]-pos;
                cv::Point2d dir2 = map.points[j+1]-pos;
                if(cross_product(dir, dir1) * cross_product(dir, dir2) <= 0)
                {
                    // 若相交，计算交点
                    dir1 = map.points[j+1] - map.points[j];
                    Eigen::Matrix<double, 2, 2> A;
                    Eigen::Matrix<double, 2, 1> b;
                    A << dir.y, -dir.x, dir1.y, -dir1.x;
                    b << pos.x*dir.y-pos.y*dir.x, map.points[j].x*dir1.y-map.points[j].y*dir1.x;
                    Eigen::Matrix<double, 2, 1> x = A.colPivHouseholderQr().solve(b);
                    pots.push_back(cv::Point2d(x(0), x(1)));
                }
            }
            double min_dis = 100;
            for(int k = 0; k < pots.size(); k++)
            {
                cv::Point2d dir1 = pots[k] - pos;
                if(product(dir, dir1) < 0) continue;
                double dis = std::sqrt((pos.x-pots[k].x)*(pos.x-pots[k].x)+(pos.y-pots[k].y)*(pos.y-pots[k].y));
                if(dis < min_dis)
                {
                    min_dis = dis;
                    scan_point[i] = pots[k];
                    data[i] = dis;
                }
            }
        }
    }
    std::vector<double> data;
    std::vector<double> angle;
    std::vector<cv::Point2d> scan_dir;
    std::vector<cv::Point2d> scan_point;
};

class Imu
{
    public:
    Imu(){
        acc[0] = 0;
        acc[1] = 0;
        gyro = 0;
    }
    ~Imu(){}
    void add_noise()
    {
        std::normal_distribution<double> noise(0.0, 0.01);
        acc[0] += noise(gen);
        acc[1] += noise(gen);
        gyro += noise(gen);
    }
    double acc[2];
    double gyro;
    std::default_random_engine gen;
};

class State
{
    public:
    State(){
        pos[0] = 0.5;
        pos[1] = 0.5;
        vel[0] = 0;
        vel[1] = 0;
        theta = 0;
        acc_bias[0] = 0;
        acc_bias[1] = 0;
        gyro_bias = 0;
    }
    ~State(){}
    void update(Eigen::Matrix<double, 8, 1> delta)
    {
        pos[0] += delta(0);
        pos[1] += delta(1);
        vel[0] += delta(2);
        vel[1] += delta(3);
        theta += delta(4);
        acc_bias[0] += delta(5);
        acc_bias[1] += delta(6);
        gyro_bias += delta(7);
    }
    double pos[2];
    double vel[2];
    double theta;
    double acc_bias[2];
    double gyro_bias;
};

class Car
{
    public:
    Car(){
        current_time = cv::getTickCount();
        P = Eigen::Matrix<double, 8, 8>::Identity() * 0.001;
        Q = Eigen::Matrix<double, 6, 6>::Identity() * 0.5;
        R = Eigen::Matrix<double, 8, 8>::Identity();
        Fx = Eigen::Matrix<double, 8, 8>::Identity();
        Fn = Eigen::Matrix<double, 8, 6>::Zero();
        H = Eigen::Matrix<double, 8, 8>::Zero();
        memcpy(new_pos, true_state.pos, 2*sizeof(double));
        lidar_points.resize(8);
    }
    ~Car(){}
    void update_pos(double x, double y){
        new_pos[0] = x/100.0;
        new_pos[1] = y/100.0;
    }
    void update_imu()
    {
        last_time = current_time;
        current_time = cv::getTickCount();
        delta_time = (current_time-last_time)/cv::getTickFrequency();
        new_vel[0] = (new_pos[0]-true_state.pos[0])/delta_time;
        new_vel[1] = (new_pos[1]-true_state.pos[1])/delta_time;
        double acc[2];
        acc[0] = (new_vel[0]-true_state.vel[0])/delta_time;
        acc[1] = (new_vel[1]-true_state.vel[1])/delta_time;
        imu.acc[0] = std::cos(true_state.theta)*acc[0] + std::sin(true_state.theta)*acc[1];
        imu.acc[1] = -std::sin(true_state.theta)*acc[0] + std::cos(true_state.theta)*acc[1];
        imu.gyro = (new_theta - true_state.theta)/delta_time;
        memcpy(true_state.vel, new_vel, 2*sizeof(double));
        memcpy(true_state.pos, new_pos, 2*sizeof(double));
        true_state.theta = new_theta;
        imu.add_noise();
    }
    void update_nominal()
    {
        nominal_state.pos[0] += nominal_state.vel[0]*delta_time;
        nominal_state.pos[1] += nominal_state.vel[1]*delta_time;
        double accx = imu.acc[0] - nominal_state.acc_bias[0];
        double accy = imu.acc[1] - nominal_state.acc_bias[1];
        nominal_state.vel[0] += (std::cos(nominal_state.theta)*accx - std::sin(nominal_state.theta)*accy)*delta_time;
        nominal_state.vel[1] += (std::sin(nominal_state.theta)*accx + std::cos(nominal_state.theta)*accy)*delta_time;
        nominal_state.theta += (imu.gyro-nominal_state.gyro_bias)*delta_time;
        // 更新先验协方差
        Eigen::Matrix<double, 2, 1> a;
        Eigen::Matrix<double, 2, 2> R;
        a << -accy, accx;
        R << std::cos(nominal_state.theta), -std::sin(nominal_state.theta), std::sin(nominal_state.theta), std::cos(nominal_state.theta);
        Fx.block<2,2>(0,2) = Eigen::Matrix<double, 2, 2>::Identity() * delta_time;
        Fx.block<2,1>(2,4) = R*a*delta_time;
        Fx.block<2,2>(2,5) = -R*delta_time;
        Fx(4,7) = -delta_time;
        Fn(4,2) = -1;
        Fn.block<3,3>(5,3) = Eigen::Matrix<double, 3, 3>::Identity();
        Fn.block<2,2>(2,0) = -R;
        P = Fx*P*Fx.transpose() + Fn*Q*Fn.transpose();
    }
    void update_estimation(Map& map)
    {
        // 构造H矩阵
        Eigen::Matrix<double, 2, 2> R;
        Eigen::Matrix<double, 2, 1> t;
        R << std::cos(nominal_state.theta), -std::sin(nominal_state.theta), std::sin(nominal_state.theta), std::cos(nominal_state.theta);
        t << nominal_state.pos[0], nominal_state.pos[1];
        for(int i = 0; i < lidar.data.size(); i++)
        {
            // 将点投影到世界坐标系
            Eigen::Matrix<double, 2, 1> p;
            p << lidar.data[i]*std::cos(lidar.angle[i]), lidar.data[i]*std::sin(lidar.angle[i]);
            p = R*p + t;
            // 寻找地图中最近的线段
            cv::Point2d dir;
            double dis = map.find_nearest(cv::Point2d(p(0), p(1)), dir);
            z(i) = dis;
            Eigen::Matrix<double, 1, 2> n;
            n << dir.x, dir.y;
            p << -lidar.data[i]*std::sin(lidar.angle[i]), lidar.data[i]*std::cos(lidar.angle[i]);
            H.block<1,2>(i,0) = n;
            H(i,4) = n*R*p;
        }
        // 计算卡尔曼增益
        Eigen::Matrix<double, 8, 8> I = Eigen::Matrix<double, 8, 8>::Identity();
        K = P*H.transpose()*(H*P*H.transpose()+I).inverse();
        // 更新名义状态
        Eigen::Matrix<double, 8, 1> deltaX = K*z;
        nominal_state.update(deltaX);
        // 更新后验协方差
        P = (I-K*H)*P*(I-K*H).transpose() + K*K.transpose();
        // 可视化点云
        R << std::cos(nominal_state.theta), -std::sin(nominal_state.theta), std::sin(nominal_state.theta), std::cos(nominal_state.theta);
        t << nominal_state.pos[0], nominal_state.pos[1];
        for(int i = 0; i < lidar.data.size(); i++)
        {
            // 将点投影到世界坐标系
            Eigen::Matrix<double, 2, 1> p;
            p << lidar.data[i]*std::cos(lidar.angle[i]), lidar.data[i]*std::sin(lidar.angle[i]);
            p = R*p + t;
            lidar_points[i] = cv::Point2d(p(0), p(1));
        }
    }
    void update_lidar(const Map& map)
    {
        lidar.measure(true_state.pos, true_state.theta, map);
    }
    Lidar lidar;
    Imu imu;
    State true_state;
    State nominal_state;
    State error_state;
    double new_pos[2];
    double new_vel[2];
    double new_theta;
    double current_time;
    double last_time;
    double delta_time;
    Eigen::Matrix<double, 8, 8> P;
    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 8, 8> R;
    Eigen::MatrixXd Fx;
    Eigen::MatrixXd Fn;
    Eigen::Matrix<double, 8, 8> H;
    Eigen::Matrix<double, 8, 1> z;
    Eigen::Matrix<double, 8, 8> K;
    std::vector<cv::Point2d> lidar_points;
};

#endif