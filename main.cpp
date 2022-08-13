#include "world.h"

int map_width = 200;
int map_height = 200;

bool mouse_move = false;
void mouse_callback(int event,int x,int y,int flags,void *ustc)
{
    
    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:
        {
            mouse_move = true;
            break;
        }
        case cv::EVENT_MOUSEMOVE:
        {
            if(x >= map_width) break;
            if(mouse_move)
            {
                Car* car = (Car*)ustc;
                car->update_pos(x, map_height-y);
            }
            break;
        }
        case cv::EVENT_LBUTTONUP:
        {
            mouse_move = false;
            break;
        }
    default:
        break;
    }
}

int main()
{
    // 初始化地图
    Map map(map_width, map_height);
    // 加入一辆车
    Car car;
    cv::namedWindow("viz");
    cv::setMouseCallback("viz", mouse_callback, (void*)&car);
    cv::Mat map_img = cv::Mat(map_height, map_width*2, CV_8UC3, cv::Scalar(255,255,255));
    cv::line(map_img, cv::Point(map_width,0), cv::Point(map_width, map_height), cv::Scalar(0,0,0), 1);
    for(int i = 0; i < map.points.size()-1; i++)
    {
        cv::Point p1 = cv::Point(map.points[i].x*100, map_height-map.points[i].y*100);
        cv::Point p2 = cv::Point(map.points[i+1].x*100, map_height-map.points[i+1].y*100);
        cv::line(map_img, p1, p2, cv::Scalar(0,255,0), 2);
        cv::line(map_img, p1+cv::Point(map_width, 0), p2+cv::Point(map_width, 0), cv::Scalar(0,255,0), 2);
    }
    double current_time;
    double last_time = cv::getTickCount();
    while(true)
    {
        // Step 1. 更新IMU测量
        car.update_imu();
        // Step 2. 更新先验协方差，积分名义状态
        car.update_nominal();
        // Step 3. 10Hz频率更新雷达测量
        if(((double)cv::getTickCount() - last_time)/cv::getTickFrequency() > 0.1)
        {
            last_time = cv::getTickCount();
            car.update_lidar(map);
            // Step 4. 测量更新
            car.update_estimation(map);
        }
        
        // 把结果可视化
        cv::Mat show_map = map_img.clone();
        cv::Point2d p1(car.true_state.pos[0]*100, map_height-car.true_state.pos[1]*100);
        cv::Point2d p2 = p1 + 30*cv::Point2d(std::cos(car.true_state.theta), -std::sin(car.true_state.theta));
        cv::arrowedLine(show_map, p1, p2, cv::Scalar(0,0,255), 2);
        cv::circle(show_map, p1, 5, cv::Scalar(255,0,0), 2);
        for(int i = 0; i < car.lidar.scan_point.size(); i++)
        {
            cv::Point2d p = car.lidar.scan_point[i];
            cv::circle(show_map, cv::Point(p.x*100, map_height-p.y*100), 3, cv::Scalar(255,255,0), 2);
            p = car.lidar_points[i];
            cv::circle(show_map, cv::Point(map_width+p.x*100, map_height-p.y*100), 3, cv::Scalar(255,0,255), 2);
        }
        p1 = cv::Point2d(map_width+car.nominal_state.pos[0]*100, map_height-car.nominal_state.pos[1]*100);
        p2 = p1 + 30*cv::Point2d(std::cos(car.nominal_state.theta), -std::sin(car.nominal_state.theta));
        cv::arrowedLine(show_map, p1, p2, cv::Scalar(0,0,255), 2);
        cv::circle(show_map, p1, 5, cv::Scalar(255,0,0), 2);
        cv::imshow("viz", show_map);
        int k = cv::waitKey(1);
        if(k == 'q')
            car.new_theta += 0.1;
        else if(k == 'e')
            car.new_theta -= 0.1;
        else if(k == 27)
            break;
    }
    return 0;
}