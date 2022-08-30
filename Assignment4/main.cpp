#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ostream>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> next_control_points;
    for(int i = 0,n = control_points.size(); i < n-1;   i++){
        next_control_points.emplace_back(control_points[i]+(control_points[i+1]-control_points[i])*t);
    }
    if(next_control_points.size() == 1) return next_control_points[0];
    return recursive_bezier(next_control_points, t);    

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(float t = 0; t < 1; t += 0.0001f){
        auto point = recursive_bezier(control_points, t);
        auto true_point = point;
        point = cv::Point2f(cvCeil(point.x), cvCeil(point.y));
        cv::Point2f lb(point.x,point.y);
        cv::Point2f rb(point.x+1,point.y);
        cv::Point2f lt(point.x,point.y+1);
        cv::Point2f rt(point.x+1,point.y+1);
        window.at<cv::Vec3b>(rb.y,rb.x)[2] += 255*sqrt((rb-true_point).dot(rb-true_point));
        window.at<cv::Vec3b>(rb.y,rb.x)[2] = std::min(window.at<cv::Vec3b>(rb.y,rb.x)[2],(unsigned char)255);
        window.at<cv::Vec3b>(lb.y,lb.x)[2] +=255*sqrt((lb-true_point).dot(lb-true_point));
        window.at<cv::Vec3b>(lb.y,lb.x)[2] = std::min(window.at<cv::Vec3b>(lb.y,lb.x)[2],(unsigned char)255);
        window.at<cv::Vec3b>(lt.y,lt.x)[2] +=255*sqrt((lt-true_point).dot(lt-true_point));
        window.at<cv::Vec3b>(lt.y,lt.x)[2] = std::min(window.at<cv::Vec3b>(lt.y,lt.x)[2],(unsigned char)255);
        window.at<cv::Vec3b>(rt.y,rt.x)[2] +=255*sqrt((rt-true_point).dot(rt-true_point));
        window.at<cv::Vec3b>(rt.y,rt.x)[2] = std::min(window.at<cv::Vec3b>(rt.y,rt.x)[2],(unsigned char)255);
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
