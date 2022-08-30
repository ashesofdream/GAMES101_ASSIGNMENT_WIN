#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float radius = rotation_angle*MY_PI/180;
    Eigen::Matrix4f translate;
    translate << cos(radius),-sin(radius),0,0,
                 sin(radius),cos(radius),0,0,
                 0,0,1,0,
                 0,0,0,1;
    model =  translate * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f translate;
    translate << zNear,0,0,0,
                 0,zNear,0,0,
                 0,0,zNear+zFar,-zNear*zFar,
                 0,0,1,0;
    projection = translate*projection;

    auto height = abs(zNear)*tan(eye_fov/2);
    auto width = height*aspect_ratio;
    translate <<1,0,0,0,
                0,1,0,0,
                0,0,1,-(zNear+zFar)/2,
                0,0,0,1;
    projection = translate*projection;
    translate <<1/width,0,0,0,
                0,1/height,0,0,
                0,0,2/(zNear-zFar),0,
                0,0,0,1;
    projection = translate*projection;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle){
    float radius = angle*MY_PI/180;

    Eigen::Matrix3f translate;
    translate << 0, -axis[2], axis[1],
             axis[2], 0, -axis[0],
            -axis[1], axis[0], 0;
    translate = sin(radius)*translate;
    Eigen::Matrix3f part2 = (1- cos(radius))*axis*axis.transpose();
    Eigen::Matrix3f part3 = cos(radius)*Eigen::Matrix3f::Identity();
    Eigen::Matrix3f ret = part3+part2+translate;
    Eigen::Matrix4f ret4;
    ret4.setZero();
    ret4.block(0,0,2,2) = ret.block(0,0,2,2);
    ret4(3,3) = 1;
    return ret4;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation({1,0,0},angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation({0,0,1},angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
