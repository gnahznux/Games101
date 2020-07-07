#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

int x_angle, y_angle, z_angle, x_offset, y_offset, z_offset;

Eigen::Vector3f angle(0.0f, 0.0f, 0.0f);
Eigen::Vector3f scales(1.0f, 1.0f, 1.0f);
Eigen::Vector3f offset(0.0f, 0.0f, 0.0f);

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(Eigen::Vector3f euler_angle, Eigen::Vector3f scales, Eigen::Vector3f offset)
{
    Matrix4f result;
    euler_angle = euler_angle * MY_PI / 180.f;
    Matrix4f rotation_x, rotation_y, rotation_z;
    rotation_x <<
               1, 0, 0, 0,
            0, cos(euler_angle[0]), -sin(euler_angle[0]), 0,
            0, sin(euler_angle[0]), cos(euler_angle[0]), 0,
            0, 0, 0, 1;
    rotation_y <<
               cos(euler_angle[1]) , 0, sin(euler_angle[1]), 0,
            0, 1, 0, 0,
            -sin(euler_angle[1]), 0, cos(euler_angle[1]), 0,
            0, 0, 0, 1;
    rotation_z <<
               cos(euler_angle[2]), -sin(euler_angle[2]), 0, 0,
            sin(euler_angle[2]), cos(euler_angle[2]), 0, 0  ,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Matrix4f scale;
    scale <<
          scales[0], 0, 0, 0,
            0, scales[1], 0, 0,
            0, 0, scales[2], 0,
            0, 0, 0, 1;
    Matrix4f translate;
    translate <<
              1, 0, 0, offset[0],
            0, 1, 0, offset[1],
            0, 0, 1, offset[2],
            0, 0, 0, 1;
    return translate * (rotation_z * rotation_y * rotation_x) * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

     projection << 
     -1.0f/(aspect_ratio*tan(eye_fov/360.0f * MY_PI)), 0.0f, 0.0f, 0.0f,
     0.0f,   -1.0f/tan(eye_fov/360.0f * MY_PI),     0.0f,     0.0f,
     0.0f,     0.0f,     (zFar+zNear)/(zNear-zFar),     2*zFar*zNear/(zFar-zNear),
     0.0f,     0.0f,     1.0f,     0.0f;

    return projection;
}

void slider_value_change_callback(int pos, void* userdata){
    angle[0] = x_angle;
    angle[1] = y_angle;
    angle[2] = z_angle;
    offset[0] = x_offset;
    offset[1] = y_offset;
    offset[2] = z_offset;
}



int main(int argc, const char** argv)
{

    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
//        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
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

        r.set_model(get_model_matrix(angle, scales, offset));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    std::string angle_slider = "x_angle_slider";
    cv::namedWindow(angle_slider);
    cv::createTrackbar("x_angle_slider", angle_slider, &x_angle, 360,slider_value_change_callback);
    cv::createTrackbar("y_angle_slider", angle_slider, &y_angle, 360,slider_value_change_callback);
    cv::createTrackbar("z_angle_slider", angle_slider, &z_angle, 360,slider_value_change_callback);
    cv::createTrackbar("x_offset_slider", angle_slider, &x_offset, 10,slider_value_change_callback);
    cv::createTrackbar("y_offset_slider", angle_slider, &y_offset, 10,slider_value_change_callback);
    cv::createTrackbar("z_offset_slider", angle_slider, &z_offset, 10,slider_value_change_callback);

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, scales, offset));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        if (key == 'q'){
            angle[0] += 10;
        }
        if (key == 'e'){
            angle[0] -= 10;
        }
        if (key == 'a') {
            angle[1] += 10;
        }
        if (key == 'd') {
            angle[1] -= 10;
        }
        if (key == 'z'){
            angle[2] += 10;
        }
        if (key == 'c'){
            angle[2] -= 10;
        }


    }

    return 0;
}
