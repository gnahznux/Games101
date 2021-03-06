// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

auto to_vec2(const Eigen::Vector3f& v3){
    return Vector2f(v3.x(), v3.y());
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f P(x, y, 1);
    Vector3f AB(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 1);
    Vector3f BC(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 1);
    Vector3f CA(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 1);
    Vector3f AP(P.x() - _v[0].x(), P.y() - _v[0].y(), 1);
    Vector3f BP(P.x() - _v[1].x(), P.y() - _v[1].y(), 1);
    Vector3f CP(P.x() - _v[2].x(), P.y() - _v[2].y(), 1);
    float z1 = AB.cross(AP).z();
    float z2 = BC.cross(BP).z();
    float z3 = CA.cross(CP).z();
    if ((z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0)){
        return true;
    }else{
        return false;
    }   
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5* 2 * width*(vert.x()+1.0);
            vert.y() = 0.5* 2 * height*(vert.y()+1.0);
            // vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
    set_result_buffer_pixel();
}

void rst::rasterizer::set_msaa(int value){
    
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    std::cout<<"rasterize_triangle"<<std::endl;
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int x_min = INT_MAX, x_max = INT_MIN, y_min =INT_MAX, y_max = INT_MIN;
    for (int i = 0; i < 3; i++){
        if (t.v[i].x() < x_min){
            x_min = t.v[i].x();
        }
        if (t.v[i].x() > x_max){
            x_max = t.v[i].x();
        }
        if (t.v[i].y() < y_min){
            y_min = t.v[i].y();
        }
        if (t.v[i].y() > y_max){
            y_max = t.v[i].y();
        }
    }
    for (int i = x_min; i <= x_max; i++){
        for (int j = y_min; j <= y_max; j++){
            if (insideTriangle(i, j, t.v)){
                auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if (z_interpolated < depth_buf[get_index(i, j)]){
                    depth_buf[get_index(i,j)] = z_interpolated;
                    set_pixel(Vector3f(i, j, 1),  t.getColor());
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), background_color);
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h, int msaa) : width(w), height(h), msaa(msaa)
{
    frame_buf.resize(msaa * w * h);
    depth_buf.resize(msaa * w * h);
    result_frame_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    auto ind = (2*height-1-y)*2*width + x;
    return ind;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    auto ind = (2*height-1-point.y())*2*width + point.x();
    if (ind > msaa * width * height){
        std::cout << "111" << std::endl;
    }
    frame_buf[ind] = color;
}

void rst::rasterizer::set_result_buffer_pixel()
{
    std::cout << "set_result_buffer_pixel" << std::endl;
    int m = 2*width;
    int size = frame_buf.capacity();
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            int k = i * width + j;
            int a = 2 * (ceil(2.0f*(k + 1)/(float)m) - 1) + 1;
            int b = 2 * (k % 2);
            int index = (a - 1) * m + b;
            if (index >= size){
                std::cout<<"index:"<<index<<std::endl;
            }
            if (index + 1 >= size){
                std::cout<<"index + 1"<<index + 1<<std::endl;
            }
            if (index + m >= size){
                std::cout<<"index + m"<<index + m<<std::endl;
            }
            if (index + 1 + m >= size){
                std::cout<<"index + 1 + m"<<index + 1 + m<<std::endl;
            }
            if (k >= result_frame_buf.capacity()){
                std::cout << k << std::endl;
            }
            result_frame_buf[k] = (frame_buf[index] + frame_buf[index + 1] + frame_buf[index + m] + frame_buf[index + 1 + m])/4.0f;
        }
        
    }
    
}

// clang-format on
