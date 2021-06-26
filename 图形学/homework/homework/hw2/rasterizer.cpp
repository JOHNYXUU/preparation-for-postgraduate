// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <cmath>

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


static bool insideTriangle(int x, int y, const Vector3f* _v)
{
    Vector3f ans[3];
    Vector3i res[3];
    // _v[1].x = 1;
    for(int i=0;i<3;i++)
    {
        // std::cout << i<<' '<<_v[i]<<'\n'<<'\n';
        Vector3f a = _v[(i+1)%3]-_v[i];
        Vector3f b(x-_v[i].x(),y-_v[i].y(),0);
        ans[i] = a.cross(b);
        // std::cout << i<<' '<<ans[i]<<'\n';
        float mod = sqrt(ans[i].x()*ans[i].x()+ans[i].y()*ans[i].y()+ans[i].z()*ans[i].z());
        ans[i] /= mod;
        res[i] <<ans[i].x()*100,ans[i].y()*100,ans[i].z()*100;
        // std::cout << i<<' '<<ans[i]<<'\n';
    }
    return res[0]==res[1]&&res[1]==res[2];
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
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
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
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
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    // t.v
    // Vector3f V[3] = t.v;
    // for(int i=0;i<3;i++)
    // {
    //     // std::cout << v[i].x()<<' '<<v[i].y()<<' '<<v[i].z()<<'\n';
    //     V[i]<<v[i].x(),v[i].y(),v[i].z();
    // }

    // std::cout << min_x<< ' ' << max_x<<' '<<min_y<<' '<<max_y<<'\n';
    int upbound = std::max(std::max(t.v[0].y(),t.v[1].y()),t.v[2].y());
    int lowbound = std::min(std::min(t.v[0].y(),t.v[1].y()),t.v[2].y());
    int leftbound = std::min(std::min(t.v[0].x(),t.v[1].x()),t.v[2].x());
    int rightbound = std::max(std::max(t.v[0].x(),t.v[1].x()),t.v[2].x());
    bool msaa = false;
    if(msaa)
    {
        for(int i=leftbound;i<=rightbound;i++)
            for(int j=lowbound;j<=upbound;j++)
            {
                int cnt = 0;
                float dir[4][2] ={{0.25,0.25},{0.25,0.75},{0.75,0.25},{0.75,0.75}};
                for(int k=0;k<4;k++)
                {
                    float x = i+dir[k][0];
                    float y = j+dir[k][1];
                    if(insideTriangle(x,y,t.v))
                        cnt++;
                }
                // std::cout << x << ' '<<y<<' '<<insideTriangle(x,y,V)<<'\n';
                if(cnt)
                {
                    float alpha, beta, gamma;
                    std::tuple<float,float,float> tup = computeBarycentric2D(i+0.5, j+0.5, t.v);
                    // alpha = std::get<0>(tmp);
                    // beta = std::get<1>(tmp);
                    // gamma = std::get<2>(tmp);
                    std::tie(alpha, beta, gamma) = tup;
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if(abs(z_interpolated)<depth_buf[get_index(i,j)])
                    {
                        depth_buf[get_index(i,j)] = z_interpolated;
                        Eigen::Vector3f p;
                        p<<(float)i,(float)j,z_interpolated;
                        set_pixel(p,t.getColor()*(cnt/4.0));
                    }
                }
            }
    }
    else
    {
        for(int i=leftbound;i<=rightbound;i++)
            for(int j=lowbound;j<=upbound;j++)
            {
                float x = i+0.5;
                float y = j+0.5;
                // std::cout << x << ' '<<y<<' '<<insideTriangle(x,y,V)<<'\n';
                if(insideTriangle(x,y,t.v))
                {
                    // std::cout << x<<' '<<y<<std::endl;
                    float alpha, beta, gamma;
                    std::tuple<float,float,float> tup = computeBarycentric2D(x, y, t.v);
                    // alpha = std::get<0>(tmp);
                    // beta = std::get<1>(tmp);
                    // gamma = std::get<2>(tmp);
                    std::tie(alpha, beta, gamma) = tup;
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if(abs(z_interpolated)<depth_buf[get_index(i,j)])
                    {
                        depth_buf[get_index(i,j)] = z_interpolated;
                        Eigen::Vector3f p;
                        p<<(float)i,(float)j,z_interpolated;
                        set_pixel(p,t.getColor());
                    }
                }
            }
    }
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on