//
// Created by goksu on 2/25/20.
//

#include <atomic>
#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <memory>
#include <mutex>
#include <condition_variable>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 512;
    
    int width=sqrt(spp),height = width;
    float step = 1.f/float(width);
    std::cout << "SPP: " << spp << "\n";
    std::mutex tmp_m;
    std::unique_lock<std::mutex> tmp_lock(tmp_m);
    std::condition_variable vari;
    std::atomic_int rest_thread = 6;
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            vari.wait(tmp_lock,[&rest_thread](){return rest_thread>0;});
            --rest_thread;
            //tmp_lock.unlock();
            std::thread t([&,m,i,j](){
                // generate primary ray direction

            for (int k = 0; k < spp; k++){
                float x = (2 * (i + step / 2 + step * (k % width)) / (float)scene.width - 1) *
                    imageAspectRatio * scale;
                float y = (1 - 2 * (j + step / 2 + step * (k / height)) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }

            ++rest_thread;
            vari.notify_one();
            });
            t.detach();
            
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
