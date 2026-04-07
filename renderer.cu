#include "api.h"

namespace penguinPT {
    __shared__ uint8_t* device_pixel_buffer;
    __shared__ nanovdb::Vec3f* device_accum_buffer;
    nanovdb::Vec3f* host_accum_buffer;
    
    void initCuda(renderer_services rs) {
        if (rs.CUDA_CAPABLE_GPU) {
            CUDA_CHECK(cudaMalloc((void**)&device_pixel_buffer, rs.width * rs.height * 4 * sizeof(uint8_t)));
            CUDA_CHECK(cudaMalloc((void**)&device_accum_buffer, rs.width * rs.height * sizeof(nanovdb::Vec3f)));
            printf("CUDA INITIALIZED\n\n");
        }
        else {
            host_accum_buffer = (nanovdb::Vec3f*)malloc(rs.width * rs.height * sizeof(nanovdb::Vec3f));
        }
    }
    void endCuda(renderer_services rs) {
        if (rs.CUDA_CAPABLE_GPU) {
            CUDA_CHECK(cudaFree(device_pixel_buffer));
            CUDA_CHECK(cudaFree(device_accum_buffer));
            printf("CUDA ENDED\n");
        }
        else {
            free(host_accum_buffer);
        }
    }

    // kernel
    __device__ nanovdb::Vec3f getColor(renderer_services& rs, nanovdb::math::Ray<float> ray, float2 uv) {
        hit_info info;
        info.t = 1e10f;
        
        if(rs.is_rendering)
            return volume_pathtrace_device_spectral(rs, ray);
        if (rs.scene.intersectScene_full(ray, info)) {
            return info.normal.dot(-ray.dir()) * rs.scene.bsdf_list[info.BSDF_index].albedo;
        }
        float pdf_e;
        return (rs.scene.environnement_map.eval_envmap(ray.dir(), pdf_e));
    }
    __host__ nanovdb::Vec3f getColor_host(renderer_services& rs, nanovdb::math::Ray<float> ray, float2 uv) {
        hit_info info;
        info.t = 1e10f;
        
        if (rs.is_rendering)
            return volume_pathtrace_host_spectral(rs, ray);

        if (rs.scene.intersectScene_full(ray, info)) {
            return info.normal.dot(-ray.dir()) * rs.scene.bsdf_list[info.BSDF_index].albedo;
        }
        float pdf_e;
        return (rs.scene.environnement_map.eval_envmap_host(ray.dir(), pdf_e));
    }
    __global__ void renderFrame_kernel(renderer_services rs, uint8_t* dev_px_buffer, nanovdb::Vec3f* dev_acc_buffer) {
        unsigned int x = threadIdx.x + blockIdx.x * blockDim.x;
        unsigned int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x > rs.width || y > rs.height) return; // check if no out of window

        int idx = x + y * rs.width;

        curand_init(idx, 0, 4096 * rs.frame_index, &rs.rng_state);

        float blur_size = 0.001f;
        float uvX = (float)x / (float)rs.width + (randC(&rs.rng_state) * 2.f - 1.f) * blur_size;
        float uvY = 1.f - (float)y / (float)rs.height + (randC(&rs.rng_state) * 2.f - 1.f) * blur_size;
        float uvX_cam = uvX - 0.5f;
        float uvY_cam = uvY - 0.5f;
        uvX_cam *= (float)rs.width / (float)rs.height;

        nanovdb::Vec3f cameraTarget = { sinf(rs.mainCam.angles[0]) * cosf(rs.mainCam.angles[1]), sinf(rs.mainCam.angles[1]), -cosf(rs.mainCam.angles[0]) * cosf(rs.mainCam.angles[1]) };
        nanovdb::Vec3f ww = cameraTarget.normalize();
        nanovdb::Vec3f uu = ww.cross(nanovdb::Vec3f(0.f, 1.f, 0.f)).normalize();
        nanovdb::Vec3f vv = uu.cross(ww).normalize();

        nanovdb::Vec3f rayDirection = (uvX_cam * uu + uvY_cam * vv + rs.mainCam.zoom * ww).normalize();
        nanovdb::math::Ray<float> first_ray = { rs.mainCam.position, rayDirection };

        nanovdb::Vec3f color = getColor(rs, first_ray, make_float2(uvX, uvY));

        if (rs.frame_index == 0)
            dev_acc_buffer[idx] = color;
        else dev_acc_buffer[idx] += color;

        color = dev_acc_buffer[idx] / float(rs.frame_index + 1);

        // post processing
        color = util::mix(nanovdb::Vec3f(util::luminance(color)), color, rs.mainCam.saturation);
        color = util::mix(nanovdb::Vec3f(0.5f), color, rs.mainCam.constrast);
        color = color * rs.mainCam.exposure * rs.mainCam.multiplier;

        float vignette_factor = expf(-sqrtf(uvX_cam * uvX_cam + uvY_cam * uvY_cam) * 2.f);
        color *= util::mix(1.f, vignette_factor, rs.mainCam.vignette);

        //color = util::AgX_tonemap(color);
        color = util::aces(color);

        // writing to buffer
        int R_255 = CLAMP(color[0] * 255, 0, 255);
        int G_255 = CLAMP(color[1] * 255, 0, 255);
        int B_255 = CLAMP(color[2] * 255, 0, 255);

        dev_px_buffer[idx * 4] = (uint8_t)R_255;
        dev_px_buffer[idx * 4 + 1] = (uint8_t)G_255;
        dev_px_buffer[idx * 4 + 2] = (uint8_t)B_255;
        dev_px_buffer[idx * 4 + 3] = (uint8_t)255;
        return;
    }

    inline void callKernel(renderer_services& rs) {
        dim3 blockSize(8, 8, 1U);
        dim3 gridSize(int(rs.width / blockSize.x), int(rs.height / blockSize.y), 1U);
        renderFrame_kernel <<< gridSize, blockSize >> > (rs, device_pixel_buffer, device_accum_buffer);
    }

    inline void renderFrame(renderer_services& rs, sf::Texture* dsp_text) {
        if (rs.CUDA_CAPABLE_GPU) {
            callKernel(rs);

            CUDA_CHECK(cudaMemcpy(rs.host_pixel_buffer, device_pixel_buffer, rs.width * rs.height * 4 * sizeof(uint8_t), cudaMemcpyDeviceToHost));
        }
        else {
            for (int x = 0; x < rs.width; x++) {
                for (int y = 0; y < rs.height; y++) {
                    
                    int idx = x + y * rs.width;

                    float blur_size = 0.001f;
                    float uvX = (float)x / (float)rs.width + (rand01 * 2.f - 1.f) * blur_size;
                    float uvY = 1.f - (float)y / (float)rs.height + (rand01 * 2.f - 1.f) * blur_size;
                    float uvX_cam = uvX - 0.5f;
                    float uvY_cam = uvY - 0.5f;
                    uvX_cam *= (float)rs.width / (float)rs.height;
                    
                    nanovdb::Vec3f cameraTarget = { sinf(rs.mainCam.angles[0]) * cosf(rs.mainCam.angles[1]), sinf(rs.mainCam.angles[1]), -cosf(rs.mainCam.angles[0]) * cosf(rs.mainCam.angles[1]) };
                    nanovdb::Vec3f ww = cameraTarget.normalize();
                    nanovdb::Vec3f uu = ww.cross(nanovdb::Vec3f(0.f, 1.f, 0.f)).normalize();
                    nanovdb::Vec3f vv = uu.cross(ww).normalize();

                    nanovdb::Vec3f rayDirection = (uvX_cam * uu + uvY_cam * vv + rs.mainCam.zoom * ww).normalize();
                    nanovdb::math::Ray<float> first_ray = { rs.mainCam.position, rayDirection };

                    nanovdb::Vec3f color = getColor_host(rs, first_ray, make_float2(uvX, uvY));

                    if (rs.frame_index == 0)
                        host_accum_buffer[idx] = color;
                    else host_accum_buffer[idx] += color;

                    color = host_accum_buffer[idx] / float(rs.frame_index + 1);

                    // post processing
                    color = util::mix(nanovdb::Vec3f(util::luminance(color)), color, rs.mainCam.saturation);
                    color = util::mix(nanovdb::Vec3f(0.5f), color, rs.mainCam.constrast);
                    color = color * rs.mainCam.exposure * rs.mainCam.multiplier;

                    float vignette_factor = expf(-sqrtf(uvX_cam * uvX_cam + uvY_cam * uvY_cam) * 2.f);
                    color *= util::mix(1.f, vignette_factor, rs.mainCam.vignette);

                    //color = util::AgX_tonemap(color);
                    color = util::aces(color);
                    
                    // writing to buffer
                    int R_255 = CLAMP(color[0] * 255, 0, 255);
                    int G_255 = CLAMP(color[1] * 255, 0, 255);
                    int B_255 = CLAMP(color[2] * 255, 0, 255);
                    
                    rs.host_pixel_buffer[idx * 4] = (uint8_t)R_255;
                    rs.host_pixel_buffer[idx * 4 + 1] = (uint8_t)G_255;
                    rs.host_pixel_buffer[idx * 4 + 2] = (uint8_t)B_255;
                    rs.host_pixel_buffer[idx * 4 + 3] = (uint8_t)255;
                }
            }
        }
        dsp_text->update(rs.host_pixel_buffer);
    }

    void save_screenshot(sf::Texture& tex) {
        std::time_t result = std::time(nullptr);

        char buffer[26];
        
        time(&result);
        ctime_s(buffer, sizeof(buffer), &result);

        std::string name;
        for (int i = 0; i < strlen(buffer); i++) { // avoid line return character
            if (buffer[i] == ' ' || buffer[i] == ':') name += '_';
            else if (buffer[i] == '\n') break;
            else name += buffer[i];
        }

        sf::Image saver = tex.copyToImage();
        saver.saveToFile("saves/screenshots/" + name + ".png");
        std::cout << "Image saved as" << name << ".png\n";
    }
    extern "C" void run() 
    {
        renderer_services rs;
        rs.check_CUDA_AVAILABLITY();
        rs.mainCam.speed = 20.f;
        rs.mainCam.zoom = 1.f;
        rs.mainCam.position = { 0.f, 30.f, 15.f };
        rs.is_rendering = false;

        rs.fill_host_pixel_buffer();
        initCuda(rs);

        loader::obj_loader loader01;
        loader::nanovdb_loader loader02;
        loader::envmap_loader loader03;
        loader::usd_scene_loader loader04;
        loader::BSDF_loader loader05;

        //loader04.load_usd_scene("untitled.usda");

        loader05.create_new_BSDF("floor", 1.f, nanovdb::Vec3f(0.4f, 0.3f, 0.1f), nanovdb::Vec3f(0.f), nanovdb::Vec3f(0.f), 0.f, 1.5f, 0.f);

        //loader05.create_new_BSDF("gold", 0.1f, nanovdb::Vec3f(0.8f, 0.4f, 0.05f), nanovdb::Vec3f(0.f), nanovdb::Vec3f(1.f), 1.f, 1.47f, 0.f);
        
        loader05.create_new_BSDF("jade", 0.05, nanovdb::Vec3f(0.7f, 1.f, 0.5f), nanovdb::Vec3f(0.f), nanovdb::Vec3f(0.2f, 0.f, 0.2f), 0.f, 1.640f, 0.8f);
        loader05.BSDF_list.at(loader05.gifn("jade")).g = -0.8f;
        loader05.BSDF_list.at(loader05.gifn("jade")).scattering = 4.f;
        
        loader05.create_new_BSDF("glass", 0.f, nanovdb::Vec3f(1.f), nanovdb::Vec3f(0.f), nanovdb::Vec3f(0.f), 0.f, 2.418f, 1.f);
        loader05.create_new_BSDF("white", { 0.6f, 0.6f, 0.6f }, 0.1f);
                
        //loader01.load_obj("just_a_plane.obj", { 0.f, -10.f, 0.f }, 500.f, loader05.gifn("floor"));
        //loader01.load_obj("diamond.obj", { 0.f, 0.05f, 50.f }, 10.f, loader05.gifm("glass"));
        //loader01.load_obj("horse_statue.obj", { 50.f, 0.05f, 0.f }, 200.f, loader05.gifn("white"));
        //loader01.load_obj("sphere.obj", { 50.f, 15.f, 0.f }, 10.f, loader05.gifn("glass"));
        loader01.load_obj("ship.obj", { 0.f, 0.f, 0.f }, 10.f, loader05.gifn("white"));
        
        loader01.send_to_scene(rs.scene);
        loader05.send_to_scene(rs.scene);
        
        rs.scene.build_BVH();
        
        // ENVMAP
        loader03.load_from_file("assets/hdris/qwantani_late_afternoon_2k.hdr");
        rs.scene.environnement_map.strength = 1.f;
        loader03.send_to_gpu(rs.scene.environnement_map, cudaFilterModeLinear);

        //loader02.load_nvdb("volume.nvdb", rs.CUDA_CAPABLE_GPU);
        //loader02.load_nvdb("volume.nvdb",rs.CUDA_CAPABLE_GPU);
        //loader02.load_nvdb("smoke2.nvdb", rs.CUDA_CAPABLE_GPU);
        
        //loader02.volume_parameters(1, nanovdb::Vec3f(0.5f), nanovdb::Vec3f(0.5f), -0.4f, nanovdb::Vec3f(0.f));
        //loader02.set_tranforms(1, 0.1f, nanovdb::Vec3f(0.f, 20.f, 0.f));
        
        loader02.volume_parameters(0, nanovdb::Vec3f(0.5f), nanovdb::Vec3f(0.5f), -0.4f, nanovdb::Vec3f(0.f));
        loader02.set_tranforms(0, 0.1f, nanovdb::Vec3f(0.f, 0.f, 0.f));

        //loader02.volume_parameters(0, nanovdb::Vec3f(20.f), nanovdb::Vec3f(0.3f), 0.7f, nanovdb::Vec3f(0.), 20.f);
        //loader02.set_tranforms(0, 0.8f, nanovdb::Vec3f(0.f, 3.f, 0.f));

        loader02.send_to_scene(rs.scene);
       
        rs.send_to_GPU_data();
        
        sf::RenderWindow window(sf::VideoMode(rs.width, rs.height), "PenguinPT, v0.0");
        
        sf::Texture displayTex;
        displayTex.create(rs.width, rs.height);
        displayTex.update(rs.host_pixel_buffer);
        sf::Sprite diplaySprite(displayTex);

        sf::Clock mainClock;

        while (window.isOpen())
        {
            sf::Event event;
            while (window.pollEvent(event))
            {
                if (event.type == sf::Event::Closed)
                    window.close();
            }

            rs.mainCam.move(&window, rs.delta_time, &rs.frame_index);
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::M)) {
                save_screenshot(displayTex);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Multiply)) {
                rs.is_rendering = true;
                rs.frame_index = 0;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Divide)) {
                rs.is_rendering = false;
                rs.frame_index = 0;
            }

            if (!rs.is_rendering) rs.frame_index = 0;

            renderFrame(rs, &displayTex);
            
            window.clear();
            window.draw(diplaySprite);
            window.display();

            rs.delta_time = mainClock.restart().asSeconds();
            //gotoxy(1, 9);
            float fps = 1.f / rs.delta_time;
            //printf("FPS : %f\n", fps);
            rs.frame_index++;
        }

        rs.clean();
        loader03.clean();

        loader02.clean();
        endCuda(rs);
    }
}