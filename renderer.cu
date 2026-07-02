// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#include <SFML/Graphics.hpp>

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>

#include <ppt/core/renderer_services.h>
#include <ppt/core/pathtracer.h>
#include <ppt/core/envmap.h>
#include <ppt/loaders/obj_loader.h>
#include <ppt/loaders/nanovdb_loader.h>
#include <ppt/loaders/SDPT_loader.h>
#include <ppt/wip/usd_scene_loader.h>
#include <ppt/user/gui/slider.h>

#include <ppt/core/Mesh.h>
#include <ppt/util/Matrix.h>
#include <ppt/core/mesh_manager.h>

#include <ppt/loaders/BVH_loader.h>
#include <ppt/user/gui/options_interface.h>

#include <ppt/core/application.h>

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
        
        if(rs.is_rendering || rs.first_frame_mode)
            return volume_pathtrace_device_spectral(rs, ray);
        //if (rs.scene.intersectScene_full(ray, info)) {
        if (rs.scene.intersectScene(ray, info)) {
            principled_BSDF& surface_bsdf = rs.scene.bsdf_list[info.BSDF_index];

            // normal modification
            nanovdb::Vec3f T, B;
            util::Onb(info.normal, T, B);
            return util::ToWorld(T, B, info.normal, surface_bsdf.getNormal_textured_CUDA(info.uv) * 2.f - nanovdb::Vec3f(1.f));
        }
        float pdf_e;
        return (rs.scene.environnement_map.eval_envmap(ray.dir(), pdf_e));
    }
    __host__ nanovdb::Vec3f getColor_host(renderer_services& rs, nanovdb::math::Ray<float> ray, float2 uv) {
        hit_info info;
        info.t = 1e10f;
        
        if (rs.is_rendering)
            return volume_pathtrace_host_spectral(rs, ray);

        //if (rs.scene.intersectScene_full(ray, info)) {
        if (rs.scene.intersectScene(ray, info)) {
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

        float blur_size = ANTIALIAS_SIZE;
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

        nanovdb::Vec3f camera_origin = rs.mainCam.position;
        if (rs.mainCam.DOF_strength > 0.f) {
            float RdoT = ww.dot(rayDirection);
            nanovdb::Vec3f target = rs.mainCam.position + rayDirection * rs.mainCam.focal_distance / RdoT;
            camera_origin += util::generateUniformSample(rs.rng_state) * rs.mainCam.DOF_strength;
            rayDirection = (target - camera_origin).normalize();
        }

        nanovdb::math::Ray<float> first_ray = { camera_origin, rayDirection };

        nanovdb::Vec3f color = getColor(rs, first_ray, make_float2(uvX, uvY));

        if (isnan(color[0]) || isnan(color[1]) || isnan(color[2])) return;

        if (rs.frame_index == 0)
            dev_acc_buffer[idx] = color;
        else dev_acc_buffer[idx] += color;

        color = dev_acc_buffer[idx] / float(rs.frame_index + 1);

        // post processing
        
        color = util::mix(nanovdb::Vec3f(util::luminance(color)), color, rs.mainCam.saturation);
        color = util::mix(nanovdb::Vec3f(0.5f), color, rs.mainCam.contrast);
        color = color * rs.mainCam.exposure * rs.mainCam.multiplier;

        float vignette_factor = expf(-sqrtf(uvX_cam * uvX_cam + uvY_cam * uvY_cam) * 2.f);
        color *= util::mix(1.f, vignette_factor, rs.mainCam.vignette);

        //color = util::AgX_tonemap(color);
        color = util::aces(color);

        // writing to buffer
        int R_255 = CLAMP(color[0] * 256, 0, 255);
        int G_255 = CLAMP(color[1] * 256, 0, 255);
        int B_255 = CLAMP(color[2] * 256, 0, 255);

        dev_px_buffer[idx * 4] = (uint8_t)R_255;
        dev_px_buffer[idx * 4 + 1] = (uint8_t)G_255;
        dev_px_buffer[idx * 4 + 2] = (uint8_t)B_255;
        dev_px_buffer[idx * 4 + 3] = (uint8_t)255;
        return;
    }

    inline void callKernel(renderer_services& rs) {
        dim3 blockSize(8, 8, 1U);
        dim3 gridSize(int(rs.width / blockSize.x), int(rs.height / blockSize.y), 1U);

        void* args[] = { &rs, &device_pixel_buffer, &device_accum_buffer };
        cudaLaunchKernel((const void*)renderFrame_kernel, gridSize, blockSize, args);
    }

    inline void renderFrame(renderer_services& rs, sf::Texture* dsp_text) {
        if (rs.CUDA_CAPABLE_GPU) {
            callKernel(rs);

            CUDA_CHECK(cudaMemcpy(rs.host_pixel_buffer, device_pixel_buffer, rs.width * rs.height * 4 * sizeof(uint8_t), cudaMemcpyDeviceToHost));
        }
        else {
            for (int x = 0; x < rs.width; x++) {
                for (int y = 0; y < rs.height; y++) {

                    if (x > rs.width || y > rs.height) return; // check if no out of window, useless here but who cares
                    
                    int idx = x + y * rs.width;

                    float blur_size = ANTIALIAS_SIZE;
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

                    nanovdb::Vec3f camera_origin = rs.mainCam.position;
                    if (rs.mainCam.DOF_strength > 0.f) {
                        float RdoT = ww.dot(rayDirection);
                        nanovdb::Vec3f target = rs.mainCam.position + rayDirection * rs.mainCam.focal_distance / RdoT;
                        camera_origin += util::generateUniformSampleHOST() * rs.mainCam.DOF_strength;
                        rayDirection = (target - camera_origin).normalize();
                    }

                    nanovdb::math::Ray<float> first_ray = { camera_origin, rayDirection };

                    nanovdb::Vec3f color = getColor_host(rs, first_ray, make_float2(uvX, uvY));

                    if (rs.frame_index == 0)
                        host_accum_buffer[idx] = color;
                    else host_accum_buffer[idx] += color;

                    color = host_accum_buffer[idx] / float(rs.frame_index + 1);

                    // post processing
                    color = util::mix(nanovdb::Vec3f(util::luminance(color)), color, rs.mainCam.saturation);
                    color = util::mix(nanovdb::Vec3f(0.5f), color, rs.mainCam.contrast);
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

    
    extern "C" void run() 
    {
        Application penguinPTApp;
        penguinPTApp.initApplication();
        initCuda(penguinPTApp.rs);

        penguinPTApp.initLoaders();

        penguinPTApp.scene_ressource_loader.loader_sdpt.forceLoad(
            &penguinPTApp.scene_ressource_loader.loader_obj, 
            &penguinPTApp.scene_ressource_loader.loader_nvdb,
            &penguinPTApp.scene_ressource_loader.loader_envmap);

        penguinPTApp.m_bvh_ptr = penguinPTApp.scene_ressource_loader.loader_obj.getBVH(penguinPTApp.rs.CUDA_CAPABLE_GPU);
        std::vector<Mesh> try_mesh;

        try_mesh.push_back(Mesh(penguinPTApp.m_bvh_ptr));
        try_mesh.back().setTransforms(math::Mat4f());

        penguinPTApp.rs.scene.setBSDFList(penguinPTApp.scene_ressource_loader.loader_obj.bsdf_loader.BSDF_list, penguinPTApp.rs.CUDA_CAPABLE_GPU);
        penguinPTApp.rs.scene.setTLAS(try_mesh, penguinPTApp.rs.CUDA_CAPABLE_GPU);
        penguinPTApp.rs.scene.setVolumeList(penguinPTApp.scene_ressource_loader.loader_nvdb.vol_list, penguinPTApp.rs.CUDA_CAPABLE_GPU);

        penguinPTApp.scene_ressource_loader.loader_envmap.send_to_gpu(penguinPTApp.rs.scene.environnement_map, cudaFilterModeLinear);

        /////////////////////////////////////// Window and display ///////////////////////////////////////
        sf::RenderWindow window_render(sf::VideoMode(WINDOW_RES_X, WINDOW_RES_Y), "PenguinPT -- Render");
        sf::RenderWindow window_options(sf::VideoMode(512, 512), "PenguinPT, V1.0.0", sf::Style::Close);
        window_options.setIcon(32, 32, penguinPTApp.user_interface.getTextureFromIndex(0).copyToImage().getPixelsPtr());
        
        sf::Texture displayTex;
        displayTex.create(penguinPTApp.rs.width, penguinPTApp.rs.height);
        displayTex.update(penguinPTApp.rs.host_pixel_buffer);
        sf::Sprite diplaySprite(displayTex);

        {
            sf::Vector2u wnd_size = penguinPTApp.getViewportResolution();
            window_render.setView(sf::View(sf::FloatRect({0,0}, (sf::Vector2f)wnd_size)));
        }

        sf::Clock mainClock;

        while (window_options.isOpen())
        {
            sf::Event event;
            while (window_render.pollEvent(event))
            {
                if (event.type == sf::Event::Closed) {
                    window_render.close();
                }
            }
            while (window_options.pollEvent(event))
            {
                if (event.type == sf::Event::Closed) {
                    window_options.close();
                    window_render.close();
                }
            }

            if (window_render.hasFocus() || !penguinPTApp.rs.focus_needed) {
                if (!penguinPTApp.rs.lock_camera) penguinPTApp.rs.mainCam.move(&window_render, penguinPTApp.rs.delta_time, &penguinPTApp.rs.frame_index);
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::M)) {
                    save_screenshot(displayTex);
                }
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Multiply)) {
                    penguinPTApp.rs.is_rendering = true;
                    penguinPTApp.rs.frame_index = 0;
                }
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Divide)) {
                    penguinPTApp.rs.is_rendering = false;
                    penguinPTApp.rs.frame_index = 0;
                }
            }
            if (!penguinPTApp.rs.focus_needed && window_render.hasFocus() && !penguinPTApp.rs.lock_camera) window_options.requestFocus();
            if (!penguinPTApp.rs.is_rendering || penguinPTApp.rs.first_frame_mode) penguinPTApp.rs.frame_index = 0;
            penguinPTApp.user_interface.oiUpdate(
                &window_options, 
                &penguinPTApp.rs, 
                &displayTex, 
                &penguinPTApp.scene_ressource_loader.loader_obj,
                &penguinPTApp.scene_ressource_loader.loader_nvdb,
                &penguinPTApp.scene_ressource_loader.loader_envmap, 
                &penguinPTApp.scene_ressource_loader.loader_sdpt,
                penguinPTApp.m_bvh_ptr);

            if(window_render.isOpen()) renderFrame(penguinPTApp.rs, &displayTex);
            
            window_render.clear();
            window_render.draw(diplaySprite);
            window_render.display();

            window_options.clear();
            penguinPTApp.user_interface.oiDraw(&window_options);
            window_options.display();

            penguinPTApp.rs.delta_time = mainClock.restart().asSeconds();
            //gotoxy(1, 9);
            float fps = 1.f / penguinPTApp.rs.delta_time;
            //printf("FPS : %f\n", fps);
            penguinPTApp.rs.frame_index++;
        }

        penguinPTApp.clean();
        endCuda(penguinPTApp.rs);
    }
}