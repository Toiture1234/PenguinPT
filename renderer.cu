#include "api.h"

namespace penguinPT {
    __shared__ uint8_t* device_pixel_buffer;
    __shared__ nanovdb::Vec3f* device_accum_buffer;

    void initCuda(renderer_services rs) {
        CUDA_CHECK(cudaMalloc((void**)&device_pixel_buffer, rs.width * rs.height * 4 * sizeof(uint8_t)));
        CUDA_CHECK(cudaMalloc((void**)&device_accum_buffer, rs.width * rs.height * sizeof(nanovdb::Vec3f)));
        printf("CUDA INITIALIZED\n");
    }
    void endCuda(renderer_services rs) {
        CUDA_CHECK(cudaFree(device_pixel_buffer));
        CUDA_CHECK(cudaFree(device_accum_buffer));
        printf("CUDA ENDED\n");
    }

    // kernel
    __device__ inline nanovdb::Vec3f getColor(renderer_services& rs, nanovdb::math::Ray<float> ray, float2 uv) {
        hit_info info;
        info.t = 1e10f;
        float pdf = 1.f;

        //if (rs.scene.intersectScene_full(ray, info))
            //return rs.scene.bsdf_list[info.BSDF_index].eval(ray, info.normal, { 0.f, 1.f, 0. }, pdf, rs.scene.bsdf_list[info.BSDF_index].params);
            //return info.normal * 0.5f + nanovdb::Vec3f(0.5f);
        //return (nanovdb::Vec3f)info.debug * nanovdb::Vec3f(1.f, 0.f, 0.01f);
        //return { 0.f, 0.f, 1.f };
        return volume_pathtrace_device(rs, ray);
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
        callKernel(rs);

        CUDA_CHECK(cudaMemcpy(rs.host_pixel_buffer, device_pixel_buffer, rs.width * rs.height * 4 * sizeof(uint8_t), cudaMemcpyDeviceToHost));
        dsp_text->update(rs.host_pixel_buffer);
    }

    extern "C" void run() 
    {
        renderer_services rs;
        rs.mainCam.speed = 1.f;
        rs.fill_host_pixel_buffer();
        initCuda(rs);

        loader::obj_loader loader01;
        loader::nanovdb_loader loader02;

        loader01.load_obj("assets/models/sphere.obj");
        loader01.send_to_scene(rs.scene);
        
        rs.scene.build_BVH();
        rs.scene.send_to_gpu_solid();

        // load quickly bsdf, TODO : move !!
        BSDF* h_list = new BSDF[2];
        h_list[0].bsdf_type = BSDF_trough;
        h_list[1].bsdf_type = BSDF_specular;
        h_list[1].roughness = 0.2f;
        h_list[1].metalness = 1.f;
        h_list[1].albedo = nanovdb::Vec3f(1.f, 0.7f, 0.3f);
        
        // transfer
        cudaMalloc((void**)&rs.scene.bsdf_list, 2 * sizeof(BSDF));
        cudaMemcpy(rs.scene.bsdf_list, h_list, 2 * sizeof(BSDF), cudaMemcpyHostToDevice);

        delete[] h_list;

        loader02.load_nvdb("fire.nvdb");
        loader02.volume_parameters(0, 10.f, nanovdb::Vec3f(1.f), 0.f);
        loader02.set_tranforms(0, 0.1f, nanovdb::Vec3f(0.f, 5.f, 0.f));

        loader02.send_to_scene(rs.scene);
        rs.scene.send_to_gpu_volumes();
        
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

            renderFrame(rs, &displayTex);
            
            window.clear();
            window.draw(diplaySprite);
            window.display();

            rs.delta_time = mainClock.restart().asSeconds();
            printf("FPS : %f\n", 1.f / rs.delta_time);
            rs.frame_index++;
        }

        endCuda(rs);
    }
}