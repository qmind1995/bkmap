
#include "util/cuda.h"

#include <algorithm>
#include <iostream>
#include <vector>

#include <cuda_runtime.h>

#include "util/cudacc.h"
#include "util/logging.h"

namespace bkmap {
    namespace {

        // Check whether the first Cuda device is better than the second.
        bool CompareCudaDevice(const cudaDeviceProp& d1, const cudaDeviceProp& d2) {
            bool result = (d1.major > d2.major) ||
                          ((d1.major == d2.major) && (d1.minor > d2.minor)) ||
                          ((d1.major == d2.major) && (d1.minor == d2.minor) &&
                           (d1.multiProcessorCount > d2.multiProcessorCount));
            return result;
        }

    }  // namespace

    int GetNumCudaDevices() {
        int num_cuda_devices;
        cudaGetDeviceCount(&num_cuda_devices);
        return num_cuda_devices;
    }

    void SetBestCudaDevice(const int gpu_index) {
        const int num_cuda_devices = GetNumCudaDevices();
        CHECK_GT(num_cuda_devices, 0) << "No CUDA devices available";

        int selected_gpu_index = -1;
        if (gpu_index >= 0) {
            selected_gpu_index = gpu_index;
        } else {
            std::vector<cudaDeviceProp> all_devices(num_cuda_devices);
            for (int device_id = 0; device_id < num_cuda_devices; ++device_id) {
                cudaGetDeviceProperties(&all_devices[device_id], device_id);
            }
            std::sort(all_devices.begin(), all_devices.end(), CompareCudaDevice);
            CUDA_SAFE_CALL(cudaChooseDevice(&selected_gpu_index, all_devices.data()));
        }

        CHECK_GE(selected_gpu_index, 0);
        CHECK_LT(selected_gpu_index, num_cuda_devices) << "Invalid CUDA GPU selected";

        cudaDeviceProp device;
        cudaGetDeviceProperties(&device, selected_gpu_index);
        CUDA_SAFE_CALL(cudaSetDevice(selected_gpu_index));
    }

}