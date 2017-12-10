#ifndef BKMAP_SRC_UTIL_CUDA_H_
#define BKMAP_SRC_UTIL_CUDA_H_

namespace bkmap {

int GetNumCudaDevices();

void SetBestCudaDevice(const int gpu_index);

}

#endif