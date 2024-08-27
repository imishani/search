/*
* Copyright (C) 2024, Itamar Mishani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   torch_utils.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   2024-08-26
*/


#ifndef TORCH_UTILS_HPP
#define TORCH_UTILS_HPP

#include <torch/torch.h>
#include <torch/script.h>


namespace torch_utils {

inline torch::Device getDevice() {
    return torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
}


inline void loadTorchModel(const std::string& model_path,
                           torch::jit::script::Module& model,
                           const torch::Device device = getDevice()) {
    try {
        model = torch::jit::load(model_path);
        model.to(device);
    } catch (const c10::Error& e) {
        std::cerr << "error loading the model\n";
    }
}
}   // namespace torch_utils


inline void convert2DMapToTensor(const std::vector<std::vector<int>>* map,
                                 torch::Tensor& map_tensor) {
    // convert the map to a tensor
    map_tensor = torch::zeros({static_cast<long>(map->size()), static_cast<long>(map->at(0).size())});
    for (int i {0}; i < map->size(); i++) {
        for (int j {0}; j < map->at(0).size(); j++) {
            map_tensor[i][j] = map->at(i).at(j);
        }
    }
}


#endif //TORCH_UTILS_HPP
