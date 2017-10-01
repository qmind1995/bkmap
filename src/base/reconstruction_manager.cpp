//
// Created by tri on 24/09/2017.
//

#include "base/reconstruction_manager.h"

#include "util/misc.h"
#include "util/option_manager.h"

namespace bkmap {

    ReconstructionManager::ReconstructionManager() {}

    ReconstructionManager::ReconstructionManager(ReconstructionManager&& other)
            : ReconstructionManager() {
        reconstructions_ = std::move(other.reconstructions_);
    }

    ReconstructionManager& ReconstructionManager::operator=(
            ReconstructionManager&& other) {
        if (this != &other) {
            reconstructions_ = std::move(other.reconstructions_);
        }
        return *this;
    }

    size_t ReconstructionManager::Size() const { return reconstructions_.size(); }

    const Reconstruction& ReconstructionManager::Get(const size_t idx) const {
        return *reconstructions_.at(idx);
    }

    Reconstruction& ReconstructionManager::Get(const size_t idx) {
        return *reconstructions_.at(idx);
    }

    size_t ReconstructionManager::Add() {
        const size_t idx = Size();
        reconstructions_.emplace_back(new Reconstruction());
        return idx;
    }

    void ReconstructionManager::Delete(const size_t idx) {
        CHECK_LT(idx, reconstructions_.size());
        reconstructions_.erase(reconstructions_.begin() + idx);
    }

    void ReconstructionManager::Clear() { reconstructions_.clear(); }

    size_t ReconstructionManager::Read(const std::string& path) {
        const size_t idx = Add();
        reconstructions_[idx]->Read(path);
        return idx;
    }

    void ReconstructionManager::Write(const std::string& path,
                                      const OptionManager* options) const {
        for (size_t i = 0; i < reconstructions_.size(); ++i) {
            const std::string reconstruction_path = JoinPaths(path, std::to_string(i));
            CreateDirIfNotExists(reconstruction_path);
            reconstructions_[i]->Write(reconstruction_path);
            if (options != nullptr) {
                options->Write(JoinPaths(reconstruction_path, "project.ini"));
            }
        }
    }

}