//
// Created by tri on 24/09/2017.
//

#ifndef BKMAP_CAMERA_SPECS_H
#define BKMAP_CAMERA_SPECS_H

#include <string>
#include <unordered_map>
#include <vector>

namespace bkmap {

    typedef std::vector<std::pair<std::string, float>> camera_make_specs_t;
    typedef std::unordered_map<std::string, camera_make_specs_t> camera_specs_t;

    camera_specs_t InitializeCameraSpecs();

}

#endif //BKMAP_CAMERA_SPECS_H
