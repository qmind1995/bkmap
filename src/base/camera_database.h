//
// Created by tri on 24/09/2017.
//

#ifndef BKMAP_CAMERA_DATABASE_H
#define BKMAP_CAMERA_DATABASE_H

#include <string>

#include "util/camera_specs.h"

namespace bkmap {

// Database that contains sensor widths for many cameras, which is useful
// to automatically extract the focal length if EXIF information is incomplete.
class CameraDatabase {
 public:
  CameraDatabase();

  size_t NumEntries() const { return specs_.size(); }

  bool QuerySensorWidth(const std::string& make, const std::string& model,
                        double* sensor_width);

 private:
  static const camera_specs_t specs_;
};

}


#endif //BKMAP_CAMERA_DATABASE_H
