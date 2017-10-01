// COLMAP - Structure-from-Motion and Multi-View Stereo.
// Copyright (C) 2017  Johannes L. Schoenberger <jsch at inf.ethz.ch>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef BKMAP_SRC_UTIL_VERSION_H_
#define BKMAP_SRC_UTIL_VERSION_H_

#include "misc.h"

namespace bkmap {

const static std::string COLMAP_VERSION = "3.2";
const static std::string COLMAP_COMMIT_ID = "30b0946";
const static std::string COLMAP_COMMIT_DATE = "2017-09-02";

std::string GetVersionInfo();

std::string GetBuildInfo();

}

#endif  // COLMAP_SRC_UTIL_VERSION_H_
