// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#ifndef __EMBREE_DEFAULT_H__
#define __EMBREE_DEFAULT_H__

#include "sys/platform.h"
#include "sys/ref.h"
#include "sys/intrinsics.h"
#include "sys/stl/vector.h"
#include "sys/stl/string.h"

#include "math/math.h"
#include "math/vec2.h"
#include "math/vec3.h"
#include "math/vec4.h"
#include "math/color.h"
#include "math/affinespace.h"

#include "simd/simd.h"

#include "sys/thread.h"
#include "sys/sync/atomic.h"
#include "sys/sync/barrier.h"

#include "samplers/sample.h"
#include "samplers/shapesampler.h"

namespace embree
{
  /* vertex and triangle layout */
  struct RTCVertex   { float x,y,z,a; };
  struct RTCTriangle { int v0, v1, v2; };

  inline bool IsPowerOf2(int v) {
	  return (v & (v - 1)) == 0;
  }


  inline uint32_t RoundUpPow2(uint32_t v) {
	  v--;
	  v |= v >> 1;    v |= v >> 2;
	  v |= v >> 4;    v |= v >> 8;
	  v |= v >> 16;
	  return v + 1;
  }

};

#endif
