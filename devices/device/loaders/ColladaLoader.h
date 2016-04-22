#pragma once

#include <vector>
#include "sys/filename.h"
#include "device/device.h"
#include "device/handle.h"

namespace embree
{
	std::vector<Handle<Device::RTPrimitive>> loadDAE(const FileName &fileName, std::vector<Handle<Device::RTCamera>> &cameras);
}
