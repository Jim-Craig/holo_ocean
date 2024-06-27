// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>

namespace holoocean {

	enum class ObjectLabel : uint8_t {
		None         =    0u,
		Boat		 =    1u,
		Ground		 =	  2u,
		Yacht		 =	  3u,
		ContainerBoat =   4u,
		DamEnvironment =  5u,
		Landscape	 =	  6u,
		Any          =  0xFF
	  };
} 