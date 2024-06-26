#pragma once


#include <cstdint>


static constexpr
#if __cplusplus >= 201703L // C++17
	inline
#endif

uint8_t HOLOOCEAN_PALETTE_MAP[][3u] = {
		{  0u,   0u,   0u},   // unlabeled				=   0u
		{128u,  64u, 128u},   // Boat						=   1u
		{244u,  35u, 232u},   // Ground					=   2u
		{ 70u,  70u,  70u},   // Yacht					=   3u
		{102u, 102u, 156u},   // ContainerBoat			=   4u
		{190u, 153u, 153u},   // DamEnvironment			=   5u
		{153u, 153u, 153u},   // Landscape				=   6u

};

class HolooceanPalette
{
public:
	static constexpr auto GetNumberOfTags()
	{
		return std::size(HOLOOCEAN_PALETTE_MAP);
	}

	/// Return an RGB uint8_t array.
	///
	/// @warning It overflows if @a tag is greater than GetNumberOfTags().
	static constexpr auto GetColor(uint8_t tag) {
		return HOLOOCEAN_PALETTE_MAP[tag % GetNumberOfTags()];
	}
};