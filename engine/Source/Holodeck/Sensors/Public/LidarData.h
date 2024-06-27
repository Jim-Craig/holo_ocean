// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "SemanticLidarData.h"

#include <cstdint>
#include <vector>

namespace holoocean
{
	namespace data
	{
		class LidarDetection
		{
		public:
			FVector3f point{};
			float intensity;

			LidarDetection() : point(0.0f, 0.0f, 0.0f), intensity{0.0f}
			{
				
			}

			LidarDetection(float x, float y, float z, float intensity) :
				point(x, y, z), intensity{intensity}
			{ }

			LidarDetection(FVector3f p, float intensity) :
				point(p), intensity{intensity}
			{ }

			void WritePlyHeaderInfo(std::ostream& out) const
			{
				out << "property float32 x\n" \
				  "property float32 y\n" \
				  "property float32 z\n" \
				  "property float32 I";
			}

			void WriteDetection(std::ostream& out) const
			{
				out << point.X << ' ' << point.Y << ' ' << point.Z << ' ' << intensity;
			}
		};

		class LidarData : public SemanticLidarData
		{

		public:
			explicit LidarData(uint32_t ChannelCount = 0u)
			  : SemanticLidarData(ChannelCount) {
			}

			LidarData &operator=(LidarData &&) = default;

			~LidarData() = default;

			virtual void ResetMemory(std::vector<uint32_t> points_per_channel) {
				std::memset(_header.data() + Index::SIZE, 0, sizeof(uint32_t) * GetChannelCount());

				uint32_t total_points = static_cast<uint32_t>(
					std::accumulate(points_per_channel.begin(), points_per_channel.end(), 0));

				_points.clear();
				_points.reserve(total_points * 4);
			}

			void WritePointSync(LidarDetection &detection) {
				_points.emplace_back(detection.point.X);
				_points.emplace_back(detection.point.Y);
				_points.emplace_back(detection.point.Z);
				_points.emplace_back(detection.intensity);
			}

			virtual void WritePointSync(SemanticLidarDetection& detection) {
				(void) detection;
			}

			std::size_t CountOfPoints()
			{
				return _points.size();
			}

			std::vector<float> _points;

			
		private:
		};
	}	
}
