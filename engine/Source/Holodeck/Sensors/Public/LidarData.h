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
			FVector3d point{};
			float intensity;

			LidarDetection() : point(0.0, 0.0, 0.0), intensity{0.0}
			{
				
			}

			LidarDetection(double x, double y, double z, float intensity) :
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

			LidarData(float* buffer, uint32_t ChannelCount = 0u) : SemanticLidarData(ChannelCount)
			{
				Buffer = buffer;
			}

			LidarData &operator=(LidarData &&) = default;

			~LidarData() = default;

			virtual void ResetMemory(std::vector<uint32_t> points_per_channel)
			{
				//std::memset(_header.data() + Index::SIZE, 0, sizeof(uint32_t) * GetChannelCount());

				uint32_t total_points = static_cast<uint32_t>(
					std::accumulate(points_per_channel.begin(), points_per_channel.end(), 0));

				points.clear();
				points.reserve(total_points * 4);

				LastFrameMaxPoints = NumPoints;
				NumPoints = 1;
				Buffer[0] = static_cast<float>(total_points);
			}

			void WritePointSync(LidarDetection& detection)
			{
				points.emplace_back(detection.point.X);
				points.emplace_back(detection.point.Y);
				points.emplace_back(detection.point.Z);
				points.emplace_back(detection.intensity);

				Buffer[4 * NumPoints] = detection.point.X;
				Buffer[4 * NumPoints + 1] = detection.point.Y;
				Buffer[4 * NumPoints + 2] = detection.point.Z;
				Buffer[4 * NumPoints + 3] = detection.intensity;
				NumPoints++;
			}

			virtual void WritePointSync(SemanticLidarDetection& detection) override
			{
				(void) detection;
			}

			std::size_t CountOfPoints()
			{
				return points.size();
			}

			std::vector<float> points;
			float* Buffer;
			int NumPoints = 0;
			int LastFrameMaxPoints = 0;

			
		private:
		};
	}	
}
