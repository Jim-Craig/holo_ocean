// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>
#include <vector>
#include <numeric>

namespace holoocean
{
	namespace data
	{
		
			
		class SemanticLidarDetection
		{
		public:
			FVector3f point{};
			float cos_inc_angle{};
			uint32_t object_idx{};
			uint32_t object_tag{};


			SemanticLidarDetection() = default;
			
			SemanticLidarDetection(float x, float y, float z, float cosTh, uint32_t idx, uint32_t tag) : point(x, y, z), cos_inc_angle{cosTh}, object_idx{idx}, object_tag{tag}
			{
				
			}
			SemanticLidarDetection(FVector3d p, float cosTh, uint32_t idx, uint32_t tag) :
				point(p), cos_inc_angle{cosTh}, object_idx{idx}, object_tag{tag}
			{
				
			}

			void WritePlyHeaderInfo(std::ostream& out) const{
				out << "property float32 x\n" \
				   "property float32 y\n" \
				   "property float32 z\n" \
				   "property float32 CosAngle\n" \
				   "property uint32 ObjIdx\n" \
				   "property uint32 ObjTag";
			}

			void WriteDetection(std::ostream& out) const{
				out << point.X << ' ' << point.Y << ' ' << point.Z << ' ' \
				  << cos_inc_angle << ' ' << object_idx << ' ' << object_tag;
			}
		};

		class SemanticLidarData {
			static_assert(sizeof(float) == sizeof(uint32_t), "Invalid float size");

		protected:
			enum Index : size_t {
				HorizontalAngle,
				ChannelCount,
				SIZE
			  };

		public:
			explicit SemanticLidarData(uint32_t ChannelCount = 0u)
			  : _header(Index::SIZE + ChannelCount, 0u) {
				_header[Index::ChannelCount] = ChannelCount;
			}

			SemanticLidarData(float* buffer, uint32_t ChannelCount = 0u)
			{
				Buffer = buffer;
				channelCount = ChannelCount;
			}

			SemanticLidarData& operator=(SemanticLidarData &&) = default;

			virtual ~SemanticLidarData() {}

			float GetHorizontalAngle() const {
				return horizontalAngle;
			}

			void SetHorizontalAngle(float angle) {
				this->horizontalAngle = angle;
			}

			uint32_t GetChannelCount() const {
				return channelCount;
			}

			virtual void ResetMemory(std::vector<uint32_t> points_per_channel) {
				//std::memset(_header.data() + Index::SIZE, 0, sizeof(uint32_t) * GetChannelCount());

				uint32_t total_points = static_cast<uint32_t>(
					std::accumulate(points_per_channel.begin(), points_per_channel.end(), 0));

				_ser_points.clear();
				_ser_points.reserve(total_points);

				points.clear();
				points.reserve(total_points * 6);
				
				LastFrameMaxPoints = NumPoints;
				NumPoints = 1;
				Buffer[0] = static_cast<float>(total_points);
			}

			virtual void WriteChannelCount(std::vector<uint32_t> points_per_channel) {
				for (auto idxChannel = 0u; idxChannel < GetChannelCount(); ++idxChannel)
					_header[Index::SIZE + idxChannel] = points_per_channel[idxChannel];
			}

			virtual void WritePointSync(SemanticLidarDetection &detection) {
				//_ser_points.emplace_back(detection);

				points.emplace_back(detection.point.X);
				points.emplace_back(detection.point.Y);
				points.emplace_back(detection.point.Z);
				points.emplace_back(detection.cos_inc_angle);
				points.emplace_back(static_cast<float>(detection.object_idx));
				points.emplace_back(static_cast<float>(detection.object_tag));

				Buffer[6 * NumPoints] = detection.point.X / 100;
				Buffer[6 * NumPoints + 1] = detection.point.Y / 100;
				Buffer[6 * NumPoints + 2] = detection.point.Z / 100;
				Buffer[6 * NumPoints + 3] = detection.cos_inc_angle;
				Buffer[6 * NumPoints + 4] = static_cast<float>(detection.object_idx);
				Buffer[6 * NumPoints + 5] = static_cast<float>(detection.object_tag);

				NumPoints++;
			}

			std::vector<SemanticLidarDetection> GetPoints()
			{
				return this->_ser_points;
			}

		protected:
			std::vector<uint32_t> _header;
			uint32_t _max_channel_points;

		private:
			std::vector<SemanticLidarDetection> _ser_points;
			std::vector<float> points;
			float* Buffer;
			int NumPoints = 0;
			int LastFrameMaxPoints = 0;

			float horizontalAngle;
			uint32_t channelCount;
		};
	}

}
