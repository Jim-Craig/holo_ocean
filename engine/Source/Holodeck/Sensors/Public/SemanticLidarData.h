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

			SemanticLidarData& operator=(SemanticLidarData &&) = default;

			virtual ~SemanticLidarData() {}

			float GetHorizontalAngle() const {
				return reinterpret_cast<const float &>(_header[Index::HorizontalAngle]);
			}

			void SetHorizontalAngle(float angle) {
				std::memcpy(&_header[Index::HorizontalAngle], &angle, sizeof(uint32_t));
			}

			uint32_t GetChannelCount() const {
				return _header[Index::ChannelCount];
			}

			virtual void ResetMemory(std::vector<uint32_t> points_per_channel) {
				std::memset(_header.data() + Index::SIZE, 0, sizeof(uint32_t) * GetChannelCount());

				uint32_t total_points = static_cast<uint32_t>(
					std::accumulate(points_per_channel.begin(), points_per_channel.end(), 0));

				_ser_points.clear();
				_ser_points.reserve(total_points);
			}

			virtual void WriteChannelCount(std::vector<uint32_t> points_per_channel) {
				for (auto idxChannel = 0u; idxChannel < GetChannelCount(); ++idxChannel)
					_header[Index::SIZE + idxChannel] = points_per_channel[idxChannel];
			}

			virtual void WritePointSync(SemanticLidarDetection &detection) {
				_ser_points.emplace_back(detection);
			}

		protected:
			std::vector<uint32_t> _header;
			uint32_t _max_channel_points;

		private:
			std::vector<SemanticLidarDetection> _ser_points;

		};
	}

}
