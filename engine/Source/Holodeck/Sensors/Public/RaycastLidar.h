// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once

#include <vector>

#include "Holodeck.h"
#include "HolodeckSensor.h"
#include "LidarData.h"
#include "LidarDescription.h"
#include "RandomEngine.h"
#include "RaycastSemanticLidar.h"
#include "SemanticLidarData.h"

#include "RaycastLidar.generated.h"
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class HOLODECK_API URaycastLidar : public URaycastSemanticLidar
{
	GENERATED_BODY()

	using FLidarData = holoocean::data::LidarData;
	using FDetection = holoocean::data::LidarDetection;

public:
	/*
	 * Default Constructor
	 */
	URaycastLidar();	

	/**
	* InitializeSensor
	* Sets up the class
	*/
	virtual void InitializeSensor() override;

	/**
	* Allows parameters to be set dynamically
	*/
	virtual void ParseSensorParms(FString ParmsJson) override;

	virtual void Set(const FLidarDescription& LidarDescription) override;

protected:
	virtual void TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	int virtual GetNumItems() override { return LidarDescription.PointsPerSecond * 4; };
	int virtual GetItemSize() override { return sizeof(float); };

	float* Buffer;
	FLidarData Data;
	
private:
	FSemanticLidarData SemanticLidarData;

	FLidarDescription LidarDescription;

	/// Compute the received intensity of the point
	float ComputeIntensity(const FSemanticDetection& RawDetection) const;
	FDetection ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const;

	virtual void PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) override;
	bool PostprocessDetection(FDetection& Detection) const;

	virtual void ComputeAndSaveDetections(const FTransform& SensorTransform) override;

	//FLidarData* Buffer;
	FLidarData TempData;

	
	/// Enable/Disable general dropoff of lidar points
	bool DropOffGenActive;

	/// Slope for the intensity dropoff of lidar points, it is calculated
	/// throught the dropoff limit and the dropoff at zero intensity
	/// The points is kept with a probality alpha*Intensity + beta where
	/// alpha = (1 - dropoff_zero_intensity) / droppoff_limit
	/// beta = (1 - dropoff_zero_intensity)
	float DropOffAlpha;
	float DropOffBeta;
	
	TUniquePtr<AActor> Parent;
};
