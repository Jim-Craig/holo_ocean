// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once

#include <vector>

#include "Holodeck.h"

#include "HolodeckSensor.h"


#include "LidarDescription.h"
#include "RandomEngine.h"
#include "SemanticLidarData.h"

DECLARE_STATS_GROUP(TEXT("HoloOceanRaycasting"), STATGROUP_RaycastFunction, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("CopyBuffer"), STAT_CopyBuffer, STATGROUP_RaycastFunction);
DECLARE_CYCLE_STAT(TEXT("ParallelFor"), STAT_ParallelForHoloocean, STATGROUP_RaycastFunction);


#include "RaycastSemanticLidar.generated.h"
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))

class HOLODECK_API URaycastSemanticLidar : public UHolodeckSensor
{
	GENERATED_BODY()


	
public:


	/*
	 * Default Constructor
	 */
	URaycastSemanticLidar();	

	/**
	* InitializeSensor
	* Sets up the class
	*/
	virtual void InitializeSensor() override;

	/**
	* Allows parameters to be set dynamically
	*/
	virtual void ParseSensorParms(FString ParmsJson) override;
	
	virtual void Set(const FLidarDescription& LidarDescription);

protected:

	using FSemanticLidarData = holoocean::data::SemanticLidarData;
	using FSemanticDetection = holoocean::data::SemanticLidarDetection;
	
	//See HolodeckSensor for the documentation of these overridden functions.
	int virtual GetNumItems() override { return Description.PointsPerSecond; };
	int virtual GetItemSize() override { return sizeof(float); };
	void virtual TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;


	/// Creates a Laser for each channel.
	void CreateLasers();

	/// Updates LidarMeasurement with the points read in DeltaTime.
	void SimulateLidar(const float DeltaTime);

	/// Shoot a laser ray-trace, return whether the laser hit something.
	bool ShootLaser(const float VerticalAngle, float HorizontalAngle, FHitResult &HitResult, FCollisionQueryParams& TraceParams) const;

	/// Method that allow to preprocess if the rays will be traced.
	virtual void PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel);

	/// Compute all raw detection information
	void ComputeRawDetection(const FHitResult &HitInfo, const FTransform& SensorTransf, FSemanticDetection& Detection) const;

	/// Saving the hits the raycast returns per channel
	void WritePointAsync(uint32_t Channel, FHitResult &Detection);

	/// Clear the recorded data structure
	void ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel);

	/// This method uses all the saved FHitResults, compute the
	/// RawDetections and then send it to the LidarData structure.
	virtual void ComputeAndSaveDetections(const FTransform &SensorTransform);

	UPROPERTY(EditAnywhere)
	FLidarDescription Description;

	TArray<float> LaserAngles;

	std::vector<std::vector<FHitResult>> RecordedHits;
	std::vector<std::vector<bool>> RayPreprocessCondition;
	std::vector<uint32_t> PointsPerChannel;


	/// Random Engine used to provide noise for sensor output.
	UPROPERTY(VisibleAnywhere)
	URandomEngine* RandomEngine = nullptr;
	bool ShouldInitialize = true;

	
private:
	/*
	 * Parent
	 * After initialization, Parent contains a pointer to whatever the sensor is attached to.
	 */
	TUniquePtr<AActor> Parent;

	FSemanticLidarData SemanticLidarData;
	float* SemanticLidarBuffer;

	


};
