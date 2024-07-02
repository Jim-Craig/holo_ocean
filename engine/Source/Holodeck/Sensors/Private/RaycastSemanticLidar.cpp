#include "RaycastSemanticLidar.h"

#include <cmath>

URaycastSemanticLidar::URaycastSemanticLidar()
{
	SensorName = "RaycastSemanticLidar";
	Parent = nullptr;
}

void URaycastSemanticLidar::ParseSensorParms(FString ParmsJson)
{
	// Parse all the configuration options for the lidar (num lasers, etc.)
}

void URaycastSemanticLidar::InitializeSensor()
{
	Super::InitializeSensor();
	Parent = this->GetAttachmentRootActor();


	// TODO: Move this into the Read JSON params
	Description = FLidarDescription();
	Set(Description);
	this->Buffer = static_cast<float*>(Super::Buffer);

	
}

void URaycastSemanticLidar::TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	SimulateLidar(DeltaTime);

	// TODO: Get the data and write it to the buffer
	{
		SCOPE_CYCLE_COUNTER(STAT_CopyBuffer)
		std::vector<holoocean::data::SemanticLidarDetection> points = SemanticLidarData.GetPoints();
		int writeIndex = 0;
		for (int i = 0; i < points.size() - 1; i++ )
		{
			if (i < 1)
			{
				/*if (i < 100)
				{
					UE_LOG(LogHolodeck, Error, TEXT("%f datapoint!"), TempData._points[i]);
				}*/
				Buffer[writeIndex] = static_cast<float>(points[i].point.X);
				Buffer[writeIndex + 1] = static_cast<float>(points[i].point.Y);
				Buffer[writeIndex + 2] = static_cast<float>(points[i].point.Z);
				//Buffer[writeIndex + 4] = static_cast<float>(points[i].cos_inc_angle);
				//Buffer[writeIndex + 5] = static_cast<float>(points[i].object_idx);
				//Buffer[writeIndex + 6] = static_cast<float>(points[i].object_tag);
				writeIndex += 3;
			}
		}
	}
}

void URaycastSemanticLidar::Set(const FLidarDescription& LidarDescription)
{
	Description = LidarDescription;
	SemanticLidarData = FSemanticLidarData(Description.Channels);
	CreateLasers();
	PointsPerChannel.resize(Description.Channels);
}

void URaycastSemanticLidar::CreateLasers()
{
	const auto NumberOfLasers = Description.Channels;
	check(NumberOfLasers > 0u);

	const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
		(Description.UpperFovLimit - Description.LowerFovLimit) /
		static_cast<float>(NumberOfLasers - 1);

	LaserAngles.Empty(NumberOfLasers);

	for (auto i = 0u; i < NumberOfLasers; ++i)
	{
		const float VerticalAngle = Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
		LaserAngles.Emplace(VerticalAngle);
	}
}

void URaycastSemanticLidar::SimulateLidar(const float DeltaTime)
{
	
	const uint32 ChannelCount = Description.Channels;
	const uint32 PointsToScanWithOneLaser = FMath::RoundHalfFromZero(
			Description.PointsPerSecond * DeltaTime / static_cast<float>(ChannelCount)
		);

	if (PointsToScanWithOneLaser <= 0)
	{
		UE_LOG(LogHolodeck, Warning, TEXT("%s: no points request this frame.. Try increasing the number of points per second."), *GetName());
		return;
	}

	check(ChannelCount == LaserAngles.Num());

	const float CurrentHorizontalAngle = FMath::RadiansToDegrees(SemanticLidarData.GetHorizontalAngle());
	const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov * DeltaTime;
	const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

	ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser);
	PreprocessRays(ChannelCount, PointsToScanWithOneLaser);

	// NOTE: The default physics engine changed between UE4 and UE5. It no longer uses PhysX, but uses Chaos.
	// Thus, this function call no longer works. But Chaos *might* be async already.
	//GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
	
	{
		SCOPE_CYCLE_COUNTER(STAT_ParallelForHoloocean)
		ParallelFor(ChannelCount, [&](int32 idxChannel) {
		  TRACE_CPUPROFILER_EVENT_SCOPE(STAT_ParallelFor);

			// TODO: Parent was "this" before, but since the sensors are not actors, it couldn't be. This might
			// cause issues though if it gets garbase collected
		  FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, Parent);
		  TraceParams.bTraceComplex = true;
		  TraceParams.bReturnPhysicalMaterial = false;

		  for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) {
			FHitResult HitResult;
			const float VertAngle = LaserAngles[idxChannel];
			const float HorizAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfLaserMeasure
				* idxPtsOneLaser, Description.HorizontalFov) - Description.HorizontalFov / 2;
			const bool PreprocessResult = RayPreprocessCondition[idxChannel][idxPtsOneLaser];

			if (PreprocessResult && ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams)) {
			  WritePointAsync(idxChannel, HitResult);
			}
		  };
		});
	}
	//GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

	FTransform ActorTransf = Parent->GetTransform();
	ComputeAndSaveDetections(ActorTransf);

	const float HorizontalAngle = FMath::RadiansToDegrees(std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov));
	SemanticLidarData.SetHorizontalAngle(HorizontalAngle);
}

void URaycastSemanticLidar::ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel)
{
	RecordedHits.resize(Channels);

	for (auto& hits : RecordedHits)
	{
		hits.clear();
		hits.reserve(MaxPointsPerChannel);
	}
}

void URaycastSemanticLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel)
{
	RayPreprocessCondition.resize(Channels);

	for (auto& conds : RayPreprocessCondition)
	{
		conds.clear();
		conds.resize(MaxPointsPerChannel);
		std::fill(conds.begin(), conds.end(), true);
	}
}

void URaycastSemanticLidar::WritePointAsync(uint32_t Channel, FHitResult& Detection)
{
	RecordedHits[Channel].emplace_back(Detection);
}

void URaycastSemanticLidar::ComputeAndSaveDetections(const FTransform& SensorTransform)
{
	for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
	{
		PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
	}
	SemanticLidarData.ResetMemory(PointsPerChannel);

	for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
	{
		for (auto& hit : RecordedHits[idxChannel])
		{
			FSemanticDetection detection;
			ComputeRawDetection(hit, SensorTransform, detection);
			SemanticLidarData.WritePointSync(detection);
		}
	}

	SemanticLidarData.WriteChannelCount(PointsPerChannel);
}

void URaycastSemanticLidar::ComputeRawDetection(const FHitResult& HitInfo, const FTransform& SensorTransf, FSemanticDetection& Detection) const
{
	const FVector HitPoint = HitInfo.ImpactPoint;
	Detection.point = static_cast<FVector3f>(SensorTransf.Inverse().TransformPosition(HitPoint));

	const FVector VecInc = - (HitPoint - SensorTransf.GetLocation()).GetSafeNormal();
	Detection.cos_inc_angle = FVector::DotProduct(VecInc, HitInfo.ImpactNormal);

	const AActor* actor = HitInfo.GetActor();
	Detection.object_idx = 0;
	Detection.object_tag = static_cast<uint32_t>(HitInfo.Component->CustomDepthStencilValue);

	if (actor != nullptr)
	{
		// TODO: Carla uses an Actor Registry to find actors and use their actor ID. We don't have one of these. Sooo ¯\_(ツ)_/¯
	}
	
}

bool URaycastSemanticLidar::ShootLaser(const float VerticalAngle, float HorizontalAngle, FHitResult& HitResult, FCollisionQueryParams& TraceParams) const
{
	FHitResult HitInfo(ForceInit);

	FTransform ActorTransf = this->GetComponentTransform();
	FVector LidarBodyLoc = ActorTransf.GetLocation();
	//UE_LOG(LogHolodeck, Error, TEXT("%f, %f, %f"), LidarBodyLoc.X, LidarBodyLoc.Y, LidarBodyLoc.Z);
	FRotator LidarBodyRot = ActorTransf.Rotator();

	FRotator LaserRot(VerticalAngle, HorizontalAngle, 0); // float InPitch, float InYaw, float InRoll
	FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
		LaserRot,
		LidarBodyRot
		);

	const auto Range = Description.Range;
	FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;
	
	// TODO: This was parallel in Carla. UE5 is different though, would need to figure out the async.
	// This is a major area for speed up in the future
	GetWorld()->LineTraceSingleByChannel(
		HitInfo,
		LidarBodyLoc,
		EndTrace,
		ECC_Visibility,
		TraceParams,
		FCollisionResponseParams::DefaultResponseParam
	  );

	
	
	if (HitInfo.bBlockingHit)
	{
		HitResult = HitInfo;
		return true;
	}
	else
	{
		return false;
	}
}









