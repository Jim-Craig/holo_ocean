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
	Description = FLidarDescription();


	TSharedPtr<FJsonObject> JsonParsed;
	TSharedRef<TJsonReader<TCHAR>> JsonReader = TJsonReaderFactory<TCHAR>::Create(ParmsJson);
	
	if (FJsonSerializer::Deserialize(JsonReader, JsonParsed))
	{
		JsonParsed->TryGetNumberField("Channels", Description.Channels);
		JsonParsed->TryGetNumberField("Range", Description.Range);
		JsonParsed->TryGetNumberField("PointsPerSecond", Description.PointsPerSecond);
		JsonParsed->TryGetNumberField("RotationFrequency", Description.RotationFrequency);
		JsonParsed->TryGetNumberField("UpperFovLimit", Description.UpperFovLimit);
		JsonParsed->TryGetNumberField("LowerFovLimit", Description.LowerFovLimit);
		JsonParsed->TryGetNumberField("HorizontalFov", Description.HorizontalFov);
		JsonParsed->TryGetNumberField("AtmospAttenRate", Description.AtmospAttenRate);
		JsonParsed->TryGetNumberField("RandomSeed", Description.RandomSeed);
		JsonParsed->TryGetNumberField("DropOffGenRate", Description.DropOffGenRate);
		JsonParsed->TryGetNumberField("DropOffIntensityLimit", Description.DropOffIntensityLimit);
		JsonParsed->TryGetNumberField("DropOffAtZeroIntensity", Description.DropOffAtZeroIntensity);
		JsonParsed->TryGetBoolField("ShowDebugPoints", Description.ShowDebugPoints);
		JsonParsed->TryGetNumberField("NoiseStdDev", Description.NoiseStdDev);

		Description.Range *= 100; // We need to convert it to cm since that is what HoloOcean uses
	}
	
	Set(Description);
}

void URaycastSemanticLidar::InitializeSensor()
{
	Super::InitializeSensor();

	if (ShouldInitialize)
	{
		ShouldInitialize = false;
		this->SemanticLidarBuffer = static_cast<float*>(Buffer);
		this->SemanticLidarData = FSemanticLidarData(this->SemanticLidarBuffer, Description.Channels);
	}
	
	Parent = TUniquePtr<AActor>(this->GetAttachmentRootActor());
}

void URaycastSemanticLidar::TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	SimulateLidar(DeltaTime);
}

void URaycastSemanticLidar::Set(const FLidarDescription& LidarDescription)
{
	Description = LidarDescription;
	UE_LOG(LogHolodeck, Error, TEXT("Range: %f"), Description.Range)

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

	{
		SCOPE_CYCLE_COUNTER(STAT_ParallelForHoloocean)
		ParallelFor(ChannelCount, [&](int32 idxChannel) {
		  TRACE_CPUPROFILER_EVENT_SCOPE(STAT_ParallelFor);
			
			// TODO: Parent was "this" before, but since the sensors are not actors, it couldn't be. This might
			// cause issues though if it gets garbage collected
		  FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, Parent.Get());
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
	
	int PointCount = 0;
	for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
	{
		for (auto& hit : RecordedHits[idxChannel])
		{
			FSemanticDetection detection;
			ComputeRawDetection(hit, SensorTransform, detection);
			SemanticLidarData.WritePointSync(detection);
			PointCount++;
		}
	}
	SemanticLidarBuffer[0] = PointCount;
}


void URaycastSemanticLidar::ComputeRawDetection(const FHitResult& HitInfo, const FTransform& SensorTransf, FSemanticDetection& Detection) const
{
	const FVector HitPoint = HitInfo.ImpactPoint;
	Detection.point = static_cast<FVector3f>(SensorTransf.Inverse().TransformPosition(HitPoint));

	
	const FVector VecInc = - (HitPoint - SensorTransf.GetLocation()).GetSafeNormal();
	Detection.cos_inc_angle = FVector::DotProduct(VecInc, HitInfo.ImpactNormal);
	
	const AActor* actor = HitInfo.GetActor();
	
	Detection.object_idx = 2;
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

	FRotator LidarBodyRot = ActorTransf.Rotator();

	FRotator LaserRot(VerticalAngle, HorizontalAngle, 0); // float InPitch, float InYaw, float InRoll
	FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
		LaserRot,
		LidarBodyRot
	);

	const auto Range = Description.Range;
	FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;
	
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

	return false;
}









