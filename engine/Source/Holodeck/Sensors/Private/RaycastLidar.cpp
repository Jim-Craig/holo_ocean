#include "RaycastLidar.h"

#include <cmath>
#include <Engine/CollisionProfile.h>

#include "RandomEngine.h"

URaycastLidar::URaycastLidar()
{
	SensorName = "RaycastLidar";
	Parent = nullptr;
	DropOffAlpha = 0.0f;
	DropOffBeta = 0.0f;
	DropOffGenActive = false;
	
	RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
}

void URaycastLidar::ParseSensorParms(FString ParmsJson)
{
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

void URaycastLidar::InitializeSensor()
{
	Super::ShouldInitialize = false;

	Super::InitializeSensor();

	Parent = TUniquePtr<AActor>(this->GetAttachmentRootActor());
	

	this->Buffer = static_cast<float*>(Super::Buffer);
	this->Data = FLidarData(this->Buffer);
}

void URaycastLidar::TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	SimulateLidar(DeltaTime);
}

void URaycastLidar::Set(const FLidarDescription& LidarDescription)
{
	Description = LidarDescription;
	UE_LOG(LogHolodeck, Error, TEXT("Range: %f"), Description.Range)

	CreateLasers();
	PointsPerChannel.resize(Description.Channels);

	// Compute drop off model parameters
	DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
	DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
	DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
}

float URaycastLidar::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
	const FVector3f HitPoint = RawDetection.point;
	const float Distance = HitPoint.Length();
	UE_LOG(LogHolodeck, Error, TEXT("Compute Intensity Distance: %f"), Distance)
	const float AttenAtm = Description.AtmospAttenRate;
	const float AbsAtm = exp(-AttenAtm * Distance);

	const float IntRec = AbsAtm;

	return IntRec;
}

URaycastLidar::FDetection URaycastLidar::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
{
	FDetection Detection;
	const FVector HitPoint = HitInfo.ImpactPoint;
	Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

	const float Distance = Detection.point.Length();
	const float AttenAtm = Description.AtmospAttenRate;
	const float AbsAtm = exp(-AttenAtm * Distance);

	const float IntRec = AbsAtm;

	Detection.intensity = IntRec;

	return Detection;
}

void URaycastLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel)
{
	Super::PreprocessRays(Channels, MaxPointsPerChannel);

	for (auto ch = 0u; ch < Channels; ch++)
	{
		for (auto p = 0u; p < MaxPointsPerChannel; p++)
		{
			RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
		}
	}
}

bool URaycastLidar::PostprocessDetection(FDetection& Detection) const
{
	if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon())
	{
		const auto ForwardVector = Detection.point.GetSafeNormal();
		UE_LOG(LogHolodeck, Error, TEXT("%f, %f, %f"), ForwardVector.X, ForwardVector.Y, ForwardVector.Z);
		const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
		Detection.point += Noise;
	}

	const float Intensity = Detection.intensity;
	if (Intensity > Description.DropOffIntensityLimit)
	{
		return true;
	}

	return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
}

void URaycastLidar::ComputeAndSaveDetections(const FTransform& SensorTransform)
{
	for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
	{
		PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
	}
	Data.ResetMemory(PointsPerChannel);
	int PointCount = 0;
	for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
	{
		for (auto& hit : RecordedHits[idxChannel])
		{
			FDetection Detection = ComputeDetection(hit, SensorTransform);
			if (PostprocessDetection(Detection))
			{
				Data.WritePointSync(Detection);
				PointCount++;
			}
			else
			{
				PointsPerChannel[idxChannel]--;
			}
		}
	}
	Buffer[0] = PointCount;
}












