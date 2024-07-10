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
	// TODO: Parse all the config options for the lidar
}

void URaycastLidar::InitializeSensor()
{
	Super::InitializeSensor();

	Parent = TUniquePtr<AActor>(this->GetAttachmentRootActor());

	LidarDescription = FLidarDescription();
	Set(LidarDescription);

	this->Buffer = static_cast<float*>(Super::Buffer);
	this->Data = FLidarData(this->Buffer);

}

void URaycastLidar::TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	SimulateLidar(DeltaTime);

	//UE_LOG(LogHolodeck, Error, TEXT("%llu number of elements!"), Buffer->CountOfPoints())

	//Buffer = Data.points.data();
	//Buffer = std::move(Data.points.data());
	
	//float* FloatBuffer = static_cast<float*>(Buffer);
	/*{
		SCOPE_CYCLE_COUNTER(STAT_CopyBuffer)
		for (int i = 0; i < Data.NumPoints - 1; i++)
		{
			Buffer[4 * i] = Data.points[4 * i];
			Buffer[4 * i + 1] = Data.points[4 * i + 1];
			Buffer[4 * i + 2] = Data.points[4 * i + 2];
			Buffer[4 * i + 3] = Data.points[4 * i + 3];

			/*if (i < Data.CountOfPoints())
			{
				Buffer[i] = Data.points[i];
			}#1#
		}
	}*/
}

void URaycastLidar::Set(const FLidarDescription& LidarDescription)
{
	Description = LidarDescription;

	/*if (Buffer != nullptr)
	{
		delete Buffer;
	}*/
	
	//Buffer = new FLidarData(Description.Channels);
	//TempData = FLidarData(Description.Channels);
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
	else
	{
		return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
	}
}

void URaycastLidar::ComputeAndSaveDetections(const FTransform& SensorTransform)
{
	for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
	{
		PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
	}

	//Buffer->ResetMemory(PointsPerChannel);

	Data.ResetMemory(PointsPerChannel);

	int index = 0;

	for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
	{
		for (auto& hit : RecordedHits[idxChannel])
		{
			FDetection Detection = ComputeDetection(hit, SensorTransform);
			if (PostprocessDetection(Detection))
			{
				//Buffer->WritePointSync(Detection);
				Data.WritePointSync(Detection);
			}
			else
			{
				PointsPerChannel[idxChannel]--;
			}
		}

		// Reset the buffer for the data that wasn't overwritten
		/*if (Data.LastFrameMaxPoints > Data.NumPoints)
		{*/

			//UE_LOG(LogHolodeck, Warning, TEXT("%d last frame max points"), Data.LastFrameMaxPoints)
			//UE_LOG(LogHolodeck, Warning, TEXT("%d num points"), Data.NumPoints)
			/*for (int i = Data.NumPoints; i < Data.LastFrameMaxPoints; i++)
			{
				Data.Buffer[4 * i] = 0.0f;
				Data.Buffer[4 * i + 1] = 0.0f;
				Data.Buffer[4 * i + 2] = 0.0f;
				Data.Buffer[4 * i + 3] = 0.0f;
			}*/
		//}
	}
}












