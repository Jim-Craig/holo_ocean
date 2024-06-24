#include "CameraSensor.h"


#include "Engine.h"

#include "Engine/SceneCapture2D.h"

#include "RHICommandList.h"


#include "Modules/ModuleManager.h"

UCameraSensor::UCameraSensor()
{
	SensorName = "CameraSensor";
	Parent = nullptr;
}

void UCameraSensor::ParseSensorParms(FString ParmsJson)
{
	Super::ParseSensorParms(ParmsJson);

	SceneCapture = NewObject<USceneCaptureComponent2D>(this, "SceneCap");
	SceneCapture->RegisterComponent();
	SceneCapture->AttachToComponent(this, FAttachmentTransformRules::KeepRelativeTransform);
	
	TSharedPtr<FJsonObject> JsonParsed;
	TSharedRef<TJsonReader<TCHAR>> JsonReader = TJsonReaderFactory<TCHAR>::Create(ParmsJson);
	if (FJsonSerializer::Deserialize(JsonReader, JsonParsed)) {
		
		if (JsonParsed->HasTypedField<EJson::Number>("TicksPerCapture")) {
			TicksPerCapture = JsonParsed->GetIntegerField("TicksPerCapture");
		}

		if (JsonParsed->HasField("FovAngle"))
		{
			
			SetFOVAngle(static_cast<float>(JsonParsed->GetNumberField("FovAngle")));
		}

		
	} else {
		UE_LOG(LogHolodeck, Fatal, TEXT("UCameraSensor::ParseSensorParms:: Unable to parse json."));
	}
}

void UCameraSensor::InitializeSensor() {
	Super::InitializeSensor();

	Parent = this->GetAttachmentRootActor();
	
	
	//Set up everything for the SceneCaptureComponent2D
	SceneCapture->CaptureSource = SCS_FinalColorLDR; //Pick what type of output you want to be sent to the texture target.

		UE_LOG(LogHolodeck, Error, TEXT("FOV IS SET TO: %f"), SceneCapture->FOVAngle);
}


void UCameraSensor::TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) {

	TickCounter++;
	if (TickCounter >= TicksPerCapture) {
		RenderRequest.RetrievePixels(Buffer, TargetTexture);
		TickCounter -= TicksPerCapture;
	}
}

void UCameraSensor::SetFOVAngle(float FOVAngle)
{
	check(this->SceneCapture != nullptr);
	this->SceneCapture->FOVAngle = FOVAngle;
}

float UCameraSensor::GetFOVAngle() const
{
	check(this->SceneCapture != nullptr);
	return this->SceneCapture->FOVAngle;
}

void UCameraSensor::SetExposureMethod(EAutoExposureMethod Method)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.AutoExposureMethod = Method;
}

EAutoExposureMethod UCameraSensor::GetExposureMethod() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.AutoExposureMethod;
}

void UCameraSensor::SetExposureCompensation(float Compensation)
{
	check(SceneCapture != nullptr);

#if PLATFORM_LINUX
	// Looks like Windows and Linux have different outputs with the
	// same exposure compensation, this fixes it.
	SceneCapture->PostProcessSettings.AutoExposureBias = Compensation + 0.75f
#else
	SceneCapture->PostProcessSettings.AutoExposureBias = Compensation;
#endif

}

float UCameraSensor::GetExposureCompensation() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.AutoExposureBias;
}

void UCameraSensor::SetShutterSpeed(float Speed)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.CameraShutterSpeed = Speed;
}

float UCameraSensor::GetShutterSpeed() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.CameraShutterSpeed;
}

void UCameraSensor::SetISO(float ISO)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.CameraISO = ISO;
}

float UCameraSensor::GetISO() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.CameraISO;
}

void UCameraSensor::SetAperture(float Aperture)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.DepthOfFieldFstop = Aperture;
}

float UCameraSensor::GetAperture() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.DepthOfFieldFstop;
}

void UCameraSensor::SetFocalDistance(float Distance)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.DepthOfFieldFocalDistance = Distance;
}

float UCameraSensor::GetFocalDistance() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.DepthOfFieldFocalDistance;
}

void UCameraSensor::SetDepthBlurAmount(float Amount)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.DepthOfFieldDepthBlurAmount = Amount;
}

float UCameraSensor::GetDepthBlurAmount() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.DepthOfFieldDepthBlurAmount;
}

void UCameraSensor::SetDepthBlurRadius(float Radius)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.DepthOfFieldDepthBlurRadius = Radius;

}

float UCameraSensor::GetDepthBlurRadius() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.DepthOfFieldDepthBlurRadius;
}

void UCameraSensor::SetBladeCount(int Count)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.DepthOfFieldBladeCount = Count;
}

int UCameraSensor::GetBladeCount() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.DepthOfFieldBladeCount;
}

void UCameraSensor::SetDepthOfFieldMinFstop(float MinFstop)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.DepthOfFieldMinFstop = MinFstop;
}

float UCameraSensor::GetDepthOfFieldMinFstop() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.DepthOfFieldMinFstop;
}

void UCameraSensor::SetFilmSlope(float Slope)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.FilmSlope = Slope;
}

float UCameraSensor::GetFilmSlope() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.FilmSlope;
}

void UCameraSensor::SetFilmToe(float Toe)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.FilmToe = Toe;

}

float UCameraSensor::GetFilmToe() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.FilmToe;
}

void UCameraSensor::SetFilmShoulder(float Shoulder)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.FilmShoulder = Shoulder;
}

float UCameraSensor::GetFilmShoulder() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.FilmShoulder;
}

void UCameraSensor::SetFilmBlackClip(float BlackClip)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.FilmBlackClip = BlackClip;
}

float UCameraSensor::GetFilmBlackClip() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.FilmBlackClip;
}

void UCameraSensor::SetFilmWhiteClip(float WhiteClip)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.FilmWhiteClip = WhiteClip;

}

float UCameraSensor::GetFilmWhiteClip() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.FilmWhiteClip;
}

void UCameraSensor::SetExposureMinBrightness(float Brightness)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.AutoExposureMinBrightness = Brightness;
}

float UCameraSensor::GetExposureMinBrightness() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.AutoExposureMinBrightness;
}

void UCameraSensor::SetExposureMaxBrightness(float Brightness)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.AutoExposureMaxBrightness = Brightness;
}

float UCameraSensor::GetExposureMaxBrightness() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.AutoExposureMaxBrightness;
}

void UCameraSensor::SetExposureSpeedDown(float Speed)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.AutoExposureSpeedDown = Speed;
}

float UCameraSensor::GetExposureSpeedDown() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.AutoExposureSpeedDown;
}

void UCameraSensor::SetExposureSpeedUp(float Speed)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.AutoExposureSpeedUp = Speed;
}

float UCameraSensor::GetExposureSpeedUp() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.AutoExposureSpeedUp;
}

void UCameraSensor::SetMotionBlurIntensity(float Intensity)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.MotionBlurAmount = Intensity;
}	

float UCameraSensor::GetMotionBlurIntensity() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.MotionBlurAmount;
}

void UCameraSensor::SetMotionBlurMaxDistortion(float MaxDistortion)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.MotionBlurMax = MaxDistortion;
}

float UCameraSensor::GetMotionBlurMaxDistortion() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.MotionBlurMax;
}

void UCameraSensor::SetMotionBlurMinObjectScreenSize(float ScreenSize)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.MotionBlurPerObjectSize = ScreenSize;
}

float UCameraSensor::GetMotionBlurMinObjectScreenSize() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.MotionBlurPerObjectSize;
}

void UCameraSensor::SetLensFlareIntensity(float Intensity)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.LensFlareIntensity = Intensity;
}

float UCameraSensor::GetLensFlareIntensity() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.LensFlareIntensity;
}

void UCameraSensor::SetBloomIntensity(float Intensity)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.BloomIntensity = Intensity;
}

float UCameraSensor::GetBloomIntensity() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.BloomIntensity;
}

void UCameraSensor::SetWhiteTemp(float Temp)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.WhiteTemp = Temp;
}

float UCameraSensor::GetWhiteTemp() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.WhiteTemp;
}

void UCameraSensor::SetWhiteTint(float Tint)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.WhiteTint = Tint;
}

float UCameraSensor::GetWhiteTint() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.WhiteTint;
}

void UCameraSensor::SetChromAberrIntensity(float Intensity)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.SceneFringeIntensity = Intensity;
}

float UCameraSensor::GetChromAberrIntensity() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.SceneFringeIntensity;
}

void UCameraSensor::SetChromAberrOffset(float chromAberrOffset)
{
	check(SceneCapture != nullptr);
	SceneCapture->PostProcessSettings.ChromaticAberrationStartOffset = chromAberrOffset;
}

float UCameraSensor::GetChromAberrOffset() const
{
	check(SceneCapture != nullptr);
	return SceneCapture->PostProcessSettings.ChromaticAberrationStartOffset;
}

void UCameraSensor::EnqueueRenderSceneImmediate()
{
	GetCaptureComponent2D()->CaptureScene();
}















