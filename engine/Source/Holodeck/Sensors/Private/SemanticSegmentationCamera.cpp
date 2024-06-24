#include "SemanticSegmentationCamera.h"


USemanticSegmentationCamera::USemanticSegmentationCamera()
{
	SensorName = "RGBDCamera";

	//AddPostProcessingMaterial();
}

void USemanticSegmentationCamera::ParseSensorParms(FString ParmsJson)
{
	Super::ParseSensorParms(ParmsJson);
}

void USemanticSegmentationCamera::InitializeSensor()
{
	Super::InitializeSensor();
	
}

void USemanticSegmentationCamera::TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) {

	TickCounter++;
	if (TickCounter >= TicksPerCapture) {
		RenderRequest.RetrievePixels(Buffer, TargetTexture);

		TickCounter -= TicksPerCapture;
	}
}




