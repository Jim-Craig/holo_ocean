#include "RGBDCamera.h"


#include "Engine.h"

#include "Engine/SceneCapture2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/GameplayStatics.h"
#include "ShowFlags.h"

#include "Materials/Material.h"

#include "RHICommandList.h"


#include "Modules/ModuleManager.h"

URGBDCamera::URGBDCamera()
{
	SensorName = "RGBDCamera";
	Parent = nullptr;
}

void URGBDCamera::ParseSensorParms(FString ParmsJson)
{
	Super::ParseSensorParms(ParmsJson);
}

void URGBDCamera::InitializeSensor()
{
	Super::InitializeSensor();
	Parent =  this->GetAttachmentRootActor();
	
	SceneCapture->CaptureSource = SCS_SceneColorSceneDepth;
}

void URGBDCamera::TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) {

	TickCounter++;
	if (TickCounter >= TicksPerCapture) {
		RenderRequest.RetrievePixels(Buffer, TargetTexture);

		TickCounter -= TicksPerCapture;
	}
}




