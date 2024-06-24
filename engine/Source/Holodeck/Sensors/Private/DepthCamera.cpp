#include "DepthCamera.h"


#include "Engine.h"

#include "Engine/SceneCapture2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/GameplayStatics.h"
#include "ShowFlags.h"

#include "Materials/Material.h"

#include "RHICommandList.h"


#include "Modules/ModuleManager.h"

UDepthCamera::UDepthCamera()
{
	SensorName = "DepthCamera";
	Parent = nullptr;
}

void UDepthCamera::ParseSensorParms(FString ParmsJson)
{
	Super::ParseSensorParms(ParmsJson);

	TSharedPtr<FJsonObject> JsonParsed;
	TSharedRef<TJsonReader<TCHAR>> JsonReader = TJsonReaderFactory<TCHAR>::Create(ParmsJson);
	if (FJsonSerializer::Deserialize(JsonReader, JsonParsed)) {
		
		if (JsonParsed->HasTypedField<EJson::Number>("TicksPerCapture")) {
			TicksPerCapture = JsonParsed->GetIntegerField("TicksPerCapture");
		}
	} else {
		UE_LOG(LogHolodeck, Fatal, TEXT("UDepthCamera::ParseSensorParms:: Unable to parse json."));
	}
}

void UDepthCamera::InitializeSensor()
{
	Super::InitializeSensor();
	Parent =  this->GetAttachmentRootActor();

	SceneCapture->CaptureSource = SCS_SceneColorSceneDepth;
}

void UDepthCamera::TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) {

	TickCounter++;
	if (TickCounter >= TicksPerCapture) {
		RenderRequest.RetrievePixels(Buffer, TargetTexture);
		TickCounter -= TicksPerCapture;
	}
}




