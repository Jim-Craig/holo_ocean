#pragma once

#include "CoreMinimal.h"
#include "CameraSensor.h"
#include "GameFramework/Actor.h"
#include "Containers/Queue.h"
#include "Holodeck.h"


#include "RGBDCamera.generated.h"
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class HOLODECK_API URGBDCamera : public UCameraSensor
{
	GENERATED_BODY()

public:
	URGBDCamera();

	/**
		* InitializeSensor
		* Sets up the class
		*/
	virtual void InitializeSensor() override;

	/**
	* Allows parameters to be set dynamically
	*/
	virtual void ParseSensorParms(FString ParmsJson) override;

protected:
	virtual void TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;


private:
	int TickCounter = 0;
	AActor* Parent;
};