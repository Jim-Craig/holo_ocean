#pragma once

#include "Holodeck.h"

#include "HolodeckPawnController.h"
#include "BlueROV2.h"
#include "HolodeckControlScheme.h"
#include "SimplePID.h"
#include <math.h>

#include "BlueROV2ControlPD.generated.h"

const float BR_CONTROL_MAX_LIN_ACCEL = 1;
const float BR_CONTROL_MAX_ANG_ACCEL = 1;

const float BR_POS_P = 100;
const float BR_POS_D = 50;

const float BR_ROT_P = 0.1;
const float BR_ROT_D = 0.1;

/**
* BlueROV2ControlPD
*/
UCLASS()
class HOLODECK_API UBlueROV2ControlPD : public UHolodeckControlScheme {
public:
	GENERATED_BODY()

	UBlueROV2ControlPD(const FObjectInitializer& ObjectInitializer);

	void Execute(void* const CommandArray, void* const InputCommand, float DeltaSeconds) override;

	unsigned int GetControlSchemeSizeInBytes() const override {
		return 8 * sizeof(float);
	}

	void SetController(AHolodeckPawnController* const Controller) { BlueROV2Controller = Controller; };


private:
	AHolodeckPawnController* BlueROV2Controller;
	ABlueROV2* BlueROV2;
	SimplePID PositionController;
	SimplePID RotationController;
};

