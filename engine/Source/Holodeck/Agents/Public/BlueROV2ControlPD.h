#pragma once

#include "Holodeck.h"

#include "HolodeckPawnController.h"
#include "BlueROV2.h"
#include "HolodeckControlScheme.h"
#include "SimplePID.h"
#include <math.h>

#include "BlueROV2ControlPD.generated.h"

const float BR_CONTROL_MAX_LIN_ACCEL = 1;
const float BR_CONTROL_MAX_ANG_ACCEL = 100;

const float BR_POS_P_X = 100;
const float BR_POS_I_X = 15;
const float BR_POS_D_X = 30;

const float BR_POS_P_Y = 100;
const float BR_POS_I_Y = 0;
const float BR_POS_D_Y = 50;

const float BR_POS_P_Z = 100;
const float BR_POS_I_Z = 0;
const float BR_POS_D_Z = 50;

const float BR_ROT_P_R = 0.1;
const float BR_ROT_I_R = 0;
const float BR_ROT_D_R = 0.1;

const float BR_ROT_P_P = 0.1;
const float BR_ROT_I_P = 0;
const float BR_ROT_D_P = 0.1;

const float BR_ROT_P_Y = 0.1;
const float BR_ROT_I_Y = 0;
const float BR_ROT_D_Y = 0.1;

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
	SimplePID PositionController_X;
	SimplePID PositionController_Y;
	SimplePID PositionController_Z;
	SimplePID RotationController_R;
	SimplePID RotationController_P;
	SimplePID RotationController_Y;
};

