// MIT License (c) 2021 BYU FRoStLab see LICENSE file

#include "Holodeck.h"
#include "FixedWingController.h"

AFixedWingController::AFixedWingController(const FObjectInitializer& ObjectInitializer)
	: AHolodeckPawnController(ObjectInitializer) {
	UE_LOG(LogTemp, Warning, TEXT("FixedWing Controller Initialized"));
}

AFixedWingController::~AFixedWingController() {}
