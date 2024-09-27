// MIT License (c) 2019 BYU PCCL see LICENSE file

#include "Holodeck.h"
#include "DrewUVController.h"

ADrewUVController::ADrewUVController(const FObjectInitializer& ObjectInitializer)
	: AHolodeckPawnController(ObjectInitializer) {
	UE_LOG(LogTemp, Warning, TEXT("DrewUV Controller Initialized"));
}

ADrewUVController::~ADrewUVController() {}
