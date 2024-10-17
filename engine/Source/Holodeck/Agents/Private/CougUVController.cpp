// MIT License (c) 2019 BYU PCCL see LICENSE file

#include "Holodeck.h"
#include "CougUVController.h"

ACougUVController::ACougUVController(const FObjectInitializer& ObjectInitializer)
	: AHolodeckPawnController(ObjectInitializer) {
	UE_LOG(LogTemp, Warning, TEXT("CougUV Controller Initialized"));
}

ACougUVController::~ACougUVController() {}
