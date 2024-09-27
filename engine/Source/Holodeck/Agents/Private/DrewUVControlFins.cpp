#include "Holodeck.h"
#include "DrewUVControlFins.h"


UDrewUVControlFins::UDrewUVControlFins(const FObjectInitializer& ObjectInitializer) :
		Super(ObjectInitializer) {}

void UDrewUVControlFins::Execute(void* const CommandArray, void* const InputCommand, float DeltaSeconds) {
	if (DrewUV == nullptr) {
		DrewUV = static_cast<ADrewUV*>(DrewUVController->GetPawn());
		if (DrewUV == nullptr) {
			UE_LOG(LogHolodeck, Error, TEXT("UDrewUVControlFins couldn't get DrewUV reference"));
			return;
		}
		
		DrewUV->EnableDamping();
	}

	float* InputCommandFloat = static_cast<float*>(InputCommand);
	float* CommandArrayFloat = static_cast<float*>(CommandArray);

	// Buoyancy forces
	DrewUV->ApplyBuoyantForce();

	// Propeller
	DrewUV->ApplyThrust(InputCommandFloat[4]);
	
	// Apply fin forces
	for(int i=0;i<4;i++){
		DrewUV->ApplyFin(i, InputCommandFloat[i]);
	}

	// Zero out the physics based controller
	for(int i=0; i<6; i++){
		CommandArrayFloat[i] = 0;
	}
}