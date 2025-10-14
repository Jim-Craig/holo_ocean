#include "Holodeck.h"
#include "TorpedoAUVFossenBuoyancy.h"


UTorpedoAUVFossenBuoyancy::UTorpedoAUVFossenBuoyancy(const FObjectInitializer& ObjectInitializer) :
		Super(ObjectInitializer) {}

void UTorpedoAUVFossenBuoyancy::Execute(void* const CommandArray, void* const InputCommand, float DeltaSeconds) {
	if (TorpedoAUV == nullptr) {
		TorpedoAUV = static_cast<ATorpedoAUV*>(TorpedoAUVController->GetPawn());
		if (TorpedoAUV == nullptr) {
			UE_LOG(LogHolodeck, Error, TEXT("UTorpedoAUVControlFins couldn't get TorpedoAUV reference"));
			return;
		}
		
		// TorpedoAUV->EnableDamping();
	}

	float* InputCommandFloat = static_cast<float*>(InputCommand);
	float* CommandArrayFloat = static_cast<float*>(CommandArray);

	// Convert linear acceleration to force
	FVector linAccel = FVector(InputCommandFloat[0], InputCommandFloat[1], InputCommandFloat[2]);
	linAccel = ConvertLinearVector(linAccel, ClientToUE);

	// Convert angular acceleration to torque
	FVector angAccel = FVector(InputCommandFloat[3], InputCommandFloat[4], InputCommandFloat[5]);
	angAccel = ConvertAngularVector(angAccel, NoScale);


	TorpedoAUV->RootMesh->GetBodyInstance()->AddForce(linAccel, true, true);
	TorpedoAUV->RootMesh->GetBodyInstance()->AddTorqueInRadians(angAccel, true, true);

    UE_LOG(LogHolodeck, Log, TEXT("Fossen Buoyancy Control Scheme Active!"));

	TorpedoAUV->ApplyBuoyancyDragForce();

	// Zero out the physics based controller
	for(int i=0; i<6; i++){
		CommandArrayFloat[i] = 0;
	}
}