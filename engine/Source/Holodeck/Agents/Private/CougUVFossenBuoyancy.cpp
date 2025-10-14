#include "Holodeck.h"
#include "CougUVFossenBuoyancy.h"

UCougUVFossenBuoyancy::UCougUVFossenBuoyancy(const FObjectInitializer& ObjectInitializer) :
		Super(ObjectInitializer) {}

void UCougUVFossenBuoyancy::Execute(void* const CommandArray, void* const InputCommand, float DeltaSeconds) {
	if (CougUV == nullptr) {
		CougUV = static_cast<ACougUV*>(CougUVController->GetPawn());
		if (CougUV == nullptr) {
			UE_LOG(LogHolodeck, Error, TEXT("UCougUVControlFins couldn't get CougUV reference"));
			return;
		}
		
		// CougUV->EnableDamping();
	}

	float* InputCommandFloat = static_cast<float*>(InputCommand);
	float* CommandArrayFloat = static_cast<float*>(CommandArray);

	// Convert linear acceleration to force
	FVector linAccel = FVector(InputCommandFloat[0], InputCommandFloat[1], InputCommandFloat[2]);
	linAccel = ClampVector(linAccel, -FVector(CUV_MAX_LIN_ACCEL), FVector(CUV_MAX_LIN_ACCEL));
	linAccel = ConvertLinearVector(linAccel, ClientToUE);

	// Convert angular acceleration to torque
	FVector angAccel = FVector(InputCommandFloat[3], InputCommandFloat[4], InputCommandFloat[5]);
	angAccel = ClampVector(angAccel, -FVector(CUV_MAX_ANG_ACCEL), FVector(CUV_MAX_ANG_ACCEL));
	angAccel = ConvertAngularVector(angAccel, NoScale);

	CougUV->RootMesh->GetBodyInstance()->AddForce(linAccel, true, true);
	CougUV->RootMesh->GetBodyInstance()->AddTorqueInRadians(angAccel, true, true);

    UE_LOG(LogHolodeck, Log, TEXT("Fossen Buoyancy Control Scheme Active!"));

	CougUV->ApplyBuoyancyDragForce();

	// Zero out the physics based controller
	for(int i=0; i<6; i++){
		CommandArrayFloat[i] = 0;
	}
}