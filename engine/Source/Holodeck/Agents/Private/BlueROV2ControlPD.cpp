#include "Holodeck.h"
#include "BlueROV2ControlPD.h"

UBlueROV2ControlPD::UBlueROV2ControlPD(const FObjectInitializer& ObjectInitializer) :
		Super(ObjectInitializer), PositionController(BR_POS_P, 0, BR_POS_D), RotationController(BR_ROT_P, 0, BR_ROT_D) { }

void UBlueROV2ControlPD::Execute(void* const CommandArray, void* const InputCommand, float DeltaSeconds) {
	if (BlueROV2 == nullptr) {

		BlueROV2 = static_cast<ABlueROV2*>(BlueROV2Controller->GetPawn());
		if (BlueROV2 == nullptr) {
			UE_LOG(LogHolodeck, Error, TEXT("UBlueROV2ControlPD couldn't get BlueROV2 reference"));
			return;
		}
		
		BlueROV2->EnableDamping();
	}

	// Apply gravity & buoyancy
	BlueROV2->ApplyBuoyantForce();
	float* InputCommandFloat = static_cast<float*>(InputCommand);
	float* CommandArrayFloat = static_cast<float*>(CommandArray);

	// ALL calculations here are done in HoloOcean frame & units. 

	// Get desired information
	FVector DesiredPosition = FVector(InputCommandFloat[0], InputCommandFloat[1], InputCommandFloat[2]);
	FVector DesiredOrientation = FVector(InputCommandFloat[3], InputCommandFloat[4], InputCommandFloat[5]);

	// Get current COM (frame we're moving), velocity, orientation, & ang. velocity
	FVector Position = BlueROV2->RootMesh->GetCenterOfMass();
	Position = ConvertLinearVector(Position, UEToClient);

	FVector LinearVelocity = BlueROV2->RootMesh->GetPhysicsLinearVelocity();
	LinearVelocity = ConvertLinearVector(LinearVelocity, UEToClient);

	FVector Orientation = RotatorToRPY(BlueROV2->GetActorRotation());

	FVector AngularVelocity = BlueROV2->RootMesh->GetPhysicsAngularVelocityInDegrees();
	AngularVelocity = ConvertAngularVector(AngularVelocity, NoScale);


	// Compute accelerations to apply

	FVector LinAccel, AngAccel;
	for(int i=0; i<3; i++){
		LinAccel[i] = PositionController.ComputePIDDirect(DesiredPosition[i], Position[i], LinearVelocity[i], DeltaSeconds);
		LinAccel[i] = FMath::Clamp(LinAccel[i], -BR_CONTROL_MAX_LIN_ACCEL, BR_CONTROL_MAX_LIN_ACCEL);

		AngAccel[i] = RotationController.ComputePIDDirect(DesiredOrientation[i], Orientation[i], AngularVelocity[i], DeltaSeconds, true, true);
		AngAccel[i] = FMath::Clamp(AngAccel[i], -BR_CONTROL_MAX_ANG_ACCEL, BR_CONTROL_MAX_ANG_ACCEL);
	}

	// Feedback linearize torque with buoyancy torque
	FRotator rotation = BlueROV2->GetActorRotation();

	FVector e3 = rotation.UnrotateVector(FVector(0,0,1));
	e3 = ConvertLinearVector(e3, NoScale);

	FVector COB = ConvertLinearVector(BlueROV2->CenterBuoyancy - BlueROV2->CenterMass, UEToClient);
	FVector tau = BlueROV2->Volume * BlueROV2->WaterDensity * 9.8 * FVector::CrossProduct(e3, COB);

	AngAccel += tau;

	FVector before = FVector(AngAccel);
	AngAccel = ConvertAngularVector(AngAccel, ClientToUE);
	AngAccel = rotation.RotateVector(AngAccel);
	AngAccel = ConvertAngularVector(AngAccel, UEToClient);
	// Fill in with the PD Control
	// Command array is then passed to vehicle as acceleration & angular velocity
	for(int i=0; i<3; i++){
		CommandArrayFloat[i] = LinAccel[i];
		CommandArrayFloat[i+3] = AngAccel[i];
	}
}

