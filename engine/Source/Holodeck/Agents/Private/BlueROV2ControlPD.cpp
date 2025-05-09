#include "Holodeck.h"
#include "BlueROV2ControlPD.h"

UBlueROV2ControlPD::UBlueROV2ControlPD(const FObjectInitializer& ObjectInitializer) :
		Super(ObjectInitializer), 
		PositionController_X(BR_POS_P_X, BR_POS_I_X, BR_POS_D_X), 
		PositionController_Y(BR_POS_P_Y, BR_POS_I_Y, BR_POS_D_Y),
		PositionController_Z(BR_POS_P_Z, BR_POS_I_Z, BR_POS_D_Z),
		RotationController_R(BR_ROT_P_R, BR_ROT_I_R, BR_ROT_D_R),
		RotationController_P(BR_ROT_P_P, BR_ROT_I_P, BR_ROT_D_P),
		RotationController_Y(BR_ROT_P_Y, BR_ROT_I_Y, BR_ROT_D_Y) { }

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
	
	LinAccel[0] = PositionController_X.ComputePIDDirect(DesiredPosition[0], Position[0], LinearVelocity[0], DeltaSeconds);
	LinAccel[0] = FMath::Clamp(LinAccel[0], -BR_CONTROL_MAX_LIN_ACCEL, BR_CONTROL_MAX_LIN_ACCEL);
	LinAccel[1] = PositionController_Y.ComputePIDDirect(DesiredPosition[1], Position[1], LinearVelocity[1], DeltaSeconds);
	LinAccel[1] = FMath::Clamp(LinAccel[1], -BR_CONTROL_MAX_LIN_ACCEL, BR_CONTROL_MAX_LIN_ACCEL);
	LinAccel[2] = PositionController_Z.ComputePIDDirect(DesiredPosition[2], Position[2], LinearVelocity[2], DeltaSeconds);
	LinAccel[2] = FMath::Clamp(LinAccel[2], -BR_CONTROL_MAX_LIN_ACCEL, BR_CONTROL_MAX_LIN_ACCEL);
	
	AngAccel[0] = RotationController_R.ComputePIDDirect(DesiredOrientation[0], Orientation[0], AngularVelocity[0], DeltaSeconds, true, true);
	AngAccel[0] = FMath::Clamp(AngAccel[0], -BR_CONTROL_MAX_ANG_ACCEL, BR_CONTROL_MAX_ANG_ACCEL);
	AngAccel[1] = RotationController_P.ComputePIDDirect(DesiredOrientation[1], Orientation[1], AngularVelocity[1], DeltaSeconds, true, true);
	AngAccel[1] = FMath::Clamp(AngAccel[1], -BR_CONTROL_MAX_ANG_ACCEL, BR_CONTROL_MAX_ANG_ACCEL);
	AngAccel[2] = RotationController_Y.ComputePIDDirect(DesiredOrientation[2], Orientation[2], AngularVelocity[2], DeltaSeconds, true, true);
	AngAccel[2] = FMath::Clamp(AngAccel[2], -BR_CONTROL_MAX_ANG_ACCEL, BR_CONTROL_MAX_ANG_ACCEL);

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

