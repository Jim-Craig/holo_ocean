// MIT License (c) 2019 BYU PCCL see LICENSE file

#pragma once

#include "Holodeck.h"

#include "HolodeckPawnController.h"
#include "DrewUV.h"
#include "DrewUVControlFins.h"

#include "DrewUVController.generated.h"

/**
* A Holodeck Turtle Agent Controller
*/
UCLASS()
class HOLODECK_API ADrewUVController : public AHolodeckPawnController
{
	GENERATED_BODY()

public:
	/**
	* Default Constructor
	*/
	ADrewUVController(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	/**
	* Default Destructor
	*/
	~ADrewUVController();

	void AddControlSchemes() override {
		// The default controller currently in ControlSchemes index 0 is the dynamics one. We push it back to index 1 with this code.

		// Thruster controller
		UDrewUVControlFins* Thrusters = NewObject<UDrewUVControlFins>();
		Thrusters->SetController(this);
		ControlSchemes.Insert(Thrusters, 0);
	}
};


// // MIT License (c) 2021 BYU FRoStLab see LICENSE file

// #pragma once

// #include "Holodeck.h"

// #include "HolodeckPawnController.h"
// #include "DrewUVControlThrusters.h"
// #include "DrewUVControlPD.h"
// #include "DrewUV.h"

// #include "DrewUVController.generated.h"

// /**
// * A Holodeck Turtle Agent Controller
// */
// UCLASS()
// class HOLODECK_API ADrewUVController : public AHolodeckPawnController
// {
// 	GENERATED_BODY()

// public:
// 	/**
// 	* Default Constructor
// 	*/
// 	ADrewUVController(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

// 	/**
// 	* Default Destructor
// 	*/
// 	~ADrewUVController();

// 	void AddControlSchemes() override {
// 		// The default controller currently in ControlSchemes index 0 is the dynamics one. We push it back to index 2 with this code.

// 		// Thruster controller
// 		UDrewUVControlThrusters* Thrusters = NewObject<UDrewUVControlThrusters>();
// 		Thrusters->SetController(this);
// 		ControlSchemes.Insert(Thrusters, 0);

// 		// Position / orientation controller
// 		UDrewUVControlPD* PD = NewObject<UDrewUVControlPD>();
// 		PD->SetController(this);
// 		ControlSchemes.Insert(PD, 1);
// 	}
// };
