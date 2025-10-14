// MIT License (c) 2019 BYU PCCL see LICENSE file

#pragma once

#include "Holodeck.h"

#include "HolodeckPawnController.h"
#include "CougUV.h"
#include "CougUVControlFins.h"
#include "CougUVFossenBuoyancy.h"

#include "CougUVController.generated.h"

/**
* A Holodeck Turtle Agent Controller
*/
UCLASS()
class HOLODECK_API ACougUVController : public AHolodeckPawnController
{
	GENERATED_BODY()

public:
	/**
	* Default Constructor
	*/
	ACougUVController(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	/**
	* Default Destructor
	*/
	~ACougUVController();

	void AddControlSchemes() override {
		// The default controller currently in ControlSchemes index 0 is the dynamics one. We push it back to index 1 with this code.

		// Thruster controller
		UCougUVControlFins* Thrusters = NewObject<UCougUVControlFins>();
		Thrusters->SetController(this);
		ControlSchemes.Insert(Thrusters, 0);

		// Add Fossen Buoyancy controller
		UCougUVFossenBuoyancy* FossenBuoyancy = NewObject<UCougUVFossenBuoyancy>();
		FossenBuoyancy->SetController(this);
		ControlSchemes.Insert(FossenBuoyancy, 2);	
	}
};


// // MIT License (c) 2021 BYU FRoStLab see LICENSE file

// #pragma once

// #include "Holodeck.h"

// #include "HolodeckPawnController.h"
// #include "CougUVControlThrusters.h"
// #include "CougUVControlPD.h"
// #include "CougUV.h"

// #include "CougUVController.generated.h"

// /**
// * A Holodeck Turtle Agent Controller
// */
// UCLASS()
// class HOLODECK_API ACougUVController : public AHolodeckPawnController
// {
// 	GENERATED_BODY()

// public:
// 	/**
// 	* Default Constructor
// 	*/
// 	ACougUVController(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

// 	/**
// 	* Default Destructor
// 	*/
// 	~ACougUVController();

// 	void AddControlSchemes() override {
// 		// The default controller currently in ControlSchemes index 0 is the dynamics one. We push it back to index 2 with this code.

// 		// Thruster controller
// 		UCougUVControlThrusters* Thrusters = NewObject<UCougUVControlThrusters>();
// 		Thrusters->SetController(this);
// 		ControlSchemes.Insert(Thrusters, 0);

// 		// Position / orientation controller
// 		UCougUVControlPD* PD = NewObject<UCougUVControlPD>();
// 		PD->SetController(this);
// 		ControlSchemes.Insert(PD, 1);
// 	}
// };
