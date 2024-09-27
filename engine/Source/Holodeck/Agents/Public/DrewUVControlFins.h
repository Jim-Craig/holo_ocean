#pragma once

#include "Holodeck.h"

#include "HolodeckPawnController.h"
#include "DrewUV.h"
#include "HolodeckControlScheme.h"
#include <math.h>

#include "DrewUVControlFins.generated.h"

/**
* UDrewUVControlFins
*/
UCLASS()
class HOLODECK_API UDrewUVControlFins : public UHolodeckControlScheme {
public:
	GENERATED_BODY()

	UDrewUVControlFins(const FObjectInitializer& ObjectInitializer);

	void Execute(void* const CommandArray, void* const InputCommand, float DeltaSeconds) override;

	/** NOTE: These go counter-clockwise, starting in front right
	* 0: Left Fin
	* 1: Top Fin
	* 2: Right Fin
	* 3: Bottom Fin
	* 4: Thruster
	*/
	unsigned int GetControlSchemeSizeInBytes() const override {
		return 5 * sizeof(float);
	}

	void SetController(AHolodeckPawnController* const Controller) { DrewUVController = Controller; };

private:
	AHolodeckPawnController* DrewUVController;
	ADrewUV* DrewUV;

};
