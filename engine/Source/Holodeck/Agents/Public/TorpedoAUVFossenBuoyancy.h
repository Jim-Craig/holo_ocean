#pragma once

#include "Holodeck.h"

#include "HolodeckPawnController.h"
#include "TorpedoAUV.h"
#include "HolodeckControlScheme.h"
#include <math.h>

#include "TorpedoAUVFossenBuoyancy.generated.h"

/**
* UTorpedoAUVFossenBuoyancy
*/
UCLASS()
class HOLODECK_API UTorpedoAUVFossenBuoyancy : public UHolodeckControlScheme {
public:
	GENERATED_BODY()

	UTorpedoAUVFossenBuoyancy(const FObjectInitializer& ObjectInitializer);

	void Execute(void* const CommandArray, void* const InputCommand, float DeltaSeconds) override;

	/** NOTE: These go counter-clockwise, starting in front right
	* 0: Left Fin
	* 1: Top Fin
	* 2: Right Fin
	* 3: Bottom Fin
	* 4: Thruster
	*/
	unsigned int GetControlSchemeSizeInBytes() const override {
		return 6 * sizeof(float);
	}

	void SetController(AHolodeckPawnController* const Controller) { TorpedoAUVController = Controller; };

private:
    // Accelerations
	float CommandArray[6];
	AHolodeckPawnController* TorpedoAUVController;
	ATorpedoAUV* TorpedoAUV;
};
