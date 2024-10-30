#include "Tagger.h"


template <typename T>
static auto CastEnum(T label)
{
	return static_cast<typename std::underlying_type_t<T>>(label);
}

holoocean::ObjectLabel ATagger::GetLabelByFolderName(const FString& String)
{
	if (String == "Boats") return holoocean::ObjectLabel::Boat;
	else if (String == "OceanFloor") return holoocean::ObjectLabel::Ground;
	else if (String == "Yachts") return holoocean::ObjectLabel::Yacht;
	else if (String == "ContainerShips") return holoocean::ObjectLabel::ContainerBoat;
	else if (String == "DamEnvironment") return holoocean::ObjectLabel::DamEnvironment;
	else if (String == "Landscape") return holoocean::ObjectLabel::Landscape;
	else return holoocean::ObjectLabel::None;
}


void ATagger::SetStencilValue(UPrimitiveComponent& Component, const holoocean::ObjectLabel& Label, const bool bShouldSetRenderCustomDepth)
{
	Component.SetCustomDepthStencilValue(CastEnum(Label));
	Component.SetRenderCustomDepth(bShouldSetRenderCustomDepth && (Label != holoocean::ObjectLabel::None));
}

// Carla's implementation looks for all things like pedestrians, traffic signs, etc. instead
bool ATagger::IsThing(const holoocean::ObjectLabel& Label)
{
	return (Label != holoocean::ObjectLabel::None);
}

FLinearColor ATagger::GetActorLabelColor(const AActor& Actor, const holoocean::ObjectLabel& Label)
{
	uint32 id = Actor.GetUniqueID();

	FLinearColor Color(0.0f, 0.0f, 0.0f, 1.0f);
	Color.R = CastEnum(Label) / 255.0f;
	Color.G = ((id & 0x00ff) >> 0) / 255.0f;
	Color.B = ((id & 0xff00) >> 8) / 255.0f;

	return Color;
}


void ATagger::TagActor(const AActor& Actor, bool bShouldTagForSemanticSegmentation)
{
	TArray<UStaticMeshComponent*> StaticMeshComponents;
	Actor.GetComponents<UStaticMeshComponent>(StaticMeshComponents);

	for (UStaticMeshComponent* Component : StaticMeshComponents)
	{
		if (Actor.Tags.Num() > 0)
		{
			auto Label = GetLabelByFolderName(Actor.Tags[0].ToString());
			//UE_LOG(LogHolodeck, Error, TEXT("TAG FOUND!: %s"), *Actor.Tags[0].ToString());
			SetStencilValue(*Component, Label, bShouldTagForSemanticSegmentation);

			if (!Component->IsVisible() || !Component->GetStaticMesh())
			{
				continue;
			}
		}
		else
		{
			auto Label = GetLabelByPath(Component->GetStaticMesh());

			SetStencilValue(*Component, Label, bShouldTagForSemanticSegmentation);

			if (!Component->IsVisible() || !Component->GetStaticMesh())
			{
				continue;
			}
		}
	}
}

void ATagger::TagActorsInLevel(UWorld& World, bool bShouldTagForSemanticSegmentation)
{
	for (TActorIterator<AActor> it(&World); it; ++it) {
		TagActor(**it, bShouldTagForSemanticSegmentation);
	}
}

void ATagger::TagActorsInLevel(ULevel& Level, bool bShouldTagForSemanticSegmentation)
{
	for (AActor* Actor : Level.Actors) {
		TagActor(*Actor, bShouldTagForSemanticSegmentation);
	}
}

void ATagger::GetTagsOfTaggedActor(const AActor& Actor, TSet<holoocean::ObjectLabel>& Tags)
{
	TArray<UPrimitiveComponent*> Components;
	Actor.GetComponents<UPrimitiveComponent>(Components);

	for (auto* Component : Components)
	{
		if (Component != nullptr)
		{
			const auto Tag = GetTagOfTaggedComponent(*Component);
			if (Tag != holoocean::ObjectLabel::None)
			{
				Tags.Add(Tag);
			}
		}
	}
}

FString ATagger::GetTagAsString(holoocean::ObjectLabel Tag)
{
	switch (Tag) {
#define HOLOOCEAN_GET_LABEL_STR(lbl) case holoocean::ObjectLabel:: lbl : return TEXT(#lbl);
	default:
		HOLOOCEAN_GET_LABEL_STR(None)
		HOLOOCEAN_GET_LABEL_STR(Boat)
		HOLOOCEAN_GET_LABEL_STR(Ground)
		HOLOOCEAN_GET_LABEL_STR(Yacht)
		HOLOOCEAN_GET_LABEL_STR(ContainerBoat)
		HOLOOCEAN_GET_LABEL_STR(DamEnvironment)
		HOLOOCEAN_GET_LABEL_STR(Landscape)

		HOLOOCEAN_GET_LABEL_STR(Any)

	#undef HOLOOCEAN_GET_LABEL_STR
	  }
}

#if WITH_EDITOR
void ATagger::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	if (PropertyChangedEvent.Property) {
		if (bTriggerTagObjects && (GetWorld() != nullptr)) {
			TagActorsInLevel(*GetWorld(), bShouldTagForSemanticSegmentation);
		}
	}
	bTriggerTagObjects = false;
}
#endif // WITH_EDITOR



ATagger::ATagger()
{
	/*
	if (GetWorld())
	{
		UE_LOG(LogHolodeck, Warning, TEXT("Tagging all actors in level"));

		TagActorsInLevel(*GetWorld(), true);
	}
	*/
	
}




