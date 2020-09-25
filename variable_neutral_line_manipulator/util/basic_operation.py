from .growing_list import GrowingList

def indices_entity_pairs_to_ordered_list(indices_entity_pairs):
    growing_list = GrowingList()
    for indices, entity in indices_entity_pairs:
        for i in indices:
            growing_list[i] = entity
    growing_list.shrink()
    return growing_list