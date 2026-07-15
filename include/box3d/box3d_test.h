// Optional test hooks used by the headless sample-parity runner.
// Kept out of box3d.h so normal consumers see the unmodified public API.
#pragma once

#include "box3d.h"

B3_API int b3World_GetBodyCapacity( b3WorldId worldId );
B3_API b3BodyId b3World_GetBodyByIndex( b3WorldId worldId, int index );
