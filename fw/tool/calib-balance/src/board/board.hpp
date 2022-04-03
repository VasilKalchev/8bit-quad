#pragma once

#if BOARD == BRD_MAD0
	#include "mad0/mad0.hpp"
#else
	#error "Unrecognized board"
#endif
