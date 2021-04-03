#ifndef _INCLUDE_HPP_
#define _INCLUDE_HPP_

#include "boards.hpp"


#ifndef HW_BOARD
	#error "Not defined `HW_BOARD` (config.hpp)."
#endif

#ifndef HW_VERSION
	#error "Not defined `HW_VERSION` (config.hpp)."
#endif


#if HW_BOARD == BOARD_PROTOTYPE
	#warning "Compiling for board 'Prototype'."
	#error "Invalid `HW_VERSION` (config.hpp)."

#elif HW_BOARD == BOARD_MAD_2
	#warning "Compiling for board 'm328_nrf24'."

	#if HW_VERSION == 100
		#include "MAD_2/v100/adc.hpp"
		#include "MAD_2/v100/clk.hpp"
		#include "MAD_2/v100/gpio.hpp"
		#include "MAD_2/v100/rf.hpp"
		#include "MAD_2/v100/debug.hpp"
	#else
		#error "Invalid `HW_VERSION` (config.hpp)."
	#endif

#else
	#error "Invalid `HW_BOARD` (config.hpp)."
#endif


#endif /* _INCLUDE_HPP_ */

/*** end of file ***/
