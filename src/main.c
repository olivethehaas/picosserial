#include "hardware/clocks.h"
#include "systick.h"

void enableSysTick(){
    systick_hw->rvr = M0PLUS_SYST_RVR_BITS;  // 24 bits
    systick_hw->csr = M0PLUS_SYST_CSR_CLKSOURCE_BITS | M0PLUS_SYST_CSR_ENABLE_BITS | M0PLUS_SYST_CSR_TICKINT_BITS; // enable with processor clock
  }

uint32_t hal_get_systick_value(){
    return((uint32_t)systick_hw->cvr) 
}

hal_get_systick_reload(){
    return((uint32_t)systick_hw->rvr)
}
uint32_t hal_get_systick_freq() {
  return (systick_hw->csr = M0PLUS_SYST_CSR_ENABLE_BITS | M0PLUS_SYST_CSR_CLKSOURCE_BITS);
}
