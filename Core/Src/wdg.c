#include "wdg.h"
#include "utils.h"

uint32_t wdg_timestamps_100us[WDG_NUM_BOARDS];

void wdg_reset(wdg_boards board, uint32_t timestamp_100us)
{
    wdg_timestamps_100us[board] = timestamp_100us;
}

uint8_t wdg_check()
{
    uint8_t boards = 0;

    for (uint8_t iboard = 0; iboard < WDG_NUM_BOARDS; iboard++)
    {
        // TODO: make overflow-safe
        if (ReturnTime_100us() > wdg_timestamps_100us[iboard] + wdg_timeouts_100us[iboard])
        {
            boards |= 1 << iboard;
        }
    }
    return boards;
}
