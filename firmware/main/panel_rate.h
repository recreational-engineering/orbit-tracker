#pragma once

// Pixel clock of the panel (MHz). With the board's porches a frame is 800 x 760 clocks, so
// 40 MHz refreshes at ~65 Hz (the board package's 80 MHz: ~131 Hz).
#define PANEL_DPI_MHZ 40
#define PANEL_REFRESH_US (800 * 760 / PANEL_DPI_MHZ)
