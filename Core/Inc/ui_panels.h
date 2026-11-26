/*****************************************************************************
 * ui_panels.h
 *
 * The 9 UI panels from the EE426 mockup, drawn for the Waveshare 1.51"
 * transparent OLED (128x64 landscape, 1-bit).  Lit pixel = WHITE.
 *
 * Intended use:
 *   UI_Init();                              // once at startup
 *   UI_ShowPanel(UI_PANEL_SWITCH_MODE, &d); // whenever the state machine
 *                                           // wants a different screen / a
 *                                           // progress-bar update
 *
 * The drawing code knows nothing about buttons or timers, so the future
 * state machine only has to pick a panel and fill in UI_Data_t.
 *****************************************************************************/
#ifndef UI_PANELS_H
#define UI_PANELS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Timing taken from the mockup (for the future state machine) -------- */
#define UI_READY_DISPLAY_MS      1000U  /* SYSTEM READY on screen             */
#define UI_RESULT_DISPLAY_MS     2000U  /* distance result screens            */
#define UI_AGAIN_TIMEOUT_MS      5000U  /* AGAIN? -> power off if no press    */
#define UI_SWITCH_SHOW_AFTER_MS  1000U  /* trigger held >1 s: SWITCH MODE?    */
#define UI_SWITCH_COMMIT_MS      3000U  /* trigger held >3 s: bar full, switch*/

/* ---- Panels, in mockup order -------------------------------------------- */
typedef enum {
    UI_PANEL_BOOTING = 0,       /* 1 SYSTEM BOOTING + progress bar            */
    UI_PANEL_READY,             /* 2 SYSTEM READY                             */
    UI_PANEL_CROSSHAIR_RANGE,   /* 3 crosshair, MODE: RANGE FINDER            */
    UI_PANEL_SWITCH_MODE,       /* 4 SWITCH MODE? + progress bar              */
    UI_PANEL_DIST_TARGET_A,     /* 5 DISTANCE TO TARGET A + boxed value       */
    UI_PANEL_DIST_TARGET,       /* 6 DISTANCE TO TARGET + boxed value         */
    UI_PANEL_CROSSHAIR_TWO,     /* 7 crosshair, MODE: TWO POINT               */
    UI_PANEL_DIST_BETWEEN,      /* 8 DISTANCE BETWEEN TARGETS + A / B lines   */
    UI_PANEL_AGAIN,             /* 9 AGAIN?                                   */
    UI_PANEL_COUNT
} UI_Panel_t;

/* ---- Values the panels display ------------------------------------------                        */
typedef struct {
    uint8_t progress_pct;   /* 0..100: UI_PANEL_BOOTING, UI_PANEL_SWITCH_MODE */
    int32_t distance_cm;    /* UI_PANEL_DIST_TARGET_A, UI_PANEL_DIST_TARGET   */
    int32_t target_a_cm;    // UI_PANEL_DISTA
    int32_t target_b_cm;    // UI_PANEL_DISTB
    int32_t between_cm;     // UI_PANEL_DIST_BETWEEN
} UI_Data_t;

/* Initialise the display and the drawing buffer. Call once. */
void UI_Init(void);

/* Render one panel and push it to the display.
 * Only the fields relevant to that panel are read; data may be NULL for
 * panels that show no values.  Call it again with a new progress_pct to
 * animate a progress bar. */
void UI_ShowPanel(UI_Panel_t panel, const UI_Data_t *data);

/* Convert how long the trigger has been held into SWITCH MODE? progress:
 * 0 % at 1 s (when the panel appears) up to 100 % at 3 s (switch commits). */
uint8_t UI_HoldToPercent(uint32_t hold_ms);

/* TEST ONLY: cycles through all 9 panels forever (blocking), animating both
 * progress bars.  Remove once the real state machine drives UI_ShowPanel(). */
void UI_Test_Cycle(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_PANELS_H */
