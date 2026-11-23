/*****************************************************************************
 * ui_panels.c  -  see ui_panels.h
 *
 * Screen is 128 x 64 (landscape).  Paint_NewImage(..., 270, ...) makes
 * Paint's logical coordinates (0,0) top-left, x to 127, y to 63.
 *
 * Font widths in this project: Font8=5, Font12=7, Font16=11, Font20=14,
 * Font24=17 px per character (heights 8/12/16/20/24).
 *****************************************************************************/
#include "ui_panels.h"
#include "GUI_Paint.h"
#include "OLED_1in51.h"
#include "DEV_Config.h"
#include <stdio.h>
#include <string.h>

/* ---- Screen geometry ---------------------------------------------------- */
#define SCR_W            128
#define SCR_H            64
#define FRAME_BYTES      ((OLED_1IN51_WIDTH / 8) * OLED_1IN51_HEIGHT)   /* 1024 */

/* ---- Strings (edit here).  Every string must fit its font/width. -------- */
#define STR_BOOTING      "SYSTEM BOOTING"
#define STR_READY        "SYSTEM READY"
#define STR_SWITCH       "SWITCH MODE?"
#define STR_AGAIN        "AGAIN?"
#define STR_DIST_A       "DISTANCE TO TARGET A"
#define STR_DIST         "DISTANCE TO TARGET"
#define STR_BETWEEN      "DISTANCE BETWEEN TARGETS:"
/* "DISTANCE TO TARGET A: 12.32m" is 28 chars = 140 px, wider than the 128 px
 * screen even in the smallest font, so the A/B lines are abbreviated.       */
#define STR_LINE_A       "DIST TO TARGET A: "
#define STR_LINE_B       "DIST TO TARGET B: "
/* Crosshair quadrants are only ~61 px wide ("RANGE FINDER" / "FIRST TARGET"
 * would be 60 px and touch the lines), so each message wraps to 3 lines.    */
#define STR_MODE_RANGE   { "MODE:",  "RANGE",  "FINDER" }
#define STR_MODE_TWO     { "MODE:",  "TWO",    "POINT"  }
#define STR_AIM_FIRST    { "AIM AT", "FIRST",  "TARGET" }
#define STR_AIM_NEXT     { "AIM AT", "NEXT",   "TARGET" }

/* ---- Progress-bar look -------------------------------------------------- */
#define BAR_X            10
#define BAR_W            108
#define BAR_H            10
#define BAR_PAD          1      /* gap between outline and fill (px)         */

/* ---- Crosshair look ----------------------------------------------------- */
#define XH_LINE          2      /* line thickness (px)                        */

static UBYTE s_frame[FRAME_BYTES];

/* =========================================================================
 * Low-level helpers
 * ========================================================================= */

/* Inclusive filled rectangle (Paint_DrawRectangle's fill excludes the last
 * row, so we do our own). Clipped to the screen. */
static void fill_rect(int x0, int y0, int x1, int y1)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > SCR_W - 1) x1 = SCR_W - 1;
    if (y1 > SCR_H - 1) y1 = SCR_H - 1;
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            Paint_SetPixel((UWORD)x, (UWORD)y, WHITE);
        }
    }
}

/* Inclusive 1 px rectangle outline. */
static void outline_rect(int x0, int y0, int x1, int y1)
{
    fill_rect(x0, y0, x1, y0);
    fill_rect(x0, y1, x1, y1);
    fill_rect(x0, y0, x0, y1);
    fill_rect(x1, y0, x1, y1);
}

static int text_w(const char *s, const sFONT *f)
{
    return (int)strlen(s) * (int)f->Width;
}

/* Draw text horizontally centred in [x_left, x_right]. */
static void text_centered(int x_left, int x_right, int y, const char *s, sFONT *f)
{
    int x = x_left + ((x_right - x_left + 1) - text_w(s, f)) / 2;
    if (x < 0) x = 0;
    Paint_DrawString_EN((UWORD)x, (UWORD)y, s, f, WHITE, BLACK);
}

/* "12.32m" from millimetres, integer maths only (no float printf needed).
 * Negative = invalid reading -> "--.--m". */
static void format_meters(char *buf, size_t n, int32_t mm)
{
    if (mm < 0) {
        snprintf(buf, n, "--.--m");
        return;
    }
    uint32_t cm = ((uint32_t)mm + 5U) / 10U;            /* round to 0.01 m */
    snprintf(buf, n, "%lu.%02lum",
             (unsigned long)(cm / 100U), (unsigned long)(cm % 100U));
}

/* Progress bar: 1 px outline, fill grows left to right with pct (0..100). */
static void draw_progress_bar(int x, int y, int w, int h, uint8_t pct)
{
    if (pct > 100) pct = 100;
    outline_rect(x, y, x + w - 1, y + h - 1);

    int in_x0 = x + 1 + BAR_PAD;
    int in_x1 = x + w - 2 - BAR_PAD;
    int in_y0 = y + 1 + BAR_PAD;
    int in_y1 = y + h - 2 - BAR_PAD;
    int in_w  = in_x1 - in_x0 + 1;

    int fill = (in_w * (int)pct + 50) / 100;            /* rounded */
    if (fill > 0) {
        fill_rect(in_x0, in_y0, in_x0 + fill - 1, in_y1);
    }
}

/* Three-line message inside a bottom crosshair quadrant (Font8, 9 px pitch). */
static void quadrant_text(int qx0, int qx1, const char *const lines[3])
{
    for (int i = 0; i < 3; i++) {
        text_centered(qx0, qx1, 35 + 9 * i, lines[i], &Font8);
    }
}

/* =========================================================================
 * Panel renderers (draw into s_frame only; UI_ShowPanel flushes)
 * ========================================================================= */

/* Title (Font12) with a progress bar underneath. Panels 1 and 4. */
static void render_title_bar(const char *title, uint8_t pct)
{
    text_centered(0, SCR_W - 1, 15, title, &Font12);
    draw_progress_bar(BAR_X, 36, BAR_W, BAR_H, pct);
}

static void render_crosshair(int two_point)
{
    /* outer border */
    fill_rect(0, 0, SCR_W - 1, XH_LINE - 1);
    fill_rect(0, SCR_H - XH_LINE, SCR_W - 1, SCR_H - 1);
    fill_rect(0, 0, XH_LINE - 1, SCR_H - 1);
    fill_rect(SCR_W - XH_LINE, 0, SCR_W - 1, SCR_H - 1);
    /* centre cross (4 equal quadrants) */
    fill_rect(SCR_W / 2 - XH_LINE / 2, 0, SCR_W / 2 + XH_LINE / 2 - 1, SCR_H - 1);
    fill_rect(0, SCR_H / 2 - XH_LINE / 2, SCR_W - 1, SCR_H / 2 + XH_LINE / 2 - 1);

    /* bottom-left: MODE, bottom-right: AIM prompt (interior of each quadrant) */
    static const char *const mode_range[3] = STR_MODE_RANGE;
    static const char *const mode_two[3]   = STR_MODE_TWO;
    static const char *const aim_first[3]  = STR_AIM_FIRST;
    static const char *const aim_next[3]   = STR_AIM_NEXT;

    quadrant_text(XH_LINE, SCR_W / 2 - XH_LINE / 2 - 1,
                  two_point ? mode_two : mode_range);
    quadrant_text(SCR_W / 2 + XH_LINE / 2, SCR_W - XH_LINE - 1,
                  two_point ? aim_next : aim_first);
}

/* Small title over a boxed distance value. Panels 5 and 6. */
static void render_distance_box(const char *title, int32_t mm)
{
    char val[16];
    format_meters(val, sizeof(val), mm);

    text_centered(0, SCR_W - 1, 10, title, &Font8);

    const int bx0 = 12, bx1 = 115, by0 = 26, by1 = 55;
    outline_rect(bx0, by0, bx1, by1);

    /* biggest font that fits with >= 5 px side margin inside the box:
     * Font20 holds up to 6 chars ("12.32m"), longer ("123.46m") uses Font16 */
    sFONT *f = (text_w(val, &Font20) <= (bx1 - bx0 + 1 - 10)) ? &Font20 : &Font16;
    int ty = by0 + ((by1 - by0 + 1) - (int)f->Height) / 2;
    text_centered(bx0, bx1, ty, val, f);
}

static void render_between(const UI_Data_t *d)
{
    char val[16], line[40];

    text_centered(0, SCR_W - 1, 2, STR_BETWEEN, &Font8);

    format_meters(val, sizeof(val), d->between_mm);
    text_centered(0, SCR_W - 1, 15, val, &Font16);

    format_meters(val, sizeof(val), d->target_a_mm);
    snprintf(line, sizeof(line), "%s%s", STR_LINE_A, val);
    text_centered(0, SCR_W - 1, 44, line, &Font8);

    format_meters(val, sizeof(val), d->target_b_mm);
    snprintf(line, sizeof(line), "%s%s", STR_LINE_B, val);
    text_centered(0, SCR_W - 1, 54, line, &Font8);
}

static void render_panel(UI_Panel_t panel, const UI_Data_t *d)
{
    Paint_Clear(BLACK);

    switch (panel) {
    case UI_PANEL_BOOTING:
        render_title_bar(STR_BOOTING, d->progress_pct);
        break;
    case UI_PANEL_READY:
        text_centered(0, SCR_W - 1, (SCR_H - (int)Font12.Height) / 2, STR_READY, &Font12);
        break;
    case UI_PANEL_CROSSHAIR_RANGE:
        render_crosshair(0);
        break;
    case UI_PANEL_SWITCH_MODE:
        render_title_bar(STR_SWITCH, d->progress_pct);
        break;
    case UI_PANEL_DIST_TARGET_A:
        render_distance_box(STR_DIST_A, d->distance_mm);
        break;
    case UI_PANEL_DIST_TARGET:
        render_distance_box(STR_DIST, d->distance_mm);
        break;
    case UI_PANEL_CROSSHAIR_TWO:
        render_crosshair(1);
        break;
    case UI_PANEL_DIST_BETWEEN:
        render_between(d);
        break;
    case UI_PANEL_AGAIN:
        text_centered(0, SCR_W - 1, (SCR_H - (int)Font24.Height) / 2, STR_AGAIN, &Font24);
        break;
    default:
        break;
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void UI_Init(void)
{
    System_Init();
    OLED_1IN51_Init();
    Driver_Delay_ms(100);

    Paint_NewImage(s_frame, OLED_1IN51_WIDTH, OLED_1IN51_HEIGHT, 270, BLACK);
    Paint_SelectImage(s_frame);
    Paint_Clear(BLACK);
    OLED_1IN51_Display(s_frame);
}

void UI_ShowPanel(UI_Panel_t panel, const UI_Data_t *data)
{
    static const UI_Data_t empty = {0, 0, 0, 0, 0};

    if ((unsigned)panel >= (unsigned)UI_PANEL_COUNT) {
        return;
    }
    render_panel(panel, data ? data : &empty);
    OLED_1IN51_Display(s_frame);
}

uint8_t UI_HoldToPercent(uint32_t hold_ms)
{
    if (hold_ms <= UI_SWITCH_SHOW_AFTER_MS) return 0;
    if (hold_ms >= UI_SWITCH_COMMIT_MS)     return 100;
    return (uint8_t)(((hold_ms - UI_SWITCH_SHOW_AFTER_MS) * 100U) /
                     (UI_SWITCH_COMMIT_MS - UI_SWITCH_SHOW_AFTER_MS));
}

/* =========================================================================
 * Test: cycle through everything (replace with the real state machine)
 * ========================================================================= */
#define TEST_DWELL_MS      2000U
#define TEST_BAR_STEP_PCT  2U
#define TEST_BAR_STEP_MS   30U

static void test_animate_bar(UI_Panel_t panel, UI_Data_t *d)
{
    for (unsigned pct = 0; pct <= 100; pct += TEST_BAR_STEP_PCT) {
        d->progress_pct = (uint8_t)pct;
        UI_ShowPanel(panel, d);
        Driver_Delay_ms(TEST_BAR_STEP_MS);
    }
    Driver_Delay_ms(300);   /* let the full bar be seen */
}

void UI_Test_Cycle(void)
{
    /* values from the mockup */
    UI_Data_t d = { 0, 12320, 12320, 12320, 12320 };

    UI_Init();

    for (;;) {
        for (int p = 0; p < (int)UI_PANEL_COUNT; p++) {
            UI_Panel_t panel = (UI_Panel_t)p;

            if (panel == UI_PANEL_BOOTING || panel == UI_PANEL_SWITCH_MODE) {
                test_animate_bar(panel, &d);
            } else {
                UI_ShowPanel(panel, &d);
                if (panel == UI_PANEL_READY)      Driver_Delay_ms(UI_READY_DISPLAY_MS);
                else if (panel == UI_PANEL_AGAIN) Driver_Delay_ms(UI_AGAIN_TIMEOUT_MS);
                else                              Driver_Delay_ms(TEST_DWELL_MS);
            }
        }
    }
}
