#include <inttypes.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fakeavr.h>
#include "configs.h"
#include "SDL.h"

//#define WINDOW_W 1000
//#define WINDOW_H 650

volatile int16_t get_motor_pace(void);
void motion_callback(uint8_t borderflags);
uint8_t get_pen_status(void);
uint8_t motors_ready(void);

static FILE* sim_out;
static SDL_Thread* timer_thread;
static SDL_Window* window;
static SDL_Renderer* renderer;
static SDL_Texture* texture;

#define TEX_UPSCALE 2
#define BASE_SCREEN_SCALE ((TEX_UPSCALE)*0.055)
static double xscale = BASE_SCREEN_SCALE;
static double yscale = BASE_SCREEN_SCALE * STEPSCALE_X / STEPSCALE_Y;

int tableflip = 0;

const int PAPER_R = 255, PAPER_G = 255, PAPER_B = 255, PAPER_A = 255;

const int PEN_R = 0;
const int PEN_G = 0;
const int PEN_B = 0;
const int PEN_A = 1;

const int UP_R = 255;
const int UP_G = 0;
const int UP_B = 0;
const int UP_A = 10;

SDL_DisplayMode display_mode;

static struct _absloc {
    int x, y;
} abs_loc, prev_screen;

static int OCR0A = 0;

static struct _timer_thread_data {
} timer_thread_data;

static int timer_thread_func(void* data);

#define MAX_RENDER_POINTS 8000

typedef struct _plot_point {
    SDL_Point xy;
    uint8_t a;
} plot_point_t;

typedef struct _trace {
    plot_point_t points[MAX_RENDER_POINTS];
    int n_points;
} trace_t;

/* Timer thread */
static trace_t trace_down;
static trace_t trace_up;

/* Main/render thread */
static trace_t r_trace_down;
static trace_t r_trace_up;

static SDL_mutex* mutex;
static struct timespec interval;

void measure_in(struct timespec* start)
{
    clock_gettime(CLOCK_REALTIME, start);
}

double measure_out(const struct timespec* start)
{
    struct timespec end;

    clock_gettime(CLOCK_REALTIME, &end);
    double t_ns = (double)(end.tv_sec - start->tv_sec) * 1.0e9 +
        (double)(end.tv_nsec - start->tv_nsec);
    return t_ns;
}


void init_io(void)
{
    abs_loc.x = 1000;
    abs_loc.y = 1000;

    prev_screen.x = prev_screen.y = 1000;


    sim_out = fopen("sim_out.txt", "w");
    if (sim_out == NULL) {
        perror("sim_out.txt");
        exit(1);
    }

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't initialize SDL: %s", 
                SDL_GetError());
        exit(1);
    }

    mutex = SDL_CreateMutex();

    timer_thread = SDL_CreateThread(timer_thread_func, "avr timer thread", 
            &timer_thread_data);

    SDL_GetDesktopDisplayMode(0, &display_mode);

    xscale = display_mode.w * xscale / 1000;
    yscale = display_mode.h * yscale / 650;

    if (SDL_CreateWindowAndRenderer(display_mode.w, display_mode.h, 
                SDL_WINDOW_RESIZABLE, 
                &window, &renderer)) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, 
                "Couldn't create window or renderer: %s", SDL_GetError());
    }

    SDL_RenderSetLogicalSize(renderer, display_mode.w, display_mode.h);

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, 
            SDL_TEXTUREACCESS_TARGET, display_mode.w * TEX_UPSCALE, 
            display_mode.h * TEX_UPSCALE);

    SDL_SetRenderTarget(renderer, texture);
    SDL_SetRenderDrawColor(renderer, PAPER_R, PAPER_G, PAPER_B, PAPER_A);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderTarget(renderer, NULL);
 
    SDL_SetRenderDrawColor(renderer, PEN_R, PEN_G, PEN_B, PEN_A);

    measure_in(&interval);
}

void timer0_init(void)
{
    OCR0A = 255;
}

void timer1_init(void)
{
}

void timer2_init(void)
{
}

void motori_init(void)
{
}

void motori_sleep(uint8_t zzz)
{
}

void step(int8_t xdir, int8_t ydir) 
{
    abs_loc.x += xdir;
    abs_loc.y += ydir;

    fprintf(sim_out, "%d %d\n", abs_loc.x, abs_loc.y);

    if (SDL_LockMutex(mutex) == 0) {
        int new_screen_x = (int)(abs_loc.x * xscale + 0.5);
        int new_screen_y = TEX_UPSCALE * display_mode.h - (int)(abs_loc.y * yscale + 0.5);
        int uu = new_screen_x != prev_screen.x || new_screen_y != prev_screen.y;
        if (get_pen_status() != 0) {
            if (uu){
                trace_down.points[trace_down.n_points].xy.x = new_screen_x;
                trace_down.points[trace_down.n_points].xy.y = new_screen_y;
                trace_down.points[trace_down.n_points].a = (get_motor_pace()-16) * 3;
                ++trace_down.n_points;
            }
        } else {
            if (uu) {
                trace_up.points[trace_up.n_points].xy.x = new_screen_x;
                trace_up.points[trace_up.n_points].xy.y = new_screen_y;
                trace_up.points[trace_up.n_points].a = (get_motor_pace()-16) * 3;
                ++trace_up.n_points;
            }
        }
        prev_screen.x = new_screen_x;
        prev_screen.y = new_screen_y;
        SDL_UnlockMutex(mutex);
    }
}

void pen_servo_set(uint8_t relax_value)
{
}

void pen_solenoid_update(void)
{
}

void global_enable_interrupts(void)
{
}


void global_set_sleep_mode_idle(void)
{
}

void DELAY_MS(long ms)
{
    usleep(ms * 1000);
}

static void do_timerjerk()
{
    uint8_t borderflags = 0;

    OCR0A -= 10;
    if (OCR0A <= 0) {
        OCR0A = get_motor_pace();
        if (abs_loc.x <= 0) borderflags |= _BV(XSUP);
        if (abs_loc.y <= 0) borderflags |= _BV(YSUP);
        motion_callback(borderflags);
    }
}

int timer_thread_func(void* data)
{
    for(;!tableflip;) {
        for (int i = 0; i < 10; ++i) {
            do_timerjerk();
            if (motors_ready()) {
                break;
            }
        }
        
        usleep(100);
    }

    return 0;
}

void do_idle()
{
    int render = 0;

    if (trace_down.n_points > MAX_RENDER_POINTS - 100 ||
            trace_up.n_points > MAX_RENDER_POINTS - 100) {
        render = 1;
    }

    if (!render) {
        if (measure_out(&interval) > 33e6) {
            render = 1;
        }
    }

    if (render) {
        SDL_Event event;
        SDL_PollEvent(&event);
        if (event.type == SDL_QUIT) {
            tableflip = 1;

            SDL_DestroyWindow(window);
            SDL_Quit();
            usleep(10000);
            exit(0);
        }

        measure_in(&interval);

        //printf("\nRENDER n=%d %d\n", trace_down.n_points, trace_up.n_points);
        if (SDL_LockMutex(mutex) == 0) {
            memcpy(r_trace_down.points, trace_down.points, 
                    sizeof(trace_down.points[0]) * trace_down.n_points);
            r_trace_down.n_points = trace_down.n_points;
            trace_down.n_points = 0;

            memcpy(r_trace_up.points, trace_up.points, 
                    sizeof(trace_up.points[0]) * trace_up.n_points);
            r_trace_up.n_points = trace_up.n_points;
            trace_up.n_points = 0;

            SDL_UnlockMutex(mutex);
        }

        SDL_SetRenderTarget(renderer, texture);
        for (int i = 0; i < r_trace_down.n_points; ++i) {
            SDL_SetRenderDrawColor(renderer, PEN_R, PEN_G, PEN_B, r_trace_down.points[i].a);
            SDL_RenderDrawPoint(renderer, r_trace_down.points[i].xy.x,
                    r_trace_down.points[i].xy.y);
        }

        //SDL_RenderSetScale(renderer, TEX_UPSCALE, TEX_UPSCALE);
        //SDL_RenderDrawPoints(renderer, r_trace_up.points, r_trace_up.n_points);
        for (int i = 0; i < r_trace_up.n_points; ++i) {
            SDL_SetRenderDrawColor(renderer, UP_R, UP_G, UP_B, r_trace_up.points[i].a);
            SDL_RenderDrawPoint(renderer, r_trace_up.points[i].xy.x,
                    r_trace_up.points[i].xy.y);
        }


        SDL_SetRenderTarget(renderer, NULL);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
    }
}
