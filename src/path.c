#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "configs.h"
#include "shvars.h"
#include "motori.h"
#include "line.h"
#include "path.h"

static STEPPER_COORD px[MAX_PATH_LOOKAHEAD];
static STEPPER_COORD py[MAX_PATH_LOOKAHEAD];
static STEPPER_COORD trail_x, trail_y; //<! xy before tail, to calculate length
static int8_t head = 0, tail = 0;
static int32_t length;
static int32_t length_from_start;

static int adv(int n) {
    ++n;
    if (n == MAX_PATH_LOOKAHEAD) {
        return 0;
    }
    return n;
}

static int prev(int n) {
    if (--n < 0) {
        return MAX_PATH_LOOKAHEAD - 1;
    }
    return n;
}

int path_can_add() 
{
    return adv(head) != tail;
}

void path_new()
{
    trail_x = stepper_loc.x;
    trail_y = stepper_loc.y;
    length = length_from_start = 0;
    fprintf(stderr, "#set trail to %d,%d NEWPATH\n", trail_x, trail_y);
}

static void print_path()
{
#if SIM
    fprintf(stderr, "~~[%d,%d -", trail_x, trail_y);
    for (int i = 0, end = path_count(), p = tail; i < end; ++i) {
        fprintf(stderr, "(%d,%d)-", px[p], py[p]);
        p = adv(p);
    }
    fprintf(stderr, "-]\n");
#endif
}

STEPPER_COORD path_add(STEPPER_COORD x, STEPPER_COORD y)
{
    //fprintf(stderr, "path_add %d,%d: ", x, y);
    STEPPER_COORD x0, y0;
    if (path_count() == 0) {
        x0 = trail_x;
        y0 = trail_y;
    } else {
        int8_t i = prev(head);
        x0 = px[i];
        y0 = py[i];
    }

    STEPPER_COORD segment_length = line_step_length(x0, y0, x, y); 
    length += segment_length;
    length_from_start += segment_length;

    px[head] = x;
    py[head] = y;
    head = adv(head);

    print_path();

    return segment_length;
}

void path_dequeue(STEPPER_COORD * x, STEPPER_COORD * y)
{
    *x = px[tail];
    *y = py[tail];
    tail = adv(tail);

    /* Reduce the length by the extracted piece */
    length -= line_step_length(trail_x, trail_y, *x, *y);
    trail_x = *x;
    trail_y = *y;
    //fprintf(stderr, "#set trail to %d,%d dequeue\n", trail_x, trail_y);
    print_path();
}

int8_t path_count()
{
    if (head == tail) {
        return 0;
    }

    if (head > tail) {
        return head - tail;
    }

    return MAX_PATH_LOOKAHEAD + head - tail;
}

int32_t path_step_length_from_start()
{
    return length_from_start;
}

int32_t path_step_length()
{
    return length;
}

float path_angle_to(STEPPER_COORD x, STEPPER_COORD y)
{
    int8_t count = path_count();
    if (count == 0) return 0;
#ifdef SIM
    fprintf(stderr, "angle to %d,%d ", x, y);
    print_path();
#endif

    int p = prev(head);
    STEPPER_COORD x1 = px[p], y1 = py[p];
    //fprintf(stderr, "prev(head)=%d: %d,%d ", p, x1, y1);

    STEPPER_COORD x0, y0;
    if (count > 1) {
        p = prev(p);
        x0 = px[p]; 
        y0 = py[p];
        //fprintf(stderr, "prev(prev)=%d: %d,%d ", p, x0, y0);
    } else {
        x0 = trail_x;
        y0 = trail_y;
        //fprintf(stderr, "trail %d,%d ", x0, y0);
    }
    
    float va_x = x1 - x0, va_y = y1 - y0;
    float vb_x = x  - x0, vb_y = y  - y0;
    //float a0 = atan2(va_y, va_x);
    //float a1 = atan2(vb_y, vb_x);

    float norm_a = hypot(va_x, va_y);
    float norm_b = hypot(vb_x, vb_y);

    float dot = va_x * vb_x + va_y * vb_y;
    float cosa = dot / (norm_a*norm_b);
#if 0
    fprintf(stderr, "norm_a=%3.3f norm_b=%3.3f va=(%1.3f,%1.3f) vb=(%1.3f,%1.3f) "
            "cosa=%3.3f\n",
            norm_a, norm_b, va_x, va_y, vb_x, vb_y, cosa);
#endif
    float angle = acos(cosa);

#if SIM
    fprintf(stderr, " angle between (%d,%d)-(%d,%d) to (%d,%d) is %3.2f\n",
            x0, y0, x1, y1, x, y,
            angle * 180/M_PI);
#endif

    return angle * 180 / M_PI; //fabs(a1 - a0) * 180 / M_PI;
}
