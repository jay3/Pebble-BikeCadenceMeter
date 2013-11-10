#include <pebble.h>
#include "config.h"

static Window *window;
static Layer *layer;

static char info_text[256] = "";
static char info_text2[256] = "";


#define KEY_PERSIST_SENSIBILITY 1

#define SENSIBILITY_INI 200
int sensibility = SENSIBILITY_INI;
int cadence = 0;

bool debug = false;


#define NB_X_CHANGE_DTS 10
#define NB_X_AVG (4*ACCEL_SAMPLING_RATE)
static struct MyAccel {
  uint32_t mts; // timestamp in milliseconds
  AccelData a; // acceleration
  //AccelData l; // low frequency
  //AccelData h; // high frequency
  
  //float a1;

  int16_t x_nb_change; // number of changes of direction on x-axis
  uint32_t x_change_mts;
  uint32_t x_change_dt;
  
  uint32_t x_change_dts_index;
  uint32_t x_change_dts[NB_X_CHANGE_DTS];
  uint32_t x_change_dts_avg;

  uint32_t x_values_index;
  int32_t x_values[NB_X_AVG];
  int32_t x_values_avg;  // average of last NB_X_AVG s_acc.a.x values, times NB_X_AVG
} s_acc;

int nb = 0, nb2 = 0;

void accel_filter() {
  static bool x_has_been_down = false;
  static bool x_has_been_up = false;

  if (s_acc.x_values_index > NB_X_AVG) {
    // substract oldest value to average
    s_acc.x_values_avg -= s_acc.x_values[s_acc.x_values_index % NB_X_AVG];
  }
  // replace oldest value by new value
  s_acc.x_values[s_acc.x_values_index % NB_X_AVG] = s_acc.a.x;
  // add new value to average
  s_acc.x_values_avg += s_acc.x_values[s_acc.x_values_index % NB_X_AVG];
  s_acc.x_values_index++;
  
  //s_acc.a1 = my_sqrt(s_acc.a.x*s_acc.a.x + s_acc.a.y*s_acc.a.y + s_acc.a.z*s_acc.a.z) - 1000;

  //#define ATTENUATION 2
  // low values
  //s_acc.l.x += (s_acc.a.x - s_acc.l.x) / ATTENUATION;
  //s_acc.l.y += (s_acc.a.y - s_acc.l.y) / ATTENUATION;
  //s_acc.l.z += (s_acc.a.z - s_acc.l.z) / ATTENUATION;
  // high values
  //s_acc.h.x = s_acc.a.x - s_acc.l.x;
  //s_acc.h.y = s_acc.a.y - s_acc.l.y;
  //s_acc.h.z = s_acc.a.z - s_acc.l.z;
  
  
  int32_t delta = s_acc.a.x - s_acc.x_values_avg / NB_X_AVG;
  if (abs(delta) > sensibility) {
    if (delta > 0) {
      x_has_been_up = true;
    }
    if (x_has_been_up && delta < 0) {
      x_has_been_down = true;
    }
    if (x_has_been_up && x_has_been_down) {
      x_has_been_down = x_has_been_up = false;
      s_acc.x_nb_change++;
      s_acc.x_change_dt = s_acc.mts - s_acc.x_change_mts;
      s_acc.x_change_mts = s_acc.mts;
      if (s_acc.x_change_dt != 0) {
       
        s_acc.x_change_dts[s_acc.x_change_dts_index % NB_X_CHANGE_DTS] = s_acc.x_change_dt;
        s_acc.x_change_dts_index++;
        
        if (s_acc.x_change_dts_index >= NB_X_CHANGE_DTS) {

          if (s_acc.x_change_dts_index % 2 == 0) {
            // only update cadence every x changes

            s_acc.x_change_dts_avg = 0;
            
            // search min & max, and remove them from the average
            uint32_t min = 100000;
            uint32_t max = 0;
            int imin = -1, imax = -1;
            for (int i = 0; i < NB_X_CHANGE_DTS; i++) {
              if (s_acc.x_change_dts[i] > max) {
                max = s_acc.x_change_dts[i];
                imax = i;
              }
              if (s_acc.x_change_dts[i] < min) {
                min = s_acc.x_change_dts[i];
                imin = i;
              }
            }

            int nb = 0;
            for (int i = 0; i < NB_X_CHANGE_DTS; i++) {
              if (i != imin && i != imax) {
                // do not use min and max values to compute the average
                s_acc.x_change_dts_avg += s_acc.x_change_dts[i];
                nb++;
              }
            }
            s_acc.x_change_dts_avg /= nb;
            
            cadence = 60 * 1000 / s_acc.x_change_dts_avg;
            
            #if LOGS
              for (int i = 0; i < NB_X_CHANGE_DTS; i++) {
                APP_LOG(
                  APP_LOG_LEVEL_DEBUG,
                  "%d>%d [%ld>%ld] %d"
                  "|%2ld %2ld %2ld %2ld %2ld %2ld %2ld %2ld %2ld %2ld |a:%2ld",
                  imin, imax, min, max, nb,
                  s_acc.x_change_dts[0]/100, s_acc.x_change_dts[1]/100, s_acc.x_change_dts[2]/100, s_acc.x_change_dts[3]/100, s_acc.x_change_dts[4]/100, s_acc.x_change_dts[5]/100, s_acc.x_change_dts[6]/100, s_acc.x_change_dts[7]/100, s_acc.x_change_dts[8]/100, s_acc.x_change_dts[9]/100, s_acc.x_change_dts_avg
                );
              }
            #endif
          }
        }
      }
    }
  }
  if (s_acc.mts - s_acc.x_change_mts > 10000) {
    // reset cadence to zero after 10s of "no activity"
    cadence = 0;
    s_acc.x_change_dts_index = 0;
  }
}

static void handle_accel(AccelData *accel_data, uint32_t num_samples) {

  nb2++;
  for(uint32_t i=0;i<num_samples; i++) {
    s_acc.a = accel_data[i];
    s_acc.mts += 1000 / ACCEL_SAMPLING_RATE;

    nb++;
    accel_filter();

  }
  // Mark the layer dirty to have it update
  layer_mark_dirty(layer);    
}

int t0 = 0;
static void update_layer_callback(Layer *layer, GContext *ctx) {
  time_t t = 0;
  uint16_t t_ms = 0;
  if (debug || (t0 == 0)) {
    time_ms(&t, &t_ms);
    if (t0 == 0) {
      t0 = t;
    }
  }

  if (debug) {
    snprintf(info_text2, sizeof(info_text2),
      "#%d %lu.%u %ld\n"
      //"%+6d %+6d %+6d\n"
      "%+6d %+6ld %+6ld\n"
      "%4d %4ld %3ld %d\n"
      "%2ld %2ld %2ld %2ld %2ld %2ld %2ld\n"
      " %2ld %2ld %2ld a:%2ld\n"
      ,
      nb, t-t0, t_ms, s_acc.mts/10,
      //s_acc.a.x,s_acc.l.x,s_acc.h.x,
      s_acc.a.x,s_acc.x_values_avg/NB_X_AVG,(s_acc.a.x - s_acc.x_values_avg / NB_X_AVG),
      s_acc.x_nb_change, (s_acc.x_change_mts/10)%1000, s_acc.x_change_dt, sensibility,
      s_acc.x_change_dts[0]/100, s_acc.x_change_dts[1]/100, s_acc.x_change_dts[2]/100, s_acc.x_change_dts[3]/100, s_acc.x_change_dts[4]/100, s_acc.x_change_dts[5]/100, s_acc.x_change_dts[6]/100,
      s_acc.x_change_dts[7]/100, s_acc.x_change_dts[8]/100, s_acc.x_change_dts[9]/100, s_acc.x_change_dts_avg
    );
  } else {
    snprintf(info_text2, sizeof(info_text2), "Sensibility: %d", sensibility);
  }
  
  graphics_context_set_text_color(ctx, GColorBlack);
  graphics_draw_text(ctx,
    info_text2,
    fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
    (GRect){ .origin = GPoint(2, 5), .size = GSize(SCREEN_W, SCREEN_H-60) },
    GTextOverflowModeWordWrap,
    GTextAlignmentLeft,
    NULL
  );  

  snprintf(info_text, sizeof(info_text), "%d", cadence);
  graphics_context_set_text_color(ctx, GColorBlack);
  graphics_draw_text(ctx,
    info_text,
    fonts_get_system_font(FONT_KEY_BITHAM_42_BOLD),
    (GRect){ .origin = GPoint(0, 90), .size = layer_get_frame(layer).size },
    GTextOverflowModeWordWrap,
    GTextAlignmentCenter,
    NULL
  );  
}


static void long_select_click_handler(ClickRecognizerRef recognizer, void *context) {
  #if DEBUG
    debug = !debug;
  #endif
}


static void up_click_handler(ClickRecognizerRef recognizer, void *context) {
  sensibility += 10;
}

static void down_click_handler(ClickRecognizerRef recognizer, void *context) {
  sensibility -= 10;
}

void click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, up_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, down_click_handler);
  
  // long click config:
  window_long_click_subscribe(BUTTON_ID_SELECT, 1000, long_select_click_handler, NULL /* No handler on button release */);
}

static void window_load(Window *window) {
  Layer *window_layer = window_get_root_layer(window);
  GRect bounds = layer_get_bounds(window_layer);

  layer = layer_create(bounds);
  layer_set_update_proc(layer, update_layer_callback);
  layer_add_child(window_layer, layer);  
}

static void window_unload(Window *window) {
  layer_destroy(layer);
}

static void init(void) {
  if (persist_exists(KEY_PERSIST_SENSIBILITY)) {
    sensibility = persist_read_int(KEY_PERSIST_SENSIBILITY);
  } else {
    sensibility = SENSIBILITY_INI;
  }
  s_acc.mts = 0;
  s_acc.x_change_mts = 0;
  s_acc.x_change_dts_index = 0;
  s_acc.x_change_dts_avg = 0;
  
  s_acc.x_values_index = 0;
  s_acc.x_values_avg = 0;

  window = window_create();
  window_set_click_config_provider(window, click_config_provider);
  window_set_window_handlers(window, (WindowHandlers) {
    .load = window_load,
    .unload = window_unload,
  });
  const bool animated = true;
  window_stack_push(window, animated);
   
  accel_data_service_subscribe(ACCEL_SAMPLES_PER_UPDATE, handle_accel);
  if (ACCEL_SAMPLING_RATE == 10) {
    accel_service_set_sampling_rate(ACCEL_SAMPLING_10HZ);
  } else if (ACCEL_SAMPLING_RATE == 25) {
    accel_service_set_sampling_rate(ACCEL_SAMPLING_25HZ);
  } else if (ACCEL_SAMPLING_RATE == 50) {
    accel_service_set_sampling_rate(ACCEL_SAMPLING_50HZ);
  } else {
    accel_service_set_sampling_rate(ACCEL_SAMPLING_1HZ);
  }
}

static void deinit(void) {
  persist_write_int(KEY_PERSIST_SENSIBILITY, sensibility);
  accel_data_service_unsubscribe();
  window_destroy(window);
  layer_destroy(layer);
}

int main(void) {
  init();

  app_event_loop();
  deinit();
}

