// generated by Fast Light User Interface Designer (fluid) version 1.0110

#include "helper-window.hpp"

static void update_tod(double val) {
  sim_win->time_offset = 60*60*val;
sim_win->redraw();
}

static void toggle_lights(bool val) {
  sim_win->do_lights = val;
sim_win->redraw();
}

void set_network_draw(int val) {
  sim_win->network_draw = (fltkview::network_draw_mode)val;
sim_win->redraw();
}

static void toggle_screenshot(bool val) {
  sim_win->screenshot_mode = val;
sim_win->redraw();
}

static void toggle_go(bool val) {
  sim_win->go = val;
sim_win->redraw();
}

static void toggle_throttle(double val) {
  sim_win->throttle = val;
sim_win->set_throttle();
sim_win->redraw();
}

static void set_bg_saturation(double val) {
  sim_win->bg_saturation = val;
sim_win->redraw();
}

static void set_fg_saturation(double val) {
  sim_win->fg_saturation = val;
sim_win->redraw();
}

fltkview *sim_win=(fltkview *)0;

static void cb_Time(Fl_Slider* o, void*) {
  update_tod(o->value());
}

static void cb_Draw(Fl_Light_Button* o, void*) {
  toggle_lights(o->value());
}

static void cb_Roads(Fl_Choice* o, void*) {
  set_network_draw(o->value());
}

Fl_Menu_Item menu_Roads[] = {
 {"None", 0,  0, 0, 0, FL_NORMAL_LABEL, 0, 14, 0},
 {"Abstract", 0,  0, 0, 0, FL_NORMAL_LABEL, 0, 14, 0},
 {"Textured", 0,  0, 0, 0, FL_NORMAL_LABEL, 0, 14, 0},
 {0,0,0,0,0,0,0,0,0}
};

static void cb_Record(Fl_Light_Button* o, void*) {
  toggle_screenshot(o->value());
}

static void cb_Go(Fl_Light_Button* o, void*) {
  toggle_go(o->value());
}

static void cb_Throttle(Fl_Light_Button* o, void*) {
  toggle_throttle(o->value());
}

static void cb_bg(Fl_Slider* o, void*) {
  set_bg_saturation(o->value());
}

static void cb_fg(Fl_Slider* o, void*) {
  set_fg_saturation(o->value());
}

static Fl_Double_Window* make_window() {
  Fl_Double_Window* w;
  { Fl_Double_Window* o = new Fl_Double_Window(1465, 780);
    w = o;
    { sim_win = new fltkview(10, 5, 1280, 720);
      sim_win->box(FL_DOWN_FRAME);
      sim_win->color((Fl_Color)FL_BACKGROUND_COLOR);
      sim_win->selection_color((Fl_Color)FL_BACKGROUND_COLOR);
      sim_win->labeltype(FL_NORMAL_LABEL);
      sim_win->labelfont(0);
      sim_win->labelsize(14);
      sim_win->labelcolor((Fl_Color)FL_FOREGROUND_COLOR);
      sim_win->align(FL_ALIGN_CENTER);
      sim_win->when(FL_WHEN_RELEASE);
      sim_win->window()->hotspot(sim_win);
    } // fltkview* sim_win
    { Fl_Slider* o = new Fl_Slider(10, 740, 1275, 20, "Time of day");
      o->type(1);
      o->maximum(24);
      o->step(0.05);
      o->callback((Fl_Callback*)cb_Time);
      o->value(sim_win->time_offset);
    } // Fl_Slider* o
    { Fl_Light_Button* o = new Fl_Light_Button(1350, 50, 100, 25, "Draw lights");
      o->callback((Fl_Callback*)cb_Draw);
      o->value(sim_win->do_lights);
    } // Fl_Light_Button* o
    { Fl_Choice* o = new Fl_Choice(1345, 15, 110, 25, "Roads");
      o->down_box(FL_BORDER_BOX);
      o->callback((Fl_Callback*)cb_Roads);
      o->menu(menu_Roads);
      o->value(sim_win->network_draw);
    } // Fl_Choice* o
    { Fl_Light_Button* o = new Fl_Light_Button(1365, 100, 85, 25, "Record");
      o->callback((Fl_Callback*)cb_Record);
      o->value(sim_win->screenshot_mode);
    } // Fl_Light_Button* o
    { Fl_Light_Button* o = new Fl_Light_Button(1300, 740, 70, 25, "Go!");
      o->callback((Fl_Callback*)cb_Go);
      o->value(sim_win->go);
    } // Fl_Light_Button* o
    { Fl_Light_Button* o = new Fl_Light_Button(1375, 740, 80, 25, "Throttle");
      o->callback((Fl_Callback*)cb_Throttle);
      o->value(sim_win->throttle);
    } // Fl_Light_Button* o
    { Fl_Slider* o = new Fl_Slider(1305, 550, 150, 20, "bg saturation");
      o->type(1);
      o->step(0.01);
      o->callback((Fl_Callback*)cb_bg);
      o->align(FL_ALIGN_TOP);
      o->value(sim_win->bg_saturation);
    } // Fl_Slider* o
    { Fl_Slider* o = new Fl_Slider(1306, 594, 150, 20, "fg saturation");
      o->type(1);
      o->step(0.01);
      o->callback((Fl_Callback*)cb_fg);
      o->align(FL_ALIGN_TOP);
      o->value(sim_win->fg_saturation);
    } // Fl_Slider* o
    o->end();
  } // Fl_Double_Window* o
  Fl::focus(sim_win);
  return w;
}