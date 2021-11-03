#pragma once

typedef struct {
    char c;
} Node_Control;

extern Node_Control nctrl;

static void _setup_clocks();
static void _generate_clocks_on_pins();

void node_control_init();
void node_control_run();
void node_control_shutdown();