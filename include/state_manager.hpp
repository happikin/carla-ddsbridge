#pragma once

enum control_signal_e {
    terminate
};

enum thread_state_e {
    alive,
    dead
};

class blockpoint {
    public:
        void wait_while() {}
};