struct scanner_state {
    char *name;
    bool power_state;
    bool wakeup_state;
    bool trig_state;
    struct platform_driver *platform_diver_addr;
};

