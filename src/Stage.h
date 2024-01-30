#ifndef STAGE_H
#define STAGE_H

#include <Arduino.h>

class Stage {
private:
    uint8_t no_stages = 0;
    uint8_t current_step = 0;
    std::function<void ()> callback_init = NULL;
    std::function<void ()> callback_destroy = NULL;
    

public:
    Stage(uint8_t no_stages, std::function<void ()> callback_init = NULL, std::function<void ()> callback_destroy = NULL);

    void init();
    void nextStep();
    bool isInitialized();
    uint8_t getCurrentStep();
    uint8_t getNoStages();
    void destroy();
};

#endif // STAGE_H