#include "Stage.h"
#include "hardware/Logger.h"

Stage::Stage(uint8_t no_stages, std::function<void ()> callback_init, std::function<void ()> callback_destroy) {
    this->no_stages = no_stages;
    this->callback_init = callback_init;
    this->callback_destroy = callback_destroy;
}

void Stage::init() {
    if (this->callback_init != NULL) {
        this->callback_init();
    }
    this->current_step = 0;
}

void Stage::nextStep() {
    this->current_step++;
    // logger.println("Next step: " + String(this->current_step) + "/" + String(this->no_stages));
}

bool Stage::isInitialized() {
    return this->current_step != 0;
}

uint8_t Stage::getCurrentStep() {
    return this->current_step;
}

uint8_t Stage::getNoStages() {
    return this->no_stages;
}

void Stage::destroy() {
    if (this->callback_destroy != NULL) {
        this->callback_destroy();
    }
    this->current_step = 0;
}

// Compare this snippet from src/Stage.cpp: