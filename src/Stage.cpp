#include "Stage.h"
#include <Preferences.h>
#include "./hardware/Logger.h"

Stage::Stage(uint8_t no_steps, std::function<void ()> callback_init, std::function<void ()> callback_destroy) {
    this->no_steps = no_steps;
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
    DEBUG(("Current step: "+String(this->current_step)).c_str());
}

bool Stage::isInitialized() {
    return this->current_step != 0;
}

void Stage::setStep(uint8_t step) {
    this->current_step = step;
}

uint8_t Stage::getCurrentStep() {
    return this->current_step;
}

uint8_t Stage::getNoStep() {
    return this->no_steps;
}

void Stage::destroy() {
    if (this->callback_destroy != NULL) {
        this->callback_destroy();
    }
    this->current_step = 0;
}

void Stage::DEBUG(const char *message) {
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "[Stage]: %s", message);
    logger.println(buffer);
}


void Stage::ERROR(ErrorType error){
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "[ERROR -> MqttClient]: %s", errorMessages[error]);
  logger.println(buffer);
}

// Compare this snippet from src/Stage.cpp: