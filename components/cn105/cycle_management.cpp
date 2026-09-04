#include "cycle_management.h"
#include "cn105.h"
#include "Globals.h"

using namespace esphome;

void cycleManagement::checkTimeout(unsigned int update_interval) {
    if (doesCycleTimeOut(update_interval)) {                          // does it last too long ?                    
        ESP_LOGW(TAG, "Cycle timeout, resetting cycle...");
        cycleEnded(true);
    }
}


bool cycleManagement::isCycleRunning() {
    return cycleRunning;
}

void cycleManagement::init() {
    cycleRunning = false;
    lastCompleteCycleMs = esphome::millis();
}

void cycleManagement::deferCycle() {

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    uint32_t delay = DEFER_SCHEDULE_UPDATE_LOOP_DELAY * 2;
#else
    uint32_t delay = DEFER_SCHEDULE_UPDATE_LOOP_DELAY;
#endif

    //ESP_LOGI(LOG_CYCLE_TAG, "Defering cycle trigger of %lu ms", delay);
    log_info_uint32(LOG_CYCLE_TAG, "Defering cycle trigger of  ", delay, " ms");
    // forces the lastCompleteCycle offset of delay ms to allow a longer rest time
    lastCompleteCycleMs = esphome::millis() + delay;

}
void cycleManagement::cycleStarted() {
    ESP_LOGI(LOG_CYCLE_TAG, "1: Cycle start");
    lastCycleStartMs = esphome::millis();
    cycleRunning = true;
}

void cycleManagement::cycleEnded(bool timedOut) {
    cycleRunning = false;

    if (lastCompleteCycleMs < esphome::millis()) {    // we check this because of defering mecanism
        // a complete cycle is done
        lastCompleteCycleMs = esphome::millis();      // to prevent next inteval from ticking too soon
    }

    ESP_LOGI(LOG_CYCLE_TAG, "6: Cycle ended in %.1f seconds (with timeout?: %s)",
        (lastCompleteCycleMs - lastCycleStartMs) / 1000.0, timedOut ? "YES" : " NO");

}

bool cycleManagement::hasUpdateIntervalPassed(unsigned int update_interval) {
    if (esphome::millis() < lastCompleteCycleMs) return false;      // must be checked because operands are they are unsigned
    return (esphome::millis() - lastCompleteCycleMs) > update_interval;
}

bool cycleManagement::doesCycleTimeOut(unsigned int update_interval) {
    if (esphome::millis() < lastCycleStartMs) return false;         // must be checked because operands are they are unsigned
    return (esphome::millis() - lastCycleStartMs) > (2 * update_interval) + 1000;
}
