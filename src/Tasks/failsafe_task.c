// battery voltage check
// watchdog RC
// failsafe triggers

// Example rc watchdog BEGIN
// #define MODE_RC 1
// #define RC_DATA_TIMEOUT 1000
//   TickType_t cur_tick, last_tick = xTaskGetTickCount();

// cur_tick = xTaskGetTickCount();
// if (cur_tick - last_tick > pdMS_TO_TICKS(RC_DATA_TIMEOUT)) {
//   if (MODE_RC) {
//     LOG_CRIT(TASK_RADIO_RX_ID, "No RX data");
//     // initiate RTH / hover procedure
//   } else {
//     LOG_DEBUG(TASK_RADIO_RX_ID, "No RX data");
//   }
// }
// last_tick = cur_tick;
// Example END