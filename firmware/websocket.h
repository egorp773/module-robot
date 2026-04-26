#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <Arduino.h>

void websocket_init();
void websocket_cleanup();
void websocket_send(const char* msg);

#endif // WEBSOCKET_H
