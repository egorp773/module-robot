#ifndef SOUND_H
#define SOUND_H

#include <Arduino.h>

enum SoundId : uint8_t {
  SND_NONE = 0,
  SND_CONNECTED = 1,
  SND_DISCONNECTED = 2,
  SND_LOW_BATT = 3,
  SND_GOING_BACK = 4,
};

void sound_init();
void enqueueSound(SoundId id);
void setAttachment(bool on);
void setMount(bool on);

#endif // SOUND_H
