#pragma once

//#include <Arduino.h>
#include <mesh/Mesh.h>

struct ChannelDetails {
  mesh::GroupChannel channel;
  char name[32];
};
