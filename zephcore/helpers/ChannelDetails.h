/*
 * SPDX-License-Identifier: MIT
 * ChannelDetails - group channel info
 */

#pragma once

#include <mesh/Mesh.h>
#include <mesh/MeshCore.h>

struct ChannelDetails {
	mesh::GroupChannel channel;
	char name[32];
};
