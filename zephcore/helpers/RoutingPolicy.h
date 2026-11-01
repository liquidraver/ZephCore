/*
 * SPDX-License-Identifier: MIT
 * ZephCore RoutingPolicy - flood hop limits and server reply routing
 *
 * Ported from Arduino MeshCore fad11c90 ("Fix replies dropped when
 * flood.max.unscoped is low", PR #3106).  Pure decision logic, no I/O: the
 * repeater and the room server share it so the two roles cannot drift.
 */

#pragma once

#include <mesh/Packet.h>
#include <stdint.h>

namespace mesh {

/**
 * An un-scoped flood: ROUTE_TYPE_FLOOD, or a transport flood whose first code
 * is 0 (no Region computes 0; upstream 01f9293b and bd882a2a treat it as
 * un-scoped, for the hop limit too).
 */
inline bool isUnscopedFlood(const Packet *packet)
{
	return packet->getRouteType() == ROUTE_TYPE_FLOOD
	       || (packet->getRouteType() == ROUTE_TYPE_TRANSPORT_FLOOD && packet->transport_codes[0] == 0);
}

/**
 * Test a flood packet against the configured hop limits.
 *
 * @param packet              inbound flood packet (caller has already checked isRouteFlood())
 * @param flood_max           hop ceiling for any flood packet
 * @param flood_max_unscoped  hop ceiling for un-scoped packets (isUnscopedFlood());
 *                            may be clamped lower than scoped/transport floods
 * @param flood_max_advert    hop ceiling for ADVERT floods, typically tighter still
 *                            so advert churn stays local
 * @returns true if a limit is exceeded and the packet must not be forwarded
 */
inline bool isFloodHopLimitExceeded(const Packet *packet, uint8_t flood_max,
				    uint8_t flood_max_unscoped, uint8_t flood_max_advert)
{
	uint8_t hops = packet->getPathHashCount();

	if (hops >= flood_max) return true;
	if (isUnscopedFlood(packet) && hops >= flood_max_unscoped) return true;
	if (packet->getPayloadType() == PAYLOAD_TYPE_ADVERT && hops >= flood_max_advert) return true;
	return false;
}

/** How a server routes a reply back to the requesting client. */
enum ReplyRoute : uint8_t {
	REPLY_ROUTE_PATH_RETURN,      // request arrived by flood: reply with a PATH return, flooded back
	REPLY_ROUTE_DIRECT_SUPPLIED,  // reply DIRECT, along the return path supplied in the request
	REPLY_ROUTE_DIRECT_OUT_PATH,  // reply DIRECT, along the out_path already stored for this client
	REPLY_ROUTE_FLOOD,            // no return path known: flood the reply
};

/**
 * @param inbound_is_flood    the request arrived as a flood packet
 * @param have_supplied_path  the request payload carried an explicit reply path
 * @param have_out_path       this server already holds a stored out_path for the client
 */
inline ReplyRoute chooseReplyRoute(bool inbound_is_flood, bool have_supplied_path,
				   bool have_out_path)
{
	if (inbound_is_flood) return REPLY_ROUTE_PATH_RETURN;
	if (have_supplied_path) return REPLY_ROUTE_DIRECT_SUPPLIED;
	if (have_out_path) return REPLY_ROUTE_DIRECT_OUT_PATH;
	return REPLY_ROUTE_FLOOD;
}

/** Which transport scope a flooded reply should carry. */
enum ReplyScope : uint8_t {
	REPLY_SCOPE_REQUEST,  // re-use the scope the request arrived on
	REPLY_SCOPE_DEFAULT,  // fall back to this node's default region scope
	REPLY_SCOPE_NONE,     // send un-scoped (ROUTE_TYPE_FLOOD)
};

/**
 * @param request_scope_known         request arrived scoped, and its Region's transport key resolved
 * @param request_was_unscoped_flood  request arrived as an un-scoped flood
 * @param default_scope_known         this node has a default Region with a usable transport key
 */
inline ReplyScope chooseReplyScope(bool request_scope_known, bool request_was_unscoped_flood,
				   bool default_scope_known)
{
	if (request_scope_known) return REPLY_SCOPE_REQUEST;

	/* The requester chose un-scoped, so mirror it.  Replying scoped would
	 * change a path that works today, and repeaters not holding our default
	 * Region would drop it anyway. */
	if (request_was_unscoped_flood) return REPLY_SCOPE_NONE;

	/* Scope unknowable: a DIRECT request carries no transport codes, or the
	 * code matched no Region.  Un-scoped would be dropped at hop 0 by every
	 * repeater running flood.max.unscoped=0, so use our own default. */
	if (default_scope_known) return REPLY_SCOPE_DEFAULT;

	return REPLY_SCOPE_NONE;
}

}
