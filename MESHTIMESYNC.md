# Mesh Time Sync

## What it does

Your node listens to the timestamps inside other nodes' signed advertisements
and, if its own clock is clearly wrong, gently corrects it — no GPS and no
phone needed. It works on every role (repeater, room server, observer,
companion). Why care: a node with a wrong clock shows garbage "last heard"
times, and after a reboot without a time sync its own adverts can be silently
ignored by every node that already knows it. On the live mesh today, almost
half of all repeaters are more than an hour off; this feature fixes that
class of problem automatically.

Default is **OFF**. Nothing changes unless you enable it.

## How to switch it on

```
set meshtimesync on
```

over the USB serial CLI or remote admin (repeaters, room servers, observers).
On companions, toggle the `meshtimesync` custom variable from the app, or use
the same command on the USB text CLI.

Check what it is doing with:

```
get meshtimesync
```

This shows a live view even while the feature is off — how many trustworthy
senders it can see, how many agree, what correction it *would* make. It is
worth watching that dry-run output for a day before (and after) enabling.

## What it will NOT do

- It never fights a recent clock set, whether from GPS or from you: any GPS
  fix or manual `time <epoch>` / `clock sync` protects the clock from
  automatic changes for 7 days (the same mechanism for both). A live GPS
  re-arms this on every fix, so it keeps owning the clock. If GPS goes dead
  (indoors, dead antenna), mesh correction takes over once 7 days pass
  without a fix — until the next real fix.
- It never steps more than 1 hour at a time, and at most one step per 6 hours.
- It does nothing unless at least 6 trustworthy senders are visible and a
  strict majority of them agree (exception: a provably dead clock after a
  reboot needs only 3 agreeing senders to recover).
- Room servers and companions never step backward (that would break message
  ordering / get their messages dropped as replays); a backward verdict is
  only reported in `get meshtimesync`.

## Drawbacks and honest limitations

- Small meshes (fewer than ~7 advert-active nodes in range) mostly won't
  reach quorum, so normal correction stays inactive — dead-clock recovery
  after a reboot still works (needs only 3).
- If most nodes around you share the SAME wrong time (it happens — one
  region of the live mesh has a 60+ node island that is 28 hours fast), your
  node may step toward them. The damage is bounded to 4 hours per day and
  every step is visible in the logs and in `get meshtimesync`.
- A backward step mutes this node's own adverts at peers for a window equal
  to the step size. It self-heals as their clocks pass the old mark, or
  delete + re-add the contact in the app to recover instantly.
- Worst case — a coordinated wrong majority around you: turn it off
  (`set meshtimesync off`) and set the clock manually, which also arms the
  7-day protection.

## Recovery recipe

If the clock ever ends up AHEAD of true time (`time` alone will refuse with
"clock cannot go backwards"):

```
set meshtimesync off
clkreboot
(log back in)
time <correct epoch>
set meshtimesync on
```

Works over remote admin too. The off/on bracket makes it deterministic (no
bootstrap race), and the manual set arms the 7-day protection anyway. Fresh
adverts take tens of minutes to accumulate after a reboot, so an operator
syncing right after login always wins.

For the design details (consensus algorithm, security model, why the limits
are what they are), see [ARCHITECTURE.md](zephcore/ARCHITECTURE.md) section 4.9.
