# The dock control mode — wire protocol

This firmware adds a **dock control mode**: a serial protocol that lets a host computer read and write
the radio's BK4819 chip registers, take over the radio entirely, and hand it a whole channel to tune
itself to. It is what [radio-server](https://github.com/kbennett2000/radio-server) drives, and this
document exists so you can drive it from **your own** software without reading that project.

The framing and the register commands are byte-compatible with nicsure's
[Quansheng Dock](https://github.com/nicsure/quansheng-dock-fw) firmware for the classic (DP32G030)
UV-K5, from which they were ported. One command, `0x0873`, is an extension that exists only here.

**Two independent implementations agree on every vector below**: this firmware's
[`App/app/dock.c`](App/app/dock.c) and radio-server's `radio_server/backends/uvk5/frames.py`. If you
write a third, the golden frames at the end are your oracle.

---

## The link

**38400 baud, 8N1**, over the radio's K1 jack — in practice via an
[AIOC cable](https://na6d.com/products/aioc-ham-radio-all-in-one-cable), which presents the radio as a
USB serial port *and* a USB sound card on one connector.

The radio **never speaks first** at top level. There is no streaming, no heartbeat, no sequence
numbers, no flow control. Every reply is caused by a command you sent. So to find out whether a radio
is there, you must **elicit** — send something that must be answered and see if it is. Silence means
"no answer", never "idle".

## Framing

```
 AB CD | Size:u16 LE | obf( payload[Size] + CRC:u16 LE ) | DC BA
```

- `payload = [opcode:u16 LE][param_len:u16 LE][params…]`, so `Size = 4 + param_len`.
- Total wire length is `Size + 8`.
- `param_len` may be 0. The payload cap is **254** bytes; a longer frame is dropped, never truncated.

**Obfuscation** is a 16-byte repeating XOR, applied to the payload **and** the two CRC bytes
(`table[i % 16]`, `i` counting from the first payload byte). It is self-inverse — the same operation
decodes.

```
16 6C 14 E6 2E 91 0D 40 21 35 D5 40 13 03 E9 80
```

It is **always on** in this firmware. The classic dock had a plaintext `0x0514` HELLO that toggled
encryption off; this tree hard-defines the link as encrypted and does not implement that toggle, so a
plaintext HELLO gets no answer here. (That is a useful tell: a radio that answers a plaintext HELLO is
on classic or stock firmware, not this one.)

**CRC-16/XMODEM** — poly `0x1021`, init `0`, no reflection, no final XOR — computed over the
**plaintext** payload.

> **The asymmetry that will cost you an afternoon if you miss it.** Commands you send carry a **real
> CRC**, and the firmware validates it and silently drops a frame that fails. Replies the radio sends
> carry a **dummy `obf(0xFF 0xFF)`** in the CRC slot — *not* a real CRC. So **validate outgoing,
> never validate incoming.** A decoder that checks reply CRCs rejects every reply the radio sends.

### Receiving

Sync on `0xAB`; require `0xCD` next; read `Size`; bounds-check `Size + 8`; wait for the whole frame;
require the tail `0xDC 0xBA`. On any mismatch, drop one byte and resync — never truncate a frame to
make it fit.

---

## Commands

| Opcode | Name | Reply | Params |
|---|---|---|---|
| `0x0850` | write registers | **none** | `[count:u16][reg:u16, value:u16] × count` |
| `0x0851` | read registers | `0x0951` **× count** | `[count:u16][reg:u16] × count` |
| `0x0870` | enter full control | **none** | — |
| `0x0871` | exit full control | **none** | — |
| `0x0873` | set VFO | `0x0874`, **always** | 13 bytes, below |
| `0x0877` | set modulation | `0x0878`, **always** | 1 byte, below |
| `0x0879` | set broadcast FM | `0x087A`, **always** | 6 bytes, below |
| `0x0951` | register info *(reply)* | — | `[reg:u16][value:u16]` |
| `0x0874` | set-VFO result *(reply)* | — | 12 bytes, below |
| `0x0878` | set-modulation result *(reply)* | — | 4 bytes, below |
| `0x087A` | set-FM result *(reply)* | — | 8 bytes, below |

An unknown opcode is dropped in silence. That is the only way to detect firmware level from the
outside: send `0x0873` or `0x0877` and see whether anything comes back.

> **`0x0875`/`0x0876` are deliberately skipped.** radio-server's ADR 0111 records the classic
> Quansheng Dock's full-control extras as *"0x0872 modulation, 0x0873/4 backlight, 0x0875/6 AM
> emulation"*. That census cannot be re-verified from this tree, so it is treated as claimed rather
> than assumed free — which is the check `0x0873`'s own allocation skipped: it was chosen after
> ruling out `0x0872` alone, from a range the same census had already spoken for. That one is
> shipped and cannot be walked back. If you are extending this protocol, **check the census before
> you allocate**; a wire opcode is the one thing here you cannot rename later.

### `0x0851` read registers — and how to probe for a radio

`ReadRegisters([0x30])` is the standard liveness elicit: one register, one `0x0951` back. Retransmit
it — opening the serial port can reset the radio, so the first attempt may be eaten by a reboot.

### `0x0870` / `0x0871` full control

`0x0870` takes the radio over: it clears the display, backs up the registers, forces the receive audio
path alive, and then **sits in a loop servicing only serial commands** until `0x0871` arrives. While
it is held:

- **The radio's own logic is suspended** — that is the point, so your register writes are not fought.
- **The PTT button and the keypad are dead.** The loop is not sampling them.
- **The radio's own speaker hisses**, because the squelch is open at the chip. Expected; it stops on
  exit.

`0x0871` calls the firmware's `RestoreRadio()`, which ends in `RADIO_SetupRegisters(true)` — and that
**retunes the synthesiser from the radio's own VFO**.

> **Every register you wrote is thrown away on exit.** Frequency (`0x38`/`0x39`), CTCSS (`0x51`/`0x07`)
> and bandwidth (`0x43`) are all transient. A host that tunes by register can never hand the radio a
> channel and let go. That is precisely why `0x0873` exists.

`0x0873` is deliberately dispatched **outside** the `0x0870` loop. Inside it, it answers `ERR_BUSY`.

---

## `0x0873` set VFO — the extension

Hands the radio a whole channel and lets **its own** code set it up: `RADIO_ApplyOffset` computes the
transmit leg, and `RADIO_ConfigureSquelchAndOutputPower` does the **per-band power-amplifier
calibration that lives in the radio's flash and is not readable from the host**. That last point is
why this command exists rather than a pile of register writes: the calibration is the radio's, so let
the radio apply it.

It writes **both** VFOs. Dual watch alternates which one is current, and a channel applied to only one
of them transmits from the wrong place roughly half the time.

### Request — 13 bytes

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u32 LE | `rx_hz` | **Hz** |
| 4 | u32 LE | `offset_hz` | **Hz** |
| 8 | u16 LE | `ctcss_tenths` | tenths of a Hz: `1000` = 100.0 Hz. `0` = no tone |
| 10 | u8 | `direction` | `0` simplex, `1` offset up, `2` offset down |
| 11 | u8 | `narrow` | `0` wide FM, `1` narrow |
| 12 | u8 | `power` | `0` low, `1` mid, `2` high |

> **Frequencies on the wire are Hz. The radio's VFO stores 10 Hz units.** The firmware converts. Send
> Hz. (This was wrong in the first cut and was silent, because the radio's band lookup *clamps* an
> out-of-range value instead of rejecting it — so a frequency 10× off landed on a band edge and tuned
> "successfully". The firmware now re-checks against the band table rather than trusting the clamp.)

**Every field is validated and a bad one is refused, never clamped**, on the grounds that a channel
silently moved is worse than a channel refused — a wrong transmit leg is somebody else's repeater.
An offset-down that would underflow is refused rather than wrapped. A CTCSS value must match the
radio's tone table **exactly**; a near-miss is refused rather than transmitted tone-less, because a
tone-less transmission into a tone-guarded repeater simply does not open it and looks like a radio
fault.

### Reply `0x0874` — 12 bytes, sent for every outcome

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `status` |
| 1 | u8 | `power` — the **radio's** `OUTPUT_POWER_*`, not the 0/1/2 you sent |
| 2 | u32 LE | `rx_hz` |
| 6 | u32 LE | `tx_hz` — **the leg that actually radiates** |
| 10 | u16 LE | `ctcss_tenths` as applied |

| `status` | Meaning |
|---|---|
| `0` | applied, exactly as requested |
| `1` | payload shorter than 13 bytes |
| `2` | busy — the host holds full control (`0x0870`); retry after `0x0871` |
| `3` | offset direction was not 0/1/2 |
| `4` | bandwidth or power off its scale |
| `5` | firmware built without the radio-side binding |
| `6` | the receive or transmit leg falls outside every band this radio has |
| `7` | tone absent from the radio's CTCSS table |

**On any non-zero status the frequency fields are zeroed**, unconditionally, in the protocol core
rather than in each binding. So a reply can never describe a channel the radio is not on, which makes
`status` the only field you have to check first.

The frequencies are read back out of the radio's VFO **after** it applied them — they are where it
landed, not what you asked for. Compare them.

### The power scale is not the scale you sent

The wire's `0`/`1`/`2` maps through the firmware to `OUTPUT_POWER_LOW1`, `MID`, `HIGH`, whose numeric
values in the radio's own enum (`USER, LOW1..LOW5, MID, HIGH`) are **1, 6 and 7**.

> Assigning the wire value raw makes "high" mean `LOW2`. That tunes perfectly, reads back perfectly,
> and **never opens a repeater** — invisible from the host until somebody notices the radio is quiet.
> That bug shipped here and was caught only once `0x0874` started reporting the applied value. If you
> implement this, check `power` in the reply against `{low:1, mid:6, high:7}`.

---

## `0x0877` set modulation — the other extension

Puts the radio on a demodulator. Like `0x0873`, it writes the radio's **own** VFO and lets the
firmware's setup path apply it, rather than poking the chip: a chip-only change does not survive, and
it applies only half of what a modulation change involves (the AM filter bandwidth is set on the way
into AM and never restored on the way out; the compander and the CTCSS interrupt mask are gated on FM
elsewhere).

It writes **both** VFOs, for the same dual-watch reason `0x0873` does.

### Request — 1 byte

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | `modulation` | `0` FM, `1` AM. `2` is **reserved for USB and refused** — see below |

**The wire's values are not the firmware's enum**, on purpose. The radio's `ModulationMode_t` is
`{ FM, AM, USB, [BYP, RAW,] UKNOWN }`, and the bracketed pair only exists on builds with
`ENABLE_BYP_RAW_DEMODULATORS` — so the enum's numeric end **moves with a build flag**. A wire value
derived from it would mean different things in two builds of the same protocol. Send the wire's
numbers; the firmware maps them, both ways.

`2` (USB) has its **number** fixed so it can never come to mean anything else, but the **value** is
refused at F7 with `ERR_FIELD`. Nobody has put this radio on USB at a bench, it cannot transmit in it
on this build anyway, and a refusal never moves the radio — so accepting it later is purely additive.
Anything above `2` is refused as well. **Refused, never clamped.**

### The modulation is sticky, and `0x0873` carries it

After one successful `0x0877`, **every later `0x0873` applies that modulation too**. This is not a
convenience. `0x0873` has to write *some* modulation into the VFO it applies; it used to write FM
literally, so "set AM, then tune" put the radio silently back on FM. And sending the two as separate
frames is not merely awkward — **this link drops frames**. The firmware is single-threaded and
anything arriving while it is busy is discarded, not queued. Tune lands, set-modulation is dropped,
and the radio sits on the right channel in the wrong demodulator with nothing on the wire saying so.

Two rules follow, and both are requirements on **you**:

- The sticky value is **session state, seeded FM**. It is never read back out of the radio — adopting
  whatever the front panel was last left on is how a repeater channel ends up in the wrong
  demodulator. A host that never sends `0x0877` gets FM on every tune, exactly as before F7.
- It is scoped to the **radio's power cycle, not your connection**. It outlives your process. **A
  reconnecting host must assert the modulation it wants**, not assume FM.

A refused `0x0877` — busy, short, out of range — **never** moves the sticky value. Only a modulation
the radio actually took becomes what later tunes apply.

### Reply `0x0878` — 4 bytes, sent for every outcome

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `status` |
| 1 | u8 | `modulation` — the wire value the radio is **now** on, or `0xFF` |
| 2 | u8 | `raw` — the radio's own `ModulationMode_t`. **Diagnostic only** |
| 3 | u8 | `flags` — bit 0: the radio will key its own transmit path in this modulation |

| `status` | Meaning |
|---|---|
| `0` | applied, exactly as requested |
| `1` | payload was empty |
| `2` | busy — the host holds full control (`0x0870`); retry after `0x0871` |
| `4` | not a modulation this firmware accepts |
| `5` | firmware built without the radio-side binding |

The numbers are `0x0874`'s, **holes and all** — `3`, `6` and `7` cannot arise here and are left unused
rather than renumbered, so one status table decodes both commands and "status 4 means a field was off
its scale" stays true whichever produced it.

`modulation` is read back out of the radio's VFO **after** it applied it, not echoed from the request.
`raw` is there for the same reason `0x0874` reports the radio's own `OUTPUT_POWER_*`: when the two
scales disagree, only the raw value makes it visible. Its numbering shifts with
`ENABLE_BYP_RAW_DEMODULATORS`, so read it, log it, and **never branch on it**.

> **On any non-zero status, `modulation` and `raw` are `0xFF` — not `0`.** `0x0874` blanks its
> frequencies to zero because 0 Hz is obviously not a channel. Copying that here would blank
> modulation to `0`, which **is FM** — a refusal shipping a plausible claim about where the radio is.
> `0xFF` also means "the radio is on something this wire cannot name", which is what you get from a
> build with the BYP/RAW demodulators compiled in.

> **A successful set-AM can stop the transmitter, and that is what `flags` is for.** Built without
> `ENABLE_TX_WHEN_AM` — which this firmware is not — the radio refuses to transmit in any modulation
> that is not FM. That refusal lives in the path the radio's **PTT pin** drives, so if you key by
> asserting a serial control line into that pin (an AIOC cable, say), **your transmitter stops
> working the moment you select AM**. Keying by writing `REG_30` over `0x0850` does *not* go through
> that path and is unaffected — so the same radio state means "cannot transmit" for one host and
> "transmits normally" for another. You cannot see a build flag from out here; check bit 0.
>
> The flag **reports** the condition. It does not remove it. **AM is receive-only on this firmware**,
> and enabling `ENABLE_TX_WHEN_AM` is not something this fork does.

---

## `0x0879` set broadcast FM — a second receiver (F8)

This one drives a **different chip**. Everything above talks to the BK4819. The radio also carries a
**BK1080**, a commercial-FM receiver covering 64–108 MHz, on the same I²C bus, sharing the antenna
front end and the audio amplifier and nothing else. `0x0850`/`0x0851` cannot reach it — those are
BK4819 register access and the BK1080's registers are not in that space.

> ### Read this before you send it
>
> **Turning this on makes the radio deaf to its own channel.**
>
> The BK1080 takes over the speaker line. If you are reading receive audio off that line — an AIOC
> cable, say — you will hear broadcast FM and nothing else, including nothing of the channel the
> radio is tuned to. That is true on every build, and `action = 0` is what gives the radio its
> ears back.
>
> **Whether it also stops transmitting depends on the image, so ask the radio rather than assuming.**
>
> Through F8, it did not. The PTT path did not consult the broadcast-FM state at all — the
> front-panel key filter whitelists PTT by name and the FM screen's key handler jumps straight to
> "start transmitting" — so a station left in this mode **transmitted normally into a channel it
> could not monitor**, including any automatic identification it sent.
>
> At F9 an interlock in `RADIO_PrepareTX` refuses that key. It is behind a build flag
> (`ENABLE_DOCK_FM_TX_INTERLOCK`, on in `Fusion`), because it changes what the radio does with no
> host involved and the editions this fork does not ship keep upstream's behaviour. It covers the
> **PTT pin and the front panel** — which is where an AIOC cable keys, by driving that same pin. It
> does **not** cover a host keying by writing `REG_30` over `0x0850`; that path never enters
> `RADIO_PrepareTX` and still transmits while deaf.
>
> **So read both flag bits, and do not read either one alone:**
>
> ```
> will_key = (flags & 0x01) && !(flags & 0x02)
> ```
>
> Bit 0 is about the *demodulator* and never goes false because broadcast FM is on. Bit 1 is the
> interlock, reporting what **this image** is actually doing right now — `0` on an F8 radio, `0` on
> an image built without the flag, `1` on a Fusion F9 radio whose receiver is running. A host that
> reads bit 0 alone gets exactly one of the four combinations wrong, and it is the dangerous one.

### Request — 6 bytes

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | `action` | `0` off, `1` on (and tune), `2` tune only. Anything above `2` is refused |
| 1–4 | u32 LE | `freq_hz` | Hz. **Must be a multiple of 100 000** — see below. Ignored when `action = 0` |
| 5 | u8 | `band` | `0` 87.5–108, `1` 76–108, `2` 76–90, `3` 64–76 (MHz). Ignored when `action = 0` |

**The wire carries Hz, and off-raster frequencies are refused rather than rounded.** The BK1080
tunes on a 100 kHz raster. `0x0873` silently truncates sub-10 Hz detail, because 10 Hz of a repeater
channel is nothing — but 100 kHz of the broadcast band is **a whole adjacent station**, so anything
off the raster comes back `ERR_FIELD` and the receiver does not move. If you want 103.2 MHz, send
`103200000`. **Refused, never rounded.**

**The band is on the wire because the firmware would clamp it silently.** Its own field for the band
is two bits wide, so a `4` becomes a `0` inside the assignment with no diagnostic anywhere, leaving
the radio on 87.5–108 while you believe otherwise. Values above `3` are refused here instead. A
frequency outside the band you named is refused too, with `ERR_BAND` — and that one is not just
tidiness: the driver computes the channel as `frequency - low_limit` in unsigned 16-bit arithmetic
with no guard, so a frequency under the floor would wrap to an enormous channel number.

**`action = 2` (tune) is not a cheaper `action = 1`.** `1` brings the chip up and takes the speaker;
`2` only moves a receiver that is already running, at the cost of three I²C writes. Tuning while the
receiver is off is refused with `ERR_OFF` rather than quietly promoted to an on — stepping across
the band must not be able to switch your station deaf by accident.

**`action = 0` (off) is never refused for the fields it ignores.** An off carrying a stale
off-raster frequency or an out-of-range band still turns the receiver off. Turning it off is how you
give the station its ears back, and that direction always stays available.

### Reply `0x087A` — 8 bytes, sent for every outcome

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `status` |
| 1 | u8 | `state` — `0` off, `1` on, or `0xFF` |
| 2–5 | u32 LE | `freq_hz` — where the receiver **is**, in Hz, or `0` |
| 6 | u8 | `band` — `0`–`3`, or `0xFF` |
| 7 | u8 | `flags` — bit 0: the demodulator will key. bit 1 (F9): broadcast FM is blocking transmit on this image. See the warning above — **read both** |

| `status` | Meaning |
|---|---|
| `0` | applied, exactly as requested |
| `1` | payload was shorter than 6 bytes |
| `2` | busy — the host holds full control (`0x0870`); retry after `0x0871` |
| `4` | unknown `action`, `band` above `3`, or a frequency off the 100 kHz raster |
| `5` | firmware built without the radio-side binding (no `ENABLE_FMRADIO`) |
| `6` | the frequency is outside the band you named |
| `8` | the radio is transmitting or monitoring — retry when it is not |
| `9` | `action = 2` (tune) with the receiver off |

The numbers are `0x0874`'s again, holes and all: `3` and `7` cannot arise here. `8` and `9` are new
to the shared table and belong to it rather than to this command — no `0x0874` or `0x0878` can
produce them, so one status table still decodes everything on this wire.

`state`, `freq_hz` and `band` are read back out of the firmware's own state **after** it applied,
not echoed from your request. That is how you see the raster and the band field doing whatever they
do without having to model them yourself.

> **On any non-zero status, `state` and `band` are `0xFF` and `freq_hz` is `0`.** Three fields, and
> deliberately not one shared sentinel: `0` is a perfectly real reading of `state` (off) and of
> `band` (87.5–108, the one most hosts want), so blanking either to `0` would answer a refusal with a
> specific and possibly wrong claim. `0` is *not* a real reading of `freq_hz`, so it blanks to `0`
> exactly as `0x0874`'s frequencies do.

> **The two `flags` bits are independent, and all four combinations are real.** Bit 0 is the BK4819
> demodulator; bit 1 is the BK1080 interlock. Nothing couples them.
>
> | bit 0 `TX_OK` | bit 1 `FM_BLOCKS_TX` | the radio | typical image |
> |---|---|---|---|
> | 1 | 0 | keys | any build, FM off — or any pre-F9/no-interlock build, FM on |
> | 1 | **1** | **refuses** | Fusion F9, receiver running |
> | 0 | 0 | refuses | on AM (see `0x0878`), receiver idle |
> | 0 | 1 | refuses | on AM *and* deaf — two independent reasons, both reported |
>
> **Bit 1 blanks to `0` on a refusal, and that direction is deliberate.** A refused frame measured
> nothing, so it must not be able to claim the radio is blocked — an unmeasured field must never
> stop a transmitter. The same reasoning that makes `state` blank to `0xFF` rather than `0` makes
> this blank to `0` rather than `1`.

> **`status 8` exists because two different things can be keying.** The radio's own transmit state
> covers the PTT pin and the front panel. It does **not** cover a host keying by writing `REG_30`
> over `0x0850` — that never enters the firmware's transmit bookkeeping, so the radio believes it is
> idle. Both are refused, and both report `8`. This is not an inconsistency with `0x0873`/`0x0877`,
> which do apply mid-transmission: their state survives the over, and this one does not — the
> firmware powers the BK1080 down on key-up, so an on applied mid-over would be torn down at once and
> the read-back would describe a receiver that is already stopping.

---

## Golden vectors

Byte-exact frames, verified against both implementations. Use these before you trust your codec.

**`0x0851` read register `0x30`, which holds `0xC1FE`** → the `0x0951` reply:

```
AB CD 08 00 47 65 10 E6 1E 91 F3 81 DE CA DC BA
```
deobfuscated payload: `51 09 04 00 30 00 FE C1`

**`0x0873` request** — receive 448.525 MHz, transmit 5 MHz down, 100.0 Hz tone, wide, high power
(params only, before framing):

```
C8 F2 BB 1A 40 4B 4C 00 E8 03 02 00 02
```

**its `0x0874` reply** — applied, `power = 7` (`HIGH`), transmit leg 443.525 MHz:

```
AB CD 10 00 62 64 18 E6 2E 96 C5 B2 9A 2F 5D E7 7C 19 01 83 E9 93 DC BA
```
deobfuscated payload: `74 08 0C 00 00 07 C8 F2 BB 1A 88 A7 6F 1A E8 03`

**`0x0877` request** — set AM (full frame):

```
AB CD 05 00 61 64 15 E6 2F 11 D5 DC BA
```
deobfuscated payload: `77 08 01 00 01`

**its `0x0878` reply** — applied, AM, `raw = 1`, `flags = 0` (**the radio will not key its own PTT
path in AM**):

```
AB CD 08 00 6E 64 10 E6 2E 90 0C 40 DE CA DC BA
```
deobfuscated payload: `78 08 04 00 00 01 01 00`

**`0x0877` with an empty payload** — the F7 probe:

```
AB CD 04 00 61 64 14 E6 D7 2B DC BA
```
deobfuscated payload: `77 08 00 00`

**its `0x0878` reply** — `ERR_SHORT`, naming no modulation:

```
AB CD 08 00 6E 64 10 E6 2F 6E F2 40 DE CA DC BA
```
deobfuscated payload: `78 08 04 00 01 FF FF 00`

**`0x0879` request** — broadcast FM on, 103.2 MHz, band 0 (full frame):

```
AB CD 0A 00 6F 64 12 E6 2F 91 B8 66 27 35 9A 85 DC BA
```
deobfuscated payload: `79 08 06 00 01 00 B5 26 06 00`

**its `0x087A` reply on a pre-F9 or no-interlock image** — applied, playing, 103.2 MHz read back,
band 0, `flags = 1` (**the radio is deaf to its own channel and will still key**):

```
AB CD 0C 00 6C 64 1C E6 2E 90 0D F5 07 33 D5 41 EC FC DC BA
```
deobfuscated payload: `7A 08 08 00 00 01 00 B5 26 06 00 01`

**the same reply from a Fusion F9 image** — identical except `flags = 3`, because the interlock is
now refusing the key (**deaf, and the radio knows it**). One byte differs, at offset 15:

```
AB CD 0C 00 6C 64 1C E6 2E 90 0D F5 07 33 D5 43 EC FC DC BA
```
deobfuscated payload: `7A 08 08 00 00 01 00 B5 26 06 00 03`

**`0x0879` request — broadcast FM OFF.** This is the frame a host sends to give a deaf station its
ears back, and it is the only `0x0879` radio-server ever sends. `freq_hz` and `band` are ignored on
this action and an off is never refused for them, so all-zero is a fine encoding of "just stop":

```
AB CD 0A 00 6F 64 12 E6 2E 91 0D 40 21 35 6E 13 DC BA
```
deobfuscated payload: `79 08 06 00 00 00 00 00 00 00`

**`0x0879` with an empty payload** — the F8 probe:

```
AB CD 04 00 6F 64 14 E6 8D 89 DC BA
```
deobfuscated payload: `79 08 00 00`

**its `0x087A` reply** — `ERR_SHORT`, naming no state, no frequency and no band:

```
AB CD 0C 00 6C 64 1C E6 2F 6E 0D 40 21 35 2A 40 EC FC DC BA
```
deobfuscated payload: `7A 08 08 00 01 FF 00 00 00 00 FF 00`

---

## Things that are not in the protocol, and will still bite you

**A six-second transmit lockout.** The radio's *stock* EEPROM commands — the HELLO `0x0514`, the
EEPROM read `0x051B`, the EEPROM write `0x051D`, and `0x052F` — each arm a **6-second timer during
which the radio refuses to transmit, and cuts an over already in progress**. Reading arms it as surely
as writing. **No dock command arms it** — not `0x0873` and not `0x0877` either — which is the main
practical reason to tune with `0x0873` rather than by writing the radio's memory. If you mix the two,
expect the transmitter to be dead for six seconds after any memory conversation.

**Receive audio needs more than the chip.** The audio path to the K1 jack passes the BK4819's audio
selector *and* an MCU GPIO that enables the external amplifier — and that GPIO is **not reachable over
this protocol**. Full-control mode starves the firmware timeslice that would normally raise it, so a
host that only writes registers gets a radio that receives **silence** while every register reads back
correct. This firmware forces the path alive on `0x0870` entry (that is what "F3" means below). On
older builds, no amount of register writing fixes it.

**Transmit needs more than `REG_30`.** Keying by writing the transmit-DSP bit is not enough: the stock
transmit path also raises the power-amplifier enable and sets the PA bias. Without those the radio
reports a successful key-up, the register read-back confirms it, and **nothing usable radiates** — a
near-field sniff sees a carrier and an antenna run sees nothing. This firmware performs the stock PA
sequence on the key-up edge (that is "F5"). Diagnosing this cost a full cycle.

**A non-FM modulation stops the radio's own transmit path.** Covered in full under `0x0877` above,
repeated here because it is the kind of thing you find at a bench rather than in a spec: built without
`ENABLE_TX_WHEN_AM`, the radio refuses to transmit in anything but FM, and that refusal is in the path
the **PTT pin** drives. Keying by serial control line into that pin stops working the moment you
select AM; keying by `REG_30` over `0x0850` does not. `0x0878`'s `flags` bit 0 is how you tell.

**A dock transmission leaves the receiver on the wrong band.** After a dock-mode transmit, the
firmware's filter-path selection leaves the *receive* front-end configured for the VFO's band, which
may not be the band you transmitted on. That is **not fixed in firmware** — radio-server corrects it
host-side. If you tune far from where the radio's own VFO sits, expect to re-assert your receive
registers after every over.

---

## Firmware levels

The dock mode arrived in stages, and each unlocked something the one before it lacked. Releases are
tagged `radio-server-fN-v5.7.0`.

| Level | Adds | Without it |
|---|---|---|
| **F1** | nothing — a build gate proving fork → build → flash → boot on an unmodified image | — |
| **F2** | the dock mode itself: `0x0850`/`0x0851`/`0x0870`/`0x0871` | no dock at all; the commands are silently ignored |
| **F3** | forces the receive audio path alive on `0x0870` | connects, and receives silence |
| **F5** | engages the power amplifier on the key-up edge | keys cleanly, radiates nothing usable |
| **F6** | `0x0873`/`0x0874` set-VFO | tuning does not survive `0x0871`; no power control |
| **F7** | `0x0877`/`0x0878` set-modulation, and `0x0873` stops forcing FM | the radio is FM-only; there is no way to receive AM |
| **F8** | `0x0879`/`0x087A` set-broadcast-FM — the BK1080 second receiver | no way to reach the second receiver, and no way to switch it off |
| **F9** | refuses to transmit while broadcast FM is running, and reports it in `0x087A` `flags` bit 1 | the radio transmits into a channel it cannot hear, station ID included |

**F9 is cumulative** — it contains F2, F3, F5, F6, F7 and F8. Flash that one.

### Detecting the level from your own software

There is no version command (the plaintext HELLO is not answered here). So ask the *command*:

1. Prove the link with a `0x0851` read of register `0x30`. A `0x0951` back means **F2 or later**.
2. Send a `0x0873` with an **empty** payload. A `0x0874` back — status `1`, `ERR_SHORT` — means
   **F6 or later**. Silence means older.
3. Send a `0x0877` with an **empty** payload. A `0x0878` back — status `1`, `ERR_SHORT` — means
   **F7 or later**. Silence means F6 or older.
4. Send a `0x0879` with an **empty** payload. A `0x087A` back — status `1`, `ERR_SHORT` — means
   **F8 or later**. Silence means F7 or older. The vectors for both halves are published above.

Steps 2, 3 and 4 are safe by construction: the length check is the first branch of each command, so
the firmware refuses before it decodes a field, before it calls its binding, and with every frequency
(or modulation, or receiver state) in the reply blanked. They are questions, not commands. Any reply
answers — `ERR_BUSY` proves the command exists just as well as `ERR_SHORT` does.

**F9 is not detectable by probing, and deliberately so.** It adds no command; it adds a refusal and a
flag bit. The bit is `0` on an F8 radio and `0` on an F9 image built without
`ENABLE_DOCK_FM_TX_INTERLOCK`, which is the right answer in both cases — neither of those radios is
blocking anything. If you need to know whether a key-up will succeed, do not infer a firmware level:
send `0x0879` and read `flags` bit 1, which reports what the image in front of you is actually doing.

---

## Implementing against this

- **[`App/app/dock.c`](App/app/dock.c) / [`dock.h`](App/app/dock.h)** are pure C with **no firmware or
  hardware includes** — all hardware sits behind a caller-supplied `dock_hal_t`. You can compile them
  on a host and use them as a reference decoder directly.
- **[`tests/host/test_dock.c`](tests/host/test_dock.c)** is 155 checks including the golden frames
  above, and **[`tests/host/test_interlock.c`](tests/host/test_interlock.c)** a further 6 across the
  three build shapes F9's interlock can take. `make -C tests/host run`. Nothing but a C compiler.
- **`radio_server/backends/uvk5/frames.py`** in
  [radio-server](https://github.com/kbennett2000/radio-server) is a complete, independently-written
  Python implementation of this protocol, with the register-level cookbook (frequency, bandwidth,
  CTCSS, key-up/key-down) in its `radio.py`.
- The BK4819 register sequences for tuning and keying are **not** part of this protocol — they are the
  chip's, reachable through `0x0850`/`0x0851`. See radio-server's ADR 0112 for the derived sequences.
