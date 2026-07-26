# Regulatory and compliance narrative

> **Read this warning before using any of it.**
>
> This is a **draft position for Raghav to verify and finish**, not a legal
> opinion, and neither of us is qualified to give one. Indian drone regulation
> has changed repeatedly since 2021 and may well have changed again since the
> assistant's knowledge cutoff of January 2026. **Every rule stated here must
> be checked against the current DGCA and Ministry of Civil Aviation text
> before it goes in front of judges.** Items we are confident about and items
> we are not are marked separately, deliberately.
>
> The one thing this document should never become is a confident recitation of
> rules that turn out to be superseded. A judge from Arduino, Qualcomm or
> Robu.in may well know the current rules better than we do.

## The position in one paragraph

MonsoonReady is a student research and competition prototype. It has been
flown only in controlled test conditions, and demonstration flights dispense
inert salt rather than a licensed larvicide, so no pesticide is applied at any
point. Registration on the Digital Sky portal has not been completed, and we
state that as an open gap rather than implying compliance we do not have. A
deployed version of this system would require the registration, certification
and pesticide-approval steps set out below, and we have documented them so that
the path from prototype to lawful operation is visible even though we have not
walked it.

## What we believe the rules require

**Confidence: reasonably high, but verify.** Under the Drone Rules, 2021,
unmanned aircraft in India are categorised by all-up weight, with the
categories running Nano at 250g or below, Micro above 250g up to 2kg, Small
above 2kg up to 25kg, and larger categories above that. MonsoonReady's F550
with payload is above 2kg, which places it in the **Small** category. Small is
meaningfully more regulated than Micro, which is the practical consequence for
this project.

**Confidence: high on existence, lower on current detail.** Registration of the
aircraft and issue of a unique identification number is done through the
Digital Sky platform. Operating categories and permitted airspace are governed
by the airspace map's green, yellow and red zones. Remote pilot certification
requirements exist and vary by category, obtained through DGCA-approved
training organisations.

**Confidence: low, must be researched properly.** Whether, and under what
conditions, an unmanned aircraft may be used to apply a larvicide. There is a
standard operating procedure for drone-based pesticide application issued on
the agriculture side of government, and pesticide registration itself runs
through the Central Insecticides Board and Registration Committee under the
Insecticides Act, 1968. We are **not** confident of the current requirements,
approval routes, or whether public-health larviciding by drone is treated
differently from agricultural spraying. `FILL: Raghav to research and write
this section properly from primary sources.`

## Our actual compliance gaps, stated plainly

**1. Digital Sky registration is not complete.** Attempts to self-register were
blocked by the portal for this class of build. We have not found a route
through it as a two-person student team. This is a genuine gap. We are not
claiming an exemption we do not have, and we are not claiming the portal is at
fault; we are stating that we could not complete it.

**2. No remote pilot certification.** Neither of us holds one.

**3. Flights have been conducted as controlled prototype testing**, in open
areas away from people and property, within visual line of sight, at low
altitude. `FILL: Raghav, describe the actual test site and conditions
accurately. Do not overstate this. If a flight happened somewhere that does not
match this description, either say so or leave it out.`

## Why the demo uses salt

Granular Bti is a biological larvicide that kills mosquito, blackfly and midge
larvae and is widely used in public health programmes. Dispensing it in a
demonstration would turn a student flight test into a pesticide application,
which brings in a body of law we are neither qualified for nor licensed under.

Demonstration flights therefore dispense **inert salt**, matched to Bti
granules for flow behaviour so the mechanism is tested honestly, while the
material dropped is not a pesticide. This is a deliberate design decision, not
a technicality we discovered late, and it is worth saying to judges directly:
the dispenser is proven with an inert material precisely so that the
demonstration stays inside what we are allowed to do.

`FILL: confirm VectoBac G, or whichever Bti product is sourced, is registered
for use in India, and record the registration details. A photograph of the
product packaging is sufficient for documentation purposes per the project
notes, but do not assert its regulatory status without checking it.`

## What a lawful deployment would require

Written as a roadmap, because the honest answer to "could this be used
tomorrow?" is no, and the useful answer is what would have to happen first.

1. Aircraft registration and unique identification number.
2. Remote pilot certification for the operating pilot.
3. Operation confined to permitted airspace, with authorisation where required.
4. Confirmation of the larvicide's registration for the intended use, and
   compliance with whatever aerial-application procedure applies to it.
5. Coordination with the municipal body responsible for vector control, since
   larviciding public spaces is their mandate and an uncoordinated private
   drone dropping granules into public water is a bad idea regardless of
   whether it is legal.
6. Insurance and an operations manual appropriate to the category.

Point 5 is the one most likely to be forgotten and the most important in
practice. This system is a tool for a municipal vector-control programme, not
a replacement for one.

## How to present this to judges

Do not hide it, and do not lead with it. The write-up mentions the gap in its
"what is not done" section; if asked directly, the answer is: this is a
prototype, it drops salt, we have documented what full compliance would
require, and we did not complete Digital Sky registration. Judges reward teams
who know where the edges of their work are. They do not reward teams who
claim compliance and get caught.
