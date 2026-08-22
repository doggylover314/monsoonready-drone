# Where this sits legally, in India

> Not legal advice, just the project's own position. Indian drone rules have
> changed repeatedly since 2021 and the assistant that drafted this has
> knowledge running to May 2026, so some of it could be out of date. Every claim
> carries a confidence level and the low-confidence ones need checking against
> current DGCA and Ministry of Civil Aviation text. A judge from Arduino,
> Qualcomm or Robu.in may know the current rules better than this document.

## The position

MonsoonReady is a student prototype. It has flown only in controlled test
conditions and demonstration flights drop inert mustard seed, so no pesticide is
applied at any point. Digital Sky registration has not been completed, and that
is written here as an open gap rather than dressed up as compliance.

## Rules as we understand them

| Claim | Confidence | Detail |
|--|--|--|
| Weight categories | Reasonably high, verify | Under the Drone Rules 2021, aircraft are categorised by all-up weight: Nano to 250 g, Micro 250 g to 2 kg, Small 2 kg to 25 kg, larger above. |
| Our category | Reasonably high | An F550 with payload is over 2 kg, so Small. Meaningfully more regulated than Micro. |
| Registration route | High that it exists, lower on detail | Registration and a unique identification number through Digital Sky |
| Airspace | High that it exists, lower on detail | Green, yellow and red zones on the airspace map |
| Pilot certification | High that it exists, lower on detail | Through DGCA-approved training organisations, requirements varying by category |
| Aerial larvicide application | Low. Needs primary sources. | A standard operating procedure exists for drone-based pesticide application on the agriculture side, and pesticide registration runs through the CIB&RC under the Insecticides Act 1968. Whether public-health larviciding by drone is treated differently is not established here. |

## The gaps

**Digital Sky registration is not done.** Self-registration was blocked by the
portal for this class of build and two students found no route through it. No
exemption is claimed and no blame is placed on the portal.

**Neither of us holds a remote pilot certificate.**

**Flight conditions.** Controlled prototype testing, in open areas away from
people and property, within visual line of sight, at low altitude. Someone still
has to write down the actual test site, accurately rather than favourably.

**Larvicide registration.** TBD. Nobody has checked whether the Bti product we
sourced is registered for this use in India. A photograph of the packaging
covers the documentation and says nothing about regulatory status.

## What lawful deployment would take

1. Aircraft registration and a unique identification number.
2. Remote pilot certification for whoever is flying.
3. Operation inside permitted airspace, with authorisation where required.
4. Confirmation that the larvicide is registered for the intended use, and
   compliance with whatever aerial-application procedure applies.
5. Coordination with the municipal body responsible for vector control.
6. Insurance and an operations manual appropriate to the category.

Item 5 is the one that gets forgotten. Larviciding public spaces is a municipal
job, and a private drone dropping granules into public water without telling
anyone is a bad idea whatever the law says. The system is meant to be used by a
vector-control programme.

## Why demonstrations drop seed

Granular Bti kills mosquito, blackfly and midge larvae and is used widely in
public health work. Dropping it in a demo would turn a student flight test into
a pesticide application, under a body of law this project is neither qualified
for nor licensed under. So the hopper carries mustard seed. Decided early, not
discovered late.

One correction worth keeping. The original plan used fine salt, chosen because
it was supposed to match Bti granules for flow. It did not. Salt bridged in the
hopper and stopped flowing, which turned out to be cohesion between
hundred-micron grains rather than the hole being too small. Mustard seed at
about 1 mm is the closer match to VectoBac G's corn-cob granule anyway, and
every flow number this project has measured belongs to mustard seed.
